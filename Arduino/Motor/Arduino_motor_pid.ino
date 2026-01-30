/**
 * Arduino_motor_pid.ino
 * 
 * 메카넘휠 4륜 엔코더 PID 속도 제어
 * 
 * 하드웨어:
 *   - Arduino UNO/Mega
 *   - PCA9685 I2C PWM Driver
 *   - 4x DC Motor with Encoder (A, B, C, D)
 * 
 * 프로토콜 (115200 baud, DTR/RTS disabled):
 *   RX: "D vx vy wz\n"     - 속도 명령 (정규화 -1~+1)
 *       "V vx vy wz\n"     - 속도 명령 (m/s, rad/s)
 *       "P max_pwm\n"      - 최대 PWM 설정 (0-4095)
 *       "K kp ki kd\n"     - PID 게인 설정
 *       "Z\n"              - 긴급 정지
 *       "O\n"              - Odometry 요청
 *       "E\n"              - Encoder raw 요청
 *       "R\n"              - Reset encoders
 *       "?\n"              - 상태 조회
 * 
 *   TX: "OK ...\n"         - 명령 확인
 *       "O vx vy wz\n"     - Odometry 응답 (m/s, rad/s)
 *       "E a b c d\n"      - Encoder ticks 응답
 *       "READY\n"          - 부팅 완료
 * 
 * 메카넘 휠 배치 (위에서 본 모습):
 *   
 *     A (FL) ╲   ╱ B (FR)
 *             ╲ ╱
 *             ╱ ╲
 *     C (RL) ╱   ╲ D (RR)
 * 
 * 좌표계:
 *   X: 전방 (+), Y: 좌측 (+), Yaw: 반시계 (+)
 * 
 * 역기구학 (body velocity → wheel velocity):
 *   ωA = (vx - vy - (Lx+Ly)*wz) / R
 *   ωB = (vx + vy + (Lx+Ly)*wz) / R
 *   ωC = (vx + vy - (Lx+Ly)*wz) / R
 *   ωD = (vx - vy + (Lx+Ly)*wz) / R
 * 
 * 정기구학 (wheel velocity → body velocity):
 *   vx = (ωA + ωB + ωC + ωD) * R / 4
 *   vy = (-ωA + ωB + ωC - ωD) * R / 4
 *   wz = (-ωA + ωB - ωC + ωD) * R / (4 * (Lx+Ly))
 */

#include <Wire.h>

// ============ 하드웨어 설정 (사용자 조정 필요) ============

// PCA9685 설정
#define PCA9685_ADDR    0x40
#define PCA9685_MODE1   0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

// 모터 PWM 채널 (PCA9685)
#define PWM_CH_A  0   // FL
#define PWM_CH_B  2   // FR
#define PWM_CH_C  4   // RL
#define PWM_CH_D  6   // RR

// 모터 방향 핀 (Arduino Digital)
#define DIR_A1  4
#define DIR_A2  5
#define DIR_B1  6
#define DIR_B2  7
#define DIR_C1  8
#define DIR_C2  9
#define DIR_D1  10
#define DIR_D2  11

// 엔코더 핀 (Arduino Digital - 인터럽트 지원 필요)
// UNO: 2, 3만 하드웨어 인터럽트
// Mega: 2, 3, 18, 19, 20, 21
#define ENC_A_A  2   // Motor A encoder phase A (interrupt)
#define ENC_A_B  A0  // Motor A encoder phase B
#define ENC_B_A  3   // Motor B encoder phase A (interrupt)
#define ENC_B_B  A1  // Motor B encoder phase B
#define ENC_C_A  A2  // Motor C encoder phase A (polling)
#define ENC_C_B  A3  // Motor C encoder phase B
#define ENC_D_A  A4  // Motor D encoder phase A (polling)
#define ENC_D_B  A5  // Motor D encoder phase B

// 로봇 물리 파라미터
#define WHEEL_RADIUS_M    0.030f   // 휠 반경 30mm (조정 필요)
#define WHEEL_BASE_X_M    0.060f   // 휠 중심 ~ 로봇 중심 X거리 (Lx)
#define WHEEL_BASE_Y_M    0.075f   // 휠 중심 ~ 로봇 중심 Y거리 (Ly)
#define ENCODER_PPR       11       // 엔코더 PPR (조정 필요)
#define GEAR_RATIO        30       // 기어비 (조정 필요)
#define TICKS_PER_REV     (ENCODER_PPR * GEAR_RATIO * 4)  // Quadrature

// PID 기본값
#define DEFAULT_KP  80.0f
#define DEFAULT_KI  40.0f
#define DEFAULT_KD  0.5f

// 제어 주기
#define CONTROL_PERIOD_MS  20   // 50Hz
#define ODOM_PERIOD_MS     50   // 20Hz 자동 발행

// 안전
#define WATCHDOG_MS       200
#define MAX_PWM_DEFAULT   3000  // 4095 중 (안전 마진)
#define MIN_PWM_DEADBAND  200   // 이 이하는 모터 안 돌아감

// ============ 전역 변수 ============

// 엔코더 카운트 (volatile for ISR)
volatile long enc_count[4] = {0, 0, 0, 0};
long enc_count_prev[4] = {0, 0, 0, 0};

// PID 상태
float pid_target_rps[4] = {0, 0, 0, 0};  // 목표 회전 속도 (rev/s)
float pid_integral[4] = {0, 0, 0, 0};
float pid_prev_error[4] = {0, 0, 0, 0};
float pid_output[4] = {0, 0, 0, 0};

// PID 게인
float kp = DEFAULT_KP;
float ki = DEFAULT_KI;
float kd = DEFAULT_KD;

// 설정
int max_pwm = MAX_PWM_DEFAULT;

// 타이밍
unsigned long last_control_ms = 0;
unsigned long last_odom_ms = 0;
unsigned long last_cmd_ms = 0;

// Odometry 출력
float odom_vx = 0.0f;
float odom_vy = 0.0f;
float odom_wz = 0.0f;

// 시리얼 버퍼
char rx_buf[64];
int rx_idx = 0;

// 상태
bool motor_enabled = false;
bool auto_odom = false;  // 자동 odometry 발행

// ============ PCA9685 함수 ============

void pca9685_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void pca9685_init() {
  Wire.begin();
  
  // Reset
  pca9685_write(PCA9685_MODE1, 0x00);
  delay(5);
  
  // Sleep mode for prescale setting
  pca9685_write(PCA9685_MODE1, 0x10);
  
  // Set frequency ~50Hz (prescale = 121 for 50Hz, 60 for ~100Hz)
  // prescale = round(25MHz / (4096 * freq)) - 1
  pca9685_write(PCA9685_PRESCALE, 121);
  
  // Wake up
  pca9685_write(PCA9685_MODE1, 0x00);
  delay(5);
  
  // Auto-increment
  pca9685_write(PCA9685_MODE1, 0x20);
}

void pca9685_set_pwm(uint8_t channel, uint16_t pwm_val) {
  // pwm_val: 0-4095
  pwm_val = constrain(pwm_val, 0, 4095);
  
  uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
  
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(0x00);          // ON_L
  Wire.write(0x00);          // ON_H
  Wire.write(pwm_val & 0xFF);       // OFF_L
  Wire.write((pwm_val >> 8) & 0x0F); // OFF_H
  Wire.endTransmission();
}

// ============ 엔코더 ISR ============

void enc_a_isr() {
  // Quadrature decoding (A 상승 엣지)
  if (digitalRead(ENC_A_B)) {
    enc_count[0]--;
  } else {
    enc_count[0]++;
  }
}

void enc_b_isr() {
  if (digitalRead(ENC_B_B)) {
    enc_count[1]--;
  } else {
    enc_count[1]++;
  }
}

// C, D는 폴링 (UNO 인터럽트 부족)
void poll_encoders_cd() {
  static uint8_t enc_c_last = 0;
  static uint8_t enc_d_last = 0;
  
  uint8_t c_a = digitalRead(ENC_C_A);
  uint8_t c_b = digitalRead(ENC_C_B);
  uint8_t c_state = (c_a << 1) | c_b;
  
  // 간단한 상태 변화 감지 (완전한 quadrature는 아님)
  if (c_state != enc_c_last) {
    if ((enc_c_last == 0 && c_state == 1) || 
        (enc_c_last == 1 && c_state == 3) ||
        (enc_c_last == 3 && c_state == 2) ||
        (enc_c_last == 2 && c_state == 0)) {
      enc_count[2]++;
    } else {
      enc_count[2]--;
    }
    enc_c_last = c_state;
  }
  
  uint8_t d_a = digitalRead(ENC_D_A);
  uint8_t d_b = digitalRead(ENC_D_B);
  uint8_t d_state = (d_a << 1) | d_b;
  
  if (d_state != enc_d_last) {
    if ((enc_d_last == 0 && d_state == 1) || 
        (enc_d_last == 1 && d_state == 3) ||
        (enc_d_last == 3 && d_state == 2) ||
        (enc_d_last == 2 && d_state == 0)) {
      enc_count[3]++;
    } else {
      enc_count[3]--;
    }
    enc_d_last = d_state;
  }
}

// ============ 모터 제어 ============

void set_motor(int idx, int pwm_val) {
  // pwm_val: -max_pwm ~ +max_pwm
  // 양수: 정방향, 음수: 역방향
  
  int dir1_pin, dir2_pin, pwm_ch;
  
  switch (idx) {
    case 0: dir1_pin = DIR_A1; dir2_pin = DIR_A2; pwm_ch = PWM_CH_A; break;
    case 1: dir1_pin = DIR_B1; dir2_pin = DIR_B2; pwm_ch = PWM_CH_B; break;
    case 2: dir1_pin = DIR_C1; dir2_pin = DIR_C2; pwm_ch = PWM_CH_C; break;
    case 3: dir1_pin = DIR_D1; dir2_pin = DIR_D2; pwm_ch = PWM_CH_D; break;
    default: return;
  }
  
  if (pwm_val > 0) {
    digitalWrite(dir1_pin, HIGH);
    digitalWrite(dir2_pin, LOW);
  } else if (pwm_val < 0) {
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, HIGH);
    pwm_val = -pwm_val;
  } else {
    // 정지: 브레이크 모드
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, LOW);
  }
  
  pca9685_set_pwm(pwm_ch, (uint16_t)pwm_val);
}

void stop_all_motors() {
  for (int i = 0; i < 4; i++) {
    set_motor(i, 0);
    pid_target_rps[i] = 0;
    pid_integral[i] = 0;
    pid_prev_error[i] = 0;
  }
  motor_enabled = false;
}

// ============ 역기구학 ============

void inverse_kinematics(float vx, float vy, float wz, float* wheel_rps) {
  // vx, vy: m/s, wz: rad/s
  // wheel_rps: 각 휠의 목표 회전속도 (rev/s)
  
  float L = WHEEL_BASE_X_M + WHEEL_BASE_Y_M;
  float R = WHEEL_RADIUS_M;
  
  // 각 휠의 선속도 (m/s)
  float v_a = vx - vy - L * wz;  // FL
  float v_b = vx + vy + L * wz;  // FR  
  float v_c = vx + vy - L * wz;  // RL
  float v_d = vx - vy + L * wz;  // RR
  
  // 선속도 → 회전속도 (rev/s)
  wheel_rps[0] = v_a / (2.0f * PI * R);
  wheel_rps[1] = v_b / (2.0f * PI * R);
  wheel_rps[2] = v_c / (2.0f * PI * R);
  wheel_rps[3] = v_d / (2.0f * PI * R);
}

// ============ 정기구학 ============

void forward_kinematics(float* wheel_rps, float* vx, float* vy, float* wz) {
  // wheel_rps: 각 휠의 실제 회전속도 (rev/s)
  // vx, vy: m/s, wz: rad/s
  
  float L = WHEEL_BASE_X_M + WHEEL_BASE_Y_M;
  float R = WHEEL_RADIUS_M;
  
  // 회전속도 → 선속도 (m/s)
  float v_a = wheel_rps[0] * 2.0f * PI * R;
  float v_b = wheel_rps[1] * 2.0f * PI * R;
  float v_c = wheel_rps[2] * 2.0f * PI * R;
  float v_d = wheel_rps[3] * 2.0f * PI * R;
  
  *vx = (v_a + v_b + v_c + v_d) / 4.0f;
  *vy = (-v_a + v_b + v_c - v_d) / 4.0f;
  *wz = (-v_a + v_b - v_c + v_d) / (4.0f * L);
}

// ============ PID 제어 ============

void pid_update(float dt) {
  if (dt <= 0.001f) return;
  
  // 각 휠별 PID
  for (int i = 0; i < 4; i++) {
    // 현재 속도 계산 (rev/s)
    long delta_ticks;
    noInterrupts();
    delta_ticks = enc_count[i] - enc_count_prev[i];
    enc_count_prev[i] = enc_count[i];
    interrupts();
    
    float actual_rps = (float)delta_ticks / (float)TICKS_PER_REV / dt;
    
    // PID 계산
    float error = pid_target_rps[i] - actual_rps;
    
    pid_integral[i] += error * dt;
    // Anti-windup
    pid_integral[i] = constrain(pid_integral[i], -100.0f, 100.0f);
    
    float derivative = (error - pid_prev_error[i]) / dt;
    pid_prev_error[i] = error;
    
    float output = kp * error + ki * pid_integral[i] + kd * derivative;
    
    // PWM 변환 (스케일링)
    int pwm_out = (int)output;
    
    // Deadband 보정: 목표가 있으면 최소 PWM 보장
    if (pid_target_rps[i] != 0.0f && abs(pwm_out) < MIN_PWM_DEADBAND) {
      pwm_out = (pid_target_rps[i] > 0) ? MIN_PWM_DEADBAND : -MIN_PWM_DEADBAND;
    }
    
    // 제한
    pwm_out = constrain(pwm_out, -max_pwm, max_pwm);
    
    pid_output[i] = pwm_out;
    
    if (motor_enabled) {
      set_motor(i, pwm_out);
    }
  }
}

void compute_odometry(float dt) {
  if (dt <= 0.001f) return;
  
  // 각 휠의 현재 속도 (rev/s)
  float wheel_rps[4];
  
  for (int i = 0; i < 4; i++) {
    long delta_ticks;
    noInterrupts();
    // 주의: pid_update에서 이미 prev를 업데이트했으므로 여기서는 참조만
    delta_ticks = enc_count[i] - enc_count_prev[i];
    interrupts();
    
    wheel_rps[i] = (float)delta_ticks / (float)TICKS_PER_REV / dt;
  }
  
  forward_kinematics(wheel_rps, &odom_vx, &odom_vy, &odom_wz);
}

// ============ 시리얼 처리 ============

void process_command(char* cmd) {
  char type = cmd[0];
  
  if (type == 'D' || type == 'd') {
    // "D vx vy wz" - 정규화 속도 (-1 ~ +1)
    float vx_n, vy_n, wz_n;
    if (sscanf(cmd + 1, "%f %f %f", &vx_n, &vy_n, &wz_n) == 3) {
      vx_n = constrain(vx_n, -1.0f, 1.0f);
      vy_n = constrain(vy_n, -1.0f, 1.0f);
      wz_n = constrain(wz_n, -1.0f, 1.0f);
      
      // 정규화 → 실제 속도 (최대 속도 가정: 0.3m/s, 1.5rad/s)
      float vx = vx_n * 0.3f;
      float vy = vy_n * 0.3f;
      float wz = wz_n * 1.5f;
      
      inverse_kinematics(vx, vy, wz, pid_target_rps);
      motor_enabled = true;
      last_cmd_ms = millis();
      
      Serial.print("OK D ");
      Serial.print(vx, 3); Serial.print(" ");
      Serial.print(vy, 3); Serial.print(" ");
      Serial.println(wz, 3);
    }
  }
  else if (type == 'V' || type == 'v') {
    // "V vx vy wz" - 실제 속도 (m/s, rad/s)
    float vx, vy, wz;
    if (sscanf(cmd + 1, "%f %f %f", &vx, &vy, &wz) == 3) {
      inverse_kinematics(vx, vy, wz, pid_target_rps);
      motor_enabled = true;
      last_cmd_ms = millis();
      
      Serial.print("OK V ");
      Serial.print(vx, 3); Serial.print(" ");
      Serial.print(vy, 3); Serial.print(" ");
      Serial.println(wz, 3);
    }
  }
  else if (type == 'P' || type == 'p') {
    // "P max_pwm"
    int new_max;
    if (sscanf(cmd + 1, "%d", &new_max) == 1) {
      max_pwm = constrain(new_max, 100, 4095);
      Serial.print("OK P ");
      Serial.println(max_pwm);
    }
  }
  else if (type == 'K' || type == 'k') {
    // "K kp ki kd"
    float new_kp, new_ki, new_kd;
    if (sscanf(cmd + 1, "%f %f %f", &new_kp, &new_ki, &new_kd) == 3) {
      kp = new_kp;
      ki = new_ki;
      kd = new_kd;
      // 적분 리셋
      for (int i = 0; i < 4; i++) pid_integral[i] = 0;
      
      Serial.print("OK K ");
      Serial.print(kp, 2); Serial.print(" ");
      Serial.print(ki, 2); Serial.print(" ");
      Serial.println(kd, 2);
    }
  }
  else if (type == 'Z' || type == 'z') {
    // 긴급 정지
    stop_all_motors();
    Serial.println("OK Z");
  }
  else if (type == 'O' || type == 'o') {
    // Odometry 응답
    Serial.print("O ");
    Serial.print(odom_vx, 4); Serial.print(" ");
    Serial.print(odom_vy, 4); Serial.print(" ");
    Serial.println(odom_wz, 4);
  }
  else if (type == 'E' || type == 'e') {
    // Encoder raw
    Serial.print("E ");
    noInterrupts();
    Serial.print(enc_count[0]); Serial.print(" ");
    Serial.print(enc_count[1]); Serial.print(" ");
    Serial.print(enc_count[2]); Serial.print(" ");
    Serial.println(enc_count[3]);
    interrupts();
  }
  else if (type == 'R' || type == 'r') {
    // Reset encoders
    noInterrupts();
    for (int i = 0; i < 4; i++) {
      enc_count[i] = 0;
      enc_count_prev[i] = 0;
    }
    interrupts();
    Serial.println("OK R");
  }
  else if (type == 'A' || type == 'a') {
    // Auto odom toggle
    auto_odom = !auto_odom;
    Serial.print("OK A ");
    Serial.println(auto_odom ? "ON" : "OFF");
  }
  else if (type == '?') {
    // 상태 조회
    Serial.print("STATUS enabled="); Serial.print(motor_enabled);
    Serial.print(" max_pwm="); Serial.print(max_pwm);
    Serial.print(" kp="); Serial.print(kp, 2);
    Serial.print(" ki="); Serial.print(ki, 2);
    Serial.print(" kd="); Serial.print(kd, 2);
    Serial.print(" odom="); Serial.print(odom_vx, 3);
    Serial.print(","); Serial.print(odom_vy, 3);
    Serial.print(","); Serial.println(odom_wz, 3);
  }
}

void serial_rx() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (rx_idx > 0) {
        rx_buf[rx_idx] = '\0';
        process_command(rx_buf);
        rx_idx = 0;
      }
    } else if (rx_idx < (int)sizeof(rx_buf) - 1) {
      rx_buf[rx_idx++] = c;
    }
  }
}

// ============ Setup / Loop ============

void setup() {
  Serial.begin(115200);
  
  // 방향 핀
  pinMode(DIR_A1, OUTPUT); pinMode(DIR_A2, OUTPUT);
  pinMode(DIR_B1, OUTPUT); pinMode(DIR_B2, OUTPUT);
  pinMode(DIR_C1, OUTPUT); pinMode(DIR_C2, OUTPUT);
  pinMode(DIR_D1, OUTPUT); pinMode(DIR_D2, OUTPUT);
  
  // 엔코더 핀
  pinMode(ENC_A_A, INPUT_PULLUP); pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP); pinMode(ENC_B_B, INPUT_PULLUP);
  pinMode(ENC_C_A, INPUT_PULLUP); pinMode(ENC_C_B, INPUT_PULLUP);
  pinMode(ENC_D_A, INPUT_PULLUP); pinMode(ENC_D_B, INPUT_PULLUP);
  
  // 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(ENC_A_A), enc_a_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), enc_b_isr, RISING);
  
  // PCA9685 초기화
  pca9685_init();
  
  // 모터 정지
  stop_all_motors();
  
  delay(100);
  Serial.println("READY");
}

void loop() {
  unsigned long now_ms = millis();
  
  // 시리얼 수신
  serial_rx();
  
  // 엔코더 C, D 폴링
  poll_encoders_cd();
  
  // 워치독: 명령 없으면 정지
  if (motor_enabled && (now_ms - last_cmd_ms > WATCHDOG_MS)) {
    stop_all_motors();
    Serial.println("WATCHDOG");
  }
  
  // PID 제어 (50Hz)
  if (now_ms - last_control_ms >= CONTROL_PERIOD_MS) {
    float dt = (now_ms - last_control_ms) / 1000.0f;
    last_control_ms = now_ms;
    
    pid_update(dt);
    compute_odometry(dt);
  }
  
  // 자동 Odom 발행 (20Hz)
  if (auto_odom && (now_ms - last_odom_ms >= ODOM_PERIOD_MS)) {
    last_odom_ms = now_ms;
    
    Serial.print("O ");
    Serial.print(odom_vx, 4); Serial.print(" ");
    Serial.print(odom_vy, 4); Serial.print(" ");
    Serial.println(odom_wz, 4);
  }
}
