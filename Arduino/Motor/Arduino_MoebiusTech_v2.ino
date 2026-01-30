#include <Arduino.h>
#include "FaBoPWM_PCA9685.h"

FaBoPWM faboPWM;

#define DIRA1 0
#define DIRA2 1
#define DIRB1 2
#define DIRB2 3
#define DIRC1 4
#define DIRC2 5
#define DIRD1 6
#define DIRD2 7

#define SERIAL Serial

// ===== PWM range =====
static int MAX_PWM = 2000;
static const int MIN_PWM = 300;

// ===== Safety =====
static const unsigned long WATCHDOG_MS = 200;

// ===== Motor calibration scale (A, B, C, D) =====
static float motorScale[4] = {1.0f, 1.1f, 1.0f, 1.0f};  // B 느려서 1.1

// ===== Velocity to normalized conversion =====
// V 명령어에서 사용 (m/s → normalized)
static const float MAX_VEL_LINEAR = 0.05f;   // 5cm/s에서 normalized 1.0
static const float MAX_VEL_ANGULAR = 0.5f;   // 0.5rad/s에서 normalized 1.0

// ===== H-bridge control macros =====
#define MOTORA_FORWARD(pwm) do { faboPWM.set_channel_value(DIRA1, pwm); faboPWM.set_channel_value(DIRA2, 0); } while(0)
#define MOTORA_STOP(x) do { faboPWM.set_channel_value(DIRA1, 0); faboPWM.set_channel_value(DIRA2, 0); } while(0)
#define MOTORA_BACKOFF(pwm) do { faboPWM.set_channel_value(DIRA1, 0); faboPWM.set_channel_value(DIRA2, pwm); } while(0)

#define MOTORB_FORWARD(pwm) do { faboPWM.set_channel_value(DIRB1, pwm); faboPWM.set_channel_value(DIRB2, 0); } while(0)
#define MOTORB_STOP(x) do { faboPWM.set_channel_value(DIRB1, 0); faboPWM.set_channel_value(DIRB2, 0); } while(0)
#define MOTORB_BACKOFF(pwm) do { faboPWM.set_channel_value(DIRB1, 0); faboPWM.set_channel_value(DIRB2, pwm); } while(0)

#define MOTORC_FORWARD(pwm) do { faboPWM.set_channel_value(DIRC1, pwm); faboPWM.set_channel_value(DIRC2, 0); } while(0)
#define MOTORC_STOP(x) do { faboPWM.set_channel_value(DIRC1, 0); faboPWM.set_channel_value(DIRC2, 0); } while(0)
#define MOTORC_BACKOFF(pwm) do { faboPWM.set_channel_value(DIRC1, 0); faboPWM.set_channel_value(DIRC2, pwm); } while(0)

#define MOTORD_FORWARD(pwm) do { faboPWM.set_channel_value(DIRD1, pwm); faboPWM.set_channel_value(DIRD2, 0); } while(0)
#define MOTORD_STOP(x) do { faboPWM.set_channel_value(DIRD1, 0); faboPWM.set_channel_value(DIRD2, 0); } while(0)
#define MOTORD_BACKOFF(pwm) do { faboPWM.set_channel_value(DIRD1, 0); faboPWM.set_channel_value(DIRD2, pwm); } while(0)

// ===== Motor polarity =====
static const int SIGN_A = -1;
static const int SIGN_B = +1;
static const int SIGN_C = -1;
static const int SIGN_D = +1;

static inline float fclamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void stopAll() {
  MOTORA_STOP(0);
  MOTORB_STOP(0);
  MOTORC_STOP(0);
  MOTORD_STOP(0);
}

static void setMotorSigned(char m, int s_pwm) {
  if (s_pwm == 0) {
    if (m == 'A') MOTORA_STOP(0);
    if (m == 'B') MOTORB_STOP(0);
    if (m == 'C') MOTORC_STOP(0);
    if (m == 'D') MOTORD_STOP(0);
    return;
  }

  int pwm = abs(s_pwm);
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  if (pwm < MIN_PWM) pwm = MIN_PWM;

  bool forward = (s_pwm > 0);

  if (m == 'A') {
    if (forward) MOTORA_FORWARD(pwm); else MOTORA_BACKOFF(pwm);
  }
  if (m == 'B') {
    if (forward) MOTORB_FORWARD(pwm); else MOTORB_BACKOFF(pwm);
  }
  if (m == 'C') {
    if (forward) MOTORC_FORWARD(pwm); else MOTORC_BACKOFF(pwm);
  }
  if (m == 'D') {
    if (forward) MOTORD_FORWARD(pwm); else MOTORD_BACKOFF(pwm);
  }
}

static void driveMecanum(float vx, float vy, float wz) {
  vx = fclamp(vx, -1.f, 1.f);
  vy = fclamp(vy, -1.f, 1.f);
  wz = fclamp(wz, -1.f, 1.f);

  // Mecanum mixing
  float a = vx + vy + wz;
  float b = vx - vy - wz;
  float c = vx - vy + wz;
  float d = vx + vy - wz;

  // Normalize
  float m = max(max(fabs(a), fabs(b)), max(fabs(c), fabs(d)));
  if (m > 1.0f) {
    a /= m; b /= m; c /= m; d /= m;
  }

  auto scale = [&](float v, int idx) -> int {
    float av = fabs(v) * motorScale[idx];  // 보정값 적용
    if (av < 0.03f) return 0;  // deadzone
    int pwm = (int)(MIN_PWM + av * (MAX_PWM - MIN_PWM));
    if (pwm > MAX_PWM) pwm = MAX_PWM;
    return (v >= 0.f) ? pwm : -pwm;
  };

  int pa = scale(a, 0) * SIGN_A;
  int pb = scale(b, 1) * SIGN_B;
  int pc = scale(c, 2) * SIGN_C;
  int pd = scale(d, 3) * SIGN_D;

  setMotorSigned('A', pa);
  setMotorSigned('B', pb);
  setMotorSigned('C', pc);
  setMotorSigned('D', pd);
}

// ===== UART =====
static String rx_buf;
static unsigned long lastCmdMs = 0;

static bool readLine(String &out) {
  while (SERIAL.available()) {
    char c = (char)SERIAL.read();
    if (c == '\n') {
      out = rx_buf;
      rx_buf = "";
      return true;
    }
    if (c != '\r') rx_buf += c;
    if (rx_buf.length() > 100) rx_buf = "";
  }
  return false;
}

void setup() {
  SERIAL.begin(115200);
  stopAll();

  if (faboPWM.begin()) {
    SERIAL.println("Find PCA9685");
    faboPWM.init(300);
  } else {
    SERIAL.println("PCA9685 not found");
  }
  faboPWM.set_hz(50);

  SERIAL.println("READY");
  lastCmdMs = millis();
}

void loop() {
  // Watchdog
  if (millis() - lastCmdMs > WATCHDOG_MS) {
    stopAll();
  }

  String line;
  if (!readLine(line)) return;
  line.trim();
  if (line.length() == 0) return;

  char cmd = line[0];

  // Z -> stop
  if (cmd == 'Z' || cmd == 'z') {
    stopAll();
    SERIAL.println("OK");
    lastCmdMs = millis();
    return;
  }

  // S -> stop (alias)
  if (cmd == 'S' || cmd == 's') {
    stopAll();
    SERIAL.println("STOPPED");
    lastCmdMs = millis();
    return;
  }

  // P <max_pwm>
  if (cmd == 'P' || cmd == 'p') {
    long v = 0;
    if (sscanf(line.c_str(), "P %ld", &v) == 1) {
      if (v < MIN_PWM + 50) v = MIN_PWM + 50;
      if (v > 4000) v = 4000;
      MAX_PWM = (int)v;
      SERIAL.print("OK P ");
      SERIAL.println(MAX_PWM);
      lastCmdMs = millis();
    }
    return;
  }

  // D vx vy wz (normalized -1~1)
  if (cmd == 'D' || cmd == 'd') {
    // sscanf %f 안됨 -> strtok + atof 사용
    char buf[64];
    line.toCharArray(buf, 64);
    char* token = strtok(buf, " ");  // "D"
    token = strtok(NULL, " ");  // vx
    if (token) {
      float vx = atof(token);
      token = strtok(NULL, " ");  // vy
      if (token) {
        float vy = atof(token);
        token = strtok(NULL, " ");  // wz
        if (token) {
          float wz = atof(token);
          driveMecanum(vx, vy, wz);
          SERIAL.println("OK");
          lastCmdMs = millis();
        }
      }
    }
    return;
  }

  // V vx vy wz (m/s, m/s, rad/s) - arduino_bridge용
  if (cmd == 'V' || cmd == 'v') {
    char buf[64];
    line.toCharArray(buf, 64);
    char* token = strtok(buf, " ");  // "V"
    token = strtok(NULL, " ");
    if (token) {
      float vx = atof(token);
      token = strtok(NULL, " ");
      if (token) {
        float vy = atof(token);
        token = strtok(NULL, " ");
        if (token) {
          float wz = atof(token);
          float nx = vx / MAX_VEL_LINEAR;
          float ny = vy / MAX_VEL_LINEAR;
          float nw = wz / MAX_VEL_ANGULAR;
          driveMecanum(nx, ny, nw);
          SERIAL.println("OK");
          lastCmdMs = millis();
        }
      }
    }
    return;
  }

  // O -> odometry (엔코더 없으므로 0)
  if (cmd == 'O' || cmd == 'o') {
    SERIAL.println("O 0.0000 0.0000 0.0000");
    lastCmdMs = millis();
    return;
  }

  // C [motor scale] -> calibration
  if (cmd == 'C' || cmd == 'c') {
    char buf[64];
    line.toCharArray(buf, 64);
    char* token = strtok(buf, " ");  // "C"
    token = strtok(NULL, " ");  // idx
    if (token) {
      int idx = atoi(token);
      token = strtok(NULL, " ");  // scale
      if (token) {
        float sc = atof(token);
        if (idx >= 0 && idx <= 3 && sc >= 0.5f && sc <= 1.5f) {
          motorScale[idx] = sc;
          SERIAL.print("OK M");
          SERIAL.print(idx);
          SERIAL.print(" SCALE ");
          SERIAL.println(sc);
        }
      }
    } else {
      SERIAL.print("SCALE ");
      for (int i = 0; i < 4; i++) {
        SERIAL.print(motorScale[i]);
        SERIAL.print(" ");
      }
      SERIAL.println();
    }
    lastCmdMs = millis();
    return;
  }

  // ? -> status
  if (cmd == '?') {
    SERIAL.println("MoebiusTech v2.0");
    SERIAL.print("MAX_PWM: ");
    SERIAL.println(MAX_PWM);
    SERIAL.print("SCALE: ");
    for (int i = 0; i < 4; i++) {
      SERIAL.print(motorScale[i]);
      SERIAL.print(" ");
    }
    SERIAL.println();
    lastCmdMs = millis();
    return;
  }
}
