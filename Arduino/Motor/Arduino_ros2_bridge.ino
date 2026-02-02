/*******************************************************************************
 * Arduino_ros2_bridge.ino
 *
 * ROS2 Jetson ↔ Arduino 통신 브릿지 (엔코더 없는 버전)
 *
 * Hardware:
 *   - Arduino UNO
 *   - MoebiusTech Motor Hat (PCA9685)
 *   - 4x DC Motors (메카넘 휠)
 *
 * Protocol (115200 baud):
 *   RX: "V vx vy wz\n"    - 속도 명령 (m/s, rad/s)
 *       "D vx vy wz\n"    - 정규화 속도 (-1 ~ +1)
 *       "S\n" or "Z\n"    - 긴급 정지
 *       "P max_pwm\n"     - 최대 PWM 설정
 *       "?\n"             - 상태 조회
 *
 *   TX: "OK\n"            - 명령 확인
 *       "READY\n"         - 부팅 완료
 *       "STOPPED\n"       - 정지됨
 *
 * Mecanum Wheel Layout (top view):
 *     A(FL) \\  // B(FR)
 *     C(RL) //  \\ D(RR)
 *
 * Coordinate: X-forward, Y-left, Yaw-CCW positive
 ******************************************************************************/

#include <Wire.h>
#include "FaBoPWM_PCA9685.h"

FaBoPWM faboPWM;

// ============================================================================
// Motor Channel (PCA9685)
// ============================================================================
#define DIRA1 0
#define DIRA2 1
#define DIRB1 2
#define DIRB2 3
#define DIRC1 4
#define DIRC2 5
#define DIRD1 6
#define DIRD2 7

// ============================================================================
// Robot Parameters
// ============================================================================
#define WHEEL_RADIUS 0.040f // 40mm
#define WHEEL_BASE_X 0.075f // 중심에서 앞/뒤 바퀴까지 거리
#define WHEEL_BASE_Y 0.090f // 중심에서 좌/우 바퀴까지 거리

// 속도 제한
#define MAX_VEL_LINEAR 0.10f // 최대 선속도 (m/s)
#define MAX_VEL_ANGULAR 1.0f // 최대 각속도 (rad/s)

// PWM 설정
int MAX_PWM = 2000;
#define MIN_PWM 300
#define PWM_CENTER 0

// Watchdog
#define WATCHDOG_MS 300
unsigned long lastCmdMs = 0;

// Motor polarity (배선에 따라 조정)
#define SIGN_A -1
#define SIGN_B +1
#define SIGN_C -1
#define SIGN_D +1

// ============================================================================
// Motor Control Macros
// ============================================================================
#define MOTORA_FORWARD(pwm)                    \
    do                                         \
    {                                          \
        faboPWM.set_channel_value(DIRA1, pwm); \
        faboPWM.set_channel_value(DIRA2, 0);   \
    } while (0)
#define MOTORA_STOP()                        \
    do                                       \
    {                                        \
        faboPWM.set_channel_value(DIRA1, 0); \
        faboPWM.set_channel_value(DIRA2, 0); \
    } while (0)
#define MOTORA_BACKOFF(pwm)                    \
    do                                         \
    {                                          \
        faboPWM.set_channel_value(DIRA1, 0);   \
        faboPWM.set_channel_value(DIRA2, pwm); \
    } while (0)

#define MOTORB_FORWARD(pwm)                    \
    do                                         \
    {                                          \
        faboPWM.set_channel_value(DIRB1, pwm); \
        faboPWM.set_channel_value(DIRB2, 0);   \
    } while (0)
#define MOTORB_STOP()                        \
    do                                       \
    {                                        \
        faboPWM.set_channel_value(DIRB1, 0); \
        faboPWM.set_channel_value(DIRB2, 0); \
    } while (0)
#define MOTORB_BACKOFF(pwm)                    \
    do                                         \
    {                                          \
        faboPWM.set_channel_value(DIRB1, 0);   \
        faboPWM.set_channel_value(DIRB2, pwm); \
    } while (0)

#define MOTORC_FORWARD(pwm)                    \
    do                                         \
    {                                          \
        faboPWM.set_channel_value(DIRC1, pwm); \
        faboPWM.set_channel_value(DIRC2, 0);   \
    } while (0)
#define MOTORC_STOP()                        \
    do                                       \
    {                                        \
        faboPWM.set_channel_value(DIRC1, 0); \
        faboPWM.set_channel_value(DIRC2, 0); \
    } while (0)
#define MOTORC_BACKOFF(pwm)                    \
    do                                         \
    {                                          \
        faboPWM.set_channel_value(DIRC1, 0);   \
        faboPWM.set_channel_value(DIRC2, pwm); \
    } while (0)

#define MOTORD_FORWARD(pwm)                    \
    do                                         \
    {                                          \
        faboPWM.set_channel_value(DIRD1, pwm); \
        faboPWM.set_channel_value(DIRD2, 0);   \
    } while (0)
#define MOTORD_STOP()                        \
    do                                       \
    {                                        \
        faboPWM.set_channel_value(DIRD1, 0); \
        faboPWM.set_channel_value(DIRD2, 0); \
    } while (0)
#define MOTORD_BACKOFF(pwm)                    \
    do                                         \
    {                                          \
        faboPWM.set_channel_value(DIRD1, 0);   \
        faboPWM.set_channel_value(DIRD2, pwm); \
    } while (0)

// ============================================================================
// Helper Functions
// ============================================================================
float fclamp(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

// 정규화 값 → PWM 변환
int normToPwm(float n)
{
    if (fabs(n) < 0.02f)
        return 0; // deadzone
    int pwm = (int)(MIN_PWM + fabs(n) * (MAX_PWM - MIN_PWM));
    return (n >= 0) ? pwm : -pwm;
}

void stopAll()
{
    MOTORA_STOP();
    MOTORB_STOP();
    MOTORC_STOP();
    MOTORD_STOP();
}

void setMotorSigned(char motor, int signedPwm)
{
    if (signedPwm == 0)
    {
        switch (motor)
        {
        case 'A':
            MOTORA_STOP();
            break;
        case 'B':
            MOTORB_STOP();
            break;
        case 'C':
            MOTORC_STOP();
            break;
        case 'D':
            MOTORD_STOP();
            break;
        }
        return;
    }

    int pwm = abs(signedPwm);
    if (pwm > MAX_PWM)
        pwm = MAX_PWM;
    if (pwm < MIN_PWM)
        pwm = MIN_PWM;

    bool forward = (signedPwm > 0);

    switch (motor)
    {
    case 'A':
        if (forward)
            MOTORA_FORWARD(pwm);
        else
            MOTORA_BACKOFF(pwm);
        break;
    case 'B':
        if (forward)
            MOTORB_FORWARD(pwm);
        else
            MOTORB_BACKOFF(pwm);
        break;
    case 'C':
        if (forward)
            MOTORC_FORWARD(pwm);
        else
            MOTORC_BACKOFF(pwm);
        break;
    case 'D':
        if (forward)
            MOTORD_FORWARD(pwm);
        else
            MOTORD_BACKOFF(pwm);
        break;
    }
}

// ============================================================================
// Mecanum Kinematics (Inverse)
// 입력: vx (m/s), vy (m/s), wz (rad/s)
// 출력: 각 모터 PWM
// ============================================================================
void driveVelocity(float vx, float vy, float wz)
{
    // 속도 제한
    vx = fclamp(vx, -MAX_VEL_LINEAR, MAX_VEL_LINEAR);
    vy = fclamp(vy, -MAX_VEL_LINEAR, MAX_VEL_LINEAR);
    wz = fclamp(wz, -MAX_VEL_ANGULAR, MAX_VEL_ANGULAR);

    float L = WHEEL_BASE_X + WHEEL_BASE_Y;
    float R = WHEEL_RADIUS;

    // 역기구학: 각 바퀴의 선속도 (m/s)
    //   v_a = vx - vy - L * wz  (Front Left)
    //   v_b = vx + vy + L * wz  (Front Right)
    //   v_c = vx + vy - L * wz  (Rear Left)
    //   v_d = vx - vy + L * wz  (Rear Right)
    float v_a = vx - vy - L * wz;
    float v_b = vx + vy + L * wz;
    float v_c = vx + vy - L * wz;
    float v_d = vx - vy + L * wz;

    // 정규화 (-1 ~ +1)
    float maxV = max(max(fabs(v_a), fabs(v_b)), max(fabs(v_c), fabs(v_d)));
    float scale = 1.0f;
    if (maxV > MAX_VEL_LINEAR)
    {
        scale = MAX_VEL_LINEAR / maxV;
    }

    float n_a = (v_a * scale) / MAX_VEL_LINEAR;
    float n_b = (v_b * scale) / MAX_VEL_LINEAR;
    float n_c = (v_c * scale) / MAX_VEL_LINEAR;
    float n_d = (v_d * scale) / MAX_VEL_LINEAR;

    // 정규화 → PWM 변환
    int pwm_a = normToPwm(n_a) * SIGN_A;
    int pwm_b = normToPwm(n_b) * SIGN_B;
    int pwm_c = normToPwm(n_c) * SIGN_C;
    int pwm_d = normToPwm(n_d) * SIGN_D;

    setMotorSigned('A', pwm_a);
    setMotorSigned('B', pwm_b);
    setMotorSigned('C', pwm_c);
    setMotorSigned('D', pwm_d);
}

// 정규화 속도 (-1 ~ +1) 입력
void driveNormalized(float nx, float ny, float nw)
{
    nx = fclamp(nx, -1.0f, 1.0f);
    ny = fclamp(ny, -1.0f, 1.0f);
    nw = fclamp(nw, -1.0f, 1.0f);

    float vx = nx * MAX_VEL_LINEAR;
    float vy = ny * MAX_VEL_LINEAR;
    float wz = nw * MAX_VEL_ANGULAR;

    driveVelocity(vx, vy, wz);
}

// ============================================================================
// Serial Communication
// ============================================================================
String rxBuffer = "";

bool readLine(String &out)
{
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\n')
        {
            out = rxBuffer;
            rxBuffer = "";
            return true;
        }
        if (c != '\r')
        {
            rxBuffer += c;
        }
        if (rxBuffer.length() > 64)
        {
            rxBuffer = "";
        }
    }
    return false;
}

// 입력: "V 0.05 0.0 0.1" → values[0]=0.05, values[1]=0.0, values[2]=0.1
bool parseThreeFloats(const String &line, float values[3])
{
    String data = line.substring(1); // 첫 글자(명령) 제거
    data.trim();

    int idx = 0;
    int start = 0;

    for (int i = 0; i <= data.length() && idx < 3; i++)
    {
        if (i == data.length() || data[i] == ' ')
        {
            if (i > start)
            {
                String token = data.substring(start, i);
                values[idx++] = token.toFloat();
            }
            start = i + 1;
        }
    }

    return (idx == 3);
}

// 공백 뒤의 정수 파싱
bool parseOneInt(const String &line, int &value)
{
    String data = line.substring(1);
    data.trim();
    if (data.length() == 0)
        return false;
    value = data.toInt();
    return true;
}

void processCommand(String &line)
{
    line.trim();
    if (line.length() == 0)
        return;

    char cmd = line[0];

    // V vx vy wz - 속도 명령 (m/s, rad/s)
    if (cmd == 'V' || cmd == 'v')
    {
        float vals[3];
        if (parseThreeFloats(line, vals))
        {
            driveVelocity(vals[0], vals[1], vals[2]);
            lastCmdMs = millis();
            Serial.println("OK");
        }
        else
        {
            Serial.println("ERR PARSE");
        }
        return;
    }

    // D nx ny nw - 정규화 속도 (-1 ~ +1)
    if (cmd == 'D' || cmd == 'd')
    {
        float vals[3];
        if (parseThreeFloats(line, vals))
        {
            driveNormalized(vals[0], vals[1], vals[2]);
            lastCmdMs = millis();
            Serial.println("OK");
        }
        else
        {
            Serial.println("ERR PARSE");
        }
        return;
    }

    // S or Z - 정지
    if (cmd == 'S' || cmd == 's' || cmd == 'Z' || cmd == 'z')
    {
        stopAll();
        lastCmdMs = millis();
        Serial.println("STOPPED");
        return;
    }

    // P max_pwm - 최대 PWM 설정
    if (cmd == 'P' || cmd == 'p')
    {
        int newMax;
        if (parseOneInt(line, newMax))
        {
            if (newMax >= MIN_PWM && newMax <= 4000)
            {
                MAX_PWM = newMax;
                Serial.print("OK PWM ");
                Serial.println(MAX_PWM);
            }
            else
            {
                Serial.println("ERR RANGE");
            }
        }
        else
        {
            Serial.println("ERR PARSE");
        }
        lastCmdMs = millis();
        return;
    }

    // ? - 상태 조회
    if (cmd == '?')
    {
        Serial.println("ROS2_BRIDGE v1.0");
        Serial.print("MAX_PWM: ");
        Serial.println(MAX_PWM);
        Serial.print("MAX_VEL: ");
        Serial.print(MAX_VEL_LINEAR);
        Serial.print(" m/s, ");
        Serial.print(MAX_VEL_ANGULAR);
        Serial.println(" rad/s");
        lastCmdMs = millis();
        return;
    }

    // 알 수 없는 명령
    Serial.println("ERR CMD");
}

// ============================================================================
// Setup & Loop
// ============================================================================
void setup()
{
    Serial.begin(115200);

    if (faboPWM.begin())
    {
        Serial.println("PCA9685 OK");
        faboPWM.init(300);
    }
    else
    {
        Serial.println("PCA9685 FAIL");
    }
    faboPWM.set_hz(50);

    stopAll();
    lastCmdMs = millis();

    Serial.println("READY");
}

void loop()
{
    // Watchdog: 명령 없으면 정지
    if (millis() - lastCmdMs > WATCHDOG_MS)
    {
        stopAll();
    }

    // 시리얼 명령 처리
    String line;
    if (readLine(line))
    {
        processCommand(line);
    }
}
