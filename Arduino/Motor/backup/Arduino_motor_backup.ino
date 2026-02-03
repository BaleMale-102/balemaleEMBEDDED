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

// ===== Default PWM range (can be overridden by 'P <max_pwm>') =====
static int MAX_PWM = 2000; // <- will be set by P command (ex: 1900)
static const int MIN_PWM = 300;

// ===== Safety =====
static const unsigned long WATCHDOG_MS = 200; // match ROS node watchdog_ms (default 200)

// ===== H-bridge control macros (KEEP) =====
#define MOTORA_FORWARD(pwm)                \
  do                                       \
  {                                        \
    faboPWM.set_channel_value(DIRA1, pwm); \
    faboPWM.set_channel_value(DIRA2, 0);   \
  } while (0)
#define MOTORA_STOP(x)                   \
  do                                     \
  {                                      \
    faboPWM.set_channel_value(DIRA1, 0); \
    faboPWM.set_channel_value(DIRA2, 0); \
  } while (0)
#define MOTORA_BACKOFF(pwm)                \
  do                                       \
  {                                        \
    faboPWM.set_channel_value(DIRA1, 0);   \
    faboPWM.set_channel_value(DIRA2, pwm); \
  } while (0)

#define MOTORB_FORWARD(pwm)                \
  do                                       \
  {                                        \
    faboPWM.set_channel_value(DIRB1, pwm); \
    faboPWM.set_channel_value(DIRB2, 0);   \
  } while (0)
#define MOTORB_STOP(x)                   \
  do                                     \
  {                                      \
    faboPWM.set_channel_value(DIRB1, 0); \
    faboPWM.set_channel_value(DIRB2, 0); \
  } while (0)
#define MOTORB_BACKOFF(pwm)                \
  do                                       \
  {                                        \
    faboPWM.set_channel_value(DIRB1, 0);   \
    faboPWM.set_channel_value(DIRB2, pwm); \
  } while (0)

#define MOTORC_FORWARD(pwm)                \
  do                                       \
  {                                        \
    faboPWM.set_channel_value(DIRC1, pwm); \
    faboPWM.set_channel_value(DIRC2, 0);   \
  } while (0)
#define MOTORC_STOP(x)                   \
  do                                     \
  {                                      \
    faboPWM.set_channel_value(DIRC1, 0); \
    faboPWM.set_channel_value(DIRC2, 0); \
  } while (0)
#define MOTORC_BACKOFF(pwm)                \
  do                                       \
  {                                        \
    faboPWM.set_channel_value(DIRC1, 0);   \
    faboPWM.set_channel_value(DIRC2, pwm); \
  } while (0)

#define MOTORD_FORWARD(pwm)                \
  do                                       \
  {                                        \
    faboPWM.set_channel_value(DIRD1, pwm); \
    faboPWM.set_channel_value(DIRD2, 0);   \
  } while (0)
#define MOTORD_STOP(x)                   \
  do                                     \
  {                                      \
    faboPWM.set_channel_value(DIRD1, 0); \
    faboPWM.set_channel_value(DIRD2, 0); \
  } while (0)
#define MOTORD_BACKOFF(pwm)                \
  do                                       \
  {                                        \
    faboPWM.set_channel_value(DIRD1, 0);   \
    faboPWM.set_channel_value(DIRD2, pwm); \
  } while (0)

// ===== Motor polarity correction (KEEP) =====
// 기존 전진(ADVANCE)에서 A/C는 BACKOFF, B/D는 FORWARD였던 점 반영
static const int SIGN_A = -1;
static const int SIGN_B = +1;
static const int SIGN_C = -1;
static const int SIGN_D = +1;

static inline float fclamp(float v, float lo, float hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static void stopAll()
{
  MOTORA_STOP(0);
  MOTORB_STOP(0);
  MOTORC_STOP(0);
  MOTORD_STOP(0);
}

static void setMotorSigned(char m, int s_pwm)
{
  if (s_pwm == 0)
  {
    if (m == 'A')
      MOTORA_STOP(0);
    if (m == 'B')
      MOTORB_STOP(0);
    if (m == 'C')
      MOTORC_STOP(0);
    if (m == 'D')
      MOTORD_STOP(0);
    return;
  }

  int pwm = abs(s_pwm);
  if (pwm > MAX_PWM)
    pwm = MAX_PWM;
  if (pwm < MIN_PWM)
    pwm = MIN_PWM;

  bool forward = (s_pwm > 0);

  if (m == 'A')
    forward ? MOTORA_FORWARD(pwm) : MOTORA_BACKOFF(pwm);
  if (m == 'B')
    forward ? MOTORB_FORWARD(pwm) : MOTORB_BACKOFF(pwm);
  if (m == 'C')
    forward ? MOTORC_FORWARD(pwm) : MOTORC_BACKOFF(pwm);
  if (m == 'D')
    forward ? MOTORD_FORWARD(pwm) : MOTORD_BACKOFF(pwm);
}

static void driveMecanum(float vx, float vy, float wz)
{
  // 입력은 -1..+1 (Jetson node에서 정규화해서 내려옴)
  vx = fclamp(vx, -1.f, 1.f);
  vy = fclamp(vy, -1.f, 1.f);
  wz = fclamp(wz, -1.f, 1.f);

  // Reference mixing 그대로 유지
  // (A,B,C,D가 어떤 휠 위치인지에 따라 vy/wz 부호는 현장 튜닝 가능)
  float a = vx + vy + wz;
  float b = vx - vy - wz;
  float c = vx - vy + wz;
  float d = vx + vy - wz;

  // 정규화
  float m = max(max(fabs(a), fabs(b)), max(fabs(c), fabs(d)));
  if (m > 1.0f)
  {
    a /= m;
    b /= m;
    c /= m;
    d /= m;
  }

  auto scale = [&](float v) -> int
  {
    float av = fabs(v);
    if (av < 0.03f)
      return 0; // deadzone
    int pwm = (int)(MIN_PWM + av * (MAX_PWM - MIN_PWM));
    if (pwm > MAX_PWM)
      pwm = MAX_PWM;
    return (v >= 0.f) ? pwm : -pwm;
  };

  // 극성 보정 적용
  int pa = scale(a) * SIGN_A;
  int pb = scale(b) * SIGN_B;
  int pc = scale(c) * SIGN_C;
  int pd = scale(d) * SIGN_D;

  setMotorSigned('A', pa);
  setMotorSigned('B', pb);
  setMotorSigned('C', pc);
  setMotorSigned('D', pd);
}

// ===== UART line parser (Reference 유지 + 안정화) =====
static String rx_buf;
static unsigned long lastCmdMs = 0;

static bool readLine(String &out)
{
  while (SERIAL.available())
  {
    char c = (char)SERIAL.read();
    if (c == '\n')
    {
      out = rx_buf;
      rx_buf = "";
      return true;
    }
    if (c != '\r')
      rx_buf += c;
    if (rx_buf.length() > 100)
      rx_buf = ""; // runaway protection
  }
  return false;
}

static void handlePCommand(long v)
{
  // v: expected around 1500~2500 (from Jetson: arduino_max_pwm=1900)
  if (v < MIN_PWM + 50)
    v = MIN_PWM + 50; // MAX는 MIN보다 커야 함
  if (v > 4000)
    v = 4000;
  MAX_PWM = (int)v;

  // (선택) 디버그 ack
  SERIAL.print("ACK P ");
  SERIAL.println(MAX_PWM);
}

void setup()
{
  SERIAL.begin(115200);
  stopAll();

  if (faboPWM.begin())
  {
    SERIAL.println("Find PCA9685");
    faboPWM.init(300);
  }
  else
  {
    SERIAL.println("PCA9685 not found");
  }
  faboPWM.set_hz(50);

  SERIAL.println("Start");
  lastCmdMs = millis();
}

void loop()
{
  // watchdog: 일정 시간 명령 없으면 정지
  if (millis() - lastCmdMs > WATCHDOG_MS)
  {
    stopAll();
    // lastCmdMs를 갱신하지 않음: 명령 들어올 때까지 계속 안전정지
  }

  String line;
  if (!readLine(line))
    return;
  line.trim();
  if (line.length() == 0)
    return;

  // Z -> stop
  if (line[0] == 'Z' || line[0] == 'z')
  {
    stopAll();
    lastCmdMs = millis();
    return;
  }

  // P <max_pwm>  (NEW: for ROS node "arduino_max_pwm")
  if (line[0] == 'P' || line[0] == 'p')
  {
    long v = 0;
    if (sscanf(line.c_str(), "P %ld", &v) == 1)
    {
      handlePCommand(v);
      lastCmdMs = millis(); // P도 유효한 "alive"로 취급
    }
    return;
  }

  // D vx vy wz
  if (line[0] == 'D' || line[0] == 'd')
  {
    float vx, vy, wz;
    if (sscanf(line.c_str(), "D %f %f %f", &vx, &vy, &wz) == 3)
    {
      driveMecanum(vx, vy, wz);
      lastCmdMs = millis();
    }
    return;
  }

  // Unknown command: ignore
}
