/*******************************************************************************
 * Arduino MoebiusTech Mecanum Robot - PID Motor Control
 *
 * Hardware:
 *   - Arduino UNO
 *   - MoebiusTech Motor Driver (PCA9685 + HR8833)
 *   - JGB37-520 DC Motors x4 with Encoders (1320 CPR)
 *   - 80mm Mecanum Wheels
 *
 * Communication: UART 115200 baud with Jetson (ROS2 arduino_bridge)
 *
 * PCA9685 PWM Channel Mapping (HR8833):
 *   Motor A (Front Left):  CH0 (IN1), CH1 (IN2)
 *   Motor B (Front Right): CH2 (IN3), CH3 (IN4)
 *   Motor C (Rear Left):   CH4 (IN5), CH5 (IN6)
 *   Motor D (Rear Right):  CH6 (IN7), CH7 (IN8)
 *
 * Encoder Pins:
 *   Motor A: D2 (INT0) - A phase, D4 - B phase
 *   Motor B: D3 (INT1) - A phase, D5 - B phase
 *   Motor C: A0 - A phase, A1 - B phase (polling)
 *   Motor D: A2 - A phase, A3 - B phase (polling)
 *
 * Serial Protocol:
 *   RX: "D vx vy wz\n"  - Normalized drive (-1.0 ~ 1.0)
 *   RX: "V vx vy wz\n"  - Velocity drive (m/s, rad/s)
 *   RX: "S\n"           - Emergency stop
 *   RX: "O\n"           - Request odometry
 *   RX: "K kp ki kd\n"  - Set PID gains
 *   RX: "R\n"           - Reset odometry
 *   RX: "E\n"           - Request encoder counts
 *   RX: "?\n"           - Status query
 *   TX: "O vx vy wz\n"  - Odometry response (m/s, rad/s)
 *   TX: "E a b c d\n"   - Encoder counts response
 ******************************************************************************/

#include <Wire.h>

// ============================================================================
// PCA9685 Configuration
// ============================================================================
#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

// PWM frequency 1000Hz for DC motors
#define PWM_FREQ 1000
#define PWM_RESOLUTION 4096

// ============================================================================
// Motor Channel Mapping (PCA9685 channels for HR8833)
// HR8833: IN1/IN2 per motor - PWM on both for speed+direction
// ============================================================================
// Motor A - Front Left
#define MOTOR_A_IN1 0
#define MOTOR_A_IN2 1
// Motor B - Front Right
#define MOTOR_B_IN1 2
#define MOTOR_B_IN2 3
// Motor C - Rear Left
#define MOTOR_C_IN1 4
#define MOTOR_C_IN2 5
// Motor D - Rear Right
#define MOTOR_D_IN1 6
#define MOTOR_D_IN2 7

// ============================================================================
// Encoder Pins
// ============================================================================
// Motor A & B use hardware interrupts
#define ENC_A_A 2 // INT0
#define ENC_A_B 4
#define ENC_B_A 3 // INT1
#define ENC_B_B 5

// Motor C & D use polling (analog pins as digital)
#define ENC_C_A A0
#define ENC_C_B A1
#define ENC_D_A A2
#define ENC_D_B A3

// ============================================================================
// Robot Physical Parameters
// ============================================================================
#define WHEEL_DIAMETER 0.080f // 80mm
#define WHEEL_RADIUS 0.040f   // 40mm
#define ENCODER_CPR 1320      // Counts per revolution (quadrature)
#define WHEEL_BASE_X 0.075f   // Half of front-back distance (m)
#define WHEEL_BASE_Y 0.090f   // Half of left-right distance (m)

// Derived constants
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)            // ~0.2513m
#define METERS_PER_COUNT (WHEEL_CIRCUMFERENCE / ENCODER_CPR) // ~0.00019m

// Max speeds
#define MAX_WHEEL_RPM 250.0f
#define MAX_WHEEL_RPS (MAX_WHEEL_RPM / 60.0f)               // ~4.17 rev/s
#define MAX_WHEEL_VEL (MAX_WHEEL_RPS * WHEEL_CIRCUMFERENCE) // ~1.05 m/s

// ============================================================================
// PID Parameters
// ============================================================================
#define DEFAULT_KP 60.0f
#define DEFAULT_KI 30.0f
#define DEFAULT_KD 0.5f

float Kp = DEFAULT_KP;
float Ki = DEFAULT_KI;
float Kd = DEFAULT_KD;

// PID state for each motor
float integral[4] = {0, 0, 0, 0};
float lastError[4] = {0, 0, 0, 0};

// Anti-windup limit
#define INTEGRAL_LIMIT 500.0f

// ============================================================================
// Motor Control Parameters
// ============================================================================
#define PWM_MAX 4095
#define PWM_MIN 0
#define DEADBAND_PWM 400   // Minimum PWM to overcome friction
#define CONTROL_FREQ_HZ 50 // 50Hz control loop
#define CONTROL_PERIOD_MS (1000 / CONTROL_FREQ_HZ)

// ============================================================================
// Encoder Variables (volatile for ISR)
// ============================================================================
volatile long encoderCount[4] = {0, 0, 0, 0};
volatile byte lastEncState[4] = {0, 0, 0, 0};

// For velocity calculation
long lastEncoderCount[4] = {0, 0, 0, 0};
float wheelVelocity[4] = {0, 0, 0, 0}; // rad/s

// Target velocities (rad/s)
float targetWheelVel[4] = {0, 0, 0, 0};

// ============================================================================
// Timing
// ============================================================================
unsigned long lastControlTime = 0;
unsigned long lastOdomTime = 0;

// Odometry state
float odom_x = 0, odom_y = 0, odom_theta = 0;

// ============================================================================
// Serial Communication
// ============================================================================
#define SERIAL_BAUD 115200
#define CMD_BUF_SIZE 64
char cmdBuffer[CMD_BUF_SIZE];
int cmdIndex = 0;

// ============================================================================
// PCA9685 Functions
// ============================================================================
void pca9685_init()
{
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C

    // Reset PCA9685
    pca9685_write(PCA9685_MODE1, 0x00);
    delay(5);

    // Set PWM frequency
    // prescale = round(25MHz / (4096 * freq)) - 1
    uint8_t prescale = (uint8_t)(25000000.0f / (4096.0f * PWM_FREQ) - 0.5f);

    uint8_t oldmode = pca9685_read(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // Sleep mode
    pca9685_write(PCA9685_MODE1, newmode);
    pca9685_write(PCA9685_PRESCALE, prescale);
    pca9685_write(PCA9685_MODE1, oldmode);
    delay(5);
    pca9685_write(PCA9685_MODE1, oldmode | 0xA0); // Auto-increment, restart
}

void pca9685_write(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t pca9685_read(uint8_t reg)
{
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(PCA9685_ADDR, (uint8_t)1);
    return Wire.read();
}

void pca9685_setPWM(uint8_t channel, uint16_t value)
{
    if (value > 4095)
        value = 4095;

    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;

    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(reg);
    Wire.write(0x00);                // ON_L
    Wire.write(0x00);                // ON_H
    Wire.write(value & 0xFF);        // OFF_L
    Wire.write((value >> 8) & 0x0F); // OFF_H
    Wire.endTransmission();
}

// ============================================================================
// Motor Control (HR8833 mode)
// HR8833 truth table:
//   IN1=H, IN2=L -> Forward
//   IN1=L, IN2=H -> Reverse
//   IN1=L, IN2=L -> Coast (free)
//   IN1=H, IN2=H -> Brake
//
// For PWM speed control with HR8833:
//   Forward: IN1=PWM, IN2=0
//   Reverse: IN1=0, IN2=PWM
// ============================================================================
void setMotorPWM(uint8_t motorIndex, int16_t pwmValue)
{
    uint8_t in1, in2;

    switch (motorIndex)
    {
    case 0:
        in1 = MOTOR_A_IN1;
        in2 = MOTOR_A_IN2;
        break;
    case 1:
        in1 = MOTOR_B_IN1;
        in2 = MOTOR_B_IN2;
        break;
    case 2:
        in1 = MOTOR_C_IN1;
        in2 = MOTOR_C_IN2;
        break;
    case 3:
        in1 = MOTOR_D_IN1;
        in2 = MOTOR_D_IN2;
        break;
    default:
        return;
    }

    // Clamp PWM value
    if (pwmValue > PWM_MAX)
        pwmValue = PWM_MAX;
    if (pwmValue < -PWM_MAX)
        pwmValue = -PWM_MAX;

    if (pwmValue > 0)
    {
        // Forward
        pca9685_setPWM(in1, pwmValue);
        pca9685_setPWM(in2, 0);
    }
    else if (pwmValue < 0)
    {
        // Reverse
        pca9685_setPWM(in1, 0);
        pca9685_setPWM(in2, -pwmValue);
    }
    else
    {
        // Stop (coast)
        pca9685_setPWM(in1, 0);
        pca9685_setPWM(in2, 0);
    }
}

void stopAllMotors()
{
    for (int i = 0; i < 4; i++)
    {
        setMotorPWM(i, 0);
        targetWheelVel[i] = 0;
        integral[i] = 0;
        lastError[i] = 0;
    }
}

// ============================================================================
// Encoder ISRs (Motor A & B)
// ============================================================================
void encoderA_ISR()
{
    byte a = digitalRead(ENC_A_A);
    byte b = digitalRead(ENC_A_B);
    byte state = (a << 1) | b;
    byte prevState = lastEncState[0];

    // Quadrature decoding
    int8_t delta = 0;
    if ((prevState == 0b00 && state == 0b01) ||
        (prevState == 0b01 && state == 0b11) ||
        (prevState == 0b11 && state == 0b10) ||
        (prevState == 0b10 && state == 0b00))
    {
        delta = 1;
    }
    else if ((prevState == 0b00 && state == 0b10) ||
             (prevState == 0b10 && state == 0b11) ||
             (prevState == 0b11 && state == 0b01) ||
             (prevState == 0b01 && state == 0b00))
    {
        delta = -1;
    }

    encoderCount[0] += delta;
    lastEncState[0] = state;
}

void encoderB_ISR()
{
    byte a = digitalRead(ENC_B_A);
    byte b = digitalRead(ENC_B_B);
    byte state = (a << 1) | b;
    byte prevState = lastEncState[1];

    int8_t delta = 0;
    if ((prevState == 0b00 && state == 0b01) ||
        (prevState == 0b01 && state == 0b11) ||
        (prevState == 0b11 && state == 0b10) ||
        (prevState == 0b10 && state == 0b00))
    {
        delta = 1;
    }
    else if ((prevState == 0b00 && state == 0b10) ||
             (prevState == 0b10 && state == 0b11) ||
             (prevState == 0b11 && state == 0b01) ||
             (prevState == 0b01 && state == 0b00))
    {
        delta = -1;
    }

    encoderCount[1] += delta;
    lastEncState[1] = state;
}

// Polling for Motor C & D encoders
void pollEncodersCD()
{
    // Motor C
    byte a = digitalRead(ENC_C_A);
    byte b = digitalRead(ENC_C_B);
    byte state = (a << 1) | b;
    byte prevState = lastEncState[2];

    if (state != prevState)
    {
        int8_t delta = 0;
        if ((prevState == 0b00 && state == 0b01) ||
            (prevState == 0b01 && state == 0b11) ||
            (prevState == 0b11 && state == 0b10) ||
            (prevState == 0b10 && state == 0b00))
        {
            delta = 1;
        }
        else if ((prevState == 0b00 && state == 0b10) ||
                 (prevState == 0b10 && state == 0b11) ||
                 (prevState == 0b11 && state == 0b01) ||
                 (prevState == 0b01 && state == 0b00))
        {
            delta = -1;
        }
        encoderCount[2] += delta;
        lastEncState[2] = state;
    }

    // Motor D
    a = digitalRead(ENC_D_A);
    b = digitalRead(ENC_D_B);
    state = (a << 1) | b;
    prevState = lastEncState[3];

    if (state != prevState)
    {
        int8_t delta = 0;
        if ((prevState == 0b00 && state == 0b01) ||
            (prevState == 0b01 && state == 0b11) ||
            (prevState == 0b11 && state == 0b10) ||
            (prevState == 0b10 && state == 0b00))
        {
            delta = 1;
        }
        else if ((prevState == 0b00 && state == 0b10) ||
                 (prevState == 0b10 && state == 0b11) ||
                 (prevState == 0b11 && state == 0b01) ||
                 (prevState == 0b01 && state == 0b00))
        {
            delta = -1;
        }
        encoderCount[3] += delta;
        lastEncState[3] = state;
    }
}

// ============================================================================
// Velocity Calculation
// ============================================================================
void calculateWheelVelocities(float dt)
{
    for (int i = 0; i < 4; i++)
    {
        long currentCount;
        noInterrupts();
        currentCount = encoderCount[i];
        interrupts();

        long deltaCount = currentCount - lastEncoderCount[i];
        lastEncoderCount[i] = currentCount;

        // Convert to rad/s
        // counts -> revolutions -> radians
        float revolutions = (float)deltaCount / ENCODER_CPR;
        float radians = revolutions * 2.0f * PI;
        wheelVelocity[i] = radians / dt;
    }
}

// ============================================================================
// Mecanum Kinematics
// ============================================================================
// Forward kinematics: wheel velocities -> robot velocity
// Inverse kinematics: robot velocity -> wheel velocities
//
// Mecanum wheel arrangement (top view):
//   [0]FL \\  // [1]FR
//   [2]RL //  \\ [3]RR
//
// Wheel velocity equations (rad/s):
//   w0 = (vx - vy - (Lx+Ly)*wz) / R  (Front Left)
//   w1 = (vx + vy + (Lx+Ly)*wz) / R  (Front Right)
//   w2 = (vx + vy - (Lx+Ly)*wz) / R  (Rear Left)
//   w3 = (vx - vy + (Lx+Ly)*wz) / R  (Rear Right)
// ============================================================================

void inverseKinematics(float vx, float vy, float wz, float *wheelVel)
{
    float L = WHEEL_BASE_X + WHEEL_BASE_Y; // Combined wheel base

    // Motor direction signs may need adjustment based on actual wiring
    // Assuming: positive = forward rotation
    wheelVel[0] = (vx - vy - L * wz) / WHEEL_RADIUS; // Front Left
    wheelVel[1] = (vx + vy + L * wz) / WHEEL_RADIUS; // Front Right (reversed)
    wheelVel[2] = (vx + vy - L * wz) / WHEEL_RADIUS; // Rear Left
    wheelVel[3] = (vx - vy + L * wz) / WHEEL_RADIUS; // Rear Right (reversed)

    // Right side motors are typically reversed
    wheelVel[1] = -wheelVel[1];
    wheelVel[3] = -wheelVel[3];
}

void forwardKinematics(float *wheelVel, float *vx, float *vy, float *wz)
{
    // Reverse the right side velocities for calculation
    float w0 = wheelVel[0];
    float w1 = -wheelVel[1];
    float w2 = wheelVel[2];
    float w3 = -wheelVel[3];

    float L = WHEEL_BASE_X + WHEEL_BASE_Y;

    *vx = (w0 + w1 + w2 + w3) * WHEEL_RADIUS / 4.0f;
    *vy = (-w0 + w1 + w2 - w3) * WHEEL_RADIUS / 4.0f;
    *wz = (-w0 + w1 - w2 + w3) * WHEEL_RADIUS / (4.0f * L);
}

// ============================================================================
// PID Control
// ============================================================================
int16_t computePID(uint8_t motorIndex, float target, float current, float dt)
{
    float error = target - current;

    // Proportional
    float P = Kp * error;

    // Integral with anti-windup
    integral[motorIndex] += error * dt;
    if (integral[motorIndex] > INTEGRAL_LIMIT)
        integral[motorIndex] = INTEGRAL_LIMIT;
    if (integral[motorIndex] < -INTEGRAL_LIMIT)
        integral[motorIndex] = -INTEGRAL_LIMIT;
    float I = Ki * integral[motorIndex];

    // Derivative
    float D = Kd * (error - lastError[motorIndex]) / dt;
    lastError[motorIndex] = error;

    float output = P + I + D;

    // Convert to PWM (-4095 ~ +4095)
    int16_t pwm = (int16_t)output;

    // Apply deadband compensation
    if (pwm > 0 && pwm < DEADBAND_PWM && target > 0.5f)
    {
        pwm = DEADBAND_PWM;
    }
    else if (pwm < 0 && pwm > -DEADBAND_PWM && target < -0.5f)
    {
        pwm = -DEADBAND_PWM;
    }

    // Clamp
    if (pwm > PWM_MAX)
        pwm = PWM_MAX;
    if (pwm < -PWM_MAX)
        pwm = -PWM_MAX;

    return pwm;
}

// ============================================================================
// Control Loop
// ============================================================================
void controlLoop()
{
    unsigned long now = millis();
    if (now - lastControlTime < CONTROL_PERIOD_MS)
        return;

    float dt = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;

    // Poll encoders C & D
    pollEncodersCD();

    // Calculate wheel velocities
    calculateWheelVelocities(dt);

    // PID control for each motor
    for (int i = 0; i < 4; i++)
    {
        int16_t pwm = computePID(i, targetWheelVel[i], wheelVelocity[i], dt);
        setMotorPWM(i, pwm);
    }

    // Update odometry
    updateOdometry(dt);
}

// ============================================================================
// Odometry Update
// ============================================================================
void updateOdometry(float dt)
{
    float vx, vy, wz;
    forwardKinematics(wheelVelocity, &vx, &vy, &wz);

    // Integrate (simple Euler)
    float cos_theta = cos(odom_theta);
    float sin_theta = sin(odom_theta);

    odom_x += (vx * cos_theta - vy * sin_theta) * dt;
    odom_y += (vx * sin_theta + vy * cos_theta) * dt;
    odom_theta += wz * dt;

    // Normalize theta to [-PI, PI]
    while (odom_theta > PI)
        odom_theta -= 2 * PI;
    while (odom_theta < -PI)
        odom_theta += 2 * PI;
}

// ============================================================================
// Serial Command Processing
// ============================================================================
void processCommand(char *cmd)
{
    char type = cmd[0];

    switch (type)
    {
    case 'D':
    {
        // Normalized drive: D vx vy wz (-1 ~ +1)
        float nx, ny, nw;
        if (sscanf(cmd + 1, "%f %f %f", &nx, &ny, &nw) == 3)
        {
            // Clamp to -1 ~ +1
            nx = constrain(nx, -1.0f, 1.0f);
            ny = constrain(ny, -1.0f, 1.0f);
            nw = constrain(nw, -1.0f, 1.0f);

            // Convert to m/s and rad/s
            float vx = nx * MAX_WHEEL_VEL;
            float vy = ny * MAX_WHEEL_VEL;
            float wz = nw * (MAX_WHEEL_VEL / (WHEEL_BASE_X + WHEEL_BASE_Y));

            inverseKinematics(vx, vy, wz, targetWheelVel);
            Serial.println("OK");
        }
        break;
    }

    case 'V':
    {
        // Velocity drive: V vx vy wz (m/s, m/s, rad/s)
        float vx, vy, wz;
        if (sscanf(cmd + 1, "%f %f %f", &vx, &vy, &wz) == 3)
        {
            inverseKinematics(vx, vy, wz, targetWheelVel);
            Serial.println("OK");
        }
        break;
    }

    case 'S':
    {
        // Emergency stop
        stopAllMotors();
        Serial.println("STOPPED");
        break;
    }

    case 'O':
    {
        // Odometry request
        float vx, vy, wz;
        forwardKinematics(wheelVelocity, &vx, &vy, &wz);

        Serial.print("O ");
        Serial.print(vx, 4);
        Serial.print(" ");
        Serial.print(vy, 4);
        Serial.print(" ");
        Serial.print(wz, 4);
        Serial.print(" ");
        Serial.print(odom_x, 4);
        Serial.print(" ");
        Serial.print(odom_y, 4);
        Serial.print(" ");
        Serial.println(odom_theta, 4);
        break;
    }

    case 'K':
    {
        // Set PID gains: K kp ki kd
        float kp, ki, kd;
        if (sscanf(cmd + 1, "%f %f %f", &kp, &ki, &kd) == 3)
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            // Reset integral terms
            for (int i = 0; i < 4; i++)
            {
                integral[i] = 0;
                lastError[i] = 0;
            }
            Serial.print("PID ");
            Serial.print(Kp);
            Serial.print(" ");
            Serial.print(Ki);
            Serial.print(" ");
            Serial.println(Kd);
        }
        break;
    }

    case 'R':
    {
        // Reset odometry
        odom_x = 0;
        odom_y = 0;
        odom_theta = 0;
        noInterrupts();
        for (int i = 0; i < 4; i++)
        {
            encoderCount[i] = 0;
            lastEncoderCount[i] = 0;
        }
        interrupts();
        Serial.println("RESET");
        break;
    }

    case 'E':
    {
        // Encoder counts
        Serial.print("E ");
        for (int i = 0; i < 4; i++)
        {
            long count;
            noInterrupts();
            count = encoderCount[i];
            interrupts();
            Serial.print(count);
            if (i < 3)
                Serial.print(" ");
        }
        Serial.println();
        break;
    }

    case '?':
    {
        // Status query
        Serial.println("MoebiusTech Mecanum v1.0");
        Serial.print("PID: ");
        Serial.print(Kp);
        Serial.print(" ");
        Serial.print(Ki);
        Serial.print(" ");
        Serial.println(Kd);
        Serial.print("Wheel vel (rad/s): ");
        for (int i = 0; i < 4; i++)
        {
            Serial.print(wheelVelocity[i], 2);
            Serial.print(" ");
        }
        Serial.println();
        Serial.print("Target vel (rad/s): ");
        for (int i = 0; i < 4; i++)
        {
            Serial.print(targetWheelVel[i], 2);
            Serial.print(" ");
        }
        Serial.println();
        break;
    }

    default:
        Serial.println("ERR Unknown command");
        break;
    }
}

void readSerial()
{
    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n' || c == '\r')
        {
            if (cmdIndex > 0)
            {
                cmdBuffer[cmdIndex] = '\0';
                processCommand(cmdBuffer);
                cmdIndex = 0;
            }
        }
        else if (cmdIndex < CMD_BUF_SIZE - 1)
        {
            cmdBuffer[cmdIndex++] = c;
        }
    }
}

// ============================================================================
// Setup
// ============================================================================
void setup()
{
    Serial.begin(SERIAL_BAUD);

    // Initialize PCA9685
    pca9685_init();

    // Stop all motors
    stopAllMotors();

    // Setup encoder pins
    pinMode(ENC_A_A, INPUT_PULLUP);
    pinMode(ENC_A_B, INPUT_PULLUP);
    pinMode(ENC_B_A, INPUT_PULLUP);
    pinMode(ENC_B_B, INPUT_PULLUP);
    pinMode(ENC_C_A, INPUT_PULLUP);
    pinMode(ENC_C_B, INPUT_PULLUP);
    pinMode(ENC_D_A, INPUT_PULLUP);
    pinMode(ENC_D_B, INPUT_PULLUP);

    // Initialize encoder states
    lastEncState[0] = (digitalRead(ENC_A_A) << 1) | digitalRead(ENC_A_B);
    lastEncState[1] = (digitalRead(ENC_B_A) << 1) | digitalRead(ENC_B_B);
    lastEncState[2] = (digitalRead(ENC_C_A) << 1) | digitalRead(ENC_C_B);
    lastEncState[3] = (digitalRead(ENC_D_A) << 1) | digitalRead(ENC_D_B);

    // Attach interrupts for Motor A & B
    attachInterrupt(digitalPinToInterrupt(ENC_A_A), encoderA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_A), encoderB_ISR, CHANGE);

    lastControlTime = millis();
    lastOdomTime = millis();

    delay(100);
    Serial.println("READY");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop()
{
    // Poll encoders C & D frequently
    pollEncodersCD();

    // Read serial commands
    readSerial();

    // Run control loop at fixed frequency
    controlLoop();
}
