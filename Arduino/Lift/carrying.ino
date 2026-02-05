#include <Servo.h> 

const int ENA_PIN = 6;
const int IN1_PIN = 5;
const int IN2_PIN = 4;

Servo myServo;
const int SERVO_PIN = 11; 

const int LIMIT_REV = 8;
const int LIMIT_FWD = 9;

int dcSpeed = 255;     
int servoAngle = 90;   

void setup() {
  Serial.begin(9600);
  
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  
  pinMode(LIMIT_REV, INPUT_PULLUP);
  pinMode(LIMIT_FWD, INPUT_PULLUP);

  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle);
}

bool isSwitchPressed(int pin) {
  if (digitalRead(pin) == LOW) { 
    delay(50); 
    if (digitalRead(pin) == LOW) { 
      return true; 
    }
  }
  return false; 
}

void loop() {
  if (digitalRead(IN1_PIN) == HIGH && isSwitchPressed(LIMIT_FWD)) {
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW); analogWrite(ENA_PIN, 0);
  }

  if (digitalRead(IN2_PIN) == HIGH && isSwitchPressed(LIMIT_REV)) {
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW); analogWrite(ENA_PIN, 0);
  }

  if (Serial.available()) {
    char command = Serial.read();

    if (command == '1') {
      
      if (!isSwitchPressed(LIMIT_FWD)) {
        digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW); analogWrite(ENA_PIN, 255);
        
        while (isSwitchPressed(LIMIT_FWD) == false) {
          delay(1); 
        }
      }
      
      digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW); analogWrite(ENA_PIN, 0);
      myServo.write(180); delay(1000); myServo.write(90);

      if (!isSwitchPressed(LIMIT_REV)) {
        digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, HIGH); analogWrite(ENA_PIN, 255);

        while (isSwitchPressed(LIMIT_REV) == false) {
          delay(1);
        }
      }
      
      digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW); analogWrite(ENA_PIN, 0);
    }

    else if (command == '2') {
      
      if (!isSwitchPressed(LIMIT_FWD)) {
        digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW); analogWrite(ENA_PIN, 255);
        
        while (isSwitchPressed(LIMIT_FWD) == false) {
          delay(1); 
        }
      }
      
      digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW); analogWrite(ENA_PIN, 0);
      myServo.write(0); delay(1000); myServo.write(90);

      if (!isSwitchPressed(LIMIT_REV)) {
        digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, HIGH); analogWrite(ENA_PIN, 255);

        while (isSwitchPressed(LIMIT_REV) == false) {
          delay(1);
        }
      }
      
      digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW); analogWrite(ENA_PIN, 0);
    }
  }
}