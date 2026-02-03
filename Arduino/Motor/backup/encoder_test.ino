/*
 * encoder_test.ino - 엔코더만 테스트 (PCA9685 없이)
 */

#define ENC_A_A 2
#define ENC_A_B 4
#define ENC_B_A 3
#define ENC_B_B 5
#define ENC_C_A A0
#define ENC_C_B A1
#define ENC_D_A A2
#define ENC_D_B A3

volatile long enc[4] = {0, 0, 0, 0};
volatile byte lastState[4] = {0, 0, 0, 0};

void setup() {
  Serial.begin(115200);

  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);
  pinMode(ENC_C_A, INPUT_PULLUP);
  pinMode(ENC_C_B, INPUT_PULLUP);
  pinMode(ENC_D_A, INPUT_PULLUP);
  pinMode(ENC_D_B, INPUT_PULLUP);

  lastState[0] = (digitalRead(ENC_A_A) << 1) | digitalRead(ENC_A_B);
  lastState[1] = (digitalRead(ENC_B_A) << 1) | digitalRead(ENC_B_B);
  lastState[2] = (digitalRead(ENC_C_A) << 1) | digitalRead(ENC_C_B);
  lastState[3] = (digitalRead(ENC_D_A) << 1) | digitalRead(ENC_D_B);

  attachInterrupt(digitalPinToInterrupt(2), encA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encB_ISR, CHANGE);

  Serial.println("ENCODER_TEST_READY");
}

void encA_ISR() {
  byte state = (digitalRead(ENC_A_A) << 1) | digitalRead(ENC_A_B);
  byte prev = lastState[0];

  if ((prev==0b00 && state==0b01) || (prev==0b01 && state==0b11) ||
      (prev==0b11 && state==0b10) || (prev==0b10 && state==0b00)) {
    enc[0]++;
  } else if ((prev==0b00 && state==0b10) || (prev==0b10 && state==0b11) ||
             (prev==0b11 && state==0b01) || (prev==0b01 && state==0b00)) {
    enc[0]--;
  }
  lastState[0] = state;
}

void encB_ISR() {
  byte state = (digitalRead(ENC_B_A) << 1) | digitalRead(ENC_B_B);
  byte prev = lastState[1];

  if ((prev==0b00 && state==0b01) || (prev==0b01 && state==0b11) ||
      (prev==0b11 && state==0b10) || (prev==0b10 && state==0b00)) {
    enc[1]++;
  } else if ((prev==0b00 && state==0b10) || (prev==0b10 && state==0b11) ||
             (prev==0b11 && state==0b01) || (prev==0b01 && state==0b00)) {
    enc[1]--;
  }
  lastState[1] = state;
}

void pollEncodersCD() {
  // Motor C
  byte state = (digitalRead(ENC_C_A) << 1) | digitalRead(ENC_C_B);
  byte prev = lastState[2];
  if (state != prev) {
    if ((prev==0b00 && state==0b01) || (prev==0b01 && state==0b11) ||
        (prev==0b11 && state==0b10) || (prev==0b10 && state==0b00)) {
      enc[2]++;
    } else if ((prev==0b00 && state==0b10) || (prev==0b10 && state==0b11) ||
               (prev==0b11 && state==0b01) || (prev==0b01 && state==0b00)) {
      enc[2]--;
    }
    lastState[2] = state;
  }

  // Motor D
  state = (digitalRead(ENC_D_A) << 1) | digitalRead(ENC_D_B);
  prev = lastState[3];
  if (state != prev) {
    if ((prev==0b00 && state==0b01) || (prev==0b01 && state==0b11) ||
        (prev==0b11 && state==0b10) || (prev==0b10 && state==0b00)) {
      enc[3]++;
    } else if ((prev==0b00 && state==0b10) || (prev==0b10 && state==0b11) ||
               (prev==0b11 && state==0b01) || (prev==0b01 && state==0b00)) {
      enc[3]--;
    }
    lastState[3] = state;
  }
}

void loop() {
  pollEncodersCD();

  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'E' || c == 'e') {
      noInterrupts();
      Serial.print("E ");
      Serial.print(enc[0]); Serial.print(" ");
      Serial.print(enc[1]); Serial.print(" ");
      Serial.print(enc[2]); Serial.print(" ");
      Serial.println(enc[3]);
      interrupts();
    }
    else if (c == 'R' || c == 'r') {
      noInterrupts();
      enc[0] = enc[1] = enc[2] = enc[3] = 0;
      interrupts();
      Serial.println("RESET");
    }
    else if (c == '?') {
      Serial.println("ENCODER_TEST v1.0");
      Serial.println("Commands: E=read, R=reset");
    }
  }
}
