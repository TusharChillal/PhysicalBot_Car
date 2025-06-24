#include <Arduino.h>

// Motor control pins
#define IN1 32 
#define IN2 33 
#define IN3 26 
#define IN4 27 

#define ENA 4 
#define ENB 23 

#define ser1 22
#define ser2 19

// Servo pins removed – no longer needed

int left_PWM = 0;
int right_PWM = 0;
int turn = 90;     // Still extracted from input
int turn_u = 90;   // Still extracted from input

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 ready to receive commands...");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ser1, OUTPUT);

  ledcAttachChannel(ser1,600,8,4);
  ledcAttachChannel(ser2,600,8,5);
}

void loop() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();

      if (input.startsWith("v")) {
        input.remove(0, 1);
        input.trim();

        int values[4];
        int i = 0;

        while (i < 4 && input.length() > 0) {
          int spaceIndex = input.indexOf(' ');
          String part = (spaceIndex == -1) ? input : input.substring(0, spaceIndex);
          values[i++] = part.toInt();
          input = (spaceIndex == -1) ? "" : input.substring(spaceIndex + 1);
        }

        if (i == 4) {
          left_PWM = values[0];
          right_PWM = values[1];
          turn = constrain(values[2], 0, 180);     // Still parsed
          turn_u = constrain(values[3], 0, 180);   // Still parsed

          Serial.printf("PWM L:%d R:%d | Servo1:%d Servo2:%d\n", left_PWM, right_PWM, turn, turn_u);

          motorControl(left_PWM, right_PWM);
          servoController(turn, turn_u);
          // Servo control removed
        } else {
          Serial.println("Invalid command format. Expected 4 values.");
        }
      }

      input = "";
    } else {
      input += c;
    }
  }
}

void motorControl(int leftPWM, int rightPWM) {
  if (leftPWM >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftPWM);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftPWM);
  }

  if (rightPWM >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightPWM);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightPWM);
  }
}

void servoController(int turn, int turn_u) {
  // Constrain angles to valid range
  turn = constrain(turn, 0, 180);
  turn_u = constrain(turn_u, 0, 180);

  // Convert angle (0–180) to duty (in 16-bit scale for 50Hz PWM)
  // 0° = 1ms (5% of 20ms), 180° = 2ms (10% of 20ms)
  // 16-bit resolution: 65536 steps => 1ms = ~3276, 2ms = ~6553
  uint32_t duty1 = map(turn, 0, 180, 3276, 6553);
  uint32_t duty2 = map(turn_u, 0, 180, 3276, 6553);

  // Write duty to channels
  ledcWrite(0, duty1);  // channel 0 for ser1
  ledcWrite(1, duty2);  // channel 1 for ser2
}

