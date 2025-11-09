#include <Wire.h>
#include "MPU6050.h"

// ---------- Motors ----------
#define ENA 11
#define IN1 7
#define IN2 8
#define ENB 6
#define IN3 9
#define IN4 10

// ---------- Encoders ----------
#define LEFT_ENC_A 2
#define LEFT_ENC_B 4
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 5

volatile long left_ticks = 0;
volatile long right_ticks = 0;

// ---------- IMU ----------
MPU6050 mpu;
float ax, ay, az, gx, gy, gz;

// ---------- Constants ----------
float wheel_base = 0.16; // meters
float max_speed_pwm = 255.0;

// ---------- Serial Protocol ----------
// Jetson sends: "CMD v w\n"
// Arduino replies: "ENC left right IMU ax ay az\n"

void setup() {
  Serial.begin(115200);

  // Motors
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Encoders
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

  // IMU
  Wire.begin();
  mpu.initialize();

  delay(1000);
  Serial.println("Arduino ready");
}

void leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B)) left_ticks++; else left_ticks--;
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B)) right_ticks++; else right_ticks--;
}

void loop() {
  // Check for incoming command
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("CMD")) {
      float v, w;
      sscanf(cmd.c_str(), "CMD %f %f", &v, &w);
      driveMotors(v, w);
    }
  }

  // Read IMU
  ax = mpu.getAccelerationX() / 16384.0;
  ay = mpu.getAccelerationY() / 16384.0;
  az = mpu.getAccelerationZ() / 16384.0;

  // Send telemetry
  Serial.print("ENC ");
  Serial.print(left_ticks);
  Serial.print(" ");
  Serial.print(right_ticks);
  Serial.print(" IMU ");
  Serial.print(ax, 3);
  Serial.print(" ");
  Serial.print(ay, 3);
  Serial.print(" ");
  Serial.println(az, 3);

  delay(20); // 50 Hz
}

void driveMotors(float v, float w) {
  int leftPWM = (int)((v - w * wheel_base / 2.0) * max_speed_pwm);
  int rightPWM = (int)((v + w * wheel_base / 2.0) * max_speed_pwm);

  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  digitalWrite(IN1, leftPWM > 0);
  digitalWrite(IN2, leftPWM < 0);
  analogWrite(ENA, abs(leftPWM));

  digitalWrite(IN3, rightPWM > 0);
  digitalWrite(IN4, rightPWM < 0);
  analogWrite(ENB, abs(rightPWM));
}
