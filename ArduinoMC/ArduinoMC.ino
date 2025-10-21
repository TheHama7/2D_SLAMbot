#include <ros.h>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include "MPU6050.h"
#include <std_msgs/Int32.h>


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

// ---------- ROS ----------
ros::NodeHandle nh;

// IMU topics
std_msgs::Float32 imu_ax_msg, imu_ay_msg, imu_az_msg;
ros::Publisher imu_ax_pub("imu/ax", &imu_ax_msg);
ros::Publisher imu_ay_pub("imu/ay", &imu_ay_msg);
ros::Publisher imu_az_pub("imu/az", &imu_az_msg);

// Encoder topics
std_msgs::Int32 left_enc_msg, right_enc_msg;
ros::Publisher left_enc_pub("encoder/left", &left_enc_msg);
ros::Publisher right_enc_pub("encoder/right", &right_enc_msg);

// Subscriber for motor commands
void cmdVelCallback(const geometry_msgs::Twist& cmd_msg){
  float v = cmd_msg.linear.x;
  float w = cmd_msg.angular.z;

  float wheel_base = 0.16;    // meters
  float max_speed = 255.0;    // PWM max

  int leftPWM = (int)((v - w*wheel_base/2.0) * max_speed);
  int rightPWM = (int)((v + w*wheel_base/2.0) * max_speed);

  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  // Left motor
  digitalWrite(IN1, leftPWM > 0);
  digitalWrite(IN2, leftPWM < 0);
  analogWrite(ENA, abs(leftPWM));

  // Right motor
  digitalWrite(IN3, rightPWM > 0);
  digitalWrite(IN4, rightPWM < 0);
  analogWrite(ENB, abs(rightPWM));
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmdVelCallback);

// ---------- Encoder ISRs ----------
void leftEncoderISR(){
  if(digitalRead(LEFT_ENC_B)) left_ticks++; else left_ticks--;
}
void rightEncoderISR(){
  if(digitalRead(RIGHT_ENC_B)) right_ticks++; else right_ticks--;
}

// ---------- Setup ----------
void setup(){
  nh.initNode();
  nh.advertise(imu_ax_pub);
  nh.advertise(imu_ay_pub);
  nh.advertise(imu_az_pub);
  nh.advertise(left_enc_pub);
  nh.advertise(right_enc_pub);
  nh.subscribe(cmd_vel_sub);

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
}

// ---------- Loop ----------
void loop(){
  // Read IMU
  ax = mpu.getAccelerationX() / 16384.0;
  ay = mpu.getAccelerationY() / 16384.0;
  az = mpu.getAccelerationZ() / 16384.0;

  imu_ax_msg.data = ax;
  imu_ay_msg.data = ay;
  imu_az_msg.data = az;
  imu_ax_pub.publish(&imu_ax_msg);
  imu_ay_pub.publish(&imu_ay_msg);
  imu_az_pub.publish(&imu_az_msg);

  // Publish encoder ticks
  left_enc_msg.data = left_ticks;
  right_enc_msg.data = right_ticks;
  left_enc_pub.publish(&left_enc_msg);
  right_enc_pub.publish(&right_enc_msg);

  nh.spinOnce();
  delay(20);  // 50 Hz loop
}
