#include <micro_ros_arduino.h>
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>  // Install an MPU6050 library compatible with Arduino
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>

MPU6050 mpu;

rcl_publisher_t publisher;
std_msgs__msg__String msg;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist motor_cmd;
char motor_cmd_frame[50];

void motor_cmd_callback(const void *msg_in) {
  const geometry_msgs__msg__Twist *cmd = (const geometry_msgs__msg__Twist *)msg_in;
  // Example: Extract linear.x and angular.z
  float lin = cmd->linear.x;
  float ang = cmd->angular.z;
  sprintf(motor_cmd_frame, "Lin: %.2f, Ang: %.2f", lin, ang);
  Serial.println(motor_cmd_frame);

  // TODO: Map to motor driver PWM signals here
}

void setup() {
  Serial.begin(115200);
  set_microros_wifi_transports("WIFI_SSID", "WIFI_PASSWORD", "192.168.1.100", 8888);

  static rcl_allocator_t allocator = rcl_get_default_allocator();
  static rclc_support_t support;

  rclc_support_init(&support, 0, NULL, &allocator);

  rcl_node_t node;
  rclc_node_init_default(&node, "esp32_node", "", &support);

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "imu_data");

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");

  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &motor_cmd, &motor_cmd_callback, ON_NEW_DATA);

  // Init MPU6050
  Wire.begin();
  mpu.initialize();
}

void loop() {
  msg.data.data = (char*)"IMU data placeholder";
  rcl_publish(&publisher, &msg, NULL);
  delay(100);

  // Spin executor to process callbacks
  // Here you may need to manually spin due to limitations in non-FreeRTOS environments
}