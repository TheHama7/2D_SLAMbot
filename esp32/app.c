#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "qmi8658.h"
#include "ak09918.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#define UART_PORT UART_NUM_1
#define UART_TX GPIO_NUM_17
#define UART_RX GPIO_NUM_16
#define UART_BAUD 115200

// Motor pins from Waveshare schematic
#define M1_IN1  GPIO_NUM_23
#define M1_IN2  GPIO_NUM_22
#define M2_IN1  GPIO_NUM_19
#define M2_IN2  GPIO_NUM_18
#define M_PWM_FREQ 20000

static rcl_publisher_t imu_pub;
static rcl_publisher_t mag_pub;
static rcl_subscription_t sub_cmd;
geometry_msgs__msg__Twist cmd_msg;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;

void init_motors()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, M1_IN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, M1_IN2);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, M2_IN1);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, M2_IN2);

    mcpwm_config_t cfg = {
        .frequency = M_PWM_FREQ,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &cfg);
}

void set_motor_pwm(float left_speed, float right_speed)
{
    // Accepts values -1.0 to 1.0
    float dutyL = fabs(left_speed) * 100;
    float dutyR = fabs(right_speed) * 100;

    if (left_speed >= 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyL);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dutyL);
    }

    if (right_speed >= 0) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, dutyR);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, dutyR);
    }
}

void cmd_callback(const void * msg_in)
{
    const geometry_msgs__msg__Twist * cmd = (const geometry_msgs__msg__Twist *)msg_in;

    // convert linear.x and angular.z into wheel speeds
    float v = cmd->linear.x;      // forward m/s
    float w = cmd->angular.z;     // rotation rad/s

    float wheel_base = 0.20; // adjust for robot wheel spacing

    float left = v - (w * wheel_base / 2);
    float right = v + (w * wheel_base / 2);

    // scale to [-1,1]
    left = fmaxf(fminf(left,1.0), -1.0);
    right = fmaxf(fminf(right,1.0), -1.0);

    set_motor_pwm(left, right);
}

static void imu_task(void * arg)
{
    qmi8658_data_t accel, gyro;
    ak09918_data_t mag;

    while(1)
    {
        QMI8658_read_accel(&accel);
        QMI8658_read_gyro(&gyro);
        AK09918_read(&mag);

        imu_msg.linear_acceleration.x = accel.x;
        imu_msg.linear_acceleration.y = accel.y;
        imu_msg.linear_acceleration.z = accel.z;

        imu_msg.angular_velocity.x = gyro.x;
        imu_msg.angular_velocity.y = gyro.y;
        imu_msg.angular_velocity.z = gyro.z;

        rcl_publish(&imu_pub, &imu_msg, NULL);

        mag_msg.magnetic_field.x = mag.x;
        mag_msg.magnetic_field.y = mag.y;
        mag_msg.magnetic_field.z = mag.z;

        rcl_publish(&mag_pub, &mag_msg, NULL);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0);

    init_motors();
    QMI8658_init();
    AK09918_init();

    static rcl_allocator_t allocator = rcl_get_default_allocator();
    static rclc_support_t support;

    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "esp32_robot", "", &support);

    rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    );

    rclc_publisher_init_default(
        &mag_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "imu/mag"
    );

    rclc_subscription_init_default(
        &sub_cmd,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &cmd_callback, ON_NEW_DATA);

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);

    while(1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
