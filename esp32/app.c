#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <math.h>

#include "IMU.h"   // Waveshare IMU wrapper

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32_multi_array.h>

// ===== MOTOR PWM PINS =====
#define M1_IN1  GPIO_NUM_25
#define M1_IN2  GPIO_NUM_26
#define M2_IN1  GPIO_NUM_27
#define M2_IN2  GPIO_NUM_14
#define PWM_FREQ 20000

// ===== ENCODER PINS =====
#define AENCA GPIO_NUM_35
#define AENCB GPIO_NUM_34
#define BENCA GPIO_NUM_27
#define BENCB GPIO_NUM_16

volatile long A_pulse = 0;
volatile long B_pulse = 0;

// Encoder parameters
double reduction_ratio = 42;
int ppr_num = 11;
double shaft_ppr = 42 * 11;
int interval = 100;

// ===== micro-ROS objects =====
static rcl_publisher_t imu_pub;
static rcl_publisher_t rpm_pub;
static rcl_subscription_t cmd_sub;

sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32MultiArray rpm_msg;
geometry_msgs__msg__Twist cmd_msg;

// ===== ENCODER ISR =====
static void IRAM_ATTR A_wheel_pulse(void* arg)
{
    if (gpio_get_level(AENCA))
        A_pulse++;
    else
        A_pulse--;
}

static void IRAM_ATTR B_wheel_pulse(void* arg)
{
    if (gpio_get_level(BENCA))
        B_pulse++;
    else
        B_pulse--;
}

// ===== MOTOR PWM =====
void pwm_init()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, M1_IN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, M1_IN2);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, M2_IN1);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, M2_IN2);

    mcpwm_config_t cfg = {
        .frequency = PWM_FREQ,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &cfg);
}

void set_motor(float left, float right)
{
    float L = fminf(fmaxf(fabsf(left) * 100.0f, 0), 100);
    float R = fminf(fmaxf(fabsf(right) * 100.0f, 0), 100);

    // Left motor
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0,
                   (left >= 0) ? MCPWM_OPR_A : MCPWM_OPR_B, L);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0,
                   (left >= 0) ? MCPWM_OPR_B : MCPWM_OPR_A, 0);

    // Right motor
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1,
                   (right >= 0) ? MCPWM_OPR_A : MCPWM_OPR_B, R);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1,
                   (right >= 0) ? MCPWM_OPR_B : MCPWM_OPR_A, 0);
}

// ===== /cmd_vel CALLBACK =====
void cmd_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *cmd = (const geometry_msgs__msg__Twist *)msg_in;
    float v = cmd->linear.x;
    float w = cmd->angular.z;
    float L = 0.20;

    float left  = v - (w * L * 0.5f);
    float right = v + (w * L * 0.5f);

    left = fminf(fmaxf(left, -1), 1);
    right = fminf(fmaxf(right, -1), 1);

    set_motor(left, right);
}

// ===== RPM TASK =====
void rpm_task(void *arg)
{
    rpm_msg.data.capacity = 2;
    rpm_msg.data.size = 2;
    rpm_msg.data.data = malloc(2 * sizeof(float));

    while (1)
    {
        float rpm_A = ((float)A_pulse / shaft_ppr) * 60 * (1000 / interval);
        float rpm_B = ((float)B_pulse / shaft_ppr) * 60 * (1000 / interval);

        A_pulse = 0;
        B_pulse = 0;

        rpm_msg.data.data[0] = rpm_A;
        rpm_msg.data.data[1] = rpm_B;

        rcl_publish(&rpm_pub, &rpm_msg, NULL);
        vTaskDelay(pdMS_TO_TICKS(interval));
    }
}

// ===== IMU TASK =====
void imu_task(void *arg)
{
    EulerAngles angles;
    IMU_ST_SENSOR_DATA_FLOAT gyro, accel;
    IMU_ST_SENSOR_DATA magn;

    while(1)
    {
        imuDataGet(&angles, &gyro, &accel, &magn);

        imu_msg.linear_acceleration.x = accel.X;
        imu_msg.linear_acceleration.y = accel.Y;
        imu_msg.linear_acceleration.z = accel.Z;

        imu_msg.angular_velocity.x = gyro.X;
        imu_msg.angular_velocity.y = gyro.Y;
        imu_msg.angular_velocity.z = gyro.Z;

        rcl_publish(&imu_pub, &imu_msg, NULL);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ===== MAIN =====
void app_main(void)
{
    // ===== GPIO CONFIG FOR ENCODERS =====
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<AENCA) | (1ULL<<AENCB) | (1ULL<<BENCA) | (1ULL<<BENCB),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    // ISR service
    gpio_install_isr_service(0);
    gpio_set_intr_type(AENCB, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(AENCB, A_wheel_pulse, NULL);

    gpio_set_intr_type(BENCB, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(BENCB, B_wheel_pulse, NULL);

    // Init motor + IMU
    pwm_init();
    imuInit();

    // ===== micro-ROS Setup =====
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "esp32_robot", "", &support);

    rclc_publisher_init_default(&imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");

    rclc_publisher_init_default(&rpm_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "wheel/rpm");

    rclc_subscription_init_default(&cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, cmd_callback, ON_NEW_DATA);

    // Tasks
    xTaskCreate(rpm_task, "rpm_task", 4096, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);

    while(1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void appMain(void *arg)
{
    // ESP-IDF calls this in some templates — redirect to app_main()
    app_main();
}
