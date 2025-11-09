#include "QMI8658.h"
#include "i2c_utils.h"
#include "esp_log.h"

static const char *TAG = "QMI8658";

static qmi8658_state g_imu;

static uint8_t qmi_read_u8(uint8_t reg)
{
    uint8_t d = 0;
    i2c_read_reg(QMI8658_ADDR, reg, &d, 1);
    return d;
}

static uint16_t qmi_read_u16(uint8_t reg)
{
    uint8_t buf[2];
    i2c_read_reg(QMI8658_ADDR, reg, buf, 2);
    return (buf[1] << 8) | buf[0];
}

static void qmi_write_u8(uint8_t reg, uint8_t val)
{
    i2c_write_reg(QMI8658_ADDR, reg, val);
}

uint8_t QMI8658_read_reg(uint8_t reg)
{
    return qmi_read_u8(reg);
}

uint16_t QMI8658_readWord_reg(uint8_t reg)
{
    return qmi_read_u16(reg);
}

void QMI8658_write_reg(uint8_t reg, uint8_t val)
{
    qmi_write_u8(reg, val);
}

unsigned char QMI8658_begin(void)
{
    uint8_t id = QMI8658_read_reg(Qmi8658Register_WhoAmI);

    if (id != 0x05)
    {
        ESP_LOGE(TAG, "QMI8658 not detected");
        return 0;
    }

    ESP_LOGI(TAG, "QMI8658 detected");

    // Full config copied from your Arduino version
    qmi_write_u8(Qmi8658Register_Reset, 0xb0);
    vTaskDelay(pdMS_TO_TICKS(10));

    qmi_write_u8(Qmi8658Register_Ctrl2, 0x60); // Accel
    qmi_write_u8(Qmi8658Register_Ctrl3, 0x60); // Gyro
    qmi_write_u8(Qmi8658Register_Ctrl5, 0x11);
    qmi_write_u8(Qmi8658Register_Ctrl7, 0x03); // Enable accel + gyro

    ESP_LOGI(TAG, "QMI8658 initialized");
    return 1;
}

void QMI8658_read_sensor_data(float acc[3], float gyro[3])
{
    int16_t ax = (int16_t) QMI8658_readWord_reg(Qmi8658Register_Ax_L);
    int16_t ay = (int16_t) QMI8658_readWord_reg(Qmi8658Register_Ay_L);
    int16_t az = (int16_t) QMI8658_readWord_reg(Qmi8658Register_Az_L);

    int16_t gx = (int16_t) QMI8658_readWord_reg(Qmi8658Register_Gx_L);
    int16_t gy = (int16_t) QMI8658_readWord_reg(Qmi8658Register_Gy_L);
    int16_t gz = (int16_t) QMI8658_readWord_reg(Qmi8658Register_Gz_L);

    acc[0] = ax / 2048.0f;
    acc[1] = ay / 2048.0f;
    acc[2] = az / 2048.0f;

    gyro[0] = gx / 16.4f;
    gyro[1] = gy / 16.4f;
    gyro[2] = gz / 16.4f;
}
