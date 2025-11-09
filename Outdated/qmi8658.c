#include "qmi8658.h"
#include "driver/i2c.h"
#include <math.h>

#define I2C_PORT I2C_NUM_0
#define SDA_PIN 21
#define SCL_PIN 22

static bool i2c_write(uint8_t reg, uint8_t val) {
    return i2c_master_write_to_device(I2C_PORT, QMI8658_I2C_ADDR, &reg, 1, 10/portTICK_PERIOD_MS) == ESP_OK
        && i2c_master_write_to_device(I2C_PORT, QMI8658_I2C_ADDR, &val, 1, 10/portTICK_PERIOD_MS) == ESP_OK;
}

static bool i2c_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_PORT, QMI8658_I2C_ADDR, &reg, 1, data, len, 10/portTICK_PERIOD_MS) == ESP_OK;
}

bool qmi8658_init() {
    // I2C setup
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_PORT, &cfg);
    i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0);

    // Try reading a known register to check connectivity
    uint8_t whoami;
    if (!i2c_read_bytes(0x00, &whoami, 1))
        return false;

    return true;
}

bool qmi8658_read(qmi8658_data_t *data) {
    uint8_t raw[12];

    if (!i2c_read_bytes(0x35, raw, 12))
        return false;

    int16_t ax = (raw[1] << 8) | raw[0];
    int16_t ay = (raw[3] << 8) | raw[2];
    int16_t az = (raw[5] << 8) | raw[4];
    int16_t gx = (raw[7] << 8) | raw[6];
    int16_t gy = (raw[9] << 8) | raw[8];
    int16_t gz = (raw[11] << 8) | raw[10];

    data->accel_x = ax / 16384.0f;
    data->accel_y = ay / 16384.0f;
    data->accel_z = az / 16384.0f;
    data->gyro_x  = gx / 131.0f;
    data->gyro_y  = gy / 131.0f;
    data->gyro_z  = gz / 131.0f;

    return true;
}
