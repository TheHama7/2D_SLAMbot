#include "ak09918.h"
#include "driver/i2c.h"

#define I2C_PORT I2C_NUM_0
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK09918_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // Read bytes
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK09918_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 20 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

bool AK09918_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ
    };

    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    // Check WHOAMI register (0x00)
    uint8_t whoami = 0;
    if (i2c_read_reg(0x00, &whoami, 1) != ESP_OK)
        return false;

    return true;
}

bool AK09918_read(ak09918_data_t *data)
{
    uint8_t raw[6];
    if (i2c_read_reg(0x11, raw, 6) != ESP_OK)
        return false;

    int16_t mx = (raw[1] << 8) | raw[0];
    int16_t my = (raw[3] << 8) | raw[2];
    int16_t mz = (raw[5] << 8) | raw[4];

    // rough scalers — can calibrate later
    data->x = mx * 0.15f;
    data->y = my * 0.15f;
    data->z = mz * 0.15f;

    return true;
}
