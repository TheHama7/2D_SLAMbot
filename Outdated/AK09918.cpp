#include "AK09918.h"
#include "i2c_utils.h"
#include "esp_log.h"

static const char *TAG = "AK09918";

uint8_t AK09918_init(void)
{
    uint8_t id[2];
    i2c_read_reg(AK09918_I2C_ADDR, AK09918_WIA1, id, 2);

    if (id[0] == 0x48 && id[1] == 0x10)
    {
        ESP_LOGI(TAG, "AK09918 detected");
        uint8_t mode = AK09918_CONTINUOUS_100HZ;
        i2c_write_reg(AK09918_I2C_ADDR, AK09918_CNTL2, mode);
        return 1;
    }
    ESP_LOGE(TAG, "AK09918 not found");
    return 0;
}

uint8_t AK09918_read_raw(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t buf[6];
    i2c_read_reg(AK09918_I2C_ADDR, AK09918_HXL, buf, 6);

    *mx = (buf[1] << 8) | buf[0];
    *my = (buf[3] << 8) | buf[2];
    *mz = (buf[5] << 8) | buf[4];
    return 1;
}
