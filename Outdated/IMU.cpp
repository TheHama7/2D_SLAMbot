#include "IMU.h"
#include "esp_log.h"
#include "i2c_utils.h"

static const char *TAG = "IMU";

void imuInit()
{
    i2c_init();

    if (!QMI8658_begin())
        ESP_LOGE(TAG, "QMI init fail");

    if (!AK09918_init())
        ESP_LOGE(TAG, "AK09918 init fail");
}

void imuDataGet(EulerAngles *ang,
                IMU_ST_SENSOR_DATA_FLOAT *gyro,
                IMU_ST_SENSOR_DATA_FLOAT *accel,
                IMU_ST_SENSOR_DATA *magn)
{
    float a[3], g[3];
    int16_t mx, my, mz;

    QMI8658_read_sensor_data(a, g);
    AK09918_read_raw(&mx, &my, &mz);

    accel->X = a[0]; accel->Y = a[1]; accel->Z = a[2];
    gyro->X  = g[0]; gyro->Y  = g[1]; gyro->Z  = g[2];

    magn->s16X = mx;
    magn->s16Y = my;
    magn->s16Z = mz;
}
