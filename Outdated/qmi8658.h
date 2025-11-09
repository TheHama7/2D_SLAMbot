#ifndef QMI8658_H
#define QMI8658_H

#include <stdint.h>
#include <stdbool.h>

#define QMI8658_I2C_ADDR  0x6A   // sometimes 0x6B depending on hardware

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} qmi8658_data_t;

bool qmi8658_init();
bool qmi8658_read(qmi8658_data_t *data);

#endif
