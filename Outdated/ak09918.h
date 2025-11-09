#ifndef AK09918_H
#define AK09918_H

#include <stdint.h>
#include <stdbool.h>

#define AK09918_I2C_ADDR 0x0C

typedef struct {
    float x;
    float y;
    float z;
} ak09918_data_t;

bool AK09918_init();
bool AK09918_read(ak09918_data_t *data);

#endif
