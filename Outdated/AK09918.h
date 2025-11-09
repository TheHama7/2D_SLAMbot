#ifndef _AK09918_H_
#define _AK09918_H_

#include <stdint.h>

#define AK09918_I2C_ADDR 0x0C

#define AK09918_WIA1  0x00
#define AK09918_WIA2  0x01
#define AK09918_HXL   0x11
#define AK09918_HXH   0x12
#define AK09918_HYL   0x13
#define AK09918_HYH   0x14
#define AK09918_HZL   0x15
#define AK09918_HZH   0x16
#define AK09918_CNTL2 0x31

#define AK09918_CONTINUOUS_100HZ 0x08

uint8_t AK09918_init(void);
uint8_t AK09918_read_raw(int16_t *mx, int16_t *my, int16_t *mz);

#endif
