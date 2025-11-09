#ifndef _QMI8658_H_
#define _QMI8658_H_

#include <stdint.h>

#define QMI8658_ADDR 0x6B

// Register map
#define Qmi8658Register_WhoAmI          0x00
#define Qmi8658Register_Reset           0x60
#define Qmi8658Register_Ctrl2           0x03
#define Qmi8658Register_Ctrl3           0x04
#define Qmi8658Register_Ctrl5           0x06
#define Qmi8658Register_Ctrl7           0x08

#define Qmi8658Register_Ax_L 0x35
#define Qmi8658Register_Ay_L 0x37
#define Qmi8658Register_Az_L 0x39
#define Qmi8658Register_Gx_L 0x3B
#define Qmi8658Register_Gy_L 0x3D
#define Qmi8658Register_Gz_L 0x3F

typedef struct {
    float imu[6];
    uint8_t slave;
} qmi8658_state;

uint8_t  QMI8658_begin(void);
void     QMI8658_read_sensor_data(float acc[3], float gyro[3]);
uint16_t QMI8658_readWord_reg(uint8_t reg);
uint8_t  QMI8658_read_reg(uint8_t reg);
void     QMI8658_write_reg(uint8_t reg, uint8_t val);

#endif
