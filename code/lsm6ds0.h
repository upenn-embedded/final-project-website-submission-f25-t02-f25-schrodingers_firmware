#ifndef LSM6DS0_H
#define LSM6DS0_H
#include <stdint.h>

// These functions work with LSM6DSO sensor (WHO_AM_I = 0x6C) at address 0x6B
uint8_t LSM6DS0_init(void);
uint8_t LSM6DS0_read_accel_mg(int16_t *ax, int16_t *ay, int16_t *az);
uint8_t LSM6DS0_read_gyro_dps(int16_t *gx, int16_t *gy, int16_t *gz);
uint8_t LSM6DS0_get_error_count(void);

#endif