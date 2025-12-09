#include "lsm6ds0.h"
#include "twi.h"
#include <stdint.h>
#include <limits.h>

#define LSM6DS0_ADDR 0x6B
#define OUTX_L_XL    0x28
#define OUTX_L_G     0x22
#define CTRL1_XL     0x10
#define CTRL2_G      0x11
#define WHO_AM_I     0x0F
#define WHO_AM_I_VAL 0x6C  // LSM6DSO value

#define ACC_UG_2G   61
#define ACC_UG_4G   122
#define ACC_UG_8G   244
#define ACC_UG_16G  732

#define GYR_SENS_245   875
#define GYR_SENS_500   1750
#define GYR_SENS_2000  7000

#define MAX_RETRIES 3

static uint8_t i2c_error_count = 0;

static uint8_t write_reg(uint8_t reg, uint8_t val)
{
    for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
        if (!TWI0_start((LSM6DS0_ADDR << 1) | 0)) {
            TWI0_stop();
            TWI0_reset();
            continue;
        }
        if (!TWI0_write(reg)) {
            TWI0_stop();
            TWI0_reset();
            continue;
        }
        if (!TWI0_write(val)) {
            TWI0_stop();
            TWI0_reset();
            continue;
        }
        TWI0_stop();
        return 1;
    }
    i2c_error_count++;
    return 0;
}

static uint8_t read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
        if (!TWI0_start((LSM6DS0_ADDR << 1) | 0)) {
            TWI0_stop();
            TWI0_reset();
            continue;
        }
        if (!TWI0_write(reg)) {
            TWI0_stop();
            TWI0_reset();
            continue;
        }
        
        if (!TWI0_start((LSM6DS0_ADDR << 1) | 1)) {
            TWI0_stop();
            TWI0_reset();
            continue;
        }
        
        for (uint8_t i = 0; i < len - 1; i++)
            buf[i] = TWI0_read_ack();
        buf[len - 1] = TWI0_read_nack();
        TWI0_stop();
        return 1;
    }
    
    i2c_error_count++;
    for (uint8_t i = 0; i < len; i++) buf[i] = 0;
    return 0;
}

static uint8_t read_reg(uint8_t reg)
{
    uint8_t value = 0;
    read_regs(reg, &value, 1);
    return value;
}

uint8_t LSM6DS0_init(void)
{
    i2c_error_count = 0;
    
    uint8_t whoami = read_reg(WHO_AM_I);
    if (whoami != WHO_AM_I_VAL) {
        TWI0_reset();
        whoami = read_reg(WHO_AM_I);
        if (whoami != WHO_AM_I_VAL) {
            return 0;
        }
    }
    
    if (!write_reg(CTRL1_XL, 0x40)) return 0;
    if (!write_reg(CTRL2_G, 0x40)) return 0;
    
    return 1;
}

static uint8_t LSM6DS0_get_accel_fs_g(void)
{
    uint8_t ctrl1 = read_reg(CTRL1_XL);
    uint8_t fs_bits = (ctrl1 >> 2) & 0x03;
    switch (fs_bits) {
        case 0b00: return 2;
        case 0b01: return 16;
        case 0b10: return 4;
        case 0b11: return 8;
        default:   return 2;
    }
}

static uint16_t LSM6DS0_get_gyro_fs_dps(void)
{
    uint8_t ctrl2 = read_reg(CTRL2_G);
    uint8_t fs_bits = (ctrl2 >> 2) & 0x03;
    switch (fs_bits) {
        case 0b00: return 245;
        case 0b01: return 500;
        case 0b10: return 1000;
        case 0b11: return 2000;
        default:   return 245;
    }
}

static int16_t lsm6ds0_acc_to_mg(int16_t raw, uint8_t fs_g)
{
    int32_t sens_ug;
    switch (fs_g) {
        case 2:  sens_ug = ACC_UG_2G;  break;
        case 4:  sens_ug = ACC_UG_4G;  break;
        case 8:  sens_ug = ACC_UG_8G;  break;
        case 16: sens_ug = ACC_UG_16G; break;
        default: sens_ug = ACC_UG_2G;  break;
    }
    int32_t mg = ((int32_t)raw * sens_ug) / 1000;
    if (mg > INT16_MAX) return INT16_MAX;
    if (mg < INT16_MIN) return INT16_MIN;
    return (int16_t)mg;
}

static int16_t lsm6ds0_gyro_to_dps(int16_t raw, uint16_t fs_dps)
{
    int32_t sens;
    switch (fs_dps) {
        case 245:  sens = 875;  break;
        case 500:  sens = 1750; break;
        case 1000: sens = 3500; break;
        case 2000: sens = 7000; break;
        default:   sens = 875;  break;
    }
    int32_t dps = (int32_t)raw * sens;
    return (int16_t)(dps / 1000);
}

uint8_t LSM6DS0_read_accel_mg(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t fs = LSM6DS0_get_accel_fs_g();
    uint8_t buf[6];
    
    if (!read_regs(OUTX_L_XL, buf, 6)) {
        *ax = 0;
        *ay = 0;
        *az = 0;
        return 0;
    }
    
    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);
    
    *ax = lsm6ds0_acc_to_mg(raw_x, fs);
    *ay = lsm6ds0_acc_to_mg(raw_y, fs);
    *az = lsm6ds0_acc_to_mg(raw_z, fs);
    
    return 1;
}

uint8_t LSM6DS0_read_gyro_dps(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint16_t fs_dps = LSM6DS0_get_gyro_fs_dps();
    uint8_t buf[6];
    
    if (!read_regs(OUTX_L_G, buf, 6)) {
        *gx = 0;
        *gy = 0;
        *gz = 0;
        return 0;
    }
    
    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);
    
    *gx = lsm6ds0_gyro_to_dps(raw_x, fs_dps);
    *gy = lsm6ds0_gyro_to_dps(raw_y, fs_dps);
    *gz = lsm6ds0_gyro_to_dps(raw_z, fs_dps);
    
    return 1;
}

uint8_t LSM6DS0_get_error_count(void)
{
    return i2c_error_count;
}