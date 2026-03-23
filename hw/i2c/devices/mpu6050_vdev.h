#ifndef MPU6050_VDEV_H
#define MPU6050_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

typedef struct MPU6050VDev {
    VDevBase base;      /* MUST be first member */
    uint8_t i2c_addr7;
    uint8_t register_address;
    uint8_t read_phase;

    /* Config registers */
    uint8_t pwr_mgmt_1;    /* 0x6B */
    uint8_t pwr_mgmt_2;    /* 0x6C */
    uint8_t smplrt_div;    /* 0x19 */
    uint8_t config;        /* 0x1A */
    uint8_t gyro_config;   /* 0x1B */
    uint8_t accel_config;  /* 0x1C */

    /* Simulated sensor values */
    float accel_x;         /* g */
    float accel_y;         /* g */
    float accel_z;         /* g */
    float gyro_x;          /* deg/s */
    float gyro_y;          /* deg/s */
    float gyro_z;          /* deg/s */
    float temperature;     /* °C */

    double noise_accel;
    double noise_gyro;
    uint64_t last_tick_ns;
} MPU6050VDev;

MPU6050VDev* mpu6050_vdev_create(uint8_t addr7, float temp_c);
const VDevI2COps* mpu6050_vdev_get_i2c_ops(void);

#endif /* MPU6050_VDEV_H */
