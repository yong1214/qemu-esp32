#ifndef ADXL345_VDEV_H
#define ADXL345_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

typedef struct ADXL345VDev {
    VDevBase base;      /* MUST be first member */
    uint8_t i2c_addr7;
    uint8_t register_address;
    uint8_t read_phase;
    uint8_t power_ctl;
    uint8_t data_format;
    float accel_x;
    float accel_y;
    float accel_z;
    double noise_accel_amplitude;
    uint64_t last_tick_ns;
} ADXL345VDev;

ADXL345VDev* adxl345_vdev_create(uint8_t addr7, float initial_x,
                                   float initial_y, float initial_z);
const VDevI2COps* adxl345_vdev_get_i2c_ops(void);

#endif /* ADXL345_VDEV_H */
