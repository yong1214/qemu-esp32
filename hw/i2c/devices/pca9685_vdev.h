#ifndef PCA9685_VDEV_H
#define PCA9685_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

typedef struct PCA9685VDev {
    VDevBase base;          /* MUST be first */
    uint8_t i2c_addr7;
    uint8_t register_address;
    uint8_t read_phase;
    uint8_t registers[256]; /* Full register space */
} PCA9685VDev;

PCA9685VDev* pca9685_vdev_create(uint8_t addr7);
const VDevI2COps* pca9685_vdev_get_i2c_ops(void);

#endif /* PCA9685_VDEV_H */
