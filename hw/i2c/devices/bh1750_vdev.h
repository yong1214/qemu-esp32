#ifndef BH1750_VDEV_H
#define BH1750_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

typedef struct BH1750VDev {
    VDevBase base;      /* MUST be first member */
    uint8_t i2c_addr7;
    uint8_t mode;
    bool powered_on;
    float lux;
    double noise_lux_amplitude;
    uint64_t last_tick_ns;
} BH1750VDev;

BH1750VDev* bh1750_vdev_create(uint8_t addr7, float initial_lux);
const VDevI2COps* bh1750_vdev_get_i2c_ops(void);

#endif /* BH1750_VDEV_H */
