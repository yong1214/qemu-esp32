#ifndef AHT20_VDEV_H
#define AHT20_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

typedef struct AHT20VDev {
    VDevBase base;      /* MUST be first member */
    uint8_t i2c_addr7;
    float temperature;
    float humidity;
    bool calibrated;
    bool measuring;
    uint8_t write_buf[3];
    uint8_t write_pos;
    double noise_temp_amplitude;
    double noise_hum_amplitude;
    uint64_t last_tick_ns;
} AHT20VDev;

AHT20VDev* aht20_vdev_create(uint8_t addr7, float initial_temp,
                               float initial_humidity);
const VDevI2COps* aht20_vdev_get_i2c_ops(void);

#endif /* AHT20_VDEV_H */
