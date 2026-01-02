#ifndef TMP105_VDEV_H
#define TMP105_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

typedef struct TMP105VDev {
    VDevBase base;
    uint8_t i2c_addr7;
    float temperature;
    uint8_t config_register;
    uint8_t resolution;
    uint8_t register_address;
    bool shutdown_mode;
    uint32_t conversion_time_us;
    uint8_t read_phase;
    double drift_c_per_sec;
    double noise_c_amplitude;
    float temp_min_c;
    float temp_max_c;
    uint64_t last_conversion_time_ns;
} TMP105VDev;

TMP105VDev* tmp105_vdev_create(uint8_t addr7, float initial_temp);
const VDevI2COps* tmp105_vdev_get_i2c_ops(void);

#endif /* TMP105_VDEV_H */


