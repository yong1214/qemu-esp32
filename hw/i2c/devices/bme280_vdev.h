#ifndef BME280_VDEV_H
#define BME280_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

typedef struct BME280VDev {
    VDevBase base;      /* MUST be first member */
    uint8_t i2c_addr7;
    uint8_t register_address;
    uint8_t read_phase;
    uint8_t ctrl_meas;
    uint8_t config_reg;
    float temperature;
    float pressure;
    float humidity;
    double noise_temp_amplitude;
    double noise_press_amplitude;
    double noise_hum_amplitude;
    uint64_t last_tick_ns;
} BME280VDev;

BME280VDev* bme280_vdev_create(uint8_t addr7, float initial_temp,
                                float initial_pressure, float initial_humidity);
const VDevI2COps* bme280_vdev_get_i2c_ops(void);

#endif /* BME280_VDEV_H */
