#ifndef INA219_VDEV_H
#define INA219_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

typedef struct INA219VDev {
    VDevBase base;      /* MUST be first member */
    uint8_t i2c_addr7;
    uint8_t register_address;
    uint8_t read_phase;
    uint16_t config;
    float bus_voltage;
    float shunt_voltage;
    float current;
    float power;
    uint16_t calibration;
    uint8_t write_buf[2];
    uint8_t write_pos;
    double noise_bus_amplitude;
    double noise_shunt_amplitude;
    uint64_t last_tick_ns;
} INA219VDev;

INA219VDev* ina219_vdev_create(uint8_t addr7, float initial_bus_v,
                                 float initial_shunt_v);
const VDevI2COps* ina219_vdev_get_i2c_ops(void);

#endif /* INA219_VDEV_H */
