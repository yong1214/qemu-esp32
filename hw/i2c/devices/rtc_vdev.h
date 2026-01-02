#ifndef RTC_VDEV_H
#define RTC_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

typedef struct RTCVDev {
    VDevBase base;
    uint8_t i2c_addr7;
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t control_register;
    uint8_t register_address;
    bool oscillator_enabled;
    uint64_t last_update_ns;
} RTCVDev;

RTCVDev* rtc_vdev_create(uint8_t addr7);
const VDevI2COps* rtc_vdev_get_i2c_ops(void);

#endif /* RTC_VDEV_H */


