#ifndef DS1307_VDEV_H
#define DS1307_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"
#include "hw/i2c/devices/rtc_common.h"

typedef struct DS1307VDev {
    VDevBase base;
    uint8_t i2c_addr7;
    RTCDateTimeBin t;
    bool oscillator_enabled;  /* inverted CH bit in seconds register */
    uint8_t control;          /* 0x07 control register */
    uint8_t reg_ptr;          /* address pointer */
    uint64_t last_update_ns;
} DS1307VDev;

DS1307VDev* ds1307_vdev_create(uint8_t addr7);
const VDevI2COps* ds1307_vdev_get_i2c_ops(void);

#endif /* DS1307_VDEV_H */


