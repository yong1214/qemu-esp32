#ifndef DS3231_VDEV_H
#define DS3231_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"
#include "hw/i2c/devices/rtc_common.h"

typedef struct DS3231VDev {
    VDevBase base;
    uint8_t i2c_addr7;
    RTCDateTimeBin t;
    uint8_t control;  /* 0x0E */
    uint8_t status;   /* 0x0F */
    uint8_t reg_ptr;
    bool oscillator_enabled;
    uint64_t last_update_ns;
    /* Alarms */
    uint8_t a1_sec;   /* 0x07 */
    uint8_t a1_min;   /* 0x08 */
    uint8_t a1_hr;    /* 0x09 */
    uint8_t a1_dydt;  /* 0x0A */
    uint8_t a2_min;   /* 0x0B */
    uint8_t a2_hr;    /* 0x0C */
    uint8_t a2_dydt;  /* 0x0D */
    /* Temperature (quarter-degree resolution) */
    float temp_c;
    /* SQW/INT */
    bool sqw_state;
    uint64_t last_sqw_toggle_ns;
} DS3231VDev;

DS3231VDev* ds3231_vdev_create(uint8_t addr7);
const VDevI2COps* ds3231_vdev_get_i2c_ops(void);

#endif /* DS3231_VDEV_H */


