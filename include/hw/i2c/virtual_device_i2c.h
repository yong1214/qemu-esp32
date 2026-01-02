/**
 * Virtual I2C Device Operations Interface
 *
 * Defines the I2C-specific operation table for virtual devices that attach
 * to an I2C bus. Devices implement these hooks to participate in I2C
 * transactions driven by the controller.
 */

#ifndef VIRTUAL_DEVICE_I2C_H
#define VIRTUAL_DEVICE_I2C_H

#include "qemu/osdep.h"
#include "hw/i2c/virtual_device.h"

#define VDEV_I2C_ADDR_MASK 0x7F

typedef struct VDevI2COps {
    /* Address phase: whether this device responds to addr7 and rw */
    bool (*i2c_can_ack)(VDevBase *device, uint8_t addr7, bool is_read);

    /* Optional: notification that device was addressed */
    void (*i2c_on_addressed)(VDevBase *device, uint8_t addr7, bool is_read);

    /* Write phase: host writes bytes to device (returns bytes consumed or <0) */
    ssize_t (*i2c_write)(VDevBase *device, uint8_t addr7, const uint8_t *data, size_t length);

    /* Read phase: host requests bytes from device (returns bytes produced or <0) */
    ssize_t (*i2c_read)(VDevBase *device, uint8_t addr7, uint8_t *out, size_t length);
} VDevI2COps;

#endif /* VIRTUAL_DEVICE_I2C_H */


