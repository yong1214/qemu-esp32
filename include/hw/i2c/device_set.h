/* Simple I2C virtual device registry mapping 7-bit addresses to VDev devices */
#ifndef I2C_DEVICE_SET_H
#define I2C_DEVICE_SET_H

#include "qemu/osdep.h"
#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

#define I2C_DEVICE_SET_MAX 32

typedef struct I2CDeviceEntry {
    uint8_t address7;
    VDevBase *device;
    const VDevI2COps *ops;
} I2CDeviceEntry;

typedef struct I2CDeviceSet {
    I2CDeviceEntry entries[I2C_DEVICE_SET_MAX];
    int count;
} I2CDeviceSet;

static inline void i2c_device_set_init(I2CDeviceSet *set)
{
    memset(set, 0, sizeof(*set));
}

static inline bool i2c_device_set_add(I2CDeviceSet *set, uint8_t addr7, VDevBase *dev, const VDevI2COps *ops)
{
    if (!set || !dev || !ops || set->count >= I2C_DEVICE_SET_MAX) return false;
    for (int i = 0; i < set->count; ++i) {
        if (set->entries[i].address7 == addr7) return false;
    }
    set->entries[set->count].address7 = addr7 & VDEV_I2C_ADDR_MASK;
    set->entries[set->count].device = dev;
    set->entries[set->count].ops = ops;
    set->count++;
    return true;
}

static inline bool i2c_device_set_remove(I2CDeviceSet *set, uint8_t addr7)
{
    if (!set) return false;
    for (int i = 0; i < set->count; ++i) {
        if (set->entries[i].address7 == (addr7 & VDEV_I2C_ADDR_MASK)) {
            for (int j = i; j < set->count - 1; ++j) set->entries[j] = set->entries[j + 1];
            set->count--;
            return true;
        }
    }
    return false;
}

static inline const I2CDeviceEntry* i2c_device_set_find(const I2CDeviceSet *set, uint8_t addr7)
{
    if (!set) return NULL;
    for (int i = 0; i < set->count; ++i) {
        if (set->entries[i].address7 == (addr7 & VDEV_I2C_ADDR_MASK)) return &set->entries[i];
    }
    return NULL;
}

#endif /* I2C_DEVICE_SET_H */


