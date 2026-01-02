#ifndef SPI_DEVICE_SET_H
#define SPI_DEVICE_SET_H

#include "qemu/osdep.h"
#include "hw/i2c/virtual_device.h"
#include "hw/spi/virtual_device_spi.h"

#define SPI_DEVICE_SET_MAX 32

typedef struct SPIDeviceKey { int bus_id; int cs; } SPIDeviceKey;

typedef struct SPIDeviceEntry {
    SPIDeviceKey key;
    VDevBase *device;
    const VDevSPIOps *ops;
} SPIDeviceEntry;

typedef struct SPIDeviceSet {
    SPIDeviceEntry entries[SPI_DEVICE_SET_MAX];
    int count;
} SPIDeviceSet;

static inline void spi_device_set_init(SPIDeviceSet *set) { memset(set, 0, sizeof(*set)); }

static inline bool spi_device_set_add(SPIDeviceSet *set, int bus_id, int cs, VDevBase *dev, const VDevSPIOps *ops)
{
    if (!set || !dev || !ops || set->count >= SPI_DEVICE_SET_MAX) return false;
    for (int i = 0; i < set->count; ++i) {
        if (set->entries[i].key.bus_id == bus_id && set->entries[i].key.cs == cs) return false;
    }
    set->entries[set->count].key.bus_id = bus_id;
    set->entries[set->count].key.cs = cs;
    set->entries[set->count].device = dev;
    set->entries[set->count].ops = ops;
    set->count++;
    return true;
}

static inline const SPIDeviceEntry* spi_device_set_find(const SPIDeviceSet *set, int bus_id, int cs)
{
    if (!set) return NULL;
    for (int i = 0; i < set->count; ++i) {
        if (set->entries[i].key.bus_id == bus_id && set->entries[i].key.cs == cs) return &set->entries[i];
    }
    return NULL;
}

#endif /* SPI_DEVICE_SET_H */


