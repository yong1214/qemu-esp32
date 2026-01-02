#ifndef VIRTUAL_DEVICE_SPI_H
#define VIRTUAL_DEVICE_SPI_H

#include "qemu/osdep.h"
#include "hw/i2c/virtual_device.h"

typedef struct VDevSPIOps {
    /* Configure SPI mode and frequency (optional) */
    void (*set_mode)(VDevBase *device, uint8_t cpol, uint8_t cpha);
    void (*set_frequency)(VDevBase *device, uint32_t hz);

    /* Chip select control (optional) */
    void (*select)(VDevBase *device, bool cs_active);

    /* Full-duplex transfer: tx may be NULL to read-only; rx may be NULL to write-only */
    ssize_t (*transfer)(VDevBase *device, const uint8_t *tx, uint8_t *rx, size_t length);
} VDevSPIOps;

#endif /* VIRTUAL_DEVICE_SPI_H */


