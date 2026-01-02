#ifndef EEPROM_VDEV_H
#define EEPROM_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

#define EEPROM_VDEV_MAX_BYTES 4096

typedef struct EEPROMVDev {
    VDevBase base;
    uint8_t i2c_addr7;
    uint8_t memory[EEPROM_VDEV_MAX_BYTES];
    uint16_t current_address;
    uint8_t write_protect;
    uint8_t page_size;
    uint8_t address_bytes;
    /* Write cycle emulation */
    uint32_t write_delay_ms;
    bool write_in_progress;
    uint64_t write_complete_time_ns;
    /* Variants and WP */
    uint16_t size_bytes;   /* effective size (<= EEPROM_VDEV_MAX_BYTES) */
    bool wp_enabled;       /* when true, block array writes */
} EEPROMVDev;

EEPROMVDev* eeprom_vdev_create(uint8_t addr7, size_t memory_size);
const VDevI2COps* eeprom_vdev_get_i2c_ops(void);

#endif /* EEPROM_VDEV_H */


