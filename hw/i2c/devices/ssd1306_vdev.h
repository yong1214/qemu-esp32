#ifndef SSD1306_VDEV_H
#define SSD1306_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "hw/i2c/virtual_device_i2c.h"

typedef struct SSD1306VDev {
    VDevBase base;
    uint8_t i2c_addr7;
    uint8_t display_buffer[1024];
    uint8_t current_column;
    uint8_t current_page;
    uint8_t display_mode;
    uint8_t brightness;
    bool display_on;
    bool command_mode;
} SSD1306VDev;

SSD1306VDev* ssd1306_vdev_create(uint8_t addr7);
const VDevI2COps* ssd1306_vdev_get_i2c_ops(void);

#endif /* SSD1306_VDEV_H */


