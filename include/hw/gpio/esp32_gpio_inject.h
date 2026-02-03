/*
 * ESP32 GPIO Input Injector - Header file
 */

#ifndef HW_GPIO_ESP32_GPIO_INJECT_H
#define HW_GPIO_ESP32_GPIO_INJECT_H

#include "hw/gpio/esp32_gpio.h"

// Initialize GPIO injector
void esp32_gpio_inject_init(Esp32GpioState *gpio, const char *pipe_path);

// Cleanup GPIO injector
void esp32_gpio_inject_cleanup(void);

#endif


