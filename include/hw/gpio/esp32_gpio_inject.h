/*
 * ESP32 GPIO Input Injector - Header file
 */

#ifndef HW_GPIO_ESP32_GPIO_INJECT_H
#define HW_GPIO_ESP32_GPIO_INJECT_H

#include "hw/gpio/esp32_gpio.h"
#include "hw/gpio/gpio_timing_executor.h"

// Initialize GPIO injector
void esp32_gpio_inject_init(Esp32GpioState *gpio, const char *pipe_path);

// Register GTPE devices for runtime param updates via inject pipe
void esp32_gpio_inject_register_gte(GteDevice *devices, int count);

// Cleanup GPIO injector
void esp32_gpio_inject_cleanup(void);

#endif


