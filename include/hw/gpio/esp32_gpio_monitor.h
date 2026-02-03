/*
 * ESP32 GPIO Monitor - Header file
 *
 * Copyright (c) 2025 Dustalon Project
 */

#ifndef HW_GPIO_ESP32_GPIO_MONITOR_H
#define HW_GPIO_ESP32_GPIO_MONITOR_H

#include "hw/gpio/esp32_gpio.h"

// Initialize GPIO monitor
void esp32_gpio_monitor_init(Esp32GpioState *gpio, const char *pipe_path);

// Cleanup GPIO monitor
void esp32_gpio_monitor_cleanup(void);

#endif /* HW_GPIO_ESP32_GPIO_MONITOR_H */


