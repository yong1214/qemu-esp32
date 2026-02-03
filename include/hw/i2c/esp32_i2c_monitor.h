/*
 * ESP32 I2C Monitor
 * 
 * Header file for I2C transaction monitoring
 */

#ifndef ESP32_I2C_MONITOR_H
#define ESP32_I2C_MONITOR_H

#include "hw/i2c/esp32_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize I2C monitor
void esp32_i2c_monitor_init(const char *pipe_path);

// Monitor I2C FIFO write (called from esp32_i2c_write)
void esp32_i2c_monitor_fifo_write(Esp32I2CState *s, uint8_t data);

// Monitor I2C FIFO read (called from esp32_i2c_read)
void esp32_i2c_monitor_fifo_read(Esp32I2CState *s, uint8_t data);

// Monitor I2C transaction start (called when address is detected)
void esp32_i2c_monitor_transaction_start(Esp32I2CState *s, uint8_t address, bool is_read);

// Cleanup I2C monitor
void esp32_i2c_monitor_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_I2C_MONITOR_H */


