/*
 * ESP32 SPI Monitor
 * 
 * Header file for SPI transaction monitoring
 */

#ifndef ESP32_SPI_MONITOR_H
#define ESP32_SPI_MONITOR_H

#include "hw/ssi/esp32_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize SPI monitor
void esp32_spi_monitor_init(const char *pipe_path);

// Monitor SPI transfer (called from esp32_spi_txrx_buffer)
void esp32_spi_monitor_transfer(Esp32SpiState *s, uint8_t tx_byte, uint8_t rx_byte);

// Cleanup SPI monitor
void esp32_spi_monitor_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_SPI_MONITOR_H */


