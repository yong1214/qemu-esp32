/*
 * ESP32 SPI Monitor
 * 
 * Monitors SPI transactions and sends updates via named pipe
 * for unified serial monitoring architecture
 *
 * Copyright (c) 2025 Dustalon Project
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/ssi/esp32_spi.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <time.h>

typedef struct {
    int pipe_fd;
    int initialized;
} ESP32SPIMonitor;

static ESP32SPIMonitor *spi_monitor = NULL;

// Get current timestamp in microseconds
static uint64_t get_timestamp_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

// Write binary serial message to pipe
// Format: [Magic: 4 bytes] [Type: 1 byte] [Interface: 1 byte] [Direction: 1 byte] [Data: 1 byte] [Timestamp: 8 bytes]
static void write_spi_message(uint8_t interface, uint8_t direction, uint8_t data)
{
    if (!spi_monitor || !spi_monitor->initialized || spi_monitor->pipe_fd < 0) {
        return;
    }
    
    uint64_t timestamp = get_timestamp_us();
    
    uint8_t msg[15] = {
        0x53, 0x45, 0x52, 0x4C,  // Magic: "SERL"
        0x00,                     // Type: SPI (0x00)
        interface,                // Interface: SPI0=0 (HSPI), SPI1=1 (VSPI)
        direction,                // Direction: 0x00=MOSI, 0x01=MISO
        data,                     // Data byte
        // Timestamp (8 bytes, little-endian)
        (uint8_t)(timestamp & 0xFF),
        (uint8_t)((timestamp >> 8) & 0xFF),
        (uint8_t)((timestamp >> 16) & 0xFF),
        (uint8_t)((timestamp >> 24) & 0xFF),
        (uint8_t)((timestamp >> 32) & 0xFF),
        (uint8_t)((timestamp >> 40) & 0xFF),
        (uint8_t)((timestamp >> 48) & 0xFF),
        (uint8_t)((timestamp >> 56) & 0xFF)
    };
    
    ssize_t written = write(spi_monitor->pipe_fd, msg, 15);
    
    // Handle errors gracefully
    if (written < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EPIPE) {
            // Unexpected error, but don't crash
        }
    }
}

// Hook into SPI transaction - monitor data transfers
// This is called from esp32_spi_txrx_buffer
void esp32_spi_monitor_transfer(Esp32SpiState *s, uint8_t tx_byte, uint8_t rx_byte)
{
    if (!spi_monitor || !spi_monitor->initialized) {
        return;
    }
    
    // Determine interface number (SPI0=HSPI, SPI1=VSPI)
    // ESP32 has HSPI (SPI2) and VSPI (SPI3), but we map them as 0 and 1
    uint8_t interface = 0;
    if (s->unit_index == 2) {
        interface = 0;  // HSPI
    } else if (s->unit_index == 3) {
        interface = 1;  // VSPI
    } else {
        interface = s->unit_index;  // Fallback
    }
    
    // Monitor MOSI (master out)
    if (tx_byte != 0 || rx_byte != 0) {  // Only log non-zero transfers
        write_spi_message(interface, 0x00, tx_byte);  // MOSI
    }
    
    // Monitor MISO (master in) - only if we received data
    if (rx_byte != 0) {
        write_spi_message(interface, 0x01, rx_byte);  // MISO
    }
}

// Initialize SPI monitor
void esp32_spi_monitor_init(const char *pipe_path)
{
    if (spi_monitor) {
        // Already initialized
        return;
    }
    
    if (!pipe_path) {
        qemu_log("❌ ESP32 SPI monitor: Invalid parameters\n");
        return;
    }
    
    spi_monitor = g_new0(ESP32SPIMonitor, 1);
    if (!spi_monitor) {
        qemu_log("❌ ESP32 SPI monitor: Failed to allocate memory\n");
        return;
    }
    
    spi_monitor->initialized = 0;
    spi_monitor->pipe_fd = -1;
    
    // Create named pipe if it doesn't exist
    if (access(pipe_path, F_OK) == 0) {
        if (unlink(pipe_path) != 0 && errno != ENOENT) {
            qemu_log("⚠️  ESP32 SPI monitor: Could not remove existing pipe\n");
        }
    }
    
    // Create new pipe
    if (mkfifo(pipe_path, 0666) != 0) {
        if (errno != EEXIST) {
            qemu_log("❌ ESP32 SPI monitor: Failed to create pipe\n");
            g_free(spi_monitor);
            spi_monitor = NULL;
            return;
        }
    }
    
    // Open pipe for writing (non-blocking)
    spi_monitor->pipe_fd = open(pipe_path, O_WRONLY | O_NONBLOCK);
    if (spi_monitor->pipe_fd < 0) {
        qemu_log("⚠️  ESP32 SPI monitor: Failed to open pipe (will retry later)\n");
    } else {
        qemu_log("✅ ESP32 SPI monitor pipe opened\n");
    }
    
    spi_monitor->initialized = 1;
    qemu_log("✅ ESP32 SPI monitor initialized\n");
}

// Cleanup SPI monitor
void esp32_spi_monitor_cleanup(void)
{
    if (spi_monitor) {
        spi_monitor->initialized = 0;
        
        if (spi_monitor->pipe_fd >= 0) {
            close(spi_monitor->pipe_fd);
            spi_monitor->pipe_fd = -1;
        }
        
        g_free(spi_monitor);
        spi_monitor = NULL;
    }
}


