/*
 * ESP32 I2C Monitor
 * 
 * Monitors I2C transactions and sends updates via named pipe
 * for unified serial monitoring architecture
 *
 * Copyright (c) 2025 Dustalon Project
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/i2c/esp32_i2c.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <time.h>

typedef struct {
    int pipe_fd;
    int initialized;
    uint8_t current_address;
    uint8_t current_direction;  // 0=write, 1=read
} ESP32I2CMonitor;

static ESP32I2CMonitor *i2c_monitor = NULL;

// Get current timestamp in microseconds
static uint64_t get_timestamp_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

// Write binary serial message to pipe
// Format: [Magic: 4 bytes] [Type: 1 byte] [Interface: 1 byte] [Direction: 1 byte] [Data: 1 byte] [Timestamp: 8 bytes]
static void write_i2c_message(uint8_t interface, uint8_t direction, uint8_t data)
{
    if (!i2c_monitor || !i2c_monitor->initialized || i2c_monitor->pipe_fd < 0) {
        return;
    }
    
    uint64_t timestamp = get_timestamp_us();
    
    uint8_t msg[15] = {
        0x53, 0x45, 0x52, 0x4C,  // Magic: "SERL"
        0x01,                     // Type: I2C (0x01)
        interface,                // Interface: I2C0=0, I2C1=1
        direction,                // Direction: 0x00=Write, 0x01=Read
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
    
    ssize_t written = write(i2c_monitor->pipe_fd, msg, 15);
    
    // Handle errors gracefully
    if (written < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EPIPE) {
            // Unexpected error, but don't crash
        }
    }
}

// Monitor I2C FIFO data write (called from esp32_i2c_write)
void esp32_i2c_monitor_fifo_write(Esp32I2CState *s, uint8_t data)
{
    if (!i2c_monitor || !i2c_monitor->initialized) {
        return;
    }
    
    // Determine interface number
    uint8_t interface = s->unit_index;  // I2C0=0, I2C1=1
    
    // Use stored direction
    uint8_t direction = i2c_monitor->current_direction;
    
    // Write transaction message
    write_i2c_message(interface, direction, data);
}

// Monitor I2C FIFO data read (called from esp32_i2c_read)
void esp32_i2c_monitor_fifo_read(Esp32I2CState *s, uint8_t data)
{
    if (!i2c_monitor || !i2c_monitor->initialized) {
        return;
    }
    
    // Determine interface number
    uint8_t interface = s->unit_index;  // I2C0=0, I2C1=1
    
    // Read direction
    write_i2c_message(interface, 0x01, data);  // Read
}

// Monitor I2C transaction start (called when address is detected)
void esp32_i2c_monitor_transaction_start(Esp32I2CState *s, uint8_t address, bool is_read)
{
    if (!i2c_monitor || !i2c_monitor->initialized) {
        return;
    }
    
    // Store current transaction info
    i2c_monitor->current_address = address;
    i2c_monitor->current_direction = is_read ? 0x01 : 0x00;
}

// Initialize I2C monitor
void esp32_i2c_monitor_init(const char *pipe_path)
{
    if (i2c_monitor) {
        // Already initialized
        return;
    }
    
    if (!pipe_path) {
        qemu_log("❌ ESP32 I2C monitor: Invalid parameters\n");
        return;
    }
    
    i2c_monitor = g_new0(ESP32I2CMonitor, 1);
    if (!i2c_monitor) {
        qemu_log("❌ ESP32 I2C monitor: Failed to allocate memory\n");
        return;
    }
    
    i2c_monitor->initialized = 0;
    i2c_monitor->pipe_fd = -1;
    i2c_monitor->current_address = 0;
    i2c_monitor->current_direction = 0;
    
    // Create named pipe if it doesn't exist
    if (access(pipe_path, F_OK) == 0) {
        if (unlink(pipe_path) != 0 && errno != ENOENT) {
            qemu_log("⚠️  ESP32 I2C monitor: Could not remove existing pipe\n");
        }
    }
    
    // Create new pipe
    if (mkfifo(pipe_path, 0666) != 0) {
        if (errno != EEXIST) {
            qemu_log("❌ ESP32 I2C monitor: Failed to create pipe\n");
            g_free(i2c_monitor);
            i2c_monitor = NULL;
            return;
        }
    }
    
    // Open pipe for writing (non-blocking)
    i2c_monitor->pipe_fd = open(pipe_path, O_WRONLY | O_NONBLOCK);
    if (i2c_monitor->pipe_fd < 0) {
        qemu_log("⚠️  ESP32 I2C monitor: Failed to open pipe (will retry later)\n");
    } else {
        qemu_log("✅ ESP32 I2C monitor pipe opened\n");
    }
    
    i2c_monitor->initialized = 1;
    qemu_log("✅ ESP32 I2C monitor initialized\n");
}

// Cleanup I2C monitor
void esp32_i2c_monitor_cleanup(void)
{
    if (i2c_monitor) {
        i2c_monitor->initialized = 0;
        
        if (i2c_monitor->pipe_fd >= 0) {
            close(i2c_monitor->pipe_fd);
            i2c_monitor->pipe_fd = -1;
        }
        
        g_free(i2c_monitor);
        i2c_monitor = NULL;
    }
}


