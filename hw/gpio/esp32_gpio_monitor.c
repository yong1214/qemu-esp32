/*
 * ESP32 GPIO Monitor
 * 
 * Monitors GPIO output changes and sends updates via named pipe
 * for unified GPIO watcher architecture
 *
 * Copyright (c) 2025 Dustalon Project
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/gpio/esp32_gpio.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>

typedef struct {
    Esp32GpioState *gpio;
    int pipe_fd;
    int initialized;
} ESP32GPIOMonitor;

static ESP32GPIOMonitor *gpio_monitor = NULL;

// GPIO pin change callback
static void esp32_gpio_monitor_callback(void *opaque, int pin, bool high)
{
    ESP32GPIOMonitor *monitor = (ESP32GPIOMonitor *)opaque;
    
    // Safety check
    if (!monitor || !monitor->initialized || monitor->pipe_fd < 0) {
        return;
    }
    
    // Write binary GPIO message to pipe
    // Format: [Magic: 4 bytes] [Type: 1 byte] [Pin: 1 byte] [Value: 1 byte]
    uint8_t msg[7] = {
        0x47, 0x50, 0x49, 0x4F,  // Magic: "GPIO"
        0x00,                     // Type: digital (0)
        (uint8_t)pin,            // Pin number (0-39)
        (uint8_t)(high ? 1 : 0)  // Value (0 or 1)
    };
    
    // Write to pipe (non-blocking, so it won't hang if pipe is full)
    ssize_t written = write(monitor->pipe_fd, msg, 7);
    
    // Handle errors gracefully - don't crash if pipe is full or closed
    if (written < 0) {
        // EAGAIN/EWOULDBLOCK means pipe buffer is full - that's OK, just drop the message
        // EPIPE means pipe is closed - that's OK too, backend might have closed it
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EPIPE) {
            // Unexpected error, but don't crash - just silently continue
        }
    } else if (written != 7) {
        // Partial write - shouldn't happen with a pipe, but handle it gracefully
        // Just continue - next write will be a new message
    }
    // Success case - message written, continue normally
}

// Initialize GPIO monitor
void esp32_gpio_monitor_init(Esp32GpioState *gpio, const char *pipe_path)
{
    if (gpio_monitor) {
        // Already initialized
        return;
    }
    
    if (!gpio || !pipe_path) {
        qemu_log("❌ ESP32 GPIO monitor: Invalid parameters\n");
        return;
    }
    
    gpio_monitor = g_new0(ESP32GPIOMonitor, 1);
    if (!gpio_monitor) {
        qemu_log("❌ ESP32 GPIO monitor: Failed to allocate memory\n");
        return;
    }
    
    gpio_monitor->gpio = gpio;
    gpio_monitor->initialized = 0;
    gpio_monitor->pipe_fd = -1;
    
    // Create named pipe if it doesn't exist
    // If pipe exists, try to remove it first (might be stale from previous run)
    if (access(pipe_path, F_OK) == 0) {
        // Pipe exists - try to remove it (might be stale)
        if (unlink(pipe_path) != 0 && errno != ENOENT) {
            qemu_log("⚠️  ESP32 GPIO monitor: Could not remove existing pipe: %s (errno: %d)\n", 
                    pipe_path, errno);
            // Continue anyway - might still work
        }
    }
    
    // Create new pipe
    if (mkfifo(pipe_path, 0666) != 0) {
        if (errno != EEXIST) {
            qemu_log("❌ ESP32 GPIO monitor: Failed to create pipe: %s (errno: %d)\n", 
                    pipe_path, errno);
            g_free(gpio_monitor);
            gpio_monitor = NULL;
            return;
        }
        // EEXIST means pipe already exists (race condition), that's OK
    }
    
    // Open pipe for writing (non-blocking)
    gpio_monitor->pipe_fd = open(pipe_path, O_WRONLY | O_NONBLOCK);
    if (gpio_monitor->pipe_fd < 0) {
        qemu_log("⚠️  ESP32 GPIO monitor: Failed to open pipe: %s (will retry later)\n", pipe_path);
        // Don't fail - pipe might be opened later by backend
    } else {
        qemu_log("✅ ESP32 GPIO monitor pipe opened: %s\n", pipe_path);
    }
    
    // Register listeners for all GPIO pins (0-39)
    // Register BEFORE setting initialized flag to prevent callbacks during registration
    for (int pin = 0; pin < 40; pin++) {
        esp32_gpio_register_output_listener(gpio, pin, 
                                            esp32_gpio_monitor_callback, 
                                            gpio_monitor);
    }
    
    // Mark as initialized AFTER all listeners are registered
    // This prevents callbacks from being triggered during registration
    gpio_monitor->initialized = 1;
    
    qemu_log("✅ ESP32 GPIO monitor initialized (40 pins)\n");
}

// Cleanup GPIO monitor
void esp32_gpio_monitor_cleanup(void)
{
    if (gpio_monitor) {
        // Mark as not initialized to prevent callbacks from running during cleanup
        gpio_monitor->initialized = 0;
        
        // Unregister all listeners
        if (gpio_monitor->gpio) {
            for (int pin = 0; pin < 40; pin++) {
                esp32_gpio_unregister_output_listener(gpio_monitor->gpio, pin,
                                                      esp32_gpio_monitor_callback,
                                                      gpio_monitor);
            }
        }
        
        if (gpio_monitor->pipe_fd >= 0) {
            close(gpio_monitor->pipe_fd);
            gpio_monitor->pipe_fd = -1;
        }
        
        // Note: We don't unlink the pipe here because:
        // 1. Backend might still be reading from it
        // 2. Backend will clean it up when simulation stops
        // 3. Multiple simulations might share the same pipe path
        
        g_free(gpio_monitor);
        gpio_monitor = NULL;
    }
}


