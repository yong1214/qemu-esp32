/*
 * ESP32 GPIO Input Injector
 * 
 * Reads GPIO input injection commands from named pipe and sets GPIO input levels
 * for unified GPIO input injection architecture
 *
 * Copyright (c) 2025 Dustalon Project
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/gpio/esp32_gpio.h"
#include "hw/gpio/gpio_timing_executor.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>

typedef struct {
    Esp32GpioState *gpio;
    int pipe_fd;
    int initialized;
    int running;
    pthread_t thread;
    /* GTPE device references for runtime param updates */
    GteDevice *gte_devices;
    int gte_device_count;
} ESP32GPIOInject;

static ESP32GPIOInject *gpio_inject = NULL;

// Thread function to read from pipe and inject GPIO inputs
static void *esp32_gpio_inject_thread(void *arg)
{
    ESP32GPIOInject *inject = (ESP32GPIOInject *)arg;
    uint8_t buffer[8]; // Max message size: 8 bytes (analog with 2-byte value)
    
    qemu_log("🔧 ESP32 GPIO inject thread: Started\n");
    
    while (inject->running) {
        // Read header (6 bytes: magic + type + pin) to determine message length
        ssize_t bytes_read = read(inject->pipe_fd, buffer, 6);
        
        if (bytes_read < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available, sleep briefly and continue
                usleep(10000); // 10ms
                continue;
            } else if (errno == EPIPE || errno == EBADF) {
                // Pipe closed or invalid - exit thread
                qemu_log("🔧 ESP32 GPIO inject thread: Pipe closed, exiting\n");
                break;
            } else {
                // Other error - log and continue
                qemu_log("⚠️ ESP32 GPIO inject thread: Read error: %s\n", strerror(errno));
                usleep(100000); // 100ms before retry
                continue;
            }
        } else if (bytes_read == 0) {
            // EOF - pipe closed
            qemu_log("🔧 ESP32 GPIO inject thread: Pipe EOF, exiting\n");
            break;
        } else if (bytes_read < 6) {
            // Partial read - wait for more data
            continue;
        }
        
        // Verify magic bytes
        if (buffer[0] != 0x47 || buffer[1] != 0x50 || buffer[2] != 0x49 || buffer[3] != 0x4F) {
            // Invalid magic - skip one byte and try again
            qemu_log("⚠️ ESP32 GPIO inject: Invalid magic bytes, skipping\n");
            continue;
        }
        
        // Parse header: [Magic: 4 bytes] [Type: 1 byte] [Byte5: 1 byte]
        uint8_t type = buffer[4];

        if (type == 3) {
            /* ── GTPE Param Update ──────────────────────────────────────
             * Header already read (6 bytes):
             *   [4B magic] [type=0x03] [device_index]
             * Remaining 5 bytes:
             *   [param_index 1B] [value float32 LE 4B]
             */
            uint8_t dev_idx = buffer[5];
            ssize_t rest = read(inject->pipe_fd, buffer + 6, 5);
            if (rest < 5) continue;

            uint8_t param_idx = buffer[6];
            float value;
            memcpy(&value, buffer + 7, sizeof(float)); /* LE on both sides */

            if (inject->gte_devices && dev_idx < inject->gte_device_count) {
                gte_update_param(&inject->gte_devices[dev_idx], param_idx, value);
                qemu_log("🔌 GTPE param update: dev[%d].param[%d] = %.2f\n",
                         dev_idx, param_idx, value);
            } else {
                qemu_log("⚠️ GTPE param update: invalid dev_idx=%d (count=%d)\n",
                         dev_idx, inject->gte_device_count);
            }
            continue;
        }

        /* ── Standard GPIO inject (type 0/1/2) ────────────────────── */
        uint8_t pin = buffer[5];

        // Validate pin number (ESP32 has GPIO 0-39)
        if (pin > 39) {
            qemu_log("⚠️ ESP32 GPIO inject: Invalid pin number: %d\n", pin);
            continue;
        }

        // Determine message length based on type
        int value_length = (type == 2) ? 2 : 1; // Analog (2) = 2 bytes, Digital/PWM (0/1) = 1 byte

        // Read remaining bytes (value)
        if (value_length > 1) {
            bytes_read = read(inject->pipe_fd, buffer + 6, value_length);
            if (bytes_read < value_length) {
                // Partial read - skip this message
                continue;
            }
        }

        // Parse value based on type
        uint16_t value = 0;
        if (type == 2) {
            // Analog: 2 bytes (little-endian)
            value = buffer[6] | (buffer[7] << 8);
            qemu_log("🔌 ESP32 ADC inject: GPIO%d = %d (0x%04X) [12-bit ADC value]\n", pin, value, value);
        } else {
            // Digital/PWM: 1 byte
            value = buffer[6];
            bool high = (value != 0);
            esp32_gpio_set_input_level(inject->gpio, pin, high);
            qemu_log("🔌 ESP32 GPIO inject: GPIO%d = %s\n", pin, high ? "HIGH" : "LOW");
        }
    }
    
    qemu_log("🔧 ESP32 GPIO inject thread: Exiting\n");
    return NULL;
}

// Initialize GPIO injector
void esp32_gpio_inject_init(Esp32GpioState *gpio, const char *pipe_path)
{
    if (gpio_inject) {
        // Already initialized
        return;
    }
    
    if (!gpio || !pipe_path) {
        qemu_log("❌ ESP32 GPIO inject: Invalid parameters\n");
        return;
    }
    
    gpio_inject = g_new0(ESP32GPIOInject, 1);
    if (!gpio_inject) {
        qemu_log("❌ ESP32 GPIO inject: Failed to allocate memory\n");
        return;
    }
    
    gpio_inject->gpio = gpio;
    gpio_inject->pipe_fd = -1;
    gpio_inject->initialized = 0;
    gpio_inject->running = 0;
    gpio_inject->gte_devices = NULL;
    gpio_inject->gte_device_count = 0;
    
    // Create named pipe if it doesn't exist
    if (access(pipe_path, F_OK) != 0) {
        if (mkfifo(pipe_path, 0666) != 0) {
            qemu_log("❌ ESP32 GPIO inject: Failed to create pipe: %s\n", strerror(errno));
            g_free(gpio_inject);
            gpio_inject = NULL;
            return;
        }
    }
    
    // Open pipe for reading (non-blocking)
    gpio_inject->pipe_fd = open(pipe_path, O_RDONLY | O_NONBLOCK);
    if (gpio_inject->pipe_fd < 0) {
        qemu_log("❌ ESP32 GPIO inject: Failed to open pipe: %s\n", strerror(errno));
        g_free(gpio_inject);
        gpio_inject = NULL;
        return;
    }
    
    qemu_log("✅ ESP32 GPIO inject: Pipe opened: %s\n", pipe_path);
    
    // Start thread to read from pipe
    gpio_inject->running = 1;
    if (pthread_create(&gpio_inject->thread, NULL, esp32_gpio_inject_thread, gpio_inject) != 0) {
        qemu_log("❌ ESP32 GPIO inject: Failed to create thread: %s\n", strerror(errno));
        close(gpio_inject->pipe_fd);
        g_free(gpio_inject);
        gpio_inject = NULL;
        return;
    }
    
    gpio_inject->initialized = 1;
    qemu_log("✅ ESP32 GPIO inject: Initialized\n");
}

// Register GTPE devices for runtime param updates via inject pipe
void esp32_gpio_inject_register_gte(GteDevice *devices, int count)
{
    if (!gpio_inject) return;
    gpio_inject->gte_devices = devices;
    gpio_inject->gte_device_count = count;
    qemu_log("✅ ESP32 GPIO inject: registered %d GTPE device(s)\n", count);
}

// Cleanup GPIO injector
void esp32_gpio_inject_cleanup(void)
{
    if (!gpio_inject) {
        return;
    }
    
    // Stop thread
    gpio_inject->running = 0;
    
    // Close pipe (this will wake up the thread if it's blocking on read)
    if (gpio_inject->pipe_fd >= 0) {
        close(gpio_inject->pipe_fd);
        gpio_inject->pipe_fd = -1;
    }
    
    // Wait for thread to exit
    if (gpio_inject->thread) {
        pthread_join(gpio_inject->thread, NULL);
    }
    
    g_free(gpio_inject);
    gpio_inject = NULL;
    
    qemu_log("✅ ESP32 GPIO inject: Cleaned up\n");
}

