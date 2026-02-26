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

/* ──────────────────────────────────────────────────────────
 *  GPIO pin change callback
 * ────────────────────────────────────────────────────────── */
static void esp32_gpio_monitor_callback(void *opaque, int pin, bool high)
{
    ESP32GPIOMonitor *monitor = (ESP32GPIOMonitor *)opaque;

    if (!monitor || !monitor->initialized || monitor->pipe_fd < 0) {
        return;
    }

    /* Binary GPIO message:
     *   [Magic 4B "GPIO"] [Type 1B] [Pin 1B] [Value 1B]  = 7 bytes */
    uint8_t msg[7] = {
        0x47, 0x50, 0x49, 0x4F,   /* "GPIO" */
        0x00,                      /* digital */
        (uint8_t)pin,
        (uint8_t)(high ? 1 : 0)
    };

    ssize_t written = write(monitor->pipe_fd, msg, 7);

    if (written < 0) {
        if (errno == EPIPE) {
            /* Reader closed — mark fd stale */
            close(monitor->pipe_fd);
            monitor->pipe_fd = -1;
        }
        /* EAGAIN / EWOULDBLOCK → pipe full, drop message (OK) */
    }
}

/* ──────────────────────────────────────────────────────────
 *  Ensure the FIFO exists at `path`.
 *  • If a FIFO already exists → reuse it (don't delete!)
 *  • If a non-FIFO file exists → unlink + mkfifo
 *  • If nothing exists → mkfifo
 * ────────────────────────────────────────────────────────── */
static int ensure_fifo(const char *path)
{
    struct stat st;
    if (stat(path, &st) == 0) {
        if (S_ISFIFO(st.st_mode)) {
            return 0;   /* already a FIFO – keep it */
        }
        /* Not a FIFO → remove and recreate */
        unlink(path);
    }

    if (mkfifo(path, 0666) != 0 && errno != EEXIST) {
        qemu_log("❌ ESP32 GPIO monitor: mkfifo(%s) failed (errno=%d)\n",
                 path, errno);
        return -1;
    }
    return 0;
}

/* ──────────────────────────────────────────────────────────
 *  Public: init
 * ────────────────────────────────────────────────────────── */
void esp32_gpio_monitor_init(Esp32GpioState *gpio, const char *pipe_path)
{
    if (gpio_monitor) {
        return;   /* already initialised */
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

    gpio_monitor->gpio        = gpio;
    gpio_monitor->initialized = 0;
    gpio_monitor->pipe_fd     = -1;

    /* Make sure the FIFO exists (but don't clobber an existing one) */
    if (ensure_fifo(pipe_path) < 0) {
        g_free(gpio_monitor);
        gpio_monitor = NULL;
        return;
    }

    /*
     * Blocking open (O_WRONLY without O_NONBLOCK).
     * This naturally waits for the Node.js reader to open the other end,
     * guaranteeing both sides are synced when open() returns.
     *
     * After open, switch the fd to non-blocking so write() never stalls
     * QEMU's main loop if the pipe buffer is full.
     */
    qemu_log("⏳ ESP32 GPIO monitor: waiting for reader on %s ...\n", pipe_path);
    gpio_monitor->pipe_fd = open(pipe_path, O_WRONLY);
    if (gpio_monitor->pipe_fd < 0) {
        qemu_log("❌ ESP32 GPIO monitor: open(%s) failed (errno=%d)\n",
                 pipe_path, errno);
        g_free(gpio_monitor);
        gpio_monitor = NULL;
        return;
    }

    /* Switch to non-blocking for writes */
    fcntl(gpio_monitor->pipe_fd, F_SETFL, O_NONBLOCK);

    qemu_log("✅ ESP32 GPIO monitor pipe opened: %s\n", pipe_path);

    /* Register listeners for all GPIO pins (0-39) */
    for (int pin = 0; pin < 40; pin++) {
        esp32_gpio_register_output_listener(gpio, pin,
                                            esp32_gpio_monitor_callback,
                                            gpio_monitor);
    }

    gpio_monitor->initialized = 1;
    qemu_log("✅ ESP32 GPIO monitor initialized (40 pins)\n");
}

/* ──────────────────────────────────────────────────────────
 *  Public: cleanup
 * ────────────────────────────────────────────────────────── */
void esp32_gpio_monitor_cleanup(void)
{
    if (gpio_monitor) {
        gpio_monitor->initialized = 0;

        if (gpio_monitor->gpio) {
            for (int pin = 0; pin < 40; pin++) {
                esp32_gpio_unregister_output_listener(
                    gpio_monitor->gpio, pin,
                    esp32_gpio_monitor_callback,
                    gpio_monitor);
            }
        }

        if (gpio_monitor->pipe_fd >= 0) {
            close(gpio_monitor->pipe_fd);
            gpio_monitor->pipe_fd = -1;
        }

        g_free(gpio_monitor);
        gpio_monitor = NULL;
    }
}
