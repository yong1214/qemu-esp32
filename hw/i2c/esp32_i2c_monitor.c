/*
 * ESP32 I2C Monitor
 *
 * Monitors I2C transactions and sends updates via named pipe.
 * Uses the unified 17-byte serial monitoring protocol:
 *
 *   Offset  Size  Field
 *   ------  ----  -----
 *   0       4     Magic: "SERL" (0x53 0x45 0x52 0x4C)
 *   4       1     Type:  0x01=I2C data, 0x03=I2C START, 0x04=I2C STOP
 *   5       1     Interface number (I2C0=0, I2C1=1)
 *   6       1     Direction: 0x00=Write, 0x01=Read
 *   7       1     Address: 7-bit I2C address
 *   8       1     Data byte
 *   9       8     Timestamp (LE uint64, microseconds)
 *   Total: 17 bytes
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
    uint8_t current_direction;  /* 0=write, 1=read */
    uint8_t current_interface;
} ESP32I2CMonitor;

static ESP32I2CMonitor *i2c_monitor = NULL;

static uint64_t get_timestamp_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static int ensure_fifo(const char *path)
{
    struct stat st;
    if (stat(path, &st) == 0) {
        if (S_ISFIFO(st.st_mode)) {
            return 0;
        }
        unlink(path);
    }
    if (mkfifo(path, 0666) != 0 && errno != EEXIST) {
        qemu_log("❌ ESP32 I2C monitor: mkfifo(%s) failed (errno=%d)\n",
                 path, errno);
        return -1;
    }
    return 0;
}

static void write_i2c_message(uint8_t type, uint8_t interface,
                              uint8_t direction, uint8_t address,
                              uint8_t data)
{
    if (!i2c_monitor || !i2c_monitor->initialized || i2c_monitor->pipe_fd < 0) {
        return;
    }

    uint64_t ts = get_timestamp_us();

    uint8_t msg[17] = {
        0x53, 0x45, 0x52, 0x4C,     /* Magic "SERL" */
        type,                         /* 0x01=data, 0x03=START, 0x04=STOP */
        interface,
        direction,
        address,                      /* 7-bit I2C address */
        data,
        (uint8_t)(ts         & 0xFF),
        (uint8_t)((ts >>  8) & 0xFF),
        (uint8_t)((ts >> 16) & 0xFF),
        (uint8_t)((ts >> 24) & 0xFF),
        (uint8_t)((ts >> 32) & 0xFF),
        (uint8_t)((ts >> 40) & 0xFF),
        (uint8_t)((ts >> 48) & 0xFF),
        (uint8_t)((ts >> 56) & 0xFF)
    };

    ssize_t written = write(i2c_monitor->pipe_fd, msg, 17);
    if (written < 0 && errno == EPIPE) {
        close(i2c_monitor->pipe_fd);
        i2c_monitor->pipe_fd = -1;
    }
}

/* ── public hooks ────────────────────────────────────────── */

void esp32_i2c_monitor_transaction_start(Esp32I2CState *s, uint8_t address,
                                          bool is_read)
{
    if (!i2c_monitor || !i2c_monitor->initialized) return;
    i2c_monitor->current_address   = address;
    i2c_monitor->current_direction = is_read ? 0x01 : 0x00;
    i2c_monitor->current_interface = s->unit_index;

    /* Emit START marker */
    write_i2c_message(0x03, s->unit_index,
                      i2c_monitor->current_direction,
                      address, 0x00);
}

void esp32_i2c_monitor_transaction_stop(Esp32I2CState *s)
{
    if (!i2c_monitor || !i2c_monitor->initialized) return;

    /* Emit STOP marker */
    write_i2c_message(0x04, i2c_monitor->current_interface,
                      i2c_monitor->current_direction,
                      i2c_monitor->current_address, 0x00);
}

void esp32_i2c_monitor_fifo_write(Esp32I2CState *s, uint8_t data)
{
    if (!i2c_monitor || !i2c_monitor->initialized) return;
    write_i2c_message(0x01, s->unit_index,
                      i2c_monitor->current_direction,
                      i2c_monitor->current_address, data);
}

void esp32_i2c_monitor_fifo_read(Esp32I2CState *s, uint8_t data)
{
    if (!i2c_monitor || !i2c_monitor->initialized) return;
    write_i2c_message(0x01, s->unit_index,
                      0x01,
                      i2c_monitor->current_address, data);
}

/* ── init / cleanup ──────────────────────────────────────── */

void esp32_i2c_monitor_init(const char *pipe_path)
{
    if (i2c_monitor) return;

    if (!pipe_path) {
        qemu_log("❌ ESP32 I2C monitor: Invalid parameters\n");
        return;
    }

    i2c_monitor = g_new0(ESP32I2CMonitor, 1);

    i2c_monitor->initialized = 0;
    i2c_monitor->pipe_fd     = -1;

    if (ensure_fifo(pipe_path) < 0) {
        g_free(i2c_monitor);
        i2c_monitor = NULL;
        return;
    }

    qemu_log("⏳ ESP32 I2C monitor: waiting for reader on %s ...\n", pipe_path);
    i2c_monitor->pipe_fd = open(pipe_path, O_WRONLY);
    if (i2c_monitor->pipe_fd < 0) {
        qemu_log("❌ ESP32 I2C monitor: open(%s) failed (errno=%d)\n",
                 pipe_path, errno);
        g_free(i2c_monitor);
        i2c_monitor = NULL;
        return;
    }
    fcntl(i2c_monitor->pipe_fd, F_SETFL, O_NONBLOCK);
    qemu_log("✅ ESP32 I2C monitor pipe opened\n");

    i2c_monitor->initialized = 1;
    qemu_log("✅ ESP32 I2C monitor initialized\n");
}

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
