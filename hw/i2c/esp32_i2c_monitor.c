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
    uint8_t current_direction;  /* 0=write, 1=read */
} ESP32I2CMonitor;

static ESP32I2CMonitor *i2c_monitor = NULL;

/* ── helpers ──────────────────────────────────────────────── */

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

/* ── binary message ──────────────────────────────────────── */

/*
 * Format: [Magic 4B "SERL"] [Type 1B] [Iface 1B] [Dir 1B] [Data 1B] [TS 8B]
 *       =  4 + 1 + 1 + 1 + 1 + 8  =  16 bytes
 */
static void write_i2c_message(uint8_t interface, uint8_t direction, uint8_t data)
{
    if (!i2c_monitor || !i2c_monitor->initialized || i2c_monitor->pipe_fd < 0) {
        return;
    }

    uint64_t timestamp = get_timestamp_us();

    uint8_t msg[16] = {
        0x53, 0x45, 0x52, 0x4C,  /* "SERL" */
        0x01,                     /* Type: I2C */
        interface,
        direction,
        data,
        (uint8_t)(timestamp         & 0xFF),
        (uint8_t)((timestamp >>  8) & 0xFF),
        (uint8_t)((timestamp >> 16) & 0xFF),
        (uint8_t)((timestamp >> 24) & 0xFF),
        (uint8_t)((timestamp >> 32) & 0xFF),
        (uint8_t)((timestamp >> 40) & 0xFF),
        (uint8_t)((timestamp >> 48) & 0xFF),
        (uint8_t)((timestamp >> 56) & 0xFF)
    };

    ssize_t written = write(i2c_monitor->pipe_fd, msg, 16);
    if (written < 0 && errno == EPIPE) {
        close(i2c_monitor->pipe_fd);
        i2c_monitor->pipe_fd = -1;
    }
}

/* ── public hooks ────────────────────────────────────────── */

void esp32_i2c_monitor_fifo_write(Esp32I2CState *s, uint8_t data)
{
    if (!i2c_monitor || !i2c_monitor->initialized) return;
    write_i2c_message(s->unit_index, i2c_monitor->current_direction, data);
}

void esp32_i2c_monitor_fifo_read(Esp32I2CState *s, uint8_t data)
{
    if (!i2c_monitor || !i2c_monitor->initialized) return;
    write_i2c_message(s->unit_index, 0x01, data);
}

void esp32_i2c_monitor_transaction_start(Esp32I2CState *s, uint8_t address,
                                          bool is_read)
{
    if (!i2c_monitor || !i2c_monitor->initialized) return;
    i2c_monitor->current_address   = address;
    i2c_monitor->current_direction = is_read ? 0x01 : 0x00;
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
    if (!i2c_monitor) {
        qemu_log("❌ ESP32 I2C monitor: Failed to allocate memory\n");
        return;
    }

    i2c_monitor->initialized = 0;
    i2c_monitor->pipe_fd     = -1;
    i2c_monitor->current_address   = 0;
    i2c_monitor->current_direction = 0;

    if (ensure_fifo(pipe_path) < 0) {
        g_free(i2c_monitor);
        i2c_monitor = NULL;
        return;
    }

    /* Blocking open — waits for Node.js reader, then switch to non-blocking writes */
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
