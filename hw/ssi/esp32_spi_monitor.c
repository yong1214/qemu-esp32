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
#include "hw/sysbus.h"
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
        qemu_log("❌ ESP32 SPI monitor: mkfifo(%s) failed (errno=%d)\n",
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
static void write_spi_message(uint8_t interface, uint8_t direction, uint8_t data)
{
    if (!spi_monitor || !spi_monitor->initialized || spi_monitor->pipe_fd < 0) {
        return;
    }

    uint64_t timestamp = get_timestamp_us();

    uint8_t msg[16] = {
        0x53, 0x45, 0x52, 0x4C,  /* "SERL" */
        0x00,                     /* Type: SPI */
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

    ssize_t written = write(spi_monitor->pipe_fd, msg, 16);
    if (written < 0 && errno == EPIPE) {
        close(spi_monitor->pipe_fd);
        spi_monitor->pipe_fd = -1;
    }
}

/* ── public hook ─────────────────────────────────────────── */

void esp32_spi_monitor_transfer(Esp32SpiState *s, uint8_t tx_byte,
                                 uint8_t rx_byte)
{
    if (!spi_monitor || !spi_monitor->initialized) return;

    uint8_t interface = 0;
    if (s->unit_index == 2)      interface = 0;  /* HSPI */
    else if (s->unit_index == 3) interface = 1;  /* VSPI */
    else                         interface = s->unit_index;

    if (tx_byte != 0 || rx_byte != 0) {
        write_spi_message(interface, 0x00, tx_byte);  /* MOSI */
    }
    if (rx_byte != 0) {
        write_spi_message(interface, 0x01, rx_byte);  /* MISO */
    }
}

/* ── init / cleanup ──────────────────────────────────────── */

void esp32_spi_monitor_init(const char *pipe_path)
{
    if (spi_monitor) return;

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
    spi_monitor->pipe_fd     = -1;

    if (ensure_fifo(pipe_path) < 0) {
        g_free(spi_monitor);
        spi_monitor = NULL;
        return;
    }

    /* Blocking open — waits for Node.js reader, then switch to non-blocking writes */
    qemu_log("⏳ ESP32 SPI monitor: waiting for reader on %s ...\n", pipe_path);
    spi_monitor->pipe_fd = open(pipe_path, O_WRONLY);
    if (spi_monitor->pipe_fd < 0) {
        qemu_log("❌ ESP32 SPI monitor: open(%s) failed (errno=%d)\n",
                 pipe_path, errno);
        g_free(spi_monitor);
        spi_monitor = NULL;
        return;
    }
    fcntl(spi_monitor->pipe_fd, F_SETFL, O_NONBLOCK);
    qemu_log("✅ ESP32 SPI monitor pipe opened\n");

    spi_monitor->initialized = 1;
    qemu_log("✅ ESP32 SPI monitor initialized\n");
}

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
