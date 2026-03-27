/*
 * ESP32 RMT (Remote Control Transceiver) — QEMU peripheral
 *
 * Phase 1: RX capture only. Firmware configures an RX channel, the
 * GTPE provides the pulse timeline, and this device writes rmt_item32_t
 * entries to channel RAM so the firmware can read them.
 *
 * This bypasses QEMU's GPIO MMIO speed limitation (~23µs per read)
 * which prevents bit-bang protocols like DHT11 from working.
 *
 * Copyright (c) 2025-2026 Dustalon Project
 */

#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_RMT "misc.esp32.rmt"
#define ESP32_RMT(obj) OBJECT_CHECK(Esp32RmtState, (obj), TYPE_ESP32_RMT)

/* ── Constants ──────────────────────────────────────────────────────────── */

#define ESP32_RMT_CHANNEL_CNT   8
#define ESP32_RMT_ITEMS_PER_CH  64   /* 64 × 32-bit items per channel */
#define ESP32_RMT_REGS_SIZE     0x1000

/* APB clock for tick calculations */
#define ESP32_RMT_APB_CLK_HZ   80000000U

/* ── Register offsets ───────────────────────────────────────────────────── */

/* Data registers (FIFO access to channel RAM) */
#define A_RMT_CH0DATA       0x0000
/* stride 4: CH1=0x04, CH2=0x08 ... CH7=0x1C */

/* Channel configuration: CONF0 at 0x20 + ch*8, CONF1 at 0x24 + ch*8 */
#define A_RMT_CH0CONF0      0x0020
#define A_RMT_CH0CONF1      0x0024
#define ESP32_RMT_CONF_STRIDE  8

/* Channel status */
#define A_RMT_CH0STATUS     0x0060
/* stride 4: CH1=0x64, ... CH7=0x7C */

/* Carrier duty (Phase 2 — stub for now) */
#define A_RMT_CH0CARRIER_DUTY  0x0080
/* stride 4 */

/* Interrupt registers */
#define A_RMT_INT_RAW      0x00A0
#define A_RMT_INT_ST       0x00A4
#define A_RMT_INT_ENA      0x00A8
#define A_RMT_INT_CLR      0x00AC

/* APB configuration */
#define A_RMT_APB_CONF     0x00C0

/* TX limit per channel */
#define A_RMT_CH0TX_LIM    0x00D0
/* stride 4 */

/* Channel RAM base (8 channels × 64 items × 4 bytes = 2048 bytes) */
#define A_RMT_CH_MEM_BASE  0x0800
#define ESP32_RMT_CH_MEM_SIZE  (ESP32_RMT_ITEMS_PER_CH * 4)  /* 256 bytes */

/* ── CONF0 bit fields ──────────────────────────────────────────────────── */

#define RMT_CONF0_DIV_CNT_SHIFT     0
#define RMT_CONF0_DIV_CNT_MASK      0xFF
#define RMT_CONF0_IDLE_THRES_SHIFT  8
#define RMT_CONF0_IDLE_THRES_MASK   0xFFFF00
#define RMT_CONF0_MEM_SIZE_SHIFT    24
#define RMT_CONF0_MEM_SIZE_MASK     0x0F000000
#define RMT_CONF0_CARRIER_EN        BIT(28)
#define RMT_CONF0_CARRIER_OUT_LV    BIT(29)
#define RMT_CONF0_MEM_PD            BIT(30)
#define RMT_CONF0_CLK_EN            BIT(31)

/* ── CONF1 bit fields ──────────────────────────────────────────────────── */

#define RMT_CONF1_TX_START          BIT(0)
#define RMT_CONF1_RX_EN             BIT(1)
#define RMT_CONF1_MEM_WR_RST        BIT(2)
#define RMT_CONF1_MEM_RD_RST        BIT(3)
#define RMT_CONF1_APB_MEM_RST       BIT(4)
#define RMT_CONF1_MEM_OWNER         BIT(5)
#define RMT_CONF1_TX_CONTI_MODE     BIT(6)
#define RMT_CONF1_RX_FILTER_EN      BIT(7)
#define RMT_CONF1_RX_FILTER_THRES_SHIFT 8
#define RMT_CONF1_RX_FILTER_THRES_MASK  0xFF00
#define RMT_CONF1_REF_CNT_RST       BIT(16)
#define RMT_CONF1_REF_ALWAYS_ON      BIT(17)
#define RMT_CONF1_IDLE_OUT_LV        BIT(18)
#define RMT_CONF1_IDLE_OUT_EN        BIT(19)

/* ── Interrupt bits (per channel) ───────────────────────────────────────── */

/* Bits in INT_RAW/INT_ST/INT_ENA/INT_CLR:
 *   [ch*3 + 0] = ch_tx_end
 *   [ch*3 + 1] = ch_rx_end
 *   [ch*3 + 2] = ch_err
 *   [24 + ch]  = ch_tx_thr_event
 */
#define RMT_INT_CH_TX_END(ch)       BIT((ch) * 3)
#define RMT_INT_CH_RX_END(ch)       BIT((ch) * 3 + 1)
#define RMT_INT_CH_ERR(ch)          BIT((ch) * 3 + 2)
#define RMT_INT_CH_TX_THR(ch)       BIT(24 + (ch))

/* ── APB_CONF bit fields ───────────────────────────────────────────────── */

#define RMT_APB_CONF_FIFO_MASK      BIT(0)
#define RMT_APB_CONF_MEM_TX_WRAP    BIT(1)

/* ── Per-channel state ──────────────────────────────────────────────────── */

typedef struct {
    uint32_t conf0;
    uint32_t conf1;
    uint32_t status;
    uint32_t carrier_duty;
    uint32_t tx_lim;

    /* Channel RAM (64 items × 32 bits) */
    uint32_t mem[ESP32_RMT_ITEMS_PER_CH];

    /* FIFO read pointer for CHnDATA register access */
    int fifo_rd_ptr;

    /* RX state */
    bool rx_active;
    int gpio_pin;          /* configured GPIO pin (-1 if not set) */
} Esp32RmtChannel;

/* ── Device state ───────────────────────────────────────────────────────── */

typedef struct Esp32RmtState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    Esp32RmtChannel channel[ESP32_RMT_CHANNEL_CNT];

    /* Interrupt registers */
    uint32_t int_raw;
    uint32_t int_ena;

    /* APB configuration */
    uint32_t apb_conf;

    /* IRQ output to interrupt matrix */
    qemu_irq irq;

    /* GTPE device references for RX capture */
    void *gte_devices;     /* GteDevice array */
    int gte_device_count;
} Esp32RmtState;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * Register GTPE devices with the RMT peripheral for RX capture.
 * When firmware enables RX on a channel, the RMT device uses the GTPE
 * timeline to fill channel RAM with rmt_item32_t entries.
 */
void esp32_rmt_register_gte(Esp32RmtState *s, void *devices, int count);
