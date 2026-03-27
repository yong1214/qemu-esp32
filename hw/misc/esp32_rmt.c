/*
 * ESP32 RMT (Remote Control Transceiver) — QEMU peripheral
 *
 * Phase 1: RX capture only. When firmware enables RX on a channel,
 * the GTPE provides the pulse timeline and this device converts it
 * to rmt_item32_t entries in channel RAM.
 *
 * Copyright (c) 2025-2026 Dustalon Project
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "hw/misc/esp32_rmt.h"
#include "hw/gpio/gpio_timing_executor.h"

/* ── Helpers ────────────────────────────────────────────────────────────── */

static void esp32_rmt_update_irq(Esp32RmtState *s)
{
    uint32_t pending = s->int_raw & s->int_ena;
    qemu_set_irq(s->irq, pending ? 1 : 0);
}

/**
 * Get the clock divider for a channel (from CONF0).
 * div_cnt=0 means divide by 256.
 */
static int esp32_rmt_get_div(Esp32RmtChannel *ch)
{
    int div = (ch->conf0 >> RMT_CONF0_DIV_CNT_SHIFT) & 0xFF;
    return div == 0 ? 256 : div;
}

/* ── RX Capture via GTPE ────────────────────────────────────────────────── */

/**
 * Find GTPE device that watches the given GPIO pin.
 * Returns NULL if no matching device is found.
 */
static GteDevice *find_gte_for_pin(Esp32RmtState *s, int gpio_pin)
{
    if (!s->gte_devices || s->gte_device_count <= 0) return NULL;

    GteDevice *devs = (GteDevice *)s->gte_devices;

    /* If gpio_pin is known, match exactly */
    if (gpio_pin >= 0) {
        for (int i = 0; i < s->gte_device_count; i++) {
            if (devs[i].watch_pin == gpio_pin) return &devs[i];
            if (devs[i].bidirectional) {
                for (int p = 0; p < devs[i].num_drive_pins; p++) {
                    if (devs[i].drive_pins[p] == gpio_pin) return &devs[i];
                }
            }
        }
        return NULL;
    }

    /* gpio_pin unknown (-1): GPIO matrix writes went to a separate device.
     * Return the first GTPE device that has a bidirectional data pin
     * (most likely the DHT11-style device being captured). */
    for (int i = 0; i < s->gte_device_count; i++) {
        if (devs[i].bidirectional) return &devs[i];
    }
    /* Fallback: return first device */
    if (s->gte_device_count > 0) return &devs[0];
    return NULL;
}

/**
 * Convert a GTPE timeline to rmt_item32_t entries in channel RAM.
 *
 * The timeline is a sequence of pin state changes with timestamps.
 * We pair consecutive edges into rmt_item32_t entries:
 *   item = { duration0, level0, duration1, level1 }
 *
 * Duration is in RMT ticks = µs × (APB_CLK / div_cnt) / 1_000_000
 * Since timeline is in nanoseconds: ticks = ns × (APB_CLK / div_cnt) / 1_000_000_000
 * Simplified: ticks = ns × APB_CLK / (div_cnt × 1_000_000_000)
 *           = ns / (div_cnt × 12.5)   [for 80MHz APB]
 *           = ns × 2 / (div_cnt × 25)
 */
static void esp32_rmt_rx_capture(Esp32RmtState *s, int ch_idx)
{
    Esp32RmtChannel *ch = &s->channel[ch_idx];
    GteDevice *gte = find_gte_for_pin(s, ch->gpio_pin);

    if (!gte) {
        qemu_log("RMT: ch%d rx_en but no GTPE device for GPIO%d\n",
                 ch_idx, ch->gpio_pin);
        return;
    }

    /* Trigger the GTPE to encode data and build timeline */
    gte_encode_data(gte);
    gte->current_bit = 0;

    /* Build the timeline. We pass 0 as start_ns since the timeline
     * entries are relative offsets — we only care about durations. */
    uint64_t dummy_start = 0;
    /* Save and restore GTPE state — we just want the timeline data */
    bool was_active = gte->active;
    gte_build_timeline_for_rmt(gte, dummy_start);

    if (gte->timeline_count < 2) {
        qemu_log("RMT: ch%d GTPE '%s' produced only %d timeline entries\n",
                 ch_idx, gte->name, gte->timeline_count);
        gte->active = was_active;
        return;
    }

    int div = esp32_rmt_get_div(ch);
    /* ticks_per_ns = APB_CLK / (div × 1e9). To avoid floating point:
     * ticks = ns × APB_CLK / (div × 1e9)
     * For 80MHz: ticks = ns × 80 / (div × 1000) = ns / (div × 12.5)
     * Use integer math: ticks = (ns * 2) / (div * 25) */

    int item_idx = 0;
    int t = 0;  /* timeline index */

    /* Timeline entries: at time_ns, pin changes to pin_value.
     * RMT items describe consecutive pulses: {duration, level} pairs.
     *
     * Between entry[t] and entry[t+1], the pin is at entry[t].pin_value
     * for a duration of (entry[t+1].time_ns - entry[t].time_ns).
     *
     * We skip entry[0]'s initial delay (idle→first edge) and pair
     * consecutive entries: entry[t] gives level+start, entry[t+1] gives end. */
    t = 0; /* start from first entry */
    while (t < gte->timeline_count - 1 &&
           item_idx < ESP32_RMT_ITEMS_PER_CH) {
        /* Pulse 0: pin is at entry[t].pin_value from entry[t] to entry[t+1] */
        uint64_t dur0_ns = gte->timeline[t + 1].time_ns - gte->timeline[t].time_ns;
        int level0 = gte->timeline[t].pin_value;

        /* Pulse 1: pin is at entry[t+1].pin_value from entry[t+1] to entry[t+2] */
        uint64_t dur1_ns;
        int level1;
        if (t + 2 < gte->timeline_count) {
            dur1_ns = gte->timeline[t + 2].time_ns - gte->timeline[t + 1].time_ns;
            level1 = gte->timeline[t + 1].pin_value;
        } else {
            /* Last entry — use 0 duration to signal end */
            dur1_ns = 0;
            level1 = gte->timeline[t + 1].pin_value;
        }

        /* Convert ns to ticks: ticks = (ns * 2) / (div * 25) */
        uint32_t ticks0 = (uint32_t)((dur0_ns * 2) / ((uint64_t)div * 25));
        uint32_t ticks1 = (uint32_t)((dur1_ns * 2) / ((uint64_t)div * 25));

        /* Clamp to 15 bits */
        if (ticks0 > 0x7FFF) ticks0 = 0x7FFF;
        if (ticks1 > 0x7FFF) ticks1 = 0x7FFF;

        /* Pack as rmt_item32_t:
         * [duration0:15][level0:1][duration1:15][level1:1] */
        ch->mem[item_idx] = (ticks0 & 0x7FFF)
                          | ((level0 & 1) << 15)
                          | ((ticks1 & 0x7FFF) << 16)
                          | ((level1 & 1) << 31);

        item_idx++;
        t += 2;
    }

    /* Handle odd last entry */
    if (t < gte->timeline_count && item_idx < ESP32_RMT_ITEMS_PER_CH) {
        uint64_t dur_ns = gte->timeline[t].time_ns -
                          (t > 0 ? gte->timeline[t - 1].time_ns : dummy_start);
        uint32_t ticks = (uint32_t)((dur_ns * 2) / ((uint64_t)div * 25));
        if (ticks > 0x7FFF) ticks = 0x7FFF;
        int level = gte->timeline[t].pin_value;

        ch->mem[item_idx] = (ticks & 0x7FFF)
                          | ((level & 1) << 15);
        item_idx++;
    }

    /* End marker: item with all zeros */
    if (item_idx < ESP32_RMT_ITEMS_PER_CH) {
        ch->mem[item_idx] = 0;
    }

    /* Restore GTPE state */
    gte->active = was_active;

    /* Set RX end interrupt */
    s->int_raw |= RMT_INT_CH_RX_END(ch_idx);
    esp32_rmt_update_irq(s);

    qemu_log("RMT: ch%d captured %d items from GTPE '%s' (GPIO%d, div=%d)\n",
             ch_idx, item_idx, gte->name, ch->gpio_pin, div);
}

/* ── Register Read ──────────────────────────────────────────────────────── */

static uint64_t esp32_rmt_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32RmtState *s = ESP32_RMT(opaque);
    uint64_t r = 0;

    /* Channel RAM region: 0x800 - 0xFFF */
    if (addr >= A_RMT_CH_MEM_BASE &&
        addr < A_RMT_CH_MEM_BASE + ESP32_RMT_CHANNEL_CNT * ESP32_RMT_CH_MEM_SIZE) {
        int offset = addr - A_RMT_CH_MEM_BASE;
        int ch_idx = offset / ESP32_RMT_CH_MEM_SIZE;
        int item_idx = (offset % ESP32_RMT_CH_MEM_SIZE) / 4;
        if (ch_idx < ESP32_RMT_CHANNEL_CNT &&
            item_idx < ESP32_RMT_ITEMS_PER_CH) {
            return s->channel[ch_idx].mem[item_idx];
        }
        return 0;
    }

    /* CHnDATA registers: FIFO access */
    if (addr >= A_RMT_CH0DATA && addr < A_RMT_CH0DATA + ESP32_RMT_CHANNEL_CNT * 4) {
        int ch_idx = (addr - A_RMT_CH0DATA) / 4;
        Esp32RmtChannel *ch = &s->channel[ch_idx];
        if (ch->fifo_rd_ptr < ESP32_RMT_ITEMS_PER_CH) {
            r = ch->mem[ch->fifo_rd_ptr++];
        }
        return r;
    }

    /* CHnCONF0 / CHnCONF1 */
    if (addr >= A_RMT_CH0CONF0 &&
        addr < A_RMT_CH0CONF0 + ESP32_RMT_CHANNEL_CNT * ESP32_RMT_CONF_STRIDE) {
        int offset = addr - A_RMT_CH0CONF0;
        int ch_idx = offset / ESP32_RMT_CONF_STRIDE;
        int reg = offset % ESP32_RMT_CONF_STRIDE;
        if (reg == 0) return s->channel[ch_idx].conf0;
        if (reg == 4) return s->channel[ch_idx].conf1;
        return 0;
    }

    /* CHnSTATUS */
    if (addr >= A_RMT_CH0STATUS &&
        addr < A_RMT_CH0STATUS + ESP32_RMT_CHANNEL_CNT * 4) {
        /* Return idle state — no active transmission/reception info for now */
        return 0;
    }

    /* CHn CARRIER_DUTY (Phase 2 stub) */
    if (addr >= A_RMT_CH0CARRIER_DUTY &&
        addr < A_RMT_CH0CARRIER_DUTY + ESP32_RMT_CHANNEL_CNT * 4) {
        int ch_idx = (addr - A_RMT_CH0CARRIER_DUTY) / 4;
        return s->channel[ch_idx].carrier_duty;
    }

    /* Interrupt registers */
    switch (addr) {
    case A_RMT_INT_RAW:
        return s->int_raw;
    case A_RMT_INT_ST:
        return s->int_raw & s->int_ena;
    case A_RMT_INT_ENA:
        return s->int_ena;
    case A_RMT_APB_CONF:
        return s->apb_conf;
    default:
        break;
    }

    /* TX limit registers */
    if (addr >= A_RMT_CH0TX_LIM &&
        addr < A_RMT_CH0TX_LIM + ESP32_RMT_CHANNEL_CNT * 4) {
        int ch_idx = (addr - A_RMT_CH0TX_LIM) / 4;
        return s->channel[ch_idx].tx_lim;
    }

    return r;
}

/* ── Register Write ─────────────────────────────────────────────────────── */

static void esp32_rmt_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size)
{
    Esp32RmtState *s = ESP32_RMT(opaque);

    /* Channel RAM region: 0x800 - 0xFFF (firmware writing TX data) */
    if (addr >= A_RMT_CH_MEM_BASE &&
        addr < A_RMT_CH_MEM_BASE + ESP32_RMT_CHANNEL_CNT * ESP32_RMT_CH_MEM_SIZE) {
        int offset = addr - A_RMT_CH_MEM_BASE;
        int ch_idx = offset / ESP32_RMT_CH_MEM_SIZE;
        int item_idx = (offset % ESP32_RMT_CH_MEM_SIZE) / 4;
        if (ch_idx < ESP32_RMT_CHANNEL_CNT &&
            item_idx < ESP32_RMT_ITEMS_PER_CH) {
            s->channel[ch_idx].mem[item_idx] = (uint32_t)value;
        }
        return;
    }

    /* CHnDATA: FIFO write (for TX) — store in channel RAM */
    if (addr >= A_RMT_CH0DATA && addr < A_RMT_CH0DATA + ESP32_RMT_CHANNEL_CNT * 4) {
        /* Phase 2: implement FIFO write for TX */
        return;
    }

    /* CHnCONF0 / CHnCONF1 */
    if (addr >= A_RMT_CH0CONF0 &&
        addr < A_RMT_CH0CONF0 + ESP32_RMT_CHANNEL_CNT * ESP32_RMT_CONF_STRIDE) {
        int offset = addr - A_RMT_CH0CONF0;
        int ch_idx = offset / ESP32_RMT_CONF_STRIDE;
        int reg = offset % ESP32_RMT_CONF_STRIDE;

        if (reg == 0) {
            /* CONF0 */
            s->channel[ch_idx].conf0 = (uint32_t)value;
        } else if (reg == 4) {
            /* CONF1 */
            uint32_t old = s->channel[ch_idx].conf1;
            s->channel[ch_idx].conf1 = (uint32_t)value;

            /* Detect rx_en rising edge → trigger RX capture.
             * The GPIO pin may not be set (GPIO matrix writes go to a
             * separate device). If gpio_pin is unknown, try to capture
             * using any available GTPE device for this channel. */
            if ((value & RMT_CONF1_RX_EN) && !(old & RMT_CONF1_RX_EN)) {
                qemu_log("RMT: ch%d rx_en set (gpio_pin=%d)\n",
                         ch_idx, s->channel[ch_idx].gpio_pin);
                esp32_rmt_rx_capture(s, ch_idx);
            }

            /* Handle memory resets */
            if (value & RMT_CONF1_MEM_WR_RST) {
                s->channel[ch_idx].fifo_rd_ptr = 0;
            }
            if (value & RMT_CONF1_MEM_RD_RST) {
                s->channel[ch_idx].fifo_rd_ptr = 0;
            }
            if (value & RMT_CONF1_APB_MEM_RST) {
                s->channel[ch_idx].fifo_rd_ptr = 0;
            }

            /* Phase 2: detect tx_start rising edge → trigger TX */
        }
        return;
    }

    /* Carrier duty (Phase 2 stub) */
    if (addr >= A_RMT_CH0CARRIER_DUTY &&
        addr < A_RMT_CH0CARRIER_DUTY + ESP32_RMT_CHANNEL_CNT * 4) {
        int ch_idx = (addr - A_RMT_CH0CARRIER_DUTY) / 4;
        s->channel[ch_idx].carrier_duty = (uint32_t)value;
        return;
    }

    /* Interrupt registers */
    switch (addr) {
    case A_RMT_INT_RAW:
        /* Some bits are W1S (write-1-to-set), but typically not used */
        return;
    case A_RMT_INT_ENA:
        s->int_ena = (uint32_t)value;
        esp32_rmt_update_irq(s);
        return;
    case A_RMT_INT_CLR:
        s->int_raw &= ~(uint32_t)value;
        esp32_rmt_update_irq(s);
        return;
    case A_RMT_APB_CONF:
        s->apb_conf = (uint32_t)value;
        return;
    default:
        break;
    }

    /* TX limit registers */
    if (addr >= A_RMT_CH0TX_LIM &&
        addr < A_RMT_CH0TX_LIM + ESP32_RMT_CHANNEL_CNT * 4) {
        int ch_idx = (addr - A_RMT_CH0TX_LIM) / 4;
        s->channel[ch_idx].tx_lim = (uint32_t)value;
        return;
    }
}

/* ── Memory Region Ops ──────────────────────────────────────────────────── */

static const MemoryRegionOps esp32_rmt_ops = {
    .read = esp32_rmt_read,
    .write = esp32_rmt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* ── Public API ─────────────────────────────────────────────────────────── */

void esp32_rmt_register_gte(Esp32RmtState *s, void *devices, int count)
{
    s->gte_devices = devices;
    s->gte_device_count = count;
    qemu_log("RMT: registered %d GTPE device(s)\n", count);
}

/**
 * Set the GPIO pin assignment for a channel.
 * Called from esp32.c based on GPIO matrix configuration,
 * or hardcoded for testing.
 */
static void esp32_rmt_set_channel_pin(Esp32RmtState *s, int ch_idx, int gpio_pin)
{
    if (ch_idx >= 0 && ch_idx < ESP32_RMT_CHANNEL_CNT) {
        s->channel[ch_idx].gpio_pin = gpio_pin;
        qemu_log("RMT: ch%d assigned to GPIO%d\n", ch_idx, gpio_pin);
    }
}

/* ── Device Lifecycle ───────────────────────────────────────────────────── */

static void esp32_rmt_realize(DeviceState *dev, Error **errp)
{
    Esp32RmtState *s = ESP32_RMT(dev);

    /* Initialize channels */
    for (int i = 0; i < ESP32_RMT_CHANNEL_CNT; i++) {
        s->channel[i].conf0 = 0;
        s->channel[i].conf1 = 0;
        s->channel[i].status = 0;
        s->channel[i].carrier_duty = 0;
        s->channel[i].tx_lim = 0;
        s->channel[i].fifo_rd_ptr = 0;
        s->channel[i].rx_active = false;
        s->channel[i].gpio_pin = -1;
        memset(s->channel[i].mem, 0, sizeof(s->channel[i].mem));
    }

    s->int_raw = 0;
    s->int_ena = 0;
    s->apb_conf = 0;
    /* Note: do NOT reset gte_devices/gte_device_count here.
     * They may be set before realize() by esp32_rmt_register_gte()
     * called from the GTPE initialization block in esp32.c. */
}

static void esp32_rmt_init(Object *obj)
{
    Esp32RmtState *s = ESP32_RMT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_rmt_ops, s,
                          TYPE_ESP32_RMT, ESP32_RMT_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32_rmt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = esp32_rmt_realize;
}

static const TypeInfo esp32_rmt_info = {
    .name = TYPE_ESP32_RMT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32RmtState),
    .instance_init = esp32_rmt_init,
    .class_init = esp32_rmt_class_init,
};

static void esp32_rmt_register_types(void)
{
    type_register_static(&esp32_rmt_info);
}

type_init(esp32_rmt_register_types)
