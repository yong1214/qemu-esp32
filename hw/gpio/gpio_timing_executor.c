/*
 * GPIO Timing-Protocol Executor (GTPE) — QEMU ESP32
 *
 * Data-driven virtual GPIO device engine. Reads JSON timing scripts
 * at startup and uses QEMU_CLOCK_VIRTUAL timers for cycle-accurate
 * pin responses.
 *
 * Copyright (c) 2025-2026 Dustalon Project
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/gpio/gpio_timing_executor.h"
#include "hw/gpio/esp32_gpio.h"

#include "qapi/qmp/qjson.h"
#include "qapi/qmp/qobject.h"
#include "qapi/qmp/qdict.h"
#include "qapi/qmp/qlist.h"
#include "qapi/qmp/qnum.h"
#include "qapi/qmp/qstring.h"
#include "qapi/qmp/qbool.h"
#include "qapi/error.h"

#include <string.h>
#include <math.h>

/* ── Forward declarations ────────────────────────────────────────────────── */

static void gte_step_callback(void *opaque);
static void gte_trigger_cb(void *opaque, int pin, bool high);

/* ── Data Encoders ───────────────────────────────────────────────────────── */

static void gte_encode_dht11(GteDevice *dev)
{
    float temp = dev->params[0]; /* temperature */
    float hum  = dev->params[1]; /* humidity */

    /* Apply noise if params exist */
    if (dev->num_params > 2 && dev->params[2] > 0) {
        temp += (float)g_random_double_range(-dev->params[2], dev->params[2]);
    }
    if (dev->num_params > 3 && dev->params[3] > 0) {
        hum += (float)g_random_double_range(-dev->params[3], dev->params[3]);
    }

    /* Clamp to DHT11 range */
    if (temp < 0) temp = 0; if (temp > 50) temp = 50;
    if (hum < 20) hum = 20; if (hum > 90) hum = 90;

    uint8_t hum_int  = (uint8_t)hum;
    uint8_t hum_dec  = 0;
    uint8_t temp_int = (uint8_t)temp;
    uint8_t temp_dec = 0;

    dev->data_bytes[0] = hum_int;
    dev->data_bytes[1] = hum_dec;
    dev->data_bytes[2] = temp_int;
    dev->data_bytes[3] = temp_dec;
    dev->data_bytes[4] = (hum_int + hum_dec + temp_int + temp_dec) & 0xFF;
    dev->data_bit_count = 40;
}

void gte_encode_data(GteDevice *dev)
{
    switch (dev->encoder_type) {
    case GTE_ENCODER_DHT11_40BIT:
        gte_encode_dht11(dev);
        break;
    case GTE_ENCODER_RAW_PULSE:
    case GTE_ENCODER_NONE:
    default:
        break;
    }
}

/* ── Delay Computation ───────────────────────────────────────────────────── */

static uint64_t gte_compute_delay_ns(GteDevice *dev, const GteStep *step)
{
    switch (step->type) {
    case GTE_STEP_SET_PIN:
    case GTE_STEP_COMMENT:
        return step->delay_ns;

    case GTE_STEP_SET_PIN_EXPR: {
        float val = 0;
        int idx = step->delay_expr.param_index;
        if (idx >= 0 && idx < dev->num_params) {
            val = dev->params[idx];
        }
        /* Apply noise for HC-SR04 distance */
        if (dev->num_params > idx + 1 && dev->params[idx + 1] > 0) {
            val += (float)g_random_double_range(
                -dev->params[idx + 1], dev->params[idx + 1]);
            if (val < 2.0f) val = 2.0f; /* HC-SR04 min range */
        }
        float delay_us = val * step->delay_expr.multiplier;
        if (delay_us < 0) delay_us = 0;
        return (uint64_t)(delay_us * 1000.0f); /* µs → ns */
    }

    case GTE_STEP_SET_PIN_BIT: {
        /* Look up current data bit */
        if (dev->current_bit < dev->data_bit_count) {
            int byte_idx = dev->current_bit / 8;
            int bit_idx  = 7 - (dev->current_bit % 8); /* MSB first */
            bool bit_val = (dev->data_bytes[byte_idx] >> bit_idx) & 1;
            return bit_val ? step->delay_bit[1] : step->delay_bit[0];
        }
        return step->delay_bit[0]; /* fallback */
    }

    default:
        return 0;
    }
}

/* ── Timeline Precomputation (read-interception approach) ────────────────── */

/**
 * Build a precomputed timeline of all pin state changes with absolute
 * virtual-time stamps. Called once per trigger. During GPIO_IN reads,
 * we binary-search this timeline to return the correct pin state.
 */
static void gte_build_timeline(GteDevice *dev, uint64_t start_ns)
{
    dev->timeline_count = 0;
    dev->response_start_ns = start_ns;

    uint64_t current_ns = start_ns;
    int step_idx = 0;
    int repeat_counter = 0;
    int repeat_start = 0;
    int bit = 0;

    while (step_idx < dev->num_steps &&
           dev->timeline_count < GTE_MAX_TIMELINE) {
        GteStep *step = &dev->steps[step_idx];

        switch (step->type) {
        case GTE_STEP_COMMENT:
            step_idx++;
            break;

        case GTE_STEP_SET_PIN:
            current_ns += step->delay_ns;
            dev->timeline[dev->timeline_count].time_ns = current_ns;
            dev->timeline[dev->timeline_count].pin_index = step->pin_index;
            dev->timeline[dev->timeline_count].pin_value = step->pin_value;
            dev->timeline_count++;
            step_idx++;
            break;

        case GTE_STEP_SET_PIN_EXPR: {
            float val = 0;
            int idx = step->delay_expr.param_index;
            if (idx >= 0 && idx < dev->num_params) {
                val = dev->params[idx];
            }
            if (dev->num_params > idx + 1 && dev->params[idx + 1] > 0) {
                val += (float)g_random_double_range(
                    -dev->params[idx + 1], dev->params[idx + 1]);
                if (val < 2.0f) val = 2.0f;
            }
            float delay_us = val * step->delay_expr.multiplier;
            if (delay_us < 0) delay_us = 0;
            current_ns += (uint64_t)(delay_us * 1000.0f);
            dev->timeline[dev->timeline_count].time_ns = current_ns;
            dev->timeline[dev->timeline_count].pin_index = step->pin_index;
            dev->timeline[dev->timeline_count].pin_value = step->pin_value;
            dev->timeline_count++;
            step_idx++;
            break;
        }

        case GTE_STEP_SET_PIN_BIT: {
            if (bit < dev->data_bit_count) {
                int byte_idx = bit / 8;
                int bit_idx = 7 - (bit % 8);
                bool bit_val = (dev->data_bytes[byte_idx] >> bit_idx) & 1;
                current_ns += bit_val ? step->delay_bit[1] : step->delay_bit[0];
            } else {
                current_ns += step->delay_bit[0];
            }
            dev->timeline[dev->timeline_count].time_ns = current_ns;
            dev->timeline[dev->timeline_count].pin_index = step->pin_index;
            dev->timeline[dev->timeline_count].pin_value = step->pin_value;
            dev->timeline_count++;
            bit++;
            step_idx++;
            break;
        }

        case GTE_STEP_REPEAT_START:
            repeat_counter = step->repeat_count;
            repeat_start = step_idx + 1;
            step_idx++;
            break;

        case GTE_STEP_REPEAT_END:
            repeat_counter--;
            if (repeat_counter > 0) {
                step_idx = repeat_start;
            } else {
                step_idx++;
            }
            break;

        default:
            step_idx++;
            break;
        }
    }

    qemu_log("GTPE '%s': built timeline (%d entries, %"PRIu64"us total)\n",
             dev->name, dev->timeline_count,
             dev->timeline_count > 0
                 ? (dev->timeline[dev->timeline_count - 1].time_ns - start_ns) / 1000
                 : 0);
}

/**
 * Apply GTPE read interception to GPIO_IN register value.
 * For each active device, find the correct pin state at the current
 * virtual time by searching the precomputed timeline.
 */
uint32_t gte_apply_read_intercept(uint32_t in_val, int bank,
                                   GteDevice *devices, int count)
{
    if (!devices || count <= 0) return in_val;

    /*
     * NS_PER_READ: how many nanoseconds of "protocol time" each GPIO_IN
     * read represents. On real ESP32 at 240MHz, a tight digitalRead +
     * micros() + branch loop is ~30-50 cycles ≈ 125-210ns.
     *
     * In QEMU without icount, each GPIO_IN MMIO read takes ~30-90µs of
     * virtual time, but we IGNORE virtual time. Instead, we count reads
     * and multiply by NS_PER_READ to get protocol-relative time.
     *
     * This is the same approach as Renode's DHT11 (US_PER_IDR_READ=0.1µs).
     */
    #define NS_PER_READ 150  /* 0.15µs per read ≈ 36 cycles at 240MHz */

    for (int d = 0; d < count; d++) {
        GteDevice *dev = &devices[d];
        if (!dev->active || dev->timeline_count == 0) continue;

        /* On first GPIO_IN read after trigger: anchor */
        if (dev->pending_response) {
            dev->pending_response = false;
            dev->read_count = 0;
            dev->response_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            qemu_log("GTPE '%s': anchored (read-count mode, %d entries)\n",
                     dev->name, dev->timeline_count);
        }

        /* Protocol-relative elapsed time based on read count */
        uint64_t elapsed_ns = (uint64_t)dev->read_count * NS_PER_READ;
        dev->read_count++;

        /* Check if response has ended */
        uint64_t end_ns = dev->timeline[dev->timeline_count - 1].time_ns;
        if (elapsed_ns > end_ns + 100000) {
            dev->active = false;
            for (int i = 0; i < dev->num_drive_pins; i++) {
                int pin = dev->drive_pins[i];
                int pin_bank = (pin >= 32) ? 1 : 0;
                if (pin_bank == bank) {
                    int bit = pin & 31;
                    if (dev->idle_values[i])
                        in_val |= (1u << bit);
                    else
                        in_val &= ~(1u << bit);
                }
            }
            continue;
        }

        /* For each driven pin, find latest timeline entry <= elapsed_ns */
        for (int i = 0; i < dev->num_drive_pins; i++) {
            int pin = dev->drive_pins[i];
            int pin_bank = (pin >= 32) ? 1 : 0;
            if (pin_bank != bank) continue;

            int found_value = dev->idle_values[i];
            for (int t = dev->timeline_count - 1; t >= 0; t--) {
                if (dev->timeline[t].time_ns <= elapsed_ns &&
                    dev->timeline[t].pin_index == i) {
                    found_value = dev->timeline[t].pin_value;
                    break;
                }
            }

            int bit = pin & 31;
            if (found_value)
                in_val |= (1u << bit);
            else
                in_val &= ~(1u << bit);
        }
    }

    return in_val;
}

/* ── Step Execution (timer-based, kept for compatibility) ────────────────── */

static void gte_execute_step(GteDevice *dev)
{
    if (!dev->active || !dev->gpio) return;
    if (dev->current_step >= dev->num_steps) {
        /* Sequence complete — set idle state */
        for (int i = 0; i < dev->num_drive_pins; i++) {
            esp32_gpio_set_input_level(dev->gpio, dev->drive_pins[i],
                                       dev->idle_values[i] != 0);
        }
        dev->active = false;
        return;
    }

    GteStep *step = &dev->steps[dev->current_step];

    switch (step->type) {
    case GTE_STEP_COMMENT:
        /* Skip to next step immediately */
        dev->current_step++;
        gte_execute_step(dev);
        return;

    case GTE_STEP_SET_PIN:
    case GTE_STEP_SET_PIN_EXPR:
    case GTE_STEP_SET_PIN_BIT: {
        uint64_t delay_ns = gte_compute_delay_ns(dev, step);

        /* For bit steps, advance the bit counter */
        if (step->type == GTE_STEP_SET_PIN_BIT) {
            dev->current_bit++;
        }

        dev->current_step++;

        /* Schedule pin change after delay */
        if (delay_ns > 0) {
            uint64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            if (dev->current_step <= 8) {
                qemu_log("GTPE '%s': scheduling step[%d] delay_ns=%"PRIu64
                         " (=%"PRIu64"us) at vtime=%"PRIu64"us\n",
                         dev->name, dev->current_step - 1,
                         delay_ns, delay_ns / 1000, now_ns / 1000);
            }
            timer_mod_ns(dev->step_timer, now_ns + delay_ns);
        } else {
            /* Zero delay — execute immediately */
            gte_step_callback(dev);
        }
        return;
    }

    case GTE_STEP_REPEAT_START:
        dev->repeat_counter = step->repeat_count;
        dev->repeat_start_step = dev->current_step + 1;
        dev->current_step++;
        gte_execute_step(dev);
        return;

    case GTE_STEP_REPEAT_END:
        dev->repeat_counter--;
        if (dev->repeat_counter > 0) {
            dev->current_step = dev->repeat_start_step;
        } else {
            dev->current_step++;
        }
        gte_execute_step(dev);
        return;

    default:
        dev->current_step++;
        gte_execute_step(dev);
        return;
    }
}

static void gte_step_callback(void *opaque)
{
    GteDevice *dev = (GteDevice *)opaque;
    if (!dev->active || !dev->gpio) return;

    /* The previous step scheduled us — execute its pin change.
     * The step index was already advanced, so look at step - 1. */
    int prev = dev->current_step - 1;
    if (prev >= 0 && prev < dev->num_steps) {
        GteStep *step = &dev->steps[prev];
        if (step->pin_index >= 0 && step->pin_index < dev->num_drive_pins) {
            int gpio_pin = dev->drive_pins[step->pin_index];
            Esp32GpioState *gs = (Esp32GpioState *)dev->gpio;
            int bank = (gpio_pin >= 32) ? 1 : 0;
            int bit = gpio_pin & 31;
            bool before = (gs->in_val[bank] >> bit) & 1;
            esp32_gpio_set_input_level(dev->gpio, gpio_pin,
                                       step->pin_value != 0);
            bool after = (gs->in_val[bank] >> bit) & 1;
            /* Debug: log pin changes with virtual time + readback */
            if (dev->current_step <= 8 || dev->current_step == dev->num_steps) {
                uint64_t vt = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                qemu_log("GTPE '%s': step[%d] → GPIO%d = %d "
                         "(before=%d after=%d in_val=0x%08x vtime=%"PRIu64"us)\n",
                         dev->name, prev, gpio_pin, step->pin_value,
                         before, after, gs->in_val[bank], vt / 1000);
            }
        } else {
            qemu_log("GTPE '%s': step[%d] pin_index=%d OUT OF RANGE (num_drive=%d)\n",
                     dev->name, prev, step->pin_index, dev->num_drive_pins);
        }
    }

    /* Continue to next step */
    gte_execute_step(dev);
}

/* ── Trigger Detection ───────────────────────────────────────────────────── */

static void gte_trigger_cb(void *opaque, int pin, bool high)
{
    GteDevice *dev = (GteDevice *)opaque;

    uint64_t vt = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (dev->bidirectional) {
        /* DHT11-style: firmware pulls LOW then releases HIGH */
        if (!high && !dev->pin_was_low && !dev->active) {
            dev->pin_was_low = true;
        } else if (high && dev->pin_was_low && !dev->active) {
            dev->pin_was_low = false;
            /* Build timeline with relative offsets (start_ns=0).
             * Will be anchored to real time on the first GPIO_IN read. */
            gte_encode_data(dev);
            dev->current_bit = 0;
            dev->active = true;
            dev->pending_response = true;
            gte_build_timeline(dev, 0); /* relative offsets */
            qemu_log("GTPE '%s': response prepared (%d entries, "
                     "%"PRIu64"us duration)\n",
                     dev->name, dev->timeline_count,
                     dev->timeline_count > 0
                         ? dev->timeline[dev->timeline_count - 1].time_ns / 1000
                         : 0);
        }
    } else {
        /* HC-SR04-style: rising edge on trigger pin */
        if (high && !dev->pin_was_high && !dev->active) {
            gte_encode_data(dev);
            dev->current_bit = 0;
            dev->active = true;
            dev->pending_response = true;
            gte_build_timeline(dev, 0); /* relative offsets */
            qemu_log("GTPE '%s': trigger prepared (%d entries)\n",
                     dev->name, dev->timeline_count);
        }
        dev->pin_was_high = high;
    }
}

/* ── JSON Parsing ────────────────────────────────────────────────────────── */

static GteEncoderType gte_parse_encoder(const char *name)
{
    if (!name) return GTE_ENCODER_NONE;
    if (g_strcmp0(name, "dht11_40bit") == 0) return GTE_ENCODER_DHT11_40BIT;
    if (g_strcmp0(name, "raw_pulse") == 0)   return GTE_ENCODER_RAW_PULSE;
    return GTE_ENCODER_NONE;
}

/**
 * Parse "pN * constant" expression string.
 * Returns true on success.
 */
static bool gte_parse_expr(const char *expr, GteDelayExpr *out)
{
    /* Format: "p<digit> * <number>" */
    if (!expr || expr[0] != 'p') return false;

    char *endptr;
    int idx = (int)strtol(expr + 1, &endptr, 10);
    if (idx < 0 || idx >= GTE_MAX_PARAMS) return false;

    /* Skip whitespace and '*' */
    while (*endptr == ' ' || *endptr == '*') endptr++;

    float mult = strtof(endptr, NULL);
    out->param_index = idx;
    out->multiplier = mult;
    return true;
}

/**
 * Resolve a pin name (e.g., "echo", "data") to a drive_pins index.
 */
static int gte_resolve_pin_name(GteDevice *dev, const char *name,
                                 const char **pin_names, int num_pins)
{
    for (int i = 0; i < num_pins; i++) {
        if (g_strcmp0(pin_names[i], name) == 0) return i;
    }
    return -1;
}

static int gte_parse_steps(QList *steps_list, GteDevice *dev,
                            const char **pin_names, int num_pin_names,
                            QDict *sequences, int start_idx)
{
    int idx = start_idx;
    QListEntry *entry;

    QLIST_FOREACH_ENTRY(steps_list, entry) {
        if (idx >= GTE_MAX_STEPS) break;

        QDict *step_dict = qobject_to(QDict, qlist_entry_obj(entry));
        if (!step_dict) continue;

        /* Check for comment-only entries (skip).
         * A step with BOTH "comment" and "repeat"/"set" is NOT comment-only. */
        if (qdict_haskey(step_dict, "comment")
            && !qdict_haskey(step_dict, "set")
            && !qdict_haskey(step_dict, "repeat")) {
            dev->steps[idx].type = GTE_STEP_COMMENT;
            dev->steps[idx].delay_ns = 0;
            idx++;
            continue;
        }

        /* Check for repeat block */
        if (qdict_haskey(step_dict, "repeat")) {
            int count = qdict_get_int(step_dict, "repeat");
            const char *seq_name = qdict_get_str(step_dict, "sequence");

            dev->steps[idx].type = GTE_STEP_REPEAT_START;
            dev->steps[idx].repeat_count = count;
            idx++;

            /* Inline the referenced sequence */
            if (seq_name && sequences && qdict_haskey(sequences, seq_name)) {
                /* The sequence is a top-level key in the device JSON */
            }
            /* For now, look for sequence in the parent dict passed via sequences */
            if (seq_name && sequences) {
                QList *seq_list = qobject_to(QList, qdict_get(sequences, seq_name));
                if (seq_list) {
                    idx = gte_parse_steps(seq_list, dev, pin_names, num_pin_names,
                                          NULL, idx);
                }
            }

            dev->steps[idx].type = GTE_STEP_REPEAT_END;
            dev->steps[idx].repeat_jump_target = 0; /* set after full parse */
            idx++;
            continue;
        }

        /* Normal step: delay + set pin */
        const char *pin_name = qdict_get_try_str(step_dict, "set");
        int pin_idx = pin_name ? gte_resolve_pin_name(dev, pin_name,
                                                       pin_names, num_pin_names) : 0;

        int pin_val = 0;
        if (qdict_haskey(step_dict, "value")) {
            pin_val = (int)qdict_get_int(step_dict, "value");
        }

        GteStep *s = &dev->steps[idx];
        s->pin_index = pin_idx;
        s->pin_value = pin_val;

        if (qdict_haskey(step_dict, "delay_us_expr")) {
            const char *expr_str = qdict_get_str(step_dict, "delay_us_expr");
            s->type = GTE_STEP_SET_PIN_EXPR;
            if (!gte_parse_expr(expr_str, &s->delay_expr)) {
                qemu_log("GTPE: failed to parse expr: %s\n", expr_str);
                s->type = GTE_STEP_SET_PIN;
                s->delay_ns = 0;
            }
        } else if (qdict_haskey(step_dict, "delay_us_bit")) {
            QList *bit_list = qobject_to(QList, qdict_get(step_dict, "delay_us_bit"));
            s->type = GTE_STEP_SET_PIN_BIT;
            if (bit_list) {
                QNum *v0 = qobject_to(QNum, qlist_peek(bit_list));
                QNum *v1 = NULL;
                QListEntry *e2;
                int bi = 0;
                QLIST_FOREACH_ENTRY(bit_list, e2) {
                    QNum *n = qobject_to(QNum, qlist_entry_obj(e2));
                    if (n && bi < 2) {
                        int64_t us = qnum_get_int(n);
                        s->delay_bit[bi] = (uint64_t)us * 1000; /* µs → ns */
                        bi++;
                    }
                }
                (void)v0; (void)v1;
            }
        } else if (qdict_haskey(step_dict, "delay_us")) {
            QObject *delay_obj = qdict_get(step_dict, "delay_us");
            QNum *delay_num = qobject_to(QNum, delay_obj);
            if (delay_num) {
                int64_t us = qnum_get_int(delay_num);
                s->delay_ns = (uint64_t)us * 1000;
            } else {
                s->delay_ns = 0;
            }
            s->type = GTE_STEP_SET_PIN;
        } else {
            s->type = GTE_STEP_SET_PIN;
            s->delay_ns = 0;
        }

        idx++;
    }

    return idx;
}

static bool gte_parse_one_device(QDict *dev_dict, GteDevice *dev,
                                  QDict *resolved_pins)
{
    memset(dev, 0, sizeof(*dev));

    /* Name */
    const char *name = qdict_get_try_str(dev_dict, "name");
    if (name) {
        g_strlcpy(dev->name, name, GTE_NAME_LEN);
    }

    /* Encoder */
    const char *enc = qdict_get_try_str(dev_dict, "dataEncoder");
    dev->encoder_type = gte_parse_encoder(enc);

    /* Pins */
    QDict *pins_dict = qobject_to(QDict, qdict_get(dev_dict, "pins"));
    if (!pins_dict) return false;

    /* Collect pin names — we need TWO arrays:
     *   pin_names[]       — ALL pin names (watch + drive), for reference
     *   drive_pin_names[] — only DRIVE pin names, indexed to match drive_pins[]
     * Steps reference pins by name via "set": "echo", and the resolved index
     * must map to drive_pins[], not pin_names[]. */
    const char *pin_names[GTE_MAX_DRIVE_PINS * 2];
    int num_pin_names = 0;
    const char *drive_pin_names[GTE_MAX_DRIVE_PINS];
    int num_drive_pin_names = 0;

    const QDictEntry *pe;
    for (pe = qdict_first(pins_dict); pe; pe = qdict_next(pins_dict, pe)) {
        if (num_pin_names >= GTE_MAX_DRIVE_PINS * 2) break;
        const char *pname = qdict_entry_key(pe);
        QDict *pconf = qobject_to(QDict, qdict_entry_value(pe));
        if (!pconf) continue;

        pin_names[num_pin_names] = pname;

        const char *dir = qdict_get_try_str(pconf, "direction");

        /* Resolve actual GPIO pin number from resolvedPins */
        int gpio_num = -1;
        if (resolved_pins && qdict_haskey(resolved_pins, pname)) {
            gpio_num = (int)qdict_get_int(resolved_pins, pname);
        }

        if (dir && g_strcmp0(dir, "watch") == 0) {
            dev->watch_pin = gpio_num;
            const char *edge = qdict_get_try_str(pconf, "edge");
            dev->watch_rising = !edge || g_strcmp0(edge, "rising") == 0;
        } else if (dir && g_strcmp0(dir, "drive") == 0) {
            int di = dev->num_drive_pins;
            dev->drive_pins[di] = gpio_num;
            drive_pin_names[num_drive_pin_names++] = pname;
            /* Parse idle value */
            if (qdict_haskey(pconf, "idle")) {
                dev->idle_values[di] = (int)qdict_get_int(pconf, "idle");
            }
            dev->num_drive_pins++;
        } else if (dir && g_strcmp0(dir, "bidirectional") == 0) {
            /* Same pin for watch and drive */
            dev->watch_pin = gpio_num;
            dev->bidirectional = true;
            const char *wedge = qdict_get_try_str(pconf, "watch_edge");
            dev->watch_rising = wedge && g_strcmp0(wedge, "rising") == 0;

            int di = dev->num_drive_pins;
            dev->drive_pins[di] = gpio_num;
            drive_pin_names[num_drive_pin_names++] = pname;
            if (qdict_haskey(pconf, "idle")) {
                dev->idle_values[di] = (int)qdict_get_int(pconf, "idle");
            }
            dev->num_drive_pins++;
        }

        num_pin_names++;
    }

    /* Params */
    QList *params_list = qobject_to(QList, qdict_get(dev_dict, "params"));
    if (params_list) {
        QListEntry *entry;
        QLIST_FOREACH_ENTRY(params_list, entry) {
            QDict *pd = qobject_to(QDict, qlist_entry_obj(entry));
            if (!pd) continue;
            int idx = (int)qdict_get_int(pd, "index");
            if (idx >= 0 && idx < GTE_MAX_PARAMS) {
                /* Try to get 'default' as number */
                QNum *def_num = qobject_to(QNum, qdict_get(pd, "default"));
                if (def_num) {
                    double val = qnum_get_double(def_num);
                    dev->params[idx] = (float)val;
                }
                if (idx >= dev->num_params) dev->num_params = idx + 1;
            }
        }
    }

    /* Parse on_trigger steps — use drive_pin_names so that "set": "echo"
     * resolves to drive_pins[0], not pin_names[1] */
    QList *on_trigger = qobject_to(QList, qdict_get(dev_dict, "on_trigger"));
    if (on_trigger) {
        dev->num_steps = gte_parse_steps(on_trigger, dev, drive_pin_names,
                                          num_drive_pin_names, dev_dict, 0);
    }

    /* Parse end_sequence if present (appended after main steps) */
    QList *end_seq = qobject_to(QList, qdict_get(dev_dict, "end_sequence"));
    if (end_seq) {
        dev->num_steps = gte_parse_steps(end_seq, dev, drive_pin_names,
                                          num_drive_pin_names, NULL, dev->num_steps);
    }

    /* Create virtual-time timer */
    dev->step_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gte_step_callback, dev);

    return true;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int gte_parse_scripts(const char *file_path, GteDevice devices[])
{
    Error *err = NULL;

    /* Read JSON file */
    gchar *content = NULL;
    gsize length = 0;
    if (!g_file_get_contents(file_path, &content, &length, NULL)) {
        qemu_log("GTPE: failed to read file: %s\n", file_path);
        return -1;
    }

    QObject *root = qobject_from_json(content, &err);
    g_free(content);

    if (!root || err) {
        qemu_log("GTPE: failed to parse JSON: %s\n",
                 err ? error_get_pretty(err) : "unknown");
        if (err) error_free(err);
        return -1;
    }

    QList *scripts = qobject_to(QList, root);
    if (!scripts) {
        qemu_log("GTPE: JSON root is not an array\n");
        qobject_unref(root);
        return -1;
    }

    int count = 0;
    QListEntry *entry;
    QLIST_FOREACH_ENTRY(scripts, entry) {
        if (count >= GTE_MAX_DEVICES) break;

        QDict *dev_dict = qobject_to(QDict, qlist_entry_obj(entry));
        if (!dev_dict) continue;

        /* Get resolvedPins sub-object */
        QDict *resolved_pins = qobject_to(QDict, qdict_get(dev_dict, "resolvedPins"));

        if (gte_parse_one_device(dev_dict, &devices[count], resolved_pins)) {
            qemu_log("GTPE: parsed device '%s' (%d steps, %d params)\n",
                     devices[count].name, devices[count].num_steps,
                     devices[count].num_params);
            count++;
        }
    }

    qobject_unref(root);
    return count;
}

void gte_attach_gpio(GteDevice *dev, Esp32GpioState *gpio)
{
    if (!dev || !gpio) return;
    dev->gpio = gpio;

    if (dev->watch_pin >= 0) {
        esp32_gpio_register_output_listener(gpio, dev->watch_pin,
                                             gte_trigger_cb, dev);
    }

    /* Set initial idle state on drive pins */
    for (int i = 0; i < dev->num_drive_pins; i++) {
        if (dev->drive_pins[i] >= 0) {
            esp32_gpio_set_input_level(gpio, dev->drive_pins[i],
                                       dev->idle_values[i] != 0);
        }
    }

    qemu_log("GTPE: attached '%s' (watch=GPIO%d, drive=%d pins)\n",
             dev->name, dev->watch_pin, dev->num_drive_pins);
}

void gte_update_param(GteDevice *dev, int param_index, float value)
{
    if (!dev || param_index < 0 || param_index >= GTE_MAX_PARAMS) return;
    dev->params[param_index] = value;
    if (param_index >= dev->num_params) {
        dev->num_params = param_index + 1;
    }
}
