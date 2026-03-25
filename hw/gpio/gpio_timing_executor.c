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

/* ── Step Execution ──────────────────────────────────────────────────────── */

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
            timer_mod_ns(dev->step_timer,
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns);
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
            esp32_gpio_set_input_level(dev->gpio, gpio_pin,
                                       step->pin_value != 0);
        }
    }

    /* Continue to next step */
    gte_execute_step(dev);
}

/* ── Trigger Detection ───────────────────────────────────────────────────── */

static void gte_trigger_cb(void *opaque, int pin, bool high)
{
    GteDevice *dev = (GteDevice *)opaque;

    if (dev->bidirectional) {
        /* DHT11-style: firmware pulls LOW then releases HIGH */
        if (!high && !dev->pin_was_low && !dev->active) {
            dev->pin_was_low = true;
        } else if (high && dev->pin_was_low && !dev->active) {
            dev->pin_was_low = false;
            /* Start response */
            gte_encode_data(dev);
            dev->current_bit = 0;
            dev->current_step = 0;
            dev->active = true;
            gte_execute_step(dev);
        }
    } else {
        /* HC-SR04-style: rising edge on trigger pin */
        if (high && !dev->pin_was_high && !dev->active) {
            gte_encode_data(dev);
            dev->current_bit = 0;
            dev->current_step = 0;
            dev->active = true;
            gte_execute_step(dev);
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

        /* Check for comment (skip) */
        if (qdict_haskey(step_dict, "comment") && !qdict_haskey(step_dict, "set")) {
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

    /* Collect pin names and resolved GPIO numbers */
    const char *pin_names[GTE_MAX_DRIVE_PINS];
    int num_pin_names = 0;

    const QDictEntry *pe;
    for (pe = qdict_first(pins_dict); pe; pe = qdict_next(pins_dict, pe)) {
        if (num_pin_names >= GTE_MAX_DRIVE_PINS) break;
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

    /* Parse on_trigger steps */
    QList *on_trigger = qobject_to(QList, qdict_get(dev_dict, "on_trigger"));
    if (on_trigger) {
        /* Collect named sequences from the device dict
         * (e.g., "bit_sequence", "end_sequence") */
        dev->num_steps = gte_parse_steps(on_trigger, dev, pin_names,
                                          num_pin_names, dev_dict, 0);
    }

    /* Parse end_sequence if present (appended after main steps) */
    QList *end_seq = qobject_to(QList, qdict_get(dev_dict, "end_sequence"));
    if (end_seq) {
        dev->num_steps = gte_parse_steps(end_seq, dev, pin_names,
                                          num_pin_names, NULL, dev->num_steps);
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
