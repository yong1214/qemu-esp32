/*
 * GPIO Timing-Protocol Executor (GTPE) — QEMU ESP32
 *
 * Generic, data-driven engine for GPIO virtual devices.
 * Parses JSON timing scripts at startup and drives pin state
 * via QEMU_CLOCK_VIRTUAL timers, giving cycle-accurate responses
 * to firmware interactions.
 *
 * Replaces per-device C implementations (hc_sr04_vdev.c, dht11_vdev.c)
 * with a single executor that interprets timing scripts generated
 * by the DeviceOrchestrator in the Node.js backend.
 *
 * Copyright (c) 2025-2026 Dustalon Project
 */

#ifndef GPIO_TIMING_EXECUTOR_H
#define GPIO_TIMING_EXECUTOR_H

#include "qemu/timer.h"
#include <stdbool.h>
#include <stdint.h>

/* Forward declaration */
typedef struct Esp32GpioState Esp32GpioState;

/* ── Limits ──────────────────────────────────────────────────────────────── */

#define GTE_MAX_DEVICES       8
#define GTE_MAX_STEPS        128   /* max timing steps per device */
#define GTE_MAX_PARAMS         8
#define GTE_MAX_DRIVE_PINS     4
#define GTE_MAX_DATA_BYTES     8   /* 64 data bits max */
#define GTE_NAME_LEN          32

/* ── Step Types ──────────────────────────────────────────────────────────── */

typedef enum {
    GTE_STEP_SET_PIN,          /* delay_us (static), then set pin to value */
    GTE_STEP_SET_PIN_EXPR,     /* delay_us = params[idx] * constant */
    GTE_STEP_SET_PIN_BIT,      /* delay_us depends on current data bit value */
    GTE_STEP_REPEAT_START,     /* marks start of a repeat block */
    GTE_STEP_REPEAT_END,       /* marks end of repeat block, jumps back */
    GTE_STEP_COMMENT,          /* no-op, skipped at runtime */
} GteStepType;

/* ── Expression for delay_us_expr: "pN * constant" ──────────────────────── */

typedef struct {
    int param_index;           /* index into device params[] */
    float multiplier;          /* constant multiplier */
} GteDelayExpr;

/* ── A single timing step ────────────────────────────────────────────────── */

typedef struct {
    GteStepType type;

    /* Which pin to drive (index into drive_pins[]) */
    int pin_index;
    int pin_value;             /* 0 or 1 */

    /* Delay before this step executes (mutually exclusive fields) */
    uint64_t delay_ns;         /* GTE_STEP_SET_PIN: static delay */
    GteDelayExpr delay_expr;   /* GTE_STEP_SET_PIN_EXPR */
    uint64_t delay_bit[2];     /* GTE_STEP_SET_PIN_BIT: [bit0_ns, bit1_ns] */

    /* For repeat blocks */
    int repeat_count;          /* GTE_STEP_REPEAT_START: how many iterations */
    int repeat_jump_target;    /* GTE_STEP_REPEAT_END: step index to jump to */
} GteStep;

/* ── Data Encoder Types ──────────────────────────────────────────────────── */

typedef enum {
    GTE_ENCODER_NONE,          /* no data encoding (simple pulse devices) */
    GTE_ENCODER_DHT11_40BIT,   /* 5-byte [hum_i, hum_d, temp_i, temp_d, cksum] */
    GTE_ENCODER_RAW_PULSE,     /* single pulse, width from param */
} GteEncoderType;

/* ── Per-Device State ────────────────────────────────────────────────────── */

typedef struct {
    char name[GTE_NAME_LEN];
    bool active;               /* currently executing a timing sequence */

    /* Pin configuration */
    int watch_pin;             /* GPIO pin to monitor for trigger */
    bool watch_rising;         /* trigger on rising edge (true) or falling (false) */
    bool bidirectional;        /* single-pin bidirectional (e.g., DHT11 data) */
    int drive_pins[GTE_MAX_DRIVE_PINS];
    int num_drive_pins;
    int idle_values[GTE_MAX_DRIVE_PINS]; /* idle pin state */

    /* Timing script */
    GteStep steps[GTE_MAX_STEPS];
    int num_steps;
    int current_step;

    /* Repeat state (simple stack for one nesting level) */
    int repeat_counter;
    int repeat_start_step;

    /* Parameters (updatable at runtime) */
    float params[GTE_MAX_PARAMS];
    int num_params;

    /* Data encoder */
    GteEncoderType encoder_type;
    uint8_t data_bytes[GTE_MAX_DATA_BYTES];
    int data_bit_count;
    int current_bit;

    /* QEMU timer */
    QEMUTimer *step_timer;

    /* GPIO reference */
    Esp32GpioState *gpio;

    /* Trigger state tracking */
    bool pin_was_high;         /* for edge detection */
    bool pin_was_low;          /* for bidirectional: host-start detect */
} GteDevice;

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * Parse a JSON timing script file and create GteDevice instances.
 *
 * @param file_path  Path to the JSON file (array of device scripts)
 * @param devices    Output array (must have room for GTE_MAX_DEVICES)
 * @return           Number of devices parsed, or -1 on error
 */
int gte_parse_scripts(const char *file_path, GteDevice devices[]);

/**
 * Attach a parsed device to the ESP32 GPIO controller.
 * Registers output watchers for the trigger pin.
 */
void gte_attach_gpio(GteDevice *dev, Esp32GpioState *gpio);

/**
 * Update a device parameter at runtime.
 */
void gte_update_param(GteDevice *dev, int param_index, float value);

/**
 * Encode data bytes from current parameters using the device's encoder.
 */
void gte_encode_data(GteDevice *dev);

#endif /* GPIO_TIMING_EXECUTOR_H */
