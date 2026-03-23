#ifndef HC_SR04_VDEV_H
#define HC_SR04_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "qemu/timer.h"

/**
 * HC-SR04 Ultrasonic Distance Sensor — GPIO Virtual Device
 *
 * Protocol:
 *   Firmware: TRIGGER HIGH for ≥10µs
 *   Device:   After ~2µs delay, ECHO goes HIGH for distance_cm * 58µs
 *   Firmware: pulseIn(ECHO) measures echo duration
 *
 * This device watches the TRIGGER pin via GPIO watcher callback and
 * schedules ECHO pin responses using QEMU_CLOCK_VIRTUAL timers for
 * cycle-accurate timing.
 */
typedef struct HcSr04VDev {
    VDevBase base;

    /* Pin assignments */
    int trigger_pin;
    int echo_pin;

    /* Simulated sensor value */
    float distance_cm;          /* simulated distance */
    float noise_cm;             /* ±noise amplitude */

    /* State */
    bool trigger_was_high;      /* edge detection */
    QEMUTimer *echo_start_timer;
    QEMUTimer *echo_end_timer;

    /* GPIO reference (set after creation) */
    void *gpio;                 /* Esp32GpioState* */
} HcSr04VDev;

HcSr04VDev* hc_sr04_vdev_create(int trigger_pin, int echo_pin, float distance_cm);
void hc_sr04_vdev_attach_gpio(HcSr04VDev *dev, void *gpio);

#endif /* HC_SR04_VDEV_H */
