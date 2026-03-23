#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/hc_sr04_vdev.h"
#include "hw/gpio/esp32_gpio.h"

/*
 * HC-SR04 Ultrasonic Distance Sensor — QEMU GPIO Virtual Device
 *
 * Uses QEMU_CLOCK_VIRTUAL timers for cycle-accurate echo pulse timing.
 * Firmware's pulseIn() measures exact virtual microseconds.
 *
 * Distance → echo duration: echo_us = distance_cm * 58
 *   (sound speed ≈ 343 m/s → round trip: 1cm = 58µs)
 */

static void hc_sr04_echo_start(void *opaque)
{
    HcSr04VDev *d = (HcSr04VDev*)opaque;
    Esp32GpioState *gpio = (Esp32GpioState*)d->gpio;
    if (!gpio) return;

    /* Set ECHO pin HIGH */
    esp32_gpio_set_input_level(gpio, d->echo_pin, true);

    /* Schedule ECHO LOW after distance-proportional delay */
    float dist = d->distance_cm;
    if (d->noise_cm > 0) {
        dist += (float)g_random_double_range(-d->noise_cm, d->noise_cm);
        if (dist < 2.0f) dist = 2.0f;  /* HC-SR04 min range */
    }
    uint64_t echo_ns = (uint64_t)(dist * 58.0f * 1000.0f); /* cm→µs→ns */
    timer_mod_ns(d->echo_end_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + echo_ns);
}

static void hc_sr04_echo_end(void *opaque)
{
    HcSr04VDev *d = (HcSr04VDev*)opaque;
    Esp32GpioState *gpio = (Esp32GpioState*)d->gpio;
    if (!gpio) return;

    /* Set ECHO pin LOW — firmware's pulseIn() captures this edge */
    esp32_gpio_set_input_level(gpio, d->echo_pin, false);
}

/*
 * GPIO watcher callback: fires when firmware changes the TRIGGER pin.
 * Detects rising edge → schedules echo response.
 */
static void hc_sr04_trigger_cb(void *opaque, int pin, bool high)
{
    HcSr04VDev *d = (HcSr04VDev*)opaque;
    (void)pin;

    if (high && !d->trigger_was_high) {
        /* Rising edge on TRIGGER — schedule echo after ~2µs propagation delay */
        timer_mod_ns(d->echo_start_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2000); /* 2µs */
    }
    d->trigger_was_high = high;
}

HcSr04VDev* hc_sr04_vdev_create(int trigger_pin, int echo_pin, float distance_cm)
{
    HcSr04VDev *d = g_new0(HcSr04VDev, 1);
    d->base.name = "HC-SR04";
    d->base.bus_type = VDEV_BUS_GPIO;
    d->base.present = true;
    d->base.responding = true;
    d->trigger_pin = trigger_pin;
    d->echo_pin = echo_pin;
    d->distance_cm = (distance_cm > 0) ? distance_cm : 15.0f;
    d->noise_cm = 0.5f;
    d->trigger_was_high = false;
    d->gpio = NULL;

    /* Create virtual-time timers */
    d->echo_start_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, hc_sr04_echo_start, d);
    d->echo_end_timer   = timer_new_ns(QEMU_CLOCK_VIRTUAL, hc_sr04_echo_end, d);

    return d;
}

void hc_sr04_vdev_attach_gpio(HcSr04VDev *dev, void *gpio)
{
    if (!dev || !gpio) return;
    dev->gpio = gpio;
    /* Register watcher on TRIGGER pin output */
    esp32_gpio_register_output_listener(
        (Esp32GpioState*)gpio, dev->trigger_pin, hc_sr04_trigger_cb, dev);
}
