#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/dht11_vdev.h"
#include "hw/gpio/esp32_gpio.h"

/*
 * DHT11 Temperature/Humidity Sensor — QEMU GPIO Virtual Device
 *
 * Bit-bang protocol driven entirely by QEMU_CLOCK_VIRTUAL timers.
 * Each state transition is scheduled at exact virtual-time points,
 * so firmware's timing measurements are cycle-accurate.
 */

static void dht11_step(void *opaque);

static void dht11_encode_data(Dht11VDev *d)
{
    float t = d->temperature;
    float h = d->humidity;
    if (d->noise_temp > 0) t += (float)g_random_double_range(-d->noise_temp, d->noise_temp);
    if (d->noise_hum > 0)  h += (float)g_random_double_range(-d->noise_hum, d->noise_hum);
    if (t < 0) t = 0; if (t > 50) t = 50;
    if (h < 20) h = 20; if (h > 90) h = 90;

    uint8_t hum_int  = (uint8_t)h;
    uint8_t hum_dec  = 0;  /* DHT11 doesn't use decimal */
    uint8_t temp_int = (uint8_t)t;
    uint8_t temp_dec = 0;
    d->data_bytes[0] = hum_int;
    d->data_bytes[1] = hum_dec;
    d->data_bytes[2] = temp_int;
    d->data_bytes[3] = temp_dec;
    d->data_bytes[4] = (hum_int + hum_dec + temp_int + temp_dec) & 0xFF;
}

static void dht11_schedule_ns(Dht11VDev *d, uint64_t delay_ns)
{
    timer_mod_ns(d->step_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns);
}

/* State machine step — called by virtual timer */
static void dht11_step(void *opaque)
{
    Dht11VDev *d = (Dht11VDev*)opaque;
    Esp32GpioState *gpio = (Esp32GpioState*)d->gpio;
    if (!gpio) return;

    switch (d->state) {
    case DHT11_RESPONSE_LOW:
        /* Device pulls DATA LOW for 80µs */
        esp32_gpio_set_input_level(gpio, d->data_pin, false);
        d->state = DHT11_RESPONSE_HIGH;
        dht11_schedule_ns(d, 80000); /* 80µs */
        break;

    case DHT11_RESPONSE_HIGH:
        /* Device pulls DATA HIGH for 80µs */
        esp32_gpio_set_input_level(gpio, d->data_pin, true);
        d->state = DHT11_BIT_LOW;
        d->current_bit = 0;
        dht11_schedule_ns(d, 80000); /* 80µs */
        break;

    case DHT11_BIT_LOW:
        /* 50µs LOW before each data bit */
        esp32_gpio_set_input_level(gpio, d->data_pin, false);
        d->state = DHT11_BIT_HIGH;
        dht11_schedule_ns(d, 50000); /* 50µs */
        break;

    case DHT11_BIT_HIGH: {
        /* HIGH duration encodes the bit value */
        esp32_gpio_set_input_level(gpio, d->data_pin, true);

        int byte_idx = d->current_bit / 8;
        int bit_idx  = 7 - (d->current_bit % 8); /* MSB first */
        bool bit_val = (d->data_bytes[byte_idx] >> bit_idx) & 1;

        /* bit 0 = 26µs HIGH, bit 1 = 70µs HIGH */
        uint64_t high_ns = bit_val ? 70000 : 26000;

        d->current_bit++;
        if (d->current_bit >= DHT11_DATA_BITS) {
            d->state = DHT11_DONE;
        } else {
            d->state = DHT11_BIT_LOW;
        }
        dht11_schedule_ns(d, high_ns);
        break;
    }

    case DHT11_DONE:
        /* Release DATA line (back to idle HIGH) */
        esp32_gpio_set_input_level(gpio, d->data_pin, true);
        d->state = DHT11_IDLE;
        break;

    default:
        break;
    }
}

/*
 * GPIO watcher: detects firmware pulling DATA LOW (start signal).
 * When firmware releases (HIGH), start the response sequence.
 */
static void dht11_data_pin_cb(void *opaque, int pin, bool high)
{
    Dht11VDev *d = (Dht11VDev*)opaque;
    (void)pin;

    if (!high && !d->pin_was_low && d->state == DHT11_IDLE) {
        /* Falling edge — firmware starts pulling LOW (18ms) */
        d->pin_was_low = true;
    } else if (high && d->pin_was_low && d->state == DHT11_IDLE) {
        /* Rising edge — firmware released after ≥18ms LOW.
         * Start device response after ~20µs delay. */
        d->pin_was_low = false;
        dht11_encode_data(d);
        d->state = DHT11_RESPONSE_LOW;
        dht11_schedule_ns(d, 20000); /* 20µs before response */
    }
}

Dht11VDev* dht11_vdev_create(int data_pin, float temperature, float humidity)
{
    Dht11VDev *d = g_new0(Dht11VDev, 1);
    d->base.name = "DHT11";
    d->base.bus_type = VDEV_BUS_GPIO;
    d->base.present = true;
    d->base.responding = true;
    d->data_pin = data_pin;
    d->temperature = (temperature >= 0 && temperature <= 50) ? temperature : 25.0f;
    d->humidity = (humidity >= 20 && humidity <= 90) ? humidity : 60.0f;
    d->noise_temp = 0.5f;
    d->noise_hum = 1.0f;
    d->state = DHT11_IDLE;
    d->pin_was_low = false;
    d->gpio = NULL;

    d->step_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dht11_step, d);

    return d;
}

void dht11_vdev_attach_gpio(Dht11VDev *dev, void *gpio)
{
    if (!dev || !gpio) return;
    dev->gpio = gpio;
    /* Watch the DATA pin output from firmware */
    esp32_gpio_register_output_listener(
        (Esp32GpioState*)gpio, dev->data_pin, dht11_data_pin_cb, dev);
    /* Set initial state: DATA line HIGH (pull-up) */
    esp32_gpio_set_input_level((Esp32GpioState*)gpio, dev->data_pin, true);
}
