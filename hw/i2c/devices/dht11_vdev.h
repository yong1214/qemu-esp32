#ifndef DHT11_VDEV_H
#define DHT11_VDEV_H

#include "hw/i2c/virtual_device.h"
#include "qemu/timer.h"

/**
 * DHT11 Temperature/Humidity Sensor — GPIO Virtual Device
 *
 * Protocol (single-wire, 40 bits):
 *   1. Firmware pulls DATA LOW for ≥18ms, then releases (HIGH)
 *   2. Device responds:
 *      - 80µs LOW + 80µs HIGH (start signal)
 *      - 40 data bits, each: 50µs LOW + 26-28µs HIGH (bit 0) or 70µs HIGH (bit 1)
 *   3. Data format: [humidity_int][humidity_dec][temp_int][temp_dec][checksum]
 *
 * Uses QEMU_CLOCK_VIRTUAL timers for exact microsecond timing.
 */

#define DHT11_DATA_BITS 40

typedef enum {
    DHT11_IDLE,
    DHT11_HOST_START,       /* firmware is pulling low */
    DHT11_RESPONSE_LOW,     /* device 80µs low */
    DHT11_RESPONSE_HIGH,    /* device 80µs high */
    DHT11_BIT_LOW,          /* 50µs low before each bit */
    DHT11_BIT_HIGH,         /* 26-70µs high (the data bit) */
    DHT11_DONE
} Dht11State;

typedef struct Dht11VDev {
    VDevBase base;

    int data_pin;

    /* Simulated values */
    float temperature;      /* °C (DHT11: 0-50, integer only) */
    float humidity;         /* % (DHT11: 20-90, integer only) */
    float noise_temp;
    float noise_hum;

    /* Protocol state */
    Dht11State state;
    int current_bit;        /* 0-39 */
    uint8_t data_bytes[5];  /* [hum_int][hum_dec][temp_int][temp_dec][checksum] */
    bool pin_was_low;       /* edge detection */

    QEMUTimer *step_timer;
    void *gpio;             /* Esp32GpioState* */
} Dht11VDev;

Dht11VDev* dht11_vdev_create(int data_pin, float temperature, float humidity);
void dht11_vdev_attach_gpio(Dht11VDev *dev, void *gpio);

#endif /* DHT11_VDEV_H */
