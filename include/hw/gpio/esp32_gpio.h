#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj)             OBJECT_CHECK(Esp32GpioState, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32GpioClass, obj, TYPE_ESP32_GPIO)
#define ESP32_GPIO_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32GpioClass, klass, TYPE_ESP32_GPIO)

REG32(GPIO_STRAP, 0x0038)
REG32(GPIO_IN,    0x003c)
REG32(GPIO_IN1,   0x0040)
REG32(GPIO_OUT,        0x0004)
REG32(GPIO_OUT_W1TS,   0x0008)
REG32(GPIO_OUT_W1TC,   0x000c)
REG32(GPIO_OUT1,       0x0010)
REG32(GPIO_OUT1_W1TS,  0x0014)
REG32(GPIO_OUT1_W1TC,  0x0018)

#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT  0x0f

typedef struct Esp32GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t strap_mode;
    /* Simple input level model: GPIO0..31 -> IN, GPIO32..63 -> IN1 */
    uint32_t in_val[2];
    /* Output latch values */
    uint32_t out_val[2];
    /* GPIO matrix: OUT select per pin (0..39), stores func_sel (0..255) */
    uint16_t func_out_sel_cfg[40];
    struct Esp32GpioWatch *watchers[40];

    /* GTPE read interception */
    void *gte_devices;         /* GteDevice array (set by esp32.c) */
    int gte_device_count;
} Esp32GpioState;

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;
} Esp32GpioClass;

typedef void (*Esp32GpioLevelCb)(void *opaque, int pin, bool high);

typedef struct Esp32GpioWatch {
    Esp32GpioLevelCb cb;
    void *opaque;
    struct Esp32GpioWatch *next;
} Esp32GpioWatch;

/* Helper to set input level of a pin (true=HIGH, false=LOW) */
static inline void esp32_gpio_set_input_level(Esp32GpioState *s, int pin, bool high)
{
    if (!s || pin < 0) return;
    int bank = (pin >= 32) ? 1 : 0;
    int bit = pin & 31;
    if (high) {
        s->in_val[bank] |= (1u << bit);
    } else {
        s->in_val[bank] &= ~(1u << bit);
    }
}

/* Query which pin is currently mapped to a given output signal index; -1 if none */
static inline int esp32_gpio_get_pin_for_signal(Esp32GpioState *s, int signal_idx)
{
    if (!s) return -1;
    for (int pin = 0; pin < 40; ++pin) {
        if ((int)s->func_out_sel_cfg[pin] == signal_idx) {
            return pin;
        }
    }
    return -1;
}

void esp32_gpio_register_output_listener(Esp32GpioState *s, int pin,
                                         Esp32GpioLevelCb cb, void *opaque);

bool esp32_gpio_get_output_level(Esp32GpioState *s, int pin);

void esp32_gpio_unregister_output_listener(Esp32GpioState *s, int pin,
                                           Esp32GpioLevelCb cb, void *opaque);
