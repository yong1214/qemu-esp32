/*
 * ESP32 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/gpio/gpio_timing_executor.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"



static void esp32_gpio_update_output(Esp32GpioState *s, int pin, bool high)
{
    if (!s || pin < 0 || pin >= 40) {
        return;
    }
    int bank = (pin >= 32) ? 1 : 0;
    int bit = pin & 31;
    bool old = (s->out_val[bank] >> bit) & 1u;
    if (old == high) {
        return;
    }
    if (high) {
        s->out_val[bank] |= (1u << bit);
    } else {
        s->out_val[bank] &= ~(1u << bit);
    }
    Esp32GpioWatch *w = s->watchers[pin];
    while (w) {
        w->cb(w->opaque, pin, high);
        w = w->next;
    }
}

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_GPIO_STRAP:
        r = s->strap_mode;
        break;

    case A_GPIO_IN:
        r = s->in_val[0];
        /* Apply GTPE read interception: compute correct pin states
         * for active virtual GPIO devices based on virtual time */
        if (s->gte_devices && s->gte_device_count > 0) {
            r = gte_apply_read_intercept(r, 0,
                    (GteDevice *)s->gte_devices, s->gte_device_count);
        }
        break;

    case A_GPIO_IN1:
        r = s->in_val[1];
        if (s->gte_devices && s->gte_device_count > 0) {
            r = gte_apply_read_intercept(r, 1,
                    (GteDevice *)s->gte_devices, s->gte_device_count);
        }
        break;

    case A_GPIO_OUT:
        r = s->out_val[0];
        break;

    case A_GPIO_OUT1:
        r = s->out_val[1];
        break;

    default:
        break;
    }
    return r;
}

static void esp32_gpio_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    /* Handle GPIO matrix OUT select registers: DR_REG_GPIO_BASE + 0x0530 + 4*pin */
    if (addr >= 0x0530 && addr < 0x0530 + 40*4) {
        int pin = (int)((addr - 0x0530) >> 2);
        if (pin >= 0 && pin < 40) {
            /* bits [8:0] func_sel */
            s->func_out_sel_cfg[pin] = (uint16_t)(value & 0x1FF);
        }
        return;
    }

    switch (addr) {
    case A_GPIO_OUT:
        for (int bit = 0; bit < 32; ++bit) {
            bool high = (value >> bit) & 1u;
            esp32_gpio_update_output(s, bit, high);
        }
        return;
    case A_GPIO_OUT_W1TS:
        for (int bit = 0; bit < 32; ++bit) {
            if (value & (1u << bit)) {
                esp32_gpio_update_output(s, bit, true);
            }
        }
        return;
    case A_GPIO_OUT_W1TC:
        for (int bit = 0; bit < 32; ++bit) {
            if (value & (1u << bit)) {
                esp32_gpio_update_output(s, bit, false);
            }
        }
        return;
    case A_GPIO_OUT1:
        for (int bit = 0; bit < 32; ++bit) {
            bool high = (value >> bit) & 1u;
            esp32_gpio_update_output(s, 32 + bit, high);
        }
        return;
    case A_GPIO_OUT1_W1TS:
        for (int bit = 0; bit < 32; ++bit) {
            if (value & (1u << bit)) {
                esp32_gpio_update_output(s, 32 + bit, true);
            }
        }
        return;
    case A_GPIO_OUT1_W1TC:
        for (int bit = 0; bit < 32; ++bit) {
            if (value & (1u << bit)) {
                esp32_gpio_update_output(s, 32 + bit, false);
            }
        }
        return;
    default:
        break;
    }
}

static const MemoryRegionOps uart_ops = {
    .read =  esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset_hold(Object *obj, ResetType type)
{
}

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Set the default value for the strap_mode property */
    object_property_set_int(obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT, &error_fatal);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    /* Default all pins HIGH (pull-ups / idle) */
    s->in_val[0] = 0xFFFFFFFFu;
    s->in_val[1] = 0xFFFFFFFFu;
    s->out_val[0] = 0xFFFFFFFFu;
    s->out_val[1] = 0xFFFFFFFFu;
    for (int i = 0; i < 40; ++i) { s->func_out_sel_cfg[i] = 0x1FF; }
    for (int i = 0; i < 40; ++i) { s->watchers[i] = NULL; }
}

static Property esp32_gpio_properties[] = {
    /* The strap_mode needs to be explicitly set in the instance init, thus, set
     * the default value to 0. */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_gpio_reset_hold;
    dc->realize = esp32_gpio_realize;
    device_class_set_props(dc, esp32_gpio_properties);
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

void esp32_gpio_register_output_listener(Esp32GpioState *s, int pin,
                                         Esp32GpioLevelCb cb, void *opaque)
{
    if (!s || pin < 0 || pin >= 40 || !cb) {
        return;
    }
    Esp32GpioWatch *head = s->watchers[pin];
    for (Esp32GpioWatch *it = head; it; it = it->next) {
        if (it->cb == cb && it->opaque == opaque) {
            return; /* already registered */
        }
    }
    Esp32GpioWatch *w = g_new0(Esp32GpioWatch, 1);
    w->cb = cb;
    w->opaque = opaque;
    w->next = head;
    s->watchers[pin] = w;
}

bool esp32_gpio_get_output_level(Esp32GpioState *s, int pin)
{
    if (!s || pin < 0 || pin >= 64) {
        return true;
    }
    int bank = (pin >= 32) ? 1 : 0;
    int bit = pin & 31;
    return (s->out_val[bank] >> bit) & 1u;
}

void esp32_gpio_unregister_output_listener(Esp32GpioState *s, int pin,
                                           Esp32GpioLevelCb cb, void *opaque)
{
    if (!s || pin < 0 || pin >= 40 || !cb) {
        return;
    }
    Esp32GpioWatch **link = &s->watchers[pin];
    while (*link) {
        Esp32GpioWatch *node = *link;
        if (node->cb == cb && node->opaque == opaque) {
            *link = node->next;
            g_free(node);
            return;
        }
        link = &node->next;
    }
}

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
