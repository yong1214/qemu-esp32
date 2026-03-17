#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/bh1750_vdev.h"

static bool bh1750_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void bh1750_reset_v(VDevBase *base)
{
    BH1750VDev *d = (BH1750VDev*)base;
    d->mode = 0x00;
    d->powered_on = false;
    d->last_tick_ns = 0;
}

static void bh1750_tick_v(VDevBase *base, uint64_t now_ns)
{
    BH1750VDev *d = (BH1750VDev*)base;
    if (!d->powered_on) return;
    if (d->last_tick_ns == 0) {
        d->last_tick_ns = now_ns;
        return;
    }
    uint64_t elapsed = now_ns - d->last_tick_ns;
    if (elapsed >= 100000ULL) { /* 100us period */
        double noise = g_random_double_range(-d->noise_lux_amplitude,
                                              d->noise_lux_amplitude);
        double updated = (double)d->lux + noise;
        if (updated < 0.0) updated = 0.0;
        d->lux = (float)updated;
        d->last_tick_ns = now_ns;
    }
}

static bool bh1750_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    BH1750VDev *d = (BH1750VDev*)base;
    if (d->i2c_addr7 != (addr7 & 0x7F)) return false;
    return base->responding && base->present;
}

static void bh1750_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static ssize_t bh1750_i2c_write(VDevBase *base, uint8_t addr7,
                                  const uint8_t *data, size_t length)
{
    (void)addr7;
    BH1750VDev *d = (BH1750VDev*)base;
    if (!data || length == 0) return 0;

    uint8_t cmd = data[0];
    if (cmd == 0x01) {
        /* Power on */
        d->powered_on = true;
    } else if (cmd == 0x00) {
        /* Power off */
        d->powered_on = false;
    } else if (cmd == 0x07) {
        /* Reset */
        /* Only resets measurement register, keeps power state */
    } else if (cmd == 0x10 || cmd == 0x11 || cmd == 0x13 ||
               cmd == 0x20 || cmd == 0x21 || cmd == 0x23) {
        /* Measurement modes */
        d->mode = cmd;
    }
    return (ssize_t)length;
}

static ssize_t bh1750_i2c_read(VDevBase *base, uint8_t addr7,
                                 uint8_t *out, size_t length)
{
    (void)addr7;
    BH1750VDev *d = (BH1750VDev*)base;
    if (!out || length == 0) return 0;

    uint16_t raw = (uint16_t)(d->lux / 1.2f);
    if (length >= 2) {
        out[0] = (raw >> 8) & 0xFF;
        out[1] = raw & 0xFF;
        return 2;
    }
    out[0] = (raw >> 8) & 0xFF;
    return 1;
}

static const VDevVTable bh1750_vtable = {
    .init = bh1750_init_v,
    .reset = bh1750_reset_v,
    .destroy = NULL,
    .tick = bh1750_tick_v,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps bh1750_i2c_ops = {
    .i2c_can_ack = bh1750_i2c_can_ack,
    .i2c_on_addressed = bh1750_i2c_on_addressed,
    .i2c_write = bh1750_i2c_write,
    .i2c_read = bh1750_i2c_read,
};

BH1750VDev* bh1750_vdev_create(uint8_t addr7, float initial_lux)
{
    BH1750VDev *d = g_new0(BH1750VDev, 1);
    d->base.name = "BH1750";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.debug_enabled = true;
    d->base.vtable = &bh1750_vtable;
    d->i2c_addr7 = addr7 & 0x7F;
    d->mode = 0x00;
    d->powered_on = false;
    d->lux = initial_lux;
    d->noise_lux_amplitude = 5.0;
    d->last_tick_ns = 0;
    return d;
}

const VDevI2COps* bh1750_vdev_get_i2c_ops(void)
{
    return &bh1750_i2c_ops;
}
