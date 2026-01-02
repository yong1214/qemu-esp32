#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/ds1307_vdev.h"

static bool ds1307_init_v(VDevBase *base)
{
    (void)base; return true;
}

static void ds1307_reset_v(VDevBase *base)
{
    DS1307VDev *d = (DS1307VDev*)base;
    d->t.seconds = 0; d->t.minutes = 0; d->t.hours = 12; d->t.day = 1; d->t.month = 1; d->t.year = 24; d->t.dow = 1;
    d->oscillator_enabled = true;
    d->control = 0x00;
    d->reg_ptr = 0x00;
    d->last_update_ns = 0;
}

static void ds1307_tick(DS1307VDev *d)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (!d->oscillator_enabled) { d->last_update_ns = now ? now : d->last_update_ns; return; }
    if (d->last_update_ns == 0) { d->last_update_ns = now; return; }
    if (now <= d->last_update_ns) return;
    uint64_t elapsed_ns = now - d->last_update_ns;
    uint32_t add_secs = (uint32_t)(elapsed_ns / 1000000000ULL);
    if (add_secs == 0) return;
    d->last_update_ns += (uint64_t)add_secs * 1000000000ULL;
    uint32_t s = d->t.seconds + add_secs;
    d->t.seconds = s % 60; uint32_t carry_min = s / 60;
    if (carry_min) {
        uint32_t m = d->t.minutes + carry_min;
        d->t.minutes = m % 60; uint32_t carry_hr = m / 60;
        if (carry_hr) {
            d->t.hours = (uint8_t)((d->t.hours + carry_hr) % 24);
        }
    }
}

static bool ds1307_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read; DS1307VDev *d = (DS1307VDev*)base; return base->present && base->responding && (d->i2c_addr7 == (addr7 & 0x7F));
}

static void ds1307_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read; base->access_count++; base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint8_t ds1307_reg_read(DS1307VDev *d, uint8_t reg)
{
    /* timekeeping update before read */
    ds1307_tick(d);
    switch (reg) {
    case 0x00: { /* seconds with CH */
        uint8_t b = rtc_bin2bcd(d->t.seconds);
        if (!d->oscillator_enabled) b |= 0x80; /* CH=1 halts */
        return b;
    }
    case 0x01: return rtc_bin2bcd(d->t.minutes);
    case 0x02: return rtc_bin2bcd(d->t.hours) & 0x3F; /* 24h */
    case 0x03: return (d->t.dow >= 1 && d->t.dow <= 7) ? d->t.dow : 1;
    case 0x04: return rtc_bin2bcd(d->t.day);
    case 0x05: return rtc_bin2bcd(d->t.month);
    case 0x06: return rtc_bin2bcd(d->t.year);
    case 0x07: return d->control;
    default: return 0x00; /* RAM not implemented */
    }
}

static void ds1307_reg_write(DS1307VDev *d, uint8_t reg, uint8_t val)
{
    switch (reg) {
    case 0x00: { /* seconds with CH */
        d->oscillator_enabled = (val & 0x80) ? false : true;
        d->t.seconds = rtc_bcd2bin(val & 0x7F) % 60; d->last_update_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    }
    case 0x01: d->t.minutes = rtc_bcd2bin(val & 0x7F) % 60; d->last_update_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); break;
    case 0x02: d->t.hours = rtc_bcd2bin(val & 0x3F) % 24; d->last_update_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); break;
    case 0x03: d->t.dow = (val == 0) ? 1 : (val % 8); break;
    case 0x04: d->t.day = rtc_bcd2bin(val & 0x3F); if (d->t.day < 1) d->t.day = 1; break;
    case 0x05: d->t.month = rtc_bcd2bin(val & 0x1F); if (d->t.month < 1) d->t.month = 1; break;
    case 0x06: d->t.year = rtc_bcd2bin(val); break;
    case 0x07: d->control = val; break;
    default: /* ignore RAM for now */ break;
    }
}

static ssize_t ds1307_i2c_write(VDevBase *base, uint8_t addr7, const uint8_t *data, size_t length)
{
    (void)addr7; DS1307VDev *d = (DS1307VDev*)base; if (!data || length == 0) return 0;
    /* First byte is register pointer, subsequent bytes auto-increment */
    d->reg_ptr = data[0];
    for (size_t i = 1; i < length; ++i) { ds1307_reg_write(d, d->reg_ptr, data[i]); d->reg_ptr++; }
    return (ssize_t)length;
}

static ssize_t ds1307_i2c_read(VDevBase *base, uint8_t addr7, uint8_t *out, size_t length)
{
    (void)addr7; DS1307VDev *d = (DS1307VDev*)base; if (!out || length == 0) return 0;
    for (size_t i = 0; i < length; ++i) { out[i] = ds1307_reg_read(d, d->reg_ptr); d->reg_ptr++; }
    return (ssize_t)length;
}

static const VDevVTable ds1307_vtable = {
    .init = ds1307_init_v,
    .reset = ds1307_reset_v,
    .destroy = NULL,
    .tick = NULL,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps ds1307_i2c_ops = {
    .i2c_can_ack = ds1307_i2c_can_ack,
    .i2c_on_addressed = ds1307_i2c_on_addressed,
    .i2c_write = ds1307_i2c_write,
    .i2c_read = ds1307_i2c_read,
};

DS1307VDev* ds1307_vdev_create(uint8_t addr7)
{
    DS1307VDev *d = g_new0(DS1307VDev, 1);
    d->base.name = "DS1307"; d->base.bus_type = VDEV_BUS_I2C; d->base.present = true; d->base.responding = true; d->base.debug_enabled = true; d->base.vtable = &ds1307_vtable; d->i2c_addr7 = addr7 & 0x7F; ds1307_reset_v(&d->base); return d;
}

const VDevI2COps* ds1307_vdev_get_i2c_ops(void) { return &ds1307_i2c_ops; }


