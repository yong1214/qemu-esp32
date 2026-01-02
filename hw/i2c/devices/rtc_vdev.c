#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/rtc_vdev.h"

static bool rtc_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void rtc_reset_v(VDevBase *base)
{
    RTCVDev *d = (RTCVDev*)base;
    d->seconds = 0x00;
    d->minutes = 0x00;
    d->hours = 0x12;  /* 12:00 */
    d->day = 0x01;
    d->month = 0x01;
    d->year = 0x24;   /* 2024 */
    d->control_register = 0x00;
    d->register_address = 0x00;
    d->oscillator_enabled = true;
    d->last_update_ns = 0;
}

static bool rtc_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    RTCVDev *d = (RTCVDev*)base;
    return base->present && base->responding && (d->i2c_addr7 == (addr7 & 0x7F));
}

static void rtc_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static ssize_t rtc_i2c_write(VDevBase *base, uint8_t addr7, const uint8_t *data, size_t length)
{
    (void)addr7;
    RTCVDev *d = (RTCVDev*)base;
    if (!data || length == 0) return 0;
    if (length == 1) { d->register_address = data[0]; return 1; }
    /* write selected register with next byte */
    if (length >= 2) {
        switch (d->register_address) {
        case 0x00: d->seconds = data[1]; break;
        case 0x01: d->minutes = data[1]; break;
        case 0x02: d->hours = data[1]; break;
        case 0x03: d->day = data[1]; break;
        case 0x04: d->month = data[1]; break;
        case 0x05: d->year = data[1]; break;
        case 0x07: d->control_register = data[1]; d->oscillator_enabled = (data[1] & 0x80) != 0; break;
        default: break;
        }
        return 2;
    }
    return (ssize_t)length;
}

static ssize_t rtc_i2c_read(VDevBase *base, uint8_t addr7, uint8_t *out, size_t length)
{
    (void)addr7;
    RTCVDev *d = (RTCVDev*)base;
    if (!out || length == 0) return 0;
    /* Update time based on elapsed virtual time */
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (d->last_update_ns == 0) d->last_update_ns = now;
    if (d->oscillator_enabled && now > d->last_update_ns) {
        uint64_t elapsed_ns = now - d->last_update_ns;
        uint32_t add_secs = (uint32_t)(elapsed_ns / 1000000000ULL);
        if (add_secs > 0) {
            uint32_t s = d->seconds + add_secs;
            d->seconds = s % 60;
            uint32_t carry_min = s / 60;
            if (carry_min) {
                uint32_t m = d->minutes + carry_min;
                d->minutes = m % 60;
                uint32_t carry_hr = m / 60;
                if (carry_hr) {
                    d->hours = (uint8_t)((d->hours + carry_hr) % 24);
                }
            }
            d->last_update_ns += (uint64_t)add_secs * 1000000000ULL;
        }
    }
    uint8_t val = 0x00;
    switch (d->register_address) {
    case 0x00: val = d->seconds; break;
    case 0x01: val = d->minutes; break;
    case 0x02: val = d->hours; break;
    case 0x03: val = d->day; break;
    case 0x04: val = d->month; break;
    case 0x05: val = d->year; break;
    case 0x07: val = d->control_register; break;
    default: val = 0x00; break;
    }
    out[0] = val;
    return 1;
}

static const VDevVTable rtc_vtable = {
    .init = rtc_init_v,
    .reset = rtc_reset_v,
    .destroy = NULL,
    .tick = NULL,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps rtc_i2c_ops = {
    .i2c_can_ack = rtc_i2c_can_ack,
    .i2c_on_addressed = rtc_i2c_on_addressed,
    .i2c_write = rtc_i2c_write,
    .i2c_read = rtc_i2c_read,
};

RTCVDev* rtc_vdev_create(uint8_t addr7)
{
    RTCVDev *d = g_new0(RTCVDev, 1);
    d->base.name = "RTC";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.debug_enabled = true;
    d->base.vtable = &rtc_vtable;
    d->i2c_addr7 = addr7 & 0x7F;
    rtc_reset_v(&d->base);
    return d;
}

const VDevI2COps* rtc_vdev_get_i2c_ops(void)
{
    return &rtc_i2c_ops;
}


