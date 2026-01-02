#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/ds3231_vdev.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/gpio/esp32_gpio.h"

static bool ds3231_init_v(VDevBase *base) { (void)base; return true; }

static void ds3231_reset_v(VDevBase *base)
{
    DS3231VDev *d = (DS3231VDev*)base;
    d->t.seconds = 0; d->t.minutes = 0; d->t.hours = 12; d->t.day = 1; d->t.month = 1; d->t.year = 24; d->t.dow = 1;
    d->control = 0x00; d->status = 0x00; d->reg_ptr = 0x00; d->oscillator_enabled = true; d->last_update_ns = 0;
    d->a1_sec = d->a1_min = d->a1_hr = d->a1_dydt = 0x80; /* M* = 1 => disabled */
    d->a2_min = d->a2_hr = d->a2_dydt = 0x80;
    d->temp_c = 25.0f;
    d->sqw_state = false;
    d->last_sqw_toggle_ns = 0;
}

static void ds3231_tick(DS3231VDev *d)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (!d->oscillator_enabled) { d->last_update_ns = now ? now : d->last_update_ns; return; }
    if (d->last_update_ns == 0) { d->last_update_ns = now; return; }
    if (now <= d->last_update_ns) return;
    uint64_t elapsed_ns = now - d->last_update_ns;
    uint32_t add_secs = (uint32_t)(elapsed_ns / 1000000000ULL);
    if (add_secs == 0) return;
    d->last_update_ns += (uint64_t)add_secs * 1000000000ULL;
    uint32_t secs_val = d->t.seconds + add_secs;
    d->t.seconds = secs_val % 60; uint32_t carry_min = secs_val / 60;
    if (carry_min) {
        uint32_t m = d->t.minutes + carry_min;
        d->t.minutes = m % 60; uint32_t carry_hr = m / 60;
        if (carry_hr) {
            d->t.hours = (uint8_t)((d->t.hours + carry_hr) % 24);
        }
    }
    /* Evaluate alarms (basic exact-match when masks M* are 0 and DY/DT=0 date mode) */
    /* Alarm 1 */
    bool a1 = false;
    if (((d->a1_sec & 0x80) == 0) && ((d->a1_min & 0x80) == 0) && ((d->a1_hr & 0x80) == 0) && ((d->a1_dydt & 0x80) == 0)) {
        uint8_t sec = rtc_bin2bcd(d->t.seconds) & 0x7F;
        uint8_t min = rtc_bin2bcd(d->t.minutes) & 0x7F;
        uint8_t hr  = rtc_bin2bcd(d->t.hours) & 0x3F; /* 24h */
        uint8_t day = rtc_bin2bcd(d->t.day) & 0x3F;   /* DY/DT=0 -> date */
        if ((sec == (d->a1_sec & 0x7F)) && (min == (d->a1_min & 0x7F)) && (hr == (d->a1_hr & 0x3F)) && (day == (d->a1_dydt & 0x3F))) a1 = true;
    }
    if (a1) { d->status |= 0x01; }
    /* Alarm 2 */
    bool a2 = false;
    if (((d->a2_min & 0x80) == 0) && ((d->a2_hr & 0x80) == 0) && ((d->a2_dydt & 0x80) == 0)) {
        uint8_t min = rtc_bin2bcd(d->t.minutes) & 0x7F;
        uint8_t hr  = rtc_bin2bcd(d->t.hours) & 0x3F;
        uint8_t day = rtc_bin2bcd(d->t.day) & 0x3F;
        if ((min == (d->a2_min & 0x7F)) && (hr == (d->a2_hr & 0x3F)) && (day == (d->a2_dydt & 0x3F))) a2 = true;
    }
    if (a2) { d->status |= 0x02; }

    /* Drive SQW/INT output: when INTCN=0 -> square wave; when INTCN=1 -> INT low on A1F/A2F */
    bool intcn = (d->control & 0x04) != 0;
    bool a1ie = (d->control & 0x01) != 0;
    bool a2ie = (d->control & 0x02) != 0;
    bool a1f = (d->status & 0x01) != 0;
    bool a2f = (d->status & 0x02) != 0;
    bool level_high = true;
    if (intcn) {
        bool active = (a1ie && a1f) || (a2ie && a2f);
        /* INT is active-low */
        level_high = !active;
    } else {
        /* Simple 1Hz square wave (RS bits ignored for now) */
        if (d->last_sqw_toggle_ns == 0 || (now - d->last_sqw_toggle_ns) >= 500000000ULL) {
            d->sqw_state = !d->sqw_state;
            d->last_sqw_toggle_ns = now;
        }
        level_high = d->sqw_state;
    }
    Esp32I2CState *s = (Esp32I2CState*)d->base.bus_context;
    if (s && s->gpio && s->rtc_sqw_pin >= 0) {
        esp32_gpio_set_input_level(s->gpio, s->rtc_sqw_pin, level_high);
    }
}

static bool ds3231_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{ (void)is_read; DS3231VDev *d = (DS3231VDev*)base; return base->present && base->responding && (d->i2c_addr7 == (addr7 & 0x7F)); }

static void ds3231_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{ (void)addr7; (void)is_read; base->access_count++; base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); }

static uint8_t ds3231_reg_read(DS3231VDev *d, uint8_t reg)
{
    ds3231_tick(d);
    switch (reg) {
    case 0x00: return (d->oscillator_enabled ? rtc_bin2bcd(d->t.seconds) : (rtc_bin2bcd(d->t.seconds) | 0x80));
    case 0x01: return rtc_bin2bcd(d->t.minutes);
    case 0x02: return rtc_bin2bcd(d->t.hours) & 0x3F; /* 24h */
    case 0x03: return (d->t.dow >= 1 && d->t.dow <= 7) ? d->t.dow : 1;
    case 0x04: return rtc_bin2bcd(d->t.day);
    case 0x05: return rtc_bin2bcd(d->t.month);
    case 0x06: return rtc_bin2bcd(d->t.year);
    case 0x0E: return d->control;
    case 0x0F: return d->status;
    /* Alarm 1 */
    case 0x07: return d->a1_sec;
    case 0x08: return d->a1_min;
    case 0x09: return d->a1_hr;
    case 0x0A: return d->a1_dydt;
    /* Alarm 2 */
    case 0x0B: return d->a2_min;
    case 0x0C: return d->a2_hr;
    case 0x0D: return d->a2_dydt;
    /* Temperature registers */
    case 0x11: {
        int16_t q = (int16_t)(d->temp_c * 4.0f);
        int8_t msb = (int8_t)(q / 4); /* whole degrees signed */
        return (uint8_t)msb;
    }
    case 0x12: {
        int16_t q = (int16_t)(d->temp_c * 4.0f);
        uint8_t frac = (uint8_t)(q & 0x03); /* quarter degrees */
        return (uint8_t)(frac << 6);
    }
    default: return 0x00;
    }
}

static void ds3231_reg_write(DS3231VDev *d, uint8_t reg, uint8_t val)
{
    switch (reg) {
    case 0x00: d->oscillator_enabled = (val & 0x80) ? false : true; d->t.seconds = rtc_bcd2bin(val & 0x7F) % 60; d->last_update_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); break;
    case 0x01: d->t.minutes = rtc_bcd2bin(val & 0x7F) % 60; d->last_update_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); break;
    case 0x02: d->t.hours = rtc_bcd2bin(val & 0x3F) % 24; d->last_update_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); break;
    case 0x03: d->t.dow = (val == 0) ? 1 : (val % 8); break;
    case 0x04: d->t.day = rtc_bcd2bin(val & 0x3F); if (d->t.day < 1) d->t.day = 1; break;
    case 0x05: d->t.month = rtc_bcd2bin(val & 0x1F); if (d->t.month < 1) d->t.month = 1; break;
    case 0x06: d->t.year = rtc_bcd2bin(val); break;
    case 0x0E:
        /* Control: allow INTCN/A1IE/A2IE/RS bits; CONV triggers temp refresh; EOSC stops osc */
        d->control = val;
        d->oscillator_enabled = (val & 0x80) ? false : true; /* EOSC */
        if (val & 0x20) { /* CONV */ }
        break;
    case 0x0F:
        /* Writing 1 clears A1F/A2F/OSF bits as per datasheet */
        d->status &= ~(val & 0x03); /* A1F/A2F */
        if (val & 0x80) d->status &= ~0x80; /* OSF */
        /* EN32kHz bit (0x08) preserved */
        break;
    /* Alarm 1 */
    case 0x07: d->a1_sec = val; break;
    case 0x08: d->a1_min = val; break;
    case 0x09: d->a1_hr = val; break;
    case 0x0A: d->a1_dydt = val; break;
    /* Alarm 2 */
    case 0x0B: d->a2_min = val; break;
    case 0x0C: d->a2_hr = val; break;
    case 0x0D: d->a2_dydt = val; break;
    default: /* Ignore other areas for now */ break;
    }
}

static ssize_t ds3231_i2c_write(VDevBase *base, uint8_t addr7, const uint8_t *data, size_t length)
{
    (void)addr7; DS3231VDev *d = (DS3231VDev*)base; if (!data || length == 0) return 0;
    d->reg_ptr = data[0];
    for (size_t i = 1; i < length; ++i) { ds3231_reg_write(d, d->reg_ptr, data[i]); d->reg_ptr++; }
    return (ssize_t)length;
}

static ssize_t ds3231_i2c_read(VDevBase *base, uint8_t addr7, uint8_t *out, size_t length)
{
    (void)addr7; DS3231VDev *d = (DS3231VDev*)base; if (!out || length == 0) return 0;
    for (size_t i = 0; i < length; ++i) { out[i] = ds3231_reg_read(d, d->reg_ptr); d->reg_ptr++; }
    return (ssize_t)length;
}

static const VDevVTable ds3231_vtable = {
    .init = ds3231_init_v,
    .reset = ds3231_reset_v,
    .destroy = NULL,
    .tick = NULL,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps ds3231_i2c_ops = {
    .i2c_can_ack = ds3231_i2c_can_ack,
    .i2c_on_addressed = ds3231_i2c_on_addressed,
    .i2c_write = ds3231_i2c_write,
    .i2c_read = ds3231_i2c_read,
};

DS3231VDev* ds3231_vdev_create(uint8_t addr7)
{ DS3231VDev *d = g_new0(DS3231VDev, 1); d->base.name = "DS3231"; d->base.bus_type = VDEV_BUS_I2C; d->base.present = true; d->base.responding = true; d->base.debug_enabled = true; d->base.vtable = &ds3231_vtable; d->i2c_addr7 = addr7 & 0x7F; ds3231_reset_v(&d->base); return d; }

const VDevI2COps* ds3231_vdev_get_i2c_ops(void) { return &ds3231_i2c_ops; }


