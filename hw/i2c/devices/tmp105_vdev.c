#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/tmp105_vdev.h"

static bool tmp105_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void tmp105_reset_v(VDevBase *base)
{
    TMP105VDev *d = (TMP105VDev*)base;
    d->config_register = 0x00;
    d->register_address = 0x00;
    d->shutdown_mode = false;
    d->read_phase = 0;
    d->last_conversion_time_ns = 0;
}

static void tmp105_tick_v(VDevBase *base, uint64_t now_ns)
{
    TMP105VDev *d = (TMP105VDev*)base;
    if (d->shutdown_mode) return;
    if (d->last_conversion_time_ns == 0) {
        d->last_conversion_time_ns = now_ns;
        return;
    }
    uint64_t elapsed = now_ns - d->last_conversion_time_ns;
    uint64_t period = (uint64_t)d->conversion_time_us * 1000ULL;
    if (elapsed >= period) {
        double dt_s = (double)elapsed / 1e9;
        double noise = g_random_double_range(-d->noise_c_amplitude, d->noise_c_amplitude);
        double updated = (double)d->temperature + d->drift_c_per_sec * dt_s + noise;
        if (updated < d->temp_min_c) updated = d->temp_min_c;
        if (updated > d->temp_max_c) updated = d->temp_max_c;
        d->temperature = (float)updated;
        d->last_conversion_time_ns = now_ns;
    }
}

static bool tmp105_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    TMP105VDev *d = (TMP105VDev*)base;
    if (d->i2c_addr7 != (addr7 & 0x7F)) return false;
    return !d->shutdown_mode && base->responding && base->present;
}

static void tmp105_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    TMP105VDev *d = (TMP105VDev*)base;
    d->read_phase = 0;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static ssize_t tmp105_i2c_write(VDevBase *base, uint8_t addr7, const uint8_t *data, size_t length)
{
    (void)addr7;
    TMP105VDev *d = (TMP105VDev*)base;
    if (!data || length == 0) return 0;
    if (length == 1) {
        d->register_address = data[0];
        d->read_phase = 0;
        return 1;
    }
    if (length == 2 && d->register_address == 0x01) {
        d->config_register = data[0];
        d->shutdown_mode = (data[0] & 0x01) != 0;
        return 2;
    }
    return (ssize_t)length;
}

static ssize_t tmp105_i2c_read(VDevBase *base, uint8_t addr7, uint8_t *out, size_t length)
{
    (void)addr7;
    TMP105VDev *d = (TMP105VDev*)base;
    if (!out || length == 0) return 0;

    if (d->register_address == 0x00) {
        int16_t raw_temp = (int16_t)(d->temperature * 16.0);
        if (raw_temp < 0) raw_temp = raw_temp + 4096;
        uint8_t msb = (raw_temp >> 4) & 0xFF;
        uint8_t lsb = (raw_temp << 4) & 0xF0;
        if (length >= 2) {
            out[0] = msb; out[1] = lsb; d->read_phase = 0; return 2;
        } else {
            if (d->read_phase == 0) { out[0] = msb; d->read_phase = 1; }
            else { out[0] = lsb; d->read_phase = 0; }
            return 1;
        }
    } else if (d->register_address == 0x01) {
        out[0] = d->config_register;
        if (length >= 2) out[1] = 0x00;
        return (length >= 2) ? 2 : 1;
    }
    out[0] = 0x00;
    return 1;
}

static const VDevVTable tmp105_vtable = {
    .init = tmp105_init_v,
    .reset = tmp105_reset_v,
    .destroy = NULL,
    .tick = tmp105_tick_v,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps tmp105_i2c_ops = {
    .i2c_can_ack = tmp105_i2c_can_ack,
    .i2c_on_addressed = tmp105_i2c_on_addressed,
    .i2c_write = tmp105_i2c_write,
    .i2c_read = tmp105_i2c_read,
};

TMP105VDev* tmp105_vdev_create(uint8_t addr7, float initial_temp)
{
    TMP105VDev *d = g_new0(TMP105VDev, 1);
    d->base.name = "TMP105";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.debug_enabled = true;
    d->base.vtable = &tmp105_vtable;
    d->i2c_addr7 = addr7 & 0x7F;
    d->temperature = initial_temp;
    d->config_register = 0x00;
    d->resolution = 12;
    d->register_address = 0x00;
    d->shutdown_mode = false;
    d->conversion_time_us = 100;
    d->read_phase = 0;
    d->drift_c_per_sec = 0.0;
    d->noise_c_amplitude = 0.25;
    d->temp_min_c = -40.0f;
    d->temp_max_c = 125.0f;
    d->last_conversion_time_ns = 0;
    return d;
}

const VDevI2COps* tmp105_vdev_get_i2c_ops(void)
{
    return &tmp105_i2c_ops;
}


