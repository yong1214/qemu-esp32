#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/adxl345_vdev.h"

static bool adxl345_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void adxl345_reset_v(VDevBase *base)
{
    ADXL345VDev *d = (ADXL345VDev*)base;
    d->register_address = 0x00;
    d->read_phase = 0;
    d->power_ctl = 0x00;
    d->data_format = 0x00;
    d->last_tick_ns = 0;
}

static void adxl345_tick_v(VDevBase *base, uint64_t now_ns)
{
    ADXL345VDev *d = (ADXL345VDev*)base;
    /* Only update if measurement mode is active (bit 3 of power_ctl) */
    if (!(d->power_ctl & 0x08)) return;
    if (d->last_tick_ns == 0) {
        d->last_tick_ns = now_ns;
        return;
    }
    uint64_t elapsed = now_ns - d->last_tick_ns;
    if (elapsed >= 100000ULL) { /* 100us period */
        double noise_x = g_random_double_range(-d->noise_accel_amplitude,
                                                d->noise_accel_amplitude);
        double noise_y = g_random_double_range(-d->noise_accel_amplitude,
                                                d->noise_accel_amplitude);
        double noise_z = g_random_double_range(-d->noise_accel_amplitude,
                                                d->noise_accel_amplitude);
        d->accel_x = (float)((double)d->accel_x + noise_x);
        d->accel_y = (float)((double)d->accel_y + noise_y);
        d->accel_z = (float)((double)d->accel_z + noise_z);
        d->last_tick_ns = now_ns;
    }
}

static bool adxl345_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    ADXL345VDev *d = (ADXL345VDev*)base;
    if (d->i2c_addr7 != (addr7 & 0x7F)) return false;
    return base->responding && base->present;
}

static void adxl345_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    ADXL345VDev *d = (ADXL345VDev*)base;
    d->read_phase = 0;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static ssize_t adxl345_i2c_write(VDevBase *base, uint8_t addr7,
                                   const uint8_t *data, size_t length)
{
    (void)addr7;
    ADXL345VDev *d = (ADXL345VDev*)base;
    if (!data || length == 0) return 0;

    if (length == 1) {
        d->register_address = data[0];
        d->read_phase = 0;
        return 1;
    }

    /* Register write: first byte is register, second is value */
    d->register_address = data[0];
    if (d->register_address == 0x2D) {
        d->power_ctl = data[1];
    } else if (d->register_address == 0x31) {
        d->data_format = data[1];
    }
    return (ssize_t)length;
}

static ssize_t adxl345_i2c_read(VDevBase *base, uint8_t addr7,
                                  uint8_t *out, size_t length)
{
    (void)addr7;
    ADXL345VDev *d = (ADXL345VDev*)base;
    if (!out || length == 0) return 0;

    size_t i;
    for (i = 0; i < length; i++) {
        uint8_t reg = d->register_address + d->read_phase;
        uint8_t val = 0x00;

        if (reg == 0x00) {
            /* Device ID */
            val = 0xE5;
        } else if (reg == 0x2D) {
            val = d->power_ctl;
        } else if (reg == 0x31) {
            val = d->data_format;
        } else if (reg >= 0x32 && reg <= 0x37) {
            /* Data registers: X(LSB,MSB), Y(LSB,MSB), Z(LSB,MSB) */
            int16_t raw_x = (int16_t)(d->accel_x / 0.0039f);
            int16_t raw_y = (int16_t)(d->accel_y / 0.0039f);
            int16_t raw_z = (int16_t)(d->accel_z / 0.0039f);
            switch (reg) {
            case 0x32: val = (uint8_t)(raw_x & 0xFF); break;
            case 0x33: val = (uint8_t)((raw_x >> 8) & 0xFF); break;
            case 0x34: val = (uint8_t)(raw_y & 0xFF); break;
            case 0x35: val = (uint8_t)((raw_y >> 8) & 0xFF); break;
            case 0x36: val = (uint8_t)(raw_z & 0xFF); break;
            case 0x37: val = (uint8_t)((raw_z >> 8) & 0xFF); break;
            }
        }

        out[i] = val;
        d->read_phase++;
    }
    return (ssize_t)length;
}

static const VDevVTable adxl345_vtable = {
    .init = adxl345_init_v,
    .reset = adxl345_reset_v,
    .destroy = NULL,
    .tick = adxl345_tick_v,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps adxl345_i2c_ops = {
    .i2c_can_ack = adxl345_i2c_can_ack,
    .i2c_on_addressed = adxl345_i2c_on_addressed,
    .i2c_write = adxl345_i2c_write,
    .i2c_read = adxl345_i2c_read,
};

ADXL345VDev* adxl345_vdev_create(uint8_t addr7, float initial_x,
                                   float initial_y, float initial_z)
{
    ADXL345VDev *d = g_new0(ADXL345VDev, 1);
    d->base.name = "ADXL345";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.debug_enabled = true;
    d->base.vtable = &adxl345_vtable;
    d->i2c_addr7 = addr7 & 0x7F;
    d->register_address = 0x00;
    d->read_phase = 0;
    d->power_ctl = 0x00;
    d->data_format = 0x00;
    d->accel_x = initial_x;
    d->accel_y = initial_y;
    d->accel_z = initial_z;
    d->noise_accel_amplitude = 0.01;
    d->last_tick_ns = 0;
    return d;
}

const VDevI2COps* adxl345_vdev_get_i2c_ops(void)
{
    return &adxl345_i2c_ops;
}
