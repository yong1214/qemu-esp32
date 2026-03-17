#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/ina219_vdev.h"

static bool ina219_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void ina219_reset_v(VDevBase *base)
{
    INA219VDev *d = (INA219VDev*)base;
    d->register_address = 0x00;
    d->read_phase = 0;
    d->config = 0x399F;
    d->calibration = 0x0000;
    d->write_pos = 0;
    d->last_tick_ns = 0;
}

static void ina219_tick_v(VDevBase *base, uint64_t now_ns)
{
    INA219VDev *d = (INA219VDev*)base;
    if (d->last_tick_ns == 0) {
        d->last_tick_ns = now_ns;
        return;
    }
    uint64_t elapsed = now_ns - d->last_tick_ns;
    if (elapsed >= 100000ULL) { /* 100us period */
        double noise_bus = g_random_double_range(-d->noise_bus_amplitude,
                                                  d->noise_bus_amplitude);
        double noise_shunt = g_random_double_range(-d->noise_shunt_amplitude,
                                                    d->noise_shunt_amplitude);
        d->bus_voltage = (float)((double)d->bus_voltage + noise_bus);
        d->shunt_voltage = (float)((double)d->shunt_voltage + noise_shunt);
        if (d->bus_voltage < 0.0f) d->bus_voltage = 0.0f;
        /* Derive current and power from voltages */
        /* Assuming 0.1 ohm shunt: I = Vshunt / Rshunt */
        d->current = d->shunt_voltage / 0.1f;
        d->power = d->bus_voltage * d->current;
        if (d->power < 0.0f) d->power = 0.0f;
        d->last_tick_ns = now_ns;
    }
}

static bool ina219_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    INA219VDev *d = (INA219VDev*)base;
    if (d->i2c_addr7 != (addr7 & 0x7F)) return false;
    return base->responding && base->present;
}

static void ina219_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    INA219VDev *d = (INA219VDev*)base;
    d->read_phase = 0;
    d->write_pos = 0;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static ssize_t ina219_i2c_write(VDevBase *base, uint8_t addr7,
                                  const uint8_t *data, size_t length)
{
    (void)addr7;
    INA219VDev *d = (INA219VDev*)base;
    if (!data || length == 0) return 0;

    if (length == 1) {
        /* Register pointer set */
        d->register_address = data[0];
        d->read_phase = 0;
        d->write_pos = 0;
        return 1;
    }

    /* Register write: first byte is register, next 2 are MSB/LSB value */
    d->register_address = data[0];
    if (length >= 3) {
        uint16_t val = ((uint16_t)data[1] << 8) | data[2];
        if (d->register_address == 0x00) {
            d->config = val;
            /* Check for reset bit */
            if (val & 0x8000) {
                d->config = 0x399F;
            }
        } else if (d->register_address == 0x05) {
            d->calibration = val;
        }
    }
    return (ssize_t)length;
}

static ssize_t ina219_i2c_read(VDevBase *base, uint8_t addr7,
                                 uint8_t *out, size_t length)
{
    (void)addr7;
    INA219VDev *d = (INA219VDev*)base;
    if (!out || length == 0) return 0;

    uint16_t reg_val = 0x0000;

    switch (d->register_address) {
    case 0x00: /* Configuration */
        reg_val = d->config;
        break;
    case 0x01: /* Shunt Voltage */
        reg_val = (uint16_t)(int16_t)(d->shunt_voltage / 0.00001f);
        break;
    case 0x02: /* Bus Voltage */
        reg_val = ((uint16_t)(d->bus_voltage / 0.004f)) << 3;
        /* Set CNVR (conversion ready) bit */
        reg_val |= 0x02;
        break;
    case 0x03: /* Power */
        reg_val = (uint16_t)(d->power / 0.02f);
        break;
    case 0x04: /* Current */
        reg_val = (uint16_t)(int16_t)(d->current / 0.001f);
        break;
    case 0x05: /* Calibration */
        reg_val = d->calibration;
        break;
    default:
        reg_val = 0x0000;
        break;
    }

    /* All registers are 16-bit, MSB first */
    if (length >= 2) {
        if (d->read_phase == 0) {
            out[0] = (reg_val >> 8) & 0xFF;
            out[1] = reg_val & 0xFF;
            d->read_phase = 0;
            return 2;
        }
    }
    if (d->read_phase == 0) {
        out[0] = (reg_val >> 8) & 0xFF;
        d->read_phase = 1;
    } else {
        out[0] = reg_val & 0xFF;
        d->read_phase = 0;
    }
    return 1;
}

static const VDevVTable ina219_vtable = {
    .init = ina219_init_v,
    .reset = ina219_reset_v,
    .destroy = NULL,
    .tick = ina219_tick_v,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps ina219_i2c_ops = {
    .i2c_can_ack = ina219_i2c_can_ack,
    .i2c_on_addressed = ina219_i2c_on_addressed,
    .i2c_write = ina219_i2c_write,
    .i2c_read = ina219_i2c_read,
};

INA219VDev* ina219_vdev_create(uint8_t addr7, float initial_bus_v,
                                 float initial_shunt_v)
{
    INA219VDev *d = g_new0(INA219VDev, 1);
    d->base.name = "INA219";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.debug_enabled = true;
    d->base.vtable = &ina219_vtable;
    d->i2c_addr7 = addr7 & 0x7F;
    d->register_address = 0x00;
    d->read_phase = 0;
    d->config = 0x399F;
    d->bus_voltage = initial_bus_v;
    d->shunt_voltage = initial_shunt_v;
    d->current = initial_shunt_v / 0.1f;   /* Assuming 0.1 ohm shunt */
    d->power = initial_bus_v * d->current;
    d->calibration = 0x0000;
    d->write_pos = 0;
    d->noise_bus_amplitude = 0.005;
    d->noise_shunt_amplitude = 0.0001;
    d->last_tick_ns = 0;
    return d;
}

const VDevI2COps* ina219_vdev_get_i2c_ops(void)
{
    return &ina219_i2c_ops;
}
