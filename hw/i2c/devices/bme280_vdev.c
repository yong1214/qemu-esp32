#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/bme280_vdev.h"

static bool bme280_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void bme280_reset_v(VDevBase *base)
{
    BME280VDev *d = (BME280VDev*)base;
    d->register_address = 0x00;
    d->read_phase = 0;
    d->ctrl_meas = 0x00;
    d->config_reg = 0x00;
    d->last_tick_ns = 0;
}

static void bme280_tick_v(VDevBase *base, uint64_t now_ns)
{
    BME280VDev *d = (BME280VDev*)base;
    if (d->last_tick_ns == 0) {
        d->last_tick_ns = now_ns;
        return;
    }
    uint64_t elapsed = now_ns - d->last_tick_ns;
    if (elapsed >= 100000ULL) { /* 100us period */
        double noise_t = g_random_double_range(-d->noise_temp_amplitude,
                                                d->noise_temp_amplitude);
        double noise_p = g_random_double_range(-d->noise_press_amplitude,
                                                d->noise_press_amplitude);
        double noise_h = g_random_double_range(-d->noise_hum_amplitude,
                                                d->noise_hum_amplitude);
        d->temperature = (float)((double)d->temperature + noise_t);
        d->pressure = (float)((double)d->pressure + noise_p);
        d->humidity = (float)((double)d->humidity + noise_h);
        if (d->humidity < 0.0f) d->humidity = 0.0f;
        if (d->humidity > 100.0f) d->humidity = 100.0f;
        d->last_tick_ns = now_ns;
    }
}

static bool bme280_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    BME280VDev *d = (BME280VDev*)base;
    if (d->i2c_addr7 != (addr7 & 0x7F)) return false;
    return base->responding && base->present;
}

static void bme280_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    BME280VDev *d = (BME280VDev*)base;
    d->read_phase = 0;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static ssize_t bme280_i2c_write(VDevBase *base, uint8_t addr7,
                                 const uint8_t *data, size_t length)
{
    (void)addr7;
    BME280VDev *d = (BME280VDev*)base;
    if (!data || length == 0) return 0;

    if (length == 1) {
        d->register_address = data[0];
        d->read_phase = 0;
        return 1;
    }

    /* Register write: first byte is register, second is value */
    d->register_address = data[0];
    if (d->register_address == 0xF4) {
        d->ctrl_meas = data[1];
    } else if (d->register_address == 0xF5) {
        d->config_reg = data[1];
    }
    return (ssize_t)length;
}

static ssize_t bme280_i2c_read(VDevBase *base, uint8_t addr7,
                                uint8_t *out, size_t length)
{
    (void)addr7;
    BME280VDev *d = (BME280VDev*)base;
    if (!out || length == 0) return 0;

    size_t i;
    for (i = 0; i < length; i++) {
        uint8_t reg = d->register_address + d->read_phase;
        uint8_t val = 0x00;

        if (reg == 0xD0) {
            /* Chip ID */
            val = 0x60;
        } else if (reg == 0xF4) {
            val = d->ctrl_meas;
        } else if (reg == 0xF5) {
            val = d->config_reg;
        } else if (reg >= 0xF7 && reg <= 0xF9) {
            /* Pressure: 20-bit, registers F7(msb) F8(lsb) F9(xlsb) */
            int32_t raw = (int32_t)(d->pressure / 100.0f * 256.0f);
            if (reg == 0xF7) val = (raw >> 12) & 0xFF;
            else if (reg == 0xF8) val = (raw >> 4) & 0xFF;
            else val = (raw << 4) & 0xF0;
        } else if (reg >= 0xFA && reg <= 0xFC) {
            /* Temperature: 20-bit, registers FA(msb) FB(lsb) FC(xlsb) */
            int32_t raw = (int32_t)(d->temperature * 5120.0f);
            if (reg == 0xFA) val = (raw >> 12) & 0xFF;
            else if (reg == 0xFB) val = (raw >> 4) & 0xFF;
            else val = (raw << 4) & 0xF0;
        } else if (reg >= 0xFD && reg <= 0xFE) {
            /* Humidity: 16-bit, registers FD(msb) FE(lsb) */
            uint16_t raw = (uint16_t)(d->humidity * 1024.0f);
            if (reg == 0xFD) val = (raw >> 8) & 0xFF;
            else val = raw & 0xFF;
        }

        out[i] = val;
        d->read_phase++;
    }
    return (ssize_t)length;
}

static const VDevVTable bme280_vtable = {
    .init = bme280_init_v,
    .reset = bme280_reset_v,
    .destroy = NULL,
    .tick = bme280_tick_v,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps bme280_i2c_ops = {
    .i2c_can_ack = bme280_i2c_can_ack,
    .i2c_on_addressed = bme280_i2c_on_addressed,
    .i2c_write = bme280_i2c_write,
    .i2c_read = bme280_i2c_read,
};

BME280VDev* bme280_vdev_create(uint8_t addr7, float initial_temp,
                                float initial_pressure, float initial_humidity)
{
    BME280VDev *d = g_new0(BME280VDev, 1);
    d->base.name = "BME280";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.debug_enabled = true;
    d->base.vtable = &bme280_vtable;
    d->i2c_addr7 = addr7 & 0x7F;
    d->temperature = initial_temp;
    d->pressure = initial_pressure;
    d->humidity = initial_humidity;
    d->ctrl_meas = 0x00;
    d->config_reg = 0x00;
    d->register_address = 0x00;
    d->read_phase = 0;
    d->noise_temp_amplitude = 0.1;
    d->noise_press_amplitude = 10.0;
    d->noise_hum_amplitude = 0.5;
    d->last_tick_ns = 0;
    return d;
}

const VDevI2COps* bme280_vdev_get_i2c_ops(void)
{
    return &bme280_i2c_ops;
}
