#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/aht20_vdev.h"

static bool aht20_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void aht20_reset_v(VDevBase *base)
{
    AHT20VDev *d = (AHT20VDev*)base;
    d->calibrated = true;
    d->measuring = false;
    d->write_pos = 0;
    d->last_tick_ns = 0;
}

static void aht20_tick_v(VDevBase *base, uint64_t now_ns)
{
    AHT20VDev *d = (AHT20VDev*)base;
    if (d->last_tick_ns == 0) {
        d->last_tick_ns = now_ns;
        return;
    }
    uint64_t elapsed = now_ns - d->last_tick_ns;
    if (elapsed >= 100000ULL) { /* 100us period */
        /* Clear measuring flag after simulated measurement time */
        if (d->measuring) {
            d->measuring = false;
        }
        double noise_t = g_random_double_range(-d->noise_temp_amplitude,
                                                d->noise_temp_amplitude);
        double noise_h = g_random_double_range(-d->noise_hum_amplitude,
                                                d->noise_hum_amplitude);
        d->temperature = (float)((double)d->temperature + noise_t);
        double hum = (double)d->humidity + noise_h;
        if (hum < 0.0) hum = 0.0;
        if (hum > 100.0) hum = 100.0;
        d->humidity = (float)hum;
        d->last_tick_ns = now_ns;
    }
}

static bool aht20_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    AHT20VDev *d = (AHT20VDev*)base;
    if (d->i2c_addr7 != (addr7 & 0x7F)) return false;
    return base->responding && base->present;
}

static void aht20_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    AHT20VDev *d = (AHT20VDev*)base;
    d->write_pos = 0;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static ssize_t aht20_i2c_write(VDevBase *base, uint8_t addr7,
                                 const uint8_t *data, size_t length)
{
    (void)addr7;
    AHT20VDev *d = (AHT20VDev*)base;
    if (!data || length == 0) return 0;

    size_t i;
    for (i = 0; i < length; i++) {
        if (d->write_pos < 3) {
            d->write_buf[d->write_pos] = data[i];
            d->write_pos++;
        }
    }

    /* Check for trigger measurement command: [0xAC, 0x33, 0x00] */
    if (d->write_pos >= 3 && d->write_buf[0] == 0xAC &&
        d->write_buf[1] == 0x33 && d->write_buf[2] == 0x00) {
        d->measuring = true;
    }
    /* Check for initialize/calibrate command: [0xBE, 0x08, 0x00] */
    if (d->write_pos >= 3 && d->write_buf[0] == 0xBE &&
        d->write_buf[1] == 0x08 && d->write_buf[2] == 0x00) {
        d->calibrated = true;
    }

    return (ssize_t)length;
}

static ssize_t aht20_i2c_read(VDevBase *base, uint8_t addr7,
                                uint8_t *out, size_t length)
{
    (void)addr7;
    AHT20VDev *d = (AHT20VDev*)base;
    if (!out || length == 0) return 0;

    /* Build 6-byte response */
    uint8_t status = 0x00;
    if (d->calibrated) status |= 0x08;
    if (d->measuring) status |= 0x80;

    uint32_t hum_raw = (uint32_t)(d->humidity / 100.0f * 1048576.0f);
    uint32_t temp_raw = (uint32_t)((d->temperature + 50.0f) / 200.0f * 1048576.0f);

    uint8_t response[6];
    response[0] = status;
    response[1] = (hum_raw >> 12) & 0xFF;           /* h19:12 */
    response[2] = (hum_raw >> 4) & 0xFF;            /* h11:4 */
    response[3] = ((hum_raw & 0x0F) << 4) |         /* h3:0 | t19:16 */
                  ((temp_raw >> 16) & 0x0F);
    response[4] = (temp_raw >> 8) & 0xFF;            /* t15:8 */
    response[5] = temp_raw & 0xFF;                    /* t7:0 */

    size_t to_copy = (length < 6) ? length : 6;
    size_t i;
    for (i = 0; i < to_copy; i++) {
        out[i] = response[i];
    }
    return (ssize_t)to_copy;
}

static const VDevVTable aht20_vtable = {
    .init = aht20_init_v,
    .reset = aht20_reset_v,
    .destroy = NULL,
    .tick = aht20_tick_v,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps aht20_i2c_ops = {
    .i2c_can_ack = aht20_i2c_can_ack,
    .i2c_on_addressed = aht20_i2c_on_addressed,
    .i2c_write = aht20_i2c_write,
    .i2c_read = aht20_i2c_read,
};

AHT20VDev* aht20_vdev_create(uint8_t addr7, float initial_temp,
                               float initial_humidity)
{
    AHT20VDev *d = g_new0(AHT20VDev, 1);
    d->base.name = "AHT20";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.debug_enabled = true;
    d->base.vtable = &aht20_vtable;
    d->i2c_addr7 = addr7 & 0x7F;
    d->temperature = initial_temp;
    d->humidity = initial_humidity;
    d->calibrated = true;
    d->measuring = false;
    d->write_pos = 0;
    d->noise_temp_amplitude = 0.1;
    d->noise_hum_amplitude = 0.5;
    d->last_tick_ns = 0;
    return d;
}

const VDevI2COps* aht20_vdev_get_i2c_ops(void)
{
    return &aht20_i2c_ops;
}
