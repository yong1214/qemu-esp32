#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/pca9685_vdev.h"

/* PCA9685 register defaults */
#define PCA9685_MODE1_DEFAULT     0x11  /* SLEEP=1, ALLCALL=1 */
#define PCA9685_MODE2_DEFAULT     0x04  /* OUTDRV=1 */
#define PCA9685_PRE_SCALE_DEFAULT 0x1E  /* ~200Hz at 25MHz internal osc */

/* MODE1 bit definitions */
#define PCA9685_MODE1_AI          0x20  /* Auto-Increment bit (bit 5) */

static void pca9685_set_defaults(PCA9685VDev *d)
{
    memset(d->registers, 0, sizeof(d->registers));
    d->registers[0x00] = PCA9685_MODE1_DEFAULT;   /* MODE1 */
    d->registers[0x01] = PCA9685_MODE2_DEFAULT;   /* MODE2 */
    d->registers[0xFE] = PCA9685_PRE_SCALE_DEFAULT; /* PRE_SCALE */
    d->register_address = 0x00;
    d->read_phase = 0;
}

static bool pca9685_init_v(VDevBase *base)
{
    PCA9685VDev *d = (PCA9685VDev*)base;
    pca9685_set_defaults(d);
    return true;
}

static void pca9685_reset_v(VDevBase *base)
{
    PCA9685VDev *d = (PCA9685VDev*)base;
    pca9685_set_defaults(d);
}

static void pca9685_tick_v(VDevBase *base, uint64_t now_ns)
{
    (void)base;
    (void)now_ns;
    /* PWM driver doesn't need periodic updates */
}

static bool pca9685_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    PCA9685VDev *d = (PCA9685VDev*)base;
    if (d->i2c_addr7 != (addr7 & 0x7F)) return false;
    return base->present && base->responding;
}

static void pca9685_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    PCA9685VDev *d = (PCA9685VDev*)base;
    d->read_phase = 0;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static ssize_t pca9685_i2c_write(VDevBase *base, uint8_t addr7, const uint8_t *data, size_t length)
{
    (void)addr7;
    PCA9685VDev *d = (PCA9685VDev*)base;
    if (!data || length == 0) return 0;

    /* First byte is always the register pointer */
    d->register_address = data[0];
    d->read_phase = 0;

    /* Subsequent bytes write to registers */
    for (size_t i = 1; i < length; i++) {
        d->registers[d->register_address] = data[i];
        /* Auto-increment if MODE1 bit 5 (AI) is set */
        if (d->registers[0x00] & PCA9685_MODE1_AI) {
            d->register_address++;
        }
    }

    return (ssize_t)length;
}

static ssize_t pca9685_i2c_read(VDevBase *base, uint8_t addr7, uint8_t *out, size_t length)
{
    (void)addr7;
    PCA9685VDev *d = (PCA9685VDev*)base;
    if (!out || length == 0) return 0;

    for (size_t i = 0; i < length; i++) {
        out[i] = d->registers[d->register_address];
        /* Auto-increment if MODE1 bit 5 (AI) is set */
        if (d->registers[0x00] & PCA9685_MODE1_AI) {
            d->register_address++;
        }
    }

    return (ssize_t)length;
}

static const VDevVTable pca9685_vtable = {
    .init = pca9685_init_v,
    .reset = pca9685_reset_v,
    .destroy = NULL,
    .tick = pca9685_tick_v,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps pca9685_i2c_ops = {
    .i2c_can_ack = pca9685_i2c_can_ack,
    .i2c_on_addressed = pca9685_i2c_on_addressed,
    .i2c_write = pca9685_i2c_write,
    .i2c_read = pca9685_i2c_read,
};

PCA9685VDev* pca9685_vdev_create(uint8_t addr7)
{
    PCA9685VDev *d = g_new0(PCA9685VDev, 1);
    d->base.name = "PCA9685";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.debug_enabled = true;
    d->base.vtable = &pca9685_vtable;
    d->i2c_addr7 = addr7 & 0x7F;
    pca9685_set_defaults(d);
    return d;
}

const VDevI2COps* pca9685_vdev_get_i2c_ops(void)
{
    return &pca9685_i2c_ops;
}
