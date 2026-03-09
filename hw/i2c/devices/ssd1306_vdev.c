/*
 * SSD1306 OLED Virtual Device for QEMU ESP32
 *
 * Handles I2C ACKs and command/data parsing so the firmware's I2C
 * operations complete correctly.  The actual OLED rendering happens
 * in the Node.js backend SSD1306Decoder, which receives raw I2C
 * transaction bytes via the unified serial-monitor pipe.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/ssd1306_vdev.h"

/* ─── VDev lifecycle ────────────────────────────────────────────── */

static bool ssd1306_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void ssd1306_reset_v(VDevBase *base)
{
    SSD1306VDev *d = (SSD1306VDev *)base;
    memset(d->display_buffer, 0x00, sizeof(d->display_buffer));
    d->current_column = 0;
    d->current_page = 0;
    d->display_mode = 0;
    d->brightness = 0xFF;
    d->display_on = false;
    d->cmd_pending = 0;
    d->cmd_args_remaining = 0;
    d->txn_first_byte = true;
    d->txn_data_mode = false;
}

static void ssd1306_destroy_v(VDevBase *base)
{
    (void)base;
}

/* ─── I2C callbacks ─────────────────────────────────────────────── */

static bool ssd1306_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    SSD1306VDev *d = (SSD1306VDev *)base;
    return base->present && base->responding &&
           (d->i2c_addr7 == (addr7 & 0x7F));
}

static void ssd1306_i2c_on_addressed(VDevBase *base, uint8_t addr7,
                                     bool is_read)
{
    (void)addr7; (void)is_read;
    SSD1306VDev *d = (SSD1306VDev *)base;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    d->txn_first_byte = true;
}

/* ─── Command handling (keeps internal state consistent) ────────── */

static void ssd1306_handle_command(SSD1306VDev *d, uint8_t cmd)
{
    if (d->cmd_args_remaining > 0) {
        d->cmd_args_remaining--;
        switch (d->cmd_pending) {
        case 0x20: d->display_mode = cmd & 0x03; break;
        case 0x81: d->brightness = cmd; break;
        default: break;
        }
        if (d->cmd_args_remaining == 0) d->cmd_pending = 0;
        return;
    }

    if ((cmd & 0xF8) == 0xB0) { d->current_page = cmd & 0x07; return; }
    if (cmd <= 0x0F) { d->current_column = (d->current_column & 0xF0) | (cmd & 0x0F); return; }
    if ((cmd & 0xF0) == 0x10) { d->current_column = (d->current_column & 0x0F) | ((cmd & 0x0F) << 4); return; }
    if ((cmd & 0xC0) == 0x40) { return; }

    switch (cmd) {
    case 0xAE: d->display_on = false; break;
    case 0xAF: d->display_on = true; break;
    case 0xA0: case 0xA1: case 0xA4: case 0xA5:
    case 0xA6: case 0xA7: case 0xC0: case 0xC8: case 0xE3:
        break;
    case 0xD5: case 0xA8: case 0xD3: case 0x8D:
    case 0x20: case 0xDA: case 0x81: case 0xD9: case 0xDB:
        d->cmd_pending = cmd;
        d->cmd_args_remaining = 1;
        break;
    case 0x21: case 0x22:
        d->cmd_pending = cmd;
        d->cmd_args_remaining = 2;
        break;
    default: break;
    }
}

static void ssd1306_write_data(SSD1306VDev *d, uint8_t byte)
{
    size_t idx = (size_t)d->current_page * 128 + d->current_column;
    if (idx < sizeof(d->display_buffer)) {
        d->display_buffer[idx] = byte;
    }
    d->current_column++;
    if (d->current_column > 127) {
        d->current_column = 0;
        if (d->display_mode == 0) {
            d->current_page = (d->current_page + 1) & 0x07;
        }
    }
}

/* ─── I2C write ─────────────────────────────────────────────────── */

static ssize_t ssd1306_i2c_write(VDevBase *base, uint8_t addr7,
                                 const uint8_t *data, size_t length)
{
    (void)addr7;
    SSD1306VDev *d = (SSD1306VDev *)base;

    if (!data || length == 0) return 0;

    for (size_t i = 0; i < length; i++) {
        if (d->txn_first_byte) {
            d->txn_data_mode = (data[i] & 0x40) != 0;
            d->txn_first_byte = false;
            continue;
        }
        if (d->txn_data_mode) {
            ssd1306_write_data(d, data[i]);
        } else {
            ssd1306_handle_command(d, data[i]);
        }
    }

    return (ssize_t)length;
}

static ssize_t ssd1306_i2c_read(VDevBase *base, uint8_t addr7,
                                uint8_t *out, size_t length)
{
    (void)base; (void)addr7;
    if (!out || length == 0) return 0;
    memset(out, 0x00, length);
    return (ssize_t)length;
}

/* ─── VTable / ops / factory ────────────────────────────────────── */

static const VDevVTable ssd1306_vtable = {
    .init    = ssd1306_init_v,
    .reset   = ssd1306_reset_v,
    .destroy = ssd1306_destroy_v,
    .tick    = NULL,
    .on_event = NULL,
    .ioctl   = NULL,
};

static const VDevI2COps ssd1306_i2c_ops = {
    .i2c_can_ack      = ssd1306_i2c_can_ack,
    .i2c_on_addressed = ssd1306_i2c_on_addressed,
    .i2c_write        = ssd1306_i2c_write,
    .i2c_read         = ssd1306_i2c_read,
};

SSD1306VDev* ssd1306_vdev_create(uint8_t addr7)
{
    SSD1306VDev *d = g_new0(SSD1306VDev, 1);
    d->base.name = "SSD1306";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.debug_enabled = true;
    d->base.vtable = &ssd1306_vtable;
    d->i2c_addr7 = addr7 & 0x7F;

    ssd1306_reset_v(&d->base);
    return d;
}

const VDevI2COps* ssd1306_vdev_get_i2c_ops(void)
{
    return &ssd1306_i2c_ops;
}
