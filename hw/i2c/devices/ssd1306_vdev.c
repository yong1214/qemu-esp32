#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/devices/ssd1306_vdev.h"
#include <stdio.h>

/* Dump display buffer to stderr for backend parsing */
static void ssd1306_dump_buffer(SSD1306VDev *d)
{
    /* Build complete output string first, then write atomically */
    /* Format: OLED_BUFFER:<2048 hex chars>\n */
    static char output_buf[2060]; /* "OLED_BUFFER:" (12) + 2048 hex + "\n" (1) + null */
    char *p = output_buf;
    
    memcpy(p, "OLED_BUFFER:", 12);
    p += 12;
    
    for (int i = 0; i < 1024; i++) {
        static const char hex[] = "0123456789abcdef";
        *p++ = hex[(d->display_buffer[i] >> 4) & 0xF];
        *p++ = hex[d->display_buffer[i] & 0xF];
    }
    *p++ = '\n';
    *p = '\0';
    
    /* Single write call to avoid interleaving */
    fputs(output_buf, stderr);
    fflush(stderr);
}

static bool ssd1306_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void ssd1306_reset_v(VDevBase *base)
{
    SSD1306VDev *d = (SSD1306VDev*)base;
    memset(d->display_buffer, 0x00, sizeof(d->display_buffer));
    d->current_column = 0;
    d->current_page = 0;
    d->display_mode = 0;
    d->brightness = 0xFF;
    d->display_on = false;
    d->command_mode = true;
}

static bool ssd1306_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    SSD1306VDev *d = (SSD1306VDev*)base;
    return base->present && base->responding && (d->i2c_addr7 == (addr7 & 0x7F));
}

static void ssd1306_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void ssd1306_handle_command(SSD1306VDev *d, uint8_t cmd)
{
    switch (cmd) {
    case 0xAF: 
        d->display_on = true;
        /* Dump buffer when display turns on */
        ssd1306_dump_buffer(d);
        break;
    case 0xAE: d->display_on = false; break; /* Display OFF */
    case 0x00 ... 0x0F: /* Low column address */
        d->current_column = (d->current_column & 0xF0) | (cmd & 0x0F);
        break;
    case 0x10 ... 0x1F: /* High column address */
        d->current_column = (d->current_column & 0x0F) | ((cmd & 0x0F) << 4);
        break;
    case 0xB0 ... 0xB7: /* Page address */
        d->current_page = cmd & 0x07;
        break;
    case 0x21: d->command_mode = true; break; /* Set column address range */
    case 0x22: d->command_mode = true; break; /* Set page address range */
    case 0x40: d->command_mode = false; break; /* Data mode indicator */
    default: break;
    }
}

static ssize_t ssd1306_i2c_write(VDevBase *base, uint8_t addr7, const uint8_t *data, size_t length)
{
    (void)addr7;
    SSD1306VDev *d = (SSD1306VDev*)base;
    if (!data || length == 0) return 0;
    
    bool data_written = false;
    
    /* First byte indicates mode: 0x00 = command, 0x40 = data */
    for (size_t i = 0; i < length; ++i) {
        if (i == 0) {
            /* Control byte: 0x00 = commands follow, 0x40 = data follows */
            if (data[i] == 0x00) {
                d->command_mode = true;
            } else if (data[i] == 0x40) {
                d->command_mode = false;
            }
            continue;
        }
        
        if (d->command_mode) {
            ssd1306_handle_command(d, data[i]);
        } else {
            /* Data mode - write to display buffer */
            size_t idx = d->current_page * 128 + d->current_column;
            if (idx < sizeof(d->display_buffer)) {
                d->display_buffer[idx] = data[i];
                data_written = true;
            }
            if (d->current_column < 127) {
                d->current_column++;
            }
        }
    }
    
    /* Dump buffer after data writes (backend will debounce) */
    if (data_written && d->display_on) {
        ssd1306_dump_buffer(d);
    }
    
    return (ssize_t)length;
}

static ssize_t ssd1306_i2c_read(VDevBase *base, uint8_t addr7, uint8_t *out, size_t length)
{
    (void)base; (void)addr7;
    if (!out || length == 0) return 0;
    memset(out, 0x00, length);
    return (ssize_t)length; /* Typically not used */
}

static const VDevVTable ssd1306_vtable = {
    .init = ssd1306_init_v,
    .reset = ssd1306_reset_v,
    .destroy = NULL,
    .tick = NULL,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps ssd1306_i2c_ops = {
    .i2c_can_ack = ssd1306_i2c_can_ack,
    .i2c_on_addressed = ssd1306_i2c_on_addressed,
    .i2c_write = ssd1306_i2c_write,
    .i2c_read = ssd1306_i2c_read,
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


