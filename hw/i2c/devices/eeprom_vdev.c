#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/i2c/devices/eeprom_vdev.h"
#include "qemu/timer.h"

static bool eeprom_init_v(VDevBase *base)
{
    (void)base;
    return true;
}

static void eeprom_reset_v(VDevBase *base)
{
    EEPROMVDev *d = (EEPROMVDev*)base;
    d->current_address = 0;
    d->write_in_progress = false;
}

static bool eeprom_i2c_can_ack(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)is_read;
    EEPROMVDev *d = (EEPROMVDev*)base;
    if (d->write_in_progress) {
        uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if (now >= d->write_complete_time_ns) {
            d->write_in_progress = false;
        }
    }
    return base->present && base->responding && (d->i2c_addr7 == (addr7 & 0x7F)) && !d->write_in_progress;
}

static void eeprom_i2c_on_addressed(VDevBase *base, uint8_t addr7, bool is_read)
{
    (void)addr7; (void)is_read;
    base->access_count++;
    base->last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static ssize_t eeprom_i2c_write(VDevBase *base, uint8_t addr7, const uint8_t *data, size_t length)
{
    (void)addr7;
    EEPROMVDev *d = (EEPROMVDev*)base;
    if (!data || length == 0) return 0;

    /* First 1-2 bytes can be address depending on address_bytes */
    size_t idx = 0;
    if (d->address_bytes == 2 && length >= 2) {
        d->current_address = ((uint16_t)data[0] << 8) | data[1];
        idx = 2;
    } else if (d->address_bytes == 1) {
        d->current_address = data[0];
        idx = 1;
    }

    /* Page-write with wrap within page */
    if (!d->write_protect && !d->wp_enabled) {
        uint32_t base_addr = d->current_address & ~(uint32_t)(d->page_size - 1);
        uint32_t offset = d->current_address & (d->page_size - 1);
        size_t written = 0;
        for (; idx < length; ++idx) {
            uint32_t page_off = (offset + written) % d->page_size;
            uint32_t eff_addr = base_addr + page_off;
            if (eff_addr >= d->size_bytes) break;
            d->memory[eff_addr] = data[idx];
            written++;
        }
        d->current_address = (uint16_t)(base_addr + ((offset + written) % d->page_size));
        /* Begin write cycle: NACK until complete */
        d->write_in_progress = true;
        d->write_complete_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (uint64_t)d->write_delay_ms * 1000000ULL;
    }
    return (ssize_t)length;
}

static ssize_t eeprom_i2c_read(VDevBase *base, uint8_t addr7, uint8_t *out, size_t length)
{
    (void)addr7;
    EEPROMVDev *d = (EEPROMVDev*)base;
    if (!out || length == 0) return 0;
    size_t n = 0;
    for (; n < length && d->current_address < d->size_bytes; ++n) {
        out[n] = d->memory[d->current_address++];
    }
    return (ssize_t)n;
}

static const VDevVTable eeprom_vtable = {
    .init = eeprom_init_v,
    .reset = eeprom_reset_v,
    .destroy = NULL,
    .tick = NULL,
    .on_event = NULL,
    .ioctl = NULL,
};

static const VDevI2COps eeprom_i2c_ops = {
    .i2c_can_ack = eeprom_i2c_can_ack,
    .i2c_on_addressed = eeprom_i2c_on_addressed,
    .i2c_write = eeprom_i2c_write,
    .i2c_read = eeprom_i2c_read,
};

EEPROMVDev* eeprom_vdev_create(uint8_t addr7, size_t memory_size)
{
    EEPROMVDev *d = g_new0(EEPROMVDev, 1);
    d->base.name = "EEPROM";
    d->base.bus_type = VDEV_BUS_I2C;
    d->base.present = true;
    d->base.responding = true;
    d->base.debug_enabled = true;
    d->base.vtable = &eeprom_vtable;
    d->i2c_addr7 = addr7 & 0x7F;
    memset(d->memory, 0xFF, sizeof(d->memory));
    d->current_address = 0;
    d->write_protect = 0;
    d->page_size = 64;
    d->address_bytes = 2;
    d->write_delay_ms = 5;
    d->write_in_progress = false;
    d->write_complete_time_ns = 0;
    d->size_bytes = (uint16_t)((memory_size > 0 && memory_size <= EEPROM_VDEV_MAX_BYTES) ? memory_size : EEPROM_VDEV_MAX_BYTES);
    d->wp_enabled = false;
    return d;
}

const VDevI2COps* eeprom_vdev_get_i2c_ops(void)
{
    return &eeprom_i2c_ops;
}


