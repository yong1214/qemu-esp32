#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "hw/ssi/spi_eeprom.h"

/* Commands */
#define CMD_WREN  0x06
#define CMD_WRDI  0x04
#define CMD_RDSR  0x05
#define CMD_WRSR  0x01
#define CMD_READ  0x03
#define CMD_PP    0x02

static uint32_t ssi_eeprom_transfer(SSIPeripheral *dev, uint32_t val)
{
    SSIEepromState *s = SSI_EEPROM(dev);
    uint8_t in = (uint8_t)val;
    uint8_t out = 0xFF;
    if (!s || !s->storage) {
        qemu_log("spi_eeprom_transfer: invalid state (s=%p, storage=%p)\n", (void*)s, s ? (void*)s->storage : NULL);
        return out;
    }
    if (!s->cs_active) {
        return out;
    }
    switch (s->phase) {
    case 0: /* command */
        s->cmd = in;
        s->addr = 0;
        if (s->cmd == CMD_RDSR) {
            s->phase = 4; /* data phase */
        } else if (s->cmd == CMD_WRSR) {
            s->phase = 4; /* data */
        } else if (s->cmd == CMD_WREN || s->cmd == CMD_WRDI) {
            if (s->cmd == CMD_WREN) s->write_enable = true; /* be lenient without CS */
            if (s->cmd == CMD_WRDI) s->write_enable = false;
            s->phase = 0; /* no data; complete on CS */
        } else {
            s->phase = 1; /* address bytes */
        }
        break;
    case 1: /* addr byte 2 */
        s->addr = ((uint32_t)in) << 16;
        s->phase = 2;
        break;
    case 2: /* addr byte 1 */
        s->addr |= ((uint32_t)in) << 8;
        s->phase = 3;
        break;
    case 3: /* addr byte 0 */
        s->addr |= in;
        s->addr %= s->size;
        s->phase = 4;
        break;
    case 4: /* data */
        if (s->cmd == CMD_READ) {
            out = s->storage[s->addr % s->size];
            s->addr = (s->addr + 1) % s->size;
        } else if (s->cmd == CMD_PP) {
            if (s->write_enable) {
                s->storage[s->addr % s->size] = in;
                s->addr = (s->addr + 1) % s->size;
            }
        } else if (s->cmd == CMD_RDSR) {
            out = s->status | (s->write_enable ? 0x02 : 0x00);
        } else if (s->cmd == CMD_WRSR) {
            s->status = in;
        }
        break;
    }
    return out;
}

static int ssi_eeprom_set_cs(SSIPeripheral *dev, bool select)
{
    SSIEepromState *s = SSI_EEPROM(dev);
    bool active = !select; /* CS low is active */
    if (active == s->cs_active) {
        return 0; /* edge only */
    }
    s->cs_active = active;
    if (active) {
        /* Assert: start new command */
        s->phase = 0;
    } else {
        /* Deassert: finalize */
        if (s->cmd == CMD_WREN) {
            s->write_enable = true;
        } else if (s->cmd == CMD_WRDI) {
            s->write_enable = false;
        }
        s->cmd = 0;
    }
    return 0;
}

static void ssi_eeprom_realize(SSIPeripheral *dev, Error **errp)
{
    SSIEepromState *s = SSI_EEPROM(dev);
    if (!s->size) s->size = 32768; /* 32KB default */
    s->storage = g_malloc(s->size);
    memset(s->storage, 0xFF, s->size);
    qemu_log("esp.spi_eeprom: realized, size=%u bytes\n", s->size);
    s->cs_active = false;
    s->phase = 0;
    s->cmd = 0;
}

static Property ssi_eeprom_props[] = {
    DEFINE_PROP_UINT32("size", SSIEepromState, size, 32768),
    DEFINE_PROP_END_OF_LIST(),
};

static void ssi_eeprom_init(Object *obj)
{
    /* instance_init: no class setup here */
}

static void ssi_eeprom_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *spc = SSI_PERIPHERAL_CLASS(klass);
    
    spc->realize = ssi_eeprom_realize;
    spc->transfer = ssi_eeprom_transfer;
    spc->set_cs = ssi_eeprom_set_cs;
    spc->cs_polarity = SSI_CS_LOW;
    
    device_class_set_props(dc, ssi_eeprom_props);
}

static const TypeInfo ssi_eeprom_info = {
    .name = TYPE_SSI_EEPROM,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(SSIEepromState),
    .instance_init = ssi_eeprom_init,
    .class_init = ssi_eeprom_class_init,
};

static void ssi_eeprom_register_types(void)
{
    type_register_static(&ssi_eeprom_info);
}

type_init(ssi_eeprom_register_types)


