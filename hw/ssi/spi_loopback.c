#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/ssi/spi_loopback.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"

static uint32_t ssi_loopback_transfer(SSIPeripheral *dev, uint32_t val)
{
    SSILoopbackState *s = SSI_LOOPBACK(dev);
    uint8_t in = (uint8_t)(val & 0xFF);
    uint8_t out = s->last; /* simple 1-byte delay loopback */
    s->last = in;
    return out;
}


static void ssi_loopback_realize(DeviceState *dev, Error **errp)
{
    /* nothing */
}

static void ssi_loopback_init(Object *obj)
{
    SSIPeripheralClass *spc = SSI_PERIPHERAL_GET_CLASS(obj);
    spc->transfer = ssi_loopback_transfer;
}

static void ssi_loopback_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = ssi_loopback_realize;
}

static const TypeInfo ssi_loopback_info = {
    .name = TYPE_SSI_LOOPBACK,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(SSILoopbackState),
    .instance_init = ssi_loopback_init,
    .class_init = ssi_loopback_class_init,
};

static void ssi_loopback_register_types(void)
{
    type_register_static(&ssi_loopback_info);
}

type_init(ssi_loopback_register_types)


