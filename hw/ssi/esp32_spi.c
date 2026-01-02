/*
 * ESP32 SPI controller
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32_spi.h"
#include "hw/gpio/esp32_gpio.h"
#include "hw/misc/esp32_flash_enc.h"



enum {
    CMD_RES = 0xab,
    CMD_DP = 0xb9,
    CMD_CE = 0x60,
    CMD_BE = 0xD8,
    CMD_SE = 0x20,
    CMD_PP = 0x02,
    CMD_WRSR = 0x1,
    CMD_RDSR = 0x5,
    CMD_RDID = 0x9f,
    CMD_WRDI = 0x4,
    CMD_WREN = 0x6,
    CMD_READ = 0x03,
};


#define ESP32_SPI_REG_SIZE    0x1000

static void esp32_spi_do_command(Esp32SpiState* state, uint32_t cmd_reg);
static void esp32_spi_update_pin_cache(Esp32SpiState *s);
static void esp32_spi_default_gpio_mapping(Esp32SpiState *s);
static void esp32_spi_gpio_level_cb(void *opaque, int pin, bool high);

static uint64_t esp32_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_SPI_ADDR:
        r = s->addr_reg;
        break;
    case A_SPI_CTRL:
        r = s->ctrl_reg;
        break;
    case A_SPI_STATUS:
        r = s->status_reg;
        break;
    case A_SPI_CTRL1:
        r = s->ctrl1_reg;
        break;
    case A_SPI_CTRL2:
        r = s->ctrl2_reg;
        break;
    case A_SPI_USER:
        r = s->user_reg;
        break;
    case A_SPI_USER1:
        r = s->user1_reg;
        break;
    case A_SPI_USER2:
        r = s->user2_reg;
        break;
    case A_SPI_MOSI_DLEN:
        r = s->mosi_dlen_reg;
        break;
    case A_SPI_MISO_DLEN:
        r = s->miso_dlen_reg;
        break;
    case A_SPI_PIN:
        r = s->pin_reg;
        break;
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        r = s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)];
        if (s->unit_index == 3 && qemu_loglevel_mask(LOG_TRACE)) {
            qemu_log("SPI%u READ W%d -> 0x%08x\n", s->unit_index,
                     (int)((addr - A_SPI_W0) / sizeof(uint32_t)), (unsigned)r);
        }
        break;
    case A_SPI_EXT2:
        r = 0;
        break;
    case A_SPI_SLAVE:
        r = s->slave_reg;
        break;
    }
    return r;
}

static void esp32_spi_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    switch (addr) {
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)] = value;
        break;
    case A_SPI_ADDR:
        s->addr_reg = value;
        break;
    case A_SPI_CTRL:
        s->ctrl_reg = value;
        break;
    case A_SPI_STATUS:
        s->status_reg = value;
        break;
    case A_SPI_CTRL1:
        s->ctrl1_reg = value;
        break;
    case A_SPI_CTRL2:
        s->ctrl2_reg = value;
        break;
    case A_SPI_USER:
        s->user_reg = value;
        break;
    case A_SPI_USER1:
        s->user1_reg = value;
        break;
    case A_SPI_USER2:
        s->user2_reg = value;
        break;
    case A_SPI_MOSI_DLEN:
        s->mosi_dlen_reg = value;
        break;
    case A_SPI_MISO_DLEN:
        s->miso_dlen_reg = value;
        break;
    case A_SPI_PIN:
        s->pin_reg = value;
        esp32_spi_update_pin_cache(s);
        break;
    case A_SPI_SLAVE:
        s->slave_reg = value;
        break;
    case A_SPI_CMD:
        esp32_spi_do_command(s, value);
        break;
    }
}

typedef struct Esp32SpiTransaction {
    int cmd_bytes;
    uint32_t cmd;
    int addr_bytes;
    uint32_t addr;
    int data_tx_bytes;
    int data_rx_bytes;
    uint32_t* data;
} Esp32SpiTransaction;

static void esp32_spi_txrx_buffer(Esp32SpiState *s, void *buf, int tx_bytes, int rx_bytes)
{
    int bytes = MAX(tx_bytes, rx_bytes);
    if (bytes <= 0 || buf == NULL) {
        return;
    }
    uint8_t *c_buf = (uint8_t*) buf;
    bool using_data_regs = (buf == (void*)&s->data_reg[0]);
    for (int i = 0; i < bytes; ++i) {
        uint8_t b = 0;
        if (i < tx_bytes) {
            if (using_data_regs) {
                const uint8_t *bytes_le = (const uint8_t *)s->data_reg;
                b = bytes_le[i];
            } else {
                b = c_buf[i];
            }
        }
        uint32_t res = ssi_transfer(s->spi, b);
        if (i < rx_bytes) {
            if (using_data_regs) {
                uint8_t *bytes_le = (uint8_t *)s->data_reg;
                bytes_le[i] = (uint8_t)res;
            } else {
                c_buf[i] = (uint8_t)res;
            }
        }
    if (s->unit_index == 3 && qemu_loglevel_mask(LOG_TRACE)) {
        qemu_log("SPI%u tx=0x%02x rx=0x%02x\n", s->unit_index, b, (uint8_t)res);
        }
    }
}

static void esp32_spi_cs_set(Esp32SpiState *s, int value)
{
    for (int i = 0; i < ESP32_SPI_CS_COUNT; ++i) {
        int hw_level = ((s->pin_reg & (1 << i)) == 0) ? value : 1;
        s->cs_hw_level[i] = hw_level;
        int effective = hw_level;
        if (s->cs_manual[i]) {
            effective &= s->cs_manual_level[i];
        }
        qemu_set_irq(s->cs_gpio[i], effective);
    }
}

static void esp32_spi_transaction(Esp32SpiState *s, Esp32SpiTransaction *t)
{
    esp32_spi_cs_set(s, 0);
    esp32_spi_txrx_buffer(s, &t->cmd, t->cmd_bytes, 0);
    esp32_spi_txrx_buffer(s, &t->addr, t->addr_bytes, 0);
    esp32_spi_txrx_buffer(s, t->data, t->data_tx_bytes, t->data_rx_bytes);
    esp32_spi_cs_set(s, 1);
    /* Mark done and raise IRQ if enabled */
    s->slave_reg |= BIT(R_SPI_SLAVE_TRANS_DONE_SHIFT);
    if (s->slave_reg & BIT(R_SPI_SLAVE_TRANS_INTEN_SHIFT)) {
        qemu_set_irq(s->irq, 1);
        qemu_set_irq(s->irq, 0);
    }
}

/* Convert one of the hardware "bitlen" registers to a byte count */
static inline int bitlen_to_bytes(uint32_t val)
{
    return (val + 1 + 7) / 8; /* bitlen registers hold number of bits, minus one */
}

static void maybe_encrypt_data(Esp32SpiState *s)
{
    Esp32FlashEncryptionState* flash_enc = esp32_flash_encryption_find();
    if (esp32_flash_encryption_enabled(flash_enc)) {
        esp32_flash_encryption_get_result(flash_enc, &s->data_reg[0], 8);
    }
}

static void esp32_spi_do_command(Esp32SpiState* s, uint32_t cmd_reg)
{
    /* Clear DONE at start */
    s->slave_reg &= ~BIT(R_SPI_SLAVE_TRANS_DONE_SHIFT);
    esp32_spi_update_pin_cache(s);
    /* Refresh VSPI/HSPI pin routing from GPIO matrix if available */
    if (s->gpio) {
        /* Resolve HSPI/VSPI signals to pins via GPIO matrix */
        int clk_sig = -1, miso_sig = -1, mosi_sig = -1, cs_sig[ESP32_SPI_CS_COUNT] = {-1,-1,-1};
        if (s->unit_index == 2) { /* HSPI */
            clk_sig = 8;  /* HSPICLK_OUT_IDX */
            miso_sig = 9; /* HSPIQ_OUT_IDX */
            mosi_sig = 10;/* HSPID_OUT_IDX */
            cs_sig[0] = 11; /* HSPICS0_OUT_IDX */
            cs_sig[1] = 61; /* HSPICS1_OUT_IDX */
            cs_sig[2] = 62; /* HSPICS2_OUT_IDX */
        } else if (s->unit_index == 3) { /* VSPI */
            clk_sig = 63;  /* VSPICLK_OUT_IDX */
            miso_sig = 64; /* VSPIQ_OUT_IDX */
            mosi_sig = 65; /* VSPID_OUT_IDX */
            cs_sig[0] = 68; /* VSPICS0_OUT_IDX */
            cs_sig[1] = 69; /* VSPICS1_OUT_IDX */
            cs_sig[2] = 70; /* VSPICS2_OUT_IDX */
        }
        if (clk_sig >= 0) s->sck_pin = esp32_gpio_get_pin_for_signal(s->gpio, clk_sig);
        if (miso_sig >= 0) s->miso_pin = esp32_gpio_get_pin_for_signal(s->gpio, miso_sig);
        if (mosi_sig >= 0) s->mosi_pin = esp32_gpio_get_pin_for_signal(s->gpio, mosi_sig);
        for (int i = 0; i < ESP32_SPI_CS_COUNT; ++i) {
            if (cs_sig[i] >= 0) s->cs_pin[i] = esp32_gpio_get_pin_for_signal(s->gpio, cs_sig[i]);
        }
    }
    Esp32SpiTransaction t = {
        .cmd_bytes = 1
    };
    switch (cmd_reg) {
    case R_SPI_CMD_READ_MASK:
        t.cmd = CMD_READ;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        t.data = &s->data_reg[0];
        t.data_rx_bytes = bitlen_to_bytes(s->miso_dlen_reg);
        break;

    case R_SPI_CMD_WREN_MASK:
        t.cmd = CMD_WREN;
        break;

    case R_SPI_CMD_WRDI_MASK:
        t.cmd = CMD_WRDI;
        break;

    case R_SPI_CMD_RDID_MASK:
        t.cmd = CMD_RDID;
        t.data = &s->data_reg[0];
        t.data_rx_bytes = 3;
        break;

    case R_SPI_CMD_RDSR_MASK:
        t.cmd = CMD_RDSR;
        t.data = &s->status_reg;
        t.data_rx_bytes = 1;
        break;

    case R_SPI_CMD_WRSR_MASK:
        t.cmd = CMD_WRSR;
        t.data = &s->status_reg;
        t.data_tx_bytes = 1;
        break;

    case R_SPI_CMD_PP_MASK:
        maybe_encrypt_data(s);
        t.cmd = CMD_PP;
        t.data = &s->data_reg[0];
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        t.data_tx_bytes = bitlen_to_bytes(s->mosi_dlen_reg);
        break;

    case R_SPI_CMD_SE_MASK:
        t.cmd = CMD_SE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_BE_MASK:
        t.cmd = CMD_BE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_CE_MASK:
        t.cmd = CMD_CE;
        break;

    case R_SPI_CMD_DP_MASK:
        t.cmd = CMD_DP;
        break;

    case R_SPI_CMD_RES_MASK:
        t.cmd = CMD_RES;
        t.data = &s->data_reg[0];
        t.data_rx_bytes = 3;
        break;

    case R_SPI_CMD_USR_MASK:
        maybe_encrypt_data(s);
        if (FIELD_EX32(s->user_reg, SPI_USER, COMMAND)) {
            t.cmd = FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_VALUE);
            t.cmd_bytes = bitlen_to_bytes(FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN));
        } else {
            t.cmd_bytes = 0;
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, ADDR)) {
            t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
            t.addr = bswap32(s->addr_reg);
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, MOSI)) {
            t.data = &s->data_reg[0];
            t.data_tx_bytes = bitlen_to_bytes(s->mosi_dlen_reg);
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, MISO)) {
            t.data = &s->data_reg[0];
            t.data_rx_bytes = bitlen_to_bytes(s->miso_dlen_reg);
        }
        if (s->unit_index == 3 && qemu_loglevel_mask(LOG_TRACE)) {
            qemu_log("SPI%u USR: cmd=%u cmd_bytes=%d addr_bytes=%d mosi=%d miso=%d W0=0x%08x\n",
                     s->unit_index, t.cmd, t.cmd_bytes, t.addr_bytes, t.data_tx_bytes, t.data_rx_bytes,
                     s->data_reg[0]);
        }
        break;
    default:
        return;
    }

    /* Sanity clamp lengths to buffer sizes and expected ranges */
    if (t.addr_bytes < 0) t.addr_bytes = 0;
    if (t.addr_bytes > 4) t.addr_bytes = 4;
    int max_data = ESP32_SPI_BUF_WORDS * (int)sizeof(uint32_t);
    if (t.data_tx_bytes < 0) t.data_tx_bytes = 0;
    if (t.data_tx_bytes > max_data) t.data_tx_bytes = max_data;
    if (t.data_rx_bytes < 0) t.data_rx_bytes = 0;
    if (t.data_rx_bytes > max_data) t.data_rx_bytes = max_data;

    esp32_spi_transaction(s, &t);
}


static const MemoryRegionOps esp32_spi_ops = {
    .read =  esp32_spi_read,
    .write = esp32_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_spi_reset_hold(Object *obj, ResetType type)
{
    Esp32SpiState *s = ESP32_SPI(obj);
    s->pin_reg = 0x6;
    s->user1_reg = FIELD_DP32(0, SPI_USER1, ADDR_BITLEN, 23);
    s->user1_reg = FIELD_DP32(s->user1_reg, SPI_USER1, DUMMY_CYCLELEN, 7);
    s->user2_reg = FIELD_DP32(0, SPI_USER2, COMMAND_BITLEN, 7);  /* 7 = 8 bits (bitlen-1) */
    s->user2_reg = FIELD_DP32(s->user2_reg, SPI_USER2, COMMAND_VALUE, 0);
    s->status_reg = 0;
    s->slave_reg = 0;
    s->sck_pin = -1;
    s->mosi_pin = -1;
    s->miso_pin = -1;
    for (int i = 0; i < ESP32_SPI_CS_COUNT; ++i) {
        s->cs_pin[i] = -1;
        s->cs_manual[i] = false;
        s->cs_listener_pin[i] = -1;
        s->cs_manual_level[i] = 1;
        s->cs_hw_level[i] = 1;
    }
    esp32_spi_default_gpio_mapping(s);
}

static void esp32_spi_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_spi_init(Object *obj)
{
    Esp32SpiState *s = ESP32_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_spi_ops, s,
                          TYPE_ESP32_SPI, ESP32_SPI_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->spi = ssi_create_bus(DEVICE(s), "spi");
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS, ESP32_SPI_CS_COUNT);
}

static void esp32_spi_gpio_level_cb(void *opaque, int pin, bool high)
{
    Esp32SpiState *s = opaque;
    for (int i = 0; i < ESP32_SPI_CS_COUNT; ++i) {
        if (s->cs_listener_pin[i] == pin) {
            s->cs_manual[i] = true;
            s->cs_manual_level[i] = high ? 1 : 0;
            int effective = s->cs_hw_level[i] & s->cs_manual_level[i];
            qemu_set_irq(s->cs_gpio[i], effective);
        }
    }
}

static void esp32_spi_update_pin_cache(Esp32SpiState *s)
{
    if (!s->gpio) {
        return;
    }

    int clk_sig = -1, miso_sig = -1, mosi_sig = -1;
    int cs_sig[ESP32_SPI_CS_COUNT] = {-1, -1, -1};

    if (s->unit_index == 2) { /* HSPI */
        clk_sig = 8;
        miso_sig = 9;
        mosi_sig = 10;
        cs_sig[0] = 11;
        cs_sig[1] = 61;
        cs_sig[2] = 62;
    } else if (s->unit_index == 3) { /* VSPI */
        clk_sig = 63;
        miso_sig = 64;
        mosi_sig = 65;
        cs_sig[0] = 68;
        cs_sig[1] = 69;
        cs_sig[2] = 70;
    }

    if (clk_sig >= 0) {
        s->sck_pin = esp32_gpio_get_pin_for_signal(s->gpio, clk_sig);
    }
    if (miso_sig >= 0) {
        s->miso_pin = esp32_gpio_get_pin_for_signal(s->gpio, miso_sig);
    }
    if (mosi_sig >= 0) {
        s->mosi_pin = esp32_gpio_get_pin_for_signal(s->gpio, mosi_sig);
    }

    for (int i = 0; i < ESP32_SPI_CS_COUNT; ++i) {
        int pin = (cs_sig[i] >= 0) ? esp32_gpio_get_pin_for_signal(s->gpio, cs_sig[i]) : -1;
        int old_pin = s->cs_listener_pin[i];
        if (old_pin >= 0 && old_pin != pin) {
            esp32_gpio_unregister_output_listener(s->gpio, old_pin, esp32_spi_gpio_level_cb, s);
        }

        s->cs_pin[i] = pin;
        if (pin >= 0) {
            esp32_gpio_register_output_listener(s->gpio, pin, esp32_spi_gpio_level_cb, s);
            s->cs_listener_pin[i] = pin;
            s->cs_manual[i] = true;
            bool level = esp32_gpio_get_output_level(s->gpio, pin);
            s->cs_manual_level[i] = level ? 1 : 0;
            int effective = s->cs_hw_level[i] & s->cs_manual_level[i];
            qemu_set_irq(s->cs_gpio[i], effective);
        } else {
            s->cs_listener_pin[i] = -1;
            s->cs_manual[i] = false;
            s->cs_manual_level[i] = 1;
        }
    }
}

static void esp32_spi_default_gpio_mapping(Esp32SpiState *s)
{
    if (!s->gpio) {
        return;
    }

    if (s->unit_index == 3) {
        /* Default VSPI pins: SCK=18, MISO=19, MOSI=23, CS0=5 */
        struct {
            int pin;
            int signal;
        } map[] = {
            {18, 63},
            {19, 64},
            {23, 65},
            {5,  68},
        };
        for (unsigned i = 0; i < G_N_ELEMENTS(map); ++i) {
            if (map[i].pin >= 0 && map[i].pin < 40) {
                s->gpio->func_out_sel_cfg[map[i].pin] = (uint16_t)map[i].signal;
            }
        }
    }

    esp32_spi_update_pin_cache(s);
}

static Property esp32_spi_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_spi_reset_hold;
    dc->realize = esp32_spi_realize;
    device_class_set_props(dc, esp32_spi_properties);
}

static const TypeInfo esp32_spi_info = {
    .name = TYPE_ESP32_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SpiState),
    .instance_init = esp32_spi_init,
    .class_init = esp32_spi_class_init
};

static void esp32_spi_register_types(void)
{
    type_register_static(&esp32_spi_info);
}

type_init(esp32_spi_register_types)
