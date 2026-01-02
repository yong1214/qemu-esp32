#pragma once

#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define TYPE_SSI_EEPROM "esp.spi_eeprom"

typedef struct SSIEepromState {
    SSIPeripheral parent_obj;
    uint8_t *storage;
    uint32_t size;

    /* protocol state */
    bool cs_active;
    uint8_t cmd;
    uint32_t addr;
    int phase; /* 0=cmd, 1..3 address bytes, 4=data */
    bool write_enable;
    uint8_t status;
} SSIEepromState;

OBJECT_DECLARE_SIMPLE_TYPE(SSIEepromState, SSI_EEPROM)
