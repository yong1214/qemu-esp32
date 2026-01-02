#pragma once

#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define TYPE_SSI_LOOPBACK "ssi.loopback"

typedef struct SSILoopbackState {
    SSIPeripheral parent_obj;
    uint8_t last;
} SSILoopbackState;

OBJECT_DECLARE_SIMPLE_TYPE(SSILoopbackState, SSI_LOOPBACK)


