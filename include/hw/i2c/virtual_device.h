/**
 * Virtual Device - Bus Agnostic Base Types and VTable
 *
 * This header defines the common base structure and lifecycle vtable for
 * virtual devices that may attach to various serial buses (I2C/SPI/UART,...).
 * Device-specific, bus-specific operation tables are defined separately.
 */

#ifndef VIRTUAL_DEVICE_H
#define VIRTUAL_DEVICE_H

#include "qemu/osdep.h"

typedef struct VDevBase VDevBase;

typedef enum VDevBusType {
    VDEV_BUS_I2C = 0,
    VDEV_BUS_SPI,
    VDEV_BUS_UART,
    VDEV_BUS_GPIO,
    VDEV_BUS_UNKNOWN
} VDevBusType;

typedef enum VDevEventType {
    VDEV_EVENT_START = 0,
    VDEV_EVENT_RSTART,
    VDEV_EVENT_STOP,
    VDEV_EVENT_END
} VDevEventType;

typedef struct VDevVTable {
    /* Lifecycle */
    bool (*init)(VDevBase *device);
    void (*reset)(VDevBase *device);
    void (*destroy)(VDevBase *device);

    /* Periodic update (optional) */
    void (*tick)(VDevBase *device, uint64_t now_ns);

    /* Generic event hook (optional) */
    void (*on_event)(VDevBase *device, VDevEventType event_type);

    /* Generic control interface (optional) */
    int (*ioctl)(VDevBase *device, int command, void *argument);
} VDevVTable;

struct VDevBase {
    /* Identity */
    const char *name;
    VDevBusType bus_type;

    /* State */
    bool present;
    bool responding;
    bool debug_enabled;
    uint64_t last_access_time;
    uint32_t access_count;

    /* Integration */
    void *bus_context;            /* Pointer to owning controller/bus */
    const VDevVTable *vtable;     /* Lifecycle and control hooks */
};

#endif /* VIRTUAL_DEVICE_H */


