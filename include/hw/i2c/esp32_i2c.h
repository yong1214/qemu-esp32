#ifndef ESP32_I2C_H
#define ESP32_I2C_H

#include "hw/sysbus.h"
#include "qemu/fifo8.h"
#include "hw/i2c/i2c.h"
#include "hw/registerfields.h"
#include "qemu/timer.h"
#include "hw/gpio/esp32_gpio.h"

#define TYPE_ESP32_I2C "esp32.i2c"
#define Esp32_I2C(obj) OBJECT_CHECK(Esp32I2CState, (obj), TYPE_ESP32_I2C)


#define ESP32_I2C_MEM_SIZE 0x100
#define ESP32_I2C_FIFO_LENGTH 32
#define ESP32_I2C_CMD_COUNT 16

/* Phase 3: Hardware Timing Constants */
#define ESP32_I2C_CLOCK_FREQ_HZ     80000000  // 80MHz APB clock
#define ESP32_I2C_STANDARD_MODE     100000    // 100kHz I2C standard mode
#define ESP32_I2C_FAST_MODE         400000    // 400kHz I2C fast mode
#define ESP32_I2C_FAST_PLUS_MODE    1000000   // 1MHz I2C fast plus mode

/* Phase 5: I2C Protocol Constants */
#define I2C_START_CONDITION         0x01
#define I2C_STOP_CONDITION          0x02
#define I2C_ACK_BIT                 0x00
#define I2C_NACK_BIT                0x01
#define I2C_READ_BIT                0x01
#define I2C_WRITE_BIT               0x00
#define I2C_MAX_DEVICE_ADDRESS      0x7F
#define I2C_MAX_DATA_BYTES          255

/* Phase 6: I2C Bus Simulation Constants */
#define I2C_BUS_IDLE                0x00
#define I2C_BUS_START               0x01
#define I2C_BUS_ADDRESS             0x02
#define I2C_BUS_DATA                0x03
#define I2C_BUS_ACK                 0x04
#define I2C_BUS_NACK                0x05
#define I2C_BUS_STOP                0x06
#define I2C_BUS_ERROR               0x07

#define I2C_BUS_PULLUP_RESISTANCE   4000    // 4kΩ pullup resistance
#define I2C_BUS_CAPACITANCE         100     // 100pF bus capacitance
#define I2C_BUS_RISE_TIME_NS        1000    // 1μs rise time
#define I2C_BUS_FALL_TIME_NS        300     // 300ns fall time

/* Phase 7: Interrupt Generation Constants */
#define I2C_INTERRUPT_ACK_ERR        0x400   // Bit 10: Acknowledgment error
#define I2C_INTERRUPT_TRANS_COMPLETE 0x80    // Bit 7: Transaction complete
#define I2C_INTERRUPT_END_DETECT     0x08    // Bit 3: End detection
#define I2C_INTERRUPT_TX_FIFO_EMPTY  0x02    // Bit 1: TX FIFO empty (matches HAL I2C_TXFIFO_EMPTY_INT)
#define I2C_INTERRUPT_RX_FIFO_FULL   0x01    // Bit 0: RX FIFO full (matches HAL I2C_RXFIFO_FULL_INT)
#define I2C_INTERRUPT_START_DETECT   0x00    // Not modeled here; reserved

/* Phase 8: Virtual Device Integration Constants */
#define I2C_MAX_VIRTUAL_DEVICES     16       // Maximum virtual devices per bus
#define I2C_DEVICE_ADDRESS_MASK     0x7F     // 7-bit device address mask
#define I2C_DEVICE_RESPONSE_DELAY_NS 1000    // 1μs device response delay

/* Phase 3: I2C Clock Modes */
typedef enum {
    I2C_CLOCK_STANDARD = 0,    // 100kHz
    I2C_CLOCK_FAST,            // 400kHz
    I2C_CLOCK_FAST_PLUS        // 1MHz
} I2CClockMode;

/* Phase 4: Transaction Management */
typedef enum {
    I2C_TRANSACTION_IDLE = 0,
    I2C_TRANSACTION_PENDING,
    I2C_TRANSACTION_ACTIVE,
    I2C_TRANSACTION_COMPLETED,
    I2C_TRANSACTION_ERROR
} I2CTransactionStatus;

typedef enum {
    I2C_ERROR_NONE = 0,
    I2C_ERROR_TIMEOUT,
    I2C_ERROR_NACK,
    I2C_ERROR_BUS_BUSY,
    I2C_ERROR_INVALID_STATE,
    I2C_ERROR_FIFO_OVERFLOW
} I2CErrorType;

typedef struct {
    uint8_t device_address;
    uint8_t opcode;
    uint8_t byte_count;
    uint8_t *data_buffer;
    uint32_t timeout_ms;
    I2CTransactionStatus status;
    I2CErrorType error;
    uint64_t start_time;
    uint64_t completion_time;
} I2CTransaction;

/* Phase 1: I2C State Machine Foundation */
typedef enum {
    I2C_STATE_IDLE = 0,
    I2C_STATE_START,
    I2C_STATE_ADDRESS,
    I2C_STATE_DATA_WRITE,
    I2C_STATE_DATA_READ,
    I2C_STATE_STOP,
    I2C_STATE_COMPLETE,
    I2C_STATE_ERROR
} I2CState;

typedef enum {
    I2C_PHASE_START_CONDITION = 0,
    I2C_PHASE_ADDRESS_SEND,
    I2C_PHASE_ADDRESS_ACK,
    I2C_PHASE_DATA_SEND,
    I2C_PHASE_DATA_ACK,
    I2C_PHASE_STOP_CONDITION,
    I2C_PHASE_COMPLETE
} I2CPhase;

/* Stream-position state: address vs data interpretation (mirrors real hardware) */
typedef enum {
    I2C_STREAM_IDLE = 0,    /* After STOP; next byte = address */
    I2C_STREAM_IN_DATA      /* Address sent; next byte(s) = data */
} I2CStreamState;

/* Phase 7: Interrupt Types */
typedef enum {
    I2C_INTERRUPT_TYPE_ACK_ERR = 0,
    I2C_INTERRUPT_TYPE_TRANS_COMPLETE,
    I2C_INTERRUPT_TYPE_END_DETECT,
    I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY,
    I2C_INTERRUPT_TYPE_RX_FIFO_FULL,
    I2C_INTERRUPT_TYPE_START_DETECT,
    I2C_INTERRUPT_TYPE_COUNT
} I2CInterruptType;

/* Phase 8: Virtual Device Types */
typedef enum {
    I2C_DEVICE_TYPE_TMP105 = 0,
    I2C_DEVICE_TYPE_EEPROM,
    I2C_DEVICE_TYPE_RTC,
    I2C_DEVICE_TYPE_SSD1306,
    I2C_DEVICE_TYPE_UNKNOWN
} I2CVirtualDeviceType;

typedef enum {
    I2C_DEVICE_STATE_IDLE = 0,
    I2C_DEVICE_STATE_ADDRESSED,
    I2C_DEVICE_STATE_READING,
    I2C_DEVICE_STATE_WRITING,
    I2C_DEVICE_STATE_ERROR
} I2CVirtualDeviceState;

/* Phase 8: Common Base Structure for All Virtual Devices */
typedef struct {
    uint8_t address;                    // 7-bit I2C address
    I2CVirtualDeviceType type;          // Device type
    I2CVirtualDeviceState state;        // Current device state
    bool present;                       // Device present on bus
    bool responding;                    // Device responding to transactions
    uint64_t last_access_time;          // Last access timestamp
    uint32_t access_count;              // Total access count
    bool debug_enabled;                 // Debug logging enabled
} I2CVirtualDeviceBase;

/* Phase 8: TMP105 Temperature Sensor Device */
typedef struct {
    I2CVirtualDeviceBase base;          // Common base structure
    float temperature;                  // Current temperature in Celsius
    uint8_t config_register;            // Configuration register value
    uint8_t resolution;                 // Temperature resolution (9-12 bits)
    uint8_t register_address;           // Current register being accessed
    bool shutdown_mode;                 // Shutdown mode flag
    uint32_t conversion_time_us;        // Conversion time in microseconds
    uint8_t read_phase;                 // 0: next byte is MSB, 1: next byte is LSB
    // Minimal environment simulation
    double drift_c_per_sec;             // Temperature drift rate (C/s)
    double noise_c_amplitude;           // Max random noise per conversion (C)
    float temp_min_c;                   // Minimum clamp
    float temp_max_c;                   // Maximum clamp
    uint64_t last_conversion_time_ns;   // Last conversion timestamp
} TMP105Device;

/* Phase 8: EEPROM Memory Device */
typedef struct {
    I2CVirtualDeviceBase base;          // Common base structure
    uint8_t memory[4096];               // 4KB EEPROM memory
    uint16_t current_address;           // Current address pointer
    uint8_t write_protect;              // Write protection flag
    uint32_t write_cycles;              // Write cycle count (endurance)
    uint8_t page_size;                  // Page size for writes
    uint8_t address_bytes;              // Number of address bytes (1 or 2)
    uint32_t write_delay_ms;            // Write delay in milliseconds
} EEPROMDevice;

/* Phase 8: RTC Real-Time Clock Device */
typedef struct {
    I2CVirtualDeviceBase base;          // Common base structure
    uint8_t seconds;                    // BCD seconds (0-59)
    uint8_t minutes;                    // BCD minutes (0-59)
    uint8_t hours;                      // BCD hours (0-23)
    uint8_t day;                        // BCD day (1-31)
    uint8_t month;                      // BCD month (1-12)
    uint8_t year;                       // BCD year (0-99)
    uint8_t control_register;           // Control register value
    uint8_t register_address;           // Current register being accessed
    bool alarm_enabled;                 // Alarm enabled flag
    bool oscillator_enabled;            // Oscillator enabled flag
} RTCDevice;

/* Phase 8: SSD1306 OLED Display Device */
typedef struct {
    I2CVirtualDeviceBase base;          // Common base structure
    uint8_t display_buffer[1024];       // 128x64 pixel buffer
    uint8_t current_column;             // Current column address
    uint8_t current_page;               // Current page address
    uint8_t display_mode;               // Display mode (normal/inverse)
    uint8_t brightness;                 // Display brightness (0-255)
    bool display_on;                    // Display on/off flag
    uint8_t command_mode;               // Command/data mode flag
} SSD1306Device;

/* Phase 8: Generic Virtual Device Union */
typedef union {
    I2CVirtualDeviceBase base;
    TMP105Device tmp105;
    EEPROMDevice eeprom;
    RTCDevice rtc;
    SSD1306Device ssd1306;
} I2CVirtualDevice;

typedef struct Esp32I2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;
    int unit_index;                // 0 for I2C0, 1 for I2C1
    Fifo8 rx_fifo;
    Fifo8 tx_fifo;
    bool trans_ongoing;
    I2CStreamState stream_state;  /* For address vs data interpretation */
    /* Open-drain line integration with GPIO */
    Esp32GpioState *gpio;
    int sda_pin;
    int scl_pin;

    uint32_t ctr_reg;
    uint32_t timeout_reg;
    uint32_t fifo_conf_reg;
    uint32_t int_ena_reg;
    uint32_t int_raw_reg;
    uint32_t sda_hold_reg;
    uint32_t sda_sample_reg;
    uint32_t high_period_reg;
    uint32_t low_period_reg;
    uint32_t start_hold_reg;
    uint32_t rstart_setup_reg;
    uint32_t stop_hold_reg;
    uint32_t stop_setup_reg;
    uint32_t cmd_reg[ESP32_I2C_CMD_COUNT];

    /* Phase 1: State Machine Foundation */
    I2CState current_state;
    I2CState next_state;
    I2CPhase current_phase;
    uint8_t state_phase;
    bool state_machine_active;
    bool virtual_sda_state;
    bool virtual_scl_state;
    bool i2c_bus_active;

    /* Phase 2: Timer-Based Execution */
    QEMUTimer *state_timer;
    QEMUTimer *phase_timer;

    /* Phase 3: Hardware Timing Simulation */
    I2CClockMode clock_mode;
    uint32_t i2c_clock_freq;
    uint32_t apb_clock_freq;
    bool timing_validation_enabled;
    uint64_t last_transaction_time;

    /* Phase 4: Asynchronous Transaction Processing */
    I2CTransaction current_transaction;
    I2CTransaction pending_transactions[ESP32_I2C_CMD_COUNT];
    uint8_t transaction_queue_head;
    uint8_t transaction_queue_tail;
    uint8_t transaction_queue_count;
    bool transaction_processing_enabled;
    uint32_t transaction_timeout_ms;
    I2CErrorType last_error;

    /* Phase 5: I2C Protocol Implementation */
    uint8_t current_device_address;
    uint8_t current_data_byte;
    uint8_t current_byte_index;
    uint8_t expected_ack;
    bool start_condition_sent;
    bool stop_condition_sent;
    bool address_sent;
    bool data_sent;
    bool ack_received;
    bool nack_received;
    uint8_t tx_data_buffer[I2C_MAX_DATA_BYTES];
    uint8_t rx_data_buffer[I2C_MAX_DATA_BYTES];

    /* Phase 6: I2C Bus Simulation */
    uint8_t bus_state;
    uint8_t bus_previous_state;
    bool sda_line_state;
    bool scl_line_state;
    bool sda_driven;
    bool scl_driven;
    bool bus_arbitration_lost;
    bool bus_collision_detected;
    uint32_t bus_rise_time_ns;
    uint32_t bus_fall_time_ns;
    uint32_t bus_pullup_resistance;
    uint32_t bus_capacitance_pf;
    uint64_t last_bus_transition_time;
    uint32_t bus_error_count;
    bool bus_monitoring_enabled;

    /* Phase 7: Interrupt Generation */
    uint32_t interrupt_pending;
    uint32_t interrupt_enabled;
    uint32_t interrupt_priority;
    bool interrupt_generation_enabled;
    uint64_t last_interrupt_time;
    uint32_t interrupt_count;
    bool interrupt_debug_enabled;

    /* Phase 8: Virtual Device Integration */
    I2CVirtualDevice virtual_devices[I2C_MAX_VIRTUAL_DEVICES];
    uint8_t device_count;
    bool device_integration_enabled;
    bool device_scan_enabled;
    uint64_t last_device_scan_time;
    uint32_t device_scan_interval_ns;
    bool device_debug_enabled;
    /* New: I2C virtual device registry (VDev-based) */
    struct I2CDeviceSet *vdev_i2c_set;

    /* RTC model/config properties */
    char *rtc_model;            // "ds1307" | "ds3231" | NULL (default)
    int rtc_init_year;          // 0..99, -1 = unset
    int rtc_init_month;         // 1..12, -1 = unset
    int rtc_init_day;           // 1..31, -1 = unset
    int rtc_init_dow;           // 1..7, -1 = unset
    int rtc_init_hour;          // 0..23, -1 = unset
    int rtc_init_minute;        // 0..59, -1 = unset
    int rtc_init_second;        // 0..59, -1 = unset

    /* QOM device attach list: CSV like "tmp105@0x48,eeprom@0x50:4096,ds3231@0x68" */
    char *dev_list;

    /* Logging control: 0=silent, 1=info */
    int log_level;

    /* Optional DS3231 SQW/INT pin routing to GPIO */
    int rtc_sqw_pin;            // -1 = disabled; else GPIO pin number

    /* Per-device defaults (controller-wide) */
    int tmp105_init_temp_c;     // Celsius * 1
    int tmp105_drift_mcps;      // milli-C/s
    int tmp105_noise_mc;        // milli-C amplitude
    int eeprom_page_size;       // bytes per page
    int eeprom_write_delay_ms;  // ms
    int ds3231_init_temp_c;     // Celsius * 1

    /* FIFO watermark thresholds and last IRQ state */
    uint8_t tx_fifo_wm;         // interrupt when used <= wm
    uint8_t rx_fifo_wm;         // interrupt when used >= wm
    bool tx_irq_asserted;
    bool rx_irq_asserted;
} Esp32I2CState;


REG32(I2C_CTR, 0x04);
    FIELD(I2C_CTR, MS_MODE, 4, 1);
    FIELD(I2C_CTR, TRANS_START, 5, 1);

REG32(I2C_STATUS, 0x08);
    FIELD(I2C_STATUS, BUS_BUSY, 4, 1);
    FIELD(I2C_STATUS, RXFIFO_CNT, 8, 6);
    FIELD(I2C_STATUS, TXFIFO_CNT, 18, 6);

REG32(I2C_TIMEOUT, 0x0c);

REG32(I2C_FIFO_CONF, 0x18);
    FIELD(I2C_FIFO_CONF, NONFIFO_EN, 10, 1);
    FIELD(I2C_FIFO_CONF, RX_FIFO_RST, 12, 1);
    FIELD(I2C_FIFO_CONF, TX_FIFO_RST, 13, 1);

REG32(I2C_FIFO_DATA, 0x1c);

REG32(I2C_INT_RAW, 0x20);
    FIELD(I2C_INT_RAW, ACK_ERR, 10, 1);
    FIELD(I2C_INT_RAW, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_RAW, END_DETECT, 3, 1);

REG32(I2C_INT_CLR, 0x24);
    FIELD(I2C_INT_CLR, ACK_ERR, 10, 1);
    FIELD(I2C_INT_CLR, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_CLR, END_DETECT, 3, 1);

REG32(I2C_INT_ENA, 0x28);
    FIELD(I2C_INT_ENA, ACK_ERR, 10, 1);
    FIELD(I2C_INT_ENA, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_ENA, END_DETECT, 3, 1);

REG32(I2C_INT_ST, 0x2c);
    FIELD(I2C_INT_ST, ACK_ERR, 10, 1);
    FIELD(I2C_INT_ST, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_ST, END_DETECT, 3, 1);

REG32(I2C_SDA_HOLD, 0x30);
REG32(I2C_SDA_SAMPLE, 0x34);
REG32(I2C_HIGH_PERIOD, 0x38);
REG32(I2C_LOW_PERIOD, 0x00);  // 0x00 is not a typo
REG32(I2C_START_HOLD, 0x40);
REG32(I2C_RSTART_SETUP, 0x44);
REG32(I2C_STOP_HOLD, 0x48);
REG32(I2C_STOP_SETUP, 0x4c);

REG32(I2C_CMD, 0x58);
    FIELD(I2C_CMD, BYTE_NUM, 0, 8);
    FIELD(I2C_CMD, ACK_CHECK_EN, 8, 1);
    FIELD(I2C_CMD, ACK_EXP, 9, 1);
    FIELD(I2C_CMD, ACK_VAL, 10, 1);
    FIELD(I2C_CMD, OPCODE, 11, 3);
    FIELD(I2C_CMD, DONE, 31, 1);
/* 15 more command registers omitted */

/* I2C_CMD.OPCODE values */
typedef enum {
    I2C_OPCODE_RSTART = 0,
    I2C_OPCODE_WRITE  = 1,
    I2C_OPCODE_READ   = 2,
    I2C_OPCODE_STOP   = 3,
    I2C_OPCODE_END    = 4,
} i2c_opcode_t;

#endif /* ESP32_I2C_H */
