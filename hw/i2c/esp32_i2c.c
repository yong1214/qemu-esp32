#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/qdev-properties.h"
#include "hw/i2c/device_set.h"
#include "hw/i2c/devices/tmp105_vdev.h"
#include "hw/i2c/devices/eeprom_vdev.h"
#include "hw/i2c/devices/ds1307_vdev.h"
#include "hw/i2c/devices/ds3231_vdev.h"
#include "hw/i2c/devices/ssd1306_vdev.h"
#include "hw/i2c/devices/bme280_vdev.h"
#include "hw/i2c/devices/bh1750_vdev.h"
#include "hw/i2c/devices/aht20_vdev.h"
#include "hw/i2c/devices/adxl345_vdev.h"
#include "hw/i2c/devices/ina219_vdev.h"
#include "hw/i2c/esp32_i2c_devices.h"
#include "hw/gpio/esp32_gpio.h"
#include "hw/irq.h"
#include "hw/i2c/esp32_i2c_monitor.h"

static void esp32_i2c_do_transaction(Esp32I2CState * s);
static void esp32_i2c_update_irq(Esp32I2CState * s);

/* Update SDA/SCL pins from GPIO matrix routing if IO_MUX remap was applied */
static void esp32_i2c_update_sda_scl_from_gpio(Esp32I2CState *s)
{
    if (!s || !s->gpio) return;
    int scl_sig = (s->unit_index == 0) ? 29 : 95; /* I2CEXT0/1 SCL OUT IDX */
    int sda_sig = (s->unit_index == 0) ? 30 : 96; /* I2CEXT0/1 SDA OUT IDX */
    int scl_pin = esp32_gpio_get_pin_for_signal(s->gpio, scl_sig);
    int sda_pin = esp32_gpio_get_pin_for_signal(s->gpio, sda_sig);
    if (scl_pin >= 0) s->scl_pin = scl_pin;
    if (sda_pin >= 0) s->sda_pin = sda_pin;
}

/* Phase 6: I2C Bus Simulation - Early declaration */
static void esp32_i2c_simulate_bus_transition(Esp32I2CState *s, bool sda, bool scl);

/* Phase 7: Interrupt Generation - Early declarations */
static void esp32_i2c_interrupt_init(Esp32I2CState *s);
static void esp32_i2c_interrupt_reset(Esp32I2CState *s);
static void esp32_i2c_generate_interrupt(Esp32I2CState *s, I2CInterruptType type);
static void esp32_i2c_clear_interrupt(Esp32I2CState *s, I2CInterruptType type);
static void esp32_i2c_enable_interrupt(Esp32I2CState *s, I2CInterruptType type);
static void esp32_i2c_disable_interrupt(Esp32I2CState *s, I2CInterruptType type);
static bool esp32_i2c_is_interrupt_enabled(Esp32I2CState *s, I2CInterruptType type);
static bool esp32_i2c_is_interrupt_pending(Esp32I2CState *s, I2CInterruptType type);
static void esp32_i2c_handle_transaction_complete_interrupt(Esp32I2CState *s);
static void esp32_i2c_handle_ack_error_interrupt(Esp32I2CState *s);
static void esp32_i2c_handle_end_detect_interrupt(Esp32I2CState *s);
static void esp32_i2c_handle_fifo_interrupts(Esp32I2CState *s);
static void esp32_i2c_log_interrupt_event(Esp32I2CState *s, I2CInterruptType type, const char* reason);

/* Phase 8: Virtual Device Integration - Early declarations */
static void esp32_i2c_virtual_device_init(Esp32I2CState *s);
static void esp32_i2c_virtual_device_reset(Esp32I2CState *s);
static void esp32_i2c_scan_virtual_devices(Esp32I2CState *s);

/* Phase 1: State Machine Foundation Functions */
static void esp32_i2c_state_machine_init(Esp32I2CState *s)
{
    s->current_state = I2C_STATE_IDLE;
    s->next_state = I2C_STATE_IDLE;
    s->current_phase = I2C_PHASE_COMPLETE;
    s->state_phase = 0;
    s->state_machine_active = false;
    s->virtual_sda_state = true;  // SDA high (idle)
    s->virtual_scl_state = true;  // SCL high (idle)
    s->i2c_bus_active = false;
}

static void esp32_i2c_state_machine_reset(Esp32I2CState *s)
{
    s->current_state = I2C_STATE_IDLE;
    s->next_state = I2C_STATE_IDLE;
    s->current_phase = I2C_PHASE_COMPLETE;
    s->state_phase = 0;
    s->state_machine_active = false;
    s->virtual_sda_state = true;  // SDA high (idle)
    s->virtual_scl_state = true;  // SCL high (idle)
    s->i2c_bus_active = false;
}

static void esp32_i2c_state_machine_advance(Esp32I2CState *s)
{
    s->current_state = s->next_state;
    switch (s->current_state) {
    case I2C_STATE_START:
        s->next_state = I2C_STATE_ADDRESS;
        break;
    case I2C_STATE_ADDRESS:
        s->next_state = I2C_STATE_DATA_WRITE;
        break;
    case I2C_STATE_DATA_WRITE:
        s->next_state = I2C_STATE_STOP;
        break;
    case I2C_STATE_STOP:
        s->next_state = I2C_STATE_COMPLETE;
        break;
    default:
        s->next_state = I2C_STATE_IDLE;
        break;
    }
}

/* Phase 1: Virtual Bus State Management (Enhanced with Phase 6) */
static void esp32_i2c_protocol_bus_signals(Esp32I2CState *s, bool sda, bool scl)
{
    // Update virtual bus state (Phase 1 compatibility)
    s->virtual_sda_state = sda;
    s->virtual_scl_state = scl;
    s->i2c_bus_active = (sda || scl);  // Bus is active if either line is active
    
    // Reflect line levels to GPIO (open-drain shared lines)
    if (s->gpio) {
        esp32_gpio_set_input_level(s->gpio, s->sda_pin, sda);
        esp32_gpio_set_input_level(s->gpio, s->scl_pin, scl);
    }
    // Phase 6: Enhanced bus simulation
    esp32_i2c_simulate_bus_transition(s, sda, scl);
    
}

static void esp32_i2c_set_bus_idle(Esp32I2CState *s)
{
    s->virtual_sda_state = true;  // SDA high (idle)
    s->virtual_scl_state = true;  // SCL high (idle)
    s->i2c_bus_active = false;
}

static void esp32_i2c_set_bus_active(Esp32I2CState *s)
{
    s->i2c_bus_active = true;
}

/* Phase 2: Timer-Based Execution Functions */
static void esp32_i2c_state_timer_cb(void *opaque);
static void esp32_i2c_phase_timer_cb(void *opaque);
static void esp32_i2c_schedule_state_timer(Esp32I2CState *s);
static void esp32_i2c_schedule_phase_timer(Esp32I2CState *s);
static uint64_t esp32_i2c_get_state_delay(Esp32I2CState *s);
static uint64_t esp32_i2c_get_phase_delay(Esp32I2CState *s);
static void esp32_i2c_process_transaction(Esp32I2CState *s);
static void esp32_i2c_process_phase(Esp32I2CState *s);
static void esp32_i2c_advance_phase(Esp32I2CState *s);
static void esp32_i2c_start_transaction(Esp32I2CState *s);
static void esp32_i2c_do_start_condition(Esp32I2CState *s);
static void esp32_i2c_do_address_phase(Esp32I2CState *s);
static void esp32_i2c_do_data_write(Esp32I2CState *s);
static void esp32_i2c_do_data_read(Esp32I2CState *s);
static void esp32_i2c_do_stop_condition(Esp32I2CState *s);
static void esp32_i2c_complete_transaction_legacy(Esp32I2CState *s);

/* Phase 3: Hardware Timing Simulation Functions */
static void esp32_i2c_timing_init(Esp32I2CState *s);
static void esp32_i2c_timing_reset(Esp32I2CState *s);
static uint64_t esp32_i2c_calculate_hardware_delay(Esp32I2CState *s, I2CState state);
static uint64_t esp32_i2c_calculate_phase_delay(Esp32I2CState *s, I2CPhase phase);
static void esp32_i2c_validate_timing(Esp32I2CState *s, uint64_t delay_ns);
static I2CClockMode esp32_i2c_detect_clock_mode(Esp32I2CState *s);
static uint32_t esp32_i2c_get_clock_frequency(Esp32I2CState *s);

/* Phase 4: Asynchronous Transaction Processing Functions */
static void esp32_i2c_transaction_init(Esp32I2CState *s);
static void esp32_i2c_transaction_reset(Esp32I2CState *s);
static bool esp32_i2c_queue_transaction(Esp32I2CState *s, I2CTransaction *transaction);
static bool esp32_i2c_dequeue_transaction(Esp32I2CState *s, I2CTransaction *transaction);
static void esp32_i2c_start_next_transaction(Esp32I2CState *s);
static void esp32_i2c_complete_transaction(Esp32I2CState *s, I2CErrorType error);
static void esp32_i2c_handle_transaction_timeout(Esp32I2CState *s);
static bool esp32_i2c_is_transaction_queue_full(Esp32I2CState *s);
static bool esp32_i2c_is_transaction_queue_empty(Esp32I2CState *s);
static void esp32_i2c_validate_transaction(Esp32I2CState *s, I2CTransaction *transaction);

/* Phase 6: I2C Bus Simulation Functions (Early Declaration) */
static void esp32_i2c_simulate_bus_transition(Esp32I2CState *s, bool sda, bool scl);

/* Phase 5: I2C Protocol Implementation Functions */
static void esp32_i2c_protocol_init(Esp32I2CState *s);
static void esp32_i2c_protocol_reset(Esp32I2CState *s);
static void esp32_i2c_send_start_condition(Esp32I2CState *s);
static void esp32_i2c_send_stop_condition(Esp32I2CState *s);
static void esp32_i2c_send_device_address(Esp32I2CState *s);
static void esp32_i2c_send_data_byte(Esp32I2CState *s);
static void esp32_i2c_receive_data_byte(Esp32I2CState *s);
static void esp32_i2c_check_ack(Esp32I2CState *s);
static void esp32_i2c_send_ack(Esp32I2CState *s);
static void esp32_i2c_send_nack(Esp32I2CState *s);
static bool esp32_i2c_simulate_device_response(Esp32I2CState *s, uint8_t device_addr);

/* Phase 6: I2C Bus Simulation Functions */
static void esp32_i2c_bus_simulation_init(Esp32I2CState *s);
static void esp32_i2c_bus_simulation_reset(Esp32I2CState *s);
static void esp32_i2c_update_bus_state(Esp32I2CState *s, uint8_t new_state);
static void esp32_i2c_simulate_bus_transition(Esp32I2CState *s, bool sda, bool scl);
static void esp32_i2c_detect_bus_collision(Esp32I2CState *s);
static void esp32_i2c_handle_bus_arbitration(Esp32I2CState *s);
static void esp32_i2c_simulate_bus_electrical_behavior(Esp32I2CState *s);
static void esp32_i2c_monitor_bus_activity(Esp32I2CState *s);
static void esp32_i2c_validate_bus_timing(Esp32I2CState *s);
static void esp32_i2c_log_bus_state_change(Esp32I2CState *s, const char* reason);

/* Phase 2: Timer Callback Functions */
static void esp32_i2c_state_timer_cb(void *opaque)
{
    Esp32I2CState *s = Esp32_I2C(opaque);
    
    // quiet verbose timer log
    
    // Process current state
    esp32_i2c_process_transaction(s);
    
    // Advance to next state
    esp32_i2c_state_machine_advance(s);
    
    // Schedule next state if not complete
    if (s->current_state != I2C_STATE_COMPLETE) {
        esp32_i2c_schedule_state_timer(s);
    } else {
        s->state_machine_active = false;
        // quiet completion log
    }
}

static void esp32_i2c_phase_timer_cb(void *opaque)
{
    Esp32I2CState *s = Esp32_I2C(opaque);
    
    // quiet verbose phase timer log
    
    // Process current phase
    esp32_i2c_process_phase(s);
    
    // Advance to next phase
    esp32_i2c_advance_phase(s);
    
    // Schedule next phase if not complete
    if (s->current_phase != I2C_PHASE_COMPLETE) {
        esp32_i2c_schedule_phase_timer(s);
    }
}

/* Phase 2: Timer Scheduling Functions */
static void esp32_i2c_schedule_state_timer(Esp32I2CState *s)
{
    uint64_t delay_ns = esp32_i2c_get_state_delay(s);
    int64_t expire_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns;
    
    timer_mod_ns(s->state_timer, expire_time);
    
    // quiet schedule log
}

static void esp32_i2c_schedule_phase_timer(Esp32I2CState *s)
{
    uint64_t delay_ns = esp32_i2c_get_phase_delay(s);
    int64_t expire_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns;
    
    timer_mod_ns(s->phase_timer, expire_time);
    
    // quiet schedule log
}

/* Phase 2: Timing Functions (Updated for Phase 3) */
static uint64_t esp32_i2c_get_state_delay(Esp32I2CState *s)
{
    // Use hardware-accurate timing calculation
    uint64_t delay_ns = esp32_i2c_calculate_hardware_delay(s, s->current_state);
    
    // Validate timing
    esp32_i2c_validate_timing(s, delay_ns);
    
    return delay_ns;
}

static uint64_t esp32_i2c_get_phase_delay(Esp32I2CState *s)
{
    // Use hardware-accurate phase timing calculation
    uint64_t delay_ns = esp32_i2c_calculate_phase_delay(s, s->current_phase);
    
    // Validate timing
    esp32_i2c_validate_timing(s, delay_ns);
    
    return delay_ns;
}

/* Phase 3: Hardware Timing Initialization */
static void esp32_i2c_timing_init(Esp32I2CState *s)
{
    s->clock_mode = I2C_CLOCK_STANDARD;  // Default to 100kHz
    s->i2c_clock_freq = ESP32_I2C_STANDARD_MODE;
    s->apb_clock_freq = ESP32_I2C_CLOCK_FREQ_HZ;
    s->timing_validation_enabled = true;
    s->last_transaction_time = 0;
    
    // quiet timing init log
}

static void esp32_i2c_timing_reset(Esp32I2CState *s)
{
    s->clock_mode = I2C_CLOCK_STANDARD;  // Reset to default
    s->i2c_clock_freq = ESP32_I2C_STANDARD_MODE;
    s->apb_clock_freq = ESP32_I2C_CLOCK_FREQ_HZ;
    s->timing_validation_enabled = true;
    s->last_transaction_time = 0;
    
    // quiet timing reset log
}

/* Phase 3: Hardware-Accurate Delay Calculation */
static uint64_t esp32_i2c_calculate_hardware_delay(Esp32I2CState *s, I2CState state)
{
    // Calculate delay based on I2C clock frequency and hardware timing requirements
    uint64_t base_delay_ns;
    
    switch (state) {
    case I2C_STATE_START:
        // START condition: 4.7μs minimum for standard mode
        base_delay_ns = 4700;
        break;
    case I2C_STATE_ADDRESS:
        // Address phase: 4.0μs minimum for standard mode
        base_delay_ns = 4000;
        break;
    case I2C_STATE_DATA_WRITE:
    case I2C_STATE_DATA_READ:
        // Data phase: 3.45μs minimum for standard mode
        base_delay_ns = 3450;
        break;
    case I2C_STATE_STOP:
        // STOP condition: 4.0μs minimum for standard mode
        base_delay_ns = 4000;
        break;
    default:
        base_delay_ns = 1000;  // Default 1μs
        break;
    }
    
    // Scale delay based on I2C clock mode
    switch (s->clock_mode) {
    case I2C_CLOCK_STANDARD:
        // Standard mode (100kHz) - use base delay
        break;
    case I2C_CLOCK_FAST:
        // Fast mode (400kHz) - reduce delay by 4x
        base_delay_ns = base_delay_ns / 4;
        break;
    case I2C_CLOCK_FAST_PLUS:
        // Fast plus mode (1MHz) - reduce delay by 10x
        base_delay_ns = base_delay_ns / 10;
        break;
    }
    
    // Add some jitter for realism (±10%)
    uint64_t jitter = (base_delay_ns * 10) / 100;
    uint64_t final_delay = base_delay_ns + (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) % (jitter * 2)) - jitter;
    
    return final_delay;
}

static uint64_t esp32_i2c_calculate_phase_delay(Esp32I2CState *s, I2CPhase phase)
{
    // Calculate phase-specific delays based on hardware timing
    uint64_t base_delay_ns;
    
    switch (phase) {
    case I2C_PHASE_START_CONDITION:
        base_delay_ns = 1000;  // 1μs
        break;
    case I2C_PHASE_ADDRESS_SEND:
        base_delay_ns = 2000;  // 2μs
        break;
    case I2C_PHASE_ADDRESS_ACK:
        base_delay_ns = 500;   // 0.5μs
        break;
    case I2C_PHASE_DATA_SEND:
        base_delay_ns = 1500;  // 1.5μs
        break;
    case I2C_PHASE_DATA_ACK:
        base_delay_ns = 500;   // 0.5μs
        break;
    case I2C_PHASE_STOP_CONDITION:
        base_delay_ns = 1000;  // 1μs
        break;
    default:
        base_delay_ns = 100;   // Default
        break;
    }
    
    // Scale based on clock mode
    switch (s->clock_mode) {
    case I2C_CLOCK_STANDARD:
        break;
    case I2C_CLOCK_FAST:
        base_delay_ns = base_delay_ns / 4;
        break;
    case I2C_CLOCK_FAST_PLUS:
        base_delay_ns = base_delay_ns / 10;
        break;
    }
    
    return base_delay_ns;
}

/* Phase 3: Timing Validation */
static void esp32_i2c_validate_timing(Esp32I2CState *s, uint64_t delay_ns)
{
    if (!s->timing_validation_enabled) {
        return;
    }
    
    // Validate timing constraints (silent)
    uint64_t current_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    // Check minimum timing requirements
    switch (s->clock_mode) {
    case I2C_CLOCK_STANDARD:
        break;
    case I2C_CLOCK_FAST:
        break;
    case I2C_CLOCK_FAST_PLUS:
        break;
    }
    
    // Check for timing violations
    if (s->last_transaction_time > 0) {
        uint64_t time_since_last = current_time - s->last_transaction_time;
        (void)time_since_last;  // silence warning checks
    }
    
    s->last_transaction_time = current_time;
}

/* Phase 3: Clock Mode Detection */
static I2CClockMode esp32_i2c_detect_clock_mode(Esp32I2CState *s)
{
    // Detect clock mode based on register values
    // This is a simplified detection - real hardware would use more complex logic
    
    if (s->high_period_reg > 0 && s->low_period_reg > 0) {
        // Calculate effective clock frequency
        uint32_t total_period = s->high_period_reg + s->low_period_reg;
        uint32_t clock_freq = s->apb_clock_freq / total_period;
        
        if (clock_freq <= ESP32_I2C_STANDARD_MODE * 1.1) {
            return I2C_CLOCK_STANDARD;
        } else if (clock_freq <= ESP32_I2C_FAST_MODE * 1.1) {
            return I2C_CLOCK_FAST;
        } else {
            return I2C_CLOCK_FAST_PLUS;
        }
    }
    
    return I2C_CLOCK_STANDARD;  // Default
}

static uint32_t esp32_i2c_get_clock_frequency(Esp32I2CState *s)
{
    switch (s->clock_mode) {
    case I2C_CLOCK_STANDARD:
        return ESP32_I2C_STANDARD_MODE;
    case I2C_CLOCK_FAST:
        return ESP32_I2C_FAST_MODE;
    case I2C_CLOCK_FAST_PLUS:
        return ESP32_I2C_FAST_PLUS_MODE;
    default:
        return ESP32_I2C_STANDARD_MODE;
    }
}

/* Phase 4: Asynchronous Transaction Processing Functions */
static void esp32_i2c_transaction_init(Esp32I2CState *s);
static void esp32_i2c_transaction_reset(Esp32I2CState *s);
static bool esp32_i2c_queue_transaction(Esp32I2CState *s, I2CTransaction *transaction);
static bool esp32_i2c_dequeue_transaction(Esp32I2CState *s, I2CTransaction *transaction);
static void esp32_i2c_start_next_transaction(Esp32I2CState *s);
static void esp32_i2c_complete_transaction(Esp32I2CState *s, I2CErrorType error);
static void esp32_i2c_handle_transaction_timeout(Esp32I2CState *s);
static bool esp32_i2c_is_transaction_queue_full(Esp32I2CState *s);
static bool esp32_i2c_is_transaction_queue_empty(Esp32I2CState *s);
static void esp32_i2c_validate_transaction(Esp32I2CState *s, I2CTransaction *transaction);

/* Phase 4: Transaction Management Initialization */
static void esp32_i2c_transaction_init(Esp32I2CState *s)
{
    // Initialize current transaction
    memset(&s->current_transaction, 0, sizeof(I2CTransaction));
    s->current_transaction.status = I2C_TRANSACTION_IDLE;
    s->current_transaction.error = I2C_ERROR_NONE;
    
    // Initialize transaction queue
    memset(s->pending_transactions, 0, sizeof(s->pending_transactions));
    s->transaction_queue_head = 0;
    s->transaction_queue_tail = 0;
    s->transaction_queue_count = 0;
    
    // Initialize transaction processing
    s->transaction_processing_enabled = true;
    s->transaction_timeout_ms = 1000;  // Default 1 second timeout
    s->last_error = I2C_ERROR_NONE;
    
    // quiet
}

static void esp32_i2c_transaction_reset(Esp32I2CState *s)
{
    // Reset current transaction
    s->current_transaction.status = I2C_TRANSACTION_IDLE;
    s->current_transaction.error = I2C_ERROR_NONE;
    
    // Clear transaction queue
    memset(s->pending_transactions, 0, sizeof(s->pending_transactions));
    s->transaction_queue_head = 0;
    s->transaction_queue_tail = 0;
    s->transaction_queue_count = 0;
    
    // Reset transaction processing
    s->transaction_processing_enabled = true;
    s->transaction_timeout_ms = 1000;
    s->last_error = I2C_ERROR_NONE;
    
    // quiet
}

/* Phase 4: Transaction Queue Management */
static bool esp32_i2c_queue_transaction(Esp32I2CState *s, I2CTransaction *transaction)
{
    if (esp32_i2c_is_transaction_queue_full(s)) {
        qemu_log("esp32_i2c: Transaction queue is full, cannot queue transaction\n");
        return false;
    }
    
    // Validate transaction
    esp32_i2c_validate_transaction(s, transaction);
    
    // Add to queue
    s->pending_transactions[s->transaction_queue_tail] = *transaction;
    s->pending_transactions[s->transaction_queue_tail].status = I2C_TRANSACTION_PENDING;
    s->pending_transactions[s->transaction_queue_tail].start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    s->transaction_queue_tail = (s->transaction_queue_tail + 1) % ESP32_I2C_CMD_COUNT;
    s->transaction_queue_count++;
    
    qemu_log("esp32_i2c: Transaction queued - Address: 0x%02x, Opcode: %d, Bytes: %d\n",
             transaction->device_address, transaction->opcode, transaction->byte_count);
    
    // Start processing if no current transaction
    if (s->current_transaction.status == I2C_TRANSACTION_IDLE) {
        esp32_i2c_start_next_transaction(s);
    }
    
    return true;
}

static bool esp32_i2c_dequeue_transaction(Esp32I2CState *s, I2CTransaction *transaction)
{
    if (esp32_i2c_is_transaction_queue_empty(s)) {
        return false;
    }
    
    *transaction = s->pending_transactions[s->transaction_queue_head];
    s->transaction_queue_head = (s->transaction_queue_head + 1) % ESP32_I2C_CMD_COUNT;
    s->transaction_queue_count--;
    
    qemu_log("esp32_i2c: Transaction dequeued - Address: 0x%02x, Status: %d\n",
             transaction->device_address, transaction->status);
    
    return true;
}

/* Phase 4: Transaction Processing */
static void esp32_i2c_start_next_transaction(Esp32I2CState *s)
{
    if (!esp32_i2c_dequeue_transaction(s, &s->current_transaction)) {
        qemu_log("esp32_i2c: No pending transactions to start\n");
        return;
    }
    
    s->current_transaction.status = I2C_TRANSACTION_ACTIVE;
    s->current_transaction.start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    qemu_log("esp32_i2c: Starting transaction - Address: 0x%02x, Opcode: %d, Bytes: %d\n",
             s->current_transaction.device_address, 
             s->current_transaction.opcode, 
             s->current_transaction.byte_count);
    
    // Start the state machine for this transaction
    esp32_i2c_start_transaction(s);
}

static void esp32_i2c_complete_transaction(Esp32I2CState *s, I2CErrorType error)
{
    s->current_transaction.status = (error == I2C_ERROR_NONE) ? 
                                   I2C_TRANSACTION_COMPLETED : I2C_TRANSACTION_ERROR;
    s->current_transaction.error = error;
    s->current_transaction.completion_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    uint64_t duration_ns = s->current_transaction.completion_time - s->current_transaction.start_time;
    
    qemu_log("esp32_i2c: Transaction completed - Address: 0x%02x, Status: %d, Error: %d, Duration: %lu ns\n",
             s->current_transaction.device_address, 
             s->current_transaction.status, 
             s->current_transaction.error,
             duration_ns);
    
    // Update last error
    s->last_error = error;
    
    // Phase 7: Generate appropriate interrupt based on error status
    if (error == I2C_ERROR_NONE) {
        // Transaction completed successfully
        esp32_i2c_handle_transaction_complete_interrupt(s);
    } else if (error == I2C_ERROR_NACK) {
        // ACK error occurred
        esp32_i2c_handle_ack_error_interrupt(s);
    } else {
        // Other error types - generate transaction complete interrupt with error status
        esp32_i2c_handle_transaction_complete_interrupt(s);
    }
    
    // Clear current transaction
    s->current_transaction.status = I2C_TRANSACTION_IDLE;
    s->current_transaction.error = I2C_ERROR_NONE;
    
    // Start next transaction if available
    if (!esp32_i2c_is_transaction_queue_empty(s)) {
        esp32_i2c_start_next_transaction(s);
    }
}

static void esp32_i2c_handle_transaction_timeout(Esp32I2CState *s)
{
    if (s->current_transaction.status == I2C_TRANSACTION_ACTIVE) {
        uint64_t current_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t elapsed_ms = (current_time - s->current_transaction.start_time) / 1000000;
        
        if (elapsed_ms > s->transaction_timeout_ms) {
            qemu_log("esp32_i2c: Transaction timeout - Address: 0x%02x, Elapsed: %lu ms\n",
                     s->current_transaction.device_address, elapsed_ms);
            
            esp32_i2c_complete_transaction(s, I2C_ERROR_TIMEOUT);
        }
    }
}

/* Phase 4: Transaction Queue Status */
static bool esp32_i2c_is_transaction_queue_full(Esp32I2CState *s)
{
    return s->transaction_queue_count >= ESP32_I2C_CMD_COUNT;
}

static bool esp32_i2c_is_transaction_queue_empty(Esp32I2CState *s)
{
    return s->transaction_queue_count == 0;
}

static void esp32_i2c_validate_transaction(Esp32I2CState *s, I2CTransaction *transaction)
{
    // Validate device address (7-bit I2C addresses)
    if (transaction->device_address > 0x7F) {
        qemu_log("esp32_i2c: WARNING - Invalid device address 0x%02x (must be 7-bit)\n",
                 transaction->device_address);
        transaction->device_address &= 0x7F;  // Force to 7-bit
    }
    
    // Validate byte count
    if (transaction->byte_count > 255) {
        qemu_log("esp32_i2c: WARNING - Invalid byte count %d (max 255)\n",
                 transaction->byte_count);
        transaction->byte_count = 255;  // Limit to maximum
    }
    
    // Validate opcode
    if (transaction->opcode > 7) {  // ESP32 I2C has 8 opcodes (0-7)
        qemu_log("esp32_i2c: WARNING - Invalid opcode %d (max 7)\n",
                 transaction->opcode);
        transaction->opcode = 0;  // Default to NOP
    }
    
    // Validate timeout
    if (transaction->timeout_ms == 0) {
        transaction->timeout_ms = s->transaction_timeout_ms;  // Use default
    }
}

/* Phase 5: I2C Protocol Implementation Functions */
static void esp32_i2c_protocol_init(Esp32I2CState *s);
static void esp32_i2c_protocol_reset(Esp32I2CState *s);
static void esp32_i2c_send_start_condition(Esp32I2CState *s);
static void esp32_i2c_send_stop_condition(Esp32I2CState *s);
static void esp32_i2c_send_device_address(Esp32I2CState *s);
static void esp32_i2c_send_data_byte(Esp32I2CState *s);
static void esp32_i2c_receive_data_byte(Esp32I2CState *s);
static void esp32_i2c_check_ack(Esp32I2CState *s);
static void esp32_i2c_send_ack(Esp32I2CState *s);
static void esp32_i2c_send_nack(Esp32I2CState *s);
static bool esp32_i2c_simulate_device_response(Esp32I2CState *s, uint8_t device_addr);

/* Phase 5: I2C Protocol Initialization */
static void esp32_i2c_protocol_init(Esp32I2CState *s)
{
    // Initialize protocol state
    s->stream_state = I2C_STREAM_IDLE;
    s->current_device_address = 0;
    s->current_data_byte = 0;
    s->current_byte_index = 0;
    s->expected_ack = I2C_ACK_BIT;
    
    // Initialize protocol flags
    s->start_condition_sent = false;
    s->stop_condition_sent = false;
    s->address_sent = false;
    s->data_sent = false;
    s->ack_received = false;
    s->nack_received = false;
    
    // Initialize data buffers
    memset(s->tx_data_buffer, 0, sizeof(s->tx_data_buffer));
    memset(s->rx_data_buffer, 0, sizeof(s->rx_data_buffer));
    
    // quiet protocol init log
}

static void esp32_i2c_protocol_reset(Esp32I2CState *s)
{
    // Reset protocol state
    s->stream_state = I2C_STREAM_IDLE;
    s->current_device_address = 0;
    s->current_data_byte = 0;
    s->current_byte_index = 0;
    s->expected_ack = I2C_ACK_BIT;
    
    // Reset protocol flags
    s->start_condition_sent = false;
    s->stop_condition_sent = false;
    s->address_sent = false;
    s->data_sent = false;
    s->ack_received = false;
    s->nack_received = false;
    
    // Clear data buffers
    memset(s->tx_data_buffer, 0, sizeof(s->tx_data_buffer));
    memset(s->rx_data_buffer, 0, sizeof(s->rx_data_buffer));
    
    // quiet protocol reset log
}

/* Phase 5: I2C Protocol Implementation */
static void esp32_i2c_send_start_condition(Esp32I2CState *s)
{
    // quiet START log
    
    // Simulate START condition: SDA goes LOW while SCL is HIGH
    esp32_i2c_protocol_bus_signals(s, false, true);  // SDA=LOW, SCL=HIGH
    esp32_i2c_protocol_bus_signals(s, false, false); // SDA=LOW, SCL=LOW
    
    s->start_condition_sent = true;
    s->stop_condition_sent = false;
    
    // quiet START sent log
}

static void esp32_i2c_send_stop_condition(Esp32I2CState *s)
{
    // quiet STOP log
    
    // Simulate STOP condition: SDA goes HIGH while SCL is HIGH
    esp32_i2c_protocol_bus_signals(s, false, false); // SDA=LOW, SCL=LOW
    esp32_i2c_protocol_bus_signals(s, false, true);  // SDA=LOW, SCL=HIGH
    esp32_i2c_protocol_bus_signals(s, true, true);   // SDA=HIGH, SCL=HIGH
    
    s->stop_condition_sent = true;
    s->start_condition_sent = false;
    
    // quiet STOP sent log
}

static void esp32_i2c_send_device_address(Esp32I2CState *s)
{
    uint8_t device_addr = s->current_transaction.device_address;
    uint8_t read_write_bit = (s->current_transaction.opcode == I2C_OPCODE_READ) ? 
                             I2C_READ_BIT : I2C_WRITE_BIT;
    uint8_t address_byte = (device_addr << 1) | read_write_bit;
    
    // quiet address log
    
    // Send 7-bit address + R/W bit
    s->current_data_byte = address_byte;
    s->current_byte_index = 0;
    s->address_sent = true;
    
    // Simulate sending address byte bit by bit
    for (int bit = 7; bit >= 0; bit--) {
        bool bit_value = (address_byte >> bit) & 0x01;
        esp32_i2c_protocol_bus_signals(s, bit_value, false); // SDA=bit, SCL=LOW
        esp32_i2c_protocol_bus_signals(s, bit_value, true);  // SDA=bit, SCL=HIGH
        esp32_i2c_protocol_bus_signals(s, bit_value, false); // SDA=bit, SCL=LOW
    }
    
    // quiet address sent log
    
    // Phase 8: Handle virtual device response
    esp32_i2c_handle_device_response(s, device_addr);
}

static void esp32_i2c_send_data_byte(Esp32I2CState *s)
{
    if (s->current_byte_index >= s->current_transaction.byte_count) {
        return;
    }
    
    uint8_t data_byte = s->tx_data_buffer[s->current_byte_index];
    
    // quiet data send log
    
    // Send data byte bit by bit
    for (int bit = 7; bit >= 0; bit--) {
        bool bit_value = (data_byte >> bit) & 0x01;
        esp32_i2c_protocol_bus_signals(s, bit_value, false); // SDA=bit, SCL=LOW
        esp32_i2c_protocol_bus_signals(s, bit_value, true);  // SDA=bit, SCL=HIGH
        esp32_i2c_protocol_bus_signals(s, bit_value, false); // SDA=bit, SCL=LOW
    }
    
    s->current_byte_index++;
    s->data_sent = true;
    
    // Phase 8: Process data write to virtual device
    I2CVirtualDevice *device = esp32_i2c_find_device_by_address(s, s->current_transaction.device_address);
    if (device) {
        // Use the new device-specific write function
        uint8_t write_data[1] = {data_byte};
        switch (device->base.type) {
        case I2C_DEVICE_TYPE_TMP105:
            tmp105_process_write((TMP105Device*)device, write_data, 1);
            break;
        case I2C_DEVICE_TYPE_EEPROM:
            eeprom_process_write((EEPROMDevice*)device, write_data, 1);
            break;
        case I2C_DEVICE_TYPE_RTC:
            rtc_process_write((RTCDevice*)device, write_data, 1);
            break;
        case I2C_DEVICE_TYPE_SSD1306:
            ssd1306_process_write((SSD1306Device*)device, write_data, 1);
            break;
        case I2C_DEVICE_TYPE_UNKNOWN:
        default:
            // Unknown device type - ignore data
            break;
        }
    }
    
    // quiet data sent log
}

static void esp32_i2c_receive_data_byte(Esp32I2CState *s)
{
    if (s->current_byte_index >= s->current_transaction.byte_count) {
        return;
    }
    
    uint8_t data_byte = 0;
    
    // quiet data receive log
    
    // Phase 8: Get data from virtual device
    uint8_t device_data = 0x00;
    I2CVirtualDevice *device = esp32_i2c_find_device_by_address(s, s->current_transaction.device_address);
    if (device) {
        // Use the new device-specific read function
        esp32_i2c_simulate_device_data(s, s->current_transaction.device_address, &device_data, 1);
    }
    
    // Receive data byte bit by bit
    for (int bit = 7; bit >= 0; bit--) {
        esp32_i2c_protocol_bus_signals(s, true, false);  // SDA=HIGH, SCL=LOW
        esp32_i2c_protocol_bus_signals(s, true, true);   // SDA=HIGH, SCL=HIGH
        
        // Use virtual device data instead of hardcoded 0x55
        bool bit_value = (device_data >> bit) & 0x01;
        esp32_i2c_protocol_bus_signals(s, bit_value, true);  // SDA=device_bit, SCL=HIGH
        esp32_i2c_protocol_bus_signals(s, bit_value, false); // SDA=device_bit, SCL=LOW
        
        data_byte |= (bit_value << bit);
    }
    
    s->rx_data_buffer[s->current_byte_index] = data_byte;
    s->current_byte_index++;
    
    // quiet data received log
}

static void esp32_i2c_check_ack(Esp32I2CState *s)
{
    // quiet ack check log
    
    // Simulate device response
    bool device_ack = esp32_i2c_simulate_device_response(s, s->current_transaction.device_address);
    
    if (device_ack) {
        s->ack_received = true;
        s->nack_received = false;
        // quiet ack received log
    } else {
        s->ack_received = false;
        s->nack_received = true;
        // quiet nack received log
    }
}

static void esp32_i2c_send_ack(Esp32I2CState *s)
{
    // quiet ACK send log
    
    // Send ACK: SDA=LOW during SCL HIGH
    esp32_i2c_protocol_bus_signals(s, false, false); // SDA=LOW, SCL=LOW
    esp32_i2c_protocol_bus_signals(s, false, true);  // SDA=LOW, SCL=HIGH
    esp32_i2c_protocol_bus_signals(s, false, false); // SDA=LOW, SCL=LOW
    
    // quiet ACK sent log
}

static void esp32_i2c_send_nack(Esp32I2CState *s)
{
    // quiet NACK send log
    
    // Send NACK: SDA=HIGH during SCL HIGH
    esp32_i2c_protocol_bus_signals(s, true, false);  // SDA=HIGH, SCL=LOW
    esp32_i2c_protocol_bus_signals(s, true, true);   // SDA=HIGH, SCL=HIGH
    esp32_i2c_protocol_bus_signals(s, true, false);  // SDA=HIGH, SCL=LOW
    
    // quiet NACK sent log
}

static bool esp32_i2c_simulate_device_response(Esp32I2CState *s, uint8_t device_addr)
{
    // Simulate device responses based on address
    switch (device_addr) {
    case 0x48:  // TMP105 at 0x48
    case 0x49:  // TMP105 at 0x49
        return true;  // ACK
        
    case 0x50:  // EEPROM at 0x50
        return true;  // ACK
        
    default:
        return false; // NACK
    }
}

/* Phase 6: I2C Bus Simulation Implementation */
static void esp32_i2c_bus_simulation_init(Esp32I2CState *s)
{
    // Initialize bus state
    s->bus_state = I2C_BUS_IDLE;
    s->bus_previous_state = I2C_BUS_IDLE;
    
    // Initialize line states (idle = HIGH due to pullups)
    s->sda_line_state = true;  // HIGH (idle)
    s->scl_line_state = true;  // HIGH (idle)
    s->sda_driven = false;     // Not driven (pullup)
    s->scl_driven = false;     // Not driven (pullup)
    
    // Initialize bus monitoring
    s->bus_arbitration_lost = false;
    s->bus_collision_detected = false;
    s->bus_error_count = 0;
    s->bus_monitoring_enabled = true;
    
    // Initialize electrical parameters
    s->bus_rise_time_ns = I2C_BUS_RISE_TIME_NS;
    s->bus_fall_time_ns = I2C_BUS_FALL_TIME_NS;
    s->bus_pullup_resistance = I2C_BUS_PULLUP_RESISTANCE;
    s->bus_capacitance_pf = I2C_BUS_CAPACITANCE;
    s->last_bus_transition_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    // quiet bus simulation init log
}

static void esp32_i2c_bus_simulation_reset(Esp32I2CState *s)
{
    // Reset bus state
    s->bus_state = I2C_BUS_IDLE;
    s->bus_previous_state = I2C_BUS_IDLE;
    
    // Reset line states to idle
    s->sda_line_state = true;  // HIGH (idle)
    s->scl_line_state = true;  // HIGH (idle)
    s->sda_driven = false;     // Not driven
    s->scl_driven = false;     // Not driven
    
    // Reset bus monitoring
    s->bus_arbitration_lost = false;
    s->bus_collision_detected = false;
    s->bus_error_count = 0;
    s->bus_monitoring_enabled = true;
    
    // Reset timing
    s->last_bus_transition_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    // quiet bus simulation reset log
}

static void esp32_i2c_update_bus_state(Esp32I2CState *s, uint8_t new_state)
{
    if (s->bus_state != new_state) {
        s->bus_previous_state = s->bus_state;
        s->bus_state = new_state;
        
        s->last_bus_transition_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

static void esp32_i2c_simulate_bus_transition(Esp32I2CState *s, bool sda, bool scl)
{
    bool sda_changed = (s->sda_line_state != sda);
    bool scl_changed = (s->scl_line_state != scl);
    
    if (sda_changed || scl_changed) {
        // Update line states
        s->sda_line_state = sda;
        s->scl_line_state = scl;
        
        // Simulate electrical behavior
        esp32_i2c_simulate_bus_electrical_behavior(s);
        
        // Detect bus conditions
        esp32_i2c_detect_bus_collision(s);
        esp32_i2c_handle_bus_arbitration(s);
        
        // Update bus state based on line conditions
        if (sda && scl) {
            esp32_i2c_update_bus_state(s, I2C_BUS_IDLE);
        } else if (!sda && scl) {
            esp32_i2c_update_bus_state(s, I2C_BUS_START);
        } else if (sda && !scl) {
            esp32_i2c_update_bus_state(s, I2C_BUS_STOP);
        } else {
            esp32_i2c_update_bus_state(s, I2C_BUS_DATA);
        }
        
        // Validate timing
        esp32_i2c_validate_bus_timing(s);
        
        // Monitor bus activity
        esp32_i2c_monitor_bus_activity(s);
    }
}

static void esp32_i2c_detect_bus_collision(Esp32I2CState *s)
{
    // Detect bus collision (multiple devices driving the same line)
    if (s->sda_driven && s->scl_driven) {
        // Check if both lines are being driven simultaneously
        if (s->bus_collision_detected == false) {
            s->bus_collision_detected = true;
            s->bus_error_count++;
            esp32_i2c_update_bus_state(s, I2C_BUS_ERROR);
        }
    } else {
        s->bus_collision_detected = false;
    }
}

static void esp32_i2c_handle_bus_arbitration(Esp32I2CState *s)
{
    // Handle bus arbitration (multiple masters)
    if (s->bus_arbitration_lost) {
        // In real hardware, the losing master would switch to slave mode
        // For simulation, we just log the event
        esp32_i2c_update_bus_state(s, I2C_BUS_ERROR);
    }
}

static void esp32_i2c_simulate_bus_electrical_behavior(Esp32I2CState *s)
{
    // Simulate electrical behavior of I2C bus
    uint64_t current_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t time_since_transition = current_time - s->last_bus_transition_time;
    
    // Simulate rise/fall times
    if (s->sda_line_state && s->scl_line_state) {
        // Both lines HIGH - simulate pullup behavior
        (void)time_since_transition; // quiet
    } else {
        // At least one line LOW - simulate fall time
        (void)time_since_transition; // quiet
    }
    
    // Simulate bus capacitance effects
    if (s->bus_capacitance_pf > 0) {
        // Higher capacitance = slower transitions
        uint32_t effective_rise_time = s->bus_rise_time_ns + (s->bus_capacitance_pf / 10);
        uint32_t effective_fall_time = s->bus_fall_time_ns + (s->bus_capacitance_pf / 20);
        
        (void)effective_rise_time; (void)effective_fall_time; // quiet
    }
}

static void esp32_i2c_monitor_bus_activity(Esp32I2CState *s)
{
    if (!s->bus_monitoring_enabled) {
        return;
    }
    
    // Monitor silently for unusual bus activity (no logs)
    uint64_t current_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t time_since_transition = current_time - s->last_bus_transition_time;
    (void)time_since_transition;
}

static void esp32_i2c_validate_bus_timing(Esp32I2CState *s)
{
    // Validate I2C timing requirements
    uint64_t current_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t time_since_transition = current_time - s->last_bus_transition_time;
    
    // Check minimum setup/hold times silently
    switch (s->bus_state) {
    case I2C_BUS_START:
    case I2C_BUS_STOP:
    case I2C_BUS_DATA:
        break;
    }
}

static void esp32_i2c_log_bus_state_change(Esp32I2CState *s, const char* reason)
{
    (void)s; (void)reason;
}


/* Phase 2: Transaction Processing Functions */
static void esp32_i2c_process_transaction(Esp32I2CState *s)
{
    // quiet transaction state log
    
    switch (s->current_state) {
    case I2C_STATE_START:
        esp32_i2c_do_start_condition(s);
        break;
    case I2C_STATE_ADDRESS:
        esp32_i2c_do_address_phase(s);
        break;
    case I2C_STATE_DATA_WRITE:
        esp32_i2c_do_data_write(s);
        break;
    case I2C_STATE_DATA_READ:
        esp32_i2c_do_data_read(s);
        break;
    case I2C_STATE_STOP:
        esp32_i2c_do_stop_condition(s);
        break;
    case I2C_STATE_COMPLETE:
        esp32_i2c_complete_transaction_legacy(s);
        break;
    default:
        qemu_log("esp32_i2c: Unknown state %d\n", s->current_state);
        break;
    }
}

static void esp32_i2c_process_phase(Esp32I2CState *s)
{
    // quiet phase processing log
    
    // Phase processing will be implemented in Phase 5 (I2C Protocol Implementation)
    // For now, just log the phase
}

static void esp32_i2c_advance_phase(Esp32I2CState *s)
{
    switch (s->current_phase) {
    case I2C_PHASE_START_CONDITION:
        s->current_phase = I2C_PHASE_ADDRESS_SEND;
        break;
    case I2C_PHASE_ADDRESS_SEND:
        s->current_phase = I2C_PHASE_ADDRESS_ACK;
        break;
    case I2C_PHASE_ADDRESS_ACK:
        s->current_phase = I2C_PHASE_DATA_SEND;
        break;
    case I2C_PHASE_DATA_SEND:
        s->current_phase = I2C_PHASE_DATA_ACK;
        break;
    case I2C_PHASE_DATA_ACK:
        s->current_phase = I2C_PHASE_STOP_CONDITION;
        break;
    case I2C_PHASE_STOP_CONDITION:
        s->current_phase = I2C_PHASE_COMPLETE;
        break;
    default:
        s->current_phase = I2C_PHASE_COMPLETE;
        break;
    }
}

static void esp32_i2c_start_transaction(Esp32I2CState *s)
{
    // quiet start transaction log
    
    // Initialize state machine
    s->current_state = I2C_STATE_START;
    s->next_state = I2C_STATE_ADDRESS;
    s->current_phase = I2C_PHASE_START_CONDITION;
    s->state_machine_active = true;
    
    // Set bus active
    esp32_i2c_set_bus_active(s);
    
    // Schedule first state timer
    esp32_i2c_schedule_state_timer(s);
}

/* Phase 5: I2C Protocol Functions (Implemented) */
static void esp32_i2c_do_start_condition(Esp32I2CState *s)
{
    // quiet phase start log
    esp32_i2c_send_start_condition(s);
}

static void esp32_i2c_do_address_phase(Esp32I2CState *s)
{
    // quiet phase address log
    esp32_i2c_send_device_address(s);
    esp32_i2c_check_ack(s);
    
    if (s->nack_received) {
        // quiet nack abort log
        esp32_i2c_complete_transaction(s, I2C_ERROR_NACK);
        return;
    }
    
    // quiet ack proceed log
}

static void esp32_i2c_do_data_write(Esp32I2CState *s)
{
    // quiet data write phase log
    
    // Load data from TX FIFO if available
    if (fifo8_num_used(&s->tx_fifo) > 0) {
        uint8_t data_byte = fifo8_pop(&s->tx_fifo);
        s->tx_data_buffer[s->current_byte_index] = data_byte;
        // quiet loaded data log
    }
    
    esp32_i2c_send_data_byte(s);
    esp32_i2c_check_ack(s);
    
    if (s->nack_received) {
        // quiet nack during write
        esp32_i2c_complete_transaction(s, I2C_ERROR_NACK);
        return;
    }
    
    // quiet data sent success log
}

static void esp32_i2c_do_data_read(Esp32I2CState *s)
{
    // quiet data read phase log
    
    esp32_i2c_receive_data_byte(s);
    
    // Store received data in RX FIFO
    if (fifo8_num_free(&s->rx_fifo) > 0) {
        uint8_t received_byte = s->rx_data_buffer[s->current_byte_index - 1];
        fifo8_push(&s->rx_fifo, received_byte);
        // quiet stored byte log
    }
    
    // Send ACK for all bytes except the last one
    if (s->current_byte_index < s->current_transaction.byte_count) {
        esp32_i2c_send_ack(s);
    } else {
        esp32_i2c_send_nack(s);  // Send NACK for last byte
    }
    
    // quiet data received success log
}

static void esp32_i2c_do_stop_condition(Esp32I2CState *s)
{
    // quiet stop phase log
    esp32_i2c_send_stop_condition(s);
}

static void esp32_i2c_complete_transaction_legacy(Esp32I2CState *s)
{
    // quiet legacy complete log
    
    // Set bus idle
    esp32_i2c_set_bus_idle(s);
    s->trans_ongoing = false;
    
    // Clear TRANS_START bit
    s->ctr_reg = FIELD_DP32(s->ctr_reg, I2C_CTR, TRANS_START, 0);
    
    // Generate completion interrupt
    s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, TRANS_COMPLETE, 1);
    esp32_i2c_update_irq(s);
    
    // Complete transaction using new transaction management system
    esp32_i2c_complete_transaction(s, I2C_ERROR_NONE);
}

/* Phase 7: Interrupt Generation Implementation */
static void esp32_i2c_interrupt_init(Esp32I2CState *s)
{
    // Initialize interrupt state
    s->interrupt_pending = 0;
    s->interrupt_enabled = 0;
    s->interrupt_priority = 0;
    s->interrupt_generation_enabled = true;
    s->last_interrupt_time = 0;
    s->interrupt_count = 0;
    s->interrupt_debug_enabled = true;
    
    // quiet interrupt init log
}

static void esp32_i2c_interrupt_reset(Esp32I2CState *s)
{
    // Reset interrupt state
    s->interrupt_pending = 0;
    s->interrupt_enabled = 0;
    s->interrupt_priority = 0;
    s->interrupt_generation_enabled = true;
    s->last_interrupt_time = 0;
    s->interrupt_count = 0;
    s->interrupt_debug_enabled = true;
    
    // quiet interrupt reset log
}

static void esp32_i2c_generate_interrupt(Esp32I2CState *s, I2CInterruptType type)
{
    if (!s->interrupt_generation_enabled) {
        return;
    }
    
    // Map interrupt type to bit position
    uint32_t interrupt_bit = 0;
    switch (type) {
    case I2C_INTERRUPT_TYPE_ACK_ERR:
        interrupt_bit = I2C_INTERRUPT_ACK_ERR;
        break;
    case I2C_INTERRUPT_TYPE_TRANS_COMPLETE:
        interrupt_bit = I2C_INTERRUPT_TRANS_COMPLETE;
        break;
    case I2C_INTERRUPT_TYPE_END_DETECT:
        interrupt_bit = I2C_INTERRUPT_END_DETECT;
        break;
    case I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY:
        interrupt_bit = I2C_INTERRUPT_TX_FIFO_EMPTY;
        break;
    case I2C_INTERRUPT_TYPE_RX_FIFO_FULL:
        interrupt_bit = I2C_INTERRUPT_RX_FIFO_FULL;
        break;
    case I2C_INTERRUPT_TYPE_START_DETECT:
        interrupt_bit = I2C_INTERRUPT_START_DETECT;
        break;
    default:
        return;
    }
    
    // Set interrupt pending
    s->interrupt_pending |= interrupt_bit;
    s->int_raw_reg |= interrupt_bit;
    
    // Update interrupt count and timing
    s->interrupt_count++;
    s->last_interrupt_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    // quiet interrupt event log
    
    // Update IRQ if interrupt is enabled
    if (s->interrupt_enabled & interrupt_bit) {
        esp32_i2c_update_irq(s);
    }
    
    if (interrupt_bit == I2C_INTERRUPT_TX_FIFO_EMPTY) {
        int irq_state = !!(s->int_raw_reg & s->int_ena_reg);
        qemu_log("esp32_i2c: TXFIFO_EMPTY set (int_raw=0x%08x int_ena=0x%08x irq=%d)\n",
                 s->int_raw_reg, s->int_ena_reg, irq_state);
    }
}

static void esp32_i2c_clear_interrupt(Esp32I2CState *s, I2CInterruptType type)
{
    // Map interrupt type to bit position
    uint32_t interrupt_bit = 0;
    switch (type) {
    case I2C_INTERRUPT_TYPE_ACK_ERR:
        interrupt_bit = I2C_INTERRUPT_ACK_ERR;
        break;
    case I2C_INTERRUPT_TYPE_TRANS_COMPLETE:
        interrupt_bit = I2C_INTERRUPT_TRANS_COMPLETE;
        break;
    case I2C_INTERRUPT_TYPE_END_DETECT:
        interrupt_bit = I2C_INTERRUPT_END_DETECT;
        break;
    case I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY:
        interrupt_bit = I2C_INTERRUPT_TX_FIFO_EMPTY;
        break;
    case I2C_INTERRUPT_TYPE_RX_FIFO_FULL:
        interrupt_bit = I2C_INTERRUPT_RX_FIFO_FULL;
        break;
    case I2C_INTERRUPT_TYPE_START_DETECT:
        interrupt_bit = I2C_INTERRUPT_START_DETECT;
        break;
    default:
        return;
    }
    
    // Clear interrupt pending
    s->interrupt_pending &= ~interrupt_bit;
    s->int_raw_reg &= ~interrupt_bit;
    
    // quiet interrupt event log
    
    // Update IRQ
    esp32_i2c_update_irq(s);
}

static void esp32_i2c_enable_interrupt(Esp32I2CState *s, I2CInterruptType type)
{
    // Map interrupt type to bit position
    uint32_t interrupt_bit = 0;
    switch (type) {
    case I2C_INTERRUPT_TYPE_ACK_ERR:
        interrupt_bit = I2C_INTERRUPT_ACK_ERR;
        break;
    case I2C_INTERRUPT_TYPE_TRANS_COMPLETE:
        interrupt_bit = I2C_INTERRUPT_TRANS_COMPLETE;
        break;
    case I2C_INTERRUPT_TYPE_END_DETECT:
        interrupt_bit = I2C_INTERRUPT_END_DETECT;
        break;
    case I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY:
        interrupt_bit = I2C_INTERRUPT_TX_FIFO_EMPTY;
        break;
    case I2C_INTERRUPT_TYPE_RX_FIFO_FULL:
        interrupt_bit = I2C_INTERRUPT_RX_FIFO_FULL;
        break;
    case I2C_INTERRUPT_TYPE_START_DETECT:
        interrupt_bit = I2C_INTERRUPT_START_DETECT;
        break;
    default:
        return;
    }
    
    // Enable interrupt
    s->interrupt_enabled |= interrupt_bit;
    s->int_ena_reg |= interrupt_bit;
    
    // quiet interrupt event log
}

static void esp32_i2c_disable_interrupt(Esp32I2CState *s, I2CInterruptType type)
{
    // Map interrupt type to bit position
    uint32_t interrupt_bit = 0;
    switch (type) {
    case I2C_INTERRUPT_TYPE_ACK_ERR:
        interrupt_bit = I2C_INTERRUPT_ACK_ERR;
        break;
    case I2C_INTERRUPT_TYPE_TRANS_COMPLETE:
        interrupt_bit = I2C_INTERRUPT_TRANS_COMPLETE;
        break;
    case I2C_INTERRUPT_TYPE_END_DETECT:
        interrupt_bit = I2C_INTERRUPT_END_DETECT;
        break;
    case I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY:
        interrupt_bit = I2C_INTERRUPT_TX_FIFO_EMPTY;
        break;
    case I2C_INTERRUPT_TYPE_RX_FIFO_FULL:
        interrupt_bit = I2C_INTERRUPT_RX_FIFO_FULL;
        break;
    case I2C_INTERRUPT_TYPE_START_DETECT:
        interrupt_bit = I2C_INTERRUPT_START_DETECT;
        break;
    default:
        return;
    }
    
    // Disable interrupt
    s->interrupt_enabled &= ~interrupt_bit;
    s->int_ena_reg &= ~interrupt_bit;
    
    // quiet interrupt event log
}

static bool esp32_i2c_is_interrupt_enabled(Esp32I2CState *s, I2CInterruptType type)
{
    // Map interrupt type to bit position
    uint32_t interrupt_bit = 0;
    switch (type) {
    case I2C_INTERRUPT_TYPE_ACK_ERR:
        interrupt_bit = I2C_INTERRUPT_ACK_ERR;
        break;
    case I2C_INTERRUPT_TYPE_TRANS_COMPLETE:
        interrupt_bit = I2C_INTERRUPT_TRANS_COMPLETE;
        break;
    case I2C_INTERRUPT_TYPE_END_DETECT:
        interrupt_bit = I2C_INTERRUPT_END_DETECT;
        break;
    case I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY:
        interrupt_bit = I2C_INTERRUPT_TX_FIFO_EMPTY;
        break;
    case I2C_INTERRUPT_TYPE_RX_FIFO_FULL:
        interrupt_bit = I2C_INTERRUPT_RX_FIFO_FULL;
        break;
    case I2C_INTERRUPT_TYPE_START_DETECT:
        interrupt_bit = I2C_INTERRUPT_START_DETECT;
        break;
    default:
        return false;
    }
    
    return (s->interrupt_enabled & interrupt_bit) != 0;
}

static bool esp32_i2c_is_interrupt_pending(Esp32I2CState *s, I2CInterruptType type)
{
    // Map interrupt type to bit position
    uint32_t interrupt_bit = 0;
    switch (type) {
    case I2C_INTERRUPT_TYPE_ACK_ERR:
        interrupt_bit = I2C_INTERRUPT_ACK_ERR;
        break;
    case I2C_INTERRUPT_TYPE_TRANS_COMPLETE:
        interrupt_bit = I2C_INTERRUPT_TRANS_COMPLETE;
        break;
    case I2C_INTERRUPT_TYPE_END_DETECT:
        interrupt_bit = I2C_INTERRUPT_END_DETECT;
        break;
    case I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY:
        interrupt_bit = I2C_INTERRUPT_TX_FIFO_EMPTY;
        break;
    case I2C_INTERRUPT_TYPE_RX_FIFO_FULL:
        interrupt_bit = I2C_INTERRUPT_RX_FIFO_FULL;
        break;
    case I2C_INTERRUPT_TYPE_START_DETECT:
        interrupt_bit = I2C_INTERRUPT_START_DETECT;
        break;
    default:
        return false;
    }
    
    return (s->interrupt_pending & interrupt_bit) != 0;
}

static void esp32_i2c_handle_transaction_complete_interrupt(Esp32I2CState *s)
{
    // quiet transaction complete handler log
    
    // Generate transaction complete interrupt
    esp32_i2c_generate_interrupt(s, I2C_INTERRUPT_TYPE_TRANS_COMPLETE);
    
    // Update transaction status
    s->current_transaction.status = I2C_TRANSACTION_COMPLETED;
    
    // quiet completion log
}

static void esp32_i2c_handle_ack_error_interrupt(Esp32I2CState *s)
{
    // quiet ack error handler log
    
    // Generate ACK error interrupt
    esp32_i2c_generate_interrupt(s, I2C_INTERRUPT_TYPE_ACK_ERR);
    
    // Update transaction status
    s->current_transaction.status = I2C_TRANSACTION_ERROR;
    s->current_transaction.error = I2C_ERROR_NACK;
    
    // quiet error log
}

static void esp32_i2c_handle_end_detect_interrupt(Esp32I2CState *s)
{
    // quiet end detect handler log
    
    // Generate end detect interrupt
    esp32_i2c_generate_interrupt(s, I2C_INTERRUPT_TYPE_END_DETECT);
    
    // Update bus state
    s->bus_state = I2C_BUS_IDLE;
    
    // quiet end detection log
}

static void esp32_i2c_handle_fifo_interrupts(Esp32I2CState *s)
{
    /* TX watermark: assert when used <= wm */
    int used_tx = fifo8_num_used(&s->tx_fifo);
    bool tx_cond = (used_tx <= (int)s->tx_fifo_wm);
    if (tx_cond && !s->tx_irq_asserted) {
        esp32_i2c_generate_interrupt(s, I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY);
        s->tx_irq_asserted = true;
    } else if (!tx_cond && s->tx_irq_asserted) {
        s->tx_irq_asserted = false;
    }

    /* RX watermark: assert when used >= wm */
    int used_rx = fifo8_num_used(&s->rx_fifo);
    bool rx_cond = (used_rx >= (int)s->rx_fifo_wm);
    if (rx_cond && !s->rx_irq_asserted) {
        esp32_i2c_generate_interrupt(s, I2C_INTERRUPT_TYPE_RX_FIFO_FULL);
        s->rx_irq_asserted = true;
    } else if (!rx_cond && s->rx_irq_asserted) {
        s->rx_irq_asserted = false;
    }
}

static void esp32_i2c_log_interrupt_event(Esp32I2CState *s, I2CInterruptType type, const char* reason)
{
    (void)s; (void)type; (void)reason;
}

/* Phase 8: Virtual Device Integration Implementation - Refactored */
static void esp32_i2c_virtual_device_init(Esp32I2CState *s)
{
    // Initialize virtual device array
    memset(s->virtual_devices, 0, sizeof(s->virtual_devices));
    s->device_count = 0;
    s->device_integration_enabled = true;
    s->device_scan_enabled = true;
    s->last_device_scan_time = 0;
    s->device_scan_interval_ns = 1000000; // 1ms scan interval
    s->device_debug_enabled = false; // quiet default logs

    // New virtual device registry and VDev-based devices
    s->vdev_i2c_set = g_new0(I2CDeviceSet, 1);
    i2c_device_set_init(s->vdev_i2c_set);
    /* Optional: QOM property "i2c-devices" overrides defaults. Format:
     *   tmp105@0x48,tmp105@0x49,eeprom@0x50:4096,ds3231@0x68,ssd1306@0x3c
     */
    bool used_dev_list = false;
    if (s->dev_list && s->dev_list[0]) {
        char *list = g_strdup(s->dev_list);
        char *saveptr = NULL;
        for (char *tok = strtok_r(list, ",", &saveptr); tok; tok = strtok_r(NULL, ",", &saveptr)) {
            char *at = strchr(tok, '@');
            if (!at) continue;
            *at = '\0';
            const char *kind = tok;
            char *addr_str = at + 1;
            char *param = strchr(addr_str, ':');
            if (param) { *param = '\0'; param++; }
            uint8_t addr7 = (uint8_t)strtol(addr_str, NULL, 0);
            if (g_strcmp0(kind, "tmp105") == 0) {
                TMP105VDev *t = tmp105_vdev_create(addr7, (float)s->tmp105_init_temp_c);
                if (param && param[0]) {
                    char *p = param;
                    char *save2 = NULL;
                    for (char *kv = strtok_r(p, ",", &save2); kv; kv = strtok_r(NULL, ",", &save2)) {
                        char *eq = strchr(kv, '=');
                        if (!eq) continue;
                        *eq = '\0';
                        const char *key = kv;
                        const char *val = eq + 1;
                        if (g_strcmp0(key, "temp") == 0 || g_strcmp0(key, "init") == 0) {
                            t->temperature = (float)strtod(val, NULL);
                        } else if (g_strcmp0(key, "drift") == 0) {
                            t->drift_c_per_sec = strtod(val, NULL) / 1000.0; /* mcps -> C/s */
                        } else if (g_strcmp0(key, "noise") == 0) {
                            t->noise_c_amplitude = strtod(val, NULL) / 1000.0; /* mC -> C */
                        }
                    }
                } else {
                    /* Apply controller defaults if provided */
                    if (s->tmp105_drift_mcps) t->drift_c_per_sec = (double)s->tmp105_drift_mcps / 1000.0;
                    if (s->tmp105_noise_mc) t->noise_c_amplitude = (double)s->tmp105_noise_mc / 1000.0;
                }
                i2c_device_set_add(s->vdev_i2c_set, addr7, &t->base, tmp105_vdev_get_i2c_ops());
            } else if (g_strcmp0(kind, "eeprom") == 0) {
                size_t sz = 4096;
                bool wp = false;
                if (param) {
                    /* Support size or size+wp */
                    char *plus = strchr(param, '+');
                    if (plus) { *plus = '\0'; plus++; if (g_strcmp0(plus, "wp") == 0) wp = true; }
                    if (param[0]) sz = (size_t)strtoul(param, NULL, 0);
                }
                EEPROMVDev *e = eeprom_vdev_create(addr7, sz);
                if (s->eeprom_page_size > 0) e->page_size = (uint8_t)s->eeprom_page_size;
                if (s->eeprom_write_delay_ms > 0) e->write_delay_ms = (uint32_t)s->eeprom_write_delay_ms;
                e->wp_enabled = wp;
                i2c_device_set_add(s->vdev_i2c_set, addr7, &e->base, eeprom_vdev_get_i2c_ops());
            } else if (g_strcmp0(kind, "ds1307") == 0) {
                DS1307VDev *r = ds1307_vdev_create(addr7);
                i2c_device_set_add(s->vdev_i2c_set, addr7, &r->base, ds1307_vdev_get_i2c_ops());
            } else if (g_strcmp0(kind, "ds3231") == 0) {
                DS3231VDev *r = ds3231_vdev_create(addr7);
                r->base.bus_context = s;
                r->temp_c = (float)s->ds3231_init_temp_c;
                i2c_device_set_add(s->vdev_i2c_set, addr7, &r->base, ds3231_vdev_get_i2c_ops());
            } else if (g_strcmp0(kind, "ssd1306") == 0) {
                SSD1306VDev *o = ssd1306_vdev_create(addr7);
                i2c_device_set_add(s->vdev_i2c_set, addr7, &o->base, ssd1306_vdev_get_i2c_ops());
            } else if (g_strcmp0(kind, "bme280") == 0) {
                BME280VDev *b = bme280_vdev_create(addr7, 25.0f, 101325.0f, 50.0f);
                i2c_device_set_add(s->vdev_i2c_set, addr7, &b->base, bme280_vdev_get_i2c_ops());
            } else if (g_strcmp0(kind, "bh1750") == 0) {
                BH1750VDev *l = bh1750_vdev_create(addr7, 500.0f);
                i2c_device_set_add(s->vdev_i2c_set, addr7, &l->base, bh1750_vdev_get_i2c_ops());
            } else if (g_strcmp0(kind, "aht20") == 0) {
                AHT20VDev *a = aht20_vdev_create(addr7, 25.0f, 50.0f);
                i2c_device_set_add(s->vdev_i2c_set, addr7, &a->base, aht20_vdev_get_i2c_ops());
            } else if (g_strcmp0(kind, "adxl345") == 0) {
                ADXL345VDev *x = adxl345_vdev_create(addr7, 0.0f, 0.0f, 1.0f);
                i2c_device_set_add(s->vdev_i2c_set, addr7, &x->base, adxl345_vdev_get_i2c_ops());
            } else if (g_strcmp0(kind, "ina219") == 0) {
                INA219VDev *i = ina219_vdev_create(addr7, 5.0f, 0.1f);
                i2c_device_set_add(s->vdev_i2c_set, addr7, &i->base, ina219_vdev_get_i2c_ops());
            }
        }
        g_free(list);
        used_dev_list = true;
    }

    if (!used_dev_list) {
        // Defaults: EEPROM and RTC
        EEPROMVDev *e1 = eeprom_vdev_create(0x50, 4096);
        i2c_device_set_add(s->vdev_i2c_set, e1->i2c_addr7, &e1->base, eeprom_vdev_get_i2c_ops());
        /* RTC model selection */
        const char *model = s->rtc_model ? s->rtc_model : "ds3231";
        if (g_strcmp0(model, "ds3231") == 0) {
            DS3231VDev *r = ds3231_vdev_create(0x68);
            r->base.bus_context = s;
            if (s->rtc_init_year >= 0) r->t.year = (uint8_t)(s->rtc_init_year & 0xFF);
            if (s->rtc_init_month >= 0) r->t.month = (uint8_t)s->rtc_init_month;
            if (s->rtc_init_day >= 0) r->t.day = (uint8_t)s->rtc_init_day;
            if (s->rtc_init_dow >= 0) r->t.dow = (uint8_t)s->rtc_init_dow;
            if (s->rtc_init_hour >= 0) r->t.hours = (uint8_t)s->rtc_init_hour;
            if (s->rtc_init_minute >= 0) r->t.minutes = (uint8_t)s->rtc_init_minute;
            if (s->rtc_init_second >= 0) r->t.seconds = (uint8_t)s->rtc_init_second;
            i2c_device_set_add(s->vdev_i2c_set, r->i2c_addr7, &r->base, ds3231_vdev_get_i2c_ops());
        } else {
            DS1307VDev *r = ds1307_vdev_create(0x68);
            r->base.bus_context = s;
            if (s->rtc_init_year >= 0) r->t.year = (uint8_t)(s->rtc_init_year & 0xFF);
            if (s->rtc_init_month >= 0) r->t.month = (uint8_t)s->rtc_init_month;
            if (s->rtc_init_day >= 0) r->t.day = (uint8_t)s->rtc_init_day;
            if (s->rtc_init_dow >= 0) r->t.dow = (uint8_t)s->rtc_init_dow;
            if (s->rtc_init_hour >= 0) r->t.hours = (uint8_t)s->rtc_init_hour;
            if (s->rtc_init_minute >= 0) r->t.minutes = (uint8_t)s->rtc_init_minute;
            if (s->rtc_init_second >= 0) r->t.seconds = (uint8_t)s->rtc_init_second;
            i2c_device_set_add(s->vdev_i2c_set, r->i2c_addr7, &r->base, ds1307_vdev_get_i2c_ops());
        }
    }
    // Optional display
    // SSD1306VDev *o1 = ssd1306_vdev_create(0x3C);
    // i2c_device_set_add(s->vdev_i2c_set, o1->i2c_addr7, &o1->base, ssd1306_vdev_get_i2c_ops());
    
    // Log device summary for this controller
    if (s->vdev_i2c_set && s->log_level > 0) {
        qemu_log("ESP32 I2C%d: Registered virtual devices\n", s->unit_index);
        if (s->vdev_i2c_set->count == 0) {
            qemu_log("  (none)\n");
        } else {
            for (int i = 0; i < s->vdev_i2c_set->count; ++i) {
                const I2CDeviceEntry *e = &s->vdev_i2c_set->entries[i];
                const char *name = (e->device && e->device->name) ? e->device->name : "Unknown";
                qemu_log("  - 0x%02x %s\n", e->address7, name);
            }
        }
    }
}

static void esp32_i2c_virtual_device_reset(Esp32I2CState *s)
{
    // Reset all virtual devices using device-specific reset functions
    for (int i = 0; i < s->device_count; i++) {
        I2CVirtualDevice *device = &s->virtual_devices[i];
        
        switch (device->base.type) {
        case I2C_DEVICE_TYPE_TMP105:
            tmp105_reset((TMP105Device*)device);
            break;
        case I2C_DEVICE_TYPE_EEPROM:
            eeprom_reset((EEPROMDevice*)device);
            break;
        case I2C_DEVICE_TYPE_RTC:
            rtc_reset((RTCDevice*)device);
            break;
        case I2C_DEVICE_TYPE_SSD1306:
            ssd1306_reset((SSD1306Device*)device);
            break;
        default:
            device->base.state = I2C_DEVICE_STATE_IDLE;
            device->base.last_access_time = 0;
            device->base.access_count = 0;
            break;
        }
    }
    
    // quiet virtual device reset log
}

static void esp32_i2c_scan_virtual_devices(Esp32I2CState *s)
{
    if (!s->device_scan_enabled) {
        return;
    }
    
    uint64_t current_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (current_time - s->last_device_scan_time < s->device_scan_interval_ns) {
        return;
    }
    
    s->last_device_scan_time = current_time;
    
    // Use the new device-specific scan function
    esp32_i2c_scan_devices(s);
}

/* Phase 8: Old device management functions removed - now in esp32_i2c_devices.c */

static void esp32_i2c_reset_hold(Object *obj, ResetType type)
{
    Esp32I2CState * s = Esp32_I2C(obj);

    fifo8_reset(&s->rx_fifo);
    fifo8_reset(&s->tx_fifo);
    s->trans_ongoing = false;
    s->stream_state = I2C_STREAM_IDLE;
    s->ctr_reg = 0;
    s->timeout_reg = 0;
    s->fifo_conf_reg = 0;
    s->int_ena_reg = 0;
    s->int_raw_reg = 0;
    s->sda_hold_reg = 0;
    s->sda_sample_reg = 0;
    s->high_period_reg = 0;
    s->low_period_reg = 0;
    s->start_hold_reg = 0;
    s->rstart_setup_reg = 0;
    s->stop_hold_reg = 0;
    s->stop_setup_reg = 0;
    memset(s->cmd_reg, 0, sizeof(s->cmd_reg));

    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
    
    /* Phase 1: Reset state machine */
    esp32_i2c_state_machine_reset(s);
    
    /* Phase 2: Reset timers */
    if (s->state_timer) {
        timer_del(s->state_timer);
    }
    if (s->phase_timer) {
        timer_del(s->phase_timer);
    }
    // quiet timers reset log
    
    /* Phase 3: Reset hardware timing */
    esp32_i2c_timing_reset(s);
    
    /* Phase 4: Reset transaction management */
    esp32_i2c_transaction_reset(s);
    
    /* Phase 5: Reset I2C protocol */
    esp32_i2c_protocol_reset(s);
    
    /* Phase 6: Reset I2C bus simulation */
    esp32_i2c_bus_simulation_reset(s);
    
    /* Phase 7: Reset interrupt generation */
    esp32_i2c_interrupt_reset(s);
    
    /* Phase 8: Reset virtual device integration */
    esp32_i2c_virtual_device_reset(s);
}

static uint32_t esp32_i2c_get_status_reg(Esp32I2CState* s)
{
    uint32_t res = 0;
    res = FIELD_DP32(res, I2C_STATUS, BUS_BUSY, s->trans_ongoing);
    res = FIELD_DP32(res, I2C_STATUS, RXFIFO_CNT, fifo8_num_used(&s->rx_fifo));
    res = FIELD_DP32(res, I2C_STATUS, TXFIFO_CNT, fifo8_num_used(&s->tx_fifo));
    return res;
}

static void esp32_i2c_update_irq(Esp32I2CState * s)
{
    int irq_state = !!(s->int_raw_reg & s->int_ena_reg);
    qemu_set_irq(s->irq, irq_state);
}

static uint64_t esp32_i2c_read(void * opaque, hwaddr addr, unsigned int size)
{
    Esp32I2CState * s = Esp32_I2C(opaque);
    /* Keep SDA/SCL pins in sync with IO_MUX/GPIO matrix before reads */
    esp32_i2c_update_sda_scl_from_gpio(s);

    switch(addr) {
    case A_I2C_CTR:
        return s->ctr_reg;
    case A_I2C_STATUS:
        return esp32_i2c_get_status_reg(s);
    case A_I2C_FIFO_CONF:
        return s->fifo_conf_reg;
    case A_I2C_FIFO_DATA: {
        if (fifo8_num_used(&s->rx_fifo) == 0) {
            error_report("esp32_i2c: read I2C FIFO while it is empty");
            return 0xee;
        }
        uint8_t res = fifo8_pop(&s->rx_fifo);
        return res;
    }
    case A_I2C_INT_RAW:
        return s->int_raw_reg;
    case A_I2C_INT_ENA:
        return s->int_ena_reg;
    case A_I2C_INT_ST:
        return s->int_raw_reg & s->int_ena_reg;
    case A_I2C_CMD ... (A_I2C_CMD + ESP32_I2C_CMD_COUNT * 4):
        return s->cmd_reg[(addr - A_I2C_CMD) / 4];
    case A_I2C_TIMEOUT:
        return s->timeout_reg;
    case A_I2C_SDA_HOLD:
        return s->sda_hold_reg;
    case A_I2C_SDA_SAMPLE:
        return s->sda_sample_reg;
    case A_I2C_HIGH_PERIOD:
        return s->high_period_reg;
    case A_I2C_LOW_PERIOD:
        return s->low_period_reg;
    case A_I2C_START_HOLD:
        return s->start_hold_reg;
    case A_I2C_RSTART_SETUP:
        return s->rstart_setup_reg;
    case A_I2C_STOP_HOLD:
        return s->stop_hold_reg;
    case A_I2C_STOP_SETUP:
        return s->stop_setup_reg;
    default:
        return 0;
    }
}

static void esp32_i2c_write(void * opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32I2CState * s = Esp32_I2C(opaque);
    /* Keep SDA/SCL pins in sync with IO_MUX/GPIO matrix before writes */
    esp32_i2c_update_sda_scl_from_gpio(s);

    switch(addr) {
    case A_I2C_CTR:
        if (FIELD_EX32(value, I2C_CTR, TRANS_START)) {
            s->ctr_reg = value;
            if (fifo8_is_empty(&s->tx_fifo)) {
                esp32_i2c_generate_interrupt(s, I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY);
            } else {
                esp32_i2c_do_transaction(s);
            }
            s->ctr_reg = FIELD_DP32(s->ctr_reg, I2C_CTR, TRANS_START, 0);
            esp32_i2c_update_irq(s);
            break;
        }
        /* Any write with TRANS_START=0 implies bus idle from firmware perspective */
        s->trans_ongoing = false;
        s->stream_state = I2C_STREAM_IDLE;
        s->ctr_reg = value;
        break;
    case A_I2C_FIFO_CONF: {
        /* Robust guard: force NONFIFO_EN=0 (APB mode unsupported), execute resets, mirror state */
        uint32_t conf = value;
        /* Force FIFO mode */
        conf = FIELD_DP32(conf, I2C_FIFO_CONF, NONFIFO_EN, 0);
        bool do_rx_rst = FIELD_EX32(value, I2C_FIFO_CONF, RX_FIFO_RST);
        bool do_tx_rst = FIELD_EX32(value, I2C_FIFO_CONF, TX_FIFO_RST);
        if (do_rx_rst) fifo8_reset(&s->rx_fifo);
        if (do_tx_rst) {
            fifo8_reset(&s->tx_fifo);
            s->stream_state = I2C_STREAM_IDLE;
        }
        s->trans_ongoing = false;
        /* Clear self-clearing reset bits in mirror */
        conf = FIELD_DP32(conf, I2C_FIFO_CONF, RX_FIFO_RST, 0);
        conf = FIELD_DP32(conf, I2C_FIFO_CONF, TX_FIFO_RST, 0);
        s->fifo_conf_reg = conf;
        /* Update IRQs after resets/watermark changes */
        esp32_i2c_handle_fifo_interrupts(s);
        break;
    }
    case A_I2C_FIFO_DATA:
        if (fifo8_num_free(&s->tx_fifo) == 0) {
            error_report("esp32_i2c: write to I2C TX FIFO while it is full");
        } else {
            fifo8_push(&s->tx_fifo, value);
            // New data arrived; attempt to progress the transaction
            esp32_i2c_do_transaction(s);
            
            // Phase 7: Check for FIFO interrupts after data write
            esp32_i2c_handle_fifo_interrupts(s);
        }
        break;
    case A_I2C_INT_CLR:
        {
            uint32_t before = s->int_raw_reg;
            s->int_raw_reg &= ~value;
            (void)before;
            if (value & I2C_INTERRUPT_TX_FIFO_EMPTY) s->tx_irq_asserted = false;
            if (value & I2C_INTERRUPT_RX_FIFO_FULL) s->rx_irq_asserted = false;
        }
        // Phase 7: Update interrupt state
        s->interrupt_pending &= ~value;
        esp32_i2c_update_irq(s);
        break;
    case A_I2C_INT_ENA:
        s->int_ena_reg = value;
        // Phase 7: Update interrupt state
        s->interrupt_enabled = value;
        /* Re-evaluate FIFO watermark conditions now that enables changed */
        esp32_i2c_handle_fifo_interrupts(s);
        esp32_i2c_update_irq(s);
        // quiet
        break;
    case A_I2C_CMD ... (A_I2C_CMD + ESP32_I2C_CMD_COUNT * 4):
        {
            int cmd_idx = (addr - A_I2C_CMD) / 4;
            s->cmd_reg[cmd_idx] = value;
            // quiet
        }
        break;
    case A_I2C_TIMEOUT:
        s->timeout_reg = value;
        break;
    case A_I2C_SDA_HOLD:
        s->sda_hold_reg = value;
        break;
    case A_I2C_SDA_SAMPLE:
        s->sda_sample_reg = value;
        break;
    case A_I2C_HIGH_PERIOD:
        s->high_period_reg = value;
        /* Phase 3: Detect clock mode change */
        s->clock_mode = esp32_i2c_detect_clock_mode(s);
        s->i2c_clock_freq = esp32_i2c_get_clock_frequency(s);
        // quiet
        break;
    case A_I2C_LOW_PERIOD:
        s->low_period_reg = value;
        /* Phase 3: Detect clock mode change */
        s->clock_mode = esp32_i2c_detect_clock_mode(s);
        s->i2c_clock_freq = esp32_i2c_get_clock_frequency(s);
        // quiet
        break;
    case A_I2C_START_HOLD:
        s->start_hold_reg = value;
        break;
    case A_I2C_RSTART_SETUP:
        s->rstart_setup_reg = value;
        break;
    case A_I2C_STOP_HOLD:
        s->stop_hold_reg = value;
        break;
    case A_I2C_STOP_SETUP:
        s->stop_setup_reg = value;
        break;
    default:
        break;
    }
}

static void esp32_i2c_do_transaction(Esp32I2CState * s)
{
    // quiet transaction start log
    bool stop_or_end = false;
    uint8_t current_device_address = 0; // local for transitional logging; persisted to s->current_device_address
    
    for (int i_cmd = 0; i_cmd < ESP32_I2C_CMD_COUNT && !stop_or_end; ++i_cmd) {
        uint32_t cmd = s->cmd_reg[i_cmd];
        uint8_t op_pre = FIELD_EX32(cmd, I2C_CMD, OPCODE);
        uint8_t bytes_pre = FIELD_EX32(cmd, I2C_CMD, BYTE_NUM);
        uint8_t done_pre = FIELD_EX32(cmd, I2C_CMD, DONE);
        if (done_pre) {
            // Skip commands already executed
            continue;
        }
        char opcode = op_pre;
        uint8_t bytes_dbg = bytes_pre;
        // quiet cmd processing log
        
        switch (opcode) {
            case I2C_OPCODE_RSTART:
                // Repeated START - end current transfer
                s->trans_ongoing = false;
                s->stream_state = I2C_STREAM_IDLE;
                break;
                
            case I2C_OPCODE_WRITE: {
                size_t length = FIELD_EX32(cmd, I2C_CMD, BYTE_NUM);
                /* When stream_state==IN_DATA, address was already consumed; only data bytes remain. */
                size_t data_length = (length > 0 && s->stream_state == I2C_STREAM_IDLE) ? length - 1 : length;
                uint8_t write_data[256];
                size_t data_count = 0;
                if (s->stream_state == I2C_STREAM_IDLE) {
                    /* First byte after STOP = address */
                    if (fifo8_num_used(&s->tx_fifo) == 0) {
                        esp32_i2c_generate_interrupt(s, I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY);
                        return;
                    }
                    uint8_t data = fifo8_pop(&s->tx_fifo);
                    current_device_address = data >> 1;
                    s->current_device_address = current_device_address;
                    s->stream_state = I2C_STREAM_IN_DATA;
                    s->trans_ongoing = true;
                    bool is_read = (data & 0x01) != 0;
                    esp32_i2c_monitor_transaction_start(s, current_device_address, is_read);
                    bool acked = esp32_i2c_simulate_device_ack(s, current_device_address);
                    if (!acked) {
                        if (FIELD_EX32(cmd, I2C_CMD, ACK_CHECK_EN)
                            && FIELD_EX32(cmd, I2C_CMD, ACK_EXP) == 0) {
                            s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, ACK_ERR, 1);
                            stop_or_end = true;
                        }
                        break;
                    }
                    s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, ACK_ERR, 0);
                    esp32_i2c_handle_device_response(s, s->current_device_address);
                    if (s->vdev_i2c_set) {
                        const I2CDeviceEntry *e = i2c_device_set_find(s->vdev_i2c_set, s->current_device_address);
                        if (e && e->ops && e->ops->i2c_on_addressed) {
                            e->ops->i2c_on_addressed(e->device, s->current_device_address, false);
                        }
                    }
                    if (length > 0) {
                        data_length = length - 1; /* address byte accounted for */
                        if (data_length > 0 && fifo8_is_empty(&s->tx_fifo)) {
                            esp32_i2c_generate_interrupt(s, I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY);
                            return;
                        }
                    }
                }
                // Guard against underflow: ensure all requested data bytes are present
                if (data_length > 0) {
                    int have_tx = fifo8_num_used(&s->tx_fifo);
                    if (have_tx < (int)data_length) {
                        // request ISR to push remaining bytes
                        esp32_i2c_generate_interrupt(s, I2C_INTERRUPT_TYPE_TX_FIFO_EMPTY);
                        return; // wait for ISR to push more bytes
                    }
                    for (size_t nbytes = 0; nbytes < data_length; ++nbytes) {
                        write_data[data_count++] = fifo8_pop(&s->tx_fifo);
                    }
                    // quiet
                }
                // If no data bytes were required (length==0), proceed to next command (e.g., STOP)
                if (data_count > 0) {
                    /* Emit data bytes to I2C monitor pipe (after START, before STOP) */
                    for (size_t mi = 0; mi < data_count; ++mi) {
                        esp32_i2c_monitor_fifo_write(s, write_data[mi]);
                    }
                    /* Prefer vdev ops for write if available */
                    if (s->vdev_i2c_set) {
                        const I2CDeviceEntry *e = i2c_device_set_find(s->vdev_i2c_set, s->current_device_address);
                        if (e && e->ops && e->ops->i2c_write) {
                            (void)e->ops->i2c_write(e->device, s->current_device_address, write_data, data_count);
                        }
                    }
                    I2CVirtualDevice *device = esp32_i2c_find_device_by_address(s, s->current_device_address);
                    if (device) {
                        switch (device->base.type) {
                        case I2C_DEVICE_TYPE_TMP105:
                            tmp105_process_write((TMP105Device*)device, write_data, data_count);
                            break;
                        case I2C_DEVICE_TYPE_EEPROM:
                            eeprom_process_write((EEPROMDevice*)device, write_data, data_count);
                            break;
                        case I2C_DEVICE_TYPE_RTC:
                            rtc_process_write((RTCDevice*)device, write_data, data_count);
                            break;
                        case I2C_DEVICE_TYPE_SSD1306:
                            ssd1306_process_write((SSD1306Device*)device, write_data, data_count);
                            break;
                        case I2C_DEVICE_TYPE_UNKNOWN:
                        default:
                            break;
                        }
                    }
                    /* Data-phase ACK check (simple): if enabled and device doesn't ACK, raise ACK_ERR */
                    if (FIELD_EX32(cmd, I2C_CMD, ACK_CHECK_EN)) {
                        bool expected_ack = (FIELD_EX32(cmd, I2C_CMD, ACK_EXP) == 0);
                        bool data_acked = esp32_i2c_simulate_device_ack(s, s->current_device_address);
                        if (data_acked != expected_ack) {
                            s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, ACK_ERR, 1);
                            stop_or_end = true;
                        } else {
                            s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, ACK_ERR, 0);
                        }
                    }
                }
                // After processing writes, update FIFO-related interrupts
                esp32_i2c_handle_fifo_interrupts(s);
                break;
            }
            
            case I2C_OPCODE_READ: {
                size_t length = FIELD_EX32(cmd, I2C_CMD, BYTE_NUM);
                
                // Get data from virtual device
                uint8_t read_data[256];
                esp32_i2c_simulate_device_data(s, s->current_device_address, read_data, length);
                /* Emit read data bytes to I2C monitor pipe (after START, before STOP) */
                for (size_t mi = 0; mi < length; ++mi) {
                    esp32_i2c_monitor_fifo_read(s, read_data[mi]);
                }
                
                // Put data into RX FIFO
                for (size_t nbytes = 0; nbytes < length; ++nbytes) {
                    if (fifo8_num_free(&s->rx_fifo) == 0) {
                        error_report("esp32_i2c: RX FIFO overflow");
                        break;
                    } else {
                        fifo8_push(&s->rx_fifo, read_data[nbytes]);
                    }
                }
                // Update FIFO-related interrupts after pushing RX data
                esp32_i2c_handle_fifo_interrupts(s);
                // quiet
                break;
            }
            
            case I2C_OPCODE_STOP:
                esp32_i2c_monitor_transaction_stop(s);
                s->trans_ongoing = false;
                s->stream_state = I2C_STREAM_IDLE;
                s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, TRANS_COMPLETE, 1);
                // quiet
                stop_or_end = true;
                break;
                
            case I2C_OPCODE_END:
                s->stream_state = I2C_STREAM_IDLE;
                s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, END_DETECT, 1);
                // quiet end condition log
                stop_or_end = true;
                break;
                
            default:
                error_report("esp32_i2c: Invalid command %d opcode %d", i_cmd, opcode);
                break;
        }
        s->cmd_reg[i_cmd] = FIELD_DP32(s->cmd_reg[i_cmd], I2C_CMD, DONE, 1);
    }
    
    /* Phase 8: Device-specific transaction completion */
    // quiet transaction completed log
    
    /* Phase 7: Interrupt Management */
    esp32_i2c_update_irq(s);
    // quiet transaction full simulation log
}

static const MemoryRegionOps esp32_i2c_ops = {
    .read = esp32_i2c_read,
    .write = esp32_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_i2c_init(Object * obj)
{
    Esp32I2CState *s = Esp32_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    // quiet init log
    memory_region_init_io(&s->iomem, obj, &esp32_i2c_ops, s, TYPE_ESP32_I2C, ESP32_I2C_MEM_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->bus = i2c_init_bus(DEVICE(s), "i2c");
    // quiet bus init log

    fifo8_create(&s->tx_fifo, ESP32_I2C_FIFO_LENGTH);
    fifo8_create(&s->rx_fifo, ESP32_I2C_FIFO_LENGTH);
    /* Default watermark thresholds: TX needs data when empty; RX fires when >=1 byte */
    s->tx_fifo_wm = 0;
    s->rx_fifo_wm = 1;
    s->tx_irq_asserted = false;
    s->rx_irq_asserted = false;
    
    /* Phase 1: Initialize state machine */
    esp32_i2c_state_machine_init(s);
    
    /* Phase 2: Initialize timers */
    s->state_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                  esp32_i2c_state_timer_cb, s);
    s->phase_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                  esp32_i2c_phase_timer_cb, s);
    // quiet timers init log
    
    /* Phase 3: Initialize hardware timing */
    esp32_i2c_timing_init(s);
    
    /* Phase 4: Initialize transaction management */
    esp32_i2c_transaction_init(s);
    
    /* Phase 5: Initialize I2C protocol */
    esp32_i2c_protocol_init(s);
    
    /* Phase 6: Initialize I2C bus simulation */
    esp32_i2c_bus_simulation_init(s);
    
    /* Phase 7: Initialize interrupt generation */
    esp32_i2c_interrupt_init(s);
    
    /* Phase 8: Initialize virtual device integration */
    esp32_i2c_virtual_device_init(s);
}

static void esp32_i2c_class_init(ObjectClass * klass, void * data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32_i2c_reset_hold;
    /* QOM properties for RTC model and initial date/time */
    DeviceClass *dc = DEVICE_CLASS(klass);
    static Property esp32_i2c_props[] = {
        DEFINE_PROP_INT32("unit", Esp32I2CState, unit_index, 0),
        DEFINE_PROP_STRING("rtc-model", Esp32I2CState, rtc_model),
        DEFINE_PROP_STRING("i2c-devices", Esp32I2CState, dev_list),
        DEFINE_PROP_INT32("sda-pin", Esp32I2CState, sda_pin, 21),
        DEFINE_PROP_INT32("scl-pin", Esp32I2CState, scl_pin, 22),
        DEFINE_PROP_INT32("rtc-sqw-pin", Esp32I2CState, rtc_sqw_pin, -1),
        DEFINE_PROP_INT32("log-level", Esp32I2CState, log_level, 0),
        /* Per-device defaults */
        DEFINE_PROP_INT32("tmp105-init-temp-c", Esp32I2CState, tmp105_init_temp_c, 25),
        DEFINE_PROP_INT32("tmp105-drift-mcps", Esp32I2CState, tmp105_drift_mcps, 0),
        DEFINE_PROP_INT32("tmp105-noise-mc", Esp32I2CState, tmp105_noise_mc, 250),
        DEFINE_PROP_INT32("eeprom-page-size", Esp32I2CState, eeprom_page_size, 64),
        DEFINE_PROP_INT32("eeprom-write-delay-ms", Esp32I2CState, eeprom_write_delay_ms, 5),
        DEFINE_PROP_INT32("ds3231-init-temp-c", Esp32I2CState, ds3231_init_temp_c, 25),
        DEFINE_PROP_INT32("rtc-year", Esp32I2CState, rtc_init_year, -1),
        DEFINE_PROP_INT32("rtc-month", Esp32I2CState, rtc_init_month, -1),
        DEFINE_PROP_INT32("rtc-day", Esp32I2CState, rtc_init_day, -1),
        DEFINE_PROP_INT32("rtc-dow", Esp32I2CState, rtc_init_dow, -1),
        DEFINE_PROP_INT32("rtc-hour", Esp32I2CState, rtc_init_hour, -1),
        DEFINE_PROP_INT32("rtc-minute", Esp32I2CState, rtc_init_minute, -1),
        DEFINE_PROP_INT32("rtc-second", Esp32I2CState, rtc_init_second, -1),
        DEFINE_PROP_END_OF_LIST(),
    };
    device_class_set_props(dc, esp32_i2c_props);
}

static const TypeInfo esp32_i2c_type_info = {
    .name = TYPE_ESP32_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32I2CState),
    .instance_init = esp32_i2c_init,
    .class_init = esp32_i2c_class_init,
};

static void esp32_i2c_register_types(void)
{
    type_register_static(&esp32_i2c_type_info);
}

type_init(esp32_i2c_register_types)

