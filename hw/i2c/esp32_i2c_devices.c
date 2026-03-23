/**
 * ESP32 I2C Virtual Device Implementation
 * 
 * Device-specific, encapsulated implementation for Phase 8 Virtual Device Integration
 * Provides professional-grade device simulation with hardware-accurate behavior
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/i2c/esp32_i2c_devices.h"
#include "hw/i2c/device_set.h"
#include "hw/i2c/devices/tmp105_vdev.h"

/* Phase 8: TMP105 Temperature Sensor Implementation */

void tmp105_init(TMP105Device *device, uint8_t address, float initial_temperature)
{
    // Initialize base structure
    device->base.address = address;
    device->base.type = I2C_DEVICE_TYPE_TMP105;
    device->base.state = I2C_DEVICE_STATE_IDLE;
    device->base.present = true;
    device->base.responding = true;
    device->base.last_access_time = 0;
    device->base.access_count = 0;
    device->base.debug_enabled = true;
    
    // Initialize TMP105-specific fields
    device->temperature = initial_temperature;
    device->config_register = 0x00;  // Default configuration
    device->resolution = 12;         // 12-bit resolution
    device->register_address = 0x00; // Temperature register
    device->shutdown_mode = false;
    device->conversion_time_us = 100; // 100μs conversion time
    device->read_phase = 0;           // start with MSB on reads
    device->drift_c_per_sec = 0.0;    // no drift by default
    device->noise_c_amplitude = 0.25; // +-0.25C random noise per conversion
    device->temp_min_c = -40.0f;
    device->temp_max_c = 125.0f;
    device->last_conversion_time_ns = 0;
    
    if (device->base.debug_enabled) {
        qemu_log("esp32_i2c: TMP105 initialized at 0x%02x (%.2f°C)\n", address, initial_temperature);
    }
}

void tmp105_reset(TMP105Device *device)
{
    device->base.state = I2C_DEVICE_STATE_IDLE;
    device->base.responding = true;
    device->config_register = 0x00;
    device->register_address = 0x00;
    device->shutdown_mode = false;
    device->base.access_count = 0;
    device->read_phase = 0;
    device->last_conversion_time_ns = 0;
    
    if (device->base.debug_enabled) {
        qemu_log("esp32_i2c: TMP105 reset at 0x%02x\n", device->base.address);
    }
}

void tmp105_process_read(TMP105Device *device, uint8_t *data, size_t length)
{
    if (!device->base.responding || length == 0) {
        return;
    }
    
    // Minimal environment simulation: update temperature on conversion cadence
    uint64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (device->last_conversion_time_ns == 0) {
        device->last_conversion_time_ns = now_ns;
    }
    uint64_t elapsed_ns = now_ns - device->last_conversion_time_ns;
    uint64_t period_ns = (uint64_t)device->conversion_time_us * 1000ULL;
    if (elapsed_ns >= period_ns) {
        double dt_s = (double)elapsed_ns / 1e9;
        double noise = 0.0;
        // GLib RNG is available via qemu/osdep.h
        noise = g_random_double_range(-device->noise_c_amplitude, device->noise_c_amplitude);
        double updated = (double)device->temperature + device->drift_c_per_sec * dt_s + noise;
        if (updated < (double)device->temp_min_c) updated = (double)device->temp_min_c;
        if (updated > (double)device->temp_max_c) updated = (double)device->temp_max_c;
        device->temperature = (float)updated;
        device->last_conversion_time_ns = now_ns;
        qemu_log("esp32_i2c: Phase 8 - TMP105 temp update -> %.2fC (dt=%.3fs, noise=%.2fC)\n",
                 device->temperature, dt_s, noise);
    }
    
    // TMP105 temperature reading (12-bit format)
    if (device->register_address == 0x00) {  // Temperature register
        // Convert temperature to TMP105 12-bit format
        int16_t raw_temp = (int16_t)(device->temperature * 16.0);  // 0.0625°C resolution
        
        // Handle negative temperatures (2's complement)
        if (raw_temp < 0) {
            raw_temp = raw_temp + 4096;  // Convert to unsigned 12-bit
        }
        uint8_t msb = (raw_temp >> 4) & 0xFF;  // MSB
        uint8_t lsb = (raw_temp << 4) & 0xF0;  // LSB (4 bits)
        if (length >= 2) {
            data[0] = msb;
            data[1] = lsb;
            device->read_phase = 0; // reset phase after fulfilling full word
        } else { // length == 1
            if (device->read_phase == 0) {
                data[0] = msb;
                device->read_phase = 1;
            } else {
                data[0] = lsb;
                device->read_phase = 0;
            }
        }
        
        device->base.access_count++;
        device->base.last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        
        if (length >= 2) {
        if (device->base.debug_enabled) {
            qemu_log("esp32_i2c: TMP105 temperature read: %.2f°C (0x%02x%02x)\n",
                     device->temperature, data[0], data[1]);
        }
        } else {
            qemu_log("esp32_i2c: Phase 8 - TMP105 temperature read: %.2f°C (byte=%02x phase=%d)\n",
                     device->temperature, data[0], device->read_phase);
        }
    } else if (device->register_address == 0x01) {  // Configuration register
        if (length >= 2) {
            data[0] = device->config_register;
            data[1] = 0x00;  // Reserved bits
        } else {
            data[0] = device->config_register;
        }
        
        qemu_log("esp32_i2c: Phase 8 - TMP105 config read: 0x%02x\n", device->config_register);
    }
}

void tmp105_process_write(TMP105Device *device, uint8_t *data, size_t length)
{
    if (!device->base.responding || length == 0) {
        return;
    }
    
    // First byte is usually the register address
    if (length == 1) {
        device->register_address = data[0];
        device->base.state = I2C_DEVICE_STATE_ADDRESSED;
        device->read_phase = 0; // reset read phase on new pointer
        
        qemu_log("esp32_i2c: Phase 8 - TMP105 register selected: 0x%02x\n", device->register_address);
    } else if (length == 2 && device->register_address == 0x01) {  // Configuration register
        device->config_register = data[0];
        device->shutdown_mode = (data[0] & 0x01) != 0;  // Shutdown bit
        
        if (device->base.debug_enabled) {
            qemu_log("esp32_i2c: TMP105 config written: 0x%02x (shutdown=%d)\n",
                     device->config_register, device->shutdown_mode);
        }
    }
    
    device->base.access_count++;
    device->base.last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

bool tmp105_handle_register_access(TMP105Device *device, uint8_t register_addr, bool is_write)
{
    if (register_addr == 0x00 || register_addr == 0x01) {
        device->register_address = register_addr;
        device->base.state = is_write ? I2C_DEVICE_STATE_WRITING : I2C_DEVICE_STATE_READING;
        return true;
    }
    return false;
}

void tmp105_update_temperature(TMP105Device *device, float new_temperature)
{
    device->temperature = new_temperature;
    qemu_log("esp32_i2c: Phase 8 - TMP105 temperature updated: %.2f°C\n", new_temperature);
}

void tmp105_log_event(TMP105Device *device, const char *event)
{
    qemu_log("esp32_i2c: Phase 8 - TMP105 0x%02x %s (State: %d, Access: %d)\n",
             device->base.address, event, device->base.state, device->base.access_count);
}

/* Phase 8: EEPROM Memory Device Implementation */

void eeprom_init(EEPROMDevice *device, uint8_t address, size_t memory_size)
{
    // Initialize base structure
    device->base.address = address;
    device->base.type = I2C_DEVICE_TYPE_EEPROM;
    device->base.state = I2C_DEVICE_STATE_IDLE;
    device->base.present = true;
    device->base.responding = true;
    device->base.last_access_time = 0;
    device->base.access_count = 0;
    device->base.debug_enabled = true;
    
    // Initialize EEPROM-specific fields
    memset(device->memory, 0xFF, sizeof(device->memory));  // EEPROM defaults to 0xFF
    device->current_address = 0;
    device->write_protect = 0;
    device->write_cycles = 0;
    device->page_size = 64;  // 64-byte pages
    device->address_bytes = 2;  // 2-byte addressing
    device->write_delay_ms = 5;  // 5ms write delay
    
    qemu_log("esp32_i2c: Phase 8 - EEPROM initialized at 0x%02x (%zu bytes)\n", 
             address, memory_size);
}

void eeprom_reset(EEPROMDevice *device)
{
    device->base.state = I2C_DEVICE_STATE_IDLE;
    device->base.responding = true;
    device->current_address = 0;
    device->write_protect = 0;
    device->base.access_count = 0;
    
    qemu_log("esp32_i2c: Phase 8 - EEPROM reset at 0x%02x\n", device->base.address);
}

void eeprom_process_read(EEPROMDevice *device, uint8_t *data, size_t length)
{
    if (!device->base.responding) {
        return;
    }
    
    for (size_t i = 0; i < length && device->current_address < sizeof(device->memory); i++) {
        data[i] = device->memory[device->current_address++];
    }
    
    device->base.access_count++;
    device->base.last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    qemu_log("esp32_i2c: Phase 8 - EEPROM read %zu bytes from 0x%04x\n", 
             length, device->current_address - length);
}

void eeprom_process_write(EEPROMDevice *device, uint8_t *data, size_t length)
{
    if (!device->base.responding || device->write_protect) {
        return;
    }
    
    // First 2 bytes are usually the address
    if (length >= 2) {
        device->current_address = (data[0] << 8) | data[1];
        data += 2;
        length -= 2;
        
        // Write data to memory
        for (size_t i = 0; i < length && device->current_address < sizeof(device->memory); i++) {
            device->memory[device->current_address++] = data[i];
        }
        
        device->write_cycles++;
        device->base.access_count++;
        device->base.last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        
        qemu_log("esp32_i2c: Phase 8 - EEPROM write %zu bytes to 0x%04x (cycles: %d)\n",
                 length, device->current_address - length, device->write_cycles);
    }
}

bool eeprom_handle_address_write(EEPROMDevice *device, uint8_t *address_data, size_t length)
{
    if (length >= 2) {
        device->current_address = (address_data[0] << 8) | address_data[1];
        device->base.state = I2C_DEVICE_STATE_ADDRESSED;
        return true;
    }
    return false;
}

void eeprom_write_page(EEPROMDevice *device, uint16_t address, uint8_t *data, size_t length)
{
    if (device->write_protect) {
        return;
    }
    
    size_t page_start = (address / device->page_size) * device->page_size;
    size_t page_end = page_start + device->page_size;
    
    for (size_t i = 0; i < length && (address + i) < page_end; i++) {
        device->memory[address + i] = data[i];
    }
    
    device->write_cycles++;
    qemu_log("esp32_i2c: Phase 8 - EEPROM page write to 0x%04x (%zu bytes)\n", address, length);
}

void eeprom_log_event(EEPROMDevice *device, const char *event)
{
    qemu_log("esp32_i2c: Phase 8 - EEPROM 0x%02x %s (State: %d, Cycles: %d)\n",
             device->base.address, event, device->base.state, device->write_cycles);
}

/* Phase 8: RTC Real-Time Clock Implementation */

void rtc_init(RTCDevice *device, uint8_t address)
{
    // Initialize base structure
    device->base.address = address;
    device->base.type = I2C_DEVICE_TYPE_RTC;
    device->base.state = I2C_DEVICE_STATE_IDLE;
    device->base.present = true;
    device->base.responding = true;
    device->base.last_access_time = 0;
    device->base.access_count = 0;
    device->base.debug_enabled = true;
    
    // Initialize RTC-specific fields (default time)
    device->seconds = 0x00;  // BCD format
    device->minutes = 0x00;
    device->hours = 0x12;    // 12:00:00
    device->day = 0x01;
    device->month = 0x01;
    device->year = 0x24;     // 2024
    device->control_register = 0x00;
    device->register_address = 0x00;
    device->alarm_enabled = false;
    device->oscillator_enabled = true;
    
    qemu_log("esp32_i2c: Phase 8 - RTC initialized at 0x%02x\n", address);
}

void rtc_reset(RTCDevice *device)
{
    device->base.state = I2C_DEVICE_STATE_IDLE;
    device->base.responding = true;
    device->control_register = 0x00;
    device->register_address = 0x00;
    device->alarm_enabled = false;
    device->oscillator_enabled = true;
    device->base.access_count = 0;
    
    qemu_log("esp32_i2c: Phase 8 - RTC reset at 0x%02x\n", device->base.address);
}

void rtc_process_read(RTCDevice *device, uint8_t *data, size_t length)
{
    if (!device->base.responding) {
        return;
    }
    
    // RTC register reading (BCD format)
    switch (device->register_address) {
    case 0x00:  // Seconds
        data[0] = device->seconds;
        break;
    case 0x01:  // Minutes
        data[0] = device->minutes;
        break;
    case 0x02:  // Hours
        data[0] = device->hours;
        break;
    case 0x03:  // Day
        data[0] = device->day;
        break;
    case 0x04:  // Month
        data[0] = device->month;
        break;
    case 0x05:  // Year
        data[0] = device->year;
        break;
    case 0x07:  // Control register
        data[0] = device->control_register;
        break;
    default:
        data[0] = 0x00;
        break;
    }
    
    device->base.access_count++;
    device->base.last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    qemu_log("esp32_i2c: Phase 8 - RTC read register 0x%02x: 0x%02x\n",
             device->register_address, data[0]);
}

void rtc_process_write(RTCDevice *device, uint8_t *data, size_t length)
{
    if (!device->base.responding || length == 0) {
        return;
    }
    
    // First byte is usually the register address
    if (length == 1) {
        device->register_address = data[0];
        device->base.state = I2C_DEVICE_STATE_ADDRESSED;
        
        qemu_log("esp32_i2c: Phase 8 - RTC register selected: 0x%02x\n", device->register_address);
    } else if (length == 2) {
        // Write to selected register
        switch (device->register_address) {
        case 0x00:  // Seconds
            device->seconds = data[1];
            break;
        case 0x01:  // Minutes
            device->minutes = data[1];
            break;
        case 0x02:  // Hours
            device->hours = data[1];
            break;
        case 0x03:  // Day
            device->day = data[1];
            break;
        case 0x04:  // Month
            device->month = data[1];
            break;
        case 0x05:  // Year
            device->year = data[1];
            break;
        case 0x07:  // Control register
            device->control_register = data[1];
            device->oscillator_enabled = (data[1] & 0x80) != 0;
            break;
        }
        
        qemu_log("esp32_i2c: Phase 8 - RTC write register 0x%02x: 0x%02x\n",
                 device->register_address, data[1]);
    }
    
    device->base.access_count++;
    device->base.last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

bool rtc_handle_register_access(RTCDevice *device, uint8_t register_addr, bool is_write)
{
    if (register_addr <= 0x07) {
        device->register_address = register_addr;
        device->base.state = is_write ? I2C_DEVICE_STATE_WRITING : I2C_DEVICE_STATE_READING;
        return true;
    }
    return false;
}

void rtc_update_time(RTCDevice *device, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    device->hours = hours;
    device->minutes = minutes;
    device->seconds = seconds;
    
    qemu_log("esp32_i2c: Phase 8 - RTC time updated: %02d:%02d:%02d\n",
             hours, minutes, seconds);
}

void rtc_log_event(RTCDevice *device, const char *event)
{
    qemu_log("esp32_i2c: Phase 8 - RTC 0x%02x %s (State: %d, Time: %02d:%02d:%02d)\n",
             device->base.address, event, device->base.state,
             device->hours, device->minutes, device->seconds);
}

/* Phase 8: SSD1306 OLED Display Implementation */

void ssd1306_init(SSD1306Device *device, uint8_t address)
{
    // Initialize base structure
    device->base.address = address;
    device->base.type = I2C_DEVICE_TYPE_SSD1306;
    device->base.state = I2C_DEVICE_STATE_IDLE;
    device->base.present = true;
    device->base.responding = true;
    device->base.last_access_time = 0;
    device->base.access_count = 0;
    device->base.debug_enabled = true;
    
    // Initialize SSD1306-specific fields
    memset(device->display_buffer, 0x00, sizeof(device->display_buffer));
    device->current_column = 0;
    device->current_page = 0;
    device->display_mode = 0;  // Normal mode
    device->brightness = 0xFF; // Full brightness
    device->display_on = false;
    device->command_mode = true;
    
    qemu_log("esp32_i2c: Phase 8 - SSD1306 initialized at 0x%02x\n", address);
}

void ssd1306_reset(SSD1306Device *device)
{
    device->base.state = I2C_DEVICE_STATE_IDLE;
    device->base.responding = true;
    device->current_column = 0;
    device->current_page = 0;
    device->display_on = false;
    device->command_mode = true;
    device->base.access_count = 0;
    
    qemu_log("esp32_i2c: Phase 8 - SSD1306 reset at 0x%02x\n", device->base.address);
}

void ssd1306_process_read(SSD1306Device *device, uint8_t *data, size_t length)
{
    // SSD1306 typically doesn't support read operations
    // This is a placeholder for completeness
    qemu_log("esp32_i2c: Phase 8 - SSD1306 read operation (not supported)\n");
}

void ssd1306_process_write(SSD1306Device *device, uint8_t *data, size_t length)
{
    if (!device->base.responding) {
        return;
    }
    
    for (size_t i = 0; i < length; i++) {
        if (device->command_mode) {
            // Process command
            ssd1306_handle_command(device, data[i]);
        } else {
            // Process data (pixel data)
            if (device->current_column < 128 && device->current_page < 8) {
                size_t buffer_index = device->current_page * 128 + device->current_column;
                if (buffer_index < sizeof(device->display_buffer)) {
                    device->display_buffer[buffer_index] = data[i];
                }
                device->current_column++;
            }
        }
    }
    
    device->base.access_count++;
    device->base.last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

bool ssd1306_handle_command(SSD1306Device *device, uint8_t command)
{
    switch (command) {
    case 0xAF:  // Display ON
        device->display_on = true;
        qemu_log("esp32_i2c: Phase 8 - SSD1306 display ON\n");
        break;
    case 0xAE:  // Display OFF
        device->display_on = false;
        qemu_log("esp32_i2c: Phase 8 - SSD1306 display OFF\n");
        break;
    case 0x21:  // Set column address
        device->command_mode = true;
        break;
    case 0x22:  // Set page address
        device->command_mode = true;
        break;
    case 0x40:  // Set start line
        device->current_page = 0;
        break;
    default:
        // Handle other commands as needed
        break;
    }
    return true;
}

void ssd1306_set_pixel(SSD1306Device *device, uint8_t x, uint8_t y, bool on)
{
    if (x < 128 && y < 64) {
        size_t buffer_index = (y / 8) * 128 + x;
        if (buffer_index < sizeof(device->display_buffer)) {
            if (on) {
                device->display_buffer[buffer_index] |= (1 << (y % 8));
            } else {
                device->display_buffer[buffer_index] &= ~(1 << (y % 8));
            }
        }
    }
}

void ssd1306_log_event(SSD1306Device *device, const char *event)
{
    qemu_log("esp32_i2c: Phase 8 - SSD1306 0x%02x %s (State: %d, Display: %s)\n",
             device->base.address, event, device->base.state,
             device->display_on ? "ON" : "OFF");
}

/* Phase 8: Generic Device Management Implementation */

I2CVirtualDevice* esp32_i2c_find_device_by_address(Esp32I2CState *s, uint8_t address)
{
    for (int i = 0; i < s->device_count; i++) {
        if (s->virtual_devices[i].base.address == address) {
            return &s->virtual_devices[i];
        }
    }
    return NULL;
}

I2CVirtualDevice* esp32_i2c_find_device_by_type(Esp32I2CState *s, I2CVirtualDeviceType type)
{
    for (int i = 0; i < s->device_count; i++) {
        if (s->virtual_devices[i].base.type == type) {
            return &s->virtual_devices[i];
        }
    }
    return NULL;
}

bool esp32_i2c_add_device(Esp32I2CState *s, I2CVirtualDevice *device)
{
    if (s->device_count >= I2C_MAX_VIRTUAL_DEVICES) {
        return false;
    }
    
    s->virtual_devices[s->device_count] = *device;
    s->device_count++;
    
    qemu_log("esp32_i2c: Phase 8 - Device added: %s at 0x%02x\n",
             esp32_i2c_get_device_type_name(device->base.type), device->base.address);
    return true;
}

bool esp32_i2c_remove_device(Esp32I2CState *s, uint8_t address)
{
    for (int i = 0; i < s->device_count; i++) {
        if (s->virtual_devices[i].base.address == address) {
            // Shift remaining devices
            for (int j = i; j < s->device_count - 1; j++) {
                s->virtual_devices[j] = s->virtual_devices[j + 1];
            }
            s->device_count--;
            
            qemu_log("esp32_i2c: Phase 8 - Device removed: 0x%02x\n", address);
            return true;
        }
    }
    return false;
}

void esp32_i2c_scan_devices(Esp32I2CState *s)
{
    qemu_log("esp32_i2c: Phase 8 - Device scan: Found %d devices\n", s->device_count);
    
    for (int i = 0; i < s->device_count; i++) {
        I2CVirtualDevice *device = &s->virtual_devices[i];
        qemu_log("esp32_i2c: Phase 8 - Device %d: %s at 0x%02x (State: %s, Access: %d)\n",
                 i, esp32_i2c_get_device_type_name(device->base.type),
                 device->base.address, esp32_i2c_get_device_state_name(device->base.state),
                 device->base.access_count);
    }
}

void esp32_i2c_log_device_event(I2CVirtualDevice *device, const char *event)
{
    qemu_log("esp32_i2c: Phase 8 - %s 0x%02x %s (State: %s, Access: %d)\n",
             esp32_i2c_get_device_type_name(device->base.type), device->base.address, event,
             esp32_i2c_get_device_state_name(device->base.state), device->base.access_count);
}

/* Phase 8: Device Response Simulation Implementation */

bool esp32_i2c_simulate_device_ack(Esp32I2CState *s, uint8_t device_address)
{
    if (s->vdev_i2c_set) {
        const I2CDeviceEntry *e = i2c_device_set_find(s->vdev_i2c_set, device_address);
        if (e && e->ops) {
            /* If device has i2c_can_ack, use it; otherwise default ACK (passthrough) */
            if (e->ops->i2c_can_ack) {
                return e->ops->i2c_can_ack(e->device, device_address, false);
            }
            return true;
        }
    }
    I2CVirtualDevice *device = esp32_i2c_find_device_by_address(s, device_address);
    if (!device || !device->base.responding) {
        return false;
    }
    
    // Simulate device-specific ACK behavior
    switch (device->base.type) {
    case I2C_DEVICE_TYPE_TMP105:
        return !((TMP105Device*)device)->shutdown_mode;
    case I2C_DEVICE_TYPE_EEPROM:
        return !((EEPROMDevice*)device)->write_protect;
    case I2C_DEVICE_TYPE_RTC:
        return ((RTCDevice*)device)->oscillator_enabled;
    case I2C_DEVICE_TYPE_SSD1306:
        return true;  // Always responds
    default:
        return true;
    }
}

void esp32_i2c_simulate_device_data(Esp32I2CState *s, uint8_t device_address, uint8_t *data, size_t length)
{
    if (s->vdev_i2c_set) {
        const I2CDeviceEntry *e = i2c_device_set_find(s->vdev_i2c_set, device_address);
        if (e && e->ops && e->ops->i2c_read) {
            if (e->ops->i2c_on_addressed) e->ops->i2c_on_addressed(e->device, device_address, true);
            ssize_t n = e->ops->i2c_read(e->device, device_address, data, length);
            if (n > 0 && (size_t)n < length) memset(data + n, 0x00, length - (size_t)n);
            if (n <= 0) memset(data, 0x00, length);
            return;
        }
    }
    I2CVirtualDevice *device = esp32_i2c_find_device_by_address(s, device_address);
    if (!device || !device->base.responding) {
        return;
    }
    
    // Call device-specific read function
    switch (device->base.type) {
    case I2C_DEVICE_TYPE_TMP105:
        tmp105_process_read((TMP105Device*)device, data, length);
        break;
    case I2C_DEVICE_TYPE_EEPROM:
        eeprom_process_read((EEPROMDevice*)device, data, length);
        break;
    case I2C_DEVICE_TYPE_RTC:
        rtc_process_read((RTCDevice*)device, data, length);
        break;
    case I2C_DEVICE_TYPE_SSD1306:
        ssd1306_process_read((SSD1306Device*)device, data, length);
        break;
    case I2C_DEVICE_TYPE_UNKNOWN:
    default:
        // Unknown device type - send zeros
        memset(data, 0x00, length);
        break;
    }
}

void esp32_i2c_handle_device_response(Esp32I2CState *s, uint8_t device_address)
{
    I2CVirtualDevice *device = esp32_i2c_find_device_by_address(s, device_address);
    if (!device) {
        return;
    }
    
    device->base.state = I2C_DEVICE_STATE_ADDRESSED;
    device->base.access_count++;
    device->base.last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (device->base.type == I2C_DEVICE_TYPE_TMP105) {
        ((TMP105Device*)device)->read_phase = 0; // start MSB on next read window
    }
    
    if (s->device_debug_enabled) {
        qemu_log("esp32_i2c: Device 0x%02x addressed (Type: %s)\n",
                 device_address, esp32_i2c_get_device_type_name(device->base.type));
    }
}

/* Phase 8: Helper Functions Implementation */

const char* esp32_i2c_get_device_type_name(I2CVirtualDeviceType type)
{
    static const char* names[] = {
        "TMP105", "EEPROM", "RTC", "SSD1306", "UNKNOWN"
    };
    
    if (type < I2C_DEVICE_TYPE_UNKNOWN) {
        return names[type];
    }
    return names[I2C_DEVICE_TYPE_UNKNOWN];
}

const char* esp32_i2c_get_device_state_name(I2CVirtualDeviceState state)
{
    static const char* names[] = {
        "IDLE", "ADDRESSED", "READING", "WRITING", "ERROR"
    };
    
    if (state <= I2C_DEVICE_STATE_ERROR) {
        return names[state];
    }
    return "UNKNOWN";
}

void esp32_i2c_update_device_access_time(I2CVirtualDevice *device)
{
    device->base.last_access_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    device->base.access_count++;
}

bool esp32_i2c_is_device_responding(I2CVirtualDevice *device)
{
    return device && device->base.responding && device->base.present;
}
