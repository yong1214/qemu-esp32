/**
 * ESP32 I2C Virtual Device APIs
 * 
 * Device-specific, encapsulated APIs for Phase 8 Virtual Device Integration
 * Provides professional-grade device simulation with hardware-accurate behavior
 */

#ifndef ESP32_I2C_DEVICES_H
#define ESP32_I2C_DEVICES_H

#include "qemu/osdep.h"
#include "hw/i2c/esp32_i2c.h"

/* Phase 8: Device-Specific API Declarations */

/* TMP105 Temperature Sensor APIs */
void tmp105_init(TMP105Device *device, uint8_t address, float initial_temperature);
void tmp105_reset(TMP105Device *device);
void tmp105_process_read(TMP105Device *device, uint8_t *data, size_t length);
void tmp105_process_write(TMP105Device *device, uint8_t *data, size_t length);
bool tmp105_handle_register_access(TMP105Device *device, uint8_t register_addr, bool is_write);
void tmp105_update_temperature(TMP105Device *device, float new_temperature);
void tmp105_log_event(TMP105Device *device, const char *event);

/* EEPROM Memory Device APIs */
void eeprom_init(EEPROMDevice *device, uint8_t address, size_t memory_size);
void eeprom_reset(EEPROMDevice *device);
void eeprom_process_read(EEPROMDevice *device, uint8_t *data, size_t length);
void eeprom_process_write(EEPROMDevice *device, uint8_t *data, size_t length);
bool eeprom_handle_address_write(EEPROMDevice *device, uint8_t *address_data, size_t length);
void eeprom_write_page(EEPROMDevice *device, uint16_t address, uint8_t *data, size_t length);
void eeprom_log_event(EEPROMDevice *device, const char *event);

/* RTC Real-Time Clock APIs */
void rtc_init(RTCDevice *device, uint8_t address);
void rtc_reset(RTCDevice *device);
void rtc_process_read(RTCDevice *device, uint8_t *data, size_t length);
void rtc_process_write(RTCDevice *device, uint8_t *data, size_t length);
bool rtc_handle_register_access(RTCDevice *device, uint8_t register_addr, bool is_write);
void rtc_update_time(RTCDevice *device, uint8_t hours, uint8_t minutes, uint8_t seconds);
void rtc_log_event(RTCDevice *device, const char *event);

/* SSD1306 OLED Display APIs */
void ssd1306_init(SSD1306Device *device, uint8_t address);
void ssd1306_reset(SSD1306Device *device);
void ssd1306_process_read(SSD1306Device *device, uint8_t *data, size_t length);
void ssd1306_process_write(SSD1306Device *device, uint8_t *data, size_t length);
bool ssd1306_handle_command(SSD1306Device *device, uint8_t command);
void ssd1306_set_pixel(SSD1306Device *device, uint8_t x, uint8_t y, bool on);
void ssd1306_log_event(SSD1306Device *device, const char *event);

/* Generic Device Management APIs */
I2CVirtualDevice* esp32_i2c_find_device_by_address(Esp32I2CState *s, uint8_t address);
I2CVirtualDevice* esp32_i2c_find_device_by_type(Esp32I2CState *s, I2CVirtualDeviceType type);
bool esp32_i2c_add_device(Esp32I2CState *s, I2CVirtualDevice *device);
bool esp32_i2c_remove_device(Esp32I2CState *s, uint8_t address);
void esp32_i2c_scan_devices(Esp32I2CState *s);
void esp32_i2c_log_device_event(I2CVirtualDevice *device, const char *event);

/* Device Response Simulation APIs */
bool esp32_i2c_simulate_device_ack(Esp32I2CState *s, uint8_t device_address);
void esp32_i2c_simulate_device_data(Esp32I2CState *s, uint8_t device_address, uint8_t *data, size_t length);
void esp32_i2c_handle_device_response(Esp32I2CState *s, uint8_t device_address);

/* Device-Specific Helper Functions */
const char* esp32_i2c_get_device_type_name(I2CVirtualDeviceType type);
const char* esp32_i2c_get_device_state_name(I2CVirtualDeviceState state);
void esp32_i2c_update_device_access_time(I2CVirtualDevice *device);
bool esp32_i2c_is_device_responding(I2CVirtualDevice *device);

#endif /* ESP32_I2C_DEVICES_H */


