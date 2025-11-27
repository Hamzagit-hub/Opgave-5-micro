#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>

// Function prototypes
void i2c_initialize(uint32_t smclk_freq, uint32_t i2c_freq);
uint8_t i2c_send_data(uint8_t device_addr, const uint8_t *data_buffer, uint8_t data_length);
uint8_t i2c_send_byte(uint8_t device_addr, uint8_t data_byte);
uint8_t i2c_receive_data(uint8_t device_addr, uint8_t *receive_buffer, uint8_t byte_count);
uint8_t i2c_receive_byte(uint8_t device_addr);
uint8_t i2c_bus_ready(void);
void i2c_stop_condition(void);

#endif // I2C_DRIVER_H