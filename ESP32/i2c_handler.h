#pragma once
#ifndef I2C_INCLUDED
#define I2C_INCLUDED

#include <driver/i2c.h>
#include <car_pin.h>
#include <esp_lcd_types.h>
#include <esp_lcd_panel_io.h>

#define I2C_PORT I2C_NUM_0

// Toggle I2C bus on powering up to avoid bus hanging
int I2C_clearBus(gpio_num_t sda, gpio_num_t scl);

// Initializing i2c on pin 21 and 22, 
void init_i2c();
void send_command_to_uno(uint8_t byte, uint8_t slave_address, int num_bytes);
void received_data_from_uno(uint8_t* data_buffer, uint8_t slave_address,  uint8_t num_byte);

#endif