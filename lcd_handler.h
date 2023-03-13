#pragma once

#include <driver/i2c.h>
#include <esp_log.h>
#include "rom/ets_sys.h"

void LCD_init(uint8_t addr, uint8_t cols, uint8_t rows);
void LCD_setCursor(uint8_t col, uint8_t row);
void LCD_home(void);
void LCD_clearScreen(void);
void LCD_writeChar(char c);
void LCD_writeStr(char* str); 