#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include "hardware/i2c.h"

// LCD configuration
#define I2C_PORT i2c0
#define PCF8574_ADDR 0x27 // Address of the PCF8574
#define LCD_ROWS 2
#define LCD_COLS 16

// Function prototypes
void lcd_init();
void lcd_clear();
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_write_string(const char *str);
void lcd_create_char(uint8_t location, uint8_t charmap[8]);

// Helper functions (exposed for external use)
void lcd_write(uint8_t data, bool is_data);
void lcd_send_nibble(uint8_t nibble, bool is_data);

#endif // LCD_H