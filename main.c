#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define PCF8574_ADDR 0x27 // Address of the PCF8574
#define LCD_ROWS 2
#define LCD_COLS 16

// PCF8574 to LCD pin mapping
#define LCD_RS 0x01        // Register Select bit
#define LCD_RW 0x02        // Read/Write bit (not used, always write)
#define LCD_EN 0x04        // Enable bit
#define LCD_BACKLIGHT 0x08 // Backlight control bit

// Function prototypes
void lcd_write(uint8_t data, bool is_data);
void lcd_send_nibble(uint8_t nibble, bool is_data);
void lcd_init();
void lcd_clear();
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_write_string(const char *str);

void lcd_write(uint8_t data, bool is_data)
{
    // Send high nibble
    lcd_send_nibble(data >> 4, is_data);
    // Send low nibble
    lcd_send_nibble(data & 0x0F, is_data);
}

void lcd_send_nibble(uint8_t nibble, bool is_data)
{
    uint8_t control = LCD_BACKLIGHT;
    if (is_data)
    {
        control |= LCD_RS;
    }
    // Send nibble to PCF8574
    uint8_t data = (nibble << 4) | control;
    i2c_write_blocking(I2C_PORT, PCF8574_ADDR, &data, 1, false);
    // Pulse the enable pin
    data |= LCD_EN;
    i2c_write_blocking(I2C_PORT, PCF8574_ADDR, &data, 1, false);
    data &= ~LCD_EN;
    i2c_write_blocking(I2C_PORT, PCF8574_ADDR, &data, 1, false);
    sleep_us(50); // Short delay for LCD to process
}

void lcd_init()
{
    sleep_ms(50);                 // Wait for LCD to power up
    lcd_send_nibble(0x03, false); // Initialize in 4-bit mode
    sleep_ms(5);
    lcd_send_nibble(0x03, false);
    sleep_us(150);
    lcd_send_nibble(0x03, false);
    lcd_send_nibble(0x02, false); // Set to 4-bit mode

    // Function set: 4-bit mode, 2 lines, 5x8 dots
    lcd_write(0x28, false);
    // Display control: Display on, cursor off, blink off
    lcd_write(0x0C, false);
    // Clear display
    lcd_clear();
    // Entry mode set: Increment cursor, no shift
    lcd_write(0x06, false);
}

void lcd_clear()
{
    lcd_write(0x01, false); // Clear display command
    sleep_ms(2);            // Delay for clear command
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t address = col + (row == 0 ? 0x00 : 0x40);
    lcd_write(0x80 | address, false); // Set DDRAM address command
}

void lcd_write_string(const char *str)
{
    while (*str)
    {
        lcd_write(*str++, true); // Write character to LCD
    }
}

/*------------- MAIN -------------*/
int main()
{
    // Initialize stdio (not used for printing)
    stdio_init_all();

    // Initialize I2C
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz
    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_pull_up(16);
    gpio_pull_up(17);

    // Initialize the LCD
    lcd_init();

    
    while (1)
    {
        for (uint8_t i = 0; i < LCD_COLS; i++)
        {
            lcd_set_cursor(0, 0);
            char buf[3];
            snprintf(buf, sizeof(buf), "%02d", i);
            lcd_write_string(buf);
            lcd_set_cursor(1, i); // Set cursor to row 1, column i
            lcd_write_string("X"); // Write string to LCD
            sleep_ms(500);
            lcd_set_cursor(1, i);
            lcd_write_string(" ");
        }

    }

    return 0;
}
