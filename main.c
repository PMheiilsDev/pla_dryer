#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "lcd.h"

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

    // Define a custom character (e.g., a smiley face)
    uint8_t smiley[8] = 
    {
        0b11111,
        0b10101,
        0b11011,
        0b10101,
        0b11011,
        0b10101,
        0b11011,
        0b11111
    };

    // Create the custom character in CGRAM location 0
    lcd_create_char(0, smiley);

    // Display the custom character
    lcd_set_cursor(0, 0); // Set cursor to row 0, column 0
    lcd_write(0, true);   // Write custom character (index 0)

    while (1)
    {
        for (uint8_t i = 0; i < LCD_COLS; i++)
        {
            lcd_set_cursor(0, 0);
            char buf[3];
            snprintf(buf, sizeof(buf), "%02d", i);
            lcd_write_string(buf);
            lcd_set_cursor(1, i); // Set cursor to row 1, column i
            lcd_write(0, true);   // Write custom character (index 0)
            sleep_ms(500);
            lcd_set_cursor(1, i);
            lcd_write_string(" ");
        }
    }

    return 0;
}
