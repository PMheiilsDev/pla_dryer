#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "lcd.h"

// Rotary encoder pins
#define ROTARY_PIN_A 4
#define ROTARY_PIN_B 5

// Rotary encoder states
volatile int rotary_position = 0;
volatile int previous_position = 0; // Tracks the previous position
volatile uint8_t rotary_state = 0;  // Current state of the rotary encoder
volatile uint8_t edge_counter = 0;  // Counter to track edges

// Rotary encoder state machine
const int8_t ROTARY_STATES[16] = {
    0,  // 0000: No movement
    -1, // 0001: Counterclockwise
    1,  // 0010: Clockwise
    0,  // 0011: Invalid
    1,  // 0100: Clockwise
    0,  // 0101: No movement
    0,  // 0110: Invalid
    -1, // 0111: Counterclockwise
    -1, // 1000: Counterclockwise
    0,  // 1001: No movement
    0,  // 1010: Invalid
    1,  // 1011: Clockwise
    0,  // 1100: Invalid
    1,  // 1101: Clockwise
    -1, // 1110: Counterclockwise
    0   // 1111: No movement
};

// Rotary encoder interrupt handler
void rotary_encoder_isr(uint gpio, uint32_t events) {
    // Read the state of the rotary encoder pins
    bool pin_a = gpio_get(ROTARY_PIN_A);
    bool pin_b = gpio_get(ROTARY_PIN_B);

    // Update the rotary state (2 bits for each pin)
    rotary_state = ((rotary_state << 2) | (pin_a << 1) | pin_b) & 0x0F;

    // Increment the edge counter
    edge_counter++;

    // Only process movement on every 2nd edge
    if (edge_counter >= 2) {
        // Determine the direction of rotation
        int8_t movement = ROTARY_STATES[rotary_state];
        rotary_position += movement;

        // Clamp the rotary position to valid LCD column range
        if (rotary_position < 0) {
            rotary_position = 0;
        } else if (rotary_position >= LCD_COLS) {
            rotary_position = LCD_COLS - 1;
        }

        // Reset the edge counter
        edge_counter = 0;
    }
}

int main() {
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
    uint8_t smiley[8] = {
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

    // Initialize rotary encoder pins
    gpio_init(ROTARY_PIN_A);
    gpio_set_dir(ROTARY_PIN_A, GPIO_IN);
    gpio_pull_up(ROTARY_PIN_A);

    gpio_init(ROTARY_PIN_B);
    gpio_set_dir(ROTARY_PIN_B, GPIO_IN);
    gpio_pull_up(ROTARY_PIN_B);

    // Attach interrupts to the rotary encoder pins
    gpio_set_irq_enabled_with_callback(ROTARY_PIN_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &rotary_encoder_isr);
    gpio_set_irq_enabled(ROTARY_PIN_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    while (1) {
        // Use the rotary position to control the LCD
        lcd_set_cursor(0, 0);
        char buf[3];
        snprintf(buf, sizeof(buf), "%02d", rotary_position);
        lcd_write_string(buf);

        // Clear the previous position
        lcd_set_cursor(1, previous_position);
        lcd_write_string(" ");

        // Update the new position
        lcd_set_cursor(1, rotary_position);
        lcd_write(0, true); // Write custom character (index 0)

        // Store the current position as the previous position
        previous_position = rotary_position;

        sleep_ms(100);
    }

    return 0;
}
