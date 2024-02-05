#ifndef INC_LCD_H_
#define INC_LCD_H_


#include "main.h"
#include <stdint.h>

// GPIO Pins Configuration
#define RS_PIN       GPIO_PIN_0  // Replace with the appropriate pin
#define RW_PIN       GPIO_PIN_1  // Replace with the appropriate pin
#define E_PIN        GPIO_PIN_2  // Replace with the appropriate pin
#define DP0          GPIO_PIN_3
#define DP1          GPIO_PIN_4
#define DP2          GPIO_PIN_5
#define DP3          GPIO_PIN_6
#define DP4          GPIO_PIN_7
#define DP5          GPIO_PIN_8
#define DP6          GPIO_PIN_9
#define DP7          GPIO_PIN_10

#define CLEAR_BYTE    0xFF
#define CLEAR_PC      0x3FFFFF
#define PC_OUT        0x155555
#define DATA_SHIFT    3

#define WAKE_UP       0x30
#define FUNC_SET      0x38
#define DISPLAY_OFF   0xE
#define ENTRY_SET     0x6
#define CLEAR_DISPLAY 0x1
#define SET_CURSOR    0x80

// Function prototypes
void LCD_init(void);                  // initialize LCD
void LCD_command(uint8_t command);    // Send LCD a single 8-bit command
void LCD_write_char(uint8_t letter);  // write a character to the LCD
void LCD_write_string(char *str);     // write string to LCD
void LCD_set_cursor(char x, char y);  // set cursor to skip a line or write to desired LCD address
void latch_falling_edge(void);        // Pulse a E signal low-high-low for data latch


#endif /* INC_LCD_H_ */
