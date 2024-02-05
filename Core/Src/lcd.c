/*
 * lcd.c
 *
 *  Created on: Jan 29, 2024
 *      Author: aoberai
 */

#include "lcd.h"

void LCD_init(void) {

	//reset control pins
	GPIOC->ODR &= ~(E_PIN);
	GPIOC->ODR &= ~(RS_PIN);

	HAL_Delay(50);

	//wake the display up
	LCD_command(WAKE_UP);
	HAL_Delay(100);
	LCD_command(WAKE_UP);
	HAL_Delay(10);
	LCD_command(WAKE_UP);
	HAL_Delay(10);

	//function set the display
	LCD_command(FUNC_SET);

	HAL_Delay(5);

	LCD_command(DISPLAY_OFF);

	HAL_Delay(5);

	LCD_command(CLEAR_DISPLAY);

	HAL_Delay(5);

	//entry mode set for data
	LCD_command(ENTRY_SET);

	HAL_Delay(5);

}

void LCD_command(uint8_t command) {

	//clear GPIO_PINS 3-10
	GPIOC->ODR &= ~(CLEAR_BYTE << DATA_SHIFT);

	//shift instruction to appropriate GPIO_PINS 3-10
	GPIOC->ODR |= (command << DATA_SHIFT);

	//reset RS Pin for Instruction mode
	GPIOC->ODR &= ~(RS_PIN);

	//reset RW Pin for Write mode
	GPIOC->ODR &= ~(RW_PIN);

	HAL_Delay(10);

	latch_falling_edge();

	//put LCD in read mode to prevent unintended command writes
	GPIOC->ODR |= RS_PIN;
	GPIOC->ODR |= RW_PIN;

	HAL_Delay(10);
}

void LCD_write_char(uint8_t letter) {

	//clear GPIO_PINS 3-10
	GPIOC->ODR &= ~(CLEAR_BYTE << DATA_SHIFT);

	//shift data to appropriate GPIO_PINS 3-10
	GPIOC->ODR |= (letter << DATA_SHIFT);

	//set RS PIN for Data mode
	GPIOC->ODR |= RS_PIN;

	//reset RW PIN for Write mode
	GPIOC->ODR &= ~(RW_PIN);

	HAL_Delay(10);

	latch_falling_edge();

	//put LCD in read mode to prevent unintended command writes
	GPIOC->ODR |= RS_PIN;
	GPIOC->ODR |= RW_PIN;

	HAL_Delay(10);
}

void LCD_write_string(char *str) {

	for(; *str != 0; ++str) {

		LCD_write_char(*str);
	}

}

void LCD_set_cursor(char x, char y) {
	uint8_t origin = 0x0;

	if(y == 1) {
		origin = 0x40;
	}

	LCD_command(SET_CURSOR | (origin + x));
}

void latch_falling_edge(void) {

	//create high-low negative edge pulse
	GPIOC->ODR |= E_PIN;
	HAL_Delay(1);
	GPIOC->ODR &= ~(E_PIN);
	HAL_Delay(1);
}


