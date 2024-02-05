//
// Created by Russ Sobti on 2/5/24.
//

#include "keypad.h"
#include "main.h"
// Define the keypad layout (4x4)
char keypadLayout[4][4] = {
		{'1', '2', '3', 'A'},
		{'4', '5', '6', 'B'},
		{'7', '8', '9', 'C'},
		{'*', '0', '#', 'D'}
};
void initKeypad(void){
	// turns on clock to GPIO bank B
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
	// Configure GPIO pins for rows (output)
	// GPIOB pins 0-3 for rows (output)
	GPIOB->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	GPIOB->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0;

	// Configure GPIO pins for columns (input with pull-up)
	// GPIOB pins 4-7 for columns (input with pull-up)
	GPIOB->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;

	return;
}

int readKeypad(void){
	// Make ROWs LOW and all other ROWs HIGH
	for (int row = 0; row < 4; row++) {
		// Pull the current ROW low
		switch (row) {
			case 0:
				HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_RESET);
				break;
			case 1:
				HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_RESET);
				break;
			case 2:
				HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_RESET);
				break;
			case 3:
				HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_RESET);
				break;
			default:
				break;
		}

		// Check each column
		for (int col = 0; col < 4; col++) {
			// Pull the current COLUMN high
			switch (col) {
				case 0:
					HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_SET);
					break;
				case 1:
					HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_SET);
					break;
				case 2:
					HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_SET);
					break;
				case 3:
					HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_SET);
					break;
				default:
					break;
			}

			// Check if the button is pressed
			if (!HAL_GPIO_ReadPin(C1_PORT, C1_PIN)) {
				while (!HAL_GPIO_ReadPin(C1_PORT, C1_PIN));
				return (row * 4) + col;
			} else if (!HAL_GPIO_ReadPin(C2_PORT, C2_PIN)) {
				while (!HAL_GPIO_ReadPin(C2_PORT, C2_PIN));
				return (row * 4) + col + 1;
			} else if (!HAL_GPIO_ReadPin(C3_PORT, C3_PIN)) {
				while (!HAL_GPIO_ReadPin(C3_PORT, C3_PIN));
				return (row * 4) + col + 2;
			} else if (!HAL_GPIO_ReadPin(C4_PORT, C4_PIN)) {
				while (!HAL_GPIO_ReadPin(C4_PORT, C4_PIN));
				return (row * 4) + col + 3;
			}

			// Release the current COLUMN
			switch (col) {
				case 0:
					HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_RESET);
					break;
				case 1:
					HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_RESET);
					break;
				case 2:
					HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_RESET);
					break;
				case 3:
					HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_RESET);
					break;
				default:
					break;
			}
		}

		// Release the current ROW
		switch (row) {
			case 0:
				HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_SET);
				break;
			case 1:
				HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_SET);
				break;
			case 2:
				HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_SET);
				break;
			case 3:
				HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_SET);
				break;
			default:
				break;
		}
	}

	// No button pressed
	return -1;
}

char readChar() {
	int key = readKeypad();
	if (key == -1) {
		return '\0';
	} else {
		return decodeKey(key);
	}
}

char decodeKey(int key) {
	return keypadLayout[key / 4][key % 4];
}
