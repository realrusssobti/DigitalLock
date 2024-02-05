//
// Created by Russ Sobti on 2/5/24.
//

#ifndef DIGITALLOCK_KEYPAD_H
#define DIGITALLOCK_KEYPAD_H
#define R1_PORT GPIOB
#define R1_PIN GPIO_PIN_0
#define R2_PORT GPIOB
#define R2_PIN GPIO_PIN_1
#define R3_PORT GPIOB
#define R3_PIN GPIO_PIN_2
#define R4_PORT GPIOB
#define R4_PIN GPIO_PIN_3
#define C1_PORT GPIOB
#define C1_PIN GPIO_PIN_4
#define C2_PORT GPIOB
#define C2_PIN GPIO_PIN_5
#define C3_PORT GPIOB
#define C3_PIN GPIO_PIN_6
#define C4_PORT GPIOB
#define C4_PIN GPIO_PIN_7

#endif //DIGITALLOCK_KEYPAD_H



void initKeypad(void);
int readKeypad(void);
char readChar(void);
char decodeKey(int key);
