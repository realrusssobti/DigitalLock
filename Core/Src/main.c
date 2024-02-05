/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lcd.h"
#include "keypad.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void setLEDState(int state){
	if (state == 1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}
void displayLCD( char *line_one,  char *line_two) {
	char* blank_string = "                ";
	LCD_set_cursor(0,0);
	LCD_write_string(blank_string);
	LCD_set_cursor(0,0);
	LCD_write_string(line_one);
	LCD_set_cursor(0,1);
	LCD_write_string(blank_string);
	LCD_set_cursor(0,1);
	LCD_write_string(line_two);
}


/* USER CODE BEGIN PFP */
int main(void) {

    // Initialize the HAL Library
    HAL_Init();

    // System Clock Configuration
    SystemClock_Config();

    // turns on clock to GPIO banks C
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOAEN);

    //bank PC0-PC10 as all GPIO outputs
    GPIOC->MODER &= ~(CLEAR_PC);
    GPIOC->MODER |=   PC_OUT;

	// set up green LED light on pin PA5
	GPIOA->MODER &= ~(GPIO_MODER_MODE5);
	GPIOA->MODER |= GPIO_MODER_MODE5_0 ;


    LCD_init();
    char* line_one = malloc(17);
    char* line_two = malloc(17);
	strcpy(line_one, "Hello World");
	strcpy(line_two, "Assignment 3");
	displayLCD(line_one, line_two);


	// Initial state
	enum State currentState = POWER_UP;

	// Pin variables
	char enteredPin[5] = {'\0'};
	char correctPin[] = "1234";
	initKeypad(); // initialize the keypad
	char *buf_one = malloc(17);
	char *buf_two = malloc(17);

	while (currentState != END) {
		switch (currentState) {
			case POWER_UP:
				strcpy(buf_one, "Locked");
				strcpy(buf_two, "");
				if(strcmp(buf_one, line_one) != 0 || strcmp(buf_two, line_two) != 0) displayLCD(buf_one, buf_two);
				// copy over the buffer to the previous buffer
				strcpy(line_one, buf_one);
				strcpy(line_two, buf_two);

				setLEDState(1); // LED ON
				currentState = LOCKED;
				break;

			case LOCKED:
				strcpy(buf_one, "Enter PIN: ");
				strcpy(buf_two, enteredPin);

				if(strcmp(buf_one, line_one) != 0 || strcmp(buf_two, line_two) != 0) displayLCD(buf_one, buf_two);
				// copy over the buffer to the previous buffer
				strcpy(line_one, buf_one);
				strcpy(line_two, buf_two);
				currentState = ENTER_PIN;
				break;

			case CLEAR_PIN:
				strcpy(buf_one, "Clearing PIN");
				strcpy(buf_two, "");
				if(strcmp(buf_one, line_one) != 0 || strcmp(buf_two, line_two) != 0) displayLCD(buf_one, buf_two);
				// copy over the buffer to the previous buffer
				strcpy(line_one, buf_one);
				strcpy(line_two, buf_two);
				memset(enteredPin, 0, sizeof(enteredPin));
				currentState = ENTER_PIN;
				break;

			case ENTER_PIN:
			{
				char enteredChar;
//				scanf(" %c", &digit);
				// read from the keypad
				enteredChar = readChar();

				if (enteredChar == '*') {
					currentState = CLEAR_PIN;
				} else if (enteredChar == '#') {
					// check if correct pin is empty
					if (strlen(correctPin) == 0) {
						// set the correct pin to the entered pin
						strcpy(correctPin, enteredPin);
					} else
					if (strcmp(enteredPin, correctPin) == 0) {
						currentState = UNLOCK;
					} else {
						currentState = INCORRECT_PIN;
					}
				} else if (enteredChar!='\0' ) {
					enteredPin[strlen(enteredPin) % 4] = enteredChar;
					strcpy(buf_one, "Current PIN: ");
					strcpy(buf_two, enteredPin);
					if(strcmp(buf_one, line_one) != 0 || strcmp(buf_two, line_two) != 0) displayLCD(buf_one, buf_two);
					// copy over the buffer to the previous buffer
					strcpy(line_one, buf_one);
					strcpy(line_two, buf_two);
				} else {
					strcpy(buf_one, "Current PIN: ");
					strcpy(buf_two, enteredPin);
					if(strcmp(buf_one, line_one) != 0 || strcmp(buf_two, line_two) != 0) displayLCD(buf_one, buf_two);
					// copy over the buffer to the previous buffer
					strcpy(line_one, buf_one);
					strcpy(line_two, buf_two);
				}
			}
				break;

			case INCORRECT_PIN:
				strcpy(buf_one, "Locked");
				strcpy(buf_two, "Incorrect PIN");
				if(strcmp(buf_one, line_one) != 0 || strcmp(buf_two, line_two) != 0) displayLCD(buf_one, buf_two);
				// copy over the buffer to the previous buffer
				strcpy(line_one, buf_one);
				strcpy(line_two, buf_two);
				setLEDState(1); // LED ON
				currentState = CLEAR_PIN;
				break;

			case UNLOCK:
				strcpy(buf_one, "Unlocked");
				strcpy(buf_two, "* to Lock");

				if(strcmp(buf_one, line_one) != 0 || strcmp(buf_two, line_two) != 0) displayLCD(buf_one, buf_two);
				// copy over the buffer to the previous buffer
				strcpy(line_one, buf_one);
				strcpy(line_two, buf_two);
				setLEDState(0); // LED OFF
				printf("Access Granted!\n");
				currentState = CHANGE_PIN;
				break;

			case CHANGE_PIN:
				strcpy(buf_one, "* to Lock");
				strcpy(buf_two, "# to Change");
				if(strcmp(buf_one, line_one) != 0 || strcmp(buf_two, line_two) != 0) displayLCD(buf_one, buf_two);
				// copy over the buffer to the previous buffer
				strcpy(line_one, buf_one);
				strcpy(line_two, buf_two);
				// Reset the entered pin to ""
				char enteredChar = readChar();
				// read the next input for the next state
				if (enteredChar == '#') {
					// reset the correct pin to ""
					correctPin[0] = '\0';
					currentState = CLEAR_PIN;
				}
				else if (enteredChar == '*') {
					currentState = CLEAR_PIN;
					setLEDState(1);
				}
				else currentState = CHANGE_PIN;


				// Placeholder for pin change functionality
				break;

			default:
				break;
		}
	}
	free(buf_one);
	free(buf_two);
	free(line_one);
	free(line_two);
	return 0;
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
