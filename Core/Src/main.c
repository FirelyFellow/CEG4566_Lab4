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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Keypad4X4.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fonts.h"
#include "ssd1306.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId task01Handle;
osThreadId task02Handle;
osThreadId task03Handle;
osMessageQId queue01Handle;
/* USER CODE BEGIN PV */
extern char key;
char hold[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void BuzzerTask(void const *argument);
void LCDTask(void const *argument);
void KeypadTask(void const *argument);
void PIRTask(void const *argument);

int buzzer = 0; //Bool to check if buzzer is on or off
int movement = 0; //Bool to check if mouvement is detected or not
int armed = 0; //Bool to check if system is armed or not

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	SSD1306_Init();
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("Set Code", &Font_11x18, 1);
	HAL_UART_Transmit(&huart2, (uint8_t *)"Set Code\r\n", 11, 100);
	SSD1306_GotoXY(0, 30);
	HAL_UART_Transmit(&huart2, (uint8_t *)"Code:", 5, 100);
	SSD1306_Puts("Code:", &Font_11x18, 1);
	SSD1306_UpdateScreen();


	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of queue01 */
	osMessageQDef(queue01, 16, char);
	queue01Handle = osMessageCreate(osMessageQ(queue01), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, BuzzerTask, osPriorityAboveNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of task01 */
	osThreadDef(task01, LCDTask, osPriorityAboveNormal, 0, 128);
	task01Handle = osThreadCreate(osThread(task01), NULL);

	/* definition and creation of task02 */
	osThreadDef(task02, KeypadTask, osPriorityNormal, 0, 128);
	task02Handle = osThreadCreate(osThread(task02), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	osThreadDef(task03, PIRTask, osPriorityAboveNormal, 0, 128);
	task03Handle = osThreadCreate(osThread(task03), NULL);
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */
	/* Start scheduler */
	osKernelStart();
	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, KC0_Pin | KC3_Pin | KC1_Pin | KC2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : KC0_Pin KC3_Pin KC1_Pin KC2_Pin */
	GPIO_InitStruct.Pin = KC0_Pin | KC3_Pin | KC1_Pin | KC2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : KR1_Pin */
	GPIO_InitStruct.Pin = KR1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(KR1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : KR3_Pin KR2_Pin */
	GPIO_InitStruct.Pin = KR3_Pin | KR2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : KR0_Pin */
	GPIO_InitStruct.Pin = KR0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(KR0_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
void BuzzerTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		if (armed && buzzer) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET); // activate the buzzer
			osDelay(100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET); // stop the buzzer
			osDelay(100);
		} else {
			// If the system is not armed or does not have a buzzer, make sure the buzzer is off
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
			osDelay(100);
		}

		osDelay(100);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LCDTask */
/**
 * @brief Function implementing the task01 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LCDTask */
void LCDTask(void const *argument) {
	/* USER CODE BEGIN LCDTask */
	/* Infinite loop */

	char receivedKey;
	uint8_t codeLength = 0; // Track the number of entered characters
	char userCode[7] = { 0 }; // Array to store the entered code
	uint8_t maxCodeLength = 6; // Adjust based on requirements
	uint8_t minCodeLength = 4;
	char correctCode[7] = { 0 }; // Example of a correct code for comparison

	//Use to check if first time that the system is Used
	//If 1, means true so the user must enter the new password for system
	int initial = 1;

	for (;;) {

		if (armed) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
			//osDelay(2000);
		} else {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		}

		if(initial){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		}

		//	Use queue to keep track of key pressed
		if (xQueueReceive(queue01Handle, &receivedKey, portMAX_DELAY) == pdPASS) {

			//Used at the start of the application so the user can enter the code that will be use to armed and unarmed the system
			if (receivedKey == '#' && codeLength >= minCodeLength && initial) { // Enter/confirm key or max length reached
				// Check if the entered code is correct
				strncpy(correctCode, userCode, sizeof(correctCode));

				//Set the userCode as the new password
				SSD1306_Clear();
				SSD1306_GotoXY(0, 0);
				HAL_UART_Transmit(&huart2, (uint8_t *)"Code Set\r\n", 10, 100);
				SSD1306_Puts("Code Set", &Font_11x18, 1);
				SSD1306_UpdateScreen();

				HAL_Delay(2000);

				armed = 1;

				//Set as armed
				SSD1306_Clear();
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts("Armed", &Font_11x18, 1);
				HAL_UART_Transmit(&huart2, (uint8_t *)"Armed\r\n", 8, 100);
				SSD1306_GotoXY(0, 30); // Adjust Y position based on your font size
				HAL_UART_Transmit(&huart2, (uint8_t *)"Code:", 5, 100);
				SSD1306_Puts("Code:", &Font_11x18, 1);

				memset(userCode, 0, sizeof(userCode));
				codeLength = 0;


				initial = 0;
			}

			// Enter/confirm key or max length reached
				else if (receivedKey == '#' && codeLength >= minCodeLength && !initial) {
				// Check if the entered code is correct
				if (strncmp(userCode, correctCode, maxCodeLength) == 0) {
					// Code is correct
					SSD1306_Clear();
					SSD1306_GotoXY(0, 0);
					SSD1306_Puts("Success", &Font_11x18, 1);

					HAL_UART_Transmit(&huart2, (uint8_t *)"Success\r\n", 10, 100);

					if (armed) {
						armed = 0;
					} else {
						armed = 1;
					}
				} else {
					// Code is incorrect
					SSD1306_Clear();
					SSD1306_GotoXY(0, 0);
					SSD1306_Puts("Failed", &Font_11x18, 1);
					HAL_UART_Transmit(&huart2, (uint8_t *)"Failed\r\n", 9, 100);
					//Activate buzzer
					buzzer= 1;

				}

				SSD1306_UpdateScreen();
				HAL_Delay(2000); // Display message for 2 seconds

				// Reset display and code length for next entry
				SSD1306_Clear();
				SSD1306_GotoXY(0, 0);

				//Check if armed
				if (armed) {
					SSD1306_Puts("Armed", &Font_11x18, 1);
					HAL_UART_Transmit(&huart2, (uint8_t *)"Armed\r\n", 8, 100);

				} else {
					SSD1306_Puts("Not Armed", &Font_11x18, 1);
					HAL_UART_Transmit(&huart2, (uint8_t *)"Not Armed\r\n", 12, 100);

				}
				SSD1306_GotoXY(0, 30); // Adjust Y position based on your font size
				HAL_UART_Transmit(&huart2, (uint8_t *)"Code:", 5, 100);
				SSD1306_Puts("Code:", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				memset(userCode, 0, sizeof(userCode)); // Clear the entered code
				codeLength = 0;

			} else {

				// Add received key to the entered code and update the display with an additional asterisk
				if (codeLength < maxCodeLength) { // Prevent buffer overflow
					userCode[codeLength] = receivedKey; // Store the received key
					SSD1306_Puts("*", &Font_11x18, 1);
					HAL_UART_Transmit(&huart2, (uint8_t *)"*", 1, 100);
					SSD1306_UpdateScreen();
					//Update code Length
					codeLength++;
				} else {
					printf("Error: Max Code Length Reached!\r\n");
				}
			}
		}
	}

	/* USER CODE END LCDTask */
}

void KeypadTask(void const *argument) {
	const TickType_t xDelay = 20 / portTICK_PERIOD_MS; // Debounce delay
	char keyEntered; // To hold the key to be sent

	for (;;) {
		keyEntered = Get_Key(); // Assume Get_Key() is debounced and returns '\0' if no key is pressed
		if (keyEntered != '\0') { // Check if a key is pressed
			// Send the key press to the display task
			if (xQueueSend(queue01Handle, &keyEntered, portMAX_DELAY) != pdPASS) {
				printf("Error: Data couldn't be sent from KeypadTask\r\n");
			}
		}
		vTaskDelay(xDelay); // Wait for the next cycle
	}
	/* USER CODE END LCDTask */
}

void PIRTask(void const *argument) {
	/* USER CODE BEGIN PIRTask */
	/* Infinite loop */
	GPIO_PinState pirState;
	/* Infinite loop */
	for (;;) {

		pirState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

		if (pirState == GPIO_PIN_SET && armed) {
			HAL_UART_Transmit(&huart2, (uint8_t *)"Movement\r\n", 13, 100);
			buzzer = 1;
		} else {
			if(armed){
				//pirState = GPIO_PIN_RESET;
				HAL_UART_Transmit(&huart2, (uint8_t *)"No Movement\r\n", 13, 100);
			}
			buzzer = 0; // No buzzer
		}
		osDelay(100); // Check every 100ms
	}
	/* USER CODE END PIRTask */
}

/* USER CODE BEGIN Header_KeypadTask */
/**
 * @brief Function implementing the task02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_KeypadTask */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
