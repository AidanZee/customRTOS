/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//keeping task info nice and organized for use with our rtos scheduler
typedef struct task
{
	int state;
	unsigned long period;
	unsigned long elapsedTime;
	int (*Function)(int);// (float);
} task;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//defining address of ina219
#define inaAddr (0x40 << 1)
#define inaReg (0x04)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
//setting variables for rtos stuff
const unsigned int numTasks = 3;
const unsigned long period = 1;
//creating easy to read state variables
enum yellStates{start, talk};
enum uartStates{first, hello, ask1, ask2, ready, done};

task tasks[3];
//enum BL_states {BL0, BL1};
// global variables for current() task, didnt know how else to do this
float sample1 = 0;
float sample2 = 0;
uint8_t txBuf1[12];
uint8_t txBuf2[12];
uint8_t rxBuf1[12] = {0,1,1};
uint8_t rxBuf2[12];

HAL_StatusTypeDef ret;
int16_t val1;
int16_t val2;

uint8_t msg[16];
float amp1;
float amp2;
float thresh = .200;
uint8_t cal[3] = { 0x05, 0x10, 0x00 }; // reg 0x05, value 0x1000

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

//void current(int state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Task for reading sensor data
enum currentStates{init, cali1, cali2, transmit1, transmit2, receive1, receive2};

int current(int state){
	switch(state){
		case init:
			state = cali1;
			break;
		case cali1:
			state = cali2;
			break;
		case cali2:
			state = transmit1;
			break;
		case transmit1:
			state = transmit2;
			break;
		case transmit2:
			state = receive1;
			break;
		case receive1:
			state = receive2;
			break;
		case receive2:
			state = receive1;
			break;
	}
	switch(state){
	//this state cablibrates the ina219 for output scaling
		case cali1:
			HAL_I2C_Master_Transmit(&hi2c1, (0x40u<<1), cal, 3, 100);
			break;
		case cali2:
			HAL_I2C_Master_Transmit(&hi2c1, (0x45u<<1), cal, 3, 100);
			break;
		case transmit1:
			txBuf1[0] = inaReg; //set first byte of buffer to register to read, prob for the transmit function.
			HAL_I2C_Master_Transmit(&hi2c1, inaAddr, txBuf1, 1, HAL_MAX_DELAY);
			break;
		case transmit2:
			txBuf2[0] = inaReg; //set first byte of buffer to register to read, prob for the transmit function.
			HAL_I2C_Master_Transmit(&hi2c1, 0x45u<<1, txBuf2, 1, HAL_MAX_DELAY);
			break;
		case receive1:
			HAL_I2C_Master_Receive(&hi2c1, inaAddr, txBuf1, 2, 100);// read function, device addr, where to store data, number of bytes to receive
			val1 = ((int16_t)txBuf1[0] << 8) | (txBuf1[1]); 								//combine the two bytes that we are recieving
			amp1 = val1 * 0.00010138f;
			sample1 = amp1;//amp1;
			break;
		case receive2:
			HAL_I2C_Master_Receive(&hi2c1, 0x45u<<1, txBuf2, 2, 100);// read function, device addr, where to store data, number of bytes to receive
			val2 = ((int16_t)txBuf2[0] << 8) | (txBuf2[1]); 								//combine the two bytes that we are recieving
			amp2 = val2 * 0.00010138f;
			sample2 = amp2;//amp2;
			break;
	}
	return state;
}
//task to print through uart to the esp
int yell(int state){
	switch(state){
		case start:
			state = talk;
			break;
		case talk:
			state = talk;
			break;
	}
	switch(state){
		case talk:
			//variables to create message then send
		  uint8_t msgf1[32];
		  uint8_t msgf2[32];

		  //float amps = sample;
		  snprintf(msgf1, sizeof(msgf1), "Sample1: %.1f mA\r\n", sample1*1000.0f);//amps in whole, amps in mA, this stores message we want to send
		  HAL_UART_Transmit(&huart1, (uint8_t *)msgf1, strlen(msgf1), HAL_MAX_DELAY);//HAL_MAX_DELAY); this sends the message we stored
		  snprintf(msgf2, sizeof(msgf2), "Sample2: %.1f mA\r\n", sample2*1000.0f);//*1000.0f);//amps in whole, amps in mA
		  HAL_UART_Transmit(&huart1, (uint8_t *)msgf2, strlen(msgf2), HAL_MAX_DELAY);//HAL_MAX_DELAY);
		  break;
	}
	return state;
}
//this task toggles the pin that turns the relay on and onff
enum thresholdStates{start2, check};
int threshold(int state){
	switch(state){
		case start2:
			state = check;
			break;
		case check: // constantly loop to check if we are over threshold
			state = check;
			break;
	}
	switch(state){
		case check://threshold checking and then gpio toggle
			if ((sample1 > thresh) || (sample2 > thresh)){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
			}else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
			}

			break;
	}
	return state;
}

//unfinished


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//same stuff as in class
	tasks[0].state = init;
	tasks[0].period = 2; //every tick is half a milisecond through clock division, so 2 * .5ms = 1000Hz
	tasks[0].elapsedTime = tasks[0].period;
	tasks[0].Function = &current;
	tasks[1].state = start;
	tasks[1].period = 1000; //.5 seconds
	tasks[1].elapsedTime = tasks[1].period;
	tasks[1].Function = &yell;
	tasks[1].state = start2;
	tasks[2].period = 1000; //.5 seconds
	tasks[2].elapsedTime = tasks[2].period;
	tasks[2].Function = &threshold;
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim14);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 84-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 500-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//CALL BACK: TIMER HAS RESET
// same stuff as in class
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//all timers go here so check for certain timer
	if(htim == &htim14){
		//run basic rtos code
		 for (int i = 0; i < numTasks; i++){
			 if (tasks[i].elapsedTime >= tasks[i].period)
			 {
				 tasks[i].state = tasks[i].Function(tasks[i].state);
				 tasks[i].elapsedTime = 0;

			 }
			 tasks[i].elapsedTime += period;
		 }
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//if(huart == &huart1){
	    HAL_UART_Receive_DMA(&huart1, rxBuf1, 12);
	//}
}
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
#ifdef USE_FULL_ASSERT
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
