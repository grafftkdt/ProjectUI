/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
int16_t inputchar = -1;
char RxDataBuffer[32] = { 0 };
uint8_t nstation[10] = { 0 };
uint8_t test = 0;
uint8_t error = 0;
uint8_t connect = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int16_t UARTRecieveIT();
void Communication(int16_t dataIn);
void ACK1();
void ACK2();
void request(uint8_t mode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);
	   inputchar = UARTRecieveIT();
		//if input char == -1 ==> No New data
		if (inputchar != -1)
		{
			Communication(inputchar);
		}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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

}

/* USER CODE BEGIN 4 */
int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void Communication(int16_t dataIn)
{
	static uint8_t mode = 0;
	static uint8_t n = 0;
	static uint16_t parameter = 0;
	static uint8_t station[10] = {99,99,99,99,99,99,99,99,99,99};
	static uint8_t checksum = 0;
	static uint8_t check = 0;
	static uint8_t len = 0;
	test = 0;
	error = 0;

	static enum _StateMachine
	{
		State_StartMode,
		State_CollectData_2,
		State_CollectData_3,
		State_Len,
		State_CheckSum

	} STATE = State_StartMode;

	//State Machine
	  switch (STATE)
	  {
	  	  case State_StartMode :
	  		  switch (dataIn)
	  		  {
	  		  	  case 0b10010010 :
	  		  		  mode = 2;		//connect MCU #1
	  		  		  STATE = State_CheckSum;
	  		  		  break;

	  		  	  case 0b10010011 :
	  		  		  mode = 3;		//disconnect MCU #1
	  		  		  STATE = State_CheckSum;
	  		  		  break;

	  		  	  case 0b10011000 :
	  		  		  mode = 8;		//go to station/goal position #1
	  		  		  STATE = State_CheckSum;
	  		  		  break;

	  		  	  case 0b10011001 :
	  		  		  mode = 9;		//request current station #1
	  		  		  STATE = State_CheckSum;
	  		  		  break;

	  		  	  case 0b10011010 :
	  		  		  mode = 10;	//request angular position #1
	  		  		  STATE = State_CheckSum;
	  		  		  break;

	  		  	  case 0b10011011 :
	  		  		  mode = 11;	//request max angular velocity #1
	  		  		  STATE = State_CheckSum;
	  		  		  break;

	  		  	  case 0b10011100 :
	  		  		  mode = 12;	//enable gripper #1
	  		  		  STATE = State_CheckSum;
	  		  		  break;

	  		  	  case 0b10011101 :
	  		  		  mode = 13;	//disable gripper #1
	  		  		  STATE = State_CheckSum;
	  		  		  break;

	  		  	  case 0b10011110 :
	  		  		  mode = 14;	//set home #1
	  		  		  STATE = State_CheckSum;
	  		  		  break;

	  		  	  case 0b10010001 :
	  		  		  mode = 1;		//test command #2
	  		  		  n = 1;
	  		  		  STATE = State_CollectData_2;
	  		  		  break;

	  		  	  case 0b10010100 :
	  		  		  mode = 4;		//set angular velocity #2
	  		  		  n = 1;
	  		  		  STATE = State_CollectData_2;
	  		  		  break;

	  		  	  case 0b10010101 :
	  		  		  mode = 5;		//set angular position #2
	  		  		  n = 1;
	  		  		  STATE = State_CollectData_2;
	  		  		  break;

	  		  	  case 0b10010110 :
	  		  		  mode = 6;		//set goal 1 station #2
	  		  		  n = 1;
	  		  		  STATE = State_CollectData_2;
	  		  		  break;

	  		  	  case 0b10010111 :
	  		  		  mode = 7;		//set goal n station #3
	  		  		  STATE = State_Len;
	  		  		  break;
	  		  }
	  		  break;

	  	  case State_Len :
	  		len = dataIn;
	  		n = dataIn;
	  		STATE = State_CollectData_3;
	  		break;

	  	  case State_CollectData_2 :
	  		check += dataIn;
	  		parameter |= (dataIn & 0xFF) << (8*n);
	  		if (n == 0)
	  		{
	  			STATE = State_CheckSum;
	  			break;
	  		}
	  		n -= 1;
	  		break;

	  	  case State_CollectData_3 :
	  		n -= 1;
	  		check += dataIn;
	  		if (n == 0)
	  		{
	  			STATE = State_CheckSum;
	  			break;
	  		}
	  		station[n] += dataIn;
	  		break;

	  	  case State_CheckSum :
	  		  switch (mode)
	  		  {
	  		  	  case 1 :	//test command #2
	  		  		  checksum = ~(0b10010001 + check);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
	  		  			  //blabla
	  		  			  test = 1;
	  		  		  }
	  		  		  break;

	  		  	  case 2 : //connect MCU #1
	  		  		  checksum = ~(0b10010010);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else
	  		  		  {
	  		  			  connect = 1;
	  		  			  test = 2;
	  		  		  }
	  		  		  break;

	  		  	  case 3 : //disconnect MCU #1
	  		  		  checksum = ~(0b10010011);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else
	  		  		  {
	  		  			  connect = 0;
	  		  			  test = 3;
	  		  		  }

	  		  		  break;

	  		  	  case 4 : //set angular velocity #2
	  		  		  checksum = ~(0b10010100 + check);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
		  		  		  //velocity = parameter;
	  		  			  test = 4;
	  		  		  }
	  		  		  break;

	  		  	  case 5 : //set angular position #2
	  		  		  checksum = ~(0b10010101 + check);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
		  		  		  //position = parameter;
	  		  			  test = 5;
	  		  		  }
	  		  		  break;

	  		  	  case 6 : //set goal 1 station #2
	  		  		  checksum = ~(0b10010110 + check);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
		  		  		  //station = parameter;
	  		  			  test = 6;
	  		  		  }
	  		  		  break;

	  		  	  case 7 : //set goal n station #3
	  		  		  checksum = ~(0b10010111 + len + check);
	  		  		  for (uint8_t i = len - 1 ; i >= 0 ; i--)
	  		  		  {
	  		  			  nstation[len - i - 1] = station[i];
	  		  		  }
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
	  		  			  //blabla
	  		  			  test = 7;
	  		  		  }
	  		  		  break;

	  		  	  case 8 : //go to station/goal position #1
	  		  		  checksum = ~(0b10011000);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
	  		  			  //blabla
	  		  			  test = 8;
	  		  		  }
	  		  		  break;

	  		  	  case 9 : //request current station #1
	  		  		  checksum = ~(0b10011001);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
	  		  			  request(9);
	  		  			  test = 9;
	  		  		  }
	  		  		  break;

	  		  	  case 10 : //request angular position #1
	  		  		  checksum = ~(0b10011010);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
	  		  			  request(10);
	  		  			  test = 10;
	  		  		  }
	  		  		  break;

	  		  	  case 11 : //request max angular velocity #1
	  		  		  checksum = ~(0b10011011);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
	  		  			  request(11);
	  		  			  test = 11;
	  		  		  }
	  		  		  break;

	  		  	  case 12 : //enable gripper #1
	  		  		  checksum = ~(0b10011100);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
	  		  			  //blabla
	  		  			  test = 12;
	  		  		  }
	  		  		  break;

	  		  	  case 13 : //disable gripper #1
	  		  		  checksum = ~(0b10011101);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
	  		  			  //blabla
	  		  			  test = 13;
	  		  		  }
	  		  		  break;

	  		  	  case 14 : //set home #1
	  		  		  checksum = ~(0b10011110);
	  		  		  ACK1();
	  		  		  STATE = State_StartMode;
	  		  		  if (dataIn != checksum)
	  		  		  {
	  		  			  error = 1;
	  		  		  }
	  		  		  else if (connect == 1)
	  		  		  {
	  		  			  //sethome = 1;
	  		  			  test = 14;
	  		  		  }
	  		  		  break;
	  		  }
	  }
}

void ACK1()
{
	static uint8_t ack1[2] = {0x58,0b01110101};
	HAL_UART_Transmit_IT(&huart2, ack1, 2);
}

void ACK2()
{
	static uint8_t ack2[2] = {70,110};
	HAL_UART_Transmit_IT(&huart2, ack2, 2);
}

void request(uint8_t mode)
{
	if (mode == 9)		//request current station >> #2
	{
		uint8_t requested[] = {0b10011001, 0b00, 0b00, 0b00};
		//requested[2] = velocity;
		requested[3] = ~(0b10011001 + requested[2]);
		HAL_UART_Transmit_IT(&huart2, requested, 4);
	}

	if (mode == 10)		//request angular position >> #2
	{
		uint8_t requested[] = {0b10011010, 0b00, 0b00, 0b00};
		//requested[1] = velocity >> 8;
		//requested[2] = velocity;
		requested[3] = ~(0b10011010 + requested[1] + requested[2]);
		HAL_UART_Transmit_IT(&huart2, requested, 4);
	}

	if (mode == 11)		//request max angular velocity >> #2
	{
		uint8_t requested[] = {0b10011011, 0b00, 0b00, 0b00};
		//requested[2] = velocity;
		requested[3] = ~(0b10011011 + requested[2]);
		HAL_UART_Transmit_IT(&huart2, requested, 4);
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
