/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "solenoid.h"
#include "common.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOLENOID_STATE_INDEX 7U

#define SOLENOID_BOARD_CAN_STANDARD_ID 0x204  //  11 bits
#define SOLENOID_BOARD_CAN_EXTENDED_ID 0x00000  //  18 bits
#define SOLENOID_BOARD_CAN_IDE CAN_ID_STD
#define SOLENOID_BOARD_CAN_RTR CAN_RTR_DATA

#define SOLENOID_BOARD_CAN_FIFO_OVERFLOW_STDID 0x240
#define SOLENOID_BOARD_CAN_FIFO_OVERFLOW_EXTID 0x00000
#define SOLENOID_BOARD_CAN_FIFO_OVERFLOW_IDE CAN_ID_STD
#define SOLENOID_BOARD_CAN_FIFO_OVERFLOW_RTR CAN_RTR_REMOTE
#define SOLENOID_BOARD_CAN_FIFO_OVERFLOW_DLC 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef canTxOverflowMessage;
uint8_t CAN_EMPTY_DATA[8] = {};

//  Establish Pins used for solenoids
uint8_ft solenoidIndices[NUM_SOLENOIDS] = {0, 1, 2, 3, 4, 5};
GPIO_TypeDef *gpioPorts[NUM_SOLENOIDS] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t gpioPins[NUM_SOLENOIDS] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6};

//  Establish Pin for LEDs
GPIO_TypeDef *ledGpioPort = GPIOA;
uint16_t ledGpioPin = GPIO_PIN_15;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef CAN_ConfigureSolenoidBoardReceiveFilter(CAN_HandleTypeDef* hcan);

static void CAN_ReceiveMessageCallback(CAN_HandleTypeDef *hcan);
#if 0
static void CAN_ConfigureCANTxOverflowMessage(void);
static void CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
static void LedInitFlash(void);
#endif
static void LedInit(void);
static SolenoidErrorCode SolenoidInit(void);
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  LedInit();
  SolenoidInit();

  HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  if (CAN_ConfigureSolenoidBoardReceiveFilter(&hcan) != HAL_OK)
  	{
  		Error_Handler();
  	}

  	//  Enable CAN RX FIFO #0 Pending Interrupt
  	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  	{
  		Error_Handler();
  	}
  	HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_ReceiveMessageCallback);

  	//  Enable CAN RX FIFO #0 Overflow Interrupt
  	/*
  	CAN_ConfigureCANTxOverflowMessage();
  	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK)
  	{
  		Error_Handler();
  	}
  	HAL_CAN_RegisterCallback(&hcan, HAL_CAN_ERROR_CB_ID, CAN_ErrorCallback);
  	*/
  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef CAN_ConfigureSolenoidBoardReceiveFilter(CAN_HandleTypeDef* hcan)
{
	//  Configure CAN RX Filter
	CAN_FilterTypeDef solenoidBoardFilter;
	//  Filter Bank #0 assigned to FIFO #0 (two total RX FIFOs)
	solenoidBoardFilter.FilterBank = 0;
	solenoidBoardFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

	//  Configure Filter Bank #0 as 32-bit Scale in Identifier Mask Mode
	solenoidBoardFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	solenoidBoardFilter.FilterScale = CAN_FILTERSCALE_32BIT;

	//  Set CAN Filter ID High
	//  [11 bit STDID][18 bit Extended ID][1 bit IDE][1 bit RTR][1'b0]
	//  FilterIdHigh is 16 MSbits
	//  FilterIdLow is 16 LSbits
	solenoidBoardFilter.FilterIdHigh = (SOLENOID_BOARD_CAN_STANDARD_ID << 5);
	solenoidBoardFilter.FilterIdHigh |= ((SOLENOID_BOARD_CAN_EXTENDED_ID & (0x1F << 13)) >> 13);
	solenoidBoardFilter.FilterIdLow = (SOLENOID_BOARD_CAN_EXTENDED_ID & 0x1FFF) << 3;
	solenoidBoardFilter.FilterIdLow |= SOLENOID_BOARD_CAN_IDE;
	solenoidBoardFilter.FilterIdLow |= SOLENOID_BOARD_CAN_RTR;

	//  Set CAN Filter Mask
	//  Bits directly correspond to bits in Filter
	//  Each bit HIGH in the Mask has to Match for Message
	//  of specific CAN ID to be filter through into CAN RX FIFO
	solenoidBoardFilter.FilterMaskIdHigh = (0x7FF << 5);  //  Only Match STDID
	solenoidBoardFilter.FilterMaskIdLow = (0x03 << 1);  //  Only Match IDE and RTR bits
	#warning "Magic numbers above";

	//  Enable Solenoid Board Filter
	solenoidBoardFilter.FilterActivation = CAN_FILTER_ENABLE;

	//  Configure Filter
	return HAL_CAN_ConfigFilter(hcan, &solenoidBoardFilter);
}

#if 0
static void CAN_ConfigureCANTxOverflowMessage(void)
{
	//  Configure CAN RX FIFO #0 Overflow CAN TX Message ahead of time
	canTxOverflowMessage.StdId = SOLENOID_BOARD_CAN_FIFO_OVERFLOW_STDID;
	canTxOverflowMessage.ExtId = SOLENOID_BOARD_CAN_FIFO_OVERFLOW_EXTID;
	canTxOverflowMessage.IDE = SOLENOID_BOARD_CAN_FIFO_OVERFLOW_IDE;
	canTxOverflowMessage.RTR = SOLENOID_BOARD_CAN_FIFO_OVERFLOW_RTR;
	canTxOverflowMessage.DLC = SOLENOID_BOARD_CAN_FIFO_OVERFLOW_DLC;
	canTxOverflowMessage.TransmitGlobalTime = DISABLE;
}
#endif

static void CAN_ReceiveMessageCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef canPacketHeader;
	uint8_t canPacketData[8];

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canPacketHeader, canPacketData);

	uint8_t solenoidData = canPacketData[SOLENOID_STATE_INDEX];

	for (uint8_ft solenoidIndex = 0; solenoidIndex < NUM_SOLENOIDS; ++solenoidIndex)
	{
		setSolenoid(solenoidIndex, solenoidData & (1 << solenoidIndex));
	}

	toggleLed();
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
}

#if 0
//  Only configured for CAN RX FIFO #0 Overflow Errors
static void CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t txMailboxNumber;

	//  CAN RX FIFO #0 Overflow Error
	if ((hcan->ErrorCode & HAL_CAN_ERROR_RX_FOV0) != 0)
	{
		//  Send Overflow Error Message on CAN Bus
		HAL_CAN_AddTxMessage(hcan, &canTxOverflowMessage, CAN_EMPTY_DATA, &txMailboxNumber);
	}
}
#endif



static void LedInit(void)
{
	GPIO_InitTypeDef gpioInit;
	gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
	gpioInit.Pull = GPIO_NOPULL;
	gpioInit.Speed = GPIO_SPEED_FREQ_LOW;

	configureLed(ledGpioPort, ledGpioPin, &gpioInit);
}

#if 0
static void LedInitFlash(void)
{
	initializeLedTimer(TIM3);
	initializeLedFlashFrequency(TIM_FREQ_1HZ);
	flashLed();
}
#endif

static SolenoidErrorCode SolenoidInit(void)
{
	SolenoidErrorCode errorCode;

	//  Set configuration settings for solenoid GPIO Pins
	GPIO_InitTypeDef gpioInit;
	gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
	gpioInit.Pull = GPIO_NOPULL;
	gpioInit.Speed = GPIO_SPEED_FREQ_LOW;

	//  Verify and configure number of solenoids
	compile_assert(verifyNumberOfSolenoids(NUM_SOLENOIDS) == SOLENOID_SUCCESS);
	configureNumberOfSolenoids(NUM_SOLENOIDS);

	//  Verify solenoids indices
	for (uint8_ft i = 0; i < NUM_SOLENOIDS; ++i) {
		errorCode = verifySolenoidIndex(solenoidIndices[i]);
		if (errorCode != SOLENOID_SUCCESS) {
			return errorCode;
		}
	}

	addSolenoids(NUM_SOLENOIDS, solenoidIndices, gpioPorts, gpioPins);
	configureSolenoids(&gpioInit);
	disableSolenoids();

	return SOLENOID_SUCCESS;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
