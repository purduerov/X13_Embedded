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

#include "canFilterBankConfig.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_ADC_INIT_WAIT_MS 500

/*
 * The resistor voltage divisions for the CAN ID have a pull up on the ESC controller board
 * where this micro is. The pull down is on the backplane. ID 206 will be used if the board
 * is unconnected, resulting in ADC value of VCC (4095). The other voltage divisions are
 * designed to be at 0.3, 0.5, and 0.7 VCC. So the "bins" for them will be 0.2 to 0.4 are 0.3
 * or ID 201, 0.4 to 0.6 are 0.5 or ID 202, and 0.6 to 0.8 are 0.7 or ID 203.
 */

#define ADC_MAX_VALUE (4095)
#define CAN_ID_201_LOW_THRESHOLD (ADC_MAX_VALUE / 5)
#define CAN_ID_202_LOW_THRESHOLD (ADC_MAX_VALUE / 5 * 2)
#define CAN_ID_203_LOW_THRESHOLD (ADC_MAX_VALUE / 5 * 3)
#define CAN_ID_206_LOW_THRESHOLD (ADC_MAX_VALUE / 5 * 4)
#define CAN_ID_201_HIGH_THRESHOLD (CAN_ID_202_LOW_THRESHOLD - 1)
#define CAN_ID_202_HIGH_THRESHOLD (CAN_ID_203_LOW_THRESHOLD - 1)
#define CAN_ID_203_HIGH_THRESHOLD (CAN_ID_206_LOW_THRESHOLD - 1)
#define CAN_ID_206_HIGH_THRESHOLD ADC_MAX_VALUE

#define CAN_ID_201_FLASH_MS 1000
#define CAN_ID_202_FLASH_MS 500
#define CAN_ID_203_FLASH_MS 250
#define CAN_ID_206_FLASH_MS 2000
#define ERROR_FLASH_MS 4000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

uint8_t adcConfigured = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void EnablePWMOutput(TIM_HandleTypeDef *_htim);
int byte_to_pwm(int byte);  //  Imported from X12 ESC Code

void CAN_ConfigureFilterForThrusterOperation(uint32_t canId);

//  Interrupt Callback Functions
void CAN_FIFO0_RXMessagePendingCallback(CAN_HandleTypeDef *_hcan);
void ADC_ConversionCompleteCallback(ADC_HandleTypeDef *_hadc);
void TIM14_TimeElapsedCallback(TIM_HandleTypeDef *_htim);

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
  MX_ADC_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //  Used to Initialize the CAN TX Queue
  //InitializeQueueModule();
  //CreateQueue((void*)canTxData, sizeof(CanTxData), NUM_CAN_TX_QUEUE_MESSAGES, &canTxQueueHandle);

  //  Enable PWM Outputs to ESCs
  EnablePWMOutput(&htim3);

  //  Start Timer to begin sampling ESC_ID with ADC
  HAL_TIM_Base_Start_IT(&htim14);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  //  Configure ADC Interrupt Callback
  HAL_ADC_RegisterCallback(&hadc, HAL_ADC_CONVERSION_COMPLETE_CB_ID, ADC_ConversionCompleteCallback);

  //  Calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc);

  /* USER CODE END ADC_Init 2 */

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

  //  125kbps baud rate is configured
  //  HAL_CAN_Init() enters and stays in Initialization Mode
  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  //  Enable FIFO0 Message Pending Interrupt
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_FIFO0_RXMessagePendingCallback);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  //  ESC will update at a frequency of 100Hz
  //  TIM3 Count will reset every 10ms
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 160 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 75;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim14.Init.Prescaler = 8000 - 1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  //  Reconfigure Timer ARR to count up to NUM_ADC_INIT_WAIT_MS ms until ADC conversion start
  htim14.Init.Period = NUM_ADC_INIT_WAIT_MS - 1;
  htim14.Instance->ARR = htim14.Init.Period;

  //  Configure Timer Interrupt Callback
  HAL_TIM_RegisterCallback(&htim14, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM14_TimeElapsedCallback);

  /* USER CODE END TIM14_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void CAN_ConfigureFilterForThrusterOperation(uint32_t canId)
{
	CAN_FilterTypeDef thrusterOperationFilter;
	CAN_FilterBank canFilterBank;
	CAN_FilterIDMaskConfig canFilterId;
	CAN_FilterIDMaskConfig canFilterMask;

	//  Configure Filter Bank Parameters except ID and Mask
	thrusterOperationFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	thrusterOperationFilter.FilterBank = 0;
	thrusterOperationFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	thrusterOperationFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	thrusterOperationFilter.FilterActivation = CAN_FILTER_ENABLE;

	//  Configure only ID and Mask Filter Bank Parameters
	canFilterBank.filterMode = CANIDFilterMode_32BitMask;

	canFilterId.stdId = canId;
	canFilterId.extId = 0;
	canFilterId.ide = CAN_IDE_CLEAR;
	canFilterId.rtr = CAN_RTR_CLEAR;

	canFilterMask.stdId = 0x7FF;
	canFilterMask.extId = 0;
	canFilterMask.ide = CAN_IDE_CLEAR;
	canFilterMask.rtr = CAN_RTR_CLEAR;

	canFilterBank.id1 = &canFilterId;
	canFilterBank.mask1 = &canFilterMask;

	CAN_ConfigureFilterBank(&thrusterOperationFilter, &canFilterBank);
	HAL_CAN_ConfigFilter(&hcan, &thrusterOperationFilter);
}

//  Handles ONLY the Reception of Thruster Operation CAN Packets
void CAN_FIFO0_RXMessagePendingCallback(CAN_HandleTypeDef *_hcan)
{
	int data[4];

	//  Get Data Bytes received from RX FIFO
	data[0] = (int)((CAN_RDH0R_DATA7 & _hcan->Instance->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA7_Pos);
	data[1] = (int)((CAN_RDH0R_DATA6 & _hcan->Instance->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA6_Pos);
	data[2] = (int)((CAN_RDH0R_DATA5 & _hcan->Instance->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA5_Pos);
	data[3] = (int)((CAN_RDH0R_DATA4 & _hcan->Instance->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA4_Pos);

	//  Release Output Mailbox
	SET_BIT(_hcan->Instance->RF0R, CAN_RF0R_RFOM0);

	htim3.Instance->CR1 |= TIM_CR1_UDIS;  //  Disable UEV Generation

	htim3.Instance->CCR1 = byte_to_pwm(data[0]);
	htim3.Instance->CCR2 = byte_to_pwm(data[1]);
	htim3.Instance->CCR3 = byte_to_pwm(data[2]);
	htim3.Instance->CCR4 = byte_to_pwm(data[3]);

	htim3.Instance->CR1 &= ~TIM_CR1_UDIS;  //  Re-enable UEV Generation
}

void ADC_ConversionCompleteCallback(ADC_HandleTypeDef *_hadc)
{
	uint32_t adcValue = HAL_ADC_GetValue(_hadc);
	uint32_t canId;

	if (CAN_ID_201_LOW_THRESHOLD <= adcValue && adcValue <= CAN_ID_201_HIGH_THRESHOLD)
	{
		canId = 0x201;
		htim14.Init.Period = CAN_ID_201_FLASH_MS - 1;
	}
	else if (CAN_ID_202_LOW_THRESHOLD <= adcValue && adcValue <= CAN_ID_202_HIGH_THRESHOLD)
	{
		canId = 0x202;
		htim14.Init.Period = CAN_ID_202_FLASH_MS - 1;
	}
	else if (CAN_ID_203_LOW_THRESHOLD <= adcValue && adcValue <= CAN_ID_203_HIGH_THRESHOLD)
	{
		canId = 0x203;
		htim14.Init.Period = CAN_ID_203_FLASH_MS - 1;
	}
	else if (CAN_ID_206_LOW_THRESHOLD <= adcValue && adcValue <= CAN_ID_206_HIGH_THRESHOLD)
	{
		canId = 0x206;
		htim14.Init.Period = CAN_ID_206_FLASH_MS - 1;
	}
	else
	{
		htim14.Init.Period = ERROR_FLASH_MS - 1;
	}

	HAL_ADC_Stop_IT(_hadc);

	//  Configure CAN Filter for Thrusters and
	CAN_ConfigureFilterForThrusterOperation(canId);
	HAL_CAN_Start(&hcan);  //  Enters Normal Operating Mode

	//  Restart TIM14 to flash PA15 LED
	htim14.Instance->ARR = htim14.Init.Period;
	HAL_TIM_Base_Start_IT(&htim14);
}

void TIM14_TimeElapsedCallback(TIM_HandleTypeDef *_htim)
{
	if (!adcConfigured)
	{
		HAL_TIM_Base_Stop_IT(_htim);
		HAL_ADC_Start_IT(&hadc);
		adcConfigured = 1;
	}
	else
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	}
}

void EnablePWMOutput(TIM_HandleTypeDef *_htim)
{
	//  Set HAL Timer Channel Status
	TIM_CHANNEL_STATE_SET_ALL(_htim, HAL_TIM_CHANNEL_STATE_BUSY);

	//  Enable outputs for all 4 PWM Channels
	_htim->Instance->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

	//  Enable Timer Counter
	_htim->Instance->CR1 |= TIM_CR1_CEN;
}

//  Imported from X12 ESC Code
int byte_to_pwm(int byte)
{
	float exact;
	exact = byte * (40.0/255.0) + 55.0;
	return (exact + 0.5); //rounds up the integer by adding 0.5
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
