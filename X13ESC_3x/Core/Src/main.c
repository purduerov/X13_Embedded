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

#pragma GCC diagnostic warning "-Wunused-macros"
#pragma GCC diagnostic warning "-Wunused-parameter"
#pragma GCC diagnostic warning "-Wsign-compare"
#pragma GCC diagnostic warning "-Wconversion"
#pragma GCC diagnostic warning "-Wredundant-decls"
#pragma GCC diagnostic warning "-Wswitch-default"
#pragma GCC diagnostic warning "-Wswitch-enum"

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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

uint8_t adcConfigured = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // HAL_TIM_Base_Start_IT(&htim14);
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_10, GPIO_PIN_SET); //Set INLA, INLB, INLC to logic high (disable hi-z mode)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); //Set EN_1 (enable driver)

  HAL_CAN_Start(&hcan);

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

  //  125kbps baud rate is configured
  //  HAL_CAN_Init() enters and stays in Initialization Mode
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

  //  Enable FIFO0 Message Pending Interrupt
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  //  Configure CAN Interrupt Callbacks
  HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_FIFO0_RXMessagePendingCallback);

  CAN_ConfigureFilterForThrusterOperation(0x201);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8000 - 1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  HAL_TIM_MspPostInit(&htim14);

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
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|INLC_Pin|INLA_Pin
                          |INLB_Pin|EN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAL_GPIO_Port, CAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin INLC_Pin INLA_Pin
                           INLB_Pin EN_1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|INLC_Pin|INLA_Pin
                          |INLB_Pin|EN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SOA_Pin SOB_Pin SOC_Pin */
  GPIO_InitStruct.Pin = SOA_Pin|SOB_Pin|SOC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT_Pin */
  GPIO_InitStruct.Pin = FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAL_Pin */
  GPIO_InitStruct.Pin = CAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAL_GPIO_Port, &GPIO_InitStruct);

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
	uint8_t data[4];

	data[0] = (uint8_t)((CAN_RDL0R_DATA0 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos);
	data[1] = (uint8_t)((CAN_RDL0R_DATA1 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos);
	data[2] = (uint8_t)((CAN_RDL0R_DATA2 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA2_Pos);
	data[3] = (uint8_t)((CAN_RDL0R_DATA3 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA3_Pos);

	/*
	 * Use Data to set TIM->CCR registers for PWM generation
	 */

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	// HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

	//  Release Output Mailbox
	SET_BIT(_hcan->Instance->RF0R, CAN_RF0R_RFOM0);
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
