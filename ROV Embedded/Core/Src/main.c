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

#include "canFilterBankConfig.h"
#include "queue_api.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
	CAN_TxHeaderTypeDef canTxHeader;
	uint8_t data[8];
} CanTxData;

typedef struct
{
	//  Parameters to store
	uint8_t command;
	uint16_t data;
	uint8_t dev_address;
	uint8_t read_write;
} I2CTxData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SOLENOID_OPERATION_CAN_ID 0x204
#define SOLENOID_OPERATION_CAN_FIFO_NUMBER 0
#define SOLENOID_OPERATION_CAN_FILTER_BANK_NUMBER 0
#define POWER_BRICK_OPERATION_CAN_ID 0x205
#define POWER_BRICK_OPERATION_CAN_FIFO_NUMBER 1
#define POWER_BRICK_OPERATION_CAN_FILTER_BANK_NUMBER 1

#define NUM_CAN_TX_QUEUE_MESSAGES 5

#define NUM_I2C_TX_QUEUE_MESSAGES 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

int canTxQueueHandle;
CanTxData canTxData[NUM_CAN_TX_QUEUE_MESSAGES];
CanTxData canTxPrivateMessageToSend;

int i2cTxQueueHandle;
I2CTxData i2cTxData[NUM_I2C_TX_QUEUE_MESSAGES];
I2CTxData* i2cTxPrivateMessageToSend;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void LEDFlash(TIM_HandleTypeDef *htim);

void CAN_ConfigureFilterForCanRecvOperation(uint32_t canId, uint32_t fifoNumber, uint32_t filterBankNumber);

void SendCANMessage(CanTxData* canTxDataToSend);

//  Interrupt Callback Functions
void CAN_FIFO0_RXMessagePendingCallback(CAN_HandleTypeDef *_hcan);
void CAN_FIFO1_RXMessagePendingCallback(CAN_HandleTypeDef *_hcan);

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
  MX_TIM14_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  //  Initialize Queues for CAN TX and I2C TX
  InitializeQueueModule();
  CreateQueue((void*)canTxData, sizeof(CanTxData), NUM_CAN_TX_QUEUE_MESSAGES, &canTxQueueHandle);
  CreateQueue((void*)i2cTxData, sizeof(I2CTxData), NUM_I2C_TX_QUEUE_MESSAGES, &i2cTxQueueHandle);

  HAL_CAN_Start(&hcan);

  /*
  //  Create CAN Message
  canTxPrivateMessageToSend.canTxHeader.StdId = 0x212;
  canTxPrivateMessageToSend.canTxHeader.DLC = 4;
  canTxPrivateMessageToSend.canTxHeader.IDE = CAN_ID_STD;
  canTxPrivateMessageToSend.canTxHeader.RTR = CAN_RTR_DATA;
  canTxPrivateMessageToSend.data[0] = 0x01;
  canTxPrivateMessageToSend.data[1] = 0x04;
  canTxPrivateMessageToSend.data[2] = 0x09;
  canTxPrivateMessageToSend.data[3] = 0x10;

  SendCANMessage(&canTxPrivateMessageToSend);
  */

  //Use variable hi2c1 if a I2C_HandleTypeDef is needed
  uint8_t temp_request_code = 0x8D; // The request code for asking for temperature
  uint8_t temp_receive_1; // The char that will hold the temperature return data of slave 1
  uint8_t temp_receive_2; // The char that will hold the temperature return data of slave 2
  uint16_t slave_address_1 = 0x7F; // Address of the first slave
  uint16_t slave_address_2 = 0x23; // Address of the second slave
  uint8_t receiving_array[2] = {};
  uint8_t command_code = 0x01;
  uint8_t zeros = 0x0;
  uint8_t i2c_read_array[1] = {};
  I2CTxData* i2c_transfer_out_node = NULL;
  CanTxData* CAN_transfer_out_node = NULL;


  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_RegisterCallback(&htim14, HAL_TIM_PERIOD_ELAPSED_CB_ID, LEDFlash);


  //HAL_I2C_Master_Transmit(&hi2c1, slave_address_1 << 1, &temp_request_code, sizeof(uint8_t), 1000000);
  //HAL_I2C_Master_Receive(&hi2c1, slave_address_1 << 1, receiving_array, sizeof(uint8_t) * 3, 1000000);
  //HAL_I2C_Mem_Read(&hi2c1, slave_address_1 << 1, temp_request_code, 1,
		  //receiving_array, sizeof(uint16_t), 1000000);
  //HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
  //uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
    // Blocking Statements
  // if writing() HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
  //uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)

  /*
  hi2c1.Instance->CR2 |= I2C_CR2_PECBYTE;
  HAL_I2C_Mem_Write(&hi2c1, slave_address_1 << 1, command_code, 1, &zeros, 2, 1000);
  HAL_I2C_Mem_Read(&hi2c1, slave_address_1 << 1, command_code, 1,
  		  i2c_read_array, sizeof(uint8_t), 1000);
  */

  	  //Interrupt Statements
  //HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MASTER_TX_COMPLETE_CB_ID, );
  //HAL_I2C_Master_Transmit_IT(&hi2c1, slave_address_1 << 1, &temp_request_code, sizeof(uint8_t));
  // Transmits the hex code to the first slave to request temperature
  //HAL_I2C_Master_Receive_IT(&hi2c1, slave_address_1 << 1, &temp_receive_1, sizeof(uint8_t));
  // Receives the temperature response of the first slave

  //HAL_I2C_Master_Transmit_IT(&hi2c1, slave_address_2 << 1, &temp_request_code, sizeof(uint8_t));
  // Transmits the hex code to the second slave to request temperature
  //HAL_I2C_Master_Receive_IT(&hi2c1, slave_address_2 << 1, &temp_receive_2, sizeof(uint8_t));
  // Receives the temperature response of the second slave

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);
	  if (!isQueueEmpty(i2cTxQueueHandle))
	  	  {
		  	  RemoveFromQueue(i2cTxQueueHandle, i2c_transfer_out_node);
		  	  if (i2c_transfer_out_node->read_write) //It's a read
		  	  {
		  		HAL_I2C_Mem_Read(&hi2c1, i2c_transfer_out_node->dev_address << 1,
		  				i2c_transfer_out_node->command,
		  				1 , i2c_read_array, sizeof(uint8_t), 1000);
		  	  }
		  	  else //It's a write
		  	  {
		  		hi2c1.Instance->CR2 |= I2C_CR2_PECBYTE;
		  		HAL_I2C_Mem_Write(&hi2c1, slave_address_1 << 1, command_code, 1, &zeros, 2, 1000);
		  	  }
		  	  //SendCANMessage(transfer_out_node);
	  		  /*
	  		   * Remove Node from Queue
	  		   * RemoveNode(i2cTxQueueHandle, &i2cTxPrivateMessageToSend);
	  		   * Send I2C Message
	  		   */
	  	  }
	  HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
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
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
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

  //  Enable FIFO0 Message Pending Interrupt and register corresponding callback
  //  Used for activating/deactivating solenoids
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_FIFO0_RXMessagePendingCallback);
  CAN_ConfigureFilterForCanRecvOperation(
		  SOLENOID_OPERATION_CAN_ID,
		  SOLENOID_OPERATION_CAN_FIFO_NUMBER,
		  SOLENOID_OPERATION_CAN_FILTER_BANK_NUMBER);

  //  Enable FIFO1 Message Pending Interrupt and register corresponding callback
  //  Used for receiving Power Brick Communication Queries
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, CAN_FIFO1_RXMessagePendingCallback);
  CAN_ConfigureFilterForCanRecvOperation(
		  POWER_BRICK_OPERATION_CAN_ID,
		  POWER_BRICK_OPERATION_CAN_FIFO_NUMBER,
		  POWER_BRICK_OPERATION_CAN_FILTER_BANK_NUMBER);

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  __HAL_I2C_DISABLE(&hi2c1);

  hi2c1.Instance->CR1 |= I2C_CR1_PECEN;

  __HAL_I2C_ENABLE(&hi2c1);

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
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */
  htim14.Init.Prescaler = 7999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 125;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void LEDFlash(TIM_HandleTypeDef *htim) {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
}

void CAN_ConfigureFilterForCanRecvOperation(uint32_t canId, uint32_t fifoNumber, uint32_t filterBankNumber)
{
	CAN_FilterTypeDef thrusterOperationFilter;
	CAN_FilterBank canFilterBank;
	CAN_FilterIDMaskConfig canFilterId;
	CAN_FilterIDMaskConfig canFilterMask;

	//  Configure Filter Bank Parameters except ID and Mask
	thrusterOperationFilter.FilterFIFOAssignment = fifoNumber;
	thrusterOperationFilter.FilterBank = filterBankNumber;
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

//  Handles ONLY the reception of solenoid Operation Packets
void CAN_FIFO0_RXMessagePendingCallback(CAN_HandleTypeDef *_hcan)
{
	uint8_t data[4];

	data[0] = (uint8_t)((CAN_RDL0R_DATA0 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos);
	data[1] = (uint8_t)((CAN_RDL0R_DATA1 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos);
	data[2] = (uint8_t)((CAN_RDL0R_DATA2 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA2_Pos);
	data[3] = (uint8_t)((CAN_RDL0R_DATA3 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA3_Pos);

	/*
	 * Solenoid Control Code
	 */

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	// HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

	//  Release Output Mailbox
	SET_BIT(_hcan->Instance->RF0R, CAN_RF0R_RFOM0);
}

//  Handles ONLY to reception of power brick communication packets
void CAN_FIFO1_RXMessagePendingCallback(CAN_HandleTypeDef *_hcan)
{
	uint8_t data[4];
	uint32_t numBytesReceived;
	I2CTxData i2cTxData;

	numBytesReceived = (CAN_RDT1R_DLC & _hcan->Instance->sFIFOMailBox[1].RDTR) >> CAN_RDT1R_DLC_Pos;

	data[0] = (uint8_t)((CAN_RDL1R_DATA0 & _hcan->Instance->sFIFOMailBox[1].RDLR) >> CAN_RDL1R_DATA0_Pos);
	data[1] = (uint8_t)((CAN_RDL1R_DATA1 & _hcan->Instance->sFIFOMailBox[1].RDLR) >> CAN_RDL1R_DATA1_Pos);
	data[2] = (uint8_t)((CAN_RDL1R_DATA2 & _hcan->Instance->sFIFOMailBox[1].RDLR) >> CAN_RDL1R_DATA2_Pos);
	data[3] = (uint8_t)((CAN_RDL1R_DATA3 & _hcan->Instance->sFIFOMailBox[1].RDLR) >> CAN_RDL1R_DATA3_Pos);

	//  Release Output Mailbox
	SET_BIT(_hcan->Instance->RF1R, CAN_RF1R_RFOM1);

	/*
	 * I2C Communication with Bricks
	 * i2cTxData = ?
	 * Add Message to I2C Transmit Queue
	 * AddToQueue(i2cTxQueueHandle, &i2cTxData);
	 */

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	// HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);


}

void SendCANMessage(CanTxData* canTxDataToSend)
{
	uint32_t txMailboxNumber;

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0 && isQueueEmpty(canTxQueueHandle))
	{
		HAL_CAN_AddTxMessage(&hcan, &(canTxDataToSend->canTxHeader), canTxDataToSend->data, &txMailboxNumber);
	}
	else
	{
		if (AddToQueue(canTxQueueHandle, canTxDataToSend) != QUEUE_SUCCESS)
		{
			; //  Queue is Full
		}
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
