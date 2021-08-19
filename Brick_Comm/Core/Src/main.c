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
#include "queue.h"

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
    uint8_t num_data;
    uint16_t command;
    uint8_t data[8];
    uint16_t dev_address;
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

#define POWER_BRICK_REPLY_CAN_ID 0x305

#define NUM_CAN_TX_QUEUE_MESSAGES 5
#define NUM_I2C_TX_QUEUE_MESSAGES 10

#define I2C_BRICK_0_DEVICE_ADDRESS 0x7F
#define I2C_BRICK_1_DEVICE_ADDRESS 0x23
#define I2C_TRANSACTION_TIMEOUT_MS 10000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

queue_handle_t canTxQueueHandle;
CanTxData canTxData[NUM_CAN_TX_QUEUE_MESSAGES];
CanTxData canTxPrivateMessageToSend;

queue_handle_t i2cTxQueueHandle;
I2CTxData i2cTxData[NUM_I2C_TX_QUEUE_MESSAGES];
I2CTxData *i2cTxPrivateMessageToSend;

int i2cTransactionReadyToStart = 1;
I2CTxData i2c_transfer_out_node;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void CAN_ConfigureFilterForCanRecvOperation(uint32_t canId, uint32_t fifoNumber, uint32_t filterBankNumber);
void SendCANMessage(CanTxData* canTxDataToSend);

//  Interrupt Callback Functions
void CAN_FIFO0_RXMessagePendingCallback(CAN_HandleTypeDef *_hcan);
void CAN_FIFO1_RXMessagePendingCallback(CAN_HandleTypeDef *_hcan);
void CAN_TxRequestCompleteCallback(CAN_HandleTypeDef *_hcan);
void I2C_TransactionCompleteCallback(I2C_HandleTypeDef *hi2c);

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  //  Initialize Queues for CAN TX and I2C TX
  InitializeQueueModule();
  CreateQueue((void*)canTxData, sizeof(CanTxData), NUM_CAN_TX_QUEUE_MESSAGES, &canTxQueueHandle);
  CreateQueue((void*)i2cTxData, sizeof(I2CTxData), NUM_I2C_TX_QUEUE_MESSAGES, &i2cTxQueueHandle);

  HAL_CAN_Start(&hcan);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {
        if (!isQueueEmpty(i2cTxQueueHandle) && i2cTransactionReadyToStart)
        {
            i2cTransactionReadyToStart = 0;

            HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);
            RemoveFromQueue(i2cTxQueueHandle, &i2c_transfer_out_node);
            HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);

            if (i2c_transfer_out_node.read_write) // It's a read
            {
                HAL_I2C_Mem_Read_IT(&hi2c1, i2c_transfer_out_node.dev_address << 1, i2c_transfer_out_node.command,
                        1, i2c_transfer_out_node.data, i2c_transfer_out_node.num_data);
            }
            else // It's a write
            {
                hi2c1.Instance->CR2 |= I2C_CR2_PECBYTE;
                HAL_I2C_Mem_Write_IT(&hi2c1, i2c_transfer_out_node.dev_address << 1, i2c_transfer_out_node.command,
                        1, i2c_transfer_out_node.data, i2c_transfer_out_node.num_data + 1);
            }
      }

  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
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

    //  Enable TX Request Complete Interrupt
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
    HAL_CAN_RegisterCallback(&hcan, HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, CAN_TxRequestCompleteCallback);
    HAL_CAN_RegisterCallback(&hcan, HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID, CAN_TxRequestCompleteCallback);
    HAL_CAN_RegisterCallback(&hcan, HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID, CAN_TxRequestCompleteCallback);

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

  HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MEM_TX_COMPLETE_CB_ID, I2C_TransactionCompleteCallback);
  HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MEM_RX_COMPLETE_CB_ID, I2C_TransactionCompleteCallback);

  __HAL_I2C_DISABLE(&hi2c1);
  hi2c1.Instance->CR1 |= I2C_CR1_PECEN;
  __HAL_I2C_ENABLE(&hi2c1);

  /* USER CODE END I2C1_Init 2 */

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

  /*Configure GPIO pins : PB3 PB4 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
	uint32_t data32;
    uint8_t *data = (uint8_t *)&data32;

    data32 = CAN_RDL0R_DATA0 & _hcan->Instance->sFIFOMailBox[0].RDLR;
    /*
     * NOTE: NEED TO CHECK IF TRICK TO PERFORM FEWER REGISTER READS SWITCHES DATA ORDER DUE TO ENDIANNESS.
    data[0] = (uint8_t)((CAN_RDL0R_DATA0 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos);
    data[1] = (uint8_t)((CAN_RDL0R_DATA1 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos);
    data[2] = (uint8_t)((CAN_RDL0R_DATA2 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA2_Pos);
    data[3] = (uint8_t)((CAN_RDL0R_DATA3 & _hcan->Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA3_Pos);
    */

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

    i2cTxData.data[0] = data[2];
    i2cTxData.data[1] = data[3];
    i2cTxData.command = (uint16_t)data[1];
    i2cTxData.dev_address = ((data[0] & 0x01) != 0x00) ? I2C_BRICK_1_DEVICE_ADDRESS : I2C_BRICK_0_DEVICE_ADDRESS;
    i2cTxData.read_write = (data[0] & 0x02) >> 1;
    i2cTxData.num_data = numBytesReceived - 2;
    AddToQueue(i2cTxQueueHandle, &i2cTxData);

    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
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

void CAN_TxRequestCompleteCallback(CAN_HandleTypeDef *_hcan)
{
    uint32_t txMailboxNumber;
    CanTxData canTxDataToSend;

    if (!isQueueEmpty(canTxQueueHandle) && HAL_CAN_GetTxMailboxesFreeLevel(_hcan) > 0)
    {
        RemoveFromQueue(canTxQueueHandle, (void*)&canTxDataToSend);
        HAL_CAN_AddTxMessage(_hcan, &(canTxDataToSend.canTxHeader), canTxDataToSend.data, &txMailboxNumber);
    }
}

void I2C_TransactionCompleteCallback(I2C_HandleTypeDef *hi2c)
{
    CanTxData canTxMessage;

    canTxMessage.canTxHeader.StdId = POWER_BRICK_REPLY_CAN_ID;
    canTxMessage.canTxHeader.DLC = i2c_transfer_out_node.num_data + 2;
    canTxMessage.canTxHeader.IDE = CAN_ID_STD;
    canTxMessage.canTxHeader.RTR = CAN_RTR_DATA;

    canTxMessage.data[0] = i2c_transfer_out_node.num_data << 2;
    canTxMessage.data[0] |= i2c_transfer_out_node.read_write << 1;
    canTxMessage.data[0] |= (i2c_transfer_out_node.dev_address == I2C_BRICK_0_DEVICE_ADDRESS) ? 0 : 1;

    canTxMessage.data[1] = i2c_transfer_out_node.command;
    for (int i = 0; i < i2c_transfer_out_node.num_data; i++)
    {
        canTxMessage.data[i + 2] = i2c_transfer_out_node.data[i];
    }

    HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);
    SendCANMessage(&canTxMessage);
    HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);

    i2cTransactionReadyToStart = 1;
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
