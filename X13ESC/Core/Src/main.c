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
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

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
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_4TQ;
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
  CAN_FilterTypeDef thrusterOperationFilter;
  thrusterOperationFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  thrusterOperationFilter.FilterBank = 0;
  thrusterOperationFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  thrusterOperationFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  thrusterOperationFilter.FilterActivation = CAN_FILTER_ENABLE;


  HAL_CAN_ConfigFilter(hcan, TODO);

  HAL_CAN_Start(hcan);  //  Enters Normal Operating Mode
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

#define CAN_STD_ID_MASK 0x7FF

#define CAN_STD_ID_16BIT_SCALE_SHIFT 5
#define CAN_RTR_16BIT_SCALE_SHIFT 4
#define CAN_IDE_16BIT_SCALE_SHIFT 3
#define CAN_EXT_ID_16BIT_SCALE_MASK 0x38000

uint32_t CAN_ConfigFilterIdSection16Bit(CAN_IDTypeDef* id)
{
	uint32_t stdid = (id->stdId & CAN_STD_ID_MASK) << CAN_STD_ID_16BIT_SCALE_SHIFT;
	uint32_t rtr = ((id->rtr == RTR_SET) ? 1 : 0) << CAN_RTR_16BIT_SCALE_SHIFT;
	uint32_t ide = ((id->ide == IDE_SET) ? 1 : 0) << CAN_IDE_16BIT_SCALE_SHIFT;
	uint32_t extid = id->extId & CAN_EXT_ID_16BIT_SCALE_MASK;
	return stdid | rtr | ide | extid;
}

uint32_t* CAN_ConfigFilterIdSection32Bit(CAN_IDTypeDef* id)
{
	uint32_t returnBytes[2];
	uint32_t stdid = (id->stdId & CAN_STD_ID_MASK) << 5;
	uint32_t extidHigh = (id->extId & 0x3E000) >> 13;
	returnBytes[1] = stdid | extid;

	uitn32_t extidLow = (id->extId & 0x1FFF) << 3;
	uint32_t ide = ((id->ide == IDE_SET) ? 1 : 0) << 2;
	uint32_t rtr = ((id->rtr == RTR_SET) ? 1 : 0) << 1;
	returnBytes[0] = extid | ide | rtr;

	return returnBytes;
}

void CAN_ConfigFilterId(CAN_FilterTypeDef* halFilterConfig, CAN_IDTypeDef* id, CAN_IDTypeDef* mask)
{
	uint32_t* filterId;
	uint32_t* maskId;

	if (halFilterConfig->FilterMode == CAN_FILTERMODE_IDMASK)
	{
		if (halFilterConfig->FilterScale == CAN_FILTERSCALE_16BIT)
		{
			halFilterConfig->FilterIdLow = CAN_ConfigFilterIdSection16Bit(id[0]);
			halFilterConfig->FilterIdHigh = CAN_ConfigFilterIdSection16Bit(mask[0]);
			halFilterConfig->FilterMaskIdLow = CAN_ConfigFilterIdSection16Bit(id[1]);
			halFilterConfig->FilterMaskIdHigh = CAN_ConfigFilterIdSection16Bit(mask[1]);
		}
		else if (halFilterConfig->FilterScale == CAN_FILTERSCALE_32BIT)
		{
			filterId = CAN_ConfigFilterIdSection32Bit(id);
			halFilterConfig->FilterIdHigh = filterId[1];
			halFilterConfig->FilterIdLow = filterId[0];

			maskId = CAN_ConfigFilterIdSection32Bit(mask);
			halFilterConfig->FilterMaskIdHigh = maskId[1];
			halFilterConfig->FilterMaskIdLow = maskId[0];
		}
		else
		{
			//  Error
		}
	}
	else if (halFilterConfig->FilterMode == CAN_FILTERMODE_IDLIST)
	{
		if (halFilterConfig->FilterScale == CAN_FILTERSCALE_16BIT)
		{
			halFilterConfig->FilterIdLow = CAN_ConfigFilterIdSection16Bit(id[0]);
			halFilterConfig->FilterIdHigh = CAN_ConfigFilterIdSection16Bit(id[1]);
			halFilterConfig->FilterMaskIdLow = CAN_ConfigFilterIdSection16Bit(id[2]);
			halFilterConfig->FilterMaskIdHigh = CAN_ConfigFilterIdSection16Bit(id[3]);
		}
		else if (halFilterConfig->FilterScale == CAN_FILTERSCALE_32BIT)
		{
			filterId = CAN_ConfigFilterIdSection32Bit(id[0]);
			halFilterConfig->FilterIdHigh = filterId[1];
			halFilterConfig->FilterIdLow = filterId[0];

			filterId = CAN_ConfigFilterIdSection32Bit(id[1]);
			halFilterConfig->FilterMaskIdHigh = filterId[1];
			halFilterConfig->FilterMaskIdLow = filterId[0];
		}
		else
		{
			//  Error
		}
	}
	else
	{
		//  Error
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
