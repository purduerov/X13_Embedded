/* USER CODE BEGIN Header */
/**
    ******************************************************************************
    * @file                     : main.c
    * @brief                    : Main program body
    ******************************************************************************
    ** This notice applies to any and all portions of this file
    * that are not between comment pairs USER CODE BEGIN and
    * USER CODE END. Other portions of this file, whether
    * inserted by the user or by software development tools
    * are owned by their respective copyright owners.
    *
    * COPYRIGHT(c) 2019 STMicroelectronics
    *
    * Redistribution and use in source and binary forms, with or without modification,
    * are permitted provided that the following conditions are met:
    *     1. Redistributions of source code must retain the above copyright notice,
    *            this list of conditions and the following disclaimer.
    *     2. Redistributions in binary form must reproduce the above copyright notice,
    *            this list of conditions and the following disclaimer in the documentation
    *            and/or other materials provided with the distribution.
    *     3. Neither the name of STMicroelectronics nor the names of its contributors
    *            may be used to endorse or promote products derived from this software
    *            without specific prior written permission.
    *
    * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    *
    ******************************************************************************
    */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "assert.h"
#include "common.h"
#include "canFilterBankConfig.h"
#include "queue.h"
#include "stdint.h"

#pragma GCC diagnostic warning "-Wunused-macros"
#pragma GCC diagnostic warning "-Wsign-compare"
#pragma GCC diagnostic warning "-Wconversion"
#pragma GCC diagnostic warning "-Wredundant-decls"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
	CAN_TxHeaderTypeDef canTxHeader;
	uint8_t data[8];
} CanTxData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_ADC_INIT_WAIT_MS 500

#define CAN_ST_ID_REG_N_BITS 11
#define CAN_ST_ID_REG_MAX MASK_OF(CAN_ST_ID_REG_N_BITS)

/*
 * The resistor voltage divisions for the CAN ID have a pull up on the ESC controller board
 * where this micro is. The pull down is on the backplane. ID 206 will be used if the board
 * is unconnected, resulting in ADC value of VCC (4095). The other voltage divisions are
 * designed to be at 0.3, 0.5, and 0.7 VCC. So the "bins" for them will be 0.2 to 0.4 are 0.3
 * or ID 201, 0.4 to 0.6 are 0.5 or ID 202, and 0.6 to 0.8 are 0.7 or ID 203.
 */

#define ADC_MAX_VALUE (4095U)
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

#define CAN_ID_RIR_SHIFT_AMOUNT 21

#define NUM_CAN_TX_QUEUE_MESSAGES 5

#define SEND_ID_INC 0x100U
#define SEND_ID (canId + SEND_ID_INC)

#define voltToKISSFormat(volt) (volt * 100U)

#define ROV_VOLT_MIN 10U
#define ROV_VOLT_MIN_KISS (voltToKISSFormat(ROV_VOLT_MIN))
#define ROV_VOLT_MAX_KISS 4060U
#define ROV_VOLT_DIVISOR ((ROV_VOLT_MAX_KISS - ROV_VOLT_MIN_KISS) / UINT8_MAX)

// Assert that the divisor is a whole number.
compile_assert((float)ROV_VOLT_DIVISOR == ((ROV_VOLT_MAX_KISS - ROV_VOLT_MIN_KISS) / (float)UINT8_MAX));

typedef enum {
	KISS_TEMP = 0,
	KISS_VOLT_HIGH,
	KISS_VOLT_LOW,
	KISS_CURRENT_HIGH,
	KISS_CURRENT_LOW,
	KISS_ENERGY_HIGH,
	KISS_ENERGY_LOW,
	KISS_ERPM_HIGH,
	KISS_ERPM_LOW,
	KISS_CRC,
	KISS_NO_CRC_COUNT = KISS_CRC,
	KISS_CRC_COUNT
} kiss_tlm_index_t;

typedef enum {
	ROV_TEMP = 0,
	ROV_VOLT,
	ROV_CURRENT_HIGH,
	ROV_CURRENT_LOW,
	ROV_ENERGY_HIGH,
	ROV_ENERGY_LOW,
	ROV_ERPM_HIGH,
	ROV_ERPM_LOW,
	ROV_TLM_COUNT
} rov_tlm_index_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t canId = 0x206;
uint8_t adcConfigured = 0;

queue_handle_t canTxQueueHandle;
CanTxData canTxData[NUM_CAN_TX_QUEUE_MESSAGES];
CanTxData canTxPrivateMessageToSend;

static uint8_t telemetryBuffer[KISS_CRC_COUNT] = {0};
static uint8_t telemetryBytesRecieved;
static uint8_t uartRxBuffer;
static volatile uint8_t sendTelemetry;
// Not sure if some of the above variables should or shouldn't be volatile so that
// main properly checks them after the interrupt modifies them.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

void EnablePWMOutput(TIM_HandleTypeDef *_htim);
static uint32_t byte_to_pwm(uint8_t byte);

static void CAN_ConfigureFilterForThrusterOperation(void);
void SendCANMessage(CanTxData *canTxDataToSend);

//  Interrupt Callback Functions
void CAN_FIFO0_RXMessagePendingCallback(CAN_HandleTypeDef *_hcan);
void CAN_TxRequestCompleteCallback(CAN_HandleTypeDef *_hcan);
void ADC_ConversionCompleteCallback(ADC_HandleTypeDef *_hadc);
void TIM14_TimeElapsedCallback(TIM_HandleTypeDef *_htim);
void TIM16_TimeElapsedCallback(TIM_HandleTypeDef *_htim);

static uint8_t packROVVolt(uint16_t kissVolt);
static void sendTelemetryData(void);
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  InitializeQueueModule();
  CreateQueue((void*)canTxData, sizeof(canTxData[0]), N_ELEMENTS(canTxData), &canTxQueueHandle);

  //  Enable PWM Outputs to ESCs
  EnablePWMOutput(&htim3);

  //  Start Timer to begin sampling ESC_ID with ADC
  HAL_TIM_Base_Start_IT(&htim14);

  // Start waiting for a telemetry packet to be sent.
  HAL_UART_Receive_IT(&huart1, &uartRxBuffer, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(sendTelemetry) {
			if(telemetryBytesRecieved >= KISS_NO_CRC_COUNT) {
				sendTelemetryData();
			}
			telemetryBytesRecieved = 0;
			sendTelemetry = 0;
		}
		asm volatile("wfi");
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
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

  CAN_ConfigureFilterForThrusterOperation();

  //  Enable TX Request Complete Interrupt
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
  HAL_CAN_RegisterCallback(&hcan, HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, CAN_TxRequestCompleteCallback);
  HAL_CAN_RegisterCallback(&hcan, HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID, CAN_TxRequestCompleteCallback);
  HAL_CAN_RegisterCallback(&hcan, HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID, CAN_TxRequestCompleteCallback);

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
  htim14.Init.Period = NUM_ADC_INIT_WAIT_MS - 1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  //  Reconfigure Timer ARR to count up to NUM_ADC_INIT_WAIT_MS ms until ADC conversion start
  htim14.Instance->ARR = htim14.Init.Period;

  //  Configure Timer Interrupt Callback
  HAL_TIM_RegisterCallback(&htim14, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM14_TimeElapsedCallback);

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = TLM_PACKET_ARRIVAL_MS-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

    htim14.Instance->ARR = htim14.Init.Period;

	HAL_TIM_RegisterCallback(&htim16, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM16_TimeElapsedCallback);
  /* USER CODE END TIM16_Init 2 */

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
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  HAL_StatusTypeDef status = HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, HAL_UART_RxCpltCallback);
  (void)status;
  /* USER CODE END USART1_Init 2 */

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
static void CAN_ConfigureFilterForThrusterOperation()
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

	canFilterMask.stdId = CAN_ST_ID_REG_MAX;
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
	uint32_t data_32 = _hcan->Instance->sFIFOMailBox[0].RDHR;
	uint8_t *data = (uint8_t *)&data_32;

	//  Release Output Mailbox
	SET_BIT(_hcan->Instance->RF0R, CAN_RF0R_RFOM0);

	if(_hcan->Instance->sFIFOMailBox[0].RIR >> CAN_ID_RIR_SHIFT_AMOUNT != canId) {
		uint8_t REEE = 0xFF;
		(void)REEE;
		Error_Handler();
		return;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

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

	// Restart TIM14 to flash PA15 LED
	HAL_ADC_Stop_IT(_hadc);

	// Configure CAN Filter for Thrusters and
	CAN_ConfigureFilterForThrusterOperation();
	HAL_CAN_Start(&hcan);  //  Enters Normal Operating Mode

	// Restart TIM14 to flash PA15 LED
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
	} else {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	}
}

void CAN_TxRequestCompleteCallback(CAN_HandleTypeDef *_hcan)
{
	uint32_t txMailboxNumber;
	CanTxData *canTxDataToSend;

	if (!isQueueEmpty(canTxQueueHandle) && HAL_CAN_GetTxMailboxesFreeLevel(_hcan) > 0)
	{
		RemoveFromQueue(canTxQueueHandle, (void **)&canTxDataToSend);
		HAL_CAN_AddTxMessage(_hcan, &(canTxDataToSend->canTxHeader), canTxDataToSend->data, &txMailboxNumber);
	}
}

void SendCANMessage(CanTxData *canTxDataToSend)
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
			assert(0);
		}
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
uint32_t byte_to_pwm(uint8_t byte)
{
	float exact;
	exact = (uint32_t)byte * (40.0F/255.0F) + 55.0F;
	return (uint32_t) (exact + 0.5F); //rounds up the integer by adding 0.5
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	telemetryBuffer[telemetryBytesRecieved] = uartRxBuffer;
	++telemetryBytesRecieved;
	if(telemetryBytesRecieved == 1) {
		// start timer. A new packet is sent every 32 ms.
		// The Telemetry packet should arrive in the first 10 ms.
		// After 10 ms, send whatever has been received, which may or may not include a 10th CRC byte.
		HAL_TIM_Base_Start_IT(&htim16);
		telemetryBuffer[KISS_CRC] = 0;  // Clear the CRC byte in case this packet doesn't have one.
	} else if(telemetryBytesRecieved == KISS_CRC_COUNT) {
		// As an optimization, if we've gotten all 10 bytes, don't bother waiting and set them to be sent.
		// Stop timer
		HAL_TIM_Base_Stop_IT(&htim16);
		TIM16->CNT = 0;
		sendTelemetry = 1;
	}
	HAL_UART_Receive_IT(huart, &uartRxBuffer, 1);
}

static inline uint8_t packROVVolt(uint16_t kissVolt) {
	if(kissVolt <= ROV_VOLT_MIN_KISS) {
		return 0;
	} else if(kissVolt >= ROV_VOLT_MAX_KISS) {
		return UINT8_MAX;
	} else {
		return (uint8_t)((kissVolt - voltToKISSFormat(ROV_VOLT_MIN)) / ROV_VOLT_DIVISOR);
	}
	/*
	 * Does the conversion of (V - 1000) / 12
	 * The max (40.6 V / KISS 4060) maps to 255
	 * The min (10 V / KISS 1000) maps to 0
	 * Voltages outside that range are clipped
	 */
}

void sendTelemetryData(void) {
	CanTxData canSendPacket;
	canSendPacket.data[ROV_TEMP] = telemetryBuffer[KISS_TEMP];
	uint16_t voltage = (uint16_t)((((uint16_t)telemetryBuffer[KISS_VOLT_HIGH]) << 8U) + telemetryBuffer[KISS_VOLT_LOW]);
	canSendPacket.data[ROV_VOLT] = packROVVolt(voltage);
	canSendPacket.data[ROV_CURRENT_HIGH] = telemetryBuffer[KISS_CURRENT_HIGH];
	canSendPacket.data[ROV_CURRENT_LOW] = telemetryBuffer[KISS_CURRENT_LOW];
	canSendPacket.data[ROV_ENERGY_HIGH] = telemetryBuffer[KISS_ENERGY_HIGH];
	canSendPacket.data[ROV_ENERGY_LOW] = telemetryBuffer[KISS_ENERGY_LOW];
	canSendPacket.data[ROV_ERPM_HIGH] = telemetryBuffer[KISS_ERPM_HIGH];
	canSendPacket.data[ROV_ERPM_LOW] = telemetryBuffer[KISS_ERPM_LOW];
	canSendPacket.canTxHeader.StdId = SEND_ID;
	canSendPacket.canTxHeader.DLC = ROV_TLM_COUNT;
	SendCANMessage(&canSendPacket);
}

void TIM16_TimeElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Set the flag for main to send telemetry data
	sendTelemetry = 1;

	// Stop the timer.
	HAL_TIM_Base_Stop_IT(htim);
	TIM16->CNT = 0;
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
	volatile int stayInLoop = 1;
	while(stayInLoop) {}
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
