/*
 * led.c
 *
 *  Created on: Nov 14, 2020
 *      Author: Conne
 */

#include "led.h"

Led led;

TIM_HandleTypeDef timerConfiguration;

#define ARR_HALFHZ (1600 - 1)
#define ARR_1HZ (800 - 1)
#define ARR_2HZ (400 - 1)
#define ARR_4HZ (200 - 1)
#define ARR_8HZ (100 - 1)
#define ARR_16HZ (50 - 1)
#define ARR_32HZ (25 - 1)

void configureLed(GPIO_TypeDef* gpioPort, uint16_t gpioPin, GPIO_InitTypeDef* gpioInit)
{
	//  Configure Led GPIO Port and Pin Number
	led.port = gpioPort;
	led.pin = gpioPin;
	setLedOff();

	//  Override gpioInit->Pin with GPIO Pin of led
	gpioInit->Pin = led.pin;

	//  Initialize GPIO Pin of led
	HAL_GPIO_Init(led.port, gpioInit);
}

void initializeLedTimer(TIM_TypeDef* timerModule)
{
	timerConfiguration.Instance = timerModule;
	//  CK_CNT Frequency is 1.6kHz
	//  Assuming Internal 8MHz is CLK source (CK_PSC)
	timerConfiguration.Init.Prescaler = 5000 - 1;
	timerConfiguration.Init.CounterMode = TIM_COUNTERMODE_UP;
	timerConfiguration.Init.Period = ARR_1HZ;
	timerConfiguration.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	timerConfiguration.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timerConfiguration.State = HAL_TIM_STATE_RESET;

	HAL_TIM_Base_Init(&timerConfiguration);

	//  Configure Interrupt Handler
	HAL_TIM_RegisterCallback(&timerConfiguration, HAL_TIM_PERIOD_ELAPSED_CB_ID, timerUpdateEventCallback);
}

void timerUpdateEventCallback(TIM_HandleTypeDef* htim)
{
	toggleLed();
}

//  With Auto Reload Preload Enabled, new Frequency is not set until next Update Event
//  which is when counter overflows
void initializeLedFlashFrequency(TimerFrequency timerFrequency)
{
	uint32_t newArr = getAutoReloadRegisterValue(timerFrequency);
	timerConfiguration.Init.Period = newArr;
	timerConfiguration.Instance->ARR = newArr;
}

void changeLedFlashFrequency(TimerFrequency timerFrequency)
{
	//  Disable timer counting and disable Update Interrupt
	HAL_TIM_Base_Stop_IT(&timerConfiguration);

	//  Set ARR
	initializeLedFlashFrequency(timerFrequency);

	//  Generate Update Event
	//  Clears Counter, Clears Prescaler Counter, and updates ARR
	timerConfiguration.Instance->EGR = TIM_EGR_UG;

	//  Toggle LED to start fresh at count = 0
	toggleLed();

	//  Enable Timer again
	HAL_TIM_Base_Start_IT(&timerConfiguration);
}

uint32_t getAutoReloadRegisterValue(TimerFrequency timerFrequency)
{
	uint32_t newArr;

	switch (timerFrequency)
	{
		case TIM_FREQ_HALFHZ: newArr = ARR_HALFHZ; break;
		case TIM_FREQ_1HZ: newArr = ARR_1HZ; break;
		case TIM_FREQ_2HZ: newArr = ARR_2HZ; break;
		case TIM_FREQ_4HZ: newArr = ARR_4HZ; break;
		case TIM_FREQ_8HZ: newArr = ARR_8HZ; break;
		case TIM_FREQ_16HZ: newArr = ARR_16HZ; break;
		case TIM_FREQ_32HZ: newArr = ARR_32HZ; break;
		default: newArr = ARR_1HZ;
	}

	return newArr;
}

void setLedOn()
{
	HAL_GPIO_WritePin(led.port, led.pin, GPIO_PIN_SET);
	led.state = LED_ON;
}

void setLedOff()
{
	HAL_GPIO_WritePin(led.port, led.pin, GPIO_PIN_RESET);
	led.state = LED_OFF;
}

void toggleLed()
{
	if (led.state == LED_OFF)
	{
		setLedOn();
	}
	//  From On state or disabled state
	else
	{
		setLedOff();
	}
}

void flashLed()
{
	//  Enables Update Interrupt and Enables Timer
	HAL_TIM_Base_Start_IT(&timerConfiguration);
}

void disableFlashLed()
{
	//  Disable Update Interrupt and Disables Timer
	HAL_TIM_Base_Stop_IT(&timerConfiguration);
}
