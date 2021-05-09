/*
 * led.h
 *
 *  Created on: Nov 14, 2020
 *      Author: Conne
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_tim.h"

typedef enum TimerFreq
{
	TIM_FREQ_HALFHZ,
	TIM_FREQ_1HZ,
	TIM_FREQ_2HZ,
	TIM_FREQ_4HZ,
	TIM_FREQ_8HZ,
	TIM_FREQ_16HZ,
	TIM_FREQ_32HZ
} TimerFrequency;

typedef enum LEDState
{
	LED_OFF = 0,
	LED_ON = 1
} LedState;

typedef struct LEDStruct
{
	GPIO_TypeDef* port;
	uint16_t pin;
	LedState state;
} Led;

void configureLed(GPIO_TypeDef* gpioPort, uint16_t gpioPin, GPIO_InitTypeDef* gpioInit);

void initializeLedTimer(TIM_TypeDef* timerModule);
void timerUpdateEventCallback(TIM_HandleTypeDef* htim);
void initializeLedFlashFrequency(TimerFrequency timerFrequency);
uint32_t getAutoReloadRegisterValue(TimerFrequency timerFrequency);

void setLedOn();
void setLedOff();
void toggleLed();

void flashLed();
void changeLedFlashFrequency(TimerFrequency timerFrequency);
void disableFlashLed();

#endif /* INC_LED_H_ */
