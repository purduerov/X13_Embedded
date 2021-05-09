/*
 * solenoid.h
 *
 *  Created on: Nov 7, 2020
 *      Author: Conne
 */

#ifndef INC_SOLENOID_H_
#define INC_SOLENOID_H_

#include "stm32f0xx_hal.h"

//  Solenoid Error Codes
typedef enum SolenoidErrorCodeEnum
{
	SOLENOID_SUCCESS = 0,
	SOLENOID_INVALID_SOLENOID_INDEX = 1,
	SOLENOID_INVALID_GPIO_PORT = 2,
	SOLENOID_INVALID_GPIO_PIN = 3,
	SOLENOID_INVALID_NUMBER_OF_SOLENOIDS = 4
} SolenoidErrorCode;

typedef struct SolenoidStruct
{
	GPIO_TypeDef* port;
	uint16_t pin;
} Solenoid;

//  Function Declarations
SolenoidErrorCode verifySolenoidIndex(int solenoidIndex);
SolenoidErrorCode verifyNumberOfSolenoids(int numSolenoids);
void configureNumberOfSolenoids(int numSolenoids);

SolenoidErrorCode addSolenoid(int solenoidIndex, GPIO_TypeDef* gpioPort, uint16_t gpioPin);
SolenoidErrorCode addSolenoids(int numberOfSolenoids, int* solenoidNumber, GPIO_TypeDef** gpioPorts, uint16_t* gpioPins);
void configureSolenoid(int solenoidIndex, GPIO_InitTypeDef* gpioInit);
void configureSolenoids(GPIO_InitTypeDef* gpioInit);

void enableSolenoid(int solenoidIndex);
void enableSolenoids();
void disableSolenoid(int solenoidIndex);
void disableSolenoids();


#endif /* INC_SOLENOID_H_ */
