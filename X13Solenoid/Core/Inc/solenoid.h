/*
 * solenoid.h
 *
 *  Created on: Nov 7, 2020
 *      Author: Conner
 */

#ifndef INC_SOLENOID_H_
#define INC_SOLENOID_H_

#include "stm32f0xx_hal.h"
#include "common.h"

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

#define NUM_SOLENOIDS 6U

#define verifyNumberOfSolenoids(numSolenoids) ((numSolenoids > NUM_SOLENOIDS) ? SOLENOID_INVALID_NUMBER_OF_SOLENOIDS : SOLENOID_SUCCESS)


//  Function Declarations
SolenoidErrorCode verifySolenoidIndex(uint8_ft solenoidIndex);
void configureNumberOfSolenoids(uint8_ft numSolenoids);

SolenoidErrorCode addSolenoid(uint8_ft solenoidIndex, GPIO_TypeDef *gpioPort, uint16_t gpioPin);
SolenoidErrorCode addSolenoids(uint8_ft numberOfSolenoids, uint8_ft restrict const *solenoidIndices, GPIO_TypeDef restrict * const *gpioPorts, uint16_t restrict const *gpioPins);
void configureSolenoid(uint8_ft solenoidIndex, GPIO_InitTypeDef *gpioInit);
void configureSolenoids(GPIO_InitTypeDef *gpioInit);

void enableSolenoid(uint8_ft solenoidIndex);
void enableSolenoids(void);
void disableSolenoid(uint8_ft solenoidIndex);
void disableSolenoids(void);
void setSolenoid(uint8_ft solenoidIndex, uint8_ft state);


#endif /* INC_SOLENOID_H_ */
