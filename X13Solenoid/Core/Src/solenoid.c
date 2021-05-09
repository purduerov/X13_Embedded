/*
 * solenoid.c
 *
 *  Created on: Nov 7, 2020
 *      Author: Conner
 */

#include "solenoid.h"

static Solenoid solenoidArray[NUM_SOLENOIDS];

static uint8_ft configuredNumberOfSolenoids;

SolenoidErrorCode verifySolenoidIndex(uint8_ft solenoidIndex)
{
	if (solenoidIndex >= NUM_SOLENOIDS)
	{
		return SOLENOID_INVALID_SOLENOID_INDEX;
	}
	return SOLENOID_SUCCESS;
}

void configureNumberOfSolenoids(uint8_ft numSolenoids)
{
	configuredNumberOfSolenoids = numSolenoids;
}

SolenoidErrorCode addSolenoid(uint8_ft solenoidIndex, GPIO_TypeDef *gpioPort, uint16_t gpioPin)
{
	//  Check Parameters
	if (gpioPort == NULL)
	{
		return SOLENOID_INVALID_GPIO_PORT;
	}
	if (!IS_GPIO_PIN(gpioPin))
	{
		return SOLENOID_INVALID_GPIO_PIN;
	}

	solenoidArray[solenoidIndex].port = gpioPort;
	solenoidArray[solenoidIndex].pin = gpioPin;

	return SOLENOID_SUCCESS;
}

SolenoidErrorCode addSolenoids(uint8_ft numberOfSolenoids, uint8_ft const * restrict solenoidIndices, GPIO_TypeDef * const * restrict gpioPorts, uint16_t const * restrict gpioPins)
{
	SolenoidErrorCode errorCode;

	for (uint8_ft i = 0; i < numberOfSolenoids; ++i)
	{
		errorCode = addSolenoid(solenoidIndices[i], gpioPorts[i], gpioPins[i]);
		if (errorCode != SOLENOID_SUCCESS)
		{
			return errorCode;
		}
	}

	return SOLENOID_SUCCESS;
}

void configureSolenoid(uint8_ft solenoidIndex, GPIO_InitTypeDef *gpioInit)
{
	//  Disable Pin associated with Solenoid
	disableSolenoid(solenoidIndex);

	//  Override gpioInit->Pin with GPIO Pin of selected solenoid
	gpioInit->Pin = solenoidArray[solenoidIndex].pin;

	//  Initialize GPIO Pin for selected solenoid
	HAL_GPIO_Init(solenoidArray[solenoidIndex].port, gpioInit);
}

void configureSolenoids(GPIO_InitTypeDef *gpioInit)
{
	for (uint8_ft i = 0; i < configuredNumberOfSolenoids; ++i)
	{
		configureSolenoid(i, gpioInit);
	}
}

void enableSolenoid(uint8_ft solenoidIndex)
{
	HAL_GPIO_WritePin(solenoidArray[solenoidIndex].port, solenoidArray[solenoidIndex].pin, GPIO_PIN_SET);
}

void enableSolenoids(void)
{
	for (uint8_ft i = 0; i < configuredNumberOfSolenoids; ++i)
	{
		enableSolenoid(i);
	}
}

void disableSolenoid(uint8_ft solenoidIndex)
{
	HAL_GPIO_WritePin(solenoidArray[solenoidIndex].port, solenoidArray[solenoidIndex].pin, GPIO_PIN_RESET);
}

void disableSolenoids(void)
{
	for (uint8_ft i = 0; i < configuredNumberOfSolenoids; ++i)
	{
		disableSolenoid(i);
	}
}

void setSolenoid(uint8_ft solenoidIndex, uint8_ft state)
{
	HAL_GPIO_WritePin(solenoidArray[solenoidIndex].port, solenoidArray[solenoidIndex].pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
