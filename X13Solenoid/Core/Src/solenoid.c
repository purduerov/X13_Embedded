/*
 * solenoid.c
 *
 *  Created on: Nov 7, 2020
 *      Author: Conne
 */

#include "solenoid.h"

#define NUM_SOLENOIDS 6

Solenoid solenoidArray[NUM_SOLENOIDS];

int configuredNumberOfSolenoids;

SolenoidErrorCode verifySolenoidIndex(int solenoidIndex)
{
	if (solenoidIndex >= NUM_SOLENOIDS || solenoidIndex < 0)
	{
		return SOLENOID_INVALID_SOLENOID_INDEX;
	}
	return SOLENOID_SUCCESS;
}

SolenoidErrorCode verifyNumberOfSolenoids(int numSolenoids)
{
	if (numSolenoids > NUM_SOLENOIDS || numSolenoids < 0)
	{
		return SOLENOID_INVALID_NUMBER_OF_SOLENOIDS;
	}
	return SOLENOID_SUCCESS;
}

void configureNumberOfSolenoids(int numSolenoids)
{
	configuredNumberOfSolenoids = numSolenoids;
}

SolenoidErrorCode addSolenoid(int solenoidIndex, GPIO_TypeDef* gpioPort, uint16_t gpioPin)
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

SolenoidErrorCode addSolenoids(int numberOfSolenoids, int* solenoidIndices, GPIO_TypeDef** gpioPorts, uint16_t* gpioPins)
{
	SolenoidErrorCode errorCode;

	for (int i = 0; i < numberOfSolenoids; i++)
	{
		errorCode = addSolenoid(solenoidIndices[i], gpioPorts[i], gpioPins[i]);
		if (errorCode != SOLENOID_SUCCESS)
		{
			return errorCode;
		}
	}

	return SOLENOID_SUCCESS;
}

void configureSolenoid(int solenoidIndex, GPIO_InitTypeDef* gpioInit)
{
	//  Disable Pin associated with Solenoid
	disableSolenoid(solenoidIndex);

	//  Override gpioInit->Pin with GPIO Pin of selected solenoid
	gpioInit->Pin = solenoidArray[solenoidIndex].pin;

	//  Initialize GPIO Pin for selected solenoid
	HAL_GPIO_Init(solenoidArray[solenoidIndex].port, gpioInit);
}

void configureSolenoids(GPIO_InitTypeDef* gpioInit)
{
	for (int i = 0; i < configuredNumberOfSolenoids; i++)
	{
		configureSolenoid(i, gpioInit);
	}
}

void enableSolenoid(int solenoidIndex)
{
	HAL_GPIO_WritePin(solenoidArray[solenoidIndex].port, solenoidArray[solenoidIndex].pin, GPIO_PIN_SET);
}

void enableSolenoids()
{
	for (int i = 0; i < configuredNumberOfSolenoids; i++)
	{
		enableSolenoid(i);
	}
}

void disableSolenoid(int solenoidIndex)
{
	HAL_GPIO_WritePin(solenoidArray[solenoidIndex].port, solenoidArray[solenoidIndex].pin, GPIO_PIN_RESET);
}

void disableSolenoids()
{
	for (int i = 0; i < configuredNumberOfSolenoids; i++)
	{
		disableSolenoid(i);
	}
}

