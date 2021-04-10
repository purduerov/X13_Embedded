/*
 * canFilterBankConfig.h
 *
 *  Created on: Feb 25, 2021
 *      Author: Conne
 */

#ifndef INC_CANFILTERBANKCONFIG_H_
#define INC_CANFILTERBANKCONFIG_H_

#include "stm32f0xx_hal.h"

typedef enum
{
	CAN_IDE_CLEAR = 0,
	CAN_IDE_SET = 1
} CAN_IDEStatus;

typedef enum
{
	CAN_RTR_CLEAR = 0,
	CAN_RTR_SET = 1
} CAN_RTRStatus;

typedef struct
{
	uint32_t stdId;
	uint32_t extId;
	CAN_IDEStatus ide;
	CAN_RTRStatus rtr;
} CAN_FilterIDMaskConfig;

typedef enum
{
	CANIDFilterMode_16BitId = 0,
	CANIDFilterMode_16BitMask = 1,
	CANIDFilterMode_32BitId = 2,
	CANIDFilterMode_32BitMask = 3
} CAN_IDFilterMode;

typedef struct
{
	CAN_IDFilterMode filterMode;
	CAN_FilterIDMaskConfig* id1;
	CAN_FilterIDMaskConfig* id2;
	CAN_FilterIDMaskConfig* id3;
	CAN_FilterIDMaskConfig* id4;
	CAN_FilterIDMaskConfig* mask1;
	CAN_FilterIDMaskConfig* mask2;
} CAN_FilterBank;

typedef enum
{
	CAN_FILTER_BANK_NO_ERROR = 0,
	CAN_FILTER_BANK_FILTER_SCALE_MISMATCH = 1,
	CAN_FILTER_BANK_INVALID_FILTER_SCALE = 2,
	CAN_FILTER_BANK_FILTER_MODE_MISMATCH = 3,
	CAN_FILTER_BANK_INVALID_FILTER_MODE = 4
} CAN_FilterBankConfigError;

CAN_FilterBankConfigError CAN_ConfigureFilterBank(CAN_FilterTypeDef* canFilterInstance, CAN_FilterBank* canFilterBank);
CAN_FilterBankConfigError CAN_ConfigureFilterBank16Bits(CAN_FilterTypeDef* canFilterInstance, CAN_FilterBank* canFilterBank);
CAN_FilterBankConfigError CAN_ConfigureFilterBank32Bits(CAN_FilterTypeDef* canFilterInstance, CAN_FilterBank* canFilterBank);
void CAN_ConfigureFilterBankRegister16Bits(uint32_t* filterBankHalfwordRegister, CAN_FilterIDMaskConfig* idMaskConfig);
void CAN_ConfigureFilterBankRegister32Bits(uint32_t* filterBankMSHalfwordRegister, uint32_t* filterBankLSHalfwordRegister, CAN_FilterIDMaskConfig* idMaskConfig);


#endif /* INC_CANFILTERBANKCONFIG_H_ */
