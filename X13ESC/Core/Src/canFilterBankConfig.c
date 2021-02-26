/*
 * canFilterBankConfig.c
 *
 *  Created on: Feb 25, 2021
 *      Author: Conne
 */

#include "canFilterBankConfig.h"

#define CAN_STDID_FILTER_MASK 0x7FF

#define CAN_STDID_FILTER_SHIFT_16BITS 5
#define CAN_RTR_FILTER_SHIFT_16BITS 4
#define CAN_IDE_FILTER_SHIFT_16BITS 3
#define CAN_EXTID_FILTER_MASK_16BITS 0x7
#define CAN_EXTID_FILTER_SHIFT_16BITS 0

#define CAN_STDID_FILTER_SHIFT_32BITS 21
#define CAN_RTR_FILTER_SHIFT_32BITS 1
#define CAN_IDE_FILTER_SHIFT_32BITS 2
#define CAN_EXTID_FILTER_MASK_32BITS 0x3FFFF
#define CAN_EXTID_FILTER_SHIFT_32BITS 3

CAN_FilterBankConfigError CAN_ConfigureFilterBank(CAN_FilterTypeDef* canFilterInstance, CAN_FilterBank* canFilterBank)
{
	CAN_FilterBankConfigError canFilterBankErrorStatus;

	if (canFilterInstance->FilterScale != CAN_FILTERSCALE_16BIT && canFilterInstance->FilterScale != CAN_FILTERSCALE_32BIT)
	{
		return CAN_FILTER_BANK_INVALID_FILTER_SCALE;
	}

	if (canFilterInstance->FilterScale == CAN_FILTERSCALE_16BIT)
	{
		if (canFilterBank->filterMode == CANIDFilterMode_16BitId || canFilterBank->filterMode == CANIDFilterMode_16BitMask)
		{
			CAN_ConfigureFilterBank16Bits(canFilterInstance, canFilterBank);
			canFilterBankErrorStatus = CAN_FILTER_BANK_NO_ERROR;
		}
		else
		{
			canFilterBankErrorStatus = CAN_FILTER_BANK_FILTER_SCALE_MISMATCH;
		}
	}
	else if (canFilterInstance->FilterScale == CAN_FILTERSCALE_32BIT)
	{
		if (canFilterBank->filterMode == CANIDFilterMode_32BitId || canFilterBank->filterMode == CANIDFilterMode_32BitMask)
		{
			CAN_ConfigureFilterBank32Bits(canFilterInstance, canFilterBank);
			canFilterBankErrorStatus = CAN_FILTER_BANK_NO_ERROR;
		}
		else
		{
			canFilterBankErrorStatus = CAN_FILTER_BANK_FILTER_SCALE_MISMATCH;
		}
	}

	return canFilterBankErrorStatus;
}

CAN_FilterBankConfigError CAN_ConfigureFilterBank16Bits(CAN_FilterTypeDef* canFilterInstance, CAN_FilterBank* canFilterBank)
{
	if (canFilterInstance->FilterMode != CAN_FILTERMODE_IDLIST && canFilterInstance->FilterMode != CAN_FILTERMODE_IDMASK)
	{
		return CAN_FILTER_BANK_INVALID_FILTER_MODE;
	}

	if (canFilterInstance->FilterMode == CAN_FILTERMODE_IDLIST)
	{
		//  Check 4 IDs
		//  Configure Halfword Registers
		CAN_ConfigureFilterBankRegister16Bits(&(canFilterInstance->FilterIdLow), canFilterBank->id1);
		CAN_ConfigureFilterBankRegister16Bits(&(canFilterInstance->FilterMaskIdLow), canFilterBank->id2);
		CAN_ConfigureFilterBankRegister16Bits(&(canFilterInstance->FilterIdHigh), canFilterBank->id3);
		CAN_ConfigureFilterBankRegister16Bits(&(canFilterInstance->FilterMaskIdHigh), canFilterBank->id4);
	}
	else if (canFilterInstance->FilterMode == CAN_FILTERMODE_IDMASK)
	{
		//  Check 2 IDs, 2 Masks
		//  Configure Halfword Registers
		CAN_ConfigureFilterBankRegister16Bits(&(canFilterInstance->FilterIdLow), canFilterBank->id1);
		CAN_ConfigureFilterBankRegister16Bits(&(canFilterInstance->FilterMaskIdLow), canFilterBank->mask1);
		CAN_ConfigureFilterBankRegister16Bits(&(canFilterInstance->FilterIdHigh), canFilterBank->id2);
		CAN_ConfigureFilterBankRegister16Bits(&(canFilterInstance->FilterMaskIdHigh), canFilterBank->mask2);
	}
	return CAN_FILTER_BANK_NO_ERROR;
}

CAN_FilterBankConfigError CAN_ConfigureFilterBank32Bits(CAN_FilterTypeDef* canFilterInstance, CAN_FilterBank* canFilterBank)
{
	if (canFilterInstance->FilterMode != CAN_FILTERMODE_IDLIST && canFilterInstance->FilterMode != CAN_FILTERMODE_IDMASK)
	{
		return CAN_FILTER_BANK_INVALID_FILTER_MODE;
	}

	if (canFilterInstance->FilterMode == CAN_FILTERMODE_IDLIST)
	{
		//  Check 2 IDs
		//  Configure Halfword Registers
		CAN_ConfigureFilterBankRegister32Bits(&(canFilterInstance->FilterIdHigh), &(canFilterInstance->FilterIdLow), canFilterBank->id1);
		CAN_ConfigureFilterBankRegister32Bits(&(canFilterInstance->FilterMaskIdHigh), &(canFilterInstance->FilterMaskIdLow), canFilterBank->id2);
	}
	else if (canFilterInstance->FilterMode == CAN_FILTERMODE_IDMASK)
	{
		//  Check 1 ID, 1 Mask
		//  Configure Halfword Registers
		CAN_ConfigureFilterBankRegister32Bits(&(canFilterInstance->FilterIdHigh), &(canFilterInstance->FilterIdLow), canFilterBank->id1);
		CAN_ConfigureFilterBankRegister32Bits(&(canFilterInstance->FilterMaskIdHigh), &(canFilterInstance->FilterMaskIdLow), canFilterBank->mask1);
	}
	return CAN_FILTER_BANK_NO_ERROR;
}

void CAN_ConfigureFilterBankRegister16Bits(uint32_t* filterBankHalfwordRegister, CAN_FilterIDMaskConfig* idMaskConfig)
{
	uint32_t stdid = (idMaskConfig->stdId & CAN_STDID_FILTER_MASK) << CAN_STDID_FILTER_SHIFT_16BITS;
	uint32_t rtr = ((idMaskConfig->rtr == CAN_RTR_SET) ? 1 : 0) << CAN_RTR_FILTER_SHIFT_16BITS;
	uint32_t ide = ((idMaskConfig->ide == CAN_IDE_SET) ? 1 : 0) << CAN_IDE_FILTER_SHIFT_16BITS;
	uint32_t extid = (idMaskConfig->extId & CAN_EXTID_FILTER_MASK_16BITS) << CAN_EXTID_FILTER_SHIFT_16BITS;

	*filterBankHalfwordRegister = stdid | rtr | ide | extid;
}

void CAN_ConfigureFilterBankRegister32Bits(uint32_t* filterBankMSHalfwordRegister, uint32_t* filterBankLSHalfwordRegister, CAN_FilterIDMaskConfig* idMaskConfig)
{
	uint32_t stdid = (idMaskConfig->stdId & CAN_STDID_FILTER_MASK) << CAN_STDID_FILTER_SHIFT_32BITS;
	uint32_t rtr = ((idMaskConfig->rtr == CAN_RTR_SET) ? 1 : 0) << CAN_RTR_FILTER_SHIFT_32BITS;
	uint32_t ide = ((idMaskConfig->ide == CAN_IDE_SET) ? 1 : 0) << CAN_IDE_FILTER_SHIFT_32BITS;
	uint32_t extid = (idMaskConfig->extId & CAN_EXTID_FILTER_MASK_32BITS) << CAN_EXTID_FILTER_SHIFT_32BITS;

	uint32_t fullwordRegister = stdid | rtr | ide | extid;
	*filterBankMSHalfwordRegister = (fullwordRegister & (0xFFFF << 16));
	*filterBankLSHalfwordRegister = (fullwordRegister & 0xFFFF);
}
