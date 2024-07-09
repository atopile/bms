/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Pec.h
* @brief  : Type and domain declarations used in the upper layer of the
			Pec module.
* @version: $Revision: 519 $
* @date   : $Date: 2020-10-13 16:59:52 +0530 (Tue, 13 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup Adbms6948_Pec
* @{
*/

#ifndef ADBMS6948_PEC_H
#define ADBMS6948_PEC_H


/*============= I N C L U D E S =============*/
#include "Adbms6948_Types.h"

/*======= P U B L I C P R O T O T Y P E S ========*/

uint16_t Adbms6948_Pec15Calculate
(
    uint8_t 	*pDataBuf,
    uint8_t   	 nLength
);

uint16_t Adbms6948_Pec10Calculate
(
uint8_t 	*pDataBuf,
boolean  	 bIsRxCmd,
uint8_t   	 nLength
);

#endif /* ADBMS6948_PEC_H */

/** @}*/ /* Adbms6948_Pec */

/*
*
* EOF
*
*/

