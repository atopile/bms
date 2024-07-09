/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_OtherMonitor.h
* @brief  : Type and domain declarations used in the upper layer of the
			Adbms6948 Other Monitor module.
* @version: $Revision: 517 $
* @date   : $Date: 2020-10-13 11:33:31 +0530 (Tue, 13 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_ADBMS6948_DRIVER_INTERNAL_DATA	ADBMS6948 Driver Internal Data Types
* @{
*/
#ifndef ADBMS6948_OTHER_MONITOR_H
#define ADBMS6948_OTHER_MONITOR_H


/*============= I N C L U D E S =============*/
#include "Adbms6948.h"
#include "Adbms6948_Common.h"

/** Max number of write per call. No of data bytes to be written =  (ADBMS6948_MAX_NO_OF_WRITES * 3u) - 1u */
#define ADBMS6948_MAX_NO_OF_WRITES		(342u)

/*======= P U B L I C P R O T O T Y P E S ========*/
Adbms6948_ReturnType  Adbms6948_Int_ReadDevRevCode
(
uint8_t 		*pRevCode,
const uint8_t    knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_WakeUpBmsIC
(
const uint8_t 		knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_SendSoftReset
(
const uint8_t  		knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_ReadSerialID
(
uint8_t 		*pSerialidCode,
const uint8_t 	 knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_I2CWrite
(
Adbms6948_I2CWriteCfgType 	*poWriteCfg,
uint8_t 					*pnWrStatus,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_I2CRead
(
Adbms6948_I2CReadCfgType 	*poReadCfg,
uint8_t 					*pnRdStatus,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType Adbms6948_Int_AlwaysOnMemWrite
(
uint8_t 					*panData,
const uint8_t				knChainID
);

Adbms6948_ReturnType Adbms6948_Int_AlwaysOnMemRead
(
uint8_t 					*panData,
const uint8_t				knChainID
);

/*! \addtogroup ADI_ADBMS6948_OTHER_MONITOR_DATA	ADBMS6948 Other Monitor Data Types
* @{
*/

/** @}*/
#endif /* ADBMS6948_OTHER_MONITOR_H */

/** @}*/ /* ADI_ADBMS6948_DRIVER_INTERNAL_DATA */

/*
*
* EOF
*
*/

