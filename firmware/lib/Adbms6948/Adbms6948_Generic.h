/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Generic.h
* @brief  : Header file for Generic services
* @version: $Revision: 519 $
* @date   : $Date: 2020-10-13 16:59:52 +0530 (Tue, 13 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_ADBMS6948_DRIVER_INTERNAL_DATA	ADBMS6948 Driver Internal Data Types
* @{
*/

#ifndef ADBMS6948_GENERIC_H
#define ADBMS6948_GENERIC_H

/*============= I N C L U D E S =============*/
#include "Adbms6948.h"
#include "Adbms6948_Common.h"
/*============= D A T A T Y P E S =============*/
/*! \addtogroup ADI_ADBMS6948_GENERIC_DATA	ADBMS6948 Generic Data Types
* @{
*/

/* Variable to contain the list of all the ADBMS6948 commands */
extern const uint16_t  Adbms6948_CmdList[];

#ifndef ADI_DISABLE_DOCUMENTATION
/*======= P U B L I C P R O T O T Y P E S ========*/
Adbms6948_ReturnType  Adbms6948_Int_ReadDriverVersion
(
Adbms6948_VersionInfoType *pDriverVersion
);

Adbms6948_ReturnType Adbms6948_Int_OverVoltEval
(
uint16_t *   			  		 pCellData,
Adbms6948_OVThresCfgType *  	 pOVThreshold,
Adbms6948_OVEvalType 			*pEvalResult,
const uint8_t 					 knChainID
);

Adbms6948_ReturnType Adbms6948_Int_UnderVoltEval
(
uint16_t 						*pCellData,
Adbms6948_UVThresCfgType 		*pUVThreshold,
Adbms6948_UVEvalType 			*pEvalResult,
const uint8_t 					 knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_ReadErrorCounts
(
Adbms6948_ErrorCounts 	*pErrCnt,
const uint8_t            knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_ClearErrorCounts
(
const uint8_t         knChainID
);
#endif
/** @}*/

/** @}*/		/* ADI_ADBMS6948_DRIVER_INTERNAL_DATA */

#endif /* ADBMS6948_H */



/*
*
* EOF
*
*/

