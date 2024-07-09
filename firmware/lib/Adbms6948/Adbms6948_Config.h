/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Config.h
* @brief  : Type and domain declarations used in the upper layer of the
			Adbms6948 Config module.
* @version: $Revision: 528 $
* @date   : $Date: 2020-10-13 19:41:33 +0530 (Tue, 13 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_ADBMS6948_DRIVER_INTERNAL_DATA	ADBMS6948 Driver Internal Data Types
* @{
*/
#ifndef ADBMS6948_CONFIG_H
#define ADBMS6948_CONFIG_H


/*============= I N C L U D E S =============*/
#include "Adbms6948.h"
#include "Adbms6948_Common.h"

/*======= P U B L I C P R O T O T Y P E S ========*/
Adbms6948_ReturnType  Adbms6948_Int_Init
(
const Adbms6948_CfgType *pkConfig
);

Adbms6948_ReturnType  Adbms6948_Int_DeInit
(
void
);

Adbms6948_ReturnType  Adbms6948_Int_CfgSoakControl
(
boolean 		*pEnable,
const uint8_t  	 knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_CfgSoakTime
(
Adbms6948_AuxSoakTimeCfgType 	*pAuxSoakTimeCfg,
const uint8_t 					knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_CfgOVThreshold
(
Adbms6948_OVThresCfgType 	*pOVThreshold,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_CfgUVThreshold
(
Adbms6948_UVThresCfgType 	*pUVThreshold,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_CfgIIRFilter
(
Adbms6948_IIRCornerFreqType 	*pCornerFreq,
const uint8_t  					 knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_CfgCSCompThresVolt
(
Adbms6948_CSThresVoltType 		*pCSCompThresVolt,
const uint8_t  					 knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_CfgGpioPins
(
Adbms6948_GpioPinCfgType 	*pGpioPinCfg,
const uint8_t 		  		 knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_CfgOC1Params
(
    Adbms6948_OC1CfgType    *poOC1CfgType,
    const uint8_t           knChainID
);

Adbms6948_ReturnType  Adbms6948_Int_CfgOC2Params
(
    Adbms6948_OC2CfgType    *poOC2CfgType,
    const uint8_t           knChainID
);

Adbms6948_ReturnType Adbms6948_Int_CfgCoulombCounter
(
    uint16_t            *pnNumConversions,
    const uint8_t       knChainID
);

/*! \addtogroup ADI_ADBMS6948_CONFIG_DATA	ADBMS6948 Configuration Data Types
* @{
*/

/** @}*/
#endif /* ADBMS6948_CONFIG_H */

/** @}*/ /* ADI_ADBMS6948_DRIVER_INTERNAL_DATA */

/*
*
* EOF
*
*/

