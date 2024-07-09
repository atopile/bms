/*******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Cfg.cpp
* @brief  : Adbms6948 driver source file containing the Post-Build configuration parameters.
* @version: $Revision: 481 $
* @date   : $Date: 2020-09-29 02:23:53 +0530 (Tue, 29 Sep 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADBMS6948_USER_CONFIG   ADBMS6948 User Configuration
* @{
*/
/*============= I N C L U D E S =============*/
#include "Adbms6948.h"
// #include "bms_configuration.h"

/*============= D A T A =============*/

/*!
    @brief          Array containing number of cells per device.

 */
static const uint8_t  Adbms6948_knNoOfCellsPerDevice_0[1u] =
{
	SYS_MAX_CELL_MON_CELLS
};

/*!
    @brief          Array containing the OC1 Warning window values..

    @warning        These values depend on the system
 */
static Adbms6948_OC1WarningWindowType anOC1WarningWindows[6u] =
{
    0xFFu,  /* OC1 warning configuration of window 1*/
    0xFFu,  /* OC1 warning configuration of window 2*/
    0xFFu,  /* OC1 warning configuration of window 3*/
    0xFFu,  /* OC1 warning configuration of window 4*/
    0xFFu,  /* OC1 warning configuration of window 5*/
    0xFFu   /* OC1 warning configuration of window 6*/
};

/*!
    @brief          Array containing the OC1 Threshold values.

    @warning        These values depend on the system
 */
static Adbms6948_OC1ThesholdType anOC1ThresholdTypes[6u] =
{
    0x7Du,  /* OC1 Threshold of window 1*/
    0xFFu,  /* OC1 Threshold of window 2*/
    0xFFu,  /* OC1 Threshold of window 3*/
    0xFFu,  /* OC1 Threshold of window 4*/
    0xFFu,  /* OC1 Threshold of window 5*/
    0xFFu   /* OC1 Threshold of window 6*/
};

/*!
    @brief          Array containing the time constants for OC2 algorithm.

    @warning        These values depend on the system
 */
static Adbms6948_OC2RCTimeConstantType kanOC2RCTimeConstants[11u] =
{
    0x8Bu,  /* RC01 byte value*/
    0x7Bu,  /* RC11 byte value*/
    0x78u,  /* RC12 byte value*/
    0x48u,  /* RC22 byte value*/
    0x49u,  /* RC23 byte value*/
    0x4Du,  /* RC33 byte value*/
    0x68u,  /* RC34 byte value*/
    0x1Bu,  /* RC44 byte value*/
    0x2Du,  /* RC45 byte value*/
    0x1Eu,  /* RC55 byte value*/
    0x3Du   /* RC5A byte value*/
};

/*TODO: Keep both OC1 and OC2 in a structure so that they can be directly
        updated from downstream command*/
/*!
    @brief          OC1 Algorithm Configuration Parameters

    @warning        These values depend on the system
 */
static Adbms6948_OC1CfgType Adbms6948_koOC1CfgData =
{
    3u,                     /* Adbms6948_nCyclesW1*/
    6u,                     /* Adbms6948_nCyclesW2*/
    9u,                     /* Adbms6948_nCyclesW3*/
    13u,                    /* Adbms6948_nCyclesW4*/
    12u,                    /* Adbms6948_nCyclesW5*/
    13u,                    /* Adbms6948_nCyclesW6*/
    0u,                     /* Adbms6948_nWarnCfg*/
    18u,                    /* Adbms6948_nGainCtrl1*/
    &anOC1WarningWindows[0],/* OC1 Warning configuration of windows*/
    &anOC1ThresholdTypes[0] /* OC1 Threshold of windows*/
};

/*!
    @brief          OC2 Algorithm Configuration Parameters

    @warning        These values depend on the system
 */
static Adbms6948_OC2CfgType Adbms6948_koOC2CfgData =
{
    &kanOC2RCTimeConstants[0u],
    0x3fu,
    0xFF,
    0xFF,
    0x46
};

/*!
    @brief          Configuration data for the ADBMS6948 device.

 */
static const Adbms6948_DaisyChainCfgType  Adbms6948_ChainConfigData[1u] =
{
	{
	    TRUE, /* Enable reference ON */
	    TRUE, /* Init Enable */
	    SYS_CM_DEVICES_PRESENT, /* No of devices */
		Adbms6948_knNoOfCellsPerDevice_0, /* Cells per device */
		0x03FFu, /* GPIO Pin control Selection */
		0x4E2u, /* Over voltage threshold (val * 16 * 100uV) */
		0x271u, /* Under voltage threshold (val * 16 * 100uV) */
		ADBMS6948_FILTER_DISABLED,
		ADBMS6948_CTH_VOLT_22_5mv,
        ADBMS6948_GPIO11_OC2_MODE,    /*GPIO11 mode*/
        &Adbms6948_koOC1CfgData,      /*OC1 configuration*/
        &Adbms6948_koOC2CfgData       /*OC2 configuration*/
	},
};

/*!
    @brief          Runtime error configuration parameters.

 */
static const Adbms6948_RunTimeErrorCfgType  Adbms6948_RunTimeErrorConfigData[1] =
{
	{
		/* Adbms6948_E_Invalid_PEC */
		{
			(uint8_t)STD_ON,
			(uint16_t)0x14U
		},

		/* Adbms6948_E_CommandCount */
		{
			(uint8_t)STD_ON,
			(uint16_t)0x15U
		},

		/* Adbms6948_E_Platform */
		{
			(uint8_t)STD_ON,
			(uint16_t)0x16U
		}
	}
};

const Adbms6948_CfgType  Adbms6948ConfigSet_0_PB =
{
	Adbms6948_ChainConfigData,
	Adbms6948_RunTimeErrorConfigData,
};

/** @}*/ /* ADBMS6948_USER_CONFIG */

/*
*
* EOF
*
*/

