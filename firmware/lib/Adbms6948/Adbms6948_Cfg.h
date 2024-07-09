/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Cfg.h
* @brief  : Configuration header file needs to be updated by user
* @version: $Revision: 439 $
* @date   : $Date: 2020-09-24 21:40:40 +0530 (Thu, 24 Sep 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_ADBMS6948_DRIVER_DATA	ADBMS6948 Driver Data Types
* @{
*/

#ifndef ADBMS6948_CFG_H
#define ADBMS6948_CFG_H

/*! \addtogroup ADI_ADBMS6948_PRECOMPILE_DATA	ADBMS6948 Precompile Configuration
* @{
*/
/*============= I N C L U D E S =============*/
// #include "Std_Types.h"

/*============== D E F I N E S ===============*/
/**	Major Software Version number */
#define ADBMS6948_SW_MAJOR_VERSION_CFG	            1U

/**	Minor Software Version number */
#define ADBMS6948_SW_MINOR_VERSION_CFG	            0U

/** Software Patch Version number */
#define ADBMS6948_SW_PATCH_VERSION_CFG	            0U

/**	AUTOSAR Major Version number */
#define ADBMS6948_AR_RELEASE_MAJOR_VERSION_CFG	    4U

/**	AUTOSAR Minor Version number */
#define ADBMS6948_AR_RELEASE_MINOR_VERSION_CFG	    0U

/** Unique Module Id for Adbms6948 CDD  */
#define ADBMS6948_MODULE_ID_CFG						80U

/**	ADBMS6948 vendor ID  */
#define ADBMS6948_VENDOR_ID_CFG						0U

/**	ADBMS6948 Instance ID */
#define ADBMS6948_INSTANCE_ID_CFG                 	0U


/** The configuration of the number of daisy chains connected to the BMS Controller */
#define ADBMS6948_NO_OF_DAISY_CHAIN                   (1U)
/** The maximum number of BMS devices connected in a single daisy chain */
#define ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN    (1U)
/** The configuration of the Development mode for the ADBMS6948 SW driver */
#define ADBMS6948_DEVELOPMENT_MODE_EN    FALSE


/** Configuration to select the maximum divide factor to calculate the OW threshold for CADC.*/
#define ADBMS6948_CADC_OW_MAX_DIV_FACTOR	((uint16_t)12u)
/** Configuration to select the minimum divide factor to calculate the OW threshold for CADC.*/
#define ADBMS6948_CADC_OW_MIN_DIV_FACTOR	((uint16_t)8u)
/** Configuration to select the maximum divide factor to calculate the OW threshold for SADC.*/
#define ADBMS6948_SADC_OW_MAX_DIV_FACTOR	((uint16_t)12u)
/** Configuration to select the minimum divide factor to calculate the OW threshold for SADC.*/
#define ADBMS6948_SADC_OW_MIN_DIV_FACTOR	((uint16_t)8u)

/** Assigns What Comes on OC1 Status Bus */
#define STATUS_CFG_VAL  ((uint8_t)0x00u)
/*============= D A T A T Y P E S =============*/

/*======= P U B L I C P R O T O T Y P E S ========*/


/** @}*/
#endif /* ADBMS6948_CFG_H */

/** @}*/ /* ADI_ADBMS6948_DRIVER_DATA */

/*
*
* EOF
*
*/

