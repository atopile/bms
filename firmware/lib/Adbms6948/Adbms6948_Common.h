/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Common.h
* @brief  : Type and domain declarations used in the upper layer of the
			Adbms6948 module.
* @version: $Revision: 519 $
* @date   : $Date: 2020-10-13 16:59:52 +0530 (Tue, 13 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_ADBMS6948_DRIVER_INTERNAL_DATA	ADBMS6948 Driver Internal Data Types
* @{
*/

#ifndef ADBMS6948_COMMON_H
#define ADBMS6948_COMMON_H


/*============= I N C L U D E S =============*/
#include "Adbms6948_MemMap.h"
#include "Adbms6948_ExecCmd.h"
#include "Adbms6948_Pec.h"
#include "adi_bms_platform.h"
/*============================== D E F I N E S ======================================*/
/*============== CONFIG, CELL BALANCING, PROCESSING and OTHER SERIVES ===============*/
/* Driver Version Information */
/** Vendor ID */
#define ADBMS6948_VENDOR_ID                     ADBMS6948_VENDOR_ID_CFG

/** Module ID */
#define ADBMS6948_MODULE_ID                     ADBMS6948_MODULE_ID_CFG

/** Instance ID */
#define ADBMS6948_INSTANCE_ID                   ADBMS6948_INSTANCE_ID_CFG
/****************************** SERVICE IDs **********************************/


/*************************** DEVELOPMENT ERROR CODES *************************/

/** API service called with NULL pointer as parameter */
#define ADBMS6948_E_PARAM_POINTER     ((uint8_t)0x01u)

/** API service called with module in an unsuitable state */
#define ADBMS6948_E_MODULESTATE       ((uint8_t)0x02u)

/** API service called with invalid configuration */
#define ADBMS6948_E_INVALID_CONFIG    ((uint8_t)0x03u)

/** API service called with invalid parameter */
#define ADBMS6948_E_INVALID_PARAM     ((uint8_t)0x04u)

/** API service called when chain is busy */
#define ADBMS6948_E_CHAIN_BUSY        ((uint8_t)0x05u)

/** No. of register in a group */
#define ADBMS6948_MAX_NO_OF_CELLS_PER_DEVICE	((uint8_t)16u)

/** No. of register in a group */
#define ADBMS6948_REG_GRP_LEN		  			((uint8_t)0x06u)

/** PEC length */
#define ADBMS6948_PEC_LEN						((uint8_t)0x02u)

/** Register Data length with PEC per device  */
#define ADBMS6948_REG_DATA_LEN_WITH_PEC         ((uint8_t)0x08u)

/** Register Data length without PEC per device  */
#define ADBMS6948_REG_DATA_LEN_WITHOUT_PEC      ((uint8_t)0x06u)

/** Register Data length with PEC  */
#define ADBMS6948_CMD_DATA_LEN		  			((uint8_t)0x04u)

/** To read one register group */
#define ADBMS6948_NUMOF_REGGRP_ONE				((uint8_t)0x01u)

/** To read one register group */
#define ADBMS6948_NUMOF_REGGRP_TWO				((uint8_t)0x02u)

/** To read one register group */
#define ADBMS6948_NUMOF_REGGRP_THREE			((uint8_t)0x03u)

/** To read one register group */
#define ADBMS6948_NUMOF_REGGRP_FOUR				((uint8_t)0x04u)

/** To read one register group */
#define ADBMS6948_NUMOF_REGGRP_FIVE				((uint8_t)0x05u)

/** To read one register group */
#define ADBMS6948_NUMOF_REGGRP_SIX				((uint8_t)0x06u)

/** Number of register groups for RDCIV command */
#define ADBMS6948_NUMOF_REGGRP_RDCIV			((uint8_t)(0x08u))

/** Command code to snapshot  */
#define ADBMS6948_CMD_SNAP			((uint16_t)0x002Du)
/** Command code to release snapshot  */
#define ADBMS6948_CMD_UNSNAP		((uint16_t)0x002Fu)

/** Invalid command code */
#define ADBMS6948_CMD_INVALID		((uint16_t)0x0000u)

/** BMS device Wake-Up time from sleep state*/
#define ADBMS6948_TIME_TWAKE     (600u)

/** BMS device Wake-Up time from standby state*/
#define ADBMS6948_TIME_TREADY    (10U)

/** Calibration time*/
#define ADBMS6948_TIME_CALIBRATION    (200000u)

/**Number of bytes that needs to be send after STCOMM command */
#define ADBMS6948_I2C_STCOMM_BYTES    (9u)


#define ADBMS6948_I2C_STOP_COND    ((uint8_t)0x01u)

/*============= D A T A T Y P E S =============*/
/*! \struct Adbms6948_CallbackType
    structure to specify callback details which needs to be send to the other drivers
*/
typedef struct
{
	/*! Number of 4bytes commands in the nCmdList*/
	uint16_t  		nNoOfCmdList;

	/*! Number of devices for CC/PEC error list */
	uint8_t  		nNoOfDevices;

	/*! No. of write commands e.g. WRCFGA, WRCOMM, STCOMM*/
	uint16_t  		nNoofWrCmd;

	/*! List of all the 4bytes commands */
	uint16_t 	   *pCmdList;

	/*! Received CC for all devices in chain*/
	uint8_t        *pRecCmdCC;

	/*! Pec error counts per device*/
	boolean 	   *pPecError;
}Adbms6948_CallbackType;


/*======= P U B L I C P R O T O T Y P E S ========*/

/** Pointer to global configuration */
extern const Adbms6948_CfgType *  Adbms6948_pConfig;

/** This holds the address to daisy chain configuration arrays for all chains */
extern const Adbms6948_DaisyChainCfgType *  Adbms6948_pDaisyChainCfgInput;

/** Module State Information */
extern Adbms6948_ChainStateInfoType Adbms6948_aoChainStateInfo[ADBMS6948_NO_OF_DAISY_CHAIN];

/** Variable to store module state */
extern Adbms6948_StateType Adbms6948_eState;

void Adbms6948_Cmn_WakeUpBmsIC
(
    uint8_t  knChainID
);

void Adbms6948_Cmn_ReportRuntimeErr
(
	uint16_t   ErrorId,
	uint8_t    Status
);

void Adbms6948_Cmn_ReportDevErr
(
	uint16_t   ModId,
	uint8_t   InstId,
	uint8_t   ApiId,
	uint8_t   ErrCode
);

boolean Adbms6948_Cmn_IsError
(
    uint8_t  knChainID
);

boolean  Adbms6948_Cmn_ValidatePEC
(
  uint8_t *pDataBuf,
  uint8_t  nLen,
  uint8_t  nDeviceId,
  uint8_t  knChainID
);

Adbms6948_ReturnType Adbms6948_Cmn_VerifyCmdCnt
(
	boolean 	bWithCfgRead,
    uint8_t  	knChainID
);

Adbms6948_ReturnType Adbms6948_Cmn_InitChain
(
	uint8_t  nChainIndex
);

Adbms6948_ReturnType  Adbms6948_Cmn_UpdateConfigPerDevice
(
	Adbms6948_ConfigGroupSelection  	eCfgRegGrpSel,
	uint8_t 							nRegId,
	uint8_t 							nMask,
	uint8_t 						   *pData,
	const uint8_t  						knChainID
);

Adbms6948_ReturnType  Adbms6948_Cmn_UpdateConfig16
(
	Adbms6948_ConfigGroupSelection  	eCfgRegGrpSel,
	uint8_t 							nRegId,
	uint8_t 							aMask[],
	uint8_t 						   *pData,
	const uint8_t  						knChainID
);

Adbms6948_ReturnType  Adbms6948_Cmn_PwmDischarge(
uint8_t 		*pPwmCfgA,
uint8_t 		*pPwmCfgB,
const uint8_t  	 knChainID
);

Adbms6948_ReturnType  Adbms6948_Cmn_ReadDevRevCode
(
	uint8_t 		*pRevCode,
	const uint8_t  	 knChainID
);

Adbms6948_ReturnType Adbms6948_Cmn_SoftReset
(
	const uint8_t 			knChainID
);

void  Adbms6948_Cmn_I2CWrSendRegGrpData
(
Adbms6948_I2CWriteCfgType 		*poWriteCfg,
Adbms6948_I2CIdxData 			*poIdxData,
boolean   						 bFirstWrite,
boolean   						 bReadReq,
const uint8_t  					 knChainID
);

void  Adbms6948_Cmn_I2CRdSendRegGrpData
(
uint8_t 		*pnBytesRead,
uint16_t 		*pnRdLen,
const uint8_t  	 knChainID
);

Adbms6948_ReturnType  Adbms6948_Cmn_I2CGetRdData
(
Adbms6948_I2CReadCfgType 	*poReadCfg,
uint8_t 					*pnBytesWrOrRd,
uint16_t 					*pnByteIdx,
uint8_t 					*pnWrOrRdStat,
const uint8_t   			 knChainID
);

Adbms6948_ReturnType Adbms6948_Cmn_UpdateChainStatus
(
    Adbms6948_ChainStateType    eReqSt,
    const uint8_t           	knChainID
);

void Adbms6948_Cmn_EnterCriticalSection
(
void
);

void Adbms6948_Cmn_ExitCriticalSection
(
void
);

void Adbms6948_Cmn_Memset
(
	uint8_t 	*pAddr,
	uint8_t   	 nVal,
	uint32_t  	 nSize
);

void Adbms6948_Cmn_Memcpy
(
	uint8_t 	*pDstAddr,
	uint8_t 	*pSrcAddr,
	uint32_t  	 nSize
);

Adbms6948_ReturnType  Adbms6948_Cmn_ReadRegGroup
(
	uint16_t 		*pnCmdLst,
	uint8_t 		 pRxBuff[][ADBMS6948_MAX_FRAME_SIZE],
	uint8_t 		 nCmdLstLen,
	const uint8_t  	 knChainID
);

Adbms6948_ReturnType  Adbms6948_Cmn_ReadAllRegGroups
(
    uint16_t        nCmd,
    uint8_t         *pRxBuf,
	uint8_t         nDataLenBytes,
    const uint8_t   knChainID
);

void  Adbms6948_Cmn_WriteRegGroup
(
	uint16_t 			*pnCmdLst,
	uint8_t				nDataLen,
	uint8_t 			pTxBuff[][ADBMS6948_MAX_FRAME_SIZE],
	const uint8_t  		knChainID
);

boolean  Adbms6948_Cmn_IsInvalidSnapSelType
(
    Adbms6948_SnapSelType      eSnapSel
);

void Adbms6948_Cmn_SendCommandOnly
(
uint16_t 		nCommand,
boolean 		bIncrementCmdCount,
const uint8_t 	knChainID
);

void  Adbms6948_Cmn_ExecuteCmdRD
(
	uint16_t 		*pnCmdLst,
	uint8_t     	 pRxBuf[][ADBMS6948_MAX_FRAME_SIZE],
	uint32_t  		 nDataCfg,
	boolean  		 bIsPollCmd,
	const uint8_t    knChainID
);

void Adbms6948_Cmn_ExecuteCmdWR
(
    uint16_t		nCommand,
    uint8_t			*pTxBuf,
    const uint8_t	knChainID
);

void Adbms6948_Cmn_ExecCmdWithCPECBitFlip
(
	uint16_t		nCommand,
	uint8_t			nPECBitToFlip,
	const uint8_t	knChainID
);

void Adbms6948_Cmn_ExecCmdWRWithDPECBitFLip
(
    uint16_t		nCommand,
    uint8_t			*pTxBuf,
	uint8_t			nDataPECBitToFlip,
    const uint8_t	knChainID
);

#endif /* ADBMS6948_COMMON_H */

/** @}*/ /* ADI_ADBMS6948_DRIVER_INTERNAL_DATA */

/*
*
* EOF
*
*/

