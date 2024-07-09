/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_OtherMonitor.cpp
* @brief  : Adbms6948 driver internal source file, this file contains the internal
* interface which will support the device other monitoring services
* @version: $Revision: 528 $
* @date   : $Date: 2020-10-13 19:41:33 +0530 (Tue, 13 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_INTERNAL_INTERFACE	ADBMS6948 Driver Internal Interface
* @{
*/

#ifndef ADI_DISABLE_DOCUMENTATION
/*============= I N C L U D E S =============*/

#include "Adbms6948_OtherMonitor.h"

/*============= D A T A =============*/

/*============ Static Function Prototypes ============*/
static Adbms6948_ReturnType  Adbms6948_lI2CWrData(
Adbms6948_I2CWriteCfgType 	*poWriteCfg,
uint8_t 					*pnWrStatus,
boolean   					 bReadReq,
uint16_t 					*pNoOfRegGrp,
const uint8_t   			 knChainID
);

static Adbms6948_ReturnType  Adbms6948_lI2CRdData
(
Adbms6948_I2CReadCfgType 	*poReadCfg,
uint8_t 					*pnI2CAddWrStatus,
uint16_t 					*pNoOfRegGrp,
const uint8_t   			 knChainID
);

static uint16_t  Adbms6948_lI2CGetWrComCntWr
(
Adbms6948_I2CWriteCfgType 	*poWriteCfg,
uint16_t 					*pnWrLen,
const uint8_t   			 knChainID
);

static uint16_t  Adbms6948_lI2CGetWrComCntRd
(
Adbms6948_I2CReadCfgType 	*poReadCfg,
uint16_t 					*pnRdLen,
const uint8_t   			 knChainID
);

static uint16_t  Adbms6948_lI2CGetWrComCnt
(
uint16_t   nTotalWrLen
);

static Adbms6948_ReturnType Adbms6948_lAlwaysOnMemWrite
(
uint8_t 					*panData,
const uint8_t				knChainID
);

static Adbms6948_ReturnType Adbms6948_lAlwaysOnMemRead
(
uint8_t 					*panData,
const uint8_t				knChainID
);

/*============= C O D E =============*/
/* Start of code section */

/* Code section start */
ADBMS6948_DRV_CODE_START

#endif /* ADI_DISABLE_DOCUMENTATION */
/*
 * Function Definition section
*/

/*! \addtogroup ADI_INTERNAL_OTHERMONITOR	ADBMS6948 Other Monitor Interface
* @{
*/
/*****************************************************************************/
/***************************** OTHER MONITOR SERVICES ************************/
/*****************************************************************************/


/*****************************************************************************/
 /*!
    @brief      This interface read the device revision code from all the devices
				in the chain and reports it to user.

	@param	[out]	pRevCode	Place holder to report the device revision
								code. The user must pass a buffer of size
								equal to or greater than number of devices
								in the chain. \n

								EXAMPLE: If there are 4 ADBMS6948 devices
								in the chain, then the revision codes are
								arranged as shown below: \n
								[REV(1),REV(2),REV(3),REV(4)]
								@range: NA
								@resolution: NA


    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadDevRevCode
(
uint8_t 		*pRevCode,
const uint8_t    knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_READDEVREVCODE_ID,ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR == pRevCode) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Null pointer error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_READDEVREVCODE_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_READDEVREVCODE_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read device revision code. */
		nRet = Adbms6948_Cmn_ReadDevRevCode(pRevCode, knChainID);

		if (E_OK == nRet)
		{
			/* Perform platform error check. */
			if (TRUE == Adbms6948_Cmn_IsError(knChainID))
			{
				/* Platform error. */
				nRet = E_NOT_OK;
				Adbms6948_Cmn_ReportRuntimeErr(Adbms6948_pConfig->Adbms6948_pRunTimeErrCfg->Adbms6948_E_Platform_Error.nErrID,
						ADBMS6948_RUNTIME_E_STATUS_FAILED);
			}
		}

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return (nRet);
}

/*****************************************************************************/
 /*!
    @brief      This interface wakes up the BMS IC.

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_WakeUpBmsIC
(
const uint8_t 		knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;

	if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
	{
		/* Invalid chain ID. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_UPDATEBMSWAKEUPCTRL_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else
	{
		/* Wake up BMS IC */
		Adbms6948_Cmn_WakeUpBmsIC(knChainID);
		nRet = E_OK;
	}

return nRet;
}

/*****************************************************************************/
 /*!
    @brief      This interface send the soft reset command to all the devices in \n
				the chain. \n

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_SendSoftReset
(
const uint8_t  						knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;


	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_SOFTRESET_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
	{
		/* Invalid chain ID. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_SOFTRESET_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
				ADBMS6948_SOFTRESET_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		if (TRUE == Adbms6948_pConfig->Adbms6948_pDaisyChainCfg[knChainID].Adbms6948_bEnableInit)
		{
			/* Issue soft reset.*/
			nRet = Adbms6948_Cmn_SoftReset(knChainID);
		}

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return nRet;
}

/*****************************************************************************/
 /*!
    @brief      This interface is used to read the 48bit serial ID for the devices

	@param	[out]	pSerialidCode	Place holder to report the device serial
									id. The user must pass a buffer of size
									equal to or greater than 6*number of devices
									in the chain. \n

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadSerialID
(
uint8_t 		*pSerialidCode,
const uint8_t 	 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint8_t  aSerialId[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)] = {0};
	uint8_t  nDeviceIndex,nUserBuffIndex=0,nDevDataIdx,nRegGrpDataLen,nDevStartIdx;
	uint16_t  anCmdList[ADBMS6948_NUMOF_REGGRP_ONE];

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_READDEVSERID_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR == pSerialidCode) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Null pointer error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_READDEVSERID_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_READDEVSERID_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read serial ID code. */
		anCmdList[0] = ADBMS6948_CMD_RDSID;
		nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, aSerialId,1u, knChainID);

		if (E_OK == nRet)
		{
			/* Perform platform error check. */
			if (TRUE == Adbms6948_Cmn_IsError(knChainID))
			{
				/* Platform error. */
				nRet = E_NOT_OK;
				Adbms6948_Cmn_ReportRuntimeErr(Adbms6948_pConfig->Adbms6948_pRunTimeErrCfg->Adbms6948_E_Platform_Error.nErrID,
						ADBMS6948_RUNTIME_E_STATUS_FAILED);
			}
			else
			{
				for (nDeviceIndex = 0u; nDeviceIndex < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDeviceIndex++)
				{
					if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDeviceIndex])
					{
						nDevStartIdx = (uint8_t)((uint8_t)ADBMS6948_CMD_DATA_LEN + (nDeviceIndex * (uint8_t)ADBMS6948_REG_DATA_LEN_WITH_PEC));
						for(nRegGrpDataLen = 0; nRegGrpDataLen < ADBMS6948_REG_GRP_LEN; nRegGrpDataLen++)
						{
							/* Pass the device revision code to user buffer. */
							nDevDataIdx = nDevStartIdx + nRegGrpDataLen;
							pSerialidCode[nUserBuffIndex] = (uint8_t)aSerialId[0u][nDevDataIdx];
							nUserBuffIndex++;
						}
					}
				}
			}
		}

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return nRet;
}

/*****************************************************************************/
 /*!
    @brief          This interface is used to write 6 bytes of data to the
					Always On memory of BMS IC. This data does not get cleared
					even when BMS IC goes into sleep state.

    @param	[in]	panData		Pointer to 6 bytes of data to write.
								@range: NA
								@resolution: NA

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_AlwaysOnMemWrite
(
uint8_t 					*panData,
const uint8_t				knChainID
)
{
	Adbms6948_ReturnType   nRet = E_NOT_OK;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_AOMEMWRITE_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((panData == NULL_PTR) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* NULL pointer passed */
		#if ( ADBMS6948_DEVELOPMENT_MODE_EN == TRUE )
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_AOMEMWRITE_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_AOMEMWRITE_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		nRet = Adbms6948_lAlwaysOnMemWrite(panData, knChainID);

		if (E_OK == nRet)
		{
			/* Perform platform error check. */
			if (TRUE == Adbms6948_Cmn_IsError(knChainID))
			{
				/* Platform error. */
				nRet = E_NOT_OK;
				Adbms6948_Cmn_ReportRuntimeErr(Adbms6948_pConfig->Adbms6948_pRunTimeErrCfg->Adbms6948_E_Platform_Error.nErrID, ADBMS6948_RUNTIME_E_STATUS_FAILED);
			}
		}

		/*Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return nRet;
}

/*****************************************************************************/
 /*!
    @brief          This API is used to read 6 bytes of data from the Always On
					Memory of BMS IC.

    @param	[out]	panData		Placeholder for storing 6 bytes of data read
								from IC.
								@range: NA
								@resolution: NA

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_AlwaysOnMemRead
(
uint8_t 					*panData,
const uint8_t				knChainID
)
{
	Adbms6948_ReturnType   nRet = E_NOT_OK;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_AOMEMREAD_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((panData == NULL_PTR) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* NULL pointer passed */
		#if ( ADBMS6948_DEVELOPMENT_MODE_EN == TRUE )
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_AOMEMREAD_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_AOMEMREAD_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		nRet = Adbms6948_lAlwaysOnMemRead(panData, knChainID);

		if (E_OK == nRet)
		{
			/* Perform platform error check. */
			if (TRUE == Adbms6948_Cmn_IsError(knChainID))
			{
				/* Platform error. */
				nRet = E_NOT_OK;
				Adbms6948_Cmn_ReportRuntimeErr(Adbms6948_pConfig->Adbms6948_pRunTimeErrCfg->Adbms6948_E_Platform_Error.nErrID, ADBMS6948_RUNTIME_E_STATUS_FAILED);
			}
		}

		/*Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return nRet;
}

/*!
	@brief			This local function writes the data to "Always On" memory in
					BMS IC.

	@param	[in]	panData		Pointer to the data buffer
	@param  [in]	knChainID	The daisy chain ID to perform the operation.
	@return			nRet		E_OK: Success.
								E_NOT_OK: Failure.
 */
static Adbms6948_ReturnType Adbms6948_lAlwaysOnMemWrite
(
uint8_t 					*panData,
const uint8_t				knChainID
)
{
	Adbms6948_ReturnType nRet;

	/*Unlock the "Always ON" flops*/
	Adbms6948_Cmd_ExecCmd(ADBMS6948_CMD_ULAO, TRUE, knChainID);

    /* Write the data to the "Always On" memory*/
    Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRAO, &panData[0], knChainID);

    /* Verify the command counter as the ULAO and WRAO commands are expected to
	increase command counter value*/
    nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);

	return nRet;
}

/*!
	@brief			This local function reads the data from the "Always On"
					memory in BMS IC and stores in the data buffer.

	@param	[out]	panData		Pointer to the data buffer
	@param  [in]	knChainID	The daisy chain ID to perform the operation.
	@return			nRet		E_OK: Success.
								E_NOT_OK: Failure.
 */
static Adbms6948_ReturnType Adbms6948_lAlwaysOnMemRead
(
uint8_t 					*panData,
const uint8_t				knChainID
)
{
	Adbms6948_ReturnType nRet;

    /*Read "Always On" memory and store into data buffer*/
    nRet = Adbms6948_Cmn_ReadAllRegGroups(ADBMS6948_CMD_RDAO, panData, ADBMS6948_REG_DATA_LEN_WITH_PEC, knChainID);

	return nRet;
}

/*
*
* EOF
*
*/

