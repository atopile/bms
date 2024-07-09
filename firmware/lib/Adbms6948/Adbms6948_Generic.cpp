/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Generic.cpp
* @brief  : Adbms6948 driver internal source file, this file contains the internal
* interface which will support the device generic services
* @version: $Revision: 553 $
* @date   : $Date: 2020-10-28 16:28:17 +0530 (Wed, 28 Oct 2020) $
* Developed by: Automotive Software and Systems team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_INTERNAL_INTERFACE	ADBMS6948 Driver Internal Interface
* @{
*/

#ifndef ADI_DISABLE_DOCUMENTATION
/*============= I N C L U D E S =============*/

#include "Adbms6948_Generic.h"

/*============ Static Function Prototypes ============*/

/*============= D A T A =============*/

/* Const 16 section start */
ADBMS6948_DRV_CONST_DATA_16_START

/* List of all the write commands for ADBMS6948 device */
const uint16_t  Adbms6948_CmdList[] = {
ADBMS6948_CMD_SNAP,
ADBMS6948_CMD_UNSNAP,
ADBMS6948_CMD_ADCV,
ADBMS6948_CMD_ADSV,
ADBMS6948_CMD_ADAX,
ADBMS6948_CMD_ADAX2,
ADBMS6948_CMD_CLRCELL,
ADBMS6948_CMD_CLRFC,
ADBMS6948_CMD_CLRAUX,
ADBMS6948_CMD_CLRSPIN,
ADBMS6948_CMD_CLRFLAG,
ADBMS6948_CMD_CLOVUV,
ADBMS6948_CMD_PLADC,
ADBMS6948_CMD_PLCADC,
ADBMS6948_CMD_PLSADC,
ADBMS6948_CMD_PLAUX1,
ADBMS6948_CMD_PLAUX2,
ADBMS6948_CMD_WRCOMM,
ADBMS6948_CMD_STCOMM,
ADBMS6948_CMD_MUTE,
ADBMS6948_CMD_UNMUTE,
ADBMS6948_CMD_SRST,
ADBMS6948_CMD_UNKNOWN
};

/* Const 16 section stop */
ADBMS6948_DRV_CONST_DATA_16_STOP



/*============= C O D E =============*/
/* Start of code section */

/* Code section start */
ADBMS6948_DRV_CODE_START

#endif /* ADI_DISABLE_DOCUMENTATION */
/*
 * Function Definition section
*/

/*! \addtogroup ADI_INTERNAL_GENERIC	ADBMS6948 Generic Interface
* @{
*/
/*****************************************************************************/
/*************************** GENERIC SERVICES ********************************/
/*****************************************************************************/

/*****************************************************************************/
 /*!
    @brief      This interface is used to get the current driver version information

    @param	[out]   pDriverVersion	Place holder to report the software
									version information.
									@range: NA
									@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadDriverVersion
(
	Adbms6948_VersionInfoType *pDriverVersion
)
{
    Adbms6948_ReturnType      nRet = E_NOT_OK;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
        ADBMS6948_GETVERSIONINFO_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if (NULL_PTR == pDriverVersion)
    {
        /* Null pointer error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
        ADBMS6948_GETVERSIONINFO_ID, ADBMS6948_E_PARAM_POINTER);
        #endif
    }
    else
    {
        /* Populate the software version information. */
        pDriverVersion->Adbms6948_nModuleID = (uint16_t)ADBMS6948_MODULE_ID_CFG;
        pDriverVersion->Adbms6948_nVendorID = (uint16_t)ADBMS6948_VENDOR_ID_CFG;
        pDriverVersion->Adbms6948_nSWMajorVersion = (uint8_t)ADBMS6948_SW_MAJOR_VERSION_CFG;
        pDriverVersion->Adbms6948_nSWMinorVersion = (uint8_t)ADBMS6948_SW_MINOR_VERSION_CFG;
        pDriverVersion->Adbms6948_nSWPatchVersion = (uint8_t)ADBMS6948_SW_PATCH_VERSION_CFG;

        nRet = E_OK;
    }
    return (nRet);
}

/*****************************************************************************/
 /*!
    @brief      This interface evaluates the Over voltage against the measured cell
				data and provide the evaluation result \n

	@param	[in]   pCellData	User should pass the cell data for each ADBMS6948
								device for the evaluation.
								@range: NA
								@resolution: NA

	@param	[in]   pOVThreshold	User should pass the device Over voltage threshold value
								for each ADBMS6948 device for the evaluation.
								@range: NA
								@resolution: NA

    @param	[out]   pEvalResult	Place holder to keep the OV status for all cells
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
Adbms6948_ReturnType Adbms6948_Int_OverVoltEval
(
uint16_t *   			  		 pCellData,
Adbms6948_OVThresCfgType *  	 pOVThreshold,
Adbms6948_OVEvalType 			*pEvalResult,
const uint8_t 					 knChainID
)
{
	Adbms6948_ReturnType      nRet = E_NOT_OK;
	uint8_t      nUserBuff = 0u;
	uint8_t 	nCellIdx,nNoOfDevInChain,nDevIdx,nNoOfCellPerDevice;
	uint16_t   nNextDev=0u;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
        ADBMS6948_OVERVOLTEVAL_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
	else if ((NULL_PTR == pOVThreshold) || (NULL_PTR == pEvalResult) ||
			(ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
    {
        /* Null pointer error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
        ADBMS6948_OVERVOLTEVAL_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
        ADBMS6948_OVERVOLTEVAL_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
	else
	{
		nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
		for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
		{
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				nNoOfCellPerDevice =
				Adbms6948_pDaisyChainCfgInput[knChainID].Adbms6948_pNoOfCellsPerDevice[nDevIdx];
				for (nCellIdx = 0u; nCellIdx < nNoOfCellPerDevice;nCellIdx++)
				{
					if (pCellData[nCellIdx + nNextDev] > pOVThreshold[nUserBuff])
					{
						pEvalResult[nUserBuff] |= (Adbms6948_OVEvalType)((uint16_t)1u<<nCellIdx);
					}
				}
				nUserBuff++;
				nNextDev += nCellIdx;
			}
		}
		/* Set chain state to IDLE. */
		nRet = Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);

	}
return nRet;
}

/*****************************************************************************/
 /*!
    @brief      This interface evaluates the Under voltage against the measured cell
				data and provide the evaluation result \n

	@param	[in]   pCellData	User should pass the cell data for each ADBMS6948
								device for the evaluation.
								@range: NA
								@resolution: NA

	@param	[in]   pUVThreshold	User should pass the device Under voltage threshold value
								for each ADBMS6948 device for the evaluation.
								@range: NA
								@resolution: NA

    @param	[out]   pEvalResult	Place holder to keep the UV status for all cells
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
Adbms6948_ReturnType Adbms6948_Int_UnderVoltEval
(
uint16_t 						*pCellData,
Adbms6948_UVThresCfgType 		*pUVThreshold,
Adbms6948_UVEvalType 			*pEvalResult,
const uint8_t 					 knChainID
)
{
	Adbms6948_ReturnType      nRet = E_NOT_OK;
	uint8_t      nUserBuff = 0u;
	uint8_t 	nCellIdx,nNoOfDevInChain,nDevIdx,nNoOfCellPerDevice;
	uint16_t   nNextDev=0u;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
        ADBMS6948_UNDERVOLTEVAL_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
	else if ((NULL_PTR == pUVThreshold)|| (NULL_PTR == pEvalResult) ||
			(ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
    {
        /* Null pointer error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
        ADBMS6948_UNDERVOLTEVAL_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
        ADBMS6948_UNDERVOLTEVAL_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
	else
	{
		nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
		for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
		{
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				nNoOfCellPerDevice =
				Adbms6948_pDaisyChainCfgInput[knChainID].Adbms6948_pNoOfCellsPerDevice[nDevIdx];
				for (nCellIdx = 0u; nCellIdx < nNoOfCellPerDevice;nCellIdx++)
				{
					if (pCellData[nCellIdx + nNextDev] < pUVThreshold[nUserBuff])
					{
						pEvalResult[nUserBuff] |= (Adbms6948_UVEvalType)((uint16_t)1u<<nCellIdx);
					}
				}
				nUserBuff++;
				nNextDev += nCellIdx;
			}
		}
		/* Set chain state to IDLE. */
		nRet = Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return nRet;
}

/*!
    @brief  	This API gets the counts for the errors detected in the data
                communication with the ADBMS6948 device.

    @param [out]  pErrCnt        Place holder to report the error counts.
                                User must pass the buffer of size equal to
                                or greater than the total number of devices
                                in the chain.

                                @range: NA
                                @resolution: NA

    @param  [in]  knChainID     Specifies ID of the daisy chain on
                                which the operation is to be performed.
                                @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                @resolution: NA

    @return            E_OK:         Success \n
                    E_NOT_OK:     Failure \n
 */

Adbms6948_ReturnType  Adbms6948_Int_ReadErrorCounts
(
Adbms6948_ErrorCounts 	*pErrCnt,
const uint8_t            knChainID
)
{

    Adbms6948_ReturnType nRet = E_NOT_OK;
    uint8_t nDevIdx, nNoOfDevInChain,nUserBuff=0u;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
    	Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
                ADBMS6948_READERRCNT_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if ((NULL_PTR == pErrCnt) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
    {
        /* Null pointer error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
    	Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
                ADBMS6948_READERRCNT_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
    	Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
                ADBMS6948_READERRCNT_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        /* Read Error counts. */
        nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
        for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
        {
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				pErrCnt[nUserBuff].Adbms6948_nCmdCntErrs =
                    Adbms6948_aoChainStateInfo[knChainID].nCmdCntError[nDevIdx];
				pErrCnt[nUserBuff].Adbms6948_nPECErrs =
                    Adbms6948_aoChainStateInfo[knChainID].nPECError[nDevIdx];
				nUserBuff++;
			}
		}
        /* Set chain state to IDLE. */
        nRet = Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
    }
    return (nRet);
}

/*!
    @brief  	This API clears the counts for the errors detected in the data
                communication with the ADBMS6948 device.

    @param  [in]  knChainID  Specifies ID of the daisy chain on
                                which the operation is to be performed.
                                @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                @resolution: NA

    @return            E_OK:         Success \n
                    E_NOT_OK:     Failure \n
*/
Adbms6948_ReturnType  Adbms6948_Int_ClearErrorCounts
(
const uint8_t         knChainID
)
{
    Adbms6948_ReturnType nRet = E_NOT_OK;
    uint8_t  nDevIdx, nNoOfDevInChain;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
    	Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
                ADBMS6948_CLRERRCNT_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
    {
        /* Invalid ADC operation mode and/or Invalid chain ID. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
    	Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
                ADBMS6948_CLRERRCNT_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
    	Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
                ADBMS6948_CLRERRCNT_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        /* Clear the error counter values maintained by the driver. */
        nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
        for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
        {
            Adbms6948_aoChainStateInfo[knChainID].nCmdCntError[nDevIdx] = 0u;
            Adbms6948_aoChainStateInfo[knChainID].nPECError[nDevIdx] = 0u;
        }


        /* Clear the command counter. */
        Adbms6948_Cmd_ExecCmd(ADBMS6948_CMD_RSTCC, FALSE, knChainID);

        /* Set chain state to IDLE. */
        nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
      }
    return (nRet);
}
/*! @}*/

/*! @}*/
#ifndef ADI_DISABLE_DOCUMENTATION

/* End of code section */
/* Code section stop */
ADBMS6948_DRV_CODE_STOP

#endif /* ADI_DISABLE_DOCUMENTATION */

/*
*
* EOF
*
*/

