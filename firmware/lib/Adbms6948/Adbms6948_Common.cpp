/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Common.cpp
* @brief  : Adbms6948 driver internal common file
* @version: $Revision: 532 $
* @date   : $Date: 2020-10-14 15:50:36 +0530 (Wed, 14 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_INTERNAL_INTERFACE	ADBMS6948 Driver Internal Interface
* @{
*/

#ifndef ADI_DISABLE_DOCUMENTATION
/*============= I N C L U D E S =============*/
#include "Adbms6948.h"
#include "Adbms6948_Common.h"

/*============= D A T A =============*/
/* Initialized Data section start */
ADBMS6948_DRV_INIT_DATA_START

/** Variable to store module state */
Adbms6948_StateType Adbms6948_eState = ADBMS6948_ST_UNINIT;

/* Initialized Data section stop */
ADBMS6948_DRV_INIT_DATA_STOP

/* Un-Initialized Data section start */
ADBMS6948_DRV_UNINIT_DATA_START

Adbms6948_ChainStateInfoType Adbms6948_aoChainStateInfo[ADBMS6948_NO_OF_DAISY_CHAIN];

/* Un-Initialized Data section start */
ADBMS6948_DRV_UNINIT_DATA_STOP


/*======Static Prototype==============*/
static void  Adbms6948_lI2CUpdRdBuf
(
Adbms6948_I2CReadCfgType 	*poReadCfg,
uint8_t   					 anRdRegData[][ADBMS6948_MAX_FRAME_SIZE],
uint8_t 					*pnBytesWrOrRd,
uint16_t 					*pnByteIdx,
uint8_t   					 nDevIdx
);
/*TODO: Remove this code if not to be used*/
#if 0
static Adbms6948_ReturnType  Adbms6948_lVerifyCmdCntInit
(
    uint8_t  knChainID
);
#endif
static void  Adbms6948_lInitMaxCellGrps
(
const uint8_t   knChainID
);

static void  Adbms6948_lI2CPrepWrDataWr
(
Adbms6948_I2CWriteCfgType 	*poWriteCfg,
uint8_t  					 anWrRegData[][ADBMS6948_REG_GRP_LEN],
Adbms6948_I2CIdxData 		*poIdxData,
boolean   					 bFirstWrite,
const uint8_t  				 knChainID
);

static void  Adbms6948_lI2CPrepWrDataRd
(
Adbms6948_I2CWriteCfgType 	*poWriteCfg,
uint8_t   					 anWrRegData[][ADBMS6948_REG_GRP_LEN],
Adbms6948_I2CIdxData 		*poIdxData,
const uint8_t  				 knChainID
);

static void  Adbms6948_lI2CSetNoTx
(
uint8_t   nDevIdx,
uint8_t   anWrRegData[][ADBMS6948_REG_GRP_LEN]
);

static void  Adbms6948_lPrepConfigBuff(
uint16_t 	 nCmd,
uint8_t 	 aRdCfgData[][ADBMS6948_MAX_FRAME_SIZE],
uint8_t 	*pCfgData,
uint8_t 	*pCfgDataChain,
uint8_t  	 nChainIndex
);
static uint8_t Adbms6948_lI2CGetRdStat
(
uint8_t nRdBytes,
uint8_t anFCOMM[]
);

static uint8_t Adbms6948_lI2CGetWrStat
(
uint8_t nWrBytes,
uint8_t anFCOMM[]
);
/*============= C O D E =============*/
/* Start of code section */

/* Code section start */
ADBMS6948_DRV_CODE_START

/*
 * Function Definition section
*/

/*! \addtogroup ADI_INTERNAL_COMMON		ADBMS6948 Common Interface
* @{
*/
/*****************************************************************************/
/******************************* COMMON SERVICES *****************************/
/*****************************************************************************/
/*!
    @brief         This function is used to wake up the BMS IC.

	@param	[in]   knChainID	The Id of the daisy chain on which the operation
								is done.
								It is mapped to the array index of configuration
								in Adbms6948_DaisyChainCfgType member of
								\ref Adbms6948_CfgType.
								It starts with 0 for the first chain.

    @return        	E_OK    : Operation was successful \n
                    E_NOT_OK: Operation failed \n
 */
void Adbms6948_Cmn_WakeUpBmsIC
(
    uint8_t  knChainID
)
{
    uint8_t   nLen = 1u;
    uint8_t   nDummyByte = 0xFFu;

    /* Write a dummy byte. */
    Adbms6948_Cmd_SendDummyBytes(&nDummyByte, nLen, knChainID);
    if (TRUE == Adbms6948_aoChainStateInfo[knChainID].bFirstWakeup)
    {
        /* Delay to ensure Wakeup after initialization */
        ADI_PAL_TIMERDELAY(ADBMS6948_TIME_TWAKE, knChainID);
    }
    else
    {
        /* Delay to ensure Wakeup during initialization */
        ADI_PAL_TIMERDELAY(ADBMS6948_TIME_TREADY, knChainID);
    }
return;
}

/*!
    @brief         This function is used report the runtime errors

	@param	[in]   ErrorId	Error ID

	@param	[in]   Status	Status of event

    @return        None
 */
void Adbms6948_Cmn_ReportRuntimeErr
(
	uint16_t   ErrorId,
	uint8_t   Status
)
{
	ADI_PAL_REPORT_RUNTIME_ERROR(ErrorId, Status);
}

/*!
    @brief         This function is used report the platform errors

	@param	[in]   knChainID	Chain ID

    @return        TRUE: Platform error generated
             	   FALSE: Platform error not generated
 */
boolean Adbms6948_Cmn_IsError
(
    uint8_t  knChainID
)
{
	Adbms6948_ReturnType  nRet;
	nRet = ADI_PAL_ISERROR(knChainID);

	return nRet;
}

/*!
    @brief  This function is used to validate the PEC and command counter of the read data for a
    single device on the chain. The PEC is computed for the data and verified against that received
    from the device and the command counter value is compared against the value maintained by the
    driver for the specified device. It also reports the runtime errors in the case of PEC or
    command counter mismatch occurs during the validation.

    @param  [in]   pDataBuf  Pointer to the data buffer containing the data along with the PEC.

    @param  [in]   nLen  Length of the data buffer.

    @param  [in]   nDeviceId  Device number in chain.

    @param  [in]   knChainID  The Id of the daisy chain on which the operation is done.

    @return  TRUE: PEC valid
             FALSE: PEC invalid
*/
boolean  Adbms6948_Cmn_ValidatePEC
(
  uint8_t *pDataBuf,
  uint8_t  nLen,
  uint8_t  nDeviceId,
  uint8_t  knChainID
)
{
    uint16_t   nDataPec;
    uint16_t   nRcvdPec;
    uint8_t   nCmdCntRcvd;
    boolean  bIsPecValid = TRUE;

    nDataPec = Adbms6948_Pec10Calculate(&pDataBuf[0], TRUE, nLen - ADBMS6948_PEC_LEN);
    nRcvdPec = ((uint16_t)(((uint16_t)pDataBuf[nLen-2u]) << 8) + (uint16_t)pDataBuf[nLen-1u]);
    nCmdCntRcvd = (uint8_t)((nRcvdPec & (uint16_t)0xFC00) >> 10u);
    nRcvdPec = (uint16_t)(nRcvdPec & (uint16_t)0x03FF);

    /* Set the PEC error to FALSE in the state variable */
    Adbms6948_aoChainStateInfo[knChainID].aLastReadPecEr[nDeviceId] = (boolean)FALSE;
    if (nRcvdPec != nDataPec)
    {
    	if (Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDeviceId] == ADBMS6948_DEVICE)
    	{
    		bIsPecValid = FALSE;
    		Adbms6948_aoChainStateInfo[knChainID].nPECError[nDeviceId]++;
    		if (TRUE == Adbms6948_pConfig->Adbms6948_pRunTimeErrCfg->Adbms6948_E_Invalid_PEC.nState)
    		{
    			Adbms6948_Cmn_ReportRuntimeErr(
                    Adbms6948_pConfig->Adbms6948_pRunTimeErrCfg->Adbms6948_E_Invalid_PEC.nErrID,
					ADBMS6948_RUNTIME_E_STATUS_FAILED);
    		}
    	}
    	/* Set the PEC error to TRUE in the state variable */
    	Adbms6948_aoChainStateInfo[knChainID].aLastReadPecEr[nDeviceId] = (boolean)TRUE;
    }
    else
    {
    	if (Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDeviceId] == ADBMS6948_DEVICE)
    	{
			if (nCmdCntRcvd != Adbms6948_aoChainStateInfo[knChainID].nCmdCnt[nDeviceId])
			{
				bIsPecValid = FALSE;
				Adbms6948_aoChainStateInfo[knChainID].nCmdCntError[nDeviceId]++;
				/* Latch onto the command count set in the device */
				Adbms6948_aoChainStateInfo[knChainID].nCmdCnt[nDeviceId] = (uint8_t)nCmdCntRcvd;
				if (TRUE == Adbms6948_pConfig->Adbms6948_pRunTimeErrCfg->Adbms6948_E_CommandCount.nState)
				{
					Adbms6948_Cmn_ReportRuntimeErr(
						  Adbms6948_pConfig->Adbms6948_pRunTimeErrCfg->Adbms6948_E_CommandCount.nErrID,
						  ADBMS6948_RUNTIME_E_STATUS_FAILED);
				}
			}
    	}
    	Adbms6948_aoChainStateInfo[knChainID].aLastReadCCs[nDeviceId] = (uint8_t)nCmdCntRcvd;
	}
return (bIsPecValid);
}

/*!
    @brief  This function reads the configuration register A and verifies the command counter value
    read from the device against the value maintained by the driver.

    @param 	[in]  bWithCfgRead	To include configuration read.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  E_OK: Operation was successful
             E_NOT_OK: Operation failed
*/
Adbms6948_ReturnType  Adbms6948_Cmn_VerifyCmdCnt
(
	boolean 	bWithCfgRead,
    uint8_t  	knChainID
)
{
    Adbms6948_ReturnType  nRet = E_OK;

    if (TRUE == bWithCfgRead)
    {
		uint8_t  aRdCfgData[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
		uint16_t anCmdList[ADBMS6948_NUMOF_REGGRP_ONE];
		/* Read configuration register group A and verify the PEC and command counter. */
		anCmdList[0u] = ADBMS6948_CMD_RDCFGA;
		nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, aRdCfgData, ADBMS6948_NUMOF_REGGRP_ONE, knChainID);
    }

    return (nRet);
}

/*!
    @brief         This function is used to initialize the configurations of all devices in a chain.

    @param	[in]   nChainIndex	 		Index of the chain to be initialized

	@return        	E_OK    : Operation was successful \n
                    E_NOT_OK: Operation failed \n
 */
Adbms6948_ReturnType  Adbms6948_Cmn_InitChain
(
	uint8_t  nChainIndex
)
{
	uint8_t  nCfgDataChain[(ADBMS6948_REG_GRP_LEN) *
	ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];
	uint8_t  nCfgAData[ADBMS6948_REG_GRP_LEN] = {0};
	uint8_t  nCfgBData[ADBMS6948_REG_GRP_LEN] = {0};
	uint8_t  nCfgCData[ADBMS6948_REG_GRP_LEN] = {0};
	uint16_t  nUnderVoltCompVal,nOverVoltCompVal,nTempGpo;
	uint8_t   nTemp;
    uint8_t   nGpio11Sel;
    uint16_t  anCmdList[ADBMS6948_NUMOF_REGGRP_ONE];
	Adbms6948_ReturnType  nRet = E_OK;
	uint8_t  aRdCfgData[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN +
			(ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)] = {0u};

	if (TRUE == Adbms6948_pConfig->Adbms6948_pDaisyChainCfg[nChainIndex].Adbms6948_bEnableInit)
	{

		/* Reference ON/OFF */
		nTemp = (uint8_t)Adbms6948_pDaisyChainCfgInput[nChainIndex].Adbms6948_bEnableRefOn;
		nCfgAData[0] = (uint8_t)(nTemp << ADBMS6948_CFGAR0_POS_REFON);

		/*C and S comparison threshold voltage value */
		nTemp = (uint8_t)Adbms6948_pDaisyChainCfgInput[nChainIndex].Adbms6948_eCSThresholdVolt;
		nCfgAData[0] = (uint8_t)(nCfgAData[0] |
		((uint8_t)(((uint8_t)nTemp ) & (ADBMS6948_CFGAR0_MASK_CTH))));

		/*GPIO Pin control selection */
		nTempGpo = Adbms6948_pDaisyChainCfgInput[nChainIndex].Adbms6948_nGPIOPinCtrlValue;
		nCfgAData[3] = (uint8_t)(nTempGpo & ADBMS6948_CFGAR3_MASK_GPO);
		nCfgAData[4] = (uint8_t)(((nTempGpo >> 8u) & ADBMS6948_CFGAR4_MASK_GPO) << ADBMS6948_CFGAR4_POS_GPO);

		/*FC bit value */
		nTemp = (uint8_t)Adbms6948_pDaisyChainCfgInput[nChainIndex].Adbms6948_eIIRCornerFreq;
		nCfgAData[5] = (uint8_t)(((uint8_t)nTemp) & (ADBMS6948_CFGAR5_MASK_FC));

		/* Under Voltage Settings */
		nUnderVoltCompVal = Adbms6948_pDaisyChainCfgInput[nChainIndex].Adbms6948_nUnderVoltThreshold;
		nCfgBData[0] = (uint8_t)(nCfgBData[0] | ((uint8_t)(nUnderVoltCompVal & (uint8_t)0x0FF)));
		nCfgBData[1] = (uint8_t)(nCfgBData[1] | (uint8_t)((nUnderVoltCompVal & (uint16_t)0xF00u) >> 8u));

		/* Over Voltage Settings */
		nOverVoltCompVal = Adbms6948_pDaisyChainCfgInput[nChainIndex].Adbms6948_nOverVoltThreshold;
		nCfgBData[1] = (uint8_t)(nCfgBData[1] | (uint8_t)((uint16_t)(nOverVoltCompVal & 0x000Fu) << 4u));
		nCfgBData[2] = (uint8_t)(nCfgBData[2] | (uint8_t)((nOverVoltCompVal & (uint16_t)0x0FF0u) >> 4u));

        /* GPIO11 pin selection */
        nGpio11Sel = Adbms6948_pDaisyChainCfgInput[nChainIndex].Adbms6948_eGpio11CfgType;
        nCfgCData[4] = (uint8_t)((nGpio11Sel & 0x03) << ADBMS6948_CFGCR0_POS_G11_SEL);

		/* Wake up N devices in the daisy chain */
		Adbms6948_aoChainStateInfo[nChainIndex].bFirstWakeup = TRUE;

		Adbms6948_aoChainStateInfo[nChainIndex].nCurrNoOfDevices =
							Adbms6948_pDaisyChainCfgInput[nChainIndex].Adbms6948_nNoOfDevices;

        /*Wakeup*/
		Adbms6948_Cmn_WakeUpBmsIC(nChainIndex);

        /*Software reset the IC, puts it into sleep mode*/
		Adbms6948_Cmd_ExecCmd(ADBMS6948_CMD_SRST, FALSE, nChainIndex);

        /*Wakeup the IC again*/
		Adbms6948_Cmn_WakeUpBmsIC(nChainIndex);

		/* Copy the configuration data to local variable */
		Adbms6948_lPrepConfigBuff(ADBMS6948_CMD_RDCFGA,aRdCfgData,nCfgAData,&nCfgDataChain[0u],nChainIndex);

		/* Clear the command counter. */
		Adbms6948_Cmd_ExecCmd(ADBMS6948_CMD_RSTCC, FALSE, nChainIndex);

		/* Write the configuration register to the BMS IC */
		Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCFGA, &nCfgDataChain[0], nChainIndex);
        /* Read the configuration register from the BMS IC and compare with the written values*/
        anCmdList[0u] = ADBMS6948_CMD_RDCFGA;
        nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, aRdCfgData, ADBMS6948_NUMOF_REGGRP_ONE, nChainIndex);

        /*Calibration delay*/
        ADI_PAL_TIMERDELAY(ADBMS6948_TIME_CALIBRATION, nChainIndex);
#if 0
        for(int i = 0; i < ADBMS6948_REG_DATA_LEN_WITHOUT_PEC; ++i)
        {
            if (aRdCfgData[0][i + ADBMS6948_CMD_DATA_LEN] != nCfgDataChain[i])
            {
                nRet = E_NOT_OK;
                break;
            }
        }
#endif
        if (E_OK == nRet)
        {
            /* Copy the configuration data to local variable */
            Adbms6948_lPrepConfigBuff(ADBMS6948_CMD_RDCFGB, aRdCfgData, nCfgBData, &nCfgDataChain[0u], nChainIndex);
            /* Write the configuration to the chain */
            Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCFGB, &nCfgDataChain[0], nChainIndex);
            /* Read the configuration register from the BMS IC and compare with the written values*/
            anCmdList[0u] = ADBMS6948_CMD_RDCFGB;
            nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, aRdCfgData, ADBMS6948_NUMOF_REGGRP_ONE, nChainIndex);
            for(int i = 0; i < ADBMS6948_REG_DATA_LEN_WITHOUT_PEC; ++i)
            {
                if (aRdCfgData[0][i + ADBMS6948_CMD_DATA_LEN] != nCfgDataChain[i])
                {
                    nRet = E_NOT_OK;
                    break;
                }
            }
        }

        if (E_OK == nRet)
        {
            /* Copy the configuration data to local variable */
            Adbms6948_lPrepConfigBuff(ADBMS6948_CMD_RDCFGC,aRdCfgData,nCfgCData,&nCfgDataChain[0u],nChainIndex);
            /* Write the configuration to the chain */
            Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCFGC, &nCfgDataChain[0], nChainIndex);
            /* Read the configuration register from the BMS IC and compare with the written values*/
            anCmdList[0u] = ADBMS6948_CMD_RDCFGC;
            nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, aRdCfgData, ADBMS6948_NUMOF_REGGRP_ONE, nChainIndex);
            for(int i = 0; i < ADBMS6948_REG_DATA_LEN_WITHOUT_PEC; ++i)
            {
                if (aRdCfgData[0][i + ADBMS6948_CMD_DATA_LEN] != nCfgDataChain[i])
                {
                    nRet = E_NOT_OK;
                    break;
                }
            }
        }

		if (E_OK == nRet)
		{
			if (TRUE == Adbms6948_Cmn_IsError(nChainIndex))
			{
				nRet = E_NOT_OK;
				/* Platform error. */
				Adbms6948_Cmn_ReportRuntimeErr(Adbms6948_pConfig->Adbms6948_pRunTimeErrCfg->Adbms6948_E_Platform_Error.nErrID,
						ADBMS6948_RUNTIME_E_STATUS_FAILED);
			}
		}
	}
	if (E_OK == nRet)
	{
		/* Update the state information */
		Adbms6948_aoChainStateInfo[nChainIndex].eChainState = ADBMS6948_ST_CHAIN_IDLE;
		Adbms6948_aoChainStateInfo[nChainIndex].nCurrNoOfDevices =
		Adbms6948_pDaisyChainCfgInput[nChainIndex].Adbms6948_nNoOfDevices;
		Adbms6948_aoChainStateInfo[nChainIndex].bFirstWakeup = FALSE;
		Adbms6948_lInitMaxCellGrps(nChainIndex);
	}

return (nRet);

}

/*
 * Function Definition section
 */
 /*!
    @brief          This function updates the different values for specified
					register in the configuration register group for all the
					devices with specified data.

	@param	[in]   	eCfgRegGrpSel  	Selects configuration register group.

 	@param	[in]   	nRegId	 		The register to be updated in the group.

 	@param	[in]   	nMask	 		Mask to specify the field to be updated.

 	@param	[in]   	pData	 		Array of data for all devices.

	@param	[in]   	knChainID	    The Id of the daisy chain on which the
									operation is done. It is mapped to the array
									index of configuration in
									Adbms6948_DaisyChainCfgType member of
									\ref Adbms6948_CfgType. It starts with 0
									for the first chain.

   @return        	E_OK    : Operation was successful \n
                    E_NOT_OK: Operation failed \n

 */
Adbms6948_ReturnType  Adbms6948_Cmn_UpdateConfigPerDevice
(
	Adbms6948_ConfigGroupSelection  	eCfgRegGrpSel,
	uint8_t 							nRegId,
	uint8_t 							nMask,
	uint8_t 						   *pData,
	const uint8_t  						knChainID
)
{
	Adbms6948_ReturnType  nRet = E_OK;
	uint8_t  nDeviceIndex;
	uint8_t  nRdCfgByteIndex;
	uint8_t  nWrCfgByteIndex;
	uint8_t  aRdCfgData[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)] = {0u};
	uint8_t  aWrCfgData[ADBMS6948_REG_GRP_LEN * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN] = {0u};
	uint8_t  nLoopIdx;
	uint8_t  nCfgBytesPerDev;
	uint16_t  nRdCmd, nWrCmd,nWrIdx,aCmdList[1u];
	uint32_t  nDataCfg,nDataBufLen;

	nDataBufLen = (uint32_t)((uint32_t)ADBMS6948_CMD_DATA_LEN + Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices * (uint32_t)ADBMS6948_REG_DATA_LEN_WITH_PEC);
	nDataCfg = (((uint32_t)ADBMS6948_NUMOF_REGGRP_ONE << 16u) | (nDataBufLen));

	switch(eCfgRegGrpSel)
	{
		case ADBMS6948_CONFIG_GROUP_A:
			nRdCmd = ADBMS6948_CMD_RDCFGA;
			nWrCmd = ADBMS6948_CMD_WRCFGA;
			break;
		case ADBMS6948_CONFIG_GROUP_B:
			nRdCmd = ADBMS6948_CMD_RDCFGB;
			nWrCmd = ADBMS6948_CMD_WRCFGB;
			break;
		default:
			nRet = E_NOT_OK;
			break;
	}

	if (nRet == E_OK)
	{
		if (nRegId<=5u)
		{
			nCfgBytesPerDev = (uint8_t)(ADBMS6948_REG_GRP_LEN + ADBMS6948_PEC_LEN);

			/* Read the configuration to the chain */
			aCmdList[0u] =  nRdCmd;
			Adbms6948_Cmd_ExecuteCmdRD(aCmdList,aRdCfgData,nDataCfg,FALSE,knChainID);

			for (nDeviceIndex = 0u; nDeviceIndex < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDeviceIndex++)
			{
				nRdCfgByteIndex = (uint8_t)(ADBMS6948_CMD_DATA_LEN + (uint8_t)(nDeviceIndex * ADBMS6948_REG_DATA_LEN_WITH_PEC));
				nWrCfgByteIndex = nDeviceIndex * (ADBMS6948_REG_GRP_LEN);
				if (TRUE == Adbms6948_Cmn_ValidatePEC((uint8_t*)&aRdCfgData[0u][nRdCfgByteIndex],nCfgBytesPerDev, nDeviceIndex, knChainID))
				{
					for (nLoopIdx = 0u; nLoopIdx < ADBMS6948_REG_GRP_LEN; nLoopIdx++)
					{
						aWrCfgData[nWrCfgByteIndex + nLoopIdx] = aRdCfgData[0u][nRdCfgByteIndex + nLoopIdx];
					}
					if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDeviceIndex])
					{
						nWrIdx = (uint16_t)nWrCfgByteIndex + nRegId;
						aWrCfgData[nWrIdx] &= (uint8_t)(~nMask);
						aWrCfgData[nWrIdx] |= (uint8_t)(pData[nDeviceIndex]);
					}
				}
				else
				{
					nRet |= E_NOT_OK;
				}
			}

			if (nRet == E_OK)
			{
                /* Write the configuration to the chain */
                Adbms6948_Cmd_ExecuteCmdWR(nWrCmd, &aWrCfgData[0], knChainID);

                /*Read the configuration register and verify contents along with PEC and command counter*/
    			aCmdList[0u] =  nRdCmd;
                Adbms6948_Cmd_ExecuteCmdRD(aCmdList, aRdCfgData, nDataCfg, FALSE, knChainID);
                for (nDeviceIndex = 0u; nDeviceIndex < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDeviceIndex++)
                {
                    nRdCfgByteIndex = (uint8_t)(ADBMS6948_CMD_DATA_LEN + (uint8_t)(nDeviceIndex * ADBMS6948_REG_DATA_LEN_WITH_PEC));
                    nWrCfgByteIndex = nDeviceIndex * (ADBMS6948_REG_GRP_LEN);
                    if (TRUE == Adbms6948_Cmn_ValidatePEC((uint8_t*)&aRdCfgData[0u][nRdCfgByteIndex], nCfgBytesPerDev, nDeviceIndex, knChainID))
                    {
                        for (nLoopIdx = 0u; nLoopIdx < ADBMS6948_REG_GRP_LEN; nLoopIdx++)
                        {
                            if (aRdCfgData[0u][nRdCfgByteIndex + nLoopIdx] != aWrCfgData[nWrCfgByteIndex + nLoopIdx])
                            {
                                nRet |= E_NOT_OK;
                                break;
                            }
                        }
                    }
                    else
                    {
                        nRet = E_NOT_OK;
                        break;
                    }
                }
			}
		}
		else
		{
			nRet = E_NOT_OK;
		}
	}
return(nRet);
}

/*
 * Function Definition section
 */
 /*!
    @brief         This function updates the different values for two specified registers in the configuration register group for all the devices with specified data.

	@param	[in]   	eCfgRegGrpSel  	Selects configuration register group.

 	@param	[in]   	nRegId	 		The register to be updated in the group.

 	@param	[in]   	aMask	 		Mask to specify the field to be updated.

 	@param	[in]   	pData	 		Array of data for all devices.

	@param	[in]   	knChainID	    The Id of the daisy chain on which the operation is done. It is mapped to the array index of configuration in Adbms6948_DaisyChainCfgType member of \ref Adbms6948_CfgType. It starts with 0 for the first chain.

   @return        	E_OK    : Operation was successful \n
                    E_NOT_OK: Operation failed \n

 */
Adbms6948_ReturnType  Adbms6948_Cmn_UpdateConfig16
(
	Adbms6948_ConfigGroupSelection  	eCfgRegGrpSel,
	uint8_t 							nRegId,
	uint8_t 							aMask[],
	uint8_t 						   *pData,
	const uint8_t  						knChainID
)
{
	Adbms6948_ReturnType  nRet = E_OK;
	uint8_t  nDeviceIndex;
	uint8_t  nRdCfgByteIndex;
	uint8_t  nWrCfgByteIndex;
	uint8_t  aRdCfgData[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)] = {0u};
	uint8_t  aWrCfgData[ADBMS6948_REG_GRP_LEN * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN] = {0u};
	uint8_t  nLoopIdx, nDataIndex = 0u;
	uint8_t  nCfgBytesPerDev;
	uint16_t  nRdCmd, nWrCmd,nWrIdx;
	uint16_t aCmdList[1u];
	uint32_t  nDataCfg,nDataBufLen;

	nDataBufLen = (uint32_t)((uint32_t)ADBMS6948_CMD_DATA_LEN + Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices * (uint32_t)ADBMS6948_REG_DATA_LEN_WITH_PEC);
	nDataCfg = (((uint32_t)ADBMS6948_NUMOF_REGGRP_ONE << 16u) | (nDataBufLen));

	switch(eCfgRegGrpSel)
	{
		case ADBMS6948_CONFIG_GROUP_A:
			nRdCmd = ADBMS6948_CMD_RDCFGA;
			nWrCmd = ADBMS6948_CMD_WRCFGA;
			break;
		case ADBMS6948_CONFIG_GROUP_B:
			nRdCmd = ADBMS6948_CMD_RDCFGB;
			nWrCmd = ADBMS6948_CMD_WRCFGB;
			break;
        case ADBMS6948_CONFIG_GROUP_C:
			nRdCmd = ADBMS6948_CMD_RDCFGC;
			nWrCmd = ADBMS6948_CMD_WRCFGC;
			break;
        case ADBMS6948_CONFIG_GROUP_D:
			nRdCmd = ADBMS6948_CMD_RDCFGD;
			nWrCmd = ADBMS6948_CMD_WRCFGD;
			break;
        case ADBMS6948_CONFIG_GROUP_E:
			nRdCmd = ADBMS6948_CMD_RDCFGE;
			nWrCmd = ADBMS6948_CMD_WRCFGE;
			break;
        case ADBMS6948_CONFIG_GROUP_F:
			nRdCmd = ADBMS6948_CMD_RDCFGF;
			nWrCmd = ADBMS6948_CMD_WRCFGF;
			break;
        case ADBMS6948_CONFIG_GROUP_G:
			nRdCmd = ADBMS6948_CMD_RDCFGG;
			nWrCmd = ADBMS6948_CMD_WRCFGG;
			break;
        case ADBMS6948_CONFIG_GROUP_H:
			nRdCmd = ADBMS6948_CMD_RDCFGH;
			nWrCmd = ADBMS6948_CMD_WRCFGH;
			break;
        case ADBMS6948_CONFIG_GROUP_I:
			nRdCmd = ADBMS6948_CMD_RDCFGI;
			nWrCmd = ADBMS6948_CMD_WRCFGI;
			break;
		default:
			nRet = E_NOT_OK;
			break;
	}

	if (nRegId > 4u)
	{
		nRet = E_NOT_OK;
	}

	if (nRet == E_OK)
	{
		nCfgBytesPerDev = (uint8_t)(ADBMS6948_REG_GRP_LEN + ADBMS6948_PEC_LEN);

		/* Read the configuration to the chain */
		aCmdList[0u] =  nRdCmd;
		Adbms6948_Cmd_ExecuteCmdRD(aCmdList,aRdCfgData,nDataCfg,FALSE, knChainID);

		for (nDeviceIndex = 0u; nDeviceIndex < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDeviceIndex++)
		{
			nRdCfgByteIndex = (uint8_t)(ADBMS6948_CMD_DATA_LEN + (uint8_t)(nDeviceIndex * ADBMS6948_REG_DATA_LEN_WITH_PEC));
			nWrCfgByteIndex = nDeviceIndex * (ADBMS6948_REG_GRP_LEN);
			if (TRUE == Adbms6948_Cmn_ValidatePEC((uint8_t*)&aRdCfgData[0u][nRdCfgByteIndex],nCfgBytesPerDev, nDeviceIndex, knChainID))
			{
				for (nLoopIdx = 0u; nLoopIdx < ADBMS6948_REG_GRP_LEN; nLoopIdx++)
				{
					aWrCfgData[nWrCfgByteIndex + nLoopIdx] = aRdCfgData[0u][nRdCfgByteIndex + nLoopIdx];
				}
				if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDeviceIndex])
				{
					nWrIdx = (uint16_t)nWrCfgByteIndex + nRegId;
					aWrCfgData[nWrIdx] &= (uint8_t)(~aMask[0u]);
					aWrCfgData[nWrIdx] |= (uint8_t)(pData[nDataIndex++]);
					nWrIdx = (uint16_t)nWrCfgByteIndex + nRegId + 1u;
					aWrCfgData[nWrIdx] &= (uint8_t)(~aMask[1u]);
					aWrCfgData[nWrIdx] |= (uint8_t)(pData[nDataIndex++]);
				}
			}
			else
			{
				nRet |= E_NOT_OK;
			}
		}

		if (nRet == E_OK)
		{
			/* Write the configuration to the chain */
			Adbms6948_Cmd_ExecuteCmdWR(nWrCmd, &aWrCfgData[0], knChainID);

            /*Read the configuration register and verify contents along with PEC and command counter*/
            aCmdList[0u] =  nRdCmd;
            Adbms6948_Cmd_ExecuteCmdRD(aCmdList, aRdCfgData, nDataCfg, FALSE, knChainID);

            for (nDeviceIndex = 0u; nDeviceIndex < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDeviceIndex++)
            {
                nRdCfgByteIndex = (uint8_t)(ADBMS6948_CMD_DATA_LEN + (uint8_t)(nDeviceIndex * ADBMS6948_REG_DATA_LEN_WITH_PEC));
                nWrCfgByteIndex = nDeviceIndex * (ADBMS6948_REG_GRP_LEN);
                if (TRUE == Adbms6948_Cmn_ValidatePEC((uint8_t*)&aRdCfgData[0u][nRdCfgByteIndex],nCfgBytesPerDev, nDeviceIndex, knChainID))
                {
                    for (nLoopIdx = 0u; nLoopIdx < ADBMS6948_REG_GRP_LEN; nLoopIdx++)
                    {
                        if (aRdCfgData[0u][nRdCfgByteIndex + nLoopIdx] != aWrCfgData[nWrCfgByteIndex + nLoopIdx])
                        {
                            nRet |= E_NOT_OK;
                            break;
                        }
                    }
                }
                else
                {
                    nRet = E_NOT_OK;
                    break;
                }
            }
		}
	}
return(nRet);
}

/*
 * Function Definition section
 */
 /*!
    @brief         This function updates the different values for two specified registers in the configuration register group for all the devices with specified data.

 	@param	[in]   	pPwmCfgA	 		Array of config A Pwm register data.

 	@param	[in]   	pPwmCfgB	 		Array of config B Pwm register data.

	@param	[in]   	knChainID	    The Id of the daisy chain on which the operation is done. It is mapped to the array index of configuration in Adbms6948_DaisyChainCfgType member of \ref Adbms6948_CfgType. It starts with 0 for the first chain.

   @return        	E_OK    : Operation was successful \n
                    E_NOT_OK: Operation failed \n

 */
Adbms6948_ReturnType  Adbms6948_Cmn_PwmDischarge(
uint8_t 		*pPwmCfgA,
uint8_t 		*pPwmCfgB,
const uint8_t  	 knChainID
)
{
	Adbms6948_ReturnType  nRet;
    uint16_t nCmd = ADBMS6948_CMD_RDPWMA;
	uint8_t aRdPwmData[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)] = {0u};
    uint32_t nDataCfg,nDataBufLen;
    uint8_t nDeviceIndex;
    uint32_t nRdPwmByteIndex, nWrPwmByteIndex;

	nDataBufLen = (uint32_t)((uint32_t)ADBMS6948_CMD_DATA_LEN + Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices * (uint32_t)ADBMS6948_REG_DATA_LEN_WITH_PEC);
	nDataCfg = (((uint32_t)ADBMS6948_NUMOF_REGGRP_ONE << 16u) | (nDataBufLen));

    /* Write PWM A config registers */
	Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRPWMA, &pPwmCfgA[0], knChainID);
    /* Read PWM A config register and verify the register value, PEC and command count*/
    Adbms6948_Cmd_ExecuteCmdRD(&nCmd, aRdPwmData, nDataCfg, FALSE, knChainID);
    for (nDeviceIndex = 0u; nDeviceIndex < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDeviceIndex++)
    {
        nRdPwmByteIndex = (uint8_t)(ADBMS6948_CMD_DATA_LEN + (uint8_t)(nDeviceIndex * ADBMS6948_REG_DATA_LEN_WITH_PEC));
        nWrPwmByteIndex = nDeviceIndex * (ADBMS6948_REG_GRP_LEN);
        if (TRUE == Adbms6948_Cmn_ValidatePEC((uint8_t*)&aRdPwmData[0u][nRdPwmByteIndex], ADBMS6948_REG_DATA_LEN_WITH_PEC, nDeviceIndex, knChainID))
        {
            for (uint8_t nLoopIdx = 0u; nLoopIdx < ADBMS6948_REG_GRP_LEN; nLoopIdx++)
            {
                if (aRdPwmData[0u][nRdPwmByteIndex + nLoopIdx] != pPwmCfgA[nWrPwmByteIndex + nLoopIdx])
                {
                    nRet |= E_NOT_OK;
                    break;
                }
            }
        }
        else
        {
            nRet |= E_NOT_OK;
            break;
        }
    }

    if (E_OK == nRet)
    {
        /* Write PWM B config registers */
        Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRPWMB, &pPwmCfgB[0], knChainID);
        /* Read PWM B config register and verify the register value, PEC and command count*/
        Adbms6948_Cmd_ExecuteCmdRD(&nCmd, aRdPwmData, nDataCfg, FALSE, knChainID);
        for (nDeviceIndex = 0u; nDeviceIndex < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDeviceIndex++)
        {
            nRdPwmByteIndex = (uint8_t)(ADBMS6948_CMD_DATA_LEN + (uint8_t)(nDeviceIndex * ADBMS6948_REG_DATA_LEN_WITH_PEC));
            nWrPwmByteIndex = nDeviceIndex * (ADBMS6948_REG_GRP_LEN);
            if (TRUE == Adbms6948_Cmn_ValidatePEC((uint8_t*)&aRdPwmData[0u][nRdPwmByteIndex], ADBMS6948_REG_DATA_LEN_WITH_PEC, nDeviceIndex, knChainID))
            {
                for (uint8_t nLoopIdx = 0u; nLoopIdx < ADBMS6948_REG_GRP_LEN; nLoopIdx++)
                {
                    if (aRdPwmData[0u][nRdPwmByteIndex + nLoopIdx] != pPwmCfgA[nWrPwmByteIndex + nLoopIdx])
                    {
                        nRet |= E_NOT_OK;
                        break;
                    }
                }
            }
            else
            {
                nRet |= E_NOT_OK;
                break;
            }
        }
    }

	return nRet;
}

/*!
    @brief         This function reads the device revision code for all the devices in the chain.

	@param	[in]   pRevCode		Place holder to store the devices revision code/s.

	@param	[in]   knChainID	The Id of the daisy chain on which the operation is done. It is mapped to the array index of configuration in Adbms6948_DaisyChainCfgType member of \ref Adbms6948_CfgType. It starts with 0 for the first chain.

    @return        	E_OK    : Operation was successful \n
                    E_NOT_OK: Operation failed \n
 */
Adbms6948_ReturnType  Adbms6948_Cmn_ReadDevRevCode
(
	uint8_t 		*pRevCode,
	const uint8_t  	 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_OK;
	uint8_t  aSter5data[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)] = {0};
	uint8_t  aRevCodedata[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN] = {0};
	uint8_t  nDeviceIndex, nRdCfgByteIndex,nUserBuffIndex=0;
	uint16_t aCmdList[1u];
	uint32_t  nDataCfg,nDataBufLen;

	nDataBufLen = (uint32_t)((uint32_t)ADBMS6948_CMD_DATA_LEN + Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices * (uint32_t)ADBMS6948_REG_DATA_LEN_WITH_PEC);
	nDataCfg = (((uint32_t)ADBMS6948_NUMOF_REGGRP_ONE << 16u) | (nDataBufLen));

	/* Read the configuration of the chain */
	aCmdList[0u] =  ADBMS6948_CMD_RDSTATE;
	Adbms6948_Cmd_ExecuteCmdRD(aCmdList,aSter5data,nDataCfg,FALSE,knChainID);

	for (nDeviceIndex = 0u; nDeviceIndex < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDeviceIndex++)
	{
		nRdCfgByteIndex = (uint8_t)(ADBMS6948_CMD_DATA_LEN + (uint8_t)(nDeviceIndex * ADBMS6948_REG_DATA_LEN_WITH_PEC));
		if (TRUE == Adbms6948_Cmn_ValidatePEC((uint8_t*)&aSter5data[0u][nRdCfgByteIndex],ADBMS6948_REG_DATA_LEN_WITH_PEC, nDeviceIndex, knChainID))
		{
			/* Get device revision code. */
			aRevCodedata[nDeviceIndex] = (uint8_t)((uint8_t)(aSter5data[0u][nRdCfgByteIndex + 5u] & ADBMS6948_STER5_MASK_REV) >> ADBMS6948_STER5_POS_REV);
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDeviceIndex])
			{
				/* Pass the device revision code to user buffer. */
				pRevCode[nUserBuffIndex] = aRevCodedata[nDeviceIndex];
				nUserBuffIndex++;
			}
		}
		else
		{
			nRet |= E_NOT_OK;
		}
	}
return(nRet);
}


/*!
    @brief         This function is used to send the soft reset to all the devices in chain.

	@param	[in]   knChainID	The Id of the daisy chain on which the operation is done.

    @return        	E_OK    : Operation was successful \n
                    E_NOT_OK: Operation failed \n
 */
Adbms6948_ReturnType Adbms6948_Cmn_SoftReset
(
	const uint8_t 	knChainID
)
{
	Adbms6948_ReturnType nRet = E_OK;
	uint8_t  aRdStatcData[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN +
				(ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)] = {0u};
	uint8_t  nStatDataChain[(ADBMS6948_REG_GRP_LEN) *
		ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];

	/* Set Sleep bits in the data field of clrflag command */
	uint8_t 	aClrData[6]={0x00,0x00,0x00,0x00,0x00,0x08};

	/* Clear SLEEP bit in the status register */
	Adbms6948_lPrepConfigBuff(ADBMS6948_CMD_RDSTATC,aRdStatcData,aClrData,&nStatDataChain[0],knChainID);
	Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_CLRFLAG,&nStatDataChain[0],knChainID);

	/* Execute soft reset command */
	Adbms6948_Cmd_ExecCmd(ADBMS6948_CMD_SRST,FALSE,knChainID);

    /* Set the bFirstWakeup to TRUE so that the driver uses wait time for SLEEP
     state instead of STANDBY state*/
    Adbms6948_aoChainStateInfo[knChainID].bFirstWakeup = TRUE;
	return nRet;
}

/*!
    @brief  This function prepares the write comm register group data and executes start comm to
    shift out the data bytes to the i2c device connected to all the cell monitors on the chain.
    This function writes one register group data set on to the i2c devices. The user has to invoke
    this function multiple times to send out the complete data as required.

    @param  [in]  poWriteCfg  The i2c device write configuration for all the devices on the chain.

    @param  [in]  poIdxData  The index data structure for write transaction.

    @param  [in]  bFirstWrite  Indicates if it is the first write transaction.
    TRUE - First I2C write transaction.
    FALSE - Subsequent write transactions.

    @param  [in]  bReadReq  Indicates if it is request to read the data from the i2c device/(s).
    TRUE - Write transaction for I2C data read.
    FALSE - Write transaction for I2C data write.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  E_OK: Operation was successful
             E_NOT_OK: Operation failed
*/
void  Adbms6948_Cmn_I2CWrSendRegGrpData
(
Adbms6948_I2CWriteCfgType 	*poWriteCfg,
Adbms6948_I2CIdxData 		*poIdxData,
boolean   					 bFirstWrite,
boolean   					 bReadReq,
const uint8_t  				 knChainID
)
{
    uint8_t anWrRegData[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN][ADBMS6948_REG_GRP_LEN];


    if(bReadReq == FALSE)
    {
            /* Prepare the data for write comm for i2c wr data */
            Adbms6948_lI2CPrepWrDataWr(poWriteCfg, anWrRegData, poIdxData,
                    bFirstWrite, knChainID);
    }
    else
    {
            /* Prepare the data for write comm for i2c rd data */
            Adbms6948_lI2CPrepWrDataRd(poWriteCfg, anWrRegData, poIdxData,
                    knChainID);
    }

    /* Write the comm register data on the chain */
    Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCOMM, &anWrRegData[0][0], knChainID);
    /* Transfer the comm data to the i2c device */
    Adbms6948_Cmd_ExecCmd(ADBMS6948_CMD_STCOMM, TRUE, knChainID);
}

/*!
    @brief  This function writes the COMM register group for a read transaction.

    @param  [out]  pnBytesRead  Number of bytes read from the i2c devices in
    the current transaction

    @param  [out]  pnRdLen  The number of bytes to be read from the I2C device
    in the current I2C read transaction for all the devices on the chain.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  None.
*/
void  Adbms6948_Cmn_I2CRdSendRegGrpData
(
uint8_t 		*pnBytesRead,
uint16_t 		*pnRdLen,
const uint8_t    knChainID
)
{
    uint8_t nDevIdx, nNoOfDevOnChain;
	uint8_t anWrRegData[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN][ADBMS6948_REG_GRP_LEN];

    /* Get number of devices on chain */
    nNoOfDevOnChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
    /* Loop to read all the databytes from the i2c devices */
    for (nDevIdx = 0u; nDevIdx < nNoOfDevOnChain; nDevIdx++)
    {
    	if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
    	{
			switch(pnRdLen[nDevIdx])
			{
			case 0u:
				anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_NOTX | ADBMS6948_I2C_FCOMWR_MNACKSTOP);
				anWrRegData[nDevIdx][1u] = ADBMS6948_I2C_FILLERBYTE;
				anWrRegData[nDevIdx][2u] = (ADBMS6948_I2C_ICOMWR_NOTX | ADBMS6948_I2C_FCOMWR_MNACKSTOP);
				anWrRegData[nDevIdx][3u] = ADBMS6948_I2C_FILLERBYTE;
				anWrRegData[nDevIdx][4u] = (ADBMS6948_I2C_ICOMWR_NOTX | ADBMS6948_I2C_FCOMWR_MNACKSTOP);
				anWrRegData[nDevIdx][5u] = ADBMS6948_I2C_FILLERBYTE;
				pnBytesRead[nDevIdx] = 0u;
				pnRdLen[nDevIdx] = 0u;
				break;
			case 1u:
				anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_BLANK | ADBMS6948_I2C_FCOMWR_MNACKSTOP);
				anWrRegData[nDevIdx][1u] = ADBMS6948_I2C_READBYTE;
				anWrRegData[nDevIdx][2u] = (ADBMS6948_I2C_ICOMWR_NOTX | ADBMS6948_I2C_FCOMWR_MNACKSTOP);
				anWrRegData[nDevIdx][3u] = ADBMS6948_I2C_FILLERBYTE;
				anWrRegData[nDevIdx][4u] = (ADBMS6948_I2C_ICOMWR_NOTX | ADBMS6948_I2C_FCOMWR_MNACKSTOP);
				anWrRegData[nDevIdx][5u] = ADBMS6948_I2C_FILLERBYTE;
				pnBytesRead[nDevIdx] = 1u;
				pnRdLen[nDevIdx] = 0u;
				break;
			case 2u:
				anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_BLANK | ADBMS6948_I2C_FCOMWR_MACK);
				anWrRegData[nDevIdx][1u] = ADBMS6948_I2C_READBYTE;
				anWrRegData[nDevIdx][2u] = (ADBMS6948_I2C_ICOMWR_BLANK | ADBMS6948_I2C_FCOMWR_MNACKSTOP);
				anWrRegData[nDevIdx][3u] = ADBMS6948_I2C_READBYTE;
				anWrRegData[nDevIdx][4u] = (ADBMS6948_I2C_ICOMWR_NOTX | ADBMS6948_I2C_FCOMWR_MNACKSTOP);
				anWrRegData[nDevIdx][5u] = ADBMS6948_I2C_FILLERBYTE;
				pnBytesRead[nDevIdx] = 2u;
				pnRdLen[nDevIdx] = 0u;
				break;
			default:
				anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_BLANK | ADBMS6948_I2C_FCOMWR_MACK);
				anWrRegData[nDevIdx][1u] = ADBMS6948_I2C_READBYTE;
				anWrRegData[nDevIdx][2u] = (ADBMS6948_I2C_ICOMWR_BLANK | ADBMS6948_I2C_FCOMWR_MACK);
				anWrRegData[nDevIdx][3u] = ADBMS6948_I2C_READBYTE;
				if (pnRdLen[nDevIdx] == 3U)
				{
	                 anWrRegData[nDevIdx][4u] = (ADBMS6948_I2C_ICOMWR_BLANK | ADBMS6948_I2C_FCOMWR_MNACKSTOP);
				}
				else
				{
					anWrRegData[nDevIdx][4u] = (ADBMS6948_I2C_ICOMWR_BLANK | ADBMS6948_I2C_FCOMWR_MACK);
				}
				anWrRegData[nDevIdx][5u] = ADBMS6948_I2C_READBYTE;
				pnBytesRead[nDevIdx] = 3u;
				pnRdLen[nDevIdx] = (uint8_t)(pnRdLen[nDevIdx] - 3u);
				break;
			} /* end of switch */
    	}
    } /* device loop */
    /* Write the comm register data on the chain */
    Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCOMM, &anWrRegData[0][0], knChainID);
    /* Transfer the comm data to the i2c device */
    Adbms6948_Cmd_ExecCmd(ADBMS6948_CMD_STCOMM, TRUE, knChainID);
}

/*!
    @brief  This function gets the comm register group and updates the write
    status for all the  devices on the chain and the read data from the i2c
    devices based on the user request.

    @param  [in]  poReadCfg  Pointer to the structure containing the read
    configuration for all the devices on the chain. The user can pass 'NULL_PTR'
    is read data is not required.

    @param  [in]  pnBytesWrOrRd  Pointer to the memory location that contains
    the bytes written or read in the current transaction for all the devices on
    the chain.

    @param  [in]  pnByteIdx  Pointer to the memory location that contains the
    byte index of the read buffer for all the devices on the chain.

    @param  [in]  pnWrOrRdStat  Pointer to the memory location to hold the
    write/read status for all the devices on the chain.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  E_OK: Operation Successful.
             E_NOT_OK: Operation failed.
*/
Adbms6948_ReturnType  Adbms6948_Cmn_I2CGetRdData
(
Adbms6948_I2CReadCfgType 	*poReadCfg,
uint8_t 					*pnBytesWrOrRd,
uint16_t 					*pnByteIdx,
uint8_t 					*pnWrOrRdStat,
const uint8_t   			 knChainID
)
{
    Adbms6948_ReturnType   nRet;
    uint8_t   nDevIdx, nNoOfDevOnChain,nDevStartIdx,
            anRdRegData[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN][ADBMS6948_MAX_FRAME_SIZE];
    uint8_t   anTmpWrStat[3u];
    uint16_t   anCmdLst[1u];


    /* Read the COMM register group for all the devices on the chain and update the
     * 'status'.*/
    anCmdLst[0u]=ADBMS6948_CMD_RDCOMM;
    nRet = Adbms6948_Cmn_ReadRegGroup(anCmdLst, anRdRegData,1u, knChainID);
    nNoOfDevOnChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
    /* Loop to write all the databytes on to the i2c devices */
    for (nDevIdx = 0u; nDevIdx < nNoOfDevOnChain; nDevIdx++)
    {
    	if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
    	{
    		nDevStartIdx = (uint8_t)((uint8_t)ADBMS6948_CMD_DATA_LEN + (nDevIdx * (uint8_t)ADBMS6948_REG_DATA_LEN_WITHOUT_PEC));
    		anTmpWrStat[0] = (uint8_t)(anRdRegData[0u][nDevStartIdx] & ADBMS6948_I2C_FCOM_MASK);
    		anTmpWrStat[1] = (uint8_t)(anRdRegData[0u][nDevStartIdx+ 2u] & ADBMS6948_I2C_FCOM_MASK);
    		anTmpWrStat[2] = (uint8_t)(anRdRegData[0u][nDevStartIdx+ 4u] & ADBMS6948_I2C_FCOM_MASK);

    		if(poReadCfg != NULL_PTR)
    		{
    		    pnWrOrRdStat[nDevIdx] |=
    		    Adbms6948_lI2CGetRdStat(pnBytesWrOrRd[nDevIdx], anTmpWrStat);
    		    if(poReadCfg[nDevIdx].pnRdDataBuf != NULL_PTR)
    		    {
    		        Adbms6948_lI2CUpdRdBuf(poReadCfg, anRdRegData, pnBytesWrOrRd,
    		        		                    pnByteIdx, nDevIdx);
    		    }
    		}
    		else
    		{
    		    pnWrOrRdStat[nDevIdx] |=
    		    Adbms6948_lI2CGetWrStat(pnBytesWrOrRd[nDevIdx], anTmpWrStat);
    		}

    		/* Update user result as failed if status was failed in earlier
    		transaction and passed now as the overall result is failed */
    		if(pnWrOrRdStat[nDevIdx] ==
    		    (ADBMS6948_I2C_WRFAILED | ADBMS6948_I2C_WRSUCCESS) )
    		{
    		    /* Set R/W status as failed */
    		    pnWrOrRdStat[nDevIdx] = ADBMS6948_I2C_WRFAILED;
    		}
    	}
    } /* device loop for write status check */
    return nRet;
}

/*****************************************************************************/
/******************** Measurement and diagnostic related *********************/
/*****************************************************************************/
/*****************************************************************************/
/*!
    @brief	Common function to read one more register groups.

    @param	[out]	pnCmdLst	Array of read commands.
								@range: NA
								@resolution: NA

	@param 	[in] 	pRxBuff		RX data buffer/s.
								@range: NA
								@resolution: NA

	@param 	[in] 	nCmdLstLen	Specifies the number of groups to be read.
								@range: 1 to 6
								@resolution: NA

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Cmn_ReadRegGroup
(
	uint16_t 	   *pnCmdLst,
	uint8_t 		pRxBuff[][ADBMS6948_MAX_FRAME_SIZE],
	uint8_t 		nCmdLstLen,
	const uint8_t  	knChainID
)
{
	Adbms6948_ReturnType  nRet = E_OK;
    uint8_t  aRdDataBuff[ADBMS6948_NUMOF_REGGRP_SIX][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)] = {0u};
    uint8_t  nDevIdx, nGrpIdx, nRegGrpDataLen;
    uint32_t nDataCfg;
    uint32_t nDataBufLen;
    uint16_t nDevStartIdx, nDevDataIdx;
    boolean  bValidData;

    nDataBufLen = (uint32_t)((uint32_t)ADBMS6948_CMD_DATA_LEN + (uint32_t)(Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices * (uint32_t)ADBMS6948_REG_DATA_LEN_WITH_PEC));
    nDataCfg = (((uint32_t)nCmdLstLen << 16u) | (nDataBufLen));
    /* Read the configuration of the chain */
    Adbms6948_Cmd_ExecuteCmdRD(pnCmdLst, aRdDataBuff, nDataCfg, FALSE, knChainID);

    for (nGrpIdx = 0u; nGrpIdx < nCmdLstLen; nGrpIdx++)
    {
		for (nDevIdx = 0u; nDevIdx < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDevIdx++)
		{
			nDevStartIdx = (uint16_t)((uint16_t)ADBMS6948_CMD_DATA_LEN + (nDevIdx * (uint16_t)ADBMS6948_REG_DATA_LEN_WITH_PEC));
			bValidData = Adbms6948_Cmn_ValidatePEC((uint8_t*)&aRdDataBuff[nGrpIdx][nDevStartIdx],ADBMS6948_REG_DATA_LEN_WITH_PEC, nDevIdx, knChainID);
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				if (TRUE == bValidData)
				{
					for (nRegGrpDataLen = 0u; nRegGrpDataLen < ADBMS6948_REG_GRP_LEN; nRegGrpDataLen++)
					{
						nDevDataIdx = nDevStartIdx + nRegGrpDataLen;
						pRxBuff[nGrpIdx][nDevDataIdx] = aRdDataBuff[nGrpIdx][nDevDataIdx];
					}
				}
				else
				{
					nRet |= E_NOT_OK;
				}
			}
		}
    }
return(nRet);
}
/*****************************************************************************/
/*!
	@brief			Common function to read register groups using "ALL" commands

	@param	[in]	nCmd        	"Read all" Command to use
	@param	[in]	pRxBuf      	Pointer to the receive buffer
	@param	[in]	nDataLenBytes   Number of bytes of data to read (including PEC)
	@param	[in]	knChainID   	Specifies ID of the daisy chain on
									which the operation is to be performed.
    @return 		E_OK: 			Success \n
            		E_NOT_OK: 		Failure \n
 */
Adbms6948_ReturnType  Adbms6948_Cmn_ReadAllRegGroups
(
	uint16_t        nCmd,
	uint8_t         *pRxBuf,
	uint8_t         nDataLenBytes,
	const uint8_t   knChainID
)
{
	Adbms6948_ReturnType  nRet = E_OK;
	uint8_t  nDevIdx;
	nDevIdx = 0;
	boolean  bValidData;

	/* Read all the register groups from ADBMS6948 */
	Adbms6948_Cmd_ExecuteCmdRDAll(pRxBuf, nCmd, nDataLenBytes, FALSE, knChainID);
	bValidData = Adbms6948_Cmn_ValidatePEC(pRxBuf, nDataLenBytes, nDevIdx, knChainID);
	if (TRUE == bValidData)
	{
		nRet = E_OK;
	}
	else
	{
		nRet = E_NOT_OK;
	}

return(nRet);
}
/*****************************************************************************/
/*!
    @brief	Common function to write one more register groups.

    @param [out]	pnCmdLst	Array of write commands.
								@range: NA
								@resolution: NA

    @param	[in]  	nDataLen  	Clear data length.
    							@range: 2 to 6
    							@resolution: NA

	@param 	[in] 	pTxBuff		TX data buffer.
								@range: NA
								@resolution: NA

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        void
 */
/*****************************************************************************/
void  Adbms6948_Cmn_WriteRegGroup
(
	uint16_t 		*pnCmdLst,
	uint8_t			nDataLen,
	uint8_t 		pTxBuff[][ADBMS6948_MAX_FRAME_SIZE],
	const uint8_t  	knChainID
)
{
	uint8_t  aDataTxBuf[ADBMS6948_REG_GRP_LEN * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];
	uint8_t  nByteIdx;
	uint16_t  nSrcDataIdx, nDataIdx = 0u;

	nSrcDataIdx = (uint16_t)ADBMS6948_CMD_DATA_LEN;
	for (nByteIdx = 0u; nByteIdx < ADBMS6948_REG_GRP_LEN; nByteIdx++)
	{
		aDataTxBuf[nDataIdx++] = pTxBuff[0u][nSrcDataIdx + nByteIdx];
	}
    Adbms6948_Cmd_ExecuteCmdCLR(pnCmdLst[0u], nDataLen, &aDataTxBuf[0u], knChainID);
}

/*!
    @brief  This function is used to execute any generic command which has no specific input or
    configuration data.

    @param  [in]  nCommand  The command to be executed.

    @param  [in]  bIncrementCmdCount  This parameter specifies if the command increments the
    command count on the device. This information will be used to increment internally stored
    command count that will be used to compare against the received command count during a read
    operation.
        TRUE - The command increments the command count on the device.
        FALSE - The command does not increment the command count on the device.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  None.
*/
void  Adbms6948_Cmn_SendCommandOnly
(
uint16_t   		nCommand,
boolean   		bIncrementCmdCount,
const uint8_t   knChainID
)
{
	Adbms6948_Cmd_ExecCmd(nCommand, bIncrementCmdCount, knChainID);
}


/*!
    @brief  This function is used to read the one or more register group data with the PEC from
    all the devices on the chain.

    @param  [in]  	pnCmdLst  	The command codes of the register group read/s to be executed.

    @param  [out]  	pRxBuf  	The buffer containing the data read from the requested register
     	 	 	 	 	 	 	 group/s for all the devices on the chain.

    @param	[in]	nDataCfg	Specifies the data buffers.
    							nDataCfg[15:0]: Specifies length of all data buffer.
    							nDataCfg[31:16]: Specifies number of data buffer.

	@param	[in]	bIsPollCmd	Specifies poll command.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  None.
*/
void  Adbms6948_Cmn_ExecuteCmdRD
(
	uint16_t 		*pnCmdLst,
	uint8_t     	 pRxBuf[][ADBMS6948_MAX_FRAME_SIZE],
	uint32_t  		 nDataCfg,
	boolean  		 bIsPollCmd,
	const uint8_t    knChainID
)
{
	Adbms6948_Cmd_ExecuteCmdRD(pnCmdLst, pRxBuf, nDataCfg, bIsPollCmd, knChainID);
}

/*!
    @brief  This function is used to execute a write to the configuration register group of all the
    devices on the chain.

    @param  [in]  nCommand  The command code for the register group write to be executed.

    @param  [in]  pTxBuf  The buffer containing the data to be written to the register group for all
    the devices on the chain.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  None.
*/
void Adbms6948_Cmn_ExecuteCmdWR
(
    uint16_t 		nCommand,
    uint8_t 		*pTxBuf,
    const uint8_t 	knChainID
)
{
	Adbms6948_Cmd_ExecuteCmdWR(nCommand, pTxBuf, knChainID);
}

/*!
    @brief  This function is used to execute any generic command which has no specific input or
    configuration data.

    @param  [in]  nCommand  The command to be executed.

    @param  [in]  nPECBitToFlip  Specifies the bit position to be flipped in calculated PEC

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  None.
*/
void Adbms6948_Cmn_ExecCmdWithCPECBitFlip
(
uint16_t  		nCommand,
uint8_t  		nPECBitToFlip,
const uint8_t  	knChainID
)
{
	Adbms6948_Cmd_ExecCmdWithCPECBitFlip(nCommand, nPECBitToFlip, knChainID);
}

/*!
    @brief  This function is used to execute a write to the configuration register group of all the
    devices on the chain.

    @param  [in]  nCommand  The command code for the register group write to be executed.

    @param  [in]  pTxBuf  The buffer containing the data to be written to the register group for all
    the devices on the chain.

    @param 	[in] nDataPECBitToFlip	Specifies the data PEC bit position to be flipped.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  None.
*/
void Adbms6948_Cmn_ExecCmdWRWithDPECBitFLip
(
	uint16_t 			nCommand,
    uint8_t 			*pTxBuf,
	uint8_t				nDataPECBitToFlip,
    const uint8_t 		knChainID
)
{
	Adbms6948_Cmd_ExecCmdWRWithDPECBitFLip(nCommand, pTxBuf, nDataPECBitToFlip, knChainID);
}
/*****************************************************************************/
/*!
    @brief  This function validates SNAP selection.
    @param  [in]   eSnapSel		SNAP selection to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
boolean  Adbms6948_Cmn_IsInvalidSnapSelType
(
    Adbms6948_SnapSelType      eSnapSel
)
{
    boolean  bRet;

    switch(eSnapSel)
    {
        case ADBMS6948_SEND_NONE:
        case ADBMS6948_SEND_SNAP:
        case ADBMS6948_SEND_UNSNAP:
        case ADBMS6948_SEND_BOTH:
            bRet = FALSE;
            break;

        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}

/*********************** End Measurement and Diagnostic ***********************/

/*!
    @brief  This function updates chain state also it checks for current
     	 	state of chain.

    @param  [in]  eReqSt	Chain state request

    @param  [in]  knChainID  Specifies ID of the daisy chain on
                                which the operation is to be performed.
                                @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                @resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
*/
Adbms6948_ReturnType  Adbms6948_Cmn_UpdateChainStatus
(
    Adbms6948_ChainStateType     eReqSt,
    const uint8_t                knChainID
)
{
    Adbms6948_ReturnType                 nRet = E_OK;

    Adbms6948_Cmn_EnterCriticalSection();
    /* Get the current chain state. */
    if (eReqSt == Adbms6948_aoChainStateInfo[knChainID].eChainState)
    {
        /* It is already in requested state */
        nRet = E_NOT_OK;
    }
    else
    {
        /* Update the chain state to requested state. */
        Adbms6948_aoChainStateInfo[knChainID].eChainState = eReqSt;
    }
    Adbms6948_Cmn_ExitCriticalSection();
    return nRet;
}
/*!
    @brief         This function is used report the development errors

	@param	[in]   ModId	Module ID

	@param	[in]   InstId	Instance ID

	@param	[in]   ApiId	Application ID

	@param	[in]   ErrCode	Error Code

    @return        None
 */
void Adbms6948_Cmn_ReportDevErr
(
	uint16_t   ModId,
	uint8_t   InstId,
	uint8_t   ApiId,
	uint8_t   ErrCode
)
{
	ADI_PAL_REPORT_DEVELOPMENT_ERROR(ModId, InstId, ApiId, ErrCode);
}

/*!
    @brief         This function is used to enter a critical section

    @return        None
 */
void Adbms6948_Cmn_EnterCriticalSection
(
void
)
{
	ADI_PAL_CRITICAL_SECTION_START;
}

/*!
    @brief         This function is used to exit a critical section

    @return        None
 */
void Adbms6948_Cmn_ExitCriticalSection
(
void
)
{
	ADI_PAL_CRITICAL_SECTION_STOP;
}

/*!
    @brief         This function is used to store values at particular memory
				   location.

    @return        None
 */
void Adbms6948_Cmn_Memset
(
	uint8_t *    pAddr,
	uint8_t   nVal,
	uint32_t  nSize
)
{
	ADI_PAL_MEMSET(pAddr,nVal,nSize);
}

/*!
    @brief         This function is used to copy the contents from one memory
				   location to another.

    @return        None
 */
void Adbms6948_Cmn_Memcpy
(
	uint8_t *    pDstAddr,
	uint8_t *   pSrcAddr,
	uint32_t  nSize
)
{
	ADI_PAL_MEMCPY(pDstAddr,pSrcAddr,nSize);
}

/*!
    @brief  This function gets the comm register group and updates the write
    status for all the devices on the chain and the read data from the i2c
    devices based on the user request.

    @param  [in]  poReadCfg  Pointer to the structure containing the read
    configuration for all the devices on the chain.

    @param  [in]  anRdRegData  Pointer to the memory location that contains the
    byte index of the read buffer for all the devices on the chain.

    @param  [in]  pnBytesWrOrRd  Pointer to the memory location that contains
    the bytes written or read in the current transaction for all the devices on
    the chain.

    @param  [in]  pnByteIdx  Pointer to store the read byte index.

    @param  [in]  nDevIdx  Current device index.

    @return  None.
*/
static void  Adbms6948_lI2CUpdRdBuf
(
Adbms6948_I2CReadCfgType 	*poReadCfg,
uint8_t   					 anRdRegData[][ADBMS6948_MAX_FRAME_SIZE],
uint8_t 					*pnBytesWrOrRd,
uint16_t 					*pnByteIdx,
uint8_t   					 nDevIdx
)
{
	uint8_t  nDevStartIdx;
	nDevStartIdx = (uint8_t)((uint8_t)ADBMS6948_CMD_DATA_LEN + (nDevIdx * (uint8_t)ADBMS6948_REG_DATA_LEN_WITH_PEC));
    if (pnBytesWrOrRd[nDevIdx] == 1u)
    {
        poReadCfg[nDevIdx].pnRdDataBuf[pnByteIdx[nDevIdx]++] =
                anRdRegData[0u][nDevStartIdx + 1u];
    }
    else if (pnBytesWrOrRd[nDevIdx] == 2u)
    {
        poReadCfg[nDevIdx].pnRdDataBuf[pnByteIdx[nDevIdx]++] =
                anRdRegData[0u][nDevStartIdx + 1u];
        poReadCfg[nDevIdx].pnRdDataBuf[pnByteIdx[nDevIdx]++] =
                anRdRegData[0u][nDevStartIdx + 3u];
    }
    else if (pnBytesWrOrRd[nDevIdx] == 3u)
    {
        poReadCfg[nDevIdx].pnRdDataBuf[pnByteIdx[nDevIdx]++] =
                anRdRegData[0u][nDevStartIdx + 1u];
        poReadCfg[nDevIdx].pnRdDataBuf[pnByteIdx[nDevIdx]++] =
                anRdRegData[0u][nDevStartIdx + 3u];
        poReadCfg[nDevIdx].pnRdDataBuf[pnByteIdx[nDevIdx]++] =
                anRdRegData[0u][nDevStartIdx + 5u];
    }
    else
    {
        /* zero bytes - no update required */
    }
}

/*TODO: Remove this code if not to be used*/
#if 0
/*!
    @brief  This function reads the configuration register A and verifies the command counter value
    read from the device against the value maintained by the driver.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  E_OK: Operation was successful
             E_NOT_OK: Operation failed
*/
static Adbms6948_ReturnType  Adbms6948_lVerifyCmdCntInit
(
    uint8_t  knChainID
)
{
    Adbms6948_ReturnType  nRet;
    uint8_t  aRdCfgData[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)] = {0u};
    uint16_t  anCmdList[ADBMS6948_NUMOF_REGGRP_ONE];

    /* Read configuration register group A and verify the PEC and command counter. */
    anCmdList[0u] = ADBMS6948_CMD_RDCFGA;
    nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, aRdCfgData, ADBMS6948_NUMOF_REGGRP_ONE, knChainID);

    return (nRet);
}
#endif

/*!
    @brief  This function determines the max number of cell groups that
    		are required to be read when read all cell is request for the chain.

	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
*/
static void  Adbms6948_lInitMaxCellGrps
(
const uint8_t   knChainID
)
{
    uint8_t   nRegGrpsToRead, nDevIdx, nMaxCellsPerDev = 0u;

    /* Get the maximum count of cells on a single device */
    for (nDevIdx = 0u; nDevIdx < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDevIdx++)
    {
    	if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
    	{
			if (nMaxCellsPerDev < Adbms6948_pDaisyChainCfgInput[knChainID].Adbms6948_pNoOfCellsPerDevice[nDevIdx])
			{
				nMaxCellsPerDev =  Adbms6948_pDaisyChainCfgInput[knChainID].Adbms6948_pNoOfCellsPerDevice[nDevIdx];
			}
    	}
    }

    nRegGrpsToRead = (uint8_t)(nMaxCellsPerDev/3u);
    if ((nMaxCellsPerDev % 3u) != 0u)
    {
        nRegGrpsToRead++;
    }

    /* Update the Chain State Information */
    Adbms6948_aoChainStateInfo[knChainID].nMaxCellRegGrps = nRegGrpsToRead;
}

/*!
    @brief  This function prepares the write comm register group data for write
    to the i2c device on all the cell monitors on the chain.

    @param  [in]  poWriteCfg  The i2c device write configuration for all the
    devices on the chain.

    @param  [in]  anWrRegData  The prepared register group data for all the cell
    monitors on the chain.

    @param  [out]  poIdxData  The index data structure for write transaction.

    @param  [in]  bFirstWrite  Indicates if it is the first write transaction.
    TRUE - First I2C write transaction.
    FALSE - Subsequent write transactions.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  E_OK: Operation was successful
             E_NOT_OK: Operation failed
*/
static void  Adbms6948_lI2CPrepWrDataWr
(
Adbms6948_I2CWriteCfgType 	*poWriteCfg,
uint8_t  					 anWrRegData[][ADBMS6948_REG_GRP_LEN],
Adbms6948_I2CIdxData 		*poIdxData,
boolean   					 bFirstWrite,
const uint8_t  				 knChainID
)
{
    uint8_t   nDevIdx;

    /* Loop to write all the databytes on to the i2c devices */
    for(nDevIdx = 0u;
        nDevIdx < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
        nDevIdx++)
    {
    	if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
    	{
    		switch(poIdxData->pnWrLen[nDevIdx])
    		{
    			case 0u:
    				/* Set no transmit for all data bytes */
    				Adbms6948_lI2CSetNoTx(nDevIdx, anWrRegData);
    				poIdxData->pnBytesWritten[nDevIdx] = 0u;
    				break;
    			case 1u:
    				if(bFirstWrite == TRUE)
    				{
    					anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_START |
    							ADBMS6948_I2C_FCOMWR_MNACK);
    					anWrRegData[nDevIdx][1u] = (uint8_t)(poWriteCfg[nDevIdx].nI2CDevAddr<<1u);
    					anWrRegData[nDevIdx][2u] = (ADBMS6948_I2C_ICOMWR_BLANK |
    							ADBMS6948_I2C_FCOMWR_MNACKSTOP);
    					anWrRegData[nDevIdx][3u] = poWriteCfg[nDevIdx].pnWrDataBuf[0];
    					poIdxData->pnBytesWritten[nDevIdx] = 2u;
    				}
    				else
    				{
    					anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_BLANK |
                                                ADBMS6948_I2C_FCOMWR_MNACKSTOP);
    					anWrRegData[nDevIdx][1u] =
    							poWriteCfg[nDevIdx].pnWrDataBuf[poIdxData->pnByteIdx[nDevIdx]++];
    					anWrRegData[nDevIdx][2u] = ADBMS6948_I2C_ICOMWR_NOTX;
    					poIdxData->pnBytesWritten[nDevIdx] = 1u;
    				}
    				anWrRegData[nDevIdx][4u] = ADBMS6948_I2C_ICOMWR_NOTX;
    				poIdxData->pnWrLen[nDevIdx] = 0u;
    				break;
    			case 2u:
    				if(bFirstWrite == TRUE)
    				{
    					anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_START |
                                                ADBMS6948_I2C_FCOMWR_MNACK);
    					anWrRegData[nDevIdx][1u] = (uint8_t)(poWriteCfg[nDevIdx].nI2CDevAddr<<1u);
    					anWrRegData[nDevIdx][2u] = (ADBMS6948_I2C_ICOMWR_BLANK |
                                                ADBMS6948_I2C_FCOMWR_MNACK);
    					anWrRegData[nDevIdx][3u] =
    							poWriteCfg[nDevIdx].pnWrDataBuf[poIdxData->pnByteIdx[nDevIdx]++];

    					anWrRegData[nDevIdx][4u] = (ADBMS6948_I2C_ICOMWR_BLANK |
                                                    ADBMS6948_I2C_FCOMWR_MNACKSTOP);
    					anWrRegData[nDevIdx][5u] =
    							poWriteCfg[nDevIdx].pnWrDataBuf[poIdxData->pnByteIdx[nDevIdx]++];
    					poIdxData->pnBytesWritten[nDevIdx] = 3u;
    				}
    				else
    				{
    					anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_BLANK |
    							ADBMS6948_I2C_FCOMWR_MNACK);
    					anWrRegData[nDevIdx][1u] =
    							poWriteCfg[nDevIdx].pnWrDataBuf[poIdxData->pnByteIdx[nDevIdx]++];
    					anWrRegData[nDevIdx][2u] = (ADBMS6948_I2C_ICOMWR_BLANK |
    							ADBMS6948_I2C_FCOMWR_MNACKSTOP);
    					anWrRegData[nDevIdx][3u] =
    							poWriteCfg[nDevIdx].pnWrDataBuf[poIdxData->pnByteIdx[nDevIdx]++];
    					anWrRegData[nDevIdx][4u] = ADBMS6948_I2C_ICOMWR_NOTX;
    						poIdxData->pnBytesWritten[nDevIdx] = 2u;
    				}
    				poIdxData->pnWrLen[nDevIdx] = 0u;
    				break;
    			default:
    				if(bFirstWrite == TRUE)
    				{
    					anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_START |
    							ADBMS6948_I2C_FCOMWR_MNACK);
    					anWrRegData[nDevIdx][1u] = (uint8_t)(poWriteCfg[nDevIdx].nI2CDevAddr<<1u);
    					poIdxData->pnWrLen[nDevIdx] =
    							(uint8_t)(poIdxData->pnWrLen[nDevIdx] - 2u);
    				}
    				else
    				{
    					anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_BLANK |
    							ADBMS6948_I2C_FCOMWR_MNACK);
    					anWrRegData[nDevIdx][1u] =
    							poWriteCfg[nDevIdx].pnWrDataBuf[poIdxData->pnByteIdx[nDevIdx]++];
    					poIdxData->pnWrLen[nDevIdx] =
    							(uint8_t)(poIdxData->pnWrLen[nDevIdx] - 3u);
    				}
    				anWrRegData[nDevIdx][2u] = (ADBMS6948_I2C_ICOMWR_BLANK |
    						ADBMS6948_I2C_FCOMWR_MNACK);
    				anWrRegData[nDevIdx][3u] =
    						poWriteCfg[nDevIdx].pnWrDataBuf[poIdxData->pnByteIdx[nDevIdx]++];
    				anWrRegData[nDevIdx][4u] = (ADBMS6948_I2C_ICOMWR_BLANK |
    						ADBMS6948_I2C_FCOMWR_MNACK);
    				if(poIdxData->pnWrLen[nDevIdx] == 0u )
    				{
    					/* MNACK_STOP condition */
    					anWrRegData[nDevIdx][4u] = (uint8_t)(anWrRegData[nDevIdx][4u]
                                                  | ADBMS6948_I2C_STOP_COND);
    				}
    				anWrRegData[nDevIdx][5u] =
    						poWriteCfg[nDevIdx].pnWrDataBuf[poIdxData->pnByteIdx[nDevIdx]++];
    				/* Total bytes written including the i2c dev address */
    				poIdxData->pnBytesWritten[nDevIdx] = 3u;
    				break;
    		} /* end of switch */
    	}
    } /* device loop */
}

/*!
    @brief  This function prepares the write comm register group data for
    reading from the i2c device on all the cell monitors on the chain.

    @param  [in]  poWriteCfg  The i2c device write configuration for all the
    devices on the chain.

    @param  [in]  anWrRegData  The prepared register group data for all the cell
    monitors on the chain.

    @param  [out]  poIdxData  The index data structure for write transaction.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  E_OK: Operation was successful
             E_NOT_OK: Operation failed
*/
static void  Adbms6948_lI2CPrepWrDataRd
(
Adbms6948_I2CWriteCfgType 	*poWriteCfg,
uint8_t   					 anWrRegData[][ADBMS6948_REG_GRP_LEN],
Adbms6948_I2CIdxData 		*poIdxData,
const uint8_t  				 knChainID
)
{
    uint8_t   nDevIdx;

    /* Loop to write all the databytes on to the i2c devices */
    for(nDevIdx = 0u;
            nDevIdx < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
            nDevIdx++)
    {
    	if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
    	{
			switch(poIdxData->pnWrLen[nDevIdx])
			{
			case 0u:
				anWrRegData[nDevIdx][0u] = ADBMS6948_I2C_ICOMWR_NOTX;
				anWrRegData[nDevIdx][2u] = ADBMS6948_I2C_ICOMWR_NOTX;
				anWrRegData[nDevIdx][4u] = ADBMS6948_I2C_ICOMWR_NOTX;
				poIdxData->pnBytesWritten[nDevIdx] = 0u;
				break;
			case 1u:
				anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_START |
					ADBMS6948_I2C_FCOMWR_MNACK);
				anWrRegData[nDevIdx][1u] = (uint8_t)(poWriteCfg[nDevIdx].nI2CDevAddr << 1U);
				anWrRegData[nDevIdx][2u] = (ADBMS6948_I2C_ICOMWR_BLANK |
							ADBMS6948_I2C_FCOMWR_MNACK);

				anWrRegData[nDevIdx][3u] = poWriteCfg[nDevIdx].pnWrDataBuf[0];
				anWrRegData[nDevIdx][4u] = (ADBMS6948_I2C_ICOMWR_START |
		                ADBMS6948_I2C_FCOMWR_MNACK);
				anWrRegData[nDevIdx][5u] = (uint8_t)
                            ((uint8_t)(poWriteCfg[nDevIdx].nI2CDevAddr << 1U)|
                                ADBMS6948_I2C_CTRL_RD);
				poIdxData->pnBytesWritten[nDevIdx] = 3u;
				break;
			default:
				anWrRegData[nDevIdx][0u] = (ADBMS6948_I2C_ICOMWR_START |
												ADBMS6948_I2C_FCOMWR_MNACK);
				anWrRegData[nDevIdx][1u] = (uint8_t)((uint8_t)(poWriteCfg[nDevIdx].nI2CDevAddr << 1U) |
							ADBMS6948_I2C_CTRL_RD);
				anWrRegData[nDevIdx][2u] = (ADBMS6948_I2C_ICOMWR_BLANK |
												ADBMS6948_I2C_FCOMWR_MNACK);
				anWrRegData[nDevIdx][3u] =
				  poWriteCfg[nDevIdx].pnWrDataBuf[poIdxData->pnByteIdx[nDevIdx]++];
				anWrRegData[nDevIdx][4u] = (ADBMS6948_I2C_ICOMWR_BLANK |
													ADBMS6948_I2C_FCOMWR_MNACK);
				anWrRegData[nDevIdx][5u] =
				  poWriteCfg[nDevIdx].pnWrDataBuf[poIdxData->pnByteIdx[nDevIdx]++];
				poIdxData->pnBytesWritten[nDevIdx] = 3u;
				break;
			} /* end of switch */
        poIdxData->pnWrLen[nDevIdx] = 0u;
    	}
    } /* device loop */
}

/*!
    @brief  This function sets no transmit for the i2c write comm register group.

    @param  [in]  nDevIdx  The device index.

    @param  [out]  anWrRegData  Pointer to the write data buffer.

    @return  None.
*/
static void  Adbms6948_lI2CSetNoTx
(
uint8_t   nDevIdx,
uint8_t   anWrRegData[][ADBMS6948_REG_GRP_LEN]
)
{
    anWrRegData[nDevIdx][0u] = ADBMS6948_I2C_ICOMWR_NOTX;
    anWrRegData[nDevIdx][2u] = ADBMS6948_I2C_ICOMWR_NOTX;
    anWrRegData[nDevIdx][4u] = ADBMS6948_I2C_ICOMWR_NOTX;
}

/*!
    @brief  This function is used to copy the config data into the user defined buffer

    @param  [in]  nCmd    		Read config command

    @param  [in]  aRdCfgData    Internal buffer

    @param  [in]  pCfgData     pointer to config data prepared internally

    @param  [in]  pCfgDataChain    pointer to the user buffer where config data is copied

    @param  [in]  nChainIndex    Chain Id

    @return  None.
*/
static void  Adbms6948_lPrepConfigBuff(
uint16_t 			nCmd,
uint8_t				aRdCfgData[][ADBMS6948_MAX_FRAME_SIZE],
uint8_t 		   *pCfgData,
uint8_t 		   *pCfgDataChain,
uint8_t  			nChainIndex
)
{
	uint8_t nDeviceIndex,nNoOfDevices;

	(void)Adbms6948_Cmn_Memset(pCfgDataChain,0, (
	ADBMS6948_REG_GRP_LEN * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN));

	/* number of devices in chain */
	nNoOfDevices = Adbms6948_pDaisyChainCfgInput[nChainIndex].Adbms6948_nNoOfDevices;

	/* For all devices in chain accumulate the configuration data */
	for (nDeviceIndex = 0u; nDeviceIndex < nNoOfDevices ; nDeviceIndex++)
	{
			(void)Adbms6948_Cmn_Memcpy(&pCfgDataChain[nDeviceIndex*ADBMS6948_REG_GRP_LEN],
			pCfgData, ADBMS6948_REG_GRP_LEN);
    }
}

/*!
    @brief  This function returns the FCOMM status for the WRCOMM transaction
    for I2C Read.

    @param  [in]  nRdBytes  Number of bytes read in the I2C transaction.

    @param  [in]  anFCOMM  Array containing the FCOMM data.

    @return  FCOMM status.
*/
static uint8_t Adbms6948_lI2CGetRdStat
(
uint8_t nRdBytes,
uint8_t anFCOMM[]
)
{
    uint8_t nFCOMMStat;

    if(nRdBytes == 0u)
    {
        nFCOMMStat = ADBMS6948_I2C_WRNOTREQ;
    }
    else if( (nRdBytes == 1u) &&
      (anFCOMM[0] == ADBMS6948_I2C_FCOMRD_SNACKMSTOP))
    {
        nFCOMMStat = ADBMS6948_I2C_WRSUCCESS;
    }
    else if( (nRdBytes == 2u) &&
            (anFCOMM[0] == ADBMS6948_I2C_FCOMRD_MACK) &&
                (anFCOMM[1] == ADBMS6948_I2C_FCOMRD_SNACKMSTOP))
    {
        nFCOMMStat = ADBMS6948_I2C_WRSUCCESS;
    }
    else if( (nRdBytes == 3u) && (anFCOMM[0] == ADBMS6948_I2C_FCOMRD_MACK)
           && (anFCOMM[1] == ADBMS6948_I2C_FCOMRD_MACK)
           && ((anFCOMM[2] == ADBMS6948_I2C_FCOMRD_MACK) ||
               (anFCOMM[2] == ADBMS6948_I2C_FCOMRD_SNACKMSTOP))  )
    {
        nFCOMMStat = ADBMS6948_I2C_WRSUCCESS;
    }
    else
    {
        /* Set Read status as failed */
        nFCOMMStat = ADBMS6948_I2C_WRFAILED;
    }
    return nFCOMMStat;
}

/*!
    @brief  This function returns the FCOMM status for the WRCOMM transaction
    for I2C Write.

    @param  [in]  nWrBytes  Number of bytes written in the I2C transaction.

    @param  [in]  anFCOMM  Array containing the FCOMM data.

    @return  FCOMM status.
*/
static uint8_t Adbms6948_lI2CGetWrStat
(
uint8_t nWrBytes,
uint8_t anFCOMM[]
)
{
    uint8_t nFCOMMStat;

    if(nWrBytes == 0u)
    {
        nFCOMMStat = ADBMS6948_I2C_WRNOTREQ;
    }
    else if( (nWrBytes == 1u) &&
            (anFCOMM[0] == ADBMS6948_I2C_FCOMRD_SACKMSTOP))
    {
        nFCOMMStat = ADBMS6948_I2C_WRSUCCESS;
    }
    else if( (nWrBytes == 2u) &&
            (anFCOMM[0] == ADBMS6948_I2C_FCOMRD_SACK) &&
                (anFCOMM[1] == ADBMS6948_I2C_FCOMRD_SACKMSTOP))
    {
        nFCOMMStat = ADBMS6948_I2C_WRSUCCESS;
    }
    else if( (nWrBytes == 3u) && (anFCOMM[0] == ADBMS6948_I2C_FCOMRD_SACK)
            && (anFCOMM[1] == ADBMS6948_I2C_FCOMRD_SACK)
                && ((anFCOMM[2] == ADBMS6948_I2C_FCOMRD_SACK) ||
                    (anFCOMM[2] == ADBMS6948_I2C_FCOMRD_SACKMSTOP) ||
                    /* Case for Repeated start with control word for Read */
                    (anFCOMM[2] == ADBMS6948_I2C_FCOMRD_MACK) ) )
    {
        nFCOMMStat = ADBMS6948_I2C_WRSUCCESS;
    }
    else
    {
        /* Set Write status as failed */
        nFCOMMStat = ADBMS6948_I2C_WRFAILED;
    }
    return nFCOMMStat;
}
/* End of code section */
/* Code section stop */
ADBMS6948_DRV_CODE_STOP

/*! @} */

#endif /* ADI_DISABLE_DOCUMENTATION */

/*! @} */ /* ADI_INTERNAL_INTERFACE */

/*
*
* EOF
*
*/

