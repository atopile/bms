/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Config.cpp
* @brief  : Adbms6948 driver internal source file, this file contains the internal
* interface which will support the device configuration
* @version: $Revision: 553 $
* @date   : $Date: 2020-10-28 16:28:17 +0530 (Wed, 28 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_INTERNAL_INTERFACE	ADBMS6948 Driver Internal Interface
* @{
*/

#ifndef ADI_DISABLE_DOCUMENTATION
/*============= I N C L U D E S =============*/

#include "Adbms6948_Config.h"

/*============= D A T A =============*/
/* Initialized Data section start */
ADBMS6948_DRV_INIT_DATA_START


/** Pointer to global configuration */
const Adbms6948_CfgType *Adbms6948_pConfig = NULL_PTR;

/** This holds the address to daisy chain configuration arrays for all chains */
const Adbms6948_DaisyChainCfgType *Adbms6948_pDaisyChainCfgInput = NULL_PTR;

/* Initialized Data section stop */
ADBMS6948_DRV_INIT_DATA_STOP


/*============ Static Function Prototypes ============*/
static boolean  Adbms6948_lIsInvalidConfig
(
const Adbms6948_CfgType *pConfig
);

static boolean  Adbms6948_lIsAnyOfChainsBusy
(
void
);

static void  Adbms6948_lPrepareChain
(
const uint8_t 				knChainID
);

static Adbms6948_ReturnType Adbms6948_lCfgOC1Params
(
    Adbms6948_OC1CfgType    *poOC1CfgType,
    const uint8_t           knChainID
);

static Adbms6948_ReturnType Adbms6948_lCfgOC2Params
(
    Adbms6948_OC2CfgType    *poOC2CfgType,
    const uint8_t           knChainID
);

/*============= C O D E =============*/
/* Start of code section */

/* Code section start */
ADBMS6948_DRV_CODE_START


#endif /*  ADI_DISABLE_DOCUMENTATION */
/*
 * Function Definition section
*/

/*! \addtogroup ADI_INTERNAL_CONFIGURATION	ADBMS6948 Configuration interface
* @{
*/
/*****************************************************************************/
/************************** CONFIGURATION SERVICES ***************************/
/*****************************************************************************/

/*****************************************************************************/
 /*!
    @brief      This interface performs the initialization

    @param [in]    pkConfig  Pointer to ADBMS6948 configuration set.
							@range: NA
							@resolution: NA

    @return        E_OK: Success  \n
                   E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_Init
(
const Adbms6948_CfgType 	*pkConfig
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
    uint8_t 				nChainIndex;

    if(ADBMS6948_ST_INIT == Adbms6948_eState)
    {
        /* Module is already initialized. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
    	Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
        ADBMS6948_INIT_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if(NULL_PTR == pkConfig)
	{
		/* Null pointer error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_INIT_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if (TRUE == Adbms6948_lIsInvalidConfig(pkConfig))
	{
		/* Null pointer error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_INIT_ID, ADBMS6948_E_INVALID_CONFIG);
		#endif
	}
	else
	{
		/* No error. Set status to 'E_OK'. */
		nRet = E_OK;
		/*Store configuration address. */
		Adbms6948_pConfig = pkConfig;
		Adbms6948_pDaisyChainCfgInput=Adbms6948_pConfig->Adbms6948_pDaisyChainCfg;

		/* Initialize all chain for which initialization is enabled. */
		for (nChainIndex = 0u; nChainIndex < (uint8_t)ADBMS6948_NO_OF_DAISY_CHAIN; nChainIndex++)
		{
			/* Clear the state information and initialize the number of devices in the state for the chain */
			(void) Adbms6948_Cmn_Memset((uint8_t*)&Adbms6948_aoChainStateInfo[nChainIndex],0,
				sizeof(Adbms6948_ChainStateInfoType));

			/* Prepare chain of Adbms6948 and non-Adbms6948 devices */
			Adbms6948_lPrepareChain(nChainIndex);

			/* Initialization is enabled for the chain.*/
			nRet |= Adbms6948_Cmn_InitChain(nChainIndex);
			if (E_OK != nRet)
			{
				/* Chain initialization is failed for the chain. Stop the service. */
				break;
			}

		}

		if (E_OK == nRet)
		{
			/* All the chains initialized successfully. Set module state to initialized. */
			Adbms6948_eState = ADBMS6948_ST_INIT;
		}
		else
		{
			/* Initialization service failed. */
			Adbms6948_pConfig = NULL_PTR;
			Adbms6948_pDaisyChainCfgInput = NULL_PTR;
		}
	}
return (nRet);
}

/*****************************************************************************/
 /*!
    @brief      This interface performs the deinitialization

    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_DeInit
(
void
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
    uint8_t 				nChainIndex;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is already uninitialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_DEINIT_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (TRUE == Adbms6948_lIsAnyOfChainsBusy())
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_DEINIT_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* No error. Set status to 'E_OK'. */
		nRet = E_OK;

		/* DeInitialize all chain for which initialization is enabled. */
		for (nChainIndex = 0u; nChainIndex < (uint8_t)ADBMS6948_NO_OF_DAISY_CHAIN; nChainIndex++)
		{
			if (TRUE == Adbms6948_pConfig->Adbms6948_pDaisyChainCfg[nChainIndex].Adbms6948_bEnableInit)
			{
				/* Issue soft reset.*/
				nRet = Adbms6948_Cmn_SoftReset(nChainIndex);
				if(nRet == E_OK)
				{
					if (TRUE == Adbms6948_Cmn_IsError(nChainIndex))
					{
						/* Chain de-initialization is failed for the chain. Stop the service. */		/* Platform error. */
						nRet |= E_NOT_OK;
						Adbms6948_Cmn_ReportRuntimeErr(Adbms6948_pConfig->Adbms6948_pRunTimeErrCfg->Adbms6948_E_Platform_Error.nErrID,
								ADBMS6948_RUNTIME_E_STATUS_FAILED);
						break;
					}
					else
					{
						/* Clear State information */
						(void)Adbms6948_Cmn_Memset((uint8_t*)&Adbms6948_aoChainStateInfo[nChainIndex],
						0, sizeof(Adbms6948_ChainStateInfoType));
					}
				}
			}
		}

		if (E_OK == nRet)
		{
			/*Set configuration address to Null pointer. */
			Adbms6948_pConfig = NULL_PTR;
			Adbms6948_pDaisyChainCfgInput = NULL_PTR;
			/* All the chains de-initialized successfully. Set module state to de-initialized. */
			Adbms6948_eState = ADBMS6948_ST_UNINIT;

		}
		else
		{
			/* De-Initialization service failed. */
		}
	}
return (nRet);
}

/*****************************************************************************/
/*!
    @brief      This interface enables/disables soak feature for all ADC Aux commands
				on all the devices in the chain.\n

    @param [in]    	pEnable  	Pointer to the soak control values per device.\n
								0 - Disables soak feature.\n
								1 - Enables soak feature.\n
								@range: 0-1
								@resolution: NA

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_CfgSoakControl
(
boolean 		*pEnable,
const uint8_t  	 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint8_t 			aCfgAReg2[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN]={0};
	uint8_t 			nNoOfDevInChain,nDevIdx;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGSOAKCTRL_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR==pEnable) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGSOAKCTRL_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGSOAKCTRL_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare config A register 2 value. */
		nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
		for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
		{
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				aCfgAReg2[nDevIdx] = (uint8_t)((uint8_t)pEnable[nDevIdx] << ADBMS6948_CFGAR2_POS_SOAKON);
			}
		}

		/* Update different value for 'SOAKON' fields on each device over SPI. */
		nRet = Adbms6948_Cmn_UpdateConfigPerDevice(ADBMS6948_CONFIG_GROUP_A, 2u,
		ADBMS6948_CFGAR2_MASK_SOAKON, &aCfgAReg2[0], knChainID);

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
    @brief      This interface configures soak time for auxiliary ADC commands on
				each device in the chain.\n

    @param [in]    pAuxSoakTimeCfg  Pointer to AUX SOAK time configuration
									set. \n
									@range: NA
									@resolution: NA

    @param	[in]   knChainID	    Specifies ID of the daisy chain on
									which the operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_CfgSoakTime
(
Adbms6948_AuxSoakTimeCfgType 	*pAuxSoakTimeCfg,
const uint8_t  					 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint8_t aCfgAReg2[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN] = {0u};
	uint8_t nDevIdx, nMask;
	uint8_t nNoOfDevInChain;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGAUXSOAKTIME_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR == pAuxSoakTimeCfg) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Null pointer error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGAUXSOAKTIME_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGAUXSOAKTIME_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare config A register 0 value. */
		nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
		for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
		{
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				aCfgAReg2[nDevIdx] =
				(uint8_t)((uint8_t)pAuxSoakTimeCfg[nDevIdx].Adbms6948_bRange << ADBMS6948_CFGAR2_POS_OWRNG);
				aCfgAReg2[nDevIdx] |=
				(uint8_t)((uint8_t)pAuxSoakTimeCfg[nDevIdx].Adbms6948_nAuxOWSoakTime << ADBMS6948_CFGAR2_POS_OWA);
			}
		}
		nMask = (uint8_t)(ADBMS6948_CFGAR2_MASK_OWRNG | ADBMS6948_CFGAR2_MASK_OWA);

		/* Update different value for 'OWRNG' and 'OWA' fields on each device over SPI. */
		nRet = Adbms6948_Cmn_UpdateConfigPerDevice(ADBMS6948_CONFIG_GROUP_A, 2u,
		nMask, &aCfgAReg2[0], knChainID);

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
    @brief      This interface configures over voltage threshold for the cell
                voltages.

    @param [in]    pOVThreshold  	Pointer to the over voltage threshold config. \n
									OV Threshold = (OVThreshold*16*150)uV + 1.5V
									@range: 0 to 0xFFF
									@resolution: uV

    @param	[in]   knChainID	    Specifies ID of the daisy chain on
									which the operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_CfgOVThreshold
(
Adbms6948_OVThresCfgType 		*pOVThreshold,
const uint8_t  					 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint8_t 			aMask[2u];
	uint8_t 			nCfgBReg12[2u*ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];
	uint8_t 			nNoOfDevInChain,nDevIdx;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGOVTHRSLD_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR == pOVThreshold) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Null pointer error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
				ADBMS6948_CFGOVTHRSLD_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGOVTHRSLD_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare config B register 1(VOV[3:0]) and 2(VOV[11:4]) value. */
		nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
		for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
		{
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				nCfgBReg12[(nDevIdx*2u)] =
				(uint8_t)(((uint8_t)(pOVThreshold[nDevIdx] & 0x000Fu)) << ADBMS6948_CFGBR1_POS_VOV_LSB);
				nCfgBReg12[(nDevIdx*2u)+1u] = (uint8_t)(((pOVThreshold[nDevIdx] & 0x0FF0u)) >> 4u);
			}
		}

		aMask[0u] = ADBMS6948_CFGBR1_MASK_VOV_LSB;
		aMask[1u] = ADBMS6948_CFGBR2_MASK_VOV_MSB;

		/* Update same value for VOV field on each device over SPI. */
		nRet = Adbms6948_Cmn_UpdateConfig16(ADBMS6948_CONFIG_GROUP_B, 1u,
		aMask, nCfgBReg12, knChainID);

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

		/*Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return (nRet);
}


/*****************************************************************************/
/*!
    @brief      This interface configures under voltage threshold of the cell
                voltages.

    @param 	[in]    pUVThreshold  	Pointer to the under voltage threshold config. \n
									UV Threshold = (UVThreshold*16*150)uV + 1.5V
									@range: 0 to 0xFFF
									@resolution: uV

    @param	[in]   	knChainID	    Specifies ID of the daisy chain on
									which the operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_CfgUVThreshold
(
Adbms6948_UVThresCfgType 		*pUVThreshold,
const uint8_t  					 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint8_t 			aMask[2u];
	uint8_t 			nCfgBReg01[2u*ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];
	uint8_t 			nNoOfDevInChain,nDevIdx;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGUVTHRSLD_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR == pUVThreshold) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Null pointer error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
				ADBMS6948_CFGUVTHRSLD_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGUVTHRSLD_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare config B register 0(VUV[7:0]) and 1(VUV[11:8]) value. */
		nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
		for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
		{
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				nCfgBReg01[(nDevIdx*2u)] = (uint8_t)((uint8_t)(pUVThreshold[nDevIdx] & 0x00FFu));
				nCfgBReg01[(nDevIdx*2u)+1u] = (uint8_t)(((pUVThreshold[nDevIdx] & 0x0F00u)) >> 8u);
			}
		}

		aMask[0u] = ADBMS6948_CFGBR0_MASK_VUV_LSB;
		aMask[1u] = ADBMS6948_CFGBR1_MASK_VUV_MSB;

		/* Update same value for VUV field on each device over SPI. */
		nRet = Adbms6948_Cmn_UpdateConfig16(ADBMS6948_CONFIG_GROUP_B, 0u,
		aMask, nCfgBReg01, knChainID);

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

		/*Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return (nRet);
}

/*****************************************************************************/
/*!
    @brief      This interface sets  the corner frequency for IIR filtering. \n

    @param 	[in]    pCornerFreq  	Pointer to the IIR corner frequency configuration.\n
									@range: 0 to 7
									@resolution: Hz

    @param	[in]   	knChainID	    Specifies ID of the daisy chain on
									which the operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_CfgIIRFilter
(
Adbms6948_IIRCornerFreqType 	*pCornerFreq,
const uint8_t  					 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint8_t 			aCfgAReg5[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN] = {0};
	uint8_t 			nDevIdx;
	uint8_t 			nNoOfDevInChain;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGIIRFILTER_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR == pCornerFreq) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Null pointer error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGIIRFILTER_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGIIRFILTER_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare config A register 0 value. */
		nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
		for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
		{
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				aCfgAReg5[nDevIdx] = (uint8_t)(pCornerFreq[nDevIdx]);
			}
		}
		/* Update different value for FC bits on each device over SPI. */
		nRet = Adbms6948_Cmn_UpdateConfigPerDevice(ADBMS6948_CONFIG_GROUP_A, 5u,
		ADBMS6948_CFGAR5_MASK_FC, &aCfgAReg5[0], knChainID);

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
    @brief      This interface sets the C vs S comparison voltage threshold.\n

    @param 	[in]    pCSCompThresVolt  	Pointer to the CS comparison threshold \n
										config. \n
										@range: 0 to 7
										@resolution: mv

    @param	[in]   	knChainID	    Specifies ID of the daisy chain on
									which the operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_CfgCSCompThresVolt
(
Adbms6948_CSThresVoltType 		*pCSCompThresVolt,
const uint8_t  					 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint8_t 			aCfgAReg0[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN] = {0};
	uint8_t 			nNoOfDevInChain,nDevIdx;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGCSTHRES_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR == pCSCompThresVolt) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Null pointer error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGCSTHRES_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CFGCSTHRES_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare config A register 0 value. */
		nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
		for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
		{
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				aCfgAReg0[nDevIdx] = (uint8_t)(pCSCompThresVolt[nDevIdx]);
			}
		}
		/* Update different value for CTH bitS on each device over SPI. */
		nRet = Adbms6948_Cmn_UpdateConfigPerDevice(ADBMS6948_CONFIG_GROUP_A, 0u,
		ADBMS6948_CFGAR0_MASK_CTH, &aCfgAReg0[0], knChainID);

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
    @brief      This internal interface sets pull-down ON/OFF for the particular
				pin per device. \n

    @param 	[in]    pGpioPinCfg  	Pointer to the GpioPin config for all the
									ADBMS6948 devices \n
									@range: NA
									@resolution: NA

    @param	[in]   	knChainID	    Specifies ID of the daisy chain on
									which the operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_CfgGpioPins
(
Adbms6948_GpioPinCfgType 	*pGpioPinCfg,
const uint8_t 				 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint8_t 			aMask[2u];
	uint8_t 			nCfgBReg34[2u*ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];
	uint8_t 			nNoOfDevInChain,nDevIdx;


	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGGPIO_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR == pGpioPinCfg) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID) )
	{
		/* Null pointer error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGGPIO_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID,
		ADBMS6948_CFGGPIO_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare config B register 0(VUV[7:0]) and 1(VUV[11:8]) value. */
		nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
		for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
		{
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				nCfgBReg34[(nDevIdx*2u)] = (uint8_t)((uint8_t)(pGpioPinCfg[nDevIdx] & 0x00FFu));
				nCfgBReg34[(nDevIdx*2u)+ 1u] = (uint8_t)(((pGpioPinCfg[nDevIdx] & 0x0300u)) >> 8u);
			}
		}

		aMask[0u] = ADBMS6948_CFGAR3_MASK_GPO;
		aMask[1u] = ADBMS6948_CFGAR4_MASK_GPO;

		/* Update same value for GPO field on each device over SPI. */
		nRet = Adbms6948_Cmn_UpdateConfig16(ADBMS6948_CONFIG_GROUP_A, 3u, aMask,
		nCfgBReg34, knChainID);

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

		/*Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return (nRet);
}

/*****************************************************************************/
/*!
    @brief          This function configures the Time base of the Coulomb
                    Counter.
    @param  [in]    pnNumConversions    Pointer to the Time base config
                                        @range: 0 to 65535
                                        @resolution: 1 conversion time

    @param	[in]   	knChainID	        Specifies ID of the daisy chain on
									    which the operation is to be performed.
									    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									    @resolution: NA

    @return         E_OK: 		Success \n
                    E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_CfgCoulombCounter
(
    uint16_t            *pnNumConversions,
    const uint8_t       knChainID
)
{
    Adbms6948_ReturnType    nRet = E_NOT_OK;
    uint8_t                 aMask[2u];
    uint8_t                 nCfgCReg01[2u];

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CFGCCNTR_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if ((NULL_PTR == pnNumConversions) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
    {
        /* Null pointer error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CFGCCNTR_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CFGCCNTR_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        /* Prepare config C register 0(TCC[7:0]) and 1(TCC[11:8]) value. */
        nCfgCReg01[0u] = (uint8_t)((uint8_t)(pnNumConversions[0u] & 0x00FFu));
        nCfgCReg01[1u] = (uint8_t)(((pnNumConversions[0u] & 0xFF00u)) >> 8u);

        aMask[0u] = ADBMS6948_CFGCR0_MASK_TCC_LSB;
        aMask[1u] = ADBMS6948_CFGCR0_MASK_TCC_MSB;

        /* Update same value for TCC field over SPI. */
        nRet = Adbms6948_Cmn_UpdateConfig16(ADBMS6948_CONFIG_GROUP_C, 0u, aMask, nCfgCReg01, knChainID);

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
return (nRet);
}

/*****************************************************************************/
/*!
    @brief          This functions configures the parameters of the OC1
                    algorithm.


    @param  [in]    poOC1CfgType    Pointer to specify the OC1 algorithm
                                    parameters.
                                    @range: NA
                                    @resolution: NA
    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA

    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_CfgOC1Params
(
    Adbms6948_OC1CfgType    *poOC1CfgType,
    const uint8_t           knChainID
)
{
    Adbms6948_ReturnType    nRet = E_NOT_OK;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CFGOC1PARAMS_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if ((NULL_PTR == poOC1CfgType) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
    {
        /* Null pointer error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CFGOC1PARAMS_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CFGOC1PARAMS_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        nRet = Adbms6948_lCfgOC1Params(poOC1CfgType, knChainID);


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
return (nRet);
}

/*****************************************************************************/
/*!
    @brief          This function configures the OC2 Algorithm parameters in
                    the ADBMS6948 device.

    @param  [in]    poOC2CfgType    Pointer to specify the OC2 algorithm
                                    parameters.
                                    @range: NA
                                    @resolution: NA
    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA

    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_CfgOC2Params
(
    Adbms6948_OC2CfgType    *poOC2CfgType,
    const uint8_t           knChainID
)
{
    Adbms6948_ReturnType    nRet = E_NOT_OK;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CFGOC2PARAMS_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if ((NULL_PTR == poOC2CfgType) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
    {
        /* Null pointer error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CFGOC2PARAMS_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (ADBMS6948_DEVELOPMENT_MODE_EN == TRUE)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CFGOC2PARAMS_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        nRet = Adbms6948_lCfgOC2Params(poOC2CfgType, knChainID);

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
return (nRet);
}
/*! @}*/

/*! @}*/

#ifndef ADI_DISABLE_DOCUMENTATION
/*!
    @brief  This function validates the initial configurations.

    @param  [in]   pConfig        Pointer to configuration set.

    @return    True : Invalid value for configuration/s. \n
               False: Valid value for configuration/s. \n
*/
static boolean  Adbms6948_lIsInvalidConfig
(
    const Adbms6948_CfgType *   pConfig
)
{
    boolean  bRet = FALSE;
    uint8_t  nChainIndex,nDeviceIndex,nAdbms6948Devices;

    for (nChainIndex = 0u; nChainIndex < ADBMS6948_NO_OF_DAISY_CHAIN; nChainIndex++)
    {
    	nAdbms6948Devices = pConfig->Adbms6948_pDaisyChainCfg[nChainIndex].Adbms6948_nNoOfDevices;

        if (nAdbms6948Devices > ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)
        {
            bRet = TRUE;
        }
        else
        {
            for (nDeviceIndex=0u;nDeviceIndex<nAdbms6948Devices;nDeviceIndex++)
            {
                 if (pConfig->Adbms6948_pDaisyChainCfg[nChainIndex].Adbms6948_pNoOfCellsPerDevice[nDeviceIndex] >
                       ADBMS6948_MAX_NO_OF_CELLS_PER_DEVICE)
                 {
                      bRet = TRUE;
                      break;
                 }
            }
        }

        if (TRUE == bRet)
        {
                break;
        }
     }
return bRet;
}

/*!
    @brief  This prepare the chain of Adbms6948 and non-Adbms6948 devices.

    @param  [in]   knChainID        Chain Id

    @return    None
*/
static void  Adbms6948_lPrepareChain
(
    const uint8_t 				knChainID
)
{
	uint8_t nNoOfDevices,nDeviceIndex;

	/* Identify the total number of devices in chain */
	nNoOfDevices =
	Adbms6948_pDaisyChainCfgInput[knChainID].Adbms6948_nNoOfDevices;

	/* Prepare the list of Devices in chain */
	for (nDeviceIndex=0u;nDeviceIndex<nNoOfDevices;nDeviceIndex++)
	{
		Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDeviceIndex]=
		ADBMS6948_DEVICE;
	}
}

/*!
    @brief  This checks if any of the chain is in busy state.

    @return        TRUE: One or more chain/s are in busy state  \n
                   FALSE:  \n
*/
static boolean  Adbms6948_lIsAnyOfChainsBusy
(
    void
)
{
    boolean     bRet = FALSE;
    uint8_t      nChainIndex;

	Adbms6948_Cmn_EnterCriticalSection();
    for (nChainIndex = 0u; nChainIndex < ADBMS6948_NO_OF_DAISY_CHAIN; nChainIndex++)
    {
        if (ADBMS6948_ST_CHAIN_BUSY == Adbms6948_aoChainStateInfo[nChainIndex].eChainState)
        {
            bRet = TRUE;
            break;
        }
    }
   Adbms6948_Cmn_ExitCriticalSection();
return (bRet);
}

/*!
    @brief          This local function configures the parameters for OC1
                    algorithms. It also reads back the values and verifies that


    @param  [in]    poOC1CfgType    Pointer to specify the OC1 algorithm
                                    parameters.
                                    @range: NA
                                    @resolution: NA
    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA

    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
static Adbms6948_ReturnType Adbms6948_lCfgOC1Params
(
    Adbms6948_OC1CfgType    *poOC1CfgType,
    const uint8_t           knChainID
)
{
    Adbms6948_ReturnType nRet;
	uint8_t  nCfgDData[ADBMS6948_REG_GRP_LEN] = {0};
	uint8_t  nCfgEData[ADBMS6948_REG_GRP_LEN] = {0};
	uint8_t  nCfgFData[ADBMS6948_REG_GRP_LEN] = {0};
    uint8_t anReadDataBuf[ADBMS6948_REG_DATA_LEN_WITH_PEC] = {0};

    /* Prepare config D register values. */
    nCfgDData[0] =  ((poOC1CfgType->Adbms6948_nCyclesW1 & 0x1F) << ADBMS6948_CFGDR0_POS_CYCLES_W1) |
                    ((poOC1CfgType->Adbms6948_nCyclesW2 & 0x07) << ADBMS6948_CFGDR0_POS_CYCLES_W2);
    nCfgDData[1] =  ((poOC1CfgType->Adbms6948_nCyclesW2 & 0x18) >> 0x03) <<  ADBMS6948_CFGDR1_POS_CYCLES_W2 |
                    ((poOC1CfgType->Adbms6948_nCyclesW3 & 0x1F) << ADBMS6948_CFGDR1_POS_CYCLES_W3) |
                    ((poOC1CfgType->Adbms6948_nCyclesW4 & 0x01) << ADBMS6948_CFGDR1_POS_CYCLES_W4);
    nCfgDData[2] =  ((poOC1CfgType->Adbms6948_nCyclesW4 & 0x1E) >> 0x01) |
                    ((poOC1CfgType->Adbms6948_nCyclesW5 & 0x0F) << ADBMS6948_CFGDR2_POS_CYCLES_W5);
    nCfgDData[3] =  ((poOC1CfgType->Adbms6948_nCyclesW5 & 0x10) >> 0x04) |
                    ((poOC1CfgType->Adbms6948_nCyclesW6 & 0x1E) << ADBMS6948_CFGDR3_POS_CYCLES_W6);
    nCfgDData[4] =  ((poOC1CfgType->Adbms6948_nWarnCfg & 0x1F) << ADBMS6948_CFGDR4_POS_WARN_CFG);
    nCfgDData[5] =  (poOC1CfgType->Adbms6948_nGainCtrl1 & 0xFF);

    /* Prepare config E register values. */
    for(int i = 0; i < ADBMS6948_REG_GRP_LEN; ++i)
    {
        nCfgEData[i] = poOC1CfgType->Adbms6948_panOC1Warn[i] & 0xFF;
    }

    /*Prepare config F register values. */
    for(int i = 0; i < ADBMS6948_REG_GRP_LEN; ++i)
    {
        nCfgFData[i] = poOC1CfgType->Adbms6948_panOC1Thresh[i] & 0xFF;
    }

    /* Write the configuration D to the device */
    Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCFGD, &nCfgDData[0], knChainID);
    /* Write the configuration E to the device */
    Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCFGE, &nCfgEData[0], knChainID);
    /* Write the configuration F to the device */
    Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCFGF, &nCfgFData[0], knChainID);

    /*Read CONFIG D register and compare the values*/
    nRet = Adbms6948_Cmn_ReadAllRegGroups(ADBMS6948_CMD_RDCFGD, &anReadDataBuf[0], ADBMS6948_REG_DATA_LEN_WITH_PEC, knChainID);
    if (E_OK == nRet)
    {
        for(int i = 0; i < ADBMS6948_REG_DATA_LEN_WITHOUT_PEC; ++i)
        {
            if (nCfgDData[i] != anReadDataBuf[i])
            {
                nRet = E_NOT_OK;
                break;
            }
        }
    }
    if (E_OK == nRet)
    {
        /*Read CONFIG E register and compare the values*/
        nRet = Adbms6948_Cmn_ReadAllRegGroups(ADBMS6948_CMD_RDCFGE, &anReadDataBuf[0], ADBMS6948_REG_DATA_LEN_WITH_PEC, knChainID);
        if (E_OK == nRet)
        {
            for(int i = 0; i < ADBMS6948_REG_DATA_LEN_WITHOUT_PEC; ++i)
            {
                if (nCfgEData[i] != anReadDataBuf[i])
                {
                    nRet = E_NOT_OK;
                    break;
                }
            }
        }
    }
    if (E_OK == nRet)
    {
        /*Read CONFIG F register and compare the values*/
        nRet = Adbms6948_Cmn_ReadAllRegGroups(ADBMS6948_CMD_RDCFGF, &anReadDataBuf[0], ADBMS6948_REG_DATA_LEN_WITH_PEC, knChainID);
        if (E_OK == nRet)
        {
            for(int i = 0; i < ADBMS6948_REG_DATA_LEN_WITHOUT_PEC; ++i)
            {
                if (nCfgFData[i] != anReadDataBuf[i])
                {
                    nRet = E_NOT_OK;
                    break;
                }
            }
        }
    }

    return nRet;
}

/*!
    @brief          This local function configures the OC2 parameters. It also
                    reads back the parameters from the BMS device after writing
                    to ensure whether correct values were written.


    @param  [in]    poOC2CfgType    Pointer to specify the OC2 algorithm
                                    parameters.
                                    @range: NA
                                    @resolution: NA
    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA

    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
static Adbms6948_ReturnType Adbms6948_lCfgOC2Params
(
    Adbms6948_OC2CfgType    *poOC2CfgType,
    const uint8_t           knChainID
)
{
    Adbms6948_ReturnType nRet;
	uint8_t  nCfgGData[ADBMS6948_REG_GRP_LEN] = {0};
	uint8_t  nCfgHData[ADBMS6948_REG_GRP_LEN] = {0};
	uint8_t  nCfgIData[ADBMS6948_REG_GRP_LEN] = {0};
    uint8_t  anReadDataBuf[ADBMS6948_REG_DATA_LEN_WITH_PEC] = {0};

    /* Prepare config G register values. */
    for(int i = 0; i < ADBMS6948_REG_GRP_LEN; ++i)
    {
        nCfgGData[i] = poOC2CfgType->Adbms6948_panOC2RCTimeConstant[i];
    }

    /* Prepare config H register values. */
    for(int i = 0; i < ADBMS6948_REG_GRP_LEN-1; ++i)
    {
        nCfgHData[i] = poOC2CfgType->Adbms6948_panOC2RCTimeConstant[i+ADBMS6948_REG_GRP_LEN];
    }
    nCfgHData[5] = (poOC2CfgType->Adbms6948_nMaxAmbientTemp & 0x3F);

    /* Prepare config I register values. */
    nCfgIData[0] = poOC2CfgType->Adbms6948_nOCThreshold & 0xFF;
    nCfgIData[1] = poOC2CfgType->Adbms6948_nOCThreshWarning & 0xFF;
    nCfgIData[2] = STATUS_CFG_VAL & 0xFF;
    nCfgIData[3] = poOC2CfgType->Adbms6948_nGainCtrl2 & 0xFF;

    /* Write the configuration G to the device */
    Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCFGG, &nCfgGData[0], knChainID);
    /* Write the configuration H to the device */
    Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCFGH, &nCfgHData[0], knChainID);
    /* Write the configuration I to the device */
    Adbms6948_Cmd_ExecuteCmdWR(ADBMS6948_CMD_WRCFGI, &nCfgIData[0], knChainID);

    /*Read CONFIG G register and compare the values*/
    nRet = Adbms6948_Cmn_ReadAllRegGroups(ADBMS6948_CMD_RDCFGG, &anReadDataBuf[0], ADBMS6948_REG_DATA_LEN_WITH_PEC, knChainID);
    if (E_OK == nRet)
    {
        for(int i = 0; i < ADBMS6948_REG_DATA_LEN_WITHOUT_PEC; ++i)
        {
            if (nCfgGData[i] != anReadDataBuf[i])
            {
                nRet = E_NOT_OK;
                break;
            }
        }
    }
    if (E_OK == nRet)
    {
        /*Read CONFIG H register and compare the values*/
        nRet = Adbms6948_Cmn_ReadAllRegGroups(ADBMS6948_CMD_RDCFGH, &anReadDataBuf[0], ADBMS6948_REG_DATA_LEN_WITH_PEC, knChainID);
        for(int i = 0; i < ADBMS6948_REG_DATA_LEN_WITHOUT_PEC; ++i)
        {
            if (nCfgHData[i] != anReadDataBuf[i])
            {
                nRet = E_NOT_OK;
                break;
            }
        }
    }
    if (E_OK == nRet)
    {
        /*Read CONFIG I register and compare the values*/
        nRet = Adbms6948_Cmn_ReadAllRegGroups(ADBMS6948_CMD_RDCFGI, &anReadDataBuf[0], ADBMS6948_REG_DATA_LEN_WITH_PEC, knChainID);
        for(int i = 0; i < ADBMS6948_REG_DATA_LEN_WITHOUT_PEC; ++i)
        {
            if (nCfgIData[i] != anReadDataBuf[i])
            {
                nRet = E_NOT_OK;
                break;
            }
        }
    }
    return nRet;
}

/* End of code section */
/* Code section stop */
ADBMS6948_DRV_CODE_STOP

#endif /* ADI_DISABLE_DOCUMENTATION */

/*
*
* EOF
*
*/

