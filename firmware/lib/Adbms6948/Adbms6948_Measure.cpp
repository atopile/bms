/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Measure.cpp
* @brief  : Internal source file containing the measurement functions.
* @version: $Revision: 528 $
* @date   : $Date: 2020-10-13 19:41:33 +0530 (Tue, 13 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/
/*! \addtogroup ADI_INTERNAL_INTERFACE	ADBMS6948 Driver Internal Interface
* @{
*/
#ifndef ADI_DISABLE_DOCUMENTATION
/*============= I N C L U D E S =============*/
#include "Adbms6948_Measure.h"
/*============= D A T A =============*/
/* Const 16 section start */
ADBMS6948_DRV_CONST_DATA_16_START

/*!
    @brief          Array containing the read cell commands for C voltages,
                    S voltages, Average Voltages and Filtered Cell Voltages.

 */
static const uint16_t  Adbms6948_CellDataReadCmds[ADBMS6948_NUMOF_CELLDATA_TYPES][ADBMS6948_MAX_CELLDATA_REGGRPS+1] = {
	{ ADBMS6948_CMD_RDCVA, ADBMS6948_CMD_RDCVB, ADBMS6948_CMD_RDCVC, ADBMS6948_CMD_RDCVD, ADBMS6948_CMD_RDCVE, ADBMS6948_CMD_RDCVF, ADBMS6948_CMD_RDCVALL },
	{ ADBMS6948_CMD_RDSVA, ADBMS6948_CMD_RDSVB, ADBMS6948_CMD_RDSVC, ADBMS6948_CMD_RDSVD, ADBMS6948_CMD_RDSVE, ADBMS6948_CMD_RDSVF, ADBMS6948_CMD_RDSVALL },
	{ ADBMS6948_CMD_RDACA, ADBMS6948_CMD_RDACB, ADBMS6948_CMD_RDACC, ADBMS6948_CMD_RDACD, ADBMS6948_CMD_RDACE, ADBMS6948_CMD_RDACF, ADBMS6948_CMD_RDACALL },
	{ ADBMS6948_CMD_RDFCA, ADBMS6948_CMD_RDFCB, ADBMS6948_CMD_RDFCC, ADBMS6948_CMD_RDFCD, ADBMS6948_CMD_RDFCE, ADBMS6948_CMD_RDFCF, ADBMS6948_CMD_RDFCALL }
};

/*!
    @brief          Array containing the commands to read GPIO and Redundant
                    GPIO registers.

 */
static const uint16_t  Adbms6948_GpioDataReadCmds[ADBMS6948_NUMOF_GPIODATA_TYPES][ADBMS6948_MAX_GPIODATA_REGGRPS] = {
	{ ADBMS6948_CMD_RDAUXA, ADBMS6948_CMD_RDAUXB, ADBMS6948_CMD_RDAUXC, ADBMS6948_CMD_RDAUXD },
	{ ADBMS6948_CMD_RDRAXA, ADBMS6948_CMD_RDRAXB, ADBMS6948_CMD_RDRAXC, ADBMS6948_CMD_RDRAXD }
};

/*!
    @brief          Array containing the commands to read Aux registers
                    (Status F, Status A and Status B registers respectively).

 */
static const uint16_t  Adbms6948_DevParamReadCmds[ADBMS6948_MAX_DEV_PARAM_REGGRPS] = {
	ADBMS6948_CMD_RDSTATF, ADBMS6948_CMD_RDSTATA, ADBMS6948_CMD_RDSTATB
};

/*!
    @brief          Array containing the number of bytes (including PEC) read
                    using "Read All* commands.

 */
static const uint8_t Adbms6948_ReadAllCommandBytes[ADBMS6948_READALL_COMMAND_TYPES] = {
	ADBMS6948_RDALL_CELLVOLTAGES_BYTES,
	ADBMS6948_RDALL_VOLT_CURRENT_BYTES,
	ADBMS6948_RDALL_CSVOLTAGES_BYTES,
	ADBMS6948_RDALL_CSVOLTAGE_CURRENT_BYTES,
	ADBMS6948_RDALL_AUX_STATUS_BYTES
};

#define ADBMS6948_READ
/* Const 16 section stop */
ADBMS6948_DRV_CONST_DATA_16_STOP

/*============= C O D E =============*/
/* Start of code section */
/* Code section start */
ADBMS6948_DRV_CODE_START

static Adbms6948_ReturnType  Adbms6948_lReadCADCConvCnt
(
	uint16_t 				*panConvCnt,
	Adbms6948_SnapSelType 	 eSnapSel,
	const uint8_t  			 knChainID
);
static Adbms6948_ReturnType  Adbms6948_lReadI1ADCConvCnt
(
	uint16_t 				*panConvCnt,
	Adbms6948_SnapSelType 	 eSnapSel,
	const uint8_t  			 knChainID
);
static Adbms6948_ReturnType  Adbms6948_lReadVoltageCurrentCTValues
(
    uint16_t                *pnVoltageCT,
    uint16_t                *pnCurrentCT,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
);
static void  Adbms6948_lGetCellCmdList
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
	Adbms6948_CellGrpSelType  		eCellGrpSel,
	uint16_t 					   *panCmdList,
	uint8_t 					   *pnCmdListCnt,
	const uint8_t  				    knChainID
);
static void  Adbms6948_lGetCellCmd
(
	Adbms6948_CellMeasDataType      eCellMeasData,
	Adbms6948_CellGrpSelType  	    eCellGrpSel,
	uint16_t                        *pnCmd,
	const uint8_t  					 knChainID
);
static Adbms6948_ReturnType  Adbms6948_lReadCellVolt
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
	Adbms6948_CellGrpSelType  		eCellGrpSel,
	int16_t 					   *pnCellData,
	Adbms6948_SnapSelType 			eSnapSel,
	const uint8_t  					knChainID
);
static Adbms6948_ReturnType  Adbms6948_lReadAllCellVolt
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
	int16_t 					    *pnCellData,
	Adbms6948_SnapSelType 			eSnapSel,
    const uint8_t                   knChainID
);
static Adbms6948_ReturnType  Adbms6948_lReadAllAverageCellVoltages
(
	int16_t 					   *pnCellData,
	const uint8_t  					knChainID
);
static Adbms6948_ReturnType  Adbms6948_lReadCurrent
(
	Adbms6948_CurrentMeasDataType	eCurrentMeasData,
	Adbms6948_CurrentSelType		eCurrentSel,
	int32_t						*pnCurrentData,
	Adbms6948_SnapSelType			eSnapSel,
	const uint8_t					knChainID
);
static Adbms6948_ReturnType Adbms6948_lReadCADCI1ADC
(
	int16_t				            *pnCellData,
	int32_t				            *pnCurrentData,
	Adbms6948_SnapSelType	eSnapSel,
	const uint8_t			knChainID
);
static Adbms6948_ReturnType  Adbms6948_lReadCellVoltCurrentAverage
(
	int16_t 					    *pnCellData,
    int32_t                         *pnCurrentData,
    boolean                         bSendUnsnapAtEnd,
	uint16_t						*pnVoltageCTValue,
	uint16_t						*pnCurrentCTValue,
	const uint8_t  					knChainID
);
static void  Adbms6948_lGetGPIOCmdList
(
	Adbms6948_GpioMeasDataType   eGpioMeasData,
	Adbms6948_GpioGrpSelType  	 eGpioGrpSel,
	uint16_t 					*panCmdList,
	uint8_t 					*pnCmdListCnt
);
static Adbms6948_ReturnType  Adbms6948_lReadGPIOInputVolt
(
	Adbms6948_GpioMeasDataType  	eGpioMeasData,
	Adbms6948_GpioGrpSelType  		eGpioGrpSel,
	int16_t 					   *pnGpioData,
	const uint8_t  					knChainID
);
static void  Adbms6948_lGetDevParamCmdList
(
	Adbms6948_DevParamGrpSelType  	 eDevParamGrpSel,
	uint16_t 						*panCmdList,
	uint8_t 						*pnCmdListCnt
);
static Adbms6948_ReturnType  Adbms6948_lReadDevParam
(
	Adbms6948_DevParamGrpSelType  	 eDevParamGrpSel,
	int16_t 						*pnDevParamData,
	const uint8_t  					 knChainID
);
static Adbms6948_ReturnType  Adbms6948_lReadAllAuxStatusRegisters
(
    int16_t                         *panGpioData,
    int16_t                         *panRedundantGpioData,
    Adbms6948_StatusRegValsType     *poStatusRegsVal,
    const uint8_t                    knChainID
);
static void  Adbms6948_lGetCtrlCmdCode
(
Adbms6948_CtrlCmdSelType     eCtrlCmdSel,
uint16_t 					*pCtrlCmdCode,
boolean 					*pbIsIncrCmd
);
static void  Adbms6948_lGetPollADCCmdCode
(
Adbms6948_ADCSelType  	 eADCSel,
uint16_t 				*pPollAdcCmdCode
);
static Adbms6948_ReturnType  Adbms6948_lPollADCStatus
(
	uint16_t 		 nPollCmd,
	boolean 		*pbADCStatus,
	const uint8_t  	 knChainID
);
static void Adbms6948_lSendSnap
(
	const uint8_t  	 knChainID
);
static void Adbms6948_lSendUnsnap
(
	const uint8_t  	 knChainID
);
static Adbms6948_ReturnType Adbms6948_lReadCoulombCountAndTimeBase
(
    uint32_t                *pnCoulombCount,
    uint16_t                *pnTimeBase,
	Adbms6948_SnapSelType   eSnapSel,
    const uint8_t           knChainID
);
static Adbms6948_ReturnType Adbms6948_lClearFlags
(
    const uint8_t                   knChainID
);
static Adbms6948_ReturnType Adbms6948_lClearCellVoltageRegisters
(
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
);
static Adbms6948_ReturnType Adbms6948_lClearCurrentRegisters
(
    Adbms6948_CurrentSelType        eCurrentSel,
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
);
static boolean  Adbms6948_lIsInvalidCellOWSelType
(
    Adbms6948_CellOWSelType      eOWSel
);
static boolean  Adbms6948_lIsInvalidAuxChSelType
(
    Adbms6948_AuxChSelType      eAuxChSel
);
static boolean  Adbms6948_lIsInvalidRedAuxChSelType
(
    Adbms6948_RedAuxChSelType      eRedAuxChSel
);
static boolean  Adbms6948_lIsInvalidCellMeasDataType
(
    Adbms6948_CellMeasDataType      eCellMeasData
);
static boolean  Adbms6948_lIsInvalidCellGrpSelType
(
    Adbms6948_CellGrpSelType      eCellGrpSel
);
static boolean  Adbms6948_lIsInvalidCurrentMeasDataType
(
    Adbms6948_CurrentMeasDataType      eCurrentMeasData
);
static boolean  Adbms6948_lIsInvalidCurrentSelType
(
    Adbms6948_CurrentSelType      eCurrentSel
);
static boolean  Adbms6948_lIsInvalidGpioMeasDataType
(
    Adbms6948_GpioMeasDataType      eGpioMeasData
);
static boolean  Adbms6948_lIsInvalidGpioGrpSelType
(
    Adbms6948_GpioGrpSelType      eGpioGrpSel
);
static boolean  Adbms6948_lIsInvalidDevParamGrpSelType
(
    Adbms6948_DevParamGrpSelType      eDevParamGrpSel
);
static boolean  Adbms6948_lIsInvalidCtrlCmdSelType
(
    Adbms6948_CtrlCmdSelType      eCtrlCmdSel
);
static boolean  Adbms6948_lIsInvalidADCSelType
(
    Adbms6948_ADCSelType      eADCSel
);
static Adbms6948_ReturnType  Adbms6948_lReadCellOVUVStatus
(
	uint32_t 				*pnCellOVUVStatus,
	Adbms6948_SnapSelType 	 eSnapSel,
	const uint8_t  			 knChainID
);
#endif /*  ADI_DISABLE_DOCUMENTATION */
/*
 * Function Definition section
*/

/*! \addtogroup ADI_INTERNAL_MEASUREMENT	ADBMS6948 Measurement interface
* @{
*/
/*****************************************************************************/
 /*!
    @brief	This function triggers CADC conversions on all ADBMS6948 devices
			in the specified chain.
    @param [in] poTrigCADCIp Pointer to the structure containing the C-ADC
							 trigger parameter.
							 @range: NA
							 @resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_TrigCADC
(
    Adbms6948_TrigCADCInputs 	*poTrigCADCIp,
	const uint8_t  				 knChainID
)
{
	Adbms6948_ReturnType nRet = E_NOT_OK;
	uint16_t  nCmdCode;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGCADC_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == poTrigCADCIp)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGCADC_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGCADC_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGCADC_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare the ADCV command based on the user inputs */
		nCmdCode = ADBMS6948_CMD_ADCV;
		nCmdCode =  (uint16_t)( nCmdCode | (uint16_t)poTrigCADCIp->Adbms6948_eOWSel);
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigCADCIp->Adbms6948_bRstIIRFltr << ADBMS6948_ADCV_POS_RSTF));
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigCADCIp->Adbms6948_bEnSSDP << ADBMS6948_ADCV_POS_SSDP));
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigCADCIp->Adbms6948_bContMode << ADBMS6948_ADCV_POS_CONT));
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigCADCIp->Adbms6948_bEnRedun << ADBMS6948_ADCV_POS_RD));

		/* Send the ADCV command */
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
		nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	This function triggers SADC conversions on all ADBMS6948 devices
			in the specified chain.
    @param [in] bContMode	Specifies SADC conversion mode.
							@range: FALSE,TRUE
							@resolution: NA
    @param [in] bEnSSDP		Specifies discharge permission during Single Shot
							SADC conversion.
							@range: FALSE,TRUE
							@resolution: NA
    @param [in] eOWSel		Open wire switch selection for SADC.
							@range: Enumeration
							@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_TrigSADC
(
    boolean 						bContMode,
	boolean 						bEnSSDP,
	Adbms6948_CellOWSelType  	    eOWSel,
	const uint8_t 					knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint16_t  nCmdCode;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGSADC_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((TRUE == Adbms6948_lIsInvalidCellOWSelType(eOWSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGSADC_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGSADC_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare the ADSV command based on the user inputs */
		nCmdCode = ADBMS6948_CMD_ADSV;
		nCmdCode =  (uint16_t)(nCmdCode | (uint16_t)eOWSel);
		nCmdCode =  (uint16_t)(nCmdCode | ((uint16_t)bEnSSDP << ADBMS6948_ADSV_POS_SSDP));
		nCmdCode =  (uint16_t)(nCmdCode | ((uint16_t)bContMode << ADBMS6948_ADSV_POS_CONT));

		/* Send the ADSV command */
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
		nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}

/*****************************************************************************/
 /*!
    @brief	This function triggers the auxiliary ADC conversions on all the
			ADBMS6948 devices in the specified chain.
    @param [in] bOWSel		The OW switch selection for AuxADC conversion.
							@range: FALSE,TRUE
							@resolution: NA
    @param [in] bCurrentSel	To specify current selection for OW conversion.
							@range: FALSE,TRUE
							@resolution: NA
    @param [in] eAuxChSel	Auxiliary input channel selection for AUXADC.
							@range: Enumeration
							@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_TrigAuxADC
(
    boolean 					bOWSel,
	boolean  					bCurrentSel,
	Adbms6948_AuxChSelType  	eAuxChSel,
	const uint8_t  				knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint16_t  nCmdCode;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGAUXADC_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((TRUE == Adbms6948_lIsInvalidAuxChSelType(eAuxChSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGAUXADC_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGAUXADC_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare the ADAX command based on the user inputs */
		nCmdCode = ADBMS6948_CMD_ADAX;
		nCmdCode =  (uint16_t)(nCmdCode | (uint16_t)eAuxChSel) ;
		nCmdCode =  (uint16_t)(nCmdCode | ((uint16_t)bCurrentSel << ADBMS6948_ADAX_POS_PUP));
		nCmdCode =  (uint16_t)(nCmdCode | ((uint16_t)bOWSel << ADBMS6948_ADAX_POS_OW));
		/* Send the ADAX command */
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
		nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
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
		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
 /*!
    @brief		This function triggers the redundant auxiliary ADC conversions on
				all the ADBMS6948 devices in the specified chain.

    @param [in] eRedAuxChSel	Auxiliary input channel selection for Redundant
								AUXADC.
								@range: Enumeration.
								@resolution: NA

	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA

    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_TrigRedAuxADC
(
	Adbms6948_RedAuxChSelType  eRedAuxChSel,
	const uint8_t  			   knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint16_t  nCmdCode;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGREDAUXADC_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((TRUE == Adbms6948_lIsInvalidRedAuxChSelType(eRedAuxChSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGREDAUXADC_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGREDAUXADC_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare the ADAX2 command based on the user inputs */
		nCmdCode = ADBMS6948_CMD_ADAX2;
		nCmdCode =  (uint16_t)(nCmdCode | (uint16_t)eRedAuxChSel);

		/* Send the ADAX2 command */
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
		nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	This function triggers I1-ADC conversions on all ADBMS6948 devices
			in the specified chain.
    @param [in] poTrigI1ADCIp 	Pointer to the structure containing the I1-ADC
							 	trigger parameter.
							 	@range: NA
							 	@resolution: NA
	@param [in] knChainID		Specifies ID of the daisy chain on which the
	                        	operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_TrigI1ADC
(
    Adbms6948_TrigI1ADCInputs 	*poTrigI1ADCIp,
	const uint8_t  				 knChainID
)
{
	Adbms6948_ReturnType nRet = E_NOT_OK;
	uint16_t  nCmdCode;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI1ADC_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == poTrigI1ADCIp)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI1ADC_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI1ADC_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI1ADC_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare the ADI1 command based on the user inputs */
		nCmdCode = ADBMS6948_CMD_ADI1;
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigI1ADCIp->Adbms6948_eOWSel << ADBMS6948_ADI1_POS_OW));
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigI1ADCIp->Adbms6948_bContMode << ADBMS6948_ADI1_POS_CONT));
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigI1ADCIp->Adbms6948_bEnRedun << ADBMS6948_ADI1_POS_RD));

		/* Send the ADI1 command */
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
		nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	This function triggers I2-ADC conversions on all ADBMS6948 devices
			in the specified chain.
    @param [in] poTrigI2ADCIp	Pointer to the structure containing the I2-ADC
								trigger parameter.
								@range: NA
								@resolution: NA
	@param [in] knChainID		Specifies ID of the daisy chain on which the
	                        	operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_TrigI2ADC
(
    Adbms6948_TrigI2ADCInputs 	*poTrigI2ADCIp,
	const uint8_t  				 knChainID
)
{
	Adbms6948_ReturnType nRet = E_NOT_OK;
	uint16_t  nCmdCode;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI2ADC_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == poTrigI2ADCIp)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI2ADC_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI2ADC_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI2ADC_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare the ADI2 command based on the user inputs */
		nCmdCode = ADBMS6948_CMD_ADI2;
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigI2ADCIp->Adbms6948_eOWSel << ADBMS6948_ADI2_POS_OW));
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigI2ADCIp->Adbms6948_bContMode << ADBMS6948_ADI2_POS_CONT));

		/* Send the ADI2 command */
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
		nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	This function triggers I1-ADC conversions on all ADBMS6948 devices
			in the specified chain.
    @param [in] poTrigIVADCIp	Pointer to the structure containing the I1 and C
							 	ADCs trigger parameter.
							 	@range: NA
							 	@resolution: NA
	@param [in] knChainID		Specifies ID of the daisy chain on which the
	                        	operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_TrigIVADC
(
	Adbms6948_TrigIVADCInputs  *poTrigIVADCIp,
	const uint8_t  				knChainID
)
{
	Adbms6948_ReturnType nRet = E_NOT_OK;
	uint16_t  nCmdCode;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI1CADC_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == poTrigIVADCIp)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI1CADC_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI1CADC_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_TRIGI1CADC_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare the ADCIV command based on the user inputs */
		nCmdCode = ADBMS6948_CMD_ADCIV;
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigIVADCIp->Adbms6948_eOWSel << ADBMS6948_ADCIV_POS_OW));
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigIVADCIp->Adbms6948_bRstIIRFltr << ADBMS6948_ADCIV_POS_RSTF));
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigIVADCIp->Adbms6948_bEnSSDP << ADBMS6948_ADCIV_POS_SSDP));
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigIVADCIp->Adbms6948_bContMode << ADBMS6948_ADCIV_POS_CONT));
		nCmdCode =  (uint16_t)( nCmdCode | ((uint16_t)poTrigIVADCIp->Adbms6948_bEnRedun << ADBMS6948_ADCIV_POS_RD));

		/* Send the ADCIV command */
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
		nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
 /*!
    @brief		This function reads the CADC conversion counter from all the
				ADBMS6948 devices in the specified chain.

	@param [out] panConvCnt	Place holder to report the conversion counter.
							@range: NA
							@resolution: NA
	@param [in] eSnapSel	Specifies the snapshot command/s to be sent.
							@range: Enumeration.
							@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadCADCConvCnt
(
	uint16_t 				*panConvCnt,
	Adbms6948_SnapSelType 	 eSnapSel,
	const uint8_t  		     knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCADCCONVCNT_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == panConvCnt)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCADCCONVCNT_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if ((TRUE == Adbms6948_Cmn_IsInvalidSnapSelType(eSnapSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCADCCONVCNT_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCADCCONVCNT_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read CADC conversion counter */
		nRet = Adbms6948_lReadCADCConvCnt(panConvCnt, eSnapSel, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
/*!
    @brief          This function reads the I1-ADC conversion count from the
                    ADBMS6948 device.


    @param  [out]   panConvCnt  Placeholder to report the conversion count from
                                the ADBMS6948 device.
                                @range: NA
                                @resolution: NA
    @param  [in]    eSnapSel    Specifies the snapshot command/s to be sent.
                                @range: Enumeration.
                                @resolution: NA
    @param  [in]    knChainID   Specifies ID of the daisy chain on which the
                                operation is to be performed.
                                @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                @resolution: NA
    @return     E_OK: Success       \n
                E_NOT_OK: Failed    \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadI1ADCConvCnt
(
    uint16_t                *panConvCnt,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
)
{
    Adbms6948_ReturnType    nRet = E_NOT_OK;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READI1ADCCONVCNT_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if (NULL_PTR == panConvCnt)
    {
        /* Null pointer error. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READI1ADCCONVCNT_ID, ADBMS6948_E_PARAM_POINTER);
        #endif
    }
    else if ((TRUE == Adbms6948_Cmn_IsInvalidSnapSelType(eSnapSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
    {
        /* Invalid chain ID. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READI1ADCCONVCNT_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READI1ADCCONVCNT_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        /* Read CADC conversion counter */
        nRet = Adbms6948_lReadI1ADCConvCnt(panConvCnt, eSnapSel, knChainID);
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

        /* Set chain state to IDLE. */
        nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
    }
return(nRet);
}
/*****************************************************************************/
/*!
    @brief      This function reads the Voltage and Current Conversion Count
                values from the ADBMS6948 device.

    @param  [out]	pnVoltageCT     10-bit CT value of Voltage ADC
                                    @range: 0 to 2047.
                                    @resolution: 1 count
    @param  [out]	pnCurrentCT      10-bit CT value of Current ADC
                                    @range: 0 to 2047.
                                    @resolution: 1 count
    @param  [in]    eSnapSel        Specifies the snapshot command/s to be sent.
                                    @range: Enumeration.
                                    @resolution: NA

    @param  [in]    knChainID       Specifies ID of the daisy chain on which the
                                    operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA

    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_ReadVoltageCurrentCTValues
(
    uint16_t                *pnVoltageCT,
    uint16_t                *pnCurrentCT,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
)
{
    Adbms6948_ReturnType nRet = E_NOT_OK;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READIVADCCONVCNT_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if (NULL_PTR == pnVoltageCT || NULL_PTR == pnCurrentCT)
    {
        /* Null pointer error. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READIVADCCONVCNT_ID, ADBMS6948_E_PARAM_POINTER);
        #endif
    }
    else if ((TRUE == Adbms6948_Cmn_IsInvalidSnapSelType(eSnapSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
    {
        /* Invalid chain ID. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READIVADCCONVCNT_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READIVADCCONVCNT_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        /* Read CADC and IADC CT bit values */
        nRet = Adbms6948_lReadVoltageCurrentCTValues(pnVoltageCT, pnCurrentCT, eSnapSel, knChainID);
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

        /* Set chain state to IDLE. */
        nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
    }
return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	This function reads the specified cell voltage data from all the
			ADBMS6948 devices in the specified chain.
    @param [in] 	eCellMeasData	Specifies type of cell measurement data.
									@range: Enumeration
									@resolution: NA
    @param [in] 	eCellGrpSel		Specifies the cell group/s to be read.
									@range: Enumeration
									@resolution: NA
	@param [out] 	pnCellData		Place holder to report the cell measurement
									data.
									@range: NA
									@resolution: NA
	@param [in] eSnapSel	Specifies the snapshot command/s to be sent.
							@range: Enumeration.
							@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA

    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadCellVolt
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
	Adbms6948_CellGrpSelType  		eCellGrpSel,
	int16_t 					   *pnCellData,
	Adbms6948_SnapSelType 			eSnapSel,
	const uint8_t  					knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == pnCellData)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if ((TRUE == Adbms6948_Cmn_IsInvalidSnapSelType(eSnapSel)) || (TRUE == Adbms6948_lIsInvalidCellMeasDataType(eCellMeasData)) || (TRUE == Adbms6948_lIsInvalidCellGrpSelType(eCellGrpSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read cell voltages */
		nRet = Adbms6948_lReadCellVolt(eCellMeasData, eCellGrpSel, pnCellData, eSnapSel, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	This function reads the all cell voltage data from the
			ADBMS6948 device.
    @param [in] 	eCellMeasData	Specifies type of cell measurement data.
									@range: Enumeration
									@resolution: NA
	@param [out] 	pnCellData		Place holder to report the cell measurement
									data.
									@range: NA
									@resolution: NA
	@param [in]     eSnapSel	    Specifies the snapshot command/s to be sent.
							        @range: Enumeration.
							        @resolution: NA
	@param [in] 	knChainID		Specifies ID of the daisy chain on which the
	                        		operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadAllCellVolt
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
	int16_t 					    *pnCellData,
	Adbms6948_SnapSelType 			eSnapSel,
    const uint8_t                   knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == pnCellData)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if ((TRUE == Adbms6948_Cmn_IsInvalidSnapSelType(eSnapSel)) || (TRUE == Adbms6948_lIsInvalidCellMeasDataType(eCellMeasData)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read cell voltages */
		nRet = Adbms6948_lReadAllCellVolt(eCellMeasData, pnCellData, eSnapSel, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
/*!
    @brief          This function reads the average voltages of all the cells
					from the ADBMS6948 device.

    @param 	[out]	pnCellData	Place holder to report the cell measurement
								data.
								@range: NA
								@resolution: NA
	@param	[in] 	knChainID	Specifies ID of the daisy chain on which the
	                        	operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return     	E_OK: Success  \n
                	E_NOT_OK: Failed \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadAllAverageCellVoltages
(
	int16_t 					   *pnCellData,
	const uint8_t  					knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == pnCellData)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLT_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read cell voltages */
		nRet = Adbms6948_lReadAllAverageCellVoltages(pnCellData, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}

/*****************************************************************************/
 /*!
    @brief	This function reads the specified GPIO input voltage data from all the
			ADBMS6948 devices in the specified chain.
    @param [in] 	eGpioMeasData	Specifies type of GPIO measurement data.
									@range: Enumeration
									@resolution: NA
    @param [in] 	eGpioGrpSel		Specifies the GPIO group/s to be read.
									@range: Enumeration
									@resolution: NA
	@param [out] 	pnGpioData		Place holder to report the GPIO measurement
									data.
									@range: NA
									@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA

    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadGPIOInputVolt
(
	Adbms6948_GpioMeasDataType  	eGpioMeasData,
	Adbms6948_GpioGrpSelType  		eGpioGrpSel,
	int16_t 					   *pnGpioData,
	const uint8_t  					knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READGPIOINPUTVOLT_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == pnGpioData)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READGPIOINPUTVOLT_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if ((TRUE == Adbms6948_lIsInvalidGpioMeasDataType(eGpioMeasData)) || (TRUE == Adbms6948_lIsInvalidGpioGrpSelType(eGpioGrpSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READGPIOINPUTVOLT_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READGPIOINPUTVOLT_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read GPIO input voltages */
		nRet = Adbms6948_lReadGPIOInputVolt(eGpioMeasData, eGpioGrpSel, pnGpioData, knChainID);
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
		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	This function reads the specified internal device parameter from
			all the ADBMS6948 devices in the specified chain.

    @param [in] 	eDevParamGrpSel	Specifies the device parameter group to
									be read.
									@range: Enumeration
									@resolution: NA
	@param [out] 	pnDevParamData	Place holder to report the internal device
									parameter measurement data.
									@range: NA
									@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadDevParam
(
	Adbms6948_DevParamGrpSelType  	 eDevParamGrpSel,
	int16_t 						*pnDevParamData,
	const uint8_t  					 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READDEVPARAM_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == pnDevParamData)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READDEVPARAM_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if ((TRUE == Adbms6948_lIsInvalidDevParamGrpSelType(eDevParamGrpSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READDEVPARAM_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READDEVPARAM_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read internal device parameters */
		nRet = Adbms6948_lReadDevParam(eDevParamGrpSel, pnDevParamData, knChainID);
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
		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
/*!
    @brief          This function reads all the GPIO, Redundant GPIO and Status
                    registers.

    @param  [out]   panGpioData             Placeholder to store the GPIO
                                            values.
                                            @range: NA
                                            @resolution: NA

    @param  [out]   panRedundantGpioData    Placeholder to store the Redundant
                                            GPIO values.
                                            @range: NA
                                            @resolution: NA

    @param  [out]   poStatusRegsVal         Placeholder to store the Status
                                            register values.
                                            @range: NA
                                            @resolution: NA

    @param  [in]    knChainID               Specifies ID of the daisy chain on
                                            which the operation is to be
                                            performed.
                                            @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                            @resolution: NA
    @return     E_OK: Success   \n
                E_NOT_OK: Failed\n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadAllAuxStatusRegisters
(
    int16_t                         *panGpioData,
    int16_t                         *panRedundantGpioData,
    Adbms6948_StatusRegValsType     *poStatusRegsVal,
    const uint8_t                    knChainID
)
{
    Adbms6948_ReturnType    nRet = E_NOT_OK;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READDEVPARAM_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if ((NULL_PTR == panGpioData) || (NULL_PTR == panRedundantGpioData) || (NULL_PTR == poStatusRegsVal))
    {
        /* Null pointer error. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READDEVPARAM_ID, ADBMS6948_E_PARAM_POINTER);
        #endif
    }
    else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
    {
        /* Invalid chain ID. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READDEVPARAM_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READDEVPARAM_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        /* Read the GPIOs, Redundant GPIOs and Status registers */
        nRet = Adbms6948_lReadAllAuxStatusRegisters(panGpioData, panRedundantGpioData, poStatusRegsVal, knChainID);
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
        /* Set chain state to IDLE. */
        nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
    }
return(nRet);
}
/*****************************************************************************/
/*!
    @brief          This function reads the specified current data from the
                    ADBMS6948 device in the specified chain.

    @param[in]      eCurrentMeasData	Specifies type of current measurement
										data.
										@range: Enumeration
										@resolution: NA
    @param	[in]    eCurrentSel			Specifies the current ADC to read data
										of.
										@range: Enumeration
										@resolution: NA
    @param	[out]   pnCurrentData		Placeholder to report the current
										measurement data
										@range: NA
										@resolution: NA
    @param	[in]    eSnapSel			Specifies the snapshot command/s to be sent.
							        	@range: Enumeration.
							        	@resolution: NA
	@param 	[in] 	knChainID			Specifies ID of the daisy chain on which the
	                        			operation is to be performed.
										@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
										@resolution: NA
    @return     E_OK: Success   \n
                E_NOT_OK: Failed\n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadCurrent
(
    Adbms6948_CurrentMeasDataType   eCurrentMeasData,
    Adbms6948_CurrentSelType        eCurrentSel,
    int32_t                         *pnCurrentData,
	Adbms6948_SnapSelType 			eSnapSel,
	const uint8_t  					knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCURRENT_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == pnCurrentData)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCURRENT_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if ((TRUE == Adbms6948_Cmn_IsInvalidSnapSelType(eSnapSel)) || (TRUE == Adbms6948_lIsInvalidCurrentMeasDataType(eCurrentMeasData)) || (TRUE == Adbms6948_lIsInvalidCurrentSelType(eCurrentSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCURRENT_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCURRENT_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read current */
		nRet = Adbms6948_lReadCurrent(eCurrentMeasData, eCurrentSel, pnCurrentData, eSnapSel, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
/*!
    @brief          This function reads all the cell voltages from C-ADC and
					current data from I1-ADC.

    @param	[out]	pnCellData 		Placeholder to report the cell voltage data
									@range: NA
									@resolution: NA
    @param	[out]	pnCurrentData 	Placeholder to report the current data
									@range: NA
									@resolution: NA
    @param	[in]	eSnapSel		Specifies the snapshot command/s to be sent.
							        @range: Enumeration.
							        @resolution: NA
	@param 	[in]	knChainID		Specifies ID of the daisy chain on which the
	                        		operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA
    @return     E_OK: Success   \n
                E_NOT_OK: Failed\n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_ReadCADCI1ADC
(
	int16_t			    	*pnCellData,
	int32_t			    	*pnCurrentData,
	Adbms6948_SnapSelType	eSnapSel,
	const uint8_t			knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR ==  pnCellData) || (NULL_PTR == pnCurrentData))
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if ((TRUE == Adbms6948_Cmn_IsInvalidSnapSelType(eSnapSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read current and voltage using RDCIV command*/
		nRet = Adbms6948_lReadCADCI1ADC(pnCellData, pnCurrentData, eSnapSel, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
/*!
    @brief          This function reads all average cell voltages and average
					current data from C-ADC and I1-ADC respectively.

	@param	[out]	pnCellData		    Placeholder to report the cell voltage data
									    @range: NA
									    @resolution: NA
    @param	[out]	pnCurrentData 	    Placeholder to report the current data
									    @range: NA
									    @resolution: NA
    @param  [in]    bSendUnsnapAtEnd    Specifies whether to send UNSNAP command
                                        after reading cell voltages and current.
									    @range: FALSE, TRUE
									    @resolution: NA
    @param  [out]   pnVoltageCTValue 	10-bit CT value of Voltage ADC
                                    	@range: 0 to 2047.
                                    	@resolution: 1 count
    @param  [out]   pnCurrentCTValue	10-bit CT value of Current ADC
                                    	@range: 0 to 2047.
                                    	@resolution: 1 count
	@param 	[in]	knChainID		    Specifies ID of the daisy chain on which the
	                        		    operation is to be performed.
									    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									    @resolution: NA
	@return			E_OK: Success    \n
					E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadCellVoltCurrentAverage
(
	int16_t 					   	*pnCellData,
    int32_t                        	*pnCurrentData,
    boolean                         bSendUnsnapAtEnd,
	uint16_t						*pnVoltageCTValue,
	uint16_t						*pnCurrentCTValue,
	const uint8_t  					knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR ==  pnCellData) || (NULL_PTR == pnCurrentData))
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read current and voltage using RDCIV command*/
		nRet = Adbms6948_lReadCellVoltCurrentAverage(pnCellData, pnCurrentData, bSendUnsnapAtEnd, pnVoltageCTValue, pnCurrentCTValue, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}

/*****************************************************************************/
/*!
    @brief  This function sends the specified control command to all the
			ADBMS6948 devices in the specified chain.

	@param [in] eCtrlCmdSel	Specifies the control command to be sent.
							@range: Enumeration
							@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
							operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_SendCtrlCmd
(
Adbms6948_CtrlCmdSelType  	eCtrlCmdSel,
const uint8_t  				knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint16_t  nCmdCode;
	boolean 	bIsIncrCmd;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_SENDCTRLCMD_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((TRUE == Adbms6948_lIsInvalidCtrlCmdSelType(eCtrlCmdSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_SENDCTRLCMD_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_SENDCTRLCMD_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		Adbms6948_lGetCtrlCmdCode(eCtrlCmdSel, &nCmdCode, &bIsIncrCmd);
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, bIsIncrCmd, knChainID);

        /*Verify the command counter value if the control command increases it*/
		nRet = Adbms6948_Cmn_VerifyCmdCnt(bIsIncrCmd, knChainID);

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
		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
/*!
    @brief         	This function polls the ADC status for the specified ADC/s
					on ADBMS6948 device/s in the specified chain.
	@param [in]  	eADCSel		Specifies the ADC/s to be polled for the status.
								@range: Enumeration
								@resolution: NA
	@param [in]  	pbADCStatus	Place holder to report the Poll ADC status.
								@range: NA
								@resolution: NA
	@param [in] 	knChainID	Specifies ID of the daisy chain on which the
								operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_PollADCStatus
(
Adbms6948_ADCSelType 	 eADCSel,
boolean 				*pbADCStatus,
const uint8_t  			 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;
	uint16_t  nCmdCode;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_POLLADCSTATUS_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == pbADCStatus)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_POLLADCSTATUS_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if ((TRUE == Adbms6948_lIsInvalidADCSelType(eADCSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_POLLADCSTATUS_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_POLLADCSTATUS_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		Adbms6948_lGetPollADCCmdCode(eADCSel, &nCmdCode);
		nRet = Adbms6948_lPollADCStatus(nCmdCode, pbADCStatus, knChainID);

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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}
/*****************************************************************************/
/*!
    @brief          This function enables the coulomb counter.

    @param[in]      bContMode       Specifies whether coulomb counter runs in continuous mode or single shot mode.
                                    @range: FALSE,TRUE
                                    @resolution: NA
	@param [in] 	knChainID	    Specifies ID of the daisy chain on which the
                                    operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA
    @return         E_OK: Success       \n
                    E_NOT_OK: Failed    \n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_EnableCoulombCounter
(
    boolean             bContMode,
    const uint8_t       knChainID
)
{
	Adbms6948_ReturnType nRet = E_NOT_OK;
	uint16_t  nCmdCode;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_ENABLECOULOMBCOUNTER_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_ENABLECOULOMBCOUNTER_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Prepare the CCEN command based on the user inputs */
		nCmdCode = ADBMS6948_CMD_CCEN | ((uint16_t)bContMode << ADBMS6948_CCEN_POS_CCEN_CON);

		/* Send the CCEN command */
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);

        /*Verify command count as CCEN command was sent which increments the command counter*/
		nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
    }
return(nRet);
}
/*****************************************************************************/
/*!
    @brief          This function reads the coulomb counter and time base value
                    from the ADBMS6948 device.

    @param  [out]   pnCoulombCount  Placeholder to report the coulomb counter
                                    value.
                                    @range: 0 to (2^32 - 1)
                                    @resolution: 1 count
    @param  [out]   pnTimeBase      Placeholder to report the time base value.
                                    @range: 0 to (2^16 - 1)
                                    @resolution: 1 conversion time
    @param  [in]    eSnapSel        Specifies the snapshot command/s to be sent.
                                    @range: Enumeration.
                                    @resolution: NA
    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA
    @return         E_OK:       Success \n
                    E_NOT_OK:   Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_ReadCoulombCountAndTimeBase
(
    uint32_t                *pnCoulombCount,
    uint16_t                *pnTimeBase,
	Adbms6948_SnapSelType   eSnapSel,
    const uint8_t           knChainID
)
{
    Adbms6948_ReturnType  	nRet = E_NOT_OK;
	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCOULOMBCOUNTER_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if ((NULL_PTR ==  pnCoulombCount) || (NULL_PTR == pnTimeBase))
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLVOLTAGECURRENT_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read Coulomb counter and time base register using RDCT command*/
		nRet = Adbms6948_lReadCoulombCountAndTimeBase(pnCoulombCount, pnTimeBase, eSnapSel, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}

/*****************************************************************************/
/*!
    @brief          This function clears all the fault flags in the Status C
                    register

    @param  [in]    knChainID   Specifies ID of the daisy chain on
                                which the operation is to be performed.
                                @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                @resolution: NA
    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_ClearAllFlags
(
    const uint8_t                   knChainID
)
{
    Adbms6948_ReturnType nRet = E_NOT_OK;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CLRFLAG_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID)
    {
        /* Invalid chain ID. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CLRFLAG_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CLRFLAG_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        nRet = Adbms6948_lClearFlags(knChainID);
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
        /* Set chain state to IDLE. */
        nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
    }
return(nRet);
}

/*****************************************************************************/
/*!
    @brief	This function gets the status of all the OV/UV diagnostic flags
			of all the cells in the daisy chain.\n

    @param [out]	pnCellOVUVStatus  	Place holder to report the OVUV
										status for all the connected cells
										in the chain.
										@range: NA
										@resolution: NA

	@param 	[in] 	eSnapSel	Specifies the snapshot command/s to be sent.
								@range: Enumeration
								@resolution: NA

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Int_ReadCellOVUVStatus
(
uint32_t 				*pnCellOVUVStatus,
Adbms6948_SnapSelType 	 eSnapSel,
const uint8_t  			 knChainID
)
{
	Adbms6948_ReturnType  	nRet = E_NOT_OK;

	if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
	{
		/* Module is not initialized. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLOVUVSTATUS_ID, ADBMS6948_E_MODULESTATE);
		#endif
	}
	else if (NULL_PTR == pnCellOVUVStatus)
	{
		/* Null pointer error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLOVUVSTATUS_ID, ADBMS6948_E_PARAM_POINTER);
		#endif
	}
	else if ((TRUE == Adbms6948_Cmn_IsInvalidSnapSelType(eSnapSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
	{
		/* Invalid chain ID. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLOVUVSTATUS_ID, ADBMS6948_E_INVALID_PARAM);
		#endif
	}
	else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
	{
		/* Chain busy error. */
		#if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
		Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_READCELLOVUVSTATUS_ID, ADBMS6948_E_CHAIN_BUSY);
		#endif
	}
	else
	{
		/* Read cell OVUV status */
		nRet = Adbms6948_lReadCellOVUVStatus(pnCellOVUVStatus, eSnapSel, knChainID);
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

		/* Set chain state to IDLE. */
		nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
	}
return(nRet);
}

/*****************************************************************************/
/*!
    @brief          This function clears all the cell voltage registers and also
                    gives an option to snap before, after or both before and
                    after clearing the registers.

    @param  [in]    eSnapSel    Specifies the snapshot command/s to be sent.
                                @range: Enumeration
                                @resolution: NA
    @param  [in]    knChainID   Specifies ID of the daisy chain on
                                which the operation is to be performed.
                                @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                @resolution: NA
    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_ClearCellVoltageRegisters
(
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
)
{
    Adbms6948_ReturnType nRet = E_NOT_OK;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CLRCELL_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if ((TRUE == Adbms6948_Cmn_IsInvalidSnapSelType(eSnapSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
    {
        /* Invalid chain ID. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CLRCELL_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CLRCELL_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        nRet = Adbms6948_lClearCellVoltageRegisters(eSnapSel, knChainID);
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
        /* Set chain state to IDLE. */
        nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
    }
return(nRet);
}

/*****************************************************************************/
/*!
    @brief          This function clears the I1 or I2 current values in the current
                    registers based on @c eCurrentSel and also gives an option
                    to snap before, after or both before and after clearing the
                    registers.

    @param  [in]    eCurrentSel     Specifies the current channel to clear
                                    data of.
                                    @range: Enumeration
                                    @resolution: NA
    @param  [in]    eSnapSel        Specifies the snapshot command/s to be sent.
                                    @range: Enumeration
                                    @resolution: NA
    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA
    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_Int_ClearCurrentRegisters
(
    Adbms6948_CurrentSelType        eCurrentSel,
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
)
{
    Adbms6948_ReturnType nRet = E_NOT_OK;

    if (ADBMS6948_ST_UNINIT == Adbms6948_eState)
    {
        /* Module is not initialized. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CLRCURRENTREGS_ID, ADBMS6948_E_MODULESTATE);
        #endif
    }
    else if ((TRUE == Adbms6948_Cmn_IsInvalidSnapSelType(eSnapSel)) || (TRUE == Adbms6948_lIsInvalidCurrentSelType(eCurrentSel)) || (ADBMS6948_NO_OF_DAISY_CHAIN <= knChainID))
    {
        /* Invalid chain ID. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CLRCURRENTREGS_ID, ADBMS6948_E_INVALID_PARAM);
        #endif
    }
    else if (E_NOT_OK == Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_BUSY, knChainID))
    {
        /* Chain busy error. */
        #if (TRUE == ADBMS6948_DEVELOPMENT_MODE_EN)
        Adbms6948_Cmn_ReportDevErr(ADBMS6948_MODULE_ID, ADBMS6948_INSTANCE_ID, ADBMS6948_CLRCURRENTREGS_ID, ADBMS6948_E_CHAIN_BUSY);
        #endif
    }
    else
    {
        nRet = Adbms6948_lClearCurrentRegisters(eCurrentSel, eSnapSel, knChainID);
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
        /* Set chain state to IDLE. */
        nRet |= Adbms6948_Cmn_UpdateChainStatus(ADBMS6948_ST_CHAIN_IDLE, knChainID);
    }
return(nRet);
}

/*! @}*/
/*! @}*/

#ifndef ADI_DISABLE_DOCUMENTATION
/*****************************************************************************/
/*!
    @brief		Local function to read the CADC conversion counter.
	@param [out] panConvCnt	Place holder to report the conversion counter.
							@range: NA
							@resolution: NA
	@param [in] eSnapSel	Specifies the snapshot command/s to be sent.
							@range: Enumeration.
							@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
*/
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadCADCConvCnt
(
	uint16_t 				*panConvCnt,
	Adbms6948_SnapSelType 	 eSnapSel,
	const uint8_t  			 knChainID
)
{
	Adbms6948_ReturnType  nRet;
	uint16_t  nCmdCode;
	uint8_t  anRdBuf[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
	uint16_t  anCmdList[ADBMS6948_NUMOF_REGGRP_CONVCNT];
	uint8_t  nDevIdx, nBufIdx = 0u;
	uint16_t  nConvCnt, nRdBufIdx;
	boolean 	bCfgRead = FALSE;

	if ((ADBMS6948_SEND_SNAP == eSnapSel) || (ADBMS6948_SEND_BOTH == eSnapSel))
	{
		nCmdCode = ADBMS6948_CMD_SNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
	}
	anCmdList[0u] = ADBMS6948_CMD_RDSTATG;
	nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, anRdBuf, ADBMS6948_NUMOF_REGGRP_CONVCNT, knChainID);
	if (E_OK == nRet)
	{
		/* Fetch the CADC conversion counter for all devices and populate it into the user buffer.*/
		for (nDevIdx = 0u; nDevIdx < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDevIdx++)
		{
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				nRdBufIdx = (uint16_t)((uint16_t)ADBMS6948_CMD_DATA_LEN + (nDevIdx * (uint16_t)ADBMS6948_REG_DATA_LEN_WITH_PEC));
				nConvCnt = (uint16_t)(((uint16_t)anRdBuf[0u][nRdBufIdx] << 8u) | ((uint16_t)anRdBuf[0u][nRdBufIdx + 1u]));
				panConvCnt[nBufIdx++] = nConvCnt;
			}
		}
	}
	if ((E_OK == nRet) && ((ADBMS6948_SEND_UNSNAP == eSnapSel) || (ADBMS6948_SEND_BOTH == eSnapSel)))
	{
		nCmdCode = ADBMS6948_CMD_UNSNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
		bCfgRead = TRUE;
	}

	/* Verify the command counter if UNSNAP command was sent*/
	nRet |= Adbms6948_Cmn_VerifyCmdCnt(bCfgRead, knChainID);

return(nRet);
}
/*****************************************************************************/
/*!
    @brief      Local function to read the I1-ADC conversion counter.
    @param [out] panConvCnt Place holder to report the conversion counter.
                            @range: NA
                            @resolution: NA
    @param [in] eSnapSel    Specifies the snapshot command/s to be sent.
                            @range: Enumeration.
                            @resolution: NA
    @param [in] knChainID   Specifies ID of the daisy chain on which the
                            operation is to be performed.
                            @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                            @resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
*/
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadI1ADCConvCnt
(
    uint16_t                *panConvCnt,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
)
{
    Adbms6948_ReturnType  nRet;
    uint16_t  nCmdCode;
    uint8_t  anRdBuf[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + ADBMS6948_REG_DATA_LEN_WITH_PEC];
    uint16_t  anCmdList[ADBMS6948_NUMOF_REGGRP_CONVCNT];
    uint16_t  nConvCnt, nRdBufIdx;
    boolean bCfgRead = FALSE;

    if ((ADBMS6948_SEND_SNAP == eSnapSel) || (ADBMS6948_SEND_BOTH == eSnapSel))
    {
        nCmdCode = ADBMS6948_CMD_SNAP;
        Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
    }
    anCmdList[0u] = ADBMS6948_CMD_RDSTATG;
    nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, anRdBuf, ADBMS6948_NUMOF_REGGRP_CONVCNT, knChainID);
    if (E_OK == nRet)
    {
        nRdBufIdx = (uint16_t)ADBMS6948_CMD_DATA_LEN;
        nConvCnt = (uint16_t)(((uint16_t)anRdBuf[0u][nRdBufIdx + 2u] << 8u) | ((uint16_t)anRdBuf[0u][nRdBufIdx + 3u]));
        *panConvCnt = nConvCnt;
    }
    if ((E_OK == nRet) && ((ADBMS6948_SEND_UNSNAP == eSnapSel) || (ADBMS6948_SEND_BOTH == eSnapSel)))
    {
        nCmdCode = ADBMS6948_CMD_UNSNAP;
        Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
        bCfgRead = TRUE;
    }

    /* Verify the command counter if UNSNAP command was sent*/
    nRet |= Adbms6948_Cmn_VerifyCmdCnt(bCfgRead, knChainID);

return(nRet);
}
/*****************************************************************************/
/*!
    @brief      This local function reads the Voltage and Current Conversion
                Count values from the ADBMS6948 device.

    @param  [out]   pnVoltageCT     10-bit CT value of Voltage ADC
                                    @range: 0 to 2047.
                                    @resolution: 1 count
    @param  [out]   pnCurrentCT      10-bit CT value of Current ADC
                                    @range: 0 to 2047.
                                    @resolution: 1 count
    @param  [in]    eSnapSel        Specifies the snapshot command/s to be sent.
                                    @range: Enumeration.
                                    @resolution: NA

    @param  [in]    knChainID       Specifies ID of the daisy chain on which the
                                    operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA

    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadVoltageCurrentCTValues
(
    uint16_t                *pnVoltageCT,
    uint16_t                *pnCurrentCT,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
)
{
    Adbms6948_ReturnType  nRet;
    uint16_t nCmdCode;
    uint8_t  anRdBuf[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + ADBMS6948_REG_DATA_LEN_WITH_PEC];
    uint16_t anCmdList[ADBMS6948_NUMOF_REGGRP_CONVCNT];
    uint16_t nRdBufIdx;
    boolean  bCfgRead = FALSE;

    if ((ADBMS6948_SEND_SNAP == eSnapSel) || (ADBMS6948_SEND_BOTH == eSnapSel))
    {
        nCmdCode = ADBMS6948_CMD_SNAP;
        Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
    }
    anCmdList[0u] = ADBMS6948_CMD_RDSTATG;
    nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, anRdBuf, ADBMS6948_NUMOF_REGGRP_CONVCNT, knChainID);

    if (E_OK == nRet)
    {
        /* Fetch the CT_I and CT_V values and populate them into the user buffer.*/
        nRdBufIdx    = (uint16_t)ADBMS6948_CMD_DATA_LEN;
        *pnVoltageCT = (uint16_t)(((uint16_t)(anRdBuf[0][nRdBufIdx] & 0x1F) << 6u) |
                                ((uint16_t)(anRdBuf[0][nRdBufIdx + 1u] & 0xFCu) >> 2u));
        *pnCurrentCT = (uint16_t)(((uint16_t)(anRdBuf[0][nRdBufIdx + 2u] & 0x1F) << 6u) |
                                ((uint16_t)(anRdBuf[0][nRdBufIdx + 3u] & 0xFCu) >> 2u));
    }

    if ((E_OK == nRet) && ((ADBMS6948_SEND_UNSNAP == eSnapSel) || (ADBMS6948_SEND_BOTH == eSnapSel)))
    {
        nCmdCode = ADBMS6948_CMD_UNSNAP;
        Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
        bCfgRead = TRUE;
    }

    /* Verify the command counter if UNSNAP command was sent*/
    nRet |= Adbms6948_Cmn_VerifyCmdCnt(bCfgRead, knChainID);

return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	Local function to prepare the read command codes.
    @param [in] 	eCellMeasData	Specifies type of cell measurement data.
									@range: Enumeration
									@resolution: NA
    @param [in] 	eCellGrpSel		Specifies the cell group/s to be read.
									@range: Enumeration
									@resolution: NA
	@param [out] 	panCmdList		Place holder to populate the read command list.
									data.
									@range: NA
									@resolution: NA
	@param [in] pnCmdListCnt		Specifies number of read commands to be sent.
									@range: NA.
									@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     None
 */
/*****************************************************************************/
static void  Adbms6948_lGetCellCmdList
(
	Adbms6948_CellMeasDataType  	 eCellMeasData,
	Adbms6948_CellGrpSelType  		 eCellGrpSel,
	uint16_t 						*panCmdList,
	uint8_t 						*pnCmdListCnt,
	const uint8_t  					 knChainID
)
{
	uint8_t  nLoopIdx, nNumOfGroups, nCmdIdx = 0u;
	if (ADBMS6948_CELL_GRP_SEL_ALL == eCellGrpSel)
	{
		nNumOfGroups = Adbms6948_aoChainStateInfo[knChainID].nMaxCellRegGrps;
		for (nLoopIdx = 0; nLoopIdx < nNumOfGroups; nLoopIdx++)
		{
			panCmdList[nCmdIdx++] = Adbms6948_CellDataReadCmds[(uint8_t)eCellMeasData][nLoopIdx];
		}
	}
	else
	{
		nNumOfGroups = ADBMS6948_NUMOF_REGGRP_ONE;
		panCmdList[nCmdIdx] = Adbms6948_CellDataReadCmds[(uint8_t)eCellMeasData][(uint8_t)eCellGrpSel];
	}

	*pnCmdListCnt = nNumOfGroups;
}
/*****************************************************************************/
/*!
    @brief          Local function to prepare command code for reading cell
                    voltages.

    @param  [in] 	eCellMeasData	Specifies type of cell measurement data.
									@range: Enumeration
									@resolution: NA
    @param  [in] 	eCellGrpSel		Specifies the cell group/s to be read.
									@range: Enumeration
									@resolution: NA
	@param  [out] 	pnCmd		    Place holder to populate the read command.
									@range: NA
									@resolution: NA
	@param  [in]    knChainID	    Specifies ID of the daisy chain on which the
	                                operation is to be performed.
							        @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							        @resolution: NA
 */
/*****************************************************************************/
static void  Adbms6948_lGetCellCmd
(
	Adbms6948_CellMeasDataType      eCellMeasData,
	Adbms6948_CellGrpSelType  	    eCellGrpSel,
	uint16_t                        *pnCmd,
	const uint8_t  					 knChainID
)
{
	if (ADBMS6948_CELL_GRP_SEL_ALL == eCellGrpSel)
	{
        /*Use ALL commands for reading all cell voltage groups at once*/
        *pnCmd = Adbms6948_CellDataReadCmds[(uint8_t)eCellMeasData][(uint8_t)(ADBMS6948_MAX_CELLDATA_REGGRPS)];
	}
	else
	{
		*pnCmd = Adbms6948_CellDataReadCmds[(uint8_t)eCellMeasData][(uint8_t)eCellGrpSel];
	}
}
/*****************************************************************************/
 /*!
    @brief	Local function to read the cell voltage data.
    @param [in] 	eCellMeasData	Specifies type of cell measurement data.
									@range: Enumeration
									@resolution: NA
    @param [in] 	eCellGrpSel		Specifies the cell group/s to be read.
									@range: Enumeration
									@resolution: NA
	@param [out] 	pnCellData		Place holder to report the cell measurement
									data.
									@range: NA
									@resolution: NA
	@param [in] eSnapSel	Specifies the snapshot command/s to be sent.
							@range: Enumeration.
							@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadCellVolt
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
	Adbms6948_CellGrpSelType  		eCellGrpSel,
	int16_t 					   *pnCellData,
	Adbms6948_SnapSelType 			eSnapSel,
	const uint8_t  					knChainID
)
{
	Adbms6948_ReturnType  nRet;
	uint8_t  anReadCellDataBuf[ADBMS6948_MAX_CELLDATA_REGGRPS][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
	uint16_t  anCmdList[ADBMS6948_MAX_CELLDATA_REGGRPS];
	uint16_t  nCmdCode, nCellData, nRdBufIdx, nBufIdx = 0u;
	uint8_t  nRegGrpIdx, nDevIdx, nNumOfCmds;
	boolean  bSendSnap, bSendUnsnap;

	bSendSnap = (boolean)((uint8_t)eSnapSel & 0x01u);
	bSendUnsnap = (boolean)(((uint8_t)eSnapSel & 0x02u) >> 1u);
	if(TRUE == bSendSnap)
	{
		/*Send SNAP command */
		nCmdCode = ADBMS6948_CMD_SNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
	}
	/* Send Read commands*/
	Adbms6948_lGetCellCmdList(eCellMeasData, eCellGrpSel, anCmdList, &nNumOfCmds, knChainID);
	nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, anReadCellDataBuf, nNumOfCmds, knChainID);

	if (E_OK == nRet)
	{
		/* Populate the user data buffer. */
		for (nRegGrpIdx = 0; nRegGrpIdx < nNumOfCmds; nRegGrpIdx++)
		{
			/* Fetch the cell measurement data and populate it into the user buffer.*/
			for (nDevIdx = 0u; nDevIdx < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices; nDevIdx++)
			{
				if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
				{
					nRdBufIdx = (uint16_t)((uint16_t)ADBMS6948_CMD_DATA_LEN + (nDevIdx * (uint16_t)ADBMS6948_REG_DATA_LEN_WITH_PEC));
					nCellData = (int16_t)(((uint16_t)anReadCellDataBuf[nRegGrpIdx][nRdBufIdx + 1u] << 8u) | (uint16_t)anReadCellDataBuf[nRegGrpIdx][nRdBufIdx]);
					pnCellData[nBufIdx++] = nCellData;
					if ((nRegGrpIdx != ADBMS6948_LAST_CELL_REGGRP) &&  (ADBMS6948_CELL_GRP_SEL_C16 != eCellGrpSel))
					{
						nCellData = (int16_t)((uint16_t)((uint16_t)anReadCellDataBuf[nRegGrpIdx][nRdBufIdx + 3u] << 8u) | (uint16_t)(anReadCellDataBuf[nRegGrpIdx][nRdBufIdx + 2u]));
						pnCellData[nBufIdx++] = nCellData;
						nCellData = (int16_t)((uint16_t)((uint16_t)anReadCellDataBuf[nRegGrpIdx][nRdBufIdx + 5u] << 8u) | (uint16_t)(anReadCellDataBuf[nRegGrpIdx][nRdBufIdx + 4u]));
						pnCellData[nBufIdx++] = nCellData;
					}
				}
			}
		}
	}
	if ((E_OK == nRet) && (TRUE == bSendUnsnap))
	{
		nCmdCode = ADBMS6948_CMD_UNSNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
	}

	/*Clear the Cell Voltage registers*/
	if (ADBMS6948_CELL_MEAS_DATA == eCellMeasData || ADBMS6948_CELL_MEAS_DATA_AVERAGED == eCellMeasData)
	{
		nCmdCode = ADBMS6948_CMD_CLRCELL;
	}
	else if (ADBMS6948_CELL_MEAS_DATA_REDUNDANT == eCellMeasData)
	{
		nCmdCode = ADBMS6948_CMD_CLRSPIN;
	}
	else if (ADBMS6948_CELL_MEAS_DATA_FILTERED == eCellMeasData)
	{
		nCmdCode = ADBMS6948_CMD_CLRFC;
	}

	Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);

	/* Verify the command counter if UNSNAP command was sent*/
	nRet |= Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);

return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	Local function to read the cell voltage data.
    @param [in] 	eCellMeasData	Specifies type of cell measurement data.
									@range: Enumeration
									@resolution: NA
	@param [out] 	pnCellData		Place holder to report the cell measurement
									data.
									@range: NA
									@resolution: NA
	@param [in]     eSnapSel	    Specifies the snapshot command/s to be sent.
							        @range: Enumeration.
							        @resolution: NA
	@param [in] 	knChainID		Specifies ID of the daisy chain on which the
	                        		operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA
    @return     E_OK: Success   \n
                E_NOT_OK: Failed\n
 */
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadAllCellVolt
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
	int16_t 					    *pnCellData,
	Adbms6948_SnapSelType 			eSnapSel,
    const uint8_t                   knChainID
)
{
	Adbms6948_ReturnType  nRet;
	uint8_t  anReadCellDataBuf[ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITHOUT_PEC * ADBMS6948_MAX_CELLDATA_REGGRPS) + ADBMS6948_PEC_LEN];
	uint16_t  nCmd;
	uint16_t  nCmdCode, nCellData, nRdBufIdx;
	uint8_t  nDataLenBytesWithPec;
	boolean  bSendSnap, bSendUnsnap;

	nRdBufIdx = 0u;

	bSendSnap = (boolean)((uint8_t)eSnapSel & 0x01u);
	bSendUnsnap = (boolean)(((uint8_t)eSnapSel & 0x02u) >> 1u);
	nDataLenBytesWithPec = Adbms6948_ReadAllCommandBytes[ADBMS6948_READALL_CELLVOLTAGES];
	if(TRUE == bSendSnap)
	{
		/*Send SNAP command */
		nCmdCode = ADBMS6948_CMD_SNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
	}
	/* Send Read commands*/
	Adbms6948_lGetCellCmd(eCellMeasData, ADBMS6948_CELL_GRP_SEL_ALL, &nCmd, knChainID);

	nRet = Adbms6948_Cmn_ReadAllRegGroups(nCmd, anReadCellDataBuf, nDataLenBytesWithPec, knChainID);

	if (E_OK == nRet)
	{
		for(uint8_t nCellIdx = 0; nCellIdx < ADBMS6948_MAX_NO_OF_CELLS_PER_DEVICE; ++nCellIdx)
		{
			/* Populate the user data buffer. */
        	nCellData = (int16_t)(((uint16_t)anReadCellDataBuf[nRdBufIdx + 1u] << 8u) | (uint16_t)anReadCellDataBuf[nRdBufIdx]);
        	pnCellData[nCellIdx] = nCellData;
			nRdBufIdx += 2;
		}
	}
	if ((E_OK == nRet) && (TRUE == bSendUnsnap))
	{
		nCmdCode = ADBMS6948_CMD_UNSNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
	}

	/*Clear the Cell Voltage registers*/
	if (ADBMS6948_CELL_MEAS_DATA == eCellMeasData || ADBMS6948_CELL_MEAS_DATA_AVERAGED == eCellMeasData)
	{
		nCmdCode = ADBMS6948_CMD_CLRCELL;
	}
	else if (ADBMS6948_CELL_MEAS_DATA_REDUNDANT == eCellMeasData)
	{
		nCmdCode = ADBMS6948_CMD_CLRSPIN;
	}
	else if (ADBMS6948_CELL_MEAS_DATA_FILTERED == eCellMeasData)
	{
		nCmdCode = ADBMS6948_CMD_CLRFC;
	}

	Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);

	/* Verify the command counter if UNSNAP command was sent*/
	nRet |= Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);

return(nRet);
}
/*****************************************************************************/
/*!
    @brief	Local function to read the average voltage of all the cells.

	@param [out] 	pnCellData		Place holder to report the cell measurement
									data.
									@range: NA
									@resolution: NA
	@param [in] 	knChainID		Specifies ID of the daisy chain on which the
	                        		operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA
    @return     	E_OK: Success   \n
                	E_NOT_OK: Failed\n
 */
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadAllAverageCellVoltages
(
	int16_t 					   *pnCellData,
	const uint8_t  					knChainID
)
{
	Adbms6948_ReturnType  nRet;
	uint8_t anRDSTATBuff[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + ADBMS6948_REG_DATA_LEN_WITH_PEC];
	uint8_t anReadDataBuf[(ADBMS6948_REG_DATA_LEN_WITHOUT_PEC * ADBMS6948_MAX_CELLDATA_REGGRPS) + ADBMS6948_PEC_LEN];
	uint16_t nVoltageCT;
	uint16_t nRdBufIdx = 0u;
	uint16_t nCellData, nCmd;
	uint8_t nDataLenBytesWithPec;
	boolean bResultsReady = FALSE;

	while(1)
	{
		/*Send SNAP command*/
		Adbms6948_lSendSnap(knChainID);

		/* Send RDSTATG command to read CT bits */
		nCmd  = ADBMS6948_CMD_RDSTATG;
		nRet = Adbms6948_Cmn_ReadRegGroup(&nCmd, anRDSTATBuff, 1, knChainID);
		/*Get the CT bits*/
		uint8_t nRDSTATBufIdx = (uint16_t)ADBMS6948_CMD_DATA_LEN;
		nVoltageCT = (uint16_t)(((uint16_t)(anRDSTATBuff[0][nRDSTATBufIdx] & 0x1F) << 6u) |
								((uint16_t)(anRDSTATBuff[0][nRDSTATBufIdx + 1u] & 0xFCu) >> 2u));

		/*If CT values are multiples of 8, implies conversion complete.*/
		if ((nVoltageCT & 0x07u) == 0)
		{
			bResultsReady = TRUE;
			break;
		}
		else
		{
			/*Send UNSNAP command so that CT values can update.*/
			Adbms6948_lSendUnsnap(knChainID);
		}
	}

	if (TRUE == bResultsReady)
	{
		/*Send RDACALL command to read average C-ADC cell voltage values*/
		nCmd = ADBMS6948_CMD_RDACALL;
		nDataLenBytesWithPec = Adbms6948_ReadAllCommandBytes[ADBMS6948_READALL_CELLVOLTAGES];
		nRet = Adbms6948_Cmn_ReadAllRegGroups(nCmd, anReadDataBuf, nDataLenBytesWithPec, knChainID);
		Adbms6948_lSendUnsnap(knChainID);
	}
	if (E_OK == nRet)
	{
		for(uint8_t nCellIdx = 0; nCellIdx < ADBMS6948_MAX_NO_OF_CELLS_PER_DEVICE; ++nCellIdx)
		{
			/* Populate the user data buffer. */
        	nCellData = (int16_t)(((uint16_t)anReadDataBuf[nRdBufIdx + 1u] << 8u) | (uint16_t)anReadDataBuf[nRdBufIdx]);
        	pnCellData[nCellIdx] = nCellData;
			nRdBufIdx += 2;
		}
	}

	/*Clear the Cell voltage registers*/
	nCmd = ADBMS6948_CMD_CLRCELL;
	Adbms6948_Cmn_SendCommandOnly(nCmd, TRUE, knChainID);

	/* Verify the command counter */
	nRet |= Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);

return(nRet);
}
/*****************************************************************************/
/*!
    @brief  Local function to read the current data.

    @param	[in]  	eCurrentMeasData	Specifies type of current measurement
										data.
										@range: Enumeration
										@resolution: NA
    @param	[in]    eCurrentSel			Specifies the current ADC to read data
										of.
										@range: Enumeration
										@resolution: NA
    @param	[out]   pnCurrentData		Placeholder to report the current
										measurement data
										@range: NA
										@resolution: NA
    @param	[in]    eSnapSel			Specifies the snapshot command/s to be sent.
							        	@range: Enumeration.
							        	@resolution: NA

	@param 	[in] 	knChainID			Specifies ID of the daisy chain on which the
	                        			operation is to be performed.
										@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
										@resolution: NA
    @return     E_OK: Success   \n
                E_NOT_OK: Failed\n
 */
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadCurrent
(
	Adbms6948_CurrentMeasDataType	eCurrentMeasData,
	Adbms6948_CurrentSelType		eCurrentSel,
	int32_t							*pnCurrentData,
	Adbms6948_SnapSelType			eSnapSel,
	const uint8_t					knChainID
)
{
	Adbms6948_ReturnType  nRet;
	uint8_t		anReadCurrentDataBuf[ADBMS6948_MAX_CURRENT_REGGRPS][ADBMS6948_CMD_DATA_LEN + ADBMS6948_REG_DATA_LEN_WITH_PEC];
	uint16_t	anCmdList[ADBMS6948_MAX_CURRENT_REGGRPS];
	uint16_t	nCmdCode, nRdBufIdx, nBufIdx = 0u;
	uint32_t	nCurrentData;
	uint8_t		nRegGrpIdx, nNumOfCmds = 0u;
	boolean		bSendSnap, bSendUnsnap;
	uint16_t 	nCmd;

	bSendSnap = (boolean)((uint8_t)eSnapSel & 0x01u);
	bSendUnsnap = (boolean)(((uint8_t)eSnapSel & 0x02u) >> 1u);
	if(TRUE == bSendSnap)
	{
		/*Send SNAP command */
		nCmdCode = ADBMS6948_CMD_SNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
	}

	/*Select the read command to send based on type of current data*/
	if (ADBMS6948_CURRENT_MEAS_DATA == eCurrentMeasData)
	{
		anCmdList[nNumOfCmds++] = ADBMS6948_CMD_RDI;
	}
	else if (ADBMS6948_CURRENT_MEAS_DATA_AVERAGED == eCurrentMeasData)
	{
		anCmdList[nNumOfCmds++] = ADBMS6948_CMD_RDIAV;
	}

	/* Send Read commands*/
	nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, anReadCurrentDataBuf, nNumOfCmds, knChainID);

	if (E_OK == nRet)
	{
        nRdBufIdx = (uint16_t)ADBMS6948_CMD_DATA_LEN;
        nRegGrpIdx = 0u;
		/*Populate the user data buffer*/
        /*Populate I1 value if applicable*/
        if (ADBMS6948_CURRENT_I1I2 == eCurrentSel || ADBMS6948_CURRENT_I1 == eCurrentSel)
        {
            /*Current register and Average current register have different
            number of bits*/
            switch (eCurrentMeasData)
            {
                case ADBMS6948_CURRENT_MEAS_DATA_AVERAGED:
                    nCurrentData = (uint32_t)((((uint32_t)anReadCurrentDataBuf[nRegGrpIdx][nRdBufIdx + 2u]) & 0xFF) << 16u);
                    if (nCurrentData & (0x00800000))
                    {
                        nCurrentData |= 0xFF000000;
                    }
                    break;
                case ADBMS6948_CURRENT_MEAS_DATA:
                    nCurrentData = (uint32_t)((((uint32_t)anReadCurrentDataBuf[nRegGrpIdx][nRdBufIdx + 2u]) & 0x07) << 16u);
                    if (nCurrentData & (0x00040000))
                    {
                        nCurrentData |= 0xFFF80000;
                    }
                    break;
                default:
                    nRet = E_NOT_OK;
                    break;
            }
            nCurrentData |= ((uint32_t)anReadCurrentDataBuf[nRegGrpIdx][nRdBufIdx + 1u]) << 8u;
            nCurrentData |= (uint32_t)anReadCurrentDataBuf[nRegGrpIdx][nRdBufIdx];
            pnCurrentData[nBufIdx++] = nCurrentData;

			/*Clear the I1 values in Current and Average Current Register groups*/
			nCmd = ADBMS6948_CMD_CLRCI;
			Adbms6948_Cmn_SendCommandOnly(nCmd, TRUE, knChainID);
        }

        /*Populate I2 value if applicable*/
        if (ADBMS6948_CURRENT_I1I2 == eCurrentSel || ADBMS6948_CURRENT_I2 == eCurrentSel)
        {
            /*Current register and Average current register have different
            number of bits*/
            switch (eCurrentMeasData)
            {
                case ADBMS6948_CURRENT_MEAS_DATA_AVERAGED:
                    nCurrentData = (uint32_t)((((uint32_t)anReadCurrentDataBuf[nRegGrpIdx][nRdBufIdx + 5u]) & 0xFF) << 16u);
                    if (nCurrentData & (0x00800000))
                    {
                        nCurrentData |= 0xFF000000;
                    }
                    break;
                case ADBMS6948_CURRENT_MEAS_DATA:
                    nCurrentData = (uint32_t)((((uint32_t)anReadCurrentDataBuf[nRegGrpIdx][nRdBufIdx + 5u]) & 0x07) << 16u);
                    if (nCurrentData & (0x00040000))
                    {
                        nCurrentData |= 0xFFF80000;
                    }
                    break;
                default:
                    nRet |= E_NOT_OK;
            }
            nCurrentData |= ((uint32_t)anReadCurrentDataBuf[nRegGrpIdx][nRdBufIdx + 4u] << 8u);
            nCurrentData |= (uint32_t)anReadCurrentDataBuf[nRegGrpIdx][nRdBufIdx + 3u];
            pnCurrentData[nBufIdx++] = nCurrentData;

			/*Clear the I2 values in Current and Average Current Register groups*/
			nCmd = ADBMS6948_CMD_CLRSI;
			Adbms6948_Cmn_SendCommandOnly(nCmd, TRUE, knChainID);
        }
	}
	if ((E_OK == nRet) && (TRUE == bSendUnsnap))
	{
		nCmdCode = ADBMS6948_CMD_UNSNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
	}
	/* Verify the command counter as CLRCI or CLRSI or both commands were sent*/
	nRet |= Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);

return(nRet);
}
/*****************************************************************************/
/*!
	@brief	Local function to read current and voltage from I1-ADC and
			C-ADC respectively.

	@param [out] 	pnCellData		Place holder to report the cell measurement
									data.
									@range: NA
									@resolution: NA
    @param	[out]   pnCurrentData	Placeholder to report the current
									measurement data
									@range: NA
									@resolution: NA
    @param	[in]    eSnapSel		Specifies the snapshot command/s to be sent.
							        @range: Enumeration.
							        @resolution: NA

	@param 	[in] 	knChainID		Specifies ID of the daisy chain on which the
	                        		operation is to be performed.
									@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									@resolution: NA
    @return     E_OK: Success   \n
                E_NOT_OK: Failed\n
 */
/*****************************************************************************/
static Adbms6948_ReturnType Adbms6948_lReadCADCI1ADC
(
    int16_t                 *pnCellData,
    int32_t                 *pnCurrentData,
	Adbms6948_SnapSelType	eSnapSel,
	const uint8_t			knChainID
)
{
	Adbms6948_ReturnType  nRet;
	uint8_t  anReadDataBuf[ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITHOUT_PEC * ADBMS6948_NUMOF_REGGRP_RDCIV) + ADBMS6948_PEC_LEN];
	uint16_t  nCmd;
	uint16_t  nCmdCode, nCellData, nRdBufIdx;
	uint32_t nCurrentData;
	uint8_t  nDataLenBytesWithPec;
	boolean  bSendSnap, bSendUnsnap;

	nRdBufIdx = 0u;

	bSendSnap = (boolean)((uint8_t)eSnapSel & 0x01u);
	bSendUnsnap = (boolean)(((uint8_t)eSnapSel & 0x02u) >> 1u);
	nDataLenBytesWithPec = Adbms6948_ReadAllCommandBytes[ADBMS6948_READALL_VOLT_CURRENT];
	if(TRUE == bSendSnap)
	{
		/*Send SNAP command */
		nCmdCode = ADBMS6948_CMD_SNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
	}
	/* Send RDCIV command*/
	nCmd = ADBMS6948_CMD_RDCIV;
	nRet = Adbms6948_Cmn_ReadAllRegGroups(nCmd, anReadDataBuf, nDataLenBytesWithPec, knChainID);

	if (E_OK == nRet)
	{
		for(uint8_t nCellIdx = 0; nCellIdx < ADBMS6948_MAX_NO_OF_CELLS_PER_DEVICE; ++nCellIdx)
		{
			/* Populate the user data buffer. */
        	nCellData = (uint16_t)(((uint16_t)anReadDataBuf[nRdBufIdx + 1u] << 8u) |
									(uint16_t)anReadDataBuf[nRdBufIdx]);
        	pnCellData[nCellIdx] = nCellData;
			nRdBufIdx += 2;
		}
		nCurrentData = (uint32_t)(((((uint32_t)anReadDataBuf[nRdBufIdx + 2u]) & 0x07) << 16u) |
									((uint32_t)anReadDataBuf[nRdBufIdx + 1u] << 8u) |
									((uint32_t)anReadDataBuf[nRdBufIdx]));
        if (nCurrentData & (0x00040000))
        {
            nCurrentData |= 0xFFF80000;
        }
        pnCurrentData[0] = nCurrentData;
	}
	if ((E_OK == nRet) && (TRUE == bSendUnsnap))
	{
		nCmdCode = ADBMS6948_CMD_UNSNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
	}

	/*Send CLRCELL and CLRCI commands to clear Cell voltages and I1 values*/
	nCmd = ADBMS6948_CMD_CLRCELL;
	Adbms6948_Cmn_SendCommandOnly(nCmd, TRUE, knChainID);
	nCmd = ADBMS6948_CMD_CLRCI;
	Adbms6948_Cmn_SendCommandOnly(nCmd, TRUE, knChainID);

	/* Verify the command counter as CLRCELL and CLRCI commands were sent*/
	nRet |= Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);

return(nRet);
}
/*****************************************************************************/
/*!
    @brief          Local function to read average cell voltages and current
					from C-ADC and I1-ADC respectively.

	@param	[out]	pnCellData		    Placeholder to report the cell voltage data
									    @range: NA
									    @resolution: NA
    @param	[out]	pnCurrentData 	    Placeholder to report the current data
									    @range: NA
									    @resolution: NA
    @param  [out]   pnVoltageCTValue    10-bit CT value of Voltage ADC
                                    	@range: 0 to 2047.
                                    	@resolution: 1 count
    @param  [out]   pnCurrentCTValue   	10-bit CT value of Current ADC
                                    	@range: 0 to 2047.
                                    	@resolution: 1 count
    @param  [in]    bSendUnsnapAtEnd    Specifies whether to send UNSNAP command
                                        after reading cell voltages and current.
									    @range: FALSE, TRUE
									    @resolution: NA
	@param 	[in]	knChainID		    Specifies ID of the daisy chain on which the
	                        		    operation is to be performed.
									    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									    @resolution: NA
	@return			E_OK: Success    \n
					E_NOT_OK: Failed \n
 */
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadCellVoltCurrentAverage
(
	int16_t 					    *pnCellData,
    int32_t                         *pnCurrentData,
    boolean                         bSendUnsnapAtEnd,
	uint16_t						*pnVoltageCTValue,
	uint16_t						*pnCurrentCTValue,
	const uint8_t  					knChainID
)
{
	Adbms6948_ReturnType  nRet;
	uint8_t anRDSTATBuff[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + ADBMS6948_REG_DATA_LEN_WITH_PEC];
	uint8_t anReadDataBuf[ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITHOUT_PEC * ADBMS6948_NUMOF_REGGRP_RDCIV) + ADBMS6948_PEC_LEN];
	uint16_t nVoltageCT, nCurrentCT;
	uint16_t nRdBufIdx = 0u;
	uint16_t nCellData, nCmd;
	uint32_t nCurrentData;
	uint8_t nDataLenBytesWithPec;
	boolean bCfgRead = FALSE;
    boolean bResultsReady = FALSE;
    while(1)
    {
        /*Send SNAP command*/
        Adbms6948_lSendSnap(knChainID);

        /* Send RDSTATG command to read CT bits */
        nCmd = ADBMS6948_CMD_RDSTATG;
        nRet = Adbms6948_Cmn_ReadRegGroup(&nCmd, anRDSTATBuff, 1, knChainID);

        /*Get the CT bits*/
        uint8_t nRDSTATBufIdx = (uint16_t)ADBMS6948_CMD_DATA_LEN;
        nVoltageCT = (uint16_t)(((uint16_t)(anRDSTATBuff[0][nRDSTATBufIdx] & 0x1F) << 6u) |
                                ((uint16_t)(anRDSTATBuff[0][nRDSTATBufIdx + 1u] & 0xFCu) >> 2u));
        nCurrentCT = (uint16_t)(((uint16_t)(anRDSTATBuff[0][nRDSTATBufIdx + 2u] & 0x1F) << 6u) |
                                ((uint16_t)(anRDSTATBuff[0][nRDSTATBufIdx + 3u] & 0xFCu) >> 2u));

        /*If CT values are multiples of 8, implies conversion complete.*/
        if ((((nCurrentCT & 0x07u) == 0) && ((nVoltageCT & 0x07u) == 0)))
        {
            bResultsReady = TRUE;
			if (pnVoltageCTValue != NULL)
			{
				*pnVoltageCTValue = nVoltageCT;
			}

			if (pnCurrentCTValue != NULL)
			{
				*pnCurrentCTValue = nCurrentCT;
			}
            break;
        }
        else
        {
            /*Send UNSNAP command so that CT values can update.*/
            Adbms6948_lSendUnsnap(knChainID);
        }
    }

	if (TRUE == bResultsReady)
	{
		/*Send RDACIV command to read average current and voltage values*/
		nCmd = ADBMS6948_CMD_RDACIV;
		nDataLenBytesWithPec = Adbms6948_ReadAllCommandBytes[ADBMS6948_READALL_VOLT_CURRENT];
		nRet = Adbms6948_Cmn_ReadAllRegGroups(nCmd, anReadDataBuf, nDataLenBytesWithPec, knChainID);
        /*TODO:SA Add CLRCELL, CLRCI, CLRSI commands here*/
	}
	if (E_OK == nRet)
	{
		for(uint8_t nCellIdx = 0; nCellIdx < ADBMS6948_MAX_NO_OF_CELLS_PER_DEVICE; ++nCellIdx)
		{
			/* Populate the user data buffer. */
        	nCellData = (uint16_t)(((uint16_t)anReadDataBuf[nRdBufIdx + 1u] << 8u) |
									(uint16_t)anReadDataBuf[nRdBufIdx]);
        	pnCellData[nCellIdx] = nCellData;
			nRdBufIdx += 2;
		}
        /*I1*/
		nCurrentData = (uint32_t)(((((uint32_t)anReadDataBuf[nRdBufIdx + 2u]) & 0xFFu) << 16u) |
								   (((uint32_t)anReadDataBuf[nRdBufIdx + 1u]) & 0xFFu) << 8u) |
								   (((uint32_t)anReadDataBuf[nRdBufIdx + 0u]) & 0xFFu);
        if (nCurrentData & (0x00800000))
        {
            nCurrentData |= 0xFF000000;
        }

		*pnCurrentData = nCurrentData;

        if (TRUE == bSendUnsnapAtEnd)
        {
            Adbms6948_lSendUnsnap(knChainID);
            bCfgRead = TRUE;
        }
	}

	/*Send CLRCELL and CLRCI commands to clear Cell voltages and I1 values*/
	nCmd = ADBMS6948_CMD_CLRCELL;
	Adbms6948_Cmn_SendCommandOnly(nCmd, TRUE, knChainID);
	nCmd = ADBMS6948_CMD_CLRCI;
	Adbms6948_Cmn_SendCommandOnly(nCmd, TRUE, knChainID);

	/* Verify the command counter as CLRCELL and CLRCI commands were sent*/
	nRet |= Adbms6948_Cmn_VerifyCmdCnt(bCfgRead, knChainID);

return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	Local function to prepare the read command codes.
    @param [in] 	eGpioMeasData	Specifies type of GPIO measurement data.
									@range: Enumeration
									@resolution: NA
    @param [in] 	eGpioGrpSel		Specifies the GPIO group/s to be read.
									@range: Enumeration
									@resolution: NA
	@param [out] 	panCmdList		Place holder to populate the read command list.
									data.
									@range: NA
									@resolution: NA
	@param [in] pnCmdListCnt		Specifies number of read commands to be sent.
									@range: NA.
									@resolution: NA
    @return     None
 */
/*****************************************************************************/
static void  Adbms6948_lGetGPIOCmdList
(
	Adbms6948_GpioMeasDataType  	eGpioMeasData,
	Adbms6948_GpioGrpSelType  		eGpioGrpSel,
	uint16_t 					   *panCmdList,
	uint8_t 					   *pnCmdListCnt
)
{
	uint8_t  nLoopIdx, nNumOfGroups, nCmdIdx = 0u;
	if (ADBMS6948_GPIO_GRP_ALL == eGpioGrpSel)
	{
		nNumOfGroups = ADBMS6948_MAX_GPIODATA_REGGRPS;
		for (nLoopIdx = 0; nLoopIdx < nNumOfGroups; nLoopIdx++)
		{
			panCmdList[nCmdIdx++] = Adbms6948_GpioDataReadCmds[(uint8_t)eGpioMeasData][(uint8_t)nLoopIdx];
		}
	}
	else
	{
		nNumOfGroups = ADBMS6948_NUMOF_REGGRP_ONE;
		panCmdList[nCmdIdx] = Adbms6948_GpioDataReadCmds[(uint8_t)eGpioMeasData][(uint8_t)eGpioGrpSel];
	}

	*pnCmdListCnt = nNumOfGroups;
}
/*****************************************************************************/
 /*!
    @brief	Local function to read GPIO input voltage.
    @param [in] 	eGpioMeasData	Specifies type of GPIO measurement data.
									@range: Enumeration
									@resolution: NA
    @param [in] 	eGpioGrpSel		Specifies the GPIO group/s to be read.
									@range: Enumeration
									@resolution: NA
	@param [out] 	pnGpioData		Place holder to report the GPIO measurement
									data.
									@range: NA
									@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadGPIOInputVolt
(
	Adbms6948_GpioMeasDataType  	eGpioMeasData,
	Adbms6948_GpioGrpSelType  		eGpioGrpSel,
	int16_t 					   *pnGpioData,
	const uint8_t  					knChainID
)
{
	Adbms6948_ReturnType  nRet;
	uint8_t  anReadGpioDataBuf[ADBMS6948_MAX_GPIODATA_REGGRPS][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
	uint16_t  anCmdList[ADBMS6948_MAX_GPIODATA_REGGRPS];
	uint16_t  nGpioData, nRdBufIdx;
	uint8_t  nRegGrpIdx, nNumOfCmds, nBufIdx = 0u;

	/* Send Read commands*/
	Adbms6948_lGetGPIOCmdList(eGpioMeasData, eGpioGrpSel, anCmdList, &nNumOfCmds);
	nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, anReadGpioDataBuf, nNumOfCmds, knChainID);

	if (E_OK == nRet)
	{
		/* Populate the user data buffer. */
		for (nRegGrpIdx = 0; nRegGrpIdx < nNumOfCmds; nRegGrpIdx++)
		{
            nRdBufIdx = (uint16_t)ADBMS6948_CMD_DATA_LEN;
            nGpioData = (int16_t)(((uint16_t)anReadGpioDataBuf[nRegGrpIdx][nRdBufIdx + 1u] << 8u) |
                                    (uint16_t)anReadGpioDataBuf[nRegGrpIdx][nRdBufIdx]);
            pnGpioData[nBufIdx++] = nGpioData;
            /*Read the next GPIO in same register group*/
            nGpioData = (int16_t)(((uint16_t)anReadGpioDataBuf[nRegGrpIdx][nRdBufIdx + 3u] << 8u) |
                                    (uint16_t)anReadGpioDataBuf[nRegGrpIdx][nRdBufIdx + 2u]);
            pnGpioData[nBufIdx++] = nGpioData;
            /*Read other GPIO values if not the last register group*/
            if (nRegGrpIdx != ADBMS6948_LAST_GPIO_REGGRP)
            {
                /*Read the next GPIO in the same register group*/
                nGpioData = (int16_t)(((uint16_t)anReadGpioDataBuf[nRegGrpIdx][nRdBufIdx + 5u] << 8u) |
                            (uint16_t)anReadGpioDataBuf[nRegGrpIdx][nRdBufIdx + 4u]);
                pnGpioData[nBufIdx++] = nGpioData;
            }
        }
    }
return(nRet);
}
/*****************************************************************************/
 /*!
    @brief	Local function to prepare the read command codes.
    @param [in] eDevParamGrpSel	Specifies the device parameter group/s
     	 	 	 	 	 	 	to be read.
								@range: Enumeration
								@resolution: NA
	@param [out] panCmdList		Place holder to populate the read command list.
								data.
								@range: NA
								@resolution: NA
	@param [in] pnCmdListCnt	Specifies number of read commands to be sent.
								@range: NA.
								@resolution: NA
    @return     None
 */
/*****************************************************************************/
static void  Adbms6948_lGetDevParamCmdList
(
	Adbms6948_DevParamGrpSelType  	 eDevParamGrpSel,
	uint16_t 						*panCmdList,
	uint8_t 						*pnCmdListCnt
)
{
	uint8_t  nLoopIdx, nNumOfGroups, nCmdIdx = 0u;
	if (ADBMS6948_DEV_PARAMS_ALL == eDevParamGrpSel)
	{
		nNumOfGroups = ADBMS6948_MAX_DEV_PARAM_REGGRPS;
		for (nLoopIdx = 0; nLoopIdx < ADBMS6948_MAX_DEV_PARAM_REGGRPS; nLoopIdx++)
		{
			panCmdList[nCmdIdx++] = Adbms6948_DevParamReadCmds[nLoopIdx];
		}
	}
	else
	{
		nNumOfGroups = ADBMS6948_NUMOF_REGGRP_ONE;
		panCmdList[nCmdIdx] = Adbms6948_DevParamReadCmds[(uint8_t)eDevParamGrpSel];
	}

	*pnCmdListCnt = nNumOfGroups;
return;
}
/*****************************************************************************/
 /*!
    @brief	Local function to read device parameter.
    @param [in] 	eDevParamGrpSel	Specifies the device parameter group to
									be read.
									@range: Enumeration
									@resolution: NA
	@param [out] 	pnDevParamData	Place holder to report the internal device
									parameter measurement data.
									@range: NA
									@resolution: NA
	@param [in] knChainID	Specifies ID of the daisy chain on which the
	                        operation is to be performed.
							@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
							@resolution: NA
    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadDevParam
(
	Adbms6948_DevParamGrpSelType  	 eDevParamGrpSel,
	int16_t 						*pnDevParamData,
	const uint8_t  					 knChainID
)
{
	Adbms6948_ReturnType nRet;
	uint8_t  anReadDevParamBuf[ADBMS6948_MAX_DEV_PARAM_REGGRPS][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
	uint16_t  anCmdList[ADBMS6948_MAX_DEV_PARAM_REGGRPS];
	uint16_t  nDevParam, nRdBufIdx;
	uint8_t  nRegGrpIdx, nNumOfCmds, nBufIdx = 0u;

	/* Send Read commands*/
	Adbms6948_lGetDevParamCmdList(eDevParamGrpSel, anCmdList, &nNumOfCmds);
	nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, anReadDevParamBuf, nNumOfCmds, knChainID);

	if (E_OK == nRet)
	{
		/* Populate the user data buffer. */
		for (nRegGrpIdx = 0; nRegGrpIdx < nNumOfCmds; nRegGrpIdx++)
		{
            nRdBufIdx = (uint16_t)ADBMS6948_CMD_DATA_LEN;
            nDevParam = (int16_t)((uint16_t)((uint16_t)anReadDevParamBuf[nRegGrpIdx][nRdBufIdx + 1u] << 8u) |
                                   ((uint16_t)anReadDevParamBuf[nRegGrpIdx][nRdBufIdx]));
            pnDevParamData[nBufIdx++] = nDevParam;

            nDevParam = (int16_t)((uint16_t)((uint16_t)anReadDevParamBuf[nRegGrpIdx][nRdBufIdx + 3u] << 8u) |
                                              ((uint16_t)anReadDevParamBuf[nRegGrpIdx][nRdBufIdx + 2u]));
            pnDevParamData[nBufIdx++] = nDevParam;

            if ((nRegGrpIdx != ADBMS6948_FIRST_DEVPARAM_REGGRP) || (ADBMS6948_DEV_PARAMS_P3P4P5 == eDevParamGrpSel) || (ADBMS6948_DEV_PARAMS_P6P7P8 == eDevParamGrpSel))
            {
                nDevParam = (int16_t) ((uint16_t)((uint16_t)anReadDevParamBuf[nRegGrpIdx][nRdBufIdx + 5u] << 8u) |
                                        ((uint16_t)anReadDevParamBuf[nRegGrpIdx][nRdBufIdx + 4u]));
                pnDevParamData[nBufIdx++] = nDevParam;
            }
		}
	}
return(nRet);
}
/*****************************************************************************/
/*!
    @brief          Local function to read all the GPIO, Redundant GPIO and Status
                    registers.

    @param  [out]   panGpioData             Placeholder to store the GPIO
                                            values.
                                            @range: NA
                                            @resolution: NA
    @param  [out]   panRedundantGpioData    Placeholder to store the Redundant
                                            GPIO values.
                                            @range: NA
                                            @resolution: NA
    @param  [out]   poStatusRegsVal         Placeholder to store the Status
                                            register values.
                                            @range: NA
                                            @resolution: NA
    @param  [in]    knChainID               Specifies ID of the daisy chain on
                                            which the operation is to be
                                            performed.
                                            @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                            @resolution: NA
    @return     E_OK: Success   \n
                E_NOT_OK: Failed\n
 */
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadAllAuxStatusRegisters
(
    int16_t                         *panGpioData,
    int16_t                         *panRedundantGpioData,
    Adbms6948_StatusRegValsType     *poStatusRegsVal,
    const uint8_t                    knChainID
)
{
    Adbms6948_ReturnType nRet;
    uint8_t  anReadAuxStatusBuf[ADBMS6948_RDALL_AUX_STATUS_BYTES] = {0u};
    uint8_t  nDataBytesLenWithPec;
    uint16_t nRdBufIdx;

    nDataBytesLenWithPec = Adbms6948_ReadAllCommandBytes[ADBMS6948_READALL_AUX_STATUS];

    /*Send RDASALL command and read data*/
    nRet = Adbms6948_Cmn_ReadAllRegGroups(ADBMS6948_CMD_RDASALL, &anReadAuxStatusBuf[0], nDataBytesLenWithPec, knChainID);

    if (E_OK == nRet)
    {
        /*Populate the user buffer for GPIO data*/
        ADI_PAL_MEMCPY((uint8_t *)panGpioData, anReadAuxStatusBuf, ADBMS6948_RDASALL_GPIO_BYTES);
        /*Populate the user buffer for Redundant GPIO data*/
        ADI_PAL_MEMCPY((uint8_t *)panRedundantGpioData, anReadAuxStatusBuf + ADBMS6948_RDASALL_GPIO_BYTES, ADBMS6948_RDASALL_RD_GPIO_BYTES);
        /*Populate the user buffer for Status Register A*/
        nRdBufIdx = ADBMS6948_RDASALL_GPIO_BYTES + ADBMS6948_RDASALL_RD_GPIO_BYTES;
        poStatusRegsVal->nVRef2 =   (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 1u] << 8u) |
                                                (uint16_t)anReadAuxStatusBuf[nRdBufIdx]);
        poStatusRegsVal->nITMP  =   (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 3u] << 8u) |
                                                (uint16_t)anReadAuxStatusBuf[nRdBufIdx + 2u]);
        poStatusRegsVal->nVRef3 =   (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 5u] << 8u) |
                                                (uint16_t)anReadAuxStatusBuf[nRdBufIdx + 4u]);
        nRdBufIdx += ADBMS6948_RDASALL_STATUSA_BYTES;

        /*Populate the user buffer for Status Register B*/
        poStatusRegsVal->nVD    =   (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 1u] << 8u) |
                                                (uint16_t)anReadAuxStatusBuf[nRdBufIdx]);
        poStatusRegsVal->nVA    =   (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 3u] << 8u) |
                                                (uint16_t)anReadAuxStatusBuf[nRdBufIdx + 2u]);
        poStatusRegsVal->nVR4K  =   (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 5u] << 8u) |
                                                (uint16_t)anReadAuxStatusBuf[nRdBufIdx + 4u]);
        nRdBufIdx += ADBMS6948_RDASALL_STATUSB_BYTES;

        /*Populate the user buffer for Status Register C*/
        poStatusRegsVal->nCSxFault      =   (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 1u] << 8u) |
                                                        (uint16_t)anReadAuxStatusBuf[nRdBufIdx]);
        poStatusRegsVal->nCompI         =   (anReadAuxStatusBuf[nRdBufIdx + 2u] & ADBMS6948_STCR2_MASK_COMP_I) >> ADBMS6948_STCR2_POS_COMP_I;
        poStatusRegsVal->nOC1           =   (anReadAuxStatusBuf[nRdBufIdx + 2u] & ADBMS6948_STCR2_MASK_OC1) >> ADBMS6948_STCR2_POS_OC1;
        poStatusRegsVal->nOC2           =   (anReadAuxStatusBuf[nRdBufIdx + 2u] & ADBMS6948_STCR2_MASK_OC2) >> ADBMS6948_STCR2_POS_OC2;
        poStatusRegsVal->nOCWarn1       =   (anReadAuxStatusBuf[nRdBufIdx + 2u] & ADBMS6948_STCR2_MASK_OCW1) >> ADBMS6948_STCR2_POS_OCW1;
        poStatusRegsVal->nOCWarn2       =   (anReadAuxStatusBuf[nRdBufIdx + 2u] & ADBMS6948_STCR2_MASK_OCW2) >> ADBMS6948_STCR2_POS_OCW2;
        poStatusRegsVal->nCoulCntFlt    =   (anReadAuxStatusBuf[nRdBufIdx + 2u] & ADBMS6948_STCR2_MASK_CCFLT) >> ADBMS6948_STCR2_POS_CCFLT;
        poStatusRegsVal->nTimeBaseFlt   =   (anReadAuxStatusBuf[nRdBufIdx + 2u] & ADBMS6948_STCR2_MASK_TB_OF) >> ADBMS6948_STCR2_POS_TB_OF;
        poStatusRegsVal->nCoulCntSeqErr =   (anReadAuxStatusBuf[nRdBufIdx + 2u] & ADBMS6948_STCR2_MASK_SEQCC) >> ADBMS6948_STCR2_POS_SEQCC;

        poStatusRegsVal->nSMED          =   (anReadAuxStatusBuf[nRdBufIdx + 4u] & ADBMS6948_STCR4_MASK_SMED) >> ADBMS6948_STCR4_POS_SMED;
        poStatusRegsVal->nSED           =   (anReadAuxStatusBuf[nRdBufIdx + 4u] & ADBMS6948_STCR4_MASK_SED) >> ADBMS6948_STCR4_POS_SED;
        poStatusRegsVal->nCMED          =   (anReadAuxStatusBuf[nRdBufIdx + 4u] & ADBMS6948_STCR4_MASK_CMED) >> ADBMS6948_STCR4_POS_CMED;
        poStatusRegsVal->nCED           =   (anReadAuxStatusBuf[nRdBufIdx + 4u] & ADBMS6948_STCR4_MASK_CED) >> ADBMS6948_STCR4_POS_CED;
        poStatusRegsVal->nVD_UV         =   (anReadAuxStatusBuf[nRdBufIdx + 4u] & ADBMS6948_STCR4_MASK_VD_UV) >> ADBMS6948_STCR4_POS_VD_UV;
        poStatusRegsVal->nVD_OV         =   (anReadAuxStatusBuf[nRdBufIdx + 4u] & ADBMS6948_STCR4_MASK_VD_OV) >> ADBMS6948_STCR4_POS_VD_OV;
        poStatusRegsVal->nVA_UV         =   (anReadAuxStatusBuf[nRdBufIdx + 4u] & ADBMS6948_STCR4_MASK_VA_UV) >> ADBMS6948_STCR4_POS_VA_UV;
        poStatusRegsVal->nVA_OV         =   (anReadAuxStatusBuf[nRdBufIdx + 4u] & ADBMS6948_STCR4_MASK_VA_OV) >> ADBMS6948_STCR4_POS_VA_OV;

        poStatusRegsVal->nOscChk        =   (anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STCR5_MASK_OSCHK) >> ADBMS6948_STCR5_POS_OSCHK;
        poStatusRegsVal->nTModChk       =   (anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STCR5_MASK_TMODCHK) >> ADBMS6948_STCR5_POS_TMODCHK;
        poStatusRegsVal->nThSd          =   (anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STCR5_MASK_THSD) >> ADBMS6948_STCR5_POS_THSD;
        poStatusRegsVal->nSleep         =   (anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STCR5_MASK_SLEEP) >> ADBMS6948_STCR5_POS_SLEEP;
        poStatusRegsVal->nSpiFlt        =   (anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STCR5_MASK_SPIFLT) >> ADBMS6948_STCR5_POS_SPIFLT;
        poStatusRegsVal->nCompV         =   (anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STCR5_MASK_COMP_V) >> ADBMS6948_STCR5_POS_COMP_V;
        poStatusRegsVal->nVDeLatent     =   (anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STCR5_MASK_VDEL) >> ADBMS6948_STCR5_POS_VDEL;
        poStatusRegsVal->nVDe           =   (anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STCR5_MASK_VDE) >> ADBMS6948_STCR5_POS_VDE;

        nRdBufIdx += ADBMS6948_RDASALL_STATUSC_BYTES;

        /*Populate the user buffer for Status Register D*/
        poStatusRegsVal->nCellOVUVStatus  = (uint32_t)anReadAuxStatusBuf[nRdBufIdx];
        poStatusRegsVal->nCellOVUVStatus |= (uint32_t)((uint32_t)anReadAuxStatusBuf[nRdBufIdx + 1u] << 8u);
        poStatusRegsVal->nCellOVUVStatus |= (uint32_t)((uint32_t)anReadAuxStatusBuf[nRdBufIdx + 2u] << 16u);
        poStatusRegsVal->nCellOVUVStatus |= (uint32_t)((uint32_t)anReadAuxStatusBuf[nRdBufIdx + 3u] << 24u);
        poStatusRegsVal->nOscChkCnt       = anReadAuxStatusBuf[nRdBufIdx + 5u];

        nRdBufIdx += ADBMS6948_RDASALL_STATUSD_BYTES;

        /*Populate the user buffer for Status Register E*/
        poStatusRegsVal->nOC1Val            = (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 1u] << 8u) |
                                                         ((uint16_t)anReadAuxStatusBuf[nRdBufIdx]));
        poStatusRegsVal->nOC2Val            = (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 3u] << 8u) |
                                                         ((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 2u]));

        poStatusRegsVal->nGpioPinStates     =  (uint16_t)((((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STER5_MASK_GPI) << 8u) |
                                                            ((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 4u]));

        poStatusRegsVal->nLPCMFltBStatus    = (anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STER5_MASK_FAULTB) >> ADBMS6948_STER5_POS_FAULTB;
        poStatusRegsVal->nDevRevCode        = (anReadAuxStatusBuf[nRdBufIdx + 5u] & ADBMS6948_STER5_MASK_REV) >> ADBMS6948_STER5_POS_REV;

        nRdBufIdx += ADBMS6948_RDASALL_STATUSE_BYTES;

        /*Populate the user buffer for Status Register F*/
        poStatusRegsVal->nVMV               = (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 1u] << 8u) |
                                               ((uint16_t)anReadAuxStatusBuf[nRdBufIdx]));
        poStatusRegsVal->nVP                = (uint16_t)(((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 3u] << 8u) |
                                               ((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 2u]));
        nRdBufIdx += ADBMS6948_RDASALL_STATUSF_BYTES;

        /*Populate the user buffer for Status Register G*/
        poStatusRegsVal->nCTV   = (uint16_t)(((uint16_t)(anReadAuxStatusBuf[nRdBufIdx + 1u] & ADBMS6948_STGR1_MASK_CT_V) >> ADBMS6948_STGR1_POS_CT_V) |
                                             (((uint16_t)anReadAuxStatusBuf[nRdBufIdx] & ADBMS6948_STGR0_MASK_CT_V) << 6u));
        poStatusRegsVal->nCTSV  = (anReadAuxStatusBuf[nRdBufIdx + 1u] & ADBMS6948_STGR1_MASK_CTS_V) >> ADBMS6948_STGR1_POS_CTS_V;
        poStatusRegsVal->nCTI   = (uint16_t)(((uint16_t)(anReadAuxStatusBuf[nRdBufIdx + 3u] & ADBMS6948_STGR3_MASK_CT_I) >> ADBMS6948_STGR3_POS_CT_I) |
                                             (((uint16_t)anReadAuxStatusBuf[nRdBufIdx + 2u] & ADBMS6948_STGR2_MASK_CT_I) << 6u));
        poStatusRegsVal->nCTSV  = (anReadAuxStatusBuf[nRdBufIdx + 3u] & ADBMS6948_STGR3_MASK_CTS_I) >> ADBMS6948_STGR3_POS_CTS_I;
    }
return(nRet);
}
/*****************************************************************************/
/*!
    @brief  Local function get the command code for control command.

	@param [in] eCtrlCmdSel	Specifies the control command to be sent.
							@range: Enumeration
							@resolution: NA
	@param [out] pCtrlCmdCode	Placeholder for the command code.
								@range: NA
								@resolution: NA
	@param [out] pbIsIncrCmd	Placeholder to mention whether the command
								increment the CC.
								@range: NA
								@resolution: NA
    @return     None
 */
/*****************************************************************************/
static void  Adbms6948_lGetCtrlCmdCode
(
Adbms6948_CtrlCmdSelType  	 eCtrlCmdSel,
uint16_t 					*pCtrlCmdCode,
boolean 					*pbIsIncrCmd
)
{

	switch(eCtrlCmdSel)
	{
		case ADBMS6948_CTRL_CMD_SNAP:
			*pCtrlCmdCode = ADBMS6948_CMD_SNAP;
			*pbIsIncrCmd = TRUE;
			break;
		case ADBMS6948_CTRL_CMD_UNSNAP:
			*pCtrlCmdCode = ADBMS6948_CMD_UNSNAP;
			*pbIsIncrCmd = TRUE;
			break;
		case ADBMS6948_CTRL_CMD_MUTE:
			*pCtrlCmdCode = ADBMS6948_CMD_MUTE;
			*pbIsIncrCmd = TRUE;
			break;
		case ADBMS6948_CTRL_CMD_UNMUTE:
			*pCtrlCmdCode = ADBMS6948_CMD_UNMUTE;
			*pbIsIncrCmd = TRUE;
			break;
		case ADBMS6948_CTRL_CMD_RSTCC:
			*pCtrlCmdCode = ADBMS6948_CMD_RSTCC;
			*pbIsIncrCmd = FALSE;
			break;
		case ADBMS6948_CTRL_CMD_CLRCELL:
			*pCtrlCmdCode = ADBMS6948_CMD_CLRCELL;
			*pbIsIncrCmd = TRUE;
			break;
		case ADBMS6948_CTRL_CMD_CLRFC:
			*pCtrlCmdCode = ADBMS6948_CMD_CLRFC;
			*pbIsIncrCmd = TRUE;
			break;
		case ADBMS6948_CTRL_CMD_CLRAUX:
			*pCtrlCmdCode = ADBMS6948_CMD_CLRAUX;
			*pbIsIncrCmd = TRUE;
			break;
		case ADBMS6948_CTRL_CMD_CLRSPIN:
			*pCtrlCmdCode = ADBMS6948_CMD_CLRSPIN;
			*pbIsIncrCmd = TRUE;
			break;
        case ADBMS6948_CTRL_CMD_CLRCC:
			*pCtrlCmdCode = ADBMS6948_CMD_CLRCC;
			*pbIsIncrCmd = TRUE;
			break;
        case ADBMS6948_CTRL_CMD_CLRCI:
			*pCtrlCmdCode = ADBMS6948_CMD_CLRCI;
			*pbIsIncrCmd = TRUE;
			break;
        case ADBMS6948_CTRL_CMD_CLRSI:
			*pCtrlCmdCode = ADBMS6948_CMD_CLRSI;
			*pbIsIncrCmd = TRUE;
			break;
		case ADBMS6948_CTRL_CMD_ULAO:
			*pCtrlCmdCode = ADBMS6948_CMD_ULAO;
			*pbIsIncrCmd = TRUE;
		default:
			*pCtrlCmdCode = ADBMS6948_CMD_INVALID;
			*pbIsIncrCmd = FALSE;
			break;
	}
return;
}
/*****************************************************************************/
/*!
    @brief  Local function get the command code for control command.
	@param [in] eADCSel		Specifies the ADC/s to be polled for the status.
							@range: Enumeration
							@resolution: NA
	@param [out] pPollAdcCmdCode	Placeholder for the command code.
								@range: NA
								@resolution: NA
    @return     None
 */
/*****************************************************************************/
static void  Adbms6948_lGetPollADCCmdCode
(
Adbms6948_ADCSelType  	eADCSel,
uint16_t 			   *pPollAdcCmdCode
)
{
	switch(eADCSel)
	{
		case ADBMS6948_ADC_ALL:
			*pPollAdcCmdCode = ADBMS6948_CMD_PLADC;
			break;
		case ADBMS6948_ADC_CADC:
			*pPollAdcCmdCode = ADBMS6948_CMD_PLCADC;
			break;
		case ADBMS6948_ADC_SADC:
			*pPollAdcCmdCode = ADBMS6948_CMD_PLSADC;
			break;
		case ADBMS6948_ADC_AUX:
			*pPollAdcCmdCode = ADBMS6948_CMD_PLAUX1;
			break;
		case ADBMS6948_ADC_AUX2:
			*pPollAdcCmdCode = ADBMS6948_CMD_PLAUX2;
			break;
		case ADBMS6948_ADC_I1ADC:
			*pPollAdcCmdCode = ADBMS6948_CMD_PLI1ADC;
			break;
		case ADBMS6948_ADC_I2ADC:
			*pPollAdcCmdCode = ADBMS6948_CMD_PLI2ADC;
			break;
		default:
			*pPollAdcCmdCode = ADBMS6948_CMD_INVALID;
			break;
	}
return;
}
/*****************************************************************************/
/*!
    @brief	Local function to poll the ADC status.
    @param [out]	nPollCmd  	Poll command to be sent.
								@range: Set of Poll ADC commands
								@resolution: NA
	@param 	[in] 	pbADCStatus	Place holder to report the poll ADC status.
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
static Adbms6948_ReturnType  Adbms6948_lPollADCStatus
(
	uint16_t 		 nPollCmd,
	boolean 		*pbADCStatus,
	const uint8_t  	 knChainID
)
{
	Adbms6948_ReturnType  	nRet;
	uint8_t  anRdBuf[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
	uint16_t  anCmdList[ADBMS6948_NUMOF_REGGRP_ONE];
	uint16_t  nDataLen, nADCStatusIdx, nNumOfInvalidBytes;
	uint32_t	nDataCfg;

	nNumOfInvalidBytes = (uint16_t)(((uint16_t)(Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices * (uint16_t)ADBMS6948_INVBITS_PER_DEV_POLL)/8u) + 1u);
	nDataLen = (uint16_t)((uint16_t)ADBMS6948_CMD_DATA_LEN + nNumOfInvalidBytes);
	anCmdList[0u] = nPollCmd;
	nDataCfg = (uint32_t)(((uint32_t)ADBMS6948_NUMOF_REGGRP_ONE << 16u) | ((uint32_t)nDataLen));

	Adbms6948_Cmn_ExecuteCmdRD(anCmdList, anRdBuf, nDataCfg, TRUE, knChainID);

	/*After sending a Poll command, if the conversions are not complete, MISO
	line is pulled LOW and is pulled HIGH either after end of conversion or if
	CSb is pulled high, so PEC check not valid here*/

	/*Verify the command counter value as poll commands increment the command
	counter*/
	nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
	if (E_OK == nRet)
	{
		nADCStatusIdx = (uint16_t)(nDataLen - 1u);
		/*If data read not 0, that means ADC conversions have finished*/
		if(anRdBuf[0u][nADCStatusIdx] != 0u)
		{
			*pbADCStatus = FALSE;
		}
		else
		{
			*pbADCStatus = TRUE;
		}
	}
return(nRet);
}
/*****************************************************************************/
/*!
    @brief	This function sends SNAP command to ADBMS6948 device

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return	E_OK: 		Success \n
        	E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
static void Adbms6948_lSendSnap
(
	const uint8_t  	 knChainID
)
{
	uint16_t nCmdCode = ADBMS6948_CMD_SNAP;
	Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
}
/*****************************************************************************/
/*!
    @brief	This function sends UNSNAP command to ADBMS6948 device

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA
    @return	E_OK: 		Success \n
        	E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
static void Adbms6948_lSendUnsnap
(
	const uint8_t  	 knChainID
)
{
	uint16_t nCmdCode = ADBMS6948_CMD_UNSNAP;
	Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
}
/*****************************************************************************/
/*!
    @brief          Local function to read the coulomb counter and time base
                    values.

    @param  [out]   pnCoulombCount  Placeholder to report the coulomb counter
                                    value.
                                    @range: 0 to (2^32 - 1)
                                    @resolution: 1 count
    @param  [out]   pnTimeBase      Placeholder to report the time base value.
                                    @range: 0 to (2^16 - 1)
                                    @resolution: 1 conversion time
    @param  [in]    eSnapSel        Specifies the snapshot command/s to be sent.
                                    @range: Enumeration.
                                    @resolution: NA
    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA
    @return         E_OK:       Success \n
                    E_NOT_OK:   Failure \n
 */
/*****************************************************************************/
static Adbms6948_ReturnType Adbms6948_lReadCoulombCountAndTimeBase
(
    uint32_t                *pnCoulombCount,
    uint16_t                *pnTimeBase,
	Adbms6948_SnapSelType   eSnapSel,
    const uint8_t           knChainID
)
{
    Adbms6948_ReturnType  nRet;
    uint16_t  nCmdCode;
    uint8_t  anRdBuf[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + ADBMS6948_REG_DATA_LEN_WITH_PEC];
    uint16_t  anCmdList[ADBMS6948_NUMOF_REGGRP_ONE];
    uint16_t  nTimeBase, nRdBufIdx;
    uint32_t nCoulombCnt;
    boolean bCfgRead = FALSE;

    if ((ADBMS6948_SEND_SNAP == eSnapSel) || (ADBMS6948_SEND_BOTH == eSnapSel))
    {
        nCmdCode = ADBMS6948_CMD_SNAP;
        Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
    }

    anCmdList[0u] = ADBMS6948_CMD_RDCT;
    nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, anRdBuf, ADBMS6948_NUMOF_REGGRP_ONE, knChainID);
    if (E_OK == nRet)
    {
        nRdBufIdx = (uint16_t)ADBMS6948_CMD_DATA_LEN;
        nCoulombCnt = (uint32_t)(((uint32_t)anRdBuf[0u][nRdBufIdx + 3u] << 24u) |
                                    ((uint32_t)anRdBuf[0u][nRdBufIdx + 2u] << 16u) |
                                    ((uint32_t)anRdBuf[0u][nRdBufIdx + 1u] << 8u)  |
                                    ((uint32_t)anRdBuf[0u][nRdBufIdx]));
        nTimeBase   = (uint16_t)(((uint16_t)anRdBuf[0u][nRdBufIdx + 5u] << 8u) |
                                    ((uint16_t)anRdBuf[0u][nRdBufIdx + 4u]));
        pnCoulombCount[0] = nCoulombCnt;
        pnTimeBase[0] = nTimeBase;
    }
    if ((E_OK == nRet) && ((ADBMS6948_SEND_UNSNAP == eSnapSel) || (ADBMS6948_SEND_BOTH == eSnapSel)))
    {
        nCmdCode = ADBMS6948_CMD_UNSNAP;
        Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
        bCfgRead = TRUE;
    }

    /* Verify the command counter if UNSNAP was sent*/
    nRet |= Adbms6948_Cmn_VerifyCmdCnt(bCfgRead, knChainID);
return(nRet);
}
/*****************************************************************************/
/*!
    @brief          Local function to clear all the fault flags in STATUS C
                    register

    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA
    @return         E_OK:       Success \n
                    E_NOT_OK:   Failure \n
 */
/*****************************************************************************/
static Adbms6948_ReturnType Adbms6948_lClearFlags
(
    const uint8_t                   knChainID
)
{
    Adbms6948_ReturnType nRet;
    uint16_t  nCmdCode;

    uint8_t anWrBuf[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + ADBMS6948_REG_DATA_LEN_WITH_PEC];

    /*Set all the bits that have to be cleared*/
    anWrBuf[0][ADBMS6948_CMD_DATA_LEN] = 0xFF;
    anWrBuf[0][ADBMS6948_CMD_DATA_LEN + 1u] = 0xFF;
    anWrBuf[0][ADBMS6948_CMD_DATA_LEN + 2u] = 0xFF;
    anWrBuf[0][ADBMS6948_CMD_DATA_LEN + 3u] = 0x00;
    anWrBuf[0][ADBMS6948_CMD_DATA_LEN + 4u] = 0xF0;
    anWrBuf[0][ADBMS6948_CMD_DATA_LEN + 5u] = 0xFF;

    /*Send CLRFLAG command along with all the Status C flags set to clear them*/
    nCmdCode = ADBMS6948_CMD_CLRFLAG;
    Adbms6948_Cmn_WriteRegGroup(&nCmdCode, ADBMS6948_REG_GRP_LEN, anWrBuf, 0u);

    /*Verify command counter after issuing CLRFLAG command*/
    nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
return nRet;
}
/*****************************************************************************/
/*!
    @brief          Local function to clears all the cell voltage registers.

    @param  [in]    eSnapSel    Specifies the snapshot command/s to be sent.
                                @range: Enumeration
                                @resolution: NA
    @param  [in]    knChainID   Specifies ID of the daisy chain on
                                which the operation is to be performed.
                                @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                @resolution: NA
    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
/*****************************************************************************/
static Adbms6948_ReturnType Adbms6948_lClearCellVoltageRegisters
(
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
)
{
    Adbms6948_ReturnType nRet;
    boolean   bSendSnap, bSendUnsnap;
    uint16_t  nCmdCode;
    boolean   bIsIncrCmd;

    bSendSnap = (boolean)((uint8_t)eSnapSel & 0x01u);
    bSendUnsnap = (boolean)(((uint8_t)eSnapSel & 0x02u) >> 1u);
    if(TRUE == bSendSnap)
    {
        /*Send SNAP command */
        nCmdCode = ADBMS6948_CMD_SNAP;
        Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
    }

    /*Send CLRCELL command*/
    Adbms6948_lGetCtrlCmdCode(ADBMS6948_CTRL_CMD_CLRCELL, &nCmdCode, &bIsIncrCmd);
    Adbms6948_Cmn_SendCommandOnly(nCmdCode, bIsIncrCmd, knChainID);

    if (TRUE == bSendUnsnap)
    {
        nCmdCode = ADBMS6948_CMD_UNSNAP;
        Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
    }
    /*Verify command counter after issuing either CLRCELL or UNSNAP command*/
    nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
return nRet;
}
/*****************************************************************************/
/*!
    @brief          Local function to clear the I1 or I2 current values in the
                    current registers based on @c eCurrentSel.

    @param  [in]    eCurrentSel     Specifies the current channel to clear
                                    data of.
                                    @range: Enumeration
                                    @resolution: NA
    @param  [in]    eSnapSel        Specifies the snapshot command/s to be sent.
                                    @range: Enumeration
                                    @resolution: NA
    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA
    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
/*****************************************************************************/
static Adbms6948_ReturnType Adbms6948_lClearCurrentRegisters
(
    Adbms6948_CurrentSelType        eCurrentSel,
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
)
{
    Adbms6948_ReturnType nRet;
    boolean   bSendSnap, bSendUnsnap;
    uint16_t  nCmdCode;
    boolean   bIsIncrCmd;

    bSendSnap = (boolean)((uint8_t)eSnapSel & 0x01u);
    bSendUnsnap = (boolean)(((uint8_t)eSnapSel & 0x02u) >> 1u);
    if(TRUE == bSendSnap)
    {
        /*Send SNAP command */
        nCmdCode = ADBMS6948_CMD_SNAP;
        Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
    }

    switch (eCurrentSel)
    {
        case ADBMS6948_CURRENT_I1:
            /*Send CLRCI command*/
            Adbms6948_lGetCtrlCmdCode(ADBMS6948_CTRL_CMD_CLRCI, &nCmdCode, &bIsIncrCmd);
            Adbms6948_Cmn_SendCommandOnly(nCmdCode, bIsIncrCmd, knChainID);
            break;
        case ADBMS6948_CURRENT_I2:
            /*Send CLRSI command*/
            Adbms6948_lGetCtrlCmdCode(ADBMS6948_CTRL_CMD_CLRSI, &nCmdCode, &bIsIncrCmd);
            Adbms6948_Cmn_SendCommandOnly(nCmdCode, bIsIncrCmd, knChainID);
            break;
        case ADBMS6948_CURRENT_I1I2:
            /*Send CLRCI command*/
            Adbms6948_lGetCtrlCmdCode(ADBMS6948_CTRL_CMD_CLRCI, &nCmdCode, &bIsIncrCmd);
            Adbms6948_Cmn_SendCommandOnly(nCmdCode, bIsIncrCmd, knChainID);
            /*Send CLRSI command*/
            Adbms6948_lGetCtrlCmdCode(ADBMS6948_CTRL_CMD_CLRSI, &nCmdCode, &bIsIncrCmd);
            Adbms6948_Cmn_SendCommandOnly(nCmdCode, bIsIncrCmd, knChainID);
            break;
        default:
            nRet = E_NOT_OK;
            break;
    }

    if (TRUE == bSendUnsnap)
    {
        nCmdCode = ADBMS6948_CMD_UNSNAP;
        Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
    }
    /*Verify command counter after sending Clear command(s)*/
    nRet = Adbms6948_Cmn_VerifyCmdCnt(TRUE, knChainID);
return nRet;
}
/*****************************************************************************/
/*!
    @brief  This function validates cell open wire selection.
    @param  [in]   eOWSel        OW selection to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidCellOWSelType
(
    Adbms6948_CellOWSelType      eOWSel
)
{
    boolean  bRet;
    switch(eOWSel)
    {
        case ADBMS6948_CELL_OW_NONE:
        case ADBMS6948_CELL_OW_EVEN:
        case ADBMS6948_CELL_OW_ODD:
        case ADBMS6948_CELL_OW_ALL:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}
/*****************************************************************************/
/*!
    @brief  This function validates channel selection for AUX ADC.

    @param  [in]   eAuxChSel	AUX channel selection to be validated.

    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidAuxChSelType
(
    Adbms6948_AuxChSelType      eAuxChSel
)
{
    boolean  bRet;
    switch(eAuxChSel)
    {
    	case ADBMS6948_AUX_CH_ALL:
    	case ADBMS6948_AUX_CH_GPIO1:
    	case ADBMS6948_AUX_CH_GPIO2:
    	case ADBMS6948_AUX_CH_GPIO3:
    	case ADBMS6948_AUX_CH_GPIO4:
    	case ADBMS6948_AUX_CH_GPIO5:
    	case ADBMS6948_AUX_CH_GPIO6:
    	case ADBMS6948_AUX_CH_GPIO7:
    	case ADBMS6948_AUX_CH_GPIO8:
    	case ADBMS6948_AUX_CH_GPIO9:
    	case ADBMS6948_AUX_CH_GPIO10:
    	case ADBMS6948_AUX_CH_VREF2:
    	case ADBMS6948_AUX_CH_LDO3V:
    	case ADBMS6948_AUX_CH_LDO5V:
    	case ADBMS6948_AUX_CH_ITMP:
    	case ADBMS6948_AUX_CH_VP:
    	case ADBMS6948_AUX_CH_VMV:
    	case ADBMS6948_AUX_CH_VR4K:
    	case ADBMS6948_AUX_CH_VREF3:
        case ADBMS6948_AUX_CH_GPIO1_TO_7_11:
        case ADBMS6948_AUX_CH_GPIO8_TO_10_VP_VMV:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}
/*****************************************************************************/
/*!
    @brief  This function validates channel selection for redundant AUX ADC.
    @param  [in]   eRedAuxChSel	Redundant AUX channel selection to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidRedAuxChSelType
(
    Adbms6948_RedAuxChSelType      eRedAuxChSel
)
{
    boolean  bRet;
    switch(eRedAuxChSel)
    {
		case ADBMS6948_RED_AUX_CH_ALL:
		case ADBMS6948_RED_AUX_CH_GPIO1:
		case ADBMS6948_RED_AUX_CH_GPIO2:
		case ADBMS6948_RED_AUX_CH_GPIO3:
		case ADBMS6948_RED_AUX_CH_GPIO4:
		case ADBMS6948_RED_AUX_CH_GPIO5:
		case ADBMS6948_RED_AUX_CH_GPIO6:
		case ADBMS6948_RED_AUX_CH_GPIO7:
		case ADBMS6948_RED_AUX_CH_GPIO8:
		case ADBMS6948_RED_AUX_CH_GPIO9:
		case ADBMS6948_RED_AUX_CH_GPIO10:
        case ADBMS6948_RED_AUX_CH_GPIO11:
        case ADBMS6948_RED_AUX_CH_GPIO1_TO_7_11:
        case ADBMS6948_RED_AUX_CH_GPIO8_TO_10:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}

/*****************************************************************************/
/*!
    @brief  This function validates cell measurement data type.
    @param  [in]   eCellMeasData	Cell measurement data type to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidCellMeasDataType
(
    Adbms6948_CellMeasDataType      eCellMeasData
)
{
    boolean  bRet;
    switch(eCellMeasData)
    {
        case ADBMS6948_CELL_MEAS_DATA:
        case ADBMS6948_CELL_MEAS_DATA_REDUNDANT:
        case ADBMS6948_CELL_MEAS_DATA_AVERAGED:
        case ADBMS6948_CELL_MEAS_DATA_FILTERED:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}
/*****************************************************************************/
/*!
    @brief  This function validates current measurement data type.

    @param[in]  eCurrentMeasData    Current measurement data type to be
                                    validated.
    @return     boolean             True : Invalid value. \n
                                    False: Valid value. \n
 */
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidCurrentMeasDataType
(
    Adbms6948_CurrentMeasDataType      eCurrentMeasData
)
{
    boolean  bRet;
    switch(eCurrentMeasData)
    {
        case ADBMS6948_CURRENT_MEAS_DATA:
        case ADBMS6948_CURRENT_MEAS_DATA_AVERAGED:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}
/*****************************************************************************/
/*!
    @brief  This function validates cell register group selection.
    @param  [in]   eCellGrpSel	Cell register group selection to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidCellGrpSelType
(
    Adbms6948_CellGrpSelType      eCellGrpSel
)
{
    boolean  bRet;
    switch(eCellGrpSel)
    {
        case ADBMS6948_CELL_GRP_SEL_C1C2C3:
        case ADBMS6948_CELL_GRP_SEL_C4C5C6:
        case ADBMS6948_CELL_GRP_SEL_C7C8C9:
        case ADBMS6948_CELL_GRP_SEL_C10C11C12:
        case ADBMS6948_CELL_GRP_SEL_C13C14C15:
        case ADBMS6948_CELL_GRP_SEL_C16:
        case ADBMS6948_CELL_GRP_SEL_ALL:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}
/*****************************************************************************/
/*!
    @brief  This function validates current selection.
    @param  [in]   eCurrentSel	Current selection to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidCurrentSelType
(
    Adbms6948_CurrentSelType      eCurrentSel
)
{
    boolean  bRet;
    switch(eCurrentSel)
    {
        case ADBMS6948_CURRENT_I1:
        case ADBMS6948_CURRENT_I2:
        case ADBMS6948_CURRENT_I1I2:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}
/*****************************************************************************/
/*!
    @brief  This function validates GPIO measurement data type.
    @param  [in]   eGpioMeasData	GPIO measurement data type to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidGpioMeasDataType
(
    Adbms6948_GpioMeasDataType      eGpioMeasData
)
{
    boolean  bRet;
    switch(eGpioMeasData)
    {
        case ADBMS6948_GPIO_MEAS_DATA:
        case ADBMS6948_GPIO_MEAS_DATA_REDUNDANT:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}
/*****************************************************************************/
/*!
    @brief  This function validates GPIO register group selection.
    @param  [in]   eGpioGrpSel	GPIO register group selection to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidGpioGrpSelType
(
    Adbms6948_GpioGrpSelType      eGpioGrpSel
)
{
    boolean  bRet;
    switch(eGpioGrpSel)
    {
        case ADBMS6948_GPIO_GRP_G1G2G3:
        case ADBMS6948_GPIO_GRP_G4G5G6:
        case ADBMS6948_GPIO_GRP_G7G8G9:
        case ADBMS6948_GPIO_GRP_G10G11:
        case ADBMS6948_GPIO_GRP_ALL:
            bRet = FALSE;
            break;

        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}
/*****************************************************************************/
/*!
    @brief  This function validates device parameter group selection.
    @param  [in]   eDevParamGrpSel	Device parameter group selection to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidDevParamGrpSelType
(
    Adbms6948_DevParamGrpSelType      eDevParamGrpSel
)
{
    boolean  bRet;
    switch(eDevParamGrpSel)
    {
        case ADBMS6948_DEV_PARAMS_P1P2:
        case ADBMS6948_DEV_PARAMS_P3P4P5:
        case ADBMS6948_DEV_PARAMS_P6P7P8:
        case ADBMS6948_DEV_PARAMS_ALL:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}
/*****************************************************************************/
/*!
    @brief  This function validates control command selection.
    @param  [in]   eCtrlCmdSel	control command selection to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidCtrlCmdSelType
(
    Adbms6948_CtrlCmdSelType      eCtrlCmdSel
)
{
    boolean  bRet;
    switch(eCtrlCmdSel)
    {
        case ADBMS6948_CTRL_CMD_SNAP:
        case ADBMS6948_CTRL_CMD_UNSNAP:
        case ADBMS6948_CTRL_CMD_MUTE:
        case ADBMS6948_CTRL_CMD_UNMUTE:
        case ADBMS6948_CTRL_CMD_RSTCC:
        case ADBMS6948_CTRL_CMD_CLRCELL:
        case ADBMS6948_CTRL_CMD_CLRFC:
        case ADBMS6948_CTRL_CMD_CLRAUX:
        case ADBMS6948_CTRL_CMD_CLRSPIN:
        case ADBMS6948_CTRL_CMD_CLRCC:
        case ADBMS6948_CTRL_CMD_CLRCI:
        case ADBMS6948_CTRL_CMD_CLRSI:
		case ADBMS6948_CTRL_CMD_ULAO:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}
/*****************************************************************************/
/*!
    @brief  This function validates ADC selection for polling.
    @param  [in]   eADCSel	ADC selection to be validated.
    @return        True : Invalid value. \n
                   False: Valid value. \n
*/
/*****************************************************************************/
static boolean  Adbms6948_lIsInvalidADCSelType
(
    Adbms6948_ADCSelType      eADCSel
)
{
    boolean  bRet;
    switch(eADCSel)
    {
        case ADBMS6948_ADC_ALL:
        case ADBMS6948_ADC_CADC:
        case ADBMS6948_ADC_SADC:
        case ADBMS6948_ADC_AUX:
        case ADBMS6948_ADC_AUX2:
            bRet = FALSE;
            break;
        default:
            bRet = TRUE;
            break;
    }
    return(bRet);
}

/*****************************************************************************/
/*!
    @brief	Local function to read the OV/UV diagnostic flags.

    @param [out]	pnCellOVUVStatus  	Place holder to report the OVUV
										status for all the connected cells
										in the chain.
										@range: NA
										@resolution: NA

	@param 	[in] 	eSnapSel	Specifies the snapshot command/s to be sent.
								@range: Enumeration
								@resolution: NA

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n
 */
/*****************************************************************************/
static Adbms6948_ReturnType  Adbms6948_lReadCellOVUVStatus
(
	uint32_t 				*pnCellOVUVStatus,
	Adbms6948_SnapSelType 	 eSnapSel,
	const uint8_t  			 knChainID
)
{
	Adbms6948_ReturnType  nRet;
	uint16_t  nCmdCode;
	uint8_t  anRdBuf[ADBMS6948_NUMOF_REGGRP_ONE][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)] = {0u};
	uint16_t anCmdList[ADBMS6948_NUMOF_REGGRP_ONE], nRdBufIdx;
	uint8_t  nBufIdx = 0u;
	uint32_t nCellOVUVStatus;
	boolean  bCfgRead = FALSE, bSendCLR = FALSE;

	if ((ADBMS6948_SEND_SNAP == eSnapSel) || (ADBMS6948_SEND_BOTH == eSnapSel))
	{
		nCmdCode = ADBMS6948_CMD_SNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
	}

	anCmdList[0u] = ADBMS6948_CMD_RDSTATD;
	nRet = Adbms6948_Cmn_ReadRegGroup(anCmdList, anRdBuf, ADBMS6948_NUMOF_REGGRP_ONE, knChainID);
	if (E_OK == nRet)
	{
		nRdBufIdx = (uint16_t)ADBMS6948_CMD_DATA_LEN;

		nCellOVUVStatus = (uint32_t)anRdBuf[0u][nRdBufIdx];
		nCellOVUVStatus |= (uint32_t)((uint32_t)anRdBuf[0u][nRdBufIdx + 1u] << 8u);
		nCellOVUVStatus |= (uint32_t)((uint32_t)anRdBuf[0u][nRdBufIdx + 2u] << 16u);
		nCellOVUVStatus |= (uint32_t)((uint32_t)anRdBuf[0u][nRdBufIdx + 3u] << 24u);
		pnCellOVUVStatus[nBufIdx++] = nCellOVUVStatus;

		if(0u != nCellOVUVStatus)
		{
			bSendCLR = TRUE;
		}

		/* Send zeros for undefined bits */
		anRdBuf[0u][nRdBufIdx + 4u] = 0x00u;
		anRdBuf[0u][nRdBufIdx + 5u] = 0x00u;

		if(TRUE == bSendCLR)
		{
			/*Clear OVUV flags*/
			anCmdList[0] = ADBMS6948_CMD_CLOVUV;
			Adbms6948_Cmn_WriteRegGroup(anCmdList, ADBMS6948_REG_GRP_LEN, anRdBuf, knChainID);
			bCfgRead = TRUE;
		}
	}

	if ((ADBMS6948_SEND_UNSNAP == eSnapSel) || (ADBMS6948_SEND_BOTH == eSnapSel))
	{
		nCmdCode = ADBMS6948_CMD_UNSNAP;
		Adbms6948_Cmn_SendCommandOnly(nCmdCode, TRUE, knChainID);
		bCfgRead = TRUE;
	}

	/* Verify the command counter if either CLOVUV command or UNSNAP was sent*/
	nRet |= Adbms6948_Cmn_VerifyCmdCnt(bCfgRead, knChainID);
return(nRet);
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

