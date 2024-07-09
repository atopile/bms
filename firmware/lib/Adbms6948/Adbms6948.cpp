
/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948.cpp
* @brief  : Adbms6948 driver source file, containing the  API
* specification and functions that are exported by the driver.
* @version: $Revision: 541 $
* @date   : $Date: 2020-10-16 11:46:03 +0530 (Fri, 16 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/
/*! \addtogroup ADI_ADBMS6948_DRIVER   ADBMS6948 Driver
* @{
*/

/*! \addtogroup ADI_ADBMS6948_DRIVER_API    ADBMS6948 Driver APIs
* @{
*/

#ifndef ADI_DISABLE_DOCUMENTATION
/*============= I N C L U D E S =============*/

#include "Adbms6948.h"

#include "Adbms6948_Config.h"
#include "Adbms6948_Measure.h"
#include "Adbms6948_Generic.h"
#include "Adbms6948_OtherMonitor.h"

/*============= D A T A =============*/


/*============= C O D E =============*/
/* Start of code section */

/* Code section start */
ADBMS6948_DRV_CODE_START



#endif /*  ADI_DISABLE_DOCUMENTATION */
/*
 * Function Definition section
*/

/*! \addtogroup ADI_CONFIGURATION	ADBMS6948 Configuration APIs
* @{
*/
/*****************************************************************************/
/************************** CONFIGURATION SERVICES ***************************/
/*****************************************************************************/

/*****************************************************************************/
 /*!
    @brief      This API performs all the initializations required for
                ADBMS6948 Driver. It also stores the address of configuration
                set to use when required during the driver operation.
				This is a wrapper function the actual configuration will be
				performed by the internal config function.

    @param [in]    pkConfig  Pointer to ADBMS6948 configuration set.
							@range: NA
							@resolution: NA

    @return        E_OK: Success  \n
                   E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_Init
(
const Adbms6948_CfgType *pkConfig
)
{
	return (Adbms6948_Int_Init(pkConfig));
}

/*****************************************************************************/
 /*!
    @brief      This API closes	ADBMS6948 Driver and sets the ADBMS6948 device
				registers to their default values. \n
				This is a wrapper function the actual configuration will be
				performed by the internal config function.


    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_DeInit
(
void
)
{
	return (Adbms6948_Int_DeInit());
}

/*****************************************************************************/
/*!
    @brief      This API enables/disables soak feature for all ADC Aux commands
                on ADBMS6948 device. \n

    @param [in]    	pEnable  	Pointer to the soak control values per device.\n
								Application to pass values based on the number of
								ADBMS6948 devices in chain.
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
Adbms6948_ReturnType  Adbms6948_CfgSoakControl
(
boolean 		*pEnable,
const uint8_t    knChainID
)
{
	return (Adbms6948_Int_CfgSoakControl(pEnable, knChainID));
}

/*****************************************************************************/
/*!
    @brief      This API configures soak time for
				auxiliary ADC commands. \n

    @param [in]    pAuxSoakTimeCfg  Pointer to AUX SOAK time configuration
									set.
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
Adbms6948_ReturnType  Adbms6948_CfgSoakTime
(
Adbms6948_AuxSoakTimeCfgType 	*pAuxSoakTimeCfg,
const uint8_t  			 		 knChainID
)
{
	return (Adbms6948_Int_CfgSoakTime(pAuxSoakTimeCfg, knChainID));
}


/*****************************************************************************/
/*!
    @brief      This API configures over voltage threshold of ADBMS6948.
				Based on the user input the output of this API will be set in
				volts. \n

    @param [in]    pOVThreshold  	Pointer to the over voltage threshold config. \n
									Application to pass values based on the number of
									ADBMS6948 devices in chain.
									Reference eq. - OV Threshold=(OVThreshold*16*150)uV+1.5V
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
Adbms6948_ReturnType  Adbms6948_CfgOVThreshold
(
Adbms6948_OVThresCfgType 	*pOVThreshold,
const uint8_t  				 knChainID
)
{
	return (Adbms6948_Int_CfgOVThreshold(pOVThreshold, knChainID));
}


/*****************************************************************************/
/*!
    @brief      This API configures under voltage threshold of ADBMS6948.
				Based on the user input the output of this API will be set in
				volts. \n

    @param 	[in]    pUVThreshold  	Pointer to the under voltage threshold config.\n
									Application to pass values based on the number of
									ADBMS6948 devices in chain.
									Reference eq.-UV Threshold=(UVThreshold*16*150)uV+
									1.5V
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
Adbms6948_ReturnType  Adbms6948_CfgUVThreshold
(
Adbms6948_UVThresCfgType 	*pUVThreshold,
const uint8_t  				 knChainID
)
{
	return (Adbms6948_Int_CfgUVThreshold(pUVThreshold, knChainID));
}

/*****************************************************************************/
/*!
    @brief      This API sets  the corner frequency for IIR filtering. \n

    @param 	[in]    pCornerFreq  	Pointer to the IIR corner frequency configuration.
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
Adbms6948_ReturnType  Adbms6948_CfgIIRFilter
(
Adbms6948_IIRCornerFreqType 	*pCornerFreq,
const uint8_t  					 knChainID
)
{
	return (Adbms6948_Int_CfgIIRFilter(pCornerFreq, knChainID));
}

/*****************************************************************************/
/*!
    @brief      This API sets the C vs S comparison voltage threshold. \n

    @param 	[in]    pCSCompThresVolt  	Pointer to the CS comparison threshold \n
										config.Application to pass values based
										on the number of ADBMS6948 devices in chain.
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
Adbms6948_ReturnType  Adbms6948_CfgCSCompThresVolt
(
Adbms6948_CSThresVoltType 	*pCSCompThresVolt,
const uint8_t  				 knChainID
)
{
	return (Adbms6948_Int_CfgCSCompThresVolt(pCSCompThresVolt, knChainID));
}

/*****************************************************************************/
/*!
    @brief      This API sets pull-down ON/OFF for the particular GPIO pin of
                ADBMS6948. \n

    @param 	[in]    pGpioPinCfg  	Pointer to the GpioPin config set by user for
									all the ADBMS6948 devices. \n
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
Adbms6948_ReturnType  Adbms6948_CfgGpioPins
(
Adbms6948_GpioPinCfgType 	*pGpioPinCfg,
const uint8_t 				 knChainID
)
{
	return (Adbms6948_Int_CfgGpioPins(pGpioPinCfg, knChainID));
}

/*****************************************************************************/
/*!
    @brief          This API configures the OC1 algorithm parameters. \n

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
Adbms6948_ReturnType Adbms6948_CfgOC1Params
(
    Adbms6948_OC1CfgType    *poOC1CfgType,
    const uint8_t           knChainID
)
{
    return (Adbms6948_Int_CfgOC1Params(poOC1CfgType, knChainID));
}

/*****************************************************************************/
/*!
    @brief          This API configures the OC2 algorithm parameters. \n

    @param[in]      poOC2CfgType    Pointer to specify the OC2 algorithm
                                    parameters.
    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA

    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_CfgOC2Params
(
    Adbms6948_OC2CfgType    *poOC2CfgType,
    const uint8_t           knChainID
)
{
    return (Adbms6948_Int_CfgOC2Params(poOC2CfgType, knChainID));
}

/*! @}*/


/*! \addtogroup ADI_OTHER_MONITOR	ADBMS6948 OtherMonitor APIs
* @{
*/
/*****************************************************************************/
/***************************** OTHER MONITOR SERVICES ************************/
/*****************************************************************************/


/*****************************************************************************/
 /*!
    @brief      This API read the device revision code from all the devices
				in the chain and reports it to user.

	@param	[out]	pRevCode	Place holder to report the device revision
								code. The user must pass a buffer of size
								equal to one.\n
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
Adbms6948_ReturnType  Adbms6948_ReadDevRevCode
(
uint8_t 		*pRevCode,
const uint8_t  	 knChainID
)
{
	return (Adbms6948_Int_ReadDevRevCode(pRevCode, knChainID));
}

/*****************************************************************************/
 /*!
    @brief      This API wakeup the ADBMS6948 device.

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_WakeUpBmsIC
(
const uint8_t 		knChainID
)
{
	return (Adbms6948_Int_WakeUpBmsIC(knChainID));
}

/*****************************************************************************/
 /*!
    @brief      This API send the soft reset command to ADBMS6948 device.

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_SendSoftReset
(
const uint8_t  	knChainID
)
{
	return (Adbms6948_Int_SendSoftReset(knChainID));
}

/*****************************************************************************/
 /*!
    @brief      This API is used to read the 48bit serial ID for the devices

	@param	[out]	pSerialidCode	Place holder to report the device serial
									id. The user must pass a buffer of size
									equal to or greater than 6*number of devices
									in the chain.The first six bytes represent the
									serial ID for the first device and so on
    								for all the devices on the chain. The data is ordered
    								from Device 1 to Device N \n

    @param	[in]   	knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_ReadSerialID
(
uint8_t 		*pSerialidCode,
const uint8_t  	 knChainID
)
{
	return (Adbms6948_Int_ReadSerialID(pSerialidCode, knChainID));
}

/*****************************************************************************/
 /*!
    @brief          This API is used to write 6 bytes of data to the Always On
					memory of BMS IC. This data does not get cleared when BMS IC
					goes into sleep state.

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
Adbms6948_ReturnType Adbms6948_AlwaysOnMemWrite
(
uint8_t 					*panData,
const uint8_t  				 knChainID
)
{
	return (Adbms6948_Int_AlwaysOnMemWrite(panData, knChainID));
}

/*****************************************************************************/
 /*!
    @brief          This API is used to read 6 bytes of data from the Always On
					Memory of BMS IC.

    @param	[out]	panData		Placeholder for storing 6 bytes of data.
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
Adbms6948_ReturnType Adbms6948_AlwaysOnMemRead
(
uint8_t 					*panData,
const uint8_t  				 knChainID
)
{
	return (Adbms6948_Int_AlwaysOnMemRead(panData, knChainID));
}

/*****************************************************************************/
 /*!
    @brief      This API gets the counts for the errors detected in the data
				communication with the ADBMS6948 device in the chain.It reports
				the following values-                               \n

				1. A value to report the total occurrences of PEC error while
				reading the data from the device                    \n
				2. A value to report the number of times, the incorrect value
				for command counter is received from the device.    \n
                                                                    \n
				NOTE: The error counts reported by this API, is since the driver
				initialization or the last invocation of the
				'Adbms6948_ClearErrorCounts()' API on the chain \n

    @param	[in]   pErrCnt		Place holder to report the error counts. User
								must pass the buffer of size equal to one.
								@range: NA
								@resolution: NA

    @param	[in]   knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_ReadErrorCounts(
Adbms6948_ErrorCounts *pErrCnt,
const uint8_t    	   knChainID
)
{
	return (Adbms6948_Int_ReadErrorCounts(pErrCnt, knChainID));
}

/*****************************************************************************/
 /*!
    @brief      This API clears the counts for the errors detected in the data
				communication with the ADBMS6948 device.  \n

    @param	[in]   knChainID	Specifies ID of the daisy chain on
								which the operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return        E_OK: 		Success \n
                   E_NOT_OK: 	Failure \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_ClearErrorCounts(
const uint8_t  knChainID
)
{
	return (Adbms6948_Int_ClearErrorCounts(knChainID));
}

/*! @}*/

/*! \addtogroup ADI_MEASUREMENT	ADBMS6948 Measurement APIs
* @{
*/
/*****************************************************************************/
/************************* MEASUREMENT SERVICES ******************************/
/*****************************************************************************/

/*****************************************************************************/
 /*!
    @brief		This function triggers the CADC conversions on the ADBMS6948
                device.

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
Adbms6948_ReturnType  Adbms6948_TrigCADC
(
    Adbms6948_TrigCADCInputs 	*poTrigCADCIp,
	const uint8_t  				 knChainID
)
{
	return (Adbms6948_Int_TrigCADC(poTrigCADCIp, knChainID));
}

/*****************************************************************************/
 /*!
    @brief		This API triggers the SADC conversions on the ADBMS6948 device.

    @param [in] bContMode	Specifies SADC conversion mode.
							\n 1-Continuous conversion mode
							\n 0-Single Shot conversion mode
							@range: FALSE,TRUE
							@resolution: NA

    @param [in] bEnSSDP		Specifies discharge permission during Single Shot
							SADC conversion.
							\n 1-Discharge Permitted ,
							\n 0-Discharge Not Permitted
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
Adbms6948_ReturnType  Adbms6948_TrigSADC
(
    boolean 					bContMode,
	boolean 					bEnSSDP,
	Adbms6948_CellOWSelType  	eOWSel,
	const uint8_t 				knChainID
)
{
	return (Adbms6948_Int_TrigSADC(bContMode, bEnSSDP, eOWSel, knChainID));
}

/*****************************************************************************/
 /*!
    @brief		This API triggers the auxiliary ADC conversions on the ADBMS6948
                device.

    @param [in] bOWSel		The OW switch selection for AuxADC conversion.
							\n 1-OW switch on
							\n 0-OW switch off
							@range: FALSE,TRUE
							@resolution: NA

    @param [in] bCurrentSel	To specify current selection for OW conversion.
							\n 1-Pull-up current during Aux Conversions (if bOWSel=1),
							\n 0-Pull-down current during Aux Conversions (if bOWSel=1)
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
Adbms6948_ReturnType  Adbms6948_TrigAuxADC
(
    boolean 					bOWSel,
	boolean  					bCurrentSel,
	Adbms6948_AuxChSelType  	eAuxChSel,
	const uint8_t  				knChainID
)
{
	return (Adbms6948_Int_TrigAuxADC(bOWSel, bCurrentSel, eAuxChSel, knChainID));
}

/*****************************************************************************/
 /*!
    @brief		This API triggers the redundant auxiliary ADC conversions on
                ADBMS6948 device.

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
Adbms6948_ReturnType  Adbms6948_TrigRedAuxADC
(
	Adbms6948_RedAuxChSelType  	eRedAuxChSel,
	const uint8_t  				knChainID
)
{
	return (Adbms6948_Int_TrigRedAuxADC(eRedAuxChSel, knChainID));
}
/*****************************************************************************/
 /*!
    @brief	This function triggers I1-ADC conversions on the ADBMS6948 device.

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
Adbms6948_ReturnType  Adbms6948_TrigI1ADC
(
	Adbms6948_TrigI1ADCInputs 	*poTrigI1ADCIp,
	const uint8_t  				 knChainID
)
{
	return Adbms6948_Int_TrigI1ADC(poTrigI1ADCIp, knChainID);
}
/*****************************************************************************/
 /*!
    @brief	This function triggers I2-ADC conversions on ADBMS6948 device.

    @param [in] poTrigI2ADCIp 	Pointer to the structure containing the I2-ADC
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
Adbms6948_ReturnType  Adbms6948_TrigI2ADC
(
	Adbms6948_TrigI2ADCInputs 	*poTrigI2ADCIp,
	const uint8_t  				 knChainID
)
{
	return (Adbms6948_Int_TrigI2ADC(poTrigI2ADCIp, knChainID));
}
/*****************************************************************************/
 /*!
    @brief		This API triggers the C-ADC and I1-ADC conversions
				simultaneously on the ADBMS6948 device.

    @param [in] poTrigIVADCIp	Pointer to the structure containing the C-ADC
								and I1-ADC trigger parameter.
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
Adbms6948_ReturnType  Adbms6948_TrigIVADC
(
	Adbms6948_TrigIVADCInputs	*poTrigIVADCIp,
	const uint8_t				knChainID
)
{
	return (Adbms6948_Int_TrigIVADC(poTrigIVADCIp, knChainID));
}
/*****************************************************************************/
 /*!
    @brief		This API reads the CADC conversion counter from the ADBMS6948
                device.

	@param [out] panConvCnt	Place holder to report the conversion counter
							from the ADBMS6948 device.
							User must pass the buffer of size equal to one.
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
Adbms6948_ReturnType  Adbms6948_ReadCADCConvCnt
(
	uint16_t 				*panConvCnt,
	Adbms6948_SnapSelType    eSnapSel,
	const uint8_t  			 knChainID
)
{
	return (Adbms6948_Int_ReadCADCConvCnt(panConvCnt, eSnapSel, knChainID));
}
/*****************************************************************************/
/*!
    @brief      This API reads the CT bit values of Voltage and Current ADCs.

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
Adbms6948_ReturnType Adbms6948_ReadVoltageCurrentCTValues
(
    uint16_t                *pnVoltageCT,
    uint16_t                *pnCurrentCT,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
)
{
    return (Adbms6948_Int_ReadVoltageCurrentCTValues(pnVoltageCT, pnCurrentCT, eSnapSel, knChainID));
}
/*****************************************************************************/
/*!
    @brief      This API reads the I1-ADC conversion count from the ADBMS6948
                device.

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
Adbms6948_ReturnType  Adbms6948_ReadI1ADCConvCnt
(
    uint16_t                *panConvCnt,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
)
{
    return (Adbms6948_Int_ReadI1ADCConvCnt(panConvCnt, eSnapSel, knChainID));
}
/*****************************************************************************/
 /*!
    @brief		This API reads all the cell voltage data from the
                ADBMS6948 device.

    @param [in] 	eCellMeasData	Specifies type of cell measurement data.
									@range: Enumeration
									@resolution: NA

    @param [in] 	eCellGrpSel		Specifies the cell group/s to be read.
									@range: Enumeration
									@resolution: NA

	@param [out] 	pnCellData		Place holder to report the cell measurement
									data. User must pass the buffer of size
									equal to or more than the number of cells
									to be read.

									The cell voltage data is ordered from first
									first cell to last cell:
									[C(1), C(2), ..., C(M)]
                                    where X=first cell of the group.    \n
									      M=Number of cells in the group
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
Adbms6948_ReturnType  Adbms6948_ReadCellVolt
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
	Adbms6948_CellGrpSelType  		eCellGrpSel,
	int16_t 					    *pnCellData,
	Adbms6948_SnapSelType 			eSnapSel,
	const uint8_t  					knChainID
)
{
    #ifdef ADI_LEGACY_GRP_SUPPORT
	    return (Adbms6948_Int_ReadCellVolt(eCellMeasData, eCellGrpSel, pnCellData, eSnapSel, knChainID));
	#else
        return (Adbms6948_Int_ReadAllCellVolt(eCellMeasData, pnCellData, eSnapSel, knChainID));
    #endif
}

/*****************************************************************************/
/*!
    @brief          This API reads the average voltages of all the cells from
					ADBMS6948 device.

    @param 	[out]	pnCellData	Place holder to report the cell measurement
								data.
								@range: NA
								@resolution: NA
	@param 	[in] 	knChainID	Specifies ID of the daisy chain on which the
	                        	operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return     	E_OK: Success  \n
                	E_NOT_OK: Failed \n

 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_ReadAllAverageCellVoltages
(
	int16_t 					   *pnCellData,
	const uint8_t  					knChainID
)
{
    /*TODO: Move trigger code from here to separate function*/
	return Adbms6948_Int_ReadAllAverageCellVoltages(pnCellData, knChainID);
}

/*****************************************************************************/
 /*!
    @brief		This API reads the specified GPIO input voltage data from
                ADBMS6948 device.

    @param [in] 	eGpioMeasData	Specifies type of GPIO measurement data.
									@range: Enumeration
									@resolution: NA

    @param [in] 	eGpioGrpSel		Specifies the GPIO group/s to be read.
									@range: Enumeration
									@resolution: NA

	@param [out] 	pGpioData		Place holder to report the GPIO measurement
									data. User must pass the buffer of size
									equal to or more than the number of GPIOs
									to be read.

									The GPIO voltage data is ordered from first
									group to the last group when all groups are
									read together.

									Example-1:
									When user requests to read all 11 GPIOs,
                                    it shall order data as below:
									[G1, G2, G3, G4, G5, G6, G7, G8, G9, G10, G11]

									Example-2:
									When user requests to read GPIO1, GPIO2 and
                                    GPIO3 it shall order data as below:
									[G1, G2, G3]

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
Adbms6948_ReturnType  Adbms6948_ReadGPIOInputVolt
(
	Adbms6948_GpioMeasDataType  	eGpioMeasData,
	Adbms6948_GpioGrpSelType  		eGpioGrpSel,
	int16_t 					   *pGpioData,
	const uint8_t  					knChainID
)
{
	return (Adbms6948_Int_ReadGPIOInputVolt(eGpioMeasData, eGpioGrpSel, pGpioData, knChainID));
}


/*****************************************************************************/
 /*!
    @brief		This API reads the specified internal device parameter from
				ADBMS6948 device.

    @param [in] 	eDevParamGrpSel	Specifies the device parameter group to be read.
									@range: Enumeration
									@resolution: NA

	@param [out] 	pDevParamData	Place holder to report the internal device
									parameter measurement data. User must pass
									the buffer of size equal to or more than
									the number of device parameters	to be read.

									The following internal devices parameters
									are available on the ADBMS6948 device:
									P1(VMV), P2(VP), P3(VREF2), P4(ITMP),
                                    P5(VREF3), P6(VD), P7(VA) and P8(VR4K)

									The Device parameter data is ordered from first
									group to the last group when all groups are
									read together.
									Example-1:
									When user requests to read all 8 Parameters,
									it shall order data as below:
									[P1(VMV), P2(VP), P3(VREF2), P4(ITMP),
                                     P5(VREF3), P6(VD), P7(VA) and P8(VR4K)]

									Example-2:
									When user requests to read P1(VMV) and P2(VP),
									it shall order data as below:
									[P1(VMV), P2(VP)]

									Example-3:
									When user requests to read P6(VD), P7(VA)
                                    and P8(VR4K) it shall order data as below:
									[P6(VD), P7(VA), P8(VR4K)]
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
Adbms6948_ReturnType  Adbms6948_ReadDevParam
(
	Adbms6948_DevParamGrpSelType  	eDevParamGrpSel,
	int16_t 					   *pDevParamData,
	const uint8_t  					knChainID
)
{
	return (Adbms6948_Int_ReadDevParam(eDevParamGrpSel, pDevParamData, knChainID));
}

/*****************************************************************************/
/*!
    @brief          This API reads all the AUX registers along with all the
                    status registers.   \n
                    22 byte GPIO data + 22 bytes Redundant GPIO data + 38 byte
                    status A-B-C-D-E(5 * 6 byte), F-G(2 * 4 byte)

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
Adbms6948_ReturnType  Adbms6948_ReadAllAuxStatusRegisters
(
    int16_t                        *panGpioData,
    int16_t                        *panRedundantGpioData,
    Adbms6948_StatusRegValsType    *poStatusRegsVal,
    const uint8_t                   knChainID
)
{
    return (Adbms6948_Int_ReadAllAuxStatusRegisters(panGpioData, panRedundantGpioData, poStatusRegsVal, knChainID));
}

/*****************************************************************************/
/*!
    @brief          This API reads the specified current data from the ADBMS6948
                    device.

    @param	[in]    eCurrentMeasData	Specifies type of current measurement
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
    @return     	E_OK: Success    \n
                	E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_ReadCurrent
(
    Adbms6948_CurrentMeasDataType   eCurrentMeasData,
    Adbms6948_CurrentSelType        eCurrentSel,
    int32_t                       	*pnCurrentData,
	Adbms6948_SnapSelType 			eSnapSel,
	const uint8_t  					knChainID
)
{
	return (Adbms6948_Int_ReadCurrent(eCurrentMeasData, eCurrentSel, pnCurrentData, eSnapSel, knChainID));
}
/*****************************************************************************/
/*!
    @brief  This API reads the voltage and current data from the C and I1-ADCs
			respectively in ADBMS6948 device.

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
    @return     	E_OK: Success    \n
                	E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_ReadCADCI1ADC
(
	int16_t				    *pnCellData,
	int32_t				    *pnCurrentData,
	Adbms6948_SnapSelType	eSnapSel,
	const uint8_t			knChainID
)
{
	return (Adbms6948_Int_ReadCADCI1ADC(pnCellData, pnCurrentData, eSnapSel, knChainID));
}
/*****************************************************************************/
/*!
	@brief  This API reads the average cell volatages and average current from
			all C-ADC channels and I1-ADC respectively.

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
    @param  [out]   pnVoltageCTValue    Placeholder to report the conversion
                                        count value of Voltage ADC.
                                        User must pass the buffer of size equal
                                        to one.
                                        @range: NA
                                        @resolution: NA
    @param  [out]   pnCurrentCTValue    Placeholder to report the conversion
                                        count value of Current ADC
                                        User must pass the buffer of size equal
                                        to one.
                                        @range: NA
                                        @resolution: NA
	@param 	[in]	knChainID		    Specifies ID of the daisy chain on which the
	                        		    operation is to be performed.
									    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
									    @resolution: NA
	@return			E_OK: Success    \n
					E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_ReadCellVoltCurrentAverage
(
	int16_t 				*pnCellData,
	int32_t					*pnCurrentData,
    boolean                 bSendUnsnapAtEnd,
	uint16_t				*pnVoltageCTValue,
	uint16_t				*pnCurrentCTValue,
	const uint8_t			knChainID
)
{
    return (Adbms6948_Int_ReadCellVoltCurrentAverage(pnCellData, pnCurrentData, bSendUnsnapAtEnd, pnVoltageCTValue, pnCurrentCTValue, knChainID));
}

/*****************************************************************************/
/*!
    @brief         	This API sends the specified control command to the
                    ADBMS6948 device.

	@param [in]  	eCtrlCmdSel	Specifies the control command to be sent.
								@range: Enumeration
								@resolution: NA

	@param [in] 	knChainID	Specifies ID of the daisy chain on which the
								operation is to be performed.
								@range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
								@resolution: NA

    @return     E_OK: Success  \n
                E_NOT_OK: Failed \n
 */
/*****************************************************************************/
Adbms6948_ReturnType  Adbms6948_SendCtrlCmd
(
Adbms6948_CtrlCmdSelType  	eCtrlCmdSel,
const uint8_t  				knChainID
)
{
	return (Adbms6948_Int_SendCtrlCmd(eCtrlCmdSel, knChainID));
}

/*****************************************************************************/
/*!
    @brief         	This API polls the ADC status for the specified ADC/s
					on ADBMS6948 device.

	@param [in]  	eADCSel		Specifies the ADC/s to be polled for the status.
								@range: Enumeration
								@resolution: NA

	@param [in]  	pbADCStatus	Place holder to report the Poll ADC status.
								0 - Indicates ADC conversion is completed.
								1 - Indicates ADC conversion is in progress.
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
Adbms6948_ReturnType  Adbms6948_PollADCStatus
(
Adbms6948_ADCSelType  	 eADCSel,
boolean 				*pbADCStatus,
const uint8_t  			 knChainID
)
{
	return (Adbms6948_Int_PollADCStatus(eADCSel, pbADCStatus, knChainID));
}

/*****************************************************************************/
/*!
    @brief          This API clears the fault flags in Status C register.

    @param  [in]    knChainID       Specifies ID of the daisy chain on
                                    which the operation is to be performed.
                                    @range: 0 to (ADBMS6948_NO_OF_DAISY_CHAIN-1)
                                    @resolution: NA

    @return        E_OK:        Success \n
                   E_NOT_OK:    Failure \n
 */
/*****************************************************************************/
Adbms6948_ReturnType Adbms6948_ClearAllFlags
(
    const uint8_t           knChainID
)
{
    return (Adbms6948_Int_ClearAllFlags(knChainID));
}

/*****************************************************************************/
/*!
    @brief          This API clears all the cell voltage registers and also
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
Adbms6948_ReturnType Adbms6948_ClearCellVoltageRegisters
(
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
)
{
    return (Adbms6948_Int_ClearCellVoltageRegisters(eSnapSel, knChainID));
}

/*****************************************************************************/
/*!
    @brief          This API clears the I1 or I2 current values in the current
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
Adbms6948_ReturnType Adbms6948_ClearCurrentRegisters
(
    Adbms6948_CurrentSelType        eCurrentSel,
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
)
{
    return (Adbms6948_Int_ClearCurrentRegisters(eCurrentSel, eSnapSel, knChainID));
}

/*****************************************************************************/
/*!
    @brief		This API gets the status of all the OV/UV diagnostic flags
				of all the cells.\n

    @param [out]	pnCellOVUVStatus  	Place holder to report the OVUV
										status for all the connected cells.
                                        The user must provide the buffer of size
                                        equal to one.

										The OVUV status will be reported in
										order starting from first cell to last
										cell.

										Each set of two bits indicates overall
										OVUV status for a cell of the device.
										The bit 0 and bit 1 of the set reports
										UV and OV status respectively.
										For example, COVUV1[1:0] reports overall
										OVUV status for cell 1, where
										COVUV1[0] reports the UV status and
										COVUV1[1] reports the OV status for
										the cell.

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
Adbms6948_ReturnType  Adbms6948_ReadCellOVUVStatus
(
uint32_t 				*pnCellOVUVStatus,
Adbms6948_SnapSelType 	 eSnapSel,
const uint8_t  			 knChainID
)
{
	return (Adbms6948_Int_ReadCellOVUVStatus(pnCellOVUVStatus, eSnapSel, knChainID));
}

/** @}*/
/** @}*/

/* End of code section */
/* Code section stop */
ADBMS6948_DRV_CODE_STOP



/*
*
* EOF
*
*/

