/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Measure.h
* @brief  : Contains measurement related declarations.
* @version: $Revision: 517 $
* @date   : $Date: 2020-10-13 11:33:31 +0530 (Tue, 13 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_ADBMS6948_DRIVER_INTERNAL_DATA	ADBMS6948 Driver Internal Data Types
* @{
*/

#ifndef ADBMS6948_MEASURE_H
#define ADBMS6948_MEASURE_H

/*============= I N C L U D E S =============*/
#include "Adbms6948.h"
#include "Adbms6948_Common.h"
/*============================== D E F I N E S ======================================*/
/*These service IDs are used to report dev error*/
/** Trigger CADC service ID*/
#define ADBMS6948_TRIGCADC_ID				((uint8_t)0x19u)

/** Trigger SADC service ID*/
#define ADBMS6948_TRIGSADC_ID				((uint8_t)0x1Au)

/** Trigger AUXADC service ID*/
#define ADBMS6948_TRIGAUXADC_ID				((uint8_t)0x1Bu)

/** Trigger redundant AUXADC service ID*/
#define ADBMS6948_TRIGREDAUXADC_ID			((uint8_t)0x1Cu)

/** Read CADC conversion service ID*/
#define ADBMS6948_READCADCCONVCNT_ID		((uint8_t)0x1Du)

/** Read cell voltage service ID*/
#define ADBMS6948_READCELLVOLT_ID			((uint8_t)0x1Eu)

/** Read GPIO input voltage service ID*/
#define ADBMS6948_READGPIOINPUTVOLT_ID		((uint8_t)0x1Fu)

/** Read device parameter  service ID*/
#define ADBMS6948_READDEVPARAM_ID			((uint8_t)0x20u)

/** Send control command service ID*/
#define ADBMS6948_SENDCTRLCMD_ID			((uint8_t)0x21u)

/** Poll ADC status service ID*/
#define ADBMS6948_POLLADCSTATUS_ID			((uint8_t)0x22u)

/** Trigger I1-ADC service ID*/
#define ADBMS6948_TRIGI1ADC_ID              ((uint8_t)0x40u)

/** Trigger I2-ADC service ID*/
#define ADBMS6948_TRIGI2ADC_ID              ((uint8_t)0x41u)

/** Trigger current and voltage service ID*/
#define ADBMS6948_TRIGI1CADC_ID             ((uint8_t)0x42u)

/** Read current service ID*/
#define ADBMS6948_READCURRENT_ID            ((uint8_t)0x43u)

/** Read cell voltage and current service ID*/
#define ADBMS6948_READCELLVOLTAGECURRENT_ID ((uint8_t)0x44u)

/** Enable coulomb counter service ID*/
#define ADBMS6948_ENABLECOULOMBCOUNTER_ID   ((uint8_t)0x45u)

/** Read coulomb counter service ID*/
#define ADBMS6948_READCOULOMBCOUNTER_ID     ((uint8_t)0x46u)

/** Read I1-ADC Conversion count service ID*/
#define ADBMS6948_READI1ADCCONVCNT_ID       ((uint8_t)0x47u)

/** Clear Cell voltage registers service ID*/
#define ADBMS6948_CLRCELL_ID                ((uint8_t)0x48u)

/** Clear Current register service ID*/
#define ADBMS6948_CLRCURRENTREGS_ID         ((uint8_t)0x49u)

/* Read Voltage and Current CT values service ID*/
#define ADBMS6948_READIVADCCONVCNT_ID       ((uint8_t)0x4Au)

/** Clear Fault Flags service ID*/
#define ADBMS6948_CLRFLAG_ID                ((uint8_t)0x4Bu)

/***************************** Other Defines *********************************/


/** Number of Reg group/s to be read for CADC conversion counter */
#define ADBMS6948_NUMOF_REGGRP_CONVCNT			((uint8_t)0x01u)

/** Number of cell data types */
#define ADBMS6948_NUMOF_CELLDATA_TYPES			((uint8_t)0x04u)

/** Maximum Number of cell data register groups */
#define ADBMS6948_MAX_CELLDATA_REGGRPS			((uint8_t)0x06u)

/** Maximum Number of current data register groups */
#define ADBMS6948_MAX_CURRENT_REGGRPS			((uint8_t)0x01u)

/** Number of GPIO data types */
#define ADBMS6948_NUMOF_GPIODATA_TYPES			((uint8_t)0x02u)

/** Maximum Number of GPIO data register groups */
#define ADBMS6948_MAX_GPIODATA_REGGRPS			((uint8_t)0x04u)

/** Last cell register group index */
#define ADBMS6948_LAST_CELL_REGGRP				((uint8_t)0x05u)

/** Last GPIO register group index */
#define ADBMS6948_LAST_GPIO_REGGRP				((uint8_t)0x03u)

/** Maximum Device parameter register groups */
#define ADBMS6948_MAX_DEV_PARAM_REGGRPS			((uint8_t)0x03u)

/** Last Device parameter register group index */
#define ADBMS6948_FIRST_DEVPARAM_REGGRP			((uint8_t)0x00u)

/** Number of Invalid bits per device for poll command */
#define ADBMS6948_INVBITS_PER_DEV_POLL			((uint8_t)0x02u)

/** Types of "Read All" commands based on number of data bytes they read */
#define ADBMS6948_READALL_COMMAND_TYPES 		((uint8_t)0x05u)

/** Number of bytes (including 2 PEC bytes) for commands reading all cell voltages */
#define ADBMS6948_RDALL_CELLVOLTAGES_BYTES		((uint8_t)34u)

/** Number of bytes (including 2 PEC bytes) for commands reading all cell voltages and I1, I2-ADCs current values */
#define ADBMS6948_RDALL_VOLT_CURRENT_BYTES		((uint8_t)40u)

/** Number of bytes (including 2 PEC bytes) for commands reading all C and S-ADC cell voltages */
#define ADBMS6948_RDALL_CSVOLTAGES_BYTES		((uint8_t)66u)

/** Number of bytes (including 2 PEC bytes) for commands reading all C and S-ADC cell voltages and I1, I2-ADCs current values */
#define ADBMS6948_RDALL_CSVOLTAGE_CURRENT_BYTES ((uint8_t)72u)

/** Number of bytes (including 2 PEC bytes) for commands reading all Aux voltages and status bytes */
#define ADBMS6948_RDALL_AUX_STATUS_BYTES		((uint8_t)84u)

/** Number of bytes of GPIO data in RDASALL command result */
#define ADBMS6948_RDASALL_GPIO_BYTES            ((uint8_t)22u)
/** Number of bytes of Redundant GPIO data in RDASALL command result */
#define ADBMS6948_RDASALL_RD_GPIO_BYTES         ((uint8_t)22u)
/** Number of bytes of Status register A in RDASALL command result */
#define ADBMS6948_RDASALL_STATUSA_BYTES         ((uint8_t)6u)
/** Number of bytes of Status register B in RDASALL command result */
#define ADBMS6948_RDASALL_STATUSB_BYTES         ((uint8_t)6u)
/** Number of bytes of Status register C in RDASALL command result */
#define ADBMS6948_RDASALL_STATUSC_BYTES         ((uint8_t)6u)
/** Number of bytes of Status register D in RDASALL command result */
#define ADBMS6948_RDASALL_STATUSD_BYTES         ((uint8_t)6u)
/** Number of bytes of Status register E in RDASALL command result */
#define ADBMS6948_RDASALL_STATUSE_BYTES         ((uint8_t)6u)
/** Number of bytes of Status register F in RDASALL command result */
#define ADBMS6948_RDASALL_STATUSF_BYTES         ((uint8_t)4u)
/** Number of bytes of Status register G in RDASALL command result */
#define ADBMS6948_RDASALL_STATUSG_BYTES         ((uint8_t)4u)

/*======= P U B L I C P R O T O T Y P E S ========*/
Adbms6948_ReturnType  Adbms6948_Int_TrigCADC
(
    Adbms6948_TrigCADCInputs 	*poTrigCADCIp,
	const uint8_t  				 knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_TrigSADC
(
    boolean 						bContMode,
	boolean 						bEnSSDP,
	Adbms6948_CellOWSelType  	    eOWSel,
	const uint8_t 					knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_TrigAuxADC
(
    boolean 					bOWSel,
	boolean  					bCurrentSel,
	Adbms6948_AuxChSelType  	eAuxChSel,
	const uint8_t  				knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_TrigRedAuxADC
(
	Adbms6948_RedAuxChSelType  	eRedAuxChSel,
	const uint8_t  				knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_TrigI1ADC
(
    Adbms6948_TrigI1ADCInputs 	*poTrigI1ADCIp,
	const uint8_t  				 knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_TrigI2ADC
(
    Adbms6948_TrigI2ADCInputs 	*poTrigI2ADCIp,
	const uint8_t  				 knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_TrigIVADC
(
	Adbms6948_TrigIVADCInputs  *poTrigIVADCIp,
	const uint8_t  				knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadCADCConvCnt
(
	uint16_t 				*panConvCnt,
	Adbms6948_SnapSelType 	 eSnapSel,
	const uint8_t  		     knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadI1ADCConvCnt
(
    uint16_t                *panConvCnt,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
);
Adbms6948_ReturnType Adbms6948_Int_ReadVoltageCurrentCTValues
(
    uint16_t                *pnVoltageCT,
    uint16_t                *pnCurrentCT,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadCellVolt
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
	Adbms6948_CellGrpSelType  		eCellGrpSel,
	int16_t 					   *pnCellData,
	Adbms6948_SnapSelType 			eSnapSel,
	const uint8_t  					knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadGPIOInputVolt
(
	Adbms6948_GpioMeasDataType  	eGpioMeasData,
	Adbms6948_GpioGrpSelType  		eGpioGrpSel,
	int16_t 					   *pnGpioData,
	const uint8_t  					knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadDevParam
(
	Adbms6948_DevParamGrpSelType  	 eDevParamGrpSel,
	int16_t 						*pnDevParamData,
	const uint8_t  					 knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadAllAuxStatusRegisters
(
    int16_t                         *panGpioData,
    int16_t                         *panRedundantGpioData,
    Adbms6948_StatusRegValsType     *poStatusRegsVal,
    const uint8_t                    knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadCurrent
(
    Adbms6948_CurrentMeasDataType   eCurrentMeasData,
    Adbms6948_CurrentSelType        eCurrentSel,
    int32_t                        *pnCurrentData,
	Adbms6948_SnapSelType 			eSnapSel,
	const uint8_t  					knChainID
);
Adbms6948_ReturnType Adbms6948_Int_ReadCADCI1ADC
(
    int16_t                         *pnCellData,
    int32_t                         *pnCurrentData,
	Adbms6948_SnapSelType	eSnapSel,
	const uint8_t			knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadCellVoltCurrentAverage
(
    int16_t                         *pnCellData,
    int32_t                         *pnCurrentData,
    boolean                         bSendUnsnapAtEnd,
	uint16_t						*pnVoltageCTValue,
	uint16_t						*pnCurrentCTValue,
    const uint8_t                   knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_SendCtrlCmd
(
    Adbms6948_CtrlCmdSelType  	eCtrlCmdSel,
    const uint8_t  				knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_PollADCStatus
(
    Adbms6948_ADCSelType 	 eADCSel,
    boolean 				*pbADCStatus,
    const uint8_t  			 knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadAllCellVolt
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
    int16_t                         *pnCellData,
	Adbms6948_SnapSelType 			eSnapSel,
    const uint8_t                   knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadAllAverageCellVoltages
(
	int16_t 					   *pnCellData,
	const uint8_t  					knChainID
);
Adbms6948_ReturnType Adbms6948_Int_EnableCoulombCounter
(
    boolean                         bContMode,
    const uint8_t                   knChainID
);
Adbms6948_ReturnType Adbms6948_Int_ReadCoulombCountAndTimeBase
(
    uint32_t                        *pnCoulombCount,
    uint16_t                        *pnTimeBase,
	Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
);
Adbms6948_ReturnType Adbms6948_Int_ClearAllFlags
(
    const uint8_t                   knChainID
);
Adbms6948_ReturnType  Adbms6948_Int_ReadCellOVUVStatus
(
uint32_t 				*pnCellOVUVStatus,
Adbms6948_SnapSelType 	 eSnapSel,
const uint8_t  			 knChainID
);
Adbms6948_ReturnType Adbms6948_Int_ClearCellVoltageRegisters
(
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
);
Adbms6948_ReturnType Adbms6948_Int_ClearCurrentRegisters
(
    Adbms6948_CurrentSelType        eCurrentSel,
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
);

#endif /* ADBMS6948_MEASURE_H */

/** @}*/ /* ADI_ADBMS6948_DRIVER_INTERNAL_DATA */

/*
*
* EOF
*
*/

