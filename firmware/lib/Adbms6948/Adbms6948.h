/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948.h
* @brief  : External header file
* @version: $Revision: 543 $
* @date   : $Date: 2020-10-16 15:37:07 +0530 (Fri, 16 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_ADBMS6948_DRIVER   ADBMS6948 Driver
* @{
*/

/*! \addtogroup ADI_ADBMS6948_DRIVER_DATA	ADBMS6948 Driver Data Types
* @{
*/

#ifndef ADBMS6948_H
#define ADBMS6948_H

/*============= I N C L U D E S =============*/
#include "Adbms6948_MemMap.h"
#include "Adbms6948_Cfg.h"

/*! \addtogroup ADI_ADBMS6948_CONFIG_DATA	ADBMS6948 Configuration Data Types
* @{
*/

/*=============== D E F I N E S ===============*/
/** @brief Run time error status Failed*/
#define ADBMS6948_RUNTIME_E_STATUS_FAILED			((uint8_t)0x01u)

/** @brief Macro for converting Cell voltage from Hex value to Voltage*/
#define ADBMS6948_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x)  ((float)(x) * 0.000150f + 1.5f)

/** @brief Macro for converting GPIO value from Hex value to Voltage*/
#define ADBMS6948_CONVERT_GPIO_HEX_TO_VOLT(x)         ((float)(x) * 0.000150f + 1.5f)

/** @brief Macro for converting current from Hex value to Voltage*/
#define ADBMS6948_CONVERT_CURRENT_HEX_TO_VOLT(x)      ((float)(x) * 0.000001f)

/** @brief Macro for converting average current from Hex value to Voltage*/
#define ADBMS6948_CONVERT_AVG_CURRENT_HEX_TO_VOLT(x)  ((float)(x) * 0.000000125f)

/** @brief Macro for converting Aux VREF2 from Hex value to Voltage*/
#define ADBMS6948_CONVERT_AUX_VREF2_HEX_TO_VOLTAGE(x) ((float)(x) * 0.000150f + 1.5f)

/** @brief Macro for converting Aux ITMP from Hex value to Degree Celsius*/
#define ADBMS6948_CONVERT_AUX_ITMP_HEX_TO_CELSIUS(x)  ((((float)(x) * 0.000150f + 1.5f) * 1000 / (7.5f)) - 273.0f)

/** @brief Macro for converting Aux VREF3 from Hex value to Voltage*/
#define ADBMS6948_CONVERT_AUX_VREF3_HEX_TO_VOLTAGE(x) ((float)(x) * 0.000150f + 1.5f)

/** @brief Macro for converting Aux VD from Hex value to Voltage*/
#define ADBMS6948_CONVERT_AUX_VD_HEX_TO_VOLTAGE(x)    ((float)(x) * 0.000150f + 1.5f)

/** @brief Macro for converting Aux VA from Hex value to Voltage*/
#define ADBMS6948_CONVERT_AUX_VA_HEX_TO_VOLTAGE(x)    ((float)(x) * 0.000150f + 1.5f)

/** @brief Macro for converting Aux VR4K from Hex value to Voltage*/
#define ADBMS6948_CONVERT_AUX_VR4K_HEX_TO_VOLTAGE(x) ((float)(x) * 0.000150f + 1.5f)

/** @brief Macro for converting Aux VP from Hex value to Voltage*/
#define ADBMS6948_CONVERT_VP_HEX_TO_VOLTAGE(x)        (((float)(x) * 0.000150f + 1.5f) * 25.0f)

/** @brief Macro for converting Aux VMV from Hex value to Voltage*/
#define ADBMS6948_CONVERT_VMV_HEX_TO_VOLTAGE(x)       ((float)(x) * 0.000150f + 1.5f)

/*============= D A T A T Y P E S =============*/

/**
 * Over voltage threshold value type
 */
typedef uint16_t Adbms6948_OVThresCfgType;

/**
 * Under voltage threshold value type
 */
typedef uint16_t Adbms6948_UVThresCfgType;

/**
 * Data Type to indicate the Soak Time value for Auxiliary and status conversions in ADBMS6948 device
 */
typedef uint8_t Adbms6948_OWAuxSoakTimeType;

/**
 *  GPIO pin pull-down on/off config type
 *  Each bit specifies one GPIO pin \n
 *  Bit 0 - GPIO[1] \n
 *  Bit 1 - GPIO[2] \n
 *  ............... \n
 *  Bit 9 - GPIO[10] \n
 *  Bit 10 - GPIO[11] \n
 *  Bit 11 to 15 - Do not care \n
 *  User needs to put the On/Off value corresponding to the pin
 */
typedef uint16_t Adbms6948_GpioPinCfgType;

/** Data Type to indicate the return value of the API */
typedef uint8_t Adbms6948_ReturnType;

/**
 * OC1 Warning Window type
 */
typedef uint8_t Adbms6948_OC1WarningWindowType;

/**
 * OC1 Threshold type
 */
typedef uint8_t Adbms6948_OC1ThesholdType;

/**
 * OC2 RC Time Constant type
 */
typedef uint8_t Adbms6948_OC2RCTimeConstantType;

/**
 * Function pointer for the callback of the other drivers
 */
typedef void (*Adbms6948_UpdateCCfuncPtrType)(void* oCallback,uint8_t knChainID);

/**
 * IIR corner frequency value type
 */
typedef enum
{
	ADBMS6948_FILTER_DISABLED = 0,
	ADBMS6948_FILTER_FREQ_110Hz,
	ADBMS6948_FILTER_FREQ_45Hz,
	ADBMS6948_FILTER_FREQ_21Hz,
	ADBMS6948_FILTER_FREQ_10Hz,
	ADBMS6948_FILTER_FREQ_5Hz,
	ADBMS6948_FILTER_FREQ_1_25Hz,
	ADBMS6948_FILTER_FREQ_0_625Hz
}Adbms6948_IIRCornerFreqType;

/**
 * C and S data comparison threshold value type
 */
typedef enum {
	ADBMS6948_CTH_VOLT_5_1mV = 0,
	ADBMS6948_CTH_VOLT_8_1mV,
	ADBMS6948_CTH_VOLT_10_05mV,
	ADBMS6948_CTH_VOLT_15mv,
	ADBMS6948_CTH_VOLT_22_5mv,
	ADBMS6948_CTH_VOLT_45mv,
	ADBMS6948_CTH_VOLT_75mv,
	ADBMS6948_CTH_VOLT_135mv
}Adbms6948_CSThresVoltType;

/*! \enum Adbms6948_Gpio11CfgType
    Enumeration of GPIO11 configuration type. */
typedef enum
{
    ADBMS6948_GPIO11_OC2_MODE      = 0u,
    ADBMS6948_GPIO11_OC2_1_MODE    = 1u,
    ADBMS6948_GPIO11_OC2_2_MODE    = 2u,
    ADBMS6948_GPIO11_GPIO_MODE   = 3u,
}Adbms6948_Gpio11CfgType;

/*! \struct Adbms6948_AuxSoakTimeCfgType
    structure to specify the soak time configuration for AUX ADC commands.
*/
typedef struct
{
	/*! Soak time range */
	boolean  Adbms6948_bRange;

	/*! Aux Open wire Soak time config OWA[2:0]*/
	Adbms6948_OWAuxSoakTimeType  Adbms6948_nAuxOWSoakTime;
}Adbms6948_AuxSoakTimeCfgType;

/*! \struct Adbms6948_OC1CfgType
    structure to specify the configuration for OC1 algorithm.
*/
typedef struct
{
    /*! Number of cycles in Window 1*/
    uint8_t                             Adbms6948_nCyclesW1;
    /*! Number of cycles in Window 2*/
    uint8_t                             Adbms6948_nCyclesW2;
    /*! Number of cycles in Window 3*/
    uint8_t                             Adbms6948_nCyclesW3;
    /*! Number of cycles in Window 4*/
    uint8_t                             Adbms6948_nCyclesW4;
    /*! Number of cycles in Window 5*/
    uint8_t                             Adbms6948_nCyclesW5;
    /*! Number of cycles in Window 6*/
    uint8_t                             Adbms6948_nCyclesW6;
    /*! Warning configuration for OC1*/
    uint8_t                             Adbms6948_nWarnCfg;
    /*! Gain Control for OC1 Algorithm*/
    uint8_t                             Adbms6948_nGainCtrl1;
    /*! OC1 Warning windows*/
    Adbms6948_OC1WarningWindowType      *Adbms6948_panOC1Warn;
    /*! OC1 thresholds*/
    Adbms6948_OC1ThesholdType           *Adbms6948_panOC1Thresh;
}Adbms6948_OC1CfgType;

/*! \struct Adbms6948_OC2CfgType
    structure to specify the configuration for OC1 algorithm.
*/
typedef struct
{
    /*! RC Time constants*/
    Adbms6948_OC2RCTimeConstantType     *Adbms6948_panOC2RCTimeConstant;
    /*! Max Ambient temperature proportional*/
    uint8_t                             Adbms6948_nMaxAmbientTemp;
    /*! OC threshold*/
    uint8_t                             Adbms6948_nOCThreshold;
    /*! OC threshold warning*/
    uint8_t                             Adbms6948_nOCThreshWarning;
    /*! Gain Control for OC2 Algorithm*/
    uint8_t                             Adbms6948_nGainCtrl2;
}Adbms6948_OC2CfgType;

/*! \struct Adbms6948_DaisyChainCfgType
    Configuration of a single daisy chain
*/
typedef struct
{
    /*! Flag to Enable ADC Reference Voltage */
    boolean                              Adbms6948_bEnableRefOn;

    /*! Flag to Enable/disable initialization for the chain */
    boolean                              Adbms6948_bEnableInit;

    /*! Number of devices connected in daisy chain during initialization */
    uint8_t                              Adbms6948_nNoOfDevices;

    /*! Array containing the number of cells connected to each ADBMS6948 device in the chain */
    const uint8_t                       *Adbms6948_pNoOfCellsPerDevice;

    /*! GPIO Pin control value */
    Adbms6948_GpioPinCfgType             Adbms6948_nGPIOPinCtrlValue;

    /*! Over voltage threshold per channel */
    Adbms6948_OVThresCfgType             Adbms6948_nOverVoltThreshold;

    /*! Under voltage threshold per channel */
    Adbms6948_UVThresCfgType             Adbms6948_nUnderVoltThreshold;

	/*! IIR filter corner frequency  */
	Adbms6948_IIRCornerFreqType 		 Adbms6948_eIIRCornerFreq;

	/*! C and S comparison threshold voltage value */
	Adbms6948_CSThresVoltType 			 Adbms6948_eCSThresholdVolt;

    /*! GPIO 11 configuration select */
    Adbms6948_Gpio11CfgType              Adbms6948_eGpio11CfgType;

    /*! OC1 Configuration Parameters */
    Adbms6948_OC1CfgType                *Adbms6948_poOC1CfgType;

    /*! OC2 Configuration Parameters */
    Adbms6948_OC2CfgType                *Adbms6948_poOC2CfgType;
}Adbms6948_DaisyChainCfgType;

/*!
* \struct Adbms6948_RunTimeErrorType
    Typedef for runtime error management implemented by the driver.
*/
typedef struct
{
    /*! Enables/Disables the runtime error. Enable:1, Disable:0 */
    uint8_t        nState;
    /*! ID of the runtime error (0 if disabled) */
    uint16_t       nErrID ;
}Adbms6948_RunTimeErrorType;

/*! \struct Adbms6948_RunTimeErrorConfigType
    ADBMS6948 runtime error Configuration Type
*/
typedef struct
{
    /*! PEC Error in data from Adbms6948 device */
    Adbms6948_RunTimeErrorType         Adbms6948_E_Invalid_PEC;

    /*! Command count error in data read from Adbms6948 device */
    Adbms6948_RunTimeErrorType         Adbms6948_E_CommandCount;

    /*! Platform Error in data from Adbms6948 device */
    Adbms6948_RunTimeErrorType         Adbms6948_E_Platform_Error;
}Adbms6948_RunTimeErrorCfgType;

/*! \struct Adbms6948_CfgType
    ADBMS6948 Configuration Type
*/
typedef struct
{
    /*! Array of configurations for all chains */
    const Adbms6948_DaisyChainCfgType *       Adbms6948_pDaisyChainCfg;

    /*! Pointer to Run time configuration */
    const Adbms6948_RunTimeErrorCfgType *     Adbms6948_pRunTimeErrCfg;
}Adbms6948_CfgType;
/** @}*/


/*! \addtogroup ADI_ADBMS6948_OTHERMONITOR_DATA	ADBMS6948 OtherMonitor Data Types
* @{
*/
/*! \struct Adbms6948_I2CWriteCfgType
    I2C write configuration structure containing the i2c device address and write buffer
    information for a single device.
*/
typedef struct
{
    /*! Variable containing the i2c device address to write to. */
    uint8_t   nI2CDevAddr;

    /*! Pointer to the memory location that contains the data to be written to the i2c device. */
    uint8_t *    pnWrDataBuf;

    /*! Variable containing the number of bytes to be written to the i2c device */
    uint16_t   nWrDatalen;

}Adbms6948_I2CWriteCfgType;

/*! \struct Adbms6948_I2CReadCfgType
    I2C read configuration structure containing the i2c device address, write, and read buffer
    information for a single device.
*/
typedef struct
{
    /*! Structure containing the i2c device address and write buffer information of the device to
     * read from. */
    Adbms6948_I2CWriteCfgType   oI2CWriteCfg;
    /*! Pointer to the memory location to hold the data read from the i2c device. */
    uint8_t *    pnRdDataBuf;
    /*! Variable containing the number of bytes to be read from the i2c device. */
    uint16_t   nRdDatalen;
}Adbms6948_I2CReadCfgType;
/** @}*/

/*! \addtogroup ADI_ADBMS6948_GENERIC_DATA	ADBMS6948 Generic Data Types
* @{
*/
/**
 * Over voltage eval result type
 * Each bit of the type represents one cell eval information
 * For eg.
 * Bit 0 - Cell 1 Eval result
 * Bit 1 - Cell 2 Eval result
 * ..........................
 * Bit 15 - Cell 16 Eval result
 * if Bit value is 0 means Evaluation is 'PASS'
 * if Bit value is 1 means Evaluation is 'FAIL'
 */
typedef uint16_t Adbms6948_OVEvalType;

/**
 * Under voltage eval result type
 * Each bit of the type represents one cell eval information
 * For eg.
 * Bit 0 - Cell 1 Eval result
 * Bit 1 - Cell 2 Eval result
 * ..........................
 * Bit 15 - Cell 16 Eval result
 * if Bit value is 0 means Evaluation is 'PASS'
 * if Bit value is 1 means Evaluation is 'FAIL'
 */
typedef uint16_t Adbms6948_UVEvalType;

/*! \struct Adbms6948_ErrorCounts
    structure to specify the number of times the errors are detected during the communication.
*/
typedef struct
{
    /*! Reports the number of times  the PEC error is detected while reading the data from the device */
    uint32_t  Adbms6948_nPECErrs;

    /*! Reports the number of times, incorrect command counter is received from the device */
    uint32_t  Adbms6948_nCmdCntErrs;

}Adbms6948_ErrorCounts;

/*! \struct Adbms6948_VersionInfoType
    structure to specify the version information of the driver
    Software Driver version information like: \n
									1.Module ID \n
									2.Vendor ID \n
									3.SW major version number \n
									4.SW minor version number \n
									5.SW patch version number \n
*/
typedef struct
{
    /*! Driver Module Identifier */
	uint16_t  Adbms6948_nModuleID;

    /*! Driver Vendor Identifier */
	uint16_t  Adbms6948_nVendorID;

	 /*! Driver Major version information */
	uint8_t   Adbms6948_nSWMajorVersion;

	 /*! Driver Minor version information */
	uint8_t   Adbms6948_nSWMinorVersion;

	 /*! Driver Patch version information */
	uint8_t   Adbms6948_nSWPatchVersion;
}Adbms6948_VersionInfoType;
/** @}*/

/*! \addtogroup ADI_ADBMS6948_MEASUREMENT_DATA	ADBMS6948 Measurement Data Types
* @{
*/

/*! \struct Adbms6948_StatusRegValsType
    Structure to store the status register values.
 */
typedef struct
{
    /*Status A register values*/
    int16_t nVRef2         : 16;    /*!< VREF2 voltage*/
    int16_t nITMP          : 16;    /*!< Internal Die Temperature*/
    int16_t nVRef3         : 16;    /*!< VREF3 voltage*/

    /*Status B register values*/
    int16_t nVD            : 16;    /*!< Digital Power Supply Voltage*/
    int16_t nVA            : 16;    /*!< Analog Power Supply Voltage*/
    int16_t nVR4K          : 16;    /*!< Voltage Across 4K Resistor*/

    /*Status C register values*/
    uint16_t nCSxFault      : 16;   /*!< C Vs S Fault of 16 Channels,
                                        each bit represents the C vs S Fault of
                                        1 channel, LSb denotes for Channel 1 and
                                        MSb denotes for Channel 16*/
    uint8_t  nCompI         : 1;    /*!< I1 and I2 ADCs Comparison status*/
    uint8_t  nOC1           : 1;    /*!< Over current 1 status*/
    uint8_t  nOC2           : 1;    /*!< Over current 2 status*/
    uint8_t  nOCWarn1       : 1;    /*!< Over Current Warning 1*/
    uint8_t  nOCWarn2       : 1;    /*!< Over Current Warning 2*/
    uint8_t  nCoulCntFlt    : 1;    /*!< Coulomb Count Fault*/
    uint8_t  nTimeBaseFlt   : 1;    /*!< Time Base Fault*/
    uint8_t  nCoulCntSeqErr : 1;    /*!< Coulomb Counter Sequence Error*/
    uint8_t  nSMED          : 1;    /*!< S-trim Multiple Error Detection*/
    uint8_t  nSED           : 1;    /*!< S-trim Error Detection*/
    uint8_t  nCMED          : 1;    /*!< C-trim Multiple Error Detection*/
    uint8_t  nCED           : 1;    /*!< C-trim Error Detection*/
    uint8_t  nVD_UV         : 1;    /*!< 3V Digital Rail Under Voltage flag*/
    uint8_t  nVD_OV         : 1;    /*!< 3V Digital Rail Over Voltage flag*/
    uint8_t  nVA_UV         : 1;    /*!< 3V Analog Rail Under Voltage flag*/
    uint8_t  nVA_OV         : 1;    /*!< 3V Analog Rail Over Voltage flag*/
    uint8_t  nOscChk        : 1;    /*!< Oscillator Check status*/
    uint8_t  nTModChk       : 1;    /*!< Test Mode Detection status*/
    uint8_t  nThSd          : 1;    /*!< Thermal Shut Down status*/
    uint8_t  nSleep         : 1;    /*!< Sleep Mode Detection status*/
    uint8_t  nSpiFlt        : 1;    /*!< SPI Fault status*/
    uint8_t  nCompV         : 1;    /*!< C and S ADC Comparison status*/
    uint8_t  nVDeLatent     : 1;    /*!< Supply Rail Delta Latent*/
    uint8_t  nVDe           : 1;    /*!< Supply Rail Delta*/

    /*Status D register values*/
    uint16_t nCellOVUVStatus: 16;   /*!< Cells 1 to 16 OV, UV flags.
                                        Each set of two bits indicates overall
                                        OVUV status starting from Cell 1 (LSb)
                                        to Cell 16(MSb).
                                        The bit 0 and bit 1 of the set reports
                                        UV and OV status respectively*/
    uint8_t  nOscChkCnt     : 8;    /*!< Oscillator Check Counter*/

    /*Status E register values*/
    uint16_t nOC1Val        : 16;   /*!< Over-Current Value 1*/
    uint16_t nOC2Val        : 16;   /*!< Over-Current Value 2*/
    uint16_t nGpioPinStates : 11;   /*!< GPIO pins states, from GPIO1 (bit 0) to
                                         GPIO11 (bit 11)*/
    uint8_t  nLPCMFltBStatus: 1;    /*!< LPCM FAULTB status*/
    uint8_t  nDevRevCode    : 4;    /*!< Device Revision Code*/

    /*Status F register values*/
    int16_t nVMV           : 16;    /*!< VM Measurement*/
    int16_t nVP            : 16;    /*!< VP Measurement*/

    /*Status G register values*/
    uint16_t nCTV           : 11;   /*!< Voltage Conversion Counter*/
    uint8_t  nCTSV          : 2;    /*!< Voltage Conversions Sub Counter*/
    uint16_t nCTI           : 11;   /*!< Current Conversion Counter*/
    uint8_t  nCTSI          : 2;    /*!< Current Conversions Sub Counter*/
}Adbms6948_StatusRegValsType;

/*! \enum Adbms6948_CellOWSelType
    Enumeration of the cell open wire selection type.
*/
typedef enum
{
    ADBMS6948_CELL_OW_NONE = 0u,	/*!< Open wire detection off on all channels. */
	ADBMS6948_CELL_OW_EVEN,			/*!< Open wire Selection on for even channels, off for odd channels. */
	ADBMS6948_CELL_OW_ODD,			/*!< Open wire Selection on for odd channels, off for even channels. */
	ADBMS6948_CELL_OW_ALL,			/*!< Open wire Selection on for all channels. */
	ADBMS6948_CELL_OW_INVALID		/*!< Invalid */
}Adbms6948_CellOWSelType;

/*! \enum Adbms6948_CurrentOWSelType
	Enumeration of the current open wire selection type.
*/
typedef enum
{
	ADBMS6948_CURRENT_OW_NONE = 0u,		/*!< Open wire detection off on all channels. */
	ADBMS6948_CURRENT_OW_EVEN,			/*!< Open wire Selection on for even channels, off for odd channels. */
	ADBMS6948_CURRENT_OW_ODD,			/*!< Open wire Selection on for odd channels, off for even channels. */
	ADBMS6948_CURRENT_OW_ALL,			/*!< Open wire Selection on for all channels. */
	ADBMS6948_CURRENT_OW_INVALID		/*!< Invalid */
}Adbms6948_CurrentOWSelType;

/*! \struct Adbms6948_TrigCADCInputs
    A structure containing CADC trigger inputs.
*/
typedef struct
{
    /*! To select the CADC conversion mode.
		\n 1-Continuous conversions
		\n 0-Single Shot conversion */
    boolean  Adbms6948_bContMode;

    /*! To enable/disable redundancy.
		\n 1-Enables redundancy
		\n 0-disables redundancy */
    boolean  Adbms6948_bEnRedun;

    /*! To specify Single Shot discharge permission.
		\n 1-Discharge Permitted during Single Shot CADC conversion
		\n 0-Discharge Not Permitted during Single Shot CADC conversion */
    boolean  Adbms6948_bEnSSDP;

    /*! To reset the IIR filter.
		\n 1-Reset IIR filter,
		\n 2-Do not reset IIR filter */
    boolean  Adbms6948_bRstIIRFltr;

    /*! Open wire switch selection for C-ADC. */
    Adbms6948_CellOWSelType  Adbms6948_eOWSel;
}Adbms6948_TrigCADCInputs;

/*! \struct Adbms6948_TrigI1ADCInputs
    A structure containing I1-ADC trigger inputs.
*/
typedef struct
{
    /*! To select the I1-ADC conversion mode.
		\n 1-Continuous conversions
		\n 0-Single Shot conversion */
    boolean  Adbms6948_bContMode;

    /*! To enable/disable redundancy.
		\n 1-Enables redundancy
		\n 0-disables redundancy */
    boolean  Adbms6948_bEnRedun;

    /*! Open wire switch selection for I1-ADC. */
    Adbms6948_CurrentOWSelType  Adbms6948_eOWSel;
}Adbms6948_TrigI1ADCInputs;

/*! \struct Adbms6948_TrigI2ADCInputs
    A structure containing I2-ADC trigger inputs.
*/
typedef struct
{
    /*! To select the I2-ADC conversion mode.
		\n 1-Continuous conversions
		\n 0-Single Shot conversion */
    boolean  Adbms6948_bContMode;

    /*! Open wire switch selection for I2-ADC. */
    Adbms6948_CurrentOWSelType  Adbms6948_eOWSel;
}Adbms6948_TrigI2ADCInputs;

/*! \struct Adbms6948_TrigIVADCInputs
    A structure containing I1-ADC and C-ADC trigger inputs.
*/
typedef struct
{
    /*! To select the C-ADC and I1-ADC conversion mode.
		\n 1-Continuous conversions
		\n 0-Single Shot conversion */
    boolean  Adbms6948_bContMode;

    /*! To enable/disable redundancy.
		\n 1-Enables redundancy
		\n 0-disables redundancy */
    boolean  Adbms6948_bEnRedun;

    /*! To specify Single Shot discharge permission.
		\n 1-Discharge Permitted during Single Shot conversion
		\n 0-Discharge Not Permitted during Single Shot conversion */
    boolean  Adbms6948_bEnSSDP;

    /*! To reset the IIR filter.
		\n 1-Reset IIR filter,
		\n 2-Do not reset IIR filter */
    boolean  Adbms6948_bRstIIRFltr;

    /*! Open wire switch selection for C-ADC and I1-ADC. */
    Adbms6948_CellOWSelType  Adbms6948_eOWSel;

}Adbms6948_TrigIVADCInputs;

/*! \enum Adbms6948_CellMeasDataType
    Enumeration of the cell measurement data type.
*/
typedef enum
{
    ADBMS6948_CELL_MEAS_DATA = 0u,				/*!< Cell Measurement data */
	ADBMS6948_CELL_MEAS_DATA_REDUNDANT,			/*!< Redundant cell Measurement data */
	ADBMS6948_CELL_MEAS_DATA_AVERAGED,			/*!< Averaged cell Measurement data */
	ADBMS6948_CELL_MEAS_DATA_FILTERED,			/*!< Filtered cell Measurement data */
	ADBMS6948_CELL_MEAS_DATA_INVALID			/*!< Invalid */
}Adbms6948_CellMeasDataType;

/*! \enum Adbms6948_CellGrpSelType
    Enumeration of the cell group selection type.
*/
typedef enum
{
	ADBMS6948_CELL_GRP_SEL_C1C2C3 = 0u,			/*!< Group of Cell 1, Cell 2 and Cell 3 */
	ADBMS6948_CELL_GRP_SEL_C4C5C6,				/*!< Group of Cell 4, Cell 5 and Cell 6 */
	ADBMS6948_CELL_GRP_SEL_C7C8C9,				/*!< Group of Cell 7, Cell 8 and Cell 9 */
	ADBMS6948_CELL_GRP_SEL_C10C11C12,			/*!< Group of Cell 10, Cell 11 and Cell 12 */
	ADBMS6948_CELL_GRP_SEL_C13C14C15,			/*!< Group of Cell 13, Cell 14 and Cell 15 */
	ADBMS6948_CELL_GRP_SEL_C16,					/*!< Cell 16 */
    ADBMS6948_CELL_GRP_SEL_ALL,					/*!< All cells (Cell 1 to Cell 16 */
	ADBMS6948_CELL_GRP_SEL_INVALID				/*!< Invalid */
}Adbms6948_CellGrpSelType;

/*! \enum Adbms6948_CurrentMeasDataType
    Enumeration of the Current measurement data type.
*/
typedef enum
{
    ADBMS6948_CURRENT_MEAS_DATA = 0u,		    /*!< Current Measurement data */
	ADBMS6948_CURRENT_MEAS_DATA_AVERAGED,	    /*!< Averaged Current Measurement data */
	ADBMS6948_CURRENT_MEAS_DATA_INVALID			/*!< Invalid */
}Adbms6948_CurrentMeasDataType;

/*! \enum Adbms6948_CurrentSelType
    Enumeration of the cell group selection type.
*/
typedef enum
{
	ADBMS6948_CURRENT_I1 = 0u,			    /*!< I1-ADC */
	ADBMS6948_CURRENT_I2,				    /*!< I2-ADC */
    ADBMS6948_CURRENT_I1I2,				    /*!< Both I1 and I2-ADC */
	ADBMS6948_CURRENT_INVALID				/*!< Invalid */
}Adbms6948_CurrentSelType;

/*! \enum Adbms6948_SnapSelType
    Enumeration of the Snap selection.
*/
typedef enum
{
    ADBMS6948_SEND_NONE = 0u,		/*!< No command to be sent. */
	ADBMS6948_SEND_SNAP,			/*!< Send SNAP before reading. */
	ADBMS6948_SEND_UNSNAP,			/*!< Send UNSNAP after reading. */
	ADBMS6948_SEND_BOTH,			/*!< Send SNAP before and UNSNAP after reading. */
	ADBMS6948_SEND_INVALID			/*!< Invalid */
}Adbms6948_SnapSelType;

/*! \enum Adbms6948_AuxChSelType
    Enumeration of the channel selection for AUXADC.
*/
typedef enum
{
    ADBMS6948_AUX_CH_ALL                = 0x00u,        /*!< All Channels. */
    ADBMS6948_AUX_CH_GPIO1              = 0x01u,        /*!< GPIO1 voltage. */
    ADBMS6948_AUX_CH_GPIO2              = 0x02u,        /*!< GPIO2 voltage. */
    ADBMS6948_AUX_CH_GPIO3              = 0x03u,        /*!< GPIO3 voltage. */
    ADBMS6948_AUX_CH_GPIO4              = 0x04u,        /*!< GPIO4 voltage. */
    ADBMS6948_AUX_CH_GPIO5              = 0x05u,        /*!< GPIO5 voltage. */
    ADBMS6948_AUX_CH_GPIO6              = 0x06u,        /*!< GPIO6 voltage. */
    ADBMS6948_AUX_CH_GPIO7              = 0x07u,        /*!< GPIO7 voltage. */
    ADBMS6948_AUX_CH_GPIO8              = 0x08u,        /*!< GPIO8 voltage. */
    ADBMS6948_AUX_CH_GPIO9              = 0x09u,        /*!< GPIO9 voltage. */
    ADBMS6948_AUX_CH_GPIO10             = 0x0Au,        /*!< GPIO10 voltage. */
    ADBMS6948_AUX_CH_GPIO11             = 0x0Bu,        /*!< GPIO11 voltage. */
    ADBMS6948_AUX_CH_VREF2              = 0x40u,        /*!< 2nd Reference Voltage. */
    ADBMS6948_AUX_CH_LDO3V              = 0x41u,        /*!< Internal supply voltage LDO3V. */
    ADBMS6948_AUX_CH_LDO5V              = 0x42u,        /*!< Internal supply voltage LDO5V. */
    ADBMS6948_AUX_CH_ITMP               = 0x43u,        /*!< Internal die temperature. */
    ADBMS6948_AUX_CH_VP                 = 0x44u,        /*!< V+ Measurement. */
    ADBMS6948_AUX_CH_VMV                = 0x45u,        /*!< V- Measurement. */
    ADBMS6948_AUX_CH_VR4K               = 0x46u,        /*!< Voltage across 4k register. */
    ADBMS6948_AUX_CH_VREF3              = 0x47u,        /*!< 3rd Reference Voltage. */
    ADBMS6948_AUX_CH_GPIO1_TO_7_11      = 0x0Eu,        /*!< GPIOs 1 to 7 and GPIO 11.*/
    ADBMS6948_AUX_CH_GPIO8_TO_10_VP_VMV = 0x0Fu,        /*!< GPIOs 8 to 10, V+ and V-.*/
    ADBMS6948_AUX_CH_INVALID            = 0x48u         /*!< Invalid. */
}Adbms6948_AuxChSelType;


/*! \enum Adbms6948_RedAuxChSelType
    Enumeration of the channel selection for redundant AUXADC.
*/
typedef enum
{
    ADBMS6948_RED_AUX_CH_ALL            = 0x00u,    /*!< All Channels. */
    ADBMS6948_RED_AUX_CH_GPIO1          = 0x01u,    /*!< GPIO1 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO2          = 0x02u,    /*!< GPIO2 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO3          = 0x03u,    /*!< GPIO3 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO4          = 0x04u,    /*!< GPIO4 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO5          = 0x05u,    /*!< GPIO5 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO6          = 0x06u,    /*!< GPIO6 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO7          = 0x07u,    /*!< GPIO7 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO8          = 0x08u,    /*!< GPIO8 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO9          = 0x09u,    /*!< GPIO9 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO10         = 0x0Au,    /*!< GPIO10 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO11         = 0x0Bu,    /*!< GPIO11 voltage. */
    ADBMS6948_RED_AUX_CH_GPIO1_TO_7_11  = 0x0Eu,    /*!< GPIOs 1 to 7 and GPIO 11.*/
    ADBMS6948_RED_AUX_CH_GPIO8_TO_10    = 0x0Fu,    /*!< GPIOs 8 to 10.*/
    ADBMS6948_RED_AUX_CH_INVALID        = 0x10u,    /*!< Invalid. */
}Adbms6948_RedAuxChSelType;

/*! \enum Adbms6948_GpioMeasDataType
    Enumeration of the GPIO measurement data type.
*/
typedef enum
{
    ADBMS6948_GPIO_MEAS_DATA = 0u,				/*!< GPIO Measurement data */
	ADBMS6948_GPIO_MEAS_DATA_REDUNDANT,			/*!< Redundant GPIO Measurement data */
	ADBMS6948_GPIO_MEAS_DATA_INVALID			/*!< Invalid */
}Adbms6948_GpioMeasDataType;

/*! \enum Adbms6948_GpioGrpSelType
    Enumeration of the gpio group selection type.
*/
typedef enum
{
	ADBMS6948_GPIO_GRP_G1G2G3 = 0u,			/*!< Group of GPIO 1, GPIO 2 and GPIO 3 */
	ADBMS6948_GPIO_GRP_G4G5G6,				/*!< Group of GPIO 4, GPIO 5 and GPIO 6 */
	ADBMS6948_GPIO_GRP_G7G8G9,				/*!< Group of GPIO 7, GPIO 8 and GPIO 9 */
	ADBMS6948_GPIO_GRP_G10G11,			    /*!< GPIO 10 and GPIO 11 */
    ADBMS6948_GPIO_GRP_ALL,					/*!< All GPIOs (GPIO 1 to GPIO 11 */
	ADBMS6948_GPIO_GRP_INVALID				/*!< Invalid */
}Adbms6948_GpioGrpSelType;

/*! \enum Adbms6948_DevParamGrpSelType
    Enumeration of the device parameter group selection type.
*/
typedef enum
{
	ADBMS6948_DEV_PARAMS_P1P2 = 0u,         /*!< Group of P1(VMV), P2(VP) */
	ADBMS6948_DEV_PARAMS_P3P4P5,			/*!< Group of P3(VREF2), P4(ITMP) and P5(VREF3) */
	ADBMS6948_DEV_PARAMS_P6P7P8,			/*!< Group of P6(VD), P7(VA) and P8(VR4K) */
    ADBMS6948_DEV_PARAMS_ALL,				/*!< All internal device parameters (P1(VMV), P2(VP), P3(VREF2), P4(ITMP), P5(VREF3), P6(VD), P7(VA) and P8(VR4K)) */
	ADBMS6948_DEV_PARAMS_INVALID			/*!< Invalid */
}Adbms6948_DevParamGrpSelType;

/*! \enum Adbms6948_CtrlCmdSelType
    Enumeration of the control command selection type. */
typedef enum
{
    ADBMS6948_CTRL_CMD_SNAP = 0u,			/*!< To capture the snapshot */
	ADBMS6948_CTRL_CMD_UNSNAP,				/*!< To release the snapshot */
	ADBMS6948_CTRL_CMD_MUTE,				/*!< To mute discharge */
	ADBMS6948_CTRL_CMD_UNMUTE,				/*!< To unmute discharge */
	ADBMS6948_CTRL_CMD_RSTCC,				/*!< To reset command counter */
	ADBMS6948_CTRL_CMD_CLRCELL,				/*!< To clear cell voltage register groups */
	ADBMS6948_CTRL_CMD_CLRFC,				/*!< To clear filtered cell voltage register groups */
	ADBMS6948_CTRL_CMD_CLRAUX,				/*!< To clear auxiliary register groups */
	ADBMS6948_CTRL_CMD_CLRSPIN,				/*!< To clear S-voltage register groups */
    ADBMS6948_CTRL_CMD_CLRCC,               /*!< To clear coulomb counter and time base register groups */
    ADBMS6948_CTRL_CMD_CLRCI,               /*!< To clear C-voltage and current register groups */
    ADBMS6948_CTRL_CMD_CLRSI,               /*!< To clear S-voltage and current register groups */
	ADBMS6948_CTRL_CMD_ULAO,				/*!< To unlock "Always On" flops */
	ADBMS6948_CTRL_CMD_INVALID				/*!< Invalid */
}Adbms6948_CtrlCmdSelType;

/*! \enum Adbms6948_ADCSelType
    Enumeration of the ADC/s selection type. */
typedef enum
{
    ADBMS6948_ADC_ALL = 0u,					/*!< ORed status of all ADCs */
	ADBMS6948_ADC_CADC,						/*!< CADC status */
	ADBMS6948_ADC_SADC,						/*!< SADC status */
	ADBMS6948_ADC_AUX,						/*!< Auxiliary ADC status */
	ADBMS6948_ADC_AUX2,						/*!< Redundant auxiliary ADC status */
    ADBMS6948_ADC_I1ADC,                    /*!< I1-ADC status*/
    ADBMS6948_ADC_I2ADC,                    /*!< I2-ADC status*/
	ADBMS6948_ADC_INVALID					/*!< Invalid */
}Adbms6948_ADCSelType;


/** @}*/

/*! \addtogroup ADI_ADBMS6948_LPCM_DATA	ADBMS6948 LPCM Data Types
* @{
*/

/*! \struct Adbms6948_LpcmDrvParamsType
    A structure containing the LPCM driver parameters.
*/
typedef struct
{
    /*! To specify the number of devices on the chain. */
    uint8_t		nNumOfDev;

    /*! To specify the value of wakeup control - Enable/disable wakeup before any service is executed on the chain.
     \n 1 - device wakeup required before the execution of the service.
     \n 0 - device wakeup not required. */
    boolean		bEnableWakeup;

    /*! Pointer to store the run time configuration address */
    const Adbms6948_CfgType	*pkRunTimeConfig;
}Adbms6948_LpcmDrvParamsType;

/** @}*/

/** @}*/		/* ADI_ADBMS6948_DRIVER_DATA */
/*============= E X T E R N A L S =============*/
Adbms6948_ReturnType  Adbms6948_TrigCADC
(
    Adbms6948_TrigCADCInputs 	*poTrigCADCIp,
	const uint8_t  				 knChainID
);
Adbms6948_ReturnType  Adbms6948_TrigSADC
(
    boolean 					bContMode,
	boolean 					bEnSSDP,
	Adbms6948_CellOWSelType  	eOWSel,
	const uint8_t 				knChainID
);
Adbms6948_ReturnType  Adbms6948_TrigAuxADC
(
    boolean 					bOWSel,
	boolean  					bCurrentSel,
	Adbms6948_AuxChSelType  	eAuxChSel,
	const uint8_t  				knChainID
);
Adbms6948_ReturnType  Adbms6948_TrigRedAuxADC
(
	Adbms6948_RedAuxChSelType  	eRedAuxChSel,
	const uint8_t  				knChainID
);
Adbms6948_ReturnType  Adbms6948_TrigI1ADC
(
	Adbms6948_TrigI1ADCInputs 	*poTrigI1ADCIp,
	const uint8_t  				 knChainID
);
Adbms6948_ReturnType  Adbms6948_TrigI2ADC
(
	Adbms6948_TrigI2ADCInputs 	*poTrigI2ADCIp,
	const uint8_t  				 knChainID
);
Adbms6948_ReturnType  Adbms6948_TrigIVADC
(
	Adbms6948_TrigIVADCInputs	*poTrigIVADCIp,
	const uint8_t				knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadCADCConvCnt
(
	uint16_t 				*panConvCnt,
	Adbms6948_SnapSelType    eSnapSel,
	const uint8_t  			 knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadI1ADCConvCnt
(
    uint16_t                *panConvCnt,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
);
Adbms6948_ReturnType Adbms6948_ReadVoltageCurrentCTValues
(
    uint16_t                *pnVoltageCT,
    uint16_t                *pnCurrentCT,
    Adbms6948_SnapSelType    eSnapSel,
    const uint8_t            knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadCellVolt
(
	Adbms6948_CellMeasDataType  	eCellMeasData,
	Adbms6948_CellGrpSelType  		eCellGrpSel,
	int16_t 					   *pnCellData,
	Adbms6948_SnapSelType 			eSnapSel,
	const uint8_t  					knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadAllAverageCellVoltages
(
	int16_t 					   *pnCellData,
	const uint8_t  					knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadGPIOInputVolt
(
	Adbms6948_GpioMeasDataType  	eGpioMeasData,
	Adbms6948_GpioGrpSelType  		eGpioGrpSel,
	int16_t 					   *pGpioData,
	const uint8_t  					knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadDevParam
(
	Adbms6948_DevParamGrpSelType  	eDevParamGrpSel,
	int16_t 					   *pDevParamData,
	const uint8_t  					knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadAllAuxStatusRegisters
(
    int16_t                        *panGpioData,
    int16_t                        *panRedundantGpioData,
    Adbms6948_StatusRegValsType    *poStatusRegsVal,
    const uint8_t                   knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadCurrent
(
    Adbms6948_CurrentMeasDataType   eCurrentMeasData,
    Adbms6948_CurrentSelType        eCurrentSel,
    int32_t                       	*pnCurrentData,
	Adbms6948_SnapSelType 			eSnapSel,
	const uint8_t  					knChainID
);
Adbms6948_ReturnType Adbms6948_ReadCADCI1ADC
(
	int16_t				    *pnCellData,
	int32_t				    *pnCurrentData,
	Adbms6948_SnapSelType	eSnapSel,
	const uint8_t			knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadCellVoltCurrentAverage
(
	int16_t 				*pnCellData,
	int32_t					*pnCurrentData,
    boolean                 bSendUnsnapAtEnd,
	uint16_t				*pnVoltageCTValue,
	uint16_t				*pnCurrentCTValue,
	const uint8_t			knChainID
);
Adbms6948_ReturnType  Adbms6948_SendCtrlCmd
(
Adbms6948_CtrlCmdSelType  	eCtrlCmdSel,
const uint8_t  				knChainID
);
Adbms6948_ReturnType  Adbms6948_PollADCStatus
(
Adbms6948_ADCSelType  	 eADCSel,
boolean 				*pbADCStatus,
const uint8_t  			 knChainID
);
Adbms6948_ReturnType  Adbms6948_CellOWDiag
(
    uint16_t 			*pnCellData,
    uint16_t 			*pnCellOWData,
	boolean				 bIsCADCUsed,
    uint8_t 			*pbCellOWStatus,
    const uint8_t 		 knChainID
);
Adbms6948_ReturnType  Adbms6948_CurrentOWDiag
(
	uint32_t 			*pnCurrentData,
	uint32_t 			*pnCurrentOWData,
	boolean				 bIsI1ADCUsed,
	uint8_t 			*pbCurrentOWStatus,
	const uint8_t 		 knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadCellOVUVStatus
(
uint32_t 				*pnCellOVUVStatus,
Adbms6948_SnapSelType 	 eSnapSel,
const uint8_t  			 knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadCSFltStatus
(
uint16_t 				*pnCSFltStatus,
Adbms6948_SnapSelType 	 eSnapSel,
const uint8_t  			 knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadOscChkCnt
(
uint8_t 				*pnOscChkCnt,
Adbms6948_SnapSelType 	 eSnapSel,
const uint8_t  			 knChainID
);
Adbms6948_ReturnType  Adbms6948_EnableLPFDet
(
uint8_t 		*pnLPFlts,
const uint8_t  	 knChainID
);
Adbms6948_ReturnType  Adbms6948_ReadDiagFlagStatus
(
uint32_t 				*pnDiagFlagStatus,
uint8_t                 *pnStatusE,
Adbms6948_SnapSelType 	 eSnapSel,
boolean 				 bEnSPIFault,
const uint8_t  			 knChainID
);

Adbms6948_ReturnType Adbms6948_PECAlgoDiag
(
uint8_t			*pnDiagPECResult,
const uint8_t		knChainID
);

Adbms6948_ReturnType  Adbms6948_Init
(
const Adbms6948_CfgType *pkConfig
);

Adbms6948_ReturnType  Adbms6948_DeInit
(
void
);

Adbms6948_ReturnType  Adbms6948_CfgSoakControl
(
boolean 		*pEnable,
const uint8_t    knChainID
);

Adbms6948_ReturnType  Adbms6948_CfgSoakTime
(
Adbms6948_AuxSoakTimeCfgType 	*pAuxSoakTimeCfg,
const uint8_t  			 		 knChainID
);

Adbms6948_ReturnType  Adbms6948_CfgOVThreshold
(
Adbms6948_OVThresCfgType 	*pOVThreshold,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType  Adbms6948_CfgUVThreshold
(
Adbms6948_UVThresCfgType 	*pUVThreshold,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType  Adbms6948_CfgIIRFilter
(
Adbms6948_IIRCornerFreqType 	*pCornerFreq,
const uint8_t  					 knChainID
);

Adbms6948_ReturnType  Adbms6948_CfgCSCompThresVolt
(
Adbms6948_CSThresVoltType 	*pCSCompThresVolt,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType  Adbms6948_CfgGpioPins
(
Adbms6948_GpioPinCfgType 	*pGpioPinCfg,
const uint8_t 				 knChainID
);

Adbms6948_ReturnType Adbms6948_CfgOC1Params
(
    Adbms6948_OC1CfgType    *poOC1CfgType,
    const uint8_t           knChainID
);

Adbms6948_ReturnType  Adbms6948_CfgOC2Params
(
    Adbms6948_OC2CfgType    *poOC2CfgType,
    const uint8_t           knChainID
);


Adbms6948_ReturnType  Adbms6948_ReadDevRevCode
(
uint8_t 		*pRevCode,
const uint8_t  	 knChainID
);

Adbms6948_ReturnType  Adbms6948_WakeUpBmsIC
(
const uint8_t 		knChainID
);

Adbms6948_ReturnType  Adbms6948_SendSoftReset
(
const uint8_t  		knChainID
);

Adbms6948_ReturnType  Adbms6948_ReadSerialID
(
uint8_t 		*pSerialidCode,
const uint8_t  	 knChainID
);

Adbms6948_ReturnType  Adbms6948_I2CWrite
(
Adbms6948_I2CWriteCfgType 	*pWriteCommCfg,
uint8_t 					*pnWrStatus,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType  Adbms6948_I2CRead
(
Adbms6948_I2CReadCfgType 	*pReadCommCfg,
uint8_t 					*pnRdStatus,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType Adbms6948_AlwaysOnMemWrite
(
uint8_t 					*panData,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType Adbms6948_AlwaysOnMemRead
(
uint8_t 					*panData,
const uint8_t  				 knChainID
);

Adbms6948_ReturnType  Adbms6948_ReadDriverVersion
(
Adbms6948_VersionInfoType *pDriverVersion
);

Adbms6948_ReturnType  Adbms6948_UpdateDeviceCount
(
uint8_t 		nDevCount,
const uint8_t 	knChainID
);

Adbms6948_ReturnType  Adbms6948_OverVoltEval
(
uint16_t 					*pCellData,
Adbms6948_OVThresCfgType 	*pOVThreshold,
Adbms6948_OVEvalType		*pEvalResult,
const uint8_t 				 knChainID
);

Adbms6948_ReturnType  Adbms6948_UnderVoltEval
(
uint16_t 					*pCellData,
Adbms6948_UVThresCfgType 	*pUVThreshold,
Adbms6948_UVEvalType 		*pEvalResult,
const uint8_t 				 knChainID
);

void  Adbms6948_CmdCntCallbck
(
void 				*pCallback,
const uint8_t 		 knChainID
);

Adbms6948_ReturnType  Adbms6948_ReadErrorCounts(
Adbms6948_ErrorCounts *pErrCnt,
const uint8_t    	   knChainID
);

Adbms6948_ReturnType  Adbms6948_ClearErrorCounts(
const uint8_t knChainID
);

Adbms6948_ReturnType Adbms6948_EnableCoulombCounter
(
    boolean         bContMode,
    const uint8_t	knChainID
);

Adbms6948_ReturnType Adbms6948_CfgCoulombCounter
(
    uint16_t        *pnNumConversions,
    const uint8_t	knChainID
);

Adbms6948_ReturnType Adbms6948_ReadCoulombCountAndTimeBase
(
    uint32_t                *pnCoulombCount,
    uint16_t                *pnTimeBase,
	Adbms6948_SnapSelType   eSnapSel,
    const uint8_t	        knChainID
);

Adbms6948_ReturnType Adbms6948_ClearCoulombCountAndTimeBase
(
    const uint8_t   knChainID
);

Adbms6948_ReturnType Adbms6948_ClearAllFlags
(
    const uint8_t                   knChainID
);

Adbms6948_ReturnType Adbms6948_ClearCellVoltageRegisters
(
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
);

Adbms6948_ReturnType Adbms6948_ClearCurrentRegisters
(
    Adbms6948_CurrentSelType        eCurrentSel,
    Adbms6948_SnapSelType           eSnapSel,
    const uint8_t                   knChainID
);

/****Newly added functions****/
/*************************Cell Voltage Current functions***********************/
/*If we have Avg type, then need to poll CT bit*/
/*If we have continuous, then don't need to trigger ADC inside this function*/
/*If we have single shot, then we need to trigger ADC inside this function*/

// OWSEL parameter to select Odd vs Even OW
Adbms6948_ReturnType Adbms6948_ReadCellVoltCurrentOWStatus
(

);


Adbms6948_ReturnType Adbms6948_StoreCalibrationParams
(
);

Adbms6948_ReturnType Adbms6948_SPIWrite
(
);

// In the case of a SPI master, GPIO3, GPIO4 and GPIO5 become the CSBM, SDIOM and SCKM ports of the
// SPI interface respectively. The SPI master on ADBMS6830 supports SPI mode 3 (CHPA = 1, CPOL = 1).
Adbms6948_ReturnType Adbms6948_SPIWriteRead
(
);

/*TODO: Might need to add functions for Polling and Triggering ADCs*/

/*! Declaration of Spi Post Build Configuration */
extern const Adbms6948_CfgType Adbms6948ConfigSet_0_PB;

#endif /* ADBMS6948_H */

/** @} */

/*
*
* EOF
*
*/

