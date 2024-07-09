/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_ExecCmd.h
* @brief  : Adbms6948 driver low-level header file, containing the Device
			command execution functions.
* @version: $Revision: 532 $
* @date   : $Date: 2020-10-14 15:50:36 +0530 (Wed, 14 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup Adbms6948_Exec_Cmd
* @{
*/

#ifndef ADBMS6948_EXEC_CMD_H
#define ADBMS6948_EXEC_CMD_H


/*============= I N C L U D E S =============*/
#include "Adbms6948_Types.h"

/*============== D E F I N E S ===============*/
/** This macro defines maximum possible value of the command counter */
#define ADBMS6948_MAX_COMMAND_COUNT_VAL    (uint8_t)0x3Fu

/** Maximum number of register groups/frame that can be read together */
#define ADBMS6948_CMD_MAX_TX_FRAMES	((uint8_t)0x01u)

/** Maximum number of register groups/frame that can be read together */
#define ADBMS6948_CMD_MAX_RX_FRAMES	((uint8_t)0x06u)

/******************* MEASURE and DIAGNOSTIC COMMAND CODES *********************/

/** Command code to trigger CADC conversion */
#define ADBMS6948_CMD_ADCV			((uint16_t)0x0260u)
/** Command code to trigger SADC conversion  */
#define ADBMS6948_CMD_ADSV			((uint16_t)0x0168u)
/** Command code to trigger Current ADC conversion  */
#define ADBMS6948_CMD_ADI1			((uint16_t)0x0200u)
/** Command code to trigger S-Current ADC conversion  */
#define ADBMS6948_CMD_ADI2			((uint16_t)0x0108u)
/** Command code to trigger CADC and I1-ADC conversion  */
#define ADBMS6948_CMD_ADCIV			((uint16_t)0x240u)
/** Command code to trigger AUXADC conversion  */
#define ADBMS6948_CMD_ADAX			((uint16_t)0x0410u)
/** Command code to trigger redundant AUXADC conversion  */
#define ADBMS6948_CMD_ADAX2			((uint16_t)0x0400u)

/** Command code to read cell voltage register group A */
#define ADBMS6948_CMD_RDCVA			((uint16_t)0x0004u)
/** Command code to read cell voltage register group B */
#define ADBMS6948_CMD_RDCVB			((uint16_t)0x0006u)
/** Command code to read cell voltage register group C */
#define ADBMS6948_CMD_RDCVC			((uint16_t)0x0008u)
/** Command code to read cell voltage register group D */
#define ADBMS6948_CMD_RDCVD			((uint16_t)0x000Au)
/** Command code to read cell voltage register group E */
#define ADBMS6948_CMD_RDCVE			((uint16_t)0x0009u)
/** Command code to read cell voltage register group F */
#define ADBMS6948_CMD_RDCVF			((uint16_t)0x000Bu)
/** Command code to read all cell voltage register groups  */
#define ADBMS6948_CMD_RDCVALL		((uint16_t)0x000Cu)

/** Command code to read averaged cell voltage register group A */
#define ADBMS6948_CMD_RDACA			((uint16_t)0x0044u)
/** Command code to read averaged cell voltage register group B */
#define ADBMS6948_CMD_RDACB			((uint16_t)0x0046u)
/** Command code to read averaged cell voltage register group C */
#define ADBMS6948_CMD_RDACC			((uint16_t)0x0048u)
/** Command code to read averaged cell voltage register group D */
#define ADBMS6948_CMD_RDACD			((uint16_t)0x004Au)
/** Command code to read averaged cell voltage register group F */
#define ADBMS6948_CMD_RDACE			((uint16_t)0x0049u)
/** Command code to read averaged cell voltage register group E */
#define ADBMS6948_CMD_RDACF			((uint16_t)0x004Bu)
/** Command code to read all averaged cell voltage register groups */
#define ADBMS6948_CMD_RDACALL		((uint16_t)0x004Cu)

/** Command code to read S voltage register group A */
#define ADBMS6948_CMD_RDSVA			((uint16_t)0x0003u)
/** Command code to read S voltage register group B */
#define ADBMS6948_CMD_RDSVB			((uint16_t)0x0005u)
/** Command code to read S voltage register group C */
#define ADBMS6948_CMD_RDSVC			((uint16_t)0x0007u)
/** Command code to read S voltage register group D */
#define ADBMS6948_CMD_RDSVD			((uint16_t)0x000Du)
/** Command code to read S voltage register group E */
#define ADBMS6948_CMD_RDSVE			((uint16_t)0x000Eu)
/** Command code to read S voltage register group F */
#define ADBMS6948_CMD_RDSVF			((uint16_t)0x000Fu)
/** Command code to read all S voltage register groups */
#define ADBMS6948_CMD_RDSVALL		((uint16_t)0x0010u)

/** Command code to read all C and S voltage register groups */
#define ADBMS6948_CMD_RDCSVALL		((uint16_t)0x0011u)
/** Command code to read all averaged C and S voltage register groups */
#define ADBMS6948_CMD_RDACSALL		((uint16_t)0x0051u)
/** Command code to read all C voltages and current register groups */
#define ADBMS6948_CMD_RDCIV			((uint16_t)0x0090u)
/** Command code to read all average C voltages and average current register groups */
#define ADBMS6948_CMD_RDACIV		((uint16_t)0x00D0u)
/** Command code to read all C and S Cell voltage and current register groups */
#define ADBMS6948_CMD_RDCSIVALL		((uint16_t)0x0094u)
/** Command code to read all averaged Cell voltage and average current register groups */
#define ADBMS6948_CMD_RDACSIVALL	((uint16_t)0x00D4u)

/** Command code to read filtered cell voltage register group A */
#define ADBMS6948_CMD_RDFCA			((uint16_t)0x0012u)
/** Command code to read filtered cell voltage register group B */
#define ADBMS6948_CMD_RDFCB			((uint16_t)0x00013u)
/** Command code to read filtered cell voltage register group C */
#define ADBMS6948_CMD_RDFCC			((uint16_t)0x0014u)
/** Command code to read filtered cell voltage register group D */
#define ADBMS6948_CMD_RDFCD			((uint16_t)0x0015u)
/** Command code to read filtered cell voltage register group E */
#define ADBMS6948_CMD_RDFCE			((uint16_t)0x0016u)
/** Command code to read filtered cell voltage register group F */
#define ADBMS6948_CMD_RDFCF			((uint16_t)0x0017u)
/** Command code to read all filtered cell voltage register groups */
#define ADBMS6948_CMD_RDFCALL		((uint16_t)0x0018u)

/** Command code to read auxiliary register group A */
#define ADBMS6948_CMD_RDAUXA		((uint16_t)0x0019u)
/** Command code to read auxiliary register group B */
#define ADBMS6948_CMD_RDAUXB		((uint16_t)0x001Au)
/** Command code to read auxiliary register group C */
#define ADBMS6948_CMD_RDAUXC		((uint16_t)0x001Bu)
/** Command code to read auxiliary register group D */
#define ADBMS6948_CMD_RDAUXD		((uint16_t)0x001Fu)

/** Command code to read redundant auxiliary register group A */
#define ADBMS6948_CMD_RDRAXA		((uint16_t)0x001Cu)
/** Command code to read redundant auxiliary register group B */
#define ADBMS6948_CMD_RDRAXB		((uint16_t)0x001Du)
/** Command code to read redundant auxiliary register group C */
#define ADBMS6948_CMD_RDRAXC		((uint16_t)0x001Eu)
/** Command code to read redundant auxiliary register group D */
#define ADBMS6948_CMD_RDRAXD		((uint16_t)0x0025u)

/** Command code to read status register group A */
#define ADBMS6948_CMD_RDSTATA		((uint16_t)0x0030u)
/** Command code to read status register group B */
#define ADBMS6948_CMD_RDSTATB		((uint16_t)0x0031u)
/** Command code to read status register group C */
#define ADBMS6948_CMD_RDSTATC		((uint16_t)0x0032u)
/** Command code to read status register group D */
#define ADBMS6948_CMD_RDSTATD		((uint16_t)0x0033u)
/** Command code to read status register group E */
#define ADBMS6948_CMD_RDSTATE		((uint16_t)0x0034u)
/** Command code to read status register group F */
#define ADBMS6948_CMD_RDSTATF		((uint16_t)0x00B0u)
/** Command code to read status register group G */
#define ADBMS6948_CMD_RDSTATG		((uint16_t)0x00B1u)

/** Command code to read all aux and all status register groups */
#define ADBMS6948_CMD_RDASALL		((uint16_t)0x0035u)

/** Command code to read Current register group*/
#define ADBMS6948_CMD_RDI			((uint16_t)0x0084)
/** Command code to read Average Current register group*/
#define ADBMS6948_CMD_RDIAV			((uint16_t)0x00C4)

/** Command code to clear cell voltage register groups */
#define ADBMS6948_CMD_CLRCELL		((uint16_t)0x0711u)
/** Command code to clear I1 values in current register groups */
#define ADBMS6948_CMD_CLRCI		    ((uint16_t)0x0790u)
/** Command code to clear I2 values in current register groups */
#define ADBMS6948_CMD_CLRSI 		((uint16_t)0x0792u)
/** Command code to clear Coulomb counter and Time Base registers */
#define ADBMS6948_CMD_CLRCC 		((uint16_t)0x0794)
/** Command code to clear filtered cell voltage register groups */
#define ADBMS6948_CMD_CLRFC			((uint16_t)0x0714u)
/** Command code to clear auxiliary register groups */
#define ADBMS6948_CMD_CLRAUX		((uint16_t)0x0712u)
/** Command code to clear S voltage register groups */
#define ADBMS6948_CMD_CLRSPIN		((uint16_t)0x0716u)

/** Command code to poll any ADC and authentication status */
#define ADBMS6948_CMD_PLADC			((uint16_t)0x0718u)
/** Command code to poll CADC status */
#define ADBMS6948_CMD_PLCADC		((uint16_t)0x071Cu)
/** Command code to poll SADC status */
#define ADBMS6948_CMD_PLSADC		((uint16_t)0x071Du)
/** Command code to poll AUXADC status */
#define ADBMS6948_CMD_PLAUX1		((uint16_t)0x071Eu)
/** Command code to poll redundant AUXADC status */
#define ADBMS6948_CMD_PLAUX2		((uint16_t)0x071Fu)
/** Command code to poll I1-ADC status */
#define ADBMS6948_CMD_PLI1ADC		((uint16_t)0x071Au)
/** Command code to poll I2-ADC status */
#define ADBMS6948_CMD_PLI2ADC		((uint16_t)0x071Bu)

/** Command code to clear flags */
#define ADBMS6948_CMD_CLRFLAG		((uint16_t)0x0717u)
/** Command code to clear OVUV */
#define ADBMS6948_CMD_CLOVUV		((uint16_t)0x0715u)

/** Command code to enable Coulomb Counter */
#define ADBMS6948_CMD_CCEN          ((uint16_t)0x0056u)
/** Command code to read Coulomb Counter */
#define ADBMS6948_CMD_RDCT          ((uint16_t)0x0086u)

/*---------------------------------- Write Command Codes -----------------------------------------*/
/* Configuration Registers Commands */
/** Write Configuration Register Group A */
#define ADBMS6948_CMD_WRCFGA    ((uint16_t)0x0001u)
/** Write Configuration Register Group B */
#define ADBMS6948_CMD_WRCFGB    ((uint16_t)0x0024u)
/** Write Configuration Register Group C */
#define ADBMS6948_CMD_WRCFGC    ((uint16_t)0x0081u)
/** Write Configuration Register Group D */
#define ADBMS6948_CMD_WRCFGD    ((uint16_t)0x00A4u)
/** Write Configuration Register Group E */
#define ADBMS6948_CMD_WRCFGE    ((uint16_t)0x0073u)
/** Write Configuration Register Group F */
#define ADBMS6948_CMD_WRCFGF    ((uint16_t)0x0075u)
/** Write Configuration Register Group G */
#define ADBMS6948_CMD_WRCFGG    ((uint16_t)0x0077u)
/** Write Configuration Register Group H */
#define ADBMS6948_CMD_WRCFGH    ((uint16_t)0x0079u)
/** Write Configuration Register Group I */
#define ADBMS6948_CMD_WRCFGI    ((uint16_t)0x007Bu)

/** Read Configuration Register Group A */
#define ADBMS6948_CMD_RDCFGA    ((uint16_t)0x0002u)
/** Read Configuration Register Group B */
#define ADBMS6948_CMD_RDCFGB    ((uint16_t)0x0026u)
/** Read Configuration Register Group C */
#define ADBMS6948_CMD_RDCFGC    ((uint16_t)0x0082u)
/** Read Configuration Register Group D */
#define ADBMS6948_CMD_RDCFGD    ((uint16_t)0x00A6u)
/** Read Configuration Register Group E */
#define ADBMS6948_CMD_RDCFGE    ((uint16_t)0x0074u)
/** Read Configuration Register Group F */
#define ADBMS6948_CMD_RDCFGF    ((uint16_t)0x0076u)
/** Read Configuration Register Group G */
#define ADBMS6948_CMD_RDCFGG    ((uint16_t)0x0078u)
/** Read Configuration Register Group H */
#define ADBMS6948_CMD_RDCFGH    ((uint16_t)0x007Au)
/** Read Configuration Register Group I */
#define ADBMS6948_CMD_RDCFGI    ((uint16_t)0x007Cu)

/* PWM Registers Commands */
/** Write PWM A Register Group */
#define ADBMS6948_CMD_WRPWMA     ((uint16_t)0x0020u)
/** Write PWM B Register Group */
#define ADBMS6948_CMD_WRPWMB     ((uint16_t)0x0021u)
/** Read PWM ARegister Group */
#define ADBMS6948_CMD_RDPWMA     ((uint16_t)0x0022u)
/** Read PWM ARegister Group */
#define ADBMS6948_CMD_RDPWMB     ((uint16_t)0x0023u)

/* Master I2C Commands */
/** Command code write COMM register */
#define ADBMS6948_CMD_WRCOMM		((uint16_t)0x721u)
/** Command code read COMM register */
#define ADBMS6948_CMD_RDCOMM		((uint16_t)0x722u)
/** Command code to start I2C communication */
#define ADBMS6948_CMD_STCOMM		((uint16_t)0x723u)

/* Control Commands */
/** Command code to reset command counter  */
#define ADBMS6948_CMD_RSTCC			((uint16_t)0x002Eu)
/** Command code to mute the discharge  */
#define ADBMS6948_CMD_MUTE			((uint16_t)0x0028u)
/** Command code to unmute the discharge  */
#define ADBMS6948_CMD_UNMUTE		((uint16_t)0x0029u)
/** Read Serial ID Register Group RDSID 000 0010 1100 */
#define  ADBMS6948_CMD_RDSID    	((uint16_t)0x002C)
/** Soft Reset Command */
#define ADBMS6948_CMD_SRST      	((uint16_t)0x0027u)
/**Unknown Command*/
#define ADBMS6948_CMD_UNKNOWN		0x00u


/* LPCM Commands */
/** Command code to disable LPCM mode */
#define ADBMS6948_CMD_CMDIS			((uint16_t)0x0040u)

/** Command code to enable LPCM mode */
#define ADBMS6948_CMD_CMEN			((uint16_t)0x0041u)

/** Command code for LPCM heartbeat message */
#define ADBMS6948_CMD_CMHB			((uint16_t)0x0043u)

/** Write LPCM configuration register group */
#define ADBMS6948_CMD_WRCMCFG		((uint16_t)0x0058u)

/** Read LPCM configuration register group */
#define ADBMS6948_CMD_RDCMCFG		((uint16_t)0x0059u)

/** Write LPCM cell threshold register group */
#define ADBMS6948_CMD_WRCMCELLT		((uint16_t)0x005Au)

/** Read LPCM cell threshold register group */
#define ADBMS6948_CMD_RDCMCELLT		((uint16_t)0x005Bu)

/** Write LPCM gpio threshold register group */
#define ADBMS6948_CMD_WRCMGPIOT		((uint16_t)0x005Cu)

/** Read LPCM gpio threshold register group */
#define ADBMS6948_CMD_RDCMGPIOT		((uint16_t)0x005Du)

/** Clear LPCM flags register group */
#define ADBMS6948_CMD_CMCLRFLAG		((uint16_t)0x005Eu)

/** Read LPCM flags register group */
#define ADBMS6948_CMD_RDCMFLAG		((uint16_t)0x005Fu)


/* Always ON commands */
/** Unlock "Always On" flops */
#define ADBMS6948_CMD_ULAO			((uint16_t)0x0038u)

/** Write "Always On" flops */
#define ADBMS6948_CMD_WRAO			((uint16_t)0x0039u)

/** Read "Always On" flops */
#define ADBMS6948_CMD_RDAO			((uint16_t)0x003Au)

/*============= D A T A T Y P E S =============*/

/*======= P U B L I C P R O T O T Y P E S ========*/
void  Adbms6948_Cmd_ExecuteCmdRD
(
    uint16_t 		*pnCmdLst,
    uint8_t     	 pRxBuf[][ADBMS6948_MAX_FRAME_SIZE],
    uint32_t  		 nDataCfg,
	boolean  		 bIsPollCmd,
    const uint8_t    knChainID
);

void  Adbms6948_Cmd_ExecuteCmdWR
(
    uint16_t  		 nCommand,
    uint8_t 		*pTxBuf,
    const uint8_t    knChainID
);

void Adbms6948_Cmd_ExecuteCmdCLR
(
    uint16_t		nCommand,
	uint8_t			nDataLen,
    uint8_t			*pTxBuf,
    const uint8_t	knChainID
);

void  Adbms6948_Cmd_ExecCmd
(
uint16_t   	 	nCommand,
boolean  	 	bIncrementCmdCount,
const uint8_t   knChainID
);

void  Adbms6948_Cmd_SendDummyBytes
(
    uint8_t 			*pBuff,
    uint8_t  		     nLen,
    const uint8_t        knChainID
);

void Adbms6948_Cmd_ExecCmdWithCPECBitFlip
(
uint16_t  		nCommand,
uint8_t			nPECBitToFlip,
const uint8_t  	knChainID
);

void Adbms6948_Cmd_ExecCmdWRWithDPECBitFLip
(
    uint16_t 			nCommand,
    uint8_t				*pTxBuf,
	uint8_t				nDataPECBitToFlip,
    const uint8_t		knChainID
);

void  Adbms6948_Cmd_ExecuteCmdRDAll
(
    uint8_t             *pRxBuf,
    uint16_t            nCmd,
    uint8_t             nRegGrps,
    boolean             bIsPollCmd,
    const uint8_t       knChainID
);


#endif /* ADBMS6948_EXEC_CMD_H */

/** @}*/ /* addtogroup */

/*
*
* EOF
*
*/

