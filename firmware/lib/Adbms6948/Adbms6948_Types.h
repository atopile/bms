/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_Types.h
* @brief  : Type and domain declarations used in the upper layer of the
            Adbms6948 module.
* @version: $Revision: 517 $
* @date   : $Date: 2020-10-13 11:33:31 +0530 (Tue, 13 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup ADI_ADBMS6948_DRIVER_INTERNAL_DATA	ADBMS6948 Driver Internal Data Types
* @{
*/

#ifndef ADBMS6948_TYPES_H
#define ADBMS6948_TYPES_H


/*============= I N C L U D E S =============*/
#include "adi_bms_platform.h"
#include "Adbms6948.h"

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
/*These service IDs are used to report dev error*/

/** Initialization service ID*/
#define ADBMS6948_INIT_ID                     ((uint8_t)0x00u)

/** Initialization service ID*/
#define ADBMS6948_DEINIT_ID                   ((uint8_t)0x01u)

/** Configure soak control service ID*/
#define ADBMS6948_CFGSOAKCTRL_ID              ((uint8_t)0x02u)

/** Configure auxiliary soak time  service ID*/
#define ADBMS6948_CFGAUXSOAKTIME_ID           ((uint8_t)0x03u)

/** Configure Over voltage threshold service ID*/
#define ADBMS6948_CFGOVTHRSLD_ID          	  ((uint8_t)0x05u)

/** Configure under voltage threshold service ID*/
#define ADBMS6948_CFGUVTHRSLD_ID          	  ((uint8_t)0x06u)

/** Configure discharge monitor control service ID*/
#define ADBMS6948_CFGDTMCTRL_ID          	  ((uint8_t)0x07u)

/** Configure discharge timeout value service ID*/
#define ADBMS6948_CFGDISCHARGTIMEOUTVAL_ID	  ((uint8_t)0x08u)

/** Configure continuous discharge control service ID*/
#define ADBMS6948_CFGCONTDISCHARGCTRL_ID      ((uint8_t)0x09u)

/** Configure PWM discharge service ID*/
#define ADBMS6948_CFGPWMDISCHARGCTRL_ID       ((uint8_t)0x0Au)

/** Configure soft reset service ID*/
#define ADBMS6948_SOFTRESET_ID       ((uint8_t)0x0Bu)

/** Configure config filter service ID*/
#define ADBMS6948_CFGIIRFILTER_ID       ((uint8_t)0x0Cu)

/** Configure config CS threshold service ID*/
#define ADBMS6948_CFGCSTHRES_ID       ((uint8_t)0x0Du)

/** Configure config GPIO service ID*/
#define ADBMS6948_CFGGPIO_ID       ((uint8_t)0x0Eu)

/** Configure over voltage eval service ID*/
#define ADBMS6948_OVERVOLTEVAL_ID       ((uint8_t)0x0Fu)

/** Configure under voltage eval service ID*/
#define ADBMS6948_UNDERVOLTEVAL_ID       ((uint8_t)0x10u)

/** Read device revision code service ID. */
#define ADBMS6948_READDEVREVCODE_ID			((uint8_t)0x11)

/** Read Error Count ID */
#define ADBMS6948_READERRCNT_ID		((uint8_t)0x12)

/** Clear Error Count ID */
#define ADBMS6948_CLRERRCNT_ID		((uint8_t)0x13)
/** Command Count callback */
#define ADBMS6948_CMDCNTCB_ID		((uint8_t)0x34)

/** Update device count service ID. */
#define ADBMS6948_UPDATEDEVCNT_ID            ((uint8_t)0x14)

/** Update device count service ID. */
#define ADBMS6948_UPDATEBMSWAKEUPCTRL_ID  ((uint8_t)0x15)

/** Update device count service ID. */
#define ADBMS6948_GETVERSIONINFO_ID          ((uint8_t)0x16)

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

/** Cell open wire diagnostic service ID*/
#define ADBMS6948_CELLOWDIAG_ID				((uint8_t)0x23u)

/** Read cell OVUV status service ID*/
#define ADBMS6948_READCELLOVUVSTATUS_ID		((uint8_t)0x24u)

/** Read CvsS fault status service ID*/
#define ADBMS6948_READCSFLTSTATUS_ID		((uint8_t)0x25u)

/** Read oscillator check count service ID*/
#define ADBMS6948_READOSCCHKCNT_ID			((uint8_t)0x26u)

/** Enable LPF detection service ID*/
#define ADBMS6948_ENABLELPFDET_ID			((uint8_t)0x27u)

/** Read diagnostic flag status service ID*/
#define ADBMS6948_READDIAGFLAGSTATUS_ID		((uint8_t)0x28u)

/** I2C read service ID */
#define ADBMS6948_I2CREAD_ID    			((uint8_t)0x29u)

/** I2C write service ID */
#define ADBMS6948_I2CWRITE_ID				((uint8_t)0x30u)

/** Read device revision code service ID. */
#define ADBMS6948_READDEVSERID_ID    		((uint8_t)0x31u)

/** Configure Coulomb Counter service ID. */
#define ADBMS6948_CFGCCNTR_ID               ((uint8_t)0x32u)

/** Configure OC1 parameters service ID. */
#define ADBMS6948_CFGOC1PARAMS_ID           ((uint8_t)0x33u)

/** Configure OC2 parameters service ID. */
#define ADBMS6948_CFGOC2PARAMS_ID           ((uint8_t)0x35u)

/** Read OC1 parameters service ID. */
#define ADBMS6948_READOC1PARAMS_ID          ((uint8_t)0x36u)

/** Read OC2 parameters service ID. */
#define ADBMS6948_READOC2PARAMS_ID          ((uint8_t)0x37u)

/** Write AlwaysOn Memory service ID. */
#define ADBMS6948_AOMEMWRITE_ID				((uint8_t)0x38u)

/** Read AlwaysOn Memory service ID. */
#define ADBMS6948_AOMEMREAD_ID				((uint8_t)0x39u)

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

/***************************** Other Defines *********************************/
#define ADBMS6948EVAL_MODE	(FALSE)

/** No. of register in a group */
#define ADBMS6948_MAX_NO_OF_CELLS_PER_DEVICE	((uint8_t)16u)

/** No. of register in a group */
#define ADBMS6948_REG_GRP_LEN		  			((uint8_t)0x06u)

/** PEC length */
#define ADBMS6948_PEC_LEN						((uint8_t)0x02u)

/** This macro defines maximum number of register groups read at a time */
#define ADBMS6948_MAX_REG_GRP_READ              ((uint8_t)0x06u)

/** Register Data length with PEC per device  */
#define ADBMS6948_REG_DATA_LEN_WITH_PEC         ((uint8_t)0x08u)

/** Register Data length without PEC per device  */
#define ADBMS6948_REG_DATA_LEN_WITHOUT_PEC      ((uint8_t)0x06u)

/** Register Data length with PEC  */
#define ADBMS6948_CMD_DATA_LEN		  			((uint8_t)0x04u)

/*TODO: Remove ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN macro as Leonie not intended for daisy chain*/
/*FIXME: Change length of one register later*/
/** Maximum size of a frame */
#define ADBMS6948_MAX_FRAME_SIZE				(ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN))

/** Maximum frame size of "read All" frame*/
/**FIXME: Above macro will fail for ReadCIV types of commands*/
#define ADBMS6948_MAX_READALL_FRAME_SIZE        (ADBMS6948_CMD_DATA_LEN + ADBMS6948_REG_DATA_LEN_WITHOUT_PEC * ADBMS6948_MAX_REGISTERS_IN_A_GRP)

/** Number of Reg group/s to be read for CADC conversion counter */
#define ADBMS6948_NUMOF_REGGRP_CONVCNT			((uint8_t)0x01u)

/** Number of cell data types */
#define ADBMS6948_NUMOF_CELLDATA_TYPES			((uint8_t)0x04u)

/** Maximum Number of cell data register groups */
#define ADBMS6948_MAX_CELLDATA_REGGRPS			((uint8_t)0x06u)

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

/** Maximum number of control (4 bytes) commands for which update and verify CC for non-Leonie CDD is pending.*/
#define ADBMS6948_CTRLCMDLST_SIZE_MAX			((uint8_t)10u)

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

/** Cell OW status: Not Detected */
#define ADBMS6948_CELLOW_NOT_DETECTED			((uint8_t)0u)

/** Cell OW status: Detected */
#define ADBMS6948_CELLOW_DETECTED				((uint8_t)1u)

/** Cell OW status: Invalid */
#define ADBMS6948_CELLOW_INVALID				((uint8_t)2u)

/** MASKs and POSITIONs */
/** Mask and position for OW[1:0] bits in ADCV command */
#define ADBMS6948_ADCV_MASK_OW		((uint16_t)0x0003u)
#define ADBMS6948_ADCV_POS_OW		((uint8_t)0x00u)
/** Mask and position for RSTF bit in ADCV command */
#define ADBMS6948_ADCV_MASK_RSTF	((uint16_t)0x0004u)
#define ADBMS6948_ADCV_POS_RSTF		((uint8_t)0x02u)
/** Mask and position for SSDP bit in ADCV command */
#define ADBMS6948_ADCV_MASK_SSDP	((uint16_t)0x0010u)
#define ADBMS6948_ADCV_POS_SSDP		((uint8_t)0x04u)
/** Mask and position for CONT bit in ADCV command */
#define ADBMS6948_ADCV_MASK_CONT	((uint16_t)0x0080u)
#define ADBMS6948_ADCV_POS_CONT		((uint8_t)0x07u)
/** Mask and position for RD bit in ADCV command */
#define ADBMS6948_ADCV_MASK_RD		((uint16_t)0x0100u)
#define ADBMS6948_ADCV_POS_RD		((uint8_t)0x08u)

/** Mask and position for OW[1:0] bits in ADSV command */
#define ADBMS6948_ADSV_MASK_OW		((uint16_t)0x0003u)
#define ADBMS6948_ADSV_POS_OW		((uint8_t)0x00u)
/** Mask and position for SSDP bit in ADSV command */
#define ADBMS6948_ADSV_MASK_SSDP	((uint16_t)0x0010u)
#define ADBMS6948_ADSV_POS_SSDP		((uint8_t)0x04u)
/** Mask and position for CONT bit in ADSV command */
#define ADBMS6948_ADSV_MASK_CONT	((uint16_t)0x0080u)
#define ADBMS6948_ADSV_POS_CONT		((uint8_t)0x07u)

/** Mask and position for OW[1:0] bits in ADI1 command */
#define ADBMS6948_ADI1_MASK_OW		((uint16_t)0x0003u)
#define ADBMS6948_ADI1_POS_OW		((uint8_t)0x00u)
/** Mask and position for CONT bit in ADI1 command */
#define ADBMS6948_ADI1_MASK_CONT	((uint16_t)0x0080u)
#define ADBMS6948_ADI1_POS_CONT		((uint8_t)0x07u)
/** Mask and position for RD bit in ADI1 command */
#define ADBMS6948_ADI1_MASK_RD		((uint16_t)0x0100u)
#define ADBMS6948_ADI1_POS_RD		((uint8_t)0x08u)

/** Mask and position for OW[1:0] bits in ADI2 command */
#define ADBMS6948_ADI2_MASK_OW		((uint16_t)0x0003u)
#define ADBMS6948_ADI2_POS_OW		((uint8_t)0x00u)
/** Mask and position for CONT bit in ADI2 command */
#define ADBMS6948_ADI2_MASK_CONT	((uint16_t)0x0080u)
#define ADBMS6948_ADI2_POS_CONT		((uint8_t)0x07u)

/** Mask and position for OW[1:0] bits in ADCIV command */
#define ADBMS6948_ADCIV_MASK_OW		((uint16_t)0x0003u)
#define ADBMS6948_ADCIV_POS_OW		((uint8_t)0x00u)
/** Mask and position for RSTF bit in ADCIV command */
#define ADBMS6948_ADCIV_MASK_RSTF	((uint16_t)0x0004u)
#define ADBMS6948_ADCIV_POS_RSTF	((uint8_t)0x02u)
/** Mask and position for SSDP bit in ADCIV command */
#define ADBMS6948_ADCIV_MASK_SSDP	((uint16_t)0x0010u)
#define ADBMS6948_ADCIV_POS_SSDP	((uint8_t)0x04u)
/** Mask and position for CONT bit in ADCIV command */
#define ADBMS6948_ADCIV_MASK_CONT	((uint16_t)0x0080u)
#define ADBMS6948_ADCIV_POS_CONT	((uint8_t)0x07u)
/** Mask and position for RD bit in ADCIV command */
#define ADBMS6948_ADCIV_MASK_RD		((uint16_t)0x0100u)
#define ADBMS6948_ADCIV_POS_RD		((uint8_t)0x08u)

/** Mask and position for CHB[4:0] bits in ADAX command */
#define ADBMS6948_ADAX_MASK_CH		((uint16_t)0x004Fu)
#define ADBMS6948_ADAX_POS_CH		((uint8_t)0x00u)
/** Mask and position for CH_LSB[3:0] bits in ADAX command */
#define ADBMS6948_ADAX_MASK_CH_LSB	((uint16_t)0x000Fu)
#define ADBMS6948_ADAX_POS_CH_LSB	((uint8_t)0x00u)
/** Mask and position for CH_MSB[4] bit in ADAX command */
#define ADBMS6948_ADAX_MASK_CH_MSB	((uint16_t)0x0040u)
#define ADBMS6948_ADAX_POS_CH_MSB	((uint8_t)0x06u)
/** Mask and position for PUP bit in ADAX command */
#define ADBMS6948_ADAX_MASK_PUP		((uint16_t)0x0080u)
#define ADBMS6948_ADAX_POS_PUP		((uint8_t)0x07u)
/** Mask and position for OW bit in ADAX command */
#define ADBMS6948_ADAX_MASK_OW		((uint16_t)0x0100u)
#define ADBMS6948_ADAX_POS_OW		((uint8_t)0x08u)

/** Mask and position for CH[3:0] bits in ADAX2 command */
#define ADBMS6948_ADAX2_MASK_CH		((uint16_t)0x000Fu)
#define ADBMS6948_ADAX2_POS_CH		((uint8_t)0x00u)

/** Mask and position for ERR bit in RDSTATC command */
#define ADBMS6948_RDSTATC_MASK_ERR	((uint16_t)0x0040u)
#define ADBMS6948_RDSTATC_POS_ERR	((uint8_t)0x06u)

/** Mask and position for CCEN_CON bit in CCEN command */
#define ADBMS6948_CCEN_MASK_CCEN_CON    ((uint16_t)0x0080)
#define ADBMS6948_CCEN_POS_CCEN_CON     ((uint8_t)0x07u)

/******************* CONFIG MASK and POS *********************/
/** Mask and position of REFON bit in CFGAR0. */
#define ADBMS6948_CFGAR0_MASK_REFON			((uint8_t)0x80u)
#define ADBMS6948_CFGAR0_POS_REFON			((uint8_t)0x07u)

/** Mask and position of VA_CLKON bit in CFGAR0. */
#define ADBMS6948_CFGAR0_MASK_VA_CLKON      ((uint8_t)0x40u)
#define ADBMS6948_CFGAR0_POS_VA_CLKON       ((uint8_t)0x06u)

/** Mask and position of CTH field in CFGAR0. */
#define ADBMS6948_CFGAR0_MASK_CTH			((uint8_t)0x07u)
#define ADBMS6948_CFGAR0_POS_CTH			((uint8_t)0x00u)

/** Mask and position of FLAG_D field in CFGAR1. */
#define ADBMS6948_CFGAR0_MASK_FLAG_D		((uint8_t)0xFFu)
#define ADBMS6948_CFGAR0_POS_FLAG_D			((uint8_t)0x00u)

/** Mask and position of SOAKON bit in CFGAR2. */
#define ADBMS6948_CFGAR2_MASK_SOAKON		((uint8_t)0x80u)
#define ADBMS6948_CFGAR2_POS_SOAKON			((uint8_t)0x07u)

/** Mask and position of OWRNG bit in CFGAR2. */
#define ADBMS6948_CFGAR2_MASK_OWRNG			((uint8_t)0x40u)
#define ADBMS6948_CFGAR2_POS_OWRNG			((uint8_t)0x06u)

/** Mask and position of OWA field in CFGAR2. */
#define ADBMS6948_CFGAR2_MASK_OWA			((uint8_t)0x38u)
#define ADBMS6948_CFGAR2_POS_OWA			((uint8_t)0x03u)

/** Mask and position of GPO field in CFGAR3. */
#define ADBMS6948_CFGAR3_MASK_GPO			((uint8_t)0xFFu)
#define ADBMS6948_CFGAR3_POS_GPO			((uint8_t)0x00u)

/** Mask and position of FLAG_D field in CFGAR4. */
#define ADBMS6948_CFGAR4_MASK_FLAG_D		((uint8_t)0xC0u)
#define ADBMS6948_CFGAR4_POS_FLAG_D			((uint8_t)0x06u)

/** Mask and position of GPO field in CFGAR4. */
#define ADBMS6948_CFGAR4_MASK_GPO			((uint8_t)0x07u)
#define ADBMS6948_CFGAR4_POS_GPO			((uint8_t)0x00u)

/** Mask and position of FC bit in CFGAR5. */
#define ADBMS6948_CFGAR5_MASK_FC			((uint8_t)0x07u)
#define ADBMS6948_CFGAR5_POS_FC				((uint8_t)0x00u)

/** Mask and position of MUTE_ST bit in CFGAR5. */
#define ADBMS6948_CFGAR5_MASK_MUTE_ST		((uint8_t)0x10u)
#define ADBMS6948_CFGAR5_POS_MUTE_ST		((uint8_t)0x04u)

/** Mask and position of SNAP_ST bit in CFGAR5. */
#define ADBMS6948_CFGAR5_MASK_SNAP_ST       ((uint8_t)0x20u)
#define ADBMS6948_CFGAR5_POS_SNAP_ST        ((uint8_t)0x05u)

/** Mask and position of VUV_LSB field in CFGBR0. */
#define ADBMS6948_CFGBR0_MASK_VUV_LSB		((uint8_t)0xFFu)
#define ADBMS6948_CFGBR0_POS_VUV_LSB		((uint8_t)0x00u)

/** Mask and position of VUV_MSB field in CFGBR1. */
#define ADBMS6948_CFGBR1_MASK_VUV_MSB		((uint8_t)0x0Fu)
#define ADBMS6948_CFGBR1_POS_VUV_MSB		((uint8_t)0x00u)

/** Mask and position of VOV_LSB field in CFGBR1. */
#define ADBMS6948_CFGBR1_MASK_VOV_LSB		((uint8_t)0xF0u)
#define ADBMS6948_CFGBR1_POS_VOV_LSB		((uint8_t)0x04u)

/** Mask and position of VOV_MSB field in CFGBR2. */
#define ADBMS6948_CFGBR2_MASK_VOV_MSB		((uint8_t)0xFFu)
#define ADBMS6948_CFGBR2_POS_VOV_MSB		((uint8_t)0x00u)

/** Mask and position of DTMEN field in CFGBR3. */
#define ADBMS6948_CFGBR3_MASK_DTMEN			((uint8_t)0x80u)
#define ADBMS6948_CFGBR3_POS_DTMEN			((uint8_t)0x07u)

/** Mask and position of DTRNG field in CFGBR3. */
#define ADBMS6948_CFGBR3_MASK_DTRNG			((uint8_t)0x40u)
#define ADBMS6948_CFGBR3_POS_DTRNG			((uint8_t)0x06u)

/** Mask and position of DCTO field in CFGBR3. */
#define ADBMS6948_CFGBR3_MASK_DCTO			((uint8_t)0x3Fu)
#define ADBMS6948_CFGBR3_POS_DCTO			((uint8_t)0x00u)

/** Mask and position of DCC_LSB field in CFGBR4. */
#define ADBMS6948_CFGBR4_MASK_DCC_LSB		((uint8_t)0xFFu)
#define ADBMS6948_CFGBR4_POS_DCC_LSB		((uint8_t)0x00u)

/** Mask and position of DCC_MSB field in CFGBR5. */
#define ADBMS6948_CFGBR5_MASK_DCC_MSB		((uint8_t)0xFFu)
#define ADBMS6948_CFGBR5_POS_DCC_MSB		((uint8_t)0x00u)

/** Mask and position of TCC_LSB field in CFGCR0. */
#define ADBMS6948_CFGCR0_MASK_TCC_LSB		((uint8_t)0xFFu)
#define ADBMS6948_CFGCR0_POS_TCC_LSB		((uint8_t)0x00u)

/** Mask and position of TCC_MSB field in CFGCR1. */
#define ADBMS6948_CFGCR0_MASK_TCC_MSB		((uint8_t)0xFFu)
#define ADBMS6948_CFGCR0_POS_TCC_MSB		((uint8_t)0x00u)

/** Mask and position of CHOP_BUFF field in CFGCR2. */
#define ADBMS6948_CFGCR0_MASK_CHOP_BUFF		((uint8_t)0x01u)
#define ADBMS6948_CFGCR0_POS_CHOP_BUFF		((uint8_t)0x00u)

/** Mask and position of PD_BUFF field in CFGCR2. */
#define ADBMS6948_CFGCR0_MASK_PD_BUFF		((uint8_t)0x02u)
#define ADBMS6948_CFGCR0_POS_PD_BUFF		((uint8_t)0x01u)

/** Mask and position of PD_CALBG field in CFGCR2. */
#define ADBMS6948_CFGCR0_MASK_PD_CALBG		((uint8_t)0x04u)
#define ADBMS6948_CFGCR0_POS_PD_CALBG		((uint8_t)0x02u)

/** Mask and position of PD_CALBG_OTA field in CFGCR2. */
#define ADBMS6948_CFGCR0_MASK_PD_CALBG_OTA	((uint8_t)0x08u)
#define ADBMS6948_CFGCR0_POS_PD_CALBG_OTA	((uint8_t)0x03u)

/** Mask and position of DEM_CALBG field in CFGCR2. */
#define ADBMS6948_CFGCR0_MASK_DEM_CALBG		((uint8_t)0xF0u)
#define ADBMS6948_CFGCR0_POS_DEM_CALBG		((uint8_t)0x04u)

/** Mask and position of MUXSEL_CALBG field in CFGCR3. */
#define ADBMS6948_CFGCR0_MASK_MUXSEL_CALBG	((uint8_t)0x07u)
#define ADBMS6948_CFGCR0_POS_MUXSEL_CALBG	((uint8_t)0x00u)

/** Mask and position of CHOP_CALBG_LSB field in CFGCR3. */
#define ADBMS6948_CFGCR0_MASK_CHOP_CALBG_LSB ((uint8_t)0xF8u)
#define ADBMS6948_CFGCR0_POS_CHOP_CALBG_LSB	 ((uint8_t)0x04u)

/** Mask and position of CHOP_CALBG_MSB field in CFGCR4. */
#define ADBMS6948_CFGCR0_MASK_CHOP_CALBG_MSB ((uint8_t)0x01u)
#define ADBMS6948_CFGCR0_POS_CHOP_CALBG_MSB	 ((uint8_t)0x00u)

/** Mask and position of G11_SEL field in CFGCR4. */
#define ADBMS6948_CFGCR0_MASK_G11_SEL	    ((uint8_t)0xC0u)
#define ADBMS6948_CFGCR0_POS_G11_SEL	    ((uint8_t)0x06u)

/** Mask and position of CYCLES_W1 field in CFGDR0. */
#define ADBMS6948_CFGDR0_MASK_CYCLES_W1     ((uint8_t)0x1Fu)
#define ADBMS6948_CFGDR0_POS_CYCLES_W1      ((uint8_t)0x00u)
/** Mask and position of CYCLES_W2 field in CFGDR0. */
#define ADBMS6948_CFGDR0_MASK_CYCLES_W2     ((uint8_t)0xE0u)
#define ADBMS6948_CFGDR0_POS_CYCLES_W2      ((uint8_t)0x05u)
/** Mask and position of CYCLES_W2 field in CFGDR1. */
#define ADBMS6948_CFGDR1_MASK_CYCLES_W2     ((uint8_t)0x03u)
#define ADBMS6948_CFGDR1_POS_CYCLES_W2      ((uint8_t)0x00u)
/** Mask and position of CYCLES_W3 field in CFGDR1. */
#define ADBMS6948_CFGDR1_MASK_CYCLES_W3     ((uint8_t)0x7Cu)
#define ADBMS6948_CFGDR1_POS_CYCLES_W3      ((uint8_t)0x02u)
/** Mask and position of CYCLES_W4 field in CFGDR1. */
#define ADBMS6948_CFGDR1_MASK_CYCLES_W4     ((uint8_t)0x80u)
#define ADBMS6948_CFGDR1_POS_CYCLES_W4      ((uint8_t)0x07u)
/** Mask and position of CYCLES_W4 field in CFGDR2. */
#define ADBMS6948_CFGDR2_MASK_CYCLES_W4     ((uint8_t)0x0Fu)
#define ADBMS6948_CFGDR2_POS_CYCLES_W4      ((uint8_t)0x00u)
/** Mask and position of CYCLES_W5 field in CFGDR2. */
#define ADBMS6948_CFGDR2_MASK_CYCLES_W5     ((uint8_t)0xF0u)
#define ADBMS6948_CFGDR2_POS_CYCLES_W5      ((uint8_t)0x04u)
/** Mask and position of CYCLES_W5 field in CFGDR3. */
#define ADBMS6948_CFGDR3_MASK_CYCLES_W5     ((uint8_t)0x01u)
#define ADBMS6948_CFGDR3_POS_CYCLES_W5      ((uint8_t)0x00u)
/** Mask and position of CYCLES_W6 field in CFGDR3. */
#define ADBMS6948_CFGDR3_MASK_CYCLES_W6     ((uint8_t)0x3Eu)
#define ADBMS6948_CFGDR3_POS_CYCLES_W6      ((uint8_t)0x01u)
/** Mask and position of WARN_CFG field in CFGDR4. */
#define ADBMS6948_CFGDR4_MASK_WARN_CFG     ((uint8_t)0xF8u)
#define ADBMS6948_CFGDR4_POS_WARN_CFG      ((uint8_t)0x03u)
/** Mask and position of GAIN_CTRL1 field in CFGDR5. */
#define ADBMS6948_CFGDR5_MASK_GAIN_CTRL1     ((uint8_t)0xFFu)
#define ADBMS6948_CFGDR5_POS_GAIN_CTRL1      ((uint8_t)0x00u)


/** Mask and position of WARN_W1 field in CFGER0. */
#define ADBMS6948_CFGER0_MASK_WARN_W1     ((uint8_t)0xFFu)
#define ADBMS6948_CFGER0_POS_WARN_W1      ((uint8_t)0x00u)
/** Mask and position of WARN_W2 field in CFGER1. */
#define ADBMS6948_CFGER1_MASK_WARN_W2     ((uint8_t)0xFFu)
#define ADBMS6948_CFGER1_POS_WARN_W2      ((uint8_t)0x00u)
/** Mask and position of WARN_W3 field in CFGER2. */
#define ADBMS6948_CFGER2_MASK_WARN_W3     ((uint8_t)0xFFu)
#define ADBMS6948_CFGER2_POS_WARN_W3      ((uint8_t)0x00u)
/** Mask and position of WARN_W4 field in CFGER3. */
#define ADBMS6948_CFGER3_MASK_WARN_W4     ((uint8_t)0xFFu)
#define ADBMS6948_CFGER3_POS_WARN_W4      ((uint8_t)0x00u)
/** Mask and position of WARN_W5 field in CFGER4. */
#define ADBMS6948_CFGER4_MASK_WARN_W5     ((uint8_t)0xFFu)
#define ADBMS6948_CFGER4_POS_WARN_W5      ((uint8_t)0x00u)
/** Mask and position of WARN_W6 field in CFGER5. */
#define ADBMS6948_CFGER5_MASK_WARN_W6     ((uint8_t)0xFFu)
#define ADBMS6948_CFGER5_POS_WARN_W6      ((uint8_t)0x00u)

/** Mask and position of TH_W1 field in CFGFR0. */
#define ADBMS6948_CFGFR0_MASK_TH_W1     ((uint8_t)0xFFu)
#define ADBMS6948_CFGFR0_POS_TH_W1      ((uint8_t)0x00u)
/** Mask and position of TH_W2 field in CFGFR1. */
#define ADBMS6948_CFGFR1_MASK_TH_W2     ((uint8_t)0xFFu)
#define ADBMS6948_CFGFR1_POS_TH_W2      ((uint8_t)0x00u)
/** Mask and position of TH_W3 field in CFGFR2. */
#define ADBMS6948_CFGFR2_MASK_TH_W3     ((uint8_t)0xFFu)
#define ADBMS6948_CFGFR2_POS_TH_W3      ((uint8_t)0x00u)
/** Mask and position of TH_W4 field in CFGFR3. */
#define ADBMS6948_CFGFR3_MASK_TH_W4     ((uint8_t)0xFFu)
#define ADBMS6948_CFGFR3_POS_TH_W4      ((uint8_t)0x00u)
/** Mask and position of TH_W5 field in CFGFR4. */
#define ADBMS6948_CFGFR4_MASK_TH_W5     ((uint8_t)0xFFu)
#define ADBMS6948_CFGFR4_POS_TH_W5      ((uint8_t)0x00u)
/** Mask and position of TH_W6 field in CFGFR5. */
#define ADBMS6948_CFGFR5_MASK_TH_W6     ((uint8_t)0xFFu)
#define ADBMS6948_CFGFR5_POS_TH_W6      ((uint8_t)0x00u)

/******************* LPCM CONFIG MASK and POS *********************/
/** Mask and position of CMC_MAN field in CMCFGR0. */
#define ADBMS6948_CMCFGR0_MASK_CMC_MAN  ((uint8_t)0x80u)
#define ADBMS6948_CMCFGR0_POS_CMC_MAN   ((uint8_t)0x07u)
/** Mask and position of CMC_MPER field in CMCFGR0. */
#define ADBMS6948_CMCFGR0_MASK_CMC_MPER ((uint8_t)0x70u)
#define ADBMS6948_CMCFGR0_POS_CMC_MPER  ((uint8_t)0x04u)
/** Mask and position of CMC_SEM field in CMCFGR0. */
#define ADBMS6948_CMCFGR0_MASK_CMC_SEM  ((uint8_t)0x08u)
#define ADBMS6948_CMCFGR0_POS_CMC_SEM   ((uint8_t)0x03u)
/** Mask and position of CMC_SPER field in CMCFGR0. */
#define ADBMS6948_CMCFGR0_MASK_CMC_SPER ((uint8_t)0x07u)
#define ADBMS6948_CMCFGR0_POS_CMC_SPER  ((uint8_t)0x00u)
/** Mask and position of CMC_NDEV field in CMCFGR1. */
#define ADBMS6948_CMCFGR1_MASK_CMC_NDEV ((uint8_t)0xFFu)
#define ADBMS6948_CMCFGR1_POS_CMC_NDEV  ((uint8_t)0x00u)
/** Mask and position of CMM_C field in CMCFGR2. */
#define ADBMS6948_CMCFGR2_MASK_CMM_C    ((uint8_t)0xFFu)
#define ADBMS6948_CMCFGR2_POS_CMM_C     ((uint8_t)0x00u)
/** Mask and position of CMM_C field in CMCFGR3. */
#define ADBMS6948_CMCFGR3_MASK_CMM_C    ((uint8_t)0xFFu)
#define ADBMS6948_CMCFGR3_POS_CMM_C     ((uint8_t)0x00u)
/** Mask and position of CMM_C field in CMCFGR4. */
#define ADBMS6948_CMCFGR4_MASK_CMM_C    ((uint8_t)0x03u)
#define ADBMS6948_CMCFGR4_POS_CMM_C     ((uint8_t)0x00u)
/** Mask and position of CMM_GOE field in CMCFGR4. */
#define ADBMS6948_CMCFGR4_MASK_CMM_GOE  ((uint8_t)0x1Cu)
#define ADBMS6948_CMCFGR4_POS_CMM_GOE   ((uint8_t)0x02u)
/** Mask and position of CMM_G field in CMCFGR4. */
#define ADBMS6948_CMCFGR4_MASK_CMM_G    ((uint8_t)0xE0u)
#define ADBMS6948_CMCFGR4_POS_CMM_G     ((uint8_t)0x05u)
/** Mask and position of CMM_G field in CMCFGR5. */
#define ADBMS6948_CMCFGR5_MASK_CMM_G    ((uint8_t)0xFFu)
#define ADBMS6948_CMCFGR5_POS_CMM_G     ((uint8_t)0x00u)

/******************* STATUS MASK and POS *********************/
/** Mask and position of COMP_I bit in STCR2*/
#define ADBMS6948_STCR2_MASK_COMP_I	    ((uint8_t)0x01u)
#define ADBMS6948_STCR2_POS_COMP_I	    ((uint8_t)0x00u)
/** Mask and position of OC1 bit in STCR2*/
#define ADBMS6948_STCR2_MASK_OC1	    ((uint8_t)0x02u)
#define ADBMS6948_STCR2_POS_OC1	        ((uint8_t)0x01u)
/** Mask and position of OC2 bit in STCR2*/
#define ADBMS6948_STCR2_MASK_OC2	    ((uint8_t)0x04u)
#define ADBMS6948_STCR2_POS_OC2	        ((uint8_t)0x02u)
/** Mask and position of OCW1 bit in STCR2*/
#define ADBMS6948_STCR2_MASK_OCW1	    ((uint8_t)0x08u)
#define ADBMS6948_STCR2_POS_OCW1        ((uint8_t)0x03u)
/** Mask and position of OCW2 bit in STCR2*/
#define ADBMS6948_STCR2_MASK_OCW2	    ((uint8_t)0x10u)
#define ADBMS6948_STCR2_POS_OCW2        ((uint8_t)0x04u)
/** Mask and position of CCFLT bit in STCR2*/
#define ADBMS6948_STCR2_MASK_CCFLT	    ((uint8_t)0x20u)
#define ADBMS6948_STCR2_POS_CCFLT       ((uint8_t)0x05u)
/** Mask and position of TB_OF bit in STCR2*/
#define ADBMS6948_STCR2_MASK_TB_OF	    ((uint8_t)0x40u)
#define ADBMS6948_STCR2_POS_TB_OF        ((uint8_t)0x06u)
/** Mask and position of SEQCC bit in STCR2*/
#define ADBMS6948_STCR2_MASK_SEQCC	    ((uint8_t)0x80u)
#define ADBMS6948_STCR2_POS_SEQCC       ((uint8_t)0x07u)

/** Mask and position of SMED bit in STCR4*/
#define ADBMS6948_STCR4_MASK_SMED	    ((uint8_t)0x01u)
#define ADBMS6948_STCR4_POS_SMED        ((uint8_t)0x00u)
/** Mask and position of SED bit in STCR4*/
#define ADBMS6948_STCR4_MASK_SED	    ((uint8_t)0x02u)
#define ADBMS6948_STCR4_POS_SED         ((uint8_t)0x01u)
/** Mask and position of CMED bit in STCR4*/
#define ADBMS6948_STCR4_MASK_CMED	    ((uint8_t)0x04u)
#define ADBMS6948_STCR4_POS_CMED        ((uint8_t)0x02u)
/** Mask and position of CED bit in STCR4*/
#define ADBMS6948_STCR4_MASK_CED	    ((uint8_t)0x08u)
#define ADBMS6948_STCR4_POS_CED         ((uint8_t)0x03u)
/** Mask and position of VD_UV bit in STCR4*/
#define ADBMS6948_STCR4_MASK_VD_UV	    ((uint8_t)0x10u)
#define ADBMS6948_STCR4_POS_VD_UV       ((uint8_t)0x04u)
/** Mask and position of VD_OV bit in STCR4*/
#define ADBMS6948_STCR4_MASK_VD_OV	    ((uint8_t)0x20u)
#define ADBMS6948_STCR4_POS_VD_OV       ((uint8_t)0x05u)
/** Mask and position of VA_UV bit in STCR4*/
#define ADBMS6948_STCR4_MASK_VA_UV	    ((uint8_t)0x40u)
#define ADBMS6948_STCR4_POS_VA_UV       ((uint8_t)0x06u)
/** Mask and position of VA_OV bit in STCR4*/
#define ADBMS6948_STCR4_MASK_VA_OV	    ((uint8_t)0x80u)
#define ADBMS6948_STCR4_POS_VA_OV       ((uint8_t)0x07u)

/** Mask and position of OSCHK bit in STCR5*/
#define ADBMS6948_STCR5_MASK_OSCHK	    ((uint8_t)0x01u)
#define ADBMS6948_STCR5_POS_OSCHK       ((uint8_t)0x00u)
/** Mask and position of TMODCHK bit in STCR5*/
#define ADBMS6948_STCR5_MASK_TMODCHK	((uint8_t)0x02u)
#define ADBMS6948_STCR5_POS_TMODCHK     ((uint8_t)0x01u)
/** Mask and position of THSD bit in STCR5*/
#define ADBMS6948_STCR5_MASK_THSD	    ((uint8_t)0x04u)
#define ADBMS6948_STCR5_POS_THSD        ((uint8_t)0x02u)
/** Mask and position of SLEEP bit in STCR5*/
#define ADBMS6948_STCR5_MASK_SLEEP	    ((uint8_t)0x08u)
#define ADBMS6948_STCR5_POS_SLEEP       ((uint8_t)0x03u)
/** Mask and position of SPIFLT bit in STCR5*/
#define ADBMS6948_STCR5_MASK_SPIFLT	    ((uint8_t)0x10u)
#define ADBMS6948_STCR5_POS_SPIFLT      ((uint8_t)0x04u)
/** Mask and position of COMP_V bit in STCR5*/
#define ADBMS6948_STCR5_MASK_COMP_V	    ((uint8_t)0x20u)
#define ADBMS6948_STCR5_POS_COMP_V      ((uint8_t)0x05u)
/** Mask and position of VDEL bit in STCR5*/
#define ADBMS6948_STCR5_MASK_VDEL	    ((uint8_t)0x40u)
#define ADBMS6948_STCR5_POS_VDEL        ((uint8_t)0x06u)
/** Mask and position of VDE bit in STCR5*/
#define ADBMS6948_STCR5_MASK_VDE	    ((uint8_t)0x80u)
#define ADBMS6948_STCR5_POS_VDE         ((uint8_t)0x07u)

/** Mask and position of GPI8 to GPI1 values in STER4*/
#define ADBMS6948_STER4_MASK_GPI        ((uint8_t)0xFFu)
#define ADBMS6948_STER4_POS_GPI         ((uint8_t)0x00u)

/** Mask and position of GPI11 to GP9 values in STER5*/
#define ADBMS6948_STER5_MASK_GPI        ((uint8_t)0x07u)
#define ADBMS6948_STER5_POS_GPI         ((uint8_t)0x00u)
/** Mask and position of GPI11 to GP9 values in STER5*/
#define ADBMS6948_STER5_MASK_GPI        ((uint8_t)0x07u)
#define ADBMS6948_STER5_POS_GPI         ((uint8_t)0x00u)
/** Mask and position of FAULTB bit in STER5*/
#define ADBMS6948_STER5_MASK_FAULTB     ((uint8_t)0x08u)
#define ADBMS6948_STER5_POS_FAULTB      ((uint8_t)0x03u)
/** Mask and position of Revision ID in STER5*/
#define ADBMS6948_STER5_MASK_REV        ((uint8_t)0xF0u)
#define ADBMS6948_STER5_POS_REV         ((uint8_t)0x04u)

/** Mask and position of CT_V in STGR0*/
#define ADBMS6948_STGR0_MASK_CT_V       ((uint8_t)0x1Fu)
#define ADBMS6948_STGR0_POS_CT_V        ((uint8_t)0x00u)

/** Mask and position of CT_V in STGR1*/
#define ADBMS6948_STGR1_MASK_CT_V       ((uint8_t)0xFCu)
#define ADBMS6948_STGR1_POS_CT_V        ((uint8_t)0x02u)
/** Mask and position of CTS_V in STGR1*/
#define ADBMS6948_STGR1_MASK_CTS_V      ((uint8_t)0x03u)
#define ADBMS6948_STGR1_POS_CTS_V       ((uint8_t)0x00u)

/** Mask and position of CT_I in STGR2*/
#define ADBMS6948_STGR2_MASK_CT_I       ((uint8_t)0x1Fu)
#define ADBMS6948_STGR2_POS_CT_I        ((uint8_t)0x00u)

/** Mask and position of CT_I in STGR3*/
#define ADBMS6948_STGR3_MASK_CT_I       ((uint8_t)0xFCu)
#define ADBMS6948_STGR3_POS_CT_I        ((uint8_t)0x02u)
/** Mask and position of CTS_I in STGR3*/
#define ADBMS6948_STGR3_MASK_CTS_I      ((uint8_t)0x03u)
#define ADBMS6948_STGR3_POS_CTS_I       ((uint8_t)0x00u)

/** Refon enable. */
#define ADBMS6948_CFGAR0_REFON_EN			((uint8_t)0x80u)

#define ADBMS6948_GET_SOAK_TIME(nPowOf2)	(uint32_t)((uint32_t)1u << (nPowOf2))

#define ADBMS6948_SERIAL_ID_LEN				((uint8_t)6u);

/*========================================= I2C Defines ==========================================*/

/* I2C write control */
#define ADBMS6948_I2C_CTRL_WR    ((uint8_t)0x00)
#define ADBMS6948_I2C_CTRL_RD    ((uint8_t)0x01)

/* 0110 Start Generate a start signal on the I2C port, followed by data
  * transmission */
#define ADBMS6948_I2C_ICOMWR_START    ((uint8_t)0x60)
/* 0001 Stop Generate a stop signal on the I2C port */
#define ADBMS6948_I2C_ICOMWR_STOP    ((uint8_t)0x10)
/* 0000 Blank Proceed directly to data transmission on the I2C port */
#define ADBMS6948_I2C_ICOMWR_BLANK    ((uint8_t)0x00)
/* 0111 No transmit Release SDA and SCL and ignore the rest of the data */
#define ADBMS6948_I2C_ICOMWR_NOTX    ((uint8_t)0x70)

/* 0000 Master acknowledge Master generates an acknowledge signal on the ninth
 * clock cycle */
#define ADBMS6948_I2C_FCOMWR_MACK    ((uint8_t)0x00)
/* 1000 Master no acknowledge   Master generates a no acknowledge signal on the
 * ninth clock cycle */
#define ADBMS6948_I2C_FCOMWR_MNACK    ((uint8_t)0x08)
/* 1001 Master no acknowledge and stop Master generates a no acknowledge signal
 * followed by stop signal */
#define ADBMS6948_I2C_FCOMWR_MNACKSTOP    ((uint8_t)0x09)

/* 0000 Master generated an acknowledge signal */
#define ADBMS6948_I2C_FCOMRD_MACK    ((uint8_t)0x00)
/* 0111 Slave generated an acknowledge signal */
#define ADBMS6948_I2C_FCOMRD_SACK    ((uint8_t)0x07)
/* 1111 Slave generated a no acknowledge signal */
#define ADBMS6948_I2C_FCOMRD_SNACK    ((uint8_t)0x0F)
/* 0001 Slave generated an acknowledge signal, master generated a stop signal */
#define ADBMS6948_I2C_FCOMRD_SACKMSTOP    ((uint8_t)0x01)
/* 1001 Slave generated a no acknowledge signal, master generated a stop signal */
#define ADBMS6948_I2C_FCOMRD_SNACKMSTOP    ((uint8_t)0x09)

/* FCOMx Mask */
#define ADBMS6948_I2C_FCOM_MASK    ((uint8_t)0x0F)

/* I2C read byte */
#define ADBMS6948_I2C_READBYTE    ((uint8_t)0xFF)

/* I2C filler byte */
#define ADBMS6948_I2C_FILLERBYTE    ((uint8_t)0xFF)

/* I2C Write not requested */
#define ADBMS6948_I2C_WRNOTREQ    ((uint8_t)0x00)

/* I2C Write success */
#define ADBMS6948_I2C_WRSUCCESS    ((uint8_t)0x01)

/* I2C Write success */
#define ADBMS6948_I2C_WRFAILED    ((uint8_t)0x02)



/*!< All Cells selection */
#define ADBMS6948_CELL_SEL_ALL    ((uint8_t)0)

/* Module Internal Defines */
#define ZERO_UINT8    (uint8_t)0u
#define ADBMS6948_SHIFT_BY_8    ((uint8_t)8U)

/*============= D A T A T Y P E S =============*/
/**
 * Runtime error ID type.
 */
typedef uint16_t Adbms6948_RuntimeErrorIdType;

/**
 * Runtime error status type.
 */
typedef uint8_t Adbms6948_RuntimeErrorStatusType;

/* Enumerations */

/*! \enum Adbms6948_eDevChainType
    Enumeration of the possible device types in chain
*/
typedef enum
{
    ADBMS6948_DEVICE = 0x01,
    NON_ADBMS6948_DEVICE = 0x2
}Adbms6948_eDevChainType;

/*! \enum Adbms6948_ChainStateType
    Enumeration of the possible states of the ADBMS6948 Monitor daisy-chain.
*/
typedef enum
{
    ADBMS6948_ST_CHAIN_IDLE, /*!< Chain is idle */
    ADBMS6948_ST_CHAIN_BUSY  /*!< Operation in progress on the chain  */
}Adbms6948_ChainStateType;

/*! \enum Adbms6948_StateType
    Enumeration of the possible states of the ADBMS6948 driver.
*/
typedef enum
{
    ADBMS6948_ST_UNINIT, /*!< Driver module Uninitialized */
    ADBMS6948_ST_INIT   /*!< Driver module Initalized */
}Adbms6948_StateType;


/*! \enum Adbms6948_ConfigGroupSelection
 * Configuration register group selection.
 */
typedef enum
{
    ADBMS6948_CONFIG_GROUP_A,
    ADBMS6948_CONFIG_GROUP_B,
    ADBMS6948_CONFIG_GROUP_C,
    ADBMS6948_CONFIG_GROUP_D,
    ADBMS6948_CONFIG_GROUP_E,
    ADBMS6948_CONFIG_GROUP_F,
    ADBMS6948_CONFIG_GROUP_G,
    ADBMS6948_CONFIG_GROUP_H,
    ADBMS6948_CONFIG_GROUP_I,
    ADBMS6948_CONFIG_GROUP_INVALID
}Adbms6948_ConfigGroupSelection;

/*! \enum Adbms6948_ReadAllCommandType
 * "Read All" Command type selection
 */
typedef enum
{
    ADBMS6948_READALL_CELLVOLTAGES=0u,
    ADBMS6948_READALL_VOLT_CURRENT,
    ADBMS6948_READALL_CSVOLTAGES,
	ADBMS6948_READALL_CSVOLTAGE_CURRENT,
	ADBMS6948_READALL_AUX_STATUS,
	ADBMS6948_READALL_INVALID
}Adbms6948_ReadAllCommandType;

/* Structures */
/*! \struct Adbms6948_ChainStateInfoType
   Chain State Information
*/
typedef struct
{
    /*! Status for the first wakeup. */
    boolean     bFirstWakeup;

    /*! Last PEC error state for all devices in the chain. */
    boolean   aLastReadPecEr[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];

     /*! Maximum cell register groups that have necessary cell data */
    uint8_t     nMaxCellRegGrps;

    /*! Last read CC values for all devices in the chain */
    uint8_t 	 aLastReadCCs[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];

    /*! Current value of command count */
    uint8_t     nCmdCnt[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];

    /*! Current value for Number of devices in the chain. */
    uint8_t     nCurrNoOfDevices;

    /*! Current count of Errors in command count */
    uint32_t     nCmdCntError[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];

    /*! Current count of PEC Errors */
    uint32_t     nPECError[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];

    /*! Chain State */
    volatile Adbms6948_ChainStateType    eChainState;

    /*! Device type in the chain */
    Adbms6948_eDevChainType	Adbms6948_eDevChain[ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN];
	/*! Wakeup enable flag for LPCM services */
	boolean		bEnableWakeup;

	/*! LPCM driver parameter setup status */
	boolean		bLpcmDrvParamsSet;
}Adbms6948_ChainStateInfoType;

/*! \struct Adbms6948_I2CIdxData
   I2C write index data.
*/
typedef struct
{
    /*! Byte index for write buffer */
    uint16_t *    pnByteIdx;
    /*! Number of bytes written in the current transaction */
    uint8_t *    pnBytesWritten;
    /*! Number of bytes pending to be written */
    uint16_t *    pnWrLen;
}Adbms6948_I2CIdxData;

/*======= P U B L I C P R O T O T Y P E S ========*/

#endif /* ADBMS6948_TYPES_H */

/** @}*/ /* ADI_ADBMS6948_DRIVER_INTERNAL_DATA */

/*
*
* EOF
*
*/

