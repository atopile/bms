/*!
******************************************************************************
Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc. and its
licensors. By using this software you agree to the terms of the associated
Analog Devices License Agreement.
******************************************************************************
* @file   : Adbms6948_ExecCmd.cpp
* @brief  : Adbms6948 driver low level source file, containing the Device command
            execution functions.
* @version: $Revision: 519 $
* @date   : $Date: 2020-10-13 16:59:52 +0530 (Tue, 13 Oct 2020) $
* Developed by: BMS Applications Team, Bangalore, India
*****************************************************************************/

/*! \addtogroup Adbms6948_Exec_Cmd
* @{
*/
/*============= I N C L U D E S =============*/
#include "Adbms6948_ExecCmd.h"
#include "Adbms6948_Common.h"
#include "Adbms6948_Pec.h"
/*============= D A T A =============*/

/*============ Static Function Prototypes ============*/
static void  Adbms6948_lIncCmdCntAllDev
(
const uint8_t  knChainID
);

/*============= C O D E =============*/
/* Start of code section */
/* Code section start */
ADBMS6948_DRV_CODE_START

/*
 * Function Definition section
*/

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
void  Adbms6948_Cmd_ExecCmd
(
uint16_t   		nCommand,
boolean   		bIncrementCmdCount,
const uint8_t   knChainID
)
{
    uint16_t   nCmdPec;
    uint8_t   aCmd[ADBMS6948_CMD_DATA_LEN];
    uint8_t   nDevIdx, nNoOfDevInChain;

    aCmd[0] = (uint8_t)((nCommand & 0xFF00u) >> ADBMS6948_SHIFT_BY_8);
    aCmd[1] = (uint8_t)((nCommand & 0x00FFu));
    /* Calculate the 15-bit PEC for the command bytes */
    nCmdPec = (uint16_t)Adbms6948_Pec15Calculate(&aCmd[0], 2u);
    /* Append the PEC to the command buffer */
    aCmd[2] = (uint8_t)(nCmdPec >> ADBMS6948_SHIFT_BY_8);
    aCmd[3] = (uint8_t)(nCmdPec);

    if(nCommand == ADBMS6948_CMD_STCOMM)
    {
    	/* Transmit the command buffer on the SPI bus */
    	ADI_PAL_SPIWRITE(&aCmd[0], (uint8_t)(ADBMS6948_CMD_DATA_LEN + ADBMS6948_I2C_STCOMM_BYTES), knChainID);
    }
    else
    {
    	/* Transmit the command buffer on the SPI bus */
    	ADI_PAL_SPIWRITE(&aCmd[0], ADBMS6948_CMD_DATA_LEN, knChainID);
    }

    /* Does the command increment the device command count */
    if (bIncrementCmdCount == TRUE)
    {
        /* Increment the command counter maintained by the driver for all the devices on the daisy
         * chain */
        Adbms6948_lIncCmdCntAllDev(knChainID);
    }
    /* Is the command to be sent RSTCC */
    if (nCommand == ADBMS6948_CMD_RSTCC)
    {
        /* Clear the command counter value maintained by the driver for all the devices on the
         * chain */
        nNoOfDevInChain = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
	    for (nDevIdx = 0u; nDevIdx < nNoOfDevInChain; nDevIdx++)
        {
			if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIdx])
			{
				Adbms6948_aoChainStateInfo[knChainID].nCmdCnt[nDevIdx] = 0;
			}
			Adbms6948_aoChainStateInfo[knChainID].aLastReadCCs[nDevIdx] = 0;
        }
    }
}

/*!
    @brief  This function is used to execute a clear commands on all the
    devices in the chain.

    @param  [in]  nCommand  The command code for the register group write to be executed.

    @param	[in]  nDataLen  Clear data length.

    @param  [in]  pTxBuf  The buffer containing the data to be written to the register group for all
    the devices on the chain.

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  None.
*/
void Adbms6948_Cmd_ExecuteCmdCLR
(
    uint16_t nCommand,
	uint8_t  nDataLen,
    uint8_t  *pTxBuf,
    const uint8_t	knChainID
)
{
    uint8_t aTxBuf[ADBMS6948_CMD_DATA_LEN +
                             (ADBMS6948_REG_DATA_LEN_WITH_PEC *ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
    uint8_t nDevIndex, nByteIndex, nNoOfDevices;
    uint8_t	*pDevCfgArray;
    uint16_t nCfgPec, nCmdPec;
    uint8_t nLen = 0u;

    aTxBuf[nLen++] = (uint8_t)((uint16_t)(nCommand & (uint16_t)0xFF00U) >> 8U);
    aTxBuf[nLen++] = (uint8_t)(nCommand & (uint16_t)0x00FFU);
    /* Calculate the 15-bit PEC for the command bytes */
    nCmdPec = (uint16_t)Adbms6948_Pec15Calculate(&aTxBuf[0], 2u);
    /* Append the command PEC to the transmit buffer */
    aTxBuf[nLen++] = (uint8_t)(nCmdPec >> 8U);
    aTxBuf[nLen++] = (uint8_t)(nCmdPec);

    nNoOfDevices = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;

    /* Start from last configuration data to first as first data written goes to last device */
    for (nDevIndex = nNoOfDevices; nDevIndex > 0u ; nDevIndex--)
    {
        pDevCfgArray = pTxBuf + ((nDevIndex - 1u) * (ADBMS6948_REG_GRP_LEN));
        for (nByteIndex = 0u; nByteIndex < nDataLen; nByteIndex++)
        {
            aTxBuf[nLen++] = *(pDevCfgArray + nByteIndex);
        }
        /* Calculate the 10-bit PEC for the transmit data bytes for all the devices on the chain */
        nCfgPec = (uint16_t)Adbms6948_Pec10Calculate(pDevCfgArray, FALSE, nDataLen);
        /* Append the data PEC to the transmit buffer */
        aTxBuf[nLen++]  = (uint8_t)((nCfgPec & (uint16_t)0x0300u) >> 8u);
        aTxBuf[nLen++] = (uint8_t)(nCfgPec & (uint16_t)0x00FFu);
    }

    /* Transmit the command + data buffer on the SPI bus */
    ADI_PAL_SPIWRITE(aTxBuf, nLen, knChainID);

    /* Increment the command counter for all the devices in the daisy chain */
    Adbms6948_lIncCmdCntAllDev(knChainID);

    return;
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
void  Adbms6948_Cmd_ExecuteCmdWR
(
    uint16_t  		 nCommand,
    uint8_t 		*pTxBuf,
    const uint8_t    knChainID
)
{
    uint8_t  aTxBuf[ADBMS6948_CMD_DATA_LEN +
                             (ADBMS6948_REG_DATA_LEN_WITH_PEC *ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
    uint8_t  nDevIndex, nByteIndex, nNoOfDevices;
    uint8_t *   pDevCfgArray;
    uint16_t  nCfgPec, nCmdPec;
    uint8_t  nLen = 0u;

    aTxBuf[nLen++] = (uint8_t)((uint16_t)(nCommand & (uint16_t)0xFF00U) >> 8U);
    aTxBuf[nLen++] = (uint8_t)(nCommand & (uint16_t)0x00FFU);
    /* Calculate the 15-bit PEC for the command bytes */
    nCmdPec = (uint16_t)Adbms6948_Pec15Calculate(&aTxBuf[0], 2u);
    /* Append the command PEC to the transmit buffer */
    aTxBuf[nLen++] = (uint8_t)(nCmdPec >> 8U);
    aTxBuf[nLen++] = (uint8_t)(nCmdPec);

    nNoOfDevices = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;

    /* Start from last configuration data to first as first data written goes to last device */
    for (nDevIndex = nNoOfDevices; nDevIndex > 0u ; nDevIndex--)
    {
        pDevCfgArray = pTxBuf + ((nDevIndex - 1u) * (ADBMS6948_REG_GRP_LEN));
        for (nByteIndex = 0u; nByteIndex < ADBMS6948_REG_GRP_LEN; nByteIndex++)
        {
            aTxBuf[nLen++] = *(pDevCfgArray + nByteIndex);
        }
        /* Calculate the 10-bit PEC for the transmit data bytes for all the devices on the chain */
        nCfgPec = (uint16_t)Adbms6948_Pec10Calculate(pDevCfgArray, FALSE, ADBMS6948_REG_GRP_LEN);
        /* Append the data PEC to the transmit buffer */
        aTxBuf[nLen++]  = (uint8_t)((nCfgPec & (uint16_t)0x0300u) >> 8u);
        aTxBuf[nLen++] = (uint8_t)(nCfgPec & (uint16_t)0x00FFu);
    }

    /* Transmit the command + data buffer on the SPI bus */
    ADI_PAL_SPIWRITE(aTxBuf, nLen, knChainID);

    /* Increment the command counter for all the devices in the daisy chain */
    Adbms6948_lIncCmdCntAllDev(knChainID);

    return;
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
void  Adbms6948_Cmd_ExecuteCmdRD
(
    uint16_t 			*pnCmdLst,
    uint8_t     		 pRxBuf[][ADBMS6948_MAX_FRAME_SIZE],
    uint32_t  		  	 nDataCfg,
	boolean  			 bIsPollCmd,
    const uint8_t        knChainID
)
{
    uint8_t   aTxBuf[ADBMS6948_CMD_MAX_RX_FRAMES][ADBMS6948_CMD_DATA_LEN + (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
    uint16_t  nCmdPec;
    uint16_t  nRegGroups, nGroupDataLen;
    uint16_t   nGrpIdx, nGrpDataIdx;
    uint16_t  nFrameStartIdx;

    nRegGroups = (uint16_t)(nDataCfg >> 16u);
    nGroupDataLen = (uint16_t)(((uint16_t)nDataCfg) - ADBMS6948_CMD_DATA_LEN);
    for (nGrpIdx = 0u; nGrpIdx < nRegGroups; nGrpIdx++)
    {
    	nFrameStartIdx = 0u;
    	/* Populate the command code */
    	aTxBuf[nGrpIdx][nFrameStartIdx++] = (uint8_t)((uint16_t)(pnCmdLst[nGrpIdx] & (uint16_t)0xFF00U) >> 8U);
    	aTxBuf[nGrpIdx][nFrameStartIdx++] = (uint8_t)(pnCmdLst[nGrpIdx] & (uint16_t)0x00FFU);

        /* Calculate the 15-bit PEC for the command bytes */
        nCmdPec = Adbms6948_Pec15Calculate((uint8_t*)&aTxBuf[nGrpIdx][0u], 2u);

        /* Append the PEC to the command buffer */
        aTxBuf[nGrpIdx][nFrameStartIdx++] = (uint8_t)(nCmdPec >> 8U);
        aTxBuf[nGrpIdx][nFrameStartIdx++] = (uint8_t)(nCmdPec);

        for (nGrpDataIdx = 0u; nGrpDataIdx < nGroupDataLen; nGrpDataIdx++)
        {
			aTxBuf[nGrpIdx][nFrameStartIdx++] = (uint8_t)0xFFu;
        }
    }

    /* Transmit the command buffer on the SPI bus */
    ADI_PAL_SPIWRITEREADS(aTxBuf, pRxBuf, nDataCfg, knChainID);

    /* Does the command increment the device command count */
    if (bIsPollCmd == TRUE)
    {
        /* Increment the command counter maintained by the driver for all the devices in the daisy chain */
        Adbms6948_lIncCmdCntAllDev(knChainID);
    }
return;
}

/*!
    @brief  This function is used to send the dummy bytes to the device in case of wakeup and I2C
     	 	functionality

    @param  [in]  	pBuff  	    The user buffer with dummy bytes

    @param  [in]  	nLen  		Length of the user buffer

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  None.
*/
void  Adbms6948_Cmd_SendDummyBytes
(
    uint8_t 		*pBuff,
    uint8_t  		 nLen,
    const uint8_t    knChainID
)
{
	ADI_PAL_SPIWRITE(pBuff, nLen, knChainID);
}


/*!
    @brief  This function is used to execute any generic command which has no specific input or
    configuration data.

    @param  [in]  nCommand  The command to be executed.

    @param  [in]  nPECBitToFlip  Specifies the bit position to be flipped in calculated PEC

    @param  [in]  knChainID  The daisy chain ID to perform the operation.

    @return  None.
*/
void Adbms6948_Cmd_ExecCmdWithCPECBitFlip
(
uint16_t  		nCommand,
uint8_t			nPECBitToFlip,
const uint8_t  	knChainID
)
{
    uint16_t  nCmdPec;
   	uint8_t  aCmd[ADBMS6948_CMD_DATA_LEN + ADBMS6948_I2C_STCOMM_BYTES] = {0};
   	uint8_t nBitPosToFlip;

    aCmd[0] = (uint8_t)((nCommand & 0xFF00u) >> ADBMS6948_SHIFT_BY_8);
    aCmd[1] = (uint8_t)((nCommand & 0x00FFu));
    /* Calculate the 15-bit PEC for the command bytes */
    nCmdPec = (uint16_t)Adbms6948_Pec15Calculate(&aCmd[0], 2u);

    /*Flip the Nth PEC bit */
    nBitPosToFlip = (uint8_t)(nPECBitToFlip + 1u);
    nCmdPec ^= (uint16_t)((uint16_t)0x0001u << nBitPosToFlip);

    /* Append the PEC to the command buffer */
    aCmd[2] = (uint8_t)(nCmdPec >> ADBMS6948_SHIFT_BY_8);
    aCmd[3] = (uint8_t)(nCmdPec);

	/* Transmit the command buffer on the SPI bus */
	ADI_PAL_SPIWRITE(&aCmd[0], ADBMS6948_CMD_DATA_LEN, knChainID);

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
void Adbms6948_Cmd_ExecCmdWRWithDPECBitFLip
(
    uint16_t 			nCommand,
    uint8_t				*pTxBuf,
	uint8_t				nDataPECBitToFlip,
    const uint8_t		knChainID
)
{
	uint8_t aTxBuf[ADBMS6948_CMD_DATA_LEN +
                             (ADBMS6948_REG_DATA_LEN_WITH_PEC *ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
	uint8_t nDevIndex, nByteIndex, nNoOfDevices;
	uint8_t *pDevCfgArray;
    uint16_t nCfgPec, nCmdPec;
    uint8_t nLen = 0u;

    aTxBuf[nLen++] = (uint8_t)((uint16_t)(nCommand & (uint16_t)0xFF00U) >> 8U);
    aTxBuf[nLen++] = (uint8_t)(nCommand & (uint16_t)0x00FFU);
    /* Calculate the 15-bit PEC for the command bytes */
    nCmdPec = (uint16_t)Adbms6948_Pec15Calculate(&aTxBuf[0], 2u);
    /* Append the command PEC to the transmit buffer */
    aTxBuf[nLen++] = (uint8_t)(nCmdPec >> 8U);
    aTxBuf[nLen++] = (uint8_t)(nCmdPec);

    nNoOfDevices = Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;

    /* Start from last configuration data to first as first data written goes to last device */
    for (nDevIndex = nNoOfDevices; nDevIndex > 0u ; nDevIndex--)
    {
        pDevCfgArray = pTxBuf + ((nDevIndex - 1u) * (ADBMS6948_REG_GRP_LEN));
        for (nByteIndex = 0u; nByteIndex < ADBMS6948_REG_GRP_LEN; nByteIndex++)
        {
            aTxBuf[nLen++] = *(pDevCfgArray + nByteIndex);
        }
        /* Calculate the 10-bit PEC for the transmit data bytes for all the devices on the chain */
        nCfgPec = (uint16_t)Adbms6948_Pec10Calculate(pDevCfgArray, FALSE, ADBMS6948_REG_GRP_LEN);

        /* Flip the data PEC bit */
        nCfgPec ^= (uint16_t)((uint16_t)1u << nDataPECBitToFlip);

        /* Append the data PEC to the transmit buffer */
        aTxBuf[nLen++]  = (uint8_t)((nCfgPec & (uint16_t)0x0300u) >> 8u);
        aTxBuf[nLen++] = (uint8_t)(nCfgPec & (uint16_t)0x00FFu);
    }

    /* Transmit the command + data buffer on the SPI bus */
    ADI_PAL_SPIWRITE(aTxBuf, nLen, knChainID);

    return;
}


/*
 * Local Function Definition section
 */

/*!
    @brief  This function increments the global command count for all the devices in the chain.

    @param [in]  knChainID  The Id of the daisy chain on which the operation is done.

*/
static void  Adbms6948_lIncCmdCntAllDev
(
const uint8_t  knChainID
)
{
    uint8_t   nDevIndex;
    /* Loop for all the devices */
    for (nDevIndex = 0u; nDevIndex < Adbms6948_aoChainStateInfo[knChainID].nCurrNoOfDevices;
            nDevIndex++)
    {
		if (ADBMS6948_DEVICE == Adbms6948_aoChainStateInfo[knChainID].Adbms6948_eDevChain[nDevIndex])
		{
			/* Increment the command counter */
			Adbms6948_aoChainStateInfo[knChainID].nCmdCnt[nDevIndex] =
                (Adbms6948_aoChainStateInfo[knChainID].nCmdCnt[nDevIndex] + 1u);
			if (Adbms6948_aoChainStateInfo[knChainID].nCmdCnt[nDevIndex] >
                ADBMS6948_MAX_COMMAND_COUNT_VAL)
			{
				Adbms6948_aoChainStateInfo[knChainID].nCmdCnt[nDevIndex] = 0x1u;
			}
		}
    }
}

/*!
    @brief          Executes the command to read all register values

    @param[in]      nCmd        Command to send
    @param[in]      pRxBuf      Pointer to the receive buffer
    @param[in]      nRegGrps    Number of register groups to read
    @param[in]      bIsPollCmd  Specifies poll command
    @param[in]      knChainID   The daisy chain ID to perform the operation.
 */
void  Adbms6948_Cmd_ExecuteCmdRDAll
(
    uint8_t             *pRxBuf,
    uint16_t            nCmd,
    uint8_t             nRegGrps,
    boolean             bIsPollCmd,
    const uint8_t       knChainID
)
{
    uint8_t aTxBuf[ADBMS6948_CMD_DATA_LEN];
	uint16_t  nCmdPec;
    uint16_t  nFrameStartIdx;

    nFrameStartIdx = 0u;

    /* Populate the command code */
    aTxBuf[nFrameStartIdx++] = (uint8_t)((uint16_t)(nCmd & (uint16_t)0xFF00U) >> 8U);
    aTxBuf[nFrameStartIdx++] = (uint8_t)(nCmd & (uint16_t)0x00FFU);

    /* Calculate the 15-bit PEC for the command bytes */
    nCmdPec = Adbms6948_Pec15Calculate((uint8_t*)&aTxBuf[0u], 2u);

    /* Append the PEC to the command buffer */
    aTxBuf[nFrameStartIdx++] = (uint8_t)(nCmdPec >> 8U);
    aTxBuf[nFrameStartIdx++] = (uint8_t)(nCmdPec);

    /* Transmit the command buffer on the SPI bus */
    ADI_PAL_SPIWRITEREADALL(aTxBuf, pRxBuf, nRegGrps, knChainID);

    /* Does the command increment the device command count */
    if (bIsPollCmd == TRUE)
    {
        /* Increment the command counter maintained by the driver for all the devices in the daisy chain */
        Adbms6948_lIncCmdCntAllDev(knChainID);
    }
return;
}

/* End of code section */
/* Code section stop */
ADBMS6948_DRV_CODE_STOP

/*! @}*/ /* addtogroup */

/*
*
* EOF
*
*/

