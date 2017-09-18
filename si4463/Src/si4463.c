/*
 * si4463.c
 *
 *  Created on: 30 θών. 2017 γ.
 *      Author: MINI
 */

#include <stdbool.h>
#include <stdint.h>
#include "si4463.h"


int8_t SI4463_SendCommand(si4463_t * si4463, uint8_t * cmdChain, uint16_t cmdLen)
{
	int8_t result = SI4463_NG;
	uint8_t nop[1] = {0x00};
	uint8_t tryCount = 5;
	uint8_t ctsData[cmdLen];
	memset(ctsData, 0, cmdLen);

	/* Busy-wait CTS cycle with several tries */
	while(tryCount)
	{
		if (si4463->IsClearToSend())
		{
			/* CTS is clear and we can send data */
			si4463->Select();
			si4463->WriteRead(cmdChain, ctsData, cmdLen);
			si4463->Deselect();

			result = SI4463_OK;
			break;
		}
		else
		{
			/* CTS is no clear. We wait 5 times to try send again */
			/* Try to get CTS clear by done NOP operation */
			si4463->WriteRead(nop, ctsData, 1);
			si4463->DelayMs(10);
			--tryCount;
			result = SI4463_ERR_NO_HW_CTS;
		}
	}


	return result;
}

int8_t SI4463_ReadCommandBuffer(si4463_t * si4463, uint8_t * cmdBuf, uint8_t cmdChainLen)
{
	int8_t result = SI4463_NG;
	uint8_t nop[1] = {0x00};
	uint8_t tryCount = 5;
	uint8_t cmdChain[cmdChainLen+1];
	memset(cmdChain, 0, cmdChainLen+1);
	cmdChain[0] = SI4463_CMD_READ_CMD_BUF;

	/* Busy-wait CTS cycle with several tries */
	while(tryCount)
	{
		if(si4463->IsClearToSend())
		{
			/* CTS is clear and we can send data */
			si4463->Select();
			si4463->WriteRead(cmdChain, cmdBuf, sizeof(cmdChain));
			si4463->Deselect();

			result = SI4463_OK;
			break;
		}
		else
		{
			/* CTS is no clear. We wait 5 times to try send again */
			/* Try to get CTS clear by done NOP operation */
			si4463->WriteRead(nop, cmdBuf, 1);
			si4463->DelayMs(10);
			--tryCount;
			result = SI4463_ERR_NO_HW_CTS;
		}
	}

	return result;
}

void SI4463_Init(si4463_t * si4463)
{
	uint8_t radioConfigurationDataArray[]  = RADIO_CONFIGURATION_DATA_ARRAY;
	uint8_t * currentPt = &radioConfigurationDataArray[0];

	/* Start with RESET and POWER_UP commands.
	 * By the way, POWER_UP there is into RADIO_CONFIGURATION_DATA_ARRAY*/
	SI4463_Reset(si4463);
	SI4463_PowerUp(si4463);

	/* Send all commands while pointer not equal 0x00 (0x00 presence in the end of the configuration array) */
	while(*currentPt != 0x00)
	{
		uint8_t len = *currentPt;
		uint8_t command[len];
		memset(command, 0, len);
		currentPt++;
		memcpy(command, currentPt, len);
		SI4463_SendCommand(si4463, command, len);
		currentPt += len;
		/* In the SI4463_SendCommand there is a polling of CTS signal.
		 * But without delay after applying command from configuration array
		 * invoke chip error "CMD_ERROR_COMMAND_BUSY" that means sending command
		 * before accepting previous.
		 */
		si4463->DelayMs(100);
	}
}


void SI4463_Reset(si4463_t * si4463)
{
	si4463->SetShutdown();
	si4463->DelayMs(10);
	si4463->ClearShutdown();
	si4463->DelayMs(100);

	/* Delay need for set up the chip in the default state */
	si4463->DelayMs(1000);
}

void SI4463_PowerUp(si4463_t * si4463)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[7] = {SI4463_CMD_POWER_UP,
							0x00,
							0x00,
							(RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ & 0xFF000000) >> 24,
							(RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ & 0x00FF0000) >> 16,
							(RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ & 0x0000FF00) >> 8,
							(RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ & 0x000000FF)};

	SI4463_SendCommand(si4463, cmdChain, 7);

	/* Delay need for set up the chip in the POWER_UP state */
	si4463->DelayMs(1000);
}

void SI4463_GetInterrupts(si4463_t * si4463)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[4] = {SI4463_CMD_GET_INT_STATUS, 0xFF, 0xFF, 0xFF};
	uint8_t phPend, modemPend, chipPend;

	SI4463_SendCommand(si4463, cmdChain, 4);
	SI4463_ReadCommandBuffer(si4463, answer, 9);

	/* Get pend bytes */
	phPend = answer[4];
	modemPend = answer[6];
	chipPend = answer[8];

	/* Get interrupts for structure pointer */

	/* PH pending interrupts */
	si4463->interrupts.filterMatch = ((phPend & 0x80) >> 7) & 0x01;
	si4463->interrupts.filterMiss = ((phPend & 0x40) >> 6) & 0x01;
	si4463->interrupts.packetSent = ((phPend & 0x20) >> 5) & 0x01;
	si4463->interrupts.packetRx = ((phPend & 0x10) >> 4) & 0x01;
	si4463->interrupts.crcError = ((phPend & 0x08) >> 3) & 0x01;
	// Null bit
	si4463->interrupts.txFifoAlmostEmpty = ((phPend & 0x02) >> 1) & 0x01;
	si4463->interrupts.rxFifoAlmostFull = phPend & 0x01;

	/* Modem interrupts */
	// Null bit
	si4463->interrupts.postambleDetect = ((modemPend & 0x40) >> 6) & 0x01;
	si4463->interrupts.invalidSync = ((modemPend & 0x20) >> 5) & 0x01;
	si4463->interrupts.rssiJump = ((modemPend & 0x10) >> 4) & 0x01;
	si4463->interrupts.rssi = ((modemPend & 0x08) >> 3) & 0x01;
	si4463->interrupts.invalidPreamble = ((modemPend & 0x04) >> 2) & 0x01;
	si4463->interrupts.preambleDetect = ((modemPend & 0x02) >> 1) & 0x01;
	si4463->interrupts.syncDetect = modemPend & 0x01;

	/* Chip interrupts */
	//Null bit
	si4463->interrupts.cal = ((chipPend & 0x40) >> 6) & 0x01;
	si4463->interrupts.fifoUnderflowOverflowError = ((chipPend & 0x20) >> 5) & 0x01;
	si4463->interrupts.stateChange = ((chipPend & 0x10) >> 4) & 0x01;
	si4463->interrupts.cmdError = ((chipPend & 0x08) >> 3) & 0x01;
	si4463->interrupts.chipReady = ((chipPend & 0x04) >> 2) & 0x01;
	si4463->interrupts.lowBatt = ((chipPend & 0x02) >> 1) & 0x01;
	si4463->interrupts.wut = chipPend & 0x01;
}

void SI4463_ClearInterrupts(si4463_t * si4463)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[4] = {SI4463_CMD_GET_INT_STATUS, 0x00, 0x00, 0x00};

	SI4463_SendCommand(si4463, cmdChain, 4);
	SI4463_ReadCommandBuffer(si4463, answer, 9);
}

void SI4463_ClearAllInterrupts(si4463_t * si4463)
{
	SI4463_ClearInterrupts(si4463);
	SI4463_ClearChipStatus(si4463);
}

void SI4463_GetPartInfo(si4463_t * si4463, uint8_t * pRxData)
{
	uint8_t cmdChain[1] = {SI4463_CMD_PART_INFO};

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, cmdChain, 1);
	SI4463_ReadCommandBuffer(si4463, pRxData, 9);
}

void SI4463_GetChipStatus(si4463_t * si4463)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_GET_CHIP_STATUS, 0x7F};

	SI4463_SendCommand(si4463, cmdChain, 2);
	SI4463_ReadCommandBuffer(si4463, answer, 5);

	si4463->chipStatus.cmdError = answer[4];
	si4463->chipStatus.cmdErrCmdId = answer[5];
}

void SI4463_ClearChipStatus(si4463_t * si4463)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_GET_CHIP_STATUS, 0x00};

	SI4463_SendCommand(si4463, cmdChain, 2);
	SI4463_ReadCommandBuffer(si4463, answer, 5);
}

void SI4463_GetCurrentState(si4463_t * si4463, uint8_t * state)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[1] = {SI4463_CMD_REQUEST_DEVICE_STATE};

	SI4463_SendCommand(si4463, cmdChain, 1);
	SI4463_ReadCommandBuffer(si4463, answer, 3);

	*state = answer[2];
}

void SI4463_SetCurrentState(si4463_t * si4463, uint8_t * state)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_CHANGE_STATE, *state};

	SI4463_SendCommand(si4463, cmdChain, 1);
	SI4463_ReadCommandBuffer(si4463, answer, 1);
}

void SI4463_StartRx(si4463_t * si4463, uint16_t len, bool goToRxAfterTimeout, bool goToRxAfterValid, bool goToRxAfterInvalid)
{
	uint8_t stateAfterTimeout = 0x00;
	uint8_t stateAfterValid = 0x00;
	uint8_t stateAfterInvalid = 0x00;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);

	if (goToRxAfterTimeout)
	{
		stateAfterTimeout = 0x08;
	}

	if (goToRxAfterValid)
	{
		stateAfterValid = 0x08;
	}

	if (goToRxAfterInvalid)
	{
		stateAfterInvalid = 0x08;
	}

	uint8_t cmdChain[8] = {SI4463_CMD_START_RX,
							RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER,
							0x00,
							(len & 0xFF00) >> 8,
							len & 0xFF,
							stateAfterTimeout,
							stateAfterValid,
							stateAfterInvalid};

	SI4463_SendCommand(si4463, cmdChain, 8);
	SI4463_ReadCommandBuffer(si4463, answer, 1);
}

void SI4463_StartTx(si4463_t * si4463, uint16_t len, bool goToRxAfterTx)
{
	uint8_t stateAfterTx = 0x00;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);

	if (goToRxAfterTx)
	{
		stateAfterTx = 0x80;
	}

	uint8_t cmdChain[5] = {SI4463_CMD_START_TX,
								RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER,
								stateAfterTx,
								((len & 0xFF00) >> 8) & 0x1F,
								len & 0xFF};

	SI4463_SendCommand(si4463, cmdChain, 5);
	SI4463_ReadCommandBuffer(si4463, answer, 1);
}

void SI4463_WriteTxFifo(si4463_t * si4463, uint8_t * msg, uint16_t msgLen)
{
	//TODO check what len < FIFO size
	uint8_t command[msgLen+1];
	memset(command, 0, msgLen+1);

	command[0] = SI4463_CMD_WRITE_TX_FIFO;
	memcpy(&command[1], msg, msgLen);

	SI4463_SendCommand(si4463, command, sizeof(command));
}

int8_t SI4463_ReadRxFifo(si4463_t * si4463, uint8_t * msg, uint16_t msgLen)
{
	int8_t result = SI4463_NG;
	uint8_t nop[1] = {0x00};
	uint8_t tryCount = 5;
	uint8_t cmdBuf[msgLen + 1];
	memset(cmdBuf, 0x00, msgLen + 1);

	uint8_t command[msgLen+1];
	memset(command, 0, msgLen+1);
	command[0] = SI4463_CMD_READ_RX_FIFO;

	//TODO check what len < FIFO size

	while(tryCount)
	{
		if(si4463->IsClearToSend())
		{
			/* CTS is clear and we can send data */
			si4463->Select();
			si4463->WriteRead(command, cmdBuf, sizeof(command));
			si4463->Deselect();

			/* Copy RX buffer data to out exlcude dummy byte */
			memcpy(msg, &cmdBuf[1], msgLen);

			/* Everything is OK and we can return OK code */
			result = SI4463_OK;
			break;
		}
		else
		{
			/* CTS is no clear. We wait 5 times to try send again */
			/* Try to get CTS clear by done NOP operation */
			si4463->WriteRead(nop, cmdBuf, 1);
			si4463->DelayMs(10);
			--tryCount;
			result = SI4463_ERR_NO_HW_CTS;
		}
	}

	return result;
}

uint8_t SI4463_GetTxFifoRemainBytes(si4463_t * si4463)
{
	uint8_t result = 0;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x00};

	SI4463_SendCommand(si4463, cmdChain, sizeof(cmdChain));
	SI4463_ReadCommandBuffer(si4463, answer, SI4463_CMD_BUF_LEN);

	result = answer[3];
	return result;
}

uint8_t SI4463_GetRxFifoReceivedBytes(si4463_t * si4463)
{
	uint8_t result = 0;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x00};

	SI4463_SendCommand(si4463, cmdChain, sizeof(cmdChain));
	SI4463_ReadCommandBuffer(si4463, answer, SI4463_CMD_BUF_LEN);

	result = answer[2];
	return result;
}

void SI4463_ClearRxFifo(si4463_t * si4463)
{
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
							0x02};

	SI4463_SendCommand(si4463, cmdChain, 2);
}

void SI4463_ClearTxFifo(si4463_t * si4463)
{
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x01};

	SI4463_SendCommand(si4463, cmdChain, 2);
}

void SI4463_ClearSharedFifo(si4463_t * si4463)
{
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x01 | 0x02};

	SI4463_SendCommand(si4463, cmdChain, 2);
}

void SI4463_Transmit(si4463_t * si4463, uint8_t * packet, uint8_t len)
{
	SI4463_WriteTxFifo(si4463, packet, len);
	SI4463_StartTx(si4463, len, true);
}

/***************************************************
 * Properties section
 ***************************************************/

int8_t SI4463_GetProperty(si4463_t * si4463, uint8_t group, uint8_t numProps, uint8_t startProp, uint8_t * data)
{
	int8_t result = SI4463_NG;
	uint8_t cmdChain[4];
	memset(cmdChain, 0, 4);
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0, SI4463_CMD_BUF_LEN);

	/* Adding corresponding bytes to cmdChain */
	cmdChain[0] = SI4463_CMD_GET_PROPERTY;
	cmdChain[1] = group;
	cmdChain[2] = numProps;
	cmdChain[3] = startProp;

	SI4463_SendCommand(si4463, cmdChain, 4);
	SI4463_ReadCommandBuffer(si4463, answer, SI4463_CMD_BUF_LEN);

	/* Copy answer from third byte because
	 * - first byte is 0x00 dummy byte
	 * - second byte is CTS - 0xFF in case of successful exchange
	 * - third byte - first data byte*/
	memcpy(data, answer + 2, numProps);

	/* Check what data is correctly */
 	if (answer[1] == SI4463_CTS)
	{
		result = SI4463_OK;
	}
 	else
 	{
 		result = SI4463_ERR_NO_SW_CTS;
 	}

	return result;
}

void SI4463_SetProperty(si4463_t * si4463, uint8_t group, const uint8_t numProps, uint8_t startProp, uint8_t * data)
{
	uint8_t cmdChain[4 + numProps];
	memset(cmdChain, 0, 4 + numProps);

	/* Adding corresponding bytes to cmdChain */
	cmdChain[0] = SI4463_CMD_SET_PROPERTY;
	cmdChain[1] = group;
	cmdChain[2] = numProps;
	cmdChain[3] = startProp;
	memcpy(&cmdChain[4], data, numProps);

	/* Set property */
	SI4463_SendCommand(si4463, cmdChain, 4 + numProps);
}

int8_t SI4463_SetSplitFifo(si4463_t * si4463)
{
	int8_t result = SI4463_NG;
	uint8_t buffer[1] = {0x00};
	uint8_t answer[1] = {0x00};

	/* Get current value of property for non-intrusive setup of buffer */
	SI4463_GetProperty(si4463, 0x00, 0x01, 0x03, buffer);
	/* Set FIFO_MODE to 1 for half-duplex fifo - 129 byte size buffer */
	buffer[0] &= ~0x10;
	/* Set RESERVED bit to 1 (according to datasheet) */
	buffer[0] |= 0x40;
	/* Set new property value */
	SI4463_SetProperty(si4463, 0x00, 0x01, 0x03, buffer);

	/* Verify what property is set */
	SI4463_GetProperty(si4463, 0x00, 0x01, 0x03, answer);
	if((buffer[0] & 0x10) == (answer[0] & 0x10))
	{
		result = SI4463_OK;
	}
	else
	{
		result = SI4463_NG;
	}

	/* There is some interesting moment.
	 * For activate FIFO splittng or merging need to clear FIFOs (TX and RX) after set this
	 * See more:
	 * http://community.silabs.com/t5/Interface-Knowledge-Base/Shared-FIFO-on-the-Si446x/ta-p/126802
	 */
	SI4463_ClearSharedFifo(si4463);

	return result;
}

int8_t SI4463_SetHalfDuplexFifo(si4463_t * si4463)
{
	int8_t result = SI4463_NG;
	uint8_t buffer[1] = {0x00};
	uint8_t answer[1] = {0x00};

	/* Get current value of property for non-intrusive setup of buffer */
	SI4463_GetProperty(si4463, 0x00, 0x01, 0x03, buffer);
	/* Set FIFO_MODE to 1 for half-duplex fifo - 129 byte size buffer */
	buffer[0] |= 0x10;
	/* Set RESERVED bit to 1 (according to datasheet) */
	buffer[0] |= 0x40;
	/* Set new property value */
	SI4463_SetProperty(si4463, 0x00, 0x01, 0x03, buffer);

	/* Verify what property is set */
	SI4463_GetProperty(si4463, 0x00, 0x01, 0x03, answer);
	if((buffer[0] & 0x10) == (answer[0] & 0x10))
	{
		result = SI4463_OK;
	}

	/* There is some interesting moment.
	 * For activate FIFO splittng or merging need to clear FIFOs (TX and RX) after set this
	 * See more:
	 * http://community.silabs.com/t5/Interface-Knowledge-Base/Shared-FIFO-on-the-Si446x/ta-p/126802
	 */
	SI4463_ClearSharedFifo(si4463);

	return result;
}

uint32_t SI4463_GetBytePosition(uint8_t neededByte, uint8_t * array, uint32_t arrayLen)
{
	uint32_t result = ~0L;

	return result;
}
