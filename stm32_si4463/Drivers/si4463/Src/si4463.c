/*
 * si4463.c
 *
 *  Created on: 30 θών. 2017 γ.
 *      Author: MINI
 */

#include "si4463.h"

void SI4463_SendCommand(si4463_t * si4463, uint8_t * cmdChain, uint16_t cmdLen)
{
	uint8_t ctsData[cmdLen];
	memset(ctsData, 0, cmdLen);

	/* Wait CTS signal */
	while (!si4463->IsCTS());;

	si4463->Select();
	si4463->WriteRead(cmdChain, ctsData, cmdLen);
	si4463->Deselect();

	/* Wait CTS signal */
	while (!si4463->IsCTS());;
}

void SI4463_ReadCommandBuffer(si4463_t * si4463, uint8_t * cmdBuf, uint8_t cmdChainLen)
{
	uint8_t cmdChain[cmdChainLen+1];
	memset(cmdChain, 0, cmdChainLen+1);
	cmdChain[0] = SI4463_CMD_READ_CMD_BUF;

	/* Wait CTS signal */
	while (!si4463->IsCTS());;

	si4463->Select();
	si4463->WriteRead(cmdChain, cmdBuf, sizeof(cmdChain));
	si4463->Deselect();

	/* Wait CTS signal */
	while (!si4463->IsCTS());;
}

void SI4463_Init(si4463_t * si4463)
{
	uint8_t radioConfigurationDataArray[]  = RADIO_CONFIGURATION_DATA_ARRAY;
	uint8_t * currentPt = &radioConfigurationDataArray[0];

	/* Start with RESET and POWER_UP commands.
	 * By the way, POWER_UP there is into RADIO_CONFIGURATION_DATA_ARRAY*/
	SI4463_Reset(si4463);
	SI4463_PowerUp(si4463);

	//Send all commands while pointer not equal 0x00 (0x00 presence in the end of the configuration array)
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
	/* Clear all interrupts invoked during configuration */
	//SI4463_ClearAllInterrupts(si4463);
}


void SI4463_Reset(si4463_t * si4463)
{
	si4463->SetShutdown();
	si4463->DelayMs(10);
	si4463->ClearShutdown();
	si4463->DelayMs(100);

	/* Wait CTS signal */
	while (!si4463->IsCTS());;

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

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, cmdChain, 7);

	/* Wait CTS signal */
	while (!si4463->IsCTS());;

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
	SI4463_GetChipStatus(si4463);
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
	uint8_t cmdChain[2] = {SI4463_CMD_GET_CHIP_STATUS, 0xFF};

	SI4463_SendCommand(si4463, cmdChain, 1);
	SI4463_ReadCommandBuffer(si4463, answer, 5);

	si4463->chipStatus.cmdError = answer[4];
	si4463->chipStatus.cmdErrCmdId = answer[5];
}

void SI4463_ClearChipStatus(si4463_t * si4463)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_GET_CHIP_STATUS, 0x00};

	SI4463_SendCommand(si4463, cmdChain, 1);
	SI4463_ReadCommandBuffer(si4463, answer, 5);
}

void SI4463_GetCurrentState(si4463_t * si4463, uint8_t * state)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[1] = {SI4463_CMD_REQUEST_DEVICE_STATE};

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, cmdChain, 1);
	SI4463_ReadCommandBuffer(si4463, answer, 3);

	*state = answer[2];
}

void SI4463_SetCurrentState(si4463_t * si4463, uint8_t * state)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_CHANGE_STATE, *state};

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, cmdChain, 1);
	SI4463_ReadCommandBuffer(si4463, answer, 1);
}

void SI4463_StartRx(si4463_t * si4463, bool goToRxAfterTimeout, bool goToRxAfterValid, bool goToRxAfterInvalid)
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
							(RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH & 0xFF00) >> 8,
							RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH & 0xFF,
							stateAfterTimeout,
							stateAfterValid,
							stateAfterInvalid};

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, cmdChain, 8);
	SI4463_ReadCommandBuffer(si4463, answer, 1);
}

void SI4463_StartTx(si4463_t * si4463, bool goToRxAfterTx)
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
								((RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH & 0xFF00) >> 8) & 0x1F,
								RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH & 0xFF};

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, cmdChain, 5);
	SI4463_ReadCommandBuffer(si4463, answer, 1);
}

void SI4463_WriteTxFifo(si4463_t * si4463, uint8_t * msg, uint8_t msgLen)
{
	//TODO check what len < FIFO size
	uint8_t command[msgLen+1];
	memset(command, 0, msgLen+1);

	command[0] = SI4463_CMD_WRITE_TX_FIFO;
	memcpy(&command[1], msg, msgLen);

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, command, sizeof(command));
}

void SI4463_ReadRxFifo(si4463_t * si4463, uint8_t * msg, uint8_t msgLen)
{
	uint8_t cmdBuf[SI4463_MAX_RX_FIFO_LEN + 1];
	memset(cmdBuf, 0x00, SI4463_MAX_RX_FIFO_LEN + 1);
	//TODO check what len < FIFO size
	uint8_t command[msgLen+1];
	memset(command, 0, msgLen+1);
	command[0] = SI4463_CMD_READ_RX_FIFO;

	while (!si4463->IsCTS());;

	si4463->Select();
	si4463->WriteRead(command, cmdBuf, sizeof(command));
	si4463->Deselect();

	memcpy(msg, &cmdBuf[1], msgLen);

	while (!si4463->IsCTS());;
}

void SI4463_GetTxFifoBytesCount(si4463_t * si4463, uint8_t * bytesCount)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x00};

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, cmdChain, 2);
	SI4463_ReadCommandBuffer(si4463, answer, 1);

	*bytesCount = answer[3];
}

void SI4463_GetRxFifoEmptyBytes(si4463_t * si4463, uint8_t * emptyBytes)
{
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x00};

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, cmdChain, 2);
	SI4463_ReadCommandBuffer(si4463, answer, 1);

	*emptyBytes = answer[2];
}

void SI4463_ClearRxFifo(si4463_t * si4463)
{
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
							0x02};

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, cmdChain, 2);
}

void SI4463_ClearTxFifo(si4463_t * si4463)
{
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x02};

	//SI4463_ClearAllInterrupts(si4463);
	SI4463_SendCommand(si4463, cmdChain, 2);
}

void SI4463_Transmit(si4463_t * si4463, uint8_t * packet, uint8_t len)
{
	SI4463_WriteTxFifo(si4463, packet, len);
	SI4463_StartTx(si4463, true);
}
