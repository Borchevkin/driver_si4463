/*
 * si4463.c
 *
 *  Created on: 30 θών. 2017 γ.
 *      Author: MINI
 */

#include <stdbool.h>
#include <stdint.h>
#include "si4463.h"

uint8_t SI4463_WaitCTS(const si4463_t * si4463, uint8_t times, const uint8_t delayPerTime)
{
	uint8_t result = 1;

	while(times)
	{
		if (si4463->IsClearToSend())
		{
			result = 1;
			break;
		}
		else
		{
			if(times == 1)
			{
				/* If it last try when try to send NOP command */
				SI4463_SendNop(si4463);
			}

			result = 0;
			si4463->DelayMs(delayPerTime);
			times--;
		}
	}

	return result;
}

/**
 * @brief Reset SI4463 chip by toggling SDN line. Wait CTS and send NOP.
 * @param[in] si4463 Initialized si4463 structure
 * @warning Have to invokes only before SI4463_Init()
 */
void SI4463_Reset(const si4463_t * si4463)
{
	/*
	 *  In the Si4x6x chips there is a timeout after POR built-in to make sure
	 *  if there is no host activity (SPI comms), the chip would go back to
	 *  inactive state saving energy. Inactive state is the state the chip is
	 *  sitting in after POR.
	 *  Because of this time-out, some refinement is necessary on the recommended
	 *  startup sequence of AN633, as follows:
	 *  1. Assert SDN.
	 *  2. Wait at least 10us.
	 *  3. Deassert SDN.
	 *  4. Wait at least 14ms or until GPIO1(CTS) goes  HIGH.
	 *  5. Issue the POWER_UP command over SPI (or send first line of patch if applied).
	 *  This first SPI transaction has to take less than 4ms (NSEL LOW time).
	 *  If it cannot be guaranteed, send a shorter command (e.g. NOP) first,
	 *  check CTS, then send POWER_UP or patch.
	 */
	si4463->SetShutdown();
	si4463->DelayMs(10);
	si4463->ClearShutdown();
	SI4463_WaitCTS(si4463, SI4463_TRIES, SI4463_DELAY_TRIES);

	SI4463_SendNop(si4463);
	SI4463_WaitCTS(si4463, SI4463_TRIES, SI4463_DELAY_TRIES);
}

int8_t SI4463_SendCommand(const si4463_t * si4463, const uint8_t * cmdChain, const uint16_t cmdLen)
{
	int8_t result = SI4463_OK;
	uint8_t ctsData[cmdLen];
	memset(ctsData, 0, cmdLen);

	if(SI4463_WaitCTS(si4463, SI4463_TRIES, SI4463_DELAY_TRIES))
	{
		/* CTS is clear and we can send data */
		si4463->Select();
		si4463->WriteRead(cmdChain, ctsData, cmdLen);
		si4463->Deselect();
		result = SI4463_OK;
	}
	else
	{
		result = SI4463_ERR_NO_HW_CTS;
	}

	return result;
}

int8_t SI4463_ReadCommandBuffer(const si4463_t * si4463, uint8_t * cmdBuf, const uint8_t cmdChainLen)
{
	int8_t result = SI4463_OK;
	uint8_t cmdChain[cmdChainLen+1];
	memset(cmdChain, 0, cmdChainLen+1);
	cmdChain[0] = SI4463_CMD_READ_CMD_BUF;

	if(SI4463_WaitCTS(si4463, SI4463_TRIES, SI4463_DELAY_TRIES))
	{
		/* CTS is clear and we can send data */
		si4463->Select();
		si4463->WriteRead(cmdChain, cmdBuf, sizeof(cmdChain));
		si4463->Deselect();
		result = SI4463_OK;
	}
	else
	{
		result = SI4463_ERR_NO_HW_CTS;
	}

	return result;
}

void SI4463_SendNop(const si4463_t * si4463)
{
	uint8_t nop[1] = {SI4463_CMD_NOP};
	uint8_t ctsData[1] = {0x00};

	si4463->Select();
	si4463->WriteRead(nop, ctsData, sizeof(nop));
	si4463->Deselect();
}

uint8_t SI4463_GetSwCts(const si4463_t * si4463)
{
	uint8_t result = 0;
	int8_t commResult = SI4463_OK;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);

	commResult += SI4463_ReadCommandBuffer(si4463, answer, 2);
	if((answer[1] == SI4463_BYTE_CTS))
	{
		result = SI4463_BYTE_CTS;
	}
	else
	{
		result = SI4463_BYTE_DUMMY;
	}

	return result;
}

int8_t SI4463_WaitSwCTS(const si4463_t * si4463, uint8_t times, const uint8_t delayPerTime)
{
	uint8_t result = 1;

	while(times)
	{
		if (si4463->IsClearToSend())
		{
			result = 1;
			break;
		}
		else
		{
			result = 0;
			si4463->DelayMs(delayPerTime);
			times--;
		}
	}

	return result;
}

int8_t SI4463_Init(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t radioConfigurationDataArray[]  = RADIO_CONFIGURATION_DATA_ARRAY;
	uint8_t * currentPt = &radioConfigurationDataArray[0];

	/* Start with RESET and POWER_UP commands.
	 * POWER_UP there is into RADIO_CONFIGURATION_DATA_ARRAY*/
	SI4463_Reset(si4463);

	/* Send all commands while pointer not equal 0x00 (0x00 presence in the end of the configuration array) */
	while(*currentPt != 0x00)
	{
		uint8_t len = *currentPt;
		uint8_t command[len];
		memset(command, 0, len);
		currentPt++;
		memcpy(command, currentPt, len);

		result = SI4463_SendCommand(si4463, command, len);
		if (result != SI4463_OK)
		{
			break;
		}
		currentPt += len;
		/* In the SI4463_SendCommand there is a polling of CTS signal.
		 * But without delay after applying command from configuration array
		 * invoke chip error "CMD_ERROR_COMMAND_BUSY" that means sending command
		 * before accepting previous.
		 * upd. Seems to issues may send only NOP command for delay between commands.
		 * upd2. No. Nop isnt working, back to delay scheme
		 */
		//si4463->DelayMs(100);
		if(!(SI4463_GetSwCts(si4463) == SI4463_BYTE_CTS))
		{
			result = SI4463_ERR_NO_SW_CTS;
			break;
		}
	}

	return result;
}

int8_t SI4463_VerifyInit(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t buffer[16];
	memset(buffer, 0x00, 16);

	//0x07, RF_POWER_UP,
	//TODO think about it
	memset(buffer, 0x00, 16);

	//0x08, RF_GPIO_PIN_CFG,
	//TODO think about it
	memset(buffer, 0x00, 16);

	//0x06, RF_GLOBAL_XO_TUNE_2,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x00, 0x02, 0x00, buffer);

	//0x05, RF_GLOBAL_CONFIG_1,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x00, 0x01, 0x03, buffer);

	//0x08, RF_INT_CTL_ENABLE_4,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x01, 0x04, 0x00, buffer);

	//0x08, RF_FRR_CTL_A_MODE_4,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x02, 0x04, 0x00, buffer);

	//0x0D, RF_PREAMBLE_TX_LENGTH_9,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x10, 0x09, 0x00, buffer);

	//0x09, RF_SYNC_CONFIG_5,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x11, 0x05, 0x00, buffer);

	//0x0B, RF_PKT_CRC_CONFIG_7,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x12, 0x07, 0x00, buffer); //SEEMS NO VALID

	//0x10, RF_PKT_LEN_12,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x12, 0x0C, 0x08, buffer);

	//0x10, RF_PKT_FIELD_2_CRC_CONFIG_12,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x12, 0x0C, 0x14, buffer); //SEEMS NO VALID

	//0x10, RF_PKT_FIELD_5_CRC_CONFIG_12,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x12, 0x0C, 0x20, buffer); //SEEMS NO VALID

	//0x0D, RF_PKT_RX_FIELD_3_CRC_CONFIG_9,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x12, 0x09, 0x2C, buffer);

	//0x10, RF_MODEM_MOD_TYPE_12,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x20, 0x0C, 0x00, buffer);


	//0x05, RF_MODEM_FREQ_DEV_0_1,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x12, 0x01, 0x0C, buffer);

	//0x0C, RF_MODEM_TX_RAMP_DELAY_8,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x20, 0x08, 0x18, buffer);

	//0x0D, RF_MODEM_BCR_OSR_1_9,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x20, 0x09, 0x22, buffer);

	//0x0B, RF_MODEM_AFC_GEAR_7,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x20, 0x07, 0x2C, buffer);

	//0x05, RF_MODEM_AGC_CONTROL_1,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x20, 0x01, 0x35, buffer);

	//0x0D, RF_MODEM_AGC_WINDOW_SIZE_9,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x20, 0x09, 0x38, buffer);

	//0x0D, RF_MODEM_OOK_CNT1_9,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x20, 0x09, 0x42, buffer);

	//0x05, RF_MODEM_RSSI_CONTROL_1,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x20, 0x01, 0x4C, buffer);

	//0x05, RF_MODEM_RSSI_COMP_1,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x20, 0x01, 0x4E, buffer);

	//0x05, RF_MODEM_CLKGEN_BAND_1,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x20, 0x01, 0x51, buffer);

	//0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x21, 0x0C, 0x00, buffer);

	//0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x21, 0x0C, 0x0C, buffer);

	//0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x21, 0x0C, 0x18, buffer);

	//0x08, RF_PA_MODE_4,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x22, 0x04, 0x00, buffer);

	//0x0B, RF_SYNTH_PFDCP_CPFF_7,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x23, 0x07, 0x00, buffer);

	//0x10, RF_MATCH_VALUE_1_12,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x30, 0x0C, 0x00, buffer);

	//0x0C, RF_FREQ_CONTROL_INTE_8,
	memset(buffer, 0x00, 16);
	SI4463_GetProperty(si4463, 0x40, 0x08, 0x00, buffer);

	return result;
}

int8_t SI4463_PowerUp(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[7] = {SI4463_CMD_POWER_UP,
							0x00,
							0x00,
							(RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ & 0xFF000000) >> 24,
							(RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ & 0x00FF0000) >> 16,
							(RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ & 0x0000FF00) >> 8,
							(RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ & 0x000000FF)};

	result = SI4463_SendCommand(si4463, cmdChain, 7);

	/* Delay need for set up the chip in the POWER_UP state */
	si4463->DelayMs(1000);

	return result;
}

int8_t SI4463_GetInterrupts(si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[4] = {SI4463_CMD_GET_INT_STATUS, 0xFF, 0xFF, 0xFF};
	uint8_t phPend, modemPend, chipPend;

	result += SI4463_SendCommand(si4463, cmdChain, 4);
	result += SI4463_ReadCommandBuffer(si4463, answer, 9);

	/* Get pend bytes */
	if(result == SI4463_OK && answer[1] == SI4463_BYTE_CTS)
	{
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


	return result;
}

int8_t SI4463_ClearInterrupts(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[4] = {SI4463_CMD_GET_INT_STATUS, 0x00, 0x00, 0x00};

	result += SI4463_SendCommand(si4463, cmdChain, 4);
	result += SI4463_ReadCommandBuffer(si4463, answer, 9);

	if((result == SI4463_OK) && (answer[1] == SI4463_BYTE_CTS))
	{
		result = SI4463_OK;

	}

	return result;
}

int8_t SI4463_ClearAllInterrupts(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;

	result += SI4463_ClearInterrupts(si4463);
	result += SI4463_ClearChipStatus(si4463);

	return result;
}

int8_t SI4463_GetPartInfo(const si4463_t * si4463, uint8_t * pRxData)
{
	int8_t result = SI4463_OK;
	uint8_t cmdChain[1] = {SI4463_CMD_PART_INFO};

	result += SI4463_SendCommand(si4463, cmdChain, 1);
	result += SI4463_ReadCommandBuffer(si4463, pRxData, 9);

	return result;
}

int8_t SI4463_GetChipStatus(si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_GET_CHIP_STATUS, 0x7F};

	result += SI4463_SendCommand(si4463, cmdChain, 2);
	result += SI4463_ReadCommandBuffer(si4463, answer, 5);

	if((result == SI4463_OK) && (answer[1] == SI4463_BYTE_CTS))
	{
		si4463->chipStatus.cmdError = answer[4];
		si4463->chipStatus.cmdErrCmdId = answer[5];
		result = SI4463_OK;
	}
	else
	{
		si4463->chipStatus.cmdError = 0xFF;
		si4463->chipStatus.cmdErrCmdId = 0xFF;
	}

	return result;
}

int8_t SI4463_ClearChipStatus(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_GET_CHIP_STATUS, 0x00};

	result += SI4463_SendCommand(si4463, cmdChain, 2);
	result += SI4463_ReadCommandBuffer(si4463, answer, 5);

	if((result == SI4463_OK) && (answer[1] == SI4463_BYTE_CTS))
	{
		result = SI4463_OK;
	}

	return result;
}

si4463_state_t SI4463_GetCurrentState(const si4463_t * si4463)
{
	int8_t commResult = SI4463_OK;
	si4463_state_t result = noState;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[1] = {SI4463_CMD_REQUEST_DEVICE_STATE};

	commResult += SI4463_SendCommand(si4463, cmdChain, 1);
	commResult += SI4463_ReadCommandBuffer(si4463, answer, 3);

	if((answer[1] != SI4463_BYTE_CTS) || (commResult != SI4463_OK))
	{
		result = noState;
	}
	else
	{
		result = (si4463_state_t)answer[2];
	}

	return result;
}

int8_t SI4463_SetCurrentState(const si4463_t * si4463, const si4463_state_t state)
{
	int8_t result = SI4463_OK;
	si4463_state_t newState = noState;
	uint8_t cmdChain[2] = {SI4463_CMD_CHANGE_STATE, (uint8_t)state};

	result += SI4463_SendCommand(si4463, cmdChain, sizeof(cmdChain));

	/* Check software CTS */
	if(!(SI4463_GetSwCts(si4463) == SI4463_BYTE_CTS))
	{
		result += SI4463_ERR_NO_SW_CTS;
	}

	/* Get current state */
	newState = SI4463_GetCurrentState(si4463);

	/* Check what new state is setted */
	if (state != newState)
	{
		result += SI4463_NG;
	}

	return result;
}

int8_t SI4463_StartRx(const si4463_t * si4463, const uint16_t len, const bool goToRxAfterTimeout, const bool goToRxAfterValid, const bool goToRxAfterInvalid)
{
	int8_t result = SI4463_OK;
	uint8_t stateAfterTimeout = 0x00;
	uint8_t stateAfterValid = 0x00;
	uint8_t stateAfterInvalid = 0x00;

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

	result += SI4463_SendCommand(si4463, cmdChain, sizeof(cmdChain));


	//if(SI4463_WaitCTS(si4463, SI4463_TRIES, SI4463_DELAY_TRIES))
	//{
		/* Got HW CTS signal */
		/* Try to get SW CTS signal */
		//if(!SI4463_WaitSwCTS(si4463, SI4463_TRIES, SI4463_DELAY_TRIES))
		//{
			/* Can't got SW CTS signal */
		//	result = SI4463_WARN_NO_SW_CTS_AFTER_CMD;
		//}
	//}
	//else
	//{
	//	/* Can't got HW CTS after send command */
	//	result += SI4463_WARN_NO_HW_CTS_AFTER_CMD;
	//}

	return result;
}

int8_t SI4463_StartTx(const si4463_t * si4463, const uint16_t len, const bool goToRxAfterTx)
{
	int8_t result = SI4463_OK;
	uint8_t stateAfterTx = 0x00;

	if (goToRxAfterTx)
	{
		stateAfterTx = 0x80;
	}

	uint8_t cmdChain[5] = {SI4463_CMD_START_TX,
								RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER,
								stateAfterTx,
								((len & 0xFF00) >> 8) & 0x1F,
								len & 0xFF};

	result += SI4463_SendCommand(si4463, cmdChain, 5);

	/* TODO post to the forum
	 * If no wait of CTS line polling there is possibility what
	 * for next command data will not be translated */
	if(SI4463_WaitCTS(si4463, SI4463_TRIES, SI4463_DELAY_TRIES))
	{
		/* Got HW CTS signal */
		/* Try to get SW CTS signal */
		if(!SI4463_WaitSwCTS(si4463, SI4463_TRIES, SI4463_DELAY_TRIES))
		{
			/* Can't got SW CTS signal */
			result = SI4463_WARN_NO_SW_CTS_AFTER_CMD;
		}
	}
	else
	{
		/* Can't got HW CTS after send command */
		result += SI4463_WARN_NO_HW_CTS_AFTER_CMD;
	}

	return result;
}

int8_t SI4463_WriteTxFifo(const si4463_t * si4463, const uint8_t * msg, const uint16_t msgLen)
{
	//TODO check what len < FIFO size
	int8_t result = SI4463_OK;
	uint8_t command[msgLen+1];
	memset(command, 0, msgLen+1);

	command[0] = SI4463_CMD_WRITE_TX_FIFO;
	memcpy(&command[1], msg, msgLen);

	result += SI4463_SendCommand(si4463, command, sizeof(command));

	return result;
}

int8_t SI4463_ReadRxFifo(const si4463_t * si4463, uint8_t * msg, const uint16_t msgLen)
{
	int8_t result = SI4463_OK;
	uint8_t tryCount = SI4463_TRIES;
	uint8_t cmdBuf[msgLen + 1];
	memset(cmdBuf, 0x00, msgLen + 1);

	uint8_t command[msgLen+1];
	memset(command, 0, msgLen+1);
	command[0] = SI4463_CMD_READ_RX_FIFO;

	//TODO check what len < FIFO size
	//TODO rewrite
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
			si4463->DelayMs(SI4463_DELAY_TRIES);
			--tryCount;
			result = SI4463_ERR_NO_HW_CTS;
		}
	}

	return result;
}

uint8_t SI4463_GetTxFifoRemainBytes(const si4463_t * si4463)
{
	int8_t commResult = SI4463_OK;
	uint8_t result = 0;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x00};

	commResult += SI4463_SendCommand(si4463, cmdChain, sizeof(cmdChain));
	commResult += SI4463_ReadCommandBuffer(si4463, answer, SI4463_MAX_ANSWER_LEN);

	if(commResult == SI4463_OK)
	{
		result = answer[3];
	}
	else
	{
		result = SI4463_VALUE_ERR;
	}
	return result;
}

uint8_t SI4463_GetRxFifoReceivedBytes(const si4463_t * si4463)
{
	uint8_t commResult = SI4463_OK;
	uint8_t result = 0;
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0x00, SI4463_CMD_BUF_LEN);
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x00};

	commResult += SI4463_SendCommand(si4463, cmdChain, sizeof(cmdChain));
	commResult += SI4463_ReadCommandBuffer(si4463, answer, SI4463_MAX_ANSWER_LEN);

	if(commResult == SI4463_OK)
	{
		result = answer[2];
	}
	else
	{
		result = SI4463_VALUE_ERR;
	}
	return result;
}

int8_t SI4463_ClearRxFifo(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
							0x02};

	result += SI4463_SendCommand(si4463, cmdChain, 2);

	return result;
}

int8_t SI4463_ClearTxFifo(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x01};

	result += SI4463_SendCommand(si4463, cmdChain, 2);

	return result;
}

int8_t SI4463_ClearSharedFifo(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t cmdChain[2] = {SI4463_CMD_FIFO_INFO,
								0x01 | 0x02};

	result += SI4463_SendCommand(si4463, cmdChain, 2);

	return result;
}

int8_t SI4463_Transmit(const si4463_t * si4463, const uint8_t * packet, const uint8_t len)
{
	int8_t result = SI4463_OK;
	uint8_t remainBytes = 0;

	result += SI4463_ClearAllInterrupts(si4463);

	/* Check what FIFO has enought empty bytes and what it value is valid */
	remainBytes = SI4463_GetTxFifoRemainBytes(si4463);
	if((remainBytes >= len) && (remainBytes != SI4463_VALUE_ERR))
	{
		result += SI4463_WriteTxFifo(si4463, packet, len);
		result += SI4463_StartTx(si4463, len, false);
	}
	else
	{
		result += SI4463_ERR_OVER_TX_FIFO;
	}

	return result;
}

/***************************************************
 * Properties section
 ***************************************************/

int8_t SI4463_GetProperty(const si4463_t * si4463, const uint8_t group, const uint8_t numProps, const uint8_t startProp, uint8_t * data)
{
	int8_t result = SI4463_OK;
	uint8_t cmdChain[4];
	memset(cmdChain, 0, 4);
	uint8_t answer[SI4463_CMD_BUF_LEN];
	memset(answer, 0, SI4463_CMD_BUF_LEN);

	/* Adding corresponding bytes to cmdChain */
	cmdChain[0] = SI4463_CMD_GET_PROPERTY;
	cmdChain[1] = group;
	cmdChain[2] = numProps;
	cmdChain[3] = startProp;

	result += SI4463_SendCommand(si4463, cmdChain, 4);
	result += SI4463_ReadCommandBuffer(si4463, answer, SI4463_MAX_ANSWER_LEN);

	/* Copy answer from third byte because
	 * - first byte is 0x00 dummy byte
	 * - second byte is CTS - 0xFF in case of successful exchange
	 * - third byte - first data byte*/
	memcpy(data, answer + 2, numProps);

	/* Check what data is correctly sended*/
	if(!(answer[1] == SI4463_BYTE_CTS))
	{
		result += SI4463_ERR_NO_SW_CTS;
	}

	return result;
}

int8_t SI4463_SetProperty(const si4463_t * si4463, const uint8_t group, const uint8_t numProps, const uint8_t startProp, const uint8_t * data)
{
	int8_t result = SI4463_OK;
	uint8_t cmdChain[4 + numProps];
	memset(cmdChain, 0, 4 + numProps);

	/* Adding corresponding bytes to cmdChain */
	cmdChain[0] = SI4463_CMD_SET_PROPERTY;
	cmdChain[1] = group;
	cmdChain[2] = numProps;
	cmdChain[3] = startProp;
	memcpy(&cmdChain[4], data, numProps);

	/* Set property */
	result += SI4463_SendCommand(si4463, cmdChain, 4 + numProps);

	/* Check what data is correctly sended*/
	if(!(SI4463_GetSwCts(si4463) == SI4463_BYTE_CTS))
	{
		result += SI4463_ERR_NO_SW_CTS;
	}

	return result;
}

int8_t SI4463_SetSplitFifo(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t buffer[1] = {0x00};
	uint8_t answer[1] = {0x00};

	/* Get current value of property for non-intrusive setup of buffer */
	result += SI4463_GetProperty(si4463, 0x00, 0x01, 0x03, buffer);
	/* Set FIFO_MODE to 1 for half-duplex fifo - 129 byte size buffer */
	buffer[0] &= ~0x10;
	/* Set RESERVED bit to 1 (according to datasheet) */
	buffer[0] |= 0x40;
	/* Set new property value */
	result += SI4463_SetProperty(si4463, 0x00, 0x01, 0x03, buffer);

	/* Verify what property is set */
	result += SI4463_GetProperty(si4463, 0x00, 0x01, 0x03, answer);

	if(((buffer[0] & 0x10) != (answer[0] & 0x10)) && (result != SI4463_OK))
	{
		result += SI4463_NG;
	}

	/* There is some interesting moment.
	 * For activate FIFO splitting or merging need to clear FIFOs (TX and RX) after set this
	 * See more:
	 * http://community.silabs.com/t5/Interface-Knowledge-Base/Shared-FIFO-on-the-Si446x/ta-p/126802
	 */
	result += SI4463_ClearSharedFifo(si4463);

	return result;
}

int8_t SI4463_SetHalfDuplexFifo(const si4463_t * si4463)
{
	int8_t result = SI4463_OK;
	uint8_t buffer[1] = {0x00};
	uint8_t answer[1] = {0x00};

	/* Get current value of property for non-intrusive setup of buffer */
	result += SI4463_GetProperty(si4463, 0x00, 0x01, 0x03, buffer);
	/* Set FIFO_MODE to 1 for half-duplex fifo - 129 byte size buffer */
	buffer[0] |= 0x10;
	/* Set RESERVED bit to 1 (according to datasheet) */
	buffer[0] |= 0x40;
	/* Set new property value */
	result += SI4463_SetProperty(si4463, 0x00, 0x01, 0x03, buffer);

	/* Verify what property is set */
	SI4463_GetProperty(si4463, 0x00, 0x01, 0x03, answer);
	if(((buffer[0] & 0x10) != (answer[0] & 0x10)) || (result != SI4463_OK))
	{
		result += SI4463_NG;
	}

	/* There is some interesting moment.
	 * For activate FIFO splittng or merging need to clear FIFOs (TX and RX) after set this
	 * See more:
	 * http://community.silabs.com/t5/Interface-Knowledge-Base/Shared-FIFO-on-the-Si446x/ta-p/126802
	 */
	result += SI4463_ClearSharedFifo(si4463);

	return result;
}

uint32_t SI4463_GetBytePosition(uint8_t neededByte, uint8_t * array, uint32_t arrayLen)
{
	uint32_t result = ~0L;

	return result;
}
