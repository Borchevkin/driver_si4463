/*
 * si4463.c
 *
 *  Created on: 30 θών. 2017 γ.
 *      Author: MINI
 */

#include "si4463.h"

void SI4463_Init(si4463_t * si4463)
{
	uint8_t radioConfigurationDataArray[]  = RADIO_CONFIGURATION_DATA_ARRAY;
	uint16_t len = sizeof(radioConfigurationDataArray);

	SI4463_SendCommand(si4463, radioConfigurationDataArray, len);
}

void SI4463_GetPartInfo(si4463_t * si4463, uint8_t * pRxData)
{
	uint8_t cmdChain[1] = {SI4463_CMD_PART_INFO};

	SI4463_SendCommand(si4463, cmdChain, 1);
	SI4463_ReadCommandBuffer(si4463, pRxData);
}

void SI4463_SendCommand(si4463_t * si4463, uint8_t * cmdChain, uint16_t cmdLen)
{
	uint8_t ctsData[cmdLen];
	memset(ctsData, 0, cmdLen * sizeof(uint8_t));

	si4463->Select();
	si4463->WriteRead(cmdChain, ctsData, cmdLen);
	si4463->Deselect();
	//TODO check cstData
}

void SI4463_ReadCommandBuffer(si4463_t * si4463, uint8_t * cmdBuf)
{
	//uint8_t cmdChain[SI4463_CMD_BUF_LEN];
	//memset(cmdChain, 0, SI4463_CMD_BUF_LEN);
	//cmdChain[0] = SI4463_CMD_READ_CMD_BUF;

	si4463->Select();
	si4463->WriteRead(SI4463_CMD_BUF_LEN, cmdBuf, 1);
	si4463->Deselect();
}

void SI4463_ClearAllInterrupts(si4463_t * si4463)
{
	uint8_t answer[17] = {0};
	uint8_t cmdChain[4] = {SI4463_CMD_GET_INT_STATUS, 0x00, 0x00, 0x00};
	SI4463_SendCommand(si4463, cmdChain, 4);
	SI4463_ReadCommandBuffer(si4463, answer);
}

void SI4463_SetRxState(si4463_t * si4463)
{

}

void SI4463_SetTxState(si4463_t * si4463)
{

}
