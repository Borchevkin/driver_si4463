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
	uint8_t rxData[512] = {0};

	si4463->Select();
	si4463->WriteRead(radioConfigurationDataArray, rxData, len);
	si4463->Deselect();
}

void SI4463_GetPartInfo(si4463_t * si4463, uint8_t * pRxData)
{
	uint8_t txData[1] = {SI4463_CMD_PART_INFO};

	si4463->Select();
	si4463->WriteRead(txData, pRxData, 1);
	si4463->Deselect();
}

