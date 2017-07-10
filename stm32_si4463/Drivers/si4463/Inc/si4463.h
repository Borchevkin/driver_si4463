/*
 * si4463.h
 *
 *  Created on: 30 θών. 2017 γ.
 *      Author: MINI
 */

#ifndef INC_SI4463_H_
#define INC_SI4463_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "radio_config_Si4463.h"

/* Define section */

/*Values*/
#define SI4463_CMD_BUF_LEN				(17)
#define SI4463_MAX_CMD_LEN				(16)
#define SI4463_MAX_TX_FIFO_LEN			(64)
#define SI4463_MAX_RX_FIFO_LEN			(64)

/* Commands */
#define SI4463_CMD_POWER_UP				(0x02)

#define SI4463_CMD_NOP					(0x00)
#define SI4463_CMD_PART_INFO			(0x01)
#define SI4463_CMD_FUNC_INFO			(0x10)
#define SI4463_CMD_SET_PROPERTY			(0x11)
#define SI4463_CMD_GET_PROPERTY			(0x12)
#define SI4463_CMD_GPIO_PIN_CFG			(0x13)
#define SI4463_CMD_GET_ADC_READING		(0x14)
#define SI4463_CMD_FIFO_INFO			(0x15)
#define SI4463_CMD_PACKET_INFO			(0x16)
#define SI4463_CMD_IRCAL				(0x17)
#define SI4463_CMD_PROTOCOL_CFG			(0x18)
#define SI4463_CMD_GET_INT_STATUS		(0x20)
#define SI4463_CMD_GET_PH_STATUS		(0x21)
#define SI4463_CMD_GET_MODEM_STATUS		(0x22)
#define SI4463_CMD_GET_CHIP_STATUS		(0x23)

#define SI4463_CMD_START_TX				(0x31)
#define SI4463_CMD_START_RX				(0x32)

#define SI4463_CMD_REQUEST_DEVICE_STATE	(0x33)
#define SI4463_CMD_CHANGE_STATE			(0x34)

#define SI4463_CMD_READ_CMD_BUF			(0x44)

#define SI4463_CMD_FRR_A_READ			(0x50)
#define SI4463_CMD_FRR_B_READ			(0x51)
#define SI4463_CMD_FRR_C_READ			(0x53)
#define SI4463_CMD_FRR_D_READ			(0x57)

#define SI4463_CMD_WRITE_TX_FIFO		(0x66)
#define SI4463_CMD_READ_RX_FIFO			(0x77)

#define SI4463_CMD_RX_HOP				(0x36)
/* End of commands */

/* End Define section */

/* Const section */

/* End of const section */

/* Types section */
typedef struct
{
	/* PH interrupts */
	bool filterMatch;
	bool filterMiss;
	bool packetSent;
	bool packetRx;
	bool crcError;
	bool txFifoAlmostEmpty;
	bool rxFifoAlmostFull;
	/* Modem interrupts */
	bool postambleDetect;
	bool invalidSync;
	bool rssiJump;
	bool rssi;
	bool invalidPreamble;
	bool preambleDetect;
	bool syncDetect;
	/* Chip interrupts */
	bool cal;
	bool fifoUnderflowOverflowError;
	bool stateChange;
	bool cmdError;
	bool chipReady;
	bool lowBatt;
	bool wut;
} si4463_interrupts_t;

typedef struct
{
	uint8_t cmdError;
	uint8_t cmdErrCmdId;
} si4463_chip_status_t;

typedef struct
{
	void (*WriteRead)(uint8_t * pTxData, uint8_t * pRxData, uint16_t txSize);
	void (*SetShutdown)(void);
	void (*ClearShutdown)(void);
	void (*Select)(void);
	void (*Deselect)(void);
	void (*DelayMs)(uint32_t delayMs);
	bool (*IsCTS)(void);
	si4463_interrupts_t interrupts;
	si4463_chip_status_t chipStatus;
} si4463_t;

/* End types section */

/* Prototypes section */

void SI4463_SendCommand(si4463_t * si4463, uint8_t * cmdChain, uint16_t cmdLen);
void SI4463_ReadCommandBuffer(si4463_t * si4463, uint8_t * cmdBuf, uint8_t cmdBufLen);
void SI4463_Init(si4463_t * si4463);
void SI4463_PowerUp(si4463_t * si4463);
void SI4463_Reset(si4463_t * si4463);
void SI4463_GetPartInfo(si4463_t * si4463, uint8_t * pRxData);
void SI4463_GetChipStatus(si4463_t * si4463);
void SI4463_ClearChipStatus(si4463_t * si4463);
void SI4463_GetInterrupts(si4463_t * si4463);
void SI4463_ClearInterrupts(si4463_t * si4463);
void SI4463_ClearAllInterrupts(si4463_t * si4463);
void SI4463_GetCurrentState(si4463_t * si4463, uint8_t * state);
void SI4463_SetCurrentState(si4463_t * si4463, uint8_t * state);
void SI4463_StartTx(si4463_t * si4463, bool goToRxAfterTx);
void SI4463_StartRx(si4463_t * si4463, bool goToRxAfterTimeout, bool goToRxAfterValid, bool goToRxAfterInvalid);
void SI4463_WriteTxFifo(si4463_t * si4463, uint8_t * packet, uint8_t len);
void SI4463_ReadRxFifo(si4463_t * si4463, uint8_t * packet, uint8_t len);
void SI4463_GetTxFifoBytesCount(si4463_t * si4463, uint8_t * bytesCount);
void SI4463_GetRxFifoEmptyBytes(si4463_t * si4463, uint8_t * emptyBytes);
void SI4463_ClearRxFifo(si4463_t * si4463);
void SI4463_ClearTxFifo(si4463_t * si4463);
void SI4463_Transmit(si4463_t * si4463, uint8_t * packet, uint8_t len);

/* End of prototypes section */

/* Config */


#endif /* INC_SI4463_H_ */
