/*
 * si4463.h
 *
 *  Created on: 30 ���. 2017 �.
 *      Author: MINI
 */

#ifndef INC_SI4463_H_
#define INC_SI4463_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "radio_config_Si4463.h"

/* Define section */

/* Default delays and tries */
#define SI4463_DELAY_RESET				(1000)
#define SI4463_DELAY_TRIES				(10)
#define SI4463_TRIES					(200)

/* Return codes */
#define SI4463_OK						(0)
#define SI4463_NG						(-1)
#define SI4463_ERR_DEV_NULL				(-10)
#define SI4463_ERR_NO_DEV_ANSWER		(-20)
#define SI4463_ERR_NO_HW_CTS			(-30)
#define SI4463_ERR_NO_SW_CTS			(-40)
#define SI4463_ERR_OVER_TX_FIFO 		(-50)
#define SI4463_WARN_NO_HW_CTS_AFTER_CMD	(-60)
#define SI4463_WARN_NO_SW_CTS_AFTER_CMD	(-70)

/* Values for resulting functions */
#define SI4463_VALUE_ERR				(0xFF)

/* SPI bytes */
#define SI4463_BYTE_CTS					(0xFF)
#define SI4463_BYTE_DUMMY				(0x00)

/*Values*/
#define SI4463_CMD_BUF_LEN				(17)
#define SI4463_MAX_CMD_LEN				(16)
#define SI4463_MAX_ANSWER_LEN			(16)
#define SI4463_MAX_SET_PROPS			(12)
#define SI4463_MAX_TX_FIFO_LEN			(64)
#define SI4463_MAX_RX_FIFO_LEN			(64)
#define SI4463_MAX_HALF_DUPLEX_FIFO_LEN	(129)

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
#define SI4463_CMD_IRCAL_MANUAL			(0x1A)
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

/* Properties */
#define SI4463_PROP_GLOBAL				(0x00)
#define SI4463_PROP_INT_CTL				(0x01)
#define SI4463_PROP_FRR_CTL				(0x02)
#define SI4463_PROP_PREAMBLE			(0x10)
#define SI4463_PROP_SYNC				(0x11)
#define SI4463_PROP_PKT					(0x12)
#define SI4463_PROP_MODEM				(0x20)
#define SI4463_PROP_MODEM_CHFLT			(0x21)
#define SI4463_PROP_PA					(0x22)
#define SI4463_PROP_SYNTH				(0x23)
#define SI4463_PROP_MATCH				(0x24)
#define SI4463_PROP_FREQ_CONTROL		(0x40)
#define SI4463_PROP_RX_HOP				(0x50)
/* End of properties */

/* End Define section */

/* Const section */

/* End of const section */

/* Types section */
typedef enum
{
	cmdErrorNone = 0,
	cmdErrorBadCommand = 16,
	cmdErrorBadArg = 17,
	cmdErrorErrorCommandBusy = 18,
	cmdErrorBadBootMode = 49,
	cmdErrorBadProperty = 64
} si4463_cmd_err_status_t;

typedef enum
{
	noState,
	sleepState,
	spiActiveState,
	readyState,
	ready2State,
	txTuneState,
	rxTuneState,
	txState,
	rxState
} si4463_state_t;

typedef enum
{
	global = 0x00,
	intClt = 0x01,
	frrCtl = 0x02,
	preamble = 0x10,
	sync = 0x11,
	pkt = 0x12,
	modem = 0x20,
	modemChflt = 0x21,
	pa = 0x22,
	synth = 0x23,
	match = 0x30,
	freqControl = 0x40,
	rxHop = 0x50
} si4463_prop_group_t;

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
	bool filter_match_pend; // If set, FILTER_MATCH interrupt is pending.
	bool filter_miss_pend;  // If set, FILTER_MISS interrupt is pending.
	bool packet_sent_pend; //If set, PACKET_SENT interrupt is pending.
	bool packet_rx_pend; //If set, PACKET_RX interrupt is pending.
	bool crc_error_pend; //If set, CRC_ERROR interrupt is pending.
	bool tx_fifo_almost_empty_pend; //If set, TX_FIFO_ALMOST_EMPTY interrupt is pending.
	bool rx_fifo_almost_full_pend; //If set, RX_FIFO_ALMOST_FULL interrupt is pending.

	bool filter_match; //If set, incoming packet matched filter.
	bool filter_miss; //If set, incoming packet was discarded because filter did not match
	bool packet_sent; //If set, Packet Sent
	bool packet_rx; //If set, Packet Received
	bool crc_error; //If set, CRC-32 error
	bool tx_fifo_almost_empty; //If set, TX fifo is below watermark
	bool rx_fifo_almost_full; //If set, RX fifo is above watermark
}si4463_ph_status_t;

typedef struct
{
	si4463_cmd_err_status_t cmdError;
	uint8_t cmdErrCmdId;
} si4463_chip_status_t;

typedef struct
{
	void (*WriteRead)(const uint8_t * pTxData, uint8_t * pRxData, const uint16_t txSize);
	void (*SetShutdown)(void);
	void (*ClearShutdown)(void);
	void (*Select)(void);
	void (*Deselect)(void);
	void (*DelayMs)(uint32_t delayMs);
	uint8_t (*IsClearToSend)(void);
	si4463_interrupts_t interrupts;
	si4463_chip_status_t chipStatus;
	si4463_ph_status_t phStatus;
} si4463_t;

/* End types section */

/* Prototypes section */

uint8_t SI4463_WaitCTS(const si4463_t * si4463, uint8_t times, const uint8_t delayPerTime);
void SI4463_Reset(const si4463_t * si4463);
int8_t SI4463_SendCommand(const si4463_t * si4463, const uint8_t * cmdChain, const uint16_t cmdLen);
int8_t SI4463_ReadCommandBuffer(const si4463_t * si4463, uint8_t * cmdBuf, const uint8_t cmdBufLen);
void SI4463_SendNop(const si4463_t * si4463);
uint8_t SI4463_GetSwCts(const si4463_t * si4463);
int8_t SI4463_WaitSwCTS(const si4463_t * si4463, uint8_t times, const uint8_t delayPerTime);
int8_t SI4463_Init(const si4463_t * si4463);
int8_t SI4463_VerifyInit(const si4463_t * si4463);
int8_t SI4463_PowerUp(const si4463_t * si4463);
int8_t SI4463_GetPartInfo(const si4463_t * si4463, uint8_t * pRxData);
int8_t SI4463_GetChipStatus(si4463_t * si4463);
int8_t SI4463_ClearChipStatus(const si4463_t * si4463);
int8_t SI4463_GetInterrupts(si4463_t * si4463);
int8_t SI4463_Get_PH_Status(si4463_t *si4463);
int8_t SI4463_ClearInterrupts(const si4463_t * si4463);
int8_t SI4463_ClearAllInterrupts(const si4463_t * si4463);
si4463_state_t SI4463_GetCurrentState(const si4463_t * si4463);
int8_t SI4463_SetCurrentState(const si4463_t * si4463, const si4463_state_t state);
int8_t SI4463_WriteTxFifo(const si4463_t * si4463, const uint8_t * msg, const uint16_t msgLen);
int8_t SI4463_ReadRxFifo(const si4463_t * si4463, uint8_t * msg, const uint16_t msgLen);
uint8_t SI4463_GetTxFifoRemainBytes(const si4463_t * si4463);
uint8_t SI4463_GetRxFifoReceivedBytes(const si4463_t * si4463);
int8_t SI4463_ClearRxFifo(const si4463_t * si4463);
int8_t SI4463_ClearTxFifo(const si4463_t * si4463);
int8_t SI4463_ClearSharedFifo(const si4463_t * si4463);
int8_t SI4463_Transmit(const si4463_t * si4463, const uint8_t * packet, const uint8_t len);
int8_t SI4463_StartTx(const si4463_t * si4463, const uint16_t len, const bool goToRxAfterTx);
int8_t SI4463_StartRx(const si4463_t * si4463, const uint16_t len, const bool goToRxAfterTimeout, const bool goToRxAfterValid, const bool goToRxAfterInvalid);
int8_t SI4463_GetProperty(const si4463_t * si4463, const uint8_t group, const uint8_t numProps, const uint8_t startProp, uint8_t * data);
int8_t SI4463_SetProperty(const si4463_t * si4463, const uint8_t group, const uint8_t numProps, const uint8_t startProp, const uint8_t * data);
int8_t SI4463_SetSplitFifo(const si4463_t * si4463);
int8_t SI4463_SetHalfDuplexFifo(const si4463_t * si4463);

/* Utils */
uint32_t SI4463_GetBytePosition(uint8_t neededByte, uint8_t * array, uint32_t arrayLen);

/* End of prototypes section */

/* Config */


#endif /* INC_SI4463_H_ */
