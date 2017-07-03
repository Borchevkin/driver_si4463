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

/*Commands*/
#define SI4463_CMD_PART_INFO			(0x01)
#define SI4463_CMD_READ_CMD_BUF			(0x44)
#define SI4463_CMD_GET_INT_STATUS		(0x20)

/* End Define section */

/* Const section  */

/* End of const section */

/* Types section */
typedef struct
{
	void (*WriteRead)(uint8_t * pTxData, uint8_t * pRxData, uint16_t txSize);
	void (*SetShutdown)(void);
	void (*ClearShurdown)(void);
	void (*Select)(void);
	void (*Deselect)(void);
	void (*DelayMs)(uint32_t delayMs);
} si4463_t;
/* End types section */

/* Prototypes section */

void SI4463_SendCommand(si4463_t * si4463, uint8_t * cmdChain, uint16_t cmdLen);
void SI4463_ReadCommandBuffer(si4463_t * si4463, uint8_t * cmdBuf);
void SI4463_Init(si4463_t * si4463);
void SI4463_GetPartInfo(si4463_t * si4463, uint8_t * pRxData);
void SI4463_ClearAllInterrupts(si4463_t * si4463);
void SI4463_SetRxState(si4463_t * si4463);
void SI4463_SetTxState(si4463_t * si4463);
void SI4463_Transmit(uint8_t * msg, uint8_t len);
void SI4463_Receive(uint8_t * msg, uint8_t len);

/* End of prototypes section */

/* Config */


#endif /* INC_SI4463_H_ */
