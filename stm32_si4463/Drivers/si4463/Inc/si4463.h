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

/* Define section */

/* End Define section */
#define SI4463_MAX_BUFFER_LEN		(32)

/* Prototypes section */

void SI4463_Init(void);
bool SI4463_Transmit(uint8_t * msg, uint8_t len);
void SI4463_Receive(uint8_t * msg, uint8_t len);

/* End of prototypes section */


#endif /* INC_SI4463_H_ */
