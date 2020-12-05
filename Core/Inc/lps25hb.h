/*
 * lps25hb.h
 *
 *  Created on: 5. 12. 2020
 *      Author: matej
 */
#include "main.h"

#ifndef INC_LPS25HB_H_
#define INC_LPS25HB_H_

#define WHO_AM_I 0x0f
#define WHO_AM_I_RESPONSE 0xbd

uint8_t lps25hb_init(void);
void lps25hb_write_byte(uint8_t reg_addr, uint8_t value);
void lps25hb_readArray(uint8_t * data, uint8_t reg, uint8_t length);
uint8_t lps25hb_read_byte(uint8_t reg_addr);
uint16_t getPressure();


#endif /* INC_LPS25HB_H_ */
