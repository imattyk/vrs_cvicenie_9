/*
 * lps25hb.h
 *
 *  Created on: 5. 12. 2020
 *      Author: matej
 */
#include "main.h"

#ifndef INC_LPS25HB_H_
#define INC_LPS25HB_H_


enum LPS25HB_REG{
	LPS25HB_WHO_AM_I					= 0x0F,
	LPS25HB_WHO_AM_I_RESPONSE			= 0xBD,
	LPS25HB_ADDR						= 0xBB,
	LPS25HB_CTRL_REG1					= 0x20,
	LPS25HB_CTRL_REG1_SETUP				= 0x90,		// 0b1001 0000 (PD set to 1, ODR0 set to 1)
	LPS25HB_STATUS_REG					= 0x27,
	LPS25HB_PRESS_OUT_XL				= 0x28,
	LPS25HB_PRESS_OUT_L					= 0x29,
	LPS25HB_PRESS_OUT_H					= 0x2A,
};

#define WHO_AM_I 0x0f
#define WHO_AM_I_RESPONSE 0xbd

uint8_t lps25hb_init(void);
void lps25hb_write_byte(uint8_t reg_addr, uint8_t value);
void lps25hb_readArray(uint8_t * data, uint8_t reg, uint8_t length);
uint8_t lps25hb_read_byte(uint8_t reg_addr);
float lps25hb_getPressure();


#endif /* INC_LPS25HB_H_ */
