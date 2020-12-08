/*
 * hsm221.h
 *
 *  Created on: Dec 3, 2020
 *      Author: Marek
 */

#include "main.h"
#ifndef INC_HTS221_H_
#define INC_HTS221_H_

#endif /* INC_HTS221_H_ */

enum HTS221_REG{
	HSM221_DEVICE_ADDRESS	=	0x5F,
	WHO_AM_I				=	0x0F,
	WHO_AM_I_RESPONSE		=	0xBC,
	T0_DEGC_REG				=	0x32,
	T1_DEGC_REG				=	0x33,
	MSB_T1_T0				=	0x35,
	T1_OUT					=	0x3E,
	T1_OUT_2				=	0x3F,
	T0_OUT					= 	0x3C,
	T0_OUT_2				=	0x3D,
	T_OUT					=	0x2A,
	T_OUT_2					=	0x2B,
	H0_RH_X2				=	0x30,
	H0_T0_OUT				=	0x36,
	H1_T0_OUT				=	0x3A,
	H_OUT					=	0x28,
};

uint8_t hts221_read_byte(uint8_t reg_addr);
uint16_t HTS221_Get_Humidity();
int16_t HTS221_Get_Temperature();
void hts221_readArray(uint8_t * data, uint8_t reg, uint8_t length);
uint8_t hts221_init(void);
void hts221_write_byte(uint8_t reg_addr, uint8_t value);
