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
	HTS221_DEVICE_ADDRESS			=	0x5F,
	HTS221_WHO_AM_I					=	0x0F,
	HTS221_WHO_AM_I_RESPONSE		=	0xBC,
	HTS221_T0_DEGC_REG				=	0x32,
	HTS221_T1_DEGC_REG				=	0x33,
	HTS221_MSB_T1_T0				=	0x35,
	HTS221_T1_OUT					=	0x3E,
	HTS221_T1_OUT_2					=	0x3F,
	HTS221_T0_OUT					= 	0x3C,
	HTS221_T0_OUT_2					=	0x3D,
	HTS221_T_OUT					=	0x2A,
	HTS221_T_OUT_2					=	0x2B,
	HTS221_H0_RH_X2					=	0x30,
	HTS221_H0_T0_OUT				=	0x36,
	HTS221_H1_T0_OUT				=	0x3A,
	HTS221_H_OUT					=	0x28,
	HTS221_CTRL_REG1				=	0x20,
	HTS221_CTRL_REG1_SETUP			=	0x81,	// 0b1000 0001
	HTS221_STATUS_REG				=	0x27,
};

uint8_t hts221_read_byte(uint8_t reg_addr);
uint16_t HTS221_Get_Humidity();
int16_t HTS221_Get_Temperature();
void hts221_readArray(uint8_t * data, uint8_t reg, uint8_t length);
uint8_t hts221_init(void);
void hts221_write_byte(uint8_t reg_addr, uint8_t value);
