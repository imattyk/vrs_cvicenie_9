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

#define HSM221_DEVICE_ADDRESS 	0x5F
#define HSM221_DEVICE_WRITE		0xBF
#define WHO_AM_I			0x0F
#define WHO_AM_I_RESPONSE 0xbc
#define T0_DEGC_REG		0x32
#define T1_DEGC_REG		0x33
#define MSB_T1_T0		0x35
#define T1_OUT			0x3E
#define T1_OUT_2			0x3F
#define T0_OUT			0x3C
#define T0_OUT_2			0x3D
#define T_OUT 			0x2A
#define T_OUT_2			0x2B

uint8_t hts221_read_byte(uint8_t reg_addr);
uint16_t HTS221_Get_Humidity();
int16_t HTS221_Get_Temperature();
void hts221_readArray(uint8_t * data, uint8_t reg, uint8_t length);
uint8_t hts221_init(void);
void hts221_write_byte(uint8_t reg_addr, uint8_t value);
