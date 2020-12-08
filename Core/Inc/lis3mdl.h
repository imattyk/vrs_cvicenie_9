/*
 * lis3mdl.h
 *
 *  Created on: 8 Dec 2020
 *      Author: Marek
 */

#ifndef INC_LIS3MDL_H_
#define INC_LIS3MDL_H_

#include "i2c.h"

enum LIS3MDL{

	LIS3MDL_WHO_AM_I	=	0x0F,
	LIS3MDL_CTRL_REG1	=	0x20,
	LIS3MDL_STATUS_REG	=	0x27,

	LIS3MDL_OUT_X_L		=	0x28,
	LIS3MDL_OUT_X_H		=	0x29,

	LIS3MDL_OUT_Y_L		=	0x2A,
	LIS3MDL_OUT_Y_H		=	0x2B,

	LIS3MDL_OUT_Z_L		=	0x2C,
	LIS3MDL_OUT_Z_H		=	0x2D,


};
uint8_t lis3mdl_read_byte(uint8_t reg_addr);
void lis3mdl_readArray(uint8_t * data, uint8_t reg, uint8_t length);
void lis3mdl_write_byte(uint8_t reg_addr, uint8_t value);

#endif /* INC_LIS3MDL_H_ */
