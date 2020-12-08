/*
 * lis3mdl.c
 *
 *  Created on: 8 Dec 2020
 *      Author: Marek
 */

#include <lis3mdl.h>


uint8_t lis3mdl_read_byte(uint8_t reg_addr)
{
	uint8_t data = 0;
	return *(i2c_master_read(&data, 1, reg_addr, 0xbe, 0));
}

void lis3mdl_readArray(uint8_t * data, uint8_t reg, uint8_t length)
{
	i2c_master_read(data, length, reg, 0xbe, 1);
}


void lis3mdl_write_byte(uint8_t reg_addr, uint8_t value){
	i2c_master_write(value, reg_addr, 0xbe, 0);
}
