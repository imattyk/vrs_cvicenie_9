/*
 * lps25hb.c
 *
 *  Created on: 5. 12. 2020
 *      Author: matej
 */


#include <lps25hb.h>
#include "i2c.h"

uint8_t adress = 0xbb;

uint8_t lps25hb_read_byte(uint8_t reg_addr)
{
	uint8_t data = 0;
	return *(i2c_master_read(&data, 1, reg_addr, adress, 0));
}

void lps25hb_readArray(uint8_t * data, uint8_t reg, uint8_t length)
{
	i2c_master_read(data, length, reg, adress, 1);
}

void lps25hb_write_byte(uint8_t reg_addr, uint8_t value)
{
	i2c_master_write(value, reg_addr, adress, 0);
}

uint8_t lps25hb_init(void)
{

	uint8_t status = 1;

	//LIS3MDL_ACC_ON;

	LL_mDelay(100);

	uint8_t val = lps25hb_read_byte(WHO_AM_I);

	if(val == WHO_AM_I_RESPONSE)
	{
		status = 1;
	}
	else			//if the device is not found on one address, try another one
	{
		status = 0;
	}

	//acc device init

	lps25hb_write_byte(0x20, 0x90);
	uint8_t device_status = lps25hb_read_byte(0x20);
	uint8_t data_status = lps25hb_read_byte(0x27);

	return status;
}

uint16_t getPressure(){
	uint8_t press_out_h = 0;
	uint8_t press_out_l = 0;
	uint8_t press_out_xl = 0;

	press_out_h = lps25hb_read_byte(0x2A);
	press_out_l = lps25hb_read_byte(0x29);
	press_out_xl = lps25hb_read_byte(0x28);

	uint32_t temp = press_out_h << 16 | press_out_l << 8 | press_out_xl;
	uint16_t pressure = temp/4096;
	return pressure;
}


