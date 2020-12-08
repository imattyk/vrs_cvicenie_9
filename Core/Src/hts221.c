/*
 * hsm221.c
 *
 *  Created on: Dec 3, 2020
 *      Author: Marek
 */


#include <hts221.h>
#include "i2c.h"

uint8_t hts221_read_byte(uint8_t reg_addr)
{
	uint8_t data = 0;
	return *(i2c_master_read(&data, 1, reg_addr, 0xbe, 0));
}

void hts221_readArray(uint8_t * data, uint8_t reg, uint8_t length)
{
	i2c_master_read(data, length, reg, 0xbe, 1);
}

void hts221_write_byte(uint8_t reg_addr, uint8_t value)
{
	i2c_master_write(value, reg_addr, 0xbe, 0);
}


uint16_t HTS221_Get_Humidity()
{
	int16_t H0_T0_out, H1_T0_out, H_T_out;
	int16_t H0_rh, H1_rh;
	uint8_t buffer[2];
	int32_t tmp;
	uint16_t value = 0;

	/* 1. Read H0_rH and H1_rH coefficients*/
	hts221_readArray(buffer, H0_RH_X2, 2);
	H0_rh = buffer[0]>>1;
	H1_rh = buffer[1]>>1;

	/*2. Read H0_T0_OUT */

	hts221_readArray(buffer, H0_T0_OUT, 2);
	H0_T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	/*3. Read H1_T0_OUT */

	hts221_readArray(buffer, H1_T0_OUT, 2);
	H1_T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	/*4. Read H_T_OUT */

	hts221_readArray(buffer, H_OUT, 2);

	H_T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
	/*5. Compute the RH [%] value by linear interpolation */
	tmp = ((int32_t)(H_T_out - H0_T0_out)) * ((int32_t)(H1_rh - H0_rh)*10);
	value = (tmp/(H1_T0_out - H0_T0_out) + H0_rh*10);
	/* Saturation condition*/
	if(value>1000) value = 1000;
		return value;
}

int16_t HTS221_Get_Temperature()
{
	 int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
	 int16_t T0_degC, T1_degC;
	 uint8_t buffer[4], tmp;
	 int32_t tmp32;
	 int16_t value;

	/*1. Read from 0x32 & 0x33 registers the value of coefficients T0_degC_x8 and T1_degC_x8*/
	 hts221_readArray(buffer, T0_DEGC_REG, 2);

	/*2. Read from 0x35 register the value of the MSB bits of T1_degC and T0_degC */
	 tmp = hts221_read_byte(MSB_T1_T0);

	/*Calculate the T0_degC and T1_degC values*/
	 T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
	 T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);
	 T0_degC = T0_degC_x8_u16>>3;
	 T1_degC = T1_degC_x8_u16>>3;

	/*3. Read from 0x3C & 0x3D registers the value of T0_OUT*/
	/*4. Read from 0x3E & 0x3F registers the value of T1_OUT*/
	 hts221_readArray(buffer, T0_OUT, 4);

	 T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
	 T1_out = (((uint16_t)buffer[3])<<8) | (uint16_t)buffer[2];

	/* 5.Read from 0x2A & 0x2B registers the value T_OUT (ADC_OUT).*/
	 hts221_readArray(buffer, T_OUT, 2);

	 T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
	/* 6. Compute the Temperature value by linear interpolation*/
	 tmp32 = ((int32_t)(T_out - T0_out)) * ((int32_t)(T1_degC - T0_degC)*10);
	 value = tmp32 /(T1_out - T0_out) + T0_degC*10;
	 return value;
}

uint8_t hts221_init(void)
{

	uint8_t status = 1;

	//LIS3MDL_ACC_ON;

	LL_mDelay(100);

	uint8_t val = hts221_read_byte(WHO_AM_I);

	if(val == WHO_AM_I_RESPONSE)
	{
		status = 1;
	}
	else			//if the device is not found on one address, try another one
	{
		status = 0;
	}

	//acc device init

	//uint8_t ctrl1 = 8 << 4; // +-2g res
	//hts221_write_byte(, ctrl1);

	hts221_write_byte(0x20, 0x81); // zapneme senzor a nastavime refresh rate senzora
	uint8_t device_status = hts221_read_byte(0x20); // skontrolujeme ci device bezi, manualne cez breakpoint ak tam mame 1000001 tak bezi
	uint8_t data_status = hts221_read_byte(0x27); // tu pozrieme ci senzor ma pripravene data na posielanie, humidity aj temperature, dve jednotky
	return status;
}


