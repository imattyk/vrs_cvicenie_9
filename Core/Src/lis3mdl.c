/*
 * lis3mdl.c
 *
 *  Created on: 8 Dec 2020
 *      Author: Marek
 */

#include <lis3mdl.h>
#include "math.h"
uint8_t address = 0x3d;

uint8_t lis3mdl_read_byte(uint8_t reg_addr)
{
	uint8_t data = 0;
	return *(i2c_master_read(&data, 1, reg_addr, address, 0));
}

void lis3mdl_readArray(uint8_t * data, uint8_t reg, uint8_t length)
{
	i2c_master_read(data, length, reg, address, 1);
}


void lis3mdl_write_byte(uint8_t reg_addr, uint8_t value){
	i2c_master_write(value, reg_addr, address, 0);
}

float lis3mdl_get_mag_z()
{
	uint8_t data[6];
	int16_t xx, yy, zz;

	uint8_t temp;

	//get current scale and use it for final calculation
    temp = lis3mdl_read_byte(LIS3MDL_CTRL_REG3);

	temp = temp >> 5;
    temp &= 0x03;			//full scale bits exctracted

	lis3mdl_readArray(data, LIS3MDL_OUT_X_L, 6);

	xx = ((uint16_t)data[1]) << 8 | data[0];
	yy = ((uint16_t)data[3]) << 8 | data[2];
	zz = ((uint16_t)data[5]) << 8 | data[4];

    const float range_scale = 0.48828125; //range +-4gaus

	float x = (float)(xx*range_scale);
	float y = (float)(yy*range_scale);
	float z = (float)(zz*range_scale);

	float D;

	if(x != 0) {
		D = atan(y/x)*(180/3.14);
	}else{
		if(y<0){
			D=90;
		}else{
			D=0;
		}
	}

	if(D > 360){
		D -= 360;
	}

	if(D < 0){
		D += 360;
	}

	return D;
}

uint8_t lis3mdl_init(void)
{

	uint8_t status = 1;

	//LIS3MDL_ACC_ON;

	LL_mDelay(100);

	uint8_t val = lis3mdl_read_byte(LIS3MDL_WHO_AM_I);

	if(val == LIS3DML_WHO_AM_I_RESPONSE)
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

	uint8_t ctrl1 = LIS3MDL_CTRL1_OM_UHP | LIS3MDL_CTRL1_DO_80HZ | (1 << 7);
	lis3mdl_write_byte(LIS3MDL_CTRL_REG1, ctrl1);

	uint8_t ctrl2 = LIS3MDL_CTRL2_FS_4GAUS;
	lis3mdl_write_byte(LIS3MDL_CTRL_REG2, ctrl2);

	uint8_t ctrl3 = LIS3MDL_CTRL3_MD_CONTINUES;
	lis3mdl_write_byte(LIS3MDL_CTRL_REG3, ctrl3);

	uint8_t ctrl4 = LIS3MDL_CTRL4_OMZ_UHP;
	lis3mdl_write_byte(LIS3MDL_CTRL_REG4, ctrl4);

	uint8_t device_status = lis3mdl_read_byte(LIS3MDL_CTRL_REG1); // skontrolujeme ci device bezi, manualne cez breakpoint ak tam mame 1000001 tak bezi
	uint8_t data_status = lis3mdl_read_byte(LIS3MDL_STATUS_REG); // tu pozrieme ci senzor ma pripravene data na posielanie, humidity aj temperature, dve jednotky
	return status;
}


