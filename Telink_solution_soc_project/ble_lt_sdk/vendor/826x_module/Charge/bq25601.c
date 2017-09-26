/*****************************************************************************
* Copyright(c) Sprocomm, 2017. All rights reserved.
*	
* TI  BQ25601 charger driver
* File: bluewhale_charger.c

* This program is free software and can be edistributed and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*	
* This Source Code Reference Design for O2MICRO OZ63810G charger access (\Reference Design\) 
* is sole for the use of PRODUCT INTEGRATION REFERENCE ONLY, and contains confidential 
* and privileged information of O2Micro International Limited. O2Micro shall have no 
* liability to any PARTY FOR THE RELIABILITY, SERVICEABILITY FOR THE RESULT OF PRODUCT 
* INTEGRATION, or results from: (i) any modification or attempted modification of the 
* Reference Design by any party, or (ii) the combination, operation or use of the 
* Reference Design with non-O2Micro Reference Design.
*****************************************************************************/
/**************************************************************
* #include section
* add #include here if any
**************************************************************/
#ifdef __BQ25601_SWCHR_SUPPORT__


#include "bq25601.h"

#define 		BQ25601_I2C_ADDR		0x6B
LOCAL	int32	s_BQ25601_IIC_Handler = 	-1;

LOCAL I2C_DEV BQ25601_i2c_cfg =
{
    3,		//Gpio 2/3
    100000, // freq is standard for FT5206
    0,      // bus
    (BQ25601_I2C_ADDR<<1),   // slave_addr<<1  0x27<<1
    1,      // reg_addr_num
    1,      // check_ack
    1       // no_stop
};

LOCAL int32 BQ25601_init_i2c_handler(void)
{
	if((s_BQ25601_IIC_Handler == -1) && 
		((s_BQ25601_IIC_Handler = I2C_HAL_Open(&BQ25601_i2c_cfg)) == -1))
	{
		return -1;
	}
	return s_BQ25601_IIC_Handler;
}

LOCAL int32 BQ25601_release_i2c_handler(void)
{
	if(s_BQ25601_IIC_Handler != -1)
	{
 		I2C_HAL_Close(s_BQ25601_IIC_Handler);
		s_BQ25601_IIC_Handler = -1;
	}
	return s_BQ25601_IIC_Handler;
}


/*return value: 0: is ok    other: is failed*/
LOCAL uint8 i2c_read_byte_data( unsigned char addr)
{
    uint8 val = 0;
	if(-1 == s_BQ25601_IIC_Handler)
    {
        BQ25601_init_i2c_handler();
    }
    I2C_HAL_Read(s_BQ25601_IIC_Handler, &addr, &val, 1);
	BQ25601_release_i2c_handler();
	return val;
}

/*return value: 0: is ok    other: is failed*/
LOCAL uint8 i2c_write_byte_data( unsigned char addr, unsigned char data){
	if(-1 == s_BQ25601_IIC_Handler)
    {
        BQ25601_init_i2c_handler();
    }
    I2C_HAL_Write(s_BQ25601_IIC_Handler, &addr, &data, 1);
	BQ25601_release_i2c_handler();
}

void bq25601_read_chip_id(void)
{
	unsigned char chip_id = 0x00;
	chip_id = i2c_read_byte_data(BQ25601_CON11);
	SCI_TRACE_LOW("bq25601:%s chip_id=%x", __FUNCTION__, chip_id);
}

/*****************************************************************************
 * Description:
 *		bluewhale_charger_init, called during system power on init.
 * Parameters:
 *		n/a
 * Return:
 *		negative errno if error happens
 *****************************************************************************/
void bq25601_charger_init(void)
{
	uint8 i;
	uint8 data = 0;
	int res = 0;
	res = BQ25601_init_i2c_handler();
	//SCI_TRACE_LOW("bq25601 init start!");
	i2c_write_byte_data(BQ25601_CON0,BQ25601_REG_VAL0);
	i2c_write_byte_data(BQ25601_CON1,BQ25601_REG_VAL1);
	i2c_write_byte_data(BQ25601_CON2,BQ25601_REG_VAL2);
	i2c_write_byte_data(BQ25601_CON3,BQ25601_REG_VAL3);
	i2c_write_byte_data(BQ25601_CON4,BQ25601_REG_VAL4);
	i2c_write_byte_data(BQ25601_CON5,BQ25601_REG_VAL5);
	i2c_write_byte_data(BQ25601_CON6,BQ25601_REG_VAL6);
	i2c_write_byte_data(BQ25601_CON7,BQ25601_REG_VAL7);
	bq25601_read_chip_id();
#if 1
	for(i=0;i<11;i++)
	{
		data = i2c_read_byte_data(i);
		SCI_TRACE_LOW("bq25601:data[%d]=0x%02x",i,data);
		data = 0x00;
	}
#endif
}

void bq25601_charger_en(BOOLEAN chr_en)
{
	if(chr_en)
	{
		SCI_TRACE_LOW("charge ON");
		i2c_write_byte_data(BQ25601_CON1,BQ25601_REG_VAL1);
	}
	else
	{
		SCI_TRACE_LOW("charge OFF");
		i2c_write_byte_data(BQ25601_CON1,0x0A);
	}
}

void iot_bat_charger_en(BOOLEAN chr_en)
{
	bq25601_charger_en(chr_en);
}

#endif

