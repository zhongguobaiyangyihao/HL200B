/*******************************************************************************
 * Copyright (c) 2015, mCube, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of Bosch Sensortec GmbH nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*******************************************************************************
 * REVISON HISTORY
 -------------------------------------------------------------------------------
 * VERSION      DATE            WHO        DESCRIPTION
 -------------------------------------------------------------------------------
 * 1.3.0        08/27/2015      Louis      Initial version.
********************************************************************************/

#ifdef __MC3416_H__
/*============================================================================
INCLUDE FILES
============================================================================*/
#include "drv_mc3416.h"
#include "../../proj/tl_common.h"
#include "../../proj/drivers/i2c.h"
#include "../app_config.h"
#include "../../vendor/common/myprintf.h"
/*============================================================================
FUNCTION DEFINITIONS
============================================================================*/
typedef enum
{
	GSENSOR_SUCCESS = 0,
	GSENSOR_UNINIT = -1,
	GSENSOR_ID_MISMATCH = -2
}err_code_t;
/******************************************
*******************************************/
#define 		MC3416_I2C_ADDR   			0x6C//(0xD8>>1)
u8              is_gsensor_init = 0;
extern device_state_t           device_state;
/*******************************************
hardware i2c interface init
*******************************************/
static void gsensor_i2c_init(void)
{
	i2c_pin_init(I2C_GPIO_GROUP_A3A4);
	i2c_master_init_div(MC3416_I2C_ADDR,0x14);//para1:ID;para2:DivClock,i2c clock = system_clock/4*DivClock
	i2c_master_init_khz(MC3416_I2C_ADDR, 200);//200Khz
	is_gsensor_init = 1;
}
/*******************************************
读取mc3416寄存器数据
*******************************************/
static void mc3416_register_read(unsigned char addr, unsigned char *data)
{
	u8 reg_addr = addr;
	*data =	i2c_read_byte(&reg_addr, 1);
}
/*******************************************
配置mc3416寄存器数据
*******************************************/
static void mc3416_register_write(unsigned char addr, unsigned char data)
{
	u8 reg_addr = addr;
	i2c_write_byte(&reg_addr, 1, data);
}
/*******************************************
gsensor id 查询
*******************************************/
s8 gsensor_id_match(void)
{
	u8 chip_id = 0;
	if(!is_gsensor_init)
	{
		return GSENSOR_UNINIT;
	}
	mc3416_register_read(MC34XX_REG_CHIPID,&chip_id);
	printf("Gsensor chip id:0x%x.\r\n",chip_id);
	if(chip_id != MC3416_CHIP_ID)
	{
		return GSENSOR_ID_MISMATCH;
	}
	return GSENSOR_SUCCESS;
}
/*******************************************
gsensor init function
1: lock;
2: unlock.
*******************************************/
s8 gsensor_init(void)
{
	s8 err_code = GSENSOR_SUCCESS;
	gsensor_i2c_init();
	err_code = gsensor_id_match();
    if(err_code != GSENSOR_SUCCESS)
    {
    	return err_code;
    }
    if(device_state.lock_onoff_state == lock_onoff_state_off)//关锁状态下
    {
    	mc3416_register_write(MC34XX_REG_SAMPLE_RATE,0x00); 	    //	SAMPLE_RATE:128hz
    	mc3416_register_write(MC34XX_REG_LPF_RANGE_RES,0x01); 	    //	range:+/-2g
    	mc3416_register_write(MC34X6_REG_AM_THRESHOLD_LSB,0x33); 	//  threshold:0x333=~50mg  (0.061mg per)
    	mc3416_register_write(MC34X6_REG_AM_THRESHOLD_MSB,0x03);
    	mc3416_register_write(MC34X6_REG_AM_DB,50);                 //  bigger and easier to be triggered
    	//Automatically clear pending interrupts if the interrupt condition is no longer valid
    	mc3416_register_write(MC34XX_REG_INTERRUPT_ENABLE,0x44);    //  AnyMotion interrupt is enabled
    	mc3416_register_write(MC34X6_REG_MOTION_CTRL,0x04);         //  AnyMotion feature is enabled
    	mc3416_register_write(MC34X6_REG_TIMER_CTRL,0xA0);          //  The temporary latch feature is enabled;temporary latch on the TEST_INT pin:800ms
    }
    else if(device_state.lock_onoff_state == lock_onoff_state_on)//开锁状态下
    {
    	mc3416_register_write(MC34XX_REG_SAMPLE_RATE,0x00); 	    //	SAMPLE_RATE:128hz
		mc3416_register_write(MC34XX_REG_LPF_RANGE_RES,0x31); 	    //	range:+/-16g
		mc3416_register_write(MC34X6_REG_AM_THRESHOLD_LSB,0x02); 	//  threshold:0x1402=~2500mg  (0.488mg per)
		mc3416_register_write(MC34X6_REG_AM_THRESHOLD_MSB,0x14);
		mc3416_register_write(MC34X6_REG_AM_DB,50);                 //  bigger and easier to be triggered
		//Automatically clear pending interrupts if the interrupt condition is no longer valid
		mc3416_register_write(MC34XX_REG_INTERRUPT_ENABLE,0x44);    //  AnyMotion interrupt is enabled
		mc3416_register_write(MC34X6_REG_MOTION_CTRL,0x04);         //  AnyMotion feature is enabled
		mc3416_register_write(MC34X6_REG_TIMER_CTRL,0xA0);          //  The temporary latch feature is enabled;temporary latch on the TEST_INT pin:800ms
    }
	return GSENSOR_SUCCESS;
}
/*******************************************
gsensor enter wakeup mode
*******************************************/
void gsensor_pwr_on(void)
{
	u8 tmp_data;
	mc3416_register_write(MC34XX_REG_MODE_FEATURE,(MC34XX_INT_IPP_OPEN_DRAIN|MC34XX_MODE_WAKE));
	mc3416_register_read(MC34XX_REG_MODE_FEATURE, &tmp_data);
	printf("Gsensor MC34XX_REG_MODE_FEATURE = 0x%x.\r\n",tmp_data);
}
/*******************************************
gsensor enter standyby mode
*******************************************/
void gsensor_pwr_off(void)
{
	u8 tmp_data;
	mc3416_register_write(MC34XX_REG_MODE_FEATURE,(MC34XX_INT_IPP_OPEN_DRAIN|MC34XX_MODE_SLEEP));
	mc3416_register_read(MC34XX_REG_MODE_FEATURE, &tmp_data);
	printf("Gsensor MC34XX_REG_MODE_FEATURE = 0x%x.\r\n",tmp_data);
}
/*******************************************
查询加速度三轴数据
*******************************************/
void gsensor_inquiry_3axis_data(s16 *x, s16 *y, s16 *z)
{
	u8	tmp_data[6] = {0};
	mc3416_register_read(MC34XX_REG_XOUT_EX_L, &tmp_data[0]);
	mc3416_register_read(MC34XX_REG_XOUT_EX_H, &tmp_data[1]);
	mc3416_register_read(MC34XX_REG_YOUT_EX_L, &tmp_data[2]);
	mc3416_register_read(MC34XX_REG_YOUT_EX_H, &tmp_data[3]);
	mc3416_register_read(MC34XX_REG_ZOUT_EX_L, &tmp_data[4]);
	mc3416_register_read(MC34XX_REG_ZOUT_EX_H, &tmp_data[5]);
    *x = ((s16)(((tmp_data[1]&0x00FF) << 8) | (tmp_data[0]&0x00FF)));
    *y = ((s16)(((tmp_data[3]&0x00FF) << 8) | (tmp_data[2]&0x00FF)));
    *z = ((s16)(((tmp_data[5]&0x00FF) << 8) | (tmp_data[4]&0x00FF)));
}
/*******************************************
gsensor 外部中断管脚初始化
*******************************************/
void gsensor_gpio_interrupt_init(void)
{
	/***step1. set pin as gpio and enable input********/
	gpio_set_func(GSENSOR_ACTIVE_INTERRUPT_PIN, AS_GPIO);           //enable GPIO func
	gpio_set_input_en(GSENSOR_ACTIVE_INTERRUPT_PIN, 1);             //enable input
	gpio_set_output_en(GSENSOR_ACTIVE_INTERRUPT_PIN, 0);            //disable output

	/***step2.      set the polarity and open pullup ***/
	gpio_setup_up_down_resistor(GSENSOR_ACTIVE_INTERRUPT_PIN, PM_PIN_UP_DOWN_FLOAT);  //open pull down resistor
	gpio_set_interrupt_pol(GSENSOR_ACTIVE_INTERRUPT_PIN, 1);                          //falling edge

	/***step3.      set irq enable  ***/
	reg_irq_src = FLD_IRQ_GPIO_RISC0_EN; //clean irq status
	reg_irq_mask |= FLD_IRQ_GPIO_RISC0_EN;
	gpio_en_interrupt_risc0(GSENSOR_ACTIVE_INTERRUPT_PIN, 1);
	/************************************************************/
#if 0
	/***step1. set pin as gpio and enable output high********/
	gpio_set_func(GSENSOR_FREEFALL_INTERRUPT_PIN, AS_GPIO);           //enable GPIO func
	gpio_setup_up_down_resistor(GSENSOR_FREEFALL_INTERRUPT_PIN, PM_PIN_UP_DOWN_FLOAT);  //open pull down resistor
	gpio_set_input_en(GSENSOR_FREEFALL_INTERRUPT_PIN, 0);             //enable input
	gpio_set_output_en(GSENSOR_FREEFALL_INTERRUPT_PIN, 1);            //disable output
	gpio_write(GSENSOR_FREEFALL_INTERRUPT_PIN,ON);
#endif
}
/*******************************************
gsensor 外部中断处理函数
*******************************************/
_attribute_ram_code_ void gsensor_gpio_irq_proc(void)
{
	/************* gpio irq risc0 *************/
	if(reg_irq_src & FLD_IRQ_GPIO_RISC0_EN)
	{
		reg_irq_src = FLD_IRQ_GPIO_RISC0_EN;        // clear irq_gpio irq flag
		printf("Gsensor active interrupt  detect.\r\n");
	}
}
/*******************************************
定时获取三轴数据
*******************************************/
void gsensor_3axis_data_inquiry(void)
{
	s16 gsensor_x,gsensor_y,gsensor_z;
	gsensor_inquiry_3axis_data(&gsensor_x, &gsensor_y, &gsensor_z);
	printf("Gsensor 3axis data:0x%x-0x%x-0x%x.\r\n",(u16)(gsensor_x),(u16)(gsensor_y),(u16)(gsensor_z));
}
#endif
