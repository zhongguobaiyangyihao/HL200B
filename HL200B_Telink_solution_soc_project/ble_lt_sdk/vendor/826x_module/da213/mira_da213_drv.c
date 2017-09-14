#ifdef __MIRADA213_H__
#include "mira_std.h"
#include "../../proj/tl_common.h"
#include "../../proj/drivers/i2c.h"
#include "../app_config.h"
#include "../../vendor/common/myprintf.h"
typedef enum
{
	GSENSOR_SUCCESS = 0,
	GSENSOR_UNINIT = -1,
	GSENSOR_ID_MISMATCH = -2
}err_code_t;
/******************************************
*******************************************/
#define 		DA213_I2C_ADDR   			0x4E
#define 		DA213_I2C_ADDR_R			0x4F
#define 		DA213_I2C_ADDR_W			0x4E
u8              is_gsensor_init = 0;
extern u8       lock_unlock_state;
/*******************************************
hardware i2c interface init
*******************************************/
static void gsensor_i2c_init(void)
{
	i2c_pin_init(I2C_GPIO_GROUP_A3A4);
	i2c_master_init_div(DA213_I2C_ADDR,0x14);//para1:ID;para2:DivClock,i2c clock = system_clock/4*DivClock
	i2c_master_init_khz(DA213_I2C_ADDR, 200);//200Khz
	is_gsensor_init = 1;
}
/*******************************************
读取DA213寄存器数据
*******************************************/
static void da213_register_read(unsigned char addr, unsigned char *data)
{
	u8 reg_addr = addr;
	*data =	i2c_read_byte(&reg_addr, 1);
}
/*******************************************
配置DA213寄存器数据
*******************************************/
static void da213_register_write(unsigned char addr, unsigned char data)
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
	da213_register_read(NSA_REG_WHO_AM_I,&chip_id);
	printf("Gsensor chip id:0x%x.\r\n",chip_id);
	if(chip_id != DA213_CHIP_ID)
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
    if(lock_unlock_state == 1)//关锁状态下
    {
		da213_register_write(NSA_REG_G_RANGE,0x00); 	        //	resolution:14bit;full scale: +/-2g
		da213_register_write(NSA_REG_INTERRUPT_MAPPING1,0x04); 	//	map INT1 active
		da213_register_write(NSA_REG_ACTIVE_THRESHOLD,80); 	    //  threshold:~300mg  (3.91mg per)
		da213_register_write(NSA_REG_ACTIVE_DURATION,1);	    //  Active duration time =  (active_dur + 1)ms = 2ms
		da213_register_write(NSA_REG_INT_PIN_CONFIG,0x0F); 	    //	selects OD output for INT1andINT2，selects active level high for pin INT1andINT2
		da213_register_write(NSA_REG_INT_LATCH,0x82); 	        //  reset all latched int,temporary 500ms latch time
		da213_register_write(NSA_REG_INTERRUPT_SETTINGS1,0x07);	//  enable x y z int
    }
    else if(lock_unlock_state == 2)//开锁状态下
    {
    	da213_register_write(NSA_REG_G_RANGE,0x03); 	        //	resolution:14bit;full scale: +/-16g
		da213_register_write(NSA_REG_INTERRUPT_MAPPING1,0x04); 	//	map INT1 active
		da213_register_write(NSA_REG_ACTIVE_THRESHOLD,96); 	    //  threshold:~2500mg  (31.25mg per)
		da213_register_write(NSA_REG_ACTIVE_DURATION,1);	    //  Active duration time =  (active_dur + 1)ms = 2ms
		da213_register_write(NSA_REG_INT_PIN_CONFIG,0x0F); 	    //	selects OD output for INT1andINT2，selects active level high for pin INT1andINT2
		da213_register_write(NSA_REG_INT_LATCH,0x82); 	        //  reset all latched int,temporary 500ms latch time
		da213_register_write(NSA_REG_INTERRUPT_SETTINGS1,0x07);	//  enable x y z int
    }
	da213_register_write(NSA_REG_INTERRUPT_MAPPING3,0x01); 	//	map INT2 freefall
	da213_register_write(NSA_REG_INTERRUPT_SETTINGS2,0x04); //	enable the freefall interrupt
	da213_register_write(NSA_REG_FREEFALL_HYST,0x05);       //	freefall_mode:sum mode,the hysteresis for freefall detection:125mg;
	da213_register_write(NSA_REG_FREEFALL_THS,100);         //	freefall threshold = 781mg;
	da213_register_write(NSA_REG_FREEFALL_DUR,9);           //	Delay time for freefall: ( freefall_dur + 1 ) * 2ms=20ms;
    return GSENSOR_SUCCESS;
}
/*******************************************
gsensor init function
*******************************************/
void gsensor_pwr_on(void)
{
	da213_register_write(NSA_REG_POWERMODE_BW,0x10);
}
/*******************************************
gsensor init function
*******************************************/
void gsensor_pwr_off(void)
{
	da213_register_write(NSA_REG_POWERMODE_BW,0x80);
}
/*******************************************
查询加速度三轴数据
*******************************************/
void gsensor_inquiry_3axis_data(short *x, short *y, short *z)
{
	u8	tmp_data[6] = {0};
	da213_register_read(NSA_REG_ACC_X_LSB, &tmp_data[0]);
	da213_register_read(NSA_REG_ACC_X_MSB, &tmp_data[1]);
	da213_register_read(NSA_REG_ACC_Y_LSB, &tmp_data[2]);
	da213_register_read(NSA_REG_ACC_Y_MSB, &tmp_data[3]);
	da213_register_read(NSA_REG_ACC_Z_LSB, &tmp_data[4]);
	da213_register_read(NSA_REG_ACC_Z_MSB, &tmp_data[5]);
    *x = ((short)((tmp_data[1]&0x00FF << 8) | tmp_data[0]))>> 4;
    *y = ((short)((tmp_data[3]&0x00FF << 8) | tmp_data[2]))>> 4;
    *z = ((short)((tmp_data[5]&0x00FF << 8) | tmp_data[4]))>> 4;
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
	gpio_set_interrupt_pol(GSENSOR_ACTIVE_INTERRUPT_PIN, 0);    //rising edge

	/***step3.      set irq enable  ***/
	reg_irq_src = FLD_IRQ_GPIO_RISC0_EN; //clean irq status
	reg_irq_mask |= FLD_IRQ_GPIO_RISC0_EN;
	gpio_en_interrupt_risc0(GSENSOR_ACTIVE_INTERRUPT_PIN, 1);

	/***step1. set pin as gpio and enable input********/
	gpio_set_func(GSENSOR_FREEFALL_INTERRUPT_PIN, AS_GPIO);           //enable GPIO func
	gpio_set_input_en(GSENSOR_FREEFALL_INTERRUPT_PIN, 1);             //enable input
	gpio_set_output_en(GSENSOR_FREEFALL_INTERRUPT_PIN, 0);            //disable output

	/***step2.      set the polarity and open pullup ***/
	gpio_setup_up_down_resistor(GSENSOR_FREEFALL_INTERRUPT_PIN, PM_PIN_UP_DOWN_FLOAT);  //open pull down resistor
	gpio_set_interrupt_pol(GSENSOR_FREEFALL_INTERRUPT_PIN, 0);    //rising edge

	/***step3.      set irq enable  ***/
	reg_irq_src = FLD_IRQ_GPIO_RISC1_EN; //clean irq status
	reg_irq_mask |= FLD_IRQ_GPIO_RISC1_EN;
	gpio_en_interrupt_risc1(GSENSOR_FREEFALL_INTERRUPT_PIN, 1);
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
	/************* gpio irq risc1 *************/
	if(reg_irq_src & FLD_IRQ_GPIO_RISC1_EN)
	{
		reg_irq_src = FLD_IRQ_GPIO_RISC1_EN;        // clear irq_gpio irq flag
		printf("Gsensor freefall interrupt  detect.\r\n");
	}
}
#endif

