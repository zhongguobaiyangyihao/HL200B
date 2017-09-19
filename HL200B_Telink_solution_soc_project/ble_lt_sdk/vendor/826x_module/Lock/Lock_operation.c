/*
 * Lock_operation.c
 *
 *  Created on: 2017-9-15
 *      Author: Kyle
 */
#include "../app_config.h"
#include "../../../proj/tl_common.h"

extern Flag_t           Flag;
extern device_state_t   device_state;
/*******************************************
Lock 外部中断管脚初始化（关锁）
*******************************************/
void Lock_gpio_interrupt_init(void)
{
	/***step1. set pin as gpio and enable input********/
	gpio_set_func(CLOSE_LOCK_CHECK_PIN, AS_GPIO);           //enable GPIO func
	gpio_set_input_en(CLOSE_LOCK_CHECK_PIN, 1);             //enable input
	gpio_set_output_en(CLOSE_LOCK_CHECK_PIN, 0);            //disable output

	/***step2.      set the polarity and open pullup ***/
	gpio_setup_up_down_resistor(CLOSE_LOCK_CHECK_PIN, PM_PIN_UP_DOWN_FLOAT);
	gpio_set_interrupt_pol(CLOSE_LOCK_CHECK_PIN, 1);    //rising edge

	/***step3.      set irq enable  ***/
	reg_irq_src = FLD_IRQ_GPIO_RISC1_EN; //clean irq status
	reg_irq_mask |= FLD_IRQ_GPIO_RISC1_EN;
	gpio_en_interrupt_risc1(CLOSE_LOCK_CHECK_PIN, 1);

	if(!gpio_read(CLOSE_LOCK_CHECK_PIN))
	{
		device_state.lock_onoff_state = lock_onoff_state_off;
	}
	else
	{
		device_state.lock_onoff_state = lock_onoff_state_on;
	}
}
/*******************************************
Lock 外部中断处理函数(关锁)
*******************************************/
_attribute_ram_code_ void Lock_gpio_irq_proc(void)
{
	/************* gpio irq risc0 *************/
	if(reg_irq_src & FLD_IRQ_GPIO_RISC1_EN)
	{
		reg_irq_src = FLD_IRQ_GPIO_RISC1_EN;
        Flag.is_lockoff_event_occur = 1;
		printf("Lock interrupt detect.\r\n");
	}
}
/*******************************************
电机开始转动
*******************************************/
static void Lock_motor_start(void)
{
	gpio_write(MOTOR_PWM1,ON);
	gpio_write(MOTOR_PWM0,OFF);
}
/*******************************************
电机停止转动
*******************************************/
static void Lock_motor_stop(void)
{
	u32 start_tick = clock_time();
	gpio_write(MOTOR_PWM1,ON);
	gpio_write(MOTOR_PWM0,ON);
	while(!clock_time_exceed(start_tick,10*1000));
	gpio_write(MOTOR_PWM1,OFF);
	gpio_write(MOTOR_PWM0,OFF);
}
/*******************************************
开锁操作
*******************************************/
u8 Lock_turnon_operation(void)
{
	u32 start_tick = clock_time();
	u32 delay_tick;
	Lock_motor_start();
	while(1)//waiting until CLOSE_LOCK_CHECK_PIN pull high
	{
		if(clock_time_exceed(start_tick,5000*1000))
		{
			Lock_motor_stop();
			return 1;
		}

		if(!((volatile u32)gpio_read(CLOSE_LOCK_CHECK_PIN)))
			continue;

		delay_tick = clock_time();
		while(!clock_time_exceed(delay_tick,20*1000));

		if(!(volatile u32)gpio_read(CLOSE_LOCK_CHECK_PIN))
			continue;
		else
			break;
	}
	start_tick = clock_time();
	while(1)//waiting until OPEN_LOCK_CHECK_PIN pull low
	{
		if(clock_time_exceed(start_tick,5000*1000))
		{
			Lock_motor_stop();
			return 1;
		}

		if((volatile u32)gpio_read(OPEN_LOCK_CHECK_PIN))
			continue;

		delay_tick = clock_time();
		while(!clock_time_exceed(delay_tick,20*1000));

		if((volatile u32)gpio_read(OPEN_LOCK_CHECK_PIN))
			continue;
		else
			break;
	}
	start_tick = clock_time();
	while(!clock_time_exceed(start_tick,20*1000));
	Lock_motor_stop();
	device_state.lock_onoff_state = lock_onoff_state_on;
	return 0;
}
/*******************************************
关锁操作
*******************************************/
void Lock_turnoff_operation(void)
{
	u32 start_tick = clock_time();
	u32 delay_tick;
	Lock_motor_start();
	while(1)//waiting until OPEN_LOCK_CHECK_PIN pull high
	{
		if(clock_time_exceed(start_tick,5000*1000))
		{
			Lock_motor_stop();
			return;
		}

		if(!((volatile u32)gpio_read(OPEN_LOCK_CHECK_PIN)))
			continue;

		delay_tick = clock_time();
		while(!clock_time_exceed(delay_tick,20*1000));

		if(!((volatile u32)gpio_read(OPEN_LOCK_CHECK_PIN)))
			continue;
		else
			break;
	}
	start_tick = clock_time();
	while(1)//waiting until OPEN_LOCK_CHECK_PIN pull low
	{
		if(clock_time_exceed(start_tick,5000*1000))
		{
			Lock_motor_stop();
			return;
		}

		if((volatile u32)gpio_read(OPEN_LOCK_CHECK_PIN))
			continue;

		delay_tick = clock_time();
		while(!clock_time_exceed(delay_tick,20*1000));

		if((volatile u32)gpio_read(OPEN_LOCK_CHECK_PIN))
			continue;
		else
			break;
	}
	start_tick = clock_time();
	while(!clock_time_exceed(start_tick,20*1000));
	Lock_motor_stop();
}
/*
 * lock_onoff_state_polling
 */
void lock_onoff_state_polling(void)
{
	u32 start_tick = clock_time();
    if(Flag.is_lockoff_event_occur)
	{
    	Flag.is_lockoff_event_occur = 0;
    	while(!clock_time_exceed(start_tick,20*1000));
    	if(!gpio_read(CLOSE_LOCK_CHECK_PIN))
    	{
			if(device_state.lock_onoff_state == lock_onoff_state_on)
			{
				Lock_turnoff_operation();
				Flag.is_lockoff_occur = 1;
				printf("Lock is switch off!\r\n");
			}
			device_state.lock_onoff_state = lock_onoff_state_off;
    	}
	}
}
