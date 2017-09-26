/*
 * Buzzer.c
 *
 *  Created on: 2017-9-18
 *      Author: Kyle
 */
#include "Buzzer.h"
/************************************************************************/

static void buzzer_turnon_lock_schedule(void);
void buzzer_schedule_handler(void);
/************************************************************************
 * buzzer on
************************************************************************/
static void buzzer_turnon(void)
{
	gpio_write(BUZZ_EN_PIN,ON);
}
/************************************************************************
 * buzzer off
************************************************************************/
static void buzzer_turnoff(void)
{
	gpio_write(BUZZ_EN_PIN,OFF);
}
/************************************************************************
 * buzzer_turnon_lock_schedule
************************************************************************/
#if 0
static void buzzer_turnon_lock_schedule(void)
{
	static u16                    waiting_time = 0;
	static Buzzer_buzz_schedule_t Buzzer_buzz_schedule = BUZZER_SCHEDULE_START;
	static u8                     Buzzer_buzz_cnt = 0;
	switch(Buzzer_buzz_schedule)
	{
		case BUZZER_SCHEDULE_START:
		{
			if(turnon_lock_flag)
			{
				waiting_time = 0;
				Buzzer_buzz_schedule = BUZZER_BUZZ_ON;
			}
			break;
		}
		case BUZZER_BUZZ_ON:
		{
			waiting_time++;
			if(waiting_time == 1)
			{
				buzzer_turnon();
			}
			if(waiting_time>=BUZZER_TURNON_LOCK_BUZZ_DELAY)
			{
				waiting_time = 0;
				Buzzer_buzz_schedule = BUZZER_BUZZ_OFF;
			}
			break;
		}
		case BUZZER_BUZZ_OFF:
		{
			waiting_time++;
			if(waiting_time == 1)
			{
				buzzer_turnoff();
			}
			if(waiting_time>=BUZZER_TURNOFF_LOCK_BUZZ_DELAY)
			{
				waiting_time = 0;
				Buzzer_buzz_cnt++;
				if(Buzzer_buzz_cnt>=BUZZER_TURNON_LOCK_BUZZ_NUMBER)
				{
					Buzzer_buzz_cnt = 0;
					Buzzer_buzz_schedule = BUZZER_SCHEDULE_STOP;
				}
				else
				{
					Buzzer_buzz_schedule = BUZZER_BUZZ_ON;
				}
			}
			break;
		}
		case BUZZER_SCHEDULE_STOP:
		{
			turnon_lock_flag = 0;
			Buzzer_buzz_schedule = BUZZER_SCHEDULE_START;
			break;
		}
		default:
		{
			Buzzer_buzz_schedule = BUZZER_SCHEDULE_START;
			break;
		}
	}
}
#else
/************************************************************************
 * buzzer_turnon_lock_schedule
************************************************************************/
static void buzzer_turnon_lock_schedule(void)
{
	u32                           start_tick;
	u8                            Buzzer_buzz_cnt = 0;
	for(;Buzzer_buzz_cnt<BUZZER_TURNON_LOCK_BUZZ_NUMBER;Buzzer_buzz_cnt++)
	{
		buzzer_turnon();
		start_tick = clock_time();
		while(!clock_time_exceed(start_tick,100*1000));
		buzzer_turnoff();
		start_tick = clock_time();
		while(!clock_time_exceed(start_tick,20*1000));
	}
}
#endif
/************************************************************************
 * buzzer_schedule_handler
************************************************************************/
void buzzer_schedule_handler(void)
{
	buzzer_turnon_lock_schedule();
}

