/*
 * gprs_gps_module_communicate.c
 *
 *  Created on: 2017-9-13
 *      Author: Kyle
 */
#include "../../proj/tl_common.h"
#include "../../proj_lib/ble/blt_config.h"
#include "gprs_gps_module_communicate.h"
#include "../app_config.h"

MODULE_TRANSREC_RETURN_RESUL_t  MODULE_TRANSREC_RETURN_RESULT;
u8                       UART_RX_BUF_FROMMODULE[UART_RX_BUF_FROMMODULE_SIZE];
u8                       UART_TX_BUF_TOMODULE[UART_TX_BUF_TOMODULE_SIZE];
u16                      UART_RX_BUF_FROMMODULE_CNT;
u16                      Transmit_tomodule_Number;
u8                       MODULE_TRANSREC_RETURN_DATA_CHECK[100];
u8                       MODULE_TRANSREC_RETURN_DATA_CHECK_LEN;
u16                      MODULE_TRANSREC_RETURN_DATA_WAITING_TIME;
u8                       MODULE_TRANSREC_RETURN_DATA_RETRY_NUMBER;
Flag_t                   Flag;
MODULE_schedule_t        MODULE_schedule = MODULE_TURNON;

const u8                 module_cmd_AT[] = "AT\r\n";
/************************************************************************/
static void module_config_params(u8 *transmit_data,u8 transmit_len,char *return_string,u8 retry_number,u16 timeout);
static u8 Check_String(u8 *src, u16 srcLen, const u8 *target, u16 tgtLen);
static void module_turnon_schedule(void);
extern void MODULE_Data_Send(u8 * p_data, u16 length);
/************************************************************************
 * module_pwron
************************************************************************/
void module_pwron(void)
{
	gpio_write(RF_POWERON_PIN,ON);
}
/************************************************************************
 * module_wakeup
************************************************************************/
void module_wakeup(void)
{
	gpio_write(GSM_WAKEUP,0);
}
/************************************************************************
 * module_pwroff
************************************************************************/
void module_pwroff(void)
{
	gpio_write(RF_POWERON_PIN,OFF);
}
/************************************************************************
 * module_init
************************************************************************/
void module_init(void)
{
	module_pwron();
	MODULE_TRANSREC_RETURN_RESULT = Waiting_for_return;
	printf("module init finish!\r\n");
}
/************************************************************************
 * module operate main schedule
************************************************************************/
void module_schedule_handler(void)
{
	switch(MODULE_schedule)
	{
		case MODULE_IDLE:
		{
			break;
		}
		case MODULE_TURNON:
		{
			module_turnon_schedule();
			break;
		}
		default:
		{
			MODULE_schedule = MODULE_IDLE;
			break;
		}
	}
}
/************************************************************************
 * module_turnon_schedule
************************************************************************/
static void module_turnon_schedule(void)
{
	static module_turnon_schedule_t module_turnon_schedule = module_turnon_pwron;
	static u16                      waiting_time = 0;
	static u8                       is_config_sent = 0;
	switch(module_turnon_schedule)
	{
		case module_turnon_pwron:
		{
            waiting_time++;
            if(waiting_time == WAIT_TIME_THREE_SECOND)
            {
            	module_init();
            }
            if(waiting_time >= WAIT_TIME_MIDDLE)
			{
				waiting_time = 0;
				module_turnon_schedule = module_turnon_check;
			}
			break;
		}
		case module_turnon_check:
		{
			if(!is_config_sent)
			{
				module_config_params((u8 *)module_cmd_AT,strlen(module_cmd_AT),"OK",5,WAIT_TIME_ONE_SECOND);
				is_config_sent = 1;
			}
			else
			{
				if(MODULE_TRANSREC_RETURN_RESULT == Waiting_for_return)
				{
					return;
				}
				is_config_sent = false;
				if(MODULE_TRANSREC_RETURN_RESULT == Config_success)
				{
					printf("Module is OK.\r\n");
				}
				else if(MODULE_TRANSREC_RETURN_RESULT == Config_failure)
				{
					printf("Module is failure.\r\n");
				}
				module_turnon_schedule = module_turnon_pwron;
				MODULE_schedule = MODULE_IDLE;
			}
			break;
		}
		default:
		{
			module_turnon_schedule = module_turnon_pwron;
			break;
		}
	}
}
/************************************************************************
 * module_config_params
************************************************************************/
static void module_config_params(u8 *transmit_data,u8 transmit_len,char *return_string,u8 retry_number,u16 timeout)
{
	Transmit_tomodule_Number = transmit_len;
	memcpy(UART_TX_BUF_TOMODULE,transmit_data,Transmit_tomodule_Number);
	MODULE_TRANSREC_RETURN_DATA_CHECK_LEN = strlen(return_string);
	memcpy(MODULE_TRANSREC_RETURN_DATA_CHECK,return_string,MODULE_TRANSREC_RETURN_DATA_CHECK_LEN);
	Flag.is_module_transmit = 1;
	MODULE_TRANSREC_RETURN_RESULT = Waiting_for_return;
	MODULE_TRANSREC_RETURN_DATA_RETRY_NUMBER = retry_number;
	MODULE_TRANSREC_RETURN_DATA_WAITING_TIME = timeout;
}
/************************************************************************
 * module_transmit_receive_Handler
************************************************************************/
void module_transmit_receive_handler(void)
{
	static module_transmit_receive_schedule_t module_transmit_receive_schedule = module_transrec_idle;
	static u16 transrec_waiting_time_cnt = 0;
	static u8  transrec_retry_number = 0;
	switch(module_transmit_receive_schedule)
	{
		case module_transrec_idle:
		{
			if(Flag.is_module_transmit)
			{
				Flag.is_module_transmit = 0;
				module_transmit_receive_schedule = module_transrec_transmit;
			}
			break;
		}
		case module_transrec_transmit:
		{
			UART_RX_BUF_FROMMODULE_CNT = 0;
			MODULE_Data_Send(UART_TX_BUF_TOMODULE,Transmit_tomodule_Number);
			module_transmit_receive_schedule = module_transrec_receive;
			break;
		}
		case module_transrec_receive:
		{
			transrec_waiting_time_cnt++;
			if(transrec_waiting_time_cnt<MODULE_TRANSREC_RETURN_DATA_WAITING_TIME)
			{
				if(UART_RX_BUF_FROMMODULE_CNT)
				{
					if(Check_String(UART_RX_BUF_FROMMODULE, UART_RX_BUF_FROMMODULE_CNT, MODULE_TRANSREC_RETURN_DATA_CHECK, MODULE_TRANSREC_RETURN_DATA_CHECK_LEN))
					{
						printf("uart rec: ");foreach(i, UART_RX_BUF_FROMMODULE_CNT){PrintHex(*((u8*)UART_RX_BUF_FROMMODULE+i));}printf("\r\n");
						MODULE_TRANSREC_RETURN_RESULT = Config_success;
						transrec_waiting_time_cnt = 0;
						transrec_retry_number = 0;
						module_transmit_receive_schedule = module_transrec_idle;
					}
				}
			}
			else
			{
				printf("uart rec: ");foreach(i, UART_RX_BUF_FROMMODULE_CNT){printf("%c",*((u8*)UART_RX_BUF_FROMMODULE+i));}printf("\r\n");
				transrec_waiting_time_cnt = 0;
				transrec_retry_number++;
				if(transrec_retry_number < MODULE_TRANSREC_RETURN_DATA_RETRY_NUMBER)
				{
					module_transmit_receive_schedule = module_transrec_transmit;
				}
				else
				{
					MODULE_TRANSREC_RETURN_RESULT = Config_failure;
					transrec_retry_number = 0;
					module_transmit_receive_schedule = module_transrec_idle;
				}
			}
			break;
		}
		default:
		{
			module_transmit_receive_schedule = module_transrec_idle;
			break;
		}
	}
}
/****************************************************************************************************
string check
****************************************************************************************************/
static u8 Check_String(u8 *src, u16 srcLen, const u8 *target, u16 tgtLen)
{
  u16 i,j;
  for(i=0; i<srcLen; i++)
  {
    if((srcLen-i)<tgtLen)
      return 0;
    for(j=0; j<tgtLen; j++)
    {
      if(*(src+i+j) != *(target+j))
        break;
    }
    if(j>=tgtLen)
      break;
  }
  return 1;
}
