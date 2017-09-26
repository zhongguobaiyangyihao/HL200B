/*
 * slip_protocol.c
 *
 *  Created on: 2017-9-15
 *      Author: Kyle
 *      Serial Line Interface Protocol 串行线路接口协议
 */
#ifdef SLIP_PROTOCOL
#include "slip_protocol.h"
#include "../../proj/tl_common.h"
u8    slip_input_buffer[SLIP_RX_BUFFER_SIZE];
u16   slip_input_len = 0;
u8    slip_state_flag = SLIP_STATE_FLAG_NONE;

u8    slip_output_buffer[SLIP_TX_BUFFER_SIZE];
u16   slip_output_len = 0;
u16   slip_output_len_done = 0;
/************************************************************************
lrc check
************************************************************************/
static bool slip_lrc_check (u8 lrc, u8 *data)
{
	u8 lrc_cal = data[0];
	u8 i=0;
	for (i=1; i<6; i++)
	{
		lrc_cal ^= data[i];
	}
	return (lrc == lrc_cal);
}
/************************************************************************
slip protocol check
************************************************************************/
static u8 slip_protocol_check(ble_watchsetting_params_t  *watchsetting_params_point)
{
	u16  plen = 0;
	u16  pcmd = 0;
	u16  pssc = 0;
	u16  ppos = 0;
	if(slip_input_len<4)//btc report length error
	{
		ble_slip_protocol_reply(BTC_CONTACT, BTC_ILLEAGAL_CMD, 0, 0, NULL);
		return 0;
	}

	pssc = ((slip_input_buffer[2] << 8) & 0xff00) | (slip_input_buffer[3] & 0x00ff);

	if(slip_input_len < 9) //btc report length error
	{
		ble_slip_protocol_reply(BTC_CONTACT, BTC_ILLEAGAL_CMD, pssc, 0, NULL);
		return 0;
	}

	pcmd = ((slip_input_buffer[4] << 8) & 0xff00) | (slip_input_buffer[5] & 0x00ff);

	plen = ((slip_input_buffer[6] << 8) & 0xff00) | (slip_input_buffer[7] & 0x00ff);
	if((plen+9) != slip_input_len) //btc report length error
	{
		ble_slip_protocol_reply(BTC_CONTACT, BTC_ILLEAGAL_CMD, pssc, 0, NULL);
		return 0;
	}

	if(!unionpay_lrc_check(slip_input_buffer[plen+8], slip_input_buffer))//lrc check error
	{
		ble_slip_protocol_reply(BTC_CONTACT, BTC_ILLEAGAL_CMD, pssc, 0, NULL);
		return 0;
	}
	watchsetting_params_point->work_mode = slip_input_buffer[0];
	watchsetting_params_point->pssc      = pssc;
	watchsetting_params_point->pcmd      = pcmd;
	watchsetting_params_point->length    = plen;
	memcpy(watchsetting_params_point->params.data,&slip_input_buffer[ppos],plen);
	return 1;
}
/**
 ****************************************************************************************
 * @brief   slip_input(u8 len, u8 *data)
 ****************************************************************************************
 */
u8 slip_input(u8 len, u8 *data)
{
	u16 result = 0;
	u16 i;

	if(len == 0)
	{
		return result;
	}
	if(!data)
	{
		return result;
	}

	if((slip_input_len == 0) && (data[0] != SLIP_END))
	{
		return result;
	}

	for (i=0; i<len; i++)
	{
		if (slip_input_len>=SLIP_RX_BUFFER_SIZE)
		{
			memset(slip_input_buffer, 0, SLIP_RX_BUFFER_SIZE);
			slip_input_len = 0;
			slip_state_flag = SLIP_STATE_FLAG_NONE;
			return result;
		}
		switch (data[i])
		{
			case SLIP_END:                                       //0xC0
			{
				if(slip_input_len == 0)// slip report start
				{
					memset(slip_input_buffer, 0, SLIP_RX_BUFFER_SIZE);
					slip_input_len = 0;
				}
				else
				{
					if((i == 0) && (len > 1))// slip report restart
					{
						slip_input_len = 0;
						memset(slip_input_buffer, 0, SLIP_RX_BUFFER_SIZE);
					}
					else if(i == (len-1))//slip report end
					{
						result = slip_input_len;
					}
					else
					{
						slip_input_len = 0;
						result = 0;
						return result;
					}
				}
				slip_state_flag = SLIP_STATE_FLAG_NONE;
				break;
			}
			case SLIP_ESC:                                       //0xDB
			{
				slip_state_flag = SLIP_STATE_FLAG_GET_ESC;
				break;
			}
			case SLIP_ESC_END:                                   //0xDC
			{
				if (slip_state_flag == SLIP_STATE_FLAG_GET_ESC)
				{
					slip_input_buffer[slip_input_len] = SLIP_END;
					slip_input_len ++;
				}
				else
				{
					slip_input_buffer[slip_input_len] = SLIP_ESC_END;
					slip_input_len ++;
				}
				slip_state_flag = SLIP_STATE_FLAG_NONE;
				break;
			}
			case SLIP_ESC_ESC:                                   //0xDD
			{
				if (slip_state_flag == SLIP_STATE_FLAG_GET_ESC)
				{
					slip_input_buffer[slip_input_len] = SLIP_ESC;
					slip_input_len ++;
				}
				else
				{
					slip_input_buffer[slip_input_len] = SLIP_ESC_ESC;
					slip_input_len ++;
				}
				slip_state_flag = SLIP_STATE_FLAG_NONE;
				break;
			}
			default:
			{
				if (slip_state_flag == SLIP_STATE_FLAG_GET_ESC)
				{
					slip_input_buffer[slip_input_len] = SLIP_ESC;
					slip_input_len ++;
				}
				else
				{
					slip_input_buffer[slip_input_len] = data[i];
					slip_input_len ++;
				}
				slip_state_flag = SLIP_STATE_FLAG_NONE;
				break;
			}
		}
	}
	return result;
}
/************************************************************************
ble_slip_protocol_reply
************************************************************************/
static void ble_slip_protocol_reply(u8 head, u16 status, u16 ssc, u16 len, u8 *data)
{
	u8 i=0;
	u8 BLE_SendBuff[SLIP_TX_BUFFER_SIZE];
	u32   err_code;
	u16   send_len;

	memset(BLE_SendBuff, 0, SLIP_TX_BUFFER_SIZE);
	if ((len+9)>=SLIP_TX_BUFFER_SIZE)
	{
		return;
	}
	BLE_SendBuff[0] = head;
	BLE_SendBuff[2] = ssc >> 8;
	BLE_SendBuff[3] = ssc;
	BLE_SendBuff[4] = status >> 8;
	BLE_SendBuff[5] = status;
	BLE_SendBuff[6] = len >> 8;
	BLE_SendBuff[7] = len;
	if(data)
	{
		memcpy(BLE_SendBuff+8,data,len);
	}
	BLE_SendBuff[8+len] = BLE_SendBuff[0];
	for (i=1; i<6; i++)
	{
		BLE_SendBuff[8+len] ^= BLE_SendBuff[i];
	}
	//    if (BTC_SSC_ERROR != status) {btc_ssc_update ();}
	if (slip_output(9+len, BLE_SendBuff))
	{
		while((send_len=slip_output_len - slip_output_len_done)>0)
		{
			send_len = (send_len > 20) ? 20 : send_len;
			err_code = ble_location_lm_notif_transmit(&m_ble_location, slip_output_buffer + slip_output_len_done, send_len);
			if (err_code != NRF_SUCCESS)
			{
				break;
			}
			slip_output_len_done += send_len;
		}
	}
}
#endif
