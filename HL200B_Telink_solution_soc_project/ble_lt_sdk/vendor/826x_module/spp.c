#include "../../proj/tl_common.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/ble/ll/ll.h"
#include "../../proj_lib/ble/blt_config.h"
#include "../../vendor/826x_module/Module_Communicate/gprs_gps_module_communicate.h"
#include "spp.h"
#include "nv.h"


#if (HCI_ACCESS==HCI_USE_UART)
#include "../../proj/drivers/uart.h"
#endif

extern int	module_uart_data_flg;
extern u32  module_wakeup_module_tick;
extern u8   UART_RX_BUF_FROMMODULE[UART_RX_BUF_FROMMODULE_SIZE];
extern u8   UART_TX_BUF_TOMODULE[UART_TX_BUF_TOMODULE_SIZE];
extern u16  UART_RX_BUF_FROMMODULE_CNT;

extern my_fifo_t uart_rx_fifo;
extern my_fifo_t uart_tx_fifo;
extern void app_suspend_exit ();
extern void	task_connect(void);
///////////the code below is just for demonstration of the event callback only////////////
int event_handler(u32 h, u8 *para, int n)
{
	if((h&HCI_FLAG_EVENT_TLK_MODULE)!= 0)			//module event
	{
		switch((u8)(h&0xff))
		{
			case BLT_EV_FLAG_SCAN_RSP:
				break;
			case BLT_EV_FLAG_CONNECT:
			{
				printf("BLE connection event occured!\n\r");
				task_connect();
				break;
			}
			case BLT_EV_FLAG_TERMINATE:
			{
				printf("BLE disconnect event occured!\n\r");
				//gpio_write(GREEN_LED,OFF);
				//param_clear_flash(LOCK_AES_KEY_ADR);
				break;
			}
			case BLT_EV_FLAG_PAIRING_BEGIN:
				break;
			case BLT_EV_FLAG_PAIRING_END:
				break;
			case BLT_EV_FLAG_ENCRYPTION_CONN_DONE:
				break;
			case BLT_EV_FLAG_GPIO_EARLY_WAKEUP:
				break;
			case BLT_EV_FLAG_CHN_MAP_REQ:
				break;
			case BLT_EV_FLAG_CONN_PARA_REQ:
				break;
			case BLT_EV_FLAG_CHN_MAP_UPDATE:
				break;
			case BLT_EV_FLAG_CONN_PARA_UPDATE:
				break;
			case BLT_EV_FLAG_ADV_DURATION_TIMEOUT:
				break;
			case BLT_EV_FLAG_SUSPEND_ENTER:
				ADC_MODULE_CLOSED;
				break;
			case BLT_EV_FLAG_SUSPEND_EXIT:
				app_suspend_exit ();
				break;
			default:
				break;
		}
	}
}
/////////////////////////////////////blc_register_hci_handler for spp////////////////////////////
int rx_from_uart_cb (void)//UART data send to Master,we will handler the data as CMD or DATA
{
	if(my_fifo_get(&uart_rx_fifo) == 0)
	{
		return 0;
	}

	u8* p = my_fifo_get(&uart_rx_fifo);
	u32 rx_len = p[0]; //usually <= 255 so 1 byte should be sufficient

	if (rx_len)
	{
#if 0
		extern int bls_uart_handler (u8 *p, int n);
		bls_uart_handler(&p[4], rx_len - 4);
#else
		extern int bls_uart_handler (u8 *p, int n);
		bls_uart_handler(&p[4], rx_len);
#endif
		my_fifo_pop(&uart_rx_fifo);
	}
	return 0;
}
/*
 *uart data receive handler
 */
int bls_uart_handler(u8 *p, int n)
{
	memcpy(&UART_RX_BUF_FROMMODULE[UART_RX_BUF_FROMMODULE_CNT],p,n);
	UART_RX_BUF_FROMMODULE_CNT += n;
	if(UART_RX_BUF_FROMMODULE_CNT>=UART_RX_BUF_FROMMODULE_SIZE)
	{
		UART_RX_BUF_FROMMODULE_CNT = 0;
	}
	return 0;
}

int hci_send_data (u32 h, u8 *para, int n)
{
	u8 *p = my_fifo_wptr (&uart_tx_fifo);
	if (!p || n >= uart_tx_fifo.size)
	{
		return -1;
	}
#if (BLE_MODULE_INDICATE_DATA_TO_MCU)
	if(!module_uart_data_flg){ //UART idle, new data is sent
		GPIO_WAKEUP_MCU_HIGH;  //Notify MCU that there is data here
		module_wakeup_module_tick = clock_time() | 1;
		module_uart_data_flg = 1;
	}
#endif
#if 0
	int nl = n + 4;
	if (h & HCI_FLAG_EVENT_TLK_MODULE)
	{
		*p++ = nl;
		*p++ = nl >> 8;
		*p++ = 0xff;
		*p++ = n + 2;
		*p++ = h;
		*p++ = h>>8;
		memcpy (p, para, n);
		p += n;
	}
#else
	int nl = n;
	if (h & HCI_FLAG_EVENT_TLK_MODULE)
	{
		*p++ = nl;
		*p++ = nl >> 8;
		memcpy (p, para, n);
		p += n;
	}
#endif
	my_fifo_next (&uart_tx_fifo);
	return 0;
}
/*
 *MODULE_Data_Send
 */
void MODULE_Data_Send(u8 * p_data, u16 length)
{
	hci_send_data(HCI_FLAG_EVENT_TLK_MODULE, p_data, length);
	//hci_send_data(0, p_data, length);
}
uart_data_t T_txdata_buf;
int tx_to_uart_cb (void)
{
	u8 *p = my_fifo_get (&uart_tx_fifo);
	if (p && !uart_tx_is_busy ())
	{
		memcpy(&T_txdata_buf.data, p + 2, p[0]+p[1]*256);
		T_txdata_buf.len = p[0]+p[1]*256 ;


#if (BLE_MODULE_INDICATE_DATA_TO_MCU)
		//If the MCU side is designed to have low power consumption and the module has data to pull up
		//the GPIO_WAKEUP_MCU will only wake up the MCU, then you need to consider whether MCU needs a
		//reply time T from wakeup to a stable receive UART data. If you need a response time of T, ch-
		//ange the following 100US to the actual time required by user.
		if(module_wakeup_module_tick){
			while( !clock_time_exceed(module_wakeup_module_tick, 100) );
		}
#endif


#if __PROJECT_8266_MODULE__
		if (uart_Send_kma((u8 *)(&T_txdata_buf)))
#else
		if (uart_Send((u8 *)(&T_txdata_buf)))
#endif
		{
			my_fifo_pop (&uart_tx_fifo);
		}
	}
	return 0;
}
