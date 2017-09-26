#include "../../proj/tl_common.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/ble/ll/ll.h"
#include "../../proj_lib/ble/hci/hci.h"
#include "../../proj_lib/ble/blt_config.h"
#include "../../proj_lib/ble/trace.h"
#include "../../proj/mcu/pwm.h"
#include "../../proj_lib/ble/service/ble_ll_ota.h"
#include "../../proj/drivers/adc.h"
#include "../../proj/drivers/battery.h"
#include "../../proj_lib/ble/blt_config.h"
#include "../../proj_lib/ble/ble_smp.h"
#include "../../vendor/826x_module/Module_Communicate/gprs_gps_module_communicate.h"
#include "nv.h"
#include "../common/blt_soft_timer.h"
#if (HCI_ACCESS==HCI_USE_UART)
#include "../../proj/drivers/uart.h"
#endif

int	mcu_uart_data_flg;
u32 mcu_wakeup_module_tick;
///////////////////////////////// BLE variable concered  ////////////////////////////////////
MYFIFO_INIT(blt_rxfifo, 64, 8);
MYFIFO_INIT(blt_txfifo, 40, 8);
u8 	ui_ota_is_working = 0;
u32	ui_advertise_begin_tick;


//////////////////////////////// UART variable concered  ////////////////////////////////////
int		module_uart_working;
int     mcu_uart_working;
int		mcu_task_busy;
//		MYFIFO_INIT(uart_rx_fifo, 72, 2);
//		MYFIFO_INIT(uart_tx_fifo, 72, 4);
		MYFIFO_INIT(uart_rx_fifo, UART_RX_BUF_FROMMODULE_SIZE, 2);
		MYFIFO_INIT(uart_tx_fifo, UART_TX_BUF_TOMODULE_SIZE, 2);
#define UART_TX_BUSY			((uart_tx_fifo.rptr != uart_tx_fifo.wptr) || uart_tx_is_busy())
#define UART_RX_BUSY			(uart_rx_fifo.rptr != uart_rx_fifo.wptr)
/////////////////////////////// AES-ECB concerned ///////////////////////////////////////////
u32 	g_token;//Token
u8      g_private_AES_key[16] 	= { 0x20, 0x57, 0x2F, 0x52, 0x36, 0x4B, 0x3F, 0x47, 0x30, 0x50, 0x41, 0x58, 0x11, 0x63, 0x2D, 0x2B};//Key fixing
//u8      g_private_lockon_key[6] = {1, 2, 3, 4, 5, 6};
u8      g_private_lockon_key[6] = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30};
device_state_t           device_state;
nv_params_t              *nv_params_ptr;
BJ_Time_t                BJ_Time;
const u8                 daynumber_leap[13]={0,31,29,31,30,31,30,31,31,30,31,30,31};
const u8                 daynumber[13]     ={0,31,28,31,30,31,30,31,31,30,31,30,31};
extern  Flag_t           Flag;
/**************************************************************************/
static void user_timer0_timeout_handler(void);
void 	AES_ECB_Encryption(u8 *key, u8 *plaintext, u8 *encrypted_data);
void 	AES_ECB_Decryption(u8 *key, u8 *encrypted_data, u8 *decrypted_data);
int TIMER0_Timeout_handler(void);
extern void lock_onoff_state_polling(void);
extern void my_att_init(void);
extern s8 gsensor_init(void);
extern void gsensor_gpio_interrupt_init(void);
extern void gsensor_pwr_on(void);
extern void gsensor_pwr_off(void);
extern void gsensor_3axis_data_inquiry(void);
extern int rx_from_uart_cb(void);
extern int tx_to_uart_cb(void);
extern void module_init(void);
extern void module_transmit_receive_handler(void);
extern void module_receive_data_finish_judge(void);
extern void module_receive_data_handler(void);
extern void module_pwron(void);
extern void module_pwroff(void);
extern void module_schedule_handler(void);
extern void ble_return_domain_operation(void);
extern void battery_power_check(void);
extern void Lock_gpio_interrupt_init(void);
extern u8 Lock_turnon_operation(void);
extern void ble_return_lock_on_result(u8 result);
extern void ble_return_lock_on_result_serial_number(u8 result);
extern void ble_return_lock_on_result_order(u8 result);
extern nv_params_storage(nv_params_t *nv_params_ptr);
extern void buzzer_schedule_handler(void);
extern nv_params_t *ble_update_nv_params_ptr_inquiry(void);
extern void ble_return_lockon_password_update_result(u8 result);
extern void ble_return_lock_off_result_order(u8 result);
extern void ble_return_lock_off_result(u8 result);
extern void ble_return_aes_password_update_result(u8 result);
extern int TIMER0_Timeout_handler(void);
extern int TIMER1_Timeout_handler(void);
/**************************************************************************/
/////////////////////////////// HemiaoLock concerned ///////////////////////////////////////////
u32 g_current_time;
u32 g_locked_time;
u16 g_serialNum;
u8  g_curr_power_level;
u8  g_GSM_ID[6] = {1,2,3,4,5,6};
u8  g_GSM_ver[GSM_VER_LEN] = {GSM_VER};
u8  g_lock_ICCID[LOCK_ICCID_LEN] = {LOCK_ICCID};
u8  g_lock_IMEI[LOCK_IMEI_LEN] = {LOCK_IMEI};
u8  g_lock_SN[LOCK_SN_LEN] = {LOCK_SN};
u8  g_lock_domains[LOCK_DOMAIN_LEN] = {LOCK_DOMAIN};
u8  g_curr_lock_work_pattern = 0x00;
u16 g_curr_vol;
s16 g_curr_temp;
u16 g_curr_charge_vol;
u8  g_password[6];
u8  g_old_password[6];
u8  g_new_password[6];
u8  g_new_key[16];
u16 g_serial_num;//NO
u8  g_order[108];
u8  g_order_number;

BJ_Time_t get_RTC_value(void);

//get RTC : 年-月-日-时-分-秒信息
//TODO:待实现
BJ_Time_t get_RTC_value(void){

	BJ_Time_t RTC;
	RTC.year = 0x2017;
	RTC.month = 0x08;
	RTC.day =  0x25;
	RTC.hour = 0x15;
	RTC.minute =0x13;
	RTC.second = 0x45;
	return RTC;
}

#if SIG_PROC_ENABLE
/*------------------------------------------------------------------- l2cap data pkt(SIG) ---------------------------------------------------*
 | stamp_time(4B) |llid nesn sn md |  pdu-len   | l2cap_len(2B)| chanId(2B)| Code(1B)|Id(1B)|Data-Len(2B) |           Result(2B)             |
 |                |   type(1B)     | rf_len(1B) |       L2CAP header       |          SIG pkt Header      |  SIG_Connection_param_Update_Rsp |
 |                |                |            |     0x0006   |    0x05   |   0x13  | 0x01 |  0x0002     |             0x0000               |
 |                |          data_headr         |                                                       payload                              |
 *-------------------------------------------------------------------------------------------------------------------------------------------*/
u8 conn_update_cnt;//连接参数table的当前index
int att_sig_proc_handler (u16 connHandle, u8 * p)
{
	rf_pkt_l2cap_sig_connParaUpRsp_t* pp = (rf_pkt_l2cap_sig_connParaUpRsp_t*)p;

#if 0//test debug
	foreach(i, 16){PrintHex(*((u8*)pp + i));}printf(".\n");
#endif

	u8 sig_conn_param_update_rsp[9] = { 0x0A, 0x06, 0x00, 0x05, 0x00, 0x13, 0x01, 0x02, 0x00 };
	if(!memcmp(sig_conn_param_update_rsp, &pp->rf_len, 9) && ((pp->type&0b11) == 2)){//l2cap data pkt, start pkt
		if(pp->result == 0x0000){
			printf("SIG: the LE master Host has accepted the connection parameters.\n");
			conn_update_cnt = 0;
		}
		else if(pp->result == 0x0001)
		{
			printf("SIG: the LE master Host has rejected the connection parameters..\n");
			printf("Current Connection interval:%dus.\n", bls_ll_getConnectionInterval() * 1250 );
			conn_update_cnt++;
            if(conn_update_cnt < 4){
            	printf("Slave sent update connPara req!\n");
            }
			if(conn_update_cnt == 1){
				bls_l2cap_requestConnParamUpdate (8, 16, 0, 400);//18.75ms iOS
			}
			else if(conn_update_cnt == 2){
				bls_l2cap_requestConnParamUpdate (16,32, 0, 400);
			}
			else if(conn_update_cnt == 3){
				bls_l2cap_requestConnParamUpdate (32,60, 0, 400);
			}
			else{
				conn_update_cnt = 0;
				printf("Slave Connection Parameters Update table all tested and failed!\n");
			}
		}
	}

}
#endif
/*
 * AES_ECB_Encryption
 */
void AES_ECB_Encryption(u8 *key, u8 *plaintext, u8 *encrypted_data)
{
	extern void aes_ll_encryption(u8 *key, u8 *plaintext, u8 *result);
	extern void swapX(const u8 *src, u8 *dst, int len);
	u8 rplaintext[16],rkey[16];
	swapX(plaintext, rplaintext, 16);
	swapX(key, rkey, 16);
	aes_ll_encryption(rkey,  rplaintext, encrypted_data);
}
/*
 * AES_ECB_Decryption
 */
void AES_ECB_Decryption(u8 *key, u8 *encrypted_data, u8 *decrypted_data)
{
	extern void aes_ll_decryption(u8 *key, u8 *plaintext, u8 *result);
	extern void swapX(const u8 *src, u8 *dst, int len);
	u8 rencrypted_data[16], rkey[16];
	swapX(key, rkey, 16);
	swapX(encrypted_data, rencrypted_data, 16);
	aes_ll_decryption(rkey, rencrypted_data, decrypted_data);
}
/*
 * task_connect
 */
void task_connect(void)
{
	//bls_l2cap_requestConnParamUpdate (12, 32, 0, 400);
	//gpio_write(GREEN_LED,ON);
	//generate token:4bytes
	generateRandomNum(4, (u8*)&g_token);//random token
}
/*
 * led_init
 */
void user_led_init(void)
{
	//gpio_set_func(GREEN_LED, AS_GPIO);
	//gpio_set_input_en(GREEN_LED,0);
	//gpio_set_output_en(GREEN_LED,1);
	//gpio_write(GREEN_LED,OFF);
}

u32 tick_wakeup;
/*
 * app_module_busy
 */
int app_module_busy ()
{
	module_uart_working = (!gpio_read(MODULE_WAKEUP_MCU));  //mcu use GPIO_WAKEUP_MODULE to indicate the UART data transmission or receiving state
	mcu_uart_working = UART_TX_BUSY || UART_RX_BUSY;      //mcu checks to see if UART rx and tX are all processed
	mcu_task_busy = mcu_uart_working || module_uart_working;
	return mcu_task_busy;
}
/*
 * app_suspend_exit
 */
void app_suspend_exit()
{
	MODULE_WAKEUP_MCU_HIGH;// pullup MODULE_WAKEUP_MCU: exit from suspend退出suspend模式时上拉该管脚告知模块芯片进入wakeup模式
	bls_pm_setSuspendMask(SUSPEND_DISABLE);
	tick_wakeup = clock_time () | 1;
}
/*
 * app_suspend_enter
 */
int app_suspend_enter()
{
	if (app_module_busy ())
	{
		app_suspend_exit ();
		return 0;
	}
	return 1;
}
/*
 * app_power_management
 */
void app_power_management ()
{
//	bls_pm_setSuspendMask(SUSPEND_ADV | SUSPEND_CONN);
#if (BLE_MCU_INDICATE_DATA_TO_MODULE)
	mcu_uart_working = UART_TX_BUSY || UART_RX_BUSY;
	//When mcu's UART data is sent, the GPIO_WAKEUP_MODULE is lowered or suspended (depending on how the user is designed)
	if(mcu_uart_data_flg && !mcu_uart_working)
	{
		mcu_uart_data_flg = 0;
		mcu_wakeup_module_tick = 0;
		GPIO_WAKEUP_MODULE_HIGH;
	}
#endif
#if (BLE_MCU_PM_ENABLE)
	if(!app_module_busy() && !tick_wakeup)
	{
		bls_pm_setSuspendMask(SUSPEND_ADV | SUSPEND_CONN);
		bls_pm_setWakeupSource(PM_WAKEUP_CORE);
	}
	if(tick_wakeup && clock_time_exceed(tick_wakeup, 500))
	{
		//MODULE_WAKEUP_MCU_LOW;// pulldown MODULE_WAKEUP_MCU: enter suspend
		tick_wakeup = 0;
	}
#endif
}
/*
 * gpio interrupt handler
 */
static void user_gpio_interrupt_init(void)
{
	gpio_core_irq_enable_all(1);  //gpio interrupt must enable
	gsensor_gpio_interrupt_init();
	Lock_gpio_interrupt_init();
}
/*
 *timer init
 */
static void user_timer_init(void)
{
#if defined(Hardware_timer)
	//timer0 irq
	reg_irq_mask |= FLD_IRQ_TMR0_EN;
	reg_tmr0_tick = 0; //clear counter
	reg_tmr0_capt = CLOCK_SYS_CLOCK_1MS*GPRS_GPS_MODULE_COMMUNICATE_INTERVAL;
	reg_tmr_sta = FLD_TMR_STA_TMR0; //clear irq status
	reg_tmr_ctrl |= FLD_TMR0_EN;  //start timer

	//timer1 irq
	reg_irq_mask |= FLD_IRQ_TMR1_EN;
	reg_tmr1_tick = 0; //clear counter
	reg_tmr1_capt = CLOCK_SYS_CLOCK_1S;
	reg_tmr_sta = FLD_TMR_STA_TMR1; //clear irq status
	reg_tmr_ctrl |= FLD_TMR1_EN;  //start timer
#else
	blt_soft_timer_init();
	blt_soft_timer_add(TIMER0_Timeout_handler, GPRS_GPS_MODULE_COMMUNICATE_INTERVAL*1000);
	blt_soft_timer_add(TIMER1_Timeout_handler, 1000*1000);
#endif
}
/*
 * TIMER0 Timeout handler
 */
int TIMER0_Timeout_handler(void)
{
#if defined(Hardware_timer)
	if(reg_tmr_sta & FLD_TMR_STA_TMR0)
	{
		reg_tmr_sta = FLD_TMR_STA_TMR0; //clear irq status
	}
#else
	Flag.is_module_excute = 1;
	return 0;
#endif
}
/******************************************************************************
leapyear judgement
******************************************************************************/
static u8 leapyear_judgement(void)
{
	u8 is_leap_year;
    u16 year = BJ_Time.year;
    if((((year%4)==0)&&(year%100!=0))||(year%400==0))
      is_leap_year = 1;
    else
      is_leap_year = 0;
    return is_leap_year;
}
/*
 * TIMER1 Timeout handler
 */
int TIMER1_Timeout_handler(void)
{
#if defined(Hardware_timer)
	if(reg_tmr_sta & FLD_TMR_STA_TMR1)
	{
		reg_tmr_sta = FLD_TMR_STA_TMR1; //clear irq status
	}
#else
	u8 tmp_daynumber;
	BJ_Time.second++;
	if(BJ_Time.second>=60)
	{
		BJ_Time.second = 0;
		BJ_Time.minute++;
		/***************************************************/
		if(BJ_Time.minute >= 60)
		{
			BJ_Time.minute = 0;
			BJ_Time.hour++;
			if(BJ_Time.hour >=24)
			{
				BJ_Time.hour = 0;
				BJ_Time.day ++;
				if(leapyear_judgement())
					tmp_daynumber = daynumber_leap[BJ_Time.month];
				else
					tmp_daynumber = daynumber[BJ_Time.month];
				if(BJ_Time.day>tmp_daynumber)
				{
					BJ_Time.day = 1;
					BJ_Time.month++;
					if(BJ_Time.month>=13)
					{
						BJ_Time.month = 1;
						BJ_Time.year++;
					}
				}
			}
		}
	}
	return 0;
#endif
}
/*
 * user_timer0_timeout_handler
 */
static void user_timer0_timeout_handler(void)
{
	static u16 time_cnt = 0;
	time_cnt++;
	if(time_cnt>=(1000/GPRS_GPS_MODULE_COMMUNICATE_INTERVAL))
	{
		time_cnt = 0;
//		printf("Program run heart.\r\n");
//		gsensor_3axis_data_inquiry();
	}
	module_receive_data_finish_judge();
	module_receive_data_handler();
	module_transmit_receive_handler();
	module_schedule_handler();
}
/*
 * user_gpio_init
 */
static void user_gpio_init(void)
{
	//开锁检测管脚初始化
    gpio_set_func(OPEN_LOCK_CHECK_PIN,AS_GPIO);
    gpio_setup_up_down_resistor(OPEN_LOCK_CHECK_PIN, PM_PIN_UP_DOWN_FLOAT);
    gpio_set_input_en(OPEN_LOCK_CHECK_PIN,1);
    gpio_set_output_en(OPEN_LOCK_CHECK_PIN, 0);

    //MOTOR管脚初始化
	gpio_set_func(MOTOR_PWM0,AS_GPIO);
	gpio_setup_up_down_resistor(MOTOR_PWM0, PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(MOTOR_PWM0,0);
	gpio_set_output_en(MOTOR_PWM0, 1);
	gpio_write(MOTOR_PWM0,OFF);

	gpio_set_func(MOTOR_PWM1,AS_GPIO);
	gpio_setup_up_down_resistor(MOTOR_PWM1, PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(MOTOR_PWM1,0);
	gpio_set_output_en(MOTOR_PWM1, 1);
	gpio_write(MOTOR_PWM1,OFF);
#if 1
	//SC6531 POWER
	gpio_set_func(RF_POWERON_PIN,AS_GPIO);
	gpio_setup_up_down_resistor(RF_POWERON_PIN, PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(RF_POWERON_PIN,0);
	gpio_set_output_en(RF_POWERON_PIN, 1);
	gpio_write(RF_POWERON_PIN,OFF);

	//SC6531 RESET PIN
	gpio_set_func(SC6531_RESET,AS_GPIO);
	gpio_setup_up_down_resistor(SC6531_RESET, PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(SC6531_RESET,0);
	gpio_set_output_en(SC6531_RESET, 1);
	gpio_write(SC6531_RESET,ON);

	//SC6531F WAKEUP PIN
	gpio_set_func(GSM_WAKEUP,AS_GPIO);
	gpio_setup_up_down_resistor(GSM_WAKEUP, PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(GSM_WAKEUP,0);
	gpio_set_output_en(GSM_WAKEUP, 1);
	gpio_write(GSM_WAKEUP,ON);
#endif
	//BUZZ_EN PIN
	gpio_set_func(BUZZ_EN_PIN,AS_GPIO);
	gpio_setup_up_down_resistor(BUZZ_EN_PIN, PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(BUZZ_EN_PIN,0);
	gpio_set_output_en(BUZZ_EN_PIN, 1);
	gpio_write(BUZZ_EN_PIN,OFF);
}
/*
 * flash_init
 */
static void user_flash_init(void)
{
	nv_params_t        nv_params;
	u8                 is_need_storage = 0;
	nv_params_ptr = (volatile u32 *)NV_PARAMS_ADR;
	u8 rec_buff[16];
	u8 err_code;
	printf("/************************load->NV_PARAMS_ADR*******************************/\r\n");
	printf("AES_KEY:");
	for(u8 i=0;i<16;i++)
	{
		printf("0x%x ",g_private_AES_key[i]);
	}
	printf("\r\n");
	printf("Flash_storage_AES_KEY:");
	for(u8 i=0;i<16;i++)
	{
		printf("0x%x ",nv_params_ptr->AES_Key[i]);
	}
	printf("\r\n");
	if((nv_params_ptr->AES_Key[0] == 0xFF)&&(nv_params_ptr->AES_Key[1] == 0xFF)&&
	   (nv_params_ptr->AES_Key[2] == 0xFF)&&(nv_params_ptr->AES_Key[3] == 0xFF)&&
	   (nv_params_ptr->AES_Key[4] == 0xFF)&&(nv_params_ptr->AES_Key[5] == 0xFF)&&
	   (nv_params_ptr->AES_Key[6] == 0xFF)&&(nv_params_ptr->AES_Key[7] == 0xFF)&&
	   (nv_params_ptr->AES_Key[8] == 0xFF)&&(nv_params_ptr->AES_Key[9] == 0xFF)&&
	   (nv_params_ptr->AES_Key[10] == 0xFF)&&(nv_params_ptr->AES_Key[11] == 0xFF)&&
	   (nv_params_ptr->AES_Key[12] == 0xFF)&&(nv_params_ptr->AES_Key[13] == 0xFF)&&
	   (nv_params_ptr->AES_Key[14] == 0xFF)&&(nv_params_ptr->AES_Key[15] == 0xFF)
	  )
	{
		is_need_storage = 1;
		memcpy(nv_params.AES_Key,g_private_AES_key,16);
	}
	if((nv_params_ptr->AES_Key[0] == 0xFF)&&(nv_params_ptr->AES_Key[1] == 0xFF)&&
	   (nv_params_ptr->AES_Key[2] == 0xFF)&&(nv_params_ptr->AES_Key[3] == 0xFF)&&
	   (nv_params_ptr->AES_Key[4] == 0xFF)&&(nv_params_ptr->AES_Key[5] == 0xFF)
      )
	{
		is_need_storage = 1;
		memcpy(nv_params.lock_on_pwd,g_private_lockon_key,6);
	}
	if(is_need_storage)
	{
		is_need_storage = 0;
		err_code = nv_params_storage(&nv_params);
		printf("Flash storage result:%d.\r\n",err_code);
	}
}
/*
 * user_update_nv_params
 */
static u8 user_update_nv_params(nv_params_t *tmp_nv_params_ptr)
{
	u8 err_code;
	err_code = nv_params_storage(tmp_nv_params_ptr);
	printf("user_update_nv_params is %d.\r\n",err_code);
	printf("nv data: \r\n");foreach(i, sizeof(nv_params_t)){PrintHex(*((u8*)nv_params_ptr+i));}printf("\r\n");
	return err_code;
}
/*
 * uart_init
 */
static void user_uart_init(void)
{
	//uart
	gpio_set_input_en(GPIO_PC2, 1);
	gpio_set_input_en(GPIO_PC3, 1);
	gpio_setup_up_down_resistor(GPIO_PC2, PM_PIN_PULLUP_1M);
	gpio_setup_up_down_resistor(GPIO_PC3, PM_PIN_PULLUP_1M);
	uart_io_init(UART_GPIO_8267_PC2_PC3);

	reg_dma_rx_rdy0 = FLD_DMA_UART_RX | FLD_DMA_UART_TX;                    //clear uart rx/tx status
	CLK16M_UART115200;
	uart_BuffInit(uart_rx_fifo_b, uart_rx_fifo.size, uart_tx_fifo_b);
	blc_register_hci_handler(rx_from_uart_cb,tx_to_uart_cb);				//customized uart handler
}
/*
 * user_uart_close
 */
void user_uart_close(void)
{
	//uart
	gpio_set_input_en(GPIO_PC2, 0);
	gpio_set_input_en(GPIO_PC3, 0);
	gpio_set_output_en(GPIO_PC2, 0);
	gpio_set_output_en(GPIO_PC3, 0);
	gpio_setup_up_down_resistor(GPIO_PC2, PM_PIN_UP_DOWN_FLOAT);
	gpio_setup_up_down_resistor(GPIO_PC3, PM_PIN_UP_DOWN_FLOAT);
    //close UART clk
	BM_CLR(reg_clk_en1, FLD_CLK_UART_EN);
}
/*
 * blt_system_power_optimize
 */
static void blt_system_power_optimize(void)  //to lower system power
{
	//disable_unnecessary_module_clock
	reg_rst_clk0 &= ~FLD_RST_SPI;  //spi not use
	//reg_rst_clk0 &= ~FLD_RST_I2C;  //iic not use
	reg_rst_clk0 &= ~(FLD_RST_USB | FLD_RST_USB_PHY);//usb not use
}
/*
 * ble init
 */
static void ble_init(void)
{
	u8  MAC_Buffer[6];
	u8  tbl_mac [] = {0x1c, 0xfb, 0x62, 0x38, 0xc1, 0xa4};
	/*BLE MAC addr*/
	load_param_from_flash(CFG_ADR_MAC, MAC_Buffer, 6);//泰凌的量产治具系统会将实际产品的MAC地址烧写到0x76000这个地址，低字节在前
	if((MAC_Buffer[0] != 0xFF)||(MAC_Buffer[1] != 0xFF)||(MAC_Buffer[2] != 0xFF)||
	   (MAC_Buffer[3] != 0xFF)||(MAC_Buffer[4] != 0xFF)||(MAC_Buffer[5] != 0xFF)
	  )
	{
		memcpy(tbl_mac,MAC_Buffer,6);
	}
	else
	{
		tbl_mac[0] = (u8)rand();
		flash_write_page(CFG_ADR_MAC, 6, tbl_mac);
	}
	/*advertising packet*/
	u8 tbl_advData[] =
	{
			 0x02, 0x01,                            //BLE_GAP_AD_TYPE_FLAGS
				   0x05, 							// BLE limited discoverable mode and BR/EDR not supported
	#if SUPPORT_WETCHAT_ENABLE                      //if support wechat,
			 0x09, 0xFF,
				   0x01, 0x02,
				   tbl_mac[5], tbl_mac[4], tbl_mac[3], tbl_mac[2], tbl_mac[1], tbl_mac[0], //MAC addresse
	#else
			 0x0e, 0xFF,                            //BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA
				   0x01, 0x02,                      //ID: default is 01 02,can filter lock device
				   tbl_mac[5], tbl_mac[4], tbl_mac[3], tbl_mac[2], tbl_mac[1], tbl_mac[0], //MAC addresse
				   IDL,                             //IDL产品编号低字节，这 2 个字节由物联锁统一进行分配和管理
				   g_curr_power_level,              //PWR 是当前电量级别
				   (u8)device_state.lock_onoff_state,  //STA 是当前锁的开关状态
				   IDH,                             //IDH产品编号高字节，IDL 和 IDH 组成 16 位的产品编号，这 2 个字节由物联锁统一进行分配和管理
				   VER,                             //VER 是蓝牙锁固件版本号，由固件开发人员自行维护，典型是拆成 2 个 4 位的 BCD码
	#endif
			 0x03, 0x03,                            //BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE
				   0xE7, 0xFE,                      //广播数据中的 Service UUIDs 段，必须包含 FEE7 这个 16 位类型的UUID
	};
	/*scan response packet*/
	u8 tbl_scanRsp [] =
	{
				0x0E, 0x09,                         //BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME
				'S', 'p', 'r', 'o', 'c', 'o', 'm', 'm',
				'_', 'L', 'o', 'c', 'k'             //scan response name "Sprocomm_Lock"
	};

	blc_app_loadCustomizedParameters();             //load customized freq_offset cap value and tp value
	blt_system_power_optimize();
	/*BLE stack Initialization*/
	/*Controller Initialization*/
	blc_ll_initBasicMCU(tbl_mac);                   //mandatory
	//blc_ll_initScanning_module(tbl_mac);		    //scan module: 		 optional
	blc_ll_initAdvertising_module(tbl_mac); 	    //adv module: 		 mandatory for BLE slave,
	blc_ll_initSlaveRole_module();				    //slave module: 	 mandatory for BLE slave,
	blc_ll_initPowerManagement_module();            //pm module:      	 optional
	/*Host Initialization*/
	my_att_init();                                  //gatt initialization
	blc_l2cap_register_handler (blc_l2cap_packet_receive);  	//l2cap initialization
	//smp initialization
	//bls_smp_enableParing (SMP_PARING_CONN_TRRIGER );
	bls_ll_setAdvData((u8 *)tbl_advData, sizeof(tbl_advData) );
	bls_ll_setScanRspData((u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));
	bls_ll_setAdvParam( ADV_INTERVAL_500MS, ADV_INTERVAL_500MS + 16, \
						ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC, \
						0,  NULL,  BLT_ENABLE_ADV_ALL, ADV_FP_NONE);
	printf("Adv parameters setting success!\r\n");
	bls_ll_setAdvEnable(1);                         //adv enable
	printf("Enable ble adv!\r\n");
	rf_set_power_level_index (RF_POWER_0dBm);
	bls_pm_setSuspendMask(SUSPEND_DISABLE);        //(SUSPEND_ADV | SUSPEND_CONN)
}
/*
 *user params init
 */
void user_init()
{
	/*USER application initialization */
	s8  err_code = 0;
	memset(&Flag,0,sizeof(Flag));
	memset(&device_state,0,sizeof(device_state));
	BJ_Time.year = 2017;BJ_Time.month = 1;BJ_Time.day = 1;BJ_Time.hour = 12;BJ_Time.minute = 5;BJ_Time.second = 5;
	err_code = gsensor_init();
	if(err_code == 0)
	{
		printf("Gsensor init success.\r\n");
		gsensor_pwr_off();
		device_state.Gsensor_is_abnormal = 0;
	}
	else
	{
		device_state.Gsensor_is_abnormal = 1;
		printf("Gsensor init fail,err_code = 0x%x.\r\n",err_code);
	}
	user_gpio_interrupt_init();
	user_gpio_init();
	user_led_init();
	user_uart_init();
	user_flash_init();
	ble_init();
	user_timer_init();
//	usb_dp_pullup_en(1);                           //open USB enum
#if SIG_PROC_ENABLE
	blc_l2cap_reg_att_sig_hander(att_sig_proc_handler);         //register sig process handler
#endif
	/////////////// period task softer timer ///////////////
	//blt_soft_timer_init();
	//500ms -> 96uA
	//400ms -> 100uA
	//200ms -> 144uA
	//soft timer close  60uA
	//blt_soft_timer_add(&hemiao_400ms_period_task, 400000);//400ms
	//blt_soft_timer_add(&hemiao_1s_period_task, 1000000);//1s
	////////////////// SPP initialization ///////////////////////////////////
#if (HCI_ACCESS==HCI_USE_USB)
	blt_set_bluetooth_version (BLUETOOTH_VER_4_2);
	bls_ll_setAdvChannelMap (BLT_ENABLE_ADV_37);
	usb_bulk_drv_init (0);
	blc_register_hci_handler (blc_hci_rx_from_usb, blc_hci_tx_to_usb);
	bls_smp_enableParing (SMP_PARING_CONN_TRRIGER );
#endif
	extern int event_handler(u32 h, u8 *para, int n);
	blc_hci_registerControllerEventHandler(event_handler);		//register event callback
	bls_hci_mod_setEventMask_cmd(0xffff);			//enable all 15 events,event list see ble_ll.h
	// OTA init
#if (TELIK_OTA_SERVICE_ENABLE)
	void entry_ota_mode(void);
	void show_ota_result(int result);
	bls_ota_clearNewFwDataArea(); //must
	bls_ota_registerStartCmdCb(entry_ota_mode);
	bls_ota_registerResultIndicateCb(show_ota_result);
#endif
#if (BLE_MCU_PM_ENABLE)
	//设置gsesor uart接收唤醒中断源
	gpio_set_wakeup(GSENSOR_ACTIVE_INTERRUPT_PIN, 0, 1);  // core(gpio) high wakeup suspend
	gpio_set_wakeup(MODULE_WAKEUP_MCU, 0, 1);               // core(gpio) high wakeup suspend
	MODULE_WAKEUP_MCU_HIGH;
	bls_pm_enableAdvMcuStall(1);
	bls_pm_registerFuncBeforeSuspend(&app_suspend_enter);
#endif

#if (BATT_CHECK_ENABLE)
	#if((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
		adc_BatteryCheckInit(ADC_CLK_4M, 1, Battery_Chn_VCC, 0, SINGLEEND, RV_1P428, RES14, S_3);
	#elif(MCU_CORE_TYPE == MCU_CORE_8266)
		adc_Init(ADC_CLK_4M, ADC_CHN_D2, SINGLEEND, ADC_REF_VOL_1V3, ADC_SAMPLING_RES_14BIT, ADC_SAMPLING_CYCLE_6);
	#endif
#endif
	ui_advertise_begin_tick = clock_time();

}
/*
 * main loop flow
 */
void main_loop()
{
	static u32 tick_loop;
	u8         lock_on_result;
	u8         err_code;
	tick_loop ++;
	////////////////////////////////////// BLE entry /////////////////////////////////
	blt_sdk_main_loop();
	////////////////////////////////////// UI entry /////////////////////////////////
	blt_soft_timer_process(MAINLOOP_ENTRY);

	#if (BATT_CHECK_ENABLE)
	battery_power_check();
#endif
	//  add spp UI task
	lock_onoff_state_polling();
    if(Flag.is_module_excute)
    {
    	Flag.is_module_excute = 0;
    	user_timer0_timeout_handler();
    }
    if(Flag.is_return_domain_via_ble)
    {
    	Flag.is_return_domain_via_ble = 0;
    	ble_return_domain_operation();
    }
    if(Flag.is_turnon_lock_via_ble)
    {
    	Flag.is_turnon_lock_via_ble = 0;
    	buzzer_schedule_handler();
    	lock_on_result = Lock_turnon_operation();
        ble_return_lock_on_result(lock_on_result);
    }
    if(Flag.is_turnon_lock_serial_number_via_ble)
	{
		Flag.is_turnon_lock_serial_number_via_ble = 0;
		buzzer_schedule_handler();
		lock_on_result = Lock_turnon_operation();
		ble_return_lock_on_result_serial_number(lock_on_result);
	}
    if(Flag.is_turnon_lock_order_via_ble)
	{
		Flag.is_turnon_lock_order_via_ble = 0;
		buzzer_schedule_handler();
		lock_on_result = Lock_turnon_operation();
		ble_return_lock_on_result_order(lock_on_result);
		if(!lock_on_result)
		{
			Flag.is_lockon_by_order = 1;
		}
	}
    if(Flag.is_lockoff_occur)
	{
		Flag.is_lockoff_occur = 0;
		if(Flag.is_lockon_by_order)
		{
			Flag.is_lockon_by_order = 0;
			ble_return_lock_off_result_order((device_state.lock_onoff_state == lock_onoff_state_off)?0:1);
		}
		else
		{
			ble_return_lock_off_result((device_state.lock_onoff_state == lock_onoff_state_off)?0:1);
		}
	}
    if(Flag.is_lockon_password_update)
    {
    	Flag.is_lockon_password_update = 0;
    	err_code = user_update_nv_params(ble_update_nv_params_ptr_inquiry());
    	if(err_code == 0)
    		ble_return_lockon_password_update_result(0);
    	else
    		ble_return_lockon_password_update_result(1);
    }
    if(Flag.is_aes_password_update)
    {
    	Flag.is_aes_password_update = 0;
    	err_code = user_update_nv_params(ble_update_nv_params_ptr_inquiry());
		if(err_code == 0)
			ble_return_aes_password_update_result(0);
		else
			ble_return_aes_password_update_result(1);
    }
    app_power_management();
}


#if (TELIK_OTA_SERVICE_ENABLE)
void entry_ota_mode(void)
{
	ui_ota_is_working = 1;

	bls_ota_setTimeout(100 * 1000000); //set OTA timeout  100 S

	//gpio_write(GPIO_LED, 1);  //LED on for indicate OTA mode
}

void show_ota_result(int result)
{
#if 0
	if(result == OTA_SUCCESS){
		for(int i=0; i< 8;i++){  //4Hz shine for  4 second
			gpio_write(BLUE_LED, 0);
			sleep_us(125000);
			gpio_write(BLUE_LED, 1);
			sleep_us(125000);
		}
	}
	else{
		for(int i=0; i< 8;i++){  //1Hz shine for  4 second
			gpio_write(BLUE_LED, 0);
			sleep_us(500000);
			gpio_write(BLUE_LED, 1);
			sleep_us(500000);
		}

		//write_reg8(0x8000,result); ;while(1);  //debug which err lead to OTA fail
	}


	gpio_write(BLUE_LED, 0);
#endif
}
#endif
