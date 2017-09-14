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
#include "../../vendor/826x_module/GPS_GPRS_Communicate/gprs_gps_module_communicate.h"
#include "nv.h"

#if (HCI_ACCESS==HCI_USE_UART)
#include "../../proj/drivers/uart.h"
#endif


///////////////////////////////// BLE variable concered  ////////////////////////////////////
MYFIFO_INIT(blt_rxfifo, 64, 8);
MYFIFO_INIT(blt_txfifo, 40, 8);
u8 	ui_ota_is_working = 0;
u32	ui_advertise_begin_tick;


//////////////////////////////// UART variable concered  ////////////////////////////////////
int		module_uart_data_flg;
u32 	module_wakeup_module_tick;
int		mcu_uart_working;
int		module_uart_working;
int		module_task_busy;
		MYFIFO_INIT(uart_rx_fifo, 72, 2);
		MYFIFO_INIT(uart_tx_fifo, 72, 4);
#define UART_TX_BUSY			((uart_tx_fifo.rptr != uart_tx_fifo.wptr) || uart_tx_is_busy())
#define UART_RX_BUSY			(uart_rx_fifo.rptr != uart_rx_fifo.wptr)
/////////////////////////////// AES-ECB concerned ///////////////////////////////////////////
u32 	g_token;//Token
u8      g_private_AES_key[16] 	= { 32, 87, 47, 82, 54, 75, 63, 71, 48, 80, 65, 88, 17, 99, 45, 43};//Key fixing
//u8      g_private_AES_key[16] 	= {0x3A, 0x60, 0x43, 0x2A, 0x5C, 0x01, 0x21, 0x1F, 0x29, 0x1E, 0x0F, 0x4E, 0x0C, 0x13, 0x28, 0x25};//Key fixing
extern  u8               lock_unlock_state;
extern  Flag_t           Flag;
/**************************************************************************/
static void user_timer0_timeout_handler(void);
void 	AES_ECB_Encryption(u8 *key, u8 *plaintext, u8 *encrypted_data);
void 	AES_ECB_Decryption(u8 *key, u8 *encrypted_data, u8 *decrypted_data);
void TIMER0_Timeout_handler(void);
void unlock_lock_state_polling(void);
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
extern void module_pwron(void);
extern void module_pwroff(void);
extern void module_schedule_handler(void);
/**************************************************************************/
/////////////////////////////// HemiaoLock concerned ///////////////////////////////////////////
u32 g_current_time;
u32 g_locked_time;
u16 g_serialNum;
u8  g_curr_power_level;
u8  g_curr_lock_state = close;
u8  g_curr_status_vib_func = normal;
u8  g_curr_status_vib_status = motionless;
u8  g_curr_chg_dischg_state = discharge;
u8  g_GSM_ID[6] = {1,2,3,4,5,6};
u8  g_GSM_ver[GSM_VER_LEN] = {GSM_VER_LEN};
u8  g_lock_ICCID[LOCK_ICCID_LEN] = {LOCK_ICCID};
u8  g_lock_domains[LOCK_DOMAIN_LEN] = {LOCK_DOMAIN};
u8  g_is_order_num_unlock;//是否是订单号开锁，如果是，手动关锁，锁会自动发送订单号关锁指令给手机
u8  g_curr_lock_work_pattern = 0x00;
u16 g_curr_vol;
s16 g_curr_temp;
u16 g_curr_charge_vol;
u8  g_password[6];
u8  g_old_password[6];
u8  g_new_password[6];
u8  g_new_key[16];
u16 g_serial_num;//NO

rtc_t get_RTC_value(void);

//get RTC : 年-月-日-时-分-秒信息
//TODO:待实现
rtc_t get_RTC_value(void){

	rtc_t RTC;
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

int app_module_busy ()
{
	mcu_uart_working = gpio_read(GPIO_WAKEUP_MODULE);
	module_uart_working = UART_TX_BUSY || UART_RX_BUSY;
	module_task_busy = mcu_uart_working || module_uart_working;
	return module_task_busy;
}

void app_suspend_exit ()
{
	GPIO_WAKEUP_MODULE_HIGH;  //module enter working state
	bls_pm_setSuspendMask(SUSPEND_DISABLE);
	tick_wakeup = clock_time () | 1;
}

int app_suspend_enter ()
{
	if (app_module_busy ())
	{
		app_suspend_exit ();
		return 0;
	}
	return 1;
}

void app_power_management ()
{
#if (BLE_MODULE_INDICATE_DATA_TO_MCU)
	module_uart_working = UART_TX_BUSY || UART_RX_BUSY;

	if(module_uart_data_flg && !module_uart_working){
		module_uart_data_flg = 0;
		module_wakeup_module_tick = 0;
		GPIO_WAKEUP_MCU_LOW;
	}
#endif

	// pullup GPIO_WAKEUP_MODULE: exit from suspend
	// pulldown GPIO_WAKEUP_MODULE: enter suspend

#if (BLE_MODULE_PM_ENABLE)

	if (!app_module_busy() && !tick_wakeup)
	{
		bls_pm_setSuspendMask(SUSPEND_ADV | SUSPEND_CONN);
		bls_pm_setWakeupSource(PM_WAKEUP_CORE);
	}

	if (tick_wakeup && clock_time_exceed (tick_wakeup, 500))
	{
		GPIO_WAKEUP_MODULE_LOW;
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
}
/*
 *timer init
 */
static void user_timer_init(void)
{
	//timer0 irq
	reg_irq_mask |= FLD_IRQ_TMR0_EN;
	reg_tmr0_tick = 0; //clear counter
	reg_tmr0_capt = CLOCK_SYS_CLOCK_1MS*GPRS_GPS_MODULE_COMMUNICATE_INTERVAL;
	reg_tmr_sta = FLD_TMR_STA_TMR0; //clear irq status
	reg_tmr_ctrl |= FLD_TMR0_EN;  //start timer
}
/*
 * TIMER0 Timeout handler
 */
void TIMER0_Timeout_handler(void)
{
	if(reg_tmr_sta & FLD_TMR_STA_TMR0)
	{
		reg_tmr_sta = FLD_TMR_STA_TMR0; //clear irq status
		Flag.is_module_excute = 1;
	}
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
		printf("Program run heart.\r\n");
		//gsensor_3axis_data_inquiry();
	}
	module_transmit_receive_handler();
	module_schedule_handler();
}
/*
 * user_gpio_init
 */
static void user_gpio_init(void)
{
	//开锁管脚初始化
    gpio_set_func(LPLUG_IN_PIN,AS_GPIO);
    gpio_setup_up_down_resistor(LPLUG_IN_PIN, PM_PIN_UP_DOWN_FLOAT);
    gpio_set_input_en(LPLUG_IN_PIN,1);
    gpio_set_output_en(LPLUG_IN_PIN, 0);
    //开锁管脚初始化
	gpio_set_func(MPOS_IN_PIN,AS_GPIO);
	gpio_setup_up_down_resistor(MPOS_IN_PIN, PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(MPOS_IN_PIN,1);
	gpio_set_output_en(MPOS_IN_PIN, 0);
	//展讯模块控制上下电管脚
	gpio_set_func(RF_POWERON_PIN,AS_GPIO);
	gpio_setup_up_down_resistor(RF_POWERON_PIN, PM_PIN_PULLUP_10K);
	gpio_set_input_en(RF_POWERON_PIN,0);
	gpio_set_output_en(RF_POWERON_PIN, 1);
	gpio_write(RF_POWERON_PIN,OFF);
}
/*
 * falsh_init
 */
static void user_flash_init(void)
{
	u8  AES_Buffer[16];
#if 0
	/*user flash data load*/
	//1.load lock aes key
	printf("/************************load->LOCK_AES_KEY_ADR*******************************/\r\n");
	load_param_from_flash(LOCK_AES_KEY_ADR, g_private_AES_key, 16);//data len should NOT bigger then PARAM_NV_PDU_UNIT
	//2.load lock work pattern
	printf("/************************load->SET_LOCK_WORK_MOD_ADR*******************************/\r\n");
	load_param_from_flash(SET_LOCK_WORK_MOD_ADR, &g_curr_lock_work_pattern, 1);
	//check user data in flash full or not, if full erase the area
	printf("/************************clear->LOCK_AES_KEY_ADR*******************************/\r\n");
	param_clear_flash(LOCK_AES_KEY_ADR);
	printf("/************************clear->SET_LOCK_WORK_MOD_ADR*******************************/\r\n");
	param_clear_flash(SET_LOCK_WORK_MOD_ADR);
#else
	u8 rec_buff[16];
	printf("/************************load->LOCK_AES_KEY_ADR*******************************/\r\n");
	printf("AES_KEY:");
	for(u8 i=0;i<16;i++)
	{
		printf("0x%x ",g_private_AES_key[i]);
	}
	printf("\r\n");
	if(!load_param_from_flash(LOCK_AES_KEY_ADR, AES_Buffer, 16))
	{
		printf("Flash_storage_AES_KEY:");
		for(u8 i=0;i<16;i++)
		{
			printf("0x%x ",AES_Buffer[i]);
		}
		printf("\r\n");
		if((AES_Buffer[0] != 0xFF)||(AES_Buffer[1] != 0xFF)||(AES_Buffer[2] != 0xFF)||(AES_Buffer[3] != 0xFF)||
		   (AES_Buffer[4] != 0xFF)||(AES_Buffer[5] != 0xFF)||(AES_Buffer[6] != 0xFF)||(AES_Buffer[7] != 0xFF)
		  )
		{
			memcpy(g_private_AES_key,AES_Buffer,16);
		}
	}
#endif
}
/*
 * uart_init
 */
static user_uart_init(void)
{
	//uart
	//one gpio should be configured to act as the wakeup pin if in power saving mode; pending
	#if __PROJECT_8266_MODULE__
		gpio_set_func(GPIO_UTX, AS_UART);
		gpio_set_func(GPIO_URX, AS_UART);
		gpio_set_input_en(GPIO_UTX, 1);
		gpio_set_input_en(GPIO_URX, 1);
		gpio_write (GPIO_UTX, 1);			//pull-high RX to avoid mis-trig by floating signal
		gpio_write (GPIO_URX, 1);			//pull-high RX to avoid mis-trig by floating signal
	#else
		gpio_set_input_en(GPIO_PC2, 1);
		gpio_set_input_en(GPIO_PC3, 1);
		gpio_setup_up_down_resistor(GPIO_PC2, PM_PIN_PULLUP_1M);
		gpio_setup_up_down_resistor(GPIO_PC3, PM_PIN_PULLUP_1M);
		uart_io_init(UART_GPIO_8267_PC2_PC3);
	#endif
	reg_dma_rx_rdy0 = FLD_DMA_UART_RX | FLD_DMA_UART_TX;                    //clear uart rx/tx status
	CLK16M_UART115200;
	uart_BuffInit(uart_rx_fifo_b, uart_rx_fifo.size, uart_tx_fifo_b);
	blc_register_hci_handler(rx_from_uart_cb,tx_to_uart_cb);				//customized uart handler
}
/*
 * unlock_lock_state_polling
 */
void unlock_lock_state_polling(void)
{
	u32 start_tick = clock_time();
    if((!gpio_read(LPLUG_IN_PIN))&&(gpio_read(MPOS_IN_PIN)))//开锁状态
    {
    	while(!clock_time_exceed(start_tick,20*1000));
    	if((!gpio_read(LPLUG_IN_PIN))&&(gpio_read(MPOS_IN_PIN)))
    	{
			if(lock_unlock_state == lock_unlock_state_lock)
			{
				printf("Lock is switch on!\r\n");
			}
			lock_unlock_state = lock_unlock_state_unlock;
    	}
    }
    if((gpio_read(LPLUG_IN_PIN))&&(!gpio_read(MPOS_IN_PIN)))//闭锁状态
	{
    	while(!clock_time_exceed(start_tick,20*1000));
    	if((gpio_read(LPLUG_IN_PIN))&&(!gpio_read(MPOS_IN_PIN)))
    	{
			if(lock_unlock_state == lock_unlock_state_unlock)
			{
				printf("Lock is switch off!\r\n");
			}
			lock_unlock_state = lock_unlock_state_lock;
    	}
	}
}
/*
 * ble init
 */
static void ble_init(void)
{
	u8  MAC_Buffer[6];
	u8  tbl_mac [] = {0x1c, 0xfb, 0x62, 0x38, 0xc1, 0xa4};
	/*advertising packet*/
	u8 tbl_advData[] = {
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
				   g_curr_lock_state,               //STA 是当前锁的开关状态
				   IDH,                             //IDH产品编号高字节，IDL 和 IDH 组成 16 位的产品编号，这 2 个字节由物联锁统一进行分配和管理
				   VER,                             //VER 是蓝牙锁固件版本号，由固件开发人员自行维护，典型是拆成 2 个 4 位的 BCD码
	#endif
			 0x03, 0x03,                            //BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE
				   0xE7, 0xFE,                      //广播数据中的 Service UUIDs 段，必须包含 FEE7 这个 16 位类型的UUID
		};
	/*scan response packet*/
	u8 tbl_scanRsp [] = {
				0x0E, 0x09,                         //BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME
				'S', 'p', 'r', 'o', 'c', 'o', 'm', 'm',
				'_', 'L', 'o', 'c', 'k'             //scan response name "Sprocomm_Lock"
		};
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
		flash_write_page (CFG_ADR_MAC, 6, tbl_mac);
	}
	blc_app_loadCustomizedParameters();             //load customized freq_offset cap value and tp value
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
	bls_ll_setAdvData( (u8 *)tbl_advData, sizeof(tbl_advData) );
	bls_ll_setScanRspData( (u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));
	bls_ll_setAdvParam( ADV_INTERVAL_30MS, ADV_INTERVAL_30MS + 16, \
						ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC, \
						0,  NULL,  BLT_ENABLE_ADV_ALL, ADV_FP_NONE);
	//printf("adv parameters setting success!\r\n");
	bls_ll_setAdvEnable(1);                         //adv enable
	printf("Enable ble adv!\r\n");
	rf_set_power_level_index (RF_POWER_0dBm);
	bls_pm_setSuspendMask (SUSPEND_DISABLE);        //(SUSPEND_ADV | SUSPEND_CONN)
}
/*
 *user params init
 */
void user_init()
{
	/*USER application initialization */
	s8  err_code = 0;
	memset(&Flag,0,sizeof(Flag));
	ble_init();
	usb_dp_pullup_en(1);                           //open USB enum
#if SIG_PROC_ENABLE
	blc_l2cap_reg_att_sig_hander(att_sig_proc_handler);         //register sig process handler
#endif
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
#if (BLE_MODULE_PM_ENABLE)
	gpio_set_wakeup		(GPIO_WAKEUP_MODULE, 1, 1);  // core(gpio) high wakeup suspend
	cpu_set_gpio_wakeup (GPIO_WAKEUP_MODULE, 1, 1);  // pad high wakeup deepsleep

	GPIO_WAKEUP_MODULE_LOW;

	bls_pm_registerFuncBeforeSuspend( &app_suspend_enter );
#endif

#if (BATT_CHECK_ENABLE)
	#if((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
		adc_BatteryCheckInit(ADC_CLK_4M, 1, Battery_Chn_VCC, 0, SINGLEEND, RV_1P428, RES14, S_3);
	#elif(MCU_CORE_TYPE == MCU_CORE_8266)
		adc_Init(ADC_CLK_4M, ADC_CHN_D2, SINGLEEND, ADC_REF_VOL_1V3, ADC_SAMPLING_RES_14BIT, ADC_SAMPLING_CYCLE_6);
	#endif
#endif
	ui_advertise_begin_tick = clock_time();
	err_code = gsensor_init();
	if(err_code == 0)
	{
		printf("Gsensor init success.\r\n");
		gsensor_pwr_on();
	}
	else
	{
		printf("Gsensor init fail,err_code = 0x%x.\r\n",err_code);
	}
	user_gpio_interrupt_init();
    user_timer_init();
	user_gpio_init();
	user_led_init();
	user_uart_init();
	user_flash_init();
}

extern void battery_power_check(void);
/////////////////////////////////////////////////////////////////////
// main loop flow
/////////////////////////////////////////////////////////////////////
void main_loop ()
{
	static u32 tick_loop;

	tick_loop ++;

	////////////////////////////////////// BLE entry /////////////////////////////////
	blt_sdk_main_loop();


	////////////////////////////////////// UI entry /////////////////////////////////
#if (BATT_CHECK_ENABLE)
	battery_power_check();
#endif
	//  add spp UI task
	app_power_management ();
	unlock_lock_state_polling();
    if(Flag.is_module_excute)
    {
    	Flag.is_module_excute = 0;
    	user_timer0_timeout_handler();
    }
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
