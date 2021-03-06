#pragma once

#include "../../proj/common/types.h"
#include "../../proj/common/string.h"

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif
#if		(__PROJECT_8261_MODULE__)
	#define CHIP_TYPE				CHIP_TYPE_8261
#elif   (__PROJECT_8267_MODULE__)
	#define CHIP_TYPE				CHIP_TYPE_8267
#elif   (__PROJECT_8269_MODULE__)
	#define CHIP_TYPE				CHIP_TYPE_8269
#elif   (__PROJECT_8266_MODULE__)
	#define CHIP_TYPE				CHIP_TYPE_8266
#endif
/****************************************************************************/
#define GSENSOR_ACTIVE_INTERRUPT_PIN         GPIO_PE0
#define GSENSOR_FREEFALL_INTERRUPT_PIN       GPIO_PE1
#define CLOSE_LOCK_CHECK_PIN                 GPIO_PA0//关锁LPLUG_IN_PIN
#define OPEN_LOCK_CHECK_PIN                  GPIO_PA1//开锁MPOS_IN_PIN
#define RF_POWERON_PIN                       GPIO_PE3
#define SC6531_RESET                         GPIO_PE2
#define GSM_WAKEUP                           GPIO_PD3

#define MOTOR_PWM0                           GPIO_PC5
#define MOTOR_PWM1                           GPIO_PC4

#define BUZZ_EN_PIN                          GPIO_PA7

#define SWS_PIN_USED_AS_LED				    gpio_set_func(GPIO_PB0, AS_GPIO); \
        									gpio_set_input_en(GPIO_PB0, 0); \
        									gpio_set_output_en(GPIO_PB0, 1); \
		                                    gpio_setup_up_down_resistor(GPIO_PB0, PM_PIN_UP_DOWN_FLOAT);

#define SWS_PIN_USED_AS_SWS				    gpio_set_func(GPIO_PB0, AS_SWS); \
		                                    gpio_set_input_en(GPIO_PB0, 1); \
		                                    gpio_set_output_en(GPIO_PB0, 0); \
		                                    gpio_setup_up_down_resistor(GPIO_PB0, PM_PIN_PULLUP_1M);
#if 1//UART WAKEUP PIN
#define BLE_MCU_INDICATE_DATA_TO_MODULE      1
#define MODULE_WAKEUP_MCU 					GPIO_PB4   //gsm wakeup mcu
#define	PB4_FUNC							AS_GPIO
#define PB4_INPUT_ENABLE					1
#define	PB4_OUTPUT_ENABLE					0
#define	PB4_DATA_OUT						0
#define MODULE_WAKEUP_MCU_HIGH				gpio_setup_up_down_resistor(GPIO_PB4, PM_PIN_PULLUP_10K);
#define MODULE_WAKEUP_MCU_LOW				gpio_setup_up_down_resistor(GPIO_PB4, PM_PIN_PULLDOWN_100K);

#define GPIO_WAKEUP_MODULE  				GPIO_PD3   //mcu wakeup gsm
#define	PD3_FUNC							AS_GPIO
#define PD3_INPUT_ENABLE					0
#define	PD3_OUTPUT_ENABLE					1
#define	PD3_DATA_OUT						0
#define GPIO_WAKEUP_MODULE_HIGH				do{gpio_set_output_en(GPIO_PD3, 1); gpio_write(GPIO_PD3, 1);}while(0)
#define GPIO_WAKEUP_MODULE_LOW				do{gpio_set_output_en(GPIO_PD3, 1); gpio_write(GPIO_PD3, 0);}while(0)
#define GPIO_WAKEUP_MODULE_FLOAT			do{gpio_set_output_en(GPIO_PD3, 0); gpio_write(GPIO_PD3, 0);}while(0)
#endif
/****************************************************************************/
/////////////////////HCI ACCESS OPTIONS///////////////////////////////////////
#define HCI_USE_UART	1
#define HCI_USE_USB		0
#define HCI_ACCESS		HCI_USE_UART
//#define HCI_ACCESS		HCI_USE_USB
/////////////////////Hemiao Lock define ////////////////////
#define IDL				0x05//产品编号低字节	物联锁统一进行分配和管理
#define IDH             0x0B//产品编号低字节
#define VER             0x11//蓝牙锁固件版本号，由固件开发人员自行维护，典型是拆成 2 个 4 位的 BCD 码
typedef struct
{
	u8                 AES_Key[16];
    u8                 lock_on_pwd[6];
    u8                 reserved[4];
}nv_params_t;
typedef enum
{
	lock_onoff_state_on = 0,
	lock_onoff_state_off = 1
}lock_unlock_state_t;
typedef struct
{
	u8  reserved0                     : 1;
	u8  reserved1                     : 1;
	u8  reserved2                     : 1;
	u8  is_uart_receive_start         : 1;
	u8  is_uart_receive_finish        : 1;
	u8  is_lockon_by_order            : 1;
	u8  is_turnon_lock_order_via_ble  : 1;
	u8  is_turnon_lock_serial_number_via_ble   : 1;

	u8  is_aes_password_update        : 1;
	u8  is_lockon_password_update     : 1;
	u8  is_turnon_lock_via_ble        : 1;
	u8  is_lockoff_occur              : 1;
	u8  is_lockoff_event_occur        : 1;
	u8  is_return_domain_via_ble      : 1;
	u8  is_module_excute              : 1;
	u8  is_module_transmit            : 1;
}Flag_t;

typedef struct
{
	u8  reserved0                     : 1;
	u8  reserved1                     : 1;
	u8  reserved2                     : 1;
	u8  reserved3                     : 1;
	u8  battery_is_charging           : 1;
	u8  Gsensor_is_vibrating          : 1;
	u8  Gsensor_is_abnormal           : 1;
	u8  lock_onoff_state              : 1;
}device_state_t;

typedef struct
{
	u16 year;
	u8 month;
	u8 day;
	u8 hour;
	u8 minute;
	u8 second;
}BJ_Time_t;



////////////////////// 需要实现的接口API ///////////////////////////
#define GET_RTC_VAL()                       get_RTC_value()



///////////////////// 锁的工作状态 ////////////////////////////
//R0
enum work_status_R0
{
	//bit3
	discharge = 0,//第 3 位为充放电状态，0 放电1 有充电。
	charging,
};

////////////////////// GSM concerned (test)///////////////////////
#define GSM_VER_LEN                         12
#define GSM_VER                             '2', '1', '1', '7', '1', '5', '2', '1', '1', '6', '1', '1'
#define LOCK_ICCID_LEN                      10
#define LOCK_ICCID                          0x12, 0x34, 0x56, 0x78, 0x00, 0x12, 0x34, 0x56, 0x78,0x00
#define LOCK_IMEI_LEN                       8
#define LOCK_IMEI                           0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78
#define LOCK_SN_LEN                         13
#define LOCK_SN                             '1', 'P', 'Q', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A'
#define LOCK_DOMAIN_LEN                     12
#define LOCK_DOMAIN                         'r', 'o', 'c', 'o', 'l', 'o', 'c', 'k', '.', 'c', 'o', 'm'




/////////////////// MODULE /////////////////////////////////
#define BLE_MCU_PM_ENABLE				    1
#define TELIK_SPP_SERVICE_ENABLE			0
#define TELIK_OTA_SERVICE_ENABLE            1
#define SUPPORT_WETCHAT_ENABLE              0//support wechat
//#define BLE_MODULE_APPLICATION_ENABLE		1
#define BATT_CHECK_ENABLE       			0   //enable or disable battery voltage detection



/////////////////// led pin /////////////////////////////////
#if	(__PROJECT_8261_MODULE__ || __PROJECT_8267_MODULE__ || __PROJECT_8269_MODULE__)//8267/8269 EVK board(C1T80A30_V1.0)
//#define RED_LED                           GPIO_PD5
//#define WHITE_LED     					GPIO_PD6
//#define GREEN_LED    		    			GPIO_PD7
//#define BLUE_LED                			GPIO_PB4
#else//__PROJECT_8266_MODULE__ //8266 EVK board(C1T53A20_V2.0)
#define RED_LED                             GPIO_PA5
#define WHITE_LED     						GPIO_PB0
#define GREEN_LED    		    			GPIO_PA1
#define BLUE_LED                			GPIO_PA6
#endif
#define ON            						1
#define OFF           						0


/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Follow the telink's ble  architecture for this sdk!
//please take 8267 BLE SDK Developer Handbook for reference(page 24).
//         8267 512K flash address setting:
//			0x80000 |~~~~~~~~~~~~~~~~~~| -> 512k
//					|  user data area  |
//			0x78000 |~~~~~~~~~~~~~~~~~~|
//					|     cap value    |
//			0x77000 |~~~~~~~~~~~~~~~~~~|
//					|  mac address(6B) |
//			0x76000 |~~~~~~~~~~~~~~~~~~|
//					|    pair & sec    |
//					|       info       |
//			0x74000 |~~~~~~~~~~~~~~~~~~| ->
//					|                  |
//			0x73000 |~~~~~~~~~~~~~~~~~~|
//			        |                  |
//			0x72000 |~~~~~~~~~~~~~~~~~~|
//			        |                  |
//			0x71000 |~~~~~~~~~~~~~~~~~~|
//			        |                  |
//			0x70000 |~~~~~~~~~~~~~~~~~~| -> 448k
//					| get_key_from_UID | signature check
//			0X6f000 |~~~~~~~~~~~~~~~~~~|
//					|                  |
//                  |  user data area  |
//					|                  |
//			0x40000 |~~~~~~~~~~~~~~~~~~| -> 256k
//					|                  |
//					|                  |
//					|                  |
//					|                  |
//					| OTA firmwave bin |
//			0x20000 |~~~~~~~~~~~~~~~~~~| -> 128k
//					|                  |
//					|                  |
//			0x10000 |-                -| -> 64k
//					|                  |
//					| APP firmwave bin |
//			0x00000 |~~~~~~~~~~~~~~~~~~|
////////////// nv management //////////////////////////////

//User data adr in FLASH
#define NV_PARAMS_ADR                       0x70000
#define LOCK_AES_KEY_ADR 			        0x70000
#define SET_LOCK_WORK_MOD_ADR 			    0x71000


/////////////////// PRINT DEBUG INFO ///////////////////////
/* 826x module's pin simulate as a uart tx, Just for debugging */
#define PRINT_DEBUG_INFO                    1//open/close myprintf
#if PRINT_DEBUG_INFO
//defination debug printf pin
#define PRINT_BAUD_RATE             		115200 //1M baud rate,should Not bigger than 1M, when system clock is 16M.
#if	(__PROJECT_8261_MODULE__ || __PROJECT_8267_MODULE__ || __PROJECT_8269_MODULE__)
#define DEBUG_INFO_TX_PIN           		GPIO_PB1//GPIO_PC6//G0 for 8267/8269 EVK board(C1T80A30_V1.0)
//#define PC6_OUTPUT_ENABLE	        		1       //mini_printf function contain this
#define PULL_WAKEUP_SRC_PC6         		PM_PIN_PULLUP_1M
#else//__PROJECT_8266_MODULE__
#define DEBUG_INFO_TX_PIN           		GPIO_PD3//G9 for 8266 EVK board(C1T53A20_V2.0)
//#define PD3_OUTPUT_ENABLE	        		1       //mini_printf function contain this
#define PULL_WAKEUP_SRC_PD3         		PM_PIN_PULLUP_1M
#endif
#endif



//////////////////////////// MODULE PM GPIO	/////////////////////////////////
#if	(__PROJECT_8261_MODULE__ || __PROJECT_8267_MODULE__ || __PROJECT_8269_MODULE__)

#else//__PROJECT_8266_MODULE__
#define GPIO_WAKEUP_MODULE					GPIO_PC5   //mcu wakeup module //G3 for 8266 EVK board(C1T53A20_V2.0)
#define	PC5_FUNC							AS_GPIO
#define PC5_INPUT_ENABLE					1
#define	PC5_OUTPUT_ENABLE					0
#define	PC5_DATA_OUT						0
#define GPIO_WAKEUP_MODULE_HIGH				gpio_setup_up_down_resistor(GPIO_PC5, PM_PIN_PULLUP_10K);
#define GPIO_WAKEUP_MODULE_LOW				gpio_setup_up_down_resistor(GPIO_PC5, PM_PIN_PULLDOWN_100K);

#define GPIO_WAKEUP_MCU						GPIO_PC3   //module wakeup mcu //G2 for 8266 EVK board(C1T53A20_V2.0)
#define	PC3_FUNC							AS_GPIO
#define PC3_INPUT_ENABLE					1
#define	PC3_OUTPUT_ENABLE					1
#define	PC3_DATA_OUT						0
#define GPIO_WAKEUP_MCU_HIGH				do{gpio_set_output_en(GPIO_PC3, 1); gpio_write(GPIO_PC3, 1);}while(0)
#define GPIO_WAKEUP_MCU_LOW					do{gpio_set_output_en(GPIO_PC3, 1); gpio_write(GPIO_PC3, 0);}while(0)
#define GPIO_WAKEUP_MCU_FLOAT				do{gpio_set_output_en(GPIO_PC3, 0); gpio_write(GPIO_PC3, 0);}while(0)
#endif



/////////////////////// POWER OPTIMIZATION  AT SUSPEND ///////////////////////
//notice that: all setting here aims at power optimization ,they depends on
//the actual hardware design.You should analyze your hardware board and then
//find out the io leakage

//shut down the input enable of some gpios, to lower io leakage at suspend state
//for example:  #define PA2_INPUT_ENABLE   0



///////////// avoid ADC module current leakage (when module on suspend status) //////////////////////////////
#define ADC_MODULE_CLOSED               BM_CLR(reg_adc_mod, FLD_ADC_CLK_EN)  // adc clk disable
#define ADC_MODULE_ENABLE               BM_SET(reg_adc_mod, FLD_ADC_CLK_EN) // adc clk open



/////////////////// Clock  /////////////////////////////////
#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#define CLOCK_SYS_CLOCK_HZ  	16000000



//////////////////Extern Crystal Type///////////////////////
#define CRYSTAL_TYPE			XTAL_12M		//  extern 12M crystal



/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE		0
#define WATCHDOG_INIT_TIMEOUT		500  //ms



#if	(__PROJECT_8261_MODULE__ || __PROJECT_8267_MODULE__ || __PROJECT_8269_MODULE__)
/////////////open SWS digital pullup to prevent MCU err, this is must ////////////
#define PB0_DATA_OUT					1

#if 0  //debug GPIO
	#define	PD0_FUNC							AS_GPIO
	#define PD0_INPUT_ENABLE					0
	#define	PD0_OUTPUT_ENABLE					1
	#define DBG_CHN0_LOW						gpio_write(GPIO_PD0, 0)
	#define DBG_CHN0_HIGH						gpio_write(GPIO_PD0, 1)
	#define DBG_CHN0_TOGGLE						BM_FLIP(reg_gpio_out(GPIO_PD0), GPIO_PD0 & 0xff);


	#define	PD1_FUNC							AS_GPIO
	#define PD1_INPUT_ENABLE					0
	#define	PD1_OUTPUT_ENABLE					1
	#define DBG_CHN1_LOW						gpio_write(GPIO_PD1, 0)
	#define DBG_CHN1_HIGH						gpio_write(GPIO_PD1, 1)
	#define DBG_CHN1_TOGGLE						BM_FLIP(reg_gpio_out(GPIO_PD1), GPIO_PD1 & 0xff);


	#define	PC5_FUNC							AS_GPIO
	#define PC5_INPUT_ENABLE					0
	#define	PC5_OUTPUT_ENABLE					1
	#define DBG_CHN2_LOW						gpio_write(GPIO_PC5, 0)
	#define DBG_CHN2_HIGH						gpio_write(GPIO_PC5, 1)
	#define DBG_CHN2_TOGGLE						BM_FLIP(reg_gpio_out(GPIO_PC5), GPIO_PC5 & 0xff);


	#define	PC6_FUNC							AS_GPIO
	#define PC6_INPUT_ENABLE					0
	#define	PC6_OUTPUT_ENABLE					1
	#define DBG_CHN3_LOW						gpio_write(GPIO_PC6, 0)
	#define DBG_CHN3_HIGH						gpio_write(GPIO_PC6, 1)
	#define DBG_CHN3_TOGGLE						BM_FLIP(reg_gpio_out(GPIO_PC6), GPIO_PC6 & 0xff);
#endif

#else//__PROJECT_8266_MODULE__

#if 0
//debug chn  P2 : C4
#define PC4_FUNC							AS_GPIO
#define	PC4_OUTPUT_ENABLE					1
#define	PC4_DATA_OUT						0
#define DBG_CHN0_LOW		( *(unsigned char *)0x800593 &= (~0x10) )
#define DBG_CHN0_HIGH		( *(unsigned char *)0x800593 |= 0x10 )
#define DBG_CHN0_TOGGLE		( *(unsigned char *)0x800593 ^= 0x10 )
#endif

#endif



/////////////////// set default   ////////////////

#include "../common/default_config.h"

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

