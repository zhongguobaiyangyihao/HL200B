#include "../../proj/tl_common.h"
#include "../../proj_lib/ble/ll/ll.h"
#include "../../proj_lib/ble/blt_config.h"
#include "../../proj_lib/ble/service/ble_ll_ota.h"
#include "spp.h"
#include "nv.h"
#include "./time_stamp/time_stamp.h"

#if(__PROJECT_8261_MODULE__ || __PROJECT_8266_MODULE__ || __PROJECT_8267_MODULE__ || __PROJECT_8269_MODULE__)

typedef struct
{
  /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  u16 intervalMin;
  /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  u16 intervalMax;
  /** Number of LL latency connection events (0x0000 - 0x03e8) */
  u16 latency;
  /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
  u16 timeout;
} gap_periConnectParams_t;


const u16 	clientCharacterCfgUUID = GATT_UUID_CLIENT_CHAR_CFG;
const u16 	extReportRefUUID = GATT_UUID_EXT_REPORT_REF;
const u16 	reportRefUUID = GATT_UUID_REPORT_REF;
const u16 	characterPresentFormatUUID = GATT_UUID_CHAR_PRESENT_FORMAT;
const u16 	my_primaryServiceUUID = GATT_UUID_PRIMARY_SERVICE;
static const u16 my_characterUUID = GATT_UUID_CHARACTER;
const u16 	my_devServiceUUID = SERVICE_UUID_DEVICE_INFORMATION;
const u16 	my_PnPUUID = CHARACTERISTIC_UUID_PNP_ID;
const u16 	my_devNameUUID = GATT_UUID_DEVICE_NAME;


//device information
const u16 	my_gapServiceUUID = SERVICE_UUID_GENERIC_ACCESS;
// Device Name Characteristic Properties
static u8 	my_devNameCharacter = CHAR_PROP_READ | CHAR_PROP_NOTIFY;
// Appearance Characteristic Properties
const u16 	my_appearanceUIID = 0x2a01;
const u16 	my_periConnParamUUID = 0x2a04;
static u8 	my_appearanceCharacter = CHAR_PROP_READ;
// Peripheral Preferred Connection Parameters Characteristic Properties
static u8 	my_periConnParamChar = CHAR_PROP_READ;
u16 		my_appearance = GAP_APPEARE_UNKNOWN;
gap_periConnectParams_t my_periConnParameters = {20, 40, 0, 1000};


const u16 	my_gattServiceUUID = SERVICE_UUID_GENERIC_ATTRIBUTE;
const u8  	serviceChangedProp = CHAR_PROP_INDICATE;
const u16 	serviceChangeUIID = GATT_UUID_SERVICE_CHANGE;
u16 		serviceChangeVal[2] = {0};
static u8 	indCharCfg[6] = {0x0b, 0x00, 0x02, 0x29};
const u16 	userdesc_UUID		= GATT_UUID_CHAR_USER_DESC;


#define 	DEV_NAME                       			"TNLKLock"
extern u8  	ble_devName[];


///////////////////// HemiaoLock /////////////////////////////////////////////
extern u32 	g_token;//Token
extern u8 	g_private_AES_key[16];
extern u32  g_current_time;
extern u16  g_serialNum;
extern u32  g_locked_time;
extern u16  g_curr_vol;
extern s16  g_curr_temp;
extern u16  g_curr_charge_vol;
extern u8   g_password[6];
extern u16  g_serial_num;
extern u8   g_GSM_ID[6];
extern u8   g_curr_lock_work_pattern;
extern u8   g_old_password[6];
extern u8   g_new_password[6];
extern u8   g_new_key[16];
extern u8   g_GSM_ver[GSM_VER_LEN];
extern u8   g_lock_ICCID[LOCK_ICCID_LEN];
extern u8   g_lock_IMEI[LOCK_IMEI_LEN];
extern u8   g_lock_SN[LOCK_SN_LEN];
extern u8   g_lock_domains[LOCK_DOMAIN_LEN];
extern u8   g_is_order_num_unlock;
extern Flag_t Flag;
extern device_state_t device_state;
extern BJ_Time_t BJ_Time;
extern nv_params_t *nv_params_ptr;

extern void AES_ECB_Encryption(u8 *key, u8 *plaintext, u8 *encrypted_data);
extern void AES_ECB_Decryption(u8 *key, u8 *encrypted_data, u8 *decrypted_data);
extern BJ_Time_t get_RTC_value(void);



////////////////////////////////////// lock service /////////////////////////////////////
const u16 	BleLockServiceUUID         				= 0xFEE7;
const u16 	BleLockChar1UUID         				= 0x36F5;
const u16 	BleLockChar2UUID         				= 0x36F6;
static u8 	BleLockChar1Prop 				 		= CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RSP;
u8  		BleLockChar1Data[ATT_MTU_SIZE - 3];
static u8 	BleLockChar2Prop 				 		= CHAR_PROP_READ | CHAR_PROP_NOTIFY;
u8  		BleLockChar2Data[ATT_MTU_SIZE - 3];
static u8   BleLockChar2DataCCC[2]					={0};

#if SUPPORT_WETCHAT_ENABLE//if support wechat,
const u16 	BleLockChar3UUID         				= 0xFEC7;
const u16 	BleLockChar4UUID         				= 0xFEC8;
const u16 	BleLockChar5UUID         				= 0xFEC9;
static u8 	BleLockChar3Prop 				 		= CHAR_PROP_WRITE;
u8  		BleLockChar3Data[ATT_MTU_SIZE - 3];
static u8 	BleLockChar4Prop 				 		= CHAR_PROP_READ | CHAR_PROP_INDICATE;
static u8   BleLockChar4DataCCC[2]					={0};
u8  		BleLockChar4Data[ATT_MTU_SIZE - 3];
static u8 	BleLockChar5Prop 				 		= CHAR_PROP_READ;
u8  		BleLockChar5Data[ATT_MTU_SIZE - 3];
#endif


#define     BleLockChar2DataHdl                     0x10



//获取令牌指令后返回：  06 02 LEN TOKEN[4] CHIP_TP VER[2] IDL IDH FILL[4]
typedef struct
{
	u8  tokenack0;          //固定为令牌返回标识
	u8  tokenack1;
	u8	len;				//LEN 为后续有效字节数
	u32 curToken;			//4个字节的令牌
	u8	chipType;           //芯片类型，由物联锁统一进行分配和管理；
	u8  ver_major;          //版本号,如01 03 ，这里理解为 1.3 版，由固件工程师自行维护；
	u8	ver_minor;
	u8  productID_IDL;      //IDL 和 IDH 是产品编号
	u8  productID_IDH;
	u32 rsvd;
}token_packet_ack_t;

//获取同步时间指令后返回：  06 04 01 RET FILL[12]
typedef struct{
	u8 tokenack0;    //06
	u8 tokenack1;	 //04
	u8 tokenack2;	 //01
	u8 ret;          //RET 为状态返回，00 表示同步成功，01 表示同步失败。
	u8 rsvd[12];
}sync_time_ack_t;

//获取查询关锁时间指令后返回： 06 06 06 NO[2] TIME[4] FILL[7]
typedef struct{
	u8 tokenack0;    //06
	u8 tokenack1;	 //06
	u8 tokenack2;	 //06
	u16 serialNum;   //流水号
	u32 lastLockTime;//最后一次关锁的时间
	u8 rsvd[7];
}query_lock_time_ack_t;

//获取电池电压及温度指令后返回： 02 02 01 VL VOLT TL TEMP FILL[7]
typedef struct{
	u8 tokenack0;    //02
	u8 tokenack1;	 //02
	u8 tokenack2;	 //01
	u8 vl;           //VL为电压值VOLT的长度，
	u16 volt;        //电压值
	u8 tl;           //TL为温度TEMP长度
	s16 temp;        //温度，TEMP为短整型，包括负值。
	u8 rsvd[7];
}get_vol_temp_ack_t;

//获取内外部电量指令后返回： 02 02 02 VL VOLT TL TEMP CL CHARGE FILL[4]
typedef struct{
	u8 tokenack0;    //02
	u8 tokenack1;	 //02
	u8 tokenack2;	 //02
	u8 vl;           //VL为电压值VOLT的长度，
	u16 volt;        //电压值
	u8 tl;           //TL为温度TEMP长度
	s16 temp;        //温度，TEMP为短整型，包括负值。
	u8 cl;           //CL为温度CHARGE长度
	u16 charge;      //充电电压
	u8 rsvd[4];
}get_int_ext_electricity_ack_t;

//查询锁状态指令后返回： 05 0f 01 STA FILL[12]
typedef struct{
	u8 tokenack0;    //05
	u8 tokenack1;	 //0f
	u8 tokenack2;	 //01
	u8 sta;          //STA 为锁状态，00 表示开启状态，01 表示关闭状态。
	u8 rsvd[12];
}query_lock_status_ack_t;

//设置还车状态指令后返回： 05 15 01 RET FILL[12]
typedef struct{
	u8 tokenack0;    //05
	u8 tokenack1;	 //15
	u8 tokenack2;	 //01
	u8 RET;          //RET 为状态返回，00 表示设置成功，01 表示设置失败。
	u8 rsvd[12];
}set_return_car_status_ack_t;

//查询锁的工作模式指令后返回： 05 20 01 MOD FILL[12]
typedef struct
{
	u8 tokenack0;    //05
	u8 tokenack1;	 //20
	u8 tokenack2;	 //01
	u8 MOD;          //MOD 为锁工作模式，00 表示正常模式，01 表示运输模式。
	u8 rsvd[12];
}get_lock_work_pattern_ack_t;

//设置锁的工作模式指令后返回： 05 21 01  FILL[12]
typedef struct{
	u8 tokenack0;    //05
	u8 tokenack1;	 //21
	u8 tokenack2;	 //01
	u8 RET;          //RET 为状态返回，00 表示设置成功，01 表示设置失败。
	u8 rsvd[12];
}set_lock_work_pattern_ack_t;

//////////////// 查询锁的工作状态 //////////////////////
//R0
//define at file: app_config.h line 30~45
typedef union
{
	struct
	{
		u8 lock_switch_status : 1;
		u8 Gsensor_func       : 1;//振动功能
		u8 Gsensor_is_vibrating : 1;
		u8 battery_is_charging  : 1;
	};
	u8 g_status_byte;//R[0] 为状态字节
}work_status_R0_t;
//R1
u8 g_GSM_status;//R[1] 为 GSM 状态，
enum work_status_R1{//为 GSM 状态
	switch_off = 0,//表示关电状态
	query_SIM_card,//表示查询 SIM 卡
	reg_network,   //注册网络
	init_SMS_func, //初始化短信功能
	query_GPRS_network, //查询 GPRS 网络
	conn_PPP,      //连接PPP
	conn_TCP,      //连接TCP
	conn_platform_sucess,//与平台通讯成功
	power_on = 0xfe,      //已经上电
};
//R2
u8 g_last_GPRS_online_need_time;//R[2] 为上一次GPRS 上线需要的时间
//R3
u8 g_last_GPRS_rssi_val;//R[3] 为上一次GSM 信号强度值
//R4
u8 g_GPS_status;//R[4] 为 GPS 状态，00 表示关电状态，01 表示未定位，02 表示已定位，FF 表示故障。
enum work_status_R4{//为 GPS 状态
	power_off,//关电状态
	un_positioned,//未定位
	positioned,//已经定位
	lock_fault = 0xff, //故障
};
//R5
u8 g_last_GPS_location_need_time;//R[5] 为上一次GPS 定位需要的时间
//R6
u8 g_last_GPS_stars_received_num;//R[6] 为上一次GPS 的收星数量。

typedef struct
{
	//R[0~6]
	work_status_R0_t R0;
	u8 R1_GSM_status;
	u8 R2_last_GPRS_online_need_time;
	u8 R3_last_GPRS_rssi_val;
	u8 R4_GPS_status;
	u8 R5_last_GPS_location_need_time;
	u8 R6_last_GPS_stars_received_num;
}get_lock_work_status_R0_R6_t;

//查询锁的工作状态指令后返回： 05 22 08 R[7] VL VOLT TL TEMP
typedef struct
{
	u8 tokenack0;    //05
	u8 tokenack1;	 //22
	u8 tokenack2;	 //08
	get_lock_work_status_R0_R6_t R;
	u8 vl;//VL为电压值VOLT的长度
	u16 volt;
	u8 tl;//TL为温度TEMP长度，TEMP为短整型，包括负值。
	s16 temp;
}get_lock_work_status_ack_t;
//////////////////////////////////////////////////////////////

//查询锁的 GSM ID指令后返回： 05 23 06 ID[6]  FILL[7]
typedef struct{
	u8 tokenack0;    //05
	u8 tokenack1;	 //23
	u8 tokenack2;	 //06
	u8 GSM_ID[6];    //ID 为 GSM ID 号。
	u8 rsvd[7];
}get_GSM_ID_ack_t;

//查询锁的 GSM 版本号指令后返回： 05 24 01 N VER[N]
typedef struct
{
	u8 tokenack0;    //05
	u8 tokenack1;	 //24
	u8 tokenack2;	 //01
	u8 N;            //N 为 GSM 版本号字节数
	u8 ver[12];      //VER 为 GSM 版本号，其字节数是不固定的，通常为 ASCII 字符。
}get_GSM_VER_ack_t;
//查询锁的 ICCID指令后返回： 05 28  L ICCID
typedef struct
{
	u8 tokenack0;    //05
	u8 tokenack1;	 //28
	u8 L;            //L为ICCID字节数
	u8 iccid[13];    //可能为多帧
}get_ICCID_ack_t;
//查询锁的 ICCID指令后返回： 05 29  L IMEI
typedef struct
{
	u8 tokenack0;    //05
	u8 tokenack1;	 //29
	u8 L;            //L为IMEI字节数
	u8 imei[13];    //可能为多帧
}get_IMEI_ack_t;
//查询锁的 SN指令后返回： 05 41  L SN
typedef struct
{
	u8 tokenack0;    //05
	u8 tokenack1;	 //41
	u8 L;            //L为IMEI字节数
	u8 SN[13];       //可能为多帧
}get_SN_ack_t;
//查询锁的域名指令后返回： 05 30 T CI DOMAIN[12]
typedef struct
{
	u8 tokenack0;    //05
	u8 tokenack1;	 //30
	u8 T;	         //T为域名总长度
	u8 CI;           //CI为当前序号（从0开始计数）
	u8 DOMAIN[50];      //DOMAIN为锁的域名，通常为 ASCII 字符。
}get_lock_domains_ack_t;

//开锁指令后返回： 05 02 01 RET TL YY YY MM DD HH mm ss FILL[4]
typedef struct{
	u8 tokenack0;    //05
	u8 tokenack1;	 //02
	u8 tokenack2;	 //01
	u8 RET;          //RET 为状态返回，00 表示开锁成功，01 表示开锁失败
	u8 tl;           //TL为时间长度，其中年份占2个字节，其余月日时分秒各一个字节
	BJ_Time_t rtc;
	u8 rsvd[4];
}open_lock_ack_t;

//流水号开锁指令后返回： 05 32 01 RET FILL[12]
typedef struct{
	u8 tokenack0;    //05
	u8 tokenack1;	 //32
	u8 tokenack2;	 //01
	u8 RET;          //RET 为状态返回，00 表示开锁成功，01 表示开锁失败
	u8 rsvd[12];
}serial_number_lock_ack_t;

//订单号开锁指令后返回： 05 13 01 RET TL YY YY MM DD HH mm ss FILL[4]
typedef struct{
	u8 tokenack0;    //05
	u8 tokenack1;	 //13
	u8 tokenack2;	 //01
	u8 RET;          //RET 为状态返回，00 表示开锁成功，01 表示开锁失败
	u8 tl;           //TL为时间长度，其中年份占2个字节，其余月日时分秒各一个字节
	BJ_Time_t rtc;
	u8 rsvd[4];
}order_num_unlock_ack_t;

/////////////////////// 订单号开锁后再进行手动关锁，锁自动发送包含订单号的通信帧 //////////////////////
//订单号开锁后再进行手动关锁，锁自动发送包含订单号的通信帧。锁返回通信帧:
//transmit1: 05 14 01 RET TL YY YY MM DD HH mm ss FILL[4]
//transmit2: 05 15 T CI ORDER[12]
//transmit3: 05 15 T CI ORDER[?]
//...根据订单号长度定
typedef struct{
	u8 tokenack0;    //05
	u8 tokenack1;	 //14
	u8 tokenack2;	 //01
	u8 RET;          //RET 为状态返回，00 表示关锁成功，01 表示关锁失败
	u8 tl;           //TL为时间长度，其中年份占2个字节，其余月日时分秒各一个字节
	BJ_Time_t rtc;
	u8 rsvd[4];
}order_num_lock_send1_t;
typedef struct{
	u8 tokenack0;    //05
	u8 tokenack1;	 //15
	u8 T;            //T为订单号总长度
	u8 CI;           //CI为发送序列的序号（从0开始计数）
    u8 order[12];    //ORDER为ASCII 字符。
}order_num_lock_send2_t;
/////////////////////////////////////////////////////////////////////////////////////////////

//修改密码指令后返回： 05 05 01 RET FILL[12]
typedef struct{
	u8 tokenack0;    //05
	u8 tokenack1;	 //05
	u8 tokenack2;	 //01
	u8 RET;          //RET 为状态返回，00 表示修改密码成功，01 表示修改密码失败
	u8 rsvd[12];
}modify_pwd_ack_t;

//修改密钥指令后返回： 07 03 01 RET FILL[12]
typedef struct{
	u8 tokenack0;    //07
	u8 tokenack1;	 //03
	u8 tokenack2;	 //01
	u8 RET;          //RET 为状态返回，00 表示修改密钥成功，01 表示修改密钥失败
	u8 rsvd[12];
}modify_key_ack_t;


u8 order[24];//单号缓存数组24byts,根据实际修改
u8 order_len;
u8 tmp_order_len;


//订单号开锁后再进行手动关锁，锁自动发送包含订单号的通信帧。锁返回通信帧：
u8 order_num_lock_notify2master(void){
	u8 encrypted_data[16];
	if(g_is_order_num_unlock){
		g_is_order_num_unlock = 0;

		order_num_lock_send1_t order_num_lock_send1;
		order_num_lock_send1.tokenack0 = 0x05;
		order_num_lock_send1.tokenack1 = 0x14;
		order_num_lock_send1.tokenack2 = 0x01;
		//TODO:实际电机控制锁关闭后，需要立即更新全局变量g_curr_lock_state
		order_num_lock_send1.tl = 7; //TL为时间长度，其中年份占2个字节，其余月日时分秒各一个字节。
		order_num_lock_send1.RET = device_state.lock_onoff_state;//RET 为状态返回，00 表示关锁成功，01 表示关锁失败
		order_num_lock_send1.rtc = GET_RTC_VAL();

		//send cmd1 data
		AES_ECB_Encryption(g_private_AES_key, (u8*)&order_num_lock_send1, encrypted_data);
		bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);

		//send left data
		order_num_lock_send2_t order_num_lock_send2;
		order_num_lock_send2.tokenack0 = 0x05;
		order_num_lock_send2.tokenack1 = 0x15;
		order_num_lock_send2.T = tmp_order_len;//T为订单号总长度

		s8 notify_cnt = tmp_order_len/12;
		u8 left_notify_cnt = tmp_order_len%12;
		u8 send_cnt = 0;

		do{
			memset(order_num_lock_send2.order, 0, 12);
			order_num_lock_send2.CI = send_cnt;
			memcpy(order_num_lock_send2.order, order+(send_cnt*12), tmp_order_len > 12 ? 12 : tmp_order_len);
			AES_ECB_Encryption(g_private_AES_key, (u8*)&order_num_lock_send1, encrypted_data);
			bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
			send_cnt++;
		}while((--notify_cnt) > 0);

		if(left_notify_cnt){
			memset(order_num_lock_send2.order, 0, 12);
			order_num_lock_send2.CI = send_cnt;
			memcpy(order_num_lock_send2.order, order+(send_cnt*12), left_notify_cnt);
			AES_ECB_Encryption(g_private_AES_key, (u8*)&order_num_lock_send2, encrypted_data);
			bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
		}
		return 1;
	}
	return 0;
}


int hemiaolock_read(void * p)
{
	u8 encrypted_data[16] = {0xaa, 0xbb, 0xdd};
	bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
	return 0;
}
/************************************************************
ble_return_lock_on_result
************************************************************/
void ble_return_lock_on_result(u8 result)
{
	u8 tmp_buffer[20];
	u8 encrypted_data[16];//S->M encryption
	open_lock_ack_t* open_lock_ack = (open_lock_ack_t*)tmp_buffer;
	open_lock_ack->tokenack0 = 0x05;
	open_lock_ack->tokenack1 = 0x02;
	open_lock_ack->tokenack2 = 0x01;
	open_lock_ack->RET = result;//RET 为状态返回，00 表示开锁成功，01 表示开锁失败。
	open_lock_ack->tl = 7;//TL为时间长度，其中年份占2个字节，其余月日时分秒各一个字节
	open_lock_ack->rtc = BJ_Time;

	AES_ECB_Encryption(g_private_AES_key, (u8*)open_lock_ack, encrypted_data);
	bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
}
/************************************************************
ble_return_domain_operation
************************************************************/
void ble_return_domain_operation(void)
{
	u8 current_index = 0;
	u8 total_index = (LOCK_DOMAIN_LEN+11)/12;
	u8 current_transmiting[12];
	u8 tmp_buffer[20];
	u8 encrypted_data[16];//S->M encryption
	get_lock_domains_ack_t* get_lock_domains_ack = (get_lock_domains_ack_t*)tmp_buffer;
	u32 start_tick;
    for(;current_index<total_index;current_index++)
    {
    	memset(current_transmiting,0,sizeof(current_transmiting));
    	if(current_index == (total_index-1))
    		memcpy(current_transmiting,(g_lock_domains+current_index*12),(LOCK_DOMAIN_LEN-current_index*12));
    	else
    		memcpy(current_transmiting,(g_lock_domains+current_index*12),sizeof(current_transmiting));
		get_lock_domains_ack->tokenack0 = 0x05;
		get_lock_domains_ack->tokenack1 = 0x30;
		get_lock_domains_ack->T = LOCK_DOMAIN_LEN;
		get_lock_domains_ack->CI = current_index;
		memcpy(get_lock_domains_ack->DOMAIN, current_transmiting, 12);
		printf("rsponse data: ");foreach(i, 16){PrintHex(*((u8*)get_lock_domains_ack+i));}printf("\r\n");
		AES_ECB_Encryption(g_private_AES_key, (u8*)get_lock_domains_ack, encrypted_data);
		bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
		if(current_index != (total_index-1))
		{
			start_tick = clock_time();
			while(!clock_time_exceed(start_tick,10*1000));
		}
    }
}
/************************************************************
token check
************************************************************/
static u8 gap_check_token_is_met(u8 *token_ptr)
{
	return(
			((*token_ptr)     == ((g_token&0x000000FF)>>0))  &&
			((*(token_ptr+1)) == ((g_token&0x0000FF00)>>8))  &&
			((*(token_ptr+2)) == ((g_token&0x00FF0000)>>16)) &&
			((*(token_ptr+3)) == ((g_token&0xFF000000)>>24))
		  );
}
/************************************************************
Function for handling the Write event.
Event received from the BLE stack.
************************************************************/
int sprocomm_lock_write_evt_handler(rf_packet_att_write_t *p)
{
	u8 len = p->l2capLen - 3;
	u8 decrypted_data[16];//M->S decryption
	u8 encrypted_data[16];//S->M encryption
	u16 cmdOpCode;
	u8 cmd_len = 4;//default 4ytes bcmd
	static u8 continue_cmd_flg;
	static u8 order_transmit_seq;
	static u8 cnt;
	if(len == 16)
	{   //HemiaoLock data length id fixed as 16 bytes

		AES_ECB_Decryption(g_private_AES_key, &p->value, decrypted_data);

		memcpy((u8*)&cmdOpCode, decrypted_data, sizeof(u16));
		memset((void*)p, 0, sizeof(rf_packet_att_write_t));
		printf("\r\n\r\ndecrypted_data: ");foreach(i, 16){PrintHex(*((u8*)decrypted_data+i));}printf("\r\n");
		switch(CMD_LEN2BYTES(cmdOpCode))
		{
		    case GET_LOCK_TOKEN_CMD://0x06 0x01
			{
				printf("GET_LOCK_TOKEN_CMD.\r\n");
				token_packet_ack_t* token_ack = (token_packet_ack_t*)p;
				token_ack->tokenack0 = 0x06;
				token_ack->tokenack1 = 0x02;
				token_ack->len = 0x09;
				token_ack->curToken = g_token;
				token_ack->chipType = 0x01;
				token_ack->ver_major = (VER&0xF0)>>4;
				token_ack->ver_minor = (VER&0x0F);
				token_ack->productID_IDL = IDL;
				token_ack->productID_IDH = IDH;
				printf("rsponse data: ");foreach(i, 16){PrintHex(*((u8*)token_ack+i));}printf("\r\n");
				AES_ECB_Encryption(g_private_AES_key, (u8*)token_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				break;
			}
		    case GET_LOCK_GSM_ID_CMD://0x05 0x23
		    {
                if(!gap_check_token_is_met(decrypted_data+4))
                {
                	break;
                }
				printf("GET_LOCK_GSM_ID_CMD.\r\n");
				get_GSM_ID_ack_t* get_GSM_ID_ack = (get_GSM_ID_ack_t*)p;
				get_GSM_ID_ack->tokenack0 = 0x05;
				get_GSM_ID_ack->tokenack1 = 0x23;
				get_GSM_ID_ack->tokenack2 = 0x06;
				memcpy(get_GSM_ID_ack->GSM_ID, g_GSM_ID, 6);
				printf("rsponse data: ");foreach(i, 16){PrintHex(*((u8*)get_GSM_ID_ack+i));}printf("\r\n");
				AES_ECB_Encryption(g_private_AES_key, (u8*)get_GSM_ID_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
		    	break;
		    }
		    case GET_LOCK_WORK_PATTERN_CMD://0x05 0x20
		    {
		    	if(!gap_check_token_is_met(decrypted_data+4))
		    	{
		    		break;
		    	}
				printf("GET_LOCK_WORK_PATTERN_CMD.\r\n");
				get_lock_work_pattern_ack_t* get_lock_work_pattern_ack = (get_lock_work_pattern_ack_t*)p;
				get_lock_work_pattern_ack->tokenack0 = 0x05;
				get_lock_work_pattern_ack->tokenack1 = 0x20;
				get_lock_work_pattern_ack->tokenack2 = 0x01;
				get_lock_work_pattern_ack->MOD = g_curr_lock_work_pattern;//MOD 为锁工作模式，00 表示正常模式，01 表示运输模式
				printf("rsponse data: ");foreach(i, 16){PrintHex(*((u8*)get_lock_work_pattern_ack+i));}printf("\r\n");
				AES_ECB_Encryption(g_private_AES_key, (u8*)get_lock_work_pattern_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
		    	break;
		    }
		    case GET_LOCK_WORKING_STATUS_CMD://0x05 0x22
		    {
		    	if(!gap_check_token_is_met(decrypted_data+4))
		    	{
		    		break;
		    	}
				printf("GET_LOCK_WORKING_STATUS_CMD.\r\n");
				get_lock_work_status_ack_t* get_lock_work_status_ack = (get_lock_work_status_ack_t*)p;
				get_lock_work_status_ack->tokenack0 = 0x05;
				get_lock_work_status_ack->tokenack1 = 0x22;
				get_lock_work_status_ack->tokenack2 = 0x08;
				get_lock_work_status_ack->R.R0.lock_switch_status = device_state.lock_onoff_state;
				get_lock_work_status_ack->R.R0.Gsensor_func = device_state.Gsensor_is_abnormal;
				get_lock_work_status_ack->R.R0.Gsensor_is_vibrating = device_state.Gsensor_is_vibrating;
				get_lock_work_status_ack->R.R0.battery_is_charging = device_state.battery_is_charging;
				get_lock_work_status_ack->R.R1_GSM_status = g_GSM_status;
				get_lock_work_status_ack->R.R2_last_GPRS_online_need_time = g_last_GPRS_online_need_time;
				get_lock_work_status_ack->R.R3_last_GPRS_rssi_val = g_last_GPRS_rssi_val;
				get_lock_work_status_ack->R.R4_GPS_status = g_GPS_status;
				get_lock_work_status_ack->R.R5_last_GPS_location_need_time = g_last_GPS_location_need_time;
				get_lock_work_status_ack->R.R6_last_GPS_stars_received_num = g_last_GPS_stars_received_num;
				get_lock_work_status_ack->vl = 2;//电压值volt的长度,这里默认2
				get_lock_work_status_ack->volt = g_curr_vol;
				get_lock_work_status_ack->tl = 2;//温度值temp的长度,这里默认2
				get_lock_work_status_ack->temp = g_curr_temp;
				printf("rsponse data: ");foreach(i, 16){PrintHex(*((u8*)get_lock_work_status_ack+i));}printf("\r\n");
				AES_ECB_Encryption(g_private_AES_key, (u8*)get_lock_work_status_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
		    	break;
		    }
		    case GET_LOCK_GSM_VER_CMD://0x05 0x24
		    {
		    	if(!gap_check_token_is_met(decrypted_data+4))
		    	{
		    		break;
		    	}
				printf("GET_LOCK_GSM_VER_CMD.\r\n");
				get_GSM_VER_ack_t* get_GSM_VER_ack = (get_GSM_VER_ack_t*)p;
				get_GSM_VER_ack->tokenack0 = 0x05;
				get_GSM_VER_ack->tokenack1 = 0x24;
				get_GSM_VER_ack->tokenack2 = 0x01;
				get_GSM_VER_ack->N = GSM_VER_LEN;
				memcpy(get_GSM_VER_ack->ver, g_GSM_ver, GSM_VER_LEN);
				printf("rsponse data: ");foreach(i, 16){PrintHex(*((u8*)get_GSM_VER_ack+i));}printf("\r\n");
				AES_ECB_Encryption(g_private_AES_key, (u8*)get_GSM_VER_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				break;
		    }
		    case GET_LOCK_ICCID_CMD://0x05 0x28
		    {
		    	if(!gap_check_token_is_met(decrypted_data+4))
				{
		    		break;
				}
				printf("GET_LOCK_ICCID_CMD.\r\n");
				get_ICCID_ack_t* get_ICCID_ack = (get_ICCID_ack_t*)p;
				get_ICCID_ack->tokenack0 = 0x05;
				get_ICCID_ack->tokenack1 = 0x28;
				get_ICCID_ack->L = LOCK_ICCID_LEN;
				memcpy(get_ICCID_ack->iccid, g_lock_ICCID, LOCK_ICCID_LEN);
				printf("rsponse data: ");foreach(i, 16){PrintHex(*((u8*)get_ICCID_ack+i));}printf("\r\n");
				AES_ECB_Encryption(g_private_AES_key, (u8*)get_ICCID_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
		    	break;
		    }
		    case GET_LOCK_IMEI_CMD://0x05 0x29
			{
				if(!gap_check_token_is_met(decrypted_data+4))
				{
					break;
				}
				printf("GET_LOCK_IMEI_CMD.\r\n");
				get_IMEI_ack_t* get_IMEI_ack = (get_IMEI_ack_t*)p;
				get_IMEI_ack->tokenack0 = 0x05;
				get_IMEI_ack->tokenack1 = 0x29;
				get_IMEI_ack->L = LOCK_IMEI_LEN;
				memcpy(get_IMEI_ack->imei, g_lock_IMEI, LOCK_IMEI_LEN);
				printf("rsponse data: ");foreach(i, 16){PrintHex(*((u8*)get_IMEI_ack+i));}printf("\r\n");
				AES_ECB_Encryption(g_private_AES_key, (u8*)get_IMEI_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				break;
			}
		    case GET_LOCK_SN_CMD://0x05 0x41
			{
				if(!gap_check_token_is_met(decrypted_data+4))
				{
					break;
				}
				printf("GET_LOCK_SN_CMD.\r\n");
				get_SN_ack_t* get_SN_ack = (get_SN_ack_t*)p;
				get_SN_ack->tokenack0 = 0x05;
				get_SN_ack->tokenack1 = 0x42;
				get_SN_ack->L = LOCK_SN_LEN;
				memcpy(get_SN_ack->SN, g_lock_SN, LOCK_SN_LEN);
				printf("rsponse data: ");foreach(i, 16){PrintHex(*((u8*)get_SN_ack+i));}printf("\r\n");
				AES_ECB_Encryption(g_private_AES_key, (u8*)get_SN_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				break;
			}
		    case GET_LOCK_DOMAINS_CMD://0x05 0x30
			{
				if(!gap_check_token_is_met(decrypted_data+4))
				{
					break;
				}
				Flag.is_return_domain_via_ble = 1;
				printf("GET_LOCK_DOMAINS_CMD.\r\n");
				break;
			}
		    case OPEN_THE_LOCK_CMD: //0x05 0x01
			{
				if(!gap_check_token_is_met(decrypted_data+9))
				{
					printf("token is 0x%x  0x%x  0x%x  0x%x.\r\n",(*(u8 *)(decrypted_data+9)),(*(u8 *)(decrypted_data+10)),(*(u8 *)(decrypted_data+11)),(*(u8 *)(decrypted_data+12)));
					break;
				}
				printf("OPEN_THE_LOCK_CMD.\r\n");
				if(
					(nv_params_ptr->lock_on_pwd[0] != *((u8 *)(decrypted_data+3)))||
					(nv_params_ptr->lock_on_pwd[1] != *((u8 *)(decrypted_data+4)))||
					(nv_params_ptr->lock_on_pwd[2] != *((u8 *)(decrypted_data+5)))||
					(nv_params_ptr->lock_on_pwd[3] != *((u8 *)(decrypted_data+6)))||
					(nv_params_ptr->lock_on_pwd[4] != *((u8 *)(decrypted_data+7)))||
					(nv_params_ptr->lock_on_pwd[5] != *((u8 *)(decrypted_data+8)))
				  )
				{
					printf("LOCK on key fail.\r\n");
					break;
				}
				if(device_state.lock_onoff_state == lock_onoff_state_on)
				{
					printf("LOCK is on state.\r\n");
					ble_return_lock_on_result(0);
				}
				else
				{
					printf("LOCK is open....\r\n");
				    Flag.is_turnon_lock_via_ble = 1;
				}
				break;
			}
		    case SET_CURRENT_TIME_CMD://0x06 03
			{
				if(!gap_check_token_is_met(decrypted_data+7))
				{
					break;
				}
				printf("SET_CURRENT_TIME_CMD.\r\n");
				mytime_struct mytime;
				g_current_time = MAKE_U32(decrypted_data[6], decrypted_data[5], decrypted_data[4], decrypted_data[3]);
				utc_sec_2_mytime(g_current_time, &mytime, 0);
				BJ_Time.year = mytime.nYear;BJ_Time.month = mytime.nMonth;BJ_Time.day = mytime.nDay;
				BJ_Time.hour = mytime.nHour;BJ_Time.minute = mytime.nMin;BJ_Time.second = mytime.nSec;
				printf("Current time: %d-%d-%d-%d-%d-%d.",BJ_Time.year,BJ_Time.month,BJ_Time.day,BJ_Time.hour,BJ_Time.minute,BJ_Time.second);printf("\r\n");
				sync_time_ack_t* sync_time_ack = (sync_time_ack_t*)p;
				sync_time_ack->tokenack0 = 0x06;
				sync_time_ack->tokenack1 = 0x04;
				sync_time_ack->tokenack2 = 0x01;
				sync_time_ack->ret = 0x00;
				AES_ECB_Encryption(g_private_AES_key, (u8*)sync_time_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				break;
			}
		    case GET_LOCK_STATUS_CMD://0x05 0E
			{
				if(!gap_check_token_is_met(decrypted_data+4))
				{
					break;
				}
				printf("GET_LOCK_STATUS_CMD.\n");
				query_lock_status_ack_t* query_lock_status_ack = (query_lock_status_ack_t*)p;
				query_lock_status_ack->tokenack0 = 0x05;
				query_lock_status_ack->tokenack1 = 0x0f;
				query_lock_status_ack->tokenack2 = 0x01;
				query_lock_status_ack->sta = device_state.lock_onoff_state;//STA 为锁状态，00 表示开启状态，01 表示关闭状态。
				AES_ECB_Encryption(g_private_AES_key, (u8*)query_lock_status_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				break;
			}
#if 0
		    case CHANGE_PASSWORD_CMD1://0x05 03
			{
				printf("CHANGE_PASSWORD_CMD1.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[12], decrypted_data[11], decrypted_data[10], decrypted_data[9]);
				memcpy(g_old_password, decrypted_data+3, 6);//OLDPWD 为旧密码，NEWPWD 为新密码。
				//if(g_session_id == get_token){//?
				//	chg_pwd_cmd1 = 1;
				//}
				break;
			}
			case CHANGE_PASSWORD_CMD2://0x05 04
			{
				printf("CHANGE_PASSWORD_CMD2.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[12], decrypted_data[11], decrypted_data[10], decrypted_data[9]);
				memcpy(g_new_password, decrypted_data+3, 6);//OLDPWD 为旧密码，NEWPWD 为新密码。
				//if(g_session_id == get_token){//?
				//	if(chg_pwd_cmd1 == 1){
				//        chg_pwd_cmd1 = 0;
						modify_pwd_ack_t* modify_pwd_ack = (modify_pwd_ack_t*)p;
						modify_pwd_ack->tokenack0 = 0x05;
						modify_pwd_ack->tokenack1 = 0x05;
						modify_pwd_ack->tokenack2 = 0x01;
						modify_pwd_ack->RET = 0x00;//RET 为状态返回，00 表示修改密码成功，01 表示修改密码失败。
						AES_ECB_Encryption(g_private_AES_key, (u8*)modify_pwd_ack, encrypted_data);
						bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//  }
				//}
						break;
			}
#endif
			default:
			{
				break;
			}
		}
#if 0
		memcpy((u8*)&cmdOpCode, decrypted_data, 4);
		memset((void*)p, 0, sizeof(rf_packet_att_write_t));
		printf("|    decrypted_data: ");foreach(i, 16){PrintHex(*((u8*)decrypted_data+i));}printf("|\r\n");
		/////////////////////////////////// 4字节命令解析 /////////////////////////////////////////
		switch(CMD_LEN4BYTES(cmdOpCode))
		{
			case GET_LOCK_TOKEN_CMD://get Token CMD
			{
				printf("GET_LOCK_TOKEN_CMD.\r\n");
				token_packet_ack_t* token_ack = (token_packet_ack_t*)p;
				token_ack->tokenack0 = 0x06;
				token_ack->tokenack1 = 0x02;
				token_ack->len = 0x09;
				token_ack->curToken = g_session_id;
				token_ack->chipType = 0x01;
				token_ack->ver_major = (VER&0xF0)>>4;
				token_ack->ver_minor = (VER&0x0F);
				token_ack->productID_IDL = IDL;
				token_ack->productID_IDH = IDH;
				printf("|    rsponse data: ");foreach(i, 16){PrintHex(*((u8*)token_ack+i));}printf("|\n");
				AES_ECB_Encryption(g_private_AES_key, (u8*)token_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				break;
			}
			case GET_LOCK_TIME_CMD://Query lock time
			{
				printf("GET_LOCK_TIME_CMD.\n");
				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//? 关锁的时候，如果连接是断开的，那么主机无法正常收到关锁通知，这时可以主动去查询最后一次关锁的时间??
					query_lock_time_ack_t* query_lock_time_ack = (query_lock_time_ack_t*)p;
					query_lock_time_ack->tokenack0 = 0x06;
					query_lock_time_ack->tokenack1 = 0x06;
					query_lock_time_ack->tokenack2 = 0x06;
					query_lock_time_ack->serialNum = g_serialNum;
					query_lock_time_ack->lastLockTime = g_locked_time;

					AES_ECB_Encryption(g_private_AES_key, (u8*)query_lock_time_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case GET_INT_EXT_ELECTRICITY_CMD: //Get internal and external electricity
			{
				printf("GET_INT_EXT_ELECTRICITY_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//?
					get_int_ext_electricity_ack_t* get_int_ext_electricity_ack = (get_int_ext_electricity_ack_t*)p;
					get_int_ext_electricity_ack->tokenack0 = 0x02;
					get_int_ext_electricity_ack->tokenack1 = 0x02;
					get_int_ext_electricity_ack->tokenack2 = 0x02;
					get_int_ext_electricity_ack->vl = 2;//电压值volt的长度,这里默认2
					get_int_ext_electricity_ack->volt = g_curr_vol;
					get_int_ext_electricity_ack->tl = 2;//温度值temp的长度,这里默认2
					get_int_ext_electricity_ack->temp = g_curr_temp;
					get_int_ext_electricity_ack->cl = 2;//充电电压charge的长度,这里默认2
					get_int_ext_electricity_ack->charge = g_curr_charge_vol;
					AES_ECB_Encryption(g_private_AES_key, (u8*)get_int_ext_electricity_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case GET_LOCK_STATUS_CMD://Query lock status
			{
				printf("GET_LOCK_STATUS_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//?
					query_lock_status_ack_t* query_lock_status_ack = (query_lock_status_ack_t*)p;
					query_lock_status_ack->tokenack0 = 0x05;
					query_lock_status_ack->tokenack1 = 0x0f;
					query_lock_status_ack->tokenack2 = 0x01;
					query_lock_status_ack->sta = g_curr_lock_state;//STA 为锁状态，00 表示开启状态，01 表示关闭状态。
					AES_ECB_Encryption(g_private_AES_key, (u8*)query_lock_status_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case SET_RETURN_CAR_CONDITION_CMD://set return car condition
			{
				printf("SET_RETURN_CAR_CONDITION_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//?
					set_return_car_status_ack_t* set_return_car_status_ack = (set_return_car_status_ack_t*)p;
					set_return_car_status_ack->tokenack0 = 0x05;
					set_return_car_status_ack->tokenack1 = 0x15;
					set_return_car_status_ack->tokenack2 = 0x01;
					set_return_car_status_ack->RET = 0x00;//RET 为状态返回，00 表示设置成功，01 表示设置失败
					AES_ECB_Encryption(g_private_AES_key, (u8*)set_return_car_status_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case GET_LOCK_WORK_PATTERN_CMD://Query lock work pattern
			{
				printf("GET_LOCK_WORK_PATTERN_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//?
					get_lock_work_pattern_ack_t* get_lock_work_pattern_ack = (get_lock_work_pattern_ack_t*)p;
					get_lock_work_pattern_ack->tokenack0 = 0x05;
					get_lock_work_pattern_ack->tokenack1 = 0x20;
					get_lock_work_pattern_ack->tokenack2 = 0x01;
					get_lock_work_pattern_ack->MOD = g_curr_lock_work_pattern;//MOD 为锁工作模式，00 表示正常模式，01 表示运输模式
					AES_ECB_Encryption(g_private_AES_key, (u8*)get_lock_work_pattern_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case GET_LOCK_WORKING_STATUS_CMD://Inquire about the working state of the lock
			{
				printf("GET_LOCK_WORKING_STATUS_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//?
					get_lock_work_status_ack_t* get_lock_work_status_ack = (get_lock_work_status_ack_t*)p;
					get_lock_work_status_ack->tokenack0 = 0x05;
					get_lock_work_status_ack->tokenack1 = 0x22;
					get_lock_work_status_ack->tokenack2 = 0x08;
					get_lock_work_status_ack->R.R0.lock_switch_status = g_curr_lock_state;
					get_lock_work_status_ack->R.R0.vib_func = g_curr_status_vib_func;
					get_lock_work_status_ack->R.R0.vib_state = g_curr_status_vib_status;
					get_lock_work_status_ack->R.R0.chg_dischg_state = g_curr_chg_dischg_state;
					get_lock_work_status_ack->R.R1_GSM_status = g_GSM_status;
					get_lock_work_status_ack->R.R2_last_GPRS_online_need_time = g_last_GPRS_online_need_time;
					get_lock_work_status_ack->R.R3_last_GPRS_rssi_val = g_last_GPRS_rssi_val;
					get_lock_work_status_ack->R.R4_GPS_status = g_GPS_status;
					get_lock_work_status_ack->R.R5_last_GPS_location_need_time = g_last_GPS_location_need_time;
					get_lock_work_status_ack->R.R6_last_GPS_stars_received_num = g_last_GPS_stars_received_num;
					get_lock_work_status_ack->vl = 2;//电压值volt的长度,这里默认2
					get_lock_work_status_ack->volt = g_curr_vol;
					get_lock_work_status_ack->tl = 2;//温度值temp的长度,这里默认2
					get_lock_work_status_ack->temp = g_curr_temp;
					AES_ECB_Encryption(g_private_AES_key, (u8*)get_lock_work_status_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case GET_LOCK_GSM_ID_CMD://Query lock domains
			{
				printf("GET_LOCK_GSM_ID_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//?
					get_GSM_ID_ack_t* get_GSM_ID_ack = (get_GSM_ID_ack_t*)p;
					get_GSM_ID_ack->tokenack0 = 0x05;
					get_GSM_ID_ack->tokenack1 = 0x23;
					get_GSM_ID_ack->tokenack2 = 0x06;
					memcpy(get_GSM_ID_ack->GSM_ID, g_GSM_ID, 6);
					AES_ECB_Encryption(g_private_AES_key, (u8*)get_GSM_ID_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case GET_LOCK_GSM_VER_CMD:
			{
				printf("GET_LOCK_GSM_VER_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//?
					get_GSM_VER_ack_t* get_GSM_VER_ack = (get_GSM_VER_ack_t*)p;
					get_GSM_VER_ack->tokenack0 = 0x05;
					get_GSM_VER_ack->tokenack1 = 0x24;
					get_GSM_VER_ack->tokenack2 = 0x01;
					get_GSM_VER_ack->N = GSM_VER_LEN;
					memcpy(get_GSM_VER_ack->ver, g_GSM_ver, GSM_VER_LEN);
					AES_ECB_Encryption(g_private_AES_key, (u8*)get_GSM_VER_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case GET_LOCK_DOMAINS_CMD:
			{
				printf("GET_LOCK_DOMAINS_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//?
				    get_lock_domains_ack_t* get_lock_domains_ack = (get_lock_domains_ack_t*)p;
				    get_lock_domains_ack->tokenack0 = 0x05;
				    get_lock_domains_ack->tokenack1 = 0x30;
				    get_lock_domains_ack->T = LOCK_DOMAIN_LEN;
				    memcpy(get_lock_domains_ack->DOMAIN, g_lock_domains, LOCK_DOMAIN_LEN);
					AES_ECB_Encryption(g_private_AES_key, (u8*)get_lock_domains_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

#if 0
			case START_OTA_CMD:
			{
				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//?

				//}
			}
			break;
#endif

			default:
			{
				printf("not 4bytes cmd.\n");
				cmd_len = 3;//不是4bytes cmd
			}
		}//the end of switch(CMD_LEN4BYTES(cmdOpCode))

		if(cmd_len == 4)return 0;

		/////////////////////////////////// 3字节命令解析 /////////////////////////////////////////
		switch(CMD_LEN3BYTES(cmdOpCode))
		{

		    //static u8 chg_pwd_cmd1 = 0;
		    static u8 chg_key_cmd1 = 0;

			case GET_SYNC_TIME_CMD://get Time
			{
				printf("GET_SYNC_TIME_CMD.\n");

				g_current_time = MAKE_U32(decrypted_data[6], decrypted_data[5], decrypted_data[4], decrypted_data[3]);
				//u32 get_token =  MAKE_U32(decrypted_data[10], decrypted_data[9], decrypted_data[8], decrypted_data[7]);

				//if(g_session_id == get_token){
					sync_time_ack_t* sync_time_ack = (sync_time_ack_t*)p;
					sync_time_ack->tokenack0 = 0x06;
					sync_time_ack->tokenack1 = 0x04;
					sync_time_ack->tokenack2 = 0x01;
					sync_time_ack->ret = 0x00;
				//}
				//else{
				//	sync_time_ack->ret = 0x01;
				//}
				AES_ECB_Encryption(g_private_AES_key, (u8*)sync_time_ack, encrypted_data);
				bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
			}
			break;

			case GET_CHRG_VOL_TEMP_CMD://Get the battery voltage and temperature
			{
				printf("GET_CHRG_VOL_TEMP_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);

				//if(g_session_id == get_token){//?
					get_vol_temp_ack_t* get_vol_temp_ack = (get_vol_temp_ack_t*)p;
					get_vol_temp_ack->tokenack0 = 0x02;
					get_vol_temp_ack->tokenack1 = 0x02;
					get_vol_temp_ack->tokenack2 = 0x01;
					get_vol_temp_ack->vl = 2;//电压值volt的长度,这里默认2
					get_vol_temp_ack->volt = g_curr_vol;
					get_vol_temp_ack->tl = 2;//温度值temp的长度,这里默认2
					get_vol_temp_ack->temp = g_curr_temp;

					AES_ECB_Encryption(g_private_AES_key, (u8*)get_vol_temp_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case OPEN_THE_LOCK_CMD: //open the lock
			{
				printf("OPEN_THE_LOCK_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[12], decrypted_data[11], decrypted_data[10], decrypted_data[9]);
				memcpy(g_password, decrypted_data+3, 6);
				//if(g_session_id == get_token){//
					open_lock_ack_t* open_lock_ack = (open_lock_ack_t*)p;
					open_lock_ack->tokenack0 = 0x05;
					open_lock_ack->tokenack1 = 0x02;
					open_lock_ack->tokenack2 = 0x01;
					open_lock_ack->RET = 0;//RET 为状态返回，00 表示开锁成功，01 表示开锁失败。
					open_lock_ack->tl = 7;//TL为时间长度，其中年份占2个字节，其余月日时分秒各一个字节
					open_lock_ack->rtc = GET_RTC_VAL();

					AES_ECB_Encryption(g_private_AES_key, (u8*)open_lock_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case SERIAL_NUM_LOCK_CMD:
			{
				printf("SERIAL_NUM_LOCK_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[14], decrypted_data[13], decrypted_data[12], decrypted_data[11]);
				memcpy(g_password, decrypted_data+3, 6);
				memcpy((u8*)&g_serial_num, decrypted_data+9, 2);
				//if(g_session_id == get_token){//?
					serial_number_lock_ack_t* serial_number_lock_ack = (serial_number_lock_ack_t*)p;
					serial_number_lock_ack->tokenack0 = 0x05;
					serial_number_lock_ack->tokenack1 = 0x32;
					serial_number_lock_ack->tokenack2 = 0x01;
					//TODO: 获取开锁成功or失败的信息
					serial_number_lock_ack->RET = g_curr_lock_state;//RET 为状态返回，00 表示开锁成功，01 表示开锁失败。

					AES_ECB_Encryption(g_private_AES_key, (u8*)serial_number_lock_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case ORDER_NUM_UNLOCK_CMD:
			{
				printf("ORDER_NUM_UNLOCK_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[12], decrypted_data[11], decrypted_data[10], decrypted_data[9]);
				memcpy(g_password, decrypted_data+3, 6);
				//if(g_session_id == get_token){//?
					continue_cmd_flg = 1;
				//}
				//else{
				//	order_num_unlock_ack_t* order_num_unlock_ack = (order_num_unlock_ack_t*)p;
				//	order_num_unlock_ack->tokenack0 = 0x05;
				//	order_num_unlock_ack->tokenack1 = 0x13;
				//	order_num_unlock_ack->tokenack2 = 0x01;
				//	order_num_unlock_ack->RET = 1;//RET 为状态返回，00 表示开锁成功，01 表示开锁失败。
				//	order_num_unlock_ack->tl = 7;//TL为时间长度，其中年份占2个字节，其余月日时分秒各一个字节
				//	order_num_unlock_ack->rtc = GET_RTC_VAL();
                //
				//	AES_ECB_Encryption(g_private_AES_key, (u8*)order_num_unlock_ack, encrypted_data);
				//	bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}
			}
			break;

			case CHANGE_PASSWORD_CMD1://change password
			{
				printf("CHANGE_PASSWORD_CMD1.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[12], decrypted_data[11], decrypted_data[10], decrypted_data[9]);
				memcpy(g_old_password, decrypted_data+3, 6);//OLDPWD 为旧密码，NEWPWD 为新密码。
				//if(g_session_id == get_token){//?
				//	chg_pwd_cmd1 = 1;
				//}
			}
			break;

			case CHANGE_PASSWORD_CMD2://change password
			{
				printf("CHANGE_PASSWORD_CMD2.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[12], decrypted_data[11], decrypted_data[10], decrypted_data[9]);
				memcpy(g_new_password, decrypted_data+3, 6);//OLDPWD 为旧密码，NEWPWD 为新密码。
				//if(g_session_id == get_token){//?
				//	if(chg_pwd_cmd1 == 1){
				//        chg_pwd_cmd1 = 0;
						modify_pwd_ack_t* modify_pwd_ack = (modify_pwd_ack_t*)p;
						modify_pwd_ack->tokenack0 = 0x05;
						modify_pwd_ack->tokenack1 = 0x05;
						modify_pwd_ack->tokenack2 = 0x01;
						modify_pwd_ack->RET = 0x00;//RET 为状态返回，00 表示修改密码成功，01 表示修改密码失败。
						AES_ECB_Encryption(g_private_AES_key, (u8*)modify_pwd_ack, encrypted_data);
						bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//  }
				//}
			}
			break;

			case MODIFY_KEY_CMD1://Modify key1
			{
				printf("MODIFY_KEY_CMD1.\n");

				memcpy(g_new_key, decrypted_data+3, 8);//KEYL 为新密钥前 8 个字节,KEYH 为新密钥后 8 个字节，密钥字节按照小端模式排列。
				chg_key_cmd1 = 1;
			}
			break;

			case MODIFY_KEY_CMD2://Modify key2
			{
				if(chg_key_cmd1 == 1){

					printf("MODIFY_KEY_CMD2.\n");

					chg_key_cmd1 = 0;
					memcpy(g_new_key+8, decrypted_data+3, 8);//KEYL 为新密钥前 8 个字节,KEYH 为新密钥后 8 个字节，密钥字节按照小端模式排列。

					modify_key_ack_t* modify_key_ack = (modify_key_ack_t*)p;
					modify_key_ack->tokenack0 = 0x07;
					modify_key_ack->tokenack1 = 0x03;
					modify_key_ack->tokenack2 = 0x01;

					if(save_param_nv(LOCK_AES_KEY_ADR, g_new_key, 16)){
						modify_key_ack->RET = 0x00;//RET 为状态返回，00 表示修改密钥成功，01 表示修改密钥失败;
					}
					else{
						modify_key_ack->RET = 0x01;//RET 为状态返回，00 表示修改密钥成功，01 表示修改密钥失败;
					}

					AES_ECB_Encryption(g_private_AES_key, (u8*)modify_key_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);

					//update aes_key
					if(!modify_key_ack->RET){
						u8 lock_key[16];//data len should NOT bigger then PARAM_NV_PDU_UNIT
						extern u8 g_private_AES_key[16];
						if(load_param_from_flash(LOCK_AES_KEY_ADR, lock_key, 16)){
							memcpy(g_private_AES_key, lock_key, 16);
						}
					}
				}
			}
			break;

			case SET_LOCK_WORK_PATTERN_CMD://set lock mode of operation
			{
				printf("SET_LOCK_WORK_PATTERN_CMD.\n");

				//u32 get_token =  MAKE_U32(decrypted_data[7], decrypted_data[6], decrypted_data[5], decrypted_data[4]);
				memcpy(&g_curr_lock_work_pattern, decrypted_data+3, 1);//MOD 为锁工作模式，00 表示正常模式，01 表示运输模式，02 表示锁重启。
				//if(g_session_id == get_token){//?
				    if(g_curr_lock_work_pattern == 0x02){
				    	cpu_reboot();
				    }

					set_lock_work_pattern_ack_t* set_lock_work_pattern_ack = (set_lock_work_pattern_ack_t*)p;
					set_lock_work_pattern_ack->tokenack0 = 0x05;
					set_lock_work_pattern_ack->tokenack1 = 0x21;
					set_lock_work_pattern_ack->tokenack2 = 0x01;

					if(save_param_nv(SET_LOCK_WORK_MOD_ADR, &g_curr_lock_work_pattern, 1)){
						set_lock_work_pattern_ack->RET = 0x00;//RET 为状态返回，00 表示设置成功，01 表示设置失败。
					}
					else{
						set_lock_work_pattern_ack->RET = 0x01;//RET 为状态返回，00 表示设置成功，01 表示设置失败。
					}

					AES_ECB_Encryption(g_private_AES_key, (u8*)set_lock_work_pattern_ack, encrypted_data);
					bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);
				//}

			}
			break;

			default:{
				printf("not 3byts cmd.\n");
				cmd_len = 2;//不是3byts cmd
			}
		}//the end of switch(CMD_LEN3BYTES(cmdOpCode))

		if(continue_cmd_flg && (cmd_len == 2))
		{
			switch(CMD_LEN2BYTES(cmdOpCode)){
				case ORDER_NUM_UNLOCK_CONTINUE_CMD:
				{
					printf("ORDER_NUM_UNLOCK_CONTINUE_CMD.\n");

					cnt++;//第一次cnt=1;

					order_transmit_seq = decrypted_data[3];//CI为发送序列的序号

					if(cnt == 1){//记录第一笔2bytecmd数据中的订单号总长度信息+CI
						order_len = decrypted_data[2];//T为订单号总长度
						tmp_order_len = order_len;
						memcpy(order, decrypted_data+4, (order_len > 12) ? 12 : order_len);
						(order_len > 12) ? (order_len -= 12) : (continue_cmd_flg = 0, cnt = 0);
					}
					else{
						memcpy(order+12*(cnt-1), decrypted_data+4, (order_len > 12) ? 12 : order_len);
						(order_len > 12) ? (order_len -= 12) : (continue_cmd_flg = 0, cnt = 0);
					}

					if(!continue_cmd_flg){

						order_num_unlock_ack_t* order_num_unlock_ack = (order_num_unlock_ack_t*)p;
						order_num_unlock_ack->tokenack0 = 0x05;
						order_num_unlock_ack->tokenack1 = 0x13;
						order_num_unlock_ack->tokenack2 = 0x01;
						order_num_unlock_ack->RET = 0;//RET 为状态返回，00 表示开锁成功，01 表示开锁失败。
						order_num_unlock_ack->tl = 7;//TL为时间长度，其中年份占2个字节，其余月日时分秒各一个字节
						order_num_unlock_ack->rtc = GET_RTC_VAL();

						AES_ECB_Encryption(g_private_AES_key, (u8*)order_num_unlock_ack, encrypted_data);
						bls_att_pushNotifyData(BleLockChar2DataHdl, encrypted_data, 16);

						g_is_order_num_unlock = 1;//订单号开锁成功。
					}
				}
				break;

				default:{
					printf("CMD do not understand!.\n");
					cmd_len = 1;//CMD do not understand!
				}
			}
		}
#endif
	}//the end of if(len == 16)
	else
	{
		printf("write data len not 16.\n");
	}

	return 0;
}


/////////////////////////////////////////ota/////////////////////////////////////
#if (TELIK_OTA_SERVICE_ENABLE)
const u8	my_OtaServiceUUID[16]		= TELINK_OTA_UUID_SERVICE;
const u8	my_OtaUUID[16]		= TELINK_SPP_DATA_OTA;

static u8	my_OtaProp		= CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP;
const u8 	my_OtaName[] = {'O', 'T', 'A'};
u8	 		my_OtaData 		= 0x00;
#endif



// TM : to modify
const attribute_t my_Attributes[] =
{
#if SUPPORT_WETCHAT_ENABLE//if support wechat,
	#if (TELIK_OTA_SERVICE_ENABLE)
		{28,0,0,0,0,0},	// total num of attribute
	#else
		{24,0,0,0,0,0},	// total num of attribute
	#endif
#else
	#if (TELIK_OTA_SERVICE_ENABLE)
		{21,0,0,0,0,0},	// total num of attribute
	#else
		{17,0,0,0,0,0},	// total num of attribute
	#endif
#endif

	// 0001 - 0007  gap
	{7,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_gapServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&my_characterUUID), 		(u8*)(&my_devNameCharacter), 0},
	{0,ATT_PERMISSIONS_READ,2,MAX_DEV_NAME_LEN, (u8*)(&my_devNameUUID), (u8*)(&ble_devName), 0},
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&my_characterUUID), 		(u8*)(&my_appearanceCharacter), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof (my_appearance), (u8*)(&my_appearanceUIID), 	(u8*)(&my_appearance), 0},
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&my_characterUUID), 		(u8*)(&my_periConnParamChar), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof (my_periConnParameters),(u8*)(&my_periConnParamUUID), 	(u8*)(&my_periConnParameters), 0},


	// 0008 - 000b gatt
	{4,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_gattServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&my_characterUUID), 		(u8*)(&serviceChangedProp), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof (serviceChangeVal), (u8*)(&serviceChangeUIID), 	(u8*)(&serviceChangeVal), 0},
	{0,ATT_PERMISSIONS_RDWR,2,sizeof (indCharCfg),(u8*)(&clientCharacterCfgUUID), (u8*)(indCharCfg), 0},


	////////////////////////////////////// BleLock Service /////////////////////////////////////////////////////
#if SUPPORT_WETCHAT_ENABLE//if support wechat,
	{13,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&BleLockServiceUUID), 0},
#else
	{6,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&BleLockServiceUUID), 0},
#endif

	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&my_characterUUID), 		(u8*)(&BleLockChar1Prop), 0},				//prop
	{0,ATT_PERMISSIONS_RDWR,2,sizeof(BleLockChar1Data),(u8*)(&BleLockChar1UUID), (u8*)(BleLockChar1Data),  (att_readwrite_callback_t)&sprocomm_lock_write_evt_handler},	//value

	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&my_characterUUID), 		(u8*)(&BleLockChar2Prop), 0},				//prop
	{0,ATT_PERMISSIONS_READ,2,sizeof(BleLockChar2Data),(u8*)(&BleLockChar2UUID), (u8*)(BleLockChar2Data), 0},	//value
	{0,ATT_PERMISSIONS_RDWR,2,2,(u8*)&clientCharacterCfgUUID,(u8*)(&BleLockChar2DataCCC)},

#if SUPPORT_WETCHAT_ENABLE//if support wechat,
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&my_characterUUID), 		(u8*)(&BleLockChar3Prop), 0},				//prop
	{0,ATT_PERMISSIONS_READ,2,sizeof(BleLockChar3Data),(u8*)(&BleLockChar3UUID), (u8*)(BleLockChar3Data), 0},	//value

	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&my_characterUUID), 		(u8*)(&BleLockChar4Prop), 0},				//prop
	{0,ATT_PERMISSIONS_READ,2,sizeof(BleLockChar4Data),(u8*)(&BleLockChar4UUID), (u8*)(BleLockChar4Data), 0},	//value
	{0,ATT_PERMISSIONS_RDWR,2,2,(u8*)&clientCharacterCfgUUID,(u8*)(&BleLockChar4DataCCC)},

	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&my_characterUUID), 		(u8*)(&BleLockChar5Prop), 0},				//prop
	{0,ATT_PERMISSIONS_READ,2,sizeof(BleLockChar5Data),(u8*)(&BleLockChar5UUID), (u8*)(BleLockChar5Data), 0},	//value
#endif

#if (TELIK_OTA_SERVICE_ENABLE)
	// OTA
	{4,ATT_PERMISSIONS_READ, 2,16,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_OtaServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ, 2, 1,(u8*)(&my_characterUUID), 		(u8*)(&my_OtaProp), 0},				//prop
	{0,ATT_PERMISSIONS_RDWR,16,sizeof(my_OtaData),(u8*)(&my_OtaUUID),	(&my_OtaData), &otaWrite, &otaRead},			//value
	{0,ATT_PERMISSIONS_READ, 2,sizeof (my_OtaName),(u8*)(&userdesc_UUID), (u8*)(my_OtaName), 0},
#endif
};
/*******************************************************
gatt initialization
*******************************************************/
void my_att_init(void)
{
	bls_att_setAttributeTable((u8 *)my_Attributes);
	u8 device_name[] = DEV_NAME;
	bls_att_setDeviceName(device_name, sizeof(DEV_NAME));
}

#endif
