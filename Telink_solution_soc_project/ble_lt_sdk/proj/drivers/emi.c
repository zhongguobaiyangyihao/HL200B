/*
 * emi.c
 *
 *  Created on: 2017-8-23
 *      Author: Administrator
 */
#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "emi.h"
unsigned char  emi_var[5];
unsigned char  emi_tx[16]  __attribute__ ((aligned (4))) = {0xc,0x00,0x00,0x00,0x00,0x20,0xaa,0xbb};
int state0,state1,state2,state3;
unsigned char depth=1;
#define STATE0		0x1234
#define STATE1		0x5678
#define STATE2		0xabcd
#define STATE3		0xef01
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) test  initialization
*
*	@param		None
*
*
*	@return		None
*/

int Rf_EmiInit(void)
{
	// for registers recover.
	emi_var[0] = ReadAnalogReg(0xa5);
	emi_var[1] = read_reg8(0x8004e8);
	//emi_var[2] = read_reg8(0x800408);
	emi_var[2] = read_reg8(0x800402);
	emi_var[3] = read_reg8(0x80050f);
	emi_var[4] = read_reg8(0x80050e);

	return 1;

}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) test  recovery setting
*
*	@param		None
*
*
*	@return		None
*/
extern int xtalType_rfMode;
int Rf_EmiCarrierRecovery(void)
{
	//reset zb & dma

	write_reg16(0x800060, 0x0480);
	write_reg16(0x800060, 0x0000);
	WriteAnalogReg (0xa5, emi_var[0]);
    write_reg8 (0x8004e8, emi_var[1]);
    if(( xtalType_rfMode == XTAL_12M_RF_2m_MODE ) || (xtalType_rfMode == XTAL_16M_RF_2m_MODE))
	{
    	write_reg8 (0x800402, 0x26);
	}
	else if(( xtalType_rfMode == XTAL_12M_RF_1m_MODE ) || (xtalType_rfMode == XTAL_16M_RF_1m_MODE))
	{
		write_reg8 (0x800402, 0x26);
	}
//	else if(( xtalType_rfMode == XTAL_12M_RF_250k_MODE ) || (xtalType_rfMode == XTAL_16M_RF_250k_MODE))
//	{
//		write_reg8 (0x800402, 0x26);
//	}

	write_reg8(0x80050f, emi_var[3]);
    write_reg8(0x80050e, emi_var[4]);
    return 1;

}
#if 0
void PhyTest_PRBS9 (unsigned char *p, int n)
{
	//PRBS9: (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100)
	unsigned short x = 0x1ff;
	int i;
	int j;
	for ( i=0; i<n; i++)
	{
		unsigned char d = 0;
		for (j=0; j<8; j++)
		{
			if (x & 1)
			{
				d |= BIT(j);
			}
			x = (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100);
		}
		*p++ = d;
	}
}
#else
//unsigned char prbs9[128];
void PhyTest_PRBS9 (unsigned char *p, int n,unsigned char words)
{
	//PRBS9: (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100)
	unsigned short x = 0x1ff;
	int i;
	int j;
	for ( i=0; i<n; i++)
	{
		unsigned char d = 0;
		for (j=0; j<8; j++)
		{
			if (x & 1)
			{
				d |= BIT(j);
			}
			x = (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100);
		}
		if(( i<((words+1)*4))&&( i>= (words*4)))
		{
			*p++ = d;
		}

	}
}
#endif
int pnGen(int state)
{
	int feed = 0;
	feed = (state&0x4000) >> 1;
	state ^= feed;
	state <<= 1;
	state = (state&0xfffe) + ((state&0x8000)>>15);
	return state;
}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) CarrierOnly Test
*
*	@param		power_level: set power level(0~14)
*				rf_chn	   : set tx channel((0~100))
*
*
*	@return		None
*/
void Rf_EmiCarrierOnlyTest(int power_level,signed char rf_chn)
{
	Rf_EmiCarrierRecovery();
	SetTxMode(rf_chn,0);
	WaitUs(150);//wait pllclock

	rf_set_power_level_index(power_level);
	WriteAnalogReg(0xa5,0x44);   // for carrier  mode
	write_reg8 (0x8004e8, 0x04); // for  carrier mode
}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) CarrierData Test
*
*	@param		power_level: set power level(0~14)
*				rf_chn	   : set tx channel((0 ~ 100))
*
*
*	@return		None
*/

void Rf_EmiCarrierDataTest(int power_level,signed char rf_chn)
{

	int i;

	Rf_EmiCarrierRecovery();

	rf_set_power_level_index(power_level);
	SetTxMode(rf_chn,0);
	WaitUs(150);//wait pllclock

	write_reg8(0x80050e,depth); // this size must small than the beacon_packet dma send length

	state0 = STATE0;
	state1 = STATE1;
	state2 = STATE2;
	state3 = STATE3;
	emi_tx[0] = depth*16-4;
	write_reg8(0x80050f, 0x80);  // must fix to 0x80
	write_reg8(0x800402, 0x21);	//preamble length=1
	TxPkt(emi_tx);
}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) CarrierData Test Data Update
*
*	@param		None
*
*
*	@return		None
*/

void Rf_EmiDataUpdate(void)
{
	//write_reg32((emi_tx+depth*16-4),(state0<<16)+state1); // the last value
//	//advance PN generator
	//state0 = pnGen(state0);
	//state1 = pnGen(state1);
	//write_reg32((emi_tx+depth*16-4),0xf0f0f0f0); // the last value
	//phyTest_PRBS9((emi_tx+12),0x04);
	int i;
	for(i=0;i<64;i++)
	{
		PhyTest_PRBS9((emi_tx+depth*16-4),64,i);
	}


}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) Rx Test
*
*	@param		addr       :set receiving address pointer
*
*				buffer_size: set power level(0~14)
*				rf_chn	   : set tx channel((0~ 100))
*
*
*	@return		None
*/

void Rf_EmiRxTest(unsigned char *addr,signed char rf_chn,signed char buffer_size,unsigned char  pingpong_en)
{

	reg_dma_rf_rx_addr = (u16)(u32) (addr);
	reg_dma2_ctrl = FLD_DMA_WR_MEM | (buffer_size>>4);   // rf rx buffer enable & size
	//Rf_RxBufferSet(addr,buffer_size,pingpong_en);
	SetRxMode(rf_chn,0);
	WaitUs(200);//wait pllclock
	Rf_EmiCarrierRecovery();

	//Rf_BaseBandReset();
}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) Tx Test initialization
*
*	@param		power_level: set power level(0~14)
*				rf_chn	   : set tx channel((0 ~ 100))
*
*
*	@return		None
*/

void Rf_EmiTxInit(int power_level,signed char rf_chn)
{

	rf_set_power_level_index(power_level);
	SetTxMode(rf_chn,0);
	WaitUs(200);//wait pllclock
	//Rf_BaseBandReset();
	Rf_EmiCarrierRecovery();

}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) CarrierData Test
*
*	@param		addr       :set tx address pointer
*
*
*	@return		None
*/

void Rf_EmiSingleTx(unsigned char *addr)
{
	TxPkt(addr);
	while((read_reg8(0x800f20) & 0x02)==0);
	write_reg8 (0x800f20, read_reg8(0x800f20)|0x02);
}


/******************************************************************************
 * 									EMI
******************************************************************************/
void emi_test(void)
{
#if (MCU_CORE_TYPE == MCU_CORE_8266)
	analog_write(0x86, analog_read(0x86) & 0xfe);
#endif

//#if (CLOCK_SYS_CLOCK_HZ == 16000000)
//	write_reg8(0xb2,0x01);
//#endif


	static int first_flg = 0;
	unsigned char  mode=1;
	unsigned char  power_level = 0;
	unsigned char  chn = 2;
	unsigned char cmd_now=1;
	unsigned char run=0;
	unsigned char tx_mode=0;
	unsigned char  *packet;
	//nordic 2M
	unsigned char  nordic_tx_packet[16]  __attribute__ ((aligned (4))) = {0x9,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x34,0x56,0x78,0x12};
	//Ble 1M
	unsigned char  ble_tx_packet[16]  __attribute__ ((aligned (4))) = {0xa,0x00,0x00,0x00,0x00,0x8,0x00,0x00,0x00,0x22,0x33,0x44,0x55,0x66};
	unsigned char  rx_packet[128] __attribute__ ((aligned (4)));
	Rf_EmiInit();
	write_reg8(0x808006,run);//run
	write_reg8(0x808007,cmd_now);//cmd
	write_reg8(0x808008,power_level);//power
	write_reg8(0x808009,chn);//chn
	write_reg8(0x80800a,mode);//mode
	while(1)
	{

	   run = read_reg8(0x808006);  // get the run state!
	   if(!first_flg || run!=0)
	   {
			if(!first_flg){
				first_flg = 1;
			}
			else{
			   power_level = read_reg8(0x808008);
			   chn = read_reg8(0x808009);
			   mode=read_reg8(0x80800a);
			   cmd_now = read_reg8(0x808007);  // get the command!
			   tx_mode	= read_reg8(0x808005);
			}
			if(mode==1)
			{
				 rf_drv_1m();
				 packet = ble_tx_packet;
			}
			else if(mode==0)
			{
				rf_drv_2m();
				packet = nordic_tx_packet;
			}
			if(cmd_now == 0x01)//EmiCarrierOnly
			{
				Rf_EmiCarrierOnlyTest(power_level,chn);
			}
			else if(cmd_now == 0x02)//EmiCarrierData
			{
				unsigned char run = read_reg8(0x808006);  // get the run state!
				unsigned char cmd_now = read_reg8(0x808007) ;
				unsigned char power = read_reg8(0x808008);
				unsigned char chn = read_reg8(0x808009);
				unsigned char mode=read_reg8(0x80800a);

				Rf_EmiCarrierDataTest(power_level,chn);
				while( ((read_reg8(0x808006)) == run ) &&  ((read_reg8(0x808007)) == cmd_now )\
				&& ((read_reg8(0x808008)) == power ) &&  ((read_reg8(0x808009)) == chn )\
				&& ((read_reg8(0x80800a)) == mode )) // if not new command
				{
					Rf_EmiDataUpdate();
				}
			}
			else if(cmd_now == 0x03)//EmiRx
			{
				Rf_EmiRxTest(rx_packet,rf_chn,128,0);
			}
			else if(cmd_now == 0x04)//EmiTx_prbs9
			{
				unsigned char run = read_reg8(0x808006);  // get the run state!
				unsigned char cmd_now = read_reg8(0x808007) ;
				unsigned char power = read_reg8(0x808008);
				unsigned char chn = read_reg8(0x808009);
				unsigned char mode=read_reg8(0x80800a);
				unsigned char tx_mode_temp = read_reg8(0x808005);
				unsigned char rx_now;
				Rf_EmiTxInit(power_level,chn);
				while( ((read_reg8(0x808006)) == run ) &&  ((read_reg8(0x808007)) == cmd_now )\
						&& ((read_reg8(0x808008)) == power ) &&  ((read_reg8(0x808009)) == chn )\
						&& ((read_reg8(0x80800a)) == mode &&  ((read_reg8(0x808005)) == tx_mode_temp ) )) // if not new command
				{
					packet[4] = 0;//type
					phyTest_PRBS9 (packet + 6, 37);
//					WaitUs(20000);
					Rf_EmiSingleTx(packet);
				}

			}
			else if(cmd_now == 0x05)//EmiTx_0x55
				{
					unsigned char run = read_reg8(0x808006);  // get the run state!
					unsigned char cmd_now = read_reg8(0x808007) ;
					unsigned char power = read_reg8(0x808008);
					unsigned char chn = read_reg8(0x808009);
					unsigned char mode=read_reg8(0x80800a);
					unsigned char tx_mode_temp = read_reg8(0x808005);
					unsigned char rx_now;

					Rf_EmiTxInit(power_level,chn);
					while( ((read_reg8(0x808006)) == run ) &&  ((read_reg8(0x808007)) == cmd_now )\
							&& ((read_reg8(0x808008)) == power ) &&  ((read_reg8(0x808009)) == chn )\
							&& ((read_reg8(0x80800a)) == mode &&  ((read_reg8(0x808005)) == tx_mode_temp ))) // if not new command
					{
						int i;
						packet[4] = 2;//type

						for( i=0;i<37;i++)
						{
							packet[6+i]=0x55;
						}
//						WaitUs(20000);
						Rf_EmiSingleTx(packet);

					}

				}
			else if(cmd_now == 0x06)//EmiTx_0xff
				{
					unsigned char run = read_reg8(0x808006);  // get the run state!
					unsigned char cmd_now = read_reg8(0x808007) ;
					unsigned char power = read_reg8(0x808008);
					unsigned char chn = read_reg8(0x808009);
					unsigned char mode=read_reg8(0x80800a);
					unsigned char tx_mode_temp = read_reg8(0x808005);
					unsigned char rx_now;

					Rf_EmiTxInit(power_level,chn);
					while( ((read_reg8(0x808006)) == run ) &&  ((read_reg8(0x808007)) == cmd_now )\
							&& ((read_reg8(0x808008)) == power ) &&  ((read_reg8(0x808009)) == chn )\
							&& ((read_reg8(0x80800a)) == mode&&  ((read_reg8(0x808005)) == tx_mode_temp ) )) // if not new command
					{
						int i;
						packet[4] = 1;//type

						for( i=0;i<37;i++)
						{
							packet[6+i]=0x0f;
						}
//						WaitUs(20000);
						Rf_EmiSingleTx(packet);

					}

				}
			run = 0;
			write_reg8(0x808006, 0);

	   }

	}
}



