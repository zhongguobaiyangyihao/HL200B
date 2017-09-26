/*
 * emi.h
 *
 *  Created on: 2017-8-23
 *      Author: Administrator
 */

#ifndef EMI_H_
#define EMI_H_
extern int pnGen(int state);
extern int Rf_EmiInit(void);
extern void	Rf_EmiCarrierOnlyTest(int power_level,signed char rf_chn);
extern void	Rf_EmiCarrierDataTest(int power_level,signed char rf_chn);
extern void Rf_EmiDataUpdate(void);
extern void Rf_EmiRxTest(unsigned char *addr,signed char rf_chn,signed char buffer_size,unsigned char  pingpong_en);
extern void Rf_EmiTxInit(int power_level,signed char rf_chn);
extern void Rf_EmiSingleTx(unsigned char *addr);
#endif /* EMI_H_ */

