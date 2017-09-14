/*
 * nv.h
 *
 *  Created on: 2017-8-25
 *      Author: Administrator
 */

#ifndef NV_H_
#define NV_H_

#include "../../proj/tl_common.h"


#define PARAM_NV_PDU_UNIT       			(30)
#define PARAM_NV_UNIT						(PARAM_NV_PDU_UNIT + 2)//header(2B) + userData(30B)
#define PARAM_NV_MAX_IDX        			(4096 - PARAM_NV_UNIT)

#define FLASH_PAGE_SIZE                     256
#define FLASH_SECTOR_SIZE                   4096

typedef struct{//used to indicate para address index
	unsigned char  curNum;
	unsigned char  tmp;
	unsigned char  data[PARAM_NV_PDU_UNIT];
}nv_manage_t;

////////////////////////////////// FLASH Management ////////////////////////////////////////////
int get_current_flash_idx(u32 addr);//Gets the current IDX value stored in the current flash specified area
u8 load_param_from_flash(u32 addr, u8* p, u8 len);//Get the parameters of the current IDX store from the flash
u8 param_clear_flash(u32 addr);//Determines whether the flash sector for the specified region needs to be erased (4K)
u8 save_param_nv(u32 addr, u8* buf, u16 len);//Store user data into the current IDX region in flash


#endif /* NV_H_ */
