
#include "nv.h"

//////////////////////////////////// flash NV management ////////////////////////////////////
//Notice:flash_erase_sector should place at user_init(), Do Not use this function in main_loop()!!!
// FLASH struct : | index | 00 | data[PARAM_NV_UNIT - 2] |
/**********************************************************************
Get the current IDX value stored in the current flash specified area
/**********************************************************************/
int get_current_flash_idx(u32 addr)
{
	int idx=0;
	for (idx=0; idx < 4*1024; idx+= PARAM_NV_UNIT)//4K per sector
	{
		u16 read_from_flash;
		flash_read_page((addr+idx), 2, (u8*)&read_from_flash);
		if (read_from_flash == U16_MAX)//end
		{
			break;
		}
	}
	idx -= PARAM_NV_UNIT;
	printf("<get_current_flash_idx>idx = 0x%x.\r\n",idx);
	if(idx < 0)
	{
		printf("<get_current_flash_idx>empty.\r\n");
		printf("<get_current_flash_idx>return value 0x%x.\r\n",-1);
		return -1;//empty
	}
	else if(idx >= PARAM_NV_MAX_IDX)
	{
		printf("full. \n");
		return 0xffff;//full
	}
	else
	{
		printf("current idx:0x%x. \n", idx);
		return idx;//current idx num
	}
}
/**********************************************************************
Get the parameters of the current IDX store from the flash
/**********************************************************************/
#if 0
u8 load_param_from_flash(u32 addr, u8* p, u8 len)
{
	int currIdx = get_current_flash_idx(addr);
	printf("<load_param_from_flash>currIdx = 0x%x.\r\n", currIdx);
    if( currIdx >= 0 && currIdx <= (PARAM_NV_MAX_IDX - PARAM_NV_UNIT))
    {
    	if(len > (PARAM_NV_UNIT-2))
    	{//len check
    		return 0;//failed
    	}

    	//Copy the parameters corresponding to the current flash in IDX to p
    	memcpy (p, (u32 *)(addr+currIdx+2), len);
    	printf("<load_param_from_flash>return value %d.\r\n", 1);
    	return 1;//succeed
	}
    //The current flash area is empty or full
    printf("<load_param_from_flash>return value %d.\r\n", 0);
    return 0;//failed
}
#else
u8 load_param_from_flash(u32 addr, u8* p, u8 len)
{
  if((addr%FLASH_PAGE_SIZE) != 0)//page align
  {
	  return -1;
  }
  for(u32 i=0;i<len;)
  {
	  if((len-i) >= FLASH_PAGE_SIZE)
		  flash_read_page(addr+i, FLASH_PAGE_SIZE, p+i);
	  else
		  flash_read_page(addr+i, len, p+i);
	  i += FLASH_PAGE_SIZE;
  }
  return 0;
}
#endif
/**********************************************************************
Determines whether the flash sector for the specified region needs to be erased (4K)
/**********************************************************************/
#if 0
void param_clear_flash(u32 addr)
{
	//get index in flash
	int currIdx = get_current_flash_idx(addr);
	printf("<param_clear_flash>currIdx = 0x%x.\r\n",currIdx);
	if(currIdx == 0xffff)
	{//full
		//The current flash zone is full
		nv_manage_t p;
		memcpy (p.data, (u32*)(addr + PARAM_NV_MAX_IDX + 2), PARAM_NV_UNIT - 2);
        //Erase the current flash region
		flash_erase_sector(addr);//erase
		//Save the last data in the flsh area to the current flash first address area
		save_param_nv(addr, (u8*)&p.data, PARAM_NV_UNIT - 2);
	}
}
#else
u8 param_clear_flash(u32 addr)
{
	if((addr%FLASH_SECTOR_SIZE) != 0)//sector align
	{
		return -1;
	}
	flash_erase_sector(addr);//erase
	return 0;
}
#endif
/**********************************************************************
Store user data into the current IDX region in flash
/**********************************************************************/
#if 0
u8 save_param_nv(u32 addr, u8* buf, u16 len)
{
	//get index in flash
	int currIdx = get_current_flash_idx(addr);

	if(currIdx == 0xffff)
	{//full
		//The current flash zone is full
		return 0;//failed
	}

	if(len > (PARAM_NV_UNIT - 2)){
		//Save parameter length too long
		return 0;//failed
	}

	//                     0:   start idx
	//  4096 - PARAM_NV_UNIT:   full idx
	//4096 - 2*PARAM_NV_UNIT:   full idx - PARAM_NV_UNIT
	if(currIdx >= 0  && currIdx <= (PARAM_NV_MAX_IDX - PARAM_NV_UNIT))
	{
		u8 clr[2] = {0};
		flash_write_page(addr + currIdx, 2, clr);
		printf("Erase flash prefix header锛�x0000. \n");
	}

	currIdx = PARAM_NV_UNIT + ((currIdx == -1) ? -PARAM_NV_UNIT : currIdx);

	printf("Update currIdx: 0x%x. \n", currIdx);

	nv_manage_t p;
	p.curNum = 0x01;
	p.tmp    = 0xFF;

	memcpy(p.data, buf, len);

	flash_write_page((addr + currIdx), len + 2, (u8*)&p);

	return 1;
}
#else
u8 save_param_nv(u32 addr, u8* buf, u16 len)
{
	u8 error_code = 0;
	error_code = param_clear_flash(addr);
	if(error_code != 0)
	{
		return error_code;
	}
	for(u32 i=0;i<len;)
	{
	  if((len-i) >= FLASH_PAGE_SIZE)
		  flash_write_page(addr+i, FLASH_PAGE_SIZE, buf+i);
	  else
		  flash_write_page(addr+i, len, buf+i);
	  i += FLASH_PAGE_SIZE;
	}
	return 0;
}
#endif
