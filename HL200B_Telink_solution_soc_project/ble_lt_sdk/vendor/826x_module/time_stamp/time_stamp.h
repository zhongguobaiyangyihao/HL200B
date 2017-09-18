/*
 * time_stamp.h
 *
 *  Created on: 2017-9-16
 *      Author: Administrator
 */

#ifndef TIME_STAMP_H_
#define TIME_STAMP_H_
#include "../../proj/tl_common.h"

#define UTC_BASE_YEAR 1970
#define MONTH_PER_YEAR 12
#define DAY_PER_YEAR 365
#define SEC_PER_DAY 86400
#define SEC_PER_HOUR 3600
#define SEC_PER_MIN 60
/* 自定义的时间结构体 */
typedef struct
{
    unsigned short nYear;
    unsigned char nMonth;
    unsigned char nDay;
    unsigned char nHour;
    unsigned char nMin;
    unsigned char nSec;
    unsigned char DayIndex; /* 0 = Sunday */
} mytime_struct;

unsigned int mytime_2_utc_sec(mytime_struct *currTime, bool daylightSaving);
void utc_sec_2_mytime(unsigned int utc_sec, mytime_struct *result, bool daylightSaving);
#endif /* TIME_STAMP_H_ */
