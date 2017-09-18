/*
 * time_stamp.c
 *
 *  Created on: 2017-9-16
 *      Author: Kyle
 */
#include "time_stamp.h"
/* ÿ���µ����� */
const unsigned char g_day_per_mon[MONTH_PER_YEAR] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
/*
 * ���ܣ�
 *     �ж��Ƿ�������
 * ������
 *     year����Ҫ�жϵ������
 *
 * ����ֵ��
 *     ���귵��1�����򷵻�0
 */
unsigned char applib_dt_is_leap_year(unsigned short year)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if ((year % 400) == 0) {
        return 1;
    } else if ((year % 100) == 0) {
        return 0;
    } else if ((year % 4) == 0) {
        return 1;
    } else {
        return 0;
    }
}
/*
 * ���ܣ�
 *     �õ�ÿ�����ж�����
 * ������
 *     month����Ҫ�õ��������·���
 *     year����������Ӧ�������
 *
 * ����ֵ��
 *     �����ж�����
 *
 */
unsigned char applib_dt_last_day_of_mon(unsigned char month, unsigned short year)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if ((month == 0) || (month > 12)) {
        return g_day_per_mon[1] + applib_dt_is_leap_year(year);
    }

    if (month != 2) {
        return g_day_per_mon[month - 1];
    } else {
        return g_day_per_mon[1] + applib_dt_is_leap_year(year);
    }
}
/*
 * ���ܣ�
 *     ���ݸ��������ڵõ���Ӧ������
 * ������
 *     year�����������
 *     month���������·�
 *     day������������
 *
 * ����ֵ��
 *     ��Ӧ����������0 - ������ ... 6 - ������
 */
unsigned char applib_dt_dayindex(unsigned short year, unsigned char month, unsigned char day)
{
    char century_code, year_code, month_code, day_code;
    int week = 0;

    century_code = year_code = month_code = day_code = 0;

    if (month == 1 || month == 2) {
        century_code = (year - 1) / 100;
        year_code = (year - 1) % 100;
        month_code = month + 12;
        day_code = day;
    } else {
        century_code = year / 100;
        year_code = year % 100;
        month_code = month;
        day_code = day;
    }

    /* ���ݲ��չ�ʽ�������� */
    week = year_code + year_code / 4 + century_code / 4 - 2 * century_code + 26 * ( month_code + 1 ) / 10 + day_code - 1;
    week = week > 0 ? (week % 7) : ((week % 7) + 7);

    return week;
}
/*
 * ���ܣ�
 *     ����ʱ������UTCʱ���
 * ������
 *     currTime��������ʱ��
 *     daylightSaving���Ƿ�������ʱ
 *
 * ����ֵ��
 *     UTCʱ���
 */
unsigned int mytime_2_utc_sec(mytime_struct *currTime, bool daylightSaving)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    unsigned short i;
    unsigned int no_of_days = 0;
    int utc_time;
    unsigned char dst;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if (currTime->nYear < UTC_BASE_YEAR) {
        return 0;
    }

    /* year */
    for (i = UTC_BASE_YEAR; i < currTime->nYear; i++) {
        no_of_days += (DAY_PER_YEAR + applib_dt_is_leap_year(i));
    }

    /* month */
    for (i = 1; i < currTime->nMonth; i++) {
        no_of_days += applib_dt_last_day_of_mon((unsigned char) i, currTime->nYear);
    }

    /* day */
    no_of_days += (currTime->nDay - 1);

    /* sec */
    utc_time = (unsigned int) no_of_days * SEC_PER_DAY + (unsigned int) (currTime->nHour * SEC_PER_HOUR +
                                                                currTime->nMin * SEC_PER_MIN + currTime->nSec);

    if (dst && daylightSaving) {
        utc_time -= SEC_PER_HOUR;
    }

    return utc_time;
}
/*
 * ���ܣ�
 *     ����UTCʱ����õ���Ӧ������
 * ������
 *     utc_sec��������UTCʱ���
 *     result��������Ľ��
 *     daylightSaving���Ƿ�������ʱ
 *
 * ����ֵ��
 *     ��
 */
void utc_sec_2_mytime(unsigned int utc_sec, mytime_struct *result, bool daylightSaving)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int sec, day;
    unsigned short y;
    unsigned char m;
    unsigned short d;
    unsigned char dst;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/

    if (daylightSaving) {
        utc_sec += SEC_PER_HOUR;
    }

    /* hour, min, sec */
    /* hour */
    sec = utc_sec % SEC_PER_DAY;
    result->nHour = sec / SEC_PER_HOUR;

    /* min */
    sec %= SEC_PER_HOUR;
    result->nMin = sec / SEC_PER_MIN;

    /* sec */
    result->nSec = sec % SEC_PER_MIN;

    /* year, month, day */
    /* year */
    /* year */
    day = utc_sec / SEC_PER_DAY;
    for (y = UTC_BASE_YEAR; day > 0; y++) {
        d = (DAY_PER_YEAR + applib_dt_is_leap_year(y));
        if (day >= d)
        {
            day -= d;
        }
        else
        {
            break;
        }
    }

    result->nYear = y;

    for (m = 1; m < MONTH_PER_YEAR; m++) {
        d = applib_dt_last_day_of_mon(m, y);
        if (day >= d) {
            day -= d;
        } else {
            break;
        }
    }

    result->nMonth = m;
    result->nDay = (unsigned char) (day + 1);
    /* ���ݸ��������ڵõ���Ӧ������ */
    result->DayIndex = applib_dt_dayindex(result->nYear, result->nMonth, result->nDay);
}

