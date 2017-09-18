/*
 * Buzzer.h
 *
 *  Created on: 2017-9-18
 *      Author: Kyle
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#include "../app_config.h"
#include "../../proj/tl_common.h"
#include "../Module_Communicate/gprs_gps_module_communicate.h"

#define  BUZZER_TURNON_LOCK_BUZZ_NUMBER        3
#define  BUZZER_TURNON_LOCK_BUZZ_DELAY         (100/GPRS_GPS_MODULE_COMMUNICATE_INTERVAL)
#define  BUZZER_TURNOFF_LOCK_BUZZ_DELAY        (40/GPRS_GPS_MODULE_COMMUNICATE_INTERVAL)

typedef enum
{
	BUZZER_SCHEDULE_START = 0,
	BUZZER_BUZZ_ON        = 1,
	BUZZER_BUZZ_OFF       = 2,
	BUZZER_SCHEDULE_STOP =3
}Buzzer_buzz_schedule_t;
#endif /* BUZZER_H_ */
