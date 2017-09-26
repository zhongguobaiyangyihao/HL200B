/*
 * gprs_gps_module_communicate.h
 *
 *  Created on: 2017-9-13
 *      Author: Administrator
 */

#ifndef GPRS_GPS_MODULE_COMMUNICATE_H_
#define GPRS_GPS_MODULE_COMMUNICATE_H_

#define GPRS_GPS_MODULE_COMMUNICATE_INTERVAL             500//ms
#define UART_RX_BUF_FROMMODULE_SIZE                      200
#define UART_TX_BUF_TOMODULE_SIZE                        200
#define WAIT_TIME_ONE_SECOND          (1000/GPRS_GPS_MODULE_COMMUNICATE_INTERVAL)//S
#define WAIT_TIME_TWO_SECOND          (2000/GPRS_GPS_MODULE_COMMUNICATE_INTERVAL)
#define WAIT_TIME_THREE_SECOND        (3000/GPRS_GPS_MODULE_COMMUNICATE_INTERVAL)
#define WAIT_TIME_SHORT               (5000/GPRS_GPS_MODULE_COMMUNICATE_INTERVAL)
#define WAIT_TIME_MIDDLE              (10000/GPRS_GPS_MODULE_COMMUNICATE_INTERVAL)

/////////////////////HCI UART variables///////////////////////////////////////
#define UART_TX_BUF_TOMODULE_SIZE    64      // data max 252
typedef struct
{
    unsigned int len;        // data max 252
    unsigned char data[UART_TX_BUF_TOMODULE_SIZE];

}uart_data_t;

typedef enum
{
	module_turnon_pwron = 0,
	module_turnon_check = 1
}module_turnon_schedule_t;

typedef enum
{
  MODULE_IDLE = 0,
  MODULE_TURNON = 1,
}MODULE_schedule_t;

typedef enum
{
  Waiting_for_return = 0,
  Config_success,
  Config_failure
}MODULE_TRANSREC_RETURN_RESUL_t;

typedef enum
{
	module_transrec_idle = 0,
	module_transrec_transmit = 1,
	module_transrec_receive  = 2
}module_transmit_receive_schedule_t;
#endif /* GPRS_GPS_MODULE_COMMUNICATE_H_ */
