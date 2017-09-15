/*
 * spp.h
 *
 *  Created on: 2016-11-3
 *      Author: Administrator
 */

#ifndef SPP_H_
#define SPP_H_

//TODO:根据后续项目要求实现，这里作为功能接口



//这里定义蓝牙锁通信协议，主要用在手机和锁的交互过程中
#define CMD_LEN4BYTES(i)                    (i & 0xffffffff)
#define CMD_LEN2BYTES(i)                    (i & 0xffff)
#define GET_LOCK_TOKEN_CMD                  (0x0106)
#define GET_LOCK_TIME_CMD                   (0x01010506)
#define GET_INT_EXT_ELECTRICITY_CMD         (0x02010102)
#define CLOSE_THE_LOCK_CMD                  (0x01010c05)
#define GET_LOCK_STATUS_CMD                 (0x01010e05) //Query lock status
#define SET_RETURN_CAR_CONDITION_CMD        (0x01011405) //set return car condition
#define GET_LOCK_WORK_PATTERN_CMD           (0x2005)     //Query lock work pattern
#define GET_LOCK_WORKING_STATUS_CMD         (0x2205)     //Inquire about the working state of the lock
#define GET_LOCK_GSM_ID_CMD                 (0x2305)
#define GET_LOCK_GSM_VER_CMD                (0x2405)
#define GET_LOCK_ICCID_CMD                  (0x2805)
#define GET_LOCK_IMEI_CMD                   (0x2905)
#define GET_LOCK_SN_CMD                     (0x4105)
#define GET_LOCK_DOMAINS_CMD                (0x3005) //Query lock domains
#define START_OTA_CMD                       (0x01010103)

#define CMD_LEN3BYTES(i)                    (i & 0x00ffffff)
#define GET_CHRG_VOL_TEMP_CMD               (0x010102)
#define GET_SYNC_TIME_CMD                   (0x040306)
#define OPEN_THE_LOCK_CMD                   (0x0105)
#define SERIAL_NUM_LOCK_CMD                 (0x083105)
#define ORDER_NUM_UNLOCK_CMD                (0x061105)
#define CHANGE_PASSWORD_CMD1                (0x060305) //change password1
#define CHANGE_PASSWORD_CMD2                (0x060405) //change password2
#define MODIFY_KEY_CMD1                     (0x080107) //Modify key1
#define MODIFY_KEY_CMD2                     (0x080207) //Modify key2
#define SET_LOCK_WORK_PATTERN_CMD           (0x012105) //set lock mode of operation

#define CMD_LEN2BYTES(i)                    (i & 0x0000ffff)//spectial cmd followed the cmd: ORDER_NUM_UNLOCK_CMD
#define ORDER_NUM_UNLOCK_CONTINUE_CMD       (0x1205)

#endif /* SPP_H_ */
