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
#define GET_INT_EXT_ELECTRICITY_CMD         (0x0102)
#define START_OTA_CMD                       (0x01010103)
#define OPEN_THE_LOCK_CMD                   (0x0105)
#define MODIFY_LOCKON_PASSWORD_CMD1         (0x0305)     //change password1
#define MODIFY_LOCKON_PASSWORD_CMD2         (0x0405)     //change password2
#define CLOSE_THE_LOCK_CMD                  (0x0c05)
#define GET_LOCK_STATUS_CMD                 (0x0e05)     //Query lock status
#define ORDER_NUM_UNLOCK_CMD                (0x1105)
#define ORDER_NUM_UNLOCK_RECEIVE_ORDER_CMD  (0x1205)
#define SET_RETURN_CAR_STATUS_CMD           (0x1405)
#define GET_LOCK_WORK_PATTERN_CMD           (0x2005)     //Query lock work pattern
#define SET_LOCK_WORK_PATTERN_CMD           (0x2105)     //set lock mode of operation
#define GET_LOCK_WORKING_STATUS_CMD         (0x2205)     //Inquire about the working state of the lock
#define GET_LOCK_GSM_ID_CMD                 (0x2305)
#define GET_LOCK_GSM_VER_CMD                (0x2405)
#define GET_LOCK_ICCID_CMD                  (0x2805)
#define GET_LOCK_IMEI_CMD                   (0x2905)
#define GET_LOCK_DOMAINS_CMD                (0x3005)     //Query lock domains
#define SERIAL_NUM_LOCK_CMD                 (0x3105)
#define GET_LOCK_SN_CMD                     (0x4105)

#define CMD_LEN3BYTES(i)                    (i & 0x00ffffff)

#define GET_LOCK_TOKEN_CMD                  (0x0106)
#define SET_CURRENT_TIME_CMD                (0x0306)
#define MODIFY_AES_PASSWORD_CMD1            (0x0107) //Modify key1
#define MODIFY_AES_PASSWORD_CMD2            (0x0207) //Modify key2
#define CMD_LEN2BYTES(i)                    (i & 0x0000ffff)//spectial cmd followed the cmd: ORDER_NUM_UNLOCK_CMD


#endif /* SPP_H_ */
