/*
 * slip_protocol.h
 *
 *  Created on: 2017-9-15
 *      Author: Administrator
 */

#ifndef SLIP_PROTOCOL_H_
#define SLIP_PROTOCOL_H_

/************************************************************************************************/
#define SLIP_RX_BUFFER_SIZE        (50+11)
#define SLIP_TX_BUFFER_SIZE        (50+11)

#define SLIP_END                   (0xC0)
#define SLIP_ESC                   (0xDB)
#define SLIP_ESC_END               (0xDC)
#define SLIP_ESC_ESC               (0xDD)

#define SLIP_STATE_FLAG_NONE       (0x00)
#define SLIP_STATE_FLAG_GET_ESC    (0x01)

#define PROTOCOL_HEAD_ID       (0x10)
#define CLIENT_ID              (0x0001)

#define BTC_CONTACT_TOTAL      (0xAC)
#define BTC_TOTAL_DEVINFO      (0x2001)

#define BTC_CONTACT            (0x01)
#define BTC_INFO               (0x0001)
#define BTC_IDLE               (0x0002)
#define BTC_UNIT               (0x0003)
#define BTC_DATA               (0x0004)
#define BTC_AUTH               (0x0005)

#define BTC_WATCH_COMMUNICATE (((PROTOCOL_HEAD_ID&0x00FE)<<8)|(0x0010))

#define BTC_DISCONNECT         (0x0101)
#define BTC_CONNECT            (0x0102)
#define BTC_ATR                (0x0103)
#define BTC_APDU               (0x0104)
#define BTC_PPS                (0x0105)


#define BTC_TEST_M1            (0xE1E2)
#define BTC_TEST_M2            (0xE1E3)
#define BTC_TEST_M3            (0xE1E4)

#define BTC_IO_OK              (0x0000)
#define BTC_ILLEAGAL_CMD       (0x0001)
#define BTC_IO_TIMEOUT         (0x0002)
#define BTC_IO_ERROR           (0x0003)
#define BTC_IO_BUSY            (0x0004)
#define BTC_ILLEAGLE_STATUS    (0x0005)
#define BTC_SSC_ERROR          (0x0006)
#define BTC_MODE_ERROR         (0x0007)
#define BTC_UNBIND_ERROR       (0x0008)
#endif /* SLIP_PROTOCOL_H_ */
