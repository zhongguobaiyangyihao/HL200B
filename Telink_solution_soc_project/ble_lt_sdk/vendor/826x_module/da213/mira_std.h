/*******************************************************************************/
 /**
  ******************************************************************************
  * @file    mira_std.h
  * @author  ycwang@miramems.com
  * @version V1.0
  * @date    26-Nov-2014
  * @brief
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, MiraMEMS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2014 MiraMEMS</center></h2>
  */
/*******************************************************************************/
#ifndef __mira_std_h
#define __mira_std_h

/*****************************************************
	ADC value configure of 0g 1g and -1g in X,Y,Z axis
******************************************************/				
#define ACC_0G_X		(2059)
#define ACC_1G_X		(2059+128)
#define ACC_MINUS1G_X	(2059+128)

#define ACC_0G_Y		(2059)
#define ACC_1G_Y		(2059+128)
#define ACC_MINUS1G_Y	(2059+128)

#define ACC_0G_Z		(2059)
#define ACC_1G_Z		(2059+128)
#define ACC_MINUS1G_Z	(2059+128)

#define DA213_CHIP_ID                   0x13
                                                 
#define NSA_REG_SPI_I2C                 0x00
#define NSA_REG_WHO_AM_I                0x01
#define NSA_REG_ACC_X_LSB               0x02
#define NSA_REG_ACC_X_MSB               0x03
#define NSA_REG_ACC_Y_LSB               0x04
#define NSA_REG_ACC_Y_MSB               0x05
#define NSA_REG_ACC_Z_LSB               0x06
#define NSA_REG_ACC_Z_MSB               0x07
#define NSA_REG_MOTION_FLAG				0x09
#define NSA_REG_G_RANGE                 0x0f
#define NSA_REG_ODR_AXIS_DISABLE        0x10
#define NSA_REG_POWERMODE_BW            0x11
#define NSA_REG_SWAP_POLARITY           0x12
#define NSA_REG_FIFO_CTRL               0x14
#define NSA_REG_INTERRUPT_SETTINGS1     0x16
#define NSA_REG_INTERRUPT_SETTINGS2     0x17
#define NSA_REG_INTERRUPT_MAPPING1      0x19
#define NSA_REG_INTERRUPT_MAPPING2      0x1a
#define NSA_REG_INTERRUPT_MAPPING3      0x1b
#define NSA_REG_INT_PIN_CONFIG          0x20
#define NSA_REG_INT_LATCH               0x21
#define NSA_REG_FREEFALL_DUR            0x22
#define NSA_REG_FREEFALL_THS            0x23
#define NSA_REG_FREEFALL_HYST           0x24
#define NSA_REG_ACTIVE_DURATION         0x27
#define NSA_REG_ACTIVE_THRESHOLD        0x28
#define NSA_REG_TAP_DURATION            0x2A
#define NSA_REG_TAP_THRESHOLD           0x2B
#define NSA_REG_CUSTOM_OFFSET_X         0x38
#define NSA_REG_CUSTOM_OFFSET_Y         0x39
#define NSA_REG_CUSTOM_OFFSET_Z         0x3a
#define NSA_REG_ENGINEERING_MODE        0x7f
#define NSA_REG_SENSITIVITY_TRIM_X      0x80
#define NSA_REG_SENSITIVITY_TRIM_Y      0x81
#define NSA_REG_SENSITIVITY_TRIM_Z      0x82
#define NSA_REG_COARSE_OFFSET_TRIM_X    0x83
#define NSA_REG_COARSE_OFFSET_TRIM_Y    0x84
#define NSA_REG_COARSE_OFFSET_TRIM_Z    0x85
#define NSA_REG_FINE_OFFSET_TRIM_X      0x86
#define NSA_REG_FINE_OFFSET_TRIM_Y      0x87
#define NSA_REG_FINE_OFFSET_TRIM_Z      0x88
#define NSA_REG_SENS_COMP               0x8c
#define NSA_REG_SENS_COARSE_TRIM        0xd1

#endif
