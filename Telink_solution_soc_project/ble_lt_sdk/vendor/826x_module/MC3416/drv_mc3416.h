/*******************************************************************************
 * Copyright (c) 2015, mCube, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of Bosch Sensortec GmbH nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*******************************************************************************
 * REVISON HISTORY
 -------------------------------------------------------------------------------
 * VERSION      DATE            WHO        DESCRIPTION
 -------------------------------------------------------------------------------
 * 1.3.0        08/27/2015      Louis      Initial version.
 * 1.3.2		09/25/2015		Kevin	   Adjust the order in sns_dd_mc34xx_enable_sched_data(), 
 										   sns_ddf_signal_register() first, and then drv_mc34xx_set_dri().
********************************************************************************/


#ifndef _DRV_MC34XX_H
#define _DRV_MC34XX_H


/*============================================================================
DEFINITIONS
============================================================================*/

/***********************************************
 *** INFORMATION
 ***********************************************/
#define MC34XX_VERSION    "1.3.2"

/***********************************************
 *** COMMON
 ***********************************************/
#define MC34XX_RESOLUTION_LOW     (1)
#define MC34XX_RESOLUTION_HIGH    (2)

#define MC34XX_SENSOR_DATA_SIZE_RES_LOW     (3)
#define MC34XX_SENSOR_DATA_SIZE_RES_HIGH    (6)

#define MC34XX_RANGE_MAX(range)     ( range * G)
#define MC34XX_RANGE_MIN(range)     (-range * G)

#define MC34XX_RESOLUTION(range, counts)     ((range * G) / counts)

#define MC34XX_AXIS_X      (0)
#define MC34XX_AXIS_Y      (1)
#define MC34XX_AXIS_Z      (2)
#define MC34XX_AXES_NUM    (3)

#define MC34XX_REGMAP_SIZE    (80)

/***********************************************
 *** REGISTER MAP
 ***********************************************/
#define MC34XX_REG_XOUT                    			0x00
#define MC34XX_REG_YOUT                    			0x01
#define MC34XX_REG_ZOUT                    			0x02

#define MC34XX_REG_INTERRUPT_ENABLE    				0x06
#define MC34XX_REG_MODE_FEATURE            			0x07
#define MC34XX_REG_SAMPLE_RATE             			0x08

#define MC34XX_REG_XOUT_EX_L               			0x0D
#define MC34XX_REG_XOUT_EX_H		            	0x0E
#define MC34XX_REG_YOUT_EX_L        		    	0x0F
#define MC34XX_REG_YOUT_EX_H               			0x10
#define MC34XX_REG_ZOUT_EX_L		            	0x11
#define MC34XX_REG_ZOUT_EX_H        		    	0x12

#define MC34XX_REG_CHIPID							0x18
#define MC34XX_REG_LPF_RANGE_RES          			0x20
#define MC34XX_REG_MCLK_POLARITY          			0x2A
#define MC34XX_REG_PRODUCT_CODE_H      				0x34
#define MC34XX_REG_PRODUCT_CODE_L      				0x3B



#ifndef MENSA    
    #define MENSA		//MENSA = MC3416, 3436
    
#define MC34X6_REG_STATUS_1 						0x03
#define MC34X6_REG_INTR_STAT_1						0x04
#define MC34X6_REG_DEVICE_STATUS					0x05
                                          		
#define MC34X6_REG_MOTION_CTRL		   				0x09
#define MC34X6_REG_STATUS_2 						0x13
#define MC34X6_REG_INTR_STAT_2						0x14
#define MC34X6_REG_SDM_X							0x15
                                          		
#define MC34X6_REG_RANGE_CONTROL          			0x20
#define MC34X6_REG_MISC 							0x2C
                                          		
#define MC34X6_REG_TF_THRESHOLD_LSB 				0x40
#define MC34X6_REG_TF_THRESHOLD_MSB 				0x41
#define MC34X6_REG_TF_DB							0x42
#define MC34X6_REG_AM_THRESHOLD_LSB 				0x43
#define MC34X6_REG_AM_THRESHOLD_MSB 				0x44
#define MC34X6_REG_AM_DB							0x45
#define MC34X6_REG_SHK_THRESHOLD_LSB				0x46
#define MC34X6_REG_SHK_THRESHOLD_MSB				0x47
#define MC34X6_REG_PK_P2P_DUR_THRESHOLD_LSB 		0x48
#define MC34X6_REG_PK_P2P_DUR_THRESHOLD_MSB 		0x49
#define MC34X6_REG_TIMER_CTRL						0x4A
#endif

/***********************************************
 *** MODE
 ***********************************************/
#define MC34XX_MODE_STANDBY    (0x03)
#define MC34XX_MODE_WAKE       (0x01)
#define MC34XX_MODE_SLEEP      (0x00)

#define MC34XX_MODE_POLLING    (0x00)
#define MC34XX_MODE_INTERRUPT  (0x80)

/***********************************************
 *** INTERRUP
 ***********************************************/
#define MC34XX_INT_IPP_OPEN_DRAIN    (0x00)
#define MC34XX_INT_IPP_PUSH_PULL     (0x40)

#define MC34XX_INT_IAH_ACTIVE_LOW     (0x00)
#define MC34XX_INT_IAH_ACTIVE_HIGH    (0x80)
/***********************************************
 *** PRODUCT ID
 ***********************************************/
#define MC3416_CHIP_ID        0xA0
#define MC34XX_PCODE_3413     0x10
#define MC34XX_PCODE_3433     0x60
#define MC34XX_PCODE_3416     0x20
#define MC34XX_PCODE_3436     0x21
/*============================================================================
TYPE / DATA STRUCTURE DEFINITIONS
============================================================================*/
typedef struct
{
    float    fData[MC34XX_AXES_NUM];
}   mc34xx_data_t;

#endif /* End include guard  _DRV_MC34XX_H */

