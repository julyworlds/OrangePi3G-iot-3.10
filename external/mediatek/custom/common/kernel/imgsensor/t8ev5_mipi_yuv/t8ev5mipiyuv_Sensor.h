/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/*****************************************************************************

 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H

typedef enum
{
  T8EV5_MODE_PREVIEW,  //1M  	1280x960
  T8EV5_MODE_CAPTURE_5M,   //5M    2592x1944
  T8EV5_MODE_CAPTURE_OTHERS   //< pv size 
} T8EV5_MODE;

typedef enum group_enum {
    PRE_GAIN=0,
    CMMCLK_CURRENT,
    FRAME_RATE_LIMITATION,
    REGISTER_EDITOR,
    GROUP_TOTAL_NUMS
} FACTORY_GROUP_ENUM;


#define ENGINEER_START_ADDR 10
#define FACTORY_START_ADDR 0

typedef enum engineer_index
{
    CMMCLK_CURRENT_INDEX=ENGINEER_START_ADDR,
    ENGINEER_END
} FACTORY_ENGINEER_INDEX;



typedef enum register_index
{
    PRE_GAIN_INDEX=FACTORY_START_ADDR,
    GLOBAL_GAIN_INDEX,
    FACTORY_END_ADDR
} FACTORY_REGISTER_INDEX;

typedef struct
{
    SENSOR_REG_STRUCT	Reg[ENGINEER_END];
    SENSOR_REG_STRUCT	CCT[FACTORY_END_ADDR];
} SENSOR_DATA_STRUCT, *PSENSOR_DATA_STRUCT;

struct T8EV5YUV_sensor_struct
{
	kal_uint32 sceneMode;	   
	T8EV5_MODE Camco_mode;		
	kal_uint32	iShutter;
	kal_uint32	iGain;
	kal_uint32	d_gain;
	kal_uint32 awbMode;
	kal_bool userAskAeLock;
	kal_bool userAskAwbLock;
	kal_uint16 auto_iso;
	kal_uint16 isoSpeed;
	kal_uint32 capExposureTime;
};


#define CURRENT_MAIN_SENSOR                T8EV5_OMNIVISION


//Macro for Resolution
#define T8EV5_IMAGE_SENSOR_CCT_WIDTH     (2596)
#define T8EV5_IMAGE_SENSOR_CCT_HEIGHT    (1950)

/* SENSOR VGA SIZE */
#define T8EV5_IMAGE_SENSOR_PV_WIDTH   (1280)//(640)
#define T8EV5_IMAGE_SENSOR_PV_HEIGHT  (960)//(480)

/* SENSOR 5M SIZE */
#define T8EV5_IMAGE_SENSOR_FULL_WIDTH                                       (2560)
#define T8EV5_IMAGE_SENSOR_FULL_HEIGHT                                      (1920)

#define T8EV5_IMAGE_SENSOR_PV_STARTX                         4
#define T8EV5_IMAGE_SENSOR_PV_STARTY                         3

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
#define T8EV5_FULL_PERIOD_PIXEL_NUMS  (2592)  // 5M mode's pixel # in one HSYNC w/o dummy pixels
#define T8EV5_FULL_PERIOD_LINE_NUMS    (1944)  // 5M mode's HSYNC # in one HSYNC w/o dummy lines
#define T8EV5_PV_PERIOD_PIXEL_NUMS      (1280)    // 720P mode's pixel # in one HSYNC w/o dummy pixels
#define T8EV5_PV_PERIOD_LINE_NUMS        (960)   // 720P mode's HSYNC # in one HSYNC w/o dummy lines

#define T8EV5_FULL_PERIOD_EXTRA_PIXEL_NUMS  248 //450
#define T8EV5_FULL_PERIOD_EXTRA_LINE_NUMS    56 //28
#define T8EV5_PV_PERIOD_EXTRA_PIXEL_NUMS      896 //450
#define T8EV5_PV_PERIOD_EXTRA_LINE_NUMS        32

#define T8EV5_FULL_PERIOD_PIXEL_NUMS_HTS  (2808) 
#define T8EV5_FULL_PERIOD_LINE_NUMS_VTS    (1976) 
#define T8EV5_PV_PERIOD_PIXEL_NUMS_HTS      (2176) 
#define T8EV5_PV_PERIOD_LINE_NUMS_VTS        (992)   


#define T8EV5_IMAGE_SENSOR_5M_PIXELS_LINE           T8EV5_FULL_PERIOD_PIXEL_NUMS_HTS
#define T8EV5_IMAGE_SENSOR_720P_PIXELS_LINE        T8EV5_PV_PERIOD_PIXEL_NUMS_HTS


    
#define MAX_FRAME_RATE	(15)
#define MIN_FRAME_RATE  (12)

/* SENSOR EXPOSURE LINE LIMITATION */
#define T8EV5_FULL_EXPOSURE_LIMITATION    (1944-4)  // 5M mode
#define T8EV5_PV_EXPOSURE_LIMITATION  (T8EV5_PV_PERIOD_LINE_NUMS-4)  // # of lines in one 720P frame
// SENSOR VGA SIZE
//For 2x Platform camera_para.c used
#define IMAGE_SENSOR_PV_WIDTH    T8EV5_IMAGE_SENSOR_PV_WIDTH
#define IMAGE_SENSOR_PV_HEIGHT   T8EV5_IMAGE_SENSOR_PV_HEIGHT

#define IMAGE_SENSOR_FULL_WIDTH	  T8EV5_IMAGE_SENSOR_FULL_WIDTH
#define IMAGE_SENSOR_FULL_HEIGHT	  T8EV5_IMAGE_SENSOR_FULL_HEIGHT

#define T8EV5_SHUTTER_LINES_GAP	  0


#define T8EV5_WRITE_ID (0x78)
#define T8EV5_READ_ID	(0x79)

// SENSOR CHIP VERSION

//#define T8EV5_SENSOR_ID            0x5642

#define T8EV5_PAGE_SETTING_REG    (0xFF)

//s_add for porting
//s_add for porting
//s_add for porting

//#define T8EV5_SENSOR_ID    T8EV5_SENSOR_ID
//#define T8EV5_SENSOR_ID1   T8EV5_SENSOR_ID_1
//#define T8EV5_SENSOR_ID2   T8EV5_SENSOR_ID_2
//#define T8EV5_WRITE_ID     T8EV5_WRITE_ID

//export functions
UINT32 T8EV5YUVOpen(void);
UINT32 T8EV5YUVGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 T8EV5YUVGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 T8EV5YUVControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 T8EV5YUVFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 T8EV5YUVClose(void);

//#define Sleep(ms) mdelay(ms)
//#define RETAILMSG(x,...)
//#define TEXT

//e_add for porting
//e_add for porting
//e_add for porting

#endif /* __SENSOR_H */

