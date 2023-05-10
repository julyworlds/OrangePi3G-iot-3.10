/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 09 10 2010 jackie.su
 * [ALPS00002279] [Need Patch] [Volunteer Patch] ALPS.Wxx.xx Volunteer patch for
 * .10y dual sensor
 *
 * 09 02 2010 jackie.su
 * [ALPS00002279] [Need Patch] [Volunteer Patch] ALPS.Wxx.xx Volunteer patch for
 * .roll back dual sensor
 *
 * Mar 4 2010 mtk70508
 * [DUMA00154792] Sensor driver
 * 
 *
 * Feb 24 2010 mtk01118
 * [DUMA00025869] [Camera][YUV I/F & Query feature] check in camera code
 * 
 *
 * Aug 5 2009 mtk01051
 * [DUMA00009217] [Camera Driver] CCAP First Check In
 * 
 *
 * Apr 7 2009 mtk02204
 * [DUMA00004012] [Camera] Restructure and rename camera related custom folders and folder name of came
 * 
 *
 * Mar 26 2009 mtk02204
 * [DUMA00003515] [PC_Lint] Remove PC_Lint check warnings of camera related drivers.
 * 
 *
 * Mar 2 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Feb 24 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Dec 27 2008 MTK01813
 * DUMA_MBJ CheckIn Files
 * created by clearfsimport
 *
 * Dec 10 2008 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Oct 27 2008 mtk01051
 * [DUMA00000851] Camera related drivers check in
 * Modify Copyright Header
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H


typedef enum {
    SENSOR_MODE_INIT = 0,
    SENSOR_MODE_PREVIEW,
    SENSOR_MODE_CAPTURE,
    SENSOR_MODE_VIDEO
} ISX012MIPI_SENSOR_MODE;

typedef enum _ISX012MIPI_OP_TYPE_ {
        ISX012MIPI_MODE_NONE,
        ISX012MIPI_MODE_PREVIEW,
        ISX012MIPI_MODE_CAPTURE,
        ISX012MIPI_MODE_QCIF_VIDEO,
        ISX012MIPI_MODE_CIF_VIDEO,
        ISX012MIPI_MODE_QVGA_VIDEO
    } ISX012MIPI_OP_TYPE;

extern ISX012MIPI_OP_TYPE ISX012MIPI_g_iISX012MIPI_Mode;

#define ISX012MIPI_ID_REG                          (0x300A)
#define ISX012MIPI_INFO_REG                        (0x300B)
 
/* sensor size */
#define ISX012MIPI_IMAGE_SENSOR_SVGA_WIDTH          (640)
#define ISX012MIPI_IMAGE_SENSOR_SVGA_HEIGHT         (480)
#define ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH         (2560)//(2592) 
#define ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT        (1920)//(1944)
#define ISX012MIPI_IMAGE_SENSOR_VIDEO_WITDH         (1280) 
#define ISX012MIPI_IMAGE_SENSOR_VIDEO_HEIGHT        (720)


/* Sesnor Pixel/Line Numbers in One Period */	
///////#define ISX012MIPI_PV_PERIOD_PIXEL_NUMS    		(1896)  	/* Default preview line length HTS*/
///////#define ISX012MIPI_PV_PERIOD_LINE_NUMS     		(984)   	/* Default preview frame length  VTS*/
///////#define ISX012MIPI_FULL_PERIOD_PIXEL_NUMS  		(2844)  	/* Default full size line length */
///////#define ISX012MIPI_FULL_PERIOD_LINE_NUMS   		(1968)  	/* Default full size frame length */
///////#define ISX012MIPI_VIDEO_PERIOD_PIXEL_NUMS    	(1896)  	/* Default preview line length HTS*/
///////#define ISX012MIPI_VIDEO_PERIOD_LINE_NUMS     	(984)   	/* Default preview frame length  VTS*/

/* Sensor Exposure Line Limitation */
///////#define ISX012MIPI_PV_EXPOSURE_LIMITATION      	(984-4)
///////#define ISX012MIPI_FULL_EXPOSURE_LIMITATION    	(1968-4)

/* Config the ISP grab start x & start y, Config the ISP grab width & height */
#define ISX012MIPI_PV_GRAB_START_X 				   (0)
#define ISX012MIPI_PV_GRAB_START_Y  			(0)
#define ISX012MIPI_FULL_GRAB_START_X   			(0)
#define ISX012MIPI_FULL_GRAB_START_Y	  		(0)

/*50Hz,60Hz*/
#define ISX012MIPI_NUM_50HZ                        (50 * 2)
#define ISX012MIPI_NUM_60HZ                        (60 * 2)

/* FRAME RATE UNIT */
#define ISX012MIPI_FRAME_RATE_UNIT                 (10)

/* MAX CAMERA FRAME RATE */
#define ISX012MIPI_MAX_CAMERA_FPS                  (ISX012MIPI_FRAME_RATE_UNIT * 30)

#define ISX012_PREVIEW_MODE             0
#define ISX012_VIDEO_MODE               1
#define ISX012_PREVIEW_FULLSIZE_MODE    2


/* SENSOR READ/WRITE ID */
#define ISX012MIPI_WRITE_ID						    0x34
#define ISX012MIPI_READ_ID							0x35
#define ISX012MIPI_WRITE_ID1					    0x78
#define ISX012MIPI_READ_ID1							0x79

UINT32 ISX012MIPIopen(void);
UINT32 ISX012MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 ISX012MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 ISX012MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 ISX012MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 ISX012MIPIClose(void);
UINT32 ISX012MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT pfFunc);
#endif /* __SENSOR_H */
