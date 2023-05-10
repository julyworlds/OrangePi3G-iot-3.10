/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *============================================================================
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>	 
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
#include "isx012mipi_yuv_Sensor.h"
#include "isx012mipi_yuv_Camera_Sensor_para.h"
#include "isx012mipi_yuv_CameraCustomized.h" 

#define ISX012MIPIYUV_DEBUG
#ifdef ISX012MIPIYUV_DEBUG
#define ISX012MIPISENSORDB printk
#else
#define ISX012MIPISENSORDB(x,...)
#endif
static DEFINE_SPINLOCK(isx012mipi_drv_lock);
static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId) ;
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

extern int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
extern int mt_set_gpio_dir(unsigned long pin, unsigned long dir);
extern int mt_set_gpio_out(unsigned long pin, unsigned long output);


#define ISX012MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,ISX012MIPI_WRITE_ID)
//#define ISX012MIPI_write_cmos_sensor_16(addr, para) iWriteReg((u16) addr , (u32) para ,2,ISX012MIPI_WRITE_ID)
#define ISX012MIPI_write_cmos_sensor1(addr, para) iWriteReg((u16) addr , (u32) para ,1,ISX012MIPI_WRITE_ID1)
void ISX012MIPI_write_cmos_sensor_16(kal_uint32 addr, kal_uint32 para)
{

	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	iWriteRegI2C(puSendCmd , 4,ISX012MIPI_WRITE_ID);
}

void ISX012MIPI_write_cmos_sensor_16_MIPI(kal_uint32 addr, kal_uint32 para)
{

	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	iWriteRegI2C(puSendCmd , 4,ISX012MIPI_WRITE_ID);
}



#define mDELAY(ms)  mdelay(ms)

kal_uint8 ISX012MIPI_sensor_socket = DUAL_CAMERA_NONE_SENSOR;
typedef enum
{
    PRV_W=1280,
    PRV_H=960
}PREVIEW_VIEW_SIZE;
kal_uint16 ISX012MIPIYUV_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,ISX012MIPI_WRITE_ID);
    return get_byte;
}
kal_uint16 ISX012MIPIYUV_read_cmos_sensor1(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,ISX012MIPI_WRITE_ID1);
    return get_byte;
}
static struct
{
	//kal_uint8   Banding;
	kal_bool	  NightMode;
	kal_bool      pv_mode; 
	kal_bool      video_mode; 				
    kal_bool      capture_mode; 				
	kal_uint32    pv_dummy_lines;
	kal_uint32	  pv_dummy_pixels;
	kal_uint32    video_dummy_pixels;
	kal_uint32    video_dummy_lines;
	kal_uint32    cp_dummy_pixels;
	kal_uint32    cp_dummy_lines;

	kal_uint32 pv_line_length;
    kal_uint32 pv_frame_length;
	kal_uint32 video_line_length;
	kal_uint32 video_frame_length;
	kal_uint32 cp_line_length;
	kal_uint32 cp_frame_length;
	
	kal_uint32    PreviewPclk;
	kal_uint32      CapturePclk;
	kal_uint32      VideoPclk;
	
	kal_uint32 		pv_shutter;		   
	kal_uint32 		video_shutter;		   
	kal_uint32 		cp_shutter;
	
	kal_bool    	manualAEStart;
	kal_bool    	userAskAeLock;
    kal_bool    	userAskAwbLock;
	
	kal_uint32      currentExposureTime;	
    kal_uint32      currentAxDGain;
	
	kal_uint32  	sceneMode;
	
    unsigned char isoSpeed;
	
	kal_uint16 af_xcoordinate;
	kal_uint16 af_ycoordinate;
	unsigned char   awbMode;
	ISX012MIPI_SENSOR_MODE SensorMode;
} ISX012MIPISensor;
/* Global Valuable */
static kal_uint32 zoom_factor = 0; 
//static kal_int8 ISX012MIPI_DELAY_AFTER_PREVIEW = -1;
//static kal_uint8 ISX012MIPI_Banding_setting = AE_FLICKER_MODE_50HZ; 
static kal_bool ISX012MIPI_AWB_ENABLE = KAL_TRUE; 
//static kal_bool ISX012MIPI_AE_ENABLE = KAL_TRUE; 
MSDK_SENSOR_CONFIG_STRUCT ISX012MIPISensorConfigData;
#define ISX012_TEST_PATTERN_CHECKSUM (0x7ba87eae)
void ISX012MIPI_set_scene_mode(UINT16 para);
BOOL ISX012MIPI_set_param_wb(UINT16 para);
/*************************************************************************
* FUNCTION
*	ISX012MIPI_set_dummy
*
* DESCRIPTION
*	This function set the dummy pixels(Horizontal Blanking) & dummy lines(Vertical Blanking), it can be
*	used to adjust the frame rate or gain more time for back-end process.
*	
*	IMPORTANT NOTICE: the base shutter need re-calculate for some sensor, or else flicker may occur.
*
* PARAMETERS
*	1. kal_uint32 : Dummy Pixels (Horizontal Blanking)
*	2. kal_uint32 : Dummy Lines (Vertical Blanking)
*
* RETURNS
*	None
*
*************************************************************************/
static void ISX012MIPIinitalvariable(void)
{
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.video_mode = KAL_FALSE;
	ISX012MIPISensor.pv_mode = KAL_TRUE; 				
    ISX012MIPISensor.capture_mode= KAL_FALSE; 
	ISX012MIPISensor.NightMode = KAL_FALSE;
	ISX012MIPISensor.cp_dummy_pixels = 0;
	ISX012MIPISensor.cp_dummy_lines = 0;
	ISX012MIPISensor.pv_dummy_pixels = 0;
	ISX012MIPISensor.pv_dummy_lines = 0;
	ISX012MIPISensor.video_dummy_pixels = 0;
	ISX012MIPISensor.video_dummy_lines = 0;
	ISX012MIPISensor.SensorMode= SENSOR_MODE_INIT;
	ISX012MIPISensor.pv_mode= KAL_TRUE;
	/******************************xiaoyu*****************************************/
	ISX012MIPISensor.PreviewPclk=864;
	ISX012MIPISensor.CapturePclk=864;
	ISX012MIPISensor.VideoPclk=864;
	ISX012MIPISensor.pv_shutter=0;		   
	ISX012MIPISensor.video_shutter=0;		   
	ISX012MIPISensor.cp_shutter=0;
	/******************************xiaoyu*****************************************/
	ISX012MIPISensor.manualAEStart=0;
	ISX012MIPISensor.isoSpeed=AE_ISO_100;
	ISX012MIPISensor.userAskAeLock=KAL_FALSE;
    ISX012MIPISensor.userAskAwbLock=KAL_FALSE;
	
	ISX012MIPISensor.currentExposureTime=0;//for HDR save current shutter
    ISX012MIPISensor.currentAxDGain=0;//for HDR save current shutter
	
	ISX012MIPISensor.awbMode = AWB_MODE_AUTO;
	spin_unlock(&isx012mipi_drv_lock);
}
void ISX012MIPIGetExifInfo(UINT32 exifAddr)
{
	SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetExifInfo function\n");
    pExifInfo->FNumber = 20;
    pExifInfo->AEISOSpeed = ISX012MIPISensor.isoSpeed;
	pExifInfo->AWBMode = ISX012MIPISensor.awbMode;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = ISX012MIPISensor.isoSpeed;
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetExifInfo function\n");
}
/*************************************************************************
* FUNCTION
*	ISX012MIPIWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void ISX012MIPIWriteShutter(kal_uint32 shutter)
{
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIWriteShutter function\n");
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIWriteShutter function\n");
}    /* ISX012MIPI_write_shutter */
/*************************************************************************
* FUNCTION
*	ISX012MIPIWriteSensorGain
*
* DESCRIPTION
*	This function used to write the sensor gain.
*
* PARAMETERS
*	1. kal_uint32 : The sensor gain want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void ISX012MIPIWriteSensorGain(kal_uint32 gain)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIWriteSensorGain function:gain=%d\n",gain);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIWriteSensorGain function:\n ");
}  /* ISX012MIPI_write_sensor_gain */

/*************************************************************************
* FUNCTION
*	ISX012MIPIReadShutter
*
* DESCRIPTION
*	This function read current shutter for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current shutter value.
*
*************************************************************************/
#if 0
static kal_uint32 ISX012MIPIReadShutter(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIReadShutter function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.pv_shutter  = 0;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIReadShutter function:\n ");	
	return ISX012MIPISensor.pv_shutter;
} /* ISX012MIPI_read_shutter */
#endif

/*************************************************************************
* FUNCTION
*	ISX012MIPIReadSensorGain
*
* DESCRIPTION
*	This function read current sensor gain for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current sensor gain value.
*
*************************************************************************/
#if 0
static kal_uint32 ISX012MIPIReadSensorGain(void)
{
	kal_uint32 sensor_gain = 0;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIReadSensorGain function:\n ");
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIReadSensorGain function:\n ");
	return sensor_gain;
}  /* ISX012MIPIReadSensorGain */
#endif
/*************************************************************************
* FUNCTION
*	ISX012MIPI_set_AE_mode
*
* DESCRIPTION
*	This function ISX012MIPI_set_AE_mode.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void ISX012MIPI_set_AE_mode(kal_bool AE_enable)
{
    //kal_uint8 AeTemp;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_AE_mode function:\n ");
    //AeTemp = ISX012MIPIYUV_read_cmos_sensor(0x3503);
    if (AE_enable == KAL_TRUE)
    {
    }
    else
    {
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_AE_mode function:\n ");
}

/*************************************************************************
* FUNCTION
*	ISX012MIPI_set_AWB_mode
*
* DESCRIPTION
*	This function ISX012MIPI_set_AWB_mode.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void ISX012MIPI_set_AWB_mode(kal_bool AWB_enable)
{
    //kal_uint8 AwbTemp;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_AWB_mode function:\n ");
    if (AWB_enable == KAL_TRUE)
    {
            	
    }
    else
    {             
		
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_AWB_mode function:\n ");
}
#if 0
static void ISX012MIPI_set_AWB_mode_UNLOCK()
{
    ISX012MIPI_set_AWB_mode(KAL_TRUE);
    if (!((SCENE_MODE_OFF == ISX012MIPISensor.sceneMode) || (SCENE_MODE_NORMAL == 
    ISX012MIPISensor.sceneMode) || (SCENE_MODE_HDR == ISX012MIPISensor.sceneMode)))
    {
      ISX012MIPI_set_scene_mode(ISX012MIPISensor.sceneMode);        
    }
    if (!((AWB_MODE_OFF == ISX012MIPISensor.awbMode) || (AWB_MODE_AUTO == ISX012MIPISensor.awbMode)))
    {
	   ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_AWB_mode_UNLOCK function:awbMode=%d\n ",ISX012MIPISensor.awbMode);
	   ISX012MIPI_set_param_wb(ISX012MIPISensor.awbMode);
    }
    return;
}
#endif
/*************************************************************************
* FUNCTION
*	ISX012MIPI_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
//static 
kal_uint32 ISX012MIPI_GetSensorID(kal_uint32 *sensorID)
{
    volatile signed char i;
	kal_uint32 sensor_id=0;
	kal_uint32 sensor_tmp=0;
	//kal_uint8 temp_sccb_addr = 0;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_GetSensorID function:\n ");
	for(i=0;i<3;i++)
	{
		sensor_id  = (ISX012MIPIYUV_read_cmos_sensor(0x000E));
		sensor_tmp = sensor_id && 0x01;		
		ISX012MIPISENSORDB("ISX012MIPI READ ID: %x",sensor_tmp);
		if(sensor_tmp)
		{
			*sensorID=ISX012MIPI_SENSOR_ID;
		        break;
		}
	}
	if(*sensorID != ISX012MIPI_SENSOR_ID)
	{	
		*sensorID =0xffffffff;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_GetSensorID function:\n ");
    return ERROR_NONE;    
}   
UINT32 ISX012SetTestPatternMode(kal_bool bEnable)
{
	ISX012MIPISENSORDB("[ISX012MIPI_ISX012SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
	if(bEnable)
	{
	}
	else
	{
	}
	return ERROR_NONE;
}
/*************************************************************************
* FUNCTION
*	ISX012MIPI_WAIT_STAUS
*
* DESCRIPTION
*	This function wait the 0x000E bit 0 is 1;then clear the bit 0;
*      The salve address is 0x34
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void ISX012MIPI_WAIT_STAUS(void)
{
	kal_uint32 tmp;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_WAIT_STAUS function:\n ");
    do
    {
    	tmp=ISX012MIPIYUV_read_cmos_sensor(0x000E) && 0x01;
	}while(!tmp);
	ISX012MIPI_write_cmos_sensor(0x0012,0x01);
	mDELAY(10);
	do
	{
         tmp=ISX012MIPIYUV_read_cmos_sensor(0x000E) && 0x01;
	}while(tmp);
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_WAIT_STAUS function:\n ");
}
/*************************************************************************
* FUNCTION
*	ISX012MIPI_WAIT_STAUS1
*
* DESCRIPTION
*	This function wait the 0x000E bit 1 is 1;then clear the bit 1;
*      The salve address is 0x78
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void ISX012MIPI_WAIT_STAUS1(void)
{
	kal_uint32 tmp;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_WAIT_STAUS1 function:\n ");
    do
    {
    	tmp=ISX012MIPIYUV_read_cmos_sensor1(0x000E) && 0x02;
	}while(tmp !=0x02);
	ISX012MIPI_write_cmos_sensor1(0x0012,0x02);
	mDELAY(10);
	do	
	{
         tmp=ISX012MIPIYUV_read_cmos_sensor1(0x000E) && 0x02;
	}
	while(tmp);
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_WAIT_STAUS1 function:\n ");
}
/*************************************************************************
* FUNCTION
*	ISX012MIPI_WAIT_STAUS2
*
* DESCRIPTION
*	This function wait the 0x000E bit 0 is 1;then clear the bit 0;
*      The salve address is 0x78
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void ISX012MIPI_WAIT_STAUS2(void)
{
	kal_uint32 tmp;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_WAIT_STAUS function:\n ");
    do
    {
    	tmp=ISX012MIPIYUV_read_cmos_sensor1(0x000E) && 0x01;
	}while(!tmp);
	ISX012MIPI_write_cmos_sensor1(0x0012,0x01);
	mDELAY(10);
	do
	{
         tmp=ISX012MIPIYUV_read_cmos_sensor1(0x000E) && 0x01;
	}while(tmp);
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_WAIT_STAUS function:\n ");
}

/*************************************************************************
* FUNCTION
*    ISX012MIPIInitialSetting
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
//static 
void ISX012MIPIInitialSetting(void)
{
	//;ISX012MIPI preview  640*480      30fps
	//;ISX012MIPI capture  2560*1920   7.5fps
	//;ISX012MIPI video     1280*720     30fps
	//86.4Mhz, 432Mbps/Lane, 2 Lane	
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIInitialSetting function:\n ");
	ISX012MIPI_WAIT_STAUS();
	/***************pll********************/
	ISX012MIPI_write_cmos_sensor(0x5008,0x00);
	ISX012MIPI_write_cmos_sensor(0x0004,0x02);
	ISX012MIPI_write_cmos_sensor(0x0007,0x00);
	ISX012MIPI_write_cmos_sensor(0x0008,0x00);
	ISX012MIPI_write_cmos_sensor_16(0x00C2,0x0002);	
	ISX012MIPI_write_cmos_sensor(0x00c4,0x11);
	ISX012MIPI_write_cmos_sensor(0x00c5,0x11);
	/***************mipi timing clk=432MHZ,PCLK=86.2MHz**************/
	ISX012MIPI_write_cmos_sensor(0x5C01,0x00);					// RGLANESEL
	ISX012MIPI_write_cmos_sensor(0x5C04,0x04);					// RGTLPX	
	ISX012MIPI_write_cmos_sensor(0x5C05,0x03);					// RGTCLKPRE
	ISX012MIPI_write_cmos_sensor(0x5C06,0x0E);					// RGTCLKZER
	ISX012MIPI_write_cmos_sensor(0x5C07,0x02);					// RGTCLKPRE
	ISX012MIPI_write_cmos_sensor(0x5C08,0x0B);					// RGTCLKPOS
	ISX012MIPI_write_cmos_sensor(0x5C09,0x05);					// RGTCLKTRA
	ISX012MIPI_write_cmos_sensor(0x5C0A,0x07);					// RGTHSEXIT
	ISX012MIPI_write_cmos_sensor(0x5C0B,0x03);					// RGTHSPREP
	ISX012MIPI_write_cmos_sensor(0x5C0C,0x07);					// RGTHSZERO
	ISX012MIPI_write_cmos_sensor(0x5C0D,0x05);					// RGTHSTRAI
	ISX012MIPI_write_cmos_sensor(0x5C0E,0x01);					// RGTLPXESC
	ISX012MIPI_write_cmos_sensor(0x0006,0x16);
	ISX012MIPI_WAIT_STAUS();	
	// Preview 640x480@15fps												  
	ISX012MIPI_write_cmos_sensor(0x0089,0x00);					  // OUTFMT_MONI(YUV) 
	ISX012MIPI_write_cmos_sensor_16(0x0090,0x8002); 		// HSIZE_MONI(640)	  	
	ISX012MIPI_write_cmos_sensor_16(0x0096,0xE001); 		// VSIZE_MONI(480)	  	
	ISX012MIPI_write_cmos_sensor(0x0086,0x02);				  // FPSTYPE_MONI(30fps)
	ISX012MIPI_write_cmos_sensor(0x0083,0x01);				  // SENSMODE_MONI(V1/2)	
	// Capture 2560x1920@15fps												 
	ISX012MIPI_write_cmos_sensor(0x008A,0x00);				  // OUTFMT_CAP(YUV)   
	ISX012MIPI_write_cmos_sensor_16(0x0092,0x000A); 		// HSIZE_CAP(2560))  
	ISX012MIPI_write_cmos_sensor_16(0x0098,0x8007); 		// VSIZE_CAP(1920)	 
	ISX012MIPI_write_cmos_sensor(0x0087,0x03);				  // FPSTYPE_CAP(15fps)
	ISX012MIPI_write_cmos_sensor(0x0084,0x00);				  // SENSMODE_CAP(Full)
	// Moviec 1280x720@30fps													 
	ISX012MIPI_write_cmos_sensor(0x008B,0x00);					  // OUTFMT_MOVIE(YUV)	 
	ISX012MIPI_write_cmos_sensor_16(0x0094,0x0005); 		// HSIZE_MOVIE(1280)	 
	ISX012MIPI_write_cmos_sensor_16(0x009A,0xD002); 		// VSIZE_MOVIE(720) 	 
	ISX012MIPI_write_cmos_sensor(0x0088,0x02);					  // FPSTYPE_MOVIE(30fps)
	ISX012MIPI_write_cmos_sensor(0x0085,0x01);					  // SENSMODE_MOVIE(V1/2)
    
	ISX012MIPI_write_cmos_sensor_16(0x00DE,0x6911); 			// YUVCONFIG							   
	ISX012MIPI_write_cmos_sensor_16(0x6A9E,0xC015); 				// HMAX Extention ON(Capture 15fps ->
	ISX012MIPI_write_cmos_sensor(0x00AF,0x11);						// HSENS_MODE(Monitor & Movie H1/2)
	
    mt_set_gpio_mode(GPIO88,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO88,1);
	mt_set_gpio_out(GPIO88,1);
	//ISX012MIPI_WAIT_STAUS();
	//ISX012MIPI_WAIT_STAUS1();
	mDELAY(20);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIInitialSetting function:\n ");
		
} 
/*****************************************************************
* FUNCTION
*    ISX012MIPIPreviewSetting
*
* DESCRIPTION
*    This function config Preview setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
#if 0
static void ISX012MIPIPreviewSetting_SVGA(void)
{
	//;ISX012MIPI 1280x960,30fps
	//86.4Mhz, 432Mbps/Lane, 2Lane
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIPreviewSetting_SVGA function:\n ");
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIPreviewSetting_SVGA function:\n ");
}
#endif
/*************************************************************************
* FUNCTION
*     ISX012MIPIFullSizeCaptureSetting
*
* DESCRIPTION
*    This function config full size capture setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
#if 0
static void ISX012MIPIFullSizeCaptureSetting(void)
{
	//ISX012MIPI 2592x1944,7.5fps
	//86.4Mhz, 432Mbps/Lane, 2Lane
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIFullSizeCaptureSetting function:\n ");
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIFullSizeCaptureSetting function:\n ");
}
#endif
/*************************************************************************
* FUNCTION
*    ISX012MIPISetHVMirror
*
* DESCRIPTION
*    This function set sensor Mirror
*
* PARAMETERS
*    Mirror
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void ISX012MIPISetHVMirror(kal_uint8 Mirror, kal_uint8 Mode)
{
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPISetHVMirror function:\n ");
	if (Mode==SENSOR_MODE_PREVIEW)
	{
		switch (Mirror)
		{
			case IMAGE_NORMAL:
				ISX012MIPI_write_cmos_sensor1(0x008c,0x00);
				break;
			case IMAGE_H_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008c,0x02);
				break;
			case IMAGE_V_MIRROR: 
				ISX012MIPI_write_cmos_sensor1(0x008c,0x01);
				break;		
			case IMAGE_HV_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008c,0x3);
				break; 		
			default:
				break;
		}
	}
	else if (Mode== SENSOR_MODE_CAPTURE)
	{
		switch (Mirror)
		{
			case IMAGE_NORMAL:
				ISX012MIPI_write_cmos_sensor1(0x008d,0x00);
				break;
			case IMAGE_H_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008d,0x02);
				break;
			case IMAGE_V_MIRROR: 
				ISX012MIPI_write_cmos_sensor1(0x008d,0x01);
				break;		
			case IMAGE_HV_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008d,0x03);
				break; 		
			default:
				break;
		}
	}
	else if (Mode== SENSOR_MODE_VIDEO)
	{
		switch (Mirror)
		{
			case IMAGE_NORMAL:
				ISX012MIPI_write_cmos_sensor1(0x008e,0x00);
				break;
			case IMAGE_H_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008e,0x02);
				break;
			case IMAGE_V_MIRROR: 
				ISX012MIPI_write_cmos_sensor1(0x008e,0x01);
				break;		
			case IMAGE_HV_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008e,0x03);
				break; 		
			default:
				break;
		}
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPISetHVMirror function:\n ");
}

void ISX012MIPI_Standby(void)
{
}

void ISX012MIPI_Wakeup(void)
{
}
/*************************************************************************
* FUNCTION
*   ISX012_FOCUS_OVT_AFC_Init
* DESCRIPTION
*   This function is to load micro code for AF function
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/
static void ISX012_FOCUS_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{ 	  
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012_FOCUS_Get_AF_Max_Num_Focus_Areas function:\n ");
	pFeatureReturnPara32 = 0;    
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012_FOCUS_Get_AF_Max_Num_Focus_Areas function:\n ");	
}

static void ISX012_FOCUS_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{ 	
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
    pFeatureReturnPara32 = 0;    
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
}

static void ISX012_FOCUS_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{
    pFeatureReturnPara32 = 0;
}
static void ISX012_FOCUS_Get_AF_Inf(UINT32 * pFeatureReturnPara32)
{
    pFeatureReturnPara32 = 0;
}
/*************************************************************************
* FUNCTION
*   ISX012WBcalibattion
* DESCRIPTION
*   color calibration
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
#if 0
static void ISX012WBcalibattion(kal_uint32 color_r_gain,kal_uint32 color_b_gain)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012WBcalibattion function:color_r_gain=%d,color_b_gain=%d\n",color_r_gain,color_b_gain); 
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012WBcalibattion function:\n ");
}	
#endif
/*************************************************************************
* FUNCTION
*	ISX012MIPIOpen
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 ISX012MIPIOpen(void)
{
	volatile signed int i;
	kal_uint16 sensor_id = 0;
	kal_uint32 sensor_tmp=0;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIOpen function:\n ");
	for(i=0;i<3;i++)
	{
		sensor_id  = (ISX012MIPIYUV_read_cmos_sensor(0x000E));
		sensor_tmp = sensor_id && 0x01;
		if(sensor_tmp)
			break;
	}
	if(!sensor_tmp)
	{
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	ISX012MIPIinitalvariable();
	ISX012MIPIInitialSetting();
	mDELAY(20);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIOpen function:\n ");
	return ERROR_NONE;
}	/* ISX012MIPIOpen() */

/*************************************************************************
* FUNCTION
*	ISX012MIPIClose
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 ISX012MIPIClose(void)
{
 	//CISModulePowerOn(FALSE);
	return ERROR_NONE;
}	/* ISX012MIPIClose() */
/*************************************************************************
* FUNCTION
*	ISX012MIPIPreview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 ISX012MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIPreview function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.SensorMode=SENSOR_MODE_PREVIEW;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x0081,0x00);//sensor set preview mode
	mDELAY(100);
	ISX012MIPISetHVMirror(IMAGE_HV_MIRROR,SENSOR_MODE_PREVIEW);
	return ERROR_NONE ;	
}	/* ISX012MIPIPreview() */
BOOL ISX012MIPI_set_param_exposure_for_HDR(UINT16 para)
{
    kal_uint32 totalGain = 0, exposureTime = 0;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_param_exposure_for_HDR function:para=%d,manualAEStart=%d\n",para,ISX012MIPISensor.manualAEStart);
    if (0 == ISX012MIPISensor.manualAEStart)
    {       
        ISX012MIPI_set_AE_mode(KAL_FALSE);//Manual AE enable
        spin_lock(&isx012mipi_drv_lock);	
        ISX012MIPISensor.manualAEStart = 1;
		spin_unlock(&isx012mipi_drv_lock);
    }
	totalGain = ISX012MIPISensor.currentAxDGain;
    exposureTime = ISX012MIPISensor.currentExposureTime;
	switch (para)
	{
		case AE_EV_COMP_20:	//+2 EV
		case AE_EV_COMP_10:	// +1 EV
			totalGain = totalGain<<1;
			exposureTime = exposureTime<<1;
			ISX012MIPISENSORDB("[ISX012MIPI] HDR AE_EV_COMP_20\n");
		break;
		case AE_EV_COMP_00:	// +0 EV
			ISX012MIPISENSORDB("[ISX012MIPI] HDR AE_EV_COMP_00\n");
		break;
		case AE_EV_COMP_n10:  // -1 EV
		case AE_EV_COMP_n20:  // -2 EV
			totalGain = totalGain >> 1;
			exposureTime = exposureTime >> 1;
			ISX012MIPISENSORDB("[ISX012MIPI] HDR AE_EV_COMP_n20\n");
			break;
		default:
		break;//return FALSE;
	}
    ISX012MIPIWriteSensorGain(totalGain);	
	ISX012MIPIWriteShutter(exposureTime);	
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_param_exposure_for_HDR function:\n ");
	return TRUE;
}
UINT32 ISX012MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.SensorMode=SENSOR_MODE_CAPTURE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x0081,0x02);
	mDELAY(100);
	ISX012MIPISetHVMirror(IMAGE_HV_MIRROR,SENSOR_MODE_CAPTURE);
	return ERROR_NONE; 
}/* ISX012MIPICapture() */

UINT32 ISX012MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetResolution function:\n ");
	pSensorResolution->SensorPreviewWidth=  ISX012MIPI_IMAGE_SENSOR_SVGA_WIDTH-4;
	pSensorResolution->SensorPreviewHeight= ISX012MIPI_IMAGE_SENSOR_SVGA_HEIGHT-3;
	pSensorResolution->SensorFullWidth= ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH-4; 
	pSensorResolution->SensorFullHeight= ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT-3;
	pSensorResolution->SensorVideoWidth= ISX012MIPI_IMAGE_SENSOR_SVGA_WIDTH-16; 
	pSensorResolution->SensorVideoHeight= ISX012MIPI_IMAGE_SENSOR_SVGA_HEIGHT-9;
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetResolution function:\n ");
	return ERROR_NONE;
}	/* ISX012MIPIGetResolution() */

UINT32 ISX012MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,MSDK_SENSOR_INFO_STRUCT *pSensorInfo,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetInfo function:ScenarioId=%d\n",ScenarioId);
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorPreviewResolutionX=ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH-4;
			pSensorInfo->SensorPreviewResolutionY=ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT-3;
			pSensorInfo->SensorCameraPreviewFrameRate=7.5;
			break;
		default:
			pSensorInfo->SensorPreviewResolutionX=ISX012MIPI_IMAGE_SENSOR_SVGA_WIDTH-4;
			pSensorInfo->SensorPreviewResolutionY=ISX012MIPI_IMAGE_SENSOR_SVGA_HEIGHT-3;
			pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
	}		 		
	pSensorInfo->SensorFullResolutionX= ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH-4;
	pSensorInfo->SensorFullResolutionY= ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT-3;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=5;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=4;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;  
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 2;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	pSensorInfo->CaptureDelayFrame = 0;
	pSensorInfo->PreviewDelayFrame = 0; 
	pSensorInfo->VideoDelayFrame = 0; 		
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->YUVAwbDelayFrame = 2;
	pSensorInfo->YUVEffectDelayFrame= 2; 
	pSensorInfo->AEShutDelayFrame= 0;
 	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;   		
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = ISX012MIPI_PV_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = ISX012MIPI_PV_GRAB_START_Y;   
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 0x1A; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;  	
			pSensorInfo->SensorPacketECCOrder = 1;		
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = ISX012MIPI_FULL_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = ISX012MIPI_FULL_GRAB_START_Y;             
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount =0x1A; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;
			pSensorInfo->SensorPacketECCOrder = 1;
			break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=5;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = ISX012MIPI_PV_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = ISX012MIPI_PV_GRAB_START_Y; 			
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 0x1A; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;
			pSensorInfo->SensorHightSampling = 0;	
			pSensorInfo->SensorPacketECCOrder = 1;
		  break;
	}
	memcpy(pSensorConfigData, &ISX012MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));	
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetInfo function:\n ");	
	return ERROR_NONE;
}	/* ISX012MIPIGetInfo() */

UINT32 ISX012MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	  ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIControl function:ScenarioId=%d\n",ScenarioId);
	  spin_lock(&isx012mipi_drv_lock);
	  CurrentScenarioId = ScenarioId;
	  spin_unlock(&isx012mipi_drv_lock);
	  switch (ScenarioId)
	  {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 ISX012MIPIPreview(pImageWindow, pSensorConfigData);
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 ISX012MIPICapture(pImageWindow, pSensorConfigData);
	  	     break;
		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIControl function:\n ");
	return ERROR_NONE;
}	/* ISX012MIPIControl() */

/* [TC] YUV sensor */	

BOOL ISX012MIPI_set_param_wb(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_param_wb function:para=%d\n",para);
	spin_lock(&isx012mipi_drv_lock);
    ISX012MIPISensor.awbMode = para;
    spin_unlock(&isx012mipi_drv_lock);
	switch (para)
    {
        case AWB_MODE_OFF:
							spin_lock(&isx012mipi_drv_lock);
							ISX012MIPI_AWB_ENABLE = KAL_FALSE; 
							spin_unlock(&isx012mipi_drv_lock);
							ISX012MIPI_set_AWB_mode(ISX012MIPI_AWB_ENABLE);
							break;                    
        case AWB_MODE_AUTO: //auto
							spin_lock(&isx012mipi_drv_lock);
							ISX012MIPI_AWB_ENABLE = KAL_TRUE; 
							spin_unlock(&isx012mipi_drv_lock);
							ISX012MIPI_set_AWB_mode(ISX012MIPI_AWB_ENABLE);
							ISX012MIPI_write_cmos_sensor(0x0282,0x20);
							break;
        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
        					ISX012MIPI_write_cmos_sensor1(0x0282,0x26);
             				break;
        case AWB_MODE_DAYLIGHT: //sunny
							ISX012MIPI_write_cmos_sensor1(0x0282,0x25);
							break;
         case AWB_MODE_FLUORESCENT:
							ISX012MIPI_write_cmos_sensor1(0x0282,0x27); 
		case AWB_MODE_TUNGSTEN:
							ISX012MIPI_write_cmos_sensor1(0x0282,0x28);
							break;
        default:
			return FALSE;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_param_wb function:\n ");
    return TRUE;
} /* ISX012MIPI_set_param_wb */
void ISX012MIPI_set_contrast(UINT16 para)
{   
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_contrast function:\n ");
	/*
    switch (para)
    {
        case ISP_CONTRAST_LOW:
             ISX012MIPI_write_cmos_sensor(0x3212,0x03);
			 ISX012MIPI_write_cmos_sensor(0x5586,0x14);
			 ISX012MIPI_write_cmos_sensor(0x5585,0x14);
			 ISX012MIPI_write_cmos_sensor(0x3212,0x13);
			 ISX012MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
        case ISP_CONTRAST_HIGH:
             ISX012MIPI_write_cmos_sensor(0x3212,0x03);
			 ISX012MIPI_write_cmos_sensor(0x5586,0x2c);
			 ISX012MIPI_write_cmos_sensor(0x5585,0x1c);
			 ISX012MIPI_write_cmos_sensor(0x3212,0x13);
			 ISX012MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
        case ISP_CONTRAST_MIDDLE:
			 ISX012MIPI_write_cmos_sensor(0x3212,0x03);
			 ISX012MIPI_write_cmos_sensor(0x5586,0x20);
			 ISX012MIPI_write_cmos_sensor(0x5585,0x00);
			 ISX012MIPI_write_cmos_sensor(0x3212,0x13);
			 ISX012MIPI_write_cmos_sensor(0x3212,0xa3);
			 break;
        default:
             break;
    }
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_contrast function:\n ");
    */
    return;
}

void ISX012MIPI_set_brightness(UINT16 para)
{
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_brightness function:para=%d\n",para);
    switch (para)
    {
        case ISP_BRIGHT_LOW:
             ISX012MIPI_write_cmos_sensor1(0x0180, 0x01);
             break;
        case ISP_BRIGHT_HIGH:
             ISX012MIPI_write_cmos_sensor1(0x0180, 0x03);
             break;
        case ISP_BRIGHT_MIDDLE:
			 ISX012MIPI_write_cmos_sensor1(0x0180, 0x06);
			 break;
        default:
             return ;
             break;
    }
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_brightness function:\n ");
    return;
}
void ISX012MIPI_set_saturation(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_saturation function:para=%d\n",para);
    switch (para)
    {
        case ISP_SAT_HIGH:		
			break;
        case ISP_SAT_LOW:
             break;
        case ISP_SAT_MIDDLE:
			 break;
        default:
			 break;
    }
	mDELAY(50);
     return;
}
void ISX012MIPI_scene_mode_PORTRAIT(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_scene_mode_SPORTS function:\n ");
	
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&isx012mipi_drv_lock);  
    ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	 //ISO_TYPE1 : Auto 
    ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	 //SHTCTRLMAG3		
    ISX012MIPI_write_cmos_sensor1(0x038F,0x50);	 //PICT1_SN1 :		
    ISX012MIPI_write_cmos_sensor1(0x6742,0x0012);   // AF_SEARCH_OFFSE
    ISX012MIPI_write_cmos_sensor1(0x6744,0x0006);   // AF_SEARCH_OFFSE
    ISX012MIPI_write_cmos_sensor1(0x500B,0x01);	 // FAST_SHT_MODE_SE
    ISX012MIPI_write_cmos_sensor1(0x0280,0x00);	 //SCENE_SELECT 	   
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_scene_mode_SPORTS function:\n ");
}

void ISX012MIPI_scene_mode_LANDSCAPE(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_scene_mode_LANDSCAPE function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	  //ISO_TYPE1 : 
	ISX012MIPI_write_cmos_sensor1(0x039F,0x9E);	  //UISATURATION
	ISX012MIPI_write_cmos_sensor1(0x03A3,0x2C);	  //UISHARPNESS_
	ISX012MIPI_write_cmos_sensor1(0x03A6,0x2C);	  //UISHARPNESS_
	ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	  //SHTCTRLMAG3 
	ISX012MIPI_write_cmos_sensor1(0x038F,0x00);	  //PICT1_SN1 : 
	ISX012MIPI_write_cmos_sensor_16_MIPI(0x6742,0x0012);    // AF_SEA
	ISX012MIPI_write_cmos_sensor_16_MIPI(0x6744,0x0006);    // AF_SEA
	ISX012MIPI_write_cmos_sensor1(0x500B,0x01);	  // FAST_SHT_MO
	ISX012MIPI_write_cmos_sensor1(0x0280,0x01);	  //SCENE_SELECT
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_scene_mode_LANDSCAPE function:\n ");
}
void ISX012MIPI_scene_mode_SUNSET(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_scene_mode_SUNSET function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	 //ISO_TYPE1 
	ISX012MIPI_write_cmos_sensor1(0x0287,0x25);	 //AWB_SN6 : 
	ISX012MIPI_write_cmos_sensor1(0x0394,0x00);	 //PICT1_SN6 
	ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	 //SHTCTRLMAG
	ISX012MIPI_write_cmos_sensor1(0x038F,0x00);	 //PICT1_SN1 
	ISX012MIPI_write_cmos_sensor_16_MIPI(0x6742,0x0012);	  // AF_S
	ISX012MIPI_write_cmos_sensor_16_MIPI(0x6744,0x0006);	  // AF_S
	ISX012MIPI_write_cmos_sensor1(0x500B,0x01);  // FAST_SHT_MO
	ISX012MIPI_write_cmos_sensor1(0x0280,0x05);  //SCENE_SELECT
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_scene_mode_SUNSET function:\n ");
}
void ISX012MIPI_scene_mode_SPORTS(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_scene_mode_SPORTS function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	  //ISO_TYPE1 : 
	ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	  //SHTCTRLMAG3 
	ISX012MIPI_write_cmos_sensor1(0x038F,0x00);	  //PICT1_SN1 : 
	ISX012MIPI_write_cmos_sensor_16_MIPI(0x6742,0x0012);    // AF_SEA
	ISX012MIPI_write_cmos_sensor_16_MIPI(0x6744,0x0006);    // AF_SEA
	ISX012MIPI_write_cmos_sensor1(0x500B,0x01);	  // FAST_SHT_MO
	ISX012MIPI_write_cmos_sensor1(0x0280,0x02);	  //SCENE_SELECT
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_scene_mode_SPORTS function:\n ");
}
void ISX012MIPI_scene_mode_OFF(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_scene_mode_OFF function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&isx012mipi_drv_lock);	
	ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	  //ISO_TYPE1 : Au
	ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	  //SHTCTRLMAG3   
	ISX012MIPI_write_cmos_sensor1(0x038F,0x00);	  //PICT1_SN1 :   
	ISX012MIPI_write_cmos_sensor1(0x6742,0x0012);	// AF_SEARCH_O
	ISX012MIPI_write_cmos_sensor1(0x6744,0x0006);	// AF_SEARCH_O
	ISX012MIPI_write_cmos_sensor1(0x500B,0x01);	  // FAST_SHT_MODE
	ISX012MIPI_write_cmos_sensor1(0x0280,0x00);	  //SCENE_SELECT  
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_scene_mode_OFF function:\n ");
}
void ISX012MIPI_scene_mode_night(void)
{
   ISX012MIPISENSORDB("[ISX012MIPI]enter SCENE_MODE_NIGHTSCENE function\n"); 
   spin_lock(&isx012mipi_drv_lock);
   ISX012MIPISensor.NightMode=KAL_TRUE;
   spin_unlock(&isx012mipi_drv_lock);
   ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	 //ISO_TYPE1 : 
   ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	 //SHTCTRLMAG3 
   ISX012MIPI_write_cmos_sensor1(0x038F,0x00);	 //PICT1_SN1 : 
   ISX012MIPI_write_cmos_sensor_16_MIPI(0x6742,0x0012);	  // AF_SEA
   ISX012MIPI_write_cmos_sensor_16_MIPI(0x6744,0x0006);	  // AF_SEA
   ISX012MIPI_write_cmos_sensor1(0x500B,0x00);	 // FAST_SHT_MO
   ISX012MIPI_write_cmos_sensor1(0x0280,0x07);	 //SCENE_SELECT	   
   ISX012MIPISENSORDB("[ISX012MIPI]exit SCENE_MODE_NIGHTSCENE function\n");
}

void ISX012MIPI_set_scene_mode(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_scene_mode function:para=%d\n",para);	
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.sceneMode=para;
	spin_unlock(&isx012mipi_drv_lock);
    switch (para)
    { 
		case SCENE_MODE_NIGHTSCENE:
			 ISX012MIPI_scene_mode_night(); 
			break;
        case SCENE_MODE_PORTRAIT:
			ISX012MIPI_scene_mode_PORTRAIT();		 
             break;
        case SCENE_MODE_LANDSCAPE:
			ISX012MIPI_scene_mode_LANDSCAPE();		 
             break;
        case SCENE_MODE_SUNSET:
			ISX012MIPI_scene_mode_SUNSET();		 
            break;
        case SCENE_MODE_SPORTS:
            ISX012MIPI_scene_mode_SPORTS();		 
            break;
        case SCENE_MODE_HDR:
            if (1 == ISX012MIPISensor.manualAEStart)
            {
                ISX012MIPI_set_AE_mode(KAL_TRUE);//Manual AE disable
                spin_lock(&isx012mipi_drv_lock);
            	ISX012MIPISensor.manualAEStart = 0;
                ISX012MIPISensor.currentExposureTime = 0;
                ISX012MIPISensor.currentAxDGain = 0;
				spin_unlock(&isx012mipi_drv_lock);
            }
            break;
        case SCENE_MODE_OFF:
        default:
			ISX012MIPI_scene_mode_OFF();
            break;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_scene_mode function:\n ");
	return;
}
void ISX012MIPI_set_iso(UINT16 para)
{
    spin_lock(&isx012mipi_drv_lock);
    ISX012MIPISensor.isoSpeed = para;
    spin_unlock(&isx012mipi_drv_lock); 
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_iso function:para=%d\n",para);
    switch (para)
    {
        case AE_ISO_100:
			 ISX012MIPI_write_cmos_sensor1(0x02A8, 0x07);
			 ISX012MIPI_write_cmos_sensor1(0x0362, 0x57);
			 ISX012MIPI_write_cmos_sensor1(0x0365, 0x57);
			 break;
		case AE_ISO_AUTO:
             ISX012MIPI_write_cmos_sensor1(0x02A8, 0x00);
			 ISX012MIPI_write_cmos_sensor1(0x0362, 0x57);
			 ISX012MIPI_write_cmos_sensor1(0x0365, 0x57);
             break;
        case AE_ISO_200:
             ISX012MIPI_write_cmos_sensor1(0x02A8, 0x0A);
			 ISX012MIPI_write_cmos_sensor1(0x0362, 0x57);
			 ISX012MIPI_write_cmos_sensor1(0x0365, 0x57);
             break;
        case AE_ISO_400:
             ISX012MIPI_write_cmos_sensor1(0x02A8, 0x0D);
			 ISX012MIPI_write_cmos_sensor1(0x0362, 0x57);
			 ISX012MIPI_write_cmos_sensor1(0x0365, 0x57);
             break;
        default:
             break;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_iso function:para=%d\n",para);
    return;
}

BOOL ISX012MIPI_set_param_effect(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_param_effect function:para=%d\n ",para);
	switch (para)
    {
        case MEFFECT_OFF:  
			 ISX012MIPI_write_cmos_sensor1(0x01c5,0x00);
	         break;
		case MEFFECT_BLACKBOARD:
			 ISX012MIPI_write_cmos_sensor1(0x01c5,0x04);
	         break;
		case MEFFECT_NEGATIVE:
			 ISX012MIPI_write_cmos_sensor1(0x01c5,0x02);
			 break;
        case MEFFECT_SEPIA: 
			 ISX012MIPI_write_cmos_sensor1(0x01c5,0x03);
			 break;  
        default:
             return KAL_FALSE;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_param_effect function:\n ");
    return KAL_FALSE;
} /* ISX012MIPI_set_param_effect */

BOOL ISX012MIPI_set_param_banding(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_param_banding function:\n ");
	/*
	switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
						spin_lock(&isx012mipi_drv_lock);
						ISX012MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;
						spin_unlock(&isx012mipi_drv_lock);
						ISX012MIPI_write_cmos_sensor(0x3c00,0x04);
						ISX012MIPI_write_cmos_sensor(0x3c01,0x80);
            			break;
        case AE_FLICKER_MODE_60HZ:			
						spin_lock(&isx012mipi_drv_lock);
						ISX012MIPI_Banding_setting = AE_FLICKER_MODE_60HZ;
						spin_unlock(&isx012mipi_drv_lock);
						ISX012MIPI_write_cmos_sensor(0x3c00,0x00);
						ISX012MIPI_write_cmos_sensor(0x3c01,0x80);
            			break;
        default:
             return FALSE;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_param_banding function:\n ");
	*/
    return TRUE;
} /* ISX012MIPI_set_param_banding */

BOOL ISX012MIPI_set_param_exposure(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_param_exposure function:\n ");
	ISX012MIPISENSORDB("[ISX012MIPI]para=%d:\n",para);
    if (SCENE_MODE_HDR == ISX012MIPISensor.sceneMode && 
    SENSOR_MODE_CAPTURE == ISX012MIPISensor.SensorMode)
    {
       ISX012MIPI_set_param_exposure_for_HDR(para);
       return TRUE;
    }
	switch (para)
    {	
       case AE_EV_COMP_20:	                   
				ISX012MIPI_write_cmos_sensor1(0x0180, 0xff);//	EVSEL 
				break;
		case AE_EV_COMP_10:	                   
				ISX012MIPI_write_cmos_sensor1(0x0180, 0xfd);//	EVSEL  
			  break;
		case AE_EV_COMP_00:
				ISX012MIPI_write_cmos_sensor1(0x0180, 0x00);//	EVSEL 
			  break;
   		 case AE_EV_COMP_n10:
				ISX012MIPI_write_cmos_sensor1(0x0180, 0xfb);//	EVSEL 
			  break;
      	case AE_EV_COMP_n20:  // -2 EV
        	    ISX012MIPI_write_cmos_sensor1(0x0180, 0xfa);//	EVSEL 
        	  break;
        default:
              return FALSE;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_param_exposure function:\n ");
    return TRUE;
} /* ISX012MIPI_set_param_exposure */

UINT32 ISX012MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIYUVSensorSetting function:iCmd=%d,iPara=%d\n",iCmd,iPara);
	switch (iCmd) 
	{
		case FID_SCENE_MODE:
			ISX012MIPI_set_scene_mode(iPara);
	    	break; 	    
		case FID_AWB_MODE:
			ISX012MIPI_set_param_wb(iPara);
			  break;
		case FID_COLOR_EFFECT:
			ISX012MIPI_set_param_effect(iPara);
		 	  break;
		case FID_AE_EV:   
			ISX012MIPI_set_param_exposure(iPara);
		    break;
		case FID_AE_FLICKER:
			 ISX012MIPI_set_param_banding(iPara);
		 	 break; 
		case FID_ISP_CONTRAST:
            ISX012MIPI_set_contrast(iPara);
            break;
        case FID_ISP_BRIGHT:
            ISX012MIPI_set_brightness(iPara);
            break;
        case FID_ISP_SAT:
            ISX012MIPI_set_saturation(iPara);
            break;
    	case FID_ZOOM_FACTOR:    
			spin_lock(&isx012mipi_drv_lock);
	        zoom_factor = iPara; 
			spin_unlock(&isx012mipi_drv_lock);
            break;
		case FID_AE_ISO:
			ISX012MIPISENSORDB("[ISX012MIPI]FID_AE_ISO:%d\n", iPara);
            ISX012MIPI_set_iso(iPara);
            break;           
	  	default:
		    break;
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIYUVSensorSetting function\n");
	return TRUE;
}   /* ISX012MIPIYUVSensorSetting */
void ISX012MIPIYUV_SetFrameRate(UINT16 u2FrameRate)
{
	/*
	if (u2FrameRate == 30)
       {
       }
	else if (u2FrameRate == 15)   
	{
        }   
	else 
	{
	    printk("Wrong frame rate setting \n");
	} 
    */
}

UINT32 ISX012MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIYUVSetVideoMode function:u2FrameRate=%d\n",u2FrameRate);
	ISX012MIPI_write_cmos_sensor1(0x0081,0x03);
	mDELAY(100);
	ISX012MIPISetHVMirror(IMAGE_HV_MIRROR,SENSOR_MODE_VIDEO);
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.video_mode =KAL_TRUE;  
	ISX012MIPISensor.pv_mode=KAL_FALSE;
	ISX012MIPISensor.capture_mode=KAL_FALSE;
	ISX012MIPISensor.SensorMode=SENSOR_MODE_VIDEO;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPIYUV_SetFrameRate(u2FrameRate);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIYUVSetVideoMode function:\n ");
    return TRUE; 
}

/************************************************************/
static void ISX012MIPIGetEvAwbRef(UINT32 pSensorAEAWBRefStruct)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetEvAwbRef function:\n ");
	/*
	PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
	Ref->SensorAERef.AeRefLV05Shutter=0x170c;
	Ref->SensorAERef.AeRefLV05Gain=0x30;
	Ref->SensorAERef.AeRefLV13Shutter=0x24e;
	Ref->SensorAERef.AeRefLV13Gain=0x10;
	Ref->SensorAwbGainRef.AwbRefD65Rgain=0x610;
	Ref->SensorAwbGainRef.AwbRefD65Bgain=0x448;
	Ref->SensorAwbGainRef.AwbRefCWFRgain=0x4e0;
	Ref->SensorAwbGainRef.AwbRefCWFBgain=0x5a0;
	*/
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetEvAwbRef function:\n ");
}

static void ISX012MIPIGetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetCurAeAwbInfo function:\n ");
	/*
	PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
	Info->SensorAECur.AeCurShutter=ISX012MIPIReadShutter();
	Info->SensorAECur.AeCurGain=ISX012MIPIReadSensorGain() ;
	Info->SensorAwbGainCur.AwbCurRgain=((ISX012MIPIYUV_read_cmos_sensor(0x3401)&&0xff)+((ISX012MIPIYUV_read_cmos_sensor(0x3400)&&0xff)*256));
	Info->SensorAwbGainCur.AwbCurBgain=((ISX012MIPIYUV_read_cmos_sensor(0x3405)&&0xff)+((ISX012MIPIYUV_read_cmos_sensor(0x3404)&&0xff)*256));
	*/
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetCurAeAwbInfo function:\n ");
}
UINT32 ISX012MIPIMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
	{
        /*
		kal_uint32 pclk;
		kal_int16 dummyLine;
		kal_uint16 lineLength,frameHeight;
		ISX012MIPISENSORDB("ISX012MIPIMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
		ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIMaxFramerateByScenario function:\n ");
		switch (scenarioId) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				pclk = 56000000;
				lineLength = ISX012MIPI_IMAGE_SENSOR_SVGA_WIDTH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - ISX012MIPI_IMAGE_SENSOR_SVGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&isx012mipi_drv_lock);
				ISX012MIPISensor.SensorMode= SENSOR_MODE_PREVIEW;
				ISX012MIPISensor.pv_dummy_lines = dummyLine;
				spin_unlock(&isx012mipi_drv_lock);
				//ISX012MIPISetDummy(ISX012MIPISensor.PreviewDummyPixels, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pclk = 56000000;
				lineLength = ISX012MIPI_IMAGE_SENSOR_VIDEO_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - ISX012MIPI_IMAGE_SENSOR_VIDEO_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				//spin_lock(&isx012mipi_drv_lock);
				//ov8825.sensorMode = SENSOR_MODE_VIDEO;
				//spin_unlock(&isx012mipi_drv_lock);
				//ISX012MIPISetDummy(0, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:			
				pclk = 90000000;
				lineLength = ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&isx012mipi_drv_lock);
				ISX012MIPISensor.cp_dummy_lines = dummyLine;
				ISX012MIPISensor.SensorMode= SENSOR_MODE_CAPTURE;
				spin_unlock(&isx012mipi_drv_lock);
				//ISX012MIPISetDummy(ISX012MIPISensor.CaptureDummyPixels, dummyLine);			
				break;		
			case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
				break;		
			default:
				break;
		}	
		ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIMaxFramerateByScenario function:\n ");
		*/
		return ERROR_NONE;
	}
UINT32 ISX012MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetDefaultFramerateByScenario function:\n ");
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 75;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetDefaultFramerateByScenario function:\n ");
	return ERROR_NONE;
}
void ISX012MIPI_get_AEAWB_lock(UINT32 *pAElockRet32, UINT32 *pAWBlockRet32)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_get_AEAWB_lock function:\n ");
	*pAElockRet32 =1;
	*pAWBlockRet32=1;
	ISX012MIPISENSORDB("[ISX012MIPI]ISX012MIPI_get_AEAWB_lock,AE=%d,AWB=%d\n",*pAElockRet32,*pAWBlockRet32);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_get_AEAWB_lock function:\n ");
}
void ISX012MIPI_GetDelayInfo(UINT32 delayAddr)
{
	SENSOR_DELAY_INFO_STRUCT *pDelayInfo=(SENSOR_DELAY_INFO_STRUCT*)delayAddr;
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_GetDelayInfo function:\n ");
	pDelayInfo->InitDelay=0;
	pDelayInfo->EffectDelay=0;
	pDelayInfo->AwbDelay=0;
	pDelayInfo->AFSwitchDelayFrame=50;
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_GetDelayInfo function:\n ");
}
void ISX012MIPI_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
	/*
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          spin_lock(&isx012mipi_drv_lock);
          ISX012MIPISensor.userAskAeLock = TRUE;
          spin_unlock(&isx012mipi_drv_lock);
          ISX012MIPI_set_AE_mode(KAL_FALSE);
      break;
      case SENSOR_3A_AE_UNLOCK:
          spin_lock(&isx012mipi_drv_lock);
          ISX012MIPISensor.userAskAeLock = FALSE;
          spin_unlock(&isx012mipi_drv_lock);
          ISX012MIPI_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
          spin_lock(&isx012mipi_drv_lock);
          ISX012MIPISensor.userAskAwbLock = TRUE;
          spin_unlock(&isx012mipi_drv_lock);
          ISX012MIPI_set_AWB_mode(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
          spin_lock(&isx012mipi_drv_lock);
          ISX012MIPISensor.userAskAwbLock = FALSE;
          spin_unlock(&isx012mipi_drv_lock);
          ISX012MIPI_set_AWB_mode_UNLOCK();
      break;
      default:
      	break;
   }
   */
   ISX012MIPISENSORDB("[ISX012MIPI]exit ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   return;
}
#define FLASH_BV_THRESHOLD 0x25 
static void ISX012MIPI_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{   
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_FlashTriggerCheck function\n");
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_FlashTriggerCheck function\n");
	return;
}

UINT32 ISX012MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	UINT32 Tony_Temp1 = 0;
	UINT32 Tony_Temp2 = 0;
	Tony_Temp1 = pFeaturePara[0];
	Tony_Temp2 = pFeaturePara[1];
	ISX012MIPISENSORDB("[ISX012MIPI]enter[ISX012MIPIFeatureControl]feature id=%d \n",FeatureId);
	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH;
			*pFeatureReturnPara16=ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH + ISX012MIPISensor.cp_dummy_pixels;
					*pFeatureReturnPara16=ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT + ISX012MIPISensor.cp_dummy_lines;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara16++=ISX012MIPI_IMAGE_SENSOR_SVGA_WIDTH + ISX012MIPISensor.pv_dummy_pixels;
					*pFeatureReturnPara16=ISX012MIPI_IMAGE_SENSOR_SVGA_HEIGHT  + ISX012MIPISensor.pv_dummy_lines;
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = ISX012MIPISensor.VideoPclk * 1000 *100;	 //unit: Hz				
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = ISX012MIPISensor.PreviewPclk * 1000 *100;	 //unit: Hz
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		/**********************Strobe Ctrl Start *******************************/
		case SENSOR_FEATURE_SET_ESHUTTER:
			ISX012MIPISENSORDB("[ISX012MIPI] F_SET_ESHUTTER: Not Support\n");
			break;
		case SENSOR_FEATURE_SET_GAIN:
			ISX012MIPISENSORDB("[ISX012MIPI] F_SET_GAIN: Not Support\n");
			break;
		case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
			ISX012MIPISENSORDB("[ISX012MIPI] F_GET_AE_FLASHLIGHT_INFO: Not Support\n");
			break;
	    case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
            ISX012MIPI_FlashTriggerCheck(pFeatureData32);
            ISX012MIPISENSORDB("[ISX012MIPI] F_GET_TRIGGER_FLASHLIGHT_INFO: %d\n", (UINT32)pFeatureData32);
            break;		
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			ISX012MIPISENSORDB("ISX012MIPI SENSOR_FEATURE_SET_FLASHLIGHT\n");
			break;
		/**********************Strobe Ctrl End *******************************/
		
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			ISX012MIPISENSORDB("[ISX012MIPI] F_SET_ISP_MASTER_CLOCK_FREQ\n");
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			ISX012MIPI_write_cmos_sensor1(pSensorRegData->RegAddr, pSensorRegData->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = ISX012MIPIYUV_read_cmos_sensor1(pSensorRegData->RegAddr);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &ISX012MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
			break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
			break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=0;
            *pFeatureParaLen=4;	   
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			ISX012MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;	
		case SENSOR_FEATURE_SET_YUV_3A_CMD:
            ISX012MIPI_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
            break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SET_VIDEO_MODE\n");
		    ISX012MIPIYUVSetVideoMode(*pFeatureData16);
		    break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_CHECK_SENSOR_ID\n");
			ISX012MIPI_GetSensorID(pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_EV_AWB_REF\n");
			ISX012MIPIGetEvAwbRef(*pFeatureData32);
			break;		
		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN\n");
			ISX012MIPIGetCurAeAwbInfo(*pFeatureData32);			
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_EXIF_INFO\n");
            ISX012MIPIGetExifInfo(*pFeatureData32);
            break;
		case SENSOR_FEATURE_GET_DELAY_INFO:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_DELAY_INFO\n");
			ISX012MIPI_GetDelayInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_SET_SLAVE_I2C_ID:
             ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SET_SLAVE_I2C_ID\n");
             ISX012MIPI_sensor_socket = *pFeatureData32;
             break;
		case SENSOR_FEATURE_SET_TEST_PATTERN: 
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SET_TEST_PATTERN\n");
			ISX012SetTestPatternMode((BOOL)*pFeatureData16);            
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			ISX012MIPISENSORDB("[ISX012MIPI]ISX012_TEST_PATTERN_CHECKSUM\n");
			*pFeatureReturnPara32=ISX012_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;				
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO\n");
			ISX012MIPIMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:\
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO\n");
			ISX012MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,(MUINT32 *)*(pFeatureData32+1));
			break;
	    /**********************below is AF control**********************/	
		case SENSOR_FEATURE_INITIALIZE_AF:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_INITIALIZE_AF\n");
			//ISX012_FOCUS_OVT_AFC_Init();
            break;
		case SENSOR_FEATURE_MOVE_FOCUS_LENS:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_MOVE_FOCUS_LENS\n");
            //ISX012_FOCUS_Move_to(*pFeatureData16);
            break;
		case SENSOR_FEATURE_GET_AF_STATUS:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_AF_STATUS\n");
            //ISX012_FOCUS_OVT_AFC_Get_AF_Status(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;
		case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SINGLE_FOCUS_MODE\n");
			//ISX012_FOCUS_OVT_AFC_Single_Focus();
            break;
		case SENSOR_FEATURE_CONSTANT_AF:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_CONSTANT_AF\n");
			//ISX012_FOCUS_OVT_AFC_Constant_Focus();
			break;
		case SENSOR_FEATURE_CANCEL_AF:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_CANCEL_AF\n");
           // ISX012_FOCUS_OVT_AFC_Cancel_Focus();
            break;
		case SENSOR_FEATURE_GET_AF_INF:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_AF_INF\n");
            ISX012_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
		case SENSOR_FEATURE_GET_AF_MACRO:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_AF_MACRO\n");
            ISX012_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
		case SENSOR_FEATURE_SET_AF_WINDOW: 
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SET_AF_WINDOW\n");
			//ISX012_FOCUS_Set_AF_Window(*pFeatureData32);
            break;       					
        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS\n");
            ISX012_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break; 			
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_AF_STATUS\n");
			ISX012MIPI_get_AEAWB_lock((UINT32 *)(*pFeatureData32), (UINT32 *)(*(pFeatureData32+1)));
			break;					                              	               
        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			ISX012MIPISENSORDB("[ISX012MIPI]AE zone addr = 0x%x\n",*pFeatureData32);
            ISX012_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;        
        case SENSOR_FEATURE_SET_AE_WINDOW:
            ISX012MIPISENSORDB("[ISX012MIPI]AE zone addr = 0x%x\n",*pFeatureData32);			
            //ISX012_FOCUS_Set_AE_Window(*pFeatureData32);
            break; 
		default:
			break;			
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIFeatureControl function:\n ");
	return ERROR_NONE;
}	/* ISX012MIPIFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncISX012MIPI=
{
	ISX012MIPIOpen,
	ISX012MIPIGetInfo,
	ISX012MIPIGetResolution,
	ISX012MIPIFeatureControl,
	ISX012MIPIControl,
	ISX012MIPIClose
};

UINT32 ISX012_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncISX012MIPI;
	return ERROR_NONE;
}	/* SensorInit() */



