/*****************************************************************************
 *
 *
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
#include <linux/slab.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "t4k28yuv_Sensor.h"
#include "t4k28yuv_Camera_Sensor_para.h"
#include "t4k28yuv_CameraCustomized.h"

#include "kd_camera_feature.h"
//#include "parser.h"

kal_bool parser_enable = KAL_FALSE;
kal_bool  T4K28YUV_MPEG4_encode_mode = KAL_FALSE;
kal_uint16  T4K28YUV_sensor_gain_base = 0x0;
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 T4K28YUV_MAX_EXPOSURE_LINES = T4K28_PV_PERIOD_LINE_NUMS-4;
kal_uint8  T4K28YUV_MIN_EXPOSURE_LINES = 2;
kal_uint32 T4K28YUV_isp_master_clock;
kal_uint16 T4K28YUV_CURRENT_FRAME_LINES = T4K28_PV_PERIOD_LINE_NUMS;

static kal_uint16 T4K28YUV_dummy_pixels=0, T4K28YUV_dummy_lines=0;
kal_uint16 T4K28YUV_PV_dummy_pixels=0,T4K28YUV_PV_dummy_lines=0;

kal_uint8 T4K28YUV_sensor_write_I2C_address = T4K28_WRITE_ID;
kal_uint8 T4K28YUV_sensor_read_I2C_address = T4K28_READ_ID;

static kal_uint32 T4K28YUV_zoom_factor = 0;

kal_uint32 set_pv_back_es = 0xff;
kal_uint32 set_pv_back_ag = 0x20;
kal_uint32 set_pv_back_dg = 0x10;

#define LOG_TAG "[T4K28Yuv]"
#define SENSORDB(fmt, arg...) printk( LOG_TAG  fmt, ##arg)
#define RETAILMSG(x,...)
#define TEXT

kal_uint16 T4K28YUV_g_iDummyLines = 28;
//XB.PANG NEED CHECK
struct
{
  kal_bool    NightMode;
  kal_uint8   ZoomFactor; /* Zoom Index */
  kal_uint16  Banding;
  kal_uint32  PvShutter;
  kal_uint32  PvDummyPixels;
  kal_uint32  PvDummyLines;
  kal_uint32  CapDummyPixels;
  kal_uint32  CapDummyLines;
  kal_uint32  PvOpClk;
  kal_uint32  CapOpClk;
  
  kal_uint8   Effect;
  kal_uint8   Brightness;
  kal_uint32  sceneMode;//FOR HDR 
  kal_uint16  SensorMode;
} t4k28yuv_status;

//static struct T4K28YUVStatus t4k28yuv_status;
static DEFINE_SPINLOCK(T4K28_drv_lock);

UINT8 T4K28YUVPixelClockDivider=0;
kal_uint32 T4K28YUV_sensor_pclk=52000000;;
kal_uint32 T4K28YUV_PV_pclk = 5525;

kal_uint32 T4K28YUV_CAP_pclk = 6175;

kal_uint16 T4K28YUV_pv_exposure_lines=0x100, T4K28YUV_g_iBackupExtraExp = 0, T4K28YUV_extra_exposure_lines = 0;

kal_uint16 T4K28YUV_sensor_id=0;

MSDK_SENSOR_CONFIG_STRUCT T4K28YUVSensorConfigData;

kal_uint32 T4K28YUV_FAC_SENSOR_REG;
kal_uint16 T4K28YUV_sensor_flip_value;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT T4K28YUVSensorCCT[] = CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT T4K28YUVSensorReg[ENGINEER_END] = CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

typedef enum
{
  T4K28_720P,
  T4K28_2M,
} T4K28_RES_TYPE;

T4K28_RES_TYPE T4K28YUV_g_RES = T4K28_720P;

typedef enum
{
  T4K28_MODE_PREVIEW,
  T4K28_MODE_CAPTURE,
} T4K28_MODE;
T4K28_MODE g_iT4K28YUV_Mode = T4K28_MODE_PREVIEW;

typedef enum
{
  AE_enable,
  AE_lock,
  AE_unlock,
  AE_disable,
} AE_status;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define T4K28YUV_write_cmos_sensor(addr, para) iWriteReg((u16)addr , (u32)para , 1, T4K28_WRITE_ID)

//static UINT32 g_sensorAfStatus = 0;

#define PROFILE 1

#if PROFILE
static struct timeval T4K28YUV_ktv1, T4K28YUV_ktv2;
inline void T4K28YUV_imgSensorProfileStart(void)
{
    do_gettimeofday(&T4K28YUV_ktv1);
}

inline void T4K28YUV_imgSensorProfileEnd(char *tag)
{
    unsigned long TimeIntervalUS;
    do_gettimeofday(&T4K28YUV_ktv2);

    TimeIntervalUS = (T4K28YUV_ktv2.tv_sec - T4K28YUV_ktv1.tv_sec) * 1000000 + (T4K28YUV_ktv2.tv_usec - T4K28YUV_ktv1.tv_usec);
    SENSORDB("[%s]Profile = %lu\n",tag, TimeIntervalUS);
}
#else
inline static void T4K28YUV_imgSensorProfileStart() {}
inline static void T4K28YUV_imgSensorProfileEnd(char *tag) {}
#endif

kal_uint16 T4K28YUV_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,T4K28_WRITE_ID);
    return get_byte;
}

#define Sleep(ms)	mdelay(ms)

void T4K28YUV_write_shutter(kal_uint16 shutter)
{
	return;
}   /* write_T4K28_shutter */

static kal_uint16 T4K28YUVReg2Gain(const kal_uint8 iReg)
{
	return iReg;
}

static kal_uint8 T4K28YUVGain2Reg(const kal_uint16 iGain)
{
	return iGain;
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40); again_min = 0x1a; again_max = 0x9c
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_SetGain(UINT16 gain)
{
    kal_uint16 a_gain = gain;
	T4K28YUV_write_cmos_sensor(0x350a, ((a_gain >> 8) & 0xFF));//ESLIMMODE/ROOMDET/-/-/MAG[11:8]
	T4K28YUV_write_cmos_sensor(0x350b, (a_gain & 0xFF));//MAG[7:0]
	return;
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_GetGain
*
* DESCRIPTION
*    This function is to get sensor anolog gain.
*
* PARAMETERS
*    anolog gain : sensor anolog gain: again_min = 0x1a; again_max = 0x9c
*
* RETURNS
*    anolog gain.
*
* GLOBALS AFFECTED
*
*************************************************************************/

kal_uint16 T4K28YUV_GetGain(void)
{
	kal_uint16 a_gain;

	a_gain = T4K28YUV_GetGain();
	a_gain = (T4K28YUV_read_cmos_sensor(0x3561) << 8) | T4K28YUV_read_cmos_sensor(0x3562);

	return a_gain;
}

/*************************************************************************
* FUNCTION
*    read_T4K28YUV_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/

kal_uint16 read_T4K28YUV_gain(void)
{
    kal_uint16 a_gain, d_gain;

	a_gain = T4K28YUV_GetGain();
	d_gain = (T4K28YUV_read_cmos_sensor(0x3563) << 8) | T4K28YUV_read_cmos_sensor(0x3564);
    return a_gain;
}/* read_T4K28YUV_gain */

/*************************************************************************
* FUNCTION
*    write_T4K28YUV_gain
*
* DESCRIPTION
*    This function is to write anolog gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void write_T4K28YUV_gain(kal_uint16 gain)
{
    T4K28YUV_SetGain(gain);
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_camera_para_to_sensor
*
* DESCRIPTION
*    update sensor register from camera_para
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_camera_para_to_sensor(void)
{
    kal_uint32 i;
    for(i = 0; 0xFFFFFFFF != T4K28YUVSensorReg[i].Addr; i++)
    {
        T4K28YUV_write_cmos_sensor(T4K28YUVSensorReg[i].Addr, T4K28YUVSensorReg[i].Para);
    }
    for(i = ENGINEER_START_ADDR; 0xFFFFFFFF != T4K28YUVSensorReg[i].Addr; i++)
    {
        T4K28YUV_write_cmos_sensor(T4K28YUVSensorReg[i].Addr, T4K28YUVSensorReg[i].Para);
    }
    for(i = FACTORY_START_ADDR; i < FACTORY_END_ADDR; i++)
    {
        T4K28YUV_write_cmos_sensor(T4K28YUVSensorCCT[i].Addr, T4K28YUVSensorCCT[i].Para);
    }
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_sensor_to_camera_para
*
* DESCRIPTION
*    update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_sensor_to_camera_para(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=T4K28YUVSensorReg[i].Addr; i++)
    {
        T4K28YUVSensorReg[i].Para = T4K28YUV_read_cmos_sensor(T4K28YUVSensorReg[i].Addr);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=T4K28YUVSensorReg[i].Addr; i++)
    {
        T4K28YUVSensorReg[i].Para = T4K28YUV_read_cmos_sensor(T4K28YUVSensorReg[i].Addr);
    }
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_get_sensor_group_count
*
* DESCRIPTION
*    None
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

kal_int32  T4K28YUV_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void T4K28YUV_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
	}
}

void T4K28YUV_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;

    switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Global");
                    temp_addr = PRE_GAIN_INDEX;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"GLOBAL_GAIN");
                    temp_addr = GLOBAL_GAIN_INDEX;
                    break;
                default:
                    ASSERT(0);
            }
            temp_para = T4K28YUVSensorCCT[temp_addr].Para;
            temp_gain = T4K28YUVReg2Gain(temp_para);

            temp_gain=(temp_gain * 1000) / BASEGAIN;

            info_ptr->ItemValue = temp_gain;
            info_ptr->IsTrueFalse = KAL_FALSE;
            info_ptr->IsReadOnly = KAL_FALSE;
            info_ptr->IsNeedRestart = KAL_FALSE;
            info_ptr->Min = 1000;
            info_ptr->Max = 15875;
            break;

        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg == ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue = 2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue = 4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue = 6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue = 8;
                    }

                    info_ptr->IsTrueFalse = KAL_FALSE;
                    info_ptr->IsReadOnly = KAL_FALSE;
                    info_ptr->IsNeedRestart = KAL_TRUE;
                    info_ptr->Min = 2;
                    info_ptr->Max = 8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue = T4K28YUV_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse = KAL_FALSE;
                    info_ptr->IsReadOnly = KAL_TRUE;
                    info_ptr->IsNeedRestart = KAL_FALSE;
                    info_ptr->Min = 0;
                    info_ptr->Max = 0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue = 12;
                    info_ptr->IsTrueFalse = KAL_FALSE;
                    info_ptr->IsReadOnly = KAL_TRUE;
                    info_ptr->IsNeedRestart = KAL_FALSE;
                    info_ptr->Min = 0;
                    info_ptr->Max = 0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue = 0;
                    info_ptr->IsTrueFalse = KAL_FALSE;
                    info_ptr->IsReadOnly = KAL_FALSE;
                    info_ptr->IsNeedRestart = KAL_FALSE;
                    info_ptr->Min = 0;
                    info_ptr->Max = 0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue = 0;
                    info_ptr->IsTrueFalse = KAL_FALSE;
                    info_ptr->IsReadOnly = KAL_FALSE;
                    info_ptr->IsNeedRestart = KAL_FALSE;
                    info_ptr->Min = 0;
                    info_ptr->Max = 0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}

kal_bool T4K28YUV_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
   kal_uint16 temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
                case 0:
                    temp_addr = PRE_GAIN_INDEX;
                    break;
                case 1:
                    temp_addr = GLOBAL_GAIN_INDEX;
                    break;
                default:
                    ASSERT(0);
            }

            temp_para = T4K28YUVGain2Reg(ItemValue);

            T4K28YUVSensorCCT[temp_addr].Para = temp_para;
            T4K28YUV_write_cmos_sensor(T4K28YUVSensorCCT[temp_addr].Addr,temp_para);

            T4K28YUV_sensor_gain_base=read_T4K28YUV_gain();

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    if(ItemValue==2)
                    {
                        T4K28YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                        T4K28YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                        T4K28YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
                    }
                    else
                    {
                        T4K28YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
                    }
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    T4K28YUV_FAC_SENSOR_REG=ItemValue;
                    break;
                case 1:
                    T4K28YUV_write_cmos_sensor(T4K28YUV_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}

#if 0
static void T4K28YUV_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
    kal_uint16 LinesOneframe;
    kal_uint16 PixelsOneline = T4K28_FULL_PERIOD_PIXEL_NUMS;

    if(T4K28_720P == T4K28YUV_g_RES)
    {
        PixelsOneline = (T4K28_PV_PERIOD_PIXEL_NUMS_HTS + iPixels);
        LinesOneframe =iLines + T4K28_PV_PERIOD_LINE_NUMS_VTS;
        if(T4K28YUV_MPEG4_encode_mode == KAL_FALSE)
            T4K28YUV_CURRENT_FRAME_LINES = iLines + T4K28_PV_PERIOD_LINE_NUMS_VTS;
    }
    else if(T4K28_2M == T4K28YUV_g_RES)
    {
        PixelsOneline = T4K28_FULL_PERIOD_PIXEL_NUMS_HTS + iPixels;
		LinesOneframe =iLines + T4K28_FULL_PERIOD_LINE_NUMS_VTS;
        T4K28YUV_CURRENT_FRAME_LINES = iLines + T4K28_FULL_PERIOD_LINE_NUMS_VTS;
    }
    if(iPixels)
    {
    	T4K28YUV_write_cmos_sensor(0x3015, (PixelsOneline >> 8) & 0xFF);
    	T4K28YUV_write_cmos_sensor(0x3016, PixelsOneline & 0xFF);
    }
    if(iLines)
    {
    	T4K28YUV_write_cmos_sensor(0x3017, (LinesOneframe >> 8) & 0xFF);
    	T4K28YUV_write_cmos_sensor(0x3018, LinesOneframe & 0xFF);
    }
}
#endif

/*************************************************************************
* FUNCTION
*    T4K28YUV_set_AE_mode
*
* DESCRIPTION
*    ae enable or manual ae
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void T4K28YUV_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 temp_AE_reg = 0;

    if (AE_enable == KAL_TRUE) {
        //turn on AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg | 0x80);
    } else {
        //turn off AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg & ~0x80);
    }
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_set_AE_status
*
* DESCRIPTION
*    AE enable, manual AE or lock AE
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void T4K28YUV_set_AE_status(kal_uint8 AE_status)
{
    kal_uint8 temp_AE_reg = 0;

    if(AE_status == AE_enable) {
        //turn on AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, ((temp_AE_reg | 0x80) & ~0x20));
		//temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
		//T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg & ~0x20);
    } else if(AE_status == AE_lock) {
        //Lock AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg | 0x20);
    } else if(AE_status == AE_unlock) {
        //Lock AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg & ~0x20);
    } 
	else {
		//turn off AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, ((temp_AE_reg & ~0x80)|0x20));
	//	temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
    //    T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg | 0x20);
    }
}


/*************************************************************************
* FUNCTION
*    T4K28YUV_set_AWB_mode
*
* DESCRIPTION
*    awb enable or manual awb
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void T4K28YUV_set_AWB_mode(kal_bool AWB_enable)
{
	kal_uint8 temp_AWB_reg = 0;

    if(AWB_enable == KAL_TRUE)
    {
      //enable Auto WB
      temp_AWB_reg = T4K28YUV_read_cmos_sensor(0x3500);
      T4K28YUV_write_cmos_sensor(0x3500, temp_AWB_reg | 0x40);
    }
    else
    {
       //turn off AWB
       temp_AWB_reg = T4K28YUV_read_cmos_sensor(0x3500);
       T4K28YUV_write_cmos_sensor(0x3500, temp_AWB_reg & ~0x40);
    }
}

void T4K28YUV_Sensor_Init_vga(int lines);
void T4K28YUV_set_2M_init(void);

/*************************************************************************
* FUNCTION
*    T4K28YUV_Sensor_Init
*
* DESCRIPTION
*    init sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void T4K28YUV_Sensor_Init(int lines)

{

    SENSORDB("lln:: T4K28YUV_Sensor_Init, use T4K28YUV_Sensor_Init_vga");
	

   //T4K28YUV_Sensor_Init_vga(lines);
   T4K28YUV_Sensor_Init_vga(0);
	

    SENSORDB("Init Success \n");
}/*  T4K28YUV_Sensor_Init  */






void T4K28YUV_Sensor_Init_vga(int lines)
{
	//int i;
	//parser_enable = KAL_FALSE;

	if(lines)
	{
		//for(i=0;i<lines;i++)
		//T4K28YUV_write_cmos_sensor(pgParaBuffer[i].regAddr,pgParaBuffer[i].value);
		//parser_enable = KAL_TRUE;
	}
	else
	{
	    #if 0
		T4K28YUV_write_cmos_sensor(0x3010,0x00);//-/-/-/-/-/-/-/MODSEL
		T4K28YUV_write_cmos_sensor(0x3000,0x08);//[RO] VERNUM[15:8]
		T4K28YUV_write_cmos_sensor(0x3001,0x40);//[RO] VERNUM[7:0]
		T4K28YUV_write_cmos_sensor(0x3002,0x00);//[RO] PISO[15:8]
		T4K28YUV_write_cmos_sensor(0x3003,0x00);//[RO] PISO[7:0]
		T4K28YUV_write_cmos_sensor(0x3004,0x00);//-/-/-/-/-/-/-/PISO_MSKN
		T4K28YUV_write_cmos_sensor(0x3005,0x66);//[RO] FRAME_COUNT[7:0]
		T4K28YUV_write_cmos_sensor(0x3011,0x00);//-/-/-/-/-/-/VREVON/HREVON
		T4K28YUV_write_cmos_sensor(0x3012,0x02);//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD
		T4K28YUV_write_cmos_sensor(0x3014,0x03);//-/-/-/-/-/-/PARALLEL_OUT_SW[1:0]
		T4K28YUV_write_cmos_sensor(0x3015,0x07);//
		T4K28YUV_write_cmos_sensor(0x3016,0x16);//H_COUNT[7:0]
		T4K28YUV_write_cmos_sensor(0x3017,0x03);//-/-/-/V_COUNT[12:8]
		T4K28YUV_write_cmos_sensor(0x3018,0x00);//V_COUNT[7:0]
		T4K28YUV_write_cmos_sensor(0x3019,0x00);//-/-/-/-/-/-/-/SCALE_M[8]
		T4K28YUV_write_cmos_sensor(0x301A,0x10);//SCALE_M[7:0]
		T4K28YUV_write_cmos_sensor(0x301B,0x00);//-/-/-/V_ANABIN/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x301C,0x01);//-/-/-/-/-/-/-/SCALING_MODE
		T4K28YUV_write_cmos_sensor(0x3020,0x06);//-/-/-/-/-/HOUTPIX[10:8]
		T4K28YUV_write_cmos_sensor(0x3021,0x40);//HOUTPIX[7:0]
		T4K28YUV_write_cmos_sensor(0x3022,0x04);//-/-/-/-/-/VOUTPIX[10:8]
		T4K28YUV_write_cmos_sensor(0x3023,0xB0);//VOUTPIX[7:0]
		T4K28YUV_write_cmos_sensor(0x3025,0x00);//-/-/-/-/-/-/VCROP[9:8]
		T4K28YUV_write_cmos_sensor(0x3026,0x00);//VCROP[7:0]
		T4K28YUV_write_cmos_sensor(0x3027,0x00);//-/-/-/-/OUTPUT_FORMAT[3:0]
		T4K28YUV_write_cmos_sensor(0x302C,0x00);//-/-/-/-/TEST_HC/VSYNC_PH/HSYNC_PH/ESYNC_SW
		T4K28YUV_write_cmos_sensor(0x302D,0x00);//-/-/H_PRESET[13:8]
		T4K28YUV_write_cmos_sensor(0x302E,0x00);//H_PRESET[7:0]
		T4K28YUV_write_cmos_sensor(0x302F,0x00);//V_PRESET[15:8]
		T4K28YUV_write_cmos_sensor(0x3030,0x00);//V_PRESET[7:0]
		T4K28YUV_write_cmos_sensor(0x3031,0x02);//-/-/-/-/-/HREG_HRST_POS[10:8]
		T4K28YUV_write_cmos_sensor(0x3032,0x00);//HREG_HRST_POS[7:0]
		T4K28YUV_write_cmos_sensor(0x3033,0x83);//OADJ_AT_SW/OADJ_AT_SET/OADJ_MN_EN/-/OADJ_AT_DLY_H[3:0]
		T4K28YUV_write_cmos_sensor(0x3034,0x01);//OADJ_AT_STA_V[3:0]/OADJ_AT_WID_V[3:0]
		T4K28YUV_write_cmos_sensor(0x3037,0x00);//-/-/-/-/-/-/VCI[1:0]
		T4K28YUV_write_cmos_sensor(0x303C,0x80);//SDA_DRVUP/-/-/-/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x303E,0x01);//-/-/-/-/-/-/-/DCLK_POL
		T4K28YUV_write_cmos_sensor(0x303F,0x00);//-/-/CKST_SEL[1:0]/CKSP_SEL[1:0]/CKMR_SEL[1:0]
		T4K28YUV_write_cmos_sensor(0x3040,0x80);//PISO_STP_X_SW/SLEEP_SW/VCO_STP_SW/PHY_PWRON_SW/PISO_STP_X_MN/SLEEP
		T4K28YUV_write_cmos_sensor(0x3044,0x02);//-/-/-/-/PLLEV_SEL/PCMODE/ICP_PCH/ICP_NCH
		T4K28YUV_write_cmos_sensor(0x3045,0x04);//-/-/-/-/-/PRE_PLL_CNTL[2:0]
		T4K28YUV_write_cmos_sensor(0x3046,0x00);//-/-/-/-/-/-/-/PLL_MULTI[8]
		T4K28YUV_write_cmos_sensor(0x3047,0x82);//PLL_MULTI[7:0]
		T4K28YUV_write_cmos_sensor(0x3048,0x01);//-/-/-/-/VT_SYS_CNTL[3:0]
		T4K28YUV_write_cmos_sensor(0x3049,0x01);//-/-/-/-/OP_SYS_CNTL[3:0]
		T4K28YUV_write_cmos_sensor(0x304A,0x0A);//-/-/-/-/VT_PIX_CNTL[3:0]
		T4K28YUV_write_cmos_sensor(0x304B,0x0A);//-/-/-/-/ST_CLK_CNTL[3:0]
		T4K28YUV_write_cmos_sensor(0x304C,0x00);//-/-/-/-/-/-/PLL_CNTL[1:0]
		T4K28YUV_write_cmos_sensor(0x304E,0x01);//-/-/-/-/-/BST_CNTL[2:0]
		T4K28YUV_write_cmos_sensor(0x3050,0x60);//VCO_CONV[2:0]/-/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x3051,0x82);//VCO_EN/-/-/-/-/DIVRSTX_SEL/DIVRSTX_SW/DIVRSTX_MN
		T4K28YUV_write_cmos_sensor(0x3052,0x10);//-/-/REGVD_SEL[1:0]/-/AMON0_SEL[2:0]
		T4K28YUV_write_cmos_sensor(0x3053,0x00);//-/AUTO_ICP_R_SEL/-/-/-/HS_SR_CNT/-/LPFR_SEL
		T4K28YUV_write_cmos_sensor(0x3055,0x84);//CAMP15_EN/-/-/-/-/VOUT15_SEL[2:0]
		T4K28YUV_write_cmos_sensor(0x3056,0x02);//CLDET_EN/-/-/-/-/-/BIAS_SEL/-
		T4K28YUV_write_cmos_sensor(0x3059,0x18);//EXTCLK_FRQ_MHZ[15:8]
		T4K28YUV_write_cmos_sensor(0x305A,0x00);//EXTCLK_FRQ_MHZ[7:0]
		T4K28YUV_write_cmos_sensor(0x3068,0xF0);//SY_SPARE1[7:0]
		T4K28YUV_write_cmos_sensor(0x3069,0xF0);//SY_SPARE2[7:0]
		T4K28YUV_write_cmos_sensor(0x306C,0x06);//[RO] -/-/-/-/XCROP_SC[11:8]
		T4K28YUV_write_cmos_sensor(0x306D,0x40);//[RO] XCROP_SC[7:0]
		T4K28YUV_write_cmos_sensor(0x306E,0x00);//[RO] -/-/-/-/XSTA_SC[11:8]
		T4K28YUV_write_cmos_sensor(0x306F,0x04);//[RO] XSTA_SC[7:0]
		T4K28YUV_write_cmos_sensor(0x3070,0x06);//[RO] -/-/-/-/XEND_SC[11:8]
		T4K28YUV_write_cmos_sensor(0x3071,0x43);//[RO] XEND_SC[7:0]
		T4K28YUV_write_cmos_sensor(0x3072,0x04);//[RO] -/-/-/-/-/YCROP_SC[10:8]
		T4K28YUV_write_cmos_sensor(0x3073,0xB0);//[RO] YCROP_SC[7:0]
		T4K28YUV_write_cmos_sensor(0x3074,0x00);//[RO] -/-/-/-/-/YSTA_SC[10:8]
		T4K28YUV_write_cmos_sensor(0x3075,0x04);//[RO] YSTA_SC[7:0]
		T4K28YUV_write_cmos_sensor(0x3076,0x04);//[RO] -/-/-/-/-/YEND_SC[10:8]
		T4K28YUV_write_cmos_sensor(0x3077,0xB3);//[RO] YEND_SC[7:0]
		T4K28YUV_write_cmos_sensor(0x307F,0x03);//[RO] -/-/-/-/-/-/mode_vdl[1:0]
		T4K28YUV_write_cmos_sensor(0x3080,0x70);//CLKPOSPRD[4:0]/-/-/-
		T4K28YUV_write_cmos_sensor(0x3081,0x28);//HSPREPRD[4:0]/-/-/-
		T4K28YUV_write_cmos_sensor(0x3082,0x60);//HS0PRD[4:0]/-/-/-
		T4K28YUV_write_cmos_sensor(0x3083,0x48);//HSTRLPRD[4:0]/-/-/-
		T4K28YUV_write_cmos_sensor(0x3084,0x40);//CLKTRIPRD[4:0]/-/-/-
		T4K28YUV_write_cmos_sensor(0x3085,0x28);//CLKPREPRD[4:0]/-/-/-
		T4K28YUV_write_cmos_sensor(0x3086,0xF8);//CLK0PRD[4:0]/-/-/-
		T4K28YUV_write_cmos_sensor(0x3087,0x38);//TLPXPRD[4:0]/-/-/-
		T4K28YUV_write_cmos_sensor(0x3088,0x03);//-/-/-/-/-/-/LNKBTWK_ON/LNKBT_ON
		T4K28YUV_write_cmos_sensor(0x3089,0x02);//MSB_LBRATE[15:8]
		T4K28YUV_write_cmos_sensor(0x308A,0x58);//MSB_LBRATE[7:0]
		T4K28YUV_write_cmos_sensor(0x3091,0x00);//-/-/-/-/-/-/CLKULPS/ESCREQ
		T4K28YUV_write_cmos_sensor(0x3092,0x18);//[RO] RO_CRC[15:8]
		T4K28YUV_write_cmos_sensor(0x3093,0xA1);//[RO] RO_CRC[7:0]
		T4K28YUV_write_cmos_sensor(0x3095,0x78);//ESCDATA[7:0]
		T4K28YUV_write_cmos_sensor(0x3097,0x00);//LVDS_D1_DELAY[3:0]/LVDS_CLK_DELAY[3:0]
		T4K28YUV_write_cmos_sensor(0x3098,0x40);//-/PHASE_ADJUST[2:0]/-/-/-/LP_SR_CNT
		T4K28YUV_write_cmos_sensor(0x309A,0x00);//TEST_PN9/-/-/-/OP_TEST[3:0]
		T4K28YUV_write_cmos_sensor(0x309B,0x00);//T_VALUE1[7:0]
		T4K28YUV_write_cmos_sensor(0x309D,0x00);//-/-/-/-/-/-/-/MIPI_CLK_MODE
		T4K28YUV_write_cmos_sensor(0x309E,0x00);//-/-/-/-/LB_TEST_CLR/LB_TEST_EN/-/LB_MODE
		T4K28YUV_write_cmos_sensor(0x309F,0x00);//[RO] -/RO_LPBT_ERR_K/RO_LPBT_ERR_M/RO_LPBT_CNT[4:0]
		T4K28YUV_write_cmos_sensor(0x30A0,0x02);//
		T4K28YUV_write_cmos_sensor(0x30A1,0x00);//FIFODLY[7:0]
		T4K28YUV_write_cmos_sensor(0x30A2,0xA7);//NUMWAKE[7:0]
		T4K28YUV_write_cmos_sensor(0x30A3,0x00);//TX_TRIGOPT/-/TRIG_Z5_X/-/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x30A4,0xFF);//TRIG_DUMMY[7:0]
		T4K28YUV_write_cmos_sensor(0x30A5,0x80);//CLKPRE2/-/-/-/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x30A6,0xFF);//NUMINIT[7:0]
		T4K28YUV_write_cmos_sensor(0x30A7,0x00);//EN_PHASE_SEL[1:0]/-/-/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x30A8,0x01);//MIPI_FS_CD[3:0]/MIPI_FE_CD[3:0]
		T4K28YUV_write_cmos_sensor(0x30F1,0x00);//-/-/-/T_OUTSEL[4:0]
		T4K28YUV_write_cmos_sensor(0x30F2,0x00);//-/-/-/-/-/-/-/T_VWIDTH_LAT_ON
		T4K28YUV_write_cmos_sensor(0x30FE,0x80);//T_PROTECT/-/-/-/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x3100,0xD2);//POSLFIX/NEGLFIX/NEGLEAKCUT/BOOSTEN_L/BSTREADEV/POSBSTSEL/READVDSEL
		T4K28YUV_write_cmos_sensor(0x3101,0xD3);//RSTVDSEL_AZS/POSBSTCNT[2:0]/-/POSBSTHG[2:0]
		T4K28YUV_write_cmos_sensor(0x3102,0x95);//NEGBSTCNT[3:0]/-/POSBSTGA[2:0]
		T4K28YUV_write_cmos_sensor(0x3103,0x80);//TAUREADEN/-/-/-/-/-/-/LNOBMODE
		T4K28YUV_write_cmos_sensor(0x3104,0x31);//GDMOSBGREN/-/VSIGDRSEL[1:0]/GDMOSCNT[3:0]
		T4K28YUV_write_cmos_sensor(0x3105,0x04);//-/-/-/-/KBIASCNT[3:0]
		T4K28YUV_write_cmos_sensor(0x3106,0x23);//-/-/DRADRVI[1:0]/-/DRADRVLV[2:0]
		T4K28YUV_write_cmos_sensor(0x3107,0x20);//TESTDACEN/-/VREFSWG[1:0]/-/-/-/VREFDLYCNT
		T4K28YUV_write_cmos_sensor(0x3108,0x7B);//S1CNT[3:0]/CBIASIB/-/CBIASIA[1:0]
		T4K28YUV_write_cmos_sensor(0x3109,0x80);//BOOSTEN_R/-/-/IDLINGOFFEN/-/-/EDGETESTEN[1:0]
		T4K28YUV_write_cmos_sensor(0x310A,0x00);//-/-/-/-/-/SENSEMODE[2:0]
		T4K28YUV_write_cmos_sensor(0x310B,0x00);//SPARE[1:0]/-/-/ANAMON1_SEL[3:0]
		T4K28YUV_write_cmos_sensor(0x3110,0x11);//-/-/-/FBC_SUBSMPL/-/-/-/BIN_MODE
		T4K28YUV_write_cmos_sensor(0x3111,0x11);//-/-/ES_MODE[1:0]/-/-/-/ESREAD_ALT_OFF
		T4K28YUV_write_cmos_sensor(0x3112,0x00);//-/-/-/-/-/-/-/DIS_MODE
		T4K28YUV_write_cmos_sensor(0x3113,0x00);//-/-/-/-/-/-/-/ALLZEROSET_ON
		T4K28YUV_write_cmos_sensor(0x3114,0x10);//-/-/ALLZEROSET_1ST_ON[1:0]/-/-/-/ALLZEROSET_CHG_ON
		T4K28YUV_write_cmos_sensor(0x3115,0x22);//-/-/LTCH_POS[1:0]/-/RODATA_U/DMR_ON/ALLREAD_ON
		T4K28YUV_write_cmos_sensor(0x3120,0x08);//BSC_OFF/-/-/-/SADR_1PULSE/-/-/DRESET_1PULSE
		T4K28YUV_write_cmos_sensor(0x3121,0x13);//-/BSCPULSE_INTVL[6:0]
		T4K28YUV_write_cmos_sensor(0x3122,0x33);//-/DRESET_1U[6:0]
		T4K28YUV_write_cmos_sensor(0x3123,0x0E);//-/-/-/DRESET_W[4:0]
		T4K28YUV_write_cmos_sensor(0x3124,0x26);//-/FTLSNS_1U[2:0]/FTLSNS_W[3:0]
		T4K28YUV_write_cmos_sensor(0x3125,0x00);//-/-/-/-/-/SADR_1U[2:0]
		T4K28YUV_write_cmos_sensor(0x3126,0x0C);//-/-/-/SADR_1W[4:0]
		T4K28YUV_write_cmos_sensor(0x3127,0x08);//-/-/SADR_2W[5:0]
		T4K28YUV_write_cmos_sensor(0x3128,0x80);//AUTO_READ_W/-/-/-/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x3129,0x65);//ESREAD_1U[7:0]
		T4K28YUV_write_cmos_sensor(0x312A,0x27);//-/ESREAD_2U[6:0]
		T4K28YUV_write_cmos_sensor(0x312B,0x77);//-/ESREAD_1W[6:0]
		T4K28YUV_write_cmos_sensor(0x312C,0x77);//-/ESREAD_2W[6:0]
		T4K28YUV_write_cmos_sensor(0x312D,0x1A);//-/-/-/ESTGRESET_D[4:0]
		T4K28YUV_write_cmos_sensor(0x312E,0xB8);//VSIGPU_U[7:0]
		T4K28YUV_write_cmos_sensor(0x312F,0x38);//VSIGPU_W[7:0]
		T4K28YUV_write_cmos_sensor(0x3130,0x80);//VSIGPU_LOW/-/-/-/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x3131,0x33);//-/ROREAD_U[6:0]
		T4K28YUV_write_cmos_sensor(0x3132,0x63);//-/ROREAD_W[6:0]
		T4K28YUV_write_cmos_sensor(0x3133,0x00);//-/-/-/-/-/-/-/ROTGRESET_U[8]
		T4K28YUV_write_cmos_sensor(0x3134,0xDD);//ROTGRESET_U[7:0]
		T4K28YUV_write_cmos_sensor(0x3135,0x07);//EXTD_ROTGRESET/-/-/-/ROTGRESET_W[3:0]
		T4K28YUV_write_cmos_sensor(0x3136,0xB7);//ZEROSET_U[7:0]
		T4K28YUV_write_cmos_sensor(0x3137,0x11);//-/-/ZEROSET_W[5:0]
		T4K28YUV_write_cmos_sensor(0x3138,0x0B);//RSTDRAIN_HIGH/-/RSTDRAIN_D[5:0]
		T4K28YUV_write_cmos_sensor(0x313B,0x0A);//-/-/-/RSTDRAIN_U[4:0]
		T4K28YUV_write_cmos_sensor(0x313C,0x05);//-/-/-/RSTDRAIN3_U[4:0]
		T4K28YUV_write_cmos_sensor(0x313D,0x01);//DRCUT_SIGIN/DRCUT_DMY_OFF/-/DRCUT_U[4:0]
		T4K28YUV_write_cmos_sensor(0x313E,0x62);//DRCUT_NW[7:0]
		T4K28YUV_write_cmos_sensor(0x313F,0x85);//DRCUT_VDER_W[7:0]
		T4K28YUV_write_cmos_sensor(0x3140,0x01);//BGRSH_OFF/-/BGRSH_U[1:0]/-/-/BGRSH_W[1:0]
		T4K28YUV_write_cmos_sensor(0x3141,0x40);//VSIGDR_MODE[1:0]/-/VSIGDR_U[1:0]/-/VSIGDR_D[1:0]
		T4K28YUV_write_cmos_sensor(0x3142,0x80);//S1_2PULSES/-/S1_ALL_HIGH/S1_STOPBST/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x3143,0x22);//-/S1_1U[6:0]
		T4K28YUV_write_cmos_sensor(0x3144,0x3E);//S1_1W[7:0]
		T4K28YUV_write_cmos_sensor(0x3145,0x32);//-/S1_2U[6:0]
		T4K28YUV_write_cmos_sensor(0x3146,0x2E);//-/S1_2W[6:0]
		T4K28YUV_write_cmos_sensor(0x3147,0x23);//-/-/S1_AW[5:0]
		T4K28YUV_write_cmos_sensor(0x3148,0x22);//-/S1A_USHIFT[2:0]/-/S1A_DSHIFT[2:0]
		T4K28YUV_write_cmos_sensor(0x3149,0x11);//-/S1W_USHIFT[2:0]/-/S1W_DSHIFT[2:0]
		T4K28YUV_write_cmos_sensor(0x314A,0x6B);//S2_W[7:0]
		T4K28YUV_write_cmos_sensor(0x314B,0x30);//-/S3_W[6:0]
		T4K28YUV_write_cmos_sensor(0x314C,0x69);//S4_W[7:0]
		T4K28YUV_write_cmos_sensor(0x314D,0x80);//CDS_STOPBST/-/S4_AD[5:0]
		T4K28YUV_write_cmos_sensor(0x314E,0x31);//BSTCKLFIX_HIGH/BSTCKLFIX_1U[2:0]/-/BSTCKLFIX_1D[2:0]
		T4K28YUV_write_cmos_sensor(0x314F,0x32);//-/BSTCKLFIX_2U[2:0]/-/BSTCKLFIX_2D[2:0]
		T4K28YUV_write_cmos_sensor(0x3150,0x32);//BSTCKLFIX_1P/BSTCKLFIX_3U[2:0]/-/BSTCKLFIX_3D[2:0]
		T4K28YUV_write_cmos_sensor(0x3151,0x03);//-/-/-/INTEN_CU[4:0]
		T4K28YUV_write_cmos_sensor(0x3152,0x0C);//-/-/INTEN_CW[5:0]
		T4K28YUV_write_cmos_sensor(0x3153,0xB3);//INTEN_SU[7:0]
		T4K28YUV_write_cmos_sensor(0x3154,0x20);//-/-/INTEN_AU[5:0]
		T4K28YUV_write_cmos_sensor(0x3155,0x13);//-/-/BSTCKLFIX_1P_U[1:0]/INTEN_AD[3:0]
		T4K28YUV_write_cmos_sensor(0x3156,0x66);//INTRS_CU[7:0]
		T4K28YUV_write_cmos_sensor(0x3157,0x02);//-/-/-/INTRS_CD[4:0]
		T4K28YUV_write_cmos_sensor(0x3158,0x03);//-/-/-/-/INTRS_SU[3:0]
		T4K28YUV_write_cmos_sensor(0x3159,0x01);//-/-/-/-/-/-/INTRS_SD[1:0]
		T4K28YUV_write_cmos_sensor(0x315A,0x16);//-/-/INTRS_AU[5:0]
		T4K28YUV_write_cmos_sensor(0x315B,0x10);//-/-/-/INTRS_AW[4:0]
		T4K28YUV_write_cmos_sensor(0x315C,0x00);//-/DRKCLIP_U[2:0]/CLR_GCNTR_D[3:0]
		T4K28YUV_write_cmos_sensor(0x315D,0x44);//DRKCLIP_PRE_U[3:0]/-/DRKCLIP_PRE_D[2:0]
		T4K28YUV_write_cmos_sensor(0x315E,0x1B);//-/HPL2_NU[6:0]
		T4K28YUV_write_cmos_sensor(0x315F,0x52);//-/HPL2_AU[6:0]
		T4K28YUV_write_cmos_sensor(0x3160,0x00);//EXT_AG_ON/-/-/-/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x3161,0x03);//EXT_AG_PARAM[15:8]
		T4K28YUV_write_cmos_sensor(0x3162,0x00);//EXT_AG_PARAM[7:0]
		T4K28YUV_write_cmos_sensor(0x3163,0xFF);//TDAC_INT[7:0]
		T4K28YUV_write_cmos_sensor(0x3164,0x00);//TDAC_MIN[7:0]
		T4K28YUV_write_cmos_sensor(0x3165,0x01);//TDAC_SWD[1:0]/-/-/TDAC_STEP[3:0]
		T4K28YUV_write_cmos_sensor(0x3166,0x00);//DACS_INT[7:0]
		T4K28YUV_write_cmos_sensor(0x3167,0xFF);//DACS_MAX[7:0]
		T4K28YUV_write_cmos_sensor(0x3168,0x01);//DACS_SWD[1:0]/-/-/DACS_STEP[3:0]
		T4K28YUV_write_cmos_sensor(0x3169,0x00);//AG_TEST/-/LATCH_TEST[1:0]/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x3180,0x00);//SIGIN_ON/VOB_DISP/HOB_DISP/-/-/-/-/-
		T4K28YUV_write_cmos_sensor(0x3181,0x20);//DATA_LATCH_SEL/-/EN_CBLK/OADJ_MODE[1:0]/-/OADJ_INTEN_W[1:0]
		T4K28YUV_write_cmos_sensor(0x3182,0x40);//-/FBC_MODE[2:0]/-/FBC_PERIOD_SEL[2:0]
		T4K28YUV_write_cmos_sensor(0x3183,0x96);//CP1_FBC_SEL/-/CP1_OADJ_U[5:0]
		T4K28YUV_write_cmos_sensor(0x3184,0x30);//-/FBC_LEVEL[6:0]
		T4K28YUV_write_cmos_sensor(0x3185,0x8F);//FBC_ON/-/FAST_FBC_ON/-/SLOW_FBC_RANGE[3:0]
		T4K28YUV_write_cmos_sensor(0x3186,0x31);//-/FBC_START_CND[2:0]/-/FBC_SUSP_CND[2:0]
		T4K28YUV_write_cmos_sensor(0x3187,0x06);//-/-/-/FBC_STOP_CND[4:0]
		T4K28YUV_write_cmos_sensor(0x3188,0x0C);//FBC_OUT_LINES[7:0]
		T4K28YUV_write_cmos_sensor(0x3189,0x44);//FBC_HIGH_OBAVE[3:0]/FBC_LOW_OBAVE[3:0]
		T4K28YUV_write_cmos_sensor(0x318A,0x42);//FBC_IN_LINES[3:0]/FBC_IN_RANGE[3:0]
		T4K28YUV_write_cmos_sensor(0x318B,0x0B);//FBC_STOP_RANGE/FBC_START_RANGE/-/-/FBC_AG_COEF_DIV[1:0]/FBC_OUT_CN
		T4K28YUV_write_cmos_sensor(0x318C,0x11);//FBC_AG_CONT_COEF[7:0]
		T4K28YUV_write_cmos_sensor(0x318D,0xAA);//EN_ES_CHANGE/-/EN_AG_CHANGE/-/EN_OADJ_CHANGE/-/PSCLAMP_1ST_ON/-
		T4K28YUV_write_cmos_sensor(0x318E,0x40);//-/CLAMPSHIFT_SEL[2:0]/AG_CHANGE_VAL[3:0]
		T4K28YUV_write_cmos_sensor(0x318F,0x30);//CLAMPSHIFT_VAL[7:0]
		T4K28YUV_write_cmos_sensor(0x3190,0x03);//-/-/-/IDLINGOFF_U[4:0]
		T4K28YUV_write_cmos_sensor(0x3191,0x01);//-/-/-/-/-/-/IDLINGOFF_D[1:0]
		T4K28YUV_write_cmos_sensor(0x3192,0x00);//-/-/-/-/EXT_DACPARAM[11:8]
		T4K28YUV_write_cmos_sensor(0x3193,0x00);//EXT_DACPARAM[7:0]
		T4K28YUV_write_cmos_sensor(0x3194,0x00);//EXT_CLAMP_ON/EXT_OFFSET_ON/-/-/RET_INT_OFFSET/BIT_VERIF_TIME[2:0]
		T4K28YUV_write_cmos_sensor(0x3195,0x00);//-/-/-/-/-/-/BLADJ_MODE[1:0]
		T4K28YUV_write_cmos_sensor(0x3196,0x00);//-/-/-/-/-/BLADJ_COEF[10:8]
		T4K28YUV_write_cmos_sensor(0x3197,0xDE);//BLADJ_COEF[7:0]
		T4K28YUV_write_cmos_sensor(0x3198,0x00);//-/-/-/-/BLADJ_INTC[11:8]
		T4K28YUV_write_cmos_sensor(0x3199,0x00);//BLADJ_INTC[7:0]
		T4K28YUV_write_cmos_sensor(0x319A,0x00);//OB_AVE_MON_SEL[1:0]/-/ST_CKI[4:0]
		T4K28YUV_write_cmos_sensor(0x319B,0x00);//ST_RSVD_REG[7:0]
		T4K28YUV_write_cmos_sensor(0x319C,0x16);//-/-/IDLINGOFF_AU[5:0]
		T4K28YUV_write_cmos_sensor(0x319D,0x0A);//-/-/-/IDLINGOFF_AW[4:0]
		T4K28YUV_write_cmos_sensor(0x31A0,0xBF);//EXT_HCNT_MAX_ON/-/HCNT_MAX_FIXVAL[13:8]
		T4K28YUV_write_cmos_sensor(0x31A1,0xFF);//HCNT_MAX_FIXVAL[7:0]
		T4K28YUV_write_cmos_sensor(0x31A2,0x11);//HCNT_MAX_MODE/-/-/FBC_MASK_FRAM_EN/-/MASK_FRAME_NUM[2:0]
		T4K28YUV_write_cmos_sensor(0x31B0,0x00);//[RO] fbc_operation/-/-/-/-/-/ob_ave[9:8]
		T4K28YUV_write_cmos_sensor(0x31B1,0x41);//[RO] ob_ave[7:0]
		T4K28YUV_write_cmos_sensor(0x31B2,0x1A);//[RO] -/-/clamp_param[13:8]
		T4K28YUV_write_cmos_sensor(0x31B3,0x61);//[RO] clamp_param[7:0]
		T4K28YUV_write_cmos_sensor(0x31B4,0x03);//[RO] -/-/-/-/offset_param[11:8]
		T4K28YUV_write_cmos_sensor(0x31B5,0x2D);//[RO] offset_param[7:0]
		T4K28YUV_write_cmos_sensor(0x31B6,0x1A);//[RO] -/-/ps_clamp_param[13:8]
		T4K28YUV_write_cmos_sensor(0x31B7,0x83);//[RO] ps_clamp_param[7:0]
		T4K28YUV_write_cmos_sensor(0x31B8,0x00);//[RO] fbcline_cnt[15:8]
		T4K28YUV_write_cmos_sensor(0x31B9,0x03);//[RO] fbcline_cnt[7:0]
		T4K28YUV_write_cmos_sensor(0x31BA,0x3F);//[RO] -/-/hcnt_max[13:8]
		T4K28YUV_write_cmos_sensor(0x31BB,0xFF);//[RO] hcnt_max[7:0]
		T4K28YUV_write_cmos_sensor(0x3300,0xFF);//HLNRSW/DANSASW[1:0]/LDNRSW/VDNRSW/ABPCSW/WBPSW/BBPSW
		T4K28YUV_write_cmos_sensor(0x3301,0x35);//-/-/ANRSW/PEDAJSW/-/LSSCSW/LSSCAG_SW/PWBSW
		T4K28YUV_write_cmos_sensor(0x3303,0x40);//BLKADJ[7:0]
		T4K28YUV_write_cmos_sensor(0x3304,0x00);//BLKR[3:0]/BLKGR[3:0]
		T4K28YUV_write_cmos_sensor(0x3305,0x00);//BLKGB[3:0]/BLKB[3:0]
		T4K28YUV_write_cmos_sensor(0x3306,0x30);//-/-/OBWIDTH[5:0]
		T4K28YUV_write_cmos_sensor(0x3307,0x00);//-/-/-/-/-/OBCLIP/BLLVSEL/DANSA2L
		T4K28YUV_write_cmos_sensor(0x3308,0x87);//LDNRLIM1[3:0]/LDNRLIM0[3:0]
		T4K28YUV_write_cmos_sensor(0x330A,0x60);//LDNRGA1[3:0]/LDNRGA0[3:0]
		T4K28YUV_write_cmos_sensor(0x330B,0x56);//VDNRLIM1[3:0]/VDNRLIM0[3:0]
		T4K28YUV_write_cmos_sensor(0x330D,0x79);//VDNRGA1[3:0]/VDNRGA0[3:0]
		T4K28YUV_write_cmos_sensor(0x330E,0xFF);//BBPLV0[7:0]
		T4K28YUV_write_cmos_sensor(0x330F,0xFF);//BBPLV1[7:0]
		T4K28YUV_write_cmos_sensor(0x3310,0xFF);//WBPLV0[7:0]
		T4K28YUV_write_cmos_sensor(0x3311,0x7F);//WBPLV1[7:0]
		T4K28YUV_write_cmos_sensor(0x3312,0x0F);//WBPOFS[7:0]
		T4K28YUV_write_cmos_sensor(0x3313,0x0F);//BBPOFS[7:0]
		T4K28YUV_write_cmos_sensor(0x3314,0x02);//-/-/-/-/CONTLV1[3:0]
		T4K28YUV_write_cmos_sensor(0x3315,0xC0);//CONTLV2[7:0]
		T4K28YUV_write_cmos_sensor(0x3316,0x18);//ANREG_LEV0[7:0]
		T4K28YUV_write_cmos_sensor(0x3317,0x08);//ANREG_LEV1[7:0]
		T4K28YUV_write_cmos_sensor(0x3318,0x60);//ANRG0[7:0]
		T4K28YUV_write_cmos_sensor(0x3319,0x90);//ANRG1[7:0]
		T4K28YUV_write_cmos_sensor(0x331B,0x00);//GRPED0[3:0]/GBPED0[3:0]
		T4K28YUV_write_cmos_sensor(0x331C,0x00);//RPED0[3:0]/BPED0[3:0]
		T4K28YUV_write_cmos_sensor(0x331D,0x00);//GRPED1[3:0]/GBPED1[3:0]
		T4K28YUV_write_cmos_sensor(0x331E,0x00);//RPED1[3:0]/BPED1[3:0]
		T4K28YUV_write_cmos_sensor(0x3322,0x002B);	//,//PWBGAINGR[7:0];
		T4K28YUV_write_cmos_sensor(0x3323,0x002B);	//,//PWBGAINGB[7:0];
		T4K28YUV_write_cmos_sensor(0x3324,0x0000);	//,//PWBGAINR[7:0];
		T4K28YUV_write_cmos_sensor(0x3325,0x006A);	//,//PWBGAINB[7:0];
		T4K28YUV_write_cmos_sensor(0x3327,0x0000);	//,//-/-/-/-/-/-/LIPOL/CSPOL;
		T4K28YUV_write_cmos_sensor(0x3328,0x0000);	//,//-/-/-/-/-/LS1SIG[2:0];
		T4K28YUV_write_cmos_sensor(0x3329,0x0090);	//,//LSHOFG[7:0];
		T4K28YUV_write_cmos_sensor(0x332A,0x009F);	//,//LSHOFR[7:0];
		T4K28YUV_write_cmos_sensor(0x332B,0x00A4);	//,//LSHOFB[7:0];
		T4K28YUV_write_cmos_sensor(0x332C,0x0070);	//,//LSVOFG[7:0];
		T4K28YUV_write_cmos_sensor(0x332D,0x0070);	//,//LSVOFR[7:0];
		T4K28YUV_write_cmos_sensor(0x332E,0x0070);	//,//LSVOFB[7:0];
		T4K28YUV_write_cmos_sensor(0x332F,0x0000);	//,//LSALUG[7:0];
		T4K28YUV_write_cmos_sensor(0x3330,0x0000);	//,//LSALUR[7:0];
		T4K28YUV_write_cmos_sensor(0x3331,0x0008);	//,//LSALUB[7:0];
		T4K28YUV_write_cmos_sensor(0x3332,0x0000);	//,//LSARUG[7:0];
		T4K28YUV_write_cmos_sensor(0x3333,0x0000);	//,//LSARUR[7:0];
		T4K28YUV_write_cmos_sensor(0x3334,0x0000);	//,//LSARUB[7:0];
		T4K28YUV_write_cmos_sensor(0x3335,0x0000);	//,//LSALDG[7:0];
		T4K28YUV_write_cmos_sensor(0x3336,0x0000);	//,//LSALDR[7:0];
		T4K28YUV_write_cmos_sensor(0x3337,0x0008);	//,//LSALDB[7:0];
		T4K28YUV_write_cmos_sensor(0x3338,0x0000);	//,//LSARDG[7:0];
		T4K28YUV_write_cmos_sensor(0x3339,0x0000);	//,//LSARDR[7:0];
		T4K28YUV_write_cmos_sensor(0x333A,0x0000);	//,//LSARDB[7:0];
		T4K28YUV_write_cmos_sensor(0x333B,0x002C);	//,//LSBLG[7:0];
		T4K28YUV_write_cmos_sensor(0x333C,0x0058);	//,//LSBLR[7:0];
		T4K28YUV_write_cmos_sensor(0x333D,0x0018);	//,//LSBLB[7:0];
		T4K28YUV_write_cmos_sensor(0x333E,0x0030);	//,//LSBRG[7:0];
		T4K28YUV_write_cmos_sensor(0x333F,0x0050);	//,//LSBRR[7:0];
		T4K28YUV_write_cmos_sensor(0x3340,0x0020);	//,//LSBRB[7:0];
		T4K28YUV_write_cmos_sensor(0x3341,0x0030);	//,//LSCUG[7:0];
		T4K28YUV_write_cmos_sensor(0x3342,0x0048);	//,//LSCUR[7:0];
		T4K28YUV_write_cmos_sensor(0x3343,0x0020);	//,//LSCUB[7:0];
		T4K28YUV_write_cmos_sensor(0x3344,0x0030);	//,//LSCDG[7:0];
		T4K28YUV_write_cmos_sensor(0x3345,0x0048);	//,//LSCDR[7:0];
		T4K28YUV_write_cmos_sensor(0x3346,0x0020);	//,//LSCDB[7:0];
		T4K28YUV_write_cmos_sensor(0x3347,0x0000);	//,//LSBRG_L[3:0]/LSBLG_L[3:0];
		T4K28YUV_write_cmos_sensor(0x3348,0x0000);	//,//LSBR R_L[3:0]/LSBL R_L[3:0];
		T4K28YUV_write_cmos_sensor(0x3349,0x0000);	//,//LSBRB_L[3:0]/LSBLB_L[3:0];
		T4K28YUV_write_cmos_sensor(0x334A,0x0000);	//,//LSCDG_L[3:0]/LSCUG_L[3:0];
		T4K28YUV_write_cmos_sensor(0x334B,0x0000);	//,//LSCDR_L[3:0]/LSCUR_L[3:0];
		T4K28YUV_write_cmos_sensor(0x334C,0x0000);	//,//LSCDB_L[3:0]/LSCUB_L[3:0];
		T4K28YUV_write_cmos_sensor(0x334D,0x40);//LSHCNT_MPY[7:0]
		T4K28YUV_write_cmos_sensor(0x334E,0x00);//-/-/-/-/LSVCNT_MPY[11:8]
		T4K28YUV_write_cmos_sensor(0x334F,0xA0);//LSVCNT_MPY[7:0]
		T4K28YUV_write_cmos_sensor(0x3350,0x03);//-/-/-/-/-/-/LSMGSEL[1:0]
		T4K28YUV_write_cmos_sensor(0x335F,0x00);//-/-/-/-/-/-/-/TESTPAT_SW
		T4K28YUV_write_cmos_sensor(0x3360,0x00);//PP_SPARE[7:0]
		T4K28YUV_write_cmos_sensor(0x3400,0xA4);//LMCC_SW/-/PGC_SW/-/-/YSHP_SW/-/-
		T4K28YUV_write_cmos_sensor(0x3401,0x7F);//-/BRIGHT_SW/LCNT_SW/NLCNT_SW/UVLPF_SW/UVSHP_SW/FCS_SW/UVCRING_SW
		T4K28YUV_write_cmos_sensor(0x3402,0x00);//LCS_LI/LCS_CS/-/-/-/EFECT_SEL[2:0]
		T4K28YUV_write_cmos_sensor(0x3403,0x00);//LCS_CRING[7:0]
		T4K28YUV_write_cmos_sensor(0x3404,0x3A);//LM_RMG[7:0]
		T4K28YUV_write_cmos_sensor(0x3405,0xE3);//LM_RMB[7:0]
		T4K28YUV_write_cmos_sensor(0x3406,0x22);//LM_GMR[7:0]
		T4K28YUV_write_cmos_sensor(0x3407,0x25);//LM_GMB[7:0]
		T4K28YUV_write_cmos_sensor(0x3408,0x17);//LM_BMR[7:0]
		T4K28YUV_write_cmos_sensor(0x3409,0x5C);//LM_BMG[7:0]
		T4K28YUV_write_cmos_sensor(0x340A,0x20);//-/GAM01P[6:0]
		T4K28YUV_write_cmos_sensor(0x340B,0x20);//-/GAM02P[6:0]
		T4K28YUV_write_cmos_sensor(0x340C,0x3B);//-/GAM03P[6:0]
		T4K28YUV_write_cmos_sensor(0x340D,0x2E);//-/GAM04P[6:0]
		T4K28YUV_write_cmos_sensor(0x340E,0x26);//-/GAM05P[6:0]
		T4K28YUV_write_cmos_sensor(0x340F,0x3F);//-/GAM06P[6:0]
		T4K28YUV_write_cmos_sensor(0x3410,0x34);//-/GAM07P[6:0]
		T4K28YUV_write_cmos_sensor(0x3411,0x2D);//-/GAM08P[6:0]
		T4K28YUV_write_cmos_sensor(0x3412,0x28);//-/GAM09P[6:0]
		T4K28YUV_write_cmos_sensor(0x3413,0x47);//-/GAM10P[6:0]
		T4K28YUV_write_cmos_sensor(0x3414,0x3E);//-/GAM11P[6:0]
		T4K28YUV_write_cmos_sensor(0x3415,0x6A);//-/GAM12P[6:0]
		T4K28YUV_write_cmos_sensor(0x3416,0x5A);//-/GAM13P[6:0]
		T4K28YUV_write_cmos_sensor(0x3417,0x50);//-/GAM14P[6:0]
		T4K28YUV_write_cmos_sensor(0x3418,0x48);//-/GAM15P[6:0]
		T4K28YUV_write_cmos_sensor(0x3419,0x42);//-/GAM16P[6:0]
		T4K28YUV_write_cmos_sensor(0x341B,0x10);//-/-/BLK_ADJ[5:0]
		T4K28YUV_write_cmos_sensor(0x341C,0x40);//CbG_MAT[7:0]
		T4K28YUV_write_cmos_sensor(0x341D,0x70);//CbB_MAT[7:0]
		T4K28YUV_write_cmos_sensor(0x341E,0xc4);//Cb_GAIN[7:0]
		T4K28YUV_write_cmos_sensor(0x341F,0x88);//CrR_MAT[7:0]
		T4K28YUV_write_cmos_sensor(0x3420,0x80);//CrG_MAT[7:0]
		T4K28YUV_write_cmos_sensor(0x3421,0xc4);//Cr_GAIN[7:0]
		T4K28YUV_write_cmos_sensor(0x3422,0x00);//Cbr_MGAIN[7:0]
		T4K28YUV_write_cmos_sensor(0x3423,0x0F);//-/-/-/-/CbP_GAIN1[3:0]
		T4K28YUV_write_cmos_sensor(0x3424,0x0F);//-/-/-/-/CbM_GAIN1[3:0]
		T4K28YUV_write_cmos_sensor(0x3425,0x0F);//-/-/-/-/CrP_GAIN1[3:0]
		T4K28YUV_write_cmos_sensor(0x3426,0x0F);//-/-/-/-/CrM_GAIN1[3:0]
		T4K28YUV_write_cmos_sensor(0x342B,0x0C);//-/-/DTL_CRNG0[5:0]
		T4K28YUV_write_cmos_sensor(0x342C,0x20);//-/-/DTL_CRNG1[5:0]
		T4K28YUV_write_cmos_sensor(0x342D,0x90);//DTL_PG0[7:0]
		T4K28YUV_write_cmos_sensor(0x342E,0x64);//DTL_PG1[7:0]
		T4K28YUV_write_cmos_sensor(0x342F,0xB0);//DTL_MG0[7:0]
		T4K28YUV_write_cmos_sensor(0x3430,0x80);//DTL_MG1[7:0]
		T4K28YUV_write_cmos_sensor(0x3431,0x1E);//DTL_LIM_PH[7:0]
		T4K28YUV_write_cmos_sensor(0x3432,0x1E);//DTL_LIM_MH[7:0]
		T4K28YUV_write_cmos_sensor(0x3433,0x0A);//DTL_LIM_PLS[7:0]
		T4K28YUV_write_cmos_sensor(0x3434,0x0A);//DTL_LIM_MLS[7:0]
		T4K28YUV_write_cmos_sensor(0x3435,0x15);//DTL_LIM_PLE[7:0]
		T4K28YUV_write_cmos_sensor(0x3436,0x15);//DTL_LIM_MLE[7:0]
		T4K28YUV_write_cmos_sensor(0x343F,0x10);//BRIGHT1[7:0]
		T4K28YUV_write_cmos_sensor(0x3440,0xF0);//BRIGHT0[7:0]
		T4K28YUV_write_cmos_sensor(0x3441,0x85);//LCONT_LEV[7:0]
		T4K28YUV_write_cmos_sensor(0x3442,0x60);//BLK_KNEE[7:0]
		T4K28YUV_write_cmos_sensor(0x3443,0x98);//WHT_KNEE[7:0]
		T4K28YUV_write_cmos_sensor(0x3444,0x02);//-/-/-/-/BLK_CONT[3:0]
		T4K28YUV_write_cmos_sensor(0x3446,0x02);//-/-/-/-/WHT_CONT[3:0]
		T4K28YUV_write_cmos_sensor(0x3447,0x00);//NLOFF_AG[7:0]
		T4K28YUV_write_cmos_sensor(0x3448,0x00);//-/-/COL_LV[5:0]
		T4K28YUV_write_cmos_sensor(0x3449,0x00);//-/-/-/-/COL_ED_G[3:0]
		T4K28YUV_write_cmos_sensor(0x344A,0x00);//UV_CRING1[3:0]/UV_CRING0[3:0]
		T4K28YUV_write_cmos_sensor(0x344B,0x00);//UV_SHP_AG[7:0]
		T4K28YUV_write_cmos_sensor(0x344C,0x20);//FCS_EDG_CORING[7:0]
		T4K28YUV_write_cmos_sensor(0x344D,0xFF);//FCS_EDG_LIM[7:0]
		T4K28YUV_write_cmos_sensor(0x344E,0x0F);//-/-/-/-/FCS_EDG_GAIN[3:0]
		T4K28YUV_write_cmos_sensor(0x344F,0x20);//FCS_LV[7:0]
		T4K28YUV_write_cmos_sensor(0x3450,0x80);//FCS_GAIN[7:0]
		T4K28YUV_write_cmos_sensor(0x3451,0x0F);//-/-/-/-/FCS_AG[3:0]
		T4K28YUV_write_cmos_sensor(0x3452,0x55);//UNICOFSU[7:0]
		T4K28YUV_write_cmos_sensor(0x3453,0x49);//UNICOFSV[7:0]
		T4K28YUV_write_cmos_sensor(0x3454,0x6A);//SEPIAOFSU[7:0]
		T4K28YUV_write_cmos_sensor(0x3455,0x93);//SEPIAOFSV[7:0]
		T4K28YUV_write_cmos_sensor(0x345C,0x00);//MP_VLAT_OFF/-/-/-/-/-/TEST_IN_SW[1:0]
		T4K28YUV_write_cmos_sensor(0x345D,0x00);//TEST_AG[7:0]
		T4K28YUV_write_cmos_sensor(0x345E,0x00);//MP_SPARE[7:0]
		T4K28YUV_write_cmos_sensor(0x3500,0xC1);//ALCSW/AWBSW/ALCLOCK/-/ESLIMMODE/ROOMDET/-/ALCLIMMODE
		T4K28YUV_write_cmos_sensor(0x3501,0x01);//-/-/-/-/-/-/ALCAIM[9:8]
		T4K28YUV_write_cmos_sensor(0x3502,0x30);//ALCAIM[7:0]
		T4K28YUV_write_cmos_sensor(0x3503,0x1A);//AGMIN[7:0]
		T4K28YUV_write_cmos_sensor(0x3504,0x00);//-/-/-/-/AGMAX[11:8]
		T4K28YUV_write_cmos_sensor(0x3505,0x9C);//AGMAX[7:0]
		T4K28YUV_write_cmos_sensor(0x3506,0x04);//MES[15:8]
		T4K28YUV_write_cmos_sensor(0x3507,0xD0);//MES[7:0]
		T4K28YUV_write_cmos_sensor(0x3508,0x00);//MMES[15:8]
		T4K28YUV_write_cmos_sensor(0x3509,0x6b);//MMES[7:0]
		T4K28YUV_write_cmos_sensor(0x350A,0x00);//-/-/-/-/MAG[11:8]
		T4K28YUV_write_cmos_sensor(0x350B,0x20);//MAG[7:0]
		T4K28YUV_write_cmos_sensor(0x350C,0x00);//MDG[7:0]
		T4K28YUV_write_cmos_sensor(0x350D,0x15);//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0]
		T4K28YUV_write_cmos_sensor(0x350E,0x15);//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0]
		T4K28YUV_write_cmos_sensor(0x350F,0x51);//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0]
		T4K28YUV_write_cmos_sensor(0x3510,0x50);//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-
		T4K28YUV_write_cmos_sensor(0x3511,0x90);//UDMODE[1:0]/-/UPDNSPD[4:0]
		T4K28YUV_write_cmos_sensor(0x3512,0x10);//ALCOFS[2:0]/NEARSPD[4:0]
		T4K28YUV_write_cmos_sensor(0x3513,0x00);//ALC_ANJ_OFS[7:0]
		T4K28YUV_write_cmos_sensor(0x3514,0x00);//-/-/-/-/-/-/ALC_ANJ_SPD[1:0]
		T4K28YUV_write_cmos_sensor(0x3515,0x08);//ALCFRZLV1[7:0]//0x10
		T4K28YUV_write_cmos_sensor(0x3516,0x08);//ALCFRZLV2[7:0]//0x10
		T4K28YUV_write_cmos_sensor(0x3517,0x00);//ALCFRZTIM[7:0]
		T4K28YUV_write_cmos_sensor(0x3518,0x00);//ALCSIGMIN[7:0]
		T4K28YUV_write_cmos_sensor(0x3519,0xFF);//ALCSIGMAX[7:0]
		T4K28YUV_write_cmos_sensor(0x351A,0xC0);//SATSET[7:0]
		T4K28YUV_write_cmos_sensor(0x351B,0x98);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]
		T4K28YUV_write_cmos_sensor(0x351C,0x68);//FLLONGON/FRMSPD[1:0]/FL600S[12:8]
		T4K28YUV_write_cmos_sensor(0x351D,0xF5);//FL600S[7:0]
		T4K28YUV_write_cmos_sensor(0x351E,0x16);//ACFDET/AC60M/FLMANU/ACDETDLY/MSKLINE[1:0]/ACDPWAIT[1:0]
		T4K28YUV_write_cmos_sensor(0x351F,0x80);//FL600AT/TTL1V_ON/-/-/-/-/-/FLDETM[8]
		T4K28YUV_write_cmos_sensor(0x3520,0x26);//FLDETM[7:0]
		T4K28YUV_write_cmos_sensor(0x3521,0x02);//ACDET1LV[7:0]
		T4K28YUV_write_cmos_sensor(0x3522,0x08);//ACDET2LV[7:0]
		T4K28YUV_write_cmos_sensor(0x3523,0x0C);//SATGUP/SATFRZ/SAT1VDET/-/DETSEL[3:0]
		T4K28YUV_write_cmos_sensor(0x3524,0x01);//AWBONDOT[2:0]/-/-/-/WBMRG[9:8]
		T4K28YUV_write_cmos_sensor(0x3525,0x5A);//WBMRG[7:0]
		T4K28YUV_write_cmos_sensor(0x3526,0x2C);//CAREASEL[1:0]/AREAMODE[1:0]/HEXSW/YGATESW/WBMGG[9:8]
		T4K28YUV_write_cmos_sensor(0x3527,0xE0);//WBMGG[7:0]
		T4K28YUV_write_cmos_sensor(0x3528,0xfD);//SQ1SW/SQ1POL/SQ2SW/SQ2POL/SQ3SW/SQ3POL/WBMBG[9:8]
		T4K28YUV_write_cmos_sensor(0x3529,0x72);//WBMBG[7:0]
		T4K28YUV_write_cmos_sensor(0x352A,0xDE);//WBGRMAX[7:0]
		T4K28YUV_write_cmos_sensor(0x352B,0x22);//WBGRMIN[7:0]
		T4K28YUV_write_cmos_sensor(0x352C,0xD0);//WBGBMAX[7:0]
		T4K28YUV_write_cmos_sensor(0x352D,0x10);//WBGBMIN[7:0]
		T4K28YUV_write_cmos_sensor(0x352E,0x28);//RBCUT0H[7:0]
		T4K28YUV_write_cmos_sensor(0x352F,0xDc);//RBCUT0L[7:0]
		T4K28YUV_write_cmos_sensor(0x3530,0x2f);//-/RYCUT0P[6:0]
		T4K28YUV_write_cmos_sensor(0x3531,0x40);//-/RYCUT0N[6:0]
		T4K28YUV_write_cmos_sensor(0x3532,0x3a);//-/BYCUT0P[6:0]
		T4K28YUV_write_cmos_sensor(0x3533,0x2e);//-/BYCUT0N[6:0]
		T4K28YUV_write_cmos_sensor(0x3534,0x28);//RBCUT0HL[7:0]
		T4K28YUV_write_cmos_sensor(0x3535,0xDc);//RBCUT0LL[7:0]
		T4K28YUV_write_cmos_sensor(0x3536,0x2f);//-/RYCUT0PL[6:0]
		T4K28YUV_write_cmos_sensor(0x3537,0x40);//-/RYCUT0NL[6:0]
		T4K28YUV_write_cmos_sensor(0x3538,0x3a);//-/BYCUT0PL[6:0]
		T4K28YUV_write_cmos_sensor(0x3539,0x2e);//-/BYCUT0NL[6:0]
		T4K28YUV_write_cmos_sensor(0x353A,0x3C);//RYCUT1H[7:0]
		T4K28YUV_write_cmos_sensor(0x353B,0x35);//-/RYCUT1L[6:0]
		T4K28YUV_write_cmos_sensor(0x353C,0xDE);//BYCUT1H[7:0]
		T4K28YUV_write_cmos_sensor(0x353D,0x33);//-/BYCUT1L[6:0]
		T4K28YUV_write_cmos_sensor(0x353E,0xDC);//RYCUT2H[7:0]
		T4K28YUV_write_cmos_sensor(0x353F,0x38);//-/RYCUT2L[6:0]
		T4K28YUV_write_cmos_sensor(0x3540,0x55);//BYCUT2H[7:0]
		T4K28YUV_write_cmos_sensor(0x3541,0x38);//-/BYCUT2L[6:0]
		T4K28YUV_write_cmos_sensor(0x3542,0x60);//RYCUT3H[7:0]
		T4K28YUV_write_cmos_sensor(0x3543,0x3F);//-/RYCUT3L[6:0]
		T4K28YUV_write_cmos_sensor(0x3544,0xBD);//BYCUT3H[7:0]
		T4K28YUV_write_cmos_sensor(0x3545,0x46);//-/BYCUT3L[6:0]
		T4K28YUV_write_cmos_sensor(0x3546,0xF0);//YGATEH[7:0]
		T4K28YUV_write_cmos_sensor(0x3547,0x10);//YGATEL[7:0]
		T4K28YUV_write_cmos_sensor(0x3548,0x84);//CGRANGE[1:0]/-/AWBHUECOR/AWBSPD[3:0]
		T4K28YUV_write_cmos_sensor(0x3549,0x00);//YGATE_SEL/YGATE_DATA[1:0]/AWBULV[4:0]
		T4K28YUV_write_cmos_sensor(0x354A,0x00);//AWBFZTIM[2:0]/AWBVLV[4:0]
		T4K28YUV_write_cmos_sensor(0x354B,0x00);//AWBSFTU[7:0]
		T4K28YUV_write_cmos_sensor(0x354C,0x00);//AWBSFTV[7:0]
		T4K28YUV_write_cmos_sensor(0x354D,0x00);//AWBWAIT[7:0]
		T4K28YUV_write_cmos_sensor(0x354E,0x00);//-/-/-/-/CGCNGSLP[2:0]/CGCNGLV[8]
		T4K28YUV_write_cmos_sensor(0x354F,0x00);//CGCNGLV[7:0]
		T4K28YUV_write_cmos_sensor(0x3550,0x00);//SPLMKON/SPLMKBL/FAREAMK/CAREAMK/CGATEMK/-/-/-
		T4K28YUV_write_cmos_sensor(0x3551,0x03);//-/-/-/-/-/SPLADRH[10:8]
		T4K28YUV_write_cmos_sensor(0x3552,0x28);//SPLADRH[7:0]
		T4K28YUV_write_cmos_sensor(0x3553,0x20);//MKFLKON/MKFLKSPD[1:0]/-/-/SPLADRV[10:8]
		T4K28YUV_write_cmos_sensor(0x3554,0x60);//SPLADRV[7:0]
		T4K28YUV_write_cmos_sensor(0x3555,0xF0);//AU_SPARE[7:0]
		T4K28YUV_write_cmos_sensor(0x355D,0x01);//[RO] APL_DONE/-/-/-/-/-/APL_O[9:8]
		T4K28YUV_write_cmos_sensor(0x355E,0x1C);//[RO] APL_O[7:0]
		T4K28YUV_write_cmos_sensor(0x355F,0x03);//[RO] ALC_ES[15:8]
		T4K28YUV_write_cmos_sensor(0x3560,0x00);//[RO] ALC_ES[7:0]
		T4K28YUV_write_cmos_sensor(0x3561,0x00);//[RO] -/-/-/-/ALC_AG[11:8]
		T4K28YUV_write_cmos_sensor(0x3562,0x2C);//[RO] ALC_AG[7:0]
		T4K28YUV_write_cmos_sensor(0x3563,0x00);//[RO] -/-/-/-/-/-/ALC_DG[9:8]
		T4K28YUV_write_cmos_sensor(0x3564,0x50);//[RO] ALC_DG[7:0]
		T4K28YUV_write_cmos_sensor(0x3565,0x00);//[RO] -/-/FR_TIM[5:0]
		T4K28YUV_write_cmos_sensor(0x3566,0xD2);//[RO] ALC_OK/AC60M/DET_SIG/FL_ESLIM/-/-/mag_vg[9:8]
		T4K28YUV_write_cmos_sensor(0x3567,0x2E);//[RO] mag_vg[7:0]
		T4K28YUV_write_cmos_sensor(0x3568,0x00);//[RO] -/-/-/-/-/-/AVE_USIG[9:8]
		T4K28YUV_write_cmos_sensor(0x3569,0x00);//[RO] AVE_USIG[7:0]
		T4K28YUV_write_cmos_sensor(0x356A,0x00);//[RO] -/-/-/-/-/-/AVE_VSIG[9:8]
		T4K28YUV_write_cmos_sensor(0x356B,0x00);//[RO] AVE_VSIG[7:0]
		T4K28YUV_write_cmos_sensor(0x356C,0xFF);//[RO] NUM_UVON[15:8]
		T4K28YUV_write_cmos_sensor(0x356D,0xFF);//[RO] NUM_UVON[7:0]
		T4K28YUV_write_cmos_sensor(0x356E,0x01);//[RO] -/-/-/-/-/-/AWBGAINR[9:8]
		T4K28YUV_write_cmos_sensor(0x356F,0x72);//[RO] AWBGAINR[7:0]
		T4K28YUV_write_cmos_sensor(0x3570,0x01);//[RO] -/-/-/-/-/-/AWBGAING[9:8]
		T4K28YUV_write_cmos_sensor(0x3571,0x00);//[RO] AWBGAING[7:0]
		T4K28YUV_write_cmos_sensor(0x3572,0x01);//[RO] -/-/-/-/-/-/AWBGAINB[9:8]
		T4K28YUV_write_cmos_sensor(0x3573,0x4D);//[RO] AWBGAINB[7:0]
		T4K28YUV_write_cmos_sensor(0x3574,0x01);//[RO] -/-/-/-/-/-/SET_ALCLV[9:8]
		T4K28YUV_write_cmos_sensor(0x3575,0x20);//[RO] SET_ALCLV[7:0]
		T4K28YUV_write_cmos_sensor(0x3576,0x45);//[RO] MKY_DATA[7:0]
		T4K28YUV_write_cmos_sensor(0x3577,0x10);//[RO] MKU_DATA[7:0]
		T4K28YUV_write_cmos_sensor(0x3578,0xEE);//[RO] MKV_DATA[7:0]
		T4K28YUV_write_cmos_sensor(0x3579,0x09);//[RO] -/-/-/FL600A[12:8]
		T4K28YUV_write_cmos_sensor(0x357A,0x99);//[RO] FL600A[7:0]
		T4K28YUV_write_cmos_sensor(0x357B,0x00);//[RO] AG_CONT[7:0]
		T4K28YUV_write_cmos_sensor(0x357C,0xE0);//[RO] AU_RO_SPARE[7:0]
		T4K28YUV_write_cmos_sensor(0x357D,0x00);//[RO] -/-/-/-/-/-/-/ALCCLR
		T4K28YUV_write_cmos_sensor(0x3900,0x00);//OTP_STA/-/-/-/OTP_VMOD/OTP_VERIFY/OTP_WREC/OTP_ENBL
		T4K28YUV_write_cmos_sensor(0x3901,0x07);//OTP_GVRD/-/-/-/-/OTP_PCLK[2:0]
		T4K28YUV_write_cmos_sensor(0x3902,0x00);//[RO] OTP_ISTS[2:0]/-/-/OTP_TOE/OTP_VE/OTP_BUSY
		T4K28YUV_write_cmos_sensor(0x3903,0x00);//-/-/-/-/-/-/OTP_ADRS[1:0]
		T4K28YUV_write_cmos_sensor(0x3904,0x00);//OTP_DATA0[7:0]
		T4K28YUV_write_cmos_sensor(0x3905,0x00);//OTP_DATA1[7:0]
		T4K28YUV_write_cmos_sensor(0x3906,0x00);//OTP_DATA2[7:0]
		T4K28YUV_write_cmos_sensor(0x3907,0x00);//OTP_DATA3[7:0]
		T4K28YUV_write_cmos_sensor(0x3908,0x00);//OTP_DATA4[7:0]
		T4K28YUV_write_cmos_sensor(0x3909,0x00);//OTP_DATA5[7:0]
		T4K28YUV_write_cmos_sensor(0x390A,0x00);//OTP_DATA6[7:0]
		T4K28YUV_write_cmos_sensor(0x390B,0x00);//OTP_DATA7[7:0]
		T4K28YUV_write_cmos_sensor(0x390C,0x00);//OTP_PERR1/OTP_PERR2/OTP_CLRE/OTP_TEST[4:0]
		T4K28YUV_write_cmos_sensor(0x30F0,0x00);//T_MONDIRECT/-/-/T_DACTEST/-/T_TMOSEL[2:0]
		T4K28YUV_write_cmos_sensor(0x3010,0x01);//-/-/-/-/-/-/-/MODSEL
		//parser_enable = KAL_FALSE;
		#else
		//20140106
		T4K28YUV_write_cmos_sensor(0x3010,0x00);//-/-/-/-/-/-/-/MODSEL;
		T4K28YUV_write_cmos_sensor(0x3000,0x08);//[RO] VERNUM[15:8];
		T4K28YUV_write_cmos_sensor(0x3001,0x40);//[RO] VERNUM[7:0];
		T4K28YUV_write_cmos_sensor(0x3002,0x00);//[RO] PISO[15:8];
		T4K28YUV_write_cmos_sensor(0x3003,0x00);//[RO] PISO[7:0];
		T4K28YUV_write_cmos_sensor(0x3004,0x00);//-/-/-/-/-/-/-/PISO_MSKN;
		T4K28YUV_write_cmos_sensor(0x3005,0x66);//[RO] FRAME_COUNT[7:0];
		T4K28YUV_write_cmos_sensor(0x3011,0x00);//-/-/-/-/-/-/VREVON/HREVON;
		T4K28YUV_write_cmos_sensor(0x3012,0x02);//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD;
		T4K28YUV_write_cmos_sensor(0x3014,0x03);//-/-/-/-/-/-/PARALLEL_OUT_SW[1:0];
		T4K28YUV_write_cmos_sensor(0x3015,0x07);//;
		T4K28YUV_write_cmos_sensor(0x3016,0x16);//H_COUNT[7:0];
		T4K28YUV_write_cmos_sensor(0x3017,0x03);//-/-/-/V_COUNT[12:8];
		T4K28YUV_write_cmos_sensor(0x3018,0x00);//V_COUNT[7:0];
		T4K28YUV_write_cmos_sensor(0x3019,0x00);//-/-/-/-/-/-/-/SCALE_M[8];
		T4K28YUV_write_cmos_sensor(0x301A,0x10);//SCALE_M[7:0];
		T4K28YUV_write_cmos_sensor(0x301B,0x00);//-/-/-/V_ANABIN/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x301C,0x01);//-/-/-/-/-/-/-/SCALING_MODE;
		T4K28YUV_write_cmos_sensor(0x3020,0x06);//-/-/-/-/-/HOUTPIX[10:8];
		T4K28YUV_write_cmos_sensor(0x3021,0x40);//HOUTPIX[7:0];
		T4K28YUV_write_cmos_sensor(0x3022,0x04);//-/-/-/-/-/VOUTPIX[10:8];
		T4K28YUV_write_cmos_sensor(0x3023,0xB0);//VOUTPIX[7:0];
		T4K28YUV_write_cmos_sensor(0x3025,0x00);//-/-/-/-/-/-/VCROP[9:8];
		T4K28YUV_write_cmos_sensor(0x3026,0x00);//VCROP[7:0];
		T4K28YUV_write_cmos_sensor(0x3027,0x00);//-/-/-/-/OUTPUT_FORMAT[3:0];
		T4K28YUV_write_cmos_sensor(0x302C,0x00);//-/-/-/-/TEST_HC/VSYNC_PH/HSYNC_PH/ESYNC_SW;
		T4K28YUV_write_cmos_sensor(0x302D,0x00);//-/-/H_PRESET[13:8];
		T4K28YUV_write_cmos_sensor(0x302E,0x00);//H_PRESET[7:0];
		T4K28YUV_write_cmos_sensor(0x302F,0x00);//V_PRESET[15:8];
		T4K28YUV_write_cmos_sensor(0x3030,0x00);//V_PRESET[7:0];
		T4K28YUV_write_cmos_sensor(0x3031,0x02);//-/-/-/-/-/HREG_HRST_POS[10:8];
		T4K28YUV_write_cmos_sensor(0x3032,0x00);//HREG_HRST_POS[7:0];
		T4K28YUV_write_cmos_sensor(0x3033,0x83);//OADJ_AT_SW/OADJ_AT_SET/OADJ_MN_EN/-/OADJ_AT_DLY_H[3:0];
		T4K28YUV_write_cmos_sensor(0x3034,0x01);//OADJ_AT_STA_V[3:0]/OADJ_AT_WID_V[3:0];
		T4K28YUV_write_cmos_sensor(0x3037,0x00);//-/-/-/-/-/-/VCI[1:0];
		T4K28YUV_write_cmos_sensor(0x303C,0x80);//SDA_DRVUP/-/-/-/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x303E,0x01);//-/-/-/-/-/-/-/DCLK_POL;
		T4K28YUV_write_cmos_sensor(0x303F,0x00);//-/-/CKST_SEL[1:0]/CKSP_SEL[1:0]/CKMR_SEL[1:0];
		T4K28YUV_write_cmos_sensor(0x3040,0x80);//PISO_STP_X_SW/SLEEP_SW/VCO_STP_SW/PHY_PWRON_SW/PISO_STP_X_MN/SLE;
		T4K28YUV_write_cmos_sensor(0x3044,0x02);//-/-/-/-/PLLEV_SEL/PCMODE/ICP_PCH/ICP_NCH;
		T4K28YUV_write_cmos_sensor(0x3045,0x04);//-/-/-/-/-/PRE_PLL_CNTL[2:0];
		T4K28YUV_write_cmos_sensor(0x3046,0x00);//-/-/-/-/-/-/-/PLL_MULTI[8];
		T4K28YUV_write_cmos_sensor(0x3047,0x82);//PLL_MULTI[7:0];
		T4K28YUV_write_cmos_sensor(0x3048,0x01);//-/-/-/-/VT_SYS_CNTL[3:0];
		T4K28YUV_write_cmos_sensor(0x3049,0x01);//-/-/-/-/OP_SYS_CNTL[3:0];
		T4K28YUV_write_cmos_sensor(0x304A,0x0A);//-/-/-/-/VT_PIX_CNTL[3:0];
		T4K28YUV_write_cmos_sensor(0x304B,0x0A);//-/-/-/-/ST_CLK_CNTL[3:0];
		T4K28YUV_write_cmos_sensor(0x304C,0x00);//-/-/-/-/-/-/PLL_CNTL[1:0];
		T4K28YUV_write_cmos_sensor(0x304E,0x01);//-/-/-/-/-/BST_CNTL[2:0];
		T4K28YUV_write_cmos_sensor(0x3050,0x60);//VCO_CONV[2:0]/-/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3051,0x82);//VCO_EN/-/-/-/-/DIVRSTX_SEL/DIVRSTX_SW/DIVRSTX_MN;
		T4K28YUV_write_cmos_sensor(0x3052,0x10);//-/-/REGVD_SEL[1:0]/-/AMON0_SEL[2:0];
		T4K28YUV_write_cmos_sensor(0x3053,0x00);//-/AUTO_ICP_R_SEL/-/-/-/HS_SR_CNT/-/LPFR_SEL;
		T4K28YUV_write_cmos_sensor(0x3055,0x84);//CAMP15_EN/-/-/-/-/VOUT15_SEL[2:0];
		T4K28YUV_write_cmos_sensor(0x3056,0x02);//CLDET_EN/-/-/-/-/-/BIAS_SEL/-;
		T4K28YUV_write_cmos_sensor(0x3059,0x18);//EXTCLK_FRQ_MHZ[15:8];
		T4K28YUV_write_cmos_sensor(0x305A,0x00);//EXTCLK_FRQ_MHZ[7:0];
		T4K28YUV_write_cmos_sensor(0x3068,0xF0);//SY_SPARE1[7:0];
		T4K28YUV_write_cmos_sensor(0x3069,0xF0);//SY_SPARE2[7:0];
		T4K28YUV_write_cmos_sensor(0x306C,0x06);//[RO] -/-/-/-/XCROP_SC[11:8];
		T4K28YUV_write_cmos_sensor(0x306D,0x40);//[RO] XCROP_SC[7:0];
		T4K28YUV_write_cmos_sensor(0x306E,0x00);//[RO] -/-/-/-/XSTA_SC[11:8];
		T4K28YUV_write_cmos_sensor(0x306F,0x04);//[RO] XSTA_SC[7:0];
		T4K28YUV_write_cmos_sensor(0x3070,0x06);//[RO] -/-/-/-/XEND_SC[11:8];
		T4K28YUV_write_cmos_sensor(0x3071,0x43);//[RO] XEND_SC[7:0];
		T4K28YUV_write_cmos_sensor(0x3072,0x04);//[RO] -/-/-/-/-/YCROP_SC[10:8];
		T4K28YUV_write_cmos_sensor(0x3073,0xB0);//[RO] YCROP_SC[7:0];
		T4K28YUV_write_cmos_sensor(0x3074,0x00);//[RO] -/-/-/-/-/YSTA_SC[10:8];
		T4K28YUV_write_cmos_sensor(0x3075,0x04);//[RO] YSTA_SC[7:0];
		T4K28YUV_write_cmos_sensor(0x3076,0x04);//[RO] -/-/-/-/-/YEND_SC[10:8];
		T4K28YUV_write_cmos_sensor(0x3077,0xB3);//[RO] YEND_SC[7:0];
		T4K28YUV_write_cmos_sensor(0x307F,0x03);//[RO] -/-/-/-/-/-/mode_vdl[1:0];
		T4K28YUV_write_cmos_sensor(0x3080,0x70);//CLKPOSPRD[4:0]/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3081,0x28);//HSPREPRD[4:0]/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3082,0x60);//HS0PRD[4:0]/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3083,0x48);//HSTRLPRD[4:0]/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3084,0x40);//CLKTRIPRD[4:0]/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3085,0x28);//CLKPREPRD[4:0]/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3086,0xF8);//CLK0PRD[4:0]/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3087,0x38);//TLPXPRD[4:0]/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3088,0x03);//-/-/-/-/-/-/LNKBTWK_ON/LNKBT_ON;
		T4K28YUV_write_cmos_sensor(0x3089,0x02);//MSB_LBRATE[15:8];
		T4K28YUV_write_cmos_sensor(0x308A,0x58);//MSB_LBRATE[7:0];
		T4K28YUV_write_cmos_sensor(0x3091,0x00);//-/-/-/-/-/-/CLKULPS/ESCREQ;
		T4K28YUV_write_cmos_sensor(0x3092,0x18);//[RO] RO_CRC[15:8];
		T4K28YUV_write_cmos_sensor(0x3093,0xA1);//[RO] RO_CRC[7:0];
		T4K28YUV_write_cmos_sensor(0x3095,0x78);//ESCDATA[7:0];
		T4K28YUV_write_cmos_sensor(0x3097,0x00);//LVDS_D1_DELAY[3:0]/LVDS_CLK_DELAY[3:0];
		T4K28YUV_write_cmos_sensor(0x3098,0x40);//-/PHASE_ADJUST[2:0]/-/-/-/LP_SR_CNT;
		T4K28YUV_write_cmos_sensor(0x309A,0x00);//TEST_PN9/-/-/-/OP_TEST[3:0];
		T4K28YUV_write_cmos_sensor(0x309B,0x00);//T_VALUE1[7:0];
		T4K28YUV_write_cmos_sensor(0x309D,0x00);//-/-/-/-/-/-/-/MIPI_CLK_MODE;
		T4K28YUV_write_cmos_sensor(0x309E,0x00);//-/-/-/-/LB_TEST_CLR/LB_TEST_EN/-/LB_MODE;
		T4K28YUV_write_cmos_sensor(0x309F,0x00);//[RO] -/RO_LPBT_ERR_K/RO_LPBT_ERR_M/RO_LPBT_CNT[4:0];
		T4K28YUV_write_cmos_sensor(0x30A0,0x02);//;
		T4K28YUV_write_cmos_sensor(0x30A1,0x00);//FIFODLY[7:0];
		T4K28YUV_write_cmos_sensor(0x30A2,0xA7);//NUMWAKE[7:0];
		T4K28YUV_write_cmos_sensor(0x30A3,0x00);//TX_TRIGOPT/-/TRIG_Z5_X/-/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x30A4,0xFF);//TRIG_DUMMY[7:0];
		T4K28YUV_write_cmos_sensor(0x30A5,0x80);//CLKPRE2/-/-/-/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x30A6,0xFF);//NUMINIT[7:0];
		T4K28YUV_write_cmos_sensor(0x30A7,0x00);//EN_PHASE_SEL[1:0]/-/-/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x30A8,0x01);//MIPI_FS_CD[3:0]/MIPI_FE_CD[3:0];
		T4K28YUV_write_cmos_sensor(0x30F1,0x00);//-/-/-/T_OUTSEL[4:0];
		T4K28YUV_write_cmos_sensor(0x30F2,0x00);//-/-/-/-/-/-/-/T_VWIDTH_LAT_ON;
		T4K28YUV_write_cmos_sensor(0x30FE,0x80);//T_PROTECT/-/-/-/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3100,0xD2);//POSLFIX/NEGLFIX/NEGLEAKCUT/BOOSTEN_L/BSTREADEV/POSBSTSEL/READVDS;
		T4K28YUV_write_cmos_sensor(0x3101,0xD3);//RSTVDSEL_AZS/POSBSTCNT[2:0]/-/POSBSTHG[2:0];
		T4K28YUV_write_cmos_sensor(0x3102,0x95);//NEGBSTCNT[3:0]/-/POSBSTGA[2:0];
		T4K28YUV_write_cmos_sensor(0x3103,0x80);//TAUREADEN/-/-/-/-/-/-/LNOBMODE;
		T4K28YUV_write_cmos_sensor(0x3104,0x31);//GDMOSBGREN/-/VSIGDRSEL[1:0]/GDMOSCNT[3:0];
		T4K28YUV_write_cmos_sensor(0x3105,0x04);//-/-/-/-/KBIASCNT[3:0];
		T4K28YUV_write_cmos_sensor(0x3106,0x23);//-/-/DRADRVI[1:0]/-/DRADRVLV[2:0];
		T4K28YUV_write_cmos_sensor(0x3107,0x20);//TESTDACEN/-/VREFSWG[1:0]/-/-/-/VREFDLYCNT;
		T4K28YUV_write_cmos_sensor(0x3108,0x7B);//S1CNT[3:0]/CBIASIB/-/CBIASIA[1:0];
		T4K28YUV_write_cmos_sensor(0x3109,0x80);//BOOSTEN_R/-/-/IDLINGOFFEN/-/-/EDGETESTEN[1:0];
		T4K28YUV_write_cmos_sensor(0x310A,0x00);//-/-/-/-/-/SENSEMODE[2:0];
		T4K28YUV_write_cmos_sensor(0x310B,0x00);//SPARE[1:0]/-/-/ANAMON1_SEL[3:0];
		T4K28YUV_write_cmos_sensor(0x3110,0x11);//-/-/-/FBC_SUBSMPL/-/-/-/BIN_MODE;
		T4K28YUV_write_cmos_sensor(0x3111,0x11);//-/-/ES_MODE[1:0]/-/-/-/ESREAD_ALT_OFF;
		T4K28YUV_write_cmos_sensor(0x3112,0x00);//-/-/-/-/-/-/-/DIS_MODE;
		T4K28YUV_write_cmos_sensor(0x3113,0x00);//-/-/-/-/-/-/-/ALLZEROSET_ON;
		T4K28YUV_write_cmos_sensor(0x3114,0x10);//-/-/ALLZEROSET_1ST_ON[1:0]/-/-/-/ALLZEROSET_CHG_ON;
		T4K28YUV_write_cmos_sensor(0x3115,0x22);//-/-/LTCH_POS[1:0]/-/RODATA_U/DMR_ON/ALLREAD_ON;
		T4K28YUV_write_cmos_sensor(0x3120,0x08);//BSC_OFF/-/-/-/SADR_1PULSE/-/-/DRESET_1PULSE;
		T4K28YUV_write_cmos_sensor(0x3121,0x13);//-/BSCPULSE_INTVL[6:0];
		T4K28YUV_write_cmos_sensor(0x3122,0x33);//-/DRESET_1U[6:0];
		T4K28YUV_write_cmos_sensor(0x3123,0x0E);//-/-/-/DRESET_W[4:0];
		T4K28YUV_write_cmos_sensor(0x3124,0x26);//-/FTLSNS_1U[2:0]/FTLSNS_W[3:0];
		T4K28YUV_write_cmos_sensor(0x3125,0x00);//-/-/-/-/-/SADR_1U[2:0];
		T4K28YUV_write_cmos_sensor(0x3126,0x0C);//-/-/-/SADR_1W[4:0];
		T4K28YUV_write_cmos_sensor(0x3127,0x08);//-/-/SADR_2W[5:0];
		T4K28YUV_write_cmos_sensor(0x3128,0x80);//AUTO_READ_W/-/-/-/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3129,0x65);//ESREAD_1U[7:0];
		T4K28YUV_write_cmos_sensor(0x312A,0x27);//-/ESREAD_2U[6:0];
		T4K28YUV_write_cmos_sensor(0x312B,0x77);//-/ESREAD_1W[6:0];
		T4K28YUV_write_cmos_sensor(0x312C,0x77);//-/ESREAD_2W[6:0];
		T4K28YUV_write_cmos_sensor(0x312D,0x1A);//-/-/-/ESTGRESET_D[4:0];
		T4K28YUV_write_cmos_sensor(0x312E,0xB8);//VSIGPU_U[7:0];
		T4K28YUV_write_cmos_sensor(0x312F,0x38);//VSIGPU_W[7:0];
		T4K28YUV_write_cmos_sensor(0x3130,0x80);//VSIGPU_LOW/-/-/-/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3131,0x33);//-/ROREAD_U[6:0];
		T4K28YUV_write_cmos_sensor(0x3132,0x63);//-/ROREAD_W[6:0];
		T4K28YUV_write_cmos_sensor(0x3133,0x00);//-/-/-/-/-/-/-/ROTGRESET_U[8];
		T4K28YUV_write_cmos_sensor(0x3134,0xDD);//ROTGRESET_U[7:0];
		T4K28YUV_write_cmos_sensor(0x3135,0x07);//EXTD_ROTGRESET/-/-/-/ROTGRESET_W[3:0];
		T4K28YUV_write_cmos_sensor(0x3136,0xB7);//ZEROSET_U[7:0];
		T4K28YUV_write_cmos_sensor(0x3137,0x11);//-/-/ZEROSET_W[5:0];
		T4K28YUV_write_cmos_sensor(0x3138,0x0B);//RSTDRAIN_HIGH/-/RSTDRAIN_D[5:0];
		T4K28YUV_write_cmos_sensor(0x313B,0x0A);//-/-/-/RSTDRAIN_U[4:0];
		T4K28YUV_write_cmos_sensor(0x313C,0x05);//-/-/-/RSTDRAIN3_U[4:0];
		T4K28YUV_write_cmos_sensor(0x313D,0x01);//DRCUT_SIGIN/DRCUT_DMY_OFF/-/DRCUT_U[4:0];
		T4K28YUV_write_cmos_sensor(0x313E,0x62);//DRCUT_NW[7:0];
		T4K28YUV_write_cmos_sensor(0x313F,0x85);//DRCUT_VDER_W[7:0];
		T4K28YUV_write_cmos_sensor(0x3140,0x01);//BGRSH_OFF/-/BGRSH_U[1:0]/-/-/BGRSH_W[1:0];
		T4K28YUV_write_cmos_sensor(0x3141,0x40);//VSIGDR_MODE[1:0]/-/VSIGDR_U[1:0]/-/VSIGDR_D[1:0];
		T4K28YUV_write_cmos_sensor(0x3142,0x80);//S1_2PULSES/-/S1_ALL_HIGH/S1_STOPBST/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3143,0x22);//-/S1_1U[6:0];
		T4K28YUV_write_cmos_sensor(0x3144,0x3E);//S1_1W[7:0];
		T4K28YUV_write_cmos_sensor(0x3145,0x32);//-/S1_2U[6:0];
		T4K28YUV_write_cmos_sensor(0x3146,0x2E);//-/S1_2W[6:0];
		T4K28YUV_write_cmos_sensor(0x3147,0x23);//-/-/S1_AW[5:0];
		T4K28YUV_write_cmos_sensor(0x3148,0x22);//-/S1A_USHIFT[2:0]/-/S1A_DSHIFT[2:0];
		T4K28YUV_write_cmos_sensor(0x3149,0x11);//-/S1W_USHIFT[2:0]/-/S1W_DSHIFT[2:0];
		T4K28YUV_write_cmos_sensor(0x314A,0x6B);//S2_W[7:0];
		T4K28YUV_write_cmos_sensor(0x314B,0x30);//-/S3_W[6:0];
		T4K28YUV_write_cmos_sensor(0x314C,0x69);//S4_W[7:0];
		T4K28YUV_write_cmos_sensor(0x314D,0x80);//CDS_STOPBST/-/S4_AD[5:0];
		T4K28YUV_write_cmos_sensor(0x314E,0x31);//BSTCKLFIX_HIGH/BSTCKLFIX_1U[2:0]/-/BSTCKLFIX_1D[2:0];
		T4K28YUV_write_cmos_sensor(0x314F,0x32);//-/BSTCKLFIX_2U[2:0]/-/BSTCKLFIX_2D[2:0];
		T4K28YUV_write_cmos_sensor(0x3150,0x32);//BSTCKLFIX_1P/BSTCKLFIX_3U[2:0]/-/BSTCKLFIX_3D[2:0];
		T4K28YUV_write_cmos_sensor(0x3151,0x03);//-/-/-/INTEN_CU[4:0];
		T4K28YUV_write_cmos_sensor(0x3152,0x0C);//-/-/INTEN_CW[5:0];
		T4K28YUV_write_cmos_sensor(0x3153,0xB3);//INTEN_SU[7:0];
		T4K28YUV_write_cmos_sensor(0x3154,0x20);//-/-/INTEN_AU[5:0];
		T4K28YUV_write_cmos_sensor(0x3155,0x13);//-/-/BSTCKLFIX_1P_U[1:0]/INTEN_AD[3:0];
		T4K28YUV_write_cmos_sensor(0x3156,0x66);//INTRS_CU[7:0];
		T4K28YUV_write_cmos_sensor(0x3157,0x02);//-/-/-/INTRS_CD[4:0];
		T4K28YUV_write_cmos_sensor(0x3158,0x03);//-/-/-/-/INTRS_SU[3:0];
		T4K28YUV_write_cmos_sensor(0x3159,0x01);//-/-/-/-/-/-/INTRS_SD[1:0];
		T4K28YUV_write_cmos_sensor(0x315A,0x16);//-/-/INTRS_AU[5:0];
		T4K28YUV_write_cmos_sensor(0x315B,0x10);//-/-/-/INTRS_AW[4:0];
		T4K28YUV_write_cmos_sensor(0x315C,0x00);//-/DRKCLIP_U[2:0]/CLR_GCNTR_D[3:0];
		T4K28YUV_write_cmos_sensor(0x315D,0x44);//DRKCLIP_PRE_U[3:0]/-/DRKCLIP_PRE_D[2:0];
		T4K28YUV_write_cmos_sensor(0x315E,0x1B);//-/HPL2_NU[6:0];
		T4K28YUV_write_cmos_sensor(0x315F,0x52);//-/HPL2_AU[6:0];
		T4K28YUV_write_cmos_sensor(0x3160,0x00);//EXT_AG_ON/-/-/-/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3161,0x03);//EXT_AG_PARAM[15:8];
		T4K28YUV_write_cmos_sensor(0x3162,0x00);//EXT_AG_PARAM[7:0];
		T4K28YUV_write_cmos_sensor(0x3163,0xFF);//TDAC_INT[7:0];
		T4K28YUV_write_cmos_sensor(0x3164,0x00);//TDAC_MIN[7:0];
		T4K28YUV_write_cmos_sensor(0x3165,0x01);//TDAC_SWD[1:0]/-/-/TDAC_STEP[3:0];
		T4K28YUV_write_cmos_sensor(0x3166,0x00);//DACS_INT[7:0];
		T4K28YUV_write_cmos_sensor(0x3167,0xFF);//DACS_MAX[7:0];
		T4K28YUV_write_cmos_sensor(0x3168,0x01);//DACS_SWD[1:0]/-/-/DACS_STEP[3:0];
		T4K28YUV_write_cmos_sensor(0x3169,0x00);//AG_TEST/-/LATCH_TEST[1:0]/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3180,0x00);//SIGIN_ON/VOB_DISP/HOB_DISP/-/-/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3181,0x20);//DATA_LATCH_SEL/-/EN_CBLK/OADJ_MODE[1:0]/-/OADJ_INTEN_W[1:0];
		T4K28YUV_write_cmos_sensor(0x3182,0x40);//-/FBC_MODE[2:0]/-/FBC_PERIOD_SEL[2:0];
		T4K28YUV_write_cmos_sensor(0x3183,0x96);//CP1_FBC_SEL/-/CP1_OADJ_U[5:0];
		T4K28YUV_write_cmos_sensor(0x3184,0x30);//-/FBC_LEVEL[6:0];
		T4K28YUV_write_cmos_sensor(0x3185,0x8F);//FBC_ON/-/FAST_FBC_ON/-/SLOW_FBC_RANGE[3:0];
		T4K28YUV_write_cmos_sensor(0x3186,0x31);//-/FBC_START_CND[2:0]/-/FBC_SUSP_CND[2:0];
		T4K28YUV_write_cmos_sensor(0x3187,0x06);//-/-/-/FBC_STOP_CND[4:0];
		T4K28YUV_write_cmos_sensor(0x3188,0x0C);//FBC_OUT_LINES[7:0];
		T4K28YUV_write_cmos_sensor(0x3189,0x44);//FBC_HIGH_OBAVE[3:0]/FBC_LOW_OBAVE[3:0];
		T4K28YUV_write_cmos_sensor(0x318A,0x42);//FBC_IN_LINES[3:0]/FBC_IN_RANGE[3:0];
		T4K28YUV_write_cmos_sensor(0x318B,0x0B);//FBC_STOP_RANGE/FBC_START_RANGE/-/-/FBC_AG_COEF_DIV[1:0]/FBC_OUT_;
		T4K28YUV_write_cmos_sensor(0x318C,0x11);//FBC_AG_CONT_COEF[7:0];
		T4K28YUV_write_cmos_sensor(0x318D,0xAA);//EN_ES_CHANGE/-/EN_AG_CHANGE/-/EN_OADJ_CHANGE/-/PSCLAMP_1ST_ON/-;
		T4K28YUV_write_cmos_sensor(0x318E,0x40);//-/CLAMPSHIFT_SEL[2:0]/AG_CHANGE_VAL[3:0];
		T4K28YUV_write_cmos_sensor(0x318F,0x30);//CLAMPSHIFT_VAL[7:0];
		T4K28YUV_write_cmos_sensor(0x3190,0x03);//-/-/-/IDLINGOFF_U[4:0];
		T4K28YUV_write_cmos_sensor(0x3191,0x01);//-/-/-/-/-/-/IDLINGOFF_D[1:0];
		T4K28YUV_write_cmos_sensor(0x3192,0x00);//-/-/-/-/EXT_DACPARAM[11:8];
		T4K28YUV_write_cmos_sensor(0x3193,0x00);//EXT_DACPARAM[7:0];
		T4K28YUV_write_cmos_sensor(0x3194,0x00);//EXT_CLAMP_ON/EXT_OFFSET_ON/-/-/RET_INT_OFFSET/BIT_VERIF_TIME[2:0;
		T4K28YUV_write_cmos_sensor(0x3195,0x00);//-/-/-/-/-/-/BLADJ_MODE[1:0];
		T4K28YUV_write_cmos_sensor(0x3196,0x00);//-/-/-/-/-/BLADJ_COEF[10:8];
		T4K28YUV_write_cmos_sensor(0x3197,0xDE);//BLADJ_COEF[7:0];
		T4K28YUV_write_cmos_sensor(0x3198,0x00);//-/-/-/-/BLADJ_INTC[11:8];
		T4K28YUV_write_cmos_sensor(0x3199,0x00);//BLADJ_INTC[7:0];
		T4K28YUV_write_cmos_sensor(0x319A,0x00);//OB_AVE_MON_SEL[1:0]/-/ST_CKI[4:0];
		T4K28YUV_write_cmos_sensor(0x319B,0x00);//ST_RSVD_REG[7:0];
		T4K28YUV_write_cmos_sensor(0x319C,0x16);//-/-/IDLINGOFF_AU[5:0];
		T4K28YUV_write_cmos_sensor(0x319D,0x0A);//-/-/-/IDLINGOFF_AW[4:0];
		T4K28YUV_write_cmos_sensor(0x31A0,0xBF);//EXT_HCNT_MAX_ON/-/HCNT_MAX_FIXVAL[13:8];
		T4K28YUV_write_cmos_sensor(0x31A1,0xFF);//HCNT_MAX_FIXVAL[7:0];
		T4K28YUV_write_cmos_sensor(0x31A2,0x11);//HCNT_MAX_MODE/-/-/FBC_MASK_FRAM_EN/-/MASK_FRAME_NUM[2:0];
		T4K28YUV_write_cmos_sensor(0x31B0,0x00);//[RO] fbc_operation/-/-/-/-/-/ob_ave[9:8];
		T4K28YUV_write_cmos_sensor(0x31B1,0x41);//[RO] ob_ave[7:0];
		T4K28YUV_write_cmos_sensor(0x31B2,0x1A);//[RO] -/-/clamp_param[13:8];
		T4K28YUV_write_cmos_sensor(0x31B3,0x61);//[RO] clamp_param[7:0];
		T4K28YUV_write_cmos_sensor(0x31B4,0x03);//[RO] -/-/-/-/offset_param[11:8];
		T4K28YUV_write_cmos_sensor(0x31B5,0x2D);//[RO] offset_param[7:0];
		T4K28YUV_write_cmos_sensor(0x31B6,0x1A);//[RO] -/-/ps_clamp_param[13:8];
		T4K28YUV_write_cmos_sensor(0x31B7,0x83);//[RO] ps_clamp_param[7:0];
		T4K28YUV_write_cmos_sensor(0x31B8,0x00);//[RO] fbcline_cnt[15:8];
		T4K28YUV_write_cmos_sensor(0x31B9,0x03);//[RO] fbcline_cnt[7:0];
		T4K28YUV_write_cmos_sensor(0x31BA,0x3F);//[RO] -/-/hcnt_max[13:8];
		T4K28YUV_write_cmos_sensor(0x31BB,0xFF);//[RO] hcnt_max[7:0];
		T4K28YUV_write_cmos_sensor(0x3300,0xFF);//HLNRSW/DANSASW[1:0]/LDNRSW/VDNRSW/ABPCSW/WBPSW/BBPSW;
		T4K28YUV_write_cmos_sensor(0x3301,0x35);//-/-/ANRSW/PEDAJSW/-/LSSCSW/LSSCAG_SW/PWBSW;
		T4K28YUV_write_cmos_sensor(0x3303,0x40);//BLKADJ[7:0];
		T4K28YUV_write_cmos_sensor(0x3304,0x00);//BLKR[3:0]/BLKGR[3:0];
		T4K28YUV_write_cmos_sensor(0x3305,0x00);//BLKGB[3:0]/BLKB[3:0];
		T4K28YUV_write_cmos_sensor(0x3306,0x30);//-/-/OBWIDTH[5:0];
		T4K28YUV_write_cmos_sensor(0x3307,0x00);//-/-/-/-/-/OBCLIP/BLLVSEL/DANSA2L;
		T4K28YUV_write_cmos_sensor(0x3308,0x87);//LDNRLIM1[3:0]/LDNRLIM0[3:0];
		T4K28YUV_write_cmos_sensor(0x330A,0x60);//LDNRGA1[3:0]/LDNRGA0[3:0];
		T4K28YUV_write_cmos_sensor(0x330B,0x56);//VDNRLIM1[3:0]/VDNRLIM0[3:0];
		T4K28YUV_write_cmos_sensor(0x330D,0x79);//VDNRGA1[3:0]/VDNRGA0[3:0];
		T4K28YUV_write_cmos_sensor(0x330E,0xFF);//BBPLV0[7:0];
		T4K28YUV_write_cmos_sensor(0x330F,0xFF);//BBPLV1[7:0];
		T4K28YUV_write_cmos_sensor(0x3310,0xFF);//WBPLV0[7:0];
		T4K28YUV_write_cmos_sensor(0x3311,0x7F);//WBPLV1[7:0];
		T4K28YUV_write_cmos_sensor(0x3312,0x0F);//WBPOFS[7:0];
		T4K28YUV_write_cmos_sensor(0x3313,0x0F);//BBPOFS[7:0];
		T4K28YUV_write_cmos_sensor(0x3314,0x02);//-/-/-/-/CONTLV1[3:0];
		T4K28YUV_write_cmos_sensor(0x3315,0xC0);//CONTLV2[7:0];
		T4K28YUV_write_cmos_sensor(0x3316,0x00);//ANREG_LEV0[7:0];
		T4K28YUV_write_cmos_sensor(0x3317,0x00);//ANREG_LEV1[7:0];
		T4K28YUV_write_cmos_sensor(0x3318,0xFF);//ANRG0[7:0];
		T4K28YUV_write_cmos_sensor(0x3319,0xFF);//ANRG1[7:0];
		T4K28YUV_write_cmos_sensor(0x331B,0x00);//GRPED0[3:0]/GBPED0[3:0];
		T4K28YUV_write_cmos_sensor(0x331C,0x00);//RPED0[3:0]/BPED0[3:0];
		T4K28YUV_write_cmos_sensor(0x331D,0x00);//GRPED1[3:0]/GBPED1[3:0];
		T4K28YUV_write_cmos_sensor(0x331E,0x00);//RPED1[3:0]/BPED1[3:0];
		T4K28YUV_write_cmos_sensor(0x3322,0x2B);//PWBGAINGR[7:0];
		T4K28YUV_write_cmos_sensor(0x3323,0x2B);//PWBGAINGB[7:0];
		T4K28YUV_write_cmos_sensor(0x3324,0x00);//PWBGAINR[7:0];
		T4K28YUV_write_cmos_sensor(0x3325,0x6A);//PWBGAINB[7:0];
		T4K28YUV_write_cmos_sensor(0x3327,0x00);//-/-/-/-/-/-/LIPOL/CSPOL;
		T4K28YUV_write_cmos_sensor(0x3328,0x00);//-/-/-/-/-/LS1SIG[2:0];
		T4K28YUV_write_cmos_sensor(0x3329,0x90);//LSHOFG[7:0];
		T4K28YUV_write_cmos_sensor(0x332A,0x9F);//LSHOFR[7:0];
		T4K28YUV_write_cmos_sensor(0x332B,0xA4);//LSHOFB[7:0];
		T4K28YUV_write_cmos_sensor(0x332C,0x70);//LSVOFG[7:0];
		T4K28YUV_write_cmos_sensor(0x332D,0x70);//LSVOFR[7:0];
		T4K28YUV_write_cmos_sensor(0x332E,0x70);//LSVOFB[7:0];
		T4K28YUV_write_cmos_sensor(0x332F,0x00);//LSALUG[7:0];
		T4K28YUV_write_cmos_sensor(0x3330,0x00);//LSALUR[7:0];
		T4K28YUV_write_cmos_sensor(0x3331,0x08);//LSALUB[7:0];
		T4K28YUV_write_cmos_sensor(0x3332,0x00);//LSARUG[7:0];
		T4K28YUV_write_cmos_sensor(0x3333,0x00);//LSARUR[7:0];
		T4K28YUV_write_cmos_sensor(0x3334,0x00);//LSARUB[7:0];
		T4K28YUV_write_cmos_sensor(0x3335,0x00);//LSALDG[7:0];
		T4K28YUV_write_cmos_sensor(0x3336,0x00);//LSALDR[7:0];
		T4K28YUV_write_cmos_sensor(0x3337,0x08);//LSALDB[7:0];
		T4K28YUV_write_cmos_sensor(0x3338,0x00);//LSARDG[7:0];
		T4K28YUV_write_cmos_sensor(0x3339,0x00);//LSARDR[7:0];
		T4K28YUV_write_cmos_sensor(0x333A,0x00);//LSARDB[7:0];
		T4K28YUV_write_cmos_sensor(0x333B,0x2C);//LSBLG[7:0];
		T4K28YUV_write_cmos_sensor(0x333C,0x58);//LSBLR[7:0];
		T4K28YUV_write_cmos_sensor(0x333D,0x18);//LSBLB[7:0];
		T4K28YUV_write_cmos_sensor(0x333E,0x30);//LSBRG[7:0];
		T4K28YUV_write_cmos_sensor(0x333F,0x50);//LSBRR[7:0];
		T4K28YUV_write_cmos_sensor(0x3340,0x20);//LSBRB[7:0];
		T4K28YUV_write_cmos_sensor(0x3341,0x30);//LSCUG[7:0];
		T4K28YUV_write_cmos_sensor(0x3342,0x48);//LSCUR[7:0];
		T4K28YUV_write_cmos_sensor(0x3343,0x20);//LSCUB[7:0];
		T4K28YUV_write_cmos_sensor(0x3344,0x30);//LSCDG[7:0];
		T4K28YUV_write_cmos_sensor(0x3345,0x48);//LSCDR[7:0];
		T4K28YUV_write_cmos_sensor(0x3346,0x20);//LSCDB[7:0];
		T4K28YUV_write_cmos_sensor(0x3347,0x00);//LSBRG_L[3:0]/LSBLG_L[3:0];
		T4K28YUV_write_cmos_sensor(0x3348,0x00);//LSBR R_L[3:0]/LSBL R_L[3:0];
		T4K28YUV_write_cmos_sensor(0x3349,0x00);//LSBRB_L[3:0]/LSBLB_L[3:0];
		T4K28YUV_write_cmos_sensor(0x334A,0x00);//LSCDG_L[3:0]/LSCUG_L[3:0];
		T4K28YUV_write_cmos_sensor(0x334B,0x00);//LSCDR_L[3:0]/LSCUR_L[3:0];
		T4K28YUV_write_cmos_sensor(0x334C,0x00);//LSCDB_L[3:0]/LSCUB_L[3:0];
		T4K28YUV_write_cmos_sensor(0x334D,0x40);//LSHCNT_MPY[7:0];
		T4K28YUV_write_cmos_sensor(0x334E,0x00);//-/-/-/-/LSVCNT_MPY[11:8];
		T4K28YUV_write_cmos_sensor(0x334F,0xA0);//LSVCNT_MPY[7:0];
		T4K28YUV_write_cmos_sensor(0x3350,0x03);//-/-/-/-/-/-/LSMGSEL[1:0];
		T4K28YUV_write_cmos_sensor(0x335F,0x00);//-/-/-/-/-/-/-/TESTPAT_SW;
		T4K28YUV_write_cmos_sensor(0x3360,0x00);//PP_SPARE[7:0];
		T4K28YUV_write_cmos_sensor(0x3400,0xA4);//LMCC_SW/-/PGC_SW/-/-/YSHP_SW/-/-;
		T4K28YUV_write_cmos_sensor(0x3401,0x7F);//-/BRIGHT_SW/LCNT_SW/NLCNT_SW/UVLPF_SW/UVSHP_SW/FCS_SW/UVCRING_SW;
		T4K28YUV_write_cmos_sensor(0x3402,0x00);//LCS_LI/LCS_CS/-/-/-/EFECT_SEL[2:0];
		T4K28YUV_write_cmos_sensor(0x3403,0x00);//LCS_CRING[7:0];
		T4K28YUV_write_cmos_sensor(0x3404,0x3A);//LM_RMG[7:0];
		T4K28YUV_write_cmos_sensor(0x3405,0xE3);//LM_RMB[7:0];
		T4K28YUV_write_cmos_sensor(0x3406,0x22);//LM_GMR[7:0];
		T4K28YUV_write_cmos_sensor(0x3407,0x25);//LM_GMB[7:0];
		T4K28YUV_write_cmos_sensor(0x3408,0x17);//LM_BMR[7:0];
		T4K28YUV_write_cmos_sensor(0x3409,0x5C);//LM_BMG[7:0];
		T4K28YUV_write_cmos_sensor(0x340A,0x20);//-/GAM01P[6:0];
		T4K28YUV_write_cmos_sensor(0x340B,0x20);//-/GAM02P[6:0];
		T4K28YUV_write_cmos_sensor(0x340C,0x3B);//-/GAM03P[6:0];
		T4K28YUV_write_cmos_sensor(0x340D,0x2E);//-/GAM04P[6:0];
		T4K28YUV_write_cmos_sensor(0x340E,0x26);//-/GAM05P[6:0];
		T4K28YUV_write_cmos_sensor(0x340F,0x3F);//-/GAM06P[6:0];
		T4K28YUV_write_cmos_sensor(0x3410,0x34);//-/GAM07P[6:0];
		T4K28YUV_write_cmos_sensor(0x3411,0x2D);//-/GAM08P[6:0];
		T4K28YUV_write_cmos_sensor(0x3412,0x28);//-/GAM09P[6:0];
		T4K28YUV_write_cmos_sensor(0x3413,0x47);//-/GAM10P[6:0];
		T4K28YUV_write_cmos_sensor(0x3414,0x3E);//-/GAM11P[6:0];
		T4K28YUV_write_cmos_sensor(0x3415,0x6A);//-/GAM12P[6:0];
		T4K28YUV_write_cmos_sensor(0x3416,0x5A);//-/GAM13P[6:0];
		T4K28YUV_write_cmos_sensor(0x3417,0x50);//-/GAM14P[6:0];
		T4K28YUV_write_cmos_sensor(0x3418,0x48);//-/GAM15P[6:0];
		T4K28YUV_write_cmos_sensor(0x3419,0x42);//-/GAM16P[6:0];
		T4K28YUV_write_cmos_sensor(0x341B,0x10);//-/-/BLK_ADJ[5:0];
		T4K28YUV_write_cmos_sensor(0x341C,0x40);//CbG_MAT[7:0];
		T4K28YUV_write_cmos_sensor(0x341D,0x70);//CbB_MAT[7:0];
		T4K28YUV_write_cmos_sensor(0x341E,0xC4);//Cb_GAIN[7:0];
		T4K28YUV_write_cmos_sensor(0x341F,0x88);//CrR_MAT[7:0];
		T4K28YUV_write_cmos_sensor(0x3420,0x80);//CrG_MAT[7:0];
		T4K28YUV_write_cmos_sensor(0x3421,0xC4);//Cr_GAIN[7:0];
		T4K28YUV_write_cmos_sensor(0x3422,0x00);//Cbr_MGAIN[7:0];
		T4K28YUV_write_cmos_sensor(0x3423,0x0F);//-/-/-/-/CbP_GAIN1[3:0];
		T4K28YUV_write_cmos_sensor(0x3424,0x0F);//-/-/-/-/CbM_GAIN1[3:0];
		T4K28YUV_write_cmos_sensor(0x3425,0x0F);//-/-/-/-/CrP_GAIN1[3:0];
		T4K28YUV_write_cmos_sensor(0x3426,0x0F);//-/-/-/-/CrM_GAIN1[3:0];
		T4K28YUV_write_cmos_sensor(0x342B,0x24);//-/-/DTL_CRNG0[5:0];
		T4K28YUV_write_cmos_sensor(0x342C,0x3F);//-/-/DTL_CRNG1[5:0];
		T4K28YUV_write_cmos_sensor(0x342D,0x90);//DTL_PG0[7:0];
		T4K28YUV_write_cmos_sensor(0x342E,0x80);//DTL_PG1[7:0];
		T4K28YUV_write_cmos_sensor(0x342F,0xC0);//DTL_MG0[7:0];
		T4K28YUV_write_cmos_sensor(0x3430,0xB0);//DTL_MG1[7:0];
		T4K28YUV_write_cmos_sensor(0x3431,0x1E);//DTL_LIM_PH[7:0];
		T4K28YUV_write_cmos_sensor(0x3432,0x1E);//DTL_LIM_MH[7:0];
		T4K28YUV_write_cmos_sensor(0x3433,0x0A);//DTL_LIM_PLS[7:0];
		T4K28YUV_write_cmos_sensor(0x3434,0x0A);//DTL_LIM_MLS[7:0];
		T4K28YUV_write_cmos_sensor(0x3435,0x15);//DTL_LIM_PLE[7:0];
		T4K28YUV_write_cmos_sensor(0x3436,0x15);//DTL_LIM_MLE[7:0];
		T4K28YUV_write_cmos_sensor(0x343F,0x10);//BRIGHT1[7:0];
		T4K28YUV_write_cmos_sensor(0x3440,0xF0);//BRIGHT0[7:0];
		T4K28YUV_write_cmos_sensor(0x3441,0x85);//LCONT_LEV[7:0];
		T4K28YUV_write_cmos_sensor(0x3442,0x60);//BLK_KNEE[7:0];
		T4K28YUV_write_cmos_sensor(0x3443,0x70);//WHT_KNEE[7:0];
		T4K28YUV_write_cmos_sensor(0x3444,0x02);//-/-/-/-/BLK_CONT[3:0];
		T4K28YUV_write_cmos_sensor(0x3446,0x02);//-/-/-/-/WHT_CONT[3:0];
		T4K28YUV_write_cmos_sensor(0x3447,0x00);//NLOFF_AG[7:0];
		T4K28YUV_write_cmos_sensor(0x3448,0x00);//-/-/COL_LV[5:0];
		T4K28YUV_write_cmos_sensor(0x3449,0x00);//-/-/-/-/COL_ED_G[3:0];
		T4K28YUV_write_cmos_sensor(0x344A,0x00);//UV_CRING1[3:0]/UV_CRING0[3:0];
		T4K28YUV_write_cmos_sensor(0x344B,0x00);//UV_SHP_AG[7:0];
		T4K28YUV_write_cmos_sensor(0x344C,0x20);//FCS_EDG_CORING[7:0];
		T4K28YUV_write_cmos_sensor(0x344D,0xFF);//FCS_EDG_LIM[7:0];
		T4K28YUV_write_cmos_sensor(0x344E,0x0F);//-/-/-/-/FCS_EDG_GAIN[3:0];
		T4K28YUV_write_cmos_sensor(0x344F,0x20);//FCS_LV[7:0];
		T4K28YUV_write_cmos_sensor(0x3450,0x80);//FCS_GAIN[7:0];
		T4K28YUV_write_cmos_sensor(0x3451,0x0F);//-/-/-/-/FCS_AG[3:0];
		T4K28YUV_write_cmos_sensor(0x3452,0x55);//UNICOFSU[7:0];
		T4K28YUV_write_cmos_sensor(0x3453,0x49);//UNICOFSV[7:0];
		T4K28YUV_write_cmos_sensor(0x3454,0x6A);//SEPIAOFSU[7:0];
		T4K28YUV_write_cmos_sensor(0x3455,0x93);//SEPIAOFSV[7:0];
		T4K28YUV_write_cmos_sensor(0x345C,0x00);//MP_VLAT_OFF/-/-/-/-/-/TEST_IN_SW[1:0];
		T4K28YUV_write_cmos_sensor(0x345D,0x00);//TEST_AG[7:0];
		T4K28YUV_write_cmos_sensor(0x345E,0x00);//MP_SPARE[7:0];
		T4K28YUV_write_cmos_sensor(0x3500,0xC1);//ALCSW/AWBSW/ALCLOCK/-/ESLIMMODE/ROOMDET/-/ALCLIMMODE;
		T4K28YUV_write_cmos_sensor(0x3501,0x01);//-/-/-/-/-/-/ALCAIM[9:8];
		T4K28YUV_write_cmos_sensor(0x3502,0x30);//ALCAIM[7:0];
		T4K28YUV_write_cmos_sensor(0x3503,0x1A);//AGMIN[7:0];
		T4K28YUV_write_cmos_sensor(0x3504,0x00);//-/-/-/-/AGMAX[11:8];
		T4K28YUV_write_cmos_sensor(0x3505,0x9C);//AGMAX[7:0];
		T4K28YUV_write_cmos_sensor(0x3506,0x04);//MES[15:8];
		T4K28YUV_write_cmos_sensor(0x3507,0xD0);//MES[7:0];
		T4K28YUV_write_cmos_sensor(0x3508,0x00);//MMES[15:8];
		T4K28YUV_write_cmos_sensor(0x3509,0x6B);//MMES[7:0];
		T4K28YUV_write_cmos_sensor(0x350A,0x00);//-/-/-/-/MAG[11:8];
		T4K28YUV_write_cmos_sensor(0x350B,0x20);//MAG[7:0];
		T4K28YUV_write_cmos_sensor(0x350C,0x00);//MDG[7:0];
		T4K28YUV_write_cmos_sensor(0x350D,0x15);//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
		T4K28YUV_write_cmos_sensor(0x350E,0x15);//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
		T4K28YUV_write_cmos_sensor(0x350F,0x51);//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
		T4K28YUV_write_cmos_sensor(0x3510,0x50);//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
		T4K28YUV_write_cmos_sensor(0x3511,0x9f);//UDMODE[1:0]/-/UPDNSPD[4:0];
		T4K28YUV_write_cmos_sensor(0x3512,0x1f);//ALCOFS[2:0]/NEARSPD[4:0];
		T4K28YUV_write_cmos_sensor(0x3513,0x00);//ALC_ANJ_OFS[7:0];
		T4K28YUV_write_cmos_sensor(0x3514,0x00);//-/-/-/-/-/-/ALC_ANJ_SPD[1:0];
		T4K28YUV_write_cmos_sensor(0x3515,0x10);//ALCFRZLV1[7:0];
		T4K28YUV_write_cmos_sensor(0x3516,0x10);//ALCFRZLV2[7:0];
		T4K28YUV_write_cmos_sensor(0x3517,0x00);//ALCFRZTIM[7:0];
		T4K28YUV_write_cmos_sensor(0x3518,0x00);//ALCSIGMIN[7:0];
		T4K28YUV_write_cmos_sensor(0x3519,0xFF);//ALCSIGMAX[7:0];
		T4K28YUV_write_cmos_sensor(0x351A,0xC0);//SATSET[7:0];
		T4K28YUV_write_cmos_sensor(0x351B,0x98);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
		T4K28YUV_write_cmos_sensor(0x351C,0x68);//FLLONGON/FRMSPD[1:0]/FL600S[12:8];
		T4K28YUV_write_cmos_sensor(0x351D,0xF5);//FL600S[7:0];
		T4K28YUV_write_cmos_sensor(0x351E,0x16);//ACFDET/AC60M/FLMANU/ACDETDLY/MSKLINE[1:0]/ACDPWAIT[1:0];
		T4K28YUV_write_cmos_sensor(0x351F,0x80);//FL600AT/TTL1V_ON/-/-/-/-/-/FLDETM[8];
		T4K28YUV_write_cmos_sensor(0x3520,0x26);//FLDETM[7:0];
		T4K28YUV_write_cmos_sensor(0x3521,0x02);//ACDET1LV[7:0];
		T4K28YUV_write_cmos_sensor(0x3522,0x08);//ACDET2LV[7:0];
		T4K28YUV_write_cmos_sensor(0x3523,0x0C);//SATGUP/SATFRZ/SAT1VDET/-/DETSEL[3:0];
		T4K28YUV_write_cmos_sensor(0x3524,0x01);//AWBONDOT[2:0]/-/-/-/WBMRG[9:8];
		T4K28YUV_write_cmos_sensor(0x3525,0x5A);//WBMRG[7:0];
		T4K28YUV_write_cmos_sensor(0x3526,0x2C);//CAREASEL[1:0]/AREAMODE[1:0]/HEXSW/YGATESW/WBMGG[9:8];
		T4K28YUV_write_cmos_sensor(0x3527,0xE0);//WBMGG[7:0];
		T4K28YUV_write_cmos_sensor(0x3528,0xFD);//SQ1SW/SQ1POL/SQ2SW/SQ2POL/SQ3SW/SQ3POL/WBMBG[9:8];
		T4K28YUV_write_cmos_sensor(0x3529,0x72);//WBMBG[7:0];
		T4K28YUV_write_cmos_sensor(0x352A,0xDE);//WBGRMAX[7:0];
		T4K28YUV_write_cmos_sensor(0x352B,0x22);//WBGRMIN[7:0];
		T4K28YUV_write_cmos_sensor(0x352C,0xD0);//WBGBMAX[7:0];
		T4K28YUV_write_cmos_sensor(0x352D,0x10);//WBGBMIN[7:0];
		T4K28YUV_write_cmos_sensor(0x352E,0x28);//RBCUT0H[7:0];
		T4K28YUV_write_cmos_sensor(0x352F,0xDC);//RBCUT0L[7:0];
		T4K28YUV_write_cmos_sensor(0x3530,0x2F);//-/RYCUT0P[6:0];
		T4K28YUV_write_cmos_sensor(0x3531,0x40);//-/RYCUT0N[6:0];
		T4K28YUV_write_cmos_sensor(0x3532,0x3A);//-/BYCUT0P[6:0];
		T4K28YUV_write_cmos_sensor(0x3533,0x2E);//-/BYCUT0N[6:0];
		T4K28YUV_write_cmos_sensor(0x3534,0x28);//RBCUT0HL[7:0];
		T4K28YUV_write_cmos_sensor(0x3535,0xDC);//RBCUT0LL[7:0];
		T4K28YUV_write_cmos_sensor(0x3536,0x2F);//-/RYCUT0PL[6:0];
		T4K28YUV_write_cmos_sensor(0x3537,0x40);//-/RYCUT0NL[6:0];
		T4K28YUV_write_cmos_sensor(0x3538,0x3A);//-/BYCUT0PL[6:0];
		T4K28YUV_write_cmos_sensor(0x3539,0x2E);//-/BYCUT0NL[6:0];
		T4K28YUV_write_cmos_sensor(0x353A,0x3C);//RYCUT1H[7:0];
		T4K28YUV_write_cmos_sensor(0x353B,0x35);//-/RYCUT1L[6:0];
		T4K28YUV_write_cmos_sensor(0x353C,0xDE);//BYCUT1H[7:0];
		T4K28YUV_write_cmos_sensor(0x353D,0x33);//-/BYCUT1L[6:0];
		T4K28YUV_write_cmos_sensor(0x353E,0xDC);//RYCUT2H[7:0];
		T4K28YUV_write_cmos_sensor(0x353F,0x38);//-/RYCUT2L[6:0];
		T4K28YUV_write_cmos_sensor(0x3540,0x55);//BYCUT2H[7:0];
		T4K28YUV_write_cmos_sensor(0x3541,0x38);//-/BYCUT2L[6:0];
		T4K28YUV_write_cmos_sensor(0x3542,0x60);//RYCUT3H[7:0];
		T4K28YUV_write_cmos_sensor(0x3543,0x3F);//-/RYCUT3L[6:0];
		T4K28YUV_write_cmos_sensor(0x3544,0xBD);//BYCUT3H[7:0];
		T4K28YUV_write_cmos_sensor(0x3545,0x46);//-/BYCUT3L[6:0];
		T4K28YUV_write_cmos_sensor(0x3546,0xF0);//YGATEH[7:0];
		T4K28YUV_write_cmos_sensor(0x3547,0x10);//YGATEL[7:0];
		T4K28YUV_write_cmos_sensor(0x3548,0x84);//CGRANGE[1:0]/-/AWBHUECOR/AWBSPD[3:0];
		T4K28YUV_write_cmos_sensor(0x3549,0x00);//YGATE_SEL/YGATE_DATA[1:0]/AWBULV[4:0];
		T4K28YUV_write_cmos_sensor(0x354A,0x00);//AWBFZTIM[2:0]/AWBVLV[4:0];
		T4K28YUV_write_cmos_sensor(0x354B,0x00);//AWBSFTU[7:0];
		T4K28YUV_write_cmos_sensor(0x354C,0x00);//AWBSFTV[7:0];
		T4K28YUV_write_cmos_sensor(0x354D,0x00);//AWBWAIT[7:0];
		T4K28YUV_write_cmos_sensor(0x354E,0x00);//-/-/-/-/CGCNGSLP[2:0]/CGCNGLV[8];
		T4K28YUV_write_cmos_sensor(0x354F,0x00);//CGCNGLV[7:0];
		T4K28YUV_write_cmos_sensor(0x3550,0x00);//SPLMKON/SPLMKBL/FAREAMK/CAREAMK/CGATEMK/-/-/-;
		T4K28YUV_write_cmos_sensor(0x3551,0x03);//-/-/-/-/-/SPLADRH[10:8];
		T4K28YUV_write_cmos_sensor(0x3552,0x28);//SPLADRH[7:0];
		T4K28YUV_write_cmos_sensor(0x3553,0x20);//MKFLKON/MKFLKSPD[1:0]/-/-/SPLADRV[10:8];
		T4K28YUV_write_cmos_sensor(0x3554,0x60);//SPLADRV[7:0];
		T4K28YUV_write_cmos_sensor(0x3555,0xF0);//AU_SPARE[7:0];
		T4K28YUV_write_cmos_sensor(0x355D,0x01);//[RO] APL_DONE/-/-/-/-/-/APL_O[9:8];
		T4K28YUV_write_cmos_sensor(0x355E,0x1C);//[RO] APL_O[7:0];
		T4K28YUV_write_cmos_sensor(0x355F,0x03);//[RO] ALC_ES[15:8];
		T4K28YUV_write_cmos_sensor(0x3560,0x00);//[RO] ALC_ES[7:0];
		T4K28YUV_write_cmos_sensor(0x3561,0x00);//[RO] -/-/-/-/ALC_AG[11:8];
		T4K28YUV_write_cmos_sensor(0x3562,0x2C);//[RO] ALC_AG[7:0];
		T4K28YUV_write_cmos_sensor(0x3563,0x00);//[RO] -/-/-/-/-/-/ALC_DG[9:8];
		T4K28YUV_write_cmos_sensor(0x3564,0x50);//[RO] ALC_DG[7:0];
		T4K28YUV_write_cmos_sensor(0x3565,0x00);//[RO] -/-/FR_TIM[5:0];
		T4K28YUV_write_cmos_sensor(0x3566,0xD2);//[RO] ALC_OK/AC60M/DET_SIG/FL_ESLIM/-/-/mag_vg[9:8];
		T4K28YUV_write_cmos_sensor(0x3567,0x2E);//[RO] mag_vg[7:0];
		T4K28YUV_write_cmos_sensor(0x3568,0x00);//[RO] -/-/-/-/-/-/AVE_USIG[9:8];
		T4K28YUV_write_cmos_sensor(0x3569,0x00);//[RO] AVE_USIG[7:0];
		T4K28YUV_write_cmos_sensor(0x356A,0x00);//[RO] -/-/-/-/-/-/AVE_VSIG[9:8];
		T4K28YUV_write_cmos_sensor(0x356B,0x00);//[RO] AVE_VSIG[7:0];
		T4K28YUV_write_cmos_sensor(0x356C,0xFF);//[RO] NUM_UVON[15:8];
		T4K28YUV_write_cmos_sensor(0x356D,0xFF);//[RO] NUM_UVON[7:0];
		T4K28YUV_write_cmos_sensor(0x356E,0x01);//[RO] -/-/-/-/-/-/AWBGAINR[9:8];
		T4K28YUV_write_cmos_sensor(0x356F,0x72);//[RO] AWBGAINR[7:0];
		T4K28YUV_write_cmos_sensor(0x3570,0x01);//[RO] -/-/-/-/-/-/AWBGAING[9:8];
		T4K28YUV_write_cmos_sensor(0x3571,0x00);//[RO] AWBGAING[7:0];
		T4K28YUV_write_cmos_sensor(0x3572,0x01);//[RO] -/-/-/-/-/-/AWBGAINB[9:8];
		T4K28YUV_write_cmos_sensor(0x3573,0x4D);//[RO] AWBGAINB[7:0];
		T4K28YUV_write_cmos_sensor(0x3574,0x01);//[RO] -/-/-/-/-/-/SET_ALCLV[9:8];
		T4K28YUV_write_cmos_sensor(0x3575,0x20);//[RO] SET_ALCLV[7:0];
		T4K28YUV_write_cmos_sensor(0x3576,0x45);//[RO] MKY_DATA[7:0];
		T4K28YUV_write_cmos_sensor(0x3577,0x10);//[RO] MKU_DATA[7:0];
		T4K28YUV_write_cmos_sensor(0x3578,0xEE);//[RO] MKV_DATA[7:0];
		T4K28YUV_write_cmos_sensor(0x3579,0x09);//[RO] -/-/-/FL600A[12:8];
		T4K28YUV_write_cmos_sensor(0x357A,0x99);//[RO] FL600A[7:0];
		T4K28YUV_write_cmos_sensor(0x357B,0x00);//[RO] AG_CONT[7:0];
		T4K28YUV_write_cmos_sensor(0x357C,0xE0);//[RO] AU_RO_SPARE[7:0];
		T4K28YUV_write_cmos_sensor(0x357D,0x00);//[RO] -/-/-/-/-/-/-/ALCCLR;
		T4K28YUV_write_cmos_sensor(0x3900,0x00);//OTP_STA/-/-/-/OTP_VMOD/OTP_VERIFY/OTP_WREC/OTP_ENBL;
		T4K28YUV_write_cmos_sensor(0x3901,0x07);//OTP_GVRD/-/-/-/-/OTP_PCLK[2:0];
		T4K28YUV_write_cmos_sensor(0x3902,0x00);//[RO] OTP_ISTS[2:0]/-/-/OTP_TOE/OTP_VE/OTP_BUSY;
		T4K28YUV_write_cmos_sensor(0x3903,0x00);//-/-/-/-/-/-/OTP_ADRS[1:0];
		T4K28YUV_write_cmos_sensor(0x3904,0x00);//OTP_DATA0[7:0];
		T4K28YUV_write_cmos_sensor(0x3905,0x00);//OTP_DATA1[7:0];
		T4K28YUV_write_cmos_sensor(0x3906,0x00);//OTP_DATA2[7:0];
		T4K28YUV_write_cmos_sensor(0x3907,0x00);//OTP_DATA3[7:0];
		T4K28YUV_write_cmos_sensor(0x3908,0x00);//OTP_DATA4[7:0];
		T4K28YUV_write_cmos_sensor(0x3909,0x00);//OTP_DATA5[7:0];
		T4K28YUV_write_cmos_sensor(0x390A,0x00);//OTP_DATA6[7:0];
		T4K28YUV_write_cmos_sensor(0x390B,0x00);//OTP_DATA7[7:0];
		T4K28YUV_write_cmos_sensor(0x390C,0x00);//OTP_PERR1/OTP_PERR2/OTP_CLRE/OTP_TEST[4:0];
		T4K28YUV_write_cmos_sensor(0x30F0,0x00);//T_MONDIRECT/-/-/T_DACTEST/-/T_TMOSEL[2:0];
		T4K28YUV_write_cmos_sensor(0x3010,0x01);//-/-/-/-/-/-/-/MODSEL;
		#endif
	}

	Sleep(20);

}

/*************************************************************************
* FUNCTION
*    T4K28YUV_set_2M_init
*
* DESCRIPTION
*    init sensor 2Mega setting
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_set_2M_init(void)
{
	//FULL SIZE 15FPS
	T4K28YUV_write_cmos_sensor(0x3012, 0x03);//-/-/-/-/-/-/GROUP_HOLD
	T4K28YUV_write_cmos_sensor(0x3015, 0x07);//-/H_COUNT[12:8]
	T4K28YUV_write_cmos_sensor(0x3016, 0x16);//H_COUNT[7:0]
	T4K28YUV_write_cmos_sensor(0x3017, 0x03);//-/-/-/V_COUNT[12:8]
	T4K28YUV_write_cmos_sensor(0x3018, 0x00);//V_COUNT[7:0]
	T4K28YUV_write_cmos_sensor(0x3019, 0x00);//-/-/-/-/-/-/-/SCALE_M[8]
	T4K28YUV_write_cmos_sensor(0x301A, 0x10);//SCALE_M[7:0]
	T4K28YUV_write_cmos_sensor(0x301B, 0x00);//-/-/-/V_ANABIN/-/-/-/-
	T4K28YUV_write_cmos_sensor(0x301C, 0x01);//-/-/-/-/-/-/-/SCALING_MODE
	T4K28YUV_write_cmos_sensor(0x3020, 0x06);//-/-/-/-/-/HOUTPIX[10:8]
	T4K28YUV_write_cmos_sensor(0x3021, 0x40);//HOUTPIX[7:0]
	T4K28YUV_write_cmos_sensor(0x3022, 0x04);//-/-/-/-/-/VOUTPIX[10:8]
	T4K28YUV_write_cmos_sensor(0x3023, 0xB0);//VOUTPIX[7:0]
	T4K28YUV_write_cmos_sensor(0x334D, 0x40);//LSHCNT_MPY[7:0]
	T4K28YUV_write_cmos_sensor(0x334E, 0x00);//-/-/-/-/LSVCNT_MPY[11:8]
	T4K28YUV_write_cmos_sensor(0x334F, 0xA0);//LSVCNT_MPY[7:0]
	//T4K28YUV_write_cmos_sensor(0x3500, 0x40);//-/-/-/-/-/-/ALCSW/ALCLOCK
	T4K28YUV_set_AE_status(AE_disable);
	T4K28YUV_write_cmos_sensor(0x3012, 0x02);//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD

}

void T4K28YUV_set_2M(void)
{
    T4K28YUV_g_RES = T4K28_2M;
    T4K28YUV_set_2M_init();
}

void T4K28YUV_dump_2M(void)
{
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_set_pv_init
*
* DESCRIPTION
*    init sensor 2Mega setting
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_set_pv_init(void)
{

	T4K28YUV_write_cmos_sensor(0x351B,0x08);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]
	T4K28YUV_write_cmos_sensor(0x3012,0x03);//-/-/-/-/-/-/GROUP_HOLD

	//T4K28YUV_set_AE_mode(KAL_FALSE);
	T4K28YUV_set_AE_status(AE_disable);

	T4K28YUV_write_cmos_sensor(0x3506, ((set_pv_back_es>> 8) & 0xFF));//MES[15:8]
	T4K28YUV_write_cmos_sensor(0x3507, (set_pv_back_es & 0xFF));//MES[7:0]
	T4K28YUV_write_cmos_sensor(0x350a, ((set_pv_back_ag >> 8) & 0xFF));//ESLIMMODE/ROOMDET/-/-/MAG[11:8]
	T4K28YUV_write_cmos_sensor(0x350b, (set_pv_back_ag & 0xFF));//MAG[7:0]
	T4K28YUV_write_cmos_sensor(0x350c, set_pv_back_dg >> 2);//MDG[7:0]

	//	800*600
	T4K28YUV_write_cmos_sensor(0x3015,0x07);//-/-/-/H_COUNT[12:8]
	T4K28YUV_write_cmos_sensor(0x3016,0x16);//H_COUNT[7:0]
	T4K28YUV_write_cmos_sensor(0x3017,0x01);//-/-/-/V_COUNT[12:8]
	T4K28YUV_write_cmos_sensor(0x3018,0x60);//V_COUNT[7:0]
	T4K28YUV_write_cmos_sensor(0x3019,0x00);//-/-/-/-/-/-/-/SCALE_M[8]
	T4K28YUV_write_cmos_sensor(0x301A,0x20);//SCALE_M[7:0]
	T4K28YUV_write_cmos_sensor(0x301B,0x10);//-/-/-/V_ANABIN/-/-/-/-
	T4K28YUV_write_cmos_sensor(0x301C,0x01);//-/-/-/-/-/-/-/SCALING_MODE
	T4K28YUV_write_cmos_sensor(0x3020,0x03);//-/-/-/-/-/HOUTPIX[10:8]
	T4K28YUV_write_cmos_sensor(0x3021,0x20);//HOUTPIX[7:0]
	T4K28YUV_write_cmos_sensor(0x3022,0x02);//-/-/-/-/-/VOUTPIX[10:8]
	T4K28YUV_write_cmos_sensor(0x3023,0x58);//VOUTPIX[7:0]
	T4K28YUV_write_cmos_sensor(0x334D,0x50);//LSHCNT_MPY[7:0]
	T4K28YUV_write_cmos_sensor(0x334E,0x01);//-/-/-/-/LSVCNT_MPY[11:8]
	T4K28YUV_write_cmos_sensor(0x334F,0x40);//LSVCNT_MPY[7:0]

	/*
	T4K28YUV_write_cmos_sensor(0x3015,0x05);//-/-/-/H_COUNT[12:8]
	T4K28YUV_write_cmos_sensor(0x3016,0x16);//H_COUNT[7:0]
	T4K28YUV_write_cmos_sensor(0x3017,0x03);//-/-/-/V_COUNT[12:8]
	T4K28YUV_write_cmos_sensor(0x3018,0x00);//V_COUNT[7:0]
	T4K28YUV_write_cmos_sensor(0x3019,0x00);//-/-/-/-/-/-/-/SCALE_M[8]
	T4K28YUV_write_cmos_sensor(0x301A,0x10);//SCALE_M[7:0]
	T4K28YUV_write_cmos_sensor(0x301B,0x00);//-/-/-/V_ANABIN/-/-/-/-
	T4K28YUV_write_cmos_sensor(0x301C,0x01);//-/-/-/-/-/-/-/SCALING_MODE
	T4K28YUV_write_cmos_sensor(0x3020,0x06);//-/-/-/-/-/HOUTPIX[10:8]
	T4K28YUV_write_cmos_sensor(0x3021,0x40);//HOUTPIX[7:0]
	T4K28YUV_write_cmos_sensor(0x3022,0x04);//-/-/-/-/-/VOUTPIX[10:8]
	T4K28YUV_write_cmos_sensor(0x3023,0xB0);//VOUTPIX[7:0]
	T4K28YUV_write_cmos_sensor(0x334D,0x50);//LSHCNT_MPY[7:0]
	T4K28YUV_write_cmos_sensor(0x334E,0x00);//-/-/-/-/LSVCNT_MPY[11:8]
	T4K28YUV_write_cmos_sensor(0x334F,0xA0);//LSVCNT_MPY[7:0]
	*/

	//T4K28YUV_set_AE_mode(KAL_TRUE);
	T4K28YUV_set_AE_status(AE_enable);

	T4K28YUV_write_cmos_sensor(0x3012,0x02);//-/-/-/-/-/-/GROUP_HOLD

}

/*************************************************************************
* FUNCTION
*   T4K28YUVOpen
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 T4K28YUVOpen(void)
{
//    int retry = 0;
//	int lines;

    T4K28YUV_sensor_id = ((T4K28YUV_read_cmos_sensor(0x3000) << 8) | T4K28YUV_read_cmos_sensor(0x3001));
    printk("MYCAT Read Sensor ID = 0x%04x\n", T4K28YUV_sensor_id);


    if (T4K28YUV_sensor_id != T4K28_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;
    T4K28YUV_Sensor_Init(0);   //xb.pang
    spin_lock(&T4K28_drv_lock);
    t4k28yuv_status.Banding = 0;
	t4k28yuv_status.Brightness = 0;
	t4k28yuv_status.CapDummyLines = 0;
	t4k28yuv_status.CapDummyPixels = 0;
	t4k28yuv_status.CapOpClk = 0;
	t4k28yuv_status.Effect = MEFFECT_OFF;
	t4k28yuv_status.NightMode = FALSE;
	t4k28yuv_status.PvDummyLines = 0;
	t4k28yuv_status.PvDummyPixels = 0;
	t4k28yuv_status.PvOpClk = 0;
	t4k28yuv_status.PvShutter = 0;
	t4k28yuv_status.sceneMode = SCENE_MODE_NORMAL;
	t4k28yuv_status.SensorMode = SENSOR_MODE_INIT;
	t4k28yuv_status.ZoomFactor = 0;
	spin_unlock(&T4K28_drv_lock);
	
    return ERROR_NONE;
}



/*************************************************************************
* FUNCTION
*   T4K28YUV_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of T4K28 to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_SetShutter(kal_uint16 iShutter)
{

    if (iShutter < 1)
        iShutter = 1;

	//T4K28YUV_write_cmos_sensor(0x3500, 0x40);//-/-/-/-/-/-/ALCSW/ALCLOCK
	T4K28YUV_set_AE_status(AE_disable);

	T4K28YUV_write_cmos_sensor(0x3506, ((iShutter >> 8) & 0xFF));//MES[15:8]
	T4K28YUV_write_cmos_sensor(0x3507, (iShutter & 0xFF));//MES[7:0]
}

/*************************************************************************
* FUNCTION
*   T4K28YUV_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT16 T4K28YUV_read_shutter(void)
{
	kal_uint16 MESH = 0;
	kal_uint16 MESL = 0;
	kal_uint16 MES = 0;

	MESH = T4K28YUV_read_cmos_sensor(0x355F);//[RO]ALC_ES[15:8]
	MESL = T4K28YUV_read_cmos_sensor(0x3560);//[RO]ALC_ES[7:0]
	MES = (MESH << 8) | (MESL);
    return MES;
}

/*************************************************************************
* FUNCTION
*   T4K28_night_mode
*
* DESCRIPTION
*   This function night mode of T4K28.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_NightMode(kal_bool bEnable)
{
	//if(KAL_TRUE = parser_enable) return KAL_TRUE;

    if(bEnable) {
        if(T4K28YUV_MPEG4_encode_mode==KAL_TRUE) {
			T4K28YUV_write_cmos_sensor(0x351B,0x18);  //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Fix 15fps
			
        } else {
			T4K28YUV_write_cmos_sensor(0x351B,0xA8);  //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Auto Min 10fps
        }
    } else {
        if(T4K28YUV_MPEG4_encode_mode==KAL_TRUE) {
			T4K28YUV_write_cmos_sensor(0x351B,0x08); //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Fix 30fps
        } else {
			T4K28YUV_write_cmos_sensor(0x351B,0x98); //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Auto Min 15fps
        }
    }
}

/*************************************************************************
* FUNCTION
*   T4K28YUVClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 T4K28YUVClose(void)
{
    //parser_close();  //xb.pang
    return ERROR_NONE;
}

void T4K28YUV_Set_Mirror_Flip(kal_uint8 image_mirror)
{
//	kal_uint16 iMirror, iFlip;

    switch (image_mirror)
	{
	    case IMAGE_NORMAL:
			T4K28YUV_write_cmos_sensor(0x3011,0x00);//Set normal
			break;
	    case IMAGE_H_MIRROR:
			T4K28YUV_write_cmos_sensor(0x3011,0x01);//set IMAGE_H_MIRROR
			break;
	    case IMAGE_V_MIRROR:
			T4K28YUV_write_cmos_sensor(0x3011,0x02);//set IMAGE_V_MIRROR
			break;
	    case IMAGE_HV_MIRROR:
			T4K28YUV_write_cmos_sensor(0x3011,0x03);//Set IMAGE_HV_MIRROR
			break;
    }
}

/*************************************************************************
* FUNCTION
*   T4K28YUVPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
bool capture_flag = FALSE;
UINT32 T4K28YUVPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;

	g_iT4K28YUV_Mode = T4K28_MODE_PREVIEW;
	capture_flag = FALSE;

	if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)
	{
	    spin_lock(&T4K28_drv_lock);
		T4K28YUV_MPEG4_encode_mode = KAL_TRUE;
		spin_unlock(&T4K28_drv_lock);
	}
	else
	{
		spin_lock(&T4K28_drv_lock);
		T4K28YUV_MPEG4_encode_mode = KAL_FALSE;
		spin_unlock(&T4K28_drv_lock);
	}

	T4K28YUV_set_pv_init();

	iStartX = 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
	iStartY = 2 * T4K28_IMAGE_SENSOR_PV_STARTY;
	image_window->GrabStartX = iStartX;
	image_window->GrabStartY = iStartY;
	image_window->ExposureWindowWidth = T4K28_IMAGE_SENSOR_PV_WIDTH - 2 * iStartX;
	image_window->ExposureWindowHeight = T4K28_IMAGE_SENSOR_PV_HEIGHT - 2 * iStartY;

	T4K28YUV_Set_Mirror_Flip(IMAGE_NORMAL);
	spin_lock(&T4K28_drv_lock);
	t4k28yuv_status.SensorMode = SENSOR_MODE_PREVIEW;
    spin_unlock(&T4K28_drv_lock);

	memcpy(&T4K28YUVSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));


	T4K28YUV_write_cmos_sensor(0x351B,0x98);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]

	
	return ERROR_NONE;
}	/* T4K28YUVPreview() */

UINT32 T4K28YUVCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 iStartX = 0, iStartY = 0;
	
	kal_uint32 MESH = 0;
	kal_uint32 MESL = 0;
	kal_uint32 MES = 0;
	kal_uint32 MAGH = 0;
	kal_uint32 MAGL = 0;
	kal_uint32 MAG = 0;
	kal_uint32 MDG = 0;
	kal_uint32 MDGH = 0;
	kal_uint32 MDGL = 0;

//	kal_uint32 tmp_ae;
	//T4K28YUV_write_cmos_sensor(0x3500,0xE0);   //-/-/-/-/-/-/ALCSW/ALCLOCK
	T4K28YUV_set_AE_status(AE_lock);

	if(capture_flag == FALSE) {
		MESH = T4K28YUV_read_cmos_sensor(0x355F);//[RO]ALC_ES[15:8]
		MESL = T4K28YUV_read_cmos_sensor(0x3560);//[RO]ALC_ES[7:0]
		MAGH = T4K28YUV_read_cmos_sensor(0x3561);//[RO]-/-/-/-/ALC_AG[11:8]
		MAGL = T4K28YUV_read_cmos_sensor(0x3562);//[RO]ALC_AG[7:0]
		MDGH = T4K28YUV_read_cmos_sensor(0x3563);//[RO]-/-/-/-/-/-/ALC_DG[9:8]
		MDGL = T4K28YUV_read_cmos_sensor(0x3564);//[RO]ALC_DG[7:0]
	    capture_flag = TRUE;
		
		spin_lock(&T4K28_drv_lock);
	    t4k28yuv_status.SensorMode = SENSOR_MODE_CAPTURE;
        spin_unlock(&T4K28_drv_lock);
	}
//L = MES * MAG *MDG;
	MES = (MESH << 8) | (MESL);
	MAG = (MAGH << 8) | (MAGL);
	MDG = ((MDGH & 0x03) << 8) | (MDGL & 0xFF);

	set_pv_back_es = MES;
	set_pv_back_ag = MAG;
	set_pv_back_dg = MDG;

	if ((image_window->ImageTargetWidth <= T4K28_IMAGE_SENSOR_PV_WIDTH) &&
        (image_window->ImageTargetHeight <= T4K28_IMAGE_SENSOR_PV_HEIGHT)) {

        iStartX = T4K28_IMAGE_SENSOR_PV_STARTX;
        iStartY = T4K28_IMAGE_SENSOR_PV_STARTY;
        image_window->GrabStartX = iStartX;
        image_window->GrabStartY = iStartY;
        image_window->ExposureWindowWidth = T4K28_IMAGE_SENSOR_PV_WIDTH - 2 * iStartX;
        image_window->ExposureWindowHeight = T4K28_IMAGE_SENSOR_PV_HEIGHT - 2 * iStartY;
    } else { // 2M  Mode
		T4K28YUV_set_2M();
		T4K28YUV_write_cmos_sensor(0x3506, ((MES >> 8) & 0xFF));//MES[15:8]
		T4K28YUV_write_cmos_sensor(0x3507, (MES & 0xFF));//MES[7:0]
		T4K28YUV_write_cmos_sensor(0x350a, ((MAG >> 8) & 0xFF));//ESLIMMODE/ROOMDET/-/-/MAG[11:8]
		T4K28YUV_write_cmos_sensor(0x350b, (MAG & 0xFF));//MAG[7:0]
		T4K28YUV_write_cmos_sensor(0x350c, MDG >> 2);//MDG[7:0]


        if (MES > ((0x0300)*2+10))
        {
		   T4K28YUV_write_cmos_sensor(0x3017, ((MES >> 8) & 0xFF));;//-/-/-/V_COUNT[12:8]
		   T4K28YUV_write_cmos_sensor(0x3018, (MES & 0xFF));;//V_COUNT[12:8]
        }




        iStartX = 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
        iStartY = 2 * T4K28_IMAGE_SENSOR_PV_STARTY;
        image_window->GrabStartX = iStartX;
        image_window->GrabStartY = iStartY;
        image_window->ExposureWindowWidth = T4K28_IMAGE_SENSOR_FULL_WIDTH -2 * iStartX;
        image_window->ExposureWindowHeight = T4K28_IMAGE_SENSOR_FULL_HEIGHT-2 * iStartY;

    }

    memcpy(&T4K28YUVSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}
UINT32 T4K28YUVZsdPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;

	spin_lock(&T4K28_drv_lock);
	t4k28yuv_status.SensorMode = SENSOR_MODE_ZSD;
    spin_unlock(&T4K28_drv_lock);

	//T4K28YUV_write_cmos_sensor(0x3500,0xE0);   //-/-/-/-/-/-/ALCSW/ALCLOCK
	
	T4K28YUV_set_2M();
	T4K28YUV_set_AE_status(AE_enable);
	        iStartX = 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
        iStartY = 2 * T4K28_IMAGE_SENSOR_PV_STARTY;
        image_window->GrabStartX = iStartX;
        image_window->GrabStartY = iStartY;
        image_window->ExposureWindowWidth = T4K28_IMAGE_SENSOR_FULL_WIDTH -2 * iStartX;
        image_window->ExposureWindowHeight = T4K28_IMAGE_SENSOR_FULL_HEIGHT-2 * iStartY;
		
		T4K28YUV_Set_Mirror_Flip(IMAGE_NORMAL);
		memcpy(&T4K28YUVSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
}


UINT32 T4K28YUVGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth = T4K28_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight = T4K28_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth = T4K28_IMAGE_SENSOR_PV_WIDTH - 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
    pSensorResolution->SensorPreviewHeight = T4K28_IMAGE_SENSOR_PV_HEIGHT - 2 * T4K28_IMAGE_SENSOR_PV_STARTY;
   	pSensorResolution->SensorVideoWidth = T4K28_IMAGE_SENSOR_PV_WIDTH - 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
    pSensorResolution->SensorVideoHeight = T4K28_IMAGE_SENSOR_PV_HEIGHT - 2 * T4K28_IMAGE_SENSOR_PV_STARTY;

    return ERROR_NONE;
}
UINT32 T4K28YUVGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
													MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	//pSensorInfo->SensorPreviewResolutionX = T4K28_IMAGE_SENSOR_PV_WIDTH;
    //pSensorInfo->SensorPreviewResolutionY = T4K28_IMAGE_SENSOR_PV_HEIGHT;
	switch(ScenarioId)
	{
		  case MSDK_SCENARIO_ID_CAMERA_ZSD:
			   pSensorInfo->SensorPreviewResolutionX=T4K28_IMAGE_SENSOR_FULL_WIDTH - 2 * T4K28_IMAGE_SENSOR_PV_STARTX;;
			   pSensorInfo->SensorPreviewResolutionY=T4K28_IMAGE_SENSOR_FULL_HEIGHT - 2 * T4K28_IMAGE_SENSOR_PV_STARTY;
			   pSensorInfo->SensorCameraPreviewFrameRate=14;
		  break;
		  default:
			   pSensorInfo->SensorPreviewResolutionX=T4K28_IMAGE_SENSOR_PV_WIDTH;
			   pSensorInfo->SensorPreviewResolutionY=T4K28_IMAGE_SENSOR_PV_HEIGHT;
			   pSensorInfo->SensorCameraPreviewFrameRate=30;
	}

	pSensorInfo->SensorFullResolutionX = T4K28_IMAGE_SENSOR_FULL_WIDTH - 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
    pSensorInfo->SensorFullResolutionY = T4K28_IMAGE_SENSOR_FULL_HEIGHT - 2 * T4K28_IMAGE_SENSOR_PV_STARTY;

    pSensorInfo->SensorCameraPreviewFrameRate = 30;
    pSensorInfo->SensorVideoFrameRate = 30;
    pSensorInfo->SensorStillCaptureFrameRate = 14;
    pSensorInfo->SensorWebCamCaptureFrameRate = 15;
    pSensorInfo->SensorResetActiveHigh = FALSE;
    pSensorInfo->SensorResetDelayCount = 5;
    pSensorInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_UYVY;

    pSensorInfo->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 2;
    pSensorInfo->PreviewDelayFrame = 4;//2;
    pSensorInfo->VideoDelayFrame = 5;
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;

    pSensorInfo->AEShutDelayFrame = 0;/* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 1;/* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 1;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq = 24;
            pSensorInfo->SensorClockDividCount =	3;
            pSensorInfo->SensorClockRisingCount = 0;
            pSensorInfo->SensorClockFallingCount = 2;
            pSensorInfo->SensorPixelClockCount = 3;
            pSensorInfo->SensorDataLatchCount = 2;
            pSensorInfo->SensorGrabStartX = T4K28_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = T4K28_IMAGE_SENSOR_PV_STARTY;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            pSensorInfo->SensorClockFreq = 24;
            pSensorInfo->SensorClockDividCount = 3;
            pSensorInfo->SensorClockRisingCount = 0;
            pSensorInfo->SensorClockFallingCount = 2;
            pSensorInfo->SensorPixelClockCount = 3;
            pSensorInfo->SensorDataLatchCount = 2;
            pSensorInfo->SensorGrabStartX = 2*T4K28_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = 2*T4K28_IMAGE_SENSOR_PV_STARTY;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
            pSensorInfo->SensorClockFreq = 24;
            pSensorInfo->SensorClockDividCount =	3;
            pSensorInfo->SensorClockRisingCount = 0;
            pSensorInfo->SensorClockFallingCount = 2;
            pSensorInfo->SensorPixelClockCount = 3;
            pSensorInfo->SensorDataLatchCount = 2;
            pSensorInfo->SensorGrabStartX = 1;
            pSensorInfo->SensorGrabStartY = 1;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    T4K28YUVPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &T4K28YUVSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* T4K28YUVGetInfo() */


UINT32 T4K28YUVControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            T4K28YUVPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            T4K28YUVCapture(pImageWindow, pSensorConfigData);
            break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
	        SENSORDB("[HI251]CONTROLFLOW MSDK_SCENARIO_ID_CAMERA_ZSD\n" );
	        T4K28YUVZsdPreview(pImageWindow, pSensorConfigData);
	        break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    return TRUE;
}

UINT32 T4K28YUVSetVideoMode(UINT16 u2FrameRate)
{
    //XB.PANG NEED CHECK set the video mode fps---normal/night
    
    if(u2FrameRate == 15) {
			T4K28YUV_write_cmos_sensor(0x351B,0x18);  //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Fix 15fps
		
    } else if(u2FrameRate == 30) {
			T4K28YUV_write_cmos_sensor(0x351B,0x08); //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Fix 30fps
    }else
    {
      printk("Wrong frame rate setting \n");
	}
	
    spin_lock(&T4K28_drv_lock);
    T4K28YUV_MPEG4_encode_mode = KAL_TRUE;
	spin_unlock(&T4K28_drv_lock);
    return TRUE;
}

kal_uint32 T4K28_set_param_wb(kal_uint32 para)
{
	//if(KAL_TRUE = parser_enable) return KAL_TRUE;

    switch (para)
    {
        case AWB_MODE_AUTO:
			T4K28YUV_write_cmos_sensor(0x3500,0xC0);//AWBSW/AWBONDOT[2:0]/-/-/WBMRG[9:8];
			T4K28YUV_write_cmos_sensor(0x3322,0x23);//PWBGAINGR[7:0]
			T4K28YUV_write_cmos_sensor(0x3323,0x23);//PWBGAINGB[7:0]
			T4K28YUV_write_cmos_sensor(0x3324,0x05);//PWBGAINR[7:0]
			T4K28YUV_write_cmos_sensor(0x3325,0x50);//PWBGAINB[7:0]

          break;

        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			T4K28YUV_write_cmos_sensor(0x3500, 0x80);//AWBSW/AWBONDOT[2:0]/-/-/WBMRG[9:8];
			T4K28YUV_write_cmos_sensor(0x3322, 0x30);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323, 0x30);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324, 0x30);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325, 0x50);//PWBGAINB[7:0];
          break;

        case AWB_MODE_DAYLIGHT: //sunny
			T4K28YUV_write_cmos_sensor(0x3500, 0x80);//AWBSW/AWBONDOT[2:0]/-/-/WBMRG[9:8];
			T4K28YUV_write_cmos_sensor(0x3322, 0x20);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323, 0x20);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324, 0x10);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325, 0x40);//PWBGAINB[7:0];
          break;

        case AWB_MODE_INCANDESCENT: //office
			T4K28YUV_write_cmos_sensor(0x3500, 0x80);//AWBSW/AWBONDOT[2:0]/-/-/WBMRG[9:8];
			T4K28YUV_write_cmos_sensor(0x3322, 0x50);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323, 0x50);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324, 0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325, 0xF0);//PWBGAINB[7:0];
          break;

        case AWB_MODE_TUNGSTEN: //home
			T4K28YUV_write_cmos_sensor(0x3500, 0x80);//AWBSW/AWBONDOT[2:0]/-/-/WBMRG[9:8];
			T4K28YUV_write_cmos_sensor(0x3322, 0x40);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323, 0x40);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324, 0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325, 0xE0);//PWBGAINB[7:0];
          break;

        case AWB_MODE_FLUORESCENT:
			T4K28YUV_write_cmos_sensor(0x3500, 0x80);//AWBSW/AWBONDOT[2:0]/-/-/WBMRG[9:8];
			T4K28YUV_write_cmos_sensor(0x3322, 0x50);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323, 0x50);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324, 0x10);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325, 0xC0);//PWBGAINB[7:0];

          break;
	default:
            return KAL_FALSE;
    }
    return KAL_TRUE;
}

kal_uint32 T4K28_set_param_exposure(kal_uint32 para)
{
	//if(KAL_TRUE = parser_enable) return KAL_TRUE;
	if (t4k28yuv_status.sceneMode == SCENE_MODE_HDR)
	  {
		 switch (para)
					{
					  case AE_EV_COMP_n20:	
						  /* EV -2 */
						  SENSORDB("[Hi251_Debug_HDR]AE_EV_COMP_n20 Para:%d;\n",para);	 
						  T4K28YUV_write_cmos_sensor(0x3501,0x00);
				          T4K28YUV_write_cmos_sensor(0x3502,0x80);
					      break;
					  case AE_EV_COMP_20:			   /* EV +2 */
						   SENSORDB("[Hi251_Debug_HDR]AE_EV_COMP_20 Para:%d;\n",para);
						   T4K28YUV_write_cmos_sensor(0x3501,0x01);
				           T4K28YUV_write_cmos_sensor(0x3502,0xd0);
				          break;
					  case AE_EV_COMP_00:			 /* EV 00 */
						   SENSORDB("[Hi251_Debug_HDR]ISP_BRIGHT_MIDDLE Para:%d;\n",para);
						   T4K28YUV_write_cmos_sensor(0x3501,0x01);
				           T4K28YUV_write_cmos_sensor(0x3502,0x30);
						   break;
						default:
							return KAL_FALSE;
					}
	  }
	else
	{

	    switch (para)
	    {
	        case AE_EV_COMP_n20:
				T4K28YUV_write_cmos_sensor(0x3501,0x0);
				T4K28YUV_write_cmos_sensor(0x3502,0x80);
		        break;
/*
	        case AE_EV_COMP_n07:
				T4K28YUV_write_cmos_sensor(0x343F,0xe0);//BRIGHT1[7:0]
				T4K28YUV_write_cmos_sensor(0x3440,0xc0);//BRIGHT0[7:0]
	    	    break;
*/
	        case AE_EV_COMP_n10:
				T4K28YUV_write_cmos_sensor(0x3501,0x0);
				T4K28YUV_write_cmos_sensor(0x3502,0xd0);
	        	break;

	        case AE_EV_COMP_00:
				T4K28YUV_write_cmos_sensor(0x3501,0x1);
				T4K28YUV_write_cmos_sensor(0x3502,0x30);
		        break;

	        case AE_EV_COMP_10:
				T4K28YUV_write_cmos_sensor(0x3501,0x01);
				T4K28YUV_write_cmos_sensor(0x3502,0x80);
		        break;
/*
	        case AE_EV_COMP_07:
				T4K28YUV_write_cmos_sensor(0x343F,0x40);//BRIGHT1[7:0]
				T4K28YUV_write_cmos_sensor(0x3440,0x20);//BRIGHT0[7:0]
		        break;
*/
	        case AE_EV_COMP_20:
				T4K28YUV_write_cmos_sensor(0x3501,0x01);
				T4K28YUV_write_cmos_sensor(0x3502,0xd0);
	    	    break;

	        default:
	            return KAL_FALSE;
	    }
	}
    return KAL_TRUE;
}

kal_uint32 T4K28_set_param_effect(kal_uint32 para)
{
    kal_uint32 ret = KAL_TRUE;

	//if(KAL_TRUE = parser_enable) return KAL_TRUE;

    switch (para)
    {
        case MEFFECT_OFF:
			T4K28YUV_write_cmos_sensor(0x3402,0x00);//-/-/-/-/PICEFF[3:0];
            break;

	 	case MEFFECT_MONO:
			T4K28YUV_write_cmos_sensor(0x3402,0x06);//-/-/-/-/PICEFF[3:0];
            break;

        case MEFFECT_SEPIA:
			T4K28YUV_write_cmos_sensor(0x3402, 0x05);//-/-/-/-/PICEFF[3:0];
			T4K28YUV_write_cmos_sensor(0x3454, 0x6A);//
			T4K28YUV_write_cmos_sensor(0x3455, 0x93);//
            break;

        case MEFFECT_NEGATIVE:
			T4K28YUV_write_cmos_sensor(0x3402,0x03);//-/-/-/-/PICEFF[3:0];
            break;

        case MEFFECT_SEPIABLUE:
			T4K28YUV_write_cmos_sensor(0x3402, 0x05);//-/-/-/-/PICEFF[3:0];
			T4K28YUV_write_cmos_sensor(0x3454, 0xa9);//
			T4K28YUV_write_cmos_sensor(0x3455, 0x60);//
            break;

        default:
            ret = KAL_FALSE;
    }
    return KAL_TRUE;
}

kal_uint32 T4K28_set_param_banding(kal_uint32 para)
{
	//if(KAL_TRUE = parser_enable) return KAL_TRUE;

    switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
			T4K28YUV_write_cmos_sensor(0x351E,0x16);//ACFDET/AC60M/FLMANU/ACDETDLY/MSKLINE[1:0]/ACDPWAIT[1:0]
	    	break;

        case AE_FLICKER_MODE_60HZ:
			T4K28YUV_write_cmos_sensor(0x351E,0x56);//ACFDET/AC60M/FLMANU/ACDETDLY/MSKLINE[1:0]/ACDPWAIT[1:0]
	    	break;

        default:
            T4K28YUV_write_cmos_sensor(0x351E,0x16);//ACFDET/AC60M/FLMANU/ACDETDLY/MSKLINE[1:0]/ACDPWAIT[1:0]
            return KAL_FALSE;
    }
    return KAL_TRUE;
}

//add new; XB.PANG NEED CHECK
void T4K28YUVset_scene_mode(UINT16 para)
{

	spin_lock(&T4K28_drv_lock);
    t4k28yuv_status.sceneMode = para;
	spin_unlock(&T4K28_drv_lock);

	SENSORDB("iPara=%d",para);
	switch(para)
	{
		case SCENE_MODE_HDR:
			 //XB.PANG NEED CHECK
			 
			 break;
			 
		case SCENE_MODE_SUNSET:
	         //XB.PANG NEED CHECK
			 T4K28YUV_write_cmos_sensor(0x351B,0x00A8);  //,FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			 T4K28YUV_write_cmos_sensor(0x3503,0x0025);  //,AGMIN[7:0];
			 T4K28YUV_write_cmos_sensor(0x3504,0x0001);  //,-/-/-/-/AGMAX[11:8];
			 T4K28YUV_write_cmos_sensor(0x3505,0x0000);  //,AGMAX[7:0];
			 T4K28YUV_write_cmos_sensor(0x350D,0x0019);  //,A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			 T4K28YUV_write_cmos_sensor(0x350E,0x001b);  //,A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			 T4K28YUV_write_cmos_sensor(0x350F,0x0092);  //,B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			 T4K28YUV_write_cmos_sensor(0x3510,0x00a0);  //,C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			 T4K28YUV_write_cmos_sensor(0x3422,0x00A8);  //,Cbr_MGAIN[7:0];
			 T4K28YUV_write_cmos_sensor(0x3322,0x002A);  //,PWBGAINGR[7:0];
			 T4K28YUV_write_cmos_sensor(0x3323,0x002A);  //,PWBGAINGB[7:0];
			 T4K28YUV_write_cmos_sensor(0x3324,0x0000);  //,PWBGAINR[7:0];
			 T4K28YUV_write_cmos_sensor(0x3325,0x00C4);  //,PWBGAINB[7:0];
			 T4K28YUV_write_cmos_sensor(0x352A,0x0002);  //,WBGRMAX[7:0];
			 T4K28YUV_write_cmos_sensor(0x352B,0x0002);  //,WBGRMIN[7:0];
			 T4K28YUV_write_cmos_sensor(0x352C,0x0002);  //,WBGBMAX[7:0];
			 T4K28YUV_write_cmos_sensor(0x352D,0x0002);  //,WBGBMIN[7:0];

			break;
			
		case SCENE_MODE_NIGHTSCENE:
			T4K28YUV_NightMode(TRUE);
			T4K28YUV_write_cmos_sensor(0x351B,0x00A8);	//,FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			T4K28YUV_write_cmos_sensor(0x3503,0x0025);	//,AGMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3504,0x0001);	//,-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x0020);	//,AGMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x350D,0x0019);	//,A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350E,0x001b);	//,A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350F,0x0092);	//,B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x3510,0x00a0);	//,C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			T4K28YUV_write_cmos_sensor(0x3422,0x00A8);	//,Cbr_MGAIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3322,0x0000);	//,PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x0000);	//,PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x0000);	//,PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x0038);	//,PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x0020);	//,WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x0020);	//,WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x0020);	//,WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x0020);	//,WBGBMIN[7:0];

			break;
		case SCENE_MODE_PORTRAIT:
			T4K28YUV_write_cmos_sensor(0x351B,0x00A8);	//,FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			T4K28YUV_write_cmos_sensor(0x3503,0x0025);	//,AGMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3504,0x0001);	//,-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x0000);	//,AGMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x350D,0x0008);	//,A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350E,0x000B);	//,A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350F,0x0080);	//,B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x3510,0x0080);	//,C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			T4K28YUV_write_cmos_sensor(0x3422,0x00A8);	//,Cbr_MGAIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3322,0x0000);	//,PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x0000);	//,PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x0000);	//,PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x0038);	//,PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x0020);	//,WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x0020);	//,WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x0020);	//,WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x0020);	//,WBGBMIN[7:0];

			break;
			
		case SCENE_MODE_LANDSCAPE:
			T4K28YUV_write_cmos_sensor(0x351B,0x00A8);  //,FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			T4K28YUV_write_cmos_sensor(0x3503,0x0025);  //,AGMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3504,0x0001);  //,-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x0000);  //,AGMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x350D,0x0019);  //,A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350E,0x001b);  //,A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350F,0x0092);  //,B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x3510,0x00a0);  //,C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			T4K28YUV_write_cmos_sensor(0x3422,0x00D0);  //,Cbr_MGAIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3322,0x0000);  //,PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x0000);  //,PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x0000);  //,PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x0038);  //,PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x0020);  //,WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x0020);  //,WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x0020);  //,WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x0020);  //,WBGBMIN[7:0];		
			break;
			
		case SCENE_MODE_SPORTS:
			T4K28YUV_write_cmos_sensor(0x351B,0x0008);	//,FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			T4K28YUV_write_cmos_sensor(0x3503,0x0025);	//,AGMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3504,0x0001);	//,-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x0000);	//,AGMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x350D,0x0019);	//,A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350E,0x001b);	//,A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350F,0x0092);	//,B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x3510,0x00a0);	//,C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			T4K28YUV_write_cmos_sensor(0x3422,0x00A8);	//,Cbr_MGAIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3322,0x0000);	//,PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x0000);	//,PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x0000);	//,PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x0038);	//,PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x0020);	//,WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x0020);	//,WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x0020);	//,WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x0020);	//,WBGBMIN[7:0];

			break;
	
		case SCENE_MODE_OFF:
		default:
		   T4K28YUV_NightMode(FALSE);
		   T4K28YUV_write_cmos_sensor(0x3503,0x0025);  //,AGMIN[7:0];
		   T4K28YUV_write_cmos_sensor(0x3504,0x0001);  //,-/-/-/-/AGMAX[11:8];
		   T4K28YUV_write_cmos_sensor(0x3505,0x0000);  //,AGMAX[7:0];
		   T4K28YUV_write_cmos_sensor(0x350D,0x0019);  //,A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
		   T4K28YUV_write_cmos_sensor(0x350E,0x001b);  //,A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
		   T4K28YUV_write_cmos_sensor(0x350F,0x0092);  //,B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
		   T4K28YUV_write_cmos_sensor(0x3510,0x00a0);  //,C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
		   T4K28YUV_write_cmos_sensor(0x3422,0x00A8);  //,Cbr_MGAIN[7:0];
		   T4K28YUV_write_cmos_sensor(0x3322,0x0000);  //,PWBGAINGR[7:0];
		   T4K28YUV_write_cmos_sensor(0x3323,0x0000);  //,PWBGAINGB[7:0];
		   T4K28YUV_write_cmos_sensor(0x3324,0x0000);  //,PWBGAINR[7:0];
		   T4K28YUV_write_cmos_sensor(0x3325,0x0038);  //,PWBGAINB[7:0];
		   T4K28YUV_write_cmos_sensor(0x352A,0x0020);  //,WBGRMAX[7:0];
		   T4K28YUV_write_cmos_sensor(0x352B,0x0020);  //,WBGRMIN[7:0];
		   T4K28YUV_write_cmos_sensor(0x352C,0x0020);  //,WBGBMAX[7:0];
		   T4K28YUV_write_cmos_sensor(0x352D,0x0020);  //,WBGBMIN[7:0];

		   break;
	
	}

}

//add new; XB.PANG NEED CHECK
void T4K28_set_contrast(UINT16 para)
{

	SENSORDB("[T4K28]CONTROLFLOW T4K28_set_contrast Para:%d;\n",para);
//0x3441 0x85


	switch (para)
    {
        case ISP_CONTRAST_LOW:
			 //Low
			 T4K28YUV_write_cmos_sensor(0x3444, 0x0);
			 T4K28YUV_write_cmos_sensor(0x3446, 0x0);
             break;
			 
        case ISP_CONTRAST_HIGH:
			 //Hig
			 T4K28YUV_write_cmos_sensor(0x3444, 0x8);
			 T4K28YUV_write_cmos_sensor(0x3446, 0x8);
             break;

			 
        case ISP_CONTRAST_MIDDLE:
        default:
	         //Med
	         T4K28YUV_write_cmos_sensor(0x3444, 0x4);
			 T4K28YUV_write_cmos_sensor(0x3446, 0x4);
             break;
    }
    
    return;
}

//add new; XB.PANG NEED CHECK
void T4K28_set_brightness(UINT16 para)
{

	SENSORDB("[T4K28]CONTROLFLOW T4K28_set_brightness Para:%d;\n",para);

    switch (para)
    {
        case ISP_BRIGHT_LOW:
		     //Low
			 T4K28YUV_write_cmos_sensor(0x343f, 0xe0);
			 T4K28YUV_write_cmos_sensor(0x3440, 0xc0);
             break;
			 
        case ISP_BRIGHT_HIGH:
		     //Hig
			 T4K28YUV_write_cmos_sensor(0x343f, 0x40);
			 T4K28YUV_write_cmos_sensor(0x3440, 0x20);
             break;
			 
        case ISP_BRIGHT_MIDDLE:
        default:
	         //Med
			 T4K28YUV_write_cmos_sensor(0x343f, 0x10);
			 T4K28YUV_write_cmos_sensor(0x3440, 0xf0);
             break;
    }

    
    return;	
}

//add new; XB.PANG NEED CHECK
void T4K28_set_saturation(UINT16 para)
{
	
	SENSORDB("[T4K28]CONTROLFLOW T4K28_set_saturation Para:%d;\n",para);
	//0x341e
    //0x3421
    switch (para)
    {
        case ISP_SAT_HIGH:
	         //Hig
	         T4K28YUV_write_cmos_sensor(0x341e, 0xe0);
			 T4K28YUV_write_cmos_sensor(0x3421, 0xe0);
             break;
			 
        case ISP_SAT_LOW:
	         //Low
	         T4K28YUV_write_cmos_sensor(0x341e, 0xa0);
			 T4K28YUV_write_cmos_sensor(0x3421, 0xa0);
             break;
			 
        case ISP_SAT_MIDDLE:
        default:
	         //Med
	         T4K28YUV_write_cmos_sensor(0x341e, 0xc4);
			 T4K28YUV_write_cmos_sensor(0x3421, 0xc4);
             break;
    }
     return;	
}

//add new; XB.PANG NEED CHECK
void T4K28_set_iso(UINT16 para)
{

	SENSORDB("[T4K28]CONTROLFLOW T4K28_set_iso Para:%d;\n",para);

	switch (para)
		{
		case AE_ISO_100:
		     //ISO100
		     T4K28YUV_write_cmos_sensor(0x3508,0x00); //,MMES[15:8];
			 T4K28YUV_write_cmos_sensor(0x3509,0x10); //,MMES[7:0];
			 T4K28YUV_write_cmos_sensor(0x351A,0xFF); //,SATSET[7:0];
			 T4K28YUV_write_cmos_sensor(0x3503,0x25); //,AGMIN[7:0];
			 T4K28YUV_write_cmos_sensor(0x3504,0x00); //,-/-/-/-/AGMAX[11:8];
			 T4K28YUV_write_cmos_sensor(0x3505,0x25); //,AGMAX[7:0];
		     break;
			 
		case AE_ISO_200:
		     //ISO200
			 T4K28YUV_write_cmos_sensor(0x3508,0x00); //,MMES[15:8];
			 T4K28YUV_write_cmos_sensor(0x3509,0x10); //,MMES[7:0];
			 T4K28YUV_write_cmos_sensor(0x351A,0xFF); //,SATSET[7:0];
			 T4K28YUV_write_cmos_sensor(0x3503,0x25); //,AGMIN[7:0];
			 T4K28YUV_write_cmos_sensor(0x3504,0x00); //,-/-/-/-/AGMAX[11:8];
			 T4K28YUV_write_cmos_sensor(0x3505,0x4A); //,AGMAX[7:0];

		     break;
			 
		case AE_ISO_400:
		     //ISO400
			 T4K28YUV_write_cmos_sensor(0x3508,0x00);  //,MMES[15:8];
			 T4K28YUV_write_cmos_sensor(0x3509,0x10);  //,MMES[7:0];
			 T4K28YUV_write_cmos_sensor(0x351A,0xFF);  //,SATSET[7:0];
			 T4K28YUV_write_cmos_sensor(0x3503,0x25);  //,AGMIN[7:0];
			 T4K28YUV_write_cmos_sensor(0x3504,0x00);  //,-/-/-/-/AGMAX[11:8];
			 T4K28YUV_write_cmos_sensor(0x3505,0x94);  //,AGMAX[7:0];
			 break;
			 
		default:
		case AE_ISO_AUTO:
		     //Auto
			 T4K28YUV_write_cmos_sensor(0x3508,0x00); //,MMES[15:8];
			 T4K28YUV_write_cmos_sensor(0x3509,0x86); //,MMES[7:0];
			 T4K28YUV_write_cmos_sensor(0x351A,0xC0); //,SATSET[7:0];
			 T4K28YUV_write_cmos_sensor(0x3503,0x25); //,AGMIN[7:0];
			 T4K28YUV_write_cmos_sensor(0x3504,0x01); //,-/-/-/-/AGMAX[11:8];
			 T4K28YUV_write_cmos_sensor(0x3505,0x00); //,AGMAX[7:0];
			 break;
		}

	return;
}

void T4K28_set_hue(UINT16 para)
{
	
	SENSORDB("[Enter]S5K5EAYX T4K28_set_hue func:para = %d,ISP_HUE_MIDDLE=%d\n",para,ISP_HUE_MIDDLE);

	switch (para)
	{
		case ISP_HUE_LOW:
			T4K28YUV_write_cmos_sensor(0x3404, 0x3b);
			T4K28YUV_write_cmos_sensor(0x3405, 0xdd);
			T4K28YUV_write_cmos_sensor(0x3406, 0x1b);
			T4K28YUV_write_cmos_sensor(0x3407, 0x2a);
			T4K28YUV_write_cmos_sensor(0x3408, 0x2b);
			T4K28YUV_write_cmos_sensor(0x3409, 0x4c);			     
			break;
			
		case ISP_HUE_HIGH:    	     
			 T4K28YUV_write_cmos_sensor(0x3404, 0x39);
			 T4K28YUV_write_cmos_sensor(0x3405, 0xe9);
			 T4K28YUV_write_cmos_sensor(0x3406, 0x29);
			 T4K28YUV_write_cmos_sensor(0x3407, 0x20);
			 T4K28YUV_write_cmos_sensor(0x3408, 0x03);
			 T4K28YUV_write_cmos_sensor(0x3409, 0x6c);
			 break;
			 
		case ISP_HUE_MIDDLE:
		default:
			 T4K28YUV_write_cmos_sensor(0x3404, 0x3a);
			 T4K28YUV_write_cmos_sensor(0x3405, 0xe3);
			 T4K28YUV_write_cmos_sensor(0x3406, 0x22);
			 T4K28YUV_write_cmos_sensor(0x3407, 0x25);
			 T4K28YUV_write_cmos_sensor(0x3408, 0x17);
			 T4K28YUV_write_cmos_sensor(0x3409, 0x5c);			  
			 break;

	}

	return;
}


UINT32 T4K28YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
    //printk("\n T4K28YUVSensorSetting() is called ; \n");
    //SENSORDB("cmd=%d, para = 0x%x\n", iCmd, iPara);

	switch (iCmd)
	{
	case FID_SCENE_MODE:
		T4K28YUVset_scene_mode(iPara);
		//XB.PANG NEED CHECK
		/*
		 if ((iPara == SCENE_MODE_NORMAL) || (iPara == SCENE_MODE_OFF))
		 	{
		        T4K28YUV_NightMode(FALSE);
		    }
		 else if (iPara == SCENE_MODE_NIGHTSCENE)
		 	{
				T4K28YUV_NightMode(TRUE);
		    }
		    */
		break;

	case FID_AWB_MODE:
		T4K28_set_param_wb(iPara);
		break;

	case FID_COLOR_EFFECT:
		T4K28_set_param_effect(iPara);
		break;

	case FID_AE_EV:
		T4K28_set_param_exposure(iPara);
		break;

	case FID_AE_FLICKER:
		T4K28_set_param_banding(iPara);
		break;

	case FID_ZOOM_FACTOR:
        T4K28YUV_zoom_factor = iPara;
		break;
    //XB.PANG NEED CHECK
	case FID_ISP_CONTRAST:
		SENSORDB("[T4K28]FID_ISP_CONTRAST:%d\n",iPara);
		T4K28_set_contrast(iPara);
		break;
	case FID_ISP_BRIGHT:
		SENSORDB("[T4K28]FID_ISP_BRIGHT:%d\n",iPara);
		T4K28_set_brightness(iPara);
		break;
	case FID_ISP_SAT:
		SENSORDB("[T4K28]FID_ISP_SAT:%d\n",iPara);
		T4K28_set_saturation(iPara);
		break;
	case FID_AE_ISO:
		SENSORDB("[T4K28]FID_AE_ISO:%d\n",iPara);
		T4K28_set_iso(iPara);
		break;
	case FID_ISP_HUE:
		SENSORDB("[T4K28]FID_ISP_HUE:%d\n",iPara);
		T4K28_set_hue(iPara);
		break;

	default:
		break;
    }
    return TRUE;
}

/*************************************************************************
* FUNCTION
*   T4K28YUVGetSensorID
*
* DESCRIPTION
*   This function get the sensor ID
*
* PARAMETERS
*   *sensorID : return the sensor ID
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 T4K28YUVGetSensorID(UINT32 *sensorID)
{
    int  retry = 3;
	UINT32 u_sensorid1,u_sensorid2;

    do {
		u_sensorid1 = T4K28YUV_read_cmos_sensor(0x3000);
		u_sensorid2 = T4K28YUV_read_cmos_sensor(0x3001);
		printk("MYCAT Read Sensor ID1,ID2  = %x,%x\n", u_sensorid1, u_sensorid2);
		*sensorID = (((u_sensorid1&0XFF) << 8 ) | (u_sensorid2&0XFF));
		if (*sensorID == T4K28_SENSOR_ID)
		    break;
		printk("MYCAT Read Sensor ID Fail = 0x%04x\n", *sensorID);
		retry --;
    } while (retry > 0);

    if (*sensorID != T4K28_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}
typedef struct
{
  UINT16  iSensorVersion;
  UINT16  iNightMode;
  UINT16  iWB;
  UINT16  iEffect;
  UINT16  iEV;
  UINT16  iBanding;
  UINT16  iMirror;
  UINT16  iFrameRate;
  //0    :No-Fix FrameRate 
  //other:Fixed FrameRate
} T4K28Status;
T4K28Status T4K28CurrentStatus;

void T4K28GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = T4K28CurrentStatus.iWB;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}



#define T4K28YUV_FLASH_BV_THRESHOLD 120 
static void T4K28YUVMIPI_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
	unsigned int NormBr;	   

	NormBr=((T4K28YUV_read_cmos_sensor(0x3561)&0xff)<<8)+T4K28YUV_read_cmos_sensor(0x3562);	
    
    printk("[%s]gain =0x%x \n",__FUNCTION__,NormBr);


	if (NormBr < T4K28YUV_FLASH_BV_THRESHOLD)
	{
	   *pFeatureReturnPara32 = FALSE;
		return;
	}
	*pFeatureReturnPara32 = TRUE;
	return;
}

//XB.PANG NEED CHECK
void T4K28YUV_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          SENSORDB("[T4K28YUV] SENSOR_3A_AE_LOCK\n");//T4K28YUV_set_AE_mode
          T4K28YUV_set_AE_mode(KAL_FALSE);
      break;

      case SENSOR_3A_AE_UNLOCK:
          SENSORDB("[T4K28YUV] SENSOR_3A_AE_UNLOCK\n");
          T4K28YUV_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
          SENSORDB("[T4K28YUV] SENSOR_3A_AWB_LOCK\n");
		  T4K28YUV_set_AWB_mode(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
          SENSORDB("[T4K28YUV] SENSOR_3A_AWB_UNLOCK\n");
          T4K28YUV_set_AWB_mode(KAL_TRUE);
		  
      break;
	  
      default:
      break;
   }
   return;
}

#define T4K28YUV_TEST_PATTERN_CHECKSUM 0x96f31f11 //0x0334b745
UINT32 T4K28YUVSetTestPatternMode(kal_bool bEnable)
{
	kal_uint8 temp_testpattern_reg = 0;
    temp_testpattern_reg = T4K28YUV_read_cmos_sensor(0x335F);
    //XB.PANG NEED CHECK
    if (bEnable)
    {
		//enable test pattern
		T4K28YUV_write_cmos_sensor(0x335F, (temp_testpattern_reg | 0x01));
	}
	else
	{
		//disable test pattern
		T4K28YUV_write_cmos_sensor(0x335F, (temp_testpattern_reg & 0xFE));
	}
    return ERROR_NONE;
}



UINT32 T4K28YUVFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara, UINT32 *pFeatureParaLen)
{
    //UINT8  *pFeatureData8 =pFeaturePara;
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData = (PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData = (MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData = (MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo = (MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo = (MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo = (MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++ = IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16 = IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_GET_PERIOD:
            *pFeatureReturnPara16++ = T4K28_PV_PERIOD_PIXEL_NUMS + T4K28YUV_dummy_pixels;//T4K28_PV_PERIOD_PIXEL_NUMS+T4K28YUV_dummy_pixels;
            *pFeatureReturnPara16 = T4K28_PV_PERIOD_LINE_NUMS+T4K28YUV_dummy_lines;
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *pFeatureReturnPara32 = 55250000; //19500000;
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            T4K28YUV_SetShutter(*pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_NIGHTMODE:
            T4K28YUV_NightMode((BOOL) *pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_GAIN:
            T4K28YUV_SetGain((UINT16) *pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;

        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            T4K28YUV_isp_master_clock=*pFeatureData32;
            break;

        case SENSOR_FEATURE_SET_REGISTER:
            T4K28YUV_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;

        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = T4K28YUV_read_cmos_sensor(pSensorRegData->RegAddr);
            break;

        case SENSOR_FEATURE_SET_CCT_REGISTER:
			SENSORDB("SENSOR_FEATURE_SET_CCT_REGISTER\n");
            SensorRegNumber = FACTORY_END_ADDR;
            for (i = 0; i < SensorRegNumber; i++)
            {
                T4K28YUVSensorCCT[i].Addr = *pFeatureData32++;
                T4K28YUVSensorCCT[i].Para = *pFeatureData32++;
            }
            break;

        case SENSOR_FEATURE_GET_CCT_REGISTER:
			SENSORDB("SENSOR_FEATURE_GET_CCT_REGISTER\n");
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen < (SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++ = SensorRegNumber;
            for (i = 0; i < SensorRegNumber; i++)
            {
                *pFeatureData32++ = T4K28YUVSensorCCT[i].Addr;
                *pFeatureData32++ = T4K28YUVSensorCCT[i].Para;
            }
            break;

        case SENSOR_FEATURE_SET_ENG_REGISTER:
			SENSORDB("SENSOR_FEATURE_SET_ENG_REGISTER\n");
            SensorRegNumber=ENGINEER_END;
            for (i = 0; i < SensorRegNumber; i++)
            {
                T4K28YUVSensorReg[i].Addr = *pFeatureData32++;
                T4K28YUVSensorReg[i].Para = *pFeatureData32++;
            }
            break;

        case SENSOR_FEATURE_GET_ENG_REGISTER:
			SENSORDB("SENSOR_FEATURE_GET_ENG_REGISTER\n");
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen < (SensorRegNumber*sizeof(SENSOR_REG_STRUCT) + 4))
                return FALSE;
            *pFeatureData32++ = SensorRegNumber;
            for (i = 0; i < SensorRegNumber; i++)
            {
                *pFeatureData32++ = T4K28YUVSensorReg[i].Addr;
                *pFeatureData32++ = T4K28YUVSensorReg[i].Para;
            }
            break;

        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
			SENSORDB("SENSOR_FEATURE_GET_REGISTER_DEFAULT\n");
            if (*pFeatureParaLen >= sizeof(NVRAM_SENSOR_DATA_STRUCT)) {
                pSensorDefaultData->Version = NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId = T4K28_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, T4K28YUVSensorReg, sizeof(SENSOR_REG_STRUCT) * ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, T4K28YUVSensorCCT, sizeof(SENSOR_REG_STRUCT) * FACTORY_END_ADDR);
            } else
                return FALSE;
            *pFeatureParaLen = sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;

        case SENSOR_FEATURE_GET_CONFIG_PARA:
			SENSORDB("SENSOR_FEATURE_GET_CONFIG_PARA\n");
            memcpy(pSensorConfigData, &T4K28YUVSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;

        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
			SENSORDB("SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR\n");
            T4K28YUV_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			SENSORDB("SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA\n");
            T4K28YUV_sensor_to_camera_para();
            break;

        case SENSOR_FEATURE_GET_GROUP_COUNT:
			SENSORDB("SENSOR_FEATURE_GET_GROUP_COUNT\n");
            *pFeatureReturnPara32++ = T4K28YUV_get_sensor_group_count();
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_GET_GROUP_INFO:
			SENSORDB("SENSOR_FEATURE_GET_GROUP_INFO\n");
            T4K28YUV_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ITEM_INFO:
			SENSORDB("SENSOR_FEATURE_GET_ITEM_INFO\n");
            T4K28YUV_get_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
			SENSORDB("SENSOR_FEATURE_SET_ITEM_INFO\n");
            T4K28YUV_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
			SENSORDB("SENSOR_FEATURE_GET_ENG_INFO\n");
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_YUYV;
            *pFeatureParaLen = sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			SENSORDB("SENSOR_FEATURE_GET_LENS_DRIVER_ID\n");
            *pFeatureReturnPara32 = LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
			SENSORDB("SENSOR_FEATURE_INITIALIZE_AF\n");
            SENSORDB("T4K28_FOCUS_Init\n");
            break;

        case SENSOR_FEATURE_CONSTANT_AF:
			SENSORDB("SENSOR_FEATURE_CONSTANT_AF\n");
            SENSORDB("T4K28_FOCUS_Constant_Focus\n");
	    	printk("kiwi-T4K28_FOCUS_Constant_Focus\n");
            break;

        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
			SENSORDB("SENSOR_FEATURE_MOVE_FOCUS_LENS\n");
            SENSORDB("T4K28_FOCUS_AD5820_Move_to %d\n", *pFeatureData16);
            break;

        case SENSOR_FEATURE_GET_AF_STATUS:
			SENSORDB("SENSOR_FEATURE_GET_AF_STATUS\n");
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_GET_AF_INF:
			SENSORDB("SENSOR_FEATURE_GET_AF_INF\n");
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_GET_AF_MACRO:
			SENSORDB("SENSOR_FEATURE_GET_AF_MACRO\n");
            *pFeatureParaLen = 4;
            break;



		case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
			   T4K28YUVMIPI_FlashTriggerCheck(pFeatureData32);
			   printk("[T4K28] F_GET_TRIGGER_FLASHLIGHT_INFO: %d\n", *pFeatureData32);
			   break;  

			

        case SENSOR_FEATURE_SET_VIDEO_MODE:
			SENSORDB("SENSOR_FEATURE_SET_VIDEO_MODE\n");
            T4K28YUVSetVideoMode(*pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_YUV_CMD:
			SENSORDB("SENSOR_FEATURE_SET_YUV_CMD\n");
            T4K28YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
            break;

        case SENSOR_FEATURE_CHECK_SENSOR_ID:
			SENSORDB("SENSOR_FEATURE_CHECK_SENSOR_ID\n");
            T4K28YUVGetSensorID(pFeatureReturnPara32);
            break;

        case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
            SENSORDB("SENSOR_FEATURE_SINGLE_FOCUS_MODE\n");
            break;

        case SENSOR_FEATURE_CANCEL_AF:
            SENSORDB("SENSOR_FEATURE_CANCEL_AF\n");
            break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			 //1 TODO
			 //SENSORDB("[HI251] F_SET_TEST_PATTERN: FAIL: NOT Support\n");
			 T4K28YUVSetTestPatternMode((BOOL)*pFeatureData16);
			 break;

		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing
			 *pFeatureReturnPara32= T4K28YUV_TEST_PATTERN_CHECKSUM;
			 *pFeatureParaLen=4;
			 break;    

		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			SENSORDB("SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS\n");
			*pFeatureReturnPara32 = 0;
			*pFeatureParaLen = 4;
			printk("AF *pFeatureReturnPara32 = %d\n", *pFeatureReturnPara32);
	     	break;

		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			SENSORDB("SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS\n");
			*pFeatureReturnPara32 = 0;
			*pFeatureParaLen = 4;
			printk("AE *pFeatureReturnPara32 = %d\n", *pFeatureReturnPara32);
	        break;

		case SENSOR_FEATURE_GET_EXIF_INFO:
			SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
			SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32); 		 
			T4K28GetExifInfo(*pFeatureData32);
			break;

		case SENSOR_FEATURE_SET_AE_WINDOW:
            SENSORDB("SENSOR_FEATURE_SET_AE_WINDOW\n");
            printk("hwj SENSOR_FEATURE_SET_AE_WINDOW");
            SENSORDB("get zone addr = 0x%x\n", *pFeatureData32);
            break;
        case SENSOR_FEATURE_SET_AF_WINDOW:
            SENSORDB("SENSOR_FEATURE_SET_AF_WINDOW\n");
            printk("hwj SENSOR_FEATURE_SET_AF_WINDOW");
            SENSORDB("get zone addr = 0x%x\n", *pFeatureData32);
            break;
			//xb.pang
		case SENSOR_FEATURE_SET_YUV_3A_CMD:
			T4K28YUV_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
			break;	

        default:
            break;
    }
    return ERROR_NONE;
}

SENSOR_FUNCTION_STRUCT	SensorFuncT4K28YUV=
{
    T4K28YUVOpen,
    T4K28YUVGetInfo,
    T4K28YUVGetResolution,
    T4K28YUVFeatureControl,
    T4K28YUVControl,
    T4K28YUVClose
};

UINT32 T4K28_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &SensorFuncT4K28YUV;
    return ERROR_NONE;
}
