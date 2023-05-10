#ifndef _KD_IMGSENSOR_H
#define _KD_IMGSENSOR_H

#include <linux/ioctl.h>

#ifndef ASSERT
    #define ASSERT(expr)        BUG_ON(!(expr))
#endif

#define IMGSENSORMAGIC 'i'
//IOCTRL(inode * ,file * ,cmd ,arg )
//S means "set through a ptr"
//T means "tell by a arg value"
//G means "get by a ptr"
//Q means "get by return a value"
//X means "switch G and S atomically"
//H means "switch T and Q atomically"

/*******************************************************************************
*
********************************************************************************/
#define YUV_INFO(_id, name, getCalData)\
    { \
    _id, name, \
    NSFeature::YUVSensorInfo<_id>::createInstance(name, #name), \
    (NSFeature::SensorInfoBase*(*)()) \
    NSFeature::YUVSensorInfo<_id>::getInstance, \
    NSFeature::YUVSensorInfo<_id>::getDefaultData, \
    getCalData, \
	NSFeature::YUVSensorInfo<_id>::getNullFlickerPara \
    }
#define RAW_INFO(_id, name, getCalData)\
    { \
    _id, name, \
    NSFeature::RAWSensorInfo<_id>::createInstance(name, #name), \
    (NSFeature::SensorInfoBase*(*)()) \
    NSFeature::RAWSensorInfo<_id>::getInstance, \
    NSFeature::RAWSensorInfo<_id>::getDefaultData, \
    getCalData, \
	NSFeature::RAWSensorInfo<_id>::getFlickerPara \
    }
/*******************************************************************************
*
********************************************************************************/

//sensorOpen
//This command will TBD
#define KDIMGSENSORIOC_T_OPEN            _IO(IMGSENSORMAGIC,0)
//sensorGetInfo
//This command will TBD
#define KDIMGSENSORIOC_X_GETINFO            _IOWR(IMGSENSORMAGIC,5,ACDK_SENSOR_GETINFO_STRUCT)
//sensorGetResolution
//This command will TBD
#define KDIMGSENSORIOC_X_GETRESOLUTION      _IOWR(IMGSENSORMAGIC,10,ACDK_SENSOR_RESOLUTION_INFO_STRUCT)
//sensorFeatureControl
//This command will TBD
#define KDIMGSENSORIOC_X_FEATURECONCTROL    _IOWR(IMGSENSORMAGIC,15,ACDK_SENSOR_FEATURECONTROL_STRUCT)
//sensorControl
//This command will TBD
#define KDIMGSENSORIOC_X_CONTROL            _IOWR(IMGSENSORMAGIC,20,ACDK_SENSOR_CONTROL_STRUCT)
//sensorClose
//This command will TBD
#define KDIMGSENSORIOC_T_CLOSE            _IO(IMGSENSORMAGIC,25)
//sensorSearch 
#define KDIMGSENSORIOC_T_CHECK_IS_ALIVE     _IO(IMGSENSORMAGIC, 30) 
//set sensor driver
#define KDIMGSENSORIOC_X_SET_DRIVER         _IOWR(IMGSENSORMAGIC,35,SENSOR_DRIVER_INDEX_STRUCT)
//get socket postion
#define KDIMGSENSORIOC_X_GET_SOCKET_POS     _IOWR(IMGSENSORMAGIC,40,u32)
//set I2C bus 
#define KDIMGSENSORIOC_X_SET_I2CBUS     _IOWR(IMGSENSORMAGIC,45,u32)
//set I2C bus 
#define KDIMGSENSORIOC_X_RELEASE_I2C_TRIGGER_LOCK     _IO(IMGSENSORMAGIC,50)
//Set Shutter Gain Wait Done
#define KDIMGSENSORIOC_X_SET_SHUTTER_GAIN_WAIT_DONE   _IOWR(IMGSENSORMAGIC,55,u32)//HDR
//set mclk
#define KDIMGSENSORIOC_X_SET_MCLK_PLL         _IOWR(IMGSENSORMAGIC,60,ACDK_SENSOR_MCLK_STRUCT)
#define KDIMGSENSORIOC_X_GETINFO2            _IOWR(IMGSENSORMAGIC,65,IMAGESENSOR_GETINFO_STRUCT)
//set open/close sensor index
#define KDIMGSENSORIOC_X_SET_CURRENT_SENSOR   _IOWR(IMGSENSORMAGIC,70,u32)
/*******************************************************************************
*
********************************************************************************/
/* SENSOR CHIP VERSION */
#define OV2722MIPI_SENSOR_ID                    0x2722
#define OV5645MIPI_SENSOR_ID                    0x5645
 
#define MC501CB_SENSOR_ID                       0x0062
#define MC501CC_SENSOR_ID                       0x0074
#define MC501CA_SENSOR_ID                       0x0011

#define MT9D011_SENSOR_ID                       0x1511
#define MT9D111_SENSOR_ID                       0x1511
#define MT9D112_SENSOR_ID                       0x1580
#define MT9M011_SENSOR_ID                       0x1433
#define MT9M111_SENSOR_ID                       0x143A
#define MT9M112_SENSOR_ID                       0x148C
#define MT9M113_SENSOR_ID                       0x2480
#define MT9P012_SENSOR_ID						0x2800
#define MT9P012_SENSOR_ID_REV7					0x2801
#define MT9T012_SENSOR_ID                       0x1600
#define MT9T013_SENSOR_ID                       0x2600
#define MT9T113_SENSOR_ID                       0x4680
#define MT9V112_SENSOR_ID                       0x1229
#define MT9DX11_SENSOR_ID                       0x1519
#define MT9D113_SENSOR_ID                       0x2580
#define MT9D115_SENSOR_ID                       0x2580
#define MT9D115MIPI_SENSOR_ID                   0x2580

#define NOON200PC11_SENSOR_ID                   0x0013
#define NOON200PC20_SENSOR_ID                   0x0063
#define NOON200PC40_SENSOR_ID                   0x0063
#define NOON200PC51_SENSOR_ID                   0x006C
#define NOON130PC51_SENSOR_ID                   0x0076

#define HM3451_SENSOR_ID						0x3451
#define T8EV5MIPI_SENSOR_ID                     0x1011
#define OV2630_SENSOR_ID                        0x2633
#define OV2640_SENSOR_ID                        0x2642
#define OV2650_SENSOR_ID                        0x2652
#define OV3640_SENSOR_ID                        0x364C
#define OV6680_SENSOR_ID                        0x6681
#define OV7660_SENSOR_ID                        0x7660
#define OV7670_SENSOR_ID                        0x7673
#define OV7680_SENSOR_ID                        0x7680
#define OV9650_SENSOR_ID                        0x9652
#define OV9655_SENSOR_ID                        0x9657
#define OV9660_SENSOR_ID                        0x9663
#define OV3647_SENSOR_ID                        0x364A
#define OV2655_SENSOR_ID					    0x2656
#define OV2659_SENSOR_ID					    0x2656
#define OV2650_SENSOR_ID_1                      0x2651
#define OV2650_SENSOR_ID_2                      0x2652
#define OV2650_SENSOR_ID_3			            0x2655
#define OV5642_SENSOR_ID            0x5642
#define OV5650_SENSOR_ID                        0x5651
#define OV5650MIPI_SENSOR_ID                    0x5651
#define OV9665_SENSOR_ID                        0x9663
#define OV5630_SENSOR_ID                        0x5634
#define OV7675_SENSOR_ID                        0x7673
#define OV5647_SENSOR_ID                        0x5647
#define OV9740MIPI_SENSOR_ID                     0x9740
#define OV2680MIPI_SENSOR_ID					0x2680

#define HI542_SENSOR_ID                         0x00B1
#define HI542MIPI_SENSOR_ID                         0x00B1
#define OV5647MIPI_SENSOR_ID                        0x5647
#define OV8825_SENSOR_ID            			0x8825
#define OV12830_SENSOR_ID			  0xC830
#define OV5648MIPI_SENSOR_ID                    0x5648
#define OV5693_SENSOR_ID                        0x5690

#define PO6030K_SENSOR_ID                       0x0060
#define PO4010K_SENSOR_ID                       0x0040

#define SID020A_SENSOR_ID                       0x12B4
#define SIV100B_SENSOR_ID                       0x0C11
#define SIV100A_SENSOR_ID                       0x0C10
#define SIV120A_SENSOR_ID                       0x1210
#define SIV120B_SENSOR_ID                       0x0012
#define SIV121D_SENSOR_ID			0xDE
#define SIM101B_SENSOR_ID                       0x09A0
#define SIM120C_SENSOR_ID                       0x0012
#define SID130B_SENSOR_ID                       0x001b
#define SIC110A_SENSOR_ID                       0x000D

#define S5KA3DFX_SENSOR_ID                      0x00AB
#define S5K4B2FX_SENSOR_ID                      0x5080
#define S5K3AAEA_SENSOR_ID                      0x07AC
#define S5K3BAFB_SENSOR_ID                      0x7070
#define S5K53BEX_SENSOR_ID                      0x45A8
#define S5K53BEB_SENSOR_ID                      0x87A8
#define S5K83AFX_SENSOR_ID                      0x01C4
#define S5K5BAFX_SENSOR_ID                      0x05BA
#define S5K3E2FX_SENSOR_ID                      0x3E2F
#define S5K4E1GA_SENSOR_ID                      0x4E10
#define S5K4ECGX_SENSOR_ID                      0x4EC0
#define S5K5CAGX_SENSOR_ID                      0x05ca
#define S5K8AAYX_MIPI_SENSOR_ID			0x08aa
#define S5K8AAYX_SENSOR_ID			0x08aa

#define S5K3H7Y_SENSOR_ID               	0x3087
#define S5K4H5YX_2LANE_SENSOR_ID                0x485B

#define PAS105_SENSOR_ID                        0x0065
#define PAS302_SENSOR_ID                        0x0064
#define PAS5101_SENSOR_ID                       0x0067
#define PAS6180_SENSOR_ID                       0x6179

#define ET8EE6_SENSOR_ID                        0x0034
#define ET8EF2_SENSOR_ID                        0x1048

#define OM6802_SENSOR_ID                        0x1705

#define HV7131_SENSOR_ID                        0x0042

#define RJ53S1BA0C_SENSOR_ID                    0x0129

#define HI251_SENSOR_ID                         0x0084
#define HIVICF_SENSOR_ID                        0x0081
#define HI253_SENSOR_ID                         0x0092
#define HI704_SENSOR_ID                         0x0096

#define IMX058_SENSOR_ID                        0x0058
#define IMX073_SENSOR_ID                        0x0046
#define SIV120B_SENSOR_ID                       0x0012
#define MT9V113_SENSOR_ID                         0x2280
#define MT9V114_SENSOR_ID                         0x2283
#define MT9V115_SENSOR_ID						  0x2284
#define MT9P015_SENSOR_ID                       0x2803
#define MT9P017_SENSOR_ID                       0x4800
#define MT9P017MIPI_SENSOR_ID                       0x4800
#define SHARP3D_SENSOR_ID                       0x003d
#define MT9T113MIPI_SENSOR_ID                       0x4680
#define AR0833_SENSOR_ID                        0x4B03
#define IMX105_SENSOR_ID                        0x0105
#define OV8830_SENSOR_ID			  0x8830
#define OV9726_SENSOR_ID                          0x9726
#define OV9726MIPI_SENSOR_ID                          0x9726
#define OV9760MIPI_SENSOR_ID                          0x9760
#define A5141MIPI_SENSOR_ID                     0x4800
#define SP0A19_YUV_SENSOR_ID				0xA6
#define A5142MIPI_SENSOR_ID                     0x4800
#define GC0313MIPI_YUV_SENSOR_ID                0xD0
#define IMX135_SENSOR_ID                        0x0135  
#define OV16825MIPI_SENSOR_ID                       0x0168
#define IMX220_SENSOR_ID                        0x0220
#define GC0310MIPI_SENSOR_ID			0xa310
#define GC0311MIPI_SENSOR_ID			0xa311
#define GC0329_SENSOR_ID			0xC0
#define GC0330_SENSOR_ID			0xC1
#define GC0328_SENSOR_ID                         0x009d
#define GC0309_SENSOR_ID                         0x00a0
#define GC0305_SENSOR_ID                         0x00a5

#define GC0308_SENSOR_ID                     	0x009B
#define GC2035_SENSOR_ID                         0x2035
#define GC2036_SENSOR_ID                         0x2036
#define GC2035MIPI_SENSOR_ID                     0x2035
#define GC2036MIPI_SENSOR_ID                     0x2036
#define GC2145_SENSOR_ID                         0x2145
#define GC2146_SENSOR_ID                         0x2146
#define GC2035_MIPI_SENSOR_ID                    0x2035
#define GC0310_SENSOR_ID                     	0xA310
#define GC0311_SENSOR_ID                     	0xA311
#define GC0313_SENSOR_ID                     	0x00D0
#define GC0313MIPI_SENSOR_ID                    0x00D0
#define SP0A19_SENSOR_ID                         0x00a6
#define SP0718_SENSOR_ID                         0x0071
#define SP2518_SENSOR_ID			 			             0x0053
#define SP2519_SENSOR_ID                         0x0019
#define NT99340_SENSOR_ID			 			0x3400
#define HM5065_YUV_SENSOR_ID			 		0x039E
#define HM5065_MIPI_YUV_SENSOR_ID			 		0x039E
#define SOC5140_SENSOR_ID							0x2880
#define GT2005_SENSOR_ID                        0x5138
#define GC2015_SENSOR_ID                        0x2005
#define GC2155_SENSOR_ID						0x2155
#define GC2155MIPI_SENSOR_ID						0x2155
#define GC2145MIPI_SENSOR_ID						0x2145
#define GC2146MIPI_SENSOR_ID						0x2146
#define HM2057_SENSOR_ID 			0x2056
#define HM2056_MIPI_SENSOR_ID 			0x2056
#define GC2235_SENSOR_ID			0x2235
#define BF3703_SENSOR_ID						0x3703
#define BF3920_SENSOR_ID                        0x3920
#define OV2685_SENSOR_ID                        0x2685
#define C2580_SENSOR_ID                         0x0201
#define GC2145_SENSOR_ID                        0x2145
#define GC0312_SENSOR_ID 						0xB310
#define HI703_SENSOR_ID                         0x73
#define HI706_SENSOR_ID                         0x76
#define HM0357_YUV_SENSOR_ID		0x0357

#define BF3700_SENSOR_ID				    0x3704
#define GC0300_SENSOR_ID            0x00a1
#define HM0300_YUV_SENSOR_ID		    0x0358
#define BF3903_SENSOR_ID						0x3803
#define BF3A03_SENSOR_ID						0x3a03
#define BF3925_SENSOR_ID						0x3925

/* CAMERA DRIVER NAME */
#define CAMERA_HW_DEVNAME            "kd_camera_hw"

/* SENSOR DEVICE DRIVER NAME */
#define SENSOR_DRVNAME_OV5645_MIPI_YUV "ov5645_mipi_yuv"

#define SENSOR_DRVNAME_OV2722_MIPI_RAW  "ov2722mipiraw"
#define SENSOR_DRVNAME_S5K4H5YX_2LANE_MIPI_RAW   "s5k4h5yx2lanemipiraw"
#define SENSOR_DRVNAME_MT9P012_RAW  "mt9p012"
#define SENSOR_DRVNAME_MT9P015_RAW  "mt9p015"
#define SENSOR_DRVNAME_MT9P017_RAW  "mt9p017"
#define SENSOR_DRVNAME_MT9P017_MIPI_RAW  "mt9p017mipi"
#define SENSOR_DRVNAME_MT9D115_MIPI_RAW  "mt9d115mipiraw"
#define SENSOR_DRVNAME_AR0833_MIPI_RAW  "ar0833mipiraw"
#define SENSOR_DRVNAME_OV2650_RAW   "ov265x"
#define SENSOR_DRVNAME_OV2655_YUV   "ov2655yuv"
#define SENSOR_DRVNAME_OV2659_YUV   "ov2659yuv"
#define SENSOR_DRVNAME_OV5650_RAW   	"ov5650raw"
#define SENSOR_DRVNAME_OV3640_RAW   "ov3640"
#define SENSOR_DRVNAME_OV3640_YUV    "ov3640yuv"
#define SENSOR_DRVNAME_OV5642_RAW   "ov5642raw"
#define SENSOR_DRVNAME_HI542_RAW   "hi542raw"
#define SENSOR_DRVNAME_HI542MIPI_RAW   "hi542mipiraw"

#define SENSOR_DRVNAME_OV5647MIPI_RAW   	"ov5647mipiraw"
#define SENSOR_DRVNAME_HM3451_RAW				"hm3451raw"
#define SENSOR_DRVNAME_OV2680_MIPI_RAW   	"ov2680mipiraw"

#define SENSOR_DRVNAME_OV5647_RAW   	"ov5647"
#define SENSOR_DRVNAME_OV5648_MIPI_RAW   	"ov5648mipi"
#define SENSOR_DRVNAME_OV5693_MIPI_RAW     "ov5693mipi"
#define SENSOR_DRVNAME_OV5642_MIPI_YUV    "ov5642mipiyuv"
#define SENSOR_DRVNAME_OV5642_MIPI_RGB    "ov5642mipirgb"
#define SENSOR_DRVNAME_OV5642_MIPI_JPG     "ov5642mipijpg"
#define SENSOR_DRVNAME_OV5642_YUV   "ov5642yuv"
#define SENSOR_DRVNAME_OV5642_YUV_SWI2C   "ov5642yuvswi2c"
#define SENSOR_DRVNAME_OV5650MIPI_RAW   "ov5650mipiraw"
#define SENSOR_DRVNAME_OV7675_YUV   "ov7675yuv"
#define SENSOR_DRVNAME_IMX073_MIPI_RAW   "imx073mipiraw"
#define SENSOR_DRVNAME_S5K5CAGX_YUV     "s5k5cagxyuv"
#define SENSOR_DRVNAME_SIV120B_YUV    "siv120byuv"
#define SENSOR_DRVNAME_MT9V113_YUV    "mt9v113yuv"
#define SENSOR_DRVNAME_HI253_YUV    	"hi253yuv"
#define SENSOR_DRVNAME_SIV121D_YUV    	"siv121dyuv"
#define SENSOR_DRVNAME_HI704_YUV    	"hi704yuv"
#define SENSOR_DRVNAME_MT9V114_YUV    "mt9v114"
#define SENSOR_DRVNAME_MT9V115_YUV	"mt9v115yuv"
#define SENSOR_DRVNAME_MT9T113_YUV    "mt9t113yuv"
#define SENSOR_DRVNAME_PAS6180_SERIAL_YUV    "pas6180serialyuv"
#define SENSOR_DRVNAME_SHARP3D_MIPI_YUV    "sharp3dmipiyuv"
#define SENSOR_DRVNAME_MT9T113_MIPI_YUV    "mt9t113mipiyuv"
#define SENSOR_DRVNAME_IMX105_MIPI_RAW   "imx105mipiraw"
#define SENSOR_DRVNAME_OV8830_RAW   	       "ov8830"
#define SENSOR_DRVNAME_0V9726_RAW		"ov9726raw"
#define SENSOR_DRVNAME_OV9726_MIPI_RAW		"ov9726mipiraw"
#define SENSOR_DRVNAME_OV9760_MIPI_RAW		"ov9760mipiraw"
#define SENSOR_DRVNAME_S5K4E1GA_MIPI_RAW   "s5k4e1gamipiraw"
#define SENSOR_DRVNAME_S5K4ECGX_MIPI_YUV   "s5k4ecgxmipiyuv"
#define SENSOR_DRVNAME_A5141_MIPI_RAW   "a5141mipiraw"
#define SENSOR_DRVNAME_A5142_MIPI_RAW   "a5142mipiraw"
#define SENSOR_DRVNAME_S5K8AAYX_MIPI_YUV     "s5k8aayxmipiyuv"
#define SENSOR_DRVNAME_S5K8AAYX_YUV          "s5k8aayxyuv"

#define SENSOR_DRVNAME_T8EV5_YUV   "t8ev5yuv"
#define SENSOR_DRVNAME_S5K3H7Y_MIPI_RAW   "s5k3h7ymipiraw"
#define SENSOR_DRVNAME_OV8825_MIPI_RAW   "ov8825mipiraw"
#define SENSOR_DRVNAME_OV12830_MIPI_RAW   "ov12830mipiraw"
#define SENSOR_DRVNAME_GC0329_YUV   "gc0329_yuv"
#define SENSOR_DRVNAME_GC0330_YUV   "gc0330_yuv"

#define SENSOR_DRVNAME_OV9740_MIPI_YUV     "ov9740mipiyuv"
#define SENSOR_DRVNAME_S5K3H2YX_MIPI_RAW   "s5k3h2yxmipiraw"
#define SENSOR_DRVNAME_IMX135_MIPI_RAW   "imx135mipiraw"
#define SENSOR_DRVNAME_IMX220_MIPI_RAW   "imx220mipiraw"
#define SENSOR_DRVNAME_GC0313MIPI_YUV      "gc0313mipiyuv"
#define SENSOR_DRVNAME_OV16825_MIPI_RAW   "ov16825mipiraw"
#define SENSOR_DRVNAME_GC0308_YUV    	"gc0308yuv"
#define SENSOR_DRVNAME_GC0328_YUV    	"gc0328yuv"
#define SENSOR_DRVNAME_GC0309_YUV		"gc0309_yuv"
#define SENSOR_DRVNAME_GC0305_YUV		"gc0305yuv"
#define SENSOR_DRVNAME_GC2035_YUV        "gc2035yuv"
#define SENSOR_DRVNAME_GC2036_YUV        "gc2036yuv"
#define SENSOR_DRVNAME_GC2145_YUV        "gc2145yuv"
#define SENSOR_DRVNAME_GC2146_YUV        "gc2146yuv"
#define SENSOR_DRVNAME_GC2035_MIPI_YUV        "gc2035mipiyuv"
#define SENSOR_DRVNAME_GC2036_MIPI_YUV        "gc2036mipiyuv"
#define SENSOR_DRVNAME_GC0310_YUV    	"gc0310yuv"
#define SENSOR_DRVNAME_GC0311_YUV    	"gc0311yuv"
#define SENSOR_DRVNAME_GC0313_YUV    	"gc0313yuv"
#define SENSOR_DRVNAME_GC0313MIPI_YUV   "gc0313mipiyuv"
#define SENSOR_DRVNAME_SP0A19_YUV        "sp0a19yuv"
#define SENSOR_DRVNAME_SP0718_YUV        "sp0718yuv"
#define SENSOR_DRVNAME_SP2518_YUV        "sp2518yuv"
#define SENSOR_DRVNAME_SP2519_YUV        "sp2519yuv"
#define SENSOR_DRVNAME_NT99340_YUV        "nt99340yuv"
#define SENSOR_DRVNAME_HM5065_YUV        "hm5065yuv"
#define SENSOR_DRVNAME_HM5065_MIPI_YUV        "hm5065mipiyuv"
#define SENSOR_DRVNAME_SOC5140_YUV			"soc5140yuv"
#define SENSOR_DRVNAME_GT2005_YUV    	"gt2005yuv"
#define SENSOR_DRVNAME_GC2015_YUV   	"gc2015_yuv"
#define SENSOR_DRVNAME_GC2155_YUV   	"gc2155_yuv"
#define SENSOR_DRVNAME_GC2155MIPI_YUV   	"gc2155mipi_yuv"
#define SENSOR_DRVNAME_GC2145MIPI_YUV   	"gc2145mipi_yuv"
#define SENSOR_DRVNAME_GC2146MIPI_YUV   	"gc2146mipi_yuv"
#define SENSOR_DRVNAME_S5K5EAYX_MIPI_YUV   	"s5k5eayxmipiyuv"
#define SENSOR_DRVNAME_HM2057_YUV   	"hm2057_yuv"
#define SENSOR_DRVNAME_HM2056_MIPI_YUV   	"hm2056mipiyuv"
#define SENSOR_DRVNAME_GC2235_MIPI_YUV   	"gc2235mipiyuv"
#define SENSOR_DRVNAME_BF3703_YUV		"bf3703_yuv"
#define SENSOR_DRVNAME_BF3920_YUV   "bf3920yuv"
#define SENSOR_DRVNAME_OV2685_MIPI_YUV   "ov2685mpipyuv"
#define SENSOR_DRVNAME_C2580_MIPI_YUV           "c2580mipiyuv"
#define SENSOR_DRVNAME_GC2145_MIPI_YUV       "gc2145_mipi_yuv"
#define SENSOR_DRVNAME_GC2146_MIPI_YUV       "gc2146_mipi_yuv"
#define SENSOR_DRVNAME_GC0310_MIPI_YUV       "gc0310_mipi_yuv"
#define SENSOR_DRVNAME_GC0311_MIPI_YUV       "gc0311_mipi_yuv"
#define SENSOR_DRVNAME_GC0312_YUV       "gc0312_yuv"
#define SENSOR_DRVNAME_HI703_YUV    	"hi703_yuv"
#define SENSOR_DRVNAME_HI706_YUV    	"hi706_yuv"
#define SENSOR_DRVNAME_HM0357_YUV    	"hm0357_yuv"

#define SENSOR_DRVNAME_BF3700_YUV		"bf3700_yuv"
#define SENSOR_DRVNAME_GC0300_YUV		"gc0300_yuv"
#define SENSOR_DRVNAME_HM0300_YUV   "hm0300_yuv"
#define SENSOR_DRVNAME_BF3903_YUV		"bf3903yuv"
#define SENSOR_DRVNAME_BF3A03_YUV		"bf3a03yuv"
#define SENSOR_DRVNAME_BF3925_YUV		"bf3925yuv"
/*******************************************************************************
*
********************************************************************************/

void KD_IMGSENSOR_PROFILE_INIT(void); 
void KD_IMGSENSOR_PROFILE(char *tag); 

#define mDELAY(ms)     mdelay(ms) 
#define uDELAY(us)       udelay(us) 
#endif //_KD_IMGSENSOR_H


