#ifndef _MTK_MAU_H_
#define _MTK_MAU_H_

#define MTK_SMI_MAJOR_NUMBER 190

#define MTK_IOW(num, dtype)     _IOW('O', num, dtype)
#define MTK_IOR(num, dtype)     _IOR('O', num, dtype)
#define MTK_IOWR(num, dtype)    _IOWR('O', num, dtype)
#define MTK_IO(num)             _IO('O', num)

// --------------------------------------------------------------------------
#define MTK_CONFIG_MM_MAU       MTK_IOW(10, unsigned long)

typedef struct
{
    int larb;		    //0: the larb you want to monitor, 0 is the only one valid value
    int entry;          //0~2: the mau entry to use
	unsigned int port_msk;  //port mask to be monitored
    int virt;        // 1: monitor va (this port is using m4u);  0: monitor pa (this port is not using m4u)
	int monitor_read;     // monitor read transaction 1-enable, 0-disable
	int monitor_write;    //monitor write transaction 1-enable, 0-disable
	unsigned int start;	    //start address to monitor
	unsigned int end;       //end address to monitor
} MTK_MAU_CONFIG;


int mau_config(MTK_MAU_CONFIG* pMauConf);
void mau_dump_status(const int larb);


//---------------------------------------------------------------------------
typedef enum
{
    SMI_BWC_SCEN_NORMAL,
    SMI_BWC_SCEN_VP1066,
    SMI_BWC_SCEN_VR1066,
    SMI_BWC_SCEN_CNT
} MTK_SMI_BWC_SCEN;


typedef struct
{
    MTK_SMI_BWC_SCEN    scenario;
    int                 b_on_off; //0 : exit this scenario , 1 : enter this scenario
} MTK_SMI_BWC_CONFIG;

// GMP start
typedef enum
{
    SMI_BWC_INFO_CON_PROFILE = 0,
    SMI_BWC_INFO_SENSOR_SIZE,
    SMI_BWC_INFO_VIDEO_RECORD_SIZE,
    SMI_BWC_INFO_DISP_SIZE,
    SMI_BWC_INFO_TV_OUT_SIZE,
    SMI_BWC_INFO_FPS,
    SMI_BWC_INFO_VIDEO_ENCODE_CODEC,
    SMI_BWC_INFO_VIDEO_DECODE_CODEC,
    SMI_BWC_INFO_CNT
} MTK_SMI_BWC_INFO_ID;

typedef struct
{
    int       property;
    long       value1;
    long       value2;
} MTK_SMI_BWC_INFO_SET;


typedef struct
{
    unsigned int flag; // Reserved
    int concurrent_profile;
    long sensor_size[2];
    long video_record_size[2];
    long display_size[2];
    long tv_out_size[2];
    int fps;
    int video_encode_codec;
    int video_decode_codec;
} MTK_SMI_BWC_MM_INFO;

typedef struct
{
    unsigned int flag; // Reserved
    unsigned long return_address;
} MTK_SMI_BWC_INFO_GET;

#define MTK_IOC_SPC_CONFIG          MTK_IOW(20, unsigned long)
#define MTK_IOC_SPC_DUMP_REG        MTK_IOW(21, unsigned long)
#define MTK_IOC_SPC_DUMP_STA        MTK_IOW(22, unsigned long)
#define MTK_IOC_SPC_CMD             MTK_IOW(23, unsigned long)
#define MTK_IOC_SMI_BWC_CONFIG      MTK_IOW(24, MTK_SMI_BWC_CONFIG)
// For BWC.MM property setting
#define MTK_IOC_SMI_BWC_INFO_SET    MTK_IOWR(28, MTK_SMI_BWC_INFO_SET)
// For BWC.MM property get
#define MTK_IOC_SMI_BWC_INFO_GET    MTK_IOWR(29, MTK_SMI_BWC_INFO_GET)



// GMP end


int larb_clock_on(int larb_id, const char *mod_name) ;
int larb_clock_off(int larb_id, const char *mod_name) ;

void dump_smi_register(void);

#endif

