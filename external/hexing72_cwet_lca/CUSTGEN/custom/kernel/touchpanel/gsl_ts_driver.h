#ifndef __GSL_TS_DRIVER_H__
#define __GSL_TS_DRIVER_H__
/*********************************/
#define TPD_HAVE_BUTTON		//������
#define GSL_ALG_ID		//��û��id�㷨
#define GSL_DEBUG			//����
#define TPD_PROC_DEBUG		//adb����
#define GSL_GESTURE         //����

//#define GSL_TIMER				//��ʱ����
//#define TPD_PROXIMITY			//���봫������
#define GSL_DRV_WIRE_IDT_TP	//�����߼���


#define GSL9XX_VDDIO_1800		0
#define TPD_DEBUG_TIME	0x20130424
struct gsl_touch_info
{
	int x[10];
	int y[10];
	int id[10];
	int finger_num;	
};

struct gsl_ts_data {
	struct i2c_client *client;
	struct workqueue_struct *wq;
	struct work_struct work;
	unsigned int irq;
	struct early_suspend pm;
};

/*button*/
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_MENU,KEY_HOMEPAGE,KEY_BACK}
#define TPD_KEYS_DIM            {{80,900,40,20},{240,900,40,20},{400,900,40,20}}


#ifdef GSL_ALG_ID 
extern unsigned int gsl_mask_tiaoping(void);
extern unsigned int gsl_version_id(void);
extern void gsl_alg_id_main(struct gsl_touch_info *cinfo);
extern void gsl_DataInit(int *ret);
#endif
/* Fixme mem Alig */

struct fw_data
{
    u32 offset : 8;
    u32 : 0;
    u32 val;
};

#ifdef GSL_DRV_WIRE_IDT_TP
#include "gsl_idt_zirong_fw.h"
#endif

#include "gsl_ts_fw.h"
static unsigned char gsl_cfg_index = 0;

struct fw_config_type
{
	const struct fw_data *fw;
	unsigned int fw_size;
	unsigned int *data_id;
	unsigned int data_size;
};
static struct fw_config_type gsl_cfg_table[] = {
/*0*/{GSLX680_FW_DC,(sizeof(GSLX680_FW_DC)/sizeof(struct fw_data)),
	gsl_config_data_id_dc,(sizeof(gsl_config_data_id_dc)/4)},
/*1*/{GSLX680_FW_QK,(sizeof(GSLX680_FW_QK)/sizeof(struct fw_data)),
	gsl_config_data_id_qk,(sizeof(gsl_config_data_id_qk)/4)},
/*2*/{GSLX680_FW_MWD,(sizeof(GSLX680_FW_MWD)/sizeof(struct fw_data)),
	gsl_config_data_id_mwd,(sizeof(gsl_config_data_id_mwd)/4)},
};

#endif
