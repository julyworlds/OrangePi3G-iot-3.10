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


#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/time.h>
//#include <linux/hw_module_info.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <mach/dma.h>

#include <linux/gpio.h>
#include <cust_eint.h>
#include <mach/mt_typedefs.h>
#include "tpd.h"
#include "cust_gpio_usage.h"

//#include <linux/suspend.h>
#include "tpd_custom_icn85xx.h"
#include "icn85xx.h"


#if TPD_ICN85XX_COMPILE_FW_WITH_DRIVER
#include "icn85xx_fw.h"
#endif

#define TPD_PROXIMITY   //add 0711

//******************************************************************************//
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/wakelock.h>  
#endif
//******************************************************************************//
 

static char firmware[128] =  {"icn85xx_firmware"};//{"/fw.bin"};
#if TPD_ICN85XX_SUPPORT_FW_UPDATE
short log_basedata[COL_NUM][ROW_NUM] = {{0}};
short log_rawdata[COL_NUM][ROW_NUM] = {{0}};
short log_diffdata[COL_NUM][ROW_NUM] = {{0}};
unsigned int log_on_off = 0;
//static const char firmware[128] =  {"icn85xx_firmware"};//{"/fw.bin"};
#endif

#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#endif


extern struct tpd_device *tpd;                  //Device

static struct i2c_client *i2c_client = NULL;    //Client

static struct task_struct *thread = NULL;       //Thread

#if TPD_ICN85XX_I2C_DMA_SUPPORT
u8 *icn85xx_i2c_dma_va = NULL;                  // dma virtual address     
u32 icn85xx_i2c_dma_pa = NULL;                  // dma physical address
static struct mutex i2c_data_mutex;
#endif


//******************************************************************************//
#ifdef TPD_PROXIMITY
static u8 tpd_proximity_flag = 0; //flag whether start alps
static u8 tpd_proximity_flag_one=0;
static u8 tpd_proximity_detect = 0;//0-->close ; 1--> far away
static struct wake_lock ps_lock;
static u8 icn85xx_psensor_data[8]={0};
#endif
//******************************************************************************//


//******************************************************************************//
#ifdef TPD_SLIDE_WAKEUP

//extern void virkey_arr_register(int *keycode,int size);
//extern void virkey_report(int keycode,int value,bool sync);
static int suspend_status=0;         //Suspend status
static int gesture_id=0;             //Gesture ID
//static int music_flag=0;
static int gesture_status=0;         //Gesture status
static int icn85xx_ges_en = 1;
static int icn85xx_ges_stat = 0;
#define GTP_GESTURE_APP
#ifdef GTP_GESTURE_APP
struct input_dev *gesture_dev;
#endif

struct tp_wakeup_data{
	bool	suspended;
	bool	gestrue_en;
	bool	gestrue_pre_en;
	bool	glove_mode;
	struct mutex lock;
};

static struct tp_wakeup_data twd;    //TP wakeup bool variable

enum{
	GLOVE_MODE_ENABLE,     //Glove mode enable 0
	GLOVE_MODE_DISABLE,    //Glove mode disable 1
};



#if 1
#define GESTURE_UP		     0x11
#define GESTURE_C		     0x12
#define GESTURE_O		     0x13
#define GESTURE_M		     0x14
#define GESTURE_W		     0x15
#define GESTURE_E		     0x16
#define GESTURE_DOWN	     	      0x21
#define GESTURE_LEFT	     	      0x41
#define GESTURE_RIGHT	     0x31
#define GESTURE_S		     0x17
#define GESTURE_V		     0x1e
#define GESTURE_Z		     0x1d
#define GESTURE_DOUBLECLICK	 0x50
#endif

#if 1
#define  KEY_TPGESTURE_LEFT   KEY_LEFT//KEY_LEFT
#define  KEY_TPGESTURE_RIGHT  KEY_RIGHT//KEY_RIGHT   
#define  KEY_TPGESTURE_UP     KEY_UP 			//KEY_UP    //KEY_STOPCD
#define  KEY_TPGESTURE_DOWN   KEY_DOWN 		//KEY_DOWN  //KEY_PLAYPAUSE
#define  KEY_TPGESTURE_C      KEY_C
#define  KEY_TPGESTURE_E      KEY_E
#define  KEY_TPGESTURE_M      KEY_M
#define  KEY_TPGESTURE_O      KEY_O
#define  KEY_TPGESTURE_S      KEY_S
#define  KEY_TPGESTURE_V      KEY_V
#define  KEY_TPGESTURE_W      KEY_W
#define  KEY_TPGESTURE_Z            KEY_S
#define  KEY_TPGESTURE_DOUBLE      KEY_S
#endif

//suspend_state_t get_suspend_state(void);
//-------------------------------------------------'

#endif
//******************************************************************************//

#if SUPPORT_CHECK_ESD
struct hrtimer             icn85xx_esd_timer;         
struct delayed_work        icn85xx_esd_event_work;
struct workqueue_struct   *icn85xx_esd_workqueue;
#endif


static DECLARE_WAIT_QUEUE_HEAD(waiter);    //what this for .......



extern void tpd_button(unsigned int x, unsigned int y, unsigned int down);

static int  tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int  tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int  tpd_remove(struct i2c_client *client);
static int  touch_event_handler(void *unused);
static void tpd_eint_interrupt_handler(void);

static int tpd_flag = 0;
static const struct attribute_group *icn_drv_grp[];
static const struct i2c_device_id icn85xx_tpd_id[] = {{ICN85XX_NAME, 0},{}};
static struct i2c_board_info __initdata icn85xx_i2c_tpd = {I2C_BOARD_INFO(ICN85XX_NAME, ICN85XX_NORMAL_IIC_ADDR)};//0x80
static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name = ICN85XX_NAME,
#if TPD_ICN85XX_SUPPORT_SYSFS
		.groups 	= icn_drv_grp,
#endif
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = icn85xx_tpd_id,
	.detect = tpd_detect,
};

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

static void icn85xx_log(char diff);
static int icn85xx_testTP(char type, int para1, int para2, int para3);
extern void setbootmode(char bmode);


#if TPD_ICN85XX_SUPPORT_SYSFS
static ssize_t icn85xx_store_update(struct device_driver *drv,const char *buf,size_t count);
static ssize_t icn85xx_show_update(struct device_driver *drv,char* buf);

static ssize_t icn85xx_show_process(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t icn85xx_store_process(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);

static DRIVER_ATTR(icn_update, S_IRUGO | S_IWUSR, icn85xx_show_update, icn85xx_store_update);
static DRIVER_ATTR(icn_process, S_IRUGO | S_IWUSR, icn85xx_show_process, icn85xx_store_process);

static ssize_t icn85xx_show_process(struct device* cd,struct device_attribute *attr, char* buf)
{
    ssize_t ret = 0;
    sprintf(buf, "icn85xx process\n");
    ret = strlen(buf) + 1;
    return ret;
}

static ssize_t icn85xx_store_process(struct device* cd, struct device_attribute *attr,
		const char* buf, size_t len)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);
	unsigned long on_off = simple_strtoul(buf, NULL, 10);     

     log_on_off = on_off;
	memset(&log_basedata[0][0], 0, COL_NUM*ROW_NUM*2);
	if(on_off == 0)
	{
		icn85xx_ts->work_mode = 0;
	}
	else if((on_off == 1) || (on_off == 2) || (on_off == 3))
	{
		if((icn85xx_ts->work_mode == 0) && (icn85xx_ts->use_irq == 1))
		{
			hrtimer_init(&icn85xx_ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			icn85xx_ts->timer.function = NULL;//chipone_timer_func;
			hrtimer_start(&icn85xx_ts->timer, ktime_set(CTP_START_TIMER/1000, (CTP_START_TIMER%1000)*1000000), HRTIMER_MODE_REL);
		}
		icn85xx_ts->work_mode = on_off;
	}
	else if(on_off == 10)
	{
		icn85xx_ts->work_mode = 4;
		mdelay(10);
		icn85xx_trace("update baseline\n");
		icn85xx_write_reg(4, 0x30); 
		icn85xx_ts->work_mode = 0;
	}
	else
	{
		icn85xx_ts->work_mode = 0;
	}
	return len;
}

#define PARA_STATE_CMD             0X00
#define PARA_STATE_DATA1           0X01
#define PARA_STATE_DATA2           0X02
#define PARA_STATE_DATA3           0X03

#define UPDATE_CMD_NONE            0x00

static char update_cmd = UPDATE_CMD_NONE;
static int  update_data1 = 0;
static int  update_data2 = 0;
static int  update_data3 = 0;

static ssize_t icn85xx_show_update(struct device_driver *drv, char* buf)
{
    ssize_t ret = 0;       
   
    switch(update_cmd)
    {
        case 'u':
        case 'U':
            sprintf(buf, firmware);
            ret = strlen(buf) + 1;
            icn85xx_trace("firmware: %s\n", firmware);  
   
            break;
        case 't':
        case 'T':
            icn85xx_trace("cmd t,T\n");
            
            break;
        case 'r':
        case 'R':
            icn85xx_trace("cmd r,R\n");
             
            break;
        case 'd':
        case 'D':
            icn85xx_trace("cmd d,D\n");
            
            break;
        case 'c':
        case 'C':
            icn85xx_trace("cmd c,C, %d, %d, %d\n", update_data1, update_data2, update_data3);
            sprintf(buf, "%02x:%08x:%08x\n",update_data1,update_data2,update_data3);
            ret = strlen(buf) + 1;
            break;
        case 'e':
        case 'E':
            icn85xx_trace("cmd e,E, %d, %d, %d\n", update_data1, update_data2, update_data3);
            sprintf(buf, "%02x:%08x:%08x\n",update_data1,update_data2,update_data3);
            ret = strlen(buf) + 1;
            break;
        default:
            icn85xx_trace("this conmand is unknow!!\n");
            break;                

    }

    return ret;
}

/*  update cmd:
*u:/mnt/aaa/bbb/ccc   or U:/mnt/aaa/bbb/ccc, update firmware
*t or T, reset tp
*r or R, printf rawdata
*d or D, printf diffdata
*c:100 or C:100, Consistence test
*e:Vol:Max:Min or E:Vol:Max:Min, diffdata test
*        you can get the diffdata test result, and you should send e/E cmd before this cmd 
*        result formate:   result:Max:Min=%02x:%08x:%08x
*        when cmd e/E before this cmd and Max=Min=0, you can get Max/Min diffdata
*/

/* 
example 1: update firmware
echo u:/mnt/sdcard/download/icn86xx.bin > icn_update
cat icn_update

example 2: Uniformity test
echo c:100 > icn_update
cat icn_update

example 3: open short (dif) test
echo e:3:500:200 > icn_update
cat icn_update

*/

static ssize_t icn85xx_store_update(struct device_driver *drv,const char *buf,size_t count)
{
    icn85xx_trace("count: %d, update: %s\n", count, buf);
	
    int err=0;
    int i=0,j=0;
    char para_state = PARA_STATE_CMD;
    char cmd[16] = {0};
    char data1[128] = {0};
    char data2[16] = {0};
    char data3[16] = {0};
    struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);
    char * p;
    p = cmd;
    for(i=0; i<count; i++)
    {
        if(buf[i] == ':')
        {
            if(PARA_STATE_CMD == para_state)
            {
                p[j] = '\0';
                para_state = PARA_STATE_DATA1;
                p = data1;
                j = 0;
            }
            else if(PARA_STATE_DATA1 == para_state)
            {
                p[j] = '\0';
                para_state = PARA_STATE_DATA2;
                p = data2;
                j = 0;
            }
            else if(PARA_STATE_DATA2 == para_state)
            {
                p[j] = '\0';
                para_state = PARA_STATE_DATA3;
                p = data3;
                j = 0;
            }

        }
        else
        {
            p[j++] =  buf[i];
        }
    }
    p[j] = '\0';
    
    update_cmd = cmd[0];
    switch(update_cmd)
    {
        case 'u':
        case 'U':
            icn85xx_trace("firmware: %s, %d\n", firmware, strlen(firmware));
            for(i=0; i<40; i++)
                icn85xx_trace("0x%2x ", firmware[i]);
            icn85xx_trace("\n");
            memset(firmware, 0, 128);
            memcpy(firmware, data1, strlen(data1)-1);
            icn85xx_trace("firmware: %s, %d\n", firmware, strlen(firmware));            
            icn85xx_trace("fwVersion : 0x%x\n", icn85xx_read_fw_Ver(firmware)); 
            icn85xx_trace("current version: 0x%x\n", icn85xx_readVersion());
            if((icn85xx_ts->ictype == ICN85XX_WITH_FLASH_87) || (icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87))
            {
                if(R_OK == icn87xx_fw_update(firmware))
                {   
                   icn85xx_trace("87 update ok\n");
                }
                else
                {
                   icn85xx_trace("87 update error\n");   
                }

            }
            else
            {
                if(R_OK == icn85xx_fw_update(firmware))
                {   
                    icn85xx_trace("update ok\n");
                }
                else
                {
                    icn85xx_trace("update error\n");   
                }
            }
            break;
        case 't':
        case 'T':
            icn85xx_trace("cmd t,T\n");
            icn85xx_ts_reset();
            break;
        case 'r':
        case 'R':
            icn85xx_trace("cmd r,R\n");
            icn85xx_log(0); 
            break;
        case 'd':
        case 'D':
            icn85xx_trace("cmd d,D\n");
            icn85xx_log(1);
            break;
        case 'c':
        case 'C':            
            update_data1 = simple_strtoul(data1, NULL, 10);
            icn85xx_trace("cmd c,C, %d\n", update_data1);
            update_data1 = icn85xx_testTP(0, update_data1, 0, 0);
            update_data2 = 0;
            update_data3 = 0;
            icn85xx_trace("cmd c,C, result: %d\n", update_data1);
            break;
        case 'e':
        case 'E':
            update_data1 = simple_strtoul(data1, NULL, 10);
            update_data2 = simple_strtoul(data2, NULL, 10);
            update_data3 = simple_strtoul(data3, NULL, 10);
            icn85xx_trace("cmd e,E, %d, %d, %d, %d\n", update_data1, update_data2, update_data3);
            update_data1 = icn85xx_testTP(1, update_data1, update_data2, update_data3);
            break;
        default:
            icn85xx_trace("this conmand is unknow!!\n");
            break;                

    }
    
    return count;
}


static int icn85xx_create_sysfs(struct i2c_client *client)
{
	int err;

	struct device *dev = &(client->dev);
	icn85xx_trace("%s: \n",__func__);

	return err;
}

static void icn85xx_remove_sysfs(struct i2c_client *client)
{
	struct device *dev = &(client->dev);
	icn85xx_trace("%s: \n",__func__);    
}

static struct attribute *icn_drv_attr[] = {
    &driver_attr_icn_update.attr,
    &driver_attr_icn_process.attr,  
    NULL
};

static struct attribute_group icn_drv_attr_grp = {
    .attrs =icn_drv_attr,
};

static const struct attribute_group *icn_drv_grp[] = {
    &icn_drv_attr_grp,
    NULL
};
#endif

#ifdef TPD_SLIDE_WAKEUP

#ifdef GTP_GESTURE_APP
static const u16 gesture_keymap[] = {
    KEY_TPGESTURE_LEFT, 
     KEY_TPGESTURE_RIGHT, 
     KEY_TPGESTURE_UP, 
     KEY_TPGESTURE_DOWN, 
    KEY_TPGESTURE_C, 
    KEY_TPGESTURE_E, 
     KEY_TPGESTURE_M,
    KEY_TPGESTURE_O,
     KEY_TPGESTURE_S, 
     KEY_TPGESTURE_V,
    KEY_TPGESTURE_W
};
static void gtp_gesture_init(void)
{
    s32 ret = 0;
    int i = 0;
    
    gesture_dev = input_allocate_device();
    if (gesture_dev == NULL)
    {
        icn85xx_trace("Failed to allocate input device for gesture.\n");
        return;
    }
    
    gesture_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
    
	for (i = 0; i < ARRAY_SIZE(gesture_keymap); i++)
	    set_bit(gesture_keymap[i], gesture_dev->keybit);

    gesture_dev->name = "TPD_GES";
    gesture_dev->id.bustype = BUS_I2C;
    
    ret = input_register_device(gesture_dev);
    if (ret)
    {
        icn85xx_trace("Register %s input device failed", gesture_dev->name);
        return;
    }
}
void kpd_touchpanel_gesture_handler(int key_code)
{
            input_report_key(gesture_dev, key_code, 1);
            input_sync(gesture_dev);
            input_report_key(gesture_dev, key_code, 0);
            input_sync(gesture_dev);
}
#endif
//*********************************************************************************//
static void icn85xx_gesture_handler(viod)
{
	int ret;
	int ges_key = 0;
	int buf[1]={0};
	
	icn85xx_trace("%s, %d\n", __FUNCTION__, __LINE__);
	icn85xx_trace("++++++TPD_SLIDE_WAKEUP+++++++++++++++\n");
	
	if(suspend_status)
	{	
		ret = icn85xx_i2c_rxdata(0x7000, buf, 1);
		
		if(ret < 0)
		{
			icn85xx_trace("%s read_data i2c_rxdata failed[Read  gesture code]: %d\n", __func__, ret);
			return ret;
		}
		icn85xx_trace("----twd.gestrue_en==true--- 0x7000=0x%x -------\n",buf[0]);

		gesture_id=buf[0];
		
		icn85xx_trace("[icn85xx_gesture_handler]  gesture_id=0x%x\n",gesture_id);
		
		switch(gesture_id)
		{
			case GESTURE_LEFT:
				ges_key=KEY_LEFT;
			 	gesture_status=1;
				break;
			case GESTURE_RIGHT:
				ges_key=KEY_RIGHT;	            
			 	gesture_status=1;
				break;
			case GESTURE_UP:
				ges_key=KEY_UP;	             
				gesture_status=1;
			  	break;
			case GESTURE_DOWN:
				ges_key=KEY_DOWN;	            
				gesture_status=1;
				break;
			case GESTURE_DOUBLECLICK:
				ges_key=KEY_POWER;        
				gesture_status=1;
				break;
			case GESTURE_O:
				ges_key=KEY_O;
				gesture_status=1;
				break;
			case GESTURE_W:
				ges_key=KEY_W;
				gesture_status=1;
				break;
			case GESTURE_M:
				ges_key=KEY_M;
				gesture_status=1;
				break;
			case GESTURE_E:
				ges_key=KEY_E;
				gesture_status=1;
				break;
			case GESTURE_C:
				ges_key=KEY_C;
				gesture_status=1;
				break;
			case GESTURE_S:
				ges_key=KEY_S;
				gesture_status=1;
				break;
			case GESTURE_V:
				ges_key=KEY_V;
				gesture_status=1;
				break;
			case GESTURE_Z:
				ges_key=KEY_Z;
				gesture_status=1;
				break;
			default:
				ges_key=gesture_id;
				gesture_status=0;
				break;
		}
		if(gesture_status)
		{
			#ifdef GTP_GESTURE_APP
			kpd_touchpanel_gesture_handler(ges_key);
			#endif
			gesture_status=0;
		}

	} 
	return ;
}
//*********************************************************************************//
static ssize_t icn85xx_show_ges_enable(struct device *dev,struct device_attribute *attr, char *buf)
{
	ssize_t res;
	res = scnprintf(buf, PAGE_SIZE, "%d\n",icn85xx_ges_en);     
	return res;    
};
static ssize_t icn85xx_store_ges_enable(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

    sscanf(buf, "%d",&icn85xx_ges_en);
	TPD_DMESG("%s,set ges_en = %d", __func__, icn85xx_ges_en);
    return count;
};
static DEVICE_ATTR(ges_enable,  S_IWUGO | S_IRUGO, icn85xx_show_ges_enable,icn85xx_store_ges_enable);

static struct device_attribute * icn85xx_attr_list[] =
{
    &dev_attr_ges_enable,
};
#endif

#ifdef TPD_PROXIMITY
//*********************************************************************************//
static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	u8 buf[1]={0x0};
	int ret;
	if (enable) {
		wake_lock(&ps_lock);
		buf[0] = 0xA0;
		ret = icn85xx_write_reg(0x04,buf[0]);
		tpd_proximity_flag = 1;
		icn85xx_trace("tpd-ps function is on\n");
	}
	else 
	{
		tpd_proximity_flag = 0;
		wake_unlock(&ps_lock);
		buf[0] = 0xA1;
		ret = icn85xx_write_reg(0x04,buf[0]);	
		icn85xx_trace("tpd-ps function is off\n");
	}
	return ret;
}

static int tpd_ps_operate_inc85xx(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
	icn85xx_trace("%s() Line:%d\n",__FUNCTION__,__LINE__);
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				icn85xx_trace("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				icn85xx_trace("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				icn85xx_trace("value=%d\n",value);
				if(value)
				{
					if((tpd_enable_ps(1) < 0))
					{
						icn85xx_trace("enable ps fail: \n");
						return -1;
					}
					//set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if((tpd_enable_ps(0) < 0))
					{
						icn85xx_trace("disable ps fail:\n");
						return -1;
					}
					//clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				icn85xx_trace("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;

				sensor_data->values[0] = tpd_get_ps_value();
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			icn85xx_trace("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;

}

static void icn85xx_proximity_handler(int flag)
{
	
	icn85xx_trace("%s, %d\n", __FUNCTION__, __LINE__);
	icn85xx_trace("++++++TPD_PROXIMITY+++++++++++++++\n");
	
	int err; 
	int proximity=0xFF;
	
	proximity&=flag;
	icn85xx_trace("proximity=%d\n", proximity);
	hwm_sensor_data sensor_data;
	if (tpd_proximity_flag == 1)
	{
		//-------------------------------------------
		
		if (proximity == 1)
		{ 
			icn85xx_trace("Tp is to near\n");
			tpd_proximity_detect = 1;     
		}
		else if (proximity == 0)
		{
		  
			icn85xx_trace("Tp is to far\n");

			tpd_proximity_detect = 0;
		}
			
		icn85xx_trace("tpd_proximity_detect = %d\n", tpd_proximity_detect);
		//map and store data to hwm_sensor_data
		sensor_data.values[0] = tpd_get_ps_value();
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
			icn85xx_trace("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
}
//*****************************************************************************//
#endif


#if SUPPORT_PROC_FS
pack_head cmd_head;
static struct proc_dir_entry *icn85xx_proc_entry;
int  DATA_LENGTH = 0;
GESTURE_DATA structGestureData;
STRUCT_PANEL_PARA_H g_structPanelPara;

static ssize_t icn85xx_tool_write(struct file *file, const char __user * buffer, size_t count, loff_t * ppos)
{
	int ret = 0;
	int i;
	unsigned short addr;
	unsigned int prog_addr;
	char retvalue;
	struct task_struct *thread = NULL;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);
	icn85xx_trace("%s \n",__func__);  
	if(down_interruptible(&icn85xx_ts->sem))  
	{  
		return -1;   
	}     
	ret = copy_from_user(&cmd_head, buffer, CMD_HEAD_LENGTH);
	if(ret)
	{
		icn85xx_trace("copy_from_user failed.\n");
		goto write_out;
	}  
	else
	{
		ret = CMD_HEAD_LENGTH;
	}

	icn85xx_trace("wr  :0x%02x.\n", cmd_head.wr);
	icn85xx_trace("flag:0x%02x.\n", cmd_head.flag);
	icn85xx_trace("circle  :%d.\n", (int)cmd_head.circle);
	icn85xx_trace("times   :%d.\n", (int)cmd_head.times);
	icn85xx_trace("retry   :%d.\n", (int)cmd_head.retry);
	icn85xx_trace("data len:%d.\n", (int)cmd_head.data_len);
	icn85xx_trace("addr len:%d.\n", (int)cmd_head.addr_len);
	icn85xx_trace("addr:0x%02x%02x.\n", cmd_head.addr[0], cmd_head.addr[1]);
	icn85xx_trace("len:%d.\n", (int)count);
	icn85xx_trace("data:0x%02x%02x.\n", buffer[CMD_HEAD_LENGTH], buffer[CMD_HEAD_LENGTH+1]);

	if (1 == cmd_head.wr)  // write para
	{
		icn85xx_trace("cmd_head_.wr == 1  \n");
		ret = copy_from_user(&cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], cmd_head.data_len);
		if(ret)
		{
			icn85xx_trace("copy_from_user failed.\n");
			goto write_out;
		}
		//need copy to g_structPanelPara

		memcpy(&g_structPanelPara, &cmd_head.data[0], cmd_head.data_len);
		//write para to tp
		for(i=0; i<cmd_head.data_len; )
		{
			int size = ((i+64) > cmd_head.data_len)?(cmd_head.data_len-i):64;
			ret = icn85xx_i2c_txdata(0x8000+i, &cmd_head.data[i], size);
			if (ret < 0) {
				icn85xx_trace("write para failed!\n");
				goto write_out;
			}	
			i = i + 64;	    
		}
        
        ret = cmd_head.data_len + CMD_HEAD_LENGTH;
        icn85xx_ts->work_mode = 5; //reinit
        icn85xx_trace("reinit tp\n");
        icn85xx_write_reg(0, 1); 
        mdelay(100);
        icn85xx_write_reg(0, 0);
        mdelay(100);
        icn85xx_ts->work_mode = 0;
        goto write_out;
	}
	else if(3 == cmd_head.wr)   //set update file
	{
		icn85xx_trace("cmd_head_.wr == 3  \n");
		ret = copy_from_user(&cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], cmd_head.data_len);
		if(ret)
		{
			icn85xx_trace("copy_from_user failed.\n");
			goto write_out;
		}
		ret = cmd_head.data_len + CMD_HEAD_LENGTH;
		memset(firmware, 0, 128);
		memcpy(firmware, &cmd_head.data[0], cmd_head.data_len);
		icn85xx_trace("firmware : %s\n", firmware);
	}
	else if(5 == cmd_head.wr)  //start update 
	{        
		icn85xx_trace("cmd_head_.wr == 5 \n");
		icn85xx_update_status(1);  
       icn85xx_ts->is_apk_update = 1;
        if((icn85xx_ts->ictype == ICN85XX_WITH_FLASH_87) || (icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)) 
        {
            ret = kernel_thread(icn87xx_fw_update,firmware,CLONE_KERNEL);
        }
        else
        {
            ret = kernel_thread(icn85xx_fw_update,firmware,CLONE_KERNEL);
        }
		if (IS_ERR(thread))
		{
			ret = PTR_ERR(thread);
			icn85xx_trace("failed to create kernel thread: %d\n", ret);
		}
		icn85xx_trace("the kernel_thread result is:%d\n", ret);    
	}
	else if(11 == cmd_head.wr) //write hostcomm
	{ 
		icn85xx_ts->work_mode = cmd_head.flag; //for gesture test,you should set flag=6
		structGestureData.u8Status = 0;
		ret = copy_from_user(&cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], cmd_head.data_len);
		if(ret)
		{
			icn85xx_trace("copy_from_user failed.\n");
			goto write_out;
		}   	    	
		addr = (cmd_head.addr[1]<<8) | cmd_head.addr[0];
		icn85xx_write_reg(addr, cmd_head.data[0]);        
	}
	else if(13 == cmd_head.wr) //adc enable
	{ 
		icn85xx_trace("cmd_head_.wr == 13  \n");
		icn85xx_ts->work_mode = 4;
		mdelay(10);
		//set col
		icn85xx_write_reg(0x8000+STRUCT_OFFSET(STRUCT_PANEL_PARA_H, u8ColNum), 1); 
		//u8RXOrder[0] = u8RXOrder[cmd_head.addr[0]];
		icn85xx_write_reg(0x8000+STRUCT_OFFSET(STRUCT_PANEL_PARA_H, u8RXOrder[0]), g_structPanelPara.u8RXOrder[cmd_head.addr[0]]); 
		//set row
		icn85xx_write_reg(0x8000+STRUCT_OFFSET(STRUCT_PANEL_PARA_H, u8RowNum), 1);
		//u8TXOrder[0] = u8TXOrder[cmd_head.addr[1]];        
		icn85xx_write_reg(0x8000+STRUCT_OFFSET(STRUCT_PANEL_PARA_H, u8TXOrder[0]), g_structPanelPara.u8TXOrder[cmd_head.addr[1]]); 
		//scan mode
		icn85xx_write_reg(0x8000+STRUCT_OFFSET(STRUCT_PANEL_PARA_H, u8ScanMode), 0);
		//bit
		icn85xx_write_reg(0x8000+STRUCT_OFFSET(STRUCT_PANEL_PARA_H, u16BitFreq), 0xD0);
		icn85xx_write_reg(0x8000+STRUCT_OFFSET(STRUCT_PANEL_PARA_H, u16BitFreq)+1, 0x07);
		//freq
		icn85xx_write_reg(0x8000+STRUCT_OFFSET(STRUCT_PANEL_PARA_H, u16FreqCycleNum[0]), 0x64);
		icn85xx_write_reg(0x8000+STRUCT_OFFSET(STRUCT_PANEL_PARA_H, u16FreqCycleNum[0])+1, 0x00);
		//pga
		icn85xx_write_reg(0x8000+STRUCT_OFFSET(STRUCT_PANEL_PARA_H, u8PgaGain), 0x0);

		//config mode
		icn85xx_write_reg(0, 0x2);

		mdelay(1);
		icn85xx_read_reg(2, &retvalue);
		icn85xx_trace("retvalue0: %d\n", retvalue);
		while(retvalue != 1)
		{   
			icn85xx_trace("retvalue: %d\n", retvalue);
			mdelay(1);
			icn85xx_read_reg(2, &retvalue);
		}   

		if(icn85xx_goto_progmode() != 0)
		{
			icn85xx_trace("icn85xx_goto_progmode() != 0 error\n");
			goto write_out;
		}

		icn85xx_prog_write_reg(0x040870, 1);   

	}
	else if(15 == cmd_head.wr) // write hostcomm multibyte
	{
		icn85xx_trace("cmd_head_.wr == 15  \n");        
		ret = copy_from_user(&cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], cmd_head.data_len);
		if(ret)
		{
			icn85xx_trace("copy_from_user failed.\n");
			goto write_out;
		}
		addr = (cmd_head.addr[1]<<8) | cmd_head.addr[0];
		ret = icn85xx_i2c_txdata(addr, &cmd_head.data[0], cmd_head.data_len);
		if (ret < 0) {
			icn85xx_trace("write hostcomm multibyte failed!\n");
			goto write_out;
		}
	}
	else if(17 == cmd_head.wr)// write iic porgmode multibyte
	{
		icn85xx_trace("cmd_head_.wr == 17  \n");
		ret = copy_from_user(&cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], cmd_head.data_len);
		if(ret)
		{
			icn85xx_trace("copy_from_user failed.\n");
			goto write_out;
		}
		prog_addr = (cmd_head.flag<<16) | (cmd_head.addr[1]<<8) | cmd_head.addr[0];
		icn85xx_goto_progmode();
		ret = icn85xx_prog_i2c_txdata(prog_addr, &cmd_head.data[0], cmd_head.data_len);
		if (ret < 0) {
			icn85xx_trace("write hostcomm multibyte failed!\n");
			goto write_out;
		}

	}
	write_out:
		up(&icn85xx_ts->sem); 
		icn85xx_trace("icn85xx_tool_write write_out  \n");
		return count;

}

static ssize_t icn85xx_tool_read(struct file *file, char __user * buffer, size_t count, loff_t * ppos)
{
	int i, j, packetsize;
	int ret = 0;
	char row, column, retvalue, max_column;
	unsigned short addr;
	unsigned int prog_addr;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);
    char * Ptr;
    
	if(down_interruptible(&icn85xx_ts->sem))  
	{  
		return -1;   
	}     
	icn85xx_trace("%s: count:%d, off:%d, cmd_head.data_len: %d\n",__func__, count,(int)(* ppos),(int)cmd_head.data_len); 
	if (cmd_head.wr % 2)
	{
		ret = 0;
		icn85xx_trace("cmd_head_.wr == 1111111  \n");
		goto read_out;
	}
	else if (0 == cmd_head.wr)   //read para
	{
		//read para
		icn85xx_trace("cmd_head_.wr == 0  \n");

        i = cmd_head.data_len;
        Ptr = &g_structPanelPara;
        addr = 0x8000;

        while(i)
        {
            if(i > 64)
            {
                packetsize = 64;
            }
            else
            {
                packetsize = i;
            }
                
    		ret = icn85xx_i2c_rxdata(addr, Ptr, packetsize);
    		if (ret < 0) {
    			icn85xx_trace("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
    		}
            
            addr += packetsize;
            Ptr += packetsize;
            i -= packetsize;
        }

        ret = copy_to_user(buffer, (void*)(&g_structPanelPara), cmd_head.data_len);
        if(ret)
        {
            icn85xx_trace("copy_to_user failed.\n");
            goto read_out;
        }

		goto read_out;

	}
	else if(2 == cmd_head.wr)  //get update status
	{
		icn85xx_trace("cmd_head_.wr == 2  \n");
		retvalue = icn85xx_get_status();
		icn85xx_trace("status: %d\n", retvalue); 
		ret =copy_to_user(buffer, (void*)(&retvalue), 1);
		if(ret)
		{
			icn85xx_trace("copy_to_user failed.\n");
			goto read_out;
		}
	}
	else if(4 == cmd_head.wr)  //read rawdata
	{     
		icn85xx_trace("cmd_head_.wr == 4  \n");     
		row = cmd_head.addr[1];
		column = cmd_head.addr[0];
		max_column = (cmd_head.flag==0)?(24):(cmd_head.flag);
		//scan tp rawdata
		icn85xx_write_reg(4, 0x20); 
		mdelay(1);  
		for(i=0; i<1000; i++)
		{
			mdelay(1);
			icn85xx_read_reg(2, &retvalue);
			if(retvalue == 1)
				break;
		}

		for(i=0; i<row; i++)
		{ 
			ret = icn85xx_i2c_rxdata(0x2000 + i*(max_column)*2,(char *) &log_rawdata[i][0], column*2);
			if (ret < 0) {
				icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
			} 
			//icn85xx_rawdatadump(&log_rawdata[i][0], column, COL_NUM); 
			ret = copy_to_user(&buffer[column*2*i], (void*)(&log_rawdata[i][0]), column*2);
			if(ret)
			{
				proc_error("copy_to_user failed.\n");
				goto read_out;
			}    
		} 

		//finish scan tp rawdata
		icn85xx_write_reg(2, 0x0);    
		icn85xx_write_reg(4, 0x21); 
		goto read_out;        
	}
	else if(6 == cmd_head.wr)  //read diffdata
	{   
		proc_info("cmd_head_.wr == 6   \n");
		row = cmd_head.addr[1];
		column = cmd_head.addr[0];
		max_column = (cmd_head.flag==0)?(24):(cmd_head.flag);        
		//scan tp rawdata
		icn85xx_write_reg(4, 0x20); 
		mdelay(1);

		for(i=0; i<1000; i++)
		{
			mdelay(1);
			icn85xx_read_reg(2, &retvalue);
			if(retvalue == 1)
				break;
		}      

		for(i=0; i<row; i++)
		{ 
			ret = icn85xx_i2c_rxdata(0x3000 + (i+1)*(max_column+2)*2 + 2,(char *) &log_diffdata[i][0], column*2);
			if (ret < 0) {
				icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
			} 
			//icn85xx_rawdatadump(&log_diffdata[i][0], column, COL_NUM); 
			ret = copy_to_user(&buffer[column*2*i], (void*)(&log_diffdata[i][0]), column*2);
			if(ret)
			{
				proc_error("copy_to_user failed.\n");
				goto read_out;
			}
		}      
		//finish scan tp rawdata
		icn85xx_write_reg(2, 0x0);         
		icn85xx_write_reg(4, 0x21); 

		goto read_out;          
	}
	else if(8 == cmd_head.wr)  //change TxVol, read diff
	{       
		proc_info("cmd_head_.wr == 8  \n");
		row = cmd_head.addr[1];
		column = cmd_head.addr[0];
		max_column = (cmd_head.flag==0)?(24):(cmd_head.flag); 
		//scan tp rawdata
		icn85xx_write_reg(4, 0x20); 
		mdelay(1);
		for(i=0; i<1000; i++)
		{
			mdelay(1);
			icn85xx_read_reg(2, &retvalue);
			if(retvalue == 1)
				break;
		}     

		for(i=0; i<row; i++)
		{ 
			ret = icn85xx_i2c_rxdata(0x2000 + i*(max_column)*2,(char *) &log_rawdata[i][0], column*2);
			if (ret < 0) {
				icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
			} 

		} 

		//finish scan tp rawdata
		icn85xx_write_reg(2, 0x0);    
		icn85xx_write_reg(4, 0x21); 

		icn85xx_write_reg(4, 0x12);

		//scan tp rawdata
		icn85xx_write_reg(4, 0x20); 
		mdelay(1);
		for(i=0; i<1000; i++)
		{
			mdelay(1);
			icn85xx_read_reg(2, &retvalue);
			if(retvalue == 1)
				break;
		}    

		for(i=0; i<row; i++)
		{ 
			ret = icn85xx_i2c_rxdata(0x2000 + i*(max_column)*2,(char *) &log_diffdata[i][0], column*2);
			if (ret < 0) {
				icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
			} 
			for(j=0; j<column; j++)
			{
				log_basedata[i][j] = log_rawdata[i][j] - log_diffdata[i][j];
			}
			ret = copy_to_user(&buffer[column*2*i], (void*)(&log_basedata[i][0]), column*2);
			if(ret)
			{
				proc_error("copy_to_user failed.\n");
				goto read_out;
			}
		} 

		//finish scan tp rawdata
		icn85xx_write_reg(2, 0x0);    
		icn85xx_write_reg(4, 0x21); 

		icn85xx_write_reg(4, 0x10);

		goto read_out;          
	}
	else if(10 == cmd_head.wr)  //read adc data
	{
		proc_info("cmd_head_.wr == 10  \n");
		if(cmd_head.flag == 0)
		{
			icn85xx_prog_write_reg(0x040874, 0);  
		}        
		icn85xx_prog_read_page(2500*cmd_head.flag,(char *) &log_diffdata[0][0],cmd_head.data_len);
		ret = copy_to_user(buffer, (void*)(&log_diffdata[0][0]), cmd_head.data_len);
		if(ret)
		{
			proc_error("copy_to_user failed.\n");
			goto read_out;
		}
		if(cmd_head.flag == 9)
		{
			//reload code
			if((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85) || (icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_86))
			{
				if(R_OK == icn85xx_fw_update(firmware))
				{
					icn85xx_ts->code_loaded_flag = 1;
					icn85xx_trace("ICN85XX_WITHOUT_FLASH, reload code ok\n"); 
				}
				else
				{
					icn85xx_ts->code_loaded_flag = 0;
					icn85xx_trace("ICN85XX_WITHOUT_FLASH, reload code error\n"); 
				}
			}
			else
			{
				icn85xx_bootfrom_flash(icn85xx_ts->ictype);                
				msleep(150);
			}
			icn85xx_ts->work_mode = 0;
		}
	}
	else if(12 == cmd_head.wr) //read hostcomm
	{
		proc_info("cmd_head_.wr == 12  \n");
		addr = (cmd_head.addr[1]<<8) | cmd_head.addr[0];
		icn85xx_read_reg(addr, &retvalue);
		ret = copy_to_user(buffer, (void*)(&retvalue), 1);
		if(ret)
		{
			proc_error("copy_to_user failed.\n");
			goto read_out;
		}
	}
	else if(14 == cmd_head.wr) //read adc status
	{
		proc_info("cmd_head_.wr == 14  \n");
		icn85xx_prog_read_reg(0x4085E, &retvalue);
		ret = copy_to_user(buffer, (void*)(&retvalue), 1);
		if(ret)
		{
			proc_error("copy_to_user failed.\n");
			goto read_out;
		}
		printk("0x4085E: 0x%x\n", retvalue);
	}  
	else if(16 == cmd_head.wr)  //read gesture data
	{
		proc_info("cmd_head_.wr == 16  \n");
		ret = copy_to_user(buffer, (void*)(&structGestureData), sizeof(structGestureData));
		if(ret)
		{
			proc_error("copy_to_user failed.\n");
			goto read_out;
		}
		if(structGestureData.u8Status == 1)
			structGestureData.u8Status = 0;
	}
	else if(18 == cmd_head.wr) // read hostcomm multibyte
	{
		proc_info("cmd_head_.wr == 18  \n");       
		addr = (cmd_head.addr[1]<<8) | cmd_head.addr[0];
		ret = icn85xx_i2c_rxdata(addr, &cmd_head.data[0], cmd_head.data_len);
		if(ret < 0)
		{
			proc_error("copy_to_user failed.\n");
			goto read_out;
		}
		ret = copy_to_user(buffer, &cmd_head.data[0], cmd_head.data_len);
		if (ret) {
			icn85xx_error("read hostcomm multibyte failed: %d\n", ret);
		}       
		goto read_out;
	}
	else if(20 == cmd_head.wr)// read iic porgmode multibyte
	{
		proc_info("cmd_head_.wr == 20  \n");        
		prog_addr = (cmd_head.flag<<16) | (cmd_head.addr[1]<<8) | cmd_head.addr[0];
		icn85xx_goto_progmode();
		ret = icn85xx_prog_i2c_rxdata(prog_addr, &cmd_head.data[0], cmd_head.data_len);
		if (ret < 0) {
			icn85xx_error("read iic porgmode multibyte failed: %d\n", ret);
		}   
		ret = copy_to_user(buffer, &cmd_head.data[0], cmd_head.data_len);
		if(ret)
		{
			proc_error("copy_to_user failed.\n");
			goto read_out;
		}

		icn85xx_bootfrom_sram();
		goto read_out;
	} 
    else if(22 == cmd_head.wr) //read ictype
    {
        proc_info("cmd_head_.wr == 22  \n");
        ret = copy_to_user(buffer, (void*)(&icn85xx_ts->ictype), 1);
        if(ret)
        {
            proc_error("copy_to_user failed.\n");
            goto read_out;
        }
    }
read_out:
	up(&icn85xx_ts->sem);   
	proc_info("%s out: %d, cmd_head.data_len: %d\n\n",__func__, count, cmd_head.data_len); 
	return cmd_head.data_len;
}
static const struct file_operations icn85xx_proc_fops = {
	.owner		= THIS_MODULE,
	.read  = icn85xx_tool_read,
	.write  = icn85xx_tool_write,
};
void init_proc_node(void)
{
	int i;
	memset(&cmd_head, 0, sizeof(cmd_head));
	cmd_head.data = NULL;

	i = 5;
	while ((!cmd_head.data) && i)
	{
		cmd_head.data = kzalloc(i * DATA_LENGTH_UINT, GFP_KERNEL);
		if (NULL != cmd_head.data)
		{
			break;
		}
		i--;
	}
	if (i)
	{
		//DATA_LENGTH = i * DATA_LENGTH_UINT + GTP_ADDR_LENGTH;
		DATA_LENGTH = i * DATA_LENGTH_UINT;
		icn85xx_trace("alloc memory size:%d.\n", DATA_LENGTH);
	}
	else
	{
		proc_error("alloc for memory failed.\n");
		return ;
	}

	icn85xx_proc_entry = proc_create(ICN85XX_ENTRY_NAME, 0666, NULL, &icn85xx_proc_fops);   
	if (icn85xx_proc_entry == NULL)
	{
		proc_error("Couldn't create proc entry!\n");
		return ;
	}
	else
	{
		icn85xx_trace("Create proc entry success!\n");
	}

	return ;
}

void uninit_proc_node(void)
{
	kfree(cmd_head.data);
	cmd_head.data = NULL;
	remove_proc_entry(ICN85XX_ENTRY_NAME, NULL);
}

#endif

static void tpd_down(s32 x, s32 y, s32 size, s32 id)
{

	icn85xx_info("%s: button size=%d , id = %d\n",__func__, size,id);

	if ((!size) && (!id))
	{
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
	}
	else
	{
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, size);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
		/* track id Start 0 */
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	}

	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
	//TPD_EM_PRINT(x, y, x, y, id, 1);
	input_sync(tpd->dev);

}

static void tpd_up(s32 x, s32 y, s32 id)
{
	input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0);
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	input_mt_sync(tpd->dev);
//	TPD_EM_PRINT(x, y, x, y, id, 0);
	input_sync(tpd->dev);

}

#if TPD_ICN85XX_I2C_DMA_SUPPORT
int icn85xx_prog_i2c_rxdata(unsigned int addr, char *rxdata, int length)
{
	int ret = -1;
	int retries = 0;
	//unsigned char tmp_buf[2];
#if TPD_ICN85XX_I2C_DMA_SUPPORT	//Is this redifine
	mutex_lock(&i2c_data_mutex);
#endif
	{
		struct i2c_msg msgs[] = {
			{
				.addr = TPD_ICN85XX_PROG_I2C_ADDR&I2C_MASK_FLAG | I2C_DMA_FLAG,
				.flags = 0,
				.len = 3,
				.buf = icn85xx_i2c_dma_pa,
			},
			{
				.addr = TPD_ICN85XX_PROG_I2C_ADDR&I2C_MASK_FLAG | I2C_DMA_FLAG,
				.flags = I2C_M_RD,
				.len = length,
				.buf = icn85xx_i2c_dma_pa,
			},
		};
		//	icn85xx_i2c_dma_va[0] = U16HIBYTE(addr);
		//	icn85xx_i2c_dma_va[1] = U16LOBYTE(addr);
		icn85xx_i2c_dma_va[0]  = (unsigned char)(addr>>16);
		icn85xx_i2c_dma_va[1] = (unsigned char)(addr>>8);
		icn85xx_i2c_dma_va[2]  = (unsigned char)(addr);

		while(retries < I2C_RETRY_NUM)
		{
			ret = i2c_transfer(i2c_client->adapter, msgs, 2);

			if(ret == 2)
				break;
			retries++;
		}
	}

#if TPD_ICN85XX_I2C_DMA_SUPPORT 	
	mutex_unlock(&i2c_data_mutex);
#endif

	if(retries >= I2C_RETRY_NUM)
	{
		icn85xx_error("%s i2c read error: %d, rxdata_len = %d\n", __func__, ret, length);
	}
	else
	{
		int i = 0;
		for (i = 0; i<length; i++)
		{
			rxdata[i] = icn85xx_i2c_dma_va[i];
		}
	}

	return ret;
}
#else
int icn85xx_prog_i2c_rxdata(unsigned int addr, char *rxdata, int length)
{
	int ret = -1;
	int retries = 0;
	unsigned char tmp_buf[3];
	struct i2c_msg msgs[] = {
		{
			.addr = TPD_ICN85XX_PROG_I2C_ADDR,
			.flags = 0,
			.len = 3,
			.buf = tmp_buf,
		},
		{
			.addr = TPD_ICN85XX_PROG_I2C_ADDR,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxdata,
		},
	};
	tmp_buf[0] = (unsigned char)(addr>>16);
	tmp_buf[1] = (unsigned char)(addr>>8);
	tmp_buf[2] = (unsigned char)(addr);

	while(retries < I2C_RETRY_NUM)
	{

#if TPD_ICN85XX_I2C_DMA_SUPPORT 	
		mutex_lock(&i2c_data_mutex);
#endif
		ret = i2c_transfer(i2c_client->adapter, msgs, 2);

#if TPD_ICN85XX_I2C_DMA_SUPPORT 	
		mutex_unlock(&i2c_data_mutex);
#endif		
		if(ret == 2)
			break;
		retries++;
	}

	if(retries >= I2C_RETRY_NUM)
	{
		icn85xx_error("%s i2c read error: %d, rxdata_len = %d\n", __func__, ret, length);
	}

	return ret;
}
#endif


#if TPD_ICN85XX_I2C_DMA_SUPPORT 
int icn85xx_prog_i2c_txdata(unsigned int addr, char *txdata, int length)
{
	int i2c_addr_old = 0;
	int ret = -1;
	char tmp_buf[128];
	int i = 0;

	if (!i2c_client||!icn85xx_i2c_dma_va)
	{
		icn85xx_error("%s, i2c_client or icn85xx_i2c_dma_va is null pointer\n");
		return -1;
	}

	if(length > 125)
	{
		icn85xx_error("%s too big datalen = %d!\n", __func__, length);
		return -1;
	}

	tmp_buf[0] = (unsigned char)(addr>>16);
	tmp_buf[1] = (unsigned char)(addr>>8);
	tmp_buf[2] = (unsigned char)(addr);

	if(length != 0 && txdata != NULL)
	{
		memcpy(&tmp_buf[3], txdata, length);
	}

	for(i = 0; i < length+3; i++)
	{
		icn85xx_i2c_dma_va[i] = tmp_buf[i];
	}

	mutex_lock(&i2c_data_mutex);

	i2c_addr_old = i2c_client->addr;

	ret = i2c_client->addr = TPD_ICN85XX_PROG_I2C_ADDR & I2C_MASK_FLAG | I2C_DMA_FLAG;

	i2c_master_send(i2c_client, icn85xx_i2c_dma_pa, length+3);

	i2c_client->addr = i2c_addr_old;

	mutex_unlock(&i2c_data_mutex);

	if(ret <= 0)
	{
		icn85xx_error("%s i2c write error: %d\n", __func__, ret);
	}

	return ret;
}
#else
int icn85xx_prog_i2c_txdata(unsigned int addr, char *txdata, int length)
{
	int ret = -1;
	char tmp_buf[128];
	int retries = 0;
	struct i2c_msg msg[] = {
		{
			.addr = TPD_ICN85XX_PROG_I2C_ADDR,
			.flags = 0,
			.len = length + 3,
			.buf = tmp_buf,
		},
	};

	if(length > 125)
	{
		icn85xx_error("%s too big datalen = %d!\n", __func__, length);
		return -1;
	}

	tmp_buf[0] = (unsigned char)(addr>>16);
	tmp_buf[1] = (unsigned char)(addr>>8);
	tmp_buf[2] = (unsigned char)(addr);

	if(length != 0 && txdata != NULL)
		memcpy(&tmp_buf[3], txdata, length);

	while(retries < I2C_RETRY_NUM)
	{
		ret = i2c_transfer(i2c_client->adapter, msg, 1);
		if(ret > 0)
			break;
		retries++;
	}

	if(retries >= I2C_RETRY_NUM)
	{
		icn85xx_error("%s i2c write error: %d\n", __func__, ret);
	}

	return ret;
}
#endif

#if TPD_ICN85XX_I2C_DMA_SUPPORT
	int icn87xx_prog_i2c_rxdata(unsigned int addr, char *rxdata, int length)
	{
		int ret = -1;
		int retries = 0;
		//unsigned char tmp_buf[2];
#if TPD_ICN85XX_I2C_DMA_SUPPORT	
		mutex_lock(&i2c_data_mutex);
#endif
		{
			struct i2c_msg msgs[] = {
				{
					.addr = ICN87XX_PROG_IIC_ADDR&I2C_MASK_FLAG | I2C_DMA_FLAG,
					.flags = 0,
					.len = 2,
					.buf = icn85xx_i2c_dma_pa,
				},
				{
					.addr = ICN87XX_PROG_IIC_ADDR&I2C_MASK_FLAG | I2C_DMA_FLAG,
					.flags = I2C_M_RD,
					.len = length,
					.buf = icn85xx_i2c_dma_pa,
				},
			};
		//	icn85xx_i2c_dma_va[0] = U16HIBYTE(addr);
		//	icn85xx_i2c_dma_va[1] = U16LOBYTE(addr);
		  //icn85xx_i2c_dma_va[0]  = (unsigned char)(addr>>16);
		  icn85xx_i2c_dma_va[0] = (unsigned char)(addr>>8);
		  icn85xx_i2c_dma_va[1]  = (unsigned char)(addr);
	
			while(retries < I2C_RETRY_NUM)
			{
				ret = i2c_transfer(i2c_client->adapter, msgs, 2);
			
				if(ret == 2)
					break;
				retries++;
			}
		}
		
#if TPD_ICN85XX_I2C_DMA_SUPPORT 	
		mutex_unlock(&i2c_data_mutex);
#endif
	
		if(retries >= I2C_RETRY_NUM)
		{
			icn85xx_error("%s i2c read error: %d, rxdata_len = %d\n", __func__, ret, length);
		}
		else
		{
			int i = 0;
			for (i = 0; i<length; i++)
			{
				rxdata[i] = icn85xx_i2c_dma_va[i];
			}
		}
	
		return ret;
	}
#else
	#error
#endif

#if TPD_ICN85XX_I2C_DMA_SUPPORT 
int icn87xx_prog_i2c_txdata(unsigned int addr, char *txdata, int length)
{
	int i2c_addr_old = 0;
	int ret = -1;
	char tmp_buf[128];
	int i = 0;

	if (!i2c_client||!icn85xx_i2c_dma_va)
	{
		icn85xx_error("%s, i2c_client or icn85xx_i2c_dma_va is null pointer\n");
		return -1;
	}
	
	if(length > 125)
	{
		icn85xx_error("%s too big datalen = %d!\n", __func__, length);
		return -1;
	}

	 //tmp_buf[0] = (unsigned char)(addr>>16);
   tmp_buf[0] = (unsigned char)(addr>>8);
   tmp_buf[1] = (unsigned char)(addr);

	if(length != 0 && txdata != NULL)
	{
		memcpy(&tmp_buf[2], txdata, length);
	}

	for(i = 0; i < length+2; i++)
	{
		icn85xx_i2c_dma_va[i] = tmp_buf[i];
	}
	
	mutex_lock(&i2c_data_mutex);

	i2c_addr_old = i2c_client->addr;

	ret = i2c_client->addr = ICN87XX_PROG_IIC_ADDR & I2C_MASK_FLAG | I2C_DMA_FLAG;
	
	i2c_master_send(i2c_client, icn85xx_i2c_dma_pa, length+2);

	i2c_client->addr = i2c_addr_old;
	
	mutex_unlock(&i2c_data_mutex);

	if(ret <= 0)
	{
		icn85xx_error("%s i2c write error: %d\n", __func__, ret);
	}

	return ret;
}
#else
	#error
#endif

int icn85xx_prog_write_reg(unsigned int addr, char para)
{
	char buf[3];
	int ret = -1;

	buf [0] = para;
	ret = icn85xx_prog_i2c_txdata(addr, buf, 1);
	if(ret < 0) {
		icn85xx_error("write reg failed! %#x ret: %d\n", buf[0], ret);
		return -1;
	}

	return ret;
}

int icn85xx_prog_read_reg(unsigned int addr, char *pdata)
{
	int ret = -1;

	ret = icn85xx_prog_i2c_rxdata(addr, pdata, 1);
	return ret;
}

int icn85xx_prog_read_page(unsigned int Addr,unsigned char *Buffer, unsigned int Length)
{
	int ret =0;
	unsigned int StartAddr = Addr;
	while(Length){
		if(Length > MAX_LENGTH_PER_TRANSFER){
			ret = icn85xx_prog_i2c_rxdata(StartAddr, Buffer, MAX_LENGTH_PER_TRANSFER);
			Length -= MAX_LENGTH_PER_TRANSFER;
			Buffer += MAX_LENGTH_PER_TRANSFER;
			StartAddr += MAX_LENGTH_PER_TRANSFER; 
		}
		else{
			ret = icn85xx_prog_i2c_rxdata(StartAddr, Buffer, Length);
			Length = 0;
			Buffer += Length;
			StartAddr += Length;
			break;
		}
		icn85xx_error("\n icn85xx_prog_read_page StartAddr:0x%x, length: %d\n",StartAddr,Length);
	}
	if (ret < 0) {
		icn85xx_error("\n icn85xx_prog_read_page failed! StartAddr:  0x%x, ret: %d\n", StartAddr, ret);
		return ret;
	}
	else{ 
		printk("\n icn85xx_prog_read_page, StartAddr 0x%x, Length: %d\n", StartAddr, Length);
		return ret;
	}  
}
int icn85xx_i2c_rxdata(unsigned short addr, char *rxdata, int length)
{
	int ret = -1;
	int retries = 0;
	unsigned char tmp_buf[2];


	if (length<6)
	{
#if TPD_ICN85XX_I2C_DMA_SUPPORT 	
		mutex_lock(&i2c_data_mutex);
#endif
		{
			struct i2c_msg msgs[] =
			{
				{
					.addr = i2c_client->addr,
					.flags = 0,
					.len = 2,
					.buf = tmp_buf,
				},
				{
					.addr = i2c_client->addr,
					.flags = I2C_M_RD,
					.len = length,
					.buf = rxdata,
				},
			};

			//tmp_buf[0] = addr;
			tmp_buf[0] = (unsigned char)(addr>>8);
			tmp_buf[1] = (unsigned char)(addr);

			while(retries < I2C_RETRY_NUM)
			{

				ret = i2c_transfer(i2c_client->adapter, msgs, 2);

				if(ret == 2)
					break;

				retries++;
			}
		}
#if TPD_ICN85XX_I2C_DMA_SUPPORT 	
		mutex_unlock(&i2c_data_mutex);
#endif
		if(retries >= I2C_RETRY_NUM)
		{
			icn85xx_error("%s i2c read error: %d, rxdata_len = %d\n", __func__, ret, length);
		}	
	}
	else
	{
#if TPD_ICN85XX_I2C_DMA_SUPPORT 	
		mutex_lock(&i2c_data_mutex);
#endif
		{
			struct i2c_msg msgs[] =
			{
				{
					.addr = i2c_client->addr&I2C_MASK_FLAG | I2C_DMA_FLAG,
					.flags = 0,
					.len = 2,
					.buf = icn85xx_i2c_dma_pa,
				},
				{
					.addr = i2c_client->addr&I2C_MASK_FLAG | I2C_DMA_FLAG,
					.flags = I2C_M_RD,
					.len = length,
					.buf = (char*)icn85xx_i2c_dma_pa,
				},
			};
			icn85xx_i2c_dma_va[0] = (unsigned char)(addr>>8);
			icn85xx_i2c_dma_va[1] = (unsigned char)(addr);

			//	icn85xx_i2c_dma_va[0] = addr;

			while(retries < I2C_RETRY_NUM)
			{
				ret = i2c_transfer(i2c_client->adapter, msgs, 2);
				if(ret == 2)
					break;
				retries++;
			}
		}
#if TPD_ICN85XX_I2C_DMA_SUPPORT 	
		mutex_unlock(&i2c_data_mutex);
#endif
		if(retries >= I2C_RETRY_NUM)
		{
			icn85xx_error("%s i2c read error: %d, rxdata_len = %d\n", __func__, ret, length);
		}
		else
		{
			int i = 0;
			for (i = 0; i<length; i++)
			{
				rxdata[i] = icn85xx_i2c_dma_va[i];
			}
		}

	}

	return ret;
}

int icn85xx_i2c_txdata(unsigned short addr, char *txdata, int length)
{
	int ret = -1;
	unsigned char tmp_buf[128];
	int retries = 0;

	struct i2c_msg msg[] = {
		{
			.addr = i2c_client->addr,
			.flags = 0,
			.len = length + 2,
			.buf = tmp_buf,
		},
	};

	if(length > 125)
	{
		icn85xx_error("%s too big datalen = %d!\n", __func__, length);
		return -1;
	}

	tmp_buf[0] = (unsigned char)(addr>>8);
	tmp_buf[1] = (unsigned char)(addr);

	if(length != 0 && txdata != NULL)
	{
		memcpy(&tmp_buf[2], txdata, length);
	}   

	while(retries < I2C_RETRY_NUM)
	{
		ret = i2c_transfer(i2c_client->adapter, msg, 1);
		if(ret == 1)
			break;
		retries++;
	}

	if(retries >= I2C_RETRY_NUM)
	{
		icn85xx_error("%s i2c write error: %d\n", __func__, ret);
	}

	return ret;
}

/***********************************************************************************************
Name    :   icn85xx_write_reg
Input   :   addr -- address
para -- parameter
Output  :   
function    :   write register of icn85xx, normal mode
 ***********************************************************************************************/
int icn85xx_write_reg(unsigned short addr, char para)
{
	char buf[3];
	int ret = -1;

	buf[0] = para;
	ret = icn85xx_i2c_txdata(addr, buf, 1);
	if (ret < 0) {
		icn85xx_error("write reg failed! %#x ret: %d\n", buf[0], ret);
		return -1;
	}

	return ret;
}

int icn85xx_read_reg(unsigned short addr, char *pdata)
{
	int ret = -1;

	ret = icn85xx_i2c_rxdata(addr, pdata, 1);
	if(ret < 0)
	{
		icn85xx_error("addr: 0x%x: 0x%x\n", addr, *pdata); 
	}
	return ret;
}


/***********************************************************************************************
Name    :   icn85xx_log
Input   :   0: rawdata, 1: diff data
Output  :   err type
function    :   calibrate param
***********************************************************************************************/
static void icn85xx_log(char diff)
{
    char row = 0;
    char column = 0;

    char rowMax = 0;
    char columnMax = 0;
    
    int i, j, ret;
    char retvalue = 0;
    struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);

    if((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85) || (icn85xx_ts->ictype == ICN85XX_WITH_FLASH_85))
    {
        rowMax = 36;
        columnMax = 24;
    }
    else
    {
        rowMax = 42;
        columnMax = 30;
    }
    icn85xx_read_reg(0x8004, &row);
    icn85xx_read_reg(0x8005, &column);    
    //scan tp rawdata
    icn85xx_write_reg(4, 0x20); 
    mdelay(1);
    for(i=0; i<1000; i++)
    {
        mdelay(1);
        icn85xx_read_reg(2, &retvalue);
        if(retvalue == 1)
            break;
    }     
    if(diff == 0)
    {
        for(i=0; i<row; i++)
        {       
            ret = icn85xx_i2c_rxdata(0x2000 + i*columnMax*2, (char *)&log_rawdata[i][0], column*2);
            if (ret < 0) {
                icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
            } 
            icn85xx_rawdatadump(&log_rawdata[i][0], column, columnMax);  
    
        } 
    }
    if(diff == 1)
    {
        for(i=0; i<row; i++)
        { 
            ret = icn85xx_i2c_rxdata(0x3000 + (i+1)*(columnMax+2)*2 + 2, (char *)&log_diffdata[i][0], column*2);
            if (ret < 0) {
                icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
            } 
            icn85xx_rawdatadump(&log_diffdata[i][0], column, columnMax);   
            
        }           
    }
    else if(diff == 2)
    {        
        for(i=0; i<row; i++)
        {       
            ret = icn85xx_i2c_rxdata(0x2000 + i*columnMax*2, (char *)&log_rawdata[i][0], column*2);
            if (ret < 0) {
                icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
            } 
            if((log_basedata[0][0] != 0) || (log_basedata[0][1] != 0))
            {
                for(j=0; j<column; j++)
                {
                    log_rawdata[i][j] = log_basedata[i][j] - log_rawdata[i][j];
                }
            }
            icn85xx_rawdatadump(&log_rawdata[i][0], column, columnMax);  
    
        }    
        if((log_basedata[0][0] == 0) && (log_basedata[0][1] == 0))
        {
            memcpy(&log_basedata[0][0], &log_rawdata[0][0], COL_NUM*ROW_NUM*2);           
        }
    }
    
    //finish scan tp rawdata
    icn85xx_write_reg(2, 0x0);      
    icn85xx_write_reg(4, 0x21);    
}

/***********************************************************************************************
Name    :   icn85xx_testTP
Input   :   0: ca, 1: diff test
Output  :   err type
function    :   test TP
***********************************************************************************************/
static int icn85xx_testTP(char type, int para1, int para2, int para3)
{
    char retvalue = 0;  //ok
    char ret;
    char row = 0;
    char column = 0;
    char rowMax = 0;
    char columnMax = 0;
    char ScanMode = 0;
    char ictype = 85;
    int diffMax = 0;
    int diffMin = 100000;
    char vol, regValue;
    int i, j;
    int regAddr;
    struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);

    if((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85) || (icn85xx_ts->ictype == ICN85XX_WITH_FLASH_85))
    {
        rowMax = 36;
        columnMax = 24;
        ictype = 85;
    }
    else
    {
        rowMax = 42;
        columnMax = 30;
        ictype = 86;
    }
    
    icn85xx_read_reg(0x8004, &row);
    icn85xx_read_reg(0x8005, &column);  

    icn85xx_ts->work_mode = 4; //idle
    if(type == 0)
    {
        memset(&log_basedata[0][0], 0, COL_NUM*ROW_NUM*2);
        icn85xx_log(0);
        for(i=0; i < row; i++)
        {
            for (j = 0; j < (column-1); j++) 
            {                
                log_basedata[i][j] = log_rawdata[i][j+1] - log_rawdata[i][j];
            }            
        }
                        
        for(i=0; i < (row-1); i++)
        {
            for (j = 0; j < (column-1); j++) 
            {                
                log_rawdata[i][j]  = log_basedata[i+1][j] -  log_basedata[i][j];
                if(abs(log_rawdata[i][j]) >  abs(para1))
                {
                    retvalue = 1;  // error
                }
            }
            icn85xx_rawdatadump(&log_rawdata[i][0], column-1, columnMax);
        }
        
    }
    else if(type == 1)
    {
        memset(&log_rawdata[0][0], 0, COL_NUM*ROW_NUM*2);

        if(ictype == 86)
        {
            icn85xx_read_reg(0x8000+87, &ScanMode);
            icn85xx_write_reg(0x8000+87, 0);  //change scanmode

            
            printk("reinit tp1, ScanMode: %d\n", ScanMode);
            icn85xx_write_reg(0, 1); 
            mdelay(100);
            icn85xx_write_reg(0, 0);   
            mdelay(100);
            regAddr =0x040b38;
            ret = icn85xx_i2c_txdata(0xf000, &regAddr, 4);
            if (ret < 0) {
                icn85xx_error("1 write reg failed! ret: %d\n", ret);
                return -1;
            }
            ret = icn85xx_i2c_rxdata(0xf004, &regValue, 1);
            printk("regValue: 0x%x\n", regValue);
            ret = icn85xx_i2c_txdata(0xf000, &regAddr, 4);
            if (ret < 0) {
                icn85xx_error("2 write reg failed! ret: %d\n", ret);
                return -1;
            }
            regAddr = regValue&0xfe;
            ret = icn85xx_i2c_txdata(0xf004, &regAddr, 1);
            if (ret < 0) {
                icn85xx_error("3 write reg failed! ret: %d\n", ret);
                return -1;
            }
        }
        
        mdelay(100);
        for(i=0; i<5; i++)
        icn85xx_log(0);
        
        vol = (para1&0xff) | 0x10;
        printk("-------vol: 0x%x\n", vol);
        icn85xx_write_reg(4, vol);
        memcpy(&log_basedata[0][0], &log_rawdata[0][0], COL_NUM*ROW_NUM*2); 

        mdelay(100);
        for(i=0; i<5; i++)
        icn85xx_log(0);
        
        icn85xx_write_reg(4, 0x10);
        
        if(ictype == 86)
        {
            icn85xx_write_reg(0x8000+87, ScanMode);

            printk("reinit tp2\n");
            icn85xx_write_reg(0, 1); 
            mdelay(100);
            icn85xx_write_reg(0, 0); 
            mdelay(100);

            regAddr =0x040b38;
            ret = icn85xx_i2c_txdata(0xf000, &regAddr, 4);
            if (ret < 0) {
                icn85xx_error("4 write reg failed! ret: %d\n", ret);
                return -1;
            }
            ret = icn85xx_i2c_txdata(0xf004, &regValue, 1);
            if (ret < 0) {
                icn85xx_error("5 write reg failed! ret: %d\n", ret);
                return -1;
            }
        }
        update_data2 = para2*120/100;
        update_data3 = para3*80/100;
        printk("-------diff data: %d, %d\n", update_data2, update_data3);
        for(i=0; i < row; i++)
        {
            for (j = 0; j < column; j++) 
            {                
                log_diffdata[i][j] = log_basedata[i][j] - log_rawdata[i][j];               
                if(log_diffdata[i][j] > diffMax)
                {
                    diffMax = log_diffdata[i][j];
                }
                else if(log_diffdata[i][j] < diffMin)
                {
                    diffMin = log_diffdata[i][j];
                } 
                
                if((update_data2 > 0) && (update_data3 > 0))  // make sure Max/Min > 0
                {
                    if((log_diffdata[i][j] > update_data2) || (log_diffdata[i][j] < update_data3))
                    {
                        retvalue = 1;  // error
                    }
                }    
            }         
           
            icn85xx_rawdatadump(&log_diffdata[i][0], column, columnMax);
        }

        update_data2 = diffMax;
        update_data3 = diffMin;
        
    }
    icn85xx_ts->work_mode = 0;

    return retvalue;
}

#if defined(TPD_ROTATE_90) || defined(TPD_ROTATE_270)
static void tpd_swap_xy(short *x, short *y)
{
	short temp = 0;

	temp = *x;
	*x = *y;
	*y = temp;
}

static void tpd_rotate_90(short *x, short *y)
{
	short temp;

	*x = TPD_RES_X + 1 - *x;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}

static void tpd_rotate_270(short *x, short *y)
{
	short temp;

	*y = TPD_RES_Y + 1 - *y;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
#endif


static int icn85xx_i2c_test(void)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);
	int  ret = -1;
	char value = 0;
	char buf[3];
	int  retry = 0;
	int  flashid;
	icn85xx_ts->ictype = 0;
	icn85xx_trace("====%s begin=====.  \n", __func__);
        icn85xx_ts->ictype = ICTYPE_UNKNOWN;

	while(retry++ < 3)
	{        
		ret = icn85xx_read_reg(0xa, &value);
		if(ret > 0)
		{
			if(value == 0x85)
			{
				icn85xx_ts->ictype = ICN85XX_WITH_FLASH_85;
                       setbootmode(ICN85XX_WITH_FLASH_85);
				return ret;
			}
			else if((value == 0x86)||(value == 0x88) )
			{
				icn85xx_ts->ictype = ICN85XX_WITH_FLASH_86;
                setbootmode(ICN85XX_WITH_FLASH_86);
				return ret;  
			}
            else if(value == 0x87)
            {
                icn85xx_ts->ictype = ICN85XX_WITH_FLASH_87;
                setbootmode(ICN85XX_WITH_FLASH_87);
            return ret;  
            }
        }
		icn85xx_error("iic test error! retry = %d,value=0x%x.\n", retry,value);
		msleep(3);
	}

	icn85xx_goto_progmode();
	msleep(10);
	retry = 0;
	while(retry++ < 3)
	{       
       buf[0] = buf[1] = buf[2] = 0x0;
     ret = icn85xx_prog_i2c_txdata(0x040000,buf,3);
    if (ret < 0) {
                  icn85xx_error("write reg failed! ret: %d\n", ret);
                return ret;
              }
        ret = icn85xx_prog_i2c_rxdata(0x040000, buf, 3);
        icn85xx_trace("icn85xx_prog_i2c_rxdata: ret = %d, buf[0]=0x%2x,buf[0]=0x%2x, buf[0]=0x%2x\n", ret, buf[0], buf[1], buf[2]);
        if(ret > 0)
        {
            //if(value == 0x85)
            if((buf[2] == 0x85) && (buf[1] == 0x05))
            {
                flashid = icn85xx_read_flashid();
                if((MD25D40_ID1 == flashid) || (MD25D40_ID2 == flashid)
                    ||(MD25D20_ID1 == flashid) || (MD25D20_ID2 == flashid)
                    ||(GD25Q10_ID == flashid) || (MX25L512E_ID == flashid)
                    || (MD25D05_ID == flashid)|| (MD25D10_ID == flashid))
                {
                    icn85xx_ts->ictype = ICN85XX_WITH_FLASH_85;
                    setbootmode(ICN85XX_WITH_FLASH_85);

                }
                else
                {
                    icn85xx_ts->ictype = ICN85XX_WITHOUT_FLASH_85;
                    setbootmode(ICN85XX_WITHOUT_FLASH_85);

                }
                return ret;
            }
            else if((buf[2] == 0x85) && (buf[1] == 0x0e))
            {
                flashid = icn85xx_read_flashid();
                if((MD25D40_ID1 == flashid) || (MD25D40_ID2 == flashid)
                    ||(MD25D20_ID1 == flashid) || (MD25D20_ID2 == flashid)
                    ||(GD25Q10_ID == flashid) || (MX25L512E_ID == flashid)
                    || (MD25D05_ID == flashid)|| (MD25D10_ID == flashid))
                {
                    icn85xx_ts->ictype = ICN85XX_WITH_FLASH_86;
                    setbootmode(ICN85XX_WITH_FLASH_86);

                }
                else
                {
                    icn85xx_ts->ictype = ICN85XX_WITHOUT_FLASH_86;
                    setbootmode(ICN85XX_WITHOUT_FLASH_86);

                }
                return ret;
            }
            else  //for ICNT87
            {                
                ret = icn87xx_prog_i2c_rxdata(0xf001, buf, 2);
                if(ret > 0)                    
                {
                    if(buf[1] == 0x87)
                    {                        
                        flashid = icn87xx_read_flashid();                        
                        printk("icnt87 flashid: 0x%x\n",flashid);
                        if(0x114051 == flashid)
                        {
                            icn85xx_ts->ictype = ICN85XX_WITH_FLASH_87;  
                            setbootmode(ICN85XX_WITH_FLASH_87);
                        }                        
                        else
                        {
                            icn85xx_ts->ictype = ICN85XX_WITHOUT_FLASH_87;  
                            setbootmode(ICN85XX_WITHOUT_FLASH_87);
                        }
                        return ret;
                    }
								}
						}
				}
				icn85xx_error("i2c test error! %d\n",retry);
				msleep(3);
			}
			return ret;
	}
void icn85xx_ts_reset(void)
{
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(80);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(50);
}
void icn85xx_set_prog_addr(void)
{  
      // set INT mode
	//mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ZERO);   //????
	msleep(15);
	 icn85xx_ts_reset();
	 mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ONE);
	 mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	 mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	 mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	 mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	 msleep(15);
	#if 0 
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);   
    
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
   #endif
}
static void icn85xx_hw_init(void)
{

	icn85xx_trace("%s, icn85xx_hw_init!\n");
	//hwPowerOn(TPD_POWER_SOURCE_CUSTOM, TPD_POWER_SOURCE_VOL, "TP");
	//hwPowerOn(MT65XX_POWER_LDO_VIO28, VOL_2800, "TP");
       //hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");

      
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_3000, "TP");
       hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "TP");   
       hwPowerOn(MT6323_POWER_LDO_VCAM_AF, VOL_2800, "TP");

	msleep(175);


#if 0
	mt_set_gpio_mode(GPIO_CTP_PWR_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_PWR_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_PWR_PIN, GPIO_OUT_ONE);
	msleep(50);
#ifdef GPIO_CTP_WAKEUP_PIN
	mt_set_gpio_mode(GPIO_CTP_WAKEUP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_WAKEUP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_WAKEUP_PIN, GPIO_OUT_ONE);
#endif
#else
	// gpio setting
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(50);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(30);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
	msleep(100);
	//icn85xx_ts_reset();
#if 0
	// set INT mode
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif

}

static void icn85xx_ts_release(void)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);
	icn85xx_trace("%s() Line:%d\n", __FUNCTION__, __LINE__);
	//	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
	input_sync(tpd->dev);
}

static void icn85xx_report_value_A(void)
{
	icn85xx_trace("%s() Line:%d\n", __FUNCTION__, __LINE__);
	
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);
	char buf[POINT_NUM * POINT_SIZE + 3] = {0};
	int ret = -1;
	int i;
	int keys[TPD_KEY_COUNT];

#ifdef TPD_SLIDE_WAKEUP
if(icn85xx_ges_en){
	icn85xx_gesture_handler();
}
#endif

	ret = icn85xx_i2c_rxdata(0x1000, buf, POINT_NUM * POINT_SIZE + 2);
	if(ret < 0)
	{
		icn85xx_trace("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	
#ifdef TPD_PROXIMITY
	icn85xx_proximity_handler(buf[0]>>7);	
#endif

#ifdef TPD_HAVE_BUTTON 
		unsigned char button;
		static unsigned char button_last;

		button = buf[0]&0x0f;
		icn85xx_trace("button=%d\n", button);    

		if((button_last != 0) && (button == 0))
		{
			tpd_up(tpd_keys_dim_local[i][0],tpd_keys_dim_local[i][1], 0);
			button_last = button;
			return 1;       
		}
		if(button != 0)
		{

			//tpd_down(tpd_keys_dim_local[i][0],tpd_keys_dim_local[i][1], 0, 0);
			for(i = 0; i < TPD_KEY_COUNT; i++)
			{

				if (button & (0x01 << i))
				{

					icn85xx_info("%s: button tpd_down=%d , i = %d\n",__func__, button,i);

					tpd_down(tpd_keys_dim_local[i][0],tpd_keys_dim_local[i][1], 0, 0);
				}     

			}   
			button_last = button;
			return 1;
		}
#endif
		
	icn85xx_ts->point_num = buf[1];
	//icn85xx_trace(" icn85xx zby %d \n",icn85xx_ts->point_num );
	if(icn85xx_ts->point_num == 0)
	{
		icn85xx_ts_release();
		return 1;
	}

	for(i = 0; i < icn85xx_ts->point_num; i++)
	{
		if(buf[8 + POINT_SIZE * i] != 4)
			break;
	}

	if(i == icn85xx_ts->point_num)
	{
		icn85xx_ts_release();
		return 1;
	}

	for(i = 0; i < icn85xx_ts->point_num; i++)
	{
		icn85xx_ts->point_info[i].u8ID = buf[2 + POINT_SIZE*i];
		icn85xx_ts->point_info[i].u16PosX = (buf[4 + POINT_SIZE*i]<<8) + buf[3 + POINT_SIZE*i];
		icn85xx_ts->point_info[i].u16PosY = (buf[6 + POINT_SIZE*i]<<8) + buf[5 + POINT_SIZE*i];
		icn85xx_ts->point_info[i].u8Pressure = 200;//buf[7 + POINT_SIZE*i];
		icn85xx_ts->point_info[i].u8EventId = buf[8 + POINT_SIZE*i];    

		if(1 == icn85xx_ts->revert_x_flag)
			icn85xx_ts->point_info[i].u16PosX = TPD_RES_X - icn85xx_ts->point_info[i].u16PosX;
		if(1 == icn85xx_ts->revert_y_flag)
			icn85xx_ts->point_info[i].u16PosY = TPD_RES_Y - icn85xx_ts->point_info[i].u16PosY;

	//	icn85xx_trace("u8ID %d\n", icn85xx_ts->point_info[i].u8ID);
	//	icn85xx_trace("u16PosX %d\n", icn85xx_ts->point_info[i].u16PosX);
	//	icn85xx_trace("u16PosY %d\n", icn85xx_ts->point_info[i].u16PosY);
	//	icn85xx_trace("u8Pressure %d\n", icn85xx_ts->point_info[i].u8Pressure);
	//	icn85xx_trace("u8EventId %d\n", icn85xx_ts->point_info[i].u8EventId);
		input_report_key(tpd->dev, BTN_TOUCH, 1);
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, icn85xx_ts->point_info[i].u8ID);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, icn85xx_ts->point_info[i].u16PosX);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, icn85xx_ts->point_info[i].u16PosY);
		//input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(tpd->dev);
	//	icn85xx_point_info("point: %d ===x = %d,y = %d, press = %d ====\n",i, icn85xx_ts->point_info[i].u16PosX,icn85xx_ts->point_info[i].u16PosY, icn85xx_ts->point_info[i].u8Pressure);
	}
	input_sync(tpd->dev);

}

#if CTP_REPORT_PROTOCOL
static void icn85xx_report_value_B(void)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);
	char buf[POINT_NUM * POINT_SIZE + 3] = {0};
	static unsigned char finger_last[POINT_NUM + 1] = {0};
	unsigned char finger_current[POINT_NUM + 1] = {0};
	unsigned int position = 0;
	int temp = 0;
	int ret = -1;

	icn85xx_info("==icn85xx_report_value_B ==\n");

	ret = icn85xx_i2c_rxdata(0x1000, buf, POINT_NUM * POINT_SIZE + 2);
	if(ret < 0)
	{
		icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	icn85xx_ts->point_num = buf[1];
	if(icn85xx_ts->point_num > 0)
	{
		for(position = 0; position < icn85xx_ts->point_num; position++)
		{
			temp = buf[2 + POINT_SIZE * position] + 1;
			finger_current[temp] = 1;
			icn85xx_ts->point_info[temp].u8ID = buf[2 + POINT_SIZE*position];
			icn85xx_ts->point_info[temp].u16PosX = (buf[4 + POINT_SIZE*position]<<8) + buf[3 + POINT_SIZE*position];
			icn85xx_ts->point_info[temp].u16PosY = (buf[6 + POINT_SIZE*position]<<8) + buf[5 + POINT_SIZE*position];
			icn85xx_ts->point_info[temp].u8Pressure = buf[7 + POINT_SIZE*position];
			icn85xx_ts->point_info[temp].u8EventId = buf[8 + POINT_SIZE*position];

			if(icn85xx_ts->point_info[temp].u8EventId == 4)
				finger_current[temp] = 0;

			if(1 == icn85xx_ts->revert_x_flag)
				icn85xx_ts->point_info[temp].u16PosX = icn85xx_ts->screen_max_x - icn85xx_ts->point_info[temp].u16PosX;


			if(1 == icn85xx_ts->revert_y_flag)
				icn85xx_ts->point_info[temp].u16PosY = icn85xx_ts->screen_max_y - icn85xx_ts->point_info[temp].u16PosY;

			icn85xx_info("temp %d\n", temp);
			icn85xx_info("u8ID %d\n", icn85xx_ts->point_info[temp].u8ID);
			icn85xx_info("u16PosX %d\n", icn85xx_ts->point_info[temp].u16PosX);
			icn85xx_info("u16PosY %d\n", icn85xx_ts->point_info[temp].u16PosY);
			icn85xx_info("u8Pressure %d\n", icn85xx_ts->point_info[temp].u8Pressure);
			icn85xx_info("u8EventId %d\n", icn85xx_ts->point_info[temp].u8EventId);
		}
	}
	else
	{
		for(position = 1; position < POINT_NUM + 1; position++)
		{
			finger_current[position] = 0;
		}
		icn85xx_info("no touch\n");
	}

	for(position = 1; position < POINT_NUM + 1; position++)
	{
		if((finger_current[position] == 0) && (finger_last[position] != 0))
		{
			input_mt_slot(tpd->dev, position - 1);
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
			icn85xx_point_info("one touch up: %d\n", position);
		}
		else if(finger_current[position])
		{
			input_mt_slot(tpd->dev, position - 1);
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, true);
			input_report_key(tpd->dev, BTN_TOUCH, 1);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(tpd->dev, ABS_MT_PRESSURE, 200);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X, icn85xx_ts->point_info[position].u16PosX);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y, icn85xx_ts->point_info[position].u16PosY);
			icn85xx_point_info("===position: %d, x = %d,y = %d, press = %d ====\n", position, icn85xx_ts->point_info[position].u16PosX,icn85xx_ts->point_info[position].u16PosY, icn85xx_ts->point_info[position].u8Pressure);
		}
	}
	input_sync(tpd->dev);

	for(position = 1; position < POINT_NUM + 1; position++)
		finger_last[position] = finger_current[position];
}
#endif

static int touch_event_handler(void *unused)
{      
	int i = 0;

	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);

#if SUPPORT_PROC_FS
	if(down_interruptible(&icn85xx_ts->sem))  
	{  
		return -1;   
	}  
#endif

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;
		set_current_state(TASK_RUNNING);

		if(icn85xx_ts->work_mode == 0)
		{
#if CTP_REPORT_PROTOCOL
			icn85xx_report_value_B();
#else
			icn85xx_report_value_A();
#endif
		}
#if TPD_ICN85XX_SUPPORT_FW_UPDATE
		else if(icn85xx_ts->work_mode == 1)
		{
			icn85xx_trace("log raw data\n");
			icn85xx_log(0);
		}
		else if(icn85xx_ts->work_mode == 2)
		{
			icn85xx_trace("log diff data\n");
			icn85xx_log(1);
		}

		else if(icn85xx_ts->work_mode == 3)
		{
			icn85xx_trace("raw2diff data\n");
			icn85xx_log(2);   //diff data
		}
		else if(icn85xx_ts->work_mode == 4)  //idle
		{
			;
		}
		else if(icn85xx_ts->work_mode == 5)//write para, reinit
		{
			icn85xx_trace("reinit tp\n");
			icn85xx_write_reg(0, 1); 
			mdelay(100);
			icn85xx_write_reg(0, 0);			
			icn85xx_ts->work_mode = 0;
		}
		else if((icn85xx_ts->work_mode == 6) ||(icn85xx_ts->work_mode == 7))  //gesture test mode
		{
#if SUPPORT_PROC_FS
			char buf[sizeof(structGestureData)]={0};
			int ret = -1;
			int i;
			ret = icn85xx_i2c_rxdata(0x7000, buf, 2);
			if (ret < 0) {
				icn85xx_trace("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
				return ;
			}            
			structGestureData.u8Status = 1;
			structGestureData.u8Gesture = buf[0];
			structGestureData.u8GestureNum = buf[1];
			icn85xx_trace("structGestureData.u8Gesture: 0x%x\n", structGestureData.u8Gesture);
			icn85xx_trace("structGestureData.u8GestureNum: %d\n", structGestureData.u8GestureNum);
			ret = icn85xx_i2c_rxdata(0x7002, buf, structGestureData.u8GestureNum*6);
			if (ret < 0) {
				icn85xx_trace("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
				return ;
			}  
			for(i=0; i<structGestureData.u8GestureNum; i++)
			{
				structGestureData.point_info[i].u16PosX = (buf[1 + 6*i]<<8) + buf[0+ 6*i];
				structGestureData.point_info[i].u16PosY = (buf[3 + 6*i]<<8) + buf[2 + 6*i];
				structGestureData.point_info[i].u8EventId = buf[5 + 6*i];
				icn85xx_trace("(%d, %d, %d)", structGestureData.point_info[i].u16PosX, structGestureData.point_info[i].u16PosY, structGestureData.point_info[i].u8EventId);
			} 
			icn85xx_trace("\n");
			if(icn85xx_ts->use_irq)
			{
			}
			if(((icn85xx_ts->work_mode == 7) && (structGestureData.u8Gesture == 0xFB))
					|| (icn85xx_ts->work_mode == 6))
			{
				icn85xx_trace("return normal mode\n");
				icn85xx_ts->work_mode = 0;  //return normal mode
			}
#endif
		}

#if SUPPORT_PROC_FS
		up(&icn85xx_ts->sem);
#endif

#endif

	} while(!kthread_should_stop());
	return 0;
}
static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);

	icn85xx_trace("inc85xx %s, %d\n", __FUNCTION__, __LINE__);
	return 0;
}

static void tpd_eint_interrupt_handler(void)
{
	icn85xx_trace("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}

#ifdef  TPD_SLIDE_WAKEUP
//-----------------------------------------------------------------------------------------------------
static ssize_t gesture_mode_show(struct class *class, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", twd.gestrue_en == true ? "enable" : "disable");
}

static ssize_t gesture_mode_store(struct class *class, struct class_attribute *attr,const char *buf, size_t count)
{
	bool value;
	if(strtobool(buf, &value))
		return -EINVAL;
	
	//mutex_lock(&twd.lock);
	
	if(value)
		twd.gestrue_en = true;
	else
		twd.gestrue_en = false;
	
	//mutex_unlock(&twd.lock);
	
	return count;
}

static u8 charging_mode = 0;
static u8 pre_charging_mode = 0;

static struct class_attribute cls_attr[]={
	//__ATTR(glvmode,0664,glove_mode_show,glove_mode_store),
	__ATTR(gesenable, 0664, gesture_mode_show, gesture_mode_store),
	// __ATTR(charging, 0664, charging_mode_show,charging_mode_store),
	//__ATTR(event,0444,event_show,NULL),
};

static int class_create_files(struct class *cls,struct class_attribute *attr,int size)
{
	int i,ret=0;
	for(i=0;i < size ;i++){
		ret=class_create_file(cls,attr + i);
		if(ret < 0)
			goto undo;
	}
	return 0;
undo:
	for (; i >= 0 ; i--)
		class_remove_file(cls, attr + i);
	icn85xx_trace(KERN_ERR"%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}
//-----------------------------------------------------------------------------------------------------
#endif

#if SUPPORT_CHECK_ESD
static void icn85xx_esd_check_work(struct work_struct *work)
{
    int err,ret;
    u8 value;
    struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);
    int retry = 3;
    int need_update_fw = false;
	
    icn85xx_trace("icn85xx_esd_check_work enter \n");

	if ((icn85xx_ts->icn_is_suspend)||(icn85xx_ts->is_apk_update == 1)) 
	{
		icn85xx_trace("icn85xx Esd suspended or in APK update!");
		return;
	}
	
   if(icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)
    {
        while (retry-- && !need_update_fw) 
        {
            //icn85xx_ts_reset();
           // icn85xx_set_prog_addr();
           // icn87xx_boot_sram();
           // msleep(50);
            ret = icn85xx_read_reg(0xa, &value);
             if ((ret > 0) && (value == 0x87)) 
            { 
                need_update_fw = false;
                break; 
            }
        }
        if (retry <= 0) need_update_fw = true;

        if (need_update_fw) 
        {
			icn85xx_set_prog_addr();
			icn87xx_boot_sram();
        }           
    }
    if(!icn85xx_ts->icn_is_suspend) 
	{
        queue_delayed_work(icn85xx_esd_workqueue, &icn85xx_esd_event_work, icn85xx_ts->clk_tick_cnt);
    } 
	else
    {
    	icn85xx_trace("Esd suspended!");
    }
        return;

}
#endif

static int  tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct icn85xx_ts_data *icn85xx_ts;
	int retval = TPD_OK;
	int err = 0;
	short fwVersion = 0;
	short curVersion = 0;
	int retry;
//**************************************************************//
#ifdef TPD_PROXIMITY
	int errpro =0;
	struct hwmsen_object obj_ps;
#endif
//**************************************************************//	
#ifdef  TPD_SLIDE_WAKEUP
	struct class *cls;
#endif
//**************************************************************//	

	i2c_client = client;

#if TPD_ICN85XX_I2C_DMA_SUPPORT

	i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
	i2c_client->timing = 400;

	icn85xx_i2c_dma_va = (u8 *)dma_alloc_coherent(NULL, 4096, &icn85xx_i2c_dma_pa, GFP_KERNEL);
	if(!icn85xx_i2c_dma_va)
	{
		icn85xx_trace(KERN_ERR "%s, TPD dma_alloc_coherent error!\n", __FUNCTION__);
	}
	else
	{
		icn85xx_trace("%s, TPD dma_alloc_coherent success!\n", __FUNCTION__);
	}

	mutex_init(&i2c_data_mutex);

#endif

	icn85xx_ts = kzalloc(sizeof(*icn85xx_ts), GFP_KERNEL);
	if(!icn85xx_ts)
	{
		icn85xx_trace("Alloc icn85xx_ts memory failed.\n");
		return -ENOMEM;
	}
	memset(icn85xx_ts, 0, sizeof(*icn85xx_ts));
	i2c_set_clientdata(client, icn85xx_ts);

	icn85xx_ts->work_mode = 0;

	icn85xx_hw_init();

	err = icn85xx_i2c_test();
	if (err <= 0)
	{
		icn85xx_trace("icn85xx_iic_test  failed.\n");
		return -1;

	}
	else
	{
		icn85xx_trace("iic communication ok: 0x%x\n", icn85xx_ts->ictype); 
	}

	//tpd_load_status = 1;
	if((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85) || (icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_86))
	{
#if TPD_ICN85XX_COMPILE_FW_WITH_DRIVER
		icn85xx_set_fw(sizeof(icn85xx_fw), &icn85xx_fw[0]);
#endif    

		if(R_OK == icn85xx_fw_update(firmware))
		{
			icn85xx_ts->code_loaded_flag = 1;
			icn85xx_trace("ICN85XX_WITHOUT_FLASH, update ok\n"); 

		}
		else
		{
			icn85xx_ts->code_loaded_flag = 0;
			icn85xx_trace("ICN85XX_WITHOUT_FLASH, update error\n"); 
			return -1;
		}

	}
	else if((icn85xx_ts->ictype == ICN85XX_WITH_FLASH_85) || (icn85xx_ts->ictype == ICN85XX_WITH_FLASH_86))
	{
#if TPD_ICN85XX_SUPPORT_FW_UPDATE
		icn85xx_set_fw(sizeof(icn85xx_fw), &icn85xx_fw[0]);
		fwVersion = icn85xx_read_fw_Ver(firmware);
		curVersion = icn85xx_readVersion();
		icn85xx_trace("fwVersion : 0x%x\n", fwVersion);
		icn85xx_trace("current version: 0x%x\n", curVersion);

#if TPD_ICN85XX_FORCE_UPDATE_FW
		retry = 5;
		while(retry > 0)
		{
			if(icn85xx_goto_progmode() != 0)
			{
				printk("icn85xx_goto_progmode() != 0 error\n");
				return -1; 
			} 
			icn85xx_read_flashid();
			printk("begin to update\n");
			if(R_OK == icn85xx_fw_update(firmware))
			{
				break;
			}
			retry--;
			icn85xx_error("icn85xx_fw_update failed.\n");
		}
#else
		if(fwVersion > curVersion)
		{
			retry = 5;
			while(retry > 0)
			{
				if(R_OK == icn85xx_fw_update(firmware))
				{
					break;
				}
				retry--;
				icn85xx_error("icn85xx_fw_update failed.\n");
			}
		}
#endif
	}
    else if(icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)
    {
        icn85xx_trace("icn85xx_update  87 without flash\n");
        
        
        #if TPD_ICN85XX_COMPILE_FW_WITH_DRIVER
            icn85xx_set_fw(sizeof(icn85xx_fw), &icn85xx_fw[0]);
        #endif
        
        fwVersion = icn85xx_read_fw_Ver(firmware);
        curVersion = icn85xx_readVersion();
        icn85xx_trace("fwVersion : 0x%x\n", fwVersion); 
        icn85xx_trace("current version: 0x%x\n", curVersion);
  
        if(R_OK == icn87xx_fw_update(firmware))
        {
            icn85xx_ts->code_loaded_flag = 1;
            printk("ICN87XX_WITHOUT_FLASH, update default fw ok\n");
        }
        else
        {
            icn85xx_ts->code_loaded_flag = 0;
            printk("ICN87XX_WITHOUT_FLASH, update error\n"); 
        }
     
    }
    else if(icn85xx_ts->ictype == ICN85XX_WITH_FLASH_87)
    {
        printk("icn85xx_update 87 with flash\n");
           
        #if TPD_ICN85XX_COMPILE_FW_WITH_DRIVER
            icn85xx_set_fw(sizeof(icn85xx_fw), &icn85xx_fw[0]);
        #endif
        
        fwVersion = icn85xx_read_fw_Ver(firmware);
        curVersion = icn85xx_readVersion();
        icn85xx_trace("fwVersion : 0x%x\n", fwVersion); 
        icn85xx_trace("current version: 0x%x\n", curVersion); 
             
        
        
        #if TPD_ICN85XX_FORCE_UPDATE_FW        
            if(R_OK == icn87xx_fw_update(firmware))
            {
                icn85xx_ts->code_loaded_flag = 1;
                icn85xx_trace("ICN87XX_WITH_FLASH, update default fw ok\n");
            }
            else
            {
                icn85xx_ts->code_loaded_flag = 0;
                icn85xx_trace("ICN87XX_WITH_FLASH, update error\n"); 
            }    
         
        #else
            if(fwVersion > curVersion)
            {
                retry = 5;
                while(retry > 0)
                {
                    if(R_OK == icn87xx_fw_update(firmware))
                    {
                        break;
                    }
                    retry--;
                    icn85xx_error("icn87xx_fw_update failed.\n");   
                }
            }
        #endif
    }
#endif



#if SUPPORT_PROC_FS
	sema_init(&icn85xx_ts->sem, 1);
	init_proc_node();
#endif

//*************************************************//
#ifdef TPD_PROXIMITY
		hwmsen_detach(ID_PROXIMITY);
		obj_ps.self = NULL;
		obj_ps.polling = 0;//interrupt mode
		obj_ps.sensor_operate = tpd_ps_operate_inc85xx;//gsl1680p_ps_operate;
		if((errpro = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
		{
			icn85xx_trace("attach fail = %d\n", errpro);
		}
		wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");
#endif

//****************************************************//


#ifdef  TPD_SLIDE_WAKEUP
     #if 0
    input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
    input_set_capability(tpd->dev, EV_KEY, KEY_C);
    input_set_capability(tpd->dev, EV_KEY, KEY_S);
    input_set_capability(tpd->dev, EV_KEY, KEY_V);
    input_set_capability(tpd->dev, EV_KEY, KEY_Z);
    input_set_capability(tpd->dev, EV_KEY, KEY_M);
    input_set_capability(tpd->dev, EV_KEY, KEY_E);
    input_set_capability(tpd->dev, EV_KEY, KEY_O);
    input_set_capability(tpd->dev, EV_KEY, KEY_W);
    input_set_capability(tpd->dev, EV_KEY, KEY_UP);
    input_set_capability(tpd->dev, EV_KEY, KEY_DOWN);
    input_set_capability(tpd->dev, EV_KEY, KEY_LEFT);
    input_set_capability(tpd->dev, EV_KEY, KEY_RIGHT);
     input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
     #endif
	//mutex_init(&twd.lock);
	//twd.gestrue_pre_en = twd.gestrue_en = false;
	//twd.suspended = false;
	//twd.glove_mode = GLOVE_MODE_DISABLE;
	//cls = class_create(THIS_MODULE,"tp_gesture_switch");
	// if(cls){
 	//retval=class_create_files(cls,&cls_attr,ARRAY_SIZE(cls_attr));
	//
	//}
#ifdef GTP_GESTURE_APP
        gtp_gesture_init();
#endif
#endif

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if(IS_ERR(thread))
	{
		retval = PTR_ERR(thread);
		icn85xx_trace(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}
   
#if 1 
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);   
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif

#if SUPPORT_CHECK_ESD
	INIT_DELAYED_WORK(&icn85xx_esd_event_work, icn85xx_esd_check_work);
	icn85xx_esd_workqueue = create_workqueue("icn85xx_esd");

	icn85xx_ts->clk_tick_cnt = 1 * HZ;      // HZ: clock ticks in 1 second generated by system
	icn85xx_trace("Clock ticks for an esd cycle: %d", icn85xx_ts->clk_tick_cnt);
	spin_lock_init(&icn85xx_ts->esd_lock);

	spin_lock(&icn85xx_ts->esd_lock);
	if (!icn85xx_ts->esd_running)
    {
		icn85xx_ts->esd_running = 1;
		spin_unlock(&icn85xx_ts->esd_lock);
		icn85xx_trace("Esd started");
		queue_delayed_work(icn85xx_esd_workqueue, &icn85xx_esd_event_work, icn85xx_ts->clk_tick_cnt);
	}
#endif

	tpd_load_status = 1;

	icn85xx_trace("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

	return retval;
}

static int  tpd_remove(struct i2c_client *client)
{
	icn85xx_trace("TPD removed\n");
#if TPD_ICN85XX_I2C_DMA_SUPPORT
	if(icn85xx_i2c_dma_va)
	{
		dma_free_coherent(NULL, 4096, icn85xx_i2c_dma_va, icn85xx_i2c_dma_pa);
		icn85xx_i2c_dma_va = NULL;
		icn85xx_i2c_dma_pa = 0;
	}
#endif

#if SUPPORT_PROC_FS
	uninit_proc_node();
#endif

	kfree(i2c_get_clientdata(client));

	return 0;
}

static int tpd_local_init(void)
{
	icn85xx_trace("ICN85XX I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
	
	if(i2c_add_driver(&tpd_i2c_driver) != 0)
	{
		icn85xx_trace("unable to add i2c driver.\n");
		return -1;
	}
	tpd_load_status = 1;

#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif

	icn85xx_trace("end %s, %d\n", __FUNCTION__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

static int tpd_resume(struct i2c_client *client)
{
	icn85xx_trace("==icn85xx_ts_resume==\n");
	
	int retval = TPD_OK;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);
    int i;
    int ret = -1;
    int retry = 5;
    int need_update_fw = false;
    unsigned char value;
	
//*************************************************//
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1) 
	{
		//tpd_enable_ps(1);
		return;
	}
#endif
//**************************************************//
#ifdef TPD_SLIDE_WAKEUP
	suspend_status=0;
#endif
//**************************************************//

	//icn85xx_ts_reset();
    if((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85) || (icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_86)) 
    {
	    while (retry-- && !need_update_fw) 
		{
		    icn85xx_ts_reset();
		    icn85xx_bootfrom_sram();
		    msleep(50);
		    ret = icn85xx_read_reg(0xa, &value);
		    if (ret > 0) 
			{ 
				need_update_fw = false;
				break; 
            }
	    }
		
        if (retry <= 0) need_update_fw = true;

	    if (need_update_fw) 
		{
		    if(R_OK == icn85xx_fw_update(firmware))
		    {
			    icn85xx_ts->code_loaded_flag = 1;
			    icn85xx_trace("ICN85XX_WITHOUT_FLASH, reload code ok\n"); 
		    }
		    else
		    {
			    icn85xx_ts->code_loaded_flag = 0;
			    icn85xx_trace("ICN85XX_WITHOUT_FLASH, reload code error\n"); 
		    }
	    }   
    }
    else if(icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)
    {
        while (retry-- && !need_update_fw) 
        {
            //icn85xx_ts_reset();
            icn85xx_set_prog_addr();
	     	//icn85xx_write_reg(ICN85xx_REG_PMODE, 0xff);
            icn87xx_boot_sram();
            msleep(50);
            ret = icn85xx_read_reg(0xa, &value);
            if (ret > 0) 
            { 
                need_update_fw = false;
                break; 
            }
        }
        if (retry <= 0) need_update_fw = true;

        if (need_update_fw) 
        {
            if(R_OK == icn87xx_fw_update(firmware))
            {
                icn85xx_ts->code_loaded_flag = 1;
                icn85xx_trace("ICN87XX_WITHOUT_FLASH, reload code ok\n"); 
            }
            else
            {
                icn85xx_ts->code_loaded_flag = 0;
                icn85xx_trace("ICN87XX_WITHOUT_FLASH, reload code error\n"); 
            }
        }           
    }
    else 
    {
    	icn85xx_write_reg(ICN85xx_REG_PMODE, 0xff);
	    //icn85xx_ts_reset();
    }

   icn85xx_ts->work_mode = 0;
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  

#if SUPPORT_CHECK_ESD
	icn85xx_ts->icn_is_suspend = 0;
	spin_lock(&icn85xx_ts->esd_lock);
	if (!icn85xx_ts->esd_running)
	{
		icn85xx_ts->esd_running = 1;
		spin_unlock(&icn85xx_ts->esd_lock);
		icn85xx_trace("Esd started");
		queue_delayed_work(icn85xx_esd_workqueue, &icn85xx_esd_event_work, icn85xx_ts->clk_tick_cnt);
	}
#endif

	icn85xx_trace("==icn85xx_ts_resume quit==\n");
	
	return retval;
}

static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
	icn85xx_trace("==icn85xx_ts_suspend enter==\n");
	int retval = TPD_OK;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(i2c_client);

//***********************************************//
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		return;
	}
#endif
//***********************************************//


#if SUPPORT_CHECK_ESD        
	if (icn85xx_ts->esd_running) 
	{                       
		icn85xx_ts->esd_running = 0;                       
		spin_unlock(&icn85xx_ts->esd_lock);                        
		icn85xx_trace("Esd cancelled");                       
		cancel_delayed_work_sync(&icn85xx_esd_event_work);
	} 
	else 
	{                        
		spin_unlock(&icn85xx_ts->esd_lock);
	}        
	icn85xx_ts->icn_is_suspend = 1; 
#endif

#ifdef TPD_SLIDE_WAKEUP
	suspend_status=1;

	if(icn85xx_ges_en)
	{
		icn85xx_write_reg(ICN85xx_REG_PMODE, GMODE_ENTER);
		//twd.suspended = true;
	}
	else
	{
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		icn85xx_write_reg(ICN85xx_REG_PMODE, PMODE_HIBERNATE);
	}
#else
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	icn85xx_write_reg(ICN85xx_REG_PMODE, PMODE_HIBERNATE);
#endif

	icn85xx_trace("==icn85xx_ts_suspend quit==\n");
	return retval;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = ICN85XX_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
#ifdef TPD_SLIDE_WAKEUP
	.attrs.attr =icn85xx_attr_list,
	.attrs.num = (int)(sizeof(icn85xx_attr_list)/sizeof(icn85xx_attr_list[0])),
#endif
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	i2c_register_board_info(TPD_I2C_NUMBER, &icn85xx_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		icn85xx_trace("add ICN85XX driver failed\n");
	icn85xx_trace("==tpd_driver_init==\n");
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
	icn85xx_trace("MediaTek ICN85XX touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

