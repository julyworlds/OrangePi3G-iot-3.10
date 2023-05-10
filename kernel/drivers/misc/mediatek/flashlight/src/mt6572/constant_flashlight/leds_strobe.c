#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif



/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=0;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0xC6


static struct work_struct workTimeOut;

//#define FLASH_GPIO_ENF GPIO12
//#define FLASH_GPIO_ENT GPIO13

static int g_bLtVersion=0;

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

int readReg(int reg)
{
    char buf[2];
    char bufR[2];
    buf[0]=reg;
    iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
    //PK_DBG("qq reg=%d val=%d qq\n", buf[0],bufR[0]);
    return (int)bufR[0];
}

int FL_Enable(void)
{
	char buf[2];
//	char bufR[2];
    if(g_duty<0)
        g_duty=0;
    else if(g_duty>16)
        g_duty=16;
  if(g_duty<=2)
  {
    int val;
        if(g_bLtVersion==1)
        {
            if(g_duty==0)
                val=3;
            else if(g_duty==1)
                val=5;
            else //if(g_duty==2)
                val=7;
        }
        else
        {
            if(g_duty==0)
                val=1;
            else if(g_duty==1)
                val=2;
            else //if(g_duty==2)
                val=3;
        }
    buf[0]=9;
        buf[1]=val<<4;
	  iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);

	  buf[0]=10;
	  buf[1]=0x02;
	  iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
  }
  else
  {
    int val;
    val = (g_duty-1);
    buf[0]=9;
	  buf[1]=val;
	  iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);

	  buf[0]=10;
	  buf[1]=0x03;
	  iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
  }
	PK_DBG(" FL_Enable line=%d\n",__LINE__);

    readReg(0);
	readReg(1);
	readReg(6);
	readReg(8);
	readReg(9);
	readReg(0xa);
	readReg(0xb);

    return 0;
}



int FL_Disable(void)
{
		char buf[2];

///////////////////////
	buf[0]=10;
	buf[1]=0x00;
	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
	g_duty = duty;
    return 0;
}


int FL_Init(void)
{
   int regVal0;
  char buf[2];

  buf[0]=0xa;
	buf[1]=0x0;
	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);

	buf[0]=0x8;
	buf[1]=0x47;
	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);

	buf[0]=9;
	buf[1]=0x35;
	iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);





	regVal0 = readReg(0);

	if(regVal0==1)
	    g_bLtVersion=1;
	else
	    g_bLtVersion=0;


    PK_DBG(" FL_Init regVal0=%d isLtVer=%d\n",regVal0, g_bLtVersion);


/*
	if(mt_set_gpio_mode(FLASH_GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

    	if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
    */




    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
    return 0;
}


static int FL_getErr(int* err)
{
    int reg;
    int reg2;
    *err = readReg(0x0b);

    reg = readReg(0x08);
    reg2 = readReg(0x09);
    PK_DBG(" FL_getErr line=%d %d\n",reg,reg2);
    return 0;
}
/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
    int tempVal;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
        case FLASH_IOC_GET_ERR:
            FL_getErr(&tempVal);
            if(copy_to_user((void __user *) arg , (void*)&tempVal , 4))
            {
                PK_DBG(" ioctl copy to user failed\n");
                return -EFAULT;
            }

            break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


