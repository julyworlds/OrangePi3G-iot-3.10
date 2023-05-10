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
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <mach/upmu_common.h>

#include <mach/mt_gpio.h>		// For gpio control

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
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


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
static DEFINE_SPINLOCK(g_strobeSMPLock);
static u32 strobe_Res = 0;
static BOOL g_strobe_On = 0;
static int g_duty=-1;
static int g_step=-1;
static int g_timeOutTimeMs=0;

static struct work_struct workTimeOut;
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

#define GPIO_LEDS_STROBE
#ifdef GPIO_LEDS_STROBE
#define LEDS_TORCH_MODE 		0
#define LEDS_FLASH_MODE 		1
#define LEDS_CUSTOM_MODE_THRES 	20

#ifndef GPIO_CAMERA_FLASH_EN_PIN
#define GPIO_CAMERA_FLASH_EN_PIN GPIO140
#endif

#ifndef GPIO_CAMERA_FLASH_EN_PIN_M_GPIO
#define GPIO_CAMERA_FLASH_EN_PIN_M_GPIO GPIO_MODE_00
#endif

#ifndef GPIO_CAMERA_FLASH_MODE_PIN
#define GPIO_CAMERA_FLASH_MODE_PIN GPIO139
#endif

#ifndef GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO
#define GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO GPIO_MODE_00
#endif

int FL_enable(void)
{
	PK_DBG("FL_enable");
	//mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, 1);
	mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, 1);
	
#if 0
   upmu_set_rg_drv_2m_ck_pdn(0x0); // Disable power down (indicator no need?)     
   upmu_set_rg_drv_32k_ck_pdn(0x0); // Disable power down            
   upmu_set_isink_ch0_en(0x1); // Turn on ISINK Channel 0                  
   upmu_set_isink_ch1_en(0x1); // Turn on ISINK Channel 1          
   upmu_set_isink_ch2_en(0x1); // Turn on ISINK Channel 2  
#endif	

//	upmu_set_rg_bst_drv_1m_ck_pdn(0);
//	upmu_set_flash_en(1);
    return 0;
}

int FL_disable(void)
{
	PK_DBG("FL_disable");
	//mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, 0);
	mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, 0);
	
#if 0	//change by jst for flashlight-fpc     
	upmu_set_isink_ch0_en(0x0);
	upmu_set_isink_ch1_en(0x0);
	upmu_set_isink_ch2_en(0x0);
	//upmu_set_isink_ch3_en(0x0);
#endif
	
//	upmu_set_flash_en(0);
	//upmu_set_rg_bst_drv_1m_ck_pdn(1);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG("FL_dim_duty %d, thres %d", duty, LEDS_CUSTOM_MODE_THRES);
/* del by jst for cam flash light debug 2015-03-27 begin */
#if 0
	if(duty < LEDS_CUSTOM_MODE_THRES)
		mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, LEDS_TORCH_MODE);
	else
		mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, LEDS_FLASH_MODE);

	if((g_timeOutTimeMs == 0) && (duty > LEDS_CUSTOM_MODE_THRES))
	{
		PK_ERR("FL_dim_duty %d > thres %d, FLASH mode but timeout %d", duty, LEDS_CUSTOM_MODE_THRES, g_timeOutTimeMs);
		mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, LEDS_TORCH_MODE);
	}
#endif
/* del by jst for cam flash light debug 2015-03-27 begin */

//	upmu_set_flash_dim_duty(duty);
    return 0;
}

int FL_step(kal_uint32 step)
{
	int sTab[8]={0,2,4,6,9,11,13,15};
	PK_DBG("FL_step");
//	upmu_set_flash_sel(sTab[step]);
    return 0;
}

int FL_init(void)
{
//	upmu_set_flash_dim_duty(0);
//	upmu_set_flash_sel(0);
	PK_DBG("FL_init");

	//mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN, GPIO_CAMERA_FLASH_EN_PIN_M_GPIO);
	mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE_PIN, GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO);

#if 0
   upmu_set_rg_isink0_ck_pdn(0x0); // Disable power down    
   upmu_set_rg_isink0_ck_sel(0x0); // Freq = 32KHz for Indicator            
   upmu_set_isink_dim0_duty(15); // 16 / 32, no use for register mode
   upmu_set_isink_ch0_mode(2);
   upmu_set_isink_dim0_fsel(0x0); // 1KHz, no use for register mode
   upmu_set_isink_ch0_step(0x0); // 4mA
   upmu_set_isink_sfstr0_tc(0x0); // 0.5us
   upmu_set_isink_sfstr0_en(0x0); // Disable soft start
   upmu_set_rg_isink0_double_en(0x0); // Disable double current
   upmu_set_isink_phase0_dly_en(0x0); // Disable phase delay
   upmu_set_isink_chop0_en(0x0); // Disable CHOP clk

   upmu_set_rg_isink1_ck_pdn(0x0); // Disable power down    
   upmu_set_rg_isink1_ck_sel(0x0); // Freq = 32KHz for Indicator            
   upmu_set_isink_dim1_duty(15); // 16 / 32, no use for register mode
   upmu_set_isink_ch1_mode(2);
   upmu_set_isink_dim1_fsel(0x0); // 1KHz, no use for register mode
   upmu_set_isink_ch1_step(0x0); // 4mA
   upmu_set_isink_sfstr1_tc(0x0); // 0.5us
   upmu_set_isink_sfstr1_en(0x0); // Disable soft start
   upmu_set_rg_isink1_double_en(0x0); // Disable double current
   upmu_set_isink_phase1_dly_en(0x0); // Disable phase delay
   upmu_set_isink_chop1_en(0x0); // Disable CHOP clk

   upmu_set_rg_isink2_ck_pdn(0x0); // Disable power down    
   upmu_set_rg_isink2_ck_sel(0x0); // Freq = 32KHz for Indicator            
   upmu_set_isink_dim2_duty(15); // 16 / 32, no use for register mode
   upmu_set_isink_ch2_mode(2);
   upmu_set_isink_dim2_fsel(0x0); // 1KHz, no use for register mode
   upmu_set_isink_ch2_step(0x0); // 4mA
   upmu_set_isink_sfstr2_tc(0x0); // 0.5us
   upmu_set_isink_sfstr2_en(0x0); // Disable soft start
   upmu_set_rg_isink2_double_en(0x0); // Disable double current
   upmu_set_isink_phase2_dly_en(0x0); // Disable phase delay
   upmu_set_isink_chop2_en(0x0); // Disable CHOP clk
   /*
   upmu_set_rg_isink3_ck_pdn(0x0); // Disable power down    
   upmu_set_rg_isink3_ck_sel(0x0); // Freq = 32KHz for Indicator            
   upmu_set_isink_dim3_duty(15); // 16 / 32, no use for register mode
   upmu_set_isink_ch3_mode(2);
   upmu_set_isink_dim3_fsel(0x0); // 1KHz, no use for register mode
   upmu_set_isink_ch3_step(0x0); // 4mA
   upmu_set_isink_sfstr3_tc(0x0); // 0.5us
   upmu_set_isink_sfstr3_en(0x0); // Disable soft start
   upmu_set_rg_isink3_double_en(0x0); // Disable double current
   upmu_set_isink_phase3_dly_en(0x0); // Disable phase delay
   upmu_set_isink_chop3_en(0x0); // Disable CHOP clk
*/
   upmu_set_isink_ch0_en(0x0);
   upmu_set_isink_ch1_en(0x0);
   upmu_set_isink_ch2_en(0x0);
   //upmu_set_isink_ch3_en(0x0);

#endif

	FL_disable();
	INIT_WORK(&workTimeOut, work_timeOutFunc);
    return 0;
}

int FL_uninit(void)
{
	PK_DBG("FL_uninit");

	FL_disable();
    return 0;
}

#else 

int FL_enable(void)
{

   upmu_set_rg_drv_2m_ck_pdn(0x0); // Disable power down (indicator no need?)     
   upmu_set_rg_drv_32k_ck_pdn(0x0); // Disable power down            
   upmu_set_isink_ch0_en(0x1); // Turn on ISINK Channel 0                  
   upmu_set_isink_ch1_en(0x1); // Turn on ISINK Channel 1          
   upmu_set_isink_ch2_en(0x1); // Turn on ISINK Channel 2  
   //upmu_set_isink_ch3_en(0x1); // Turn on ISINK Channel 2    
   printk("FL_enable");        
            
    return 0;
}

int FL_disable(void)
{

upmu_set_isink_ch0_en(0x0);
upmu_set_isink_ch1_en(0x0);
upmu_set_isink_ch2_en(0x0);
//upmu_set_isink_ch3_en(0x0);

printk("FL_disable");

 return 0;
}

int FL_dim_duty(kal_uint32 duty)
{

    return 0;
}

int FL_step(kal_uint32 step)
{

    return 0;
}

int FL_init(void)
{
   
   upmu_set_rg_isink0_ck_pdn(0x0); // Disable power down    
   upmu_set_rg_isink0_ck_sel(0x0); // Freq = 32KHz for Indicator            
   upmu_set_isink_dim0_duty(15); // 16 / 32, no use for register mode
   upmu_set_isink_ch0_mode(2);
   upmu_set_isink_dim0_fsel(0x0); // 1KHz, no use for register mode
   upmu_set_isink_ch0_step(0x0); // 4mA
   upmu_set_isink_sfstr0_tc(0x0); // 0.5us
   upmu_set_isink_sfstr0_en(0x0); // Disable soft start
   upmu_set_rg_isink0_double_en(0x0); // Disable double current
   upmu_set_isink_phase0_dly_en(0x0); // Disable phase delay
   upmu_set_isink_chop0_en(0x0); // Disable CHOP clk

   upmu_set_rg_isink1_ck_pdn(0x0); // Disable power down    
   upmu_set_rg_isink1_ck_sel(0x0); // Freq = 32KHz for Indicator            
   upmu_set_isink_dim1_duty(15); // 16 / 32, no use for register mode
   upmu_set_isink_ch1_mode(2);
   upmu_set_isink_dim1_fsel(0x0); // 1KHz, no use for register mode
   upmu_set_isink_ch1_step(0x0); // 4mA
   upmu_set_isink_sfstr1_tc(0x0); // 0.5us
   upmu_set_isink_sfstr1_en(0x0); // Disable soft start
   upmu_set_rg_isink1_double_en(0x0); // Disable double current
   upmu_set_isink_phase1_dly_en(0x0); // Disable phase delay
   upmu_set_isink_chop1_en(0x0); // Disable CHOP clk

   upmu_set_rg_isink2_ck_pdn(0x0); // Disable power down    
   upmu_set_rg_isink2_ck_sel(0x0); // Freq = 32KHz for Indicator            
   upmu_set_isink_dim2_duty(15); // 16 / 32, no use for register mode
   upmu_set_isink_ch2_mode(2);
   upmu_set_isink_dim2_fsel(0x0); // 1KHz, no use for register mode
   upmu_set_isink_ch2_step(0x0); // 4mA
   upmu_set_isink_sfstr2_tc(0x0); // 0.5us
   upmu_set_isink_sfstr2_en(0x0); // Disable soft start
   upmu_set_rg_isink2_double_en(0x0); // Disable double current
   upmu_set_isink_phase2_dly_en(0x0); // Disable phase delay
   upmu_set_isink_chop2_en(0x0); // Disable CHOP clk
   /*
   upmu_set_rg_isink3_ck_pdn(0x0); // Disable power down    
   upmu_set_rg_isink3_ck_sel(0x0); // Freq = 32KHz for Indicator            
   upmu_set_isink_dim3_duty(15); // 16 / 32, no use for register mode
   upmu_set_isink_ch3_mode(2);
   upmu_set_isink_dim3_fsel(0x0); // 1KHz, no use for register mode
   upmu_set_isink_ch3_step(0x0); // 4mA
   upmu_set_isink_sfstr3_tc(0x0); // 0.5us
   upmu_set_isink_sfstr3_en(0x0); // Disable soft start
   upmu_set_rg_isink3_double_en(0x0); // Disable double current
   upmu_set_isink_phase3_dly_en(0x0); // Disable phase delay
   upmu_set_isink_chop3_en(0x0); // Disable CHOP clk
*/
   upmu_set_isink_ch0_en(0x0);
   upmu_set_isink_ch1_en(0x0);
   upmu_set_isink_ch2_en(0x0);
   //upmu_set_isink_ch3_en(0x0);

    INIT_WORK(&workTimeOut, work_timeOutFunc);

   printk("FL_init");
            
    return 0;
}


int FL_uninit(void)
{
	FL_disable();
    return 0;
}

#endif
/*****************************************************************************
User interface
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data)
{
	FL_disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}
enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	PK_DBG("ledTimeOut_callback\n");
	schedule_work(&workTimeOut);

    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}

static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int iFlashType = (int)FLASHLIGHT_NONE;
	int ior;
	int iow;
	int iowr;
	ior = _IOR(FLASHLIGHT_MAGIC,0, int);
	iow = _IOW(FLASHLIGHT_MAGIC,0, int);
	iowr = _IOWR(FLASHLIGHT_MAGIC,0, int);
	PK_DBG("constant_flashlight_ioctl() line=%d cmd=%d, ior=%d, iow=%d iowr=%d arg=%d\n",__LINE__, cmd, ior, iow, iowr, arg);
	PK_DBG("constant_flashlight_ioctl() line=%d cmd-ior=%d, cmd-iow=%d cmd-iowr=%d arg=%d\n",__LINE__, cmd-ior, cmd-iow, cmd-iowr, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
			break;

    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		g_duty=arg;
    		FL_dim_duty(arg);
    		break;

    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);
    		g_step=arg;
    		FL_step(arg);
    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_enable();
    			g_strobe_On=1;
    		}
    		else
    		{
    			FL_disable();
				hrtimer_cancel( &g_timeOutTimer );
				g_strobe_On=0;
    		}
    		break;
        case FLASHLIGHTIOC_G_FLASHTYPE:
            iFlashType = FLASHLIGHT_LED_CONSTANT;
            if(copy_to_user((void __user *) arg , (void*)&iFlashType , _IOC_SIZE(cmd)))
            {
                PK_DBG("[strobe_ioctl] ioctl copy to user failed\n");
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
	    FL_init();
		timerInit();
	}
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

    return i4RetValue;

}

static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_uninit();
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


#if 1//add by jst for test start
struct class *main_flashlight_class;
struct device *main_flashlight_dev;
static ssize_t main_flashlight_enable_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int enable = 0;
    if(buf != NULL && size != 0)
    {
        enable = (int)simple_strtoul(buf, NULL, 0);
    }
    if (enable)
    {
        FL_init();
        mdelay(10);
        FL_enable();
    }
    else
    {
        FL_disable();
    }
    return size;
}
static DEVICE_ATTR(main_flashlight_enable, 0777, NULL, main_flashlight_enable_store);
static int __init main_flashlight_init(void)  
{		
		main_flashlight_class = class_create(THIS_MODULE, "main_flashlight");
		main_flashlight_dev = device_create(main_flashlight_class,NULL, 0, NULL,  "main_flashlight");
		device_create_file(main_flashlight_dev, &dev_attr_main_flashlight_enable);
		
		return 0;
}
static void __exit main_flashlight_exit(void)
{
		return;
}
module_init(main_flashlight_init);
module_exit(main_flashlight_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("main_flashlight");
MODULE_AUTHOR("jst <aren.jiang@runyee.com.cn>");
#endif//add by jst for test end
