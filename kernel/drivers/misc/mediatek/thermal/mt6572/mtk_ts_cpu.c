/*
* Copyright (C) 2011-2014 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/aee.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include<mach/sync_write.h>
#include <mach/mt_irq.h>
#include "mach/mtk_thermal_monitor.h"
#include <mach/system.h>
#include <mach/mt_clkmgr.h>
//#include "mach/mtk_cpu_management.h"

#include "mach/mt_typedefs.h"
#include "mach/mt_thermal.h"
#include "mach/mt_gpufreq.h"


#define THERMAL_NAME    "mtk-thermal"
#define THERMAL_MTK_ABB_SUPPORT
#define MTK_TZ_COOLER_MAX 10
#define MTK_THERMAL_HW_PROTECT

/* Cooler */
#define CPU_COOLER_COUNT  15
static unsigned int cl_dev_sysrst_state=0;
static unsigned int cl_dvfs_cooler_cnt=0;

typedef struct
{
   char cooler_name[5];
   unsigned int cl_dev_state;
   unsigned int cl_limit;
   struct thermal_cooling_device *cl_dev;
} mtk_cpu_cooler_dev;

static int cpu_cooler_limit = 0;
static mtk_cpu_cooler_dev mtk_cl_dev[CPU_COOLER_COUNT];
//static unsigned int cl_dev_state[CPU_COOLER_COUNT]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//static char cooler_name[CPU_COOLER_COUNT][5] = {"1","2","3","4","5","6","7","8","9","10",};
//static char cooler_name[CPU_COOLER_COUNT][5] = {"600","630","640","700","760","800","850","900","980","1055"};
//static struct thermal_cooling_device *cl_dev[CPU_COOLER_COUNT];

static struct thermal_cooling_device *cl_dev_sysrst;

/* Zone */
static struct thermal_zone_device *thz_dev;
static unsigned int interval = 0; /* seconds, 0 : no auto polling */
static unsigned int trip_temp[MTK_TZ_COOLER_MAX] = {120000,110000,100000,90000,80000,70000,65000,60000,55000,50000};
static int g_THERMAL_TRIP[MTK_TZ_COOLER_MAX] = {0,0,0,0,0,0,0,0,0,0};
static int num_trip=0;
static int MA_len_temp=0;
static char g_bind[MTK_TZ_COOLER_MAX][THERMAL_NAME_LENGTH] = {{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}};
static int kernelmode = 0;
#define MTKTSCPU_TEMP_CRIT 120000 /* 120.000 degree Celsius */

/* Logging */
static int mtktscpu_debug_log = 0;
#define mtktscpu_dprintk(fmt, args...)   \
do {                                    \
	if (mtktscpu_debug_log) {                \
		pr_debug("Power/CPU_Thermal " fmt, ##args); \
	}                                   \
} while(0)

/* Cali */
static kal_int32 g_adc_ge;
static kal_int32 g_adc_oe;
static kal_int32 g_o_vtsmcu1;
static kal_int32 g_o_vtsabb;
static kal_int32 g_degc_cali;
static kal_int32 g_adc_cali_en;
static kal_int32 g_o_slope;
static kal_int32 g_o_slope_sign;
static kal_int32 g_id;
static kal_int32 g_ge;
static kal_int32 g_oe;
static kal_int32 g_gain;
static kal_int32 g_x_roomtabb = 0;
static kal_int32 g_x_roomtmcu = 0;
static kal_int32 g_ther_ver;
static kal_int32 g_slope1;
static kal_int32 g_slope2;

static kal_int32 g_intercept_mcu;
static kal_int32 g_intercept_abb;


/* CPU Info */
struct mtk_cpu_power_info
{
	unsigned int cpufreq_khz;
	unsigned int cpufreq_ncpu;
	unsigned int cpufreq_power;
};
struct mtk_gpu_power_info
{
	unsigned int gpufreq_khz;
	unsigned int gpufreq_power;
};



static bool talking_flag=false;
static kal_uint32 cl_dvfs_thermal_limit = 0;

static kal_int32 raw_to_temperature_mcu(kal_uint32 ret);
static kal_int32 temperature_to_raw_mcu(kal_int32 ret);
static int mtkts_match(struct thermal_cooling_device *cdev, char bind[MTK_TZ_COOLER_MAX][THERMAL_NAME_LENGTH]);
static int mtktscpu_register_thermal(void);
static void mtktscpu_unregister_thermal(void);

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int IMM_IsAdcInitReady(void);
extern void set_dvfs_thermal_limit(unsigned int limited_power);
extern u32 get_devinfo_with_index(u32 index);
extern struct proc_dir_entry * mtk_thermal_get_proc_drv_therm_dir_entry(void);

#if defined(THERMAL_MTK_ABB_SUPPORT)
static kal_int32 raw_to_temperature_abb(kal_uint32 ret);
#if 0
static kal_int32 temperature_to_raw_abb(kal_int32 ret);
#endif
extern int __init mtktsabb_init(void);
extern void __exit mtktsabb_exit(void);
extern	int mtktsabb_register_thermal(void);
extern	void mtktsabb_unregister_thermal(void);
#endif

void set_taklking_flag(bool flag)
{
	talking_flag = flag;
	printk("Power/CPU_Thermal: talking_flag=%d", talking_flag);
	return;
}

void get_thermal_slope_intercept(struct TS_PTPOD *ts_info)
{
	unsigned int oMTS, oBTS;

	oMTS = (-1) * (g_slope1 * 4096) / (g_slope2 * 1000);
	oBTS = ((g_intercept_mcu-25000)*4)/1000;

	//ts_info = &ts_ptpod;
	ts_info->ts_MTS = oMTS;
	ts_info->ts_BTS = oBTS;
	printk("ts_MTS=%d, ts_BTS=%d\n",oMTS, oBTS);
}
EXPORT_SYMBOL(get_thermal_slope_intercept);

static irqreturn_t thermal_interrupt_handler(int irq, void *dev_id)
{

	kal_uint32 ret = 0;
//	int i=0;
	ret = DRV_Reg32(TEMPMONINTSTS);
#if 0
	mtktscpu_dprintk("thermal_isr: [Interrupt trigger]: status = 0x%x\n", ret);
	if (ret & THERMAL_MON_CINTSTS0)
	{
		mtktscpu_dprintk("thermal_isr: thermal sensor point 0 - cold interrupt trigger\n");

	}
	if (ret & THERMAL_MON_HINTSTS0)
	{
		mtktscpu_dprintk("thermal_isr: thermal sensor point 0 - hot interrupt trigger\n");
	}

	if(ret & THERMAL_tri_SPM_State0)
		mtktscpu_dprintk("thermal_isr: Thermal state0 to trigger SPM state0 \n");
	if(ret & THERMAL_tri_SPM_State1)
		mtktscpu_dprintk("thermal_isr: Thermal state1 to trigger SPM state1\n");
	if(ret & THERMAL_tri_SPM_State2)
		mtktscpu_dprintk("thermal_isr: Thermal state2 to trigger SPM state2\n");
#endif
	return IRQ_HANDLED;
}

static void thermal_reset_and_initial(void)
{
	UINT32 tcon0_val;
	mtktscpu_dprintk("[Reset and init thermal controller]\n");
	enable_clock(MT_CG_AUX_SW_CG_ADC,"THM"); // enable aux_adc module
	enable_clock(MT_CG_THEM_SW_CG,"THM");
	tcon0_val = (DRV_Reg32(TS_CON0) & ~(0xC0));
	THERMAL_WRAP_WR32(0x00000000, TEMPMONCTL0);    // disable periodoc temperature sensing point 0
	THERMAL_WRAP_WR32(tcon0_val,  TS_CON0);
	THERMAL_WRAP_WR32(0x000003FF, TEMPMONCTL1);	// bus clock 66M counting unit is 1024 * 15.15ns
	THERMAL_WRAP_WR32(0x03FF0000, TEMPMONCTL2);	// bus clock 66M sensing interval is 1024 * 16us= 16ms
	THERMAL_WRAP_WR32(0x00E00000, TEMPAHBPOLL);	// bus clock 66M total update time = 220ms
	THERMAL_WRAP_WR32(0x000E0000, TEMPAHBTO);		// AHBPOLLTIMEOUT.
	THERMAL_WRAP_WR32(0x00000000, TEMPMONIDET0);	// times for interrupt occurrance
	THERMAL_WRAP_WR32(0x00000000, TEMPMONIDET1);	// times for interrupt occurrance
	THERMAL_WRAP_WR32(0x00000000, TEMPMSRCTL0);	// temperature sampling control, one sample
	THERMAL_WRAP_WR32(0x800, TEMPADCMUX);			// this value will be stored to TEMPADCMUXADDR automatically by hw
	THERMAL_WRAP_WR32((UINT32) AUXADC_CON1_CLR_P, TEMPADCMUXADDR);	// AHB address for auxadc mux selection
	THERMAL_WRAP_WR32(0x800, TEMPADCEN);							// AHB value for auxadc enable
	THERMAL_WRAP_WR32((UINT32) AUXADC_CON1_SET_P, TEMPADCENADDR);	// this value will be stored to TEMPADCENADDR automatically by hw
	THERMAL_WRAP_WR32(tcon0_val,TEMPADCPNP0);                       // this value will be stored to TEMPPNPMUXADDR (TEMPSPARE0) automatically by hw
	#if defined(THERMAL_MTK_ABB_SUPPORT)
		THERMAL_WRAP_WR32((tcon0_val | 0x40),TEMPADCPNP1);                       // this value will be stored to TEMPPNPMUXADDR (TEMPSPARE0) automatically by hw
	#endif
	THERMAL_WRAP_WR32((UINT32) TS_CON0_P,TEMPPNPMUXADDR);    // AHB address for pnp sensor mux selection
	THERMAL_WRAP_WR32((UINT32) AUXADC_DAT11_P, TEMPADCVALIDADDR);	// AHB address for auxadc valid bit
	THERMAL_WRAP_WR32((UINT32) AUXADC_DAT11_P, TEMPADCVOLTADDR);	// AHB address for auxadc voltage output
	THERMAL_WRAP_WR32(0x0, TEMPRDCTRL);							// read valid & voltage are at the same register
	THERMAL_WRAP_WR32(0x0000002C, TEMPADCVALIDMASK);				// indicate where the valid bit is (the 12th bit is valid bit and 1 is valid)
	THERMAL_WRAP_WR32(0x0, TEMPADCVOLTAGESHIFT);					// data do not need to shift
	THERMAL_WRAP_WR32(0x3, TEMPADCWRITECTRL);						// enable auxadc mux write transaction
	#if defined(THERMAL_MTK_ABB_SUPPORT)
		THERMAL_WRAP_WR32(0x00000003, TEMPMONCTL0);    // enable periodoc temperature sensing point 0
	#else
		THERMAL_WRAP_WR32(0x00000001, TEMPMONCTL0);    // enable periodoc temperature sensing point 0
	#endif
	//THERMAL_WRAP_WR32(0x0000FFFF, TEMPMONINT);		// enable all interrupt
}

static void set_thermal_ctrl_trigger_SPM(int temperature)
{
#if defined(MTK_THERMAL_HW_PROTECT)
	int temp = 0;
	int raw_high, raw_middle, raw_low;

	mtktscpu_dprintk("[Set_thermal_ctrl_trigger_SPM]: temperature=%d\n", temperature);

	//temperature to trigger SPM state
	raw_high	= temperature_to_raw_mcu(temperature);
	raw_middle	= temperature_to_raw_mcu(20000);
	raw_low		= temperature_to_raw_mcu(5000);

	temp = DRV_Reg32(TEMPMONINT);
	THERMAL_WRAP_WR32(temp & 0x1FFFFFFF, TEMPMONINT);	// disable trigger SPM interrupt

	THERMAL_WRAP_WR32(0x20000, TEMPPROTCTL); //monitor  sensor 0
	THERMAL_WRAP_WR32(raw_low, TEMPPROTTA);
	THERMAL_WRAP_WR32(raw_middle, TEMPPROTTB);
	THERMAL_WRAP_WR32(raw_high, TEMPPROTTC);

	THERMAL_WRAP_WR32(temp | 0x80000000, TEMPMONINT);	// enable trigger SPM interrupt
#endif
}




static void thermal_cal_prepare(void)
{
	kal_uint32 efuse_reg0, efuse_reg1;

	efuse_reg0 = get_devinfo_with_index(15);
	efuse_reg1 = get_devinfo_with_index(16);


	g_ther_ver = (efuse_reg1 >> 10) & 0xF;
	if(g_ther_ver == 0x0000)
	{
		g_adc_oe = 		(efuse_reg1 & 0x03FF0000)>>16;
		g_adc_cali_en = (efuse_reg1 & 0x80000000)>>31;
	}
	else if(g_ther_ver == 0x0001)
	{
		g_adc_oe = 		(efuse_reg1 & 0x00FFC000)>>14;
		g_adc_cali_en = (efuse_reg1 & 0x01000000)>>24;
	}
	else
	{
		g_adc_cali_en = 0;
	}

	g_adc_ge =		(efuse_reg1 & 0x000003FF);
	g_o_vtsabb =	(efuse_reg0 & 0x000001FF);
	g_id=			(efuse_reg0 & 0x00000200)>>9;
	g_degc_cali = 	(efuse_reg0 & 0x0000FC00)>>10;
	g_o_vtsmcu1 = 	(efuse_reg0 & 0x01FF0000)>>16;
	g_o_slope_sign =(efuse_reg0 & 0x02000000)>>25;

	if(g_id==0)
	{
		g_o_slope=0;
	}
	else
	{
		g_o_slope = (efuse_reg0 & 0xFC000000)>>26;
	}

	if(g_adc_cali_en == 0)
	{
		g_o_slope=0;
		g_adc_ge = 512;
		g_adc_oe = 512;
		g_degc_cali = 40;
		g_o_slope = 0;
		g_o_slope_sign = 0;
		g_o_vtsmcu1 = 260;
		g_o_vtsabb = 260;
	}
	printk("[Power/CPU_Thermal] [Thermal calibration] g_adc_ge = 0x%x, g_adc_oe = 0x%x, g_degc_cali = 0x%x, g_adc_cali_en = 0x%x, g_o_slope = 0x%x, g_o_slope_sign = 0x%x, g_id = 0x%x\n",
		g_adc_ge, g_adc_oe, g_degc_cali, g_adc_cali_en, g_o_slope, g_o_slope_sign, g_id);
	printk("[Power/CPU_Thermal] [Thermal calibration] g_o_vtsmcu1 = 0x%x, g_o_vtsabb = 0x%x g_ther_ver = 0x%x\n",
		g_o_vtsmcu1, g_o_vtsabb,g_ther_ver);
}

static void thermal_cal_prepare_2(void)
{
	g_ge = (g_adc_ge - 512);
	g_oe = (g_adc_oe - 512);
	g_gain = 4096 + g_ge;

	g_x_roomtmcu = (g_o_vtsmcu1  + 3350 - g_oe);
	g_x_roomtabb = (g_o_vtsabb + 3350 - g_oe);

	g_slope1 = (100000 * 1000 * 15) /  ((g_gain) * 18);	//1000 is for 0.001 degree
	if(g_o_slope_sign==0)
	{
		g_slope2 = -(165+g_o_slope);
	}
	else
	{
		g_slope2 = -(165-g_o_slope);
	}

	g_intercept_abb = ((-g_oe - g_x_roomtabb) * 15 * 10000) / (g_gain * 18); // if multiple 100000 it will over flow
	if(g_o_slope_sign==0)
	{
		g_intercept_abb = (g_intercept_abb*10000) / -(165+g_o_slope); 	//0.001 degree
	}
	else
	{
		g_intercept_abb = (g_intercept_abb*10000) / -(165-g_o_slope);  //0.001 degree
	}
	g_intercept_abb = g_intercept_abb + (g_degc_cali*(1000/2)); // 1000 is for 0.1 degree

	g_intercept_mcu = ((-g_oe - g_x_roomtmcu) * 15 * 10000) / (g_gain * 18); // if multiple 100000 it will over flow
	if(g_o_slope_sign==0)
	{
		g_intercept_mcu = (g_intercept_mcu*10000) / -(165+g_o_slope);	//0.001 degree
	}
	else
	{
		g_intercept_mcu = (g_intercept_mcu*10000) / -(165-g_o_slope);  //0.001 degree
	}
	g_intercept_mcu = g_intercept_mcu + (g_degc_cali*(1000/2)); // 1000 is for 0.1 degree

	printk("[Power/CPU_Thermal] [Thermal cali] SLOPE1=%d SLOPE2=%d INTERCEPT1=%d INTERCEPT2=%d=\n",
		g_slope1, g_slope2, g_intercept_mcu,g_intercept_abb);
}


static kal_int32 temperature_to_raw_mcu(kal_int32 ret)
{
	kal_int32 format_4 = 0;
	kal_int32 t_curr = ret;
	format_4 = ((t_curr-(g_intercept_mcu)) * g_slope2) / (g_slope1);
	mtktscpu_dprintk("[temperature_to_raw_mcu] temperature=%d, raw=%d", ret, format_4);
	return format_4;
}

static kal_int32 raw_to_temperature_mcu(kal_uint32 ret)
{
	kal_int32 y_curr = ret;
	kal_int32 t_current = 0;

	if(y_curr == 0)
	{
		return 0;
	}

	t_current = g_intercept_mcu + ((g_slope1 * y_curr) / (g_slope2));
	return t_current;
}
#if defined(THERMAL_MTK_ABB_SUPPORT)
#if 0
static kal_int32 temperature_to_raw_abb(kal_int32 ret)
{
	kal_int32 format_4 = 0;
	kal_int32 t_curr = ret;
	format_4 = ((t_curr-(g_intercept_abb)) * g_slope2) / (g_slope1);
	mtktscpu_dprintk("[temperature_to_raw_abb] temperature=%d, raw=%d", ret, format_4);
	return format_4;
}
#endif

static kal_int32 raw_to_temperature_abb(kal_uint32 ret)
{
	kal_int32 y_curr = ret;
	kal_int32 t_current = 0;

	if(y_curr == 0)
	{
		return 0;
	}

	t_current = g_intercept_abb + ((g_slope1 * y_curr) / (g_slope2));
	return t_current;
}
#endif

#if 0
static int thermal_auxadc_get_data(int times, int Channel)
{
	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0;

	if( IMM_IsAdcInitReady() == 0 )
	{
//        mtktscpu_dprintk("[thermal_auxadc_get_data]: AUXADC is not ready\n");
		return 0;
	}

	i = times;
	while (i--)
	{
		ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
		ret += ret_temp;
//		mtktscpu_dprintk("[thermal_auxadc_get_data(ADCIN5)]: ret_temp=%d\n",ret_temp);
	}

	ret = ret / times;
	return ret;
}
#endif

static int get_immediate_temp(void)
{
	int curr_raw, curr_temp;

	curr_raw = DRV_Reg32(TEMPMSR0);
	curr_raw = curr_raw & 0x0fff;
	curr_temp = raw_to_temperature_mcu(curr_raw);

	mtktscpu_dprintk("[get_mcu_temp] temp=%d, rawdata = %d\n", curr_temp,curr_raw);
	return curr_temp;
}


static int mtktscpu_get_temp(struct thermal_zone_device *thermal,
							unsigned long *t)
{
#if defined(MTK_THERMAL_HW_PROTECT)
	int curr_temp;

	curr_temp = get_immediate_temp(); //mtktscpu_get_TC_temp();
//	if((curr_temp>100000) || (curr_temp<-30000))
	if((curr_temp > (trip_temp[0] - 15000)) || (curr_temp<-30000))
		printk("[Power/CPU_Thermal] CPU T=%d\n",curr_temp);

	//curr_temp = mtktscpu_get_hw_temp();
	*t = (unsigned long) curr_temp;
	return 0;
#else
  int curr_temp;
  int temp_temp;
  int ret = 0;
  static int last_cpu_temp = 0;


  curr_temp = get_immediate_temp(); //mtktscpu_get_TC_temp();
	//	if((curr_temp>100000) || (curr_temp<-30000))
  if((curr_temp > (trip_temp[0] - 15000)) || (curr_temp<-30000))
	printk("[Power/CPU_Thermal] CPU T=%d\n",curr_temp);


	temp_temp = curr_temp;
	if(curr_temp != 0)
	{
		if((curr_temp > 150000) || (curr_temp < -20000))
		{
			printk("[Power/CPU_Thermal] CPU temp invalid = %d\n", curr_temp);
			temp_temp = 50000;
			ret = -1;
		}
		else if(last_cpu_temp!=0)
		{
			if ((curr_temp - last_cpu_temp > 20000) ||(last_cpu_temp - curr_temp > 20000)) //delta 20C
			{
				printk("[Power/CPU_Thermal] CPU temp float hugely temp=%d, lasttemp =%d\n", curr_temp, last_cpu_temp);
				temp_temp = 50000;
				ret = -1;
			}
		}
	}

	last_cpu_temp = curr_temp;
	curr_temp = temp_temp;
	*t = (unsigned long) curr_temp;
	return ret;
#endif
}

static int mtkts_match(struct thermal_cooling_device *cdev, char bind[MTK_TZ_COOLER_MAX][THERMAL_NAME_LENGTH])
{
	int i;

	for(i=0;i<MTK_TZ_COOLER_MAX;i++)
	{
		if(!strcmp(cdev->type, bind[i]))
		{
			return i;
		}
	}
	return i;
}

static int mtktscpu_bind(struct thermal_zone_device *thermal,
						struct thermal_cooling_device *cdev)
{
	int table_val=0;
	table_val = mtkts_match(cdev,g_bind);
	if(table_val > 9 )
	{
		return 0;
	}
	else
	{
		if(table_val == 0)
		{
			set_thermal_ctrl_trigger_SPM(trip_temp[0]);
		}
		mtktscpu_dprintk("[mtktscpu_bind] %s\n", cdev->type);
		if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) {
			mtktscpu_dprintk("[mtktscpu_bind] error binding cooling dev\n");
			return -EINVAL;
		} else {
			mtktscpu_dprintk("[mtktscpu_bind] binding OK, %d\n", table_val);
		}
	}
	return 0;
}



static int mtktscpu_unbind(struct thermal_zone_device *thermal,
						struct thermal_cooling_device *cdev)
{
	int table_val=0;
	table_val = mtkts_match(cdev,g_bind);
	if(table_val > 9 )
	{
		return 0;
	}
	else
	{
		mtktscpu_dprintk("[mtktscpu_unbind] %s\n", cdev->type);
		if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) {
			mtktscpu_dprintk("[mtktscpu_unbind] error unbinding cooling dev\n");
			return -EINVAL;
		} else {
			mtktscpu_dprintk("[mtktscpu_unbind] unbinding OK, %d\n", table_val);
		}
	}
	return 0;
}

static int mtktscpu_get_mode(struct thermal_zone_device *thermal,
							enum thermal_device_mode *mode)
{
	*mode = (kernelmode) ? THERMAL_DEVICE_ENABLED
		: THERMAL_DEVICE_DISABLED;
	return 0;
}

static int mtktscpu_set_mode(struct thermal_zone_device *thermal,
							enum thermal_device_mode mode)
{
	kernelmode = mode;
	return 0;
}

static int mtktscpu_get_trip_type(struct thermal_zone_device *thermal, int trip,
								enum thermal_trip_type *type)
{
	*type = g_THERMAL_TRIP[trip];
	return 0;
}

static int mtktscpu_get_trip_temp(struct thermal_zone_device *thermal, int trip,
								unsigned long *temp)
{
	*temp = trip_temp[trip];
	return 0;
}

static int mtktscpu_get_crit_temp(struct thermal_zone_device *thermal,
								unsigned long *temperature)
{
	*temperature = MTKTSCPU_TEMP_CRIT;
	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops mtktscpu_dev_ops = {
	.bind = mtktscpu_bind,
	.unbind = mtktscpu_unbind,
	.get_temp = mtktscpu_get_temp,
	.get_mode = mtktscpu_get_mode,
	.set_mode = mtktscpu_set_mode,
	.get_trip_type = mtktscpu_get_trip_type,
	.get_trip_temp = mtktscpu_get_trip_temp,
	.get_crit_temp = mtktscpu_get_crit_temp,
};

static int mtktscpu_set_power_consumption_state(void)
{
  int i;
  int min_limit = 0;

  for (i = 0; i < cl_dvfs_cooler_cnt; i++)
  {
	if(mtk_cl_dev[i].cl_dev)
	{
		if (mtk_cl_dev[i].cl_dev_state == 1)
		{
		  int cl_limit =  mtk_cl_dev[i].cl_limit;

		  if(min_limit == 0)
		  {
			min_limit = cl_limit;
		  }
		  else
		  {
			if ((min_limit > cl_limit) && (cl_limit > 0))
			  min_limit = cl_limit;
		  }
		}
	}
  }

//  if((min_limit != cl_dvfs_thermal_limit) || (min_limit !=0))
  if(min_limit != cl_dvfs_thermal_limit)
  {
	printk("[mtktscpu] set_power limit =  %d\n", min_limit);
	set_dvfs_thermal_limit(min_limit);
  }

  cl_dvfs_thermal_limit = min_limit;
  return 0;
}


static int cpucooler_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	*state = 1;
	return 0;
}
static int cpucooler_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	mtk_cpu_cooler_dev *priv = (mtk_cpu_cooler_dev*)cdev->devdata;
	*state = priv->cl_dev_state ;
	mtktscpu_dprintk("get_cur_state = %s %d %d\n", cdev->type,priv->cl_dev_state,priv->cl_limit);
	return 0;
}
static int cpucooler_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state)
{
	mtk_cpu_cooler_dev *priv = (mtk_cpu_cooler_dev*)cdev->devdata;
	priv->cl_dev_state = state;
	mtktscpu_set_power_consumption_state();
	mtktscpu_dprintk("set_cur_state = %s %d %d\n", cdev->type,priv->cl_dev_state,priv->cl_limit);

	return 0;
}



/*
 * cooling device callback functions (mtktscpu_cooling_sysrst_ops)
 * 1 : ON and 0 : OFF
 */
static int mtktscpu_sysrst_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	mtktscpu_dprintk("sysrst_get_max_state\n");
	*state = 1;
	return 0;
}
static int mtktscpu_sysrst_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	mtktscpu_dprintk("sysrst_get_cur_state\n");
	*state = cl_dev_sysrst_state;
	return 0;
}
static int mtktscpu_sysrst_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
	mtktscpu_dprintk("sysrst_set_cur_state\n");
	cl_dev_sysrst_state = state;
	if(cl_dev_sysrst_state == 1)
	{
		printk("Power/CPU_Thermal: reset, reset, reset!!!\n");
//		printk("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
//		printk("*****************************************");
//		printk("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");

		BUG();
		//arch_reset(0,NULL);
	}
	return 0;
}

/* bind fan callbacks to fan device */

static struct thermal_cooling_device_ops mtktscpu_cooling_ops = {
	.get_max_state = cpucooler_get_max_state,
	.get_cur_state = cpucooler_get_cur_state,
	.set_cur_state = cpucooler_set_cur_state,
};

static struct thermal_cooling_device_ops mtktscpu_cooling_sysrst_ops = {
	.get_max_state = mtktscpu_sysrst_get_max_state,
	.get_cur_state = mtktscpu_sysrst_get_cur_state,
	.set_cur_state = mtktscpu_sysrst_set_cur_state,
};

int mtktscpu_register_DVFS_hotplug_cooler(void)
{
	mtktscpu_dprintk("[mtktscpu_register_DVFS_hotplug_cooler] \n");

	cl_dev_sysrst = mtk_thermal_cooling_device_register("mtktscpu-sysrst", NULL,
					&mtktscpu_cooling_sysrst_ops);

	return 0;
}
int mtktscpu_register_thermal(void)
{
	mtktscpu_dprintk("[mtktscpu_register_thermal] \n");

	/* trips : trip 0~3 */
	thz_dev = mtk_thermal_zone_device_register("mtktscpu", num_trip, NULL,
				&mtktscpu_dev_ops, 0, 0, 0, interval*1000);
	return 0;
}

void mtktscpu_unregister_DVFS_hotplug_cooler(void)
{
	int i;
	for(i=0; i<CPU_COOLER_COUNT; i++)
	{
		if(mtk_cl_dev[i].cl_dev)
		{
			mtk_thermal_cooling_device_unregister(mtk_cl_dev[i].cl_dev);
			mtk_cl_dev[i].cl_dev = NULL;
		}
	}
	if(cl_dev_sysrst) {
		mtk_thermal_cooling_device_unregister(cl_dev_sysrst);
		cl_dev_sysrst = NULL;
	}
}

void mtktscpu_unregister_thermal(void)
{
	mtktscpu_dprintk("[mtktscpu_unregister_thermal] \n");
	if(thz_dev) {
		mtk_thermal_zone_device_unregister(thz_dev);
		thz_dev = NULL;
	}
}

static int mtk_thermal_suspend(struct platform_device *dev, pm_message_t state)
{
	mtktscpu_dprintk("[mtk_thermal_suspend] \n");
	if(talking_flag==false)
	{
		printk("[mtk_thermal_suspend] \n");
		THERMAL_WRAP_WR32(0x00000000, TEMPMONCTL0);    // disable periodoc temperature sensing point 0
		disable_clock(MT_CG_THEM_SW_CG,"THM");
		disable_clock(MT_CG_AUX_SW_CG_ADC,"THM");  // disable auxadc module.
		THERMAL_WRAP_WR32(DRV_Reg32(TS_CON0) | 0x000000C0, TS_CON0); // turn off the sensor buffer to save power
	}
	return 0;
}

static int mtk_thermal_resume(struct platform_device *dev)
{

	mtktscpu_dprintk("[mtk_thermal_resume] \n");
	if(talking_flag==false)
	{
		printk("[mtk_thermal_resume] \n");
		{
		   thermal_reset_and_initial();
		   set_thermal_ctrl_trigger_SPM(trip_temp[0]);
		}
	}

	return 0;
}

static struct platform_driver mtk_thermal_driver = {
	.remove     = NULL,
	.shutdown   = NULL,
	.probe      = NULL,
	.suspend	= mtk_thermal_suspend,
	.resume		= mtk_thermal_resume,
	.driver     = {
		.name = THERMAL_NAME,
	},
};

static ssize_t tscpu_write_log(struct file *file, const char __user *buffer, size_t count,
				   loff_t *data)
{
	char desc[32];
	int log_switch;
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &log_switch) == 1)
	{
		mtktscpu_debug_log = log_switch;
		return count;
	}
	return -EINVAL;
}
static int tscpu_read_opp(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n",cpu_cooler_limit);
	return 0;
}

static int tscpu_open_opp(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read_opp, NULL);
}

static const struct file_operations mtktscpu_opp_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_open_opp,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};



static int tscpu_read_log(struct seq_file *m, void *v)
{
	seq_printf(m, "[ tscpu_read_log] log = %d\n", mtktscpu_debug_log);

	return 0;
}



static int tscpu_open_log(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read_log, NULL);
}

static const struct file_operations mtktscpu_log_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_open_log,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = tscpu_write_log,
	.release = single_release,
};

static ssize_t tscpu_write(struct file *file, const char __user *buffer, size_t count,
			   loff_t *data)
{
	int len=0,time_msec=0;
	int trip[MTK_TZ_COOLER_MAX]={0};
	int t_type[MTK_TZ_COOLER_MAX]={0};
	int i;
	char bind[MTK_TZ_COOLER_MAX][THERMAL_NAME_LENGTH];
	char desc[512];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d",
				&num_trip,
				&trip[0],&t_type[0],bind[0], &trip[1],&t_type[1],bind[1],
				&trip[2],&t_type[2],bind[2], &trip[3],&t_type[3],bind[3],
				&trip[4],&t_type[4],bind[4], &trip[5],&t_type[5],bind[5],
				&trip[6],&t_type[6],bind[6], &trip[7],&t_type[7],bind[7],
				&trip[8],&t_type[8],bind[8], &trip[9],&t_type[9],bind[9],
				&time_msec, &MA_len_temp) == 33)
	{
		mtktscpu_dprintk("[mtktscpu_write] unregister_thermal\n");
		mtktscpu_unregister_thermal();

		for(i=0; i<MTK_TZ_COOLER_MAX; i++)
		{
			g_THERMAL_TRIP[i] = t_type[i];
			memcpy(g_bind[i], bind[i], THERMAL_NAME_LENGTH);
			trip_temp[i]=trip[i];
		}
		interval=time_msec / 1000;


		mtktscpu_dprintk("[mtktscpu_write] [trip_type]=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
				g_THERMAL_TRIP[0],g_THERMAL_TRIP[1],g_THERMAL_TRIP[2],g_THERMAL_TRIP[3],g_THERMAL_TRIP[4],
				g_THERMAL_TRIP[5],g_THERMAL_TRIP[6],g_THERMAL_TRIP[7],g_THERMAL_TRIP[8],g_THERMAL_TRIP[9]);

		mtktscpu_dprintk("[mtktscpu_write] [cool_bind]=%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
				g_bind[0],g_bind[1],g_bind[2],g_bind[3],g_bind[4],g_bind[5],g_bind[6],g_bind[7],g_bind[8],g_bind[9]);

		mtktscpu_dprintk("[mtktscpu_write] [trip_temp]==%d,%d,%d,%d,%d,%d,%d,%d,%d,%d, [time_ms]=%d\n",
				trip_temp[0],trip_temp[1],trip_temp[2],trip_temp[3],trip_temp[4],
				trip_temp[5],trip_temp[6],trip_temp[7],trip_temp[8],trip_temp[9],interval*1000);

		mtktscpu_dprintk("[mtktscpu_write] register_thermal\n");
		mtktscpu_register_thermal();

		return count;
	}
	else
	{
		mtktscpu_dprintk("[mtktscpu_write] bad argument\n");
	}

	return -EINVAL;
}

static int tscpu_read(struct seq_file *m, void *v)
{


	seq_printf(m,"[mtktscpu_read]\n\
				[trip_temp] = %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n\
				[trip_type] = %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n\
				[cool_bind] = %s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n\
				time_ms=%d\n",
				trip_temp[0],trip_temp[1],trip_temp[2],trip_temp[3],trip_temp[4],
				trip_temp[5],trip_temp[6],trip_temp[7],trip_temp[8],trip_temp[9],
				g_THERMAL_TRIP[0],g_THERMAL_TRIP[1],g_THERMAL_TRIP[2],g_THERMAL_TRIP[3],g_THERMAL_TRIP[4],
				g_THERMAL_TRIP[5],g_THERMAL_TRIP[6],g_THERMAL_TRIP[7],g_THERMAL_TRIP[8],g_THERMAL_TRIP[9],
				g_bind[0],g_bind[1],g_bind[2],g_bind[3],g_bind[4],g_bind[5],g_bind[6],g_bind[7],g_bind[8],g_bind[9],
				interval*1000);
	return 0;
}

static int tscpu_open(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read, NULL);
}

static const struct file_operations mtktscpu_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = tscpu_write,
	.release = single_release,
};





static int tscpu_read_cal(struct seq_file *m, void *v)
{

	seq_printf(m, "mtktscpu cal:\nReg1=0x%x, Reg2=0x%x",
					get_devinfo_with_index(15), get_devinfo_with_index(16));
	return 0;
}


static int tscpu_cal_open(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read_cal, NULL);
}

static const struct file_operations mtktscpu_cal_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_cal_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int tscpu_read_cooler(struct seq_file *m, void *v)
{
	int i=0;

	for(i=0; i<cl_dvfs_cooler_cnt; i++)
	{
		if(mtk_cl_dev[i].cl_dev)
		{
			seq_printf(m, "COOLER[%d]= %s\n", i,mtk_cl_dev[i].cooler_name);
		}
	}

	return 0;
}


static ssize_t tscpu_write_cooler(struct file *file, const char __user *buffer, size_t count,
			   loff_t *data)
{
	int len;
	unsigned int cooler_cnt;
	char desc[512];
	unsigned int cooler_value[CPU_COOLER_COUNT];
	int i;
	bool is_pass = 0;
	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}

	desc[len] = '\0';


	len = sscanf(desc, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
				&cooler_cnt,
				&cooler_value[0],&cooler_value[1],&cooler_value[2], &cooler_value[3],&cooler_value[4],
				&cooler_value[5],&cooler_value[6],&cooler_value[7], &cooler_value[8],&cooler_value[9],
				&cooler_value[10],&cooler_value[11],&cooler_value[12], &cooler_value[13],&cooler_value[14]);

	if (cooler_cnt > CPU_COOLER_COUNT)
	{
		return 0;
	}

	if((len > 1) && (cooler_cnt == (len -1)))
	{
		if(cooler_cnt <= CPU_COOLER_COUNT)
		{
			for(i=0; i<cooler_cnt; i++)
			{
				if (!((cooler_value[i] > 0) && (cooler_value[i] < 10000)))
					break;
			}
			is_pass = 1;
		}
	}

	if(is_pass == 1)
	{
		for(i=0; i<CPU_COOLER_COUNT; i++)
		{
			if(mtk_cl_dev[i].cl_dev)
			{
				mtk_thermal_cooling_device_unregister(mtk_cl_dev[i].cl_dev);
				mtk_cl_dev[i].cl_dev = NULL;
			}
		}
		for(i=0; i<cooler_cnt; i++)
		{
		   mtk_cl_dev[i].cl_limit = cooler_value[i];
//		   sprintf(mtk_cl_dev[i].cooler_name,"%d", cooler_value[i]);
		   snprintf(mtk_cl_dev[i].cooler_name,sizeof(mtk_cl_dev[i].cooler_name),"%d", cooler_value[i]);

		   mtk_cl_dev[i].cl_dev = mtk_thermal_cooling_device_register(mtk_cl_dev[i].cooler_name, &mtk_cl_dev[i],
							  &mtktscpu_cooling_ops);
		}
		cl_dvfs_cooler_cnt = cooler_cnt;
		return count;
	}
	else
	{
		mtktscpu_dprintk("[mtktscpu_write] bad argument\n");
	}

	return -EINVAL;
}
static int tscpu_cooler_open(struct inode *inode, struct file *file)
{
	return single_open(file, tscpu_read_cooler, NULL);
}

static const struct file_operations mtktscpu_cooler_fops = {
	.owner = THIS_MODULE,
	.open = tscpu_cooler_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = tscpu_write_cooler,
	.release = single_release,
};


static int __init mtktscpu_init(void)
{
	thermal_cal_prepare();
	thermal_cal_prepare_2();
	return 0;
}




//static int __init mtktscpu_init(void)
static int __init thermal_late_init(void)
{
	int err = 0;
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtktscpu_dir = NULL;
//	struct TS_PTPOD ts;
	mtktscpu_dprintk("[mtktscpu_init] \n");

	THERMAL_WRAP_WR32(DRV_Reg32(TS_CON0) | 0x000000C0, TS_CON0); // turn off the sensor buffer to save power


	thermal_reset_and_initial();
//	set_high_low_threshold(20000, 10000);//test

	err = platform_driver_register(&mtk_thermal_driver);
	if (err)
		return err;

//	err = init_cooler();
//	if(err)
//		return err;

	err = mtktscpu_register_DVFS_hotplug_cooler();
	if(err)
		return err;

	err = mtktscpu_register_thermal();
	if(err)
		goto err_unreg;

	err= request_irq(MT_PTP_THERM_IRQ_ID, thermal_interrupt_handler, IRQF_TRIGGER_LOW, THERMAL_NAME, NULL);
	if(err)
		mtktscpu_dprintk("[mtktscpu_init] IRQ register fail\n");

	#if defined(THERMAL_MTK_ABB_SUPPORT)
		mtktsabb_init();
	#endif

	mtktscpu_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtktscpu_dir)	{
		mtktscpu_dprintk("[%s]: mkdir /proc/driver/thermal failed\n", __func__);
	}
	else
	{
		entry =
			proc_create("tzcpu", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir,
				&mtktscpu_fops);
		if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
			proc_set_user(entry, 0, 1000);
#else
			entry->gid = 1000;
#endif
		}

		entry =
			proc_create("tzcpu_log", S_IRUGO | S_IWUSR, mtktscpu_dir,
				&mtktscpu_log_fops);

		entry =
			proc_create("tzcpu_cooler", S_IRUGO | S_IWUSR, mtktscpu_dir,
				&mtktscpu_cooler_fops);
		if (entry)
		{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
				proc_set_user(entry, 0, 1000);
#else
				entry->gid = 1000;
#endif
		}

		entry = proc_create("tzcpu_opp", S_IRUGO, mtktscpu_dir, &mtktscpu_opp_fops);
		entry = proc_create("tzcpu_cal", S_IRUSR, mtktscpu_dir, &mtktscpu_cal_fops);
	}

//	get_thermal_slope_intercept(&ts);
//	printk("INIT: ts_MTS=%d, ts_BTS=%d \n", ts.ts_MTS, ts.ts_BTS);

	return 0;

err_unreg:
	mtktscpu_unregister_DVFS_hotplug_cooler();
	return err;
}

static void __exit mtktscpu_exit(void)
{
	mtktscpu_dprintk("[mtktscpu_exit] \n");
	#if defined(THERMAL_MTK_ABB_SUPPORT)
		mtktsabb_exit();
	#endif
	mtktscpu_unregister_thermal();
	mtktscpu_unregister_DVFS_hotplug_cooler();
}

#if defined(THERMAL_MTK_ABB_SUPPORT)
/* Zone */
static unsigned int cl_dev_abbsysrst_state=0;
static struct thermal_cooling_device *cl_devabb_sysrst;

static struct thermal_zone_device *thz_dev_abb;
static unsigned int interval_abb = 0; /* seconds, 0 : no auto polling */
static unsigned int trip_temp_abb[MTK_TZ_COOLER_MAX] = {120000,110000,100000,90000,80000,70000,65000,60000,55000,50000};
static int g_THERMAL_TRIP_abb[MTK_TZ_COOLER_MAX] = {0,0,0,0,0,0,0,0,0,0};
static int num_trip_abb=0;
static char g_bind_abb[MTK_TZ_COOLER_MAX][THERMAL_NAME_LENGTH] = {{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}};
static int kernelmode_abb = 0;
#define MTKTSABB_TEMP_CRIT 120000 /* 120.000 degree Celsius */

/* Logging */
#define mtktsabb_dprintk(fmt, args...)   \
do {                                    \
	if (mtktscpu_debug_log) {                \
		pr_debug("Power/CPU_Thermal " fmt, ##args); \
	}                                   \
} while(0)

static ssize_t tsabb_write(struct file *file, const char __user *buffer, size_t count,
			   loff_t *data)
{
	int len=0,time_msec=0;
	int trip[MTK_TZ_COOLER_MAX]={0};
	int t_type[MTK_TZ_COOLER_MAX]={0};
	int i;
	char bind[MTK_TZ_COOLER_MAX][THERMAL_NAME_LENGTH];
	char desc[512];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d",
				&num_trip_abb,
				&trip[0],&t_type[0],bind[0], &trip[1],&t_type[1],bind[1],
				&trip[2],&t_type[2],bind[2], &trip[3],&t_type[3],bind[3],
				&trip[4],&t_type[4],bind[4], &trip[5],&t_type[5],bind[5],
				&trip[6],&t_type[6],bind[6], &trip[7],&t_type[7],bind[7],
				&trip[8],&t_type[8],bind[8], &trip[9],&t_type[9],bind[9],
				&time_msec) == 32)
	{
		mtktsabb_dprintk("[mtktsabb_write] unregister_thermal\n");
		mtktsabb_unregister_thermal();

		for(i=0; i<MTK_TZ_COOLER_MAX; i++)
		{
			g_THERMAL_TRIP_abb[i] = t_type[i];
			memcpy(g_bind_abb[i], bind[i], THERMAL_NAME_LENGTH);
			trip_temp_abb[i]=trip[i];
		}
		interval_abb=time_msec / 1000;


		mtktsabb_dprintk("[mtktsabb_write] [trip_type]=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
				g_THERMAL_TRIP_abb[0],g_THERMAL_TRIP_abb[1],g_THERMAL_TRIP_abb[2],g_THERMAL_TRIP_abb[3],g_THERMAL_TRIP_abb[4],
				g_THERMAL_TRIP_abb[5],g_THERMAL_TRIP_abb[6],g_THERMAL_TRIP_abb[7],g_THERMAL_TRIP_abb[8],g_THERMAL_TRIP_abb[9]);

		mtktsabb_dprintk("[mtktsabb_write] [cool_bind]=%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
				g_bind_abb[0],g_bind_abb[1],g_bind_abb[2],g_bind_abb[3],g_bind_abb[4],g_bind_abb[5],g_bind_abb[6],g_bind_abb[7],g_bind_abb[8],g_bind_abb[9]);

		mtktsabb_dprintk("[mtktsabb_write] [trip_temp]==%d,%d,%d,%d,%d,%d,%d,%d,%d,%d, [time_ms]=%d\n",
				trip_temp_abb[0],trip_temp_abb[1],trip_temp_abb[2],trip_temp_abb[3],trip_temp_abb[4],
				trip_temp_abb[5],trip_temp_abb[6],trip_temp_abb[7],trip_temp_abb[8],trip_temp_abb[9],interval_abb*1000);

		mtktsabb_dprintk("[mtktsabb_write] register_thermal\n");
		mtktsabb_register_thermal();

		return count;
	}
	else
	{
		mtktsabb_dprintk("[mtktsabb_write] bad argument\n");
	}

	return -EINVAL;
}


static int tsabb_read(struct seq_file *m, void *v)
{

	seq_printf(m,
		   "[mtktsabb_read]\n\
			[trip_temp] = %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n\
			[trip_type] = %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n\
			[cool_bind] = %s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n\
			time_ms=%d\n",
			trip_temp_abb[0],trip_temp_abb[1],trip_temp_abb[2],trip_temp_abb[3],trip_temp_abb[4],
			trip_temp_abb[5],trip_temp_abb[6],trip_temp_abb[7],trip_temp_abb[8],trip_temp_abb[9],
			g_THERMAL_TRIP_abb[0],g_THERMAL_TRIP_abb[1],g_THERMAL_TRIP_abb[2],g_THERMAL_TRIP_abb[3],g_THERMAL_TRIP_abb[4],
			g_THERMAL_TRIP_abb[5],g_THERMAL_TRIP_abb[6],g_THERMAL_TRIP_abb[7],g_THERMAL_TRIP_abb[8],g_THERMAL_TRIP_abb[9],
			g_bind_abb[0],g_bind_abb[1],g_bind_abb[2],g_bind_abb[3],g_bind_abb[4],g_bind_abb[5],g_bind_abb[6],g_bind_abb[7],g_bind_abb[8],g_bind_abb[9],
			interval_abb*1000);
	return 0;
}

static int tsabb_open(struct inode *inode, struct file *file)
{
	return single_open(file, tsabb_read, NULL);
}

static const struct file_operations mtktsabb_fops = {
	.owner = THIS_MODULE,
	.open = tsabb_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = tsabb_write,
	.release = single_release,
};

static int get_immediate_temp_abb(void)
{
	int curr_raw, curr_temp;
	curr_raw = DRV_Reg32(TEMPMSR1);
	curr_raw = curr_raw & 0x0fff;
	curr_temp = raw_to_temperature_abb(curr_raw);
	mtktsabb_dprintk("[get_abb_temp] temp=%d, rawdata = %d\n", curr_temp ,curr_raw);
	return curr_temp;
}


static int mtktsabb_get_temp(struct thermal_zone_device *thermal,
		unsigned long *t)
{
#if defined(MTK_THERMAL_HW_PROTECT)
	int curr_temp;
	curr_temp = get_immediate_temp_abb();

	if((curr_temp>100000) || (curr_temp<-30000))
		printk("[Power/CPU_Thermal] CPU T=%d\n",curr_temp);
	*t = (unsigned long) curr_temp;
	return 0;
#else
  int curr_temp;
  int temp_temp;
  int ret = 0;
  static int last_cpu_temp = 0;


  curr_temp = get_immediate_temp(); //mtktscpu_get_TC_temp();
	//	if((curr_temp>100000) || (curr_temp<-30000))
  if((curr_temp > (trip_temp[0] - 15000)) || (curr_temp<-30000))
	printk("[Power/CPU_Thermal] ABB T=%d\n",curr_temp);


	temp_temp = curr_temp;
	if(curr_temp != 0)
	{
		if((curr_temp > 150000) || (curr_temp < -20000))
		{
			printk("[Power/CPU_Thermal] ABB temp invalid = %d\n", curr_temp);
			temp_temp = 50000;
			ret = -1;
		}
		else if(last_cpu_temp!=0)
		{
			if ((curr_temp - last_cpu_temp > 20000) ||(last_cpu_temp - curr_temp > 20000)) //delta 20C
			{
				printk("[Power/CPU_Thermal] ABB temp float hugely temp=%d, lasttemp =%d\n", curr_temp, last_cpu_temp);
				temp_temp = 50000;
				ret = -1;
			}
		}
	}

	last_cpu_temp = curr_temp;
	curr_temp = temp_temp;
	*t = (unsigned long) curr_temp;
	return ret;
#endif
}

static int mtktsabb_bind(struct thermal_zone_device *thermal,
						struct thermal_cooling_device *cdev)
{
	int table_val=0;
	table_val = mtkts_match(cdev,g_bind_abb);
	if(table_val >= MTK_TZ_COOLER_MAX)
	{
		return 0;
	}
	else
	{
		mtktsabb_dprintk("[mtktsabb_bind] %s\n", cdev->type);
		if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) {
			mtktsabb_dprintk("[mtktsabb_bind] error binding cooling dev\n");
			return -EINVAL;
		} else {
			mtktsabb_dprintk("[mtktsabb_bind] binding OK, %d\n", table_val);
		}
	}
	return 0;
}



static int mtktsabb_unbind(struct thermal_zone_device *thermal,
						struct thermal_cooling_device *cdev)
{
	int table_val=0;
	table_val = mtkts_match(cdev,g_bind_abb);
	if(table_val >= MTK_TZ_COOLER_MAX)
	{
		return 0;
	}
	else
	{
		mtktsabb_dprintk("[mtktsabb_unbind] %s\n", cdev->type);
		if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) {
			mtktsabb_dprintk("[mtktsabb_unbind] error unbinding cooling dev\n");
			return -EINVAL;
		} else {
			mtktsabb_dprintk("[mtktsabb_unbind] unbinding OK, %d\n", table_val);
		}
	}
	return 0;
}

static int mtktsabb_get_mode(struct thermal_zone_device *thermal,
							enum thermal_device_mode *mode)
{
	*mode = (kernelmode_abb) ? THERMAL_DEVICE_ENABLED
		: THERMAL_DEVICE_DISABLED;
	return 0;
}

static int mtktsabb_set_mode(struct thermal_zone_device *thermal,
							enum thermal_device_mode mode)
{
	kernelmode_abb = mode;
	return 0;
}

static int mtktsabb_get_trip_type(struct thermal_zone_device *thermal, int trip,
								enum thermal_trip_type *type)
{
	*type = g_THERMAL_TRIP_abb[trip];
	return 0;
}

static int mtktsabb_get_trip_temp(struct thermal_zone_device *thermal, int trip,
								unsigned long *temp)
{
	*temp = trip_temp_abb[trip];
	return 0;
}

static int mtktsabb_get_crit_temp(struct thermal_zone_device *thermal,
								unsigned long *temperature)
{
	*temperature = MTKTSABB_TEMP_CRIT;
	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops mtktsabb_dev_ops = {
	.bind = mtktsabb_bind,
	.unbind = mtktsabb_unbind,
	.get_temp = mtktsabb_get_temp,
	.get_mode = mtktsabb_get_mode,
	.set_mode = mtktsabb_set_mode,
	.get_trip_type = mtktsabb_get_trip_type,
	.get_trip_temp = mtktsabb_get_trip_temp,
	.get_crit_temp = mtktsabb_get_crit_temp,
};

/*
 * cooling device callback functions (mtktscpu_cooling_sysrst_ops)
 * 1 : ON and 0 : OFF
 */
static int mtktsabb_sysrst_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	mtktscpu_dprintk("tsabb_get_max_state\n");
	*state = 1;
	return 0;
}
static int mtktsabb_sysrst_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	mtktscpu_dprintk("tsabb_get_cur_state\n");
	*state = cl_dev_abbsysrst_state;
	return 0;
}
static int mtktsabb_sysrst_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
	mtktscpu_dprintk("tsabb_set_cur_state\n");
	cl_dev_abbsysrst_state = state;
	if(cl_dev_abbsysrst_state == 1)
	{
		printk("Power/CPU_Thermal: ABB reset, reset, reset!!!\n");
		BUG();
	}
	return 0;
}

/* bind fan callbacks to fan device */

static struct thermal_cooling_device_ops mtktsabb_cooling_sysrst_ops = {
	.get_max_state = mtktsabb_sysrst_get_max_state,
	.get_cur_state = mtktsabb_sysrst_get_cur_state,
	.set_cur_state = mtktsabb_sysrst_set_cur_state,
};

int mtktsabb_register_DVFS_hotplug_cooler(void)
{
	mtktscpu_dprintk("[mtktsabb_register_DVFS_hotplug_cooler] \n");

	cl_devabb_sysrst = mtk_thermal_cooling_device_register("mtktsabb-sysrst", NULL,
					&mtktsabb_cooling_sysrst_ops);

	return 0;
}

void mtktsabb_unregister_DVFS_hotplug_cooler(void)
{
	if(cl_devabb_sysrst) {
		mtk_thermal_cooling_device_unregister(cl_devabb_sysrst);
		cl_devabb_sysrst = NULL;
	}
}

int mtktsabb_register_thermal(void)
{
	mtktsabb_dprintk("[mtktsabb_register_thermal] \n");

	/* trips : trip 0~3 */
	thz_dev_abb = mtk_thermal_zone_device_register("mtktsabb", num_trip_abb, NULL,
				&mtktsabb_dev_ops, 0, 0, 0, interval_abb*1000);
	return 0;
}

void mtktsabb_unregister_thermal(void)
{
	mtktsabb_dprintk("[mtktsabb_unregister_thermal] \n");
	if(thz_dev_abb) {
		mtk_thermal_zone_device_unregister(thz_dev_abb);
		thz_dev_abb = NULL;
	}
}



int __init mtktsabb_init(void)
{
	int err = 0;
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtktsabb_dir = NULL;

	err = mtktsabb_register_DVFS_hotplug_cooler();

	err = mtktsabb_register_thermal();
	if(err)
		goto err_unregabb;

	mtktsabb_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtktsabb_dir)	{
		mtktscpu_dprintk("[%s]: mkdir /proc/driver/thermal failed\n", __func__);
	}
	else
	{
		entry =
			proc_create("tzabb", S_IRUGO | S_IWUSR | S_IWGRP, mtktsabb_dir,
				&mtktsabb_fops);
		if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
			proc_set_user(entry, 0, 1000);
#else
			entry->gid = 1000;
#endif
		}
	}

	return 0;

err_unregabb:
	return err;
}

void __exit mtktsabb_exit(void)
{
	mtktsabb_dprintk("[mtktsabb_exit] \n");
	mtktsabb_unregister_thermal();
	mtktsabb_unregister_DVFS_hotplug_cooler();
}
#endif


module_init(mtktscpu_init);
module_exit(mtktscpu_exit);

late_initcall(thermal_late_init);


