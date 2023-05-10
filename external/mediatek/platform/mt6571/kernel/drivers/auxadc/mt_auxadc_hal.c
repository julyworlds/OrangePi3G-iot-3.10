/*****************************************************************************
 *
 * Filename:
 * ---------
 *    mt_auxadc_hal.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of mt6582 AUXADC
 *
 * Author:
 * -------
 * Zhong Wang
 *
 ****************************************************************************/
 
#include <linux/init.h>        /* For init/exit macros */
#include <linux/module.h>      /* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/mt_gpt.h>
#include <mach/mt_clkmgr.h>
#include <mach/sync_write.h>
#include <cust_adc.h> // generate by DCT Tool

#include "mt_auxadc_sw.h"
#include "mt_auxadc_hw.h"


#define DRV_ClearBits(addr,data)     {\
   kal_uint16 temp;\
   temp = DRV_Reg(addr);\
   temp &=~(data);\
   mt65xx_reg_sync_writew(temp, addr);\
}

#define DRV_SetBits(addr,data)     {\
   kal_uint16 temp;\
   temp = DRV_Reg(addr);\
   temp |= (data);\
   mt65xx_reg_sync_writew(temp, addr);\
}

#define DRV_SetData(addr, bitmask, value)     {\
   kal_uint16 temp;\
   temp = (~(bitmask)) & DRV_Reg(addr);\
   temp |= (value);\
   mt65xx_reg_sync_writew(temp, addr);\
}

#define AUXADC_DRV_ClearBits16(addr, data)           DRV_ClearBits(addr,data)
#define AUXADC_DRV_SetBits16(addr, data)             DRV_SetBits(addr,data)
#define AUXADC_DRV_WriteReg16(addr, data)            mt65xx_reg_sync_writew(data, addr)
#define AUXADC_DRV_ReadReg16(addr)                   DRV_Reg(addr)
#define AUXADC_DRV_SetData16(addr, bitmask, value)   DRV_SetData(addr, bitmask, value)

#define AUXADC_DVT_DELAYMACRO(u4Num)                                     \
{                                                                        \
    unsigned int u4Count = 0 ;                                           \
    for (u4Count = 0; u4Count < u4Num; u4Count++ );                      \
}

#define AUXADC_CLR_BITS(BS,REG)     {\
   kal_uint32 temp;\
   temp = DRV_Reg32(REG);\
   temp &=~(BS);\
   mt65xx_reg_sync_writel(temp, REG);\
}

#define AUXADC_SET_BITS(BS,REG)     {\
   kal_uint32 temp;\
   temp = DRV_Reg32(REG);\
   temp |= (BS);\
   mt65xx_reg_sync_writel(temp, REG);\
}

#define VOLTAGE_FULL_RANGE  1500 // VA voltage
#define AUXADC_PRECISE      4096 // 12 bits

/*****************************************************************************
 * Integrate with NVRAM 
****************************************************************************/
//use efuse cali
#if 0
static kal_uint32 g_adc_ge = 0;
static kal_uint32 g_adc_oe = 0;
//static kal_uint32 g_o_vts = 0;
static kal_uint32 g_o_vbg = 0;
//static kal_uint32 g_degc_cali = 0;
static kal_uint32 g_adc_cali_en = 0;
//static kal_uint32 g_o_vts_abb = 0;
//static kal_int32 g_o_slope = 0;
//static kal_uint32 g_o_slope_sign = 0;
//static kal_uint32 g_id = 0;
static kal_uint32 g_y_vbg = 0;//defaul 1967 if cali_en=0
#endif
static DEFINE_MUTEX(mutex_get_cali_value);
static int adc_auto_set =0;

static u16 mt_tpd_read_adc(u16 pos) {
   AUXADC_DRV_SetBits16((volatile u16 *)AUXADC_TP_ADDR, pos);
   AUXADC_DRV_SetBits16((volatile u16 *)AUXADC_TP_CON0, 0x01);
   while(0x01 & AUXADC_DRV_ReadReg16((volatile u16 *)AUXADC_TP_CON0)) { ; } //wait for write finish
   return AUXADC_DRV_ReadReg16((volatile u16 *)AUXADC_TP_DATA0); 
}

static void mt_auxadc_disable_penirq(void)
{
	//Turn off PENIRQ detection circuit
	AUXADC_DRV_SetBits16((volatile u16 *)AUXADC_TP_CMD, 1);
	//run once touch function
	mt_tpd_read_adc(TP_CMD_ADDR_X);
}


//step1 check con2 if auxadc is busy
//step2 clear bit
//step3  read channle and make sure old ready bit ==0
//step4 set bit  to trigger sample
//step5  read channle and make sure  ready bit ==1
//step6 read data

int IMM_auxadc_GetOneChannelValue(int dwChannel, int data[4], int* rawdata)
{
   unsigned int channel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
   int idle_count =0;
   int data_ready_count=0;
   
   mutex_lock(&mutex_get_cali_value);
#if 0   
   if(enable_clock(MT_PDN_PERI_AUXADC,"AUXADC"))
   {
	printk("hwEnableClock AUXADC failed.");
   }
#endif   
   if(dwChannel == PAD_AUX_XP)mt_auxadc_disable_penirq();
   //step1 check con2 if auxadc is busy
   while ((*(volatile u16 *)AUXADC_CON2) & 0x01) 
   {
       printk("[adc_api]: wait for module idle\n");
       msleep(100);
	   idle_count++;
	   if(idle_count>30)
	   {
	      //wait for idle time out
	      printk("[adc_api]: wait for auxadc idle time out\n");
		mutex_unlock(&mutex_get_cali_value);
	      return -1;
	   }
   } 
   // step2 clear bit
   if(0 == adc_auto_set)
   {
	   //clear bit
	   AUXADC_DRV_ClearBits16((volatile u16 *)AUXADC_CON1, (1 << dwChannel));
   }
   

   //step3  read channle and make sure old ready bit ==0
   while ((*(volatile u16 *)(AUXADC_DAT0 + dwChannel * 0x04)) & (1<<12)) 
   {
       printk("[adc_api]: wait for channel[%d] ready bit clear\n",dwChannel);
       msleep(10);
	   data_ready_count++;
	   if(data_ready_count>30)
	   {
	      //wait for idle time out
	      printk("[adc_api]: wait for channel[%d] ready bit clear time out\n",dwChannel);
		mutex_unlock(&mutex_get_cali_value);
	      return -2;
	   }
   }
  
   //step4 set bit  to trigger sample
   if(0==adc_auto_set)
   {  
   	  AUXADC_DRV_SetBits16((volatile u16 *)AUXADC_CON1, (1 << dwChannel));
   }
   //step5  read channle and make sure  ready bit ==1
   udelay(25);//we must dealay here for hw sample cahnnel data
   while (0==((*(volatile u16 *)(AUXADC_DAT0 + dwChannel * 0x04)) & (1<<12))) 
   {
       printk("[adc_api]: wait for channel[%d] ready bit ==1\n",dwChannel);
       msleep(10);
	 data_ready_count++;

	 if(data_ready_count>30)
	 {
	      //wait for idle time out
	      printk("[adc_api]: wait for channel[%d] data ready time out\n",dwChannel);
		mutex_unlock(&mutex_get_cali_value);
	      return -3;
	 }
   }
   //step6 read data
   
   channel[dwChannel] = (*(volatile u16 *)(AUXADC_DAT0 + dwChannel * 0x04)) & 0x0FFF;
   if(NULL != rawdata)
   {
      *rawdata = channel[dwChannel];
   }
   //printk("[adc_api: imm mode raw data => channel[%d] = %d\n",dwChannel, channel[dwChannel]);
   //printk("[adc_api]: imm mode => channel[%d] = %d.%02d\n", dwChannel, (channel[dwChannel] * 150 / AUXADC_PRECISE / 100), ((channel[dwChannel] * 150 / AUXADC_PRECISE) % 100));
   data[0] = (channel[dwChannel] * 150 / AUXADC_PRECISE / 100);
   data[1] = ((channel[dwChannel] * 150 / AUXADC_PRECISE) % 100);
   
#if 0
   if(disable_clock(MT_PDN_PERI_AUXADC,"AUXADC"))
   {
        printk("hwEnableClock AUXADC failed.");
   }
#endif   
    mutex_unlock(&mutex_get_cali_value);
   
   return 0;
   
}

// 1v == 1000000 uv
// this function voltage Unit is uv
int IMM_auxadc_GetOneChannelValue_Cali(int Channel, int*voltage)
{
     int ret = 0, data[4], rawvalue;
     
     ret = IMM_auxadc_GetOneChannelValue( Channel,  data, &rawvalue);
     if(ret)
     {
	        printk("[adc_api]:IMM_auxadc_GetOneChannelValue_Cali  get raw value error %d \n",ret);
		return -1;
     }
     *voltage = rawvalue*1500000 / AUXADC_PRECISE;
      //printk("[adc_api]:IMM_auxadc_GetOneChannelValue_Cali  voltage= %d uv \n",*voltage);
      return 0;
     
}

#if 0
static int IMM_auxadc_get_evrage_data(int times, int Channel)
{
	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0;

	i = times;
	while (i--)
	{
		ret_value = IMM_auxadc_GetOneChannelValue(Channel, data, &ret_temp);
		ret += ret_temp;
		printk("[auxadc_get_data(channel%d)]: ret_temp=%d\n",Channel,ret_temp);        
	}

	ret = ret / times;
	return ret;
}
#endif

static void mt_auxadc_cal_prepare(void)
{
	kal_uint32 reg_val = 0;

// FIXME: eFuse value setting for BGR trim
//	printk("[adc_cali]: Check BGR 0x%08x, TS_CON0 0x%08x\n",DRV_Reg32(0xF0009100), DRV_Reg32(0xF0018600));

	if (reg_val = (DRV_Reg32(0xF0009100) >> 4) & 0xF)
	{
		reg_val = DRV_Reg32(0xF0018600) & 0xFFFF0FFF | (reg_val << 12);
		DRV_WriteReg32(0xF0018600, reg_val);
	} 
	if (reg_val = (DRV_Reg32(0xF0009100) >> 0) & 0xF)
	{
		reg_val = DRV_Reg32(0xF0018600) & 0xFFFFF0FF | (reg_val << 8);
		DRV_WriteReg32(0xF0018600, reg_val);
	}

//	printk("[adc_cali]: GET BGR 0x%08x, TS_CON0 0x%08x\n",DRV_Reg32(0xF0009100), DRV_Reg32(0xF0018600));

#if 0
	kal_uint32 temp = 0;
	
	temp = DRV_Reg32(0xF1019048);
	g_adc_ge = (temp & 0x000000FF);
	printk("[auxadc]temp = 0x%x, g_adc_ge = 0x%x\n", temp, g_adc_ge);
	
	temp = DRV_Reg32(0xF1019044);
	g_adc_oe = (temp & 0x000000FF);
	printk("[auxadc]temp = 0x%x, g_adc_oe = 0x%x\n", temp, g_adc_oe);
	
	temp = DRV_Reg32(0xF1019040);
	//g_o_vts_abb   = ((temp & 0xFC000000) >> 26);
	g_o_vts       = ((temp & 0x03FE0000) >> 17);
	g_o_vbg       = ((temp & 0x0001FF00) >> 8);
	//g_degc_cali   = ((temp & 0x000000FE) >> 1);
	g_degc_cali   = ((temp & 0x0000007E) >> 1);
	g_adc_cali_en = ((temp & 0x00000001) >> 0);

	g_o_slope     = ((temp & 0xFC000000) >> 26);
	g_o_slope_sign= ((temp & 0x00000080) >> 7);    

      //get y_vbg
      mt65xx_reg_sync_writel(0x0002, 0xf0007804);//TS_CON1
	mt65xx_reg_sync_writel(0x0200, 0xf0007808);//TS_CON2
	msleep(10);
	g_y_vbg = IMM_auxadc_get_evrage_data(20,5);

	temp = DRV_Reg32(0xF1019100);
	g_id = ((temp & 0x80000000) >> 31);

	if(g_id==0)
	{
		g_o_slope = 0;
	}
	
	if(g_adc_cali_en == 1)
	{
		//get y_vbg      
	}
	else
	{
		g_adc_ge = 128;
		g_adc_oe = 128;
		g_o_vts = 292;
		g_o_vbg = 167;
		g_degc_cali = 40;
		g_o_slope = 0;
		g_o_slope_sign = 0;
		g_y_vbg = 1967;
	}
	
	printk("[auxadc]temp = 0x%x, g_y_vbg=%d, g_o_vts = 0x%x, g_o_vbg = 0x%x, g_degc_cali = 0x%x, g_adc_cali_en = 0x%x, g_o_vts_abb = 0x%x, g_o_slope = 0x%x, g_o_slope_sign = 0x%x, g_id = 0x%x\n", 
		temp, g_y_vbg, g_o_vts, g_o_vbg, g_degc_cali, g_adc_cali_en, g_o_vts_abb, g_o_slope, g_o_slope_sign, g_id);
#endif
}

void mt_auxadc_hal_init(void)
{
	mt_auxadc_cal_prepare();

	AUXADC_DRV_SetBits16((volatile u16 *)AUXADC_CON_RTP, 1);		//disable RTP
}

void mt_auxadc_hal_suspend(void)
{
	//82 no need;
}

void mt_auxadc_hal_resume(void)
{
	//read calibration data from EFUSE
	mt_auxadc_cal_prepare();

	AUXADC_DRV_SetBits16((volatile u16 *)AUXADC_CON_RTP, 1);		//disable RTP
}

int mt_auxadc_dump_register(char *buf)
{
	printk("[auxadc]: AUXADC_CON0=%x\n",*(volatile u16 *)AUXADC_CON0);
	printk("[auxadc]: AUXADC_CON1=%x\n",*(volatile u16 *)AUXADC_CON1);
	printk("[auxadc]: AUXADC_CON2=%x\n",*(volatile u16 *)AUXADC_CON2);

	return sprintf(buf, "AUXADC_CON0:%x\n AUXADC_CON1:%x\n AUXADC_CON2:%x\n"
		, *(volatile u16 *)AUXADC_CON0,*(volatile u16 *)AUXADC_CON1,*(volatile u16 *)AUXADC_CON2);
}


