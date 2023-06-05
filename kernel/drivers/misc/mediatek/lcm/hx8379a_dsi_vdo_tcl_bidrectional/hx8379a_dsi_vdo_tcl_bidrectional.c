/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifdef BUILD_LK
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFA    //0xFFF ??   // END OF REGISTERS MARKER

#define LCM_ID       (0x79)

#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#if defined(BUILD_UBOOT)
	#define LCM_PRINT printf
#else
	#define LCM_PRINT printk
#endif
#endif

#define LCM_DBG(fmt, arg...) \
	LCM_PRINT("[LCM-HX8379a-DSI] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

static unsigned int lcm_compare_id(void);
static unsigned int test_compare_id(void);
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_read_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
#if 0   //used for one direction scan
	{0xB9,	3,	{0xFF, 0x83, 0x79}},
	{0xBA,	18,	{0x51,0xA3,0x00,0x16,0xA4,0x00,0x18,0xFF,
				  0x0F,0x29,0x03,0x21,0x23,0x25,0x20,0x02,
				  0x35,0x40}},  
				  
	{0xB1,	31,	{0x00,0x50,0x44,0xEA,0x90,0x08,0x11,0x11,
			  	  0x71,0x2F,0x37,0x9A,0x1A,0x42,0x1B,0x6E,
			  	  0xF1,0x00,0xE6,0xE6,0xE6,0xE6,0xE6,0x00,
			  	  0x04,0x05,0x0A,0x0B,0x04,0x05,0x6F}},

			
	{0xB2,	13,	{0x00,0x00,0x3C,0x08,0x0C,0x19,0x22,0x00,
				  0xFF,0x08,0x0C,0x19,0x20}},
	
	//{0xB3,	4,	{0x83, 0x00, 0x31, 0x03}},
	
	{0xB4,	31,	{0x45,0x08,0x00,0x32,0x10,0x05,0x32,0x13,
				  0x29,0x32,0x10,0x08,0x17,0x01,0x28,0x07,
				  0x13,0x08,0x28,0x08,0x30,0x30,0x04,0x00,
				  0x40,0x08,0x28,0x08,0x30,0x30,0x04}},  

	{0xD5,	47,	{0x00,0x00,0x08,0x00,0x01,0x05,0x00,0x02,
				  0x00,0x88,0x99,0x88,0x88,0x01,0x23,0x01,
				  0x23,0x88,0x01,0x88,0x88,0x88,0x88,0x88,
				  0x88,0x76,0x54,0x76,0x54,0x32,0x10,0x32,
				  0x10,0x88,0x88,0x88,0x88,0x88,0x88,0x88,
				  0x88,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0xDE,	3,	{0x05,0x70,0x04}},

	{0xE0,	35,	{0x79,0x07,0x12,0x14,0x3F,0x3F,0x3F,0x25,
				  0x4F,0x06,0x0C,0x0E,0x12,0x17,0x12,0x13,
				  0x14,0x1F,0x07,0x12,0x14,0x3F,0x3F,0x3F,
				  0x25,0x4F,0x06,0x0C,0x0E,0x12,0x17,0x12,
				  0x13,0x14,0x1F}},

	{0x53,	1,	{0x40}}, 

#else
	{0xB9,	3,	{0XFF,0X83,0X79}},

	{0xBA,	1,	{0x51}},
	
	{0xB1,	20,	{0x00,0x50,0x35,0xEA,0x90,0x08,0x11,0x11,
				  0x71,0x2F,0x37,0xBF,0x3F,0x42,0x0B,0x6E,
				  0xF1,0x00,0xE6,0xE6}},
			  	  
	{0xB2,	13,	{0x00,0x00,0xFE,0x08,0x0C,0x19,0x22,0x00,
				  0xFF,0x08,0x0C,0x19,0x20}},
				  
	{0xB4,	31,	{0x80,0x08,0x00,0x32,0x10,0x05,0x32,0x13,
				  0x66,0x32,0x10,0x08,0x37,0x01,0x20,0x07,
				  0x37,0x08,0x28,0x08,0x30,0x30,0x04,0x00,
				  0x40,0x08,0x28,0x08,0x30,0x30,0x04}},  

	{0xD5,	47,	{ 0x00,0x00,0x0A,0x00,0x01,0x05,0x00,0x0A,
				   0x00,0x88,0x88,0x89,0x88,0x23,0x01,0x67,
				   0x45,0x88,0x01,0x88,0x45,0x88,0x88,0x88,
				   0x88,0x88,0x88,0x88,0x88,0x54,0x76,0x10,
				   0x32,0x88,0x54,0x88,0x10,0x88,0x88,0x88,
				   0x88,0x00,0x00,0x00,0x00,0x00,0x00}},
				   
	{0xCC,	1,	{0x02}},  
	
//	{0xDE,	3,	{0x05,0x70,0x04}},

	{0xE0,	35,	{0x79,0x07,0x12,0x15,0x3F,0x3F,0x3F,0x1F,
				  0x3C,0x08,0x0D,0x0E,0x11,0x12,0x10,0x12,
				  0x0C,0x1F,0x07,0x12,0x15,0x3F,0x3F,0x3F,
				  0x1F,0x3C,0x08,0x0D,0x0E,0x11,0x12,0x10,
				  0x12,0x0C,0x1F}},

	{0xB6,	4,	{0x00,0xae,0x00,0xae}}, // 00 81 00 81   //vcom
	
#endif
	//	{0x11, 0, {0x00}}, //Sleep Out
	//	{REGFLAG_DELAY, 150, {}},
	//	{0x29, 0, {0x00}},   //Display On
	//	{REGFLAG_DELAY, 10, {}},
	//	{0x2C, 0 , {0x00}}, //Start GRAM write
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_set_window[] = {

};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY,150, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {

};

static struct LCM_setting_table lcm_compare_id_setting[] = {

	{0xB9,	3,	{0xFF, 0x83, 0x79}},
	{REGFLAG_DELAY, 10, {}}, 	

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

//edit by Magnum 2012-12-18
static void dsi_send_cmdq_tinno(unsigned cmd, unsigned char count, unsigned char *para_list, unsigned char force_update)
{
	unsigned int item[16];
	unsigned char dsi_cmd = (unsigned char)cmd;
	unsigned char dc;
	int index = 0, length = 0;
	
	memset(item,0,sizeof(item));
	if(count+1 > 60)
	{
		LCM_DBG("Exceed 16 entry\n");
		return;
	}
/*
	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05
*/	
	if(count == 0)
	{
		item[0] = 0x0500 | (dsi_cmd<<16);
		length = 1;
	}
	else if(count == 1)
	{
		item[0] = 0x1500 | (dsi_cmd<<16) | (para_list[0]<<24);
		length = 1;
	}
	else
	{
		item[0] = 0x3902 | ((count+1)<<16);//Count include command.
		++length;
		while(1)
		{
			if (index == count+1)
				break;
			if ( 0 == index ){
				dc = cmd;
			}else{
				dc = para_list[index-1];
			}
			// an item make up of 4data. 
			item[index/4+1] |= (dc<<(8*(index%4)));  
			if ( index%4 == 0 ) ++length;
			++index;
		}
	}
	
	dsi_set_cmdq(&item, length, force_update);

}


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_send_cmdq_tinno(cmd, table[i].count, table[i].para_list, force_update);
		//	dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;
		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 7;
		params->dsi.vertical_frontporch					= 14;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 30;
		params->dsi.horizontal_backporch				= 30;
		params->dsi.horizontal_frontporch				= 30;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
	
		
		params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
		params->dsi.fbk_div =30;//30;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	


}


static void init_lcm_registers(void)
{

	/*{0xB9,	3,	{0xFF, 0x83, 0x79}},

	{0xBA,	18,	{0x51,0xA3,0x00,0x16,0xA4,0x00,0x18,0xFF,
				  0x0F,0x29,0x03,0x21,0x23,0x25,0x20,0x02,
				  0x35,0x40}},
	
	{0xB1,	31,	{0x00,0x50,0x44,0xEA,0x90,0x08,0x11,0x11,
				  0x71,0x2F,0x37,0x9A,0x1A,0x42,0x1B,0x6E,
				  0xF1,0x00,0xE6,0xE6,0xE6,0xE6,0xE6,0x00,
				  0x04,0x05,0x0A,0x0B,0x04,0x05,0x6F}},
				 
	{0xB2,	13,	{0x00,0x00,0x3C,0x08,0x0C,0x19,0x22,0x00,
				  0xFF,0x08,0x0C,0x19,0x20}},
	
	//{0xB3,	4,	{0x83, 0x00, 0x31, 0x03}},
	
	{0xB4,	31,	{0x45,0x08,0x00,0x32,0x10,0x05,0x32,0x13,
				  0x29,0x32,0x10,0x08,0x17,0x01,0x28,0x07,
				  0x13,0x08,0x28,0x08,0x30,0x30,0x04,0x00,
				  0x40,0x08,0x28,0x08,0x30,0x30,0x04}},  

	{0xD5,	47,	{0x00,0x00,0x08,0x00,0x01,0x05,0x00,0x02,
				  0x00,0x88,0x99,0x88,0x88,0x01,0x23,0x01,
				  0x23,0x88,0x01,0x88,0x88,0x88,0x88,0x88,
				  0x88,0x76,0x54,0x76,0x54,0x32,0x10,0x32,
				  0x10,0x88,0x88,0x88,0x88,0x88,0x88,0x88,
				  0x88,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0xDE,	3,	{0x05,0x70,0x04}},

	{0xE0,	35,	{0x79,0x07,0x12,0x14,0x3F,0x3F,0x3F,0x25,
				  0x4F,0x06,0x0C,0x0E,0x12,0x17,0x12,0x13,
				  0x14,0x1F,0x07,0x12,0x14,0x3F,0x3F,0x3F,
				  0x25,0x4F,0x06,0x0C,0x0E,0x12,0x17,0x12,
				  0x13,0x14,0x1F}},

	{0x53,	1,	{0x40}}, */

	unsigned int data_array[16];

	data_array[0]=0x00043902;
	data_array[1]=0x7983FFB9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00193902;
	data_array[1]=0x00A351BA;
	data_array[2]=0x1800A416;
	data_array[3]=0x03290FFF;
	data_array[4]=0x20252321;
	data_array[5]=0x00403502;
	dsi_set_cmdq(data_array, 6, 1);


//*************Enable CMD2 Page1  *******************//
	data_array[0]=0x00323902;
	data_array[1]=0x445000B1;
	data_array[2]=0x110890EA;
	data_array[3]=0x372F7111;
	data_array[4]=0x1B421A9A;
	data_array[5]=0xE600F16E;
	data_array[6]=0xE6E6E6E6;
	data_array[7]=0x0A010400;
	data_array[8]=0x6F05040B;
	dsi_set_cmdq(data_array, 9, 1);

//************* AVDD: manual  *******************//
	data_array[0]=0x00143902;
	data_array[1]=0x3C0000B2;
	data_array[2]=0x2219C008;
	data_array[3]=0x0C08FF00;
	data_array[4]=0x00002019;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0]=0x00323902;
	data_array[1]=0x000845B4;
	data_array[2]=0x32051032;
	data_array[3]=0x10322913;
	data_array[4]=0x28011708;
	data_array[5]=0x28081307;
	data_array[6]=0x04303008;
	data_array[7]=0x28084000;
	data_array[8]=0x0430500B;
	dsi_set_cmdq(data_array, 9, 1);

	data_array[0]=0x00483902;
	data_array[1]=0x080000D5;
	data_array[2]=0x00050100;
	data_array[3]=0x99880002;
	data_array[4]=0x23018888;
	data_array[5]=0x01882301;
	data_array[6]=0x88888888;
	data_array[7]=0x54768888;
	data_array[8]=0x10325476;
	data_array[9]=0x88881032;
	data_array[10]=0x88888888;
	data_array[11]=0x00008888;
	data_array[12]=0x00000000;
	dsi_set_cmdq(data_array, 13, 1);

	data_array[0]=0x00043902;
	data_array[1]=0x047006DE;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00363902;
	data_array[1]=0x120779E0;
	data_array[2]=0x3F3F3F14;
	data_array[3]=0X0C064F25;
	data_array[4]=0x1217120E;
	data_array[5]=0x071F1413;
	data_array[6]=0x3F3F1412;
	data_array[7]=0x064F253F;
	data_array[8]=0x17120E0C;
	data_array[9]=0x1F141312;
	dsi_set_cmdq(data_array, 10, 1);


	data_array[0]=0x773A1500;//TE ON 
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x40531500;// TE ON
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x00110500;		// Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	
	data_array[0] = 0x00290500;		// Display On
	dsi_set_cmdq(data_array, 1, 1);
	

}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
 //   init_lcm_registers();
}



static void lcm_suspend(void)
{
	LCM_DBG("%s",__func__);

	LCM_DBG("Magnum  reinit ========================================");
	//lcm_init();
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(120);
}


static void lcm_resume(void)
{
	LCM_DBG("%s",__func__);
	LCM_DBG("Magnum  reinit ========================================");
	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	lcm_init();
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);

		
//		LCM_DBG();
		
//		test_compare_id();
//		LCM_DBG("%s",__func__);
	//	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

#if defined(BUILD_UBOOT) || defined(BUILD_LK)
#include "cust_adc.h"
#define LCM_MAX_VOLTAGE 700 
#define LCM_MIN_VOLTAGE 600 

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static unsigned int lcm_adc_read_chip_id()
{
	int data[4] = {0, 0, 0, 0};
	int tmp = 0, rc = 0, iVoltage = -1;
	rc = IMM_GetOneChannelValue(AUXADC_LCD_ID_CHANNEL, data, &tmp);
	if(rc < 0) {
		printf("read LCD_ID vol error--Liu\n");
		return 0;
	}
	else {
		iVoltage = (data[0]*1000) + (data[1]*10) + (data[2]);
		printf("read LCD_ID success, data[0]=%d, data[1]=%d, data[2]=%d, data[3]=%d, iVoltage=%d\n", 
			data[0], data[1], data[2], data[3], iVoltage);
		if(LCM_MIN_VOLTAGE < iVoltage &&
			iVoltage < LCM_MAX_VOLTAGE)
			return 0;
		else
			return 0;
	}
	return 0;
}
#endif	


static unsigned int test_compare_id(void)
{
	LCM_DBG("%s",__func__);
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  

	//Do reset here
/*	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(25);
	
	SET_RESET_PIN(1);
	MDELAY(50);      */

//	push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);
/*	array[0]=0x00043902;
	array[1]=0x7983FFB9;
	dsi_set_cmdq(array, 2, 1); */
//	lcm_init();

	

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID

	LCM_DBG("%s, id1 = 0x%02x\n", __func__, id);

	return 1;
}
static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  


	//Do reset here
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(25);
	
	SET_RESET_PIN(1);
	MDELAY(50);      
#if 1 
	push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID
	LCM_DBG("%s, id1 = 0x%02x\n", __func__, id);
	#endif

   	return (LCM_ID == id) ? 1 : 1;

}



LCM_DRIVER hx8379a_dsi_vdo_tcl_bidrectional_lcm_drv = 
{
    .name			= "hx8379a_dsi_vdo_tcl_bidrectional",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

