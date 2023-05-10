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

#if defined(BUILD_LK)
#else

#include <linux/proc_fs.h>   //proc file use 
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(540)
#define FRAME_HEIGHT 										(960)

#define REGFLAG_DELAY             							0XFE11
#define REGFLAG_END_OF_TABLE      							0xFF11   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	{0XF0,5,{0X55,0XAA,0X52,0X08,0X03}},
	{0X90,9,{0X05,0X16,0X09,0X03,0XCD,0X00,0X00,0X00,0X00}},
	{0X91,9,{0X05,0X16,0X0B,0X03,0XCF,0X00,0X00,0X00,0X00}},
	{0X92,11,{0X40,0X0C,0X0D,0X0E,0X0F,0X00,0X8F,0X00,0X00,0X04,0X08}},
	{0X94,8,{0X00,0X08,0X0C,0X03,0XD1,0X03,0XD2,0X0C}},
	{0X95,16,{0X40,0X10,0X00,0X11,0X00,0X12,0X00,0X13,0X00,0X8F,0X00,0X00,0X00,0X04,0X00,0X08}},
	{0X99,2,{0X00,0X00}},
	{0X9A,11,{0X80,0X10,0X03,0XD5,0X03,0XD7,0X00,0X00,0X00,0X00,0X50}},
	{0X9B,6,{0X00,0X00,0X00,0X00,0X00,0X00}},
	{0X9C,2,{0X00,0X00}},
	{0X9D,8,{0X01,0X01,0X01,0X01,0X01,0X01,0X00,0X00}},
	{0X9E,2,{0X00,0X00}},
	{0XA0,10,{0X84,0X00,0X1F,0X1F,0X1F,0X1F,0X08,0X1F,0X0A,0X1F}},
	{0XA1,10,{0X1F,0X1F,0X1F,0X1F,0X0C,0X1F,0X0E,0X1F,0X1F,0X1F}},
	{0XA2,10,{0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X02,0X1F,0X06,0X1F}},
	{0XA3,10,{0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F}},
	{0XA4,10,{0X1F,0X1F,0X1F,0X1F,0X1F,0X07,0X1F,0X03,0X1F,0X0F}},
	{0XA5,10,{0X1F,0X0D,0X1F,0X1F,0X1F,0X1F,0X1F,0X0B,0X1F,0X09}},
	{0XA6,10,{0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X01,0X05}},
	{0XA7,10,{0X03,0X07,0X1F,0X1F,0X1F,0X1F,0X0B,0X1F,0X09,0X1F}},
	{0XA8,10,{0X1F,0X1F,0X1F,0X1F,0X0F,0X1F,0X0D,0X1F,0X1F,0X1F}},
	{0XA9,10,{0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X05,0X1F,0X01,0X1F}},
	{0XAA,10,{0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F}},
	{0XAB,10,{0X1F,0X1F,0X1F,0X1F,0X1F,0X00,0X1F,0X04,0X1F,0X0C}},
	{0XAC,10,{0X1F,0X0E,0X1F,0X1F,0X1F,0X1F,0X1F,0X08,0X1F,0X0A}},
	{0XAD,10,{0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X06,0X02}},
	{0XF0,5,{0X55,0XAA,0X52,0X08,0X00}},
	{0XBC,3,{0X00,0X00,0X00}},
	{0XB8,4,{0X01,0XAF,0X8F,0X8F}},
	{0XF0,5,{0X55,0XAA,0X52,0X08,0X01}},
	{0XD1,16,{0X00,0X00,0X00,0X26,0X00,0X5E,0X00,0X88,0X00,0XA8,0X00,0XDB,0X01,0X02,0X01,0X3D}},
	{0XD2,16,{0X01,0X67,0X01,0XA6,0X01,0XD3,0X02,0X16,0X02,0X49,0X02,0X4B,0X02,0X7B,0X02,0XB3}},
	{0XD3,16,{0X02,0XD9,0X03,0X0E,0X03,0X31,0X03,0X61,0X03,0X80,0X03,0XA5,0X03,0XBD,0X03,0XD2}},
	{0XD4,4,{0X03,0XE5,0X03,0XFF}},
	{0XD5,16,{0X00,0X00,0X00,0X26,0X00,0X5E,0X00,0X88,0X00,0XA8,0X00,0XDB,0X01,0X02,0X01,0X3D}},
	{0XD6,16,{0X01,0X67,0X01,0XA6,0X01,0XD3,0X02,0X16,0X02,0X49,0X02,0X4B,0X02,0X7B,0X02,0XB3}},
	{0XD7,16,{0X02,0XD9,0X03,0X0E,0X03,0X31,0X03,0X61,0X03,0X80,0X03,0XA5,0X03,0XBD,0X03,0XD2}},
	{0XD8,4,{0X03,0XE5,0X03,0XFF}},
	{0XD9,16,{0X00,0X00,0X00,0X26,0X00,0X5E,0X00,0X88,0X00,0XA8,0X00,0XDB,0X01,0X02,0X01,0X3D}},
	{0XDD,16,{0X01,0X67,0X01,0XA6,0X01,0XD3,0X02,0X16,0X02,0X49,0X02,0X4B,0X02,0X7B,0X02,0XB3}},
	{0XDE,16,{0X02,0XD9,0X03,0X0E,0X03,0X31,0X03,0X61,0X03,0X80,0X03,0XA5,0X03,0XBD,0X03,0XD2}},
	{0XDF,4,{0X03,0XE5,0X03,0XFF}},
	{0XE0,16,{0X00,0X00,0X00,0X26,0X00,0X5E,0X00,0X88,0X00,0XA8,0X00,0XDB,0X01,0X02,0X01,0X3D}},
	{0XE1,16,{0X01,0X67,0X01,0XA6,0X01,0XD3,0X02,0X16,0X02,0X49,0X02,0X4B,0X02,0X7B,0X02,0XB3}},
	{0XE2,16,{0X02,0XD9,0X03,0X0E,0X03,0X31,0X03,0X61,0X03,0X80,0X03,0XA5,0X03,0XBD,0X03,0XD2}},
	{0XE3,4,{0X03,0XE5,0X03,0XFF}},
	{0XE4,16,{0X00,0X00,0X00,0X26,0X00,0X5E,0X00,0X88,0X00,0XA8,0X00,0XDB,0X01,0X02,0X01,0X3D}},
	{0XE5,16,{0X01,0X67,0X01,0XA6,0X01,0XD3,0X02,0X16,0X02,0X49,0X02,0X4B,0X02,0X7B,0X02,0XB3}},
	{0XE6,16,{0X02,0XD9,0X03,0X0E,0X03,0X31,0X03,0X61,0X03,0X80,0X03,0XA5,0X03,0XBD,0X03,0XD2}},
	{0XE7,4,{0X03,0XE5,0X03,0XFF}},
	{0XE8,16,{0X00,0X00,0X00,0X26,0X00,0X5E,0X00,0X88,0X00,0XA8,0X00,0XDB,0X01,0X02,0X01,0X3D}},
	{0XE9,16,{0X01,0X67,0X01,0XA6,0X01,0XD3,0X02,0X16,0X02,0X49,0X02,0X4B,0X02,0X7B,0X02,0XB3}},
	{0XEA,16,{0X02,0XD9,0X03,0X0E,0X03,0X31,0X03,0X61,0X03,0X80,0X03,0XA5,0X03,0XBD,0X03,0XD2}},
	{0XEB,4,{0X03,0XE5,0X03,0XFF}},
	{0XB0,3,{0X07,0X07,0X07}},
	{0XB1,3,{0X07,0X07,0X07}},
	{0XB3,3,{0X11,0X11,0X11}},
	{0XB4,3,{0X09,0X09,0X09}},
	{0XB6,3,{0X44,0X44,0X44}},
	{0XB7,3,{0X34,0X34,0X34}},
	{0XB9,3,{0X34,0X34,0X34}},
	{0XBA,3,{0X14,0X14,0X14}},
	{0XBC,3,{0X00,0X80,0X00}},
	{0XBD,3,{0X00,0X80,0X00}},
	{0XBE,1,{0X1D}},//1A
	{0XF0,5,{0X55,0XAA,0X52,0X08,0X02}},
	{0XC5,1,{0X06}},
        {0x4C,1,{0x10}},//ce on
	{0x35,1,{0x00}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY,120,{}},
	{0x29,0,{0x00}},
	{REGFLAG_DELAY,100,{}},	

	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};





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
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				//UDELAY(5);//soso add or it will fail to send register
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


		params->dsi.mode   = SYNC_EVENT_VDO_MODE;
	
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

		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 14;
		params->dsi.vertical_frontporch					= 16;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 8;
		params->dsi.horizontal_backporch				= 24;
		params->dsi.horizontal_frontporch				= 32;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		params->dsi.pll_div1=0;	
		params->dsi.pll_div2=1;
                params->dsi.fbk_div=17; 
}

static unsigned int lcm_compare_id(void);

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(6);//Must > 5ms
    SET_RESET_PIN(1);
    MDELAY(50);//Must > 50ms

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(6);//Must > 5ms
    SET_RESET_PIN(1);
    MDELAY(50);//Must > 50ms

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	lcm_init();
	
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

#define LCM_RM68191_ID          (0x8191)

static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xf0, 5, {0x55, 0xaa, 0x52, 0x08, 0x01}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static unsigned int lcm_compare_id(void)
{
	unsigned int id;
	unsigned char buffer[5];
	unsigned int array[5];

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(150);

	push_table(lcm_compare_id_setting,sizeof(lcm_compare_id_setting)/sizeof(struct LCM_setting_table), 1);

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xc5, buffer, 2);
	id = ((buffer[0] << 8) | buffer[1]);

	#if defined(BUILD_LK)
		printf("RM68191 uboot %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("RM68191 kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif

	return ((LCM_RM68191_ID == id)? 1 : 0);
}


LCM_DRIVER ry_rm68191_qhd_dsi_vdo_hf_lcm_drv = 
{
    .name			= "ry_rm68191_qhd_dsi_vdo_hf",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
};

