#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

#include "cust_gpio_usage.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (540)
#define FRAME_HEIGHT (960)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_OTM9608A									0x9608

//#define LCM_DEBUG
#if defined(BUILD_LK)
	#if defined(BUILD_LK)
	#define LCM_LOG(fmt, args...)    printf(fmt, ##args)
	#else
	#define LCM_LOG(fmt, args...)    printk(fmt, ##args)	
	#endif
#else
#define LCM_LOG(fmt, args...)	 printk(fmt, ##args)	
#endif

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)  (lcm_util.set_gpio_out((n), (v)))

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
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

	//OTM9608_TM470_MIPI_2LINE-20141129
	{0x00, 1, {0x00}},
	{0xff, 3, {0x96,0x08,0x01}},

	{0x00, 1, {0x80}},
	{0xff, 2, {0x96,0x08}},

	{0x00, 1, {0x00}},
	{0xa0, 1, {0x00}},

	{0x00, 1, {0x80}},
	{0xb3, 5, {0x00,0x00,0x20,0x00,0x00}},

	{0x00, 1, {0xc0}},
	{0xb3, 1, {0x09}},

	{0x00, 1, {0x80}},
	{0xc0, 9, {0x00,0x48,0x00,0x0f,0x11,0x00,0x48,0x0f,0x11}},

	{0x00, 1, {0x92}},
	{0xc0, 4, {0x00,0x10,0x00,0x13}},

	{0x00, 1, {0xa2}},
	{0xc0, 3, {0x0c,0x05,0x02}},

	{0x00, 1, {0xb3}},
	{0xc0, 2, {0x00,0x10}},//50

	{0x00, 1, {0x81}},
	{0xc1, 1, {0x66}},

	{0x00, 1, {0x80}},
	{0xc4, 2, {0x00,0x84}},

	{0x00, 1, {0xa0}},
	{0xc4, 8, {0x33,0x09,0x90,0x2b,0x33,0x09,0x90,0x54}},

	{0x00, 1, {0x80}},
	{0xc5, 4, {0x08,0x00,0xa0,0x11}},

	{0x00, 1, {0x90}},
	{0xc5, 7, {0xd6,0x12,0x01,0x79,0x55,0x55,0x34}},//0xd6,0x12,0x01,0x79,0x33,0x33,0x34

	{0x00, 1, {0xa0}},
	{0xc5, 7, {0xd6,0x13,0x00,0x79,0x55,0x55,0x34}},//0xd6,0x13,0x00,0x79,0x33,0x33,0x34

	{0x00, 1, {0x00}},
	{0xd8, 2, {0x6f,0x6f}},

	{0x00, 1, {0x00}},
	{0xd9, 1, {0x3a}},//3a 48 40 3f

	{0x00, 1, {0x00}},
	{0xe1, 16, {0x01,0x0d,0x14,0x0d,0x06,0x12,0x0c,0x0c,0x02,0x06,0x09,0x07,0x0e,0x12,0x0e,0x01}},

	{0x00, 1, {0x00}},
	{0xe2, 16, {0x01,0x0e,0x15,0x0d,0x06,0x13,0x0c,0x0c,0x01,0x05,0x08,0x07,0x0d,0x12,0x0e,0x01}},

	{0x00, 1, {0xb0}},
	{0xc5, 2, {0x04,0xa8}},

	{0x00, 1, {0x80}},
	{0xc6, 1, {0x64}},

	{0x00, 1, {0xb0}},
	{0xc6, 5, {0x03,0x10,0x00,0x1f,0x12}},

	{0x00, 1, {0xb7}},
	{0xb0, 1, {0x10}},

	{0x00, 1, {0xc0}},
	{0xb0, 1, {0x55}},

	{0x00, 1, {0xb1}},
	{0xb0, 1, {0x03}},

	{0x00, 1, {0x80}},
	{0xcb, 10, {0x05,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0x90}},
	{0xcb, 15, {0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xa0}},
	{0xcb, 15, {0x00,0x00,0x00,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xb0}},
	{0xcb, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xc0}},
	{0xcb, 15, {0x55,0x55,0x00,0x00,0x00,0x04,0x00,0x04,0x00,0x04,0x00,0x04,0x04,0x04,0x00}},

	{0x00, 1, {0xd0}},
	{0xcb, 15, {0x04,0x00,0x00,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x04,0x00,0x04,0x00,0x04}},

	{0x00, 1, {0xe0}},
	{0xcb, 10, {0x00,0x04,0x04,0x04,0x00,0x04,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xf0}},
	{0xcb, 10, {0x0f,0x00,0xcc,0x00,0x00,0x0f,0x00,0xcc,0xc3,0x00}},

	{0x00, 1, {0x80}},
	{0xcc, 10, {0x25,0x26,0x00,0x00,0x00,0x0c,0x00,0x0a,0x00,0x10}},

	{0x00, 1, {0x90}},
	{0xcc, 15, {0x00,0x0e,0x02,0x04,0x00,0x06,0x00,0x00,0x00,0x00,0x25,0x26,0x00,0x00,0x00}},

	{0x00, 1, {0xa0}},
	{0xcc, 15, {0x0b,0x00,0x09,0x00,0x0f,0x00,0x0d,0x01,0x03,0x00,0x05,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xb0}},
	{0xcc, 10, {0x26,0x25,0x00,0x00,0x00,0x0d,0x00,0x0f,0x00,0x09}},

	{0x00, 1, {0xc0}},
	{0xcc, 15, {0x00,0x0b,0x03,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x26,0x25,0x00,0x00,0x00}},

	{0x00, 1, {0xd0}},
	{0xcc, 15, {0x0e,0x00,0x10,0x00,0x0a,0x00,0x0c,0x04,0x02,0x00,0x06,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0x80}},
	{0xce, 12, {0x8a,0x03,0x28,0x89,0x03,0x28,0x88,0x03,0x28,0x87,0x03,0x28}},

	{0x00, 1, {0x90}},
	{0xce, 15, {0x38,0x0f,0x28,0x38,0x0e,0x28,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xa0}},
	{0xce, 14, {0x38,0x06,0x03,0xc1,0x00,0x28,0x00,0x38,0x05,0x03,0xc2,0x00,0x28,0x00}},

	{0x00, 1, {0xb0}},
	{0xce, 14, {0x38,0x04,0x03,0xc3,0x00,0x28,0x00,0x38,0x03,0x03,0xc4,0x00,0x28,0x00}},

	{0x00, 1, {0xc0}},
	{0xce, 14, {0x38,0x02,0x03,0xc5,0x00,0x28,0x00,0x38,0x01,0x03,0xc6,0x00,0x28,0x00}},

	{0x00, 1, {0xd0}},
	{0xce, 14, {0x38,0x00,0x03,0xc7,0x00,0x28,0x00,0x30,0x00,0x03,0xc8,0x00,0x28,0x00}},

	{0x00, 1, {0x80}},
	{0xcf, 14, {0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

	{0x00, 1, {0x90}},
	{0xcf, 14, {0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

	{0x00, 1, {0xa0}},
	{0xcf, 14, {0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

	{0x00, 1, {0xb0}},
	{0xcf, 14, {0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

	{0x00, 1, {0xc0}},
	{0xcf, 10, {0x01,0x01,0x20,0x20,0x00,0x00,0x02,0x01,0x00,0x00}},

	{0x00, 1, {0x00}},
	{0xff, 3, {0xff,0xff,0xff}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 20, {}},

// Setting ending by predefined flag
{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_compare_id_setting[] = {
    // zht 
	{0x00,1,{0x00}},
	{0xFF,3,{0x96,0x08,0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

//static int vcom = 0x30;
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned int cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
	   /* case 0xD9:
	        table[i].para_list[0]=0x00;
		table[i].para_list[1]=vcom;
		dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		vcom+=1;
		break;*/	
            case REGFLAG_DELAY:
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE:
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
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

	params->dsi.mode   = SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE;
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;
		
	params->dsi.packet_size=256;
	params->dsi.intermediat_buffer_num = 2;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
	params->dsi.word_count=FRAME_WIDTH*3;	//DSI CMD mode need set these two bellow params, different to 6577

	params->dsi.vertical_sync_active				= 4;//6
	params->dsi.vertical_backporch					= 16;
	params->dsi.vertical_frontporch					= 15;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 8;//6
	params->dsi.horizontal_backporch				= 60;//32
	params->dsi.horizontal_frontporch				= 60;//32
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	
	// Bit rate calculation
  	//lizc improve clock for c919 haixu 20141009
	params->dsi.pll_div1=0; 	// div1=0,1,2,3;div1_real=1,2,4,4
	params->dsi.pll_div2=1; 	// div2=0,1,2,3;div2_real=1,2,4,4
	params->dsi.fbk_sel=1;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
	params->dsi.fbk_div=17; 	// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(2); 
	SET_RESET_PIN(0);
	MDELAY(10); 
	
	SET_RESET_PIN(1);
	MDELAY(120); 

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	unsigned int data_array[16];
	
	data_array[0]=0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);

	SET_RESET_PIN(1);
	MDELAY(2);
	SET_RESET_PIN(0);
    MDELAY(10);
}

static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_esd_recover()
{
    lcm_init();
    return TRUE;
}

static void lcm_setpwm(unsigned int divider)
{
	// TBD
}

static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	MDELAY(2);
	SET_RESET_PIN(0);
	MDELAY(30);
	SET_RESET_PIN(1);
	MDELAY(200);

    push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);  //zht 


	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xa1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;

	#if defined(BUILD_LK)
		printf("OTM9608A uboot %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("OTM9608A kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif

	return (LCM_ID_OTM9608A == id)?1:0;
}

LCM_DRIVER otm9608a_qhd_dsi_vdo_lg50_drv = 
{
    .name			= "otm9608a_qhd_dsi_vdo_lg50",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
};

