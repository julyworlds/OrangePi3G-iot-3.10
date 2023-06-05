#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 1,
    .polling_mode_ps =0,
    .polling_mode_als =1,		//Ivan Interrupt mode not support
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    .als_level  = { 1,  1,  40, 256,  512, 1024,  1228,  1536,  2000, 65535, 65535, 65535, 65535, 65535, 65535},
    .als_value  = { 1,  1,  1,  51,   51,  181,   638,   1636,  3001, 8001, 10243, 10243, 10243, 10243, 10243},
    .ps_threshold = 2,	//3,
    .ps_threshold_high = 0x10,		//0x10
    .ps_threshold_low = 0x08,		//0x08
    .als_threshold_high = 0xFFFF,
    .als_threshold_low = 0,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

