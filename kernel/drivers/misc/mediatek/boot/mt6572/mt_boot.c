#define pr_fmt(fmt) "["KBUILD_MODNAME"] " fmt
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/kfifo.h>

#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

#include <mach/mt_boot.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>

/* hardware version register 
   [warning]
   APHW_xxx and APSW_xxx are different between MT6572 and MT6582
 */
#define VER_BASE            (DEVINFO_BASE)
#define APHW_VER            (VER_BASE)
#define APSW_VER            (VER_BASE + 0x04)
#define APHW_CODE           (VER_BASE + 0x08)
#define APHW_SUBCODE        (VER_BASE + 0x0C)

/* this vairable will be set by mt_fixup.c */
META_COM_TYPE g_meta_com_type = META_UNKNOWN_COM;
unsigned int g_meta_com_id = 0;

struct meta_driver {
    struct device_driver driver;
    const struct platform_device_id *id_table;
};

static struct meta_driver meta_com_type_info = {
    .driver  = {
        .name = "meta_com_type_info",
        .bus = &platform_bus_type,
        .owner = THIS_MODULE,
    },
    .id_table = NULL,
};

static struct meta_driver meta_com_id_info = {
    .driver = {
        .name = "meta_com_id_info",
        .bus = &platform_bus_type,
        .owner = THIS_MODULE,
    },
    .id_table = NULL,
};

/* return hardware version */
unsigned int get_chip_code(void)
{     
    return DRV_Reg32(APHW_CODE);
}

unsigned int get_chip_hw_ver_code(void)
{   
    return DRV_Reg32(APHW_VER);
}

unsigned int get_chip_sw_ver_code(void)
{  
    return DRV_Reg32(APSW_VER);
}

unsigned int get_chip_hw_subcode(void)
{
    return DRV_Reg32(APHW_SUBCODE);
}

CHIP_SW_VER mt_get_chip_sw_ver(void)
{
    return (CHIP_SW_VER)get_chip_sw_ver_code();
}

CHIP_VER get_chip_eco_ver(void) /*TO BE REMOVED*/
{   
    return DRV_Reg32(APHW_VER);
}

bool com_is_enable(void)  // usb android will check whether is com port enabled default. in normal boot it is default enabled. 
{
    if(g_boot_mode == NORMAL_BOOT) {
        return false;
    } else {
        return true;
    }
}

static ssize_t meta_com_type_show(struct device_driver *driver, char *buf)
{
    return sprintf(buf, "%d\n", g_meta_com_type);
}

static ssize_t meta_com_type_store(struct device_driver *driver, const char *buf, size_t count)
{
    /*Do nothing*/
    return count;
}

DRIVER_ATTR(meta_com_type_info, 0644, meta_com_type_show, meta_com_type_store);


static ssize_t meta_com_id_show(struct device_driver *driver, char *buf)
{
    return sprintf(buf, "%d\n", g_meta_com_id);
}

static ssize_t meta_com_id_store(struct device_driver *driver, const char *buf, size_t count)
{
    /*Do nothing*/
    return count;
}

DRIVER_ATTR(meta_com_id_info, 0644, meta_com_id_show, meta_com_id_store);


static int __init boot_mod_init(void)
{
    int ret;
    
    if(g_boot_mode == META_BOOT || g_boot_mode == ADVMETA_BOOT || g_boot_mode == ATE_FACTORY_BOOT || g_boot_mode == FACTORY_BOOT) {
        /* register driver and create sysfs files */
        ret = driver_register(&meta_com_type_info.driver);
        if (ret) {
            pr_warn("fail to register META COM TYPE driver\n");
        }
        ret = driver_create_file(&meta_com_type_info.driver, &driver_attr_meta_com_type_info);
        if (ret) {
            pr_warn("fail to create META COM TPYE sysfs file\n");
        }

        ret = driver_register(&meta_com_id_info.driver);
        if (ret) {
            pr_warn("fail to register META COM ID driver\n");
        }
        ret = driver_create_file(&meta_com_id_info.driver, &driver_attr_meta_com_id_info);
        if (ret) {
            pr_warn("fail to create META COM ID sysfs file\n");
        }
    }    
    
    return 0;
}

static void __exit boot_mod_exit(void)
{
}

module_init(boot_mod_init);
module_exit(boot_mod_exit);
MODULE_DESCRIPTION("MT6572 Boot Information Querying Driver");
MODULE_LICENSE("Proprietary");
EXPORT_SYMBOL(mt_get_chip_sw_ver);
