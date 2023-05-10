#include "mach/mt_reg_base.h"
#include "mach/mt_reg_dump.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <mach/sync_write.h>
#include <mach/dbg_dump.h>
#include <linux/kallsyms.h>
#include <linux/init.h>
#include <linux/mtk_ram_console.h>
#include <linux/delay.h>

extern int mt_ahb_abt_dump(char *buf);


static struct reg_dump_driver_data reg_dump_driver_data =
{
    .mcu_regs = (MCUSYS_CFGREG_BASE + 0x300),
};

static struct platform_device reg_dump_device =
{
    .name = "dbg_reg_dump",
    .dev =
    {
        .platform_data = &(reg_dump_driver_data),
    },
};

/*
 * mt_reg_dump_init: initialize driver.
 * Always return 0.
 */

static int __init mt_reg_dump_init(void)
{
	int err;

  err = platform_device_register(&(reg_dump_device));
  if (err) {
      pr_err("Fail to register reg_dump_device");
      return err;
  }

  return 0;
}

u32 mt_get_cpu_pc(int cpu_id)
{
    if(cpu_id >= NR_CPUS)
        return 0;
    mt65xx_reg_sync_writel(DBG_MON_CTL_CORE0_PC + cpu_id, DBG_MON_CTL);
    return *(volatile u32 *)(DBG_MON_FLAG);
}


void mt_irq_dump(void)
{
    int cpu_id;
    int count = 2;
    char str[128];
    /*
      aee_wdt_printf("GICD_ISENABLER0 = 0x%x, GICD_ISPENDR0 = 0x%x, GICD_ISACTIVER0 = 0x%x\n",
      *(volatile u32 *)(GIC_DIST_BASE + 0x100),
      *(volatile u32 *)(GIC_DIST_BASE + 0x200),
      *(volatile u32 *)(GIC_DIST_BASE + 0x300));
      */
    aee_sram_fiq_log("dump other cpu pc\n");

    while(count > 0)
    {
        for(cpu_id = 1; cpu_id < NR_CPUS; cpu_id++)
        {
            sprintf(str, "cpu-%d pc=0x%08x\n", cpu_id, mt_get_cpu_pc(cpu_id));
            aee_sram_fiq_log(str);
        }
        count--;
        udelay(10);
    }
}

int reg_dump_platform(char *buf)
{
    mt_ahb_abt_dump(buf);
    return 0;
}

arch_initcall(mt_reg_dump_init);
