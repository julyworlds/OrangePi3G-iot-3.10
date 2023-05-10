#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/aee.h>

#include <mach/irqs.h>
//#include <mach/mt_cirq.h>
#include <mach/mt_spm.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_dcm.h>
#include <mach/mt_dormant.h>
#include <mach/eint.h>
#include <mach/mtk_ccci_helper.h>
#include <board-custom.h>
#include <mach/wd_api.h>

#define SPM_SUSPEND_FIRMWARE_VER   "pcm_suspend_20131115A"

static const u32 spm_pcm_suspend[] = {
    0x19c0001f, 0x001c4bd7, 0x1800001f, 0x038e0e3f, 0x1b80001f, 0x20000042,
    0x1800001f, 0x038e0e1e, 0x19c0001f, 0x001c4be7, 0x1a00001f, 0x10006604,
    0xe2200006, 0x1b80001f, 0x20000104, 0xe2200005, 0x1b80001f, 0x2000648c,
    0x1b00001f, 0x7ffff7ff, 0xf0000000, 0x17c07c1f, 0x1b00001f, 0x3fffe7ff,
    0x1b80001f, 0x20000004, 0xd800060c, 0x17c07c1f, 0x1a00001f, 0x10006604,
    0xe2200004, 0x1b80001f, 0x20000104, 0xe2200007, 0x1b80001f, 0x20000104,
    0x1b00001f, 0x3fffefff, 0xc2801d60, 0x1291041f, 0x19c0001f, 0x001c6bd7,
    0x1800001f, 0x038e0e3f, 0x1800001f, 0x039e0e3f, 0x19c0001f, 0x001c23d7,
    0xf0000000, 0x17c07c1f, 0xd8000786, 0x17c07c1f, 0x18c0001f, 0x10006234,
    0xc0c01540, 0x1200041f, 0x18c0001f, 0x10006240, 0xc0c01540, 0x1200041f,
    0xe8208000, 0x1000627c, 0x00000005, 0x1b80001f, 0x20000020, 0xe8208000,
    0x1000627c, 0x00000009, 0x19c0001f, 0x001c4ba7, 0x1b80001f, 0x20000030,
    0x1800001f, 0x038a0e1e, 0x1b80001f, 0x20000300, 0x1800001f, 0x028a0e1e,
    0x1800001f, 0x028a0a1e, 0x1800001f, 0x028a0a12, 0x19c0001f, 0x00144ba7,
    0x19c0001f, 0x00104ba5, 0x1b80001f, 0x20000030, 0xe8208000, 0x10006354,
    0xfffff806, 0x19c0001f, 0x00114aa5, 0x1b00001f, 0xbfffe7ff, 0xf0000000,
    0x17c07c1f, 0x1b80001f, 0x20000fdf, 0x1890001f, 0x10006608, 0x80c98801,
    0x810a0801, 0x10928c1f, 0xa0911002, 0x8080080d, 0x1b00001f, 0xbfffe7ff,
    0x18d0001f, 0x1000c104, 0xa0800c02, 0xd8001502, 0x17c07c1f, 0x1b00001f,
    0x3fffe7ff, 0x1b80001f, 0x20000004, 0xd800150c, 0x17c07c1f, 0xe8208000,
    0x10006354, 0xffffffff, 0x19c0001f, 0x00184be5, 0x19c0001f, 0x001c4be5,
    0x1b80001f, 0x2000000a, 0x1880001f, 0x10006320, 0xc0c01c40, 0xe080000f,
    0xd8001503, 0x17c07c1f, 0xe080001f, 0xe8208000, 0x1000627c, 0x00000005,
    0xe8208000, 0x1000627c, 0x00000004, 0xd8001266, 0x17c07c1f, 0x18c0001f,
    0x10006240, 0xc0c01740, 0x17c07c1f, 0x1800001f, 0x028a0a1e, 0x1800001f,
    0x028a0e1e, 0x1800001f, 0x038a0e1e, 0x1800001f, 0x038e0e1e, 0xd80014c6,
    0x17c07c1f, 0x18c0001f, 0x10006234, 0xc0c01740, 0x17c07c1f, 0xe8208000,
    0x10006234, 0x000f0f12, 0xc2801d60, 0x1290841f, 0x1b00001f, 0x7ffff7ff,
    0xf0000000, 0x17c07c1f, 0xe0e00f16, 0x1380201f, 0xe0e00f1e, 0x1380201f,
    0xe0e00f0e, 0x1b80001f, 0x20000100, 0x1380201f, 0xe0e00f0c, 0xe0e00f0d,
    0xe0e00e0d, 0xe0e00c0d, 0xe0e0080d, 0xe0e0000d, 0xf0000000, 0x17c07c1f,
    0xe0e00f0d, 0xe0e00f1e, 0xe0e00f12, 0xf0000000, 0x17c07c1f, 0xd80018ea,
    0x17c07c1f, 0xe2e00036, 0x1380201f, 0xe2e0003e, 0x1380201f, 0xe2e0002e,
    0x1380201f, 0xd82019ea, 0x17c07c1f, 0xe2e0006e, 0xe2e0004e, 0xe2e0004c,
    0x1b80001f, 0x20000020, 0xe2e0004d, 0xf0000000, 0x17c07c1f, 0xd8001aaa,
    0x17c07c1f, 0xe2e0006d, 0xe2e0002d, 0xd8201b4a, 0x17c07c1f, 0xe2e0002f,
    0xe2e0003e, 0xe2e00032, 0xf0000000, 0x17c07c1f, 0xa1d10407, 0x1b80001f,
    0x20000020, 0x10c07c1f, 0xf0000000, 0x17c07c1f, 0xa1d08407, 0x1b80001f,
    0x20000080, 0x80eab401, 0x1a00001f, 0x10006814, 0xe2000003, 0xf0000000,
    0x17c07c1f, 0x18d0001f, 0x10006b00, 0xa1002803, 0x18c0001f, 0x10006b00,
    0xe0c00004, 0xf0000000, 0x17c07c1f, 0x18d0001f, 0x10006b00, 0x81202803,
    0x18c0001f, 0x10006b00, 0xe0c00004, 0xf0000000, 0x17c07c1f, 0x17c07c1f,
    0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x1840001f, 0x00000001,
    0xe8208000, 0x10006b00, 0x00000000, 0x1b00001f, 0x3fffe7ff, 0x1b80001f,
    0xd00f0000, 0x8880000c, 0x3fffe7ff, 0xd8003662, 0x1140041f, 0x1990001f,
    0x10006400, 0x81471801, 0x81879801, 0xe8208000, 0x10006354, 0xfffff806,
    0xc0c01b80, 0x17c07c1f, 0xd80025c5, 0x17c07c1f, 0x89c00007, 0xffffefff,
    0x18c0001f, 0x10006200, 0xc0c01a20, 0x12807c1f, 0xe8208000, 0x1000625c,
    0x00000001, 0xc0c01a20, 0x1280041f, 0x18c0001f, 0x10006208, 0xc0c01a20,
    0x12807c1f, 0xe8208000, 0x10006248, 0x00000000, 0xc0c01a20, 0x1280041f,
    0xc2801d60, 0x1290041f, 0x19c0001f, 0x00014aa5, 0x1800001f, 0x00000012,
    0x1800001f, 0x00000a12, 0x1800001f, 0x000a0a12, 0x1800001f, 0x028a0a12,
    0xe8208000, 0x10006310, 0x0b1600f8, 0x1b00001f, 0xbfffe7ff, 0x1b80001f,
    0x90100000, 0x80c00400, 0xd80028c3, 0x80990400, 0xd8002b02, 0x17c07c1f,
    0xd82030c2, 0x17c07c1f, 0x19c0001f, 0x001c4bd7, 0x1800001f, 0x038e0e3f,
    0x1b80001f, 0x20000042, 0x1800001f, 0x038e0e16, 0x19c0001f, 0x001c4be7,
    0x1a00001f, 0x10006604, 0xe2200006, 0x1b80001f, 0x20000104, 0xe2200005,
    0x1b80001f, 0x2000648c, 0xd8002bc6, 0x17c07c1f, 0x18c0001f, 0x10006234,
    0xc0c01540, 0x1200041f, 0xd8002c86, 0x17c07c1f, 0x18c0001f, 0x10006240,
    0xc0c01540, 0x1200041f, 0xe8208000, 0x1000627c, 0x00000005, 0x1b80001f,
    0x20000020, 0xe8208000, 0x1000627c, 0x00000009, 0x19c0001f, 0x001c4ba7,
    0x1800001f, 0x038e0e16, 0x1800001f, 0x03800e16, 0x1b80001f, 0x20000300,
    0x1800001f, 0x00000e16, 0x1800001f, 0x00000016, 0x10007c1f, 0x19c0001f,
    0x00144ba7, 0x19c0001f, 0x00104ba5, 0x1b80001f, 0x20000030, 0xe8208000,
    0x10006354, 0xfffff806, 0x19c0001f, 0x00114aa5, 0xd00031e0, 0x17c07c1f,
    0x1800001f, 0x02800a12, 0x1b80001f, 0x20000300, 0x1800001f, 0x00000a12,
    0x1800001f, 0x00000012, 0x10007c1f, 0x19c0001f, 0x00014a25, 0xd8003665,
    0x17c07c1f, 0x18c0001f, 0x10006208, 0x1212841f, 0xc0c017e0, 0x12807c1f,
    0xe8208000, 0x10006248, 0x00000001, 0x1890001f, 0x10006248, 0x81040801,
    0xd8203364, 0x17c07c1f, 0x1b80001f, 0x20000020, 0xc0c017e0, 0x1280041f,
    0x18c0001f, 0x10006200, 0x1212841f, 0xc0c017e0, 0x12807c1f, 0xe8208000,
    0x1000625c, 0x00000000, 0x1890001f, 0x1000625c, 0x81040801, 0xd8003584,
    0x17c07c1f, 0xc0c017e0, 0x1280041f, 0x19c0001f, 0x00015a20, 0x10007c1f,
    0x80cab001, 0x808cb401, 0x80800c02, 0xd8203782, 0x17c07c1f, 0xa1d78407,
    0x1ac0001f, 0x55aa55aa, 0xf0000000
};
#define PCM_SUSPEND_BASE        __pa(spm_pcm_suspend)
#define PCM_SUSPEND_LEN         (447)                       /* # words */
#define PCM_SUSPEND_VEC0        EVENT_VEC(WAKE_ID_26M_WAKE, 1, 0, 0)      
#define PCM_SUSPEND_VEC1        EVENT_VEC(WAKE_ID_26M_SLP, 1, 0, 22)    
#define PCM_SUSPEND_VEC2        EVENT_VEC(WAKE_ID_AP_WAKE, 1, 0, 50)      
#define PCM_SUSPEND_VEC3        EVENT_VEC(WAKE_ID_AP_SLEEP, 1, 0, 97)
//#define PCM_SUSPEND_VEC4      EVENT_VEC(WAKE_ID_PCM_WDT, 1, 1, 225)
#define PCM_SUSPEND_VEC4        0

#define SUSPEND_PCM_TIMER_VAL   (10*1024)
#define SUSPEND_PCM_WDT_VAL     (0*1024) //need larger than PCM Timer

//#if defined (FPGA_EARLY_PORTING)
#if 1
    #define WAKE_SRC_FOR_SUSPEND  (WAKE_SRC_KP | WAKE_SRC_EINT | WAKE_SRC_CCIF | WAKE_SRC_USB_CD | WAKE_SRC_MD_WDT | WAKE_SRC_SYSPWREQ | WAKE_SRC_CONN |WAKE_SRC_THERM |WAKE_SRC_GPT)
#else    
    #define WAKE_SRC_FOR_SUSPEND                     \
        (WAKE_SRC_KP | WAKE_SRC_EINT | WAKE_SRC_CCIF | WAKE_SRC_USB_CD | WAKE_SRC_SYSPWREQ | WAKE_SRC_MD_WDT)
#endif

SPM_PCM_CONFIG pcm_config_suspend ={
    .scenario = SPM_PCM_KERNEL_SUSPEND,
    .ver = SPM_SUSPEND_FIRMWARE_VER,
    .spm_turn_off_26m = true,
    .pcm_firmware_addr =  PCM_SUSPEND_BASE,
    .pcm_firmware_len = PCM_SUSPEND_LEN,
    .pcm_pwrlevel = PWR_LVNA,         //no necessary to set pwr level when suspend
    .spm_request_uart_sleep = true,
    .pcm_vsr = {PCM_SUSPEND_VEC0,PCM_SUSPEND_VEC1,PCM_SUSPEND_VEC2,PCM_SUSPEND_VEC3,PCM_SUSPEND_VEC4,0,0,0},
    
    /*Wake up event mask*/
    .md_mask = MDCONN_UNMASK,   /* unmask MD1 and MD2 */
    .mm_mask = MMALL_UNMASK,   /* unmask DISP and MFG */
    
    .sync_r0r7=true,
    
    /*AP Sleep event mask*/
    .wfi_scu_mask=true ,   
    .wfi_l2c_mask=true, 
    .wfi_op=REDUCE_AND,//We need to ignore CPU 1 in FPGA platform
    .wfi_sel = {true,true},
    .spm_timer_opt= SPM_USE_PCM_TIMER,
    .timer_val_ms = SUSPEND_PCM_TIMER_VAL,
    .wdt_val_ms = SUSPEND_PCM_WDT_VAL,
    .wake_src = WAKE_SRC_FOR_SUSPEND,
    .reserved = SPM_SUSPEND_GET_FGUAGE,
    .cpu_pdn = true,
    .infra_pdn = true,
    .coclock_en = false,
    .lock_infra_dcm=true,
    
    .twam_log_en = true,
    .monitor_signal={twam_md1_apsrc_req,twam_conn_apsrc_req},
    
    /* debug related*/
    .dbg_wfi_cnt=0,
    .wakesta_idx=0

     };

extern kal_int32 get_dynamic_period(int first_use, int first_wakeup_time, int battery_capacity_level);

static u32 spm_get_wake_period(wake_reason_t last_wr)
{
    int period;
 
    /* battery decreases 1% */
    period = get_dynamic_period(!!(last_wr != WR_PCM_TIMER), 600, 1);
    if (period <= 1) 
    {
        spm_error("!!! CANNOT GET PERIOD FROM FUEL GAUGE !!!\n");
        period = 600;
    }
    else if (period > 36 * 3600)
    {    /* max period is 36.4 hours */
        period = 36 * 3600;
    }

    return period;
}
wake_reason_t spm_go_to_sleep(void)
{
    wake_status_t *wakesta;
    unsigned long flags,i;
    struct mtk_irq_mask mask;
    int wd_ret;
    struct wd_api *wd_api;
    static wake_reason_t last_wr = WR_NONE;

    spm_stop_normal();
    
    //mtk_wdt_suspend();
    wd_ret = get_wd_api(&wd_api);
    if (!wd_ret)
        wd_api->wd_suspend_notify();

    spin_lock_irqsave(&spm_lock, flags);
    mt_irq_mask_all(&mask);
    mt_irq_unmask_for_sleep(MT_SPM0_IRQ_ID);

    if(pcm_config_suspend.reserved & SPM_SUSPEND_GET_FGUAGE)
      pcm_config_suspend.timer_val_ms = spm_get_wake_period(WR_NONE)*1000;

    if (spm_init_pcm(&pcm_config_suspend)==false)
        goto RESTORE_IRQ;

    spm_kick_pcm(&pcm_config_suspend);

    snapshot_golden_setting(__FUNCTION__, __LINE__);
    spm_trigger_wfi(&pcm_config_suspend);
    
    wakesta = spm_get_wakeup_status(&pcm_config_suspend);
    
    if(wakesta->wakeup_event & WAKE_SRC_CCIF)
        exec_ccci_kern_func_by_md_id(0, ID_GET_MD_WAKEUP_SRC, NULL, 0);
    else if(wakesta->wakeup_event & WAKE_SRC_GPT)
    {
      //if(pcm_config_suspend.reserved & SPM_SUSPEND_GET_FGUAGE)
        wakesta->wake_reason = WR_PCM_TIMER;
    }
    
    last_wr = wakesta->wake_reason;

    spm_clean_after_wakeup();
    
 RESTORE_IRQ:
    mt_irq_mask_restore(&mask);
    spin_unlock_irqrestore(&spm_lock, flags);
   
   //mtk_wdt_resume();
   if (!wd_ret)
     wd_api->wd_resume_notify();
    
    spm_go_to_normal();
    return last_wr;
}

void spm_output_sleep_option(void)
{
    spm_notice("AP_ONLY_SLEEP:%d, PWAKE_EN:%d, BYPASS_SYSPWREQ:%d\n",
               SPM_AP_ONLY_SLEEP, SPM_PWAKE_EN, SPM_BYPASS_SYSPWREQ);
}

MODULE_AUTHOR("Scott Hung <scott.hung@mediatek.com>");
MODULE_DESCRIPTION("MT65xx SPM-Sleep Driver v0.1");
