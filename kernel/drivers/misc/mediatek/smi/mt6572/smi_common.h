#ifndef __SMI_COMMON_H__
#define __SMI_COMMON_H__

#include <linux/aee.h>
#include <mach/mt_smi.h>


#define SMIMSG(string, args...)	pr_warn("[pid=%d]"string, current->tgid, ##args)
#define SMIDBG(string, args...)  pr_warn("[pid=%d]"string, current->tgid, ##args)
#define SMIERR(string, args...) do{\
	pr_err("error: "string, ##args); \
	aee_kernel_warning(SMI_LOG_TAG, "error: "string, ##args);  \
}while(0)

#define smi_aee_print(string, args...) do{\
    char smi_name[100];\
    snprintf(smi_name,100, "["SMI_LOG_TAG"]"string, ##args); \
  aee_kernel_warning(smi_name, "["SMI_LOG_TAG"]error:"string,##args);  \
}while(0)


#define MAU_ENTRY_NR    3
#define SMI_LARB_NR     1

    
extern unsigned int gLarbBaseAddr[SMI_LARB_NR];
extern char *smi_port_name[16];

//output_gce_buffer = 1, pass log to CMDQ error dumping messages
int smi_debug_bus_hanging_detect_ext( unsigned int larbs, int show_dump, int output_gce_buffer);
int larb_clock_on(int larb_id, const char *mod_name);
int larb_clock_off(int larb_id, const char *mod_name);

int mau_init(void);

int smi_bwc_config( MTK_SMI_BWC_CONFIG* p_conf );

#endif

