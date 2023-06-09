#ifndef _MT_BOOT_MODE_H_
#define _MT_BOOT_MODE_H_

#include <platform/mt_typedefs.h>
#include <mt_partition.h>    //for part_hdr_t
#include <mt_rtc.h>

/******************************************************************************
 * FORBIDEN MODE
 ******************************************************************************/
typedef enum 
{
  F_FACTORY_MODE = 0x0001,
} FORBIDDEN_MODE;

/******************************************************************************
 * LIMITATION
 ******************************************************************************/
#define SEC_LIMIT_MAGIC  0x4C4C4C4C // LLLL

typedef struct 
{
  unsigned int magic_num;
  FORBIDDEN_MODE forbid_mode;
} SEC_LIMIT;


/* boot type definitions */
typedef enum 
{
    NORMAL_BOOT = 0,
    META_BOOT = 1,
    RECOVERY_BOOT = 2,    
    SW_REBOOT = 3,
    FACTORY_BOOT = 4,
    ADVMETA_BOOT = 5,
    ATE_FACTORY_BOOT = 6,
    ALARM_BOOT = 7,
#if defined (MTK_KERNEL_POWER_OFF_CHARGING)
    KERNEL_POWER_OFF_CHARGING_BOOT = 8,
    LOW_POWER_OFF_CHARGING_BOOT = 9,
#endif    
    FASTBOOT = 99,
    DOWNLOAD_BOOT = 100,
    UNKNOWN_BOOT
} BOOTMODE;

typedef enum {
    BR_POWER_KEY = 0,
    BR_USB,
    BR_RTC,
    BR_WDT,
    BR_WDT_BY_PASS_PWK,
    BR_TOOL_BY_PASS_PWK,
#ifdef RTC_2SEC_REBOOT_ENABLE
    BR_2SEC_REBOOT,
#endif
    BR_UNKNOWN
} boot_reason_t;

/*META COM port type*/
 typedef enum
{
    META_UNKNOWN_COM = 0,
    META_UART_COM,
    META_USB_COM
} META_COM_TYPE;

typedef struct{
  u32 addr;    /* Download Agent address */
  u32 arg1;    /* Download Agent arg 1*/
  u32 arg2;    /* Download Agent arg 2*/
  u32 len;     /* length of da */
  u32 sig_len; /* signature length of da */
} da_info_t;

typedef struct {
  u32      maggic_number;
  BOOTMODE boot_mode;
  u32      e_flag;
  u32      log_port;
  u32      log_baudrate;
  u8       log_enable;
  u8       part_num;
  u8       reserved[2];
  u32      dram_rank_num;
  u32      dram_rank_size[4];
  u32      boot_reason;
  META_COM_TYPE meta_com_type;
  u32      meta_com_id;
  u32      boot_time;
  da_info_t da_info;
  SEC_LIMIT sec_limit;
  part_hdr_t *part_info;
  u8 md_type[4];
  u32  ddr_reserve_enable;    
  u32  ddr_reserve_success;
} BOOT_ARGUMENT;

typedef enum /* TO BE REMOVED */
{
    CHIP_E1 = 0xca00,
    CHIP_E2 = 0xcb00,
    //CHIP_E3 = 0x8a02, //TBD   
} CHIP_VER;

typedef enum {
    CHIP_SW_VER_01 = 0x0000,
    CHIP_SW_VER_02 = 0x0001
} CHIP_SW_VER;

typedef struct        /* RAM configuration */
{
  unsigned long start;
  unsigned long size;
} BI_DRAM;


#define BOOT_ARGUMENT_MAGIC 0x504c504c

extern unsigned int BOOT_ARGUMENT_LOCATION;
extern void boot_mode_select(void);
extern BOOTMODE g_boot_mode;
extern CHIP_VER get_chip_eco_ver(void);         /*TO BE REMOVED*/
extern CHIP_SW_VER  mt_get_chip_sw_ver(void);

/*FIXME: NAND/eMMC boot. Needo to specifiy by MTK build system. TEMP define here*/
//#define CFG_NAND_BOOT
//#define CFG_MMC_BOOT

#endif
