#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/list.h>
#ifdef CONFIG_MTK_AEE_FEATURE
#include <linux/aee.h>
#endif  

#include <mach/mt_reg_base.h>
#include <mach/emi_mpu.h>
#include <mach/mt_device_apc.h>
#include <mach/sync_write.h>
#include <mach/irqs.h>
#include <mach/dma.h>

#define NR_REGION_ABORT 8
#define MAX_EMI_MPU_STORE_CMD_LEN 128
#define ABORT_EMI 0x1 //Hong-Rong: mark for porting
#define TIMEOUT 100

struct mst_tbl_entry
{
    u32 master;
    u32 domain;
    u32 id_mask;
    u32 id_val;
    char *name;
};

struct emi_mpu_notifier_block
{
    struct list_head list;
    emi_mpu_notifier notifier; 
};

static const struct mst_tbl_entry mst_tbl[] =
{
   /* PeriSys */	 
   { .master = MST_ID_PERI_0, .domain = 0x0, .id_mask = 0b11111111, .id_val = 0b01100000, .name = "PERISYS:APDMA" },
   { .master = MST_ID_PERI_1, .domain = 0x0, .id_mask = 0b11111111, .id_val = 0b01000101, .name = "PERISYS:NFI/USB/MSDC0" },
   { .master = MST_ID_PERI_2, .domain = 0x0, .id_mask = 0b11111111, .id_val = 0b01000010, .name = "PERISYS:SPI/MSDC1/Audio/SPM" },

    /* apmcu */
    { .master = MST_ID_APMCU_0, .domain = 0x0, .id_mask = 0b11000000, .id_val = 0b00000000, .name = "APMCU: " },
    
   /* Modem HW (2G/3G) */
   { .master = MST_ID_MDHW_0, .domain = 0x1, .id_mask = 0b11000111, .id_val = 0b01000000, .name = "MDHW:MD DMA" },
   { .master = MST_ID_MDHW_1, .domain = 0x1, .id_mask = 0b11000111, .id_val = 0b01000100, .name = "MDHW:ABM" },
   { .master = MST_ID_MDHW_2, .domain = 0x1, .id_mask = 0b11111111, .id_val = 0b01000001, .name = "MDHW:2G" },
   { .master = MST_ID_MDHW_3, .domain = 0x1, .id_mask = 0b11111111, .id_val = 0b01001001, .name = "MDHW:BYC_ACC" },
   { .master = MST_ID_MDHW_4, .domain = 0x1, .id_mask = 0b11111111, .id_val = 0b01001101, .name = "MDHW:3G" },
   { .master = MST_ID_MDHW_5, .domain = 0x1, .id_mask = 0b11111111, .id_val = 0b01011101, .name = "MDHW:3G" },
   { .master = MST_ID_MDHW_6, .domain = 0x1, .id_mask = 0b11111111, .id_val = 0b01011110, .name = "MDHW:TDSYS" }, 

    /* Modem MCU*/    
    { .master = MST_ID_MDMCU_0, .domain = 0x1, .id_mask = 0b11000011, .id_val = 0b00000000, .name = "MDMCU:CR4" },
    { .master = MST_ID_MDMCU_1, .domain = 0x1, .id_mask = 0b11000011, .id_val = 0b00000001, .name = "MDMCU:PFB" },
    { .master = MST_ID_MDMCU_2, .domain = 0x1, .id_mask = 0b11000011, .id_val = 0b00000010, .name = "MDMCU:ALC" },

    /* ConnSys */
    { .master = MST_ID_CONNSYS_0, .domain = 0x2, .id_mask = 0b11111111, .id_val = 0b00000000, .name = "CONNSYS: " }, 
       
    /* MM */
    { .master = MST_ID_MM_0, .domain = 0x3, .id_mask = 0b11100000, .id_val = 0b00100000, .name = "MM:SMI" }, 
    { .master = MST_ID_MM_1, .domain = 0x3, .id_mask = 0b11100000, .id_val = 0b00000000, .name = "MM:G3D" }, 
    { .master = MST_ID_MM_2, .domain = 0x3, .id_mask = 0b11111111, .id_val = 0b01111100, .name = "MM:M4U" },
    { .master = MST_ID_MM_3, .domain = 0x3, .id_mask = 0b11111111, .id_val = 0b01111101, .name = "MM:M4U" },
};

struct list_head emi_mpu_notifier_list[NR_MST];
static const char *UNKNOWN_MASTER = "unknown";
static spinlock_t emi_mpu_lock;

char *smi_larb0_port[10] = {"venc_rcpu", "venc_ref_luma", "venc_ref_chroma", "venc_db_read", "venc_db_write", "venc_cur_luma", "venc_cur_chroma", "venc_rd_comv", "venc_sv_comv", "venc_bsdma"};
char *smi_larb1_port[7] =  {"hw_vdec_mc_ext", "hw_vdec_pp_ext", "hw_vdec_avc_mv_ext", "hw_vdec_pred_rd_ext", "hw_vdec_pred_wr_ext", "hw_vdec_vld_ext", "hw_vdec_vld2_ext" };
char *smi_larb2_port[12] = {"rot_ext", "ovl_ch0", "ovl_ch1", "ovl_ch2", "ovl_ch3", "wdma0", "wdma1", "rdma0", "rdma1", "cmdq", "dbi", "g2d"};
char *smi_larb3_port[15] = {"jpgdec_wdma", "jpgenc_rdma", "vipi", "vip2i", "dispo", "dispco", "dispvo", "vido", "vidco", "vidvo", "gdma_smi_rd", "gdma_smi_wr", "jpgdec_bsdma", "jpgenc_bsdma", "vido_rot0"};
char *smi_larb4_port[12] = {"imgi", "imgci", "imgo", "img2o", "lsci", "flki", "lcei", "lcso", "esfko", "aao", "vidco_rot0", "fpc"};

static void __show_peri_master(u32 id)
{
  u32 	reg_peri;
 
  if(id == 0x45){
		
	reg_peri = readl(PERI_AHB_LYR0_MASTER);
	switch (reg_peri & 0xF){
	case (0x1):
		printk(KERN_CRIT "NFI.\n");
		break;

	case (0x2):
		printk(KERN_CRIT "MSDC0.\n");
		break;

	case (0x4):
		printk(KERN_CRIT "USB.\n");
		break;

	case (0x8):
		printk(KERN_CRIT "I2C.\n");
		break;	

	default:
		break;
	}	
  }

  if(id == 0x42){
		
	reg_peri = readl(PERI_AHB_LYR1_MASTER);
	if ((reg_peri & 0x80000000) == 0x80000000){
		switch (reg_peri & 0xF){
		case (0x1):
			printk(KERN_CRIT "MSDC1.\n");
			break;

		case (0x2):
			printk(KERN_CRIT "SPI.\n");
			break;

		case (0x4):
			printk(KERN_CRIT "AUDIO.\n");
			break;

		case (0x8):
			printk(KERN_CRIT "PWM.\n");
			break;	

		default:
			break;
		}
	} 
	else if ((reg_peri & 0x80000000) == 0x0){

		reg_peri = readl(PERI_AHB_LYR3_MASTER);
		switch (reg_peri & 0xF){
		case (0x1):
			printk(KERN_CRIT "FHCTL.\n");
			break;

		case (0x2):
			printk(KERN_CRIT "SPM.\n");
			break;

		default:
			break;
		}
	}	
  }	
}


static int __match_id(u32 domain, int tbl_idx, u32 id)
{
  if (((id & mst_tbl[tbl_idx].id_mask) == mst_tbl[tbl_idx].id_val) && (domain == mst_tbl[tbl_idx].domain))
        {
          printk(KERN_CRIT "Violation master name is %s.\n", mst_tbl[tbl_idx].name);

  	  if ((id == 0x45) || (id == 0x42)){
	     printk(KERN_CRIT "PERI Master is ");
	     __show_peri_master(id);
  	  } 
	   		
    return 1;
  } 
  else 
  {
    return 0;
  }
}

static u32 __id2mst(u32 domain, u32 id)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(mst_tbl); i++) {
        if (__match_id(domain, i, id)) {
            return mst_tbl[i].master;
        }
    }
    return MST_INVALID;
}

static char *__id2name(u32 domain, u32 id)
{
    int i;
        
    for (i = 0; i < ARRAY_SIZE(mst_tbl); i++) {
        if (__match_id(domain, i, id))
        {
          return mst_tbl[i].name;
        }
    }
    
    return (char *)UNKNOWN_MASTER;
}

static void __clear_emi_mpu_vio(void)
{
    u32 dbg_s, dbg_t;

    /* clear violation status */
    mt65xx_reg_sync_writel(0x00FF01FF, EMI_MPUP);
    mt65xx_reg_sync_writel(0x00FF00FF, EMI_MPUQ);

    /* clear debug info */
    mt65xx_reg_sync_writel(0x80000000 , EMI_MPUS);
    dbg_s = readl(EMI_MPUS);
    dbg_t = readl(EMI_MPUT);
    if (dbg_s || dbg_t) {
        printk(KERN_CRIT "Fail to clear EMI MPU violation\n");
        printk(KERN_CRIT "EMI_MPUS = %x, EMI_MPUT = %x", dbg_s, dbg_t);
    }
}

#if 0
static void mtk_dump_full_pgtab(void *param, void *param1)
{
    //void *va = 0, *pa;
    pgd_t *pgd;
    pmd_t *pmd;
    pte_t *pte;
    int i, j;
    struct task_struct *p = 0;
    //va = vmalloc(PAGE_SIZE);
    //
    *(volatile unsigned long *)0xf0000000 = 0x00002224; // disable WDT
    __asm__ __volatile__ ("dsb");
#if 0
    if (va) {
        *(char *)va = 3;    // touch the va to allocate the pte
        printk(KERN_ALERT"vmalloc @ %p\n", va);
        pa = mtk_virt_to_phys(&va);
        printk(KERN_ALERT"vmalloc (pa) @ %p\n", pa);
        //vfree(va);
    }
#endif 
    printk(KERN_ALERT"%s\n", __FUNCTION__);
    printk(KERN_ALERT"=====================================\n");

    p = current;
    if (!(p->mm)) {
        printk(KERN_ALERT"NULL current->mm\n");
        pgd = init_mm.pgd;
    } else {
        // we use 2-level paging here (it means pgd == pud == pmd)
        pgd = p->mm->pgd;
    }
    // show a complete pgd table
    printk(KERN_ALERT"pgd: 0x%08lx\n", (unsigned long)pgd);
    for (i = 0; i < PTRS_PER_PGD; i += 8) {
        printk(KERN_ALERT"[0x%08lx] %08lx %08lx %08lx %08lx "
                "%08lx %08lx %08lx %08lx\n", 
                (unsigned long)(pgd + i),
                (unsigned long)pgd_val(*(pgd + i + 0)),
                (unsigned long)pgd_val(*(pgd + i + 1)),
                (unsigned long)pgd_val(*(pgd + i + 2)),
                (unsigned long)pgd_val(*(pgd + i + 3)),
                (unsigned long)pgd_val(*(pgd + i + 4)),
                (unsigned long)pgd_val(*(pgd + i + 5)),
                (unsigned long)pgd_val(*(pgd + i + 6)),
                (unsigned long)pgd_val(*(pgd + i + 7)));
    }

    // dump pmd if possible
    for (i = 0; i < PTRS_PER_PGD; i++, pgd++) {
        //msleep(100);    // print the log slowly to avoid form losing log
        printk(KERN_ALERT"i: %d, pgd: 0x%08lx, val: 0x%08lx\n", 
                i,
                (unsigned long)pgd, (unsigned long)pgd_val(*pgd));
        if ((pgd_none(*pgd)) || (pgd_bad(*pgd))) {
            continue;
        }

        // we use 2-level paging here
        printk(KERN_ALERT"pgd: 0x%08lx\n", (unsigned long)pgd);
        pmd = (pmd_t *)(pgd);
        printk(KERN_ALERT"pmd: 0x%08lx, pmd_val(pmd): 0x%08lx\n", 
                (unsigned long)pmd,
                (unsigned long)pmd_val(*pmd));
        if (!pmd  || !pmd_val(*pmd)) {
            continue;
        }
        pte = __pte_map(pmd);
        if ((unsigned long)pmd_val(*pmd) >= 0x80000000 && 
                (unsigned long)pmd_val(*pmd) <= 0xC0000000) {
            printk(KERN_ALERT"pmd: 0x%08lx, mapped to 0x%08lx\n", 
                    (unsigned long)pmd_val(*pmd), (unsigned long)pte);
        } else {
            printk(KERN_ALERT"skip non-DRAM pmd: 0x%08lx\n", 
                    (unsigned long)pmd_val(*pmd));
            continue;
        }
        for (j = 0; j < PTRS_PER_PTE; j += 8) {
            printk(KERN_ALERT"[0x%08lx] %08lx %08lx %08lx %08lx "
                    "%08lx %08lx %08lx %08lx\n", 
                    (unsigned long)(pte + j),
                    (unsigned long)pte_val(*(pte + j + 0)),
                    (unsigned long)pte_val(*(pte + j + 1)),
                    (unsigned long)pte_val(*(pte + j + 2)),
                    (unsigned long)pte_val(*(pte + j + 3)),
                    (unsigned long)pte_val(*(pte + j + 4)),
                    (unsigned long)pte_val(*(pte + j + 5)),
                    (unsigned long)pte_val(*(pte + j + 6)),
                    (unsigned long)pte_val(*(pte + j + 7)));
        }
        __pte_unmap(pte);
    }
    //pa = mtk_virt_to_phys(&va);
    //vfree(va);
    //BUG();
}
#endif

void emi_mem_psrv_diable(void)
{
	u32 reg_s;
    reg_s = readl(EMI_GENA);
    reg_s &= 0xFFFFFFFB; // disable mem_psrv_req_en bit
	mt65xx_reg_sync_writel(reg_s, EMI_GENA); 
}


static void __set_emi_mpu_key(void)
{
    mt65xx_reg_sync_writel(0x5A5AA5A5, EMI_MPSW);            // password setting 
    mt65xx_reg_sync_writel(REGION_0_KEY + 0, EMI_MPSW);  // region 0 password
    
    mt65xx_reg_sync_writel(0x5A5AA5A5, EMI_MPSW);            // password setting 
    mt65xx_reg_sync_writel(REGION_1_KEY + 1, EMI_MPSW);  // region 1 password

    mt65xx_reg_sync_writel(0x5A5AA5A5, EMI_MPSW);            // password setting 
    mt65xx_reg_sync_writel(REGION_2_KEY + 2, EMI_MPSW);  // region 2 password
    
    mt65xx_reg_sync_writel(0x5A5AA5A5, EMI_MPSW);            // password setting 
    mt65xx_reg_sync_writel(REGION_3_KEY + 3, EMI_MPSW);  // region 3 password

    mt65xx_reg_sync_writel(0x5A5AA5A5, EMI_MPSW);            // password setting 
    mt65xx_reg_sync_writel(REGION_4_KEY + 4, EMI_MPSW);  // region 4 password
    
    mt65xx_reg_sync_writel(0x5A5AA5A5, EMI_MPSW);            // password setting 
    mt65xx_reg_sync_writel(REGION_5_KEY + 5, EMI_MPSW);  // region 5 password

    mt65xx_reg_sync_writel(0x5A5AA5A5, EMI_MPSW);            // password setting 
    mt65xx_reg_sync_writel(REGION_6_KEY + 6, EMI_MPSW);  // region 6 password
    
    mt65xx_reg_sync_writel(0x5A5AA5A5, EMI_MPSW);            // password setting 
    mt65xx_reg_sync_writel(REGION_7_KEY + 7, EMI_MPSW);  // region 7 password
    
    return;
}

static void __enter_emi_mpu_key(int region)
{
    switch (region) {
    case 0:
		mt65xx_reg_sync_writel(REGION_0_KEY + 0, EMI_MKEY);			
        break; 

    case 1:		
		mt65xx_reg_sync_writel(REGION_1_KEY + 1, EMI_MKEY);			
        break;

    case 2:		
		mt65xx_reg_sync_writel(REGION_2_KEY + 2, EMI_MKEY);			
        break;

    case 3:		
		mt65xx_reg_sync_writel(REGION_3_KEY + 3, EMI_MKEY);			
        break;        

    case 4:		
		mt65xx_reg_sync_writel(REGION_4_KEY + 4, EMI_MKEY);			
        break;  

    case 5:		
		mt65xx_reg_sync_writel(REGION_5_KEY + 5, EMI_MKEY);			
        break;

    case 6:		
		mt65xx_reg_sync_writel(REGION_6_KEY + 6, EMI_MKEY);			
        break;

    case 7:		
		mt65xx_reg_sync_writel(REGION_7_KEY + 7, EMI_MKEY);			
        break;

    default:
        break;
    }

    return;
}


static void __exit_emi_mpu_key(int region)
{
    switch (region) {
    case 0:
	mt65xx_reg_sync_writel(0, EMI_MKEY); 		
        break; 

    case 1:		
		mt65xx_reg_sync_writel( 1, EMI_MKEY);			
        break;

    case 2:		
		mt65xx_reg_sync_writel(2, EMI_MKEY);			
        break;

    case 3:		
		mt65xx_reg_sync_writel(3, EMI_MKEY);			
        break;        

    case 4:		
		mt65xx_reg_sync_writel(4, EMI_MKEY);			
        break;  

    case 5:		
		mt65xx_reg_sync_writel(5, EMI_MKEY);			
        break;

    case 6:		
		mt65xx_reg_sync_writel(6, EMI_MKEY);			
        break;

    case 7:		
		mt65xx_reg_sync_writel(7, EMI_MKEY);			
        break;

    default:
        break;
    }
	
	 return;
}

/*EMI MPU violation handler*/
static irqreturn_t mpu_violation_irq(int irq, void *dev_id)
{
    u32 dbg_s, dbg_t, dbg_pqry;
    u32 master_ID, domain_ID, wr_vio;
    s32 region;
    int i;
    unsigned int reg_value = 0;
    int counter = 0;
    char *master_name;
      
    if ((readl(DEVAPC_D0_VIO_STA_3) & ABORT_EMI) == 0) {
        printk(KERN_INFO "Not EMI MPU violation.\n");
        return IRQ_NONE;
    }
    
    
    dbg_s = readl(EMI_MPUS);
    dbg_t = readl(EMI_MPUT);
    
    master_ID = dbg_s & 0x000000FF;
    domain_ID = (dbg_s >> 12) & 0x00000003;
    wr_vio = (dbg_s >> 28) & 0x00000003;
    region = (dbg_s >> 16) & 0xFF;    
    
    for (i = 0 ; i < NR_REGION_ABORT; i++) {
        if ((region >> i) & 1) {
            break;
        }
    }
    region = (i >= NR_REGION_ABORT)? -1: i;
    
    switch (domain_ID) {
    case 0:
        dbg_pqry = readl(EMI_MPUP) & 0x000001FF;
        break;
    case 1:
        dbg_pqry = readl(EMI_MPUP) & 0x00FF0000;
        break;
    case 2:
        dbg_pqry = readl(EMI_MPUQ) & 0x000000FF;
        break;
    case 3:
        dbg_pqry = readl(EMI_MPUQ) & 0x00FF0000;
        break;
    default:
        dbg_pqry = 0;
        break;
    }  
    
    /*print the abort region*/        
    printk(KERN_CRIT "EMI MPU violation.\n");
    printk(KERN_CRIT "[EMI MPU] Debug info start ----------------------------------------\n");

    printk(KERN_CRIT "EMI_MPUS = %x, EMI_MPUT = %x.\n", dbg_s, dbg_t);
    printk(KERN_CRIT "Current process is \"%s \" (pid: %i).\n", current->comm, current->pid);
    printk(KERN_CRIT "Violation address is 0x%x.\n", (dbg_t)+ EMI_PHY_OFFSET);
    printk(KERN_CRIT "Violation master ID is 0x%x.\n", master_ID);
    /*print out the murderer name*/
    master_name = __id2name(domain_ID,master_ID);
    printk(KERN_CRIT "Violation domain ID is 0x%x.\n", domain_ID);
    printk(KERN_CRIT "%s violation.\n", (wr_vio == 1)? "Write": "Read");
    printk(KERN_CRIT "Corrupted region is %d\n\r", region);
    if (dbg_pqry & 0x0100) {
        printk(KERN_CRIT "APB abort.\n");
    }      
    
#ifdef CONFIG_MTK_AEE_FEATURE
    //aee_kernel_exception("EMI MPU", "EMI MPU violation.\nEMP_MPUS = 0x%x, EMI_MPUT = 0x%x, EMI_MPU(PQR).\n", dbg_s, dbg_t+EMI_PHY_OFFSET, dbg_pqry);
    aee_kernel_exception("EMI MPU", "EMI MPU violation.\nEMP_MPUS = 0x%x, EMI_MPUT = 0x%x, module is %s.\n", dbg_s, dbg_t, master_name);    
#endif

    __clear_emi_mpu_vio();
        
    // Device APC 
    mt65xx_reg_sync_writel(readl(DEVAPC_D0_VIO_STA_3) | ABORT_EMI, DEVAPC_D0_VIO_STA_3);
    do{
      reg_value = readl(DEVAPC_D0_VIO_STA_3) & ABORT_EMI;
      counter++;
      if(counter >= TIMEOUT)
      {
        counter = 0;  
        break;
      }
    }while (reg_value != 0);
    
    printk("[EMI MPU] _id2mst = %d\n", __id2mst(domain_ID,master_ID)); 
    
    //mtk_dump_full_pgtab(NULL, NULL);
    
    printk(KERN_CRIT "[EMI MPU] Debug info end------------------------------------------\n");
    return IRQ_HANDLED;
}

/*
 * emi_mpu_set_region_protection: protect a region.
 * @start: start address of the region
 * @end: end address of the region
 * @region: EMI MPU region id
 * @access_permission: EMI MPU access permission
 * Return 0 for success, otherwise negative status code.
 */
int emi_mpu_set_region_protection(unsigned int start, unsigned int end, int region, unsigned int access_permission)
{
    int ret = 0;
    unsigned int tmp;
        
    if((end != 0) || (start !=0)) {
      /*Address 32KB alignment*/
      start -= EMI_PHY_OFFSET;
      end -= EMI_PHY_OFFSET;
      start = start >> 15;
      end = end >> 15;

      if (end <= start) {
        return -EINVAL;
      }
    }
	
    spin_lock(&emi_mpu_lock);

     __enter_emi_mpu_key(region); 

    switch (region) {
    case 0:
        mt65xx_reg_sync_writel((start << 16) | end, EMI_MPUA); 
        tmp = readl(EMI_MPUI) & 0xFFFF0000;
        mt65xx_reg_sync_writel(tmp | access_permission, EMI_MPUI);
        break; 

    case 1:
        mt65xx_reg_sync_writel((start << 16) | end, EMI_MPUB);
        tmp = readl(EMI_MPUI) & 0x0000FFFF;
        mt65xx_reg_sync_writel(tmp | (access_permission << 16), EMI_MPUI);
        break;

    case 2:
        mt65xx_reg_sync_writel((start << 16) | end, EMI_MPUC);
        tmp = readl(EMI_MPUJ) & 0xFFFF0000;
        mt65xx_reg_sync_writel(tmp | access_permission, EMI_MPUJ);
        break;

    case 3:
        mt65xx_reg_sync_writel((start << 16) | end, EMI_MPUD);
        tmp = readl(EMI_MPUJ) & 0x0000FFFF;
        mt65xx_reg_sync_writel(tmp | (access_permission << 16), EMI_MPUJ);
        break;        

    case 4:
        mt65xx_reg_sync_writel((start << 16) | end, EMI_MPUE);
        tmp = readl(EMI_MPUK) & 0xFFFF0000;
        mt65xx_reg_sync_writel(tmp | access_permission, EMI_MPUK);
        break;  

    case 5:
        mt65xx_reg_sync_writel((start << 16) | end, EMI_MPUF);
        tmp = readl(EMI_MPUK) & 0x0000FFFF;
        mt65xx_reg_sync_writel(tmp | (access_permission << 16), EMI_MPUK);
        break;

    case 6:
        mt65xx_reg_sync_writel((start << 16) | end, EMI_MPUG);
        tmp = readl(EMI_MPUL) & 0xFFFF0000;
        mt65xx_reg_sync_writel(tmp | access_permission, EMI_MPUL);
        break;

    case 7:
        mt65xx_reg_sync_writel((start << 16) | end, EMI_MPUH);
        tmp = readl(EMI_MPUL) & 0x0000FFFF;
        mt65xx_reg_sync_writel(tmp | (access_permission << 16), EMI_MPUL);
        break;

    default:
        ret = -EINVAL;
        break;
    }

    __exit_emi_mpu_key(region); 

    spin_unlock(&emi_mpu_lock);

    return ret;
}

/*
 * emi_mpu_notifier_register: register a notifier.
 * master: MST_ID_xxx
 * notifier: the callback function
 * Return 0 for success, otherwise negative error code.
 */
int emi_mpu_notifier_register(int master, emi_mpu_notifier notifier)
{
    struct emi_mpu_notifier_block *block;
    static int emi_mpu_notifier_init = 0;
    int i;

    if (master >= MST_INVALID) {
        return -EINVAL;
    }

    block = kmalloc(sizeof(struct emi_mpu_notifier_block), GFP_KERNEL);
    if (!block) {
        return -ENOMEM;
    }

    if (!emi_mpu_notifier_init) {
        for (i = 0; i < NR_MST; i++) {
            INIT_LIST_HEAD(&(emi_mpu_notifier_list[i]));
        }
        emi_mpu_notifier_init = 1;
    }

    block->notifier = notifier;
    list_add(&(block->list), &(emi_mpu_notifier_list[master]));

    return 0;
}


static ssize_t emi_mpu_show(struct device_driver *driver, char *buf)
{
    char *ptr = buf;
    unsigned int start, end;
    unsigned int reg_value;
    unsigned int d0, d1, d2, d3;
    static const char *permission[6] = 
    { 
        "No protect",
        "Only R/W for secure access",
        "Only R/W for secure access, and non-secure read access",
        "Only R/W for secure access, and non-secure write access",
        "Only R for secure/non-secure",
        "Both R/W are forbidden" 
    };

    reg_value = readl(EMI_MPUA);
    start = ((reg_value >> 16) << 15) + EMI_PHY_OFFSET;
    end = ((reg_value & 0xFFFF) << 15) + EMI_PHY_OFFSET;
    ptr += sprintf(ptr, "Region 0 --> 0x%x to 0x%x\n", start, end);

    reg_value = readl(EMI_MPUB);
    start = ((reg_value >> 16) << 15) + EMI_PHY_OFFSET;
    end = ((reg_value & 0xFFFF) << 15) + EMI_PHY_OFFSET;
    ptr += sprintf(ptr, "Region 1 --> 0x%x to 0x%x\n", start, end);

    reg_value = readl(EMI_MPUC);
    start = ((reg_value >> 16) << 15) + EMI_PHY_OFFSET;
    end = ((reg_value & 0xFFFF) << 15) + EMI_PHY_OFFSET;
    ptr += sprintf(ptr, "Region 2 --> 0x%x to 0x%x\n", start, end);

    reg_value = readl(EMI_MPUD);
    start = ((reg_value >> 16) << 15) + EMI_PHY_OFFSET;
    end = ((reg_value & 0xFFFF) << 15) + EMI_PHY_OFFSET;
    ptr += sprintf(ptr, "Region 3 --> 0x%x to 0x%x\n", start, end);

    reg_value = readl(EMI_MPUE);
    start = ((reg_value >> 16) << 15) + EMI_PHY_OFFSET;
    end = ((reg_value & 0xFFFF) << 15) + EMI_PHY_OFFSET;
    ptr += sprintf(ptr, "Region 4 --> 0x%x to 0x%x\n", start, end);

    reg_value = readl(EMI_MPUF);
    start = ((reg_value >> 16) << 15) + EMI_PHY_OFFSET;
    end = ((reg_value & 0xFFFF) << 15) + EMI_PHY_OFFSET;
    ptr += sprintf(ptr, "Region 5 --> 0x%x to 0x%x\n", start, end);

    reg_value = readl(EMI_MPUG);
    start = ((reg_value >> 16) << 15) + EMI_PHY_OFFSET;
    end = ((reg_value & 0xFFFF) << 15) + EMI_PHY_OFFSET;
    ptr += sprintf(ptr, "Region 6 --> 0x%x to 0x%x\n", start, end);

    reg_value = readl(EMI_MPUH);
    start = ((reg_value >> 16) << 15) + EMI_PHY_OFFSET;
    end = ((reg_value & 0xFFFF) << 15) + EMI_PHY_OFFSET;
    ptr += sprintf(ptr, "Region 7 --> 0x%x to 0x%x\n", start, end);
    
    ptr += sprintf (ptr, "\n");

    reg_value = readl(EMI_MPUI);
    d0 = (reg_value & 0x7);
    d1 = (reg_value >> 3) & 0x7;
    d2 = (reg_value >> 6) & 0x7;
    d3 = (reg_value >> 9) & 0x7;
    ptr += sprintf(ptr, "Region 0 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);

    d0 = ((reg_value>>16) & 0x7);
    d1 = ((reg_value>>16) >> 3) & 0x7;
    d2 = ((reg_value>>16) >> 6) & 0x7;
    d3 = ((reg_value>>16) >> 9) & 0x7;
    ptr += sprintf(ptr, "Region 1 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);

    reg_value = readl(EMI_MPUJ);
    d0 = (reg_value & 0x7);
    d1 = (reg_value >> 3) & 0x7;
    d2 = (reg_value >> 6) & 0x7;
    d3 = (reg_value >> 9) & 0x7;
    ptr += sprintf(ptr, "Region 2 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);

    d0 = ((reg_value>>16) & 0x7);
    d1 = ((reg_value>>16) >> 3) & 0x7;
    d2 = ((reg_value>>16) >> 6) & 0x7;
    d3 = ((reg_value>>16) >> 9) & 0x7;
    ptr += sprintf(ptr, "Region 3 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);

    reg_value = readl(EMI_MPUK);
    d0 = (reg_value & 0x7);
    d1 = (reg_value >> 3) & 0x7;
    d2 = (reg_value >> 6) & 0x7;
    d3 = (reg_value >> 9) & 0x7;
    ptr += sprintf(ptr, "Region 4 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);

    d0 = ((reg_value>>16) & 0x7);
    d1 = ((reg_value>>16) >> 3) & 0x7;
    d2 = ((reg_value>>16) >> 6) & 0x7;
    d3 = ((reg_value>>16) >> 9) & 0x7;
    ptr += sprintf(ptr, "Region 5 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);

    reg_value = readl(EMI_MPUL);
    d0 = (reg_value & 0x7);
    d1 = (reg_value >> 3) & 0x7;
    d2 = (reg_value >> 6) & 0x7;
    d3 = (reg_value >> 9) & 0x7;
    ptr += sprintf(ptr, "Region 6 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);

    d0 = ((reg_value>>16) & 0x7);
    d1 = ((reg_value>>16) >> 3) & 0x7;
    d2 = ((reg_value>>16) >> 6) & 0x7;
    d3 = ((reg_value>>16) >> 9) & 0x7;
    ptr += sprintf(ptr, "Region 7 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);

    ptr += sprintf(ptr, "Lock status --> 0x%x \n", readl(EMI_MLST));
    return strlen(buf);
}

static ssize_t emi_mpu_store(struct device_driver *driver, const char *buf, size_t count)
{
    int i;    
    unsigned int start_addr;
    unsigned int end_addr; 
    unsigned int region; 
    unsigned int access_permission;
    char *command;
    char *ptr;
    char *token [5];

    if ((strlen(buf) + 1) > MAX_EMI_MPU_STORE_CMD_LEN) {
        printk(KERN_CRIT "emi_mpu_store command overflow.");
        return count;
    }
    printk(KERN_CRIT "emi_mpu_store: %s\n", buf);

    command = kmalloc((size_t)MAX_EMI_MPU_STORE_CMD_LEN, GFP_KERNEL);
    if (!command) {
        return count;
    }
    strcpy(command, buf);
    ptr = (char *)buf;

    if (!strncmp(buf, EN_MPU_STR, strlen(EN_MPU_STR))) {
        i = 0;
        while (ptr != NULL) {
            ptr = strsep(&command, " ");
            token[i] = ptr;
            printk(KERN_DEBUG "token[%d] = %s\n", i, token[i]);
            i++;
        }
        for (i = 0; i < 5; i++) {
            printk(KERN_DEBUG "token[%d] = %s\n", i, token[i]);
        }

        start_addr = simple_strtoul(token[1], &token[1], 16);
        end_addr = simple_strtoul(token[2], &token[2], 16);       
        region = simple_strtoul(token[3], &token[3], 16);
        access_permission = simple_strtoul(token[4], &token[4], 16);
        emi_mpu_set_region_protection(start_addr, end_addr, region, access_permission);
        printk(KERN_CRIT "Set EMI_MPU: start: 0x%x, end: 0x%x, region: %d, permission: 0x%x.\n", start_addr, end_addr, region, access_permission);
    } else if (!strncmp(buf, DIS_MPU_STR, strlen(DIS_MPU_STR))) {
        i = 0;
        while (ptr != NULL) {
            ptr = strsep (&command, " ");
            token[i] = ptr;
            printk(KERN_DEBUG "token[%d] = %s\n", i, token[i]);
            i++;
        }
        for (i = 0;i < 5; i++) {
            printk(KERN_DEBUG "token[%d] = %s\n", i, token[i]);
        }

        start_addr = simple_strtoul(token[1], &token[1], 16);
        end_addr = simple_strtoul(token[2], &token[2], 16);
        region = simple_strtoul(token[3], &token[3], 16);
        emi_mpu_set_region_protection(0x0, 0x0, region, SET_ACCESS_PERMISSON(NO_PROTECTION, NO_PROTECTION, NO_PROTECTION, NO_PROTECTION));
        printk("set EMI MPU: start: 0x%x, end: 0x%x, region: %d, permission: 0x%x\n", 0, 0, region, SET_ACCESS_PERMISSON(NO_PROTECTION, NO_PROTECTION, NO_PROTECTION, NO_PROTECTION));
    } else if (!strncmp(buf, TEST_STR, strlen(TEST_STR))) {
        i = 0;
        while (ptr != NULL) {
            ptr = strsep(&command, " ");
            token[i] = ptr;
            printk(KERN_DEBUG "token[%d] = %s\n", i, token[i]);
            i++;
        }
        for (i = 0; i < 2; i++) {
            printk(KERN_DEBUG "token[%d] = %s\n", i, token[i]);
        }

	 start_addr = simple_strtoul(token[1], &token[1], 16);	 
	 end_addr = simple_strtoul(token[2], &token[2], 16);
	 printk(KERN_CRIT "[EMI] start_addr = 0x%x, end_addr = 0x%x \n", start_addr, end_addr);
	 mt65xx_reg_sync_writel(start_addr, EMI_DRVA); 
	 mt65xx_reg_sync_writel(end_addr, EMI_DRVB); 
        //mt65xx_reg_sync_writel((start_addr<<28) | (start_addr<<24) |(start_addr<<20) |(start_addr<<16) |(start_addr<<12) |(start_addr<<8) |(start_addr<<4) |start_addr, EMI_DRVA); 
        //mt65xx_reg_sync_writel((start_addr<<20) |(start_addr<<16), EMI_DRVB); 
	 printk(KERN_CRIT "[EMI] EMI_DRVA = 0x%x, EMI_DRVB= 0x%x \n", readl(EMI_DRVA), readl(EMI_DRVB));
    } else {
        printk(KERN_CRIT "Unkown emi_mpu command.\n");
    }

    kfree(command);

    return count;
}

DRIVER_ATTR(mpu_config, 0644, emi_mpu_show, emi_mpu_store);

static struct device_driver emi_mpu_ctrl =
{
    .name = "emi_mpu_ctrl",
    .bus = &platform_bus_type,
    .owner = THIS_MODULE,
};


static int __init emi_mpu_mod_init(void)
{
    int ret;

    printk(KERN_INFO "Initialize EMI MPU.\n");    
  
    spin_lock_init(&emi_mpu_lock);
    
    __set_emi_mpu_key();

    mt65xx_reg_sync_writel(0x80008000, EMI_MPUM);   //lock EMI_MPUM
    mt65xx_reg_sync_writel(0x80008000, EMI_MPUN);   //lock EMI_MPUN
	
    __clear_emi_mpu_vio();
    

    /*
     *Set Device APC for EMI-MPU.
     */
    /* Unmask debug mask */
    mt65xx_reg_sync_writel(readl(DEVAPC_APC_CON) & (~(0x1 << 2)), DEVAPC_APC_CON);
    mt65xx_reg_sync_writel(readl(DEVAPC_PD_APC_CON) & (~(0x1 << 2)), DEVAPC_PD_APC_CON);
    /* Clear debug information */
    mt65xx_reg_sync_writel(0x80000000, DEVAPC_VIO_DBG0);
    /* Clear status */
    mt65xx_reg_sync_writel(readl(DEVAPC_D0_VIO_STA_3) | ABORT_EMI , DEVAPC_D0_VIO_STA_3);   

    
    /* 
     * NoteXXX: Interrupts of vilation (including SPC in SMI, or EMI MPU) 
     *          are triggered by the device APC.
     *          Need to share the interrupt with the SPC driver. 
     */
    ret = request_irq(MT_APARM_DOMAIN_IRQ_ID, (irq_handler_t)mpu_violation_irq, IRQF_TRIGGER_LOW | IRQF_SHARED, "mt_emi_mpu", &emi_mpu_ctrl); 
    if (ret != 0) {
        printk(KERN_CRIT "Fail to request EMI_MPU interrupt. Error = %d.\n", ret);
        return ret;
    }
    
#if !defined(USER_BUILD_KERNEL)
    /* register driver and create sysfs files */
    ret = driver_register(&emi_mpu_ctrl);
    if (ret) {
        printk(KERN_CRIT "Fail to register EMI_MPU driver.\n");
    }
    ret = driver_create_file(&emi_mpu_ctrl, &driver_attr_mpu_config);
    if (ret) {
        printk(KERN_CRIT "Fail to create EMI_MPU sysfs file.\n");
    }
#endif

    /*Init for testing*/
    //emi_mpu_set_region_protection(0x80000000,
    //                              0x815FFFFF, /*MD_IMG_REGION_LEN*/
    //                                    0,    /*region*/
    //                                    SET_ACCESS_PERMISSON(SEC_R_NSEC_R, SEC_R_NSEC_R, SEC_R_NSEC_R, SEC_R_NSEC_R));

    return 0;
}

static void __exit emi_mpu_mod_exit(void)
{
}

module_init(emi_mpu_mod_init);
module_exit(emi_mpu_mod_exit);

EXPORT_SYMBOL(emi_mpu_set_region_protection);
EXPORT_SYMBOL(emi_mem_psrv_diable);
//EXPORT_SYMBOL(start_mm_mau_protect);
