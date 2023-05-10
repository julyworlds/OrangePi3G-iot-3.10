#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/aee.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_clkmgr.h>
#include <asm/io.h>

#include <mach/m4u.h>
#include <mach/mt_smi.h>
#include "smi_reg.h"
#include "smi_common.h"

#undef pr_fmt
#define pr_fmt(fmt) "[SMI]" fmt

#define SMI_LOG_TAG "SMI"
//#define SMI_DEFAULT_VR

typedef struct
{
    spinlock_t SMI_lock;
    unsigned int pu4ConcurrencyTable[SMI_BWC_SCEN_CNT];  //one bit represent one module
} SMI_struct;

static SMI_struct g_SMIInfo;

unsigned int gLarbBaseAddr[SMI_LARB_NR] = 
    {LARB0_BASE}; 


char *smi_port_name[16] = 
{
    "DISP_OVL"             ,
    "DISP_RDMA"            ,
    "DISP_WDMA"            ,
    "DISP_DBI"             ,
    "CAM_WDMA"             ,
    "MM_CMDQ"              ,
    "VENC_BSDMA_VDEC_POST0",
    "MDP_RDMA"             ,
    "MDP_WDMA"             ,
    "MDP_ROTO"             ,
    "MDP_ROTCO"            ,
    "MDP_ROTVO"            ,
    "VENC_MVQP"            ,
    "VENCMC"               ,
    "VENC_CDMA_VDEC_CDMA"  ,
    "VENC_REC_VDEC_WDMA"   ,    
};

int larb_on_count = 0;

int larb_clock_on(int larb_id, const char *mod_name) 
{

    larb_on_count++;
    //SMIDBG("larb_clock_on, %s, %d \n", mod_name, larb_on_count);

#ifndef CONFIG_EARLY_LINUX_PORTING

    switch(larb_id)
    {
        case 0: 
            // enable_clock(MT_CG_SMI_COMMON_SW_CG, mod_name);
            // enable_clock(MT_CG_SMI_LARB0_SW_CG, mod_name);
            break;

        default: 
            SMIERR("larb_clock_on: larb_id %s error\n", mod_name);
            break;
    }

#endif	

  return 0;
}
EXPORT_SYMBOL(larb_clock_on);


int larb_clock_off(int larb_id, const char *mod_name) 
{

    larb_on_count--;
    //SMIDBG("larb_clock_off, %s, %d \n", mod_name, larb_on_count);

#ifndef CONFIG_EARLY_LINUX_PORTING

    switch(larb_id)
    {
        case 0: 
            // disable_clock(MT_CG_SMI_COMMON_SW_CG, mod_name);
            // disable_clock(MT_CG_SMI_LARB0_SW_CG, mod_name);
            break;

        default: 
            SMIERR("larb_clock_off: larb_id %s error\n", mod_name);			
            break;
    }

#endif	

    return 0;

}
EXPORT_SYMBOL(larb_clock_off);


#define LARB_BACKUP_REG_SIZE 128
static unsigned int* pLarbRegBackUp[SMI_LARB_NR];

/*****************************************************************************
 * FUNCTION
 *    larb_reg_backup
 * DESCRIPTION
 *    Backup register for system suspend.
 * PARAMETERS
 *	  param1 : [IN] const int larb
 *				  larb index.
 * RETURNS
 *    None.
 ****************************************************************************/
static void larb_reg_backup(const int larb)
{
    unsigned int* pReg = pLarbRegBackUp[larb];
    int i;
    unsigned int larb_base = gLarbBaseAddr[larb];
    
    //SMI registers
    for(i=0; i<2; i++)
        *(pReg++) = COM_ReadReg32(REG_SMI_SECUR_CON(i));
	
    *(pReg++) = M4U_ReadReg32(larb_base, SMI_LARB_CON);

    for(i=0; i<MAU_ENTRY_NR; i++)
    {
        *(pReg++) = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_START(i));
        *(pReg++) = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_END(i));
        *(pReg++) = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_GID(i));
    }
}

/*****************************************************************************
 * FUNCTION
 *    larb_reg_restore
 * DESCRIPTION
 *    Restore register for system resume.
 * PARAMETERS
 *	  param1 : [IN] const int larb
 *				  larb index.
 * RETURNS
 *    None.
 ****************************************************************************/
static void larb_reg_restore(const int larb)
{
    unsigned int* pReg = pLarbRegBackUp[larb];
    int i;
    unsigned int regval, regval1, regval2;
    unsigned int larb_base = gLarbBaseAddr[larb];

    //SMI registers
    for(i=0; i<2; i++)
        COM_WriteReg32(REG_SMI_SECUR_CON(i), *(pReg++) );
    
    //warning: larb_con is controlled by set/clr
    regval = *(pReg++);
    M4U_WriteReg32(larb_base, SMI_LARB_CON_CLR, ~(regval));
    M4U_WriteReg32(larb_base, SMI_LARB_CON_SET, (regval));

    for(i=0; i<MAU_ENTRY_NR; i++)
    {
        M4U_WriteReg32(larb_base, SMI_MAU_ENTR_START(i), *(pReg++));
        M4U_WriteReg32(larb_base, SMI_MAU_ENTR_END(i), *(pReg++));
        M4U_WriteReg32(larb_base, SMI_MAU_ENTR_GID(i), *(pReg++));
    }
    
    regval = M4U_ReadReg32(larb_base , SMI_LARB_STAT);
    regval1 = M4U_ReadReg32(larb_base , SMI_LARB_MON_BUS_REQ0);
    regval2 = M4U_ReadReg32(larb_base , SMI_LARB_MON_BUS_REQ1);
    
    if(0 == regval)
    {
        int retry_count = 0;
        
        SMIMSG("Init OSTD for larb_base: 0x%x\n" , larb_base);
        // Write 0x60 = 0xFFFF_FFFF, enable BW limiter
        M4U_WriteReg32(larb_base , 0x60 , 0xffffffff);
        // Polling 0x600 = 0xaaaa        
        for(retry_count= 0; retry_count<64; retry_count++)
        {
            if(M4U_ReadReg32(larb_base , 0x600) == 0xaaaa)
            {
                //Step3.   Once it is found 0x600 == 0xaaaa, we can start to enable outstanding limiter and set outstanding limit
                break;
            }
            SMIMSG("Larb: 0x%x busy : waiting for idle\n" , larb_base);
            udelay(500);
        }

        // Write 0x60 = 0x0, disable BW limiter
        M4U_WriteReg32(larb_base , 0x60 , 0x0);
        // enable ISTD
        M4U_WriteReg32(larb_base , SMI_LARB_OSTD_CTRL_EN , 0xffffffff);
    }
    else
    {
        SMIMSG("Larb%d is busy : 0x%x , port:0x%x,0x%x ,fail to set OSTD\n" , larb , regval , regval1 , regval2);
        //smi_dumpDebugMsg();
        //SMIERR("DISP_MDP LARB%d OSTD cannot be set:0x%x,port:0x%x,0x%x\n" , larb , regval , regval1 , regval2);
    }

    
}


/*****************************************************************************
 * FUNCTION
 *    on_larb_power_on
 * DESCRIPTION
 *    Callback after larb clock is enabled.
 * PARAMETERS
 *	  param1 : [IN] struct larb_monitor *h
 *				  No used in this function. 
 *	  param2 : [IN] int larb_idx
 *				  larb index. 
 * RETURNS
 *    None.
 ****************************************************************************/
static void on_larb_power_on(struct larb_monitor *h, int larb_idx)
{
    SMIMSG("on_larb_power_on(), larb_idx=%d \n", larb_idx);
    larb_reg_restore(larb_idx);    
    // larb_clock_on(0, "SMI");
#ifdef SMI_DEFAULT_VR    
    {
        MTK_SMI_BWC_CONFIG p_conf;
        p_conf.b_on_off = 1;
        p_conf.scenario = SMI_BWC_SCEN_VR1066;
        smi_bwc_config(&p_conf);
    }  
#else
    {
        MTK_SMI_BWC_CONFIG p_conf;
        p_conf.b_on_off = 1;
        p_conf.scenario = SMI_BWC_SCEN_NORMAL;
        smi_bwc_config(&p_conf);
    }  
#endif
}

/*****************************************************************************
 * FUNCTION
 *    on_larb_power_on
 * DESCRIPTION
 *    Callback before larb clock is disabled
 * PARAMETERS
 *	  param1 : [IN] struct larb_monitor *h
 *				  No used in this function. 
 *	  param2 : [IN] int larb_idx
 *				  larb index. 
 * RETURNS
 *    None.
 ****************************************************************************/
void on_larb_power_off(struct larb_monitor *h, int larb_idx)
{
    SMIMSG("on_larb_power_off(), larb_idx=%d \n", larb_idx);
    larb_reg_backup(larb_idx);
    // larb_clock_off(0, "SMI");	
}


void dump_smi_register(void)
{
    int i;

    SMIMSG(" SMI COMMON Register Start ======= \n");
    for(i=0;i<4096/8;i+=4)
    {
    	SMIMSG("+0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \n", 8*i, 
    	M4U_ReadReg32(SMI_COMMON_EXT_BASE, 8*i + 4*0), M4U_ReadReg32(SMI_COMMON_EXT_BASE, 8*i + 4*1),
    	M4U_ReadReg32(SMI_COMMON_EXT_BASE, 8*i + 4*2), M4U_ReadReg32(SMI_COMMON_EXT_BASE, 8*i + 4*3),
    	M4U_ReadReg32(SMI_COMMON_EXT_BASE, 8*i + 4*4), M4U_ReadReg32(SMI_COMMON_EXT_BASE, 8*i + 4*5),
    	M4U_ReadReg32(SMI_COMMON_EXT_BASE, 8*i + 4*6), M4U_ReadReg32(SMI_COMMON_EXT_BASE, 8*i + 4*7));
    }
    SMIMSG(" SMI COMMONR egister End ========== \n");

    SMIMSG(" SMI LARB Register Start ======= \n");
    for(i=0;i<4096/8;i+=4)
    {
    	SMIMSG("+0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \n", 8*i, 
    	M4U_ReadReg32(LARB0_BASE, 8*i + 4*0), M4U_ReadReg32(LARB0_BASE, 8*i + 4*1),
    	M4U_ReadReg32(LARB0_BASE, 8*i + 4*2), M4U_ReadReg32(LARB0_BASE, 8*i + 4*3),
    	M4U_ReadReg32(LARB0_BASE, 8*i + 4*4), M4U_ReadReg32(LARB0_BASE, 8*i + 4*5),
    	M4U_ReadReg32(LARB0_BASE, 8*i + 4*6), M4U_ReadReg32(LARB0_BASE, 8*i + 4*7));
    }
    SMIMSG(" SMI LARB egister End ========== \n");

}
EXPORT_SYMBOL(dump_smi_register);


int smi_bwc_config( MTK_SMI_BWC_CONFIG* p_conf )
{

    int i;
    unsigned wdata = 0;
    unsigned threshold = 0;
    unsigned int larb_base = gLarbBaseAddr[0];
    unsigned int * pu4LocalCnt = NULL;

    // Profile reference count support
    unsigned long u4Concurrency = 0;
    MTK_SMI_BWC_SCEN eFinalScen;
    static MTK_SMI_BWC_SCEN ePreviousFinalScen = SMI_BWC_SCEN_CNT;

    if((SMI_BWC_SCEN_CNT <= p_conf->scenario) || (0 > p_conf->scenario))
    {
        SMIERR("Incorrect SMI BWC config : 0x%x, how could this be...\n" , p_conf->scenario);
        return -1;
    }
//Debug - S
//SMIMSG("SMI setTo%d,%s,%d\n" , p_conf->scenario , (p_conf->b_on_off ? "on" : "off") , ePreviousFinalScen);
//Debug - E

    spin_lock(&g_SMIInfo.SMI_lock);

    if(p_conf->b_on_off)
    {
        //turn on certain scenario
        g_SMIInfo.pu4ConcurrencyTable[p_conf->scenario] += 1;

        if(NULL != pu4LocalCnt)
        {
            pu4LocalCnt[p_conf->scenario] += 1;
        }
    }
    else
    {
        //turn off certain scenario
        if(0 == g_SMIInfo.pu4ConcurrencyTable[p_conf->scenario])
        {
            SMIMSG("Too many turning off for global SMI profile:%d,%d\n" , p_conf->scenario , g_SMIInfo.pu4ConcurrencyTable[p_conf->scenario]);
        }
        else
        {
            g_SMIInfo.pu4ConcurrencyTable[p_conf->scenario] -= 1;
        }

        if(NULL != pu4LocalCnt)
        {
            if(0 == pu4LocalCnt[p_conf->scenario])
            {
                SMIMSG("Process : %s did too many turning off for local SMI profile:%d,%d\n" , current->comm ,p_conf->scenario , pu4LocalCnt[p_conf->scenario]);
            }
            else
            {
                pu4LocalCnt[p_conf->scenario] -= 1;
            }
        }
    }

    for(i=0 ; i < SMI_BWC_SCEN_CNT ; i++)
    {
        if(g_SMIInfo.pu4ConcurrencyTable[i])
        {
            u4Concurrency |= (1 << i);
        }
    }

    if((1 << SMI_BWC_SCEN_VR1066) & u4Concurrency)
    {
        eFinalScen = SMI_BWC_SCEN_VR1066;
    }
    else if((1 << SMI_BWC_SCEN_VP1066) & u4Concurrency)
    {
        eFinalScen = SMI_BWC_SCEN_VP1066;
    }
    else
    {
        eFinalScen = SMI_BWC_SCEN_NORMAL;
    }

    if(ePreviousFinalScen == eFinalScen)
    {
        SMIMSG("Scen equal%d,don't change\n" , eFinalScen);
        spin_unlock(&g_SMIInfo.SMI_lock);
        return 0;
    }
    else
    {
        ePreviousFinalScen = eFinalScen;
    }
    
    /*turn on larb clock*/    
    for(i=0; i<SMI_LARB_NR; i++){
        larb_clock_on(i, "SMI");
    }

    /*Bandwidth Limiter*/
    switch( eFinalScen )
    {
#if 0    
    case SMI_BWC_SCEN_VP1066:
        wdata = 391; //BW limit = x/4096
        wdata |= (1<<11); // bw filter enable
        wdata |= (1<<12); // bw hard limit enable		
        M4U_WriteReg32( 0x0, REG_SMI_L1ARB1, wdata );   
        wdata = 1494; //BW limit 
        wdata |= (1<<11); // bw filter enable, soft mode
        M4U_WriteReg32( 0x0, REG_SMI_L1ARB0, wdata );          
        	
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x00, 0x4 );
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x04, 0x1 ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x08, 0x1 ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x10, 0x1 ); 	
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x14, 0x1 ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x18, 0x2 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x1c, 0x1 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x20, 0x1 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x24, 0x1 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x28, 0x1 );		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x2c, 0x1 );
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x30, 0x1 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x34, 0x1 ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x38, 0x1 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x3c, 0x1 ); 

        break;
#endif        
    case SMI_BWC_SCEN_VR1066:
        SMIMSG("set as SMI_BWC_SCEN_VR1066\n");		
#if 0
        wdata = 432; //BW limit = x/4096
        wdata |= (1<<11); // bw filter enable
        wdata |= (1<<12); // bw hard limit enable		
        M4U_WriteReg32( 0x0, REG_SMI_L1ARB1, wdata );   
#else
        wdata = 0;   
        threshold = 7;
        wdata |= (1<<12); // bw hard limit enable		
        wdata |= (threshold<<13);
        wdata |= (threshold<<18);
        wdata |= (1<<23);
        M4U_WriteReg32( 0x0, REG_SMI_L1ARB1, wdata ); 
#endif

        wdata = 2026; //BW limit 
        //wdata |= (1<<11); // bw filter enable, soft mode
        M4U_WriteReg32( 0x0, REG_SMI_L1ARB0, wdata ); 

		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x00, 0x6 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x04, 0x1 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x08, 0x1 ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x10, 0x4 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x14, 0x1 ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x18, 0x2 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x1c, 0x3 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x20, 0x1 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x24, 0x3 ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x28, 0x3 ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x2c, 0x3 ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x30, 0x1 ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x34, 0x3 ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x38, 0x1 ); 				
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x3c, 0x1 ); 						


        /*reduce command buffer*/
        //if( p_conf->b_reduce_command_buffer )
        {
            /*SMI COMMON reduce command buffer*/
            M4U_WriteReg32( 0x0, REG_SMI_L1LEN, 0xb );				
            M4U_WriteReg32( 0x0, REG_SMI_READ_FIFO_TH, 0x323 );
            M4U_WriteReg32( 0x0, REG_SMI_M4U_TH, 0x10c85 );						
             
        }		
		
        break;
        
    case SMI_BWC_SCEN_NORMAL:
    default:
        SMIMSG("set as SMI_BWC_SCEN_NORMAL\n");

        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x00, 0x1f ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x04, 0x1f ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x08, 0x1f ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x10, 0x1f ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x14, 0x1f ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x18, 0x1f ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x1c, 0x1f ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x20, 0x1f ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x24, 0x1f ); 
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x28, 0x1f ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x2c, 0x1f ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x30, 0x1f ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x34, 0x1f ); 		
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x38, 0x1f ); 				
        M4U_WriteReg32( larb_base, SMI_LARB_OSTD_PORT+0x3c, 0x1f ); 	
		
        M4U_WriteReg32( 0x0, REG_SMI_L1ARB0, 0x0   );   //larb0 change to default
        M4U_WriteReg32( 0x0, REG_SMI_L1ARB1, 0x0   );   //larb1 change to default

        {
            /*SMI COMMON reduce command buffer*/
            M4U_WriteReg32( 0x0, REG_SMI_L1LEN, 0x3 );				
            M4U_WriteReg32( 0x0, REG_SMI_READ_FIFO_TH, 0x320 );
            M4U_WriteReg32( 0x0, REG_SMI_M4U_TH, 0x29ca7 );						
             
        }	
		
        break;
    }



    #if 0 /*dump message*/
    {
        #define _SMI_BC_DUMP_REG( _base_, _off_ ) \
            SMIMSG( "[SMI_REG] %s + %s = 0x%08X !\n", #_base_, #_off_, M4U_ReadReg32( _base_, _off_ ) );

        /*Bandwidth Limiter*/
        _SMI_BC_DUMP_REG( 0x0, REG_SMI_L1ARB0 );   //larb0 venc
        _SMI_BC_DUMP_REG( 0x0, REG_SMI_L1ARB1 );   //larb1 vdec

        
        /*SMI COMMON reduce command buffer*/
        _SMI_BC_DUMP_REG( 0x0, REG_SMI_L1LEN );
        _SMI_BC_DUMP_REG( 0x0, REG_SMI_READ_FIFO_TH );		
        _SMI_BC_DUMP_REG( 0x0, REG_SMI_M4U_TH );

        _SMI_BC_DUMP_REG( larb_base, SMI_LARB_OSTD_CTRL_EN);

        for(i = 0; i<20; i++)
        {
            _SMI_BC_DUMP_REG( larb_base, SMI_LARB_OSTD_PORT+0x4*i); 
        }
        
        /*SMI LARB reduce command buffer (RO register)*/
        _SMI_BC_DUMP_REG( LARB0_BASE, 0x10 );
    }


    dump_smi_register();
	
    #endif


    /*turn off larb clock*/    
    for(i=0; i<SMI_LARB_NR; i++){
        larb_clock_off(i, "SMI");
    }

    spin_unlock(&g_SMIInfo.SMI_lock);

    SMIMSG("ScenTo:%d,turn %s,Curr Scen:%d,%d,%d,%d\n" , p_conf->scenario , (p_conf->b_on_off ? "on" : "off") , eFinalScen , 
        g_SMIInfo.pu4ConcurrencyTable[SMI_BWC_SCEN_NORMAL] , g_SMIInfo.pu4ConcurrencyTable[SMI_BWC_SCEN_VR1066] , g_SMIInfo.pu4ConcurrencyTable[SMI_BWC_SCEN_VP1066]);

//Debug usage - S
//smi_dumpDebugMsg();
//SMIMSG("Config:%d,%d,%d\n" , eFinalScen , g_SMIInfo.pu4ConcurrencyTable[SMI_BWC_SCEN_NORMAL] , (NULL == pu4LocalCnt ? (-1) : pu4LocalCnt[p_conf->scenario]));
//Debug usage - E

    return 0;
    
}

#if 1
struct larb_monitor larb_monitor_handler =
{
    .level = LARB_MONITOR_LEVEL_HIGH,
    .backup = on_larb_power_off,
    .restore = on_larb_power_on	
};
#endif

/*****************************************************************************
 * FUNCTION
 *    smi_common_init
 * DESCRIPTION
 *    Allocate register backup memory.
 * PARAMETERS
 *    None.
 * RETURNS
 *    Type: Integer. always zero.
 ****************************************************************************/
int smi_common_init(void)
{
    int i;

    for(i=0; i<SMI_LARB_NR; i++)
    {
        pLarbRegBackUp[i] = (unsigned int*)kmalloc(LARB_BACKUP_REG_SIZE, GFP_KERNEL|__GFP_ZERO);
        if(pLarbRegBackUp[i]==NULL)
        {
        	  SMIERR("pLarbRegBackUp kmalloc fail %d \n", i);
        }  
    }

    register_larb_monitor(&larb_monitor_handler);
    return 0;
}


/*****************************************************************************
 * FUNCTION
 *    smi_bwc_mm_info_set
 * DESCRIPTION
 *	  Save BWC properties
 * PARAMETERS
 *    None.
 * RETURNS
 *    None.
 ****************************************************************************/

// GMP start 
static MTK_SMI_BWC_MM_INFO g_smi_bwc_mm_info= {
    0, 
    0,
    {0,0},
    {0,0},
    {0,0},
    {0,0},
    0,
    0,
    0
};
    
void smi_bwc_mm_info_set(int property_id, long val1, long val2){
    switch(property_id){
        case SMI_BWC_INFO_CON_PROFILE:
            g_smi_bwc_mm_info.concurrent_profile = (int)val1;
            break;
        case SMI_BWC_INFO_SENSOR_SIZE:
            g_smi_bwc_mm_info.sensor_size[0] = val1;
            g_smi_bwc_mm_info.sensor_size[1] = val2;
            break;
        case SMI_BWC_INFO_VIDEO_RECORD_SIZE: 
            g_smi_bwc_mm_info.video_record_size[0]= val1;
            g_smi_bwc_mm_info.video_record_size[1]= val2;
            break;
        case SMI_BWC_INFO_DISP_SIZE:
            g_smi_bwc_mm_info.display_size[0]= val1;
            g_smi_bwc_mm_info.display_size[1]= val2;
            break;
        case SMI_BWC_INFO_TV_OUT_SIZE:
            g_smi_bwc_mm_info.tv_out_size[0]= val1;
            g_smi_bwc_mm_info.tv_out_size[1]= val2;
            break;
        case SMI_BWC_INFO_FPS:
            g_smi_bwc_mm_info.fps = (int)val1;
            break;
        case SMI_BWC_INFO_VIDEO_ENCODE_CODEC:
            g_smi_bwc_mm_info.video_encode_codec = (int)val1;
            break;
        case SMI_BWC_INFO_VIDEO_DECODE_CODEC:  
            g_smi_bwc_mm_info.video_decode_codec = (int)val1;
            break;
    }    
 }

// GMP end 

/*****************************************************************************
 * FUNCTION
 *    smi_ioctl
 * DESCRIPTION
 *	  File operations - unlocked_ioctl
 *		1. call copy_from_user to get user space parameter
 *		2. call internal function by operation.
 * PARAMETERS
 *	  param1 : [IN] struct file * pFile
 *				  file structure*.
 *	  param2 : [IN] unsigned int cmd
 *				  operation command.
 *	  param3 : [IN] unsigned long param
 *				  parameter of ioctl.
 * RETURNS
 *    Type: Integer. zero means success and others mean error.
 ****************************************************************************/
static long smi_ioctl(struct file * pFile,
                       unsigned int cmd,
                       unsigned long param)
{
    int ret = 0;
    
    switch (cmd)
    {
        case MTK_CONFIG_MM_MAU:
        {
        	MTK_MAU_CONFIG b;
       		if(copy_from_user(&b, (void __user *)param, sizeof(b)))
        	{
            	SMIERR("copy_from_user failed!");
            	ret = -EFAULT;
        	} else {
                mau_config(&b);
			}
        	return ret;
    	}
        case MTK_IOC_SMI_BWC_CONFIG:
            {
                MTK_SMI_BWC_CONFIG cfg;
                ret = copy_from_user(&cfg, (void*)param , sizeof(MTK_SMI_BWC_CONFIG));
                if(ret)
                {
                    SMIMSG(" SMI_BWC_CONFIG, copy_from_user failed: %d\n", ret);
                    return -EFAULT;
                }  

                smi_bwc_config( &cfg );
            
            }
            break;
        // GMP start 
        case MTK_IOC_SMI_BWC_INFO_SET:
        {
                MTK_SMI_BWC_INFO_SET cfg;
                //SMIMSG("Handle MTK_IOC_SMI_BWC_INFO_SET request... start");
                ret = copy_from_user(&cfg, (void*)param , sizeof(MTK_SMI_BWC_INFO_SET));
                if(ret)
                {
                    SMIMSG(" MTK_IOC_SMI_BWC_INFO_SET, copy_to_user failed: %d\n", ret);
                    return -EFAULT;
                }  
                // Set the address to the value assigned by user space program
                smi_bwc_mm_info_set(cfg.property, cfg.value1, cfg.value2);
                //SMIMSG("Handle MTK_IOC_SMI_BWC_INFO_SET request... finish");
                break;  
        }
        case MTK_IOC_SMI_BWC_INFO_GET:
        {
                MTK_SMI_BWC_INFO_GET cfg;
                MTK_SMI_BWC_MM_INFO * return_address = NULL;
            
                //SMIMSG("Handle MTK_IOC_SMI_BWC_INFO_GET request... start");
                ret = copy_from_user(&cfg, (void*)param , sizeof(MTK_SMI_BWC_INFO_GET));

                if(ret)
                {
                    SMIMSG(" MTK_IOC_SMI_BWC_INFO_GET, copy_to_user failed: %d\n", ret);
                    return -EFAULT;
                }  

                return_address = (MTK_SMI_BWC_MM_INFO *)cfg.return_address;
                if( return_address != NULL){
                    ret = copy_to_user((void*) return_address, (void*)&g_smi_bwc_mm_info, sizeof(MTK_SMI_BWC_MM_INFO));

                    if(ret)
                    {
                        SMIMSG(" MTK_IOC_SMI_BWC_INFO_GET, copy_to_user failed: %d\n", ret);
                        return -EFAULT;
                    }
                }
                //SMIMSG("Handle MTK_IOC_SMI_BWC_INFO_GET request... finish");
                break;
        }
     // GMP end   
        default:
            return -1;
    }

	return ret;
}


static const struct file_operations smiFops =
{
	.owner = THIS_MODULE,
	.unlocked_ioctl = smi_ioctl,
};

static struct cdev * pSmiDev = NULL;
static dev_t smiDevNo = MKDEV(MTK_SMI_MAJOR_NUMBER,0);
/*****************************************************************************
 * FUNCTION
 *    smi_register
 * DESCRIPTION
 *    1. Register SMI Device Number
 *    2. Allocate and Initial SMI cdev struct
 *    3. Call cdev_add to add this cdev.
 * PARAMETERS
 *    None.
 * RETURNS
 *    Type: Integer.  zero mean success and others mean fail.
 ****************************************************************************/
static inline int smi_register(void)
{
    if (alloc_chrdev_region(&smiDevNo, 0, 1,"MTK_SMI")){
        SMIERR("Allocate device No. failed");
        return -EAGAIN;
    }
    //Allocate driver
    pSmiDev = cdev_alloc();

    if (NULL == pSmiDev) {
        unregister_chrdev_region(smiDevNo, 1);
        SMIERR("Allocate mem for kobject failed");
        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(pSmiDev, &smiFops);
    pSmiDev->owner = THIS_MODULE;

    //Add to system
    if (cdev_add(pSmiDev, smiDevNo, 1)) {
        SMIERR("Attatch file operation failed");
        unregister_chrdev_region(smiDevNo, 1);
        return -EAGAIN;
    }
    return 0;
}


static struct class *pSmiClass = NULL;
/*****************************************************************************
 * FUNCTION
 *    smi_probe
 * DESCRIPTION
 *    1. Call smi_register to register SMI Device Number, allocate and Initial SMI cdev struct, call cdev_add.
 *    2. Call class_create and device_create to add SMI device to kerne.
 *    3. Call smi_common_init.
 *    4. Call mau_init.
 * PARAMETERS
 *	  param1 : [IN] struct platform_device *pdev
 *				  No used in this function. 
 * RETURNS
 *    Type: Integer. 0 mean success and others mean fail.
 ****************************************************************************/
static int smi_probe(struct platform_device *pdev)
{
    struct device* smiDevice = NULL;

    if (NULL == pdev) {
        SMIERR("platform data missed");
        return -ENXIO;
    }

    if (smi_register()) {
        dev_err(&pdev->dev,"register char failed\n");
        return -EAGAIN;
    }

    pSmiClass = class_create(THIS_MODULE, "MTK_SMI");
    if (IS_ERR(pSmiClass)) {
        int ret = PTR_ERR(pSmiClass);
        SMIERR("Unable to create class, err = %d", ret);
        return ret;
    }
    smiDevice = device_create(pSmiClass, NULL, smiDevNo, NULL, "MTK_SMI");

    smi_common_init();

    mau_init();
    
#ifndef MTK_M4U_EXT_PAGE_TABLE
    {
        MTK_MAU_CONFIG mau_disp;
        mau_disp.entry = 0;
        mau_disp.larb = 0;
        mau_disp.start = 0x20000000;
        mau_disp.end = 0xFFFFFFFF;
        mau_disp.virt = 1;
        mau_disp.port_msk = 0xFFFF;
        mau_disp.monitor_read = 1;
        mau_disp.monitor_write = 1;
        mau_config(&mau_disp);
    }
#endif    

#ifdef SMI_DEFAULT_VR
    {
        MTK_SMI_BWC_CONFIG p_conf;
        p_conf.b_on_off = 1;
        p_conf.scenario = SMI_BWC_SCEN_VR1066;
        smi_bwc_config(&p_conf);
    }
#else
    {
        MTK_SMI_BWC_CONFIG p_conf;
        p_conf.b_on_off = 1;
        p_conf.scenario = SMI_BWC_SCEN_NORMAL;
        smi_bwc_config(&p_conf);
    }  
#endif

    //dump_smi_register();
    
    return 0;
}


/*****************************************************************************
 * FUNCTION
 *    smi_remove
 * DESCRIPTION
 *    1. Remove SMI device.
 *    2. Un-register SMI Device Number.
 * PARAMETERS
 *	  param1 : [IN] struct platform_device *pdev
 *				  No used in this function. 
 * RETURNS
 *    Type: Integer. always zero.
 ****************************************************************************/
static int smi_remove(struct platform_device *pdev)
{
    cdev_del(pSmiDev);
    unregister_chrdev_region(smiDevNo, 1);
    device_destroy(pSmiClass, smiDevNo);
    class_destroy(pSmiClass);
	// NO Free IRQ
    return 0;
}

static int smi_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int smi_resume(struct platform_device *pdev)
{
    return 0;
}

static struct platform_driver smiDrv = {
    .probe	= smi_probe,
    .remove	= smi_remove,
    .suspend= smi_suspend,
    .resume	= smi_resume,
    .driver	= {
    .name	= "MTK_SMI",
    .owner	= THIS_MODULE,
    }
};

/*****************************************************************************
 * FUNCTION
 *    smi_init
 * DESCRIPTION
 *    Call platform_driver_register to register SMI driver
 * PARAMETERS
 *    None.
 * RETURNS
 *    Type: Integer.  zero mean success and others mean fail.
 ****************************************************************************/
static int __init smi_init(void)
{
    if(platform_driver_register(&smiDrv)){
        SMIERR("failed to register MAU driver");
        return -ENODEV;
    }
	return 0;
}

int smi_debug_bus_hanging_detect_ext( unsigned int larbs, int show_dump, int output_gce_buffer){
    SMIMSG("smi_debug_bus_hanging_detect not support on this platform\n");
	return 0;
}


/*****************************************************************************
 * FUNCTION
 *    smi_exit
 * DESCRIPTION
 *    Call platform_driver_unregister to unregister SMI driver
 * PARAMETERS
 *    None.
 * RETURNS
 *    None.
 ****************************************************************************/
static void __exit smi_exit(void)
{
    platform_driver_unregister(&smiDrv);

}

// HAL function to notify SMI when engine state is changed
// Don't remove it.
void smi_dynamic_adj_hint_mhl(int mhl_enable)
{
}


void smi_dynamic_adj_hint(unsigned int dsi2smi_total_pixel)
{
}

module_init(smi_init);
module_exit(smi_exit);

MODULE_DESCRIPTION("MTK SMI driver");
MODULE_AUTHOR("K_zhang<k.zhang@mediatek.com>");
MODULE_LICENSE("GPL");

