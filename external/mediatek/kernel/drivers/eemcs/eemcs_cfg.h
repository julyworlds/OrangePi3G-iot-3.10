#ifndef __EEMCS_CFG_H__
#define __EEMCS_CFG_H__


#define _ECCMNI_SEQ_SUPPORT_   /* ADD sequence number in cccih->reserved */

#define __ECCMNI_SUPPORT__
#define __EEMCS_EXPT_SUPPORT__ /* exception mode support */
//#define _EEMCS_EXCEPTION_UT    //Enable exception mode UT

//#define _ECCMNI_LB_UT_       /* configure EMCS_NET  as UL loopback mode */
//#define _EEMCS_CDEV_LB_UT_     /* configure EMCS_CHAR as UL loopback mode */

//#define _EEMCS_CCCI_LB_UT      /* configure EMCS_CCCI as UL loopback mode*/

//Temp disable for testing
#define __EEMCS_XBOOT_SUPPORT__    // Enable/Disable xBoot flow
//#define _EEMCS_TRACE_SUPPORT    // Enable/Disable xBoot flow tracing
//#define _EEMCS_BOOT_UT          // Enable/Disable xBoot UT

//#define _EEMCS_FS_UT            // Enable FS UT
//#define _EEMCS_RPC_UT            // Enable RPC UT


//******other feature configure******//
#define  ENABLE_AEE_MD_EE				//disable for bring up
#define  EE_INIT_TIMER		(2*HZ)		//sdio exception handshake timeout
#define  EE_HS1_TIMER		(10*HZ)		//sdio exception handshake complete -> AP receive MD_EX msg
#define  EE_HS2_TIMER		(5*HZ)		//AP receive MD_EX msg -> AP receive MD_EX_RECV_OK msg
#define  ENABLE_MD_WDT_PROCESS			//disable for bring up for md not enable wdt at bring up


#define CCCI_MTU_3456B					//ccci user raw data size from (4096-128) to 3456(3.5K-128)
//#define CCCI_SDIO_HEAD					//add sdio header for sdio driver saving info, which is included in 128B



#ifdef _EEMCS_CCCI_LB_UT
#ifndef _EEMCS_BOOT_UT
#define _EEMCS_BOOT_UT
#endif
#endif

#endif //__EEMCS_CFG_H__

