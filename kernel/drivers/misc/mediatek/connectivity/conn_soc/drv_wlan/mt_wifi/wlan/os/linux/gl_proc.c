/*
** Id: //Department/DaVinci/BRANCHES/MT6620_WIFI_DRIVER_V2_3/os/linux/gl_proc.c#1
*/

/*! \file   "gl_proc.c"
    \brief  This file defines the interface which can interact with users in /proc fs.

    Detail description.
*/

/*
** Log: gl_proc.c
 *
 * 11 10 2011 cp.wu
 * [WCXRP00001098] [MT6620 Wi-Fi][Driver] Replace printk by DBG LOG macros in linux porting layer
 * 1. eliminaite direct calls to printk in porting layer.
 * 2. replaced by DBGLOG, which would be XLOG on ALPS platforms.
 *
 * 12 10 2010 kevin.huang
 * [WCXRP00000128] [MT6620 Wi-Fi][Driver] Add proc support to Android Driver for debug and driver status check
 * Add Linux Proc Support
**  \main\maintrunk.MT5921\19 2008-09-02 21:08:37 GMT mtk01461
**  Fix the compile error of SPRINTF()
**  \main\maintrunk.MT5921\18 2008-08-10 18:48:28 GMT mtk01461
**  Update for Driver Review
**  \main\maintrunk.MT5921\17 2008-08-04 16:52:01 GMT mtk01461
**  Add proc dbg print message of DOMAIN_INDEX level
**  \main\maintrunk.MT5921\16 2008-07-10 00:45:16 GMT mtk01461
**  Remove the check of MCR offset, we may use the MCR address which is not align to DW boundary or proprietary usage.
**  \main\maintrunk.MT5921\15 2008-06-03 20:49:44 GMT mtk01461
**  \main\maintrunk.MT5921\14 2008-06-02 22:56:00 GMT mtk01461
**  Rename some functions for linux proc
**  \main\maintrunk.MT5921\13 2008-06-02 20:23:18 GMT mtk01461
**  Revise PROC mcr read / write for supporting TELNET
**  \main\maintrunk.MT5921\12 2008-03-28 10:40:25 GMT mtk01461
**  Remove temporary set desired rate in linux proc
**  \main\maintrunk.MT5921\11 2008-01-07 15:07:29 GMT mtk01461
**  Add User Update Desired Rate Set for QA in Linux
**  \main\maintrunk.MT5921\10 2007-12-11 00:11:14 GMT mtk01461
**  Fix SPIN_LOCK protection
**  \main\maintrunk.MT5921\9 2007-12-04 18:07:57 GMT mtk01461
**  Add additional debug category to proc
**  \main\maintrunk.MT5921\8 2007-11-02 01:03:23 GMT mtk01461
**  Unify TX Path for Normal and IBSS Power Save + IBSS neighbor learning
**  \main\maintrunk.MT5921\7 2007-10-25 18:08:14 GMT mtk01461
**  Add VOIP SCAN Support  & Refine Roaming
** Revision 1.3  2007/07/05 07:25:33  MTK01461
** Add Linux initial code, modify doc, add 11BB, RF init code
**
** Revision 1.2  2007/06/27 02:18:51  MTK01461
** Update SCAN_FSM, Initial(Can Load Module), Proc(Can do Reg R/W), TX API
**
*/

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/

#include "precomp.h"

/* #include "wlan_lib.h" */
/* #include "debug.h" */

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

#define PROC_WLAN_THERMO                        "wlanThermo"
#define PROC_DRV_STATUS                         "status"
#define PROC_RX_STATISTICS                      "rx_statistics"
#define PROC_TX_STATISTICS                      "tx_statistics"
#define PROC_DBG_LEVEL                          "dbg_level"

#define PROC_MCR_ACCESS_MAX_USER_INPUT_LEN      20
#define PROC_RX_STATISTICS_MAX_USER_INPUT_LEN   10
#define PROC_TX_STATISTICS_MAX_USER_INPUT_LEN   10
#define PROC_DBG_LEVEL_MAX_USER_INPUT_LEN       20
#define PROC_DBG_LEVEL_MAX_DISPLAY_STR_LEN      30

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/
/* static UINT_32 u4McrOffset; */
#if CFG_SUPPORT_THERMO_THROTTLING
static P_GLUE_INFO_T g_prGlueInfo_proc;
#endif
/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/
/*----------------------------------------------------------------------------*/
/*!
* \brief The PROC function for reading MCR register to User Space, the offset of
*        the MCR is specified in u4McrOffset.
*
* \param[in] page       Buffer provided by kernel.
* \param[in out] start  Start Address to read(3 methods).
* \param[in] off        Offset.
* \param[in] count      Allowable number to read.
* \param[out] eof       End of File indication.
* \param[in] data       Pointer to the private data structure.
*
* \return number of characters print to the buffer from User Space.
*/
/*----------------------------------------------------------------------------*/
#if 0
static int procMCRRead(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	P_GLUE_INFO_T prGlueInfo;
	PARAM_CUSTOM_MCR_RW_STRUC_T rMcrInfo;
	UINT_32 u4BufLen;
	char *p = page;
	UINT_32 u4Count;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;

	ASSERT(data);

	/* Kevin: Apply PROC read method 1. */
	if (off != 0)
		return 0;	/* To indicate end of file. */

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv((struct net_device *)data));

	rMcrInfo.u4McrOffset = u4McrOffset;

	rStatus = kalIoctl(prGlueInfo,
			   wlanoidQueryMcrRead,
			   (PVOID) & rMcrInfo, sizeof(rMcrInfo), TRUE, TRUE, TRUE, FALSE, &u4BufLen);

	/* SPRINTF(p, ("MCR (0x%08lxh): 0x%08lx\n", */
	/* rMcrInfo.u4McrOffset, rMcrInfo.u4McrData)); */

	u4Count = (UINT_32) (p - page);

	*eof = 1;

	return (int)u4Count;

}				/* end of procMCRRead() */

/*----------------------------------------------------------------------------*/
/*!
* \brief The PROC function for writing MCR register to HW or update u4McrOffset
*        for reading MCR later.
*
* \param[in] file   pointer to file.
* \param[in] buffer Buffer from user space.
* \param[in] count  Number of characters to write
* \param[in] data   Pointer to the private data structure.
*
* \return number of characters write from User Space.
*/
/*----------------------------------------------------------------------------*/
static int procMCRWrite(struct file *file, const char *buffer, unsigned long count, void *data)
{
	P_GLUE_INFO_T prGlueInfo;
	char acBuf[PROC_MCR_ACCESS_MAX_USER_INPUT_LEN + 1];	/* + 1 for "\0" */
	int i4CopySize;
	PARAM_CUSTOM_MCR_RW_STRUC_T rMcrInfo;
	UINT_32 u4BufLen;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;

	ASSERT(data);

	i4CopySize = (count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
	if (copy_from_user(acBuf, buffer, i4CopySize))
		return 0;
	acBuf[i4CopySize] = '\0';

	switch (sscanf(acBuf, "0x%lx 0x%lx", &rMcrInfo.u4McrOffset, &rMcrInfo.u4McrData)) {
	case 2:
		/* NOTE: Sometimes we want to test if bus will still be ok, after accessing
		 * the MCR which is not align to DW boundary.
		 */
		/* if (IS_ALIGN_4(rMcrInfo.u4McrOffset)) */
		{
			prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv((struct net_device *)data));

			u4McrOffset = rMcrInfo.u4McrOffset;

			/* printk("Write 0x%lx to MCR 0x%04lx\n", */
			/* rMcrInfo.u4McrOffset, rMcrInfo.u4McrData); */

			rStatus = kalIoctl(prGlueInfo,
					   wlanoidSetMcrWrite,
					   (PVOID) & rMcrInfo, sizeof(rMcrInfo), FALSE, FALSE, TRUE, FALSE, &u4BufLen);

		}
		break;

	case 1:
		/* if (IS_ALIGN_4(rMcrInfo.u4McrOffset)) */
		{
			u4McrOffset = rMcrInfo.u4McrOffset;
		}
		break;

	default:
		break;
	}

	return count;

}				/* end of procMCRWrite() */
#endif

#if 0
/*----------------------------------------------------------------------------*/
/*!
* \brief The PROC function for reading Driver Status to User Space.
*
* \param[in] page       Buffer provided by kernel.
* \param[in out] start  Start Address to read(3 methods).
* \param[in] off        Offset.
* \param[in] count      Allowable number to read.
* \param[out] eof       End of File indication.
* \param[in] data       Pointer to the private data structure.
*
* \return number of characters print to the buffer from User Space.
*/
/*----------------------------------------------------------------------------*/
static int procDrvStatusRead(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	P_GLUE_INFO_T prGlueInfo = ((struct net_device *)data)->priv;
	char *p = page;
	UINT_32 u4Count;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(data);

	/* Kevin: Apply PROC read method 1. */
	if (off != 0)
		return 0;	/* To indicate end of file. */

	SPRINTF(p, ("GLUE LAYER STATUS:"));
	SPRINTF(p, ("\n=================="));

	SPRINTF(p, ("\n* Number of Pending Frames: %ld\n", prGlueInfo->u4TxPendingFrameNum));

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	wlanoidQueryDrvStatusForLinuxProc(prGlueInfo->prAdapter, p, &u4Count);

	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	u4Count += (UINT_32) (p - page);

	*eof = 1;

	return (int)u4Count;

}				/* end of procDrvStatusRead() */

/*----------------------------------------------------------------------------*/
/*!
* \brief The PROC function for reading Driver RX Statistic Counters to User Space.
*
* \param[in] page       Buffer provided by kernel.
* \param[in out] start  Start Address to read(3 methods).
* \param[in] off        Offset.
* \param[in] count      Allowable number to read.
* \param[out] eof       End of File indication.
* \param[in] data       Pointer to the private data structure.
*
* \return number of characters print to the buffer from User Space.
*/
/*----------------------------------------------------------------------------*/
static int procRxStatisticsRead(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	P_GLUE_INFO_T prGlueInfo = ((struct net_device *)data)->priv;
	char *p = page;
	UINT_32 u4Count;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(data);

	/* Kevin: Apply PROC read method 1. */
	if (off != 0)
		return 0;	/* To indicate end of file. */

	SPRINTF(p, ("RX STATISTICS (Write 1 to clear):"));
	SPRINTF(p, ("\n=================================\n"));

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	wlanoidQueryRxStatisticsForLinuxProc(prGlueInfo->prAdapter, p, &u4Count);

	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	u4Count += (UINT_32) (p - page);

	*eof = 1;

	return (int)u4Count;

}				/* end of procRxStatisticsRead() */

/*----------------------------------------------------------------------------*/
/*!
* \brief The PROC function for reset Driver RX Statistic Counters.
*
* \param[in] file   pointer to file.
* \param[in] buffer Buffer from user space.
* \param[in] count  Number of characters to write
* \param[in] data   Pointer to the private data structure.
*
* \return number of characters write from User Space.
*/
/*----------------------------------------------------------------------------*/
static int procRxStatisticsWrite(struct file *file, const char *buffer, unsigned long count, void *data)
{
	P_GLUE_INFO_T prGlueInfo = ((struct net_device *)data)->priv;
	char acBuf[PROC_RX_STATISTICS_MAX_USER_INPUT_LEN + 1];	/* + 1 for "\0" */
	UINT_32 u4CopySize;
	UINT_32 u4ClearCounter;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(data);

	u4CopySize = (count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
	copy_from_user(acBuf, buffer, u4CopySize);
	acBuf[u4CopySize] = '\0';

	if (sscanf(acBuf, "%ld", &u4ClearCounter) == 1) {
		if (u4ClearCounter == 1) {
			GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

			wlanoidSetRxStatisticsForLinuxProc(prGlueInfo->prAdapter);

			GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);
		}
	}

	return count;

}				/* end of procRxStatisticsWrite() */

/*----------------------------------------------------------------------------*/
/*!
* \brief The PROC function for reading Driver TX Statistic Counters to User Space.
*
* \param[in] page       Buffer provided by kernel.
* \param[in out] start  Start Address to read(3 methods).
* \param[in] off        Offset.
* \param[in] count      Allowable number to read.
* \param[out] eof       End of File indication.
* \param[in] data       Pointer to the private data structure.
*
* \return number of characters print to the buffer from User Space.
*/
/*----------------------------------------------------------------------------*/
static int procTxStatisticsRead(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	P_GLUE_INFO_T prGlueInfo = ((struct net_device *)data)->priv;
	char *p = page;
	UINT_32 u4Count;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(data);

	/* Kevin: Apply PROC read method 1. */
	if (off != 0)
		return 0;	/* To indicate end of file. */

	SPRINTF(p, ("TX STATISTICS (Write 1 to clear):"));
	SPRINTF(p, ("\n=================================\n"));

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	wlanoidQueryTxStatisticsForLinuxProc(prGlueInfo->prAdapter, p, &u4Count);

	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	u4Count += (UINT_32) (p - page);

	*eof = 1;

	return (int)u4Count;

}				/* end of procTxStatisticsRead() */

/*----------------------------------------------------------------------------*/
/*!
* \brief The PROC function for reset Driver TX Statistic Counters.
*
* \param[in] file   pointer to file.
* \param[in] buffer Buffer from user space.
* \param[in] count  Number of characters to write
* \param[in] data   Pointer to the private data structure.
*
* \return number of characters write from User Space.
*/
/*----------------------------------------------------------------------------*/
static int procTxStatisticsWrite(struct file *file, const char *buffer, unsigned long count, void *data)
{
	P_GLUE_INFO_T prGlueInfo = ((struct net_device *)data)->priv;
	char acBuf[PROC_RX_STATISTICS_MAX_USER_INPUT_LEN + 1];	/* + 1 for "\0" */
	UINT_32 u4CopySize;
	UINT_32 u4ClearCounter;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(data);

	u4CopySize = (count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
	copy_from_user(acBuf, buffer, u4CopySize);
	acBuf[u4CopySize] = '\0';

	if (sscanf(acBuf, "%ld", &u4ClearCounter) == 1) {
		if (u4ClearCounter == 1) {
			GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

			wlanoidSetTxStatisticsForLinuxProc(prGlueInfo->prAdapter);

			GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);
		}
	}

	return count;

}				/* end of procTxStatisticsWrite() */
#endif

#if DBG
static UINT_8 aucDbModuleName[][PROC_DBG_LEVEL_MAX_DISPLAY_STR_LEN] = {
	"DBG_INIT_IDX",
	"DBG_HAL_IDX",
	"DBG_INTR_IDX",
	"DBG_REQ_IDX",
	"DBG_TX_IDX",
	"DBG_RX_IDX",
	"DBG_RFTEST_IDX",
	"DBG_EMU_IDX",
	"DBG_SW1_IDX",
	"DBG_SW2_IDX",
	"DBG_SW3_IDX",
	"DBG_SW4_IDX",
	"DBG_HEM_IDX",
	"DBG_AIS_IDX",
	"DBG_RLM_IDX",
	"DBG_MEM_IDX",
	"DBG_CNM_IDX",
	"DBG_RSN_IDX",
	"DBG_BSS_IDX",
	"DBG_SCN_IDX",
	"DBG_SAA_IDX",
	"DBG_AAA_IDX",
	"DBG_P2P_IDX",
	"DBG_QM_IDX",
	"DBG_SEC_IDX",
	"DBG_BOW_IDX"
};

extern UINT_8 aucDebugModule[];

/*----------------------------------------------------------------------------*/
/*!
* \brief The PROC function for displaying current Debug Level.
*
* \param[in] page       Buffer provided by kernel.
* \param[in out] start  Start Address to read(3 methods).
* \param[in] off        Offset.
* \param[in] count      Allowable number to read.
* \param[out] eof       End of File indication.
* \param[in] data       Pointer to the private data structure.
*
* \return number of characters print to the buffer from User Space.
*/
/*----------------------------------------------------------------------------*/
static int procDbgLevelRead(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int i;

	/* Kevin: Apply PROC read method 1. */
	if (off != 0)
		return 0;	/* To indicate end of file. */

	for (i = 0; i < (sizeof(aucDbModuleName) / PROC_DBG_LEVEL_MAX_DISPLAY_STR_LEN); i++) {
		SPRINTF(p, ("%c %-15s(0x%02x): %02x\n",
			    ((i == u4DebugModule) ? '*' : ' '), &aucDbModuleName[i][0], i, aucDebugModule[i]));
	}

	*eof = 1;
	return (int)(p - page);
}

/*----------------------------------------------------------------------------*/
/*!
* \brief The PROC function for adjusting Debug Level to turn on/off debugging message.
*
* \param[in] file   pointer to file.
* \param[in] buffer Buffer from user space.
* \param[in] count  Number of characters to write
* \param[in] data   Pointer to the private data structure.
*
* \return number of characters write from User Space.
*/
/*----------------------------------------------------------------------------*/
static int procDbgLevelWrite(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char acBuf[PROC_DBG_LEVEL_MAX_USER_INPUT_LEN + 1];	/* + 1 for "\0" */
	UINT_32 u4CopySize;
	UINT_32 u4NewDbgModule, u4NewDbgLevel;

	u4CopySize = (count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
	copy_from_user(acBuf, buffer, u4CopySize);
	acBuf[u4CopySize] = '\0';

	if (sscanf(acBuf, "0x%lx 0x%lx", &u4NewDbgModule, &u4NewDbgLevel) == 2) {
		if (u4NewDbgModule < DBG_MODULE_NUM) {
			u4DebugModule = u4NewDbgModule;
			u4NewDbgLevel &= DBG_CLASS_MASK;
			aucDebugModule[u4DebugModule] = (UINT_8) u4NewDbgLevel;
		}
	}

	return count;
}
#endif /* DBG */

/*----------------------------------------------------------------------------*/
/*!
* \brief This function create a PROC fs in linux /proc/net subdirectory.
*
* \param[in] prDev      Pointer to the struct net_device.
* \param[in] pucDevName Pointer to the name of net_device.
*
* \return N/A
*/
/*----------------------------------------------------------------------------*/

#if CFG_SUPPORT_THERMO_THROTTLING

/**
 * This function is called then the /proc file is read
 *
 */
typedef struct _COEX_BUF1 {
	UINT8 buffer[128];
	INT32 availSize;
} COEX_BUF1, *P_COEX_BUF1;

COEX_BUF1 gCoexBuf1;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
static ssize_t procfile_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{

	INT32 retval = 0;
	INT32 i_ret = 0;
	CHAR *warn_msg = "no data available, please run echo 15 xx > /proc/driver/wmt_psm first\n";

	if (*f_pos > 0) {
		retval = 0;
	} else {
		/*len = sprintf(page, "%d\n", g_psm_enable); */
#if 1
		if (gCoexBuf1.availSize <= 0) {
			printk("no data available\n");
			retval = strlen(warn_msg) + 1;
			if (count < retval)
				retval = count;
			i_ret = copy_to_user(buf, warn_msg, retval);
			if (i_ret) {
				printk("copy to buffer failed, ret:%d\n", retval);
				retval = -EFAULT;
				goto err_exit;
			}
			*f_pos += retval;
		} else
#endif
		{
			INT32 i = 0;
			INT32 len = 0;
			CHAR msg_info[128];
			INT32 max_num = 0;
			/*we do not check page buffer, because there are only 100 bytes in g_coex_buf, no reason page
			buffer is not enough, a bomb is placed here on unexpected condition */

			printk("%d bytes avaliable\n", gCoexBuf1.availSize);
			max_num = ((sizeof(msg_info) > count ? sizeof(msg_info) : count) - 1) / 5;

			if (max_num > gCoexBuf1.availSize)
				max_num = gCoexBuf1.availSize;
			else
				printk("round to %d bytes due to local buffer size limitation\n", max_num);

			for (i = 0; i < max_num; i++)
				len += sprintf(msg_info + len, "%d", gCoexBuf1.buffer[i]);

			len += sprintf(msg_info + len, "\n");
			retval = len;

			i_ret = copy_to_user(buf, msg_info, retval);
			if (i_ret) {
				printk("copy to buffer failed, ret:%d\n", retval);
				retval = -EFAULT;
				goto err_exit;
			}
			*f_pos += retval;
		}
	}
	gCoexBuf1.availSize = 0;
err_exit:

	return retval;
}
#endif
#if 1
typedef INT32 (*WLAN_DEV_DBG_FUNC)();
static INT32 wlan_get_thermo_power();
static INT32 wlan_get_link_mode();

const static WLAN_DEV_DBG_FUNC wlan_dev_dbg_func[] = {
	[0] = wlan_get_thermo_power,
	[1] = wlan_get_link_mode,

};

INT32 wlan_get_thermo_power()
{
	P_ADAPTER_T prAdapter;
	prAdapter = g_prGlueInfo_proc->prAdapter;

	if (prAdapter->u4AirDelayTotal > 100)
		gCoexBuf1.buffer[0] = 100;
	else
		gCoexBuf1.buffer[0] = prAdapter->u4AirDelayTotal;
	gCoexBuf1.availSize = 1;
	DBGLOG(RLM, INFO, ("PROC %s thrmo_power(%d)\n", __func__, gCoexBuf1.buffer[0]));

	return 0;
}

INT32 wlan_get_link_mode()
{
	UINT_8 ucLinkMode = 0;
	P_ADAPTER_T prAdapter;
	BOOLEAN fgIsAPmode;
	prAdapter = g_prGlueInfo_proc->prAdapter;
	fgIsAPmode = p2pFuncIsAPMode(prAdapter->rWifiVar.prP2pFsmInfo);

	DBGLOG(RLM, INFO, ("PROC %s AIS(%d)P2P(%d)AP(%d)\n",
			   __func__,
			   prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_AIS_INDEX].eConnectionState,
			   prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_P2P_INDEX].eConnectionState, fgIsAPmode));

#if 1

	if (prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_AIS_INDEX].eConnectionState == PARAM_MEDIA_STATE_CONNECTED)
		ucLinkMode |= BIT(0);
	if (prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_P2P_INDEX].eConnectionState == PARAM_MEDIA_STATE_CONNECTED)
		ucLinkMode |= BIT(1);
	if (fgIsAPmode)
		ucLinkMode |= BIT(2);

#endif
	gCoexBuf1.buffer[0] = ucLinkMode;
	gCoexBuf1.availSize = 1;

	return 0;
}

static ssize_t procfile_write(struct file *filp, const char __user *buffer, size_t count, loff_t *f_pos)
{
	CHAR buf[256];
	CHAR *pBuf;
	ULONG len = count;
	INT32 x = 0, y = 0, z = 0;
	CHAR *pToken = NULL;
	CHAR *pDelimiter = " \t";

	if (copy_from_user(gCoexBuf1.buffer, buffer, count))
		return -EFAULT;
	/* gCoexBuf1.availSize = count; */

	/* return gCoexBuf1.availSize; */
#if 1
	printk("write parameter len = %d\n\r", (INT32) len);
	if (len >= sizeof(buf)) {
		printk("input handling fail!\n");
		len = sizeof(buf) - 1;
		return -1;
	}

	if (copy_from_user(buf, buffer, len))
		return -EFAULT;
	buf[len] = '\0';
	printk("write parameter data = %s\n\r", buf);

	pBuf = buf;
	pToken = strsep(&pBuf, pDelimiter);
	x = NULL != pToken ? simple_strtol(pToken, NULL, 16) : 0;
	printk("x(%d)\n", x);
#if 1

	pToken = strsep(&pBuf, "\t\n ");
	if (pToken != NULL) {
		y = simple_strtol(pToken, NULL, 16);
		printk("y = 0x%08x\n\r", y);
	} else {
		y = 3000;
		/*efuse, register read write default value */
		if (0x11 == x || 0x12 == x || 0x13 == x)
			y = 0x80000000;
	}

	pToken = strsep(&pBuf, "\t\n ");
	if (pToken != NULL) {
		z = simple_strtol(pToken, NULL, 16);
	} else {
		z = 10;
		/*efuse, register read write default value */
		if (0x11 == x || 0x12 == x || 0x13 == x)
			z = 0xffffffff;
	}

	printk(" x(0x%08x), y(0x%08x), z(0x%08x)\n\r", x, y, z);
#endif

	if (((sizeof(wlan_dev_dbg_func) / sizeof(wlan_dev_dbg_func[0])) > x) && NULL != wlan_dev_dbg_func[x])
		(*wlan_dev_dbg_func[x]) ();
	else
		printk("no handler defined for command id(0x%08x)\n\r", x);
#endif

	/* len = gCoexBuf1.availSize; */
	return len;
}
#endif

#endif

INT_32 procInitProcfs(struct net_device *prDev, char *pucDevName)
{

	P_GLUE_INFO_T prGlueInfo;
	struct proc_dir_entry *prEntry;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.read = procfile_read,
		.write = procfile_write,
	};
#endif

	ASSERT(prDev);
	DBGLOG(INIT, INFO, ("[%s]\n", __func__));

	if (init_net.proc_net == (struct proc_dir_entry *)NULL) {
		DBGLOG(INIT, INFO, ("init proc fs fail: proc_net == NULL\n"));
		return -ENOENT;
	}

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prDev));
#if CFG_SUPPORT_THERMO_THROTTLING
	g_prGlueInfo_proc = prGlueInfo;
#endif
	if (!prGlueInfo) {
		DBGLOG(INIT, WARN, ("The OS context is NULL\n"));
		return -ENOENT;
	}

	/*
	   /proc/net/wlan0
	   |-- mcr              (PROC_MCR_ACCESS)
	   |-- status           (PROC_DRV_STATUS)
	   |-- rx_statistics    (PROC_RX_STATISTICS)
	   |-- tx_statistics    (PROC_TX_STATISTICS)
	   |-- dbg_level        (PROC_DBG_LEVEL)
	   |-- (end)
	 */

	/*
	 * Directory: Root (/proc/net/wlan0)
	 */

	prGlueInfo->pProcRoot = proc_mkdir(pucDevName, init_net.proc_net);
	if (prGlueInfo->pProcRoot == NULL) {
		DBGLOG(INIT, WARN, ("prGlueInfo->pProcRoot == NULL\n"));
		return -ENOENT;
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
	prEntry = proc_create(PROC_WLAN_THERMO, 0664, init_net.proc_net, &proc_fops);
	if (prEntry == NULL) {
		printk("Unable to create /proc entry\n\r");
		return -1;
	}
#else
	prEntry = create_proc_entry(PROC_WLAN_THERMO, 0664, init_net.proc_net);
	if (prEntry == NULL) {
		printk("Unable to create /proc entry\n\r");
		return -1;
	}
	prEntry->read_proc = procfile_read;
	prEntry->write_proc = procfile_write;
#endif
	return 0;
}				/* end of procInitProcfs() */

/*----------------------------------------------------------------------------*/
/*!
* \brief This function clean up a PROC fs created by procInitProcfs().
*
* \param[in] prDev      Pointer to the struct net_device.
* \param[in] pucDevName Pointer to the name of net_device.
*
* \return N/A
*/
/*----------------------------------------------------------------------------*/
INT_32 procRemoveProcfs(struct net_device *prDev, char *pucDevName)
{
	P_GLUE_INFO_T prGlueInfo = NULL;

	DBGLOG(INIT, INFO, ("[%s]\n", __func__));

	ASSERT(prDev);

	if (!prDev)
		return -ENOENT;

	if (init_net.proc_net == (struct proc_dir_entry *)NULL) {
		DBGLOG(INIT, WARN, ("remove proc fs fail: proc_net == NULL\n"));
		return -ENOENT;
	}

	prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(prDev));
	if (!prGlueInfo->pProcRoot) {
		DBGLOG(INIT, WARN, ("The procfs root is NULL\n"));
		return -ENOENT;
	}
#if 0
#if DBG
	remove_proc_entry(PROC_DBG_LEVEL, prGlueInfo->pProcRoot);
#endif /* DBG */
	remove_proc_entry(PROC_TX_STATISTICS, prGlueInfo->pProcRoot);
	remove_proc_entry(PROC_RX_STATISTICS, prGlueInfo->pProcRoot);
	remove_proc_entry(PROC_DRV_STATUS, prGlueInfo->pProcRoot);
#endif
	remove_proc_entry(PROC_WLAN_THERMO, init_net.proc_net);

	/* remove root directory (proc/net/wlan0) */
	/* remove_proc_entry(pucDevName, init_net.proc_net); */
#if CFG_SUPPORT_THERMO_THROTTLING
	g_prGlueInfo_proc = NULL;
#endif

	return 0;

}				/* end of procRemoveProcfs() */
