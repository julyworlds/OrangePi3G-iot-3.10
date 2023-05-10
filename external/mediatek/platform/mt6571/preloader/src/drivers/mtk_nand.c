/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 */

#include "typedefs.h"
#include "platform.h"
#include "blkdev.h"
#include "cust_nand.h"
#include "mtk_nand.h"
#include "mtk_nand_core.h"
#include "bmt.h"
#include "part.h"
#include "partition_define.h"
#include "dram_buffer.h"

#if defined(MTK_COMBO_NAND_SUPPORT)
	// BMT_POOL_SIZE is not used anymore
#else
	#ifndef PART_SIZE_BMTPOOL
	#define BMT_POOL_SIZE (80)
	#else
	#define BMT_POOL_SIZE (PART_SIZE_BMTPOOL)
	#endif
#endif

#define PMT_POOL_SIZE (2)
/******************************************************************************
*
* Macro definition
*
*******************************************************************************/

#define NFI_SET_REG32(reg, value)   (DRV_WriteReg32(reg, DRV_Reg32(reg) | (value)))
#define NFI_SET_REG16(reg, value)   (DRV_WriteReg16(reg, DRV_Reg16(reg) | (value)))
#define NFI_CLN_REG32(reg, value)   (DRV_WriteReg32(reg, DRV_Reg32(reg) & (~(value))))
#define NFI_CLN_REG16(reg, value)   (DRV_WriteReg16(reg, DRV_Reg16(reg) & (~(value))))
#define RAND_TYPE_SAMSUNG 0
#define RAND_TYPE_TOSHIBA 1
#define RAND_TYPE_NONE 2

#define FIFO_PIO_READY(x)  (0x1 & x)
#define WAIT_NFI_PIO_READY(timeout) \
    do {\
    while( (!FIFO_PIO_READY(DRV_Reg(NFI_PIO_DIRDY_REG16))) && (--timeout) );\
    if(timeout == 0)\
   {\
   MSG(ERR, "Error: FIFO_PIO_READY timeout at line=%d, file =%s\n", __LINE__, __FILE__);\
   }\
    } while(0);

#define TIMEOUT_1   0x1fff
#define TIMEOUT_2   0x8ff
#define TIMEOUT_3   0xffff
#define TIMEOUT_4   5000        //PIO

#define STATUS_READY			(0x40)
#define STATUS_FAIL				(0x01)
#define STATUS_WR_ALLOW			(0x80)

#define NFI_ISSUE_COMMAND(cmd, col_addr, row_addr, col_num, row_num) \
    do { \
    DRV_WriteReg(NFI_CMD_REG16,cmd);\
    while (DRV_Reg32(NFI_STA_REG32) & STA_CMD_STATE);\
    DRV_WriteReg32(NFI_COLADDR_REG32, col_addr);\
    DRV_WriteReg32(NFI_ROWADDR_REG32, row_addr);\
    DRV_WriteReg(NFI_ADDRNOB_REG16, col_num | (row_num<<ADDR_ROW_NOB_SHIFT));\
    while (DRV_Reg32(NFI_STA_REG32) & STA_ADDR_STATE);\
    }while(0);

u32 PAGE_SIZE;
u32 BLOCK_SIZE;
enum flashdev_vendor gVendor;
extern u64 part_get_startaddress(u64 byte_address);
#define STORAGE_BUFFER_SIZE 0x10000
//extern u8 storage_buffer[STORAGE_BUFFER_SIZE];
#define storage_buffer g_dram_buf->storage_buffer
//u8 __DRAM__ nand_nfi_buf[NAND_NFI_BUFFER_SIZE];
#define nand_nfi_buf g_dram_buf->nand_nfi_buf

#define ERR_RTN_SUCCESS   1
#define ERR_RTN_FAIL      0
#define ERR_RTN_BCH_FAIL -1
u32 MICRON_TRANSFER(u32 pageNo);
u32 SANDISK_TRANSFER(u32 pageNo);
u32 HYNIX_TRANSFER(u32 pageNo);

u32 MICRON_feature_array[] = 
{0x00000000,0x7C7C007C,0x04040004,0x78780078,
0x08080008,0x00007C00,0x7C7C7C7C,0x04047C04,
0x78007C78,0x74007C74,0x00007800,0x7C00787C,
0x78007878,0x74007874,0x70007870,0x000C0400,
0x7C00047C,0x04100404,0x7C7C747C,0x78787478,
0x74787474,0x70787470,0x6C78746C,0x78787078,
0x74787074,0x70787070,0x6C78706C,0x74786C74,
0x60786C70,0x6C786C6C};

typedef u32 (*GetLowPageNumber)(u32 pageNo);

GetLowPageNumber functArray[]=
{
	MICRON_TRANSFER,
	HYNIX_TRANSFER,
	SANDISK_TRANSFER,
};

u32 SANDISK_TRANSFER(u32 pageNo)
{
	if(0 == pageNo)
	{
		return pageNo;
	}
	else
	{
		return pageNo+pageNo-1;
	}
}

u32 MICRON_TRANSFER(u32 pageNo)
{
	u32 temp;
	if(pageNo < 4)
		return pageNo;
	temp = (pageNo - 4) & 0xFFFFFFFE;
	if(pageNo<=130)
		return (pageNo+temp);
	else
		return (pageNo+temp-2);
}

u32 HYNIX_TRANSFER(u32 pageNo)
{
	u32 temp;
	if(pageNo < 4)
		return pageNo;
	temp = pageNo+(pageNo&0xFFFFFFFE)-2;
	return temp;
}


/**************************************************************************
*  MACRO LIKE FUNCTION
**************************************************************************/

static inline u32 PAGE_NUM(u64 logical_size)
{
    return ((unsigned long)(logical_size) / PAGE_SIZE);
}

inline u64 LOGICAL_ADDR(u32 page_addr)
{
    return ((unsigned long long)(page_addr) * PAGE_SIZE);
}

inline u64 BLOCK_ALIGN(u64 logical_addr)
{
    return (((u64) (logical_addr / BLOCK_SIZE)) * BLOCK_SIZE);
}

//---------------------------------------------------------------------------

//-------------------------------------------------------------------------
typedef U32(*STORGE_READ) (u8 * buf, u32 start, u32 img_size);

typedef struct
{
    u32 page_size;
    u32 pktsz;
} device_info_t;
//-------------------------------------------------------------------------

device_info_t gdevice_info;
boot_dev_t g_dev_vfunc;
static blkdev_t g_nand_bdev;
__attribute__((aligned(4))) unsigned char g_nand_spare[128];

unsigned int nand_maf_id;
unsigned int nand_dev_id;
uint8 ext_id1, ext_id2, ext_id3;

static u32 g_i4ErrNum;
static BOOL g_bInitDone;
BOOL g_bHwEcc = TRUE;
u32 PAGE_SIZE;
u32 BLOCK_SIZE;
__attribute__((section(".bss.uninit"))) u8 Bad_Block_Table[16384] = { 0 }; // address will be at EMI and non-ZI.

struct nand_chip g_nand_chip;
struct nand_ecclayout *nand_oob = NULL;
static bool use_randomizer = FALSE;

static struct nand_ecclayout nand_oob_16 = {
    .eccbytes = 8,
    .eccpos = {8, 9, 10, 11, 12, 13, 14, 15},
    .oobfree = {{1, 6}, {0, 0}}
};

struct nand_ecclayout nand_oob_64 = {
    .eccbytes = 32,
    .eccpos = {32, 33, 34, 35, 36, 37, 38, 39,
               40, 41, 42, 43, 44, 45, 46, 47,
               48, 49, 50, 51, 52, 53, 54, 55,
               56, 57, 58, 59, 60, 61, 62, 63},
    .oobfree = {{1, 7}, {9, 7}, {17, 7}, {25, 6}, {0, 0}}
};

struct nand_ecclayout nand_oob_128 = {
    .eccbytes = 64,
    .eccpos = {
               64, 65, 66, 67, 68, 69, 70, 71,
               72, 73, 74, 75, 76, 77, 78, 79,
               80, 81, 82, 83, 84, 85, 86, 86,
               88, 89, 90, 91, 92, 93, 94, 95,
               96, 97, 98, 99, 100, 101, 102, 103,
               104, 105, 106, 107, 108, 109, 110, 111,
               112, 113, 114, 115, 116, 117, 118, 119,
               120, 121, 122, 123, 124, 125, 126, 127},
    .oobfree = {{1, 7}, {9, 7}, {17, 7}, {25, 7}, {33, 7}, {41, 7}, {49, 7}, {57, 6}}
};

struct NAND_CMD
{
    u32 u4ColAddr;
    u32 u4RowAddr;
    u32 u4OOBRowAddr;
    u8 au1OOB[64];
    u8 *pDataBuf;
};

static struct NAND_CMD g_kCMD;
static flashdev_info devinfo;
static char *nfi_buf;
static bool mtk_nand_read_status(void);

bool get_device_info(u8*id, flashdev_info *devinfo);

struct nand_manufacturers nand_manuf_ids[] = {
    {NAND_MANFR_TOSHIBA, "Toshiba"},
    {NAND_MANFR_SAMSUNG, "Samsung"},
    {NAND_MANFR_FUJITSU, "Fujitsu"},
    {NAND_MANFR_NATIONAL, "National"},
    {NAND_MANFR_RENESAS, "Renesas"},
    {NAND_MANFR_STMICRO, "ST Micro"},
    {NAND_MANFR_HYNIX, "Hynix"},
    {NAND_MANFR_MICRON, "Micron"},
    {NAND_MANFR_AMD, "AMD"},
    {0x0, "Unknown"}
};

static inline unsigned int uffs(unsigned int x)
{
    unsigned int r = 1;

    if (!x)
        return 0;
    if (!(x & 0xffff))
    {
        x >>= 16;
        r += 16;
    }
    if (!(x & 0xff))
    {
        x >>= 8;
        r += 8;
    }
    if (!(x & 0xf))
    {
        x >>= 4;
        r += 4;
    }
    if (!(x & 3))
    {
        x >>= 2;
        r += 2;
    }
    if (!(x & 1))
    {
        x >>= 1;
        r += 1;
    }
    return r;
}

#define NAND_SECTOR_SIZE 512

u32 mtk_nand_page_transform(u64 logical_address, u32* blk, u32* map_blk)
{
	u64 start_address;
    u32 block;
    u32 page_in_block;
    u32 mapped_block;
	if(VEND_NONE != gVendor)
	{
		start_address = part_get_startaddress(logical_address);
		if(start_address == 0xFFFFFFFF)
			while(1);
		block = (start_address/BLOCK_SIZE) + (logical_address-start_address) / g_nand_chip.erasesize;
		page_in_block = PAGE_NUM(logical_address-start_address) % (1 << (g_nand_chip.phys_erase_shift-g_nand_chip.page_shift));

		if(devinfo.vendor != VEND_NONE)
		{
//			page_in_block = devinfo.feature_set.PairPage[page_in_block];
			page_in_block = functArray[devinfo.feature_set.ptbl_idx](page_in_block);
		}

	    mapped_block = get_mapping_block_index(block);
	}
	else
	{
		block = logical_address/BLOCK_SIZE;
		mapped_block = get_mapping_block_index(block);
		page_in_block = PAGE_NUM(logical_address) % (1 << (g_nand_chip.phys_erase_shift-g_nand_chip.page_shift));
	}
	*blk = block;
	*map_blk = mapped_block;
	return mapped_block*(BLOCK_SIZE/PAGE_SIZE)+page_in_block;
}

u16 randomizer_seed[128] = {
0x576A, 0x05E8, 0x629D, 0x45A3, 
0x649C, 0x4BF0, 0x2342, 0x272E, 
0x7358, 0x4FF3, 0x73EC, 0x5F70, 
0x7A60, 0x1AD8, 0x3472, 0x3612, 
0x224F, 0x0454, 0x030E, 0x70A5, 
0x7809, 0x2521, 0x48F4, 0x5A2D, 
0x492A, 0x043D, 0x7F61, 0x3969, 
0x517A, 0x3B42, 0x769D, 0x0647, 
0x7E2A, 0x1383, 0x49D9, 0x07B8, 
0x2578, 0x4EEC, 0x4423, 0x352F, 
0x5B22, 0x72B9, 0x367B, 0x24B6, 
0x7E8E, 0x2318, 0x6BD0, 0x5519, 
0x1783, 0x18A7, 0x7B6E, 0x7602, 
0x4B7F, 0x3648, 0x2C53, 0x6B99, 
0x0C23, 0x67CF, 0x7E0E, 0x4D8C, 
0x5079, 0x209D, 0x244A, 0x747B, 
0x350B, 0x0E4D, 0x7004, 0x6AC3, 
0x7F3E, 0x21F5, 0x7A15, 0x2379, 
0x1517, 0x1ABA, 0x4E77, 0x15A1, 
0x04FA, 0x2D61, 0x253A, 0x1302, 
0x1F63, 0x5AB3, 0x049A, 0x5AE8, 
0x1CD7, 0x4A00, 0x30C8, 0x3247, 
0x729C, 0x5034, 0x2B0E, 0x57F2, 
0x00E4, 0x575B, 0x6192, 0x38F8, 
0x2F6A, 0x0C14, 0x45FC, 0x41DF, 
0x38DA, 0x7AE1, 0x7322, 0x62DF, 
0x5E39, 0x0E64, 0x6D85, 0x5951,             
0x5937, 0x6281, 0x33A1, 0x6A32, 
0x3A5A, 0x2BAC, 0x743A, 0x5E74, 
0x3B2E, 0x7EC7, 0x4FD2, 0x5D28, 
0x751F, 0x3EF8, 0x39B1, 0x4E49, 
0x746B, 0x6EF6, 0x44BE, 0x6DB7};

static int mtk_nand_randomizer_config(struct gRandConfig *conf,kal_uint16 seed)
{
	if(gVendor == VEND_SANDISK || gVendor == VEND_TOSHIBA)
	{
	    kal_uint16   nfi_cnfg = 0;
		kal_uint32   nfi_ran_cnfg = 0;
		kal_uint8 i;

	    /* set up NFI_CNFG */
	    nfi_cnfg = DRV_Reg(NFI_CNFG_REG16);
		nfi_ran_cnfg = DRV_Reg32(NFI_RANDOM_CNFG_REG32);
		if(conf->type == RAND_TYPE_SAMSUNG)
		{
			nfi_ran_cnfg = 0;
			nfi_ran_cnfg |= seed << EN_SEED_SHIFT;
			nfi_ran_cnfg |= seed << DE_SEED_SHIFT;
			nfi_cnfg |= CNFG_RAN_SEC;
			nfi_cnfg |= CNFG_RAN_SEL;
			use_randomizer = TRUE;
			//nfi_ran_cnfg |= 0x00010001;
		}
		else if(conf->type == RAND_TYPE_TOSHIBA)
		{
			use_randomizer = TRUE;
			for(i = 0 ; i < 6 ; i++)
			{	
				DRV_WriteReg32(NFI_RANDOM_ENSEED01_TS_REG32+i, conf->seed[i]);
				DRV_WriteReg32(NFI_RANDOM_DESEED01_TS_REG32+i, conf->seed[i]);
			}
			nfi_cnfg |= CNFG_RAN_SEC;
			nfi_cnfg &= ~CNFG_RAN_SEL;
			//nfi_ran_cnfg |= 0x00010001;
		}
		else
		{
			nfi_ran_cnfg &= ~0x00010001;
			use_randomizer = FALSE;
			return 0;
		}
	    
	    DRV_WriteReg(NFI_CNFG_REG16, nfi_cnfg);
		DRV_WriteReg32(NFI_RANDOM_CNFG_REG32, nfi_ran_cnfg);
	}
	return 0;
}	
static bool mtk_nand_israndomizeron()
{
	if(gVendor == VEND_SANDISK || gVendor == VEND_TOSHIBA)
	{
		kal_uint32   nfi_ran_cnfg = 0;
		nfi_ran_cnfg = DRV_Reg32(NFI_RANDOM_CNFG_REG32);
		if(nfi_ran_cnfg&0x00010001)
			return TRUE;
	}
	return FALSE;
}
static void mtk_nand_turn_on_randomizer(u32 page)
{
	//struct gRandConfig *conf = &devinfo.feature_set.randConfig;
	if(gVendor == VEND_SANDISK || gVendor == VEND_TOSHIBA)
	{
		kal_uint32 page_per_blk = (BLOCK_SIZE/g_nand_chip.page_size);
		kal_uint32   nfi_ran_cnfg = 0;
		kal_uint16 seed;
		if(page_per_blk ==256)
		{
			seed = randomizer_seed[page%128];
		}
		else
		{
			seed = randomizer_seed[page%page_per_blk];
		}
		//MSG(INIT,"page %d, %d seed 0x%x",page,page%128,seed);
		mtk_nand_randomizer_config(&devinfo.feature_set.randConfig,seed);
		nfi_ran_cnfg = DRV_Reg32(NFI_RANDOM_CNFG_REG32);
		nfi_ran_cnfg |= 0x00010001;
		DRV_WriteReg32(NFI_RANDOM_CNFG_REG32, nfi_ran_cnfg);
	}
}
static void mtk_nand_turn_off_randomizer()
{
	if(gVendor == VEND_SANDISK || gVendor == VEND_TOSHIBA)
	{
		kal_uint32   nfi_ran_cnfg = 0;
		nfi_ran_cnfg = DRV_Reg32(NFI_RANDOM_CNFG_REG32);
		nfi_ran_cnfg &= ~0x00010001;
		DRV_WriteReg32(NFI_RANDOM_CNFG_REG32, nfi_ran_cnfg);
	}
}


/**************************************************************************
*  reset descriptor
**************************************************************************/
void mtk_nand_reset_descriptor(void)
{

    g_nand_chip.page_shift = 0;
    g_nand_chip.page_size = 0;
    g_nand_chip.ChipID = 0;     /* Type of DiskOnChip */
    g_nand_chip.chips_name = 0;
    g_nand_chip.chipsize = 0;
    g_nand_chip.erasesize = 0;
    g_nand_chip.mfr = 0;        /* Flash IDs - only one type of flash per device */
    g_nand_chip.id = 0;
    g_nand_chip.name = 0;
    g_nand_chip.numchips = 0;
    g_nand_chip.oobblock = 0;   /* Size of OOB blocks (e.g. 512) */
    g_nand_chip.oobsize = 0;    /* Amount of OOB data per block (e.g. 16) */
    g_nand_chip.eccsize = 0;
    g_nand_chip.bus16 = 0;
    g_nand_chip.nand_ecc_mode = 0;

}

bool get_device_info(u8*id, flashdev_info *devinfo)
{
    u32 i,m,n,mismatch;
    int target=-1,target_id_len=-1;

    for (i = 0; i<CHIP_CNT; i++){
		mismatch=0;
		for(m=0;m<gen_FlashTable[i].id_length;m++){
			if(id[m]!=gen_FlashTable[i].id[m]){
				mismatch=1;
				break;
			}
		}
		if(mismatch == 0 && gen_FlashTable[i].id_length > target_id_len){
				target=i;
				target_id_len=gen_FlashTable[i].id_length;
		}
    }

    if(target != -1){
		MSG(INIT, "Recognize NAND: ID [");
		for(n=0;n<gen_FlashTable[target].id_length;n++){
			devinfo->id[n] = gen_FlashTable[target].id[n];
			MSG(INIT, "%x ",devinfo->id[n]);
		}
		MSG(INIT, "], Device Name [%s], Page Size [%d]B Spare Size [%d]B Total Size [%d]MB\n",gen_FlashTable[target].devciename,gen_FlashTable[target].pagesize,gen_FlashTable[target].sparesize,gen_FlashTable[target].totalsize);
		devinfo->id_length=gen_FlashTable[i].id_length;
		devinfo->blocksize = gen_FlashTable[target].blocksize;
		devinfo->addr_cycle = gen_FlashTable[target].addr_cycle;
		devinfo->iowidth = gen_FlashTable[target].iowidth;
		devinfo->timmingsetting = gen_FlashTable[target].timmingsetting;
		devinfo->advancedmode = gen_FlashTable[target].advancedmode;
		devinfo->pagesize = gen_FlashTable[target].pagesize;
		devinfo->sparesize = gen_FlashTable[target].sparesize;
		devinfo->totalsize = gen_FlashTable[target].totalsize;
		devinfo->sectorsize = gen_FlashTable[target].sectorsize;
		devinfo->s_acccon= gen_FlashTable[target].s_acccon;
		devinfo->s_acccon1= gen_FlashTable[target].s_acccon1;
		devinfo->freq= gen_FlashTable[target].freq;
		devinfo->vendor = gen_FlashTable[target].vendor;
		gVendor = gen_FlashTable[target].vendor;
		memcpy((u8*)&devinfo->feature_set, (u8*)&gen_FlashTable[target].feature_set, sizeof(struct MLC_feature_set));
		memcpy(devinfo->devciename, gen_FlashTable[target].devciename, sizeof(devinfo->devciename));
    	return true;
	}else{
	    MSG(INIT, "Not Found NAND: ID [");
		for(n=0;n<NAND_MAX_ID;n++){
			MSG(INIT, "%x ",id[n]);
		}
		MSG(INIT, "]\n");
        return false;
	}
}

//---------------------------------------------------------------------------
static bool mtk_nand_check_RW_count(u16 u2WriteSize)
{
    u32 timeout = 0xFFFF;
    u16 u2SecNum = u2WriteSize >> g_nand_chip.sector_shift;
    while (ADDRCNTR_CNTR(DRV_Reg32(NFI_ADDRCNTR_REG16)) < u2SecNum)
    {
        timeout--;
        if (0 == timeout)
        {
            return FALSE;
        }
    }
    return TRUE;
}

//---------------------------------------------------------------------------
static bool mtk_nand_status_ready(u32 u4Status)
{
    u32 timeout = 0xFFFF;
    while ((DRV_Reg32(NFI_STA_REG32) & u4Status) != 0)
    {
        timeout--;
        if (0 == timeout)
        {
            return FALSE;
        }
    }
    return TRUE;
}

//---------------------------------------------------------------------------
static void mtk_nand_set_mode(u16 u2OpMode)
{
    u16 u2Mode = DRV_Reg16(NFI_CNFG_REG16);
    u2Mode &= ~CNFG_OP_MODE_MASK;
    u2Mode |= u2OpMode;
    DRV_WriteReg16(NFI_CNFG_REG16, u2Mode);
}

//---------------------------------------------------------------------------
static bool mtk_nand_set_command(u16 command)
{
    /* Write command to device */
    DRV_WriteReg16(NFI_CMD_REG16, command);
    return mtk_nand_status_ready(STA_CMD_STATE);
}

//---------------------------------------------------------------------------
static bool mtk_nand_set_address(u32 u4ColAddr, u32 u4RowAddr, u16 u2ColNOB, u16 u2RowNOB)
{
    /* fill cycle addr */
    DRV_WriteReg32(NFI_COLADDR_REG32, u4ColAddr);
    DRV_WriteReg32(NFI_ROWADDR_REG32, u4RowAddr);
    DRV_WriteReg16(NFI_ADDRNOB_REG16, u2ColNOB | (u2RowNOB << ADDR_ROW_NOB_SHIFT));
    return mtk_nand_status_ready(STA_ADDR_STATE);
}

//---------------------------------------------------------------------------
static void ECC_Decode_Start(void)
{
    /* wait for device returning idle */
    while (!(DRV_Reg16(ECC_DECIDLE_REG16) & DEC_IDLE)) ;
    DRV_WriteReg16(ECC_DECCON_REG16, DEC_EN);
}

//---------------------------------------------------------------------------
static void ECC_Decode_End(void)
{
    /* wait for device returning idle */
    while (!(DRV_Reg16(ECC_DECIDLE_REG16) & DEC_IDLE)) ;
    DRV_WriteReg16(ECC_DECCON_REG16, DEC_DE);
}

//---------------------------------------------------------------------------
static void ECC_Encode_Start(void)
{
    /* wait for device returning idle */
    while (!(DRV_Reg32(ECC_ENCIDLE_REG32) & ENC_IDLE)) ;
    DRV_WriteReg16(ECC_ENCCON_REG16, ENC_EN);
}

//---------------------------------------------------------------------------
static void ECC_Encode_End(void)
{
    /* wait for device returning idle */
    while (!(DRV_Reg32(ECC_ENCIDLE_REG32) & ENC_IDLE)) ;
    DRV_WriteReg16(ECC_ENCCON_REG16, ENC_DE);
}

//---------------------------------------------------------------------------
static void ECC_Config(u32 ecc_bit)
{
    u32 u4ENCODESize;
    u32 u4DECODESize;

    u32 ecc_bit_cfg = ECC_CNFG_ECC4;

    switch (ecc_bit)
    {
  	case 4:
  		ecc_bit_cfg = ECC_CNFG_ECC4;
  		break;
  	case 8:
  		ecc_bit_cfg = ECC_CNFG_ECC8;
  		break;
  	case 10:
  		ecc_bit_cfg = ECC_CNFG_ECC10;
  		break;
  	case 12:
  		ecc_bit_cfg = ECC_CNFG_ECC12;
  		break;
	case 14:
  		ecc_bit_cfg = ECC_CNFG_ECC14;
  		break;
	case 16:
  		ecc_bit_cfg = ECC_CNFG_ECC16;
  		break;
	case 18:
  		ecc_bit_cfg = ECC_CNFG_ECC18;
  		break;
	case 20:
  		ecc_bit_cfg = ECC_CNFG_ECC20;
  		break;
	case 22:
  		ecc_bit_cfg = ECC_CNFG_ECC22;
  		break;
	case 24:
  		ecc_bit_cfg = ECC_CNFG_ECC24;
  		break;
	case 28:
  		ecc_bit_cfg = ECC_CNFG_ECC28;
  		break;
	case 32:
  		ecc_bit_cfg = ECC_CNFG_ECC32;
  		break;
	case 36:
  		ecc_bit_cfg = ECC_CNFG_ECC36;
  		break;
	case 40:
  		ecc_bit_cfg = ECC_CNFG_ECC40;
  		break;
	case 44:
  		ecc_bit_cfg = ECC_CNFG_ECC44;
  		break;
	case 48:
  		ecc_bit_cfg = ECC_CNFG_ECC48;
  		break;
	case 52:
  		ecc_bit_cfg = ECC_CNFG_ECC52;
  		break;
	case 56:
  		ecc_bit_cfg = ECC_CNFG_ECC56;
  		break;
	case 60:
  		ecc_bit_cfg = ECC_CNFG_ECC60;
  		break;
    default:
  		break;

    }

    DRV_WriteReg16(ECC_DECCON_REG16, DEC_DE);
    do
    {;
    }
    while (!DRV_Reg16(ECC_DECIDLE_REG16));

    DRV_WriteReg16(ECC_ENCCON_REG16, ENC_DE);
    do
    {;
    }
    while (!DRV_Reg32(ECC_ENCIDLE_REG32));

    /* setup FDM register base */
//    DRV_WriteReg32(ECC_FDMADDR_REG32, NFI_FDM0L_REG32);

    u4ENCODESize = (g_nand_chip.sector_size + 8) << 3;
    u4DECODESize = ((g_nand_chip.sector_size + 8) << 3) + ecc_bit * ECC_PARITY_BIT;

    /* configure ECC decoder && encoder */
    DRV_WriteReg32(ECC_DECCNFG_REG32, ecc_bit_cfg | DEC_CNFG_NFI | DEC_CNFG_EMPTY_EN | (u4DECODESize << DEC_CNFG_CODE_SHIFT));

    DRV_WriteReg32(ECC_ENCCNFG_REG32, ecc_bit_cfg | ENC_CNFG_NFI | (u4ENCODESize << ENC_CNFG_MSG_SHIFT));

#ifndef MANUAL_CORRECT
    NFI_SET_REG32(ECC_DECCNFG_REG32, DEC_CNFG_CORRECT);
#else
    NFI_SET_REG32(ECC_DECCNFG_REG32, DEC_CNFG_EL);
#endif

}

/******************************************************************************
* mtk_nand_check_bch_error
*
* DESCRIPTION:
*   Check BCH error or not !
*
* PARAMETERS:
*   struct mtd_info *mtd
*    u8* pDataBuf
*    u32 u4SecIndex
*    u32 u4PageAddr
*
* RETURNS:
*   None
*
* NOTES:
*   None
*
******************************************************************************/
static bool mtk_nand_check_bch_error(u8 * pDataBuf, u32 u4SecIndex, u32 u4PageAddr)
{
    bool bRet = TRUE;
    u16 u2SectorDoneMask = 1 << u4SecIndex;
    u32 u4ErrorNumDebug0, u4ErrorNumDebug1, i, u4ErrNum;
    u32 timeout = 0xFFFF;

#ifdef MANUAL_CORRECT
    u32 au4ErrBitLoc[6];
    u32 u4ErrByteLoc, u4BitOffset;
    u32 u4ErrBitLoc1th, u4ErrBitLoc2nd;
#endif

    while (0 == (u2SectorDoneMask & DRV_Reg16(ECC_DECDONE_REG16)))
    {
        timeout--;
        if (0 == timeout)
        {
            return FALSE;
        }
    }
#ifndef MANUAL_CORRECT
    if(0 == (DRV_Reg32(NFI_STA_REG32) & STA_READ_EMPTY))
    {
        u4ErrorNumDebug0 = DRV_Reg32(ECC_DECENUM0_REG32);
        u4ErrorNumDebug1 = DRV_Reg32(ECC_DECENUM1_REG32);
	if (0 != (u4ErrorNumDebug0 & 0xFFFFFFFF) || 0 != (u4ErrorNumDebug1 & 0xFFFFFFFF))
        {
            for (i = 0; i <= u4SecIndex; ++i)
            {
#if 1
					u4ErrNum = (DRV_Reg32((ECC_DECENUM0_REG32+(i/4)))>>((i%4)*8))& ERR_NUM0;
#else
					if (i < 4)
					{
						u4ErrNum = DRV_Reg32(ECC_DECENUM0_REG32) >> (i * 8);
					} else
					{
						u4ErrNum = DRV_Reg32(ECC_DECENUM1_REG32) >> ((i - 4) * 8);
					}
					u4ErrNum &= ERR_NUM0;	
#endif

	        if (ERR_NUM0 == u4ErrNum)
                {
                    MSG(ERR, "In Preloader UnCorrectable at PageAddr=%d, Sector=%d\n", u4PageAddr, i);
                    bRet = false;
                } else
                {
		    if (u4ErrNum)
                    {
		        MSG(ERR, " In Preloader Correct %d at PageAddr=%d, Sector=%d\n", u4ErrNum, u4PageAddr, i);
		    }
                }
            }
        }
    }
#else
/* We will manually correct the error bits in the last sector, not all the sectors of the page!*/
    //memset(au4ErrBitLoc, 0x0, sizeof(au4ErrBitLoc));
    u4ErrorNumDebug = DRV_Reg32(ECC_DECENUM_REG32);
	u4ErrNum = (DRV_Reg32((ECC_DECENUM_REG32+(u4SecIndex/4)))>>((u4SecIndex%4)*8))& ERR_NUM0;

    if (u4ErrNum)
    {
        if (ERR_NUM0 == u4ErrNum)
        {
            //mtd->ecc_stats.failed++;
            bRet = FALSE;
        } else
        {
            for (i = 0; i < ((u4ErrNum + 1) >> 1); ++i)
            {
                au4ErrBitLoc[i] = DRV_Reg32(ECC_DECEL0_REG32 + i);
                u4ErrBitLoc1th = au4ErrBitLoc[i] & 0x3FFF;
                if (u4ErrBitLoc1th < 0x1000)
                {
                    u4ErrByteLoc = u4ErrBitLoc1th / 8;
                    u4BitOffset = u4ErrBitLoc1th % 8;
                    pDataBuf[u4ErrByteLoc] = pDataBuf[u4ErrByteLoc] ^ (1 << u4BitOffset);
                    //mtd->ecc_stats.corrected++;
                } else
                {
                    //mtd->ecc_stats.failed++;
                    MSG(INIT, "UnCorrectable ErrLoc=%d\n", au4ErrBitLoc[i]);
                }
                u4ErrBitLoc2nd = (au4ErrBitLoc[i] >> 16) & 0x3FFF;
                if (0 != u4ErrBitLoc2nd)
                {
                    if (u4ErrBitLoc2nd < 0x1000)
                    {
                        u4ErrByteLoc = u4ErrBitLoc2nd / 8;
                        u4BitOffset = u4ErrBitLoc2nd % 8;
                        pDataBuf[u4ErrByteLoc] = pDataBuf[u4ErrByteLoc] ^ (1 << u4BitOffset);
                        //mtd->ecc_stats.corrected++;
                    } else
                    {
                        //mtd->ecc_stats.failed++;
                        MSG(INIT, "UnCorrectable High ErrLoc=%d\n", au4ErrBitLoc[i]);
                    }
                }
            }
        }
        if (0 == (DRV_Reg16(ECC_DECFER_REG16) & (1 << u4SecIndex)))
        {
            bRet = FALSE;
        }
    }
#endif
    return bRet;
}

//---------------------------------------------------------------------------
static bool mtk_nand_RFIFOValidSize(u16 u2Size)
{
    u32 timeout = 0xFFFF;
    while (FIFO_RD_REMAIN(DRV_Reg16(NFI_FIFOSTA_REG16)) < u2Size)
    {
        timeout--;
        if (0 == timeout)
        {
            return FALSE;
        }
    }
    if (u2Size == 0)
    {
        while (FIFO_RD_REMAIN(DRV_Reg16(NFI_FIFOSTA_REG16)))
        {
            timeout--;
            if (0 == timeout)
            {
                return FALSE;
            }
        }
    }
    return TRUE;
}

//---------------------------------------------------------------------------
static bool mtk_nand_WFIFOValidSize(u16 u2Size)
{
    u32 timeout = 0xFFFF;
    while (FIFO_WR_REMAIN(DRV_Reg16(NFI_FIFOSTA_REG16)) > u2Size)
    {
        timeout--;
        if (0 == timeout)
        {
            return FALSE;
        }
    }
    if (u2Size == 0)
    {
        while (FIFO_WR_REMAIN(DRV_Reg16(NFI_FIFOSTA_REG16)))
        {
            timeout--;
            if (0 == timeout)
            {
                return FALSE;
            }
        }
    }
    return TRUE;
}

//---------------------------------------------------------------------------
bool mtk_nand_reset(void)
{
    int timeout = 0xFFFF;
    if (DRV_Reg16(NFI_MASTERSTA_REG16) & 0xFFF) // master is busy
    {
        DRV_WriteReg32(NFI_CON_REG16, CON_FIFO_FLUSH | CON_NFI_RST);
        while (DRV_Reg16(NFI_MASTERSTA_REG16) & 0xFFF)
        {
            timeout--;
            if (!timeout)
            {
                MSG(INIT, "MASTERSTA timeout\n");
            }
        }
    }
    /* issue reset operation */
    DRV_WriteReg32(NFI_CON_REG16, CON_FIFO_FLUSH | CON_NFI_RST);

    return mtk_nand_status_ready(STA_NFI_FSM_MASK | STA_NAND_BUSY) && mtk_nand_RFIFOValidSize(0) && mtk_nand_WFIFOValidSize(0);
}

//---------------------------------------------------------------------------
bool mtk_nand_SetFeature(u16 cmd, u32 addr, u8 *value,  u8 bytes)
{
	kal_uint16           reg_val     	 = 0;
	kal_uint8            write_count     = 0;
	kal_uint32           timeout=TIMEOUT_3;//0xffff;

	mtk_nand_reset();

	reg_val |= (CNFG_OP_CUST | CNFG_BYTE_RW);
	DRV_WriteReg(NFI_CNFG_REG16, reg_val);

	mtk_nand_set_command(cmd);
	mtk_nand_set_address(addr, 0, 1, 0);
	//NFI_ISSUE_COMMAND(cmd, addr, 0, 1, 0)

	//SAL_NFI_Config_Sector_Number(1);
	DRV_WriteReg32(NFI_CON_REG16, 1 << CON_NFI_SEC_SHIFT);
	NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BWR);
	DRV_WriteReg(NFI_STRDATA_REG16, 0x1);
	//SAL_NFI_Start_Data_Transfer(KAL_FALSE, KAL_TRUE);
	while ( (write_count < bytes) && timeout )
    {
    	WAIT_NFI_PIO_READY(timeout)
        if(timeout == 0)
        {
            break;
        }
        DRV_WriteReg32(NFI_DATAW_REG32, *value++);
        write_count++;
        timeout = TIMEOUT_3;
    }
	while ( (DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY) && (timeout) ){timeout--;}
	mtk_nand_read_status();
	
	if(timeout != 0)
		return TRUE;
	else
		return FALSE;
}

bool mtk_nand_GetFeature(u16 cmd, u32 addr, u8 *value,  u8 bytes)
{
	kal_uint16           reg_val     	 = 0;
	kal_uint8            read_count     = 0;
	kal_uint32           timeout=TIMEOUT_3;//0xffff;

	mtk_nand_reset();

	reg_val |= (CNFG_OP_CUST | CNFG_BYTE_RW | CNFG_READ_EN);
	DRV_WriteReg(NFI_CNFG_REG16, reg_val);

	mtk_nand_set_command(cmd);
	mtk_nand_set_address(addr, 0, 1, 0);

	//SAL_NFI_Config_Sector_Number(0);
	DRV_WriteReg32(NFI_CON_REG16, 0 << CON_NFI_SEC_SHIFT);
	reg_val = DRV_Reg32(NFI_CON_REG16);
    reg_val &= ~CON_NOB_MASK;
    reg_val |= ((4 << CON_NOB_SHIFT)|CON_NFI_SRD);
    DRV_WriteReg32(NFI_CON_REG16, reg_val);
	//NFI_SET_REG16(NFI_CON_REG16, CON_NFI_BWR);
	DRV_WriteReg(NFI_STRDATA_REG16, 0x1);
//	SAL_NFI_Start_Data_Transfer(KAL_TRUE, KAL_TRUE);
	while ( (read_count < bytes) && timeout )
    {
    	WAIT_NFI_PIO_READY(timeout)
        if(timeout == 0)
        {
            break;
        }
		*value++ = DRV_Reg32(NFI_DATAR_REG32);
        read_count++;
        timeout = TIMEOUT_3;
    }
	mtk_nand_read_status();
	if(timeout != 0)
		return TRUE;
	else
		return FALSE;

}

static void mtk_nand_feature_rrtry(flashdev_info deviceinfo, u32 feature, bool defValue)
{
	//u32 feature = deviceinfo.feature_set.FeatureSet.readRetryStart+retryCount;
	mtk_nand_SetFeature(deviceinfo.feature_set.FeatureSet.sfeatureCmd,\
								deviceinfo.feature_set.FeatureSet.readRetryAddress,\
								(u8*)&feature,4);
}

static void mtk_nand_precmd_rrtry(flashdev_info deviceinfo, u32 feature, bool defValue)
{
	//u32 feature = deviceinfo.feature_set.FeatureSet.readRetryStart+retryCount;
	if(FALSE == defValue)
		mtk_nand_reset();
	mtk_nand_SetFeature(deviceinfo.feature_set.FeatureSet.sfeatureCmd,\
								deviceinfo.feature_set.FeatureSet.readRetryAddress,\
								(u8*)&feature,4);
	if(FALSE == defValue)
		mtk_nand_set_command(deviceinfo.feature_set.FeatureSet.readRetryPreCmd);
	else
	{
		mtk_nand_set_command(0x26);
		mtk_nand_set_command(0x5D);
	}
}

struct toshiba_rrtry_setting{
	u32 setting[5];
};

u8 toshiba_rrtry_addr[5] = {0x4,0x5,0x6,0x7,0xD};

struct toshiba_rrtry_setting tr_setting[8] = 
{
{0x0,0x0,0x0,0x0,0x0},
{0x04,0x04,0x7C,0x7E,0x0},
{0x00,0x7C,0x78,0x78,0x0},
{0x7C,0x76,0x74,0x72,0x0},
{0x08,0x08,0x0,0x0,0x0},
{0x0B,0x7E,0x76,0x74,0x0},
{0x10,0x76,0x72,0x70,0x0},
{0x02,0x7C,0x7E,0x70,0x0}
};

static void mtk_nand_parapage_rrtry(flashdev_info deviceinfo, u32 retryCount, bool defValue)
{
	u32 i = 0;
	mtk_nand_set_command(0x5C);
	mtk_nand_set_command(0xC5);
	for(i=0;i<5;i++)
	{
		mtk_nand_SetFeature(deviceinfo.feature_set.FeatureSet.sfeatureCmd,\
								toshiba_rrtry_addr[i],\
								(u8*)&tr_setting[retryCount].setting[i],1);
	}
	if(TRUE == defValue)
		mtk_nand_set_command(0xFF);
}

u32 sandisk_rrtry_setting[32]= 
{0x00000000,0x7C00007C,0x787C0004,0x74780078,
0x7C007C08,0x787C7C00,0x74787C7C,0x70747C00,
0x7C007800,0x787C7800,0x74787800,0x70747800,
0x6C707800,0x00040400,0x7C000400,0x787C040C,
0x7478040C,0x7C000810,0x00040810,0x74787478,
0x70747478,0x6C707478,0x686C7478,0x74787078,
0x70747078,0x686C7078,0x6C707078,0x6C706C78,
0x686C6C78,0x64686C78,0x686C6874,0x64686874};

static u32 mtk_nand_rrtry_setting(flashdev_info deviceinfo, enum readRetryType type, u32 retryStart, u32 loopNo)
{
	u32 value = 0;
	if(gVendor != VEND_NONE)
	{
		if(RTYPE_FEATURE == type || RTYPE_PRECMD== type ||RTYPE_PARAPAGE== type)
		{
			if(retryStart != 0xFFFFFFFF)
			{
				value = retryStart+loopNo;
			}
			else
			{
				value = sandisk_rrtry_setting[loopNo];
			}
		}
	}
	
	return value;
}

typedef u32 (*rrtryFunctionType)(flashdev_info deviceinfo, u32 feature, bool defValue);

static rrtryFunctionType rtyFuncArray[]=
{
	mtk_nand_feature_rrtry,
	mtk_nand_precmd_rrtry,
	mtk_nand_parapage_rrtry,
};


static void mtk_nand_rrtry_func(flashdev_info deviceinfo, u32 feature, bool defValue)
{
	if(gVendor != VEND_NONE)
	{
		rtyFuncArray[deviceinfo.feature_set.FeatureSet.rtype](deviceinfo, feature,defValue);
	}
}

static bool mtk_nand_read_status(void)
{
    int status, i;
    mtk_nand_reset();
    unsigned int timeout;

    mtk_nand_reset();

    /* Disable HW ECC */
    NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);

    /* Disable 16-bit I/O */
    NFI_CLN_REG16(NFI_PAGEFMT_REG16, PAGEFMT_DBYTE_EN);
    NFI_SET_REG16(NFI_CNFG_REG16, CNFG_OP_SRD | CNFG_READ_EN | CNFG_BYTE_RW);

    DRV_WriteReg32(NFI_CON_REG16, CON_NFI_SRD | (1 << CON_NOB_SHIFT));

    DRV_WriteReg32(NFI_CON_REG16, 0x3);
    mtk_nand_set_mode(CNFG_OP_SRD);
    DRV_WriteReg16(NFI_CNFG_REG16, 0x2042);
    mtk_nand_set_command(NAND_CMD_STATUS);
    DRV_WriteReg32(NFI_CON_REG16, 0x90);

    timeout = TIMEOUT_4;
    WAIT_NFI_PIO_READY(timeout);

    if (timeout)
    {
        status = (DRV_Reg16(NFI_DATAR_REG32));
    }
    //~  clear NOB
    DRV_WriteReg32(NFI_CON_REG16, 0);

    if (g_nand_chip.bus16 == NAND_BUS_WIDTH_16)
    {
        NFI_SET_REG16(NFI_PAGEFMT_REG16, PAGEFMT_DBYTE_EN);
        NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);
    }
    // check READY/BUSY status first
    if (!(STATUS_READY & status))
    {
        MSG(ERR, "status is not ready\n");
    }
    // flash is ready now, check status code
    if (STATUS_FAIL & status)
    {
        if (!(STATUS_WR_ALLOW & status))
        {
            MSG(INIT, "status locked\n");
            return FALSE;
        } else
        {
            MSG(INIT, "status unknown\n");
            return FALSE;
        }
    } else
    {
        return TRUE;
    }
}

//---------------------------------------------------------------------------

static void mtk_nand_configure_fdm(u16 u2FDMSize)
{
    NFI_CLN_REG16(NFI_PAGEFMT_REG16, PAGEFMT_FDM_MASK | PAGEFMT_FDM_ECC_MASK);
    NFI_SET_REG16(NFI_PAGEFMT_REG16, u2FDMSize << PAGEFMT_FDM_SHIFT);
    NFI_SET_REG16(NFI_PAGEFMT_REG16, u2FDMSize << PAGEFMT_FDM_ECC_SHIFT);
}

//---------------------------------------------------------------------------
static void mtk_nand_set_autoformat(bool bEnable)
{
    if (bEnable)
    {
        NFI_SET_REG16(NFI_CNFG_REG16, CNFG_AUTO_FMT_EN);
    } else
    {
        NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_AUTO_FMT_EN);
    }
}

//---------------------------------------------------------------------------
static void mtk_nand_command_bp(unsigned command)
{
    u32 timeout;

    switch (command)
    {
      case NAND_CMD_READID:
          /* Issue NAND chip reset command */
          NFI_ISSUE_COMMAND(NAND_CMD_RESET, 0, 0, 0, 0);

          timeout = TIMEOUT_4;

          while (timeout)
              timeout--;

          mtk_nand_reset();

          /* Disable HW ECC */
          NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);

          /* Disable 16-bit I/O */
          NFI_CLN_REG16(NFI_PAGEFMT_REG16, PAGEFMT_DBYTE_EN);
          NFI_SET_REG16(NFI_CNFG_REG16, CNFG_READ_EN | CNFG_BYTE_RW);
          mtk_nand_reset();
          mtk_nand_set_mode(CNFG_OP_SRD);
          mtk_nand_set_command(NAND_CMD_READID);
          mtk_nand_set_address(0, 0, 1, 0);
          DRV_WriteReg32(NFI_CON_REG16, CON_NFI_SRD);
          while (DRV_Reg32(NFI_STA_REG32) & STA_DATAR_STATE) ;
          break;

      default:
          break;
    }
}

//-----------------------------------------------------------------------------
static u8 mtk_nand_read_byte(void)
{
    /* Check the PIO bit is ready or not */
    u32 timeout = TIMEOUT_4;
    WAIT_NFI_PIO_READY(timeout);
    return DRV_Reg8(NFI_DATAR_REG32);
}

bool getflashid(u8 * nand_id, int longest_id_number)
{
    u8 maf_id = 0;
    u8 dev_id = 0;
    int i = 0;
    u8 *id = nand_id;
	u32 regVal;
    //PDN_Power_CONA_DOWN (PDN_PERI_NFI, FALSE);

    DRV_WriteReg32(NFI_ACCCON_REG32, NFI_DEFAULT_ACCESS_TIMING);

    DRV_WriteReg16(NFI_CNFG_REG16, 0);
	regVal = DRV_Reg16(NFI_PAGEFMT_REG16) & 0x4;
    DRV_WriteReg16(NFI_PAGEFMT_REG16, regVal);

    mtk_nand_command_bp(NAND_CMD_READID);

    maf_id = mtk_nand_read_byte();
    dev_id = mtk_nand_read_byte();

    if (maf_id == 0 || dev_id == 0)
    {
        return FALSE;
    }
    //*id= (dev_id<<8)|maf_id;
    //    *id= (maf_id<<8)|dev_id;
    id[0] = maf_id;
    id[1] = dev_id;

    for (i = 2; i < longest_id_number; i++)
        id[i] = mtk_nand_read_byte();

    return TRUE;
}
/*******************************************************************************
 * GPIO(PinMux) register definition
 *******************************************************************************/
#define GPIO_MODE7_NFI        ((P_U32)(GPIO_BASE+0x0370)) //3,2,1,0
#define GPIO_MODE8_NFI        ((P_U32)(GPIO_BASE+0x0380)) //4,5,6,7,8,9,10,11
#define GPIO_MODE9_NFI        ((P_U32)(GPIO_BASE+0x0390)) //10, ALE, CLE, 4,5,7,14
#define GPIO_MODE10_NFI       ((P_U32)(GPIO_BASE+0x03A0))
#define GPIO_NFI_DRV          ((P_U32)(IO_CFG_BOTTOM_BASE+0x0060))
#define GPIO_NFI_PULL          ((P_U32)(IO_CFG_BOTTOM_BASE+0x0040))



/**********************************************************
Description : NFI_BackUp_GPIO_setting
***********************************************************/
static void mtk_nand_gpio_init(void)
{
//    NFI_Handle_st * pHandle = NFI_GET_HANDLE;
	U32 reg_val;

	//DRVING setting
	reg_val = *GPIO_NFI_DRV;
	reg_val = (reg_val & 0xFFFF0000)|0xF6DB;
	*GPIO_NFI_DRV = reg_val;

	reg_val = *GPIO_MODE9_NFI;
	reg_val = (reg_val & 0xFFFFFF0F)|0x40;
	*GPIO_MODE9_NFI = reg_val;
	//TODO wait 10 uS
	reg_val = *GPIO_MODE9_NFI;
	reg_val = (reg_val & 0xFFFFFFF0)|0x4;
	*GPIO_MODE9_NFI = reg_val;
	
	reg_val = *GPIO_MODE7_NFI;
	reg_val = (reg_val & 0x0000FFFF)|0x44440000;
	*GPIO_MODE7_NFI = reg_val;

	reg_val = *GPIO_MODE8_NFI;
	reg_val = (reg_val & 0xFFFFF0FF)|0x44444044;
	*GPIO_MODE8_NFI = reg_val;

	reg_val = *GPIO_MODE9_NFI;
	reg_val = (reg_val & 0x000000FF)|0x44444400;
	*GPIO_MODE9_NFI = reg_val;

	reg_val = *GPIO_MODE10_NFI;
	reg_val = (reg_val & 0xFFFFF000)|0x00000444;
	*GPIO_MODE10_NFI = reg_val;

	reg_val = *GPIO_MODE8_NFI;
	reg_val = (reg_val & 0xFFFFF0FF)|0x00000400;
	*GPIO_MODE8_NFI = reg_val;
}
int mtk_nand_init(void)
{
    int i, j, busw;
     u8 id[NAND_MAX_ID];
    u16 spare_bit = 0;

    u16 spare_per_sector = 16;
    u32 ecc_bit = 4;
    // Config pin mux for NAND device
    mtk_nand_gpio_init();

    nfi_buf = (unsigned char *)NAND_NFI_BUFFER;

    memset(&devinfo, 0, sizeof(devinfo));

    /* Dynamic Control */
    g_bInitDone = FALSE;    
    g_kCMD.u4OOBRowAddr = (u32) - 1;

#if CFG_FPGA_PLATFORM		// FPGA NAND is placed at CS1
		DRV_WriteReg16(NFI_CSEL_REG16, 0);
#else
    DRV_WriteReg16(NFI_CSEL_REG16, NFI_DEFAULT_CS);
#endif


    /* Set default NFI access timing control */
    DRV_WriteReg32(NFI_ACCCON_REG32, NFI_DEFAULT_ACCESS_TIMING);

    DRV_WriteReg16(NFI_CNFG_REG16, 0);
    DRV_WriteReg16(NFI_PAGEFMT_REG16, 4);

    /* Reset NFI HW internal state machine and flush NFI in/out FIFO */
    mtk_nand_reset();

    /* Read the first 4 byte to identify the NAND device */

    g_nand_chip.page_shift = NAND_LARGE_PAGE;
    g_nand_chip.page_size = 1 << g_nand_chip.page_shift;
    g_nand_chip.oobblock = NAND_PAGE_SIZE;
    g_nand_chip.oobsize = NAND_BLOCK_BLKS;

    g_nand_chip.nand_ecc_mode = NAND_ECC_HW;

    mtk_nand_command_bp(NAND_CMD_READID);

    for(i=0;i<NAND_MAX_ID;i++){
		id[i]=mtk_nand_read_byte ();
	}
	nand_maf_id = id[0];
    	nand_dev_id = id[1];
     memset(&devinfo, 0, sizeof(devinfo));

    if (!get_device_info(id, &devinfo))
    {
        MSG(INIT, "NAND unsupport\n");
        ASSERT(0);
    }

    g_nand_chip.name = devinfo.devciename;
    g_nand_chip.chipsize = (u64)devinfo.totalsize << 20;
    g_nand_chip.page_size = devinfo.pagesize;
    g_nand_chip.page_shift = uffs(g_nand_chip.page_size) - 1;
    g_nand_chip.oobblock = g_nand_chip.page_size;
	if(devinfo.sectorsize == 512)
	{
    	g_nand_chip.erasesize = devinfo.blocksize << 10;
	}
	else
	{
		g_nand_chip.erasesize = (devinfo.blocksize << 10)/2;
	}
	BLOCK_SIZE = (u32) devinfo.blocksize << 10;
	g_nand_chip.phys_erase_shift = uffs(g_nand_chip.erasesize) - 1;
	g_nand_chip.sector_size = NAND_SECTOR_SIZE;
	g_nand_chip.sector_shift= 9;
	if(devinfo.sectorsize == 1024)
	{
		g_nand_chip.sector_size = 1024;
		g_nand_chip.sector_shift= 10;
		NFI_CLN_REG32(NFI_PAGEFMT_REG16, PAGEFMT_SECTOR_SEL);
	}

    g_nand_chip.bus16 = devinfo.iowidth;
	#if 1
	if(devinfo.vendor != VEND_NONE)
	{
		if(devinfo.feature_set.FeatureSet.Async_timing.feature != 0xFF)
		{
		struct gFeatureSet *feature_set = &(devinfo.feature_set.FeatureSet);
			mtk_nand_SetFeature((u16) feature_set->sfeatureCmd, \
		feature_set->Async_timing.address, &feature_set->Async_timing.feature,\
		sizeof(feature_set->Async_timing.feature));
		}
	}
	#endif
    DRV_WriteReg32(NFI_ACCCON_REG32, devinfo.timmingsetting);

    if (!devinfo.sparesize)
        g_nand_chip.oobsize = (8 << ((ext_id2 >> 2) & 0x01)) * (g_nand_chip.oobblock / g_nand_chip.sector_size);
    else
        g_nand_chip.oobsize = devinfo.sparesize;
    spare_per_sector = g_nand_chip.oobsize / (g_nand_chip.page_size / g_nand_chip.sector_size);

	switch(spare_per_sector)
    {
        case 16:
            spare_bit = PAGEFMT_SPARE_16;
    		ecc_bit = 4;
			spare_per_sector = 16;
            break;
        case 26:
        case 27:
		case 28:
            spare_bit = PAGEFMT_SPARE_26;
    		ecc_bit = 10;
			spare_per_sector = 26;
            break;
		case 32:
            ecc_bit = 12;
			if(devinfo.sectorsize == 1024)
            	spare_bit = PAGEFMT_SPARE_32_1KS;
			else
				spare_bit = PAGEFMT_SPARE_32;
			spare_per_sector = 32;
            break;
		case 40:
            ecc_bit = 18;
            spare_bit = PAGEFMT_SPARE_40;
			spare_per_sector = 40;
            break;
		case 44:
            ecc_bit = 20;
            spare_bit = PAGEFMT_SPARE_44;
			spare_per_sector = 44;
            break;
		case 48:
		case 49:
            ecc_bit = 22;
            spare_bit = PAGEFMT_SPARE_48;
			spare_per_sector = 48;
            break;
		case 50:
		case 51:
            ecc_bit = 24;
            spare_bit = PAGEFMT_SPARE_50;
			spare_per_sector = 50;
            break;
		case 52:
		case 54:
		case 56:
            ecc_bit = 24;
			if(devinfo.sectorsize == 1024)
            	spare_bit = PAGEFMT_SPARE_52_1KS;
			else
				spare_bit = PAGEFMT_SPARE_52;
			spare_per_sector = 32;
            break;
		case 62:
		case 63:
            ecc_bit = 28;
            spare_bit = PAGEFMT_SPARE_62;
			spare_per_sector = 62;
            break;
		case 64:
            ecc_bit = 32;
			if(devinfo.sectorsize == 1024)
            	spare_bit = PAGEFMT_SPARE_64_1KS;
			else
				spare_bit = PAGEFMT_SPARE_64;
			spare_per_sector = 64;
            break;
		case 72:
			ecc_bit = 36;
			if(devinfo.sectorsize == 1024)
            	spare_bit = PAGEFMT_SPARE_72_1KS;
			spare_per_sector = 72;
            break;
		case 80:
			ecc_bit = 40;
			if(devinfo.sectorsize == 1024)
            	spare_bit = PAGEFMT_SPARE_80_1KS;
			spare_per_sector = 80;
            break;
		case 88:
			ecc_bit = 44;
			if(devinfo.sectorsize == 1024)
            	spare_bit = PAGEFMT_SPARE_88_1KS;
			spare_per_sector = 88;
            break;
		case 96:
		case 98:
			ecc_bit = 48;
			if(devinfo.sectorsize == 1024)
            	spare_bit = PAGEFMT_SPARE_96_1KS;
			spare_per_sector = 96;
            break;
		case 100:
		case 102:
		case 104:
			ecc_bit = 52;
			if(devinfo.sectorsize == 1024)
            	spare_bit = PAGEFMT_SPARE_100_1KS;
			spare_per_sector = 100;
            break;
		case 124:
		case 126:
		case 128:
			ecc_bit = 60;
			if(devinfo.sectorsize == 1024)
            	spare_bit = PAGEFMT_SPARE_124_1KS;
			spare_per_sector = 124;
            break;
		default:
			MSG(INIT, "[NAND]: NFI not support oobsize: %x\n", spare_per_sector);
        ASSERT(0);
    }

    g_nand_chip.oobsize = spare_per_sector * (g_nand_chip.page_size / g_nand_chip.sector_size);
    MSG(INIT, "[NAND]: oobsize: %x\n", g_nand_chip.oobsize);
    #if defined(MTK_COMBO_NAND_SUPPORT)
    	g_nand_chip.chipsize -= (PART_SIZE_BMTPOOL);
    #else
    	g_nand_chip.chipsize -= (g_nand_chip.sector_size == 512?g_nand_chip.erasesize : g_nand_chip.erasesize*2) * (BMT_POOL_SIZE);
    #endif
    if (g_nand_chip.bus16 == NAND_BUS_WIDTH_16)
    {
#ifdef  DBG_PRELOADER
        MSG(INIT, "USE 16 IO\n");
#endif
        NFI_SET_REG16(NFI_PAGEFMT_REG16, PAGEFMT_DBYTE_EN);
    }

	if (16384 == g_nand_chip.oobblock)
    {
        NFI_SET_REG16(NFI_PAGEFMT_REG16, (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_16K_1KS);
        nand_oob = &nand_oob_128;
    } else if (8192 == g_nand_chip.oobblock)
    {
        NFI_SET_REG16(NFI_PAGEFMT_REG16, (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_8K_1KS);
        nand_oob = &nand_oob_128;
    } else if (4096 == g_nand_chip.oobblock)
    {
        if(devinfo.sectorsize == 512)
	        NFI_SET_REG16(NFI_PAGEFMT_REG16, (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_4K);
	    else
			NFI_SET_REG16(NFI_PAGEFMT_REG16, (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_4K_1KS);
        nand_oob = &nand_oob_128;
    } else if (2048 == g_nand_chip.oobblock)
    {
     	if(devinfo.sectorsize == 512)
          	NFI_SET_REG16(NFI_PAGEFMT_REG16, (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_2K);
		else
			NFI_SET_REG16(NFI_PAGEFMT_REG16, (spare_bit << PAGEFMT_SPARE_SHIFT) | PAGEFMT_2K_1KS);
        nand_oob = &nand_oob_64;
    }    

    if (g_nand_chip.nand_ecc_mode == NAND_ECC_HW)
    {
        // MSG (INIT, "Use HW ECC\n");
        NFI_SET_REG32(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
        ECC_Config(ecc_bit);
        mtk_nand_configure_fdm(8);
    }

    /* Initilize interrupt. Clear interrupt, read clear. */
    DRV_Reg16(NFI_INTR_REG16);
	mtk_nand_randomizer_config(&devinfo.feature_set.randConfig,0);

    /* Interrupt arise when read data or program data to/from AHB is done. */
    DRV_WriteReg16(NFI_INTR_EN_REG16, 0);

		#if defined(MTK_COMBO_NAND_SUPPORT)
    if (!(init_bmt(&g_nand_chip, ((PART_SIZE_BMTPOOL)/(g_nand_chip.sector_size == 512?g_nand_chip.erasesize : g_nand_chip.erasesize*2)))))
    #else
    if (!(init_bmt(&g_nand_chip, BMT_POOL_SIZE)))
    #endif
    {
        MSG(INIT, "Error: init bmt failed, quit!\n");
        ASSERT(0);
        return 0;
    }

    g_nand_chip.chipsize -= (g_nand_chip.sector_size == 512?g_nand_chip.erasesize : g_nand_chip.erasesize*2) * (PMT_POOL_SIZE);
    return 0;
}

//-----------------------------------------------------------------------------
static void mtk_nand_stop_read(void)
{
    NFI_CLN_REG32(NFI_CON_REG16, CON_NFI_BRD);
    if (g_bHwEcc)
    {
        ECC_Decode_End();
    }
}

//-----------------------------------------------------------------------------
static void mtk_nand_stop_write(void)
{
    NFI_CLN_REG32(NFI_CON_REG16, CON_NFI_BWR);
    if (g_bHwEcc)
    {
        ECC_Encode_End();
    }
}

//-----------------------------------------------------------------------------
static bool mtk_nand_check_dececc_done(u32 u4SecNum)
{
    u32 timeout, dec_mask;
    timeout = 0xffffff;
    dec_mask = (1 << u4SecNum) - 1;
    while ((dec_mask != DRV_Reg(ECC_DECDONE_REG16)) && timeout > 0)
        timeout--;
    if (timeout == 0)
    {
        MSG(INIT, "ECC_DECDONE: timeout 0x%x %d\n",DRV_Reg(ECC_DECDONE_REG16),u4SecNum);
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
static bool mtk_nand_read_page_data(u32 * buf)
{
    u32 timeout = 0xFFFF;
    u32 u4Size = g_nand_chip.oobblock;
    u32 i;
    u32 *pBuf32;

#if (USE_AHB_MODE)
    pBuf32 = (u32 *) buf;
    NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);

    DRV_Reg16(NFI_INTR_REG16);
    DRV_WriteReg16(NFI_INTR_EN_REG16, INTR_AHB_DONE_EN);
    NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BRD);

    while (!(DRV_Reg16(NFI_INTR_REG16) & INTR_AHB_DONE))
    {
        timeout--;
        if (0 == timeout)
        {
            return FALSE;
        }
    }

    timeout = 0xFFFF;
    while ((u4Size >> g_nand_chip.sector_shift) > ((DRV_Reg32(NFI_BYTELEN_REG16) & 0x1f000) >> 12))
    {
        timeout--;
        if (0 == timeout)
        {
            return FALSE;
        }
    }

#else
    NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);
    NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BRD);
    pBuf32 = (u32 *) buf;

    for (i = 0; (i < (u4Size >> 2)) && (timeout > 0);)
    {
        if (DRV_Reg16(NFI_PIO_DIRDY_REG16) & 1)
        {
            *pBuf32++ = DRV_Reg32(NFI_DATAR_REG32);
            i++;
        } else
        {
            timeout--;
        }
        if (0 == timeout)
        {
            return FALSE;
        }
    }
#endif
    return TRUE;
}

//-----------------------------------------------------------------------------
static bool mtk_nand_write_page_data(u32 * buf)
{
    u32 timeout = 0xFFFF;
    u32 u4Size = g_nand_chip.oobblock;

#if (USE_AHB_MODE)
    u32 *pBuf32;
    pBuf32 = (u32 *) buf;

    NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);

    DRV_Reg16(NFI_INTR_REG16);
    DRV_WriteReg16(NFI_INTR_EN_REG16, INTR_AHB_DONE_EN);
    NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BWR);
    while (!(DRV_Reg16(NFI_INTR_REG16) & INTR_AHB_DONE))
    {
        timeout--;
        if (0 == timeout)
        {
            return FALSE;
        }
    }

#else
    u32 i;
    u32 *pBuf32;
    pBuf32 = (u32 *) buf;

    NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_BYTE_RW);
    NFI_SET_REG32(NFI_CON_REG16, CON_NFI_BWR);

    for (i = 0; (i < (u4Size >> 2)) && (timeout > 0);)
    {
        if (DRV_Reg16(NFI_PIO_DIRDY_REG16) & 1)
        {
            DRV_WriteReg32(NFI_DATAW_REG32, *pBuf32++);
            i++;
        } else
        {
            timeout--;
        }

        if (0 == timeout)
        {
            return FALSE;
        }
    }
#endif
    return TRUE;
}

//-----------------------------------------------------------------------------
static void mtk_nand_read_fdm_data(u32 u4SecNum, u8 * spare_buf)
{
    u32 i;
    u32 *pBuf32 = (u32 *) spare_buf;

    for (i = 0; i < u4SecNum; ++i)
    {
        *pBuf32++ = DRV_Reg32(NFI_FDM0L_REG32 + (i << 3));
        *pBuf32++ = DRV_Reg32(NFI_FDM0M_REG32 + (i << 3));
    }
}

//-----------------------------------------------------------------------------
static void mtk_nand_write_fdm_data(u32 u4SecNum, u8 * oob)
{
    u32 i;
    u32 *pBuf32 = (u32 *) oob;

    for (i = 0; i < u4SecNum; ++i)
    {
        DRV_WriteReg32(NFI_FDM0L_REG32 + (i << 3), *pBuf32++);
        DRV_WriteReg32(NFI_FDM0M_REG32 + (i << 3), *pBuf32++);
    }
}

//---------------------------------------------------------------------------
static bool mtk_nand_ready_for_read(u32 page_addr, u32 sec_num, u8 * buf)
{
    u32 u4RowAddr = page_addr;
    u32 colnob = 2;
    u32 rownob = devinfo.addr_cycle - colnob;
    bool bRet = FALSE;

    if (!mtk_nand_reset())
    {
        goto cleanup;
    }

    /* Enable HW ECC */
    NFI_SET_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);

    mtk_nand_set_mode(CNFG_OP_READ);
    NFI_SET_REG16(NFI_CNFG_REG16, CNFG_READ_EN);
    DRV_WriteReg32(NFI_CON_REG16, sec_num << CON_NFI_SEC_SHIFT);

#if USE_AHB_MODE
    NFI_SET_REG16(NFI_CNFG_REG16, CNFG_AHB);
#else
    NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_AHB);
#endif
    DRV_WriteReg32(NFI_STRADDR_REG32, buf);
    if (g_bHwEcc)
    {
        NFI_SET_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
    } else
    {
        NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
    }

    mtk_nand_set_autoformat(TRUE);
    if (g_bHwEcc)
    {
        ECC_Decode_Start();
    }
    if (!mtk_nand_set_command(NAND_CMD_READ0))
    {
        goto cleanup;
    }
    if (!mtk_nand_set_address(0, u4RowAddr, colnob, rownob))
    {
        goto cleanup;
    }

    if (!mtk_nand_set_command(NAND_CMD_READSTART))
    {
        goto cleanup;
    }

    if (!mtk_nand_status_ready(STA_NAND_BUSY))
    {
        goto cleanup;
    }

    bRet = TRUE;

  cleanup:
    return bRet;
}

//-----------------------------------------------------------------------------
static bool mtk_nand_ready_for_write(u32 page_addr, u32 sec_num, u8 * buf)
{
    bool bRet = FALSE;
    u32 u4RowAddr = page_addr;
    u32 colnob = 2;
    u32 rownob = devinfo.addr_cycle - colnob;

    if (!mtk_nand_reset())
    {
        return FALSE;
    }

    mtk_nand_set_mode(CNFG_OP_PRGM);

    NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_READ_EN);

    DRV_WriteReg32(NFI_CON_REG16, sec_num << CON_NFI_SEC_SHIFT);

#if USE_AHB_MODE
    NFI_SET_REG16(NFI_CNFG_REG16, CNFG_AHB);
    DRV_WriteReg32(NFI_STRADDR_REG32, buf);
#else
    NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_AHB);
#endif

    if (g_bHwEcc)
    {
        NFI_SET_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
    } else
    {
        NFI_CLN_REG16(NFI_CNFG_REG16, CNFG_HW_ECC_EN);
    }
    mtk_nand_set_autoformat(TRUE);
    if (g_bHwEcc)
    {
        ECC_Encode_Start();
    }

    if (!mtk_nand_set_command(NAND_CMD_SEQIN))
    {
        goto cleanup;
    }

    if (!mtk_nand_set_address(0, u4RowAddr, colnob, rownob))
    {
        goto cleanup;
    }

    if (!mtk_nand_status_ready(STA_NAND_BUSY))
    {
        goto cleanup;
    }

    bRet = TRUE;
  cleanup:

    return bRet;
}

//#############################################################################
//# NAND Driver : Page Read
//#
//# NAND Page Format (Large Page 2KB)
//#  |------ Page:2048 Bytes ----->>||---- Spare:64 Bytes -->>|
//#
//# Parameter Description:
//#     page_addr               : specify the starting page in NAND flash
//#
//#############################################################################
int mtk_nand_read_page_hwecc(u64 logical_addr, char *buf)
{
    int i, start, len, offset = 0;
	u32 page_no;
	u32 block, mapped_block;
	int rtn = ERR_RTN_SUCCESS;
    u8 *oob = buf + g_nand_chip.page_size;
	page_no = mtk_nand_page_transform(logical_addr,&block,&mapped_block);

	rtn = mtk_nand_read_page_hw(page_no, buf, g_nand_spare);
    if (rtn != ERR_RTN_SUCCESS)  // g_nand_spare
        return FALSE;

    for (i = 0; i < MTD_MAX_OOBFREE_ENTRIES && nand_oob->oobfree[i].length; i++)
    {
        /* Set the reserved bytes to 0xff */
        start = nand_oob->oobfree[i].offset;
        len = nand_oob->oobfree[i].length;
        memcpy(oob + offset, g_nand_spare + start, len);
        offset += len;
    }

    return true;
}

int mtk_nand_read_page_hw(u32 page, u8 * dat, u8 * oob)
{
    int bRet = ERR_RTN_SUCCESS;
    u8 *pPageBuf;
    u32 u4SecNum = g_nand_chip.oobblock >> g_nand_chip.sector_shift;
    pPageBuf = (u8 *) dat;
	bool readRetry = FALSE;
	int retryCount = 0;

	do{
		if(use_randomizer && page >= RAND_START_ADDR)
			mtk_nand_turn_on_randomizer(page);
	    if (mtk_nand_ready_for_read(page, u4SecNum, pPageBuf))
	    {
	        if (!mtk_nand_read_page_data((u32 *) pPageBuf))
	        {
	            bRet = ERR_RTN_FAIL;
	        }

	        if (!mtk_nand_status_ready(STA_NAND_BUSY))
	        {
	            bRet = ERR_RTN_FAIL;
	        }
	        if (g_bHwEcc)
	        {
	            if (!mtk_nand_check_dececc_done(u4SecNum))
	            {
	                bRet = ERR_RTN_FAIL;
	            }
	        }
	        mtk_nand_read_fdm_data(u4SecNum, oob);
	        if (g_bHwEcc)
	        {
	            if (!mtk_nand_check_bch_error(pPageBuf, u4SecNum - 1, page))
	            {
	                MSG(ERASE, "check bch error !\n");
					if(devinfo.vendor != VEND_NONE){
						readRetry = TRUE;
					}
	                bRet = ERR_RTN_BCH_FAIL;
	            }
	        }
			if(0 != (DRV_Reg32(NFI_STA_REG32) & STA_READ_EMPTY))
			{
				memset(pPageBuf, 0xFF,g_nand_chip.page_size);
				memset(oob, 0xFF,8*u4SecNum);
			}
	        mtk_nand_stop_read();
			if(use_randomizer)
				mtk_nand_turn_off_randomizer();
	    }
		if (bRet == ERR_RTN_BCH_FAIL)
		{
			u32 feature = mtk_nand_rrtry_setting(devinfo, devinfo.feature_set.FeatureSet.rtype,devinfo.feature_set.FeatureSet.readRetryStart,retryCount);
			if(retryCount < devinfo.feature_set.FeatureSet.readRetryCnt)
			{
				mtk_nand_rrtry_func(devinfo,feature,FALSE);
				retryCount++;
			}
			else
			{
				feature = devinfo.feature_set.FeatureSet.readRetryDefault;
				mtk_nand_rrtry_func(devinfo,feature,TRUE);
				readRetry = FALSE;
			}		
		}
		else
		{
			if(retryCount != 0)
			{
				u32 feature = devinfo.feature_set.FeatureSet.readRetryDefault;
				mtk_nand_rrtry_func(devinfo,feature,TRUE);
			}
			readRetry = FALSE;
		}
		if(TRUE == readRetry)
			bRet = ERR_RTN_SUCCESS;
	}while(readRetry);
    return bRet;
}

//#############################################################################
//# NAND Driver : Page Write
//#
//# NAND Page Format (Large Page 2KB)
//#  |------ Page:2048 Bytes ----->>||---- Spare:64 Bytes -->>|
//#
//# Parameter Description:
//#     page_addr               : specify the starting page in NAND flash
//#
//#############################################################################

int mtk_nand_write_page_hwecc(u64 logical_addr, char *buf)
{
    u32 block,mapped_block;
    u32 page_no;
    u8 *oob = buf + g_nand_chip.oobblock;
    int i;
    int start, len, offset;

	page_no = mtk_nand_page_transform(logical_addr ,&block, &mapped_block);
	
    for (i = 0; i < sizeof(g_nand_spare); i++)
        *(g_nand_spare + i) = 0xFF;

    offset = 0;
    for (i = 0; i < MTD_MAX_OOBFREE_ENTRIES && nand_oob->oobfree[i].length; i++)
    {
        /* Set the reserved bytes to 0xff */
        start = nand_oob->oobfree[i].offset;
        len = nand_oob->oobfree[i].length;
        memcpy((g_nand_spare + start), (oob + offset), len);
        offset += len;
    }

    // write bad index into oob
    if (mapped_block != block)
    {
        set_bad_index_to_oob(g_nand_spare, block);
    } else
    {
        set_bad_index_to_oob(g_nand_spare, FAKE_INDEX);
    }

    if (!mtk_nand_write_page_hw(page_no, buf, g_nand_spare))
    {
        MSG(INIT, "write fail happened @ block 0x%x, page 0x%x\n", mapped_block, page_no);
        return update_bmt((u64)page_no * g_nand_chip.oobblock, UPDATE_WRITE_FAIL, buf, g_nand_spare);
    }

    return TRUE;
}

int mtk_nand_write_page_hw(u32 page, u8 * dat, u8 * oob)
{
    bool bRet = TRUE;
    u32 pagesz = g_nand_chip.oobblock;
    u32 timeout, u4SecNum = pagesz >> g_nand_chip.sector_shift;

    int i, j, start, len;
    bool empty = TRUE;
    u8 oob_checksum = 0;

    for (i = 0; i < MTD_MAX_OOBFREE_ENTRIES && nand_oob->oobfree[i].length; i++)
    {
        /* Set the reserved bytes to 0xff */
        start = nand_oob->oobfree[i].offset;
        len = nand_oob->oobfree[i].length;
        for (j = 0; j < len; j++)
        {
            oob_checksum ^= oob[start + j];
            if (oob[start + j] != 0xFF)
                empty = FALSE;
        }
    }

    if (!empty)
    {
        oob[nand_oob->oobfree[i - 1].offset + nand_oob->oobfree[i - 1].length] = oob_checksum;
    }
	if(use_randomizer && page >= RAND_START_ADDR)
		mtk_nand_turn_on_randomizer(page);

    while (DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY) ;

    if (mtk_nand_ready_for_write(page, u4SecNum, dat))
    {
        mtk_nand_write_fdm_data(u4SecNum, oob);
        if (!mtk_nand_write_page_data((u32 *) dat))
        {
            bRet = FALSE;
        }
        if (!mtk_nand_check_RW_count(g_nand_chip.oobblock))
        {
            bRet = FALSE;
        }
        mtk_nand_stop_write();
        mtk_nand_set_command(NAND_CMD_PAGEPROG);
        mtk_nand_status_ready(STA_NAND_BUSY);
		if(use_randomizer && page > 2048)
			mtk_nand_turn_off_randomizer();
        return mtk_nand_read_status();
    } else
    {
    	if(use_randomizer && page >= RAND_START_ADDR)
			mtk_nand_turn_off_randomizer();
        return FALSE;
    }

    return bRet;
}

unsigned int nand_block_bad(u64 logical_addr)
{
    u32 block = logical_addr;
    u32 mapped_block;
	mtk_nand_page_transform(logical_addr,&block,&mapped_block);
    if (nand_block_bad_hw(mapped_block * BLOCK_SIZE))
    {
        if (update_bmt((u64)mapped_block * BLOCK_SIZE, UPDATE_UNMAPPED_BLOCK, NULL, NULL))
        {
            return logical_addr;    // return logical address
        }
        return logical_addr + BLOCK_SIZE;
    }

    return logical_addr;
}

bool nand_block_bad_hw(u64 logical_addr)
{
    bool bRet = FALSE;
    u32 page = logical_addr / g_nand_chip.oobblock;

    int i, page_num = (BLOCK_SIZE / g_nand_chip.oobblock);
    //unsigned char *pspare;
    char *tmp = (char *)nfi_buf;
    memset(tmp, 0x0, g_nand_chip.oobblock + g_nand_chip.oobsize);

    u32 u4SecNum = g_nand_chip.oobblock >> g_nand_chip.sector_shift;
    page &= ~(page_num - 1);
	//page_no = mtk_nand_page_transform(logical_addr,&block,&mapped_block);

	if(use_randomizer && page >= RAND_START_ADDR)
		mtk_nand_turn_on_randomizer(page);

    if (mtk_nand_ready_for_read(page, u4SecNum, tmp))
    {
        if (!mtk_nand_read_page_data((u32 *) tmp))
        {
            bRet = FALSE;
        }

        if (!mtk_nand_status_ready(STA_NAND_BUSY))
        {
            bRet = FALSE;
        }

        if (!mtk_nand_check_dececc_done(u4SecNum))
        {
            bRet = FALSE;
        }

        mtk_nand_read_fdm_data(u4SecNum, g_nand_spare);

        if (!mtk_nand_check_bch_error(tmp, u4SecNum - 1, page))
        {
            MSG(ERASE, "check bch error !\n");
            bRet = FALSE;
        }
		
        if(0 != (DRV_Reg32(NFI_STA_REG32) & STA_READ_EMPTY))
			{
				memset(nfi_buf, 0xFF,g_nand_chip.page_size);
				memset(g_nand_spare, 0xFF,8*u4SecNum);
			}
	    mtk_nand_stop_read();
		if(use_randomizer)
			mtk_nand_turn_off_randomizer();
    }

    if (g_nand_spare[0] != 0xFF || g_nand_spare[8] != 0xFF || g_nand_spare[16] != 0xFF || g_nand_spare[24] != 0xFF)
    {
        bRet = TRUE;
        // break;
    }

    return bRet;
}

bool mark_block_bad(u64 logical_addr)
{
    u32 block;
    u32 mapped_block;
	mtk_nand_page_transform(logical_addr,&block,&mapped_block);
    return mark_block_bad_hw(mapped_block * BLOCK_SIZE);
}

bool mark_block_bad_hw(u64 offset)
{
    bool bRet = FALSE;
    u32 index;
    u32 page_addr = (u32)(offset / g_nand_chip.oobblock);
    u32 u4SecNum = g_nand_chip.oobblock >> g_nand_chip.sector_shift;
    unsigned char *pspare;
    int i, page_num = (BLOCK_SIZE/ g_nand_chip.oobblock);
    unsigned char buf[2048];

    for (index = 0; index < 64; index++)
        *(g_nand_spare + index) = 0xFF;

    pspare = g_nand_spare;

    for (index = 8, i = 0; i < 4; i++)
        pspare[i * index] = 0x0;

    page_addr &= ~(page_num - 1);
    MSG(BAD, "Mark bad block at 0x%x\n", page_addr);
	if(use_randomizer && page_addr >= RAND_START_ADDR)
		mtk_nand_turn_on_randomizer(page_addr);
    while (DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY) ;

    if (mtk_nand_ready_for_write(page_addr, u4SecNum, buf))
    {
        mtk_nand_write_fdm_data(u4SecNum, g_nand_spare);
        if (!mtk_nand_write_page_data((u32 *) & buf))
        {
            bRet = FALSE;
        }
        if (!mtk_nand_check_RW_count(g_nand_chip.oobblock))
        {
            bRet = FALSE;
        }
        mtk_nand_stop_write();
        mtk_nand_set_command(NAND_CMD_PAGEPROG);
        mtk_nand_status_ready(STA_NAND_BUSY);
		if(use_randomizer)
			mtk_nand_turn_off_randomizer();
    } else
    {
        return FALSE;
    }

    for (index = 0; index < 64; index++)
        *(g_nand_spare + index) = 0xFF;
}

//#############################################################################
//# NAND Driver : Page Write
//#
//# NAND Page Format (Large Page 2KB)
//#  |------ Page:2048 Bytes ----->>||---- Spare:64 Bytes -->>|
//#
//# Parameter Description:
//#     page_addr               : specify the starting page in NAND flash
//#
//#############################################################################
bool mtk_nand_erase_hw(u64 offset)
{
    bool bRet = TRUE;
    u32 timeout, u4SecNum = g_nand_chip.oobblock >> g_nand_chip.sector_shift;
    u32 rownob = devinfo.addr_cycle - 2;
    u32 page_addr = (u32)(offset / g_nand_chip.oobblock);

    if (nand_block_bad_hw(offset))
    {
        return FALSE;
    }

    mtk_nand_reset();
    mtk_nand_set_mode(CNFG_OP_ERASE);
    mtk_nand_set_command(NAND_CMD_ERASE1);
    mtk_nand_set_address(0, page_addr, 0, rownob);

    mtk_nand_set_command(NAND_CMD_ERASE2);
    if (!mtk_nand_status_ready(STA_NAND_BUSY))
    {
        return FALSE;
    }

    if (!mtk_nand_read_status())
    {
        return FALSE;
    }
    return bRet;
}

int mtk_nand_erase(u64 logical_addr)
{
    u32 block;
    u32 mapped_block;
	mtk_nand_page_transform((u64)logical_addr,&block,&mapped_block);
    if (!mtk_nand_erase_hw(mapped_block * BLOCK_SIZE))
    {
        MSG(INIT, "erase block 0x%x failed\n", mapped_block);
        return update_bmt((u64)mapped_block * BLOCK_SIZE, UPDATE_ERASE_FAIL, NULL, NULL);
    }

    return TRUE;
}

bool mtk_nand_wait_for_finish(void)
{
    while (DRV_Reg32(NFI_STA_REG32) & STA_NAND_BUSY) ;
    return TRUE;
}

/**************************************************************************
*  MACRO LIKE FUNCTION
**************************************************************************/
static int nand_bread(blkdev_t * bdev, u32 blknr, u32 blks, u8 * buf)
{
    u32 i;
    u64 offset = (u64)blknr * bdev->blksz;

    for (i = 0; i < blks; i++)
    {
        offset = nand_read_data(buf, offset);
        offset += bdev->blksz;
        buf += bdev->blksz;
    }
    return 0;
}

static int nand_bwrite(blkdev_t * bdev, u32 blknr, u32 blks, u8 * buf)
{
    u32 i;
    u64 offset = (u64)blknr * bdev->blksz;

    for (i = 0; i < blks; i++)
    {
        offset = nand_write_data(buf, offset);
        offset += bdev->blksz;
        buf += bdev->blksz;
    }
    return 0;
}

// ==========================================================
// NAND Common Interface - Init
// ==========================================================

u32 nand_init_device(void)
{
    if (!blkdev_get(BOOTDEV_NAND))
    {
        mtk_nand_reset_descriptor();
        mtk_nand_init();

        PAGE_SIZE = (u32) g_nand_chip.page_size;
		if(VEND_NONE != gVendor) //MLC
	        BLOCK_SIZE = (u32) g_nand_chip.erasesize*2;
		else
			BLOCK_SIZE = (u32) g_nand_chip.erasesize;

        memset(&g_nand_bdev, 0, sizeof(blkdev_t));
        g_nand_bdev.blksz = g_nand_chip.page_size;
		if(VEND_NONE != gVendor)
        	g_nand_bdev.erasesz = g_nand_chip.erasesize*2;
		else
			g_nand_bdev.erasesz = g_nand_chip.erasesize;
        g_nand_bdev.blks = g_nand_chip.chipsize;
        g_nand_bdev.bread = nand_bread;
        g_nand_bdev.bwrite = nand_bwrite;
        g_nand_bdev.blkbuf = (u8 *) storage_buffer;
        g_nand_bdev.type = BOOTDEV_NAND;
        blkdev_register(&g_nand_bdev);
    }

    return 0;
}

void Invert_Bits(u8 * buff_ptr, u32 bit_pos)
{
    u32 byte_pos = 0;
    u8 byte_val = 0;
    u8 temp_val = 0;
    u32 invert_bit = 0;

    byte_pos = bit_pos >> 3;
    invert_bit = bit_pos & ((1 << 3) - 1);
    byte_val = buff_ptr[byte_pos];
    temp_val = byte_val & (1 << invert_bit);

    if (temp_val > 0)
        byte_val &= ~temp_val;
    else
        byte_val |= (1 << invert_bit);
    buff_ptr[byte_pos] = byte_val;
}

void compare_page(u8 * testbuff, u8 * sourcebuff, u32 length, char *s)
{
    u32 errnum = 0;
    u32 ii = 0;
    u32 index;
    printf("%s", s);
    for (index = 0; index < length; index++)
    {
        if (testbuff[index] != sourcebuff[index])
        {
            u8 t = sourcebuff[index] ^ testbuff[index];
            for (ii = 0; ii < 8; ii++)
            {
                if ((t >> ii) & 0x1 == 1)
                {
                    errnum++;
                }
            }
            printf(" ([%d]=%x) != ([%d]=%x )", index, sourcebuff[index], index, testbuff[index]);
        }

    }
    if (errnum > 0)
    {
        printf(": page have %d mismatch bits\n", errnum);
    } else
    {
        printf(" :the two buffers are same!\n");
    }
}

u8 empty_page(u8 * sourcebuff, u32 length)
{
    u32 index = 0;
    for (index = 0; index < length; index++)
    {
        if (sourcebuff[index] != 0xFF)
        {
            return 0;
        }
    }
    return 1;
}

u32 __nand_ecc_test(u32 offset, u32 max_ecc_capable)
{

    int ecc_level = max_ecc_capable;
    int sec_num = g_nand_chip.page_size >> g_nand_chip.sector_shift;
    u32 sec_size = g_nand_chip.page_size / sec_num;
    u32 NAND_MAX_PAGE_LENGTH = g_nand_chip.page_size + 8 * sec_num;
    u32 chk_bit_len = 64 * 4;
    u32 page_per_blk = BLOCK_SIZE / g_nand_chip.page_size;
    u32 sec_index, curr_error_bit, err_bits_per_sec, page_idx, errbits, err;

    u8 *testbuff = (u8*)malloc(NAND_MAX_PAGE_LENGTH);
    u8 *sourcebuff = (u8*)malloc(NAND_MAX_PAGE_LENGTH);
    u8 empty;

    for (err_bits_per_sec = 1; err_bits_per_sec <= ecc_level; err_bits_per_sec++)
    {
        printf("~~~start test ecc correct in ");
#if USE_AHB_MODE
        printf(" AHB mode");
#else
        printf(" MCU mode");
#endif
        printf(", every sector have %d bit error~~~\n", err_bits_per_sec);
        for (curr_error_bit = 0; curr_error_bit < chk_bit_len && offset < g_nand_chip.chipsize; offset += g_nand_chip.page_size)
        {
            memset(testbuff, 0x0a, NAND_MAX_PAGE_LENGTH);
            memset(sourcebuff, 0x0b, NAND_MAX_PAGE_LENGTH);
            g_bHwEcc = TRUE;
            nand_read_data(sourcebuff, offset);
            empty = empty_page(sourcebuff, g_nand_chip.page_size);
            if (empty)
            {
                printf("page %d is empty\n", offset / g_nand_chip.page_size);
                memset(sourcebuff, 0x0c, NAND_MAX_PAGE_LENGTH);
                nand_write_data(sourcebuff, offset);
                nand_read_data(sourcebuff, offset);
            }
            if (0 != (DRV_Reg32(ECC_DECENUM0_REG32) & 0xFFFFF) ||0 != (DRV_Reg32(ECC_DECENUM1_REG32) & 0xFFFFF) )
            {
                printf("skip the page %d, because it is empty ( %d )or already have error bits (%x)!\n", offset / g_nand_chip.page_size, empty, err);
            } else
            {
                printf("~~~start test ecc correct in Page 0x%x ~~~\n", offset / g_nand_chip.page_size);
                memcpy(testbuff, sourcebuff, NAND_MAX_PAGE_LENGTH);
                for (sec_index = 0; sec_index < sec_num; sec_index++)
                {
                    //printf("insert err bit @ page %d:sector %d : bit ",page_idx+offset/g_nand_chip.page_size,sec_index);
                    for (errbits = 0; errbits < err_bits_per_sec; errbits++)
                    {
                        Invert_Bits(((u8 *) testbuff) + sec_index * sec_size, curr_error_bit);
                        //printf("%d, ",curr_error_bit);
                        curr_error_bit++;
                    }
                    //printf("\n");
                }
                g_bHwEcc = FALSE;
                nand_write_data(testbuff, offset);
                compare_page(testbuff, sourcebuff, NAND_MAX_PAGE_LENGTH, "source and test buff check ");
                g_bHwEcc = TRUE;
                nand_read_data(testbuff, offset);
                compare_page(testbuff, sourcebuff, NAND_MAX_PAGE_LENGTH, "read back check ");
            }
        }
    }

    free(testbuff);
    free(sourcebuff);

}

u32 nand_ecc_test(void)
{
    part_t *part = part_get(PART_UBOOT);
    u32 offset = (part->startblk) * g_nand_chip.page_size;
    __nand_ecc_test(offset, 4);

    part_t *part2 = part_get(PART_BOOTIMG);
    offset = (part2->startblk) * g_nand_chip.page_size;
    __nand_ecc_test(offset, 4);
    return 0;
}

u32 nand_get_device_id(u8 * id, u32 len)
{
    u8 buf[16];

    // Config pin mux for NAND device, since EMI init will be called before NAND init done/
    mtk_nand_gpio_init();

    if (TRUE != getflashid(buf, len))
        return -1;

    len = len > 16 ? 16 : len;

    memcpy(id, buf, len);

    return 0;
}

/* LEGACY - TO BE REMOVED { */
// ==========================================================
// NAND Common Interface - Correct R/W Address
// ==========================================================
u64 nand_find_safe_block(u32 offset)
{

    u64 original_offset = offset;
    u64 new_offset = 0;
    unsigned int blk_index = 0;
    static BOOL Bad_Block_Table_init = FALSE;

    if (Bad_Block_Table_init == FALSE)
    {
        Bad_Block_Table_init = TRUE;
        memset(Bad_Block_Table, 0, sizeof(Bad_Block_Table));
        print("Bad_Block_Table init, sizeof(Bad_Block_Table)= %d \n", sizeof(Bad_Block_Table));
    }

    blk_index = BLOCK_ALIGN(offset) / BLOCK_SIZE;
    if (Bad_Block_Table[blk_index] == 1)
    {
        return offset;
    }
    // new_offset is block alignment
    new_offset = nand_block_bad(BLOCK_ALIGN(offset));

    // find next block until the block is good
    while (new_offset != BLOCK_ALIGN(offset))
    {
        offset = new_offset;
        new_offset = nand_block_bad(BLOCK_ALIGN(offset));
    }

    if (original_offset != offset)
    {
        Bad_Block_Table[(original_offset / BLOCK_SIZE)] = 2;
        print("offset (0x%x) is bad block. next safe block is (0x%x)\n", original_offset, offset);
    }

    Bad_Block_Table[(BLOCK_ALIGN(offset) / BLOCK_SIZE)] = 1;

    return offset;
}

/* LEGACY - TO BE REMOVED } */

// ==========================================================
// NAND Common Interface - Read Function
// ==========================================================
u64 nand_read_data(u8 * buf, u64 offset)
{

    // make sure the block is safe to flash
    offset = nand_find_safe_block(offset);

    if (mtk_nand_read_page_hwecc(offset, buf) == FALSE)
    {
        print("nand_read_data fail\n");
        return -1;
    }

    return offset;
}

// ==========================================================
// NAND Common Interface - Write Function
// ==========================================================
u64 nand_write_data(u8 * buf, u64 offset)
{
    // make sure the block is safe to flash
    offset = nand_find_safe_block(offset);

    if (mtk_nand_write_page_hwecc(offset, buf) == FALSE)
    {
        print("nand_write_data fail\n");
        ASSERT(0);
    }

    return offset;
}

// ==========================================================
// NAND Common Interface - Erase Function
// ==========================================================
bool nand_erase_data(u64 offset, u64 offset_limit, u64 size)
{

    u64 img_size = size;
    u32 tpgsz;
    u32 tblksz;
    u64 cur_offset;
    u32 i = 0;

    // do block alignment check
    if (offset % BLOCK_SIZE != 0)
    {
        print("offset must be block alignment (0x%x)\n", BLOCK_SIZE);
        ASSERT(0);
    }
    // calculate block number of this image
    if ((img_size % BLOCK_SIZE) == 0)
    {
        tblksz = img_size / BLOCK_SIZE;
    } else
    {
        tblksz = (img_size / BLOCK_SIZE) + 1;
    }

    print("[ERASE] image size = 0x%x\n", img_size);
    print("[ERASE] the number of nand block of this image = %d\n", tblksz);

    // erase nand block
    cur_offset = offset;
    while (tblksz != 0)
    {
        if (mtk_nand_erase(cur_offset) == FALSE)
        {
            print("[ERASE] erase fail\n");
            mark_block_bad(cur_offset);
            //ASSERT (0);
        }
        cur_offset += BLOCK_SIZE;

        tblksz--;

        if (tblksz != 0 && cur_offset >= offset_limit)
        {
            print("[ERASE] cur offset (0x%x) exceeds erase limit address (0x%x)\n", cur_offset, offset_limit);
            return TRUE;
        }
    }

    return TRUE;
}

