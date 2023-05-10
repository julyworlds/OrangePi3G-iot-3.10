#define NAND_ABTC_ATAG
#define NAND_MAX_ID		7
#define CHIP_CNT		2
#define RAMDOM_READ		(1<<0)
#define CACHE_READ		(1<<1)

#define ATAG_FLASH_NUMBER_INFO       0x54430006
#define ATAG_FLASH_INFO       0x54430007
struct tag_nand_number {
	u32 number;
};

typedef struct
{
   u8 id[NAND_MAX_ID];
   u8 id_length;
   u8 addr_cycle;
   u8 iowidth;
   u16 totalsize;
   u16 blocksize;
   u16 pagesize;
   u16 sparesize;
   u32 timmingsetting;
   u8 devciename[30];
   u32 advancedmode;
}flashdev_info_t;


