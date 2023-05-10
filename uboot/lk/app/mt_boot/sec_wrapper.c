/* MTK Proprietary Wrapper File */

#include <platform/mt_typedefs.h>
#include <platform/sec_status.h>
#include <mt_partition.h>
#include <platform/errno.h>
#include <debug.h>


#ifndef MTK_EMMC_SUPPORT
extern u32 mtk_nand_erasesize(void);
extern int nand_erase(u64 offset, u64 size);
#endif

char* sec_dev_get_part_r_name(u32 index)
{
    return g_part_name_map[index].r_name;
}


int sec_dev_read_wrapper(char *part_name, u64 offset, u8* data, u32 size)
{
    part_dev_t *dev;
    long len;
#ifdef MTK_EMMC_SUPPORT
#ifdef MTK_NEW_COMBO_EMMC_SUPPORT
    part_t *part;
#endif
#endif

    dev = mt_part_get_device();    
   	if (!dev){	
   	    return PART_GET_DEV_FAIL;
   	}


#ifdef MTK_EMMC_SUPPORT
#ifdef MTK_NEW_COMBO_EMMC_SUPPORT
        part = mt_part_get_partition(part_name);
        if (!part){    
            dprintf(CRITICAL,"[read_wrapper] mt_part_get_partition error, name: %s\n", part_name);
            ASSERT(0);
        }
        len = dev->read(dev, offset, (uchar *) data, size, part->part_id);
#else
    len = dev->read(dev, offset, (uchar *) data, size);
#endif
#else
	#ifdef MTK_NEW_COMBO_EMMC_SUPPORT
	len = dev->read(dev, offset, (uchar *) data, size, NAND_PART_USER);
	#else
    len = dev->read(dev, offset, (uchar *) data, size);
	#endif
#endif
    if (len != (int)size)
    {
        return PART_READ_FAIL;
    }

    return B_OK;
}

int sec_dev_write_wrapper(char *part_name, u64 offset, u8* data, u32 size)
{
    part_dev_t *dev;
    long len;
#ifdef MTK_EMMC_SUPPORT
#ifdef MTK_NEW_COMBO_EMMC_SUPPORT
    part_t *part;
#endif
#endif

    dev = mt_part_get_device();    
   	if (!dev)	
   	    return PART_GET_DEV_FAIL;


#ifndef MTK_EMMC_SUPPORT
    if(nand_erase(offset,(u64)size)!=0){
        return PART_ERASE_FAIL;
    }
#endif    

#ifdef MTK_EMMC_SUPPORT
#ifdef MTK_NEW_COMBO_EMMC_SUPPORT
   	part = mt_part_get_partition(part_name);
   	if (!part){    
   	    dprintf(CRITICAL,"[write_wrapper] mt_part_get_partition error, name: %s\n", part_name);
   	    ASSERT(0);
    }

    len = dev->write(dev, (uchar *) data, offset, size, part->part_id);
#else
    len = dev->write(dev, (uchar *) data, offset, size);
#endif
#else
	#ifdef MTK_NEW_COMBO_EMMC_SUPPORT
	len = dev->write(dev, (uchar *) data, offset, size, NAND_PART_USER);
	#else
    len = dev->write(dev, (uchar *) data, offset, size);
	#endif
#endif
    if (len != (int)size)
    {
        return PART_WRITE_FAIL;
    }

    return B_OK;
}

unsigned int sec_dev_nand_erase_size(void){

#ifdef MTK_EMMC_SUPPORT
    return 0;
#else
    return mtk_nand_erasesize();
#endif

}

