/*************************************************************************
  ��Ȩ�� ShenZhen ChipSailing Technology Co., Ltd. All rights reserved.
  �ļ�����: chips_common.c
  �ļ�����: ��spi������صĺ����ӿ�
  ����: zwp    ID:58    �汾:2.0   ����:2016/10/16
  ����:
  ��ʷ:
      1. ����:           ����:          ID:
	     �޸�˵��:
	  2.
 *************************************************************************/

 
 /********************************ͷ�ļ�����******************************/
#include <linux/spi/spi.h>
//#include <mt_spi.h>

#include "./inc/chips_main.h"
#include "./inc/chips_common.h"

#define MTK_SPI_ALIGN_MASK_NUM  10
#define MTK_SPI_ALIGN_MASK  ((0x1 << MTK_SPI_ALIGN_MASK_NUM) - 1)
#define	SPI_BUFSIZ	32

/*********************************��������********************************/

static void chips_fp_complete(void *arg)
{
	complete(arg);
}

 /**
 *  @brief chips_sync ͬ��/����SPI���ݴ���
 *  
 *  @param [in] chips_data chips_data�ṹ��ָ��
 *  @param [in] message    spi_message�ṹ��ָ��
 *  
 *  @return �ɹ�����0��ʧ�ܷ��ظ���
 */
static int chips_sync(struct chips_data *chips_data,struct spi_message *message)
{
	
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	if(NULL == chips_data || NULL == message){
		chips_dbg("invalid arguments\n");
		return -EINVAL;
	}
	message->complete = chips_fp_complete;
	message->context = &done;

	spin_lock_irq(&chips_data->spin_lock);
	if (chips_data->spi == NULL){
		status = -ESHUTDOWN;
	}else{
		status = spi_async(chips_data->spi, message);
	}
		
	spin_unlock_irq(&chips_data->spin_lock);

	if (status == 0){
		wait_for_completion(&done);

		status = message->status;
		//chips_dbg("spi_async call success,message->status = %d\n",status);
		//if (status == 0)
		//	status = message->actual_length;
	}else{
		chips_dbg("Failed to async message,status = %d\n",status);
	}
	return status;
}
 

static unsigned char buf[SPI_BUFSIZ] = {0};

/**
 * @func��chips_spi_full_duplex - SPI synchronous write followed by read
 * @txbuf: data to be written (need not be dma-safe)
 * @n_tx: size of txbuf, in bytes
 * @rxbuf: buffer into which data will be read (need not be dma-safe)
 * @n_rx: size of rxbuf, in bytes
 * @return��return 0 on success,negative on failure
 * Context: can sleep
 
 *
 * This performs a half duplex MicroWire style transaction with the
 * device, sending txbuf and then reading rxbuf.  The return value
 * is zero for success, else a negative errno status code.
 * This call may only be used from a context that may sleep.
 *
 * Parameters to this routine are always copied using a small buffer;
 * portable code should never use this for more than 32 bytes.
 * Performance-sensitive or bulk transfer code should instead use
 * spi_{async,sync}() calls with dma-safe buffers.
 */
static int chips_spi_full_duplex(struct chips_data *chips_data,void *txbuf, unsigned n_tx,void *rxbuf, unsigned n_rx)
{
	static DEFINE_MUTEX(lock);

	int			status;
	struct spi_message	message;
	//struct spi_transfer	x[2];
	struct spi_transfer x = {0};  
	unsigned char *local_buf;
	uint32_t package_num = 0;
	uint32_t remainder = 0;
	uint32_t packet_size = 0;

	/* Use preallocated DMA-safe buffer if we can.  We can't avoid
	 * copying here, (as a pure convenience thing), but we can
	 * keep heap costs out of the hot path unless someone else is
	 * using the pre-allocated buffer or the transfer is too large.
	 */
	if((!txbuf && !rxbuf)||(!n_tx && !n_rx)){
		chips_dbg("invalid arguments\n");
		return -EINVAL;
	}
  
    if((n_tx + n_rx) > 32){
		chips_data->spi_mcc->com_mod = DMA_TRANSFER;
	}else{
		chips_data->spi_mcc->com_mod = FIFO_TRANSFER;
	}
	spi_setup(chips_data->spi);

    package_num = (n_tx + n_rx)>>MTK_SPI_ALIGN_MASK_NUM;
	remainder = (n_tx + n_rx) & MTK_SPI_ALIGN_MASK;
	if((package_num > 0) && (remainder != 0)){
		packet_size = ((package_num+1) << MTK_SPI_ALIGN_MASK_NUM);
	}else{
		packet_size = n_tx + n_rx;
	}
	
	if (packet_size > SPI_BUFSIZ || !mutex_trylock(&lock)) {
		local_buf = kmalloc(max((unsigned)SPI_BUFSIZ, packet_size),GFP_KERNEL);  
		if (NULL == local_buf){
			chips_dbg("Failed to allocate mem for spi_full_duplex buffer\n");
			return -ENOMEM;
		}
	} else {
		local_buf = buf;
	}
	
	spi_message_init(&message);
	
	//initialize
	memset(&x,0,sizeof(x));
	memcpy(local_buf,txbuf,n_tx);
	
	x.cs_change = 0;
	x.delay_usecs = 1;
	x.speed_hz = 7000000;
	x.tx_buf = local_buf;
	x.rx_buf = local_buf;
	x.len = packet_size;

	spi_message_add_tail(&x, &message);
	status = chips_sync(chips_data,&message);
	if(status == 0){
		memcpy(rxbuf,local_buf+n_tx,n_rx);
	}else{
	  chips_dbg("Failed to sync message,status = %d\n",status);	
	}
	
	if (x.tx_buf == buf){
		mutex_unlock(&lock);
	}else{
		kfree(local_buf);
	}
		
	return status;
}

#if 0
/**
 *  @brief chips_spi_write spiͬ��д
 *  
 *  @param [in] chips_data chips_data�ṹ��ָ��
 *  @param [in] buf  Ҫд������ݵ�bufferָ��     
 *  @param [in] len  д������ݵ��ֽ���     
 *  
 *  @return �ɹ�����0,ʧ�ܷ��ظ���
 */
static int chips_spi_write(struct chips_data *chips_data,void *buf, int len)
{	
	struct spi_transfer	t = {
			.tx_buf		= buf,
			.len		= len,
		};
	struct spi_message	m;
	
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return chips_sync(chips_data,&m);
}


 /**
 *  @brief chips_spi_read   spiͬ����
 *  
 *  @param [in] chips_data  chips_data�ṹ��ָ��
 *  @param [out] buf  �ɹ�ʱ�洢��ȡ���ݵ�bufferָ��      
 *  @param [in] len  Ҫ��ȡ���ֽ���      
 *  
 *  @return �ɹ�����0,ʧ�ܷ��ظ���
 */
static int chips_spi_read(struct chips_data *chips_data,void *buf, int len)
{
	struct spi_transfer	t = {
			.rx_buf		= buf,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return chips_sync(chips_data,&m);
}

#endif

 /**
 *  @brief chips_sfr_read ��SFR�Ĵ���
 *  
 *  @param [in] addr  �Ĵ�����ʼ��ַ
 *  @param [out] data ����������
 *  @param [in] len   ��ȡ�����ݳ���
 *  
 *  @return �ɹ�����0��ʧ�ܷ��ظ���
 */
int chips_sfr_read(struct chips_data *chips_data,unsigned short addr,unsigned char *data,unsigned short len)
{
	int status = -1;
	unsigned char tx_buf[2] = {0};

	tx_buf[0] = CHIPS_R_SFR;
	tx_buf[1] = (unsigned char)(addr & 0x00FF);

	status = chips_spi_full_duplex(chips_data, tx_buf, 2, data, len);
	if(status < 0){
		chips_dbg("Failed to read SFR from addr = 0x%x,len = %d\n",addr,len);
	}
	
	return status;
}


 /**
 *  @brief chips_sfr_write дSFR�Ĵ���
 *  
 *  @param [in] addr �Ĵ�����ʼ��ַ
 *  @param [in] data д�������
 *  @param [in] len  д������ݳ���
 *  
 *  @return �ɹ�����0��ʧ�ܷ��ظ���
 */
int chips_sfr_write(struct chips_data *chips_data,unsigned short addr,unsigned char *data,unsigned short len)
{
	
	unsigned char *tx_buf;
	int status = -1;

	tx_buf = (unsigned char *)kmalloc(len+2,GFP_KERNEL);
	if(NULL == tx_buf){
		chips_dbg("Failed to allocate mem for write sfr buffer\n");
		return -ENOMEM;
	}
	
	tx_buf[0] = CHIPS_W_SFR;
	tx_buf[1] = (unsigned char)(addr & 0x00FF);
	memcpy(tx_buf+2,data,len);

	status = chips_spi_full_duplex(chips_data,tx_buf,len+2,NULL,0);
	if(status < 0){
		chips_dbg("Failed to write SFR at addr = 0x%x,len = %d\n",addr,len);
	}
	
	if(NULL != tx_buf){
	    kfree(tx_buf);
		tx_buf = NULL;
	}
	
	return status;
}


 /**
 *  @brief chips_sram_read ��SRAM�Ĵ���
 *  
 *  @param [in] addr �Ĵ�����ʼ��ַ
 *  @param [in] data ����������
 *  @param [in] len  ��ȡ�����ݳ���
 *  
 *  @return �ɹ�����0��ʧ�ܷ��ظ���
 */
int chips_sram_read(struct chips_data *chips_data,unsigned short addr,unsigned char *data,unsigned short len)
{
	unsigned char tx_buf[3] = {0};
	int status = -1;
	
	unsigned char *rx_buf;
	rx_buf = (unsigned char *)kmalloc(len+1, GFP_KERNEL);   //first nop
	if(NULL == rx_buf){
		chips_dbg("Failed to allocate mem for read sram buffer\n");
		return -ENOMEM;
	}
	
	tx_buf[0] = CHIPS_R_SRAM;
	tx_buf[1] = (unsigned char)((addr&0xFF00)>>8);
	tx_buf[2] = (unsigned char)(addr&0x00FF);
	
	status = chips_spi_full_duplex(chips_data, tx_buf, 3, rx_buf, len+1);
	if(status ==  0){
		memcpy(data,rx_buf+1,len);
	}else{
		chips_dbg("Failed to read SRAM from addr = 0x%x,len = %d\n",addr,len);
	}
		
	if(NULL != rx_buf){
		kfree(rx_buf);
		rx_buf = NULL;
	}
	
	return status;
}


 /**
 *  @brief chips_sram_write дSRAM�Ĵ���
 *  
 *  @param [in] addr �Ĵ�����ʼ��ַ
 *  @param [in] data д�������
 *  @param [in] len  д������ݳ���
 *  
 *  @return �ɹ�����0��ʧ�ܷ��ظ���
 */
int chips_sram_write(struct chips_data *chips_data,unsigned short addr,unsigned char *data,unsigned short len)
{ 
	unsigned char *tx_buf;
	int status = -1;
	
	tx_buf = (unsigned char *)kmalloc(len+3,GFP_KERNEL);
	if(NULL == tx_buf){
		chips_dbg("Failed to allocate mem for write sram buffer\n");
		return -ENOMEM;
	}

	tx_buf[0] = CHIPS_W_SRAM;
	tx_buf[1] = (unsigned char)((addr&0xFF00)>>8);
	tx_buf[2] = (unsigned char)(addr&0x00FF);        
	memcpy(tx_buf+3,data,len);

	status = chips_spi_full_duplex(chips_data,tx_buf,len+3,NULL,0);
	if(status < 0){
		chips_dbg("Failed to write SRAM at addr = 0x%x,len = %d\n",addr,len);
	}
	
	if(NULL != tx_buf){
		kfree(tx_buf);
		tx_buf = NULL;
	}
	
	return status;
}


 /**
 *  @brief chips_spi_send_cmd ����spi����
 *  
 *  @param [in] cmd spi����
 *  @param [in] len spi�������ݳ���
 *  
 *  @return �ɹ�����0��ʧ�ܷ��ظ���
 */
int chips_spi_send_cmd(struct chips_data *chips_data,unsigned char *cmd,unsigned short len)
{
	int status = -1;
	status = chips_spi_full_duplex(chips_data,cmd,len,NULL,0);
	if(status < 0){
		chips_dbg("Failed to send spi cmd\n");
	}
	
	return status;
}


 /**
 *  @brief chips_write_configs ��IC���ò���
 *  
 *  @param [in] p_param �Ĵ��������ṹ��ָ��
 *  @param [in] num ��������    
 *  
 *  @return �ɹ�����0��ʧ�ܷ��ظ���
 */
int chips_write_configs(struct chips_data *chips_data,struct param *p_param, int num)
{
	struct param param;
	unsigned char data;
	int i = 0;
	int retval = 0;
	unsigned char tx_buf[2] = {0};
	
	for(i = 0; i < num; i++)
	{
		param = p_param[i];
		
		if(param.cmd == CHIPS_W_SFR) {
			data = (unsigned char)(param.data&0x00FF);
			retval = chips_sfr_write(chips_data,param.addr,&data,1);
			if(retval < 0){
				chips_dbg("write config err>1\n");
				return retval;
			}
			chips_dbg("param.cmd = %x,param.addr = %x,param.data = %x\n",param.cmd,param.addr,data);
		}else if(param.cmd == CHIPS_W_SRAM){
		    tx_buf[0] = (unsigned char)(param.data&0x00FF);  //��8λ
	        tx_buf[1] = (unsigned char)((param.data&0xFF00)>>8);  //��8λ
			retval = chips_sram_write(chips_data,param.addr,tx_buf,2);
			if(retval < 0){
				chips_dbg("write config err>2\n");
				return retval;
			}
			chips_dbg("param.cmd = %x,param.addr = %x,param.data = %x\n",param.cmd,param.addr,param.data);
		}else{
			chips_dbg("write config err>3\n");
		}
	}	
	return 0;
}





