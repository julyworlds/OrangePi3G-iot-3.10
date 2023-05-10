#ifndef __CCCI_CORE_H__
#define __CCCI_CORE_H__

#include <linux/wait.h>
#include <linux/skbuff.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/netdevice.h>
#include <linux/wakelock.h>
#include "ccci_debug.h"
#include "ccci_errno.h"

#define CCCI_DEV_NAME "ccci"
#define CCCI_MTU  (4096-128) //FIXME: (3584-128)
#define CCMNI_MTU 1500
#define CCCI_MAGIC_NUM 0xFFFFFFFF

/* enumerations and marcos */
struct ccci_header{
	u32 data[2]; // do NOT assump data[1] is data length in Rx
	u32 channel;
	u32 reserved;
} __attribute__ ((packed)); // not necessary, but it's a good gesture, :)

typedef enum {
	CCCI_CONTROL_RX = 0,
	CCCI_CONTROL_TX = 1,
	CCCI_SYSTEM_RX = 2,
	CCCI_SYSTEM_TX = 3,
	CCCI_PCM_RX = 4,
	CCCI_PCM_TX = 5,
	CCCI_UART1_RX = 6, // META
	CCCI_UART1_RX_ACK = 7,
	CCCI_UART1_TX = 8,
	CCCI_UART1_TX_ACK = 9,
	CCCI_UART2_RX = 10, // MUX
	CCCI_UART2_RX_ACK = 11,
	CCCI_UART2_TX = 12,
	CCCI_UART2_TX_ACK = 13,
	CCCI_FS_RX = 14,
	CCCI_FS_TX = 15,
	CCCI_PMIC_RX = 16,
	CCCI_PMIC_TX = 17,
	CCCI_UEM_RX = 18,
	CCCI_UEM_TX = 19,
	CCCI_CCMNI1_RX = 20,
	CCCI_CCMNI1_RX_ACK = 21,
	CCCI_CCMNI1_TX = 22,
	CCCI_CCMNI1_TX_ACK = 23,
	CCCI_CCMNI2_RX = 24,
	CCCI_CCMNI2_RX_ACK = 25,
	CCCI_CCMNI2_TX = 26,
	CCCI_CCMNI2_TX_ACK = 27,
	CCCI_CCMNI3_RX = 28,
	CCCI_CCMNI3_RX_ACK = 29,
	CCCI_CCMNI3_TX = 30,
	CCCI_CCMNI3_TX_ACK = 31,
	CCCI_RPC_RX = 32,
	CCCI_RPC_TX = 33,
	CCCI_IPC_RX = 34,
	CCCI_IPC_RX_ACK = 35,
	CCCI_IPC_TX = 36,
	CCCI_IPC_TX_ACK = 37,
	CCCI_IPC_UART_RX = 38,
	CCCI_IPC_UART_RX_ACK = 39,
	CCCI_IPC_UART_TX = 40,
	CCCI_IPC_UART_TX_ACK = 41,
	CCCI_MD_LOG_RX = 42,
	CCCI_MD_LOG_TX = 43,
	/* ch44~49 reserved for ARM7 */
	CCCI_IT_RX = 50,
	CCCI_IT_TX = 51,
	CCCI_IMSV_UL = 52,
    CCCI_IMSV_DL = 53,
    CCCI_IMSC_UL = 54,
    CCCI_IMSC_DL = 55,
    CCCI_IMSA_UL = 56,
    CCCI_IMSA_DL = 57,
	CCCI_IMSDC_UL = 58,
	CCCI_IMSDC_DL = 59,
	CCCI_ICUSB_RX = 60,
	CCCI_ICUSB_TX = 61,
	CCCI_LB_IT_RX = 62,
	CCCI_LB_IT_TX = 63,
	
	CCCI_MONITOR_CH,
	CCCI_DUMMY_CH,
	CCCI_MAX_CH_NUM, // RX channel ID should NOT be >= this!!

	CCCI_MONITOR_CH_ID = 0xf0000000, // for backward compatible
	CCCI_FORCE_ASSERT_CH = 20090215,
	CCCI_INVALID_CH_ID = 0xffffffff,
}CCCI_CH;

/* AP->md_init messages on monitor channel */
typedef enum {
	CCCI_MD_MSG_BOOT_READY			= 0xFAF50001,
	CCCI_MD_MSG_BOOT_UP				= 0xFAF50002,
	CCCI_MD_MSG_EXCEPTION			= 0xFAF50003,
	CCCI_MD_MSG_RESET				= 0xFAF50004,
	CCCI_MD_MSG_RESET_RETRY			= 0xFAF50005,
	CCCI_MD_MSG_READY_TO_RESET		= 0xFAF50006,
	CCCI_MD_MSG_BOOT_TIMEOUT		= 0xFAF50007,
	CCCI_MD_MSG_STOP_MD_REQUEST		= 0xFAF50008,
	CCCI_MD_MSG_START_MD_REQUEST	= 0xFAF50009,
	CCCI_MD_MSG_ENTER_FLIGHT_MODE	= 0xFAF5000A,
	CCCI_MD_MSG_LEAVE_FLIGHT_MODE	= 0xFAF5000B,
	CCCI_MD_MSG_POWER_ON_REQUEST	= 0xFAF5000C,
	CCCI_MD_MSG_POWER_OFF_REQUEST	= 0xFAF5000D,
	CCCI_MD_MSG_SEND_BATTERY_INFO   = 0xFAF5000E,
	CCCI_MD_MSG_NOTIFY				= 0xFAF5000F,
	CCCI_MD_MSG_STORE_NVRAM_MD_TYPE = 0xFAF50010,
} CCCI_MD_MSG;

/* AP<->MD messages on system channel */
enum { 
	MD_INIT_START_BOOT = 0x00000000, 
	MD_INIT_CHK_ID = 0x5555FFFF,
	MD_EX = 0x00000004, 
	MD_EX_CHK_ID = 0x45584350,
	MD_EX_REC_OK = 0x00000006, 
	MD_EX_REC_OK_CHK_ID = 0x45524543, 
	MD_EX_RESUME_CHK_ID = 0x7, 
	CCCI_DRV_VER_ERROR = 0x5,

	// System channel, AP->MD || AP<-->MD message start from 0x100
	MD_DORMANT_NOTIFY = 0x100,
	MD_SLP_REQUEST = 0x101,
	MD_TX_POWER = 0x102,
	MD_RF_TEMPERATURE = 0x103,
	MD_RF_TEMPERATURE_3G = 0x104,
	MD_GET_BATTERY_INFO = 0x105,
	MD_SIM_TYPE = 0x107,
	MD_ICUSB_NOTIFY = 0x108,
	// System channel, MD --> AP message start from 0x1000
	MD_WDT_MONITOR = 0x1000,
	// System channel, AP --> MD message
	MD_WAKEN_UP = 0x10000,
};

typedef enum {
	ID_GET_MD_WAKEUP_SRC = 0,   // for SPM
	ID_CCCI_DORMANCY = 1,       // abandoned
    ID_LOCK_MD_SLEEP = 2,       // abandoned
	ID_ACK_MD_SLEEP = 3,        // abandoned
	ID_SSW_SWITCH_MODE = 4,     // abandoned
	ID_SET_MD_TX_LEVEL = 5,     // abandoned
	ID_GET_TXPOWER = 6,			// for thermal
	ID_IPO_H_RESTORE_CB = 7,    // abandoned
	ID_FORCE_MD_ASSERT = 8,     // abandoned
}KERN_FUNC_ID;

#define NORMAL_BOOT_ID 0
#define META_BOOT_ID 1

typedef enum {
	INVALID = 0,
	GATED, // broadcast by modem driver
	BOOTING, // broadcast by port_kernel
	READY, // broadcast by port_kernel
	EXCEPTION, // broadcast by port_kernel
	RESET, // broadcast by modem driver
	RX_IRQ, // broadcast by modem driver, illegal for md->md_state
}MD_STATE;

typedef enum { 
	MD_BOOT_STAGE_0 = 0, 
	MD_BOOT_STAGE_1 = 1, 
	MD_BOOT_STAGE_2 = 2,
	MD_BOOT_STAGE_EXCEPTION = 3
}MD_BOOT_STAGE;

typedef enum {
	EX_NONE = 0,
	EX_INIT,
	EX_DHL_DL_RDY,
	EX_INIT_DONE,
}MD_EX_STAGE;

/*
 * all code owned by CCCI should use modem index starts from ZERO
 */
typedef enum {
	MD_SYS1 = 0, // MD SYS name counts from 1, but internal index counts from 0
	MD_SYS2,
	MD_SYS5 = 4,
	MAX_MD_NUM
}MD_SYS;

/* MODEM MAUI Exception header (4 bytes)*/
typedef struct _exception_record_header_t {
	u8  ex_type;
	u8  ex_nvram;
	u16 ex_serial_num;
}EX_HEADER_T; // __attribute__ ((packed));

/* MODEM MAUI Environment information (164 bytes) */
typedef struct _ex_environment_info_t {
	u8  boot_mode;
	u8 reserved1[8];
	u8 execution_unit[8];
	u8 reserved2[147];
}EX_ENVINFO_T; // __attribute__ ((packed));

/* MODEM MAUI Special for fatal error (8 bytes)*/
typedef struct _ex_fatalerror_code_t {
	u32 code1;
	u32 code2;
}EX_FATALERR_CODE_T; // __attribute__ ((packed));

/* MODEM MAUI fatal error (296 bytes)*/
typedef struct _ex_fatalerror_t {
	EX_FATALERR_CODE_T error_code;
	u8 reserved1[288];
}EX_FATALERR_T; // __attribute__ ((packed));

/* MODEM MAUI Assert fail (296 bytes)*/
typedef struct _ex_assert_fail_t {
	u8 filename[24];
	u32  linenumber;
	u32  parameters[3];
	u8 reserved1[256];
}EX_ASSERTFAIL_T; // __attribute__ ((packed));

/* MODEM MAUI Globally exported data structure (300 bytes) */
typedef union {
	EX_FATALERR_T fatalerr;
	EX_ASSERTFAIL_T assert;
}EX_CONTENT_T; // __attribute__ ((packed));

/* MODEM MAUI Standard structure of an exception log ( */
typedef struct _ex_exception_log_t {
	EX_HEADER_T	header;
	u8	reserved1[12];
	EX_ENVINFO_T	envinfo;
	u8	reserved2[36];
	EX_CONTENT_T	content;
}EX_LOG_T; // __attribute__ ((packed));

typedef enum {
	IDLE = 0,
	FLYING,
	PARTIAL_READ,
	ERROR,
}REQ_STATE;

typedef enum {
	IN = 0,
	OUT
}DIRECTION;

/*
 * This tells request free routine how it handles skb.
 * The CCCI request structure will always be recycled, but its skb can have different policy.
 * CCCI request can work as just a wrapper, due to netowork subsys will handler skb itself. 
 * Tx: policy is determined by sender;
 * Rx: policy is determined by receiver;
 */
typedef enum {
	NOOP = 0, // don't handle the skb, just recycle the reqeust wrapper
	RECYCLE, // put the skb back into our pool
	FREE, // simply free the skb
}DATA_POLICY;

// core classes
#define CCCI_REQUEST_TRACE_DEPTH 3
struct ccci_request{
	DIRECTION dir;
	DATA_POLICY policy;
	char blocking; // only for Tx
	struct list_head entry;
	REQ_STATE state; // only update by buffer manager

	void *gpd; // virtual address for CPU
	dma_addr_t gpd_addr; // physical address for DMA
	struct sk_buff *skb;
	
#ifdef CCCI_STATISTIC
	ktime_t time_stamp;
	unsigned char time_step;
	unsigned int time_trace[CCCI_REQUEST_TRACE_DEPTH];
#endif
};

#ifdef CCCI_STATISTIC
struct ccci_statistic
{
	unsigned int tx_req_count;
	unsigned int rx_req_count;
	unsigned long long tx_req_time_trace[CCCI_REQUEST_TRACE_DEPTH];
	unsigned long long rx_req_time_trace[CCCI_REQUEST_TRACE_DEPTH];
	unsigned int alloc_count;
	unsigned int alloc_empty_count;
};
#endif

struct ccci_modem;
struct ccci_port;

struct ccci_port_ops {
	// must-have
	int (*init)(struct ccci_port *port);
	int (*recv_request)(struct ccci_port *port, struct ccci_request* req);
	// optional
	int (*req_match)(struct ccci_port *port, struct ccci_request* req);
	void (*md_state_notice)(struct ccci_port *port, MD_STATE state);
};

struct ccci_port {
	// identity
	CCCI_CH tx_ch;
	CCCI_CH rx_ch;
	/*
	 * 0xFF is used as invalid index number,  all virtual ports should use queue 0, but not 0xFF.
	 * always access queue index by using PORT_TXQ_INDEX and PORT_RXQ_INDEX macros
	 */
	unsigned char txq_index; 
	unsigned char rxq_index;
	unsigned char txq_exp_index;
	unsigned char rxq_exp_index;
	unsigned char flags;
	struct ccci_port_ops *ops;
	// device node related
	unsigned int major;
	unsigned int minor;
	char *name;
	// un-initiallized in defination, always put them at the end
	struct ccci_modem *modem;
	void *private_data;
	atomic_t usage_cnt;
	struct list_head entry;
	/*
	 * the Tx and Rx flow are asymmetric due to ports are mutilplexed on queues.
	 * Tx: data block are sent directly to queue's list, so port won't maitain a Tx list. It only
	      provide a wait_queue_head for blocking write.
	 * Rx: due to modem needs to dispatch Rx packet as quickly as possible, so port needs a
	 *    Rx list to hold packets.
	 */
	struct list_head rx_req_list;
	spinlock_t rx_req_lock;
	wait_queue_head_t rx_wq; // for uplayer user
	int rx_length;
	int rx_length_th;
	struct wake_lock rx_wakelock;
};
#define PORT_F_ALLOW_DROP 	(1<<0) // packet will be dropped if port's Rx buffer full
#define PORT_F_RX_FULLED 	(1<<1) // rx buffer has been full once
#define PORT_F_USER_HEADER 	(1<<2) // CCCI header will be provided by user, but not by CCCI
#define PORT_F_RX_EXCLUSIVE	(1<<3) // Rx queue only has this one port

struct ccci_modem_cfg {
	unsigned int load_type;
	unsigned int load_type_saving;
	unsigned int setting;
};
#define MD_SETTING_ENABLE (1<<0)
#define MD_SETTING_RELOAD (1<<1)
#define MD_SETTING_FIRST_BOOT (1<<2) // this is the first time of boot up
#define MD_SETTING_STOP_RETRY_BOOT (1<<3)
#define MD_SETTING_DUMMY  (1<<7)

typedef enum {
	IMG_MD = 0,
	IMG_DSP,
	IMG_NUM,
} MD_IMG_TYPE;

typedef enum{
	INVALID_VARSION = 0,
	DEBUG_VERSION,
	RELEASE_VERSION
} PRODUCT_VER_TYPE;

#define IMG_NAME_LEN 32
#define IMG_POSTFIX_LEN 12
#define IMG_PATH_LEN 64

struct IMG_CHECK_INFO{
	char *product_ver;	/* debug/release/invalid */
	char *image_type;	/*2G/3G/invalid*/
    char *platform;	    /* MT6573_S00(MT6573E1) or MT6573_S01(MT6573E2) */
	char *build_time;	/* build time string */
	char *build_ver;	/* project version, ex:11A_MD.W11.28 */
	unsigned int mem_size; /*md rom+ram mem size*/
	unsigned int md_img_size; /*modem image actual size, exclude head size*/
	PRODUCT_VER_TYPE version;
};

struct ccci_image_info
{
	MD_IMG_TYPE type;
	char file_name[IMG_PATH_LEN];
	phys_addr_t address; // phy memory address to load this image
	ssize_t size; // image size without signature, cipher and check header, read form check header
	loff_t offset; // signature and cipher header
	unsigned int tail_length; // signature tail
	char *ap_platform;
	struct IMG_CHECK_INFO img_info; // read from MD image header
	struct IMG_CHECK_INFO ap_info; // get from AP side configuration
};

struct ccci_mem_layout // all from AP view, AP has no haredware remap after MT6592
{
	// MD image
	void __iomem*	md_region_vir;
	phys_addr_t		md_region_phy;
	unsigned int	md_region_size;
	// DSP image
	void __iomem*	dsp_region_vir;
	phys_addr_t		dsp_region_phy;
	unsigned int	dsp_region_size;
	// Share memory
	void __iomem*	smem_region_vir;
	phys_addr_t		smem_region_phy;
	unsigned int	smem_region_size;
	unsigned int	smem_offset_AP_to_MD; // offset between AP and MD view of share memory
};

struct ccci_smem_layout
{
	// -- Exception
	void __iomem*		ccci_exp_smem_base_vir;
	phys_addr_t			ccci_exp_smem_base_phy;
	unsigned int		ccci_exp_smem_size;
};

struct ccci_modem_ops {
	// must-have
	int (*init)(struct ccci_modem *md);
	int (*start)(struct ccci_modem *md);
	int (*reset)(struct ccci_modem *md); // as pre-stop
	int (*stop)(struct ccci_modem *md, unsigned int timeout);
	int (*send_request)(struct ccci_modem *md, unsigned char qno, struct ccci_request *req);
	int (*give_more)(struct ccci_modem *md, unsigned char qno);
	int (*write_room)(struct ccci_modem *md, unsigned char qno);
	int (*napi_poll)(struct ccci_modem *md, unsigned char qno, struct napi_struct *napi ,int budget);
	int (*send_runtime_data)(struct ccci_modem *md);
	int (*broadcast_state)(struct ccci_modem *md, MD_STATE state);
	int (*force_assert)(struct ccci_modem *md);
	struct ccci_port* (*get_port_by_minor)(struct ccci_modem *md, int minor);
	/*
	 * here we assume Rx and Tx channels are in the same address space,
	 * and Rx channel should be check first, so user can save one comparison if it always sends
	 * in Rx channel ID to identify a port.
	 */
	struct ccci_port* (*get_port_by_channel)(struct ccci_modem *md, CCCI_CH ch);
};

struct ccci_modem {
	unsigned char index;
	unsigned char port_number;
	char post_fix[IMG_POSTFIX_LEN];
	unsigned int major;
	struct ccci_modem_ops *ops;
	struct ccci_port *ports;
	struct list_head ch_ports[CCCI_MAX_CH_NUM];
	struct list_head entry;
	struct kobject kobj;
	struct ccci_mem_layout mem_layout;
	struct ccci_smem_layout smem_layout;
	struct ccci_image_info img_info[IMG_NUM];
	unsigned int sim_type;
	atomic_t wakeup_src;
	
	/*
	 * the following members are readonly for CCCI core. they are maintained by modem and 
	 * port_kernel.c.
	 * port_kernel.c should not be considered as part of CCCI core, we just move common part
	 * of modem message handling into this file. current modem all follows the same message
	 * protocol during bootup and exception. if future modem abandoned this protocl, we can
	 * simply replace function set of kernel port to support it.
	 */
	MD_STATE md_state;
	MD_BOOT_STAGE boot_stage;
	MD_EX_STAGE ex_stage;
	struct ccci_modem_cfg config;
	struct timer_list bootup_timer;
	struct timer_list ex_monitor;
	spinlock_t ctrl_lock;
	volatile unsigned int ee_info_flag;
	unsigned char ex_type;
	EX_LOG_T ex_info;
	
	unsigned char *private_data;
	//unsigned char private_data[0]; // do NOT use this manner, otherwise spinlock inside private_data will trigger alignment exception
};

// APIs
static void inline ccci_setup_channel_mapping(struct ccci_modem *md)
{
	int i;
	struct ccci_port *port = NULL;
	// setup mapping
	for(i=0; i<ARRAY_SIZE(md->ch_ports); i++) {
		INIT_LIST_HEAD(&md->ch_ports[i]); // clear original list
	}
	for(i=0; i<md->port_number; i++) {
		list_add_tail(&md->ports[i].entry, &md->ch_ports[md->ports[i].rx_ch]);
	}
	for(i=0; i<ARRAY_SIZE(md->ch_ports); i++) {
		if(!list_empty(&md->ch_ports[i])) {
			CCCI_INF_MSG(md->index, CORE, "CH%d ports:", i);
			list_for_each_entry(port, &md->ch_ports[i], entry) {
				printk("%s (%d/%d)", port->name, port->rx_ch, port->tx_ch);
			}
			printk("\n");
		}
	}
}

#define PORT_TXQ_INDEX(p) ((p)->modem->md_state==EXCEPTION?(p)->txq_exp_index:(p)->txq_index)
#define PORT_RXQ_INDEX(p) ((p)->modem->md_state==EXCEPTION?(p)->rxq_exp_index:(p)->rxq_index)

/*
 * if send_request returns 0, then it's modem driver's duty to free the request, and caller should NOT reference the 
 * request any more. but if it returns error, calller should be responsible to free the request.
 */
static int inline ccci_port_send_request(struct ccci_port *port, struct ccci_request *req)
{
	struct ccci_modem *md = port->modem;
	return md->ops->send_request(md, PORT_TXQ_INDEX(port), req);
}

/*
 * if recv_request returns 0 or -CCCI_ERR_DROP_PACKET, then it's port's duty to free the request, and caller should
 * NOT reference the request any more. but if it returns other error, caller should be responsible to free the request.
 */
static int inline ccci_port_recv_request(struct ccci_modem *md, struct ccci_request *req)
{
	struct ccci_header *ccci_h = (struct ccci_header *)req->skb->data;
	struct ccci_port *port = NULL;
	struct list_head *port_list = NULL;
	int ret = -CCCI_ERR_CHANNEL_NUM_MIS_MATCH;

	if(unlikely(ccci_h->channel >= CCCI_MAX_CH_NUM)) {
		return -CCCI_ERR_INVALID_LOGIC_CHANNEL_ID;
	}
	
	port_list = &md->ch_ports[ccci_h->channel];
	list_for_each_entry(port, port_list, entry) {
		if(req->state == FLYING) { // the request may be freed by the 1st matching port
			if(likely(!port->ops->req_match)) {
				if(ccci_h->channel == port->rx_ch)
					ret = port->ops->recv_request(port, req);
			} else {
				if(port->ops->req_match(port, req))
					ret = port->ops->recv_request(port, req);
			}
			if(ret != -CCCI_ERR_CHANNEL_NUM_MIS_MATCH) 
				/*
				 * as we don't have ref. count design in ccci_request now, multi-cast is not supported,
				 * because one port may freed this request before another port can process it.
				 */
				break;
		}
	}
	return ret;
}

/*
 * caller should lock with port->rx_req_lock
 */
static int inline ccci_port_ask_more_request(struct ccci_port *port)
{
	struct ccci_modem *md = port->modem;
	int ret;
	
	if(port->flags & PORT_F_RX_FULLED)
		ret = md->ops->give_more(port->modem, PORT_RXQ_INDEX(port));
	else
		ret = -1;
	return ret;
}

#ifdef CCCI_STATISTIC
static inline void ccci_update_request_stamp(struct ccci_request *req)
{
	ktime_t curr = ktime_get_real();
	if(req->time_step<CCCI_REQUEST_TRACE_DEPTH) {
		req->time_trace[req->time_step++] = curr.tv64 - req->time_stamp.tv64;
		req->time_stamp = curr;
	}
}
#endif

struct ccci_modem *ccci_allocate_modem(int private_size, void (*setup)(struct ccci_modem *md));
int ccci_register_modem(struct ccci_modem *modem);
int ccci_register_dev_node(const char *name, int major_id, int minor);
struct ccci_port *ccci_get_port_for_node(int major, int minor);
int ccci_send_msg_to_md(struct ccci_modem *md, CCCI_CH ch, CCCI_MD_MSG msg, u32 resv);
int ccci_send_virtual_md_msg(struct ccci_modem *md, CCCI_CH ch, CCCI_MD_MSG msg, u32 resv);
#ifdef CCCI_STATISTIC
void ccci_update_request_statistic(struct ccci_request *req);
#endif
extern void ccci_md_exception_notify(struct ccci_modem *md, MD_EX_STAGE stage);

// structure initialize
static void inline ccci_port_struct_init(struct ccci_port *port, struct ccci_modem *md)
{
	INIT_LIST_HEAD(&port->rx_req_list);
	spin_lock_init(&port->rx_req_lock);
	INIT_LIST_HEAD(&port->entry);
	init_waitqueue_head(&port->rx_wq);
	port->rx_length = 0;
	atomic_set(&port->usage_cnt, 0);
	port->modem = md;
	wake_lock_init(&port->rx_wakelock, WAKE_LOCK_SUSPEND, port->name);
}

/*
 * only used during allocate buffer pool, should NOT be used after allocated a request
 */
static void inline ccci_request_struct_init(struct ccci_request *req)
{
	req->skb = NULL;
	req->state = IDLE;
	req->policy = FREE;
	/*
	 * as this request is not in any list, but pay ATTENTION, this will cause list_add(req) fail due
	 * to it's not pointing to itself.
	 */
	req->entry.next = LIST_POISON1;
	req->entry.prev = LIST_POISON2;
}

#endif // __CCCI_CORE_H__
