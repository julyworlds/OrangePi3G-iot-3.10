/* Copyright (C) 2011-2014 MediaTek Inc.
*
* * This program is free software: you can redistribute it and/or modify it under the terms of the
* * GNU General Public License version 2 as published by the Free Software Foundation.
* *
* * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* * See the GNU General Public License for more details.
* *
* * You should have received a copy of the GNU General Public License along with this program.
* * If not, see <http://www.gnu.org/licenses/>.
* */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <asm/current.h>
#include <asm/uaccess.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/delay.h>
#if WMT_CREATE_NODE_DYNAMIC
#include <linux/device.h>
#endif
#include <linux/printk.h>

#include "stp_exp.h"
#include "wmt_exp.h"

MODULE_LICENSE("Dual BSD/GPL");

#define MTK_BT_HCI 1
#ifdef MTK_BT_HCI
#define MTK_BT_DEBUG 0
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#endif

#define BT_DRIVER_NAME "mtk_stp_BT_chrdev"
#define BT_DEV_MAJOR 192	/* never used number */

#define PFX                         "[MTK-BT] "
#define BT_LOG_DBG                  3
#define BT_LOG_INFO                 2
#define BT_LOG_WARN                 1
#define BT_LOG_ERR                  0

#define COMBO_IOC_BT_HWVER           6

#define COMBO_IOC_MAGIC        0xb0
#define COMBO_IOCTL_FW_ASSERT  _IOWR(COMBO_IOC_MAGIC, 0, void*)
#if WMT_CREATE_NODE_DYNAMIC
struct class *bt_class = NULL;
struct device *bt_dev = NULL;
#endif

static unsigned int gDbgLevel = BT_LOG_INFO;

#define BT_DBG_FUNC(fmt, arg...)	\
	do { if (gDbgLevel >= BT_LOG_DBG)	\
		pr_warn(PFX "%s: "  fmt, __func__ , ##arg);	\
	} while (0)
#define BT_INFO_FUNC(fmt, arg...)	\
	do { if (gDbgLevel >= BT_LOG_INFO)	\
		pr_warn(PFX "%s: "  fmt, __func__ , ##arg);	\
	} while (0)
#define BT_WARN_FUNC(fmt, arg...)	\
	do { if (gDbgLevel >= BT_LOG_WARN)	\
		pr_err(PFX "%s: "  fmt, __func__ , ##arg);	\
	} while (0)
#define BT_ERR_FUNC(fmt, arg...)	\
	do { if (gDbgLevel >= BT_LOG_ERR)	\
		pr_err(PFX "%s: "   fmt, __func__ , ##arg);	\
	} while (0)
#define BT_TRC_FUNC(f)	\
	do { if (gDbgLevel >= BT_LOG_DBG)	\
		pr_info(PFX "<%s> <%d>\n", __func__, __LINE__);	\
	} while (0)

#define VERSION "1.0"
#define BT_NVRAM_CUSTOM_NAME "/data/BT_Addr"

#ifdef MTK_BT_HCI

#define   NUM_REASSEMBLY   32
struct mtk_hci {
	struct hci_dev *hdev;
	struct work_struct work;
	struct sk_buff_head txq;
	struct sk_buff *reassembly[NUM_REASSEMBLY];
};

static struct mtk_hci   mtk_hci;

#endif

static int BT_devs = 1;		/* device count */
static int BT_major = BT_DEV_MAJOR;	/* dynamic allocation */
module_param(BT_major, uint, 0);
static struct cdev BT_cdev;

#define BT_BUFFER_SIZE 2048
static unsigned char i_buf[BT_BUFFER_SIZE];	/* input buffer of read() */
static unsigned char o_buf[BT_BUFFER_SIZE];	/* output buffer of write() */

static struct semaphore wr_mtx, rd_mtx;
static wait_queue_head_t inq;	/* read queues */
static DECLARE_WAIT_QUEUE_HEAD(BT_wq);
static int flag;
static volatile int retflag;

#ifdef MTK_BT_HCI
static int hci_reassembly(struct hci_dev *hdev, int type, void *data,
		int count, __u8 index)
{
	int len = 0;
	int hlen = 0;
	int remain = count;
	struct sk_buff *skb;
	struct bt_skb_cb *scb;

	struct mtk_hci *info = NULL;

	info = hci_get_drvdata(hdev);
	if ( NULL == info ) {
		printk(KERN_ERR "mtk_bt_hci: invalid info point\n");
		return 0;
	}

	if ((type < HCI_ACLDATA_PKT || type > HCI_EVENT_PKT) ||
			index >= NUM_REASSEMBLY)
		return -EILSEQ;

	skb = info->reassembly[index];

	if (!skb) {
		switch (type) {
			case HCI_ACLDATA_PKT:
				len = HCI_MAX_FRAME_SIZE;
				hlen = HCI_ACL_HDR_SIZE;
				break;
			case HCI_EVENT_PKT:
				len = HCI_MAX_EVENT_SIZE;
				hlen = HCI_EVENT_HDR_SIZE;
				break;
			case HCI_SCODATA_PKT:
				len = HCI_MAX_SCO_SIZE;
				hlen = HCI_SCO_HDR_SIZE;
				break;
		}

		skb = bt_skb_alloc(len, GFP_ATOMIC);
		if (!skb)
			return -ENOMEM;

		scb = (void *) skb->cb;
		scb->expect = hlen;
		scb->pkt_type = type;

		info->reassembly[index] = skb;
	}

	while (count) {
		scb = (void *) skb->cb;
		len = min_t(uint, scb->expect, count);

		memcpy(skb_put(skb, len), data, len);

		count -= len;
		data += len;
		scb->expect -= len;
		remain = count;

		switch (type) {
			case HCI_EVENT_PKT:
				if (skb->len == HCI_EVENT_HDR_SIZE) {
					struct hci_event_hdr *h = hci_event_hdr(skb);

					scb->expect = h->plen;

					if (skb_tailroom(skb) < scb->expect) {
						kfree_skb(skb);
						info->reassembly[index] = NULL;
						return -ENOMEM;
					}
				}
				break;

			case HCI_ACLDATA_PKT:
				if (skb->len  == HCI_ACL_HDR_SIZE) {
					struct hci_acl_hdr *h = hci_acl_hdr(skb);

					scb->expect = __le16_to_cpu(h->dlen);

					if (skb_tailroom(skb) < scb->expect) {
						kfree_skb(skb);
						info->reassembly[index] = NULL;
						return -ENOMEM;
					}
				}
				break;

			case HCI_SCODATA_PKT:
				if (skb->len == HCI_SCO_HDR_SIZE) {
					struct hci_sco_hdr *h = hci_sco_hdr(skb);

					scb->expect = h->dlen;

					if (skb_tailroom(skb) < scb->expect) {
						kfree_skb(skb);
						info->reassembly[index] = NULL;
						return -ENOMEM;
					}
				}
				break;
		}

		if (scb->expect == 0) {
			/* Complete frame */

			bt_cb(skb)->pkt_type = type;
			hci_recv_frame(skb);

			info->reassembly[index] = NULL;
			return remain;
		}
	}

	return remain;
}

int _hci_recv_fragment(struct hci_dev *hdev, int type, void *data, int count)
{
	int rem = 0;

	if (type < HCI_ACLDATA_PKT || type > HCI_EVENT_PKT)
		return -EILSEQ;

	while (count) {
		rem = hci_reassembly(hdev, type, data, count, type - 1);
		if (rem < 0)
			return rem;

		data += (count - rem);
		count = rem;
	}

	return rem;
}
#endif

#ifdef MTK_BT_HCI
	void
hex_dump(char *prefix, char *p, int len)
{
	int i;

	pr_err("%s ", prefix);
	for (i = 0; i < len; i++)
		pr_err("%02x ", (*p++ & 0xff));
	pr_err("\n");
}

	static int
mtk_bt_hci_open(struct hci_dev *hdev)
{
	int err = 0;

#if MTK_BT_DEBUG == 1
	pr_err("# %s\n", __func__);
#endif

	err = mtk_wcn_wmt_func_on(WMTDRV_TYPE_BT);
	if (err != MTK_WCN_BOOL_TRUE) {
		pr_err("%s func on failed with %d\n", __func__, err);
		return -ENODEV;
	}

	set_bit(HCI_RUNNING, &hdev->flags);

	mtk_wcn_stp_set_bluez(1);

	return 0;
}

	static int
mtk_bt_hci_close(struct hci_dev *hdev)
{
	int err = 0;

#if MTK_BT_DEBUG == 1
	pr_err("# %s\n", __func__);
#endif

	mtk_wcn_stp_set_bluez(0);

	clear_bit(HCI_RUNNING, &hdev->flags);

	err = mtk_wcn_wmt_func_off(WMTDRV_TYPE_BT);
	if (err != MTK_WCN_BOOL_TRUE) {
		pr_err("%s func off failed with %d\n", __func__, err);
		return -EIO;
	}

	return 0;
}

	static void
mtk_bt_hci_work(struct work_struct *work)
{
	int err;
	struct sk_buff *skb;

#if MTK_BT_DEBUG == 1
	pr_err("# %s\n", __func__);
#endif

	while ((skb = skb_dequeue(&mtk_hci.txq))) {
		skb_push(skb, 1);
		skb->data[0] = bt_cb(skb)->pkt_type;

#if MTK_BT_DEBUG == 1
		hex_dump(">>", skb->data, skb->len);
#endif

		err = mtk_wcn_stp_send_data(skb->data, skb->len, BT_TASK_INDX);
		if (err < 0) {
			pr_err("%s err=%d\n", __func__, err);
			mtk_hci.hdev->stat.err_tx++;
			skb_queue_head(&mtk_hci.txq, skb);
			break;
		}

		mtk_hci.hdev->stat.byte_tx += skb->len;
		kfree_skb(skb);
	}
}

	static int
mtk_bt_hci_send(struct hci_dev *hdev, struct sk_buff *skb)
{
#if MTK_BT_DEBUG == 1
	pr_err("# %s\n", __func__);
#endif

	if (mtk_hci.hdev && !test_bit(HCI_RUNNING, &mtk_hci.hdev->flags))
		return -EBUSY;

	switch (bt_cb(skb)->pkt_type) {
		case HCI_COMMAND_PKT:
			mtk_hci.hdev->stat.cmd_tx++;
			break;

		case HCI_ACLDATA_PKT:
			mtk_hci.hdev->stat.acl_tx++;
			break;

		case HCI_SCODATA_PKT:
			mtk_hci.hdev->stat.sco_tx++;
			break;

		default:
			return -EILSEQ;
	}

	skb_queue_tail(&mtk_hci.txq, skb);
	schedule_work(&mtk_hci.work);

	return 0;
}

	static int
mtk_bt_hci_flush(struct hci_dev *hdev)
{
	pr_err("%s: todo\n", __func__);

	return 0;
}

	static void
mtk_bt_hci_receive(const PUINT8 data, INT32 size)
{
	int err;

#if MTK_BT_DEBUG == 1
	pr_err("# %s\n", __func__);
	hex_dump("<<", data, size);
#endif

	err = _hci_recv_fragment(mtk_hci.hdev, data[0], (void *)&data[1], size - 1);
	if (err < 0)
		pr_err("%s: hci_recv_fragment failed with %d\n", __func__, err);

	if (mtk_hci.hdev)
		mtk_hci.hdev->stat.byte_rx += size - 1;
}

	static void
mtk_bt_hci_notify(struct hci_dev *hdev, unsigned int evt)
{
	static const char * const notify_str[] = {
		"null",
		"HCI_NOTIFY_CONN_ADD",
		"HCI_NOTIFY_CONN_DEL",
		"HCI_NOTIFY_VOICE_SETTING"
	};

	if (evt > HCI_NOTIFY_VOICE_SETTING)
		pr_info("%s event=0x%x\n", __func__, evt);
	else
		pr_info("%s event(%d)=%s\n", __func__, evt, notify_str[evt]);
}
#endif

unsigned char g_bt_bd_addr[10] = { 0x01, 0x1a, 0xfc, 0x06, 0x00, 0x55, 0x66, 0x77, 0x88, 0x00 };

unsigned char g_nvram_btdata[8];

static int nvram_read(char *filename, char *buf, ssize_t len, int offset)
{
	struct file *fd;
	/* ssize_t ret; */
	int retLen = -1;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = filp_open(filename, O_WRONLY | O_CREAT, 0644);

	if (IS_ERR(fd)) {
		BT_ERR_FUNC("failed to open!!\n");
		return -1;
	}
	do {
		if ((fd->f_op == NULL) || (fd->f_op->read == NULL)) {
			BT_ERR_FUNC("file can not be read!!\n");
			break;
		}

		if (fd->f_pos != offset) {
			if (fd->f_op->llseek) {
				if (fd->f_op->llseek(fd, offset, 0) != offset) {
					BT_ERR_FUNC("[nvram_read] : failed to seek!!\n");
					break;
				}
			} else {
				fd->f_pos = offset;
			}
		}

		retLen = fd->f_op->read(fd, buf, len, &fd->f_pos);

	} while (false);

	filp_close(fd, NULL);

	set_fs(old_fs);

	return retLen;
}

int platform_load_nvram_data(char *filename, char *buf, int len)
{
	/* int ret; */
	BT_INFO_FUNC("platform_load_nvram_data ++ BDADDR\n");

	return nvram_read(filename, buf, len, 0);
}

static void bt_cdev_rst_cb(ENUM_WMTDRV_TYPE_T src,
		ENUM_WMTDRV_TYPE_T dst, ENUM_WMTMSG_TYPE_T type, void *buf, unsigned int sz)
{

	/*
	 * 	   To handle reset procedure please
	 * 	   	 */
	ENUM_WMTRSTMSG_TYPE_T rst_msg;

	BT_INFO_FUNC("sizeof(ENUM_WMTRSTMSG_TYPE_T) = %zd\n", sizeof(ENUM_WMTRSTMSG_TYPE_T));
	if (sz <= sizeof(ENUM_WMTRSTMSG_TYPE_T)) {
		memcpy((char *)&rst_msg, (char *)buf, sz);
		BT_INFO_FUNC("src = %d, dst = %d, type = %d, buf = 0x%x sz = %d, max = %d\n", src, dst, type, rst_msg,
				sz, WMTRSTMSG_RESET_MAX);
		if ((src == WMTDRV_TYPE_WMT) && (dst == WMTDRV_TYPE_BT) && (type == WMTMSG_TYPE_RESET)) {
			if (rst_msg == WMTRSTMSG_RESET_START) {
				BT_INFO_FUNC("BT restart start!\n");
				retflag = 1;
				wake_up_interruptible(&inq);
				/*reset_start message handling */

			} else if (rst_msg == WMTRSTMSG_RESET_END) {
				BT_INFO_FUNC("BT restart end!\n");
				retflag = 2;
				wake_up_interruptible(&inq);
				/*reset_end message handling */
			}
		}
	} else {
		/*message format invalid */
		BT_INFO_FUNC("message format invalid!\n");
	}
}

void BT_event_cb(void)
{
	BT_DBG_FUNC("BT_event_cb()\n");

	flag = 1;
	wake_up(&BT_wq);

	/* finally, awake any reader */
	wake_up_interruptible(&inq);	/* blocked in read() and select() */

	return;
}

unsigned int BT_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	/* down(&wr_mtx); */
	/*
	 * 	 * The buffer is circular; it is considered full
	 * 	 	 * if "wp" is right behind "rp". "left" is 0 if the
	 * 	 	 	 * buffer is empty, and it is "1" if it is completely full.
	 * 	 	 	 	 */
	if (mtk_wcn_stp_is_rxqueue_empty(BT_TASK_INDX)) {
		poll_wait(filp, &inq, wait);

		/* empty let select sleep */
		if ((!mtk_wcn_stp_is_rxqueue_empty(BT_TASK_INDX)) || retflag)
			mask |= POLLIN | POLLRDNORM;	/* readable */
	} else {
		mask |= POLLIN | POLLRDNORM;	/* readable */
	}

	/* do we need condition? */
	mask |= POLLOUT | POLLWRNORM;	/* writable */
	/* up(&wr_mtx); */
	return mask;
}

ssize_t BT_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;
	int written = 0;
	down(&wr_mtx);

	BT_DBG_FUNC("%s: count %zd pos %lld\n", __func__, count, *f_pos);
	if (retflag) {
		if (retflag == 1) {
			/* reset start */
			retval = -88;
			BT_INFO_FUNC("MT662x reset Write: start\n");
		} else if (retflag == 2) {
			/* reset end */
			retval = -99;
			BT_INFO_FUNC("MT662x reset Write: end\n");
		}
		goto OUT;
	}

	if (count > 0) {
		int copy_size;

		if (count < BT_BUFFER_SIZE) {
			copy_size = count;
		} else {
			copy_size = BT_BUFFER_SIZE;
			BT_ERR_FUNC(" count > BT_BUFFER_SIZE\n");
		}

		if (copy_from_user(&o_buf[0], &buf[0], copy_size)) {
			retval = -EFAULT;
			goto OUT;
		}
		/* printk("%02x ", val); */

		written = mtk_wcn_stp_send_data(&o_buf[0], copy_size, BT_TASK_INDX);
		if (0 == written) {
			retval = -ENOSPC;
			/*no windowspace is available, native process should not call BT_write with no delay at all */
			BT_ERR_FUNC("target packet length:%zd, write success length:%d, retval = %d.\n", count, written,
					retval);
		} else {
			retval = written;
		}

	} else {
		retval = -EFAULT;
		BT_ERR_FUNC("target packet length:%zd is not allowed, retval = %d.\n", count, retval);
	}

OUT:
	up(&wr_mtx);
	return retval;
}

ssize_t BT_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;

	down(&rd_mtx);

	BT_DBG_FUNC("BT_read(): count %zd pos %lld\n", count, *f_pos);
	if (retflag) {
		if (retflag == 1) {
			/* reset start */
			retval = -88;
			BT_INFO_FUNC("MT662x reset Read: start\n");
		} else if (retflag == 2) {
			/* reset end */
			retval = -99;
			BT_INFO_FUNC("MT662x reset Read: end\n");
		}
		goto OUT;
	}

	if (count > BT_BUFFER_SIZE) {
		count = BT_BUFFER_SIZE;
		BT_ERR_FUNC(" count > BT_BUFFER_SIZE\n");
	}
	retval = mtk_wcn_stp_receive_data(i_buf, count, BT_TASK_INDX);

	while (retval == 0) {
		/* got nothing, wait for STP's signal */

		/*If nonblocking mode, return directly O_NONBLOCK is specified during open() */
		if (filp->f_flags & O_NONBLOCK) {
			BT_DBG_FUNC("Non-blocking BT_read()\n");
			retval = -EAGAIN;
			goto OUT;
		}

		BT_DBG_FUNC("BT_read(): wait_event 1\n");
		wait_event(BT_wq, flag != 0);
		BT_DBG_FUNC("BT_read(): wait_event 2\n");
		flag = 0;
		retval = mtk_wcn_stp_receive_data(i_buf, count, BT_TASK_INDX);
		BT_DBG_FUNC("BT_read(): mtk_wcn_stp_receive_data() = %d\n", retval);
	}

	/* we got something from STP driver */
	if (copy_to_user(buf, i_buf, retval)) {
		retval = -EFAULT;
		goto OUT;
	}

OUT:
	up(&rd_mtx);
	BT_DBG_FUNC("BT_read(): retval = %d\n", retval);
	return retval;
}

/* int BT_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg) */
long BT_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	MTK_WCN_BOOL bRet = MTK_WCN_BOOL_TRUE;

	ENUM_WMTHWVER_TYPE_T hw_ver_sym = WMTHWVER_INVALID;
	BT_DBG_FUNC("BT_ioctl(): cmd (%d)\n", cmd);

	switch (cmd) {
#if 0
		case 0:		/* enable/disable STP */
			/* George: STP is controlled by WMT only */
			/* mtk_wcn_stp_enable(arg); */
			break;
#endif
		case 1:		/* send raw data */
			BT_DBG_FUNC("BT_ioctl(): disable raw data from BT dev\n");
			retval = -EINVAL;
			break;
		case COMBO_IOC_BT_HWVER:
			/*get combo hw version */
			hw_ver_sym = mtk_wcn_wmt_hwver_get();

			BT_INFO_FUNC("BT_ioctl(): get hw version = %d, sizeof(hw_ver_sym) = %zd\n", hw_ver_sym,
					sizeof(hw_ver_sym));
			if (copy_to_user((int __user *)arg, &hw_ver_sym, sizeof(hw_ver_sym)))
				retval = -EFAULT;
			break;

		case COMBO_IOCTL_FW_ASSERT:
			/* BT trigger fw assert for debug */
			BT_INFO_FUNC("BT Set fw assert......,arg(%ld)\n", arg);
			bRet = mtk_wcn_wmt_assert(WMTDRV_TYPE_BT, arg);
			if (bRet == MTK_WCN_BOOL_TRUE) {
				BT_INFO_FUNC("BT Set fw assert OK\n");
				retval = 0;
			} else {
				BT_INFO_FUNC("BT Set fw assert Failed\n");
				retval = (-1000);
			}
			break;
		default:
			retval = -EFAULT;
			BT_DBG_FUNC("BT_ioctl(): unknown cmd (%d)\n", cmd);
			break;
	}

	return retval;
}

static int BT_open(struct inode *inode, struct file *file)
{
	BT_INFO_FUNC("%s: major %d minor %d (pid %d)\n", __func__, imajor(inode), iminor(inode), current->pid);

#if 1				/* GeorgeKuo: turn on function before check stp ready */
	/* turn on BT */
	if (MTK_WCN_BOOL_FALSE == mtk_wcn_wmt_func_on(WMTDRV_TYPE_BT)) {
		BT_WARN_FUNC("WMT turn on BT fail!\n");
		return -ENODEV;
	} else {
		retflag = 0;
		mtk_wcn_wmt_msgcb_reg(WMTDRV_TYPE_BT, bt_cdev_rst_cb);
		BT_INFO_FUNC("WMT register BT rst cb!\n");
	}
#endif

	if (mtk_wcn_stp_is_ready()) {
#if 0				/* GeorgeKuo: turn on function before check stp ready */
		/* turn on BT */
		if (MTK_WCN_BOOL_FALSE == mtk_wcn_wmt_func_on(WMTDRV_TYPE_BT)) {
			BT_WARN_FUNC("WMT turn on BT fail!\n");
			return -ENODEV;
		}
#endif
		mtk_wcn_stp_set_bluez(0);

		BT_INFO_FUNC("Now it's in MTK Bluetooth Mode\n");
		BT_INFO_FUNC("WMT turn on BT OK!\n");
		BT_INFO_FUNC("STP is ready!\n");
		// platform_load_nvram_data(BT_NVRAM_CUSTOM_NAME, (char *)&g_nvram_btdata, sizeof(g_nvram_btdata));
                // 
		// BT_INFO_FUNC("Read NVRAM : BD address %02x%02x%02x%02x%02x%02x Cap 0x%02x Codec 0x%02x\n",
		// 		g_nvram_btdata[0], g_nvram_btdata[1], g_nvram_btdata[2],
		// 		g_nvram_btdata[3], g_nvram_btdata[4], g_nvram_btdata[5],
		// 		g_nvram_btdata[6], g_nvram_btdata[7]);

		mtk_wcn_stp_register_event_cb(BT_TASK_INDX, BT_event_cb);
		BT_INFO_FUNC("mtk_wcn_stp_register_event_cb finish\n");
	} else {
		BT_ERR_FUNC("STP is not ready\n");

		/*return error code */
		return -ENODEV;
	}

	/* init_MUTEX(&wr_mtx); */
	sema_init(&wr_mtx, 1);
	/* init_MUTEX(&rd_mtx); */
	sema_init(&rd_mtx, 1);
	BT_INFO_FUNC("finish\n");

	return 0;
}

static int BT_close(struct inode *inode, struct file *file)
{
	BT_INFO_FUNC("%s: major %d minor %d (pid %d)\n", __func__, imajor(inode), iminor(inode), current->pid);
	retflag = 0;
	mtk_wcn_wmt_msgcb_unreg(WMTDRV_TYPE_BT);
	mtk_wcn_stp_register_event_cb(BT_TASK_INDX, NULL);

	if (MTK_WCN_BOOL_FALSE == mtk_wcn_wmt_func_off(WMTDRV_TYPE_BT)) {
		BT_INFO_FUNC("WMT turn off BT fail!\n");
		return -EIO;	/* mostly, native programmer will not check this return value. */
	} else {
		BT_INFO_FUNC("WMT turn off BT OK!\n");
	}

	return 0;
}

const struct file_operations BT_fops = {
	.open = BT_open,
	.release = BT_close,
	.read = BT_read,
	.write = BT_write,
	/* .ioctl = BT_ioctl, */
	.unlocked_ioctl = BT_unlocked_ioctl,
	.poll = BT_poll
};

static int BT_init(void)
{
	dev_t dev = MKDEV(BT_major, 0);
	int alloc_ret = 0;
	int cdev_err = 0;

	/*static allocate chrdev */
	alloc_ret = register_chrdev_region(dev, 1, BT_DRIVER_NAME);
	if (alloc_ret) {
		BT_ERR_FUNC("fail to register chrdev\n");
		return alloc_ret;
	}

	cdev_init(&BT_cdev, &BT_fops);
	BT_cdev.owner = THIS_MODULE;

	cdev_err = cdev_add(&BT_cdev, dev, BT_devs);
	if (cdev_err)
		goto error;

	BT_INFO_FUNC("%s driver(major %d) installed.\n", BT_DRIVER_NAME, BT_major);
#if WMT_CREATE_NODE_DYNAMIC
	bt_class = class_create(THIS_MODULE, "stpbt");
	if (IS_ERR(bt_class))
		goto error;
	bt_dev = device_create(bt_class, NULL, dev, NULL, "stpbt");
	if (IS_ERR(bt_dev))
		goto error;

#endif
	retflag = 0;

	/* init wait queue */
	init_waitqueue_head(&(inq));

#ifdef MTK_BT_HCI
	mtk_hci.hdev = hci_alloc_dev();
	if (!(mtk_hci.hdev)) {
		BT_ERR_FUNC("%s hci_alloc_dev failed\n", __func__);
		goto error;
	}

	mtk_hci.hdev->bus = HCI_SDIO;
	mtk_hci.hdev->open = mtk_bt_hci_open;
	mtk_hci.hdev->close = mtk_bt_hci_close;
	mtk_hci.hdev->send = mtk_bt_hci_send;
	mtk_hci.hdev->flush = mtk_bt_hci_flush;
	mtk_hci.hdev->notify = mtk_bt_hci_notify;
	SET_HCIDEV_DEV(mtk_hci.hdev, bt_dev);
	hci_set_drvdata(mtk_hci.hdev, &mtk_hci);

	mtk_wcn_stp_register_if_rx(mtk_bt_hci_receive);

	INT32 hci_err = hci_register_dev(mtk_hci.hdev);
	if (hci_err) {
		BT_ERR_FUNC("%s hci_register_dev failed with %d\n", __func__, hci_err);
		hci_free_dev(mtk_hci.hdev);
		goto error;
	}

	skb_queue_head_init(&mtk_hci.txq);
	INIT_WORK(&mtk_hci.work, mtk_bt_hci_work);
#endif

	return 0;

error:
#if WMT_CREATE_NODE_DYNAMIC
	if (!(IS_ERR(bt_dev)))
		device_destroy(bt_class, dev);
	if (!(IS_ERR(bt_class))) {
		class_destroy(bt_class);
		bt_class = NULL;
	}
#endif
	if (cdev_err == 0)
		cdev_del(&BT_cdev);

	if (alloc_ret == 0)
		unregister_chrdev_region(dev, BT_devs);

	return -1;
}

static void BT_exit(void)
{
	dev_t dev = MKDEV(BT_major, 0);
	retflag = 0;
	//mtk_wcn_stp_register_event_cb(BT_TASK_INDX, NULL);	/* unregister event callback function */
#if WMT_CREATE_NODE_DYNAMIC
	if (bt_dev) {
		device_destroy(bt_class, dev);
		bt_dev = NULL;
	}
	if (bt_class) {
		class_destroy(bt_class);
		bt_class = NULL;
	}
#endif
	cdev_del(&BT_cdev);
	unregister_chrdev_region(dev, BT_devs);

	BT_INFO_FUNC("%s driver removed.\n", BT_DRIVER_NAME);
}

#ifdef MTK_WCN_REMOVE_KERNEL_MODULE

int mtk_wcn_stpbt_drv_init(void)
{
	return 0;
	return BT_init();

}
EXPORT_SYMBOL(mtk_wcn_stpbt_drv_init);

int mtk_wcn_stpbt_drv_init_fix(void)
{
	return BT_init();

}
EXPORT_SYMBOL(mtk_wcn_stpbt_drv_init_fix);

void mtk_wcn_stpbt_drv_exit(void)
{
	return BT_exit();
}
EXPORT_SYMBOL(mtk_wcn_stpbt_drv_exit);

#else

module_init(BT_init);
module_exit(BT_exit);

#endif

