#
# Makefile for the Linux Bluetooth HCI device drivers.
#

obj-$(CONFIG_BACKPORT_BT_HCIVHCI)	+= hci_vhci.o
obj-$(CONFIG_BACKPORT_BT_HCIUART)	+= hci_uart.o
obj-$(CONFIG_BACKPORT_BT_HCIBCM203X)	+= bcm203x.o
obj-$(CONFIG_BACKPORT_BT_HCIBPA10X)	+= bpa10x.o
obj-$(CONFIG_BACKPORT_BT_HCIBFUSB)	+= bfusb.o
obj-$(CONFIG_BACKPORT_BT_HCIDTL1)	+= dtl1_cs.o
obj-$(CONFIG_BACKPORT_BT_HCIBT3C)	+= bt3c_cs.o
obj-$(CONFIG_BACKPORT_BT_HCIBLUECARD)	+= bluecard_cs.o
obj-$(CONFIG_BACKPORT_BT_HCIBTUART)	+= btuart_cs.o

obj-$(CONFIG_BACKPORT_BT_HCISTP) 	+= hci_stp.o

obj-$(CONFIG_BACKPORT_BT_HCIBTUSB)	+= btusb.o
obj-$(CONFIG_BACKPORT_BT_HCIBTSDIO)	+= btsdio.o

obj-$(CONFIG_BACKPORT_BT_INTEL)		+= btintel.o
obj-$(CONFIG_BACKPORT_BT_ATH3K)		+= ath3k.o
obj-$(CONFIG_BACKPORT_BT_MRVL)		+= btmrvl.o
obj-$(CONFIG_BACKPORT_BT_MRVL_SDIO)	+= btmrvl_sdio.o
obj-$(CONFIG_BACKPORT_BT_WILINK)		+= btwilink.o
obj-$(CONFIG_BACKPORT_BT_BCM)		+= btbcm.o
obj-$(CONFIG_BACKPORT_BT_RTL)		+= btrtl.o

btmrvl-y			:= btmrvl_main.o
btmrvl-$(CONFIG_DEBUG_FS)	+= btmrvl_debugfs.o

hci_uart-y				:= hci_ldisc.o
hci_uart-$(CONFIG_BACKPORT_BT_HCIUART_H4)	+= hci_h4.o
hci_uart-$(CONFIG_BACKPORT_BT_HCIUART_BCSP)	+= hci_bcsp.o
hci_uart-$(CONFIG_BACKPORT_BT_HCIUART_LL)	+= hci_ll.o
hci_uart-$(CONFIG_BACKPORT_BT_HCIUART_ATH3K)	+= hci_ath.o
hci_uart-$(CONFIG_BACKPORT_BT_HCIUART_3WIRE)	+= hci_h5.o
hci_uart-$(CONFIG_BACKPORT_BT_HCIUART_INTEL)	+= hci_intel.o
hci_uart-$(CONFIG_BACKPORT_BT_HCIUART_BCM)	+= hci_bcm.o
hci_uart-objs				:= $(hci_uart-y)

ccflags-y += -D WMT_IDC_SUPPORT=1 -D MTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT -D MTK_WCN_REMOVE_KERNEL_MODULE -D CONFIG_MTK_WCN_ARM64
ccflags-y += -Idrivers/misc/mediatek/connectivity/common_detect
ccflags-y += -Idrivers/misc/mediatek/connectivity/conn_soc/common/include
ccflags-y += -Idrivers/misc/mediatek/connectivity/conn_soc/common/core/include
ccflags-y += -Idrivers/misc/mediatek/connectivity/conn_soc/common/linux/include
ccflags-y += -Idrivers/misc/mediatek/include/mt-plat
ccflags-y += -D__CHECK_ENDIAN__
