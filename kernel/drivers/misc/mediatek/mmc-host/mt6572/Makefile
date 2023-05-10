include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers

ifeq ($(TARGET_BUILD_VARIANT), user)
ccflags-y += -DUSER_BUILD_KERNEL
endif

obj-$(CONFIG_MTK_MMC)		+= msdc.o

msdc-y += dbg.o
msdc-y += sd.o mt_sd_misc.o

ifeq ($(CONFIG_MTK_AEE_IPANIC),y)
msdc-y += mt_dump.o
endif

ifeq ($(CONFIG_MTK_EMMC_SUPPORT_OTP),y)
msdc-y += emmc_otp.o
endif
