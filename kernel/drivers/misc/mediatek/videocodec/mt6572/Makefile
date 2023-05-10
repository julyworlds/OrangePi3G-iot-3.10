include $(srctree)/drivers/misc/mediatek/Makefile.custom

vcodec_kernel_driver-objs :=      videocodec_kernel_driver.o \
                                            codec/common/val/linux/kernel/val.o 



obj-$(CONFIG_MTK_VIDEOCODEC_DRIVER) += vcodec_kernel_driver.o
#obj-y += vcodec_kernel_driver.o

ccflags-y +=                                    -Iinc \
                                                -Icodec/common/val/linux/kernel \
                                                -Icodec/common/hal/inc \
                                                -Icodec/common/api/inc \
                                                -I$(srctree)/include/linux/vcodec \
                                                -I$(srctree)/include/linux/vcodec/kernel \
