# drivers/dispsys

obj-y += ddp_drv.o
obj-y += ddp_ovl.o
obj-y += ddp_bls.o
obj-y += ddp_pq.o
obj-y += ddp_rdma.o
obj-y += ddp_wdma.o
obj-y += ddp_path.o
obj-y += ddp_debug.o
obj-y += ddp_cmdq.o
obj-y += ddp_aal.o
obj-y += data_rgb888_64x64.o
obj-y += data_rgb888_64x64_golden.o

ccflags-y += -I$(MTK_PATH_SOURCE)/drivers/video/ \
             -I$(MTK_PATH_PLATFORM)/../../common/kernel/drivers/video/ \
             -I$(MTK_PATH_PLATFORM)/drivers/smi/ \
             -I$(MTK_PATH_PLATFORM)/drivers/video/ \
             -I$(MTK_PATH_PLATFORM)/drivers/devinfo/

# EOF
