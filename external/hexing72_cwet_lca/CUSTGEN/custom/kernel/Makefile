include $(MTK_PATH_BUILD)/common.mk
$(call all-modules-src-or-makefile,$(obj),CUSTOM_KERNEL_)
obj-n := dummy.o

ifeq ($(MTK_ALPS_BOX_SUPPORT), yes)
ccflags-y += -DMTK_ALPS_BOX_SUPPORT
endif


-include $(srctree)/$(obj)/touchpanel/ft5x16.mk
-include $(srctree)/$(obj)/touchpanel/gslX68X.mk
