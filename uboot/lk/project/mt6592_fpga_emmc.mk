#
LOCAL_DIR := $(GET_LOCAL_DIR)

TARGET := mediatek

MODULES += app/mt_boot \
           app/shell \
           out/lk

ifneq ($(filter user userdebug, $(TARGET_BUILD_VARIANT)),)
DEBUG := 0
else
DEBUG := 1
endif


#DEFINES += WITH_DEBUG_DCC=1
DEFINES += WITH_DEBUG_UART=1
#DEFINES += WITH_DEBUG_FBCON=1
DEFINES += MACH_FPGA=y

