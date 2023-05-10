
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_CM36283),y)
obj-y	+=  alsps/cm36283/
endif

ifeq ($(CONFIG_MTK_KXTJ2_1009),y)
obj-y   +=  accelerometer/KXTJ2_1009/
endif

ifeq ($(CONFIG_MTK_MPU3050C),y)
obj-y   +=  gyroscope/mpu3050c/
endif

ifeq ($(CONFIG_MTK_AKM8963),y)
obj-y	+=  magnetometer/akm8963/
endif

ifeq ($(CONFIG_MTK_MT5193),y)
obj-y	+=  matv/mt5193/
endif

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y	+= core/
obj-y	+= dct/dct/
endif

#obj-y	+=  hdmi/
obj-y	+=  camera/camera/
obj-y	+=  vibrator/
obj-y	+=  leds/
obj-y	+=  keypad/
obj-y	+=  sound/
obj-y	+=  accdet/
