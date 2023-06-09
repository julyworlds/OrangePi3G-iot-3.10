LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_ARM_MODE := arm
LOCAL_PRELINK_MODULE := false
PLATFORM_PATH := $(MTK_PATH_PLATFORM)/external/meta
LOCAL_SHARED_LIBRARIES := libc \
                          libnvram \
                          libcutils \
                          libnetutils \
                          libmedia \
                          libhardware_legacy \
                          libfile_op \
                          libdl \
                          libhwm \
                          libutils\
                          libaudio.primary.default \
                          #libacdk \
                          #libaudiocompensationfilter \
                          #libheadphonecompensationfilter \
                          #libaudiocustparam \


ifeq ($(HAVE_MATV_FEATURE),yes)
LOCAL_SHARED_LIBRARIES += libmatv_cust
endif

ifeq ($(TELEPHONY_DFOSET),yes)
LOCAL_SHARED_LIBRARIES += libdfo
endif  


LOCAL_C_INCLUDES += $(MTK_PATH_SOURCE)/external/nvram/libfile_op \
                    $(MTK_PATH_SOURCE)/external/meta/common/inc \
                    $(MTK_PATH_SOURCE)/external/matvctrl \
                    $(PLATFORM_PATH)/matv \
                    $(PLATFORM_PATH)/include \
                    $(PLATFORM_PATH)/ft \
                    $(PLATFORM_PATH)/meta_lock \
                    $(PLATFORM_PATH)/Meta_APEditor \
                    $(PLATFORM_PATH)/lcd \
                    $(PLATFORM_PATH)/LCDBK \
                    $(PLATFORM_PATH)/cpu \
                    $(PLATFORM_PATH)/sdcard \
                    $(PLATFORM_PATH)/gpio \
                    $(PLATFORM_PATH)/keypadbk \
                    $(PLATFORM_PATH)/ADC \
                    $(PLATFORM_PATH)/BatteryIC \
                    $(PLATFORM_PATH)/pmic \
                    $(PLATFORM_PATH)/vibrator \
                    $(PLATFORM_PATH)/msensor \
                    $(PLATFORM_PATH)/alsps \
                    $(PLATFORM_PATH)/gsensor \
                    $(PLATFORM_PATH)/gyroscope \
                    $(PLATFORM_PATH)/touch \
                    $(PLATFORM_PATH)/cameratool/CCAP \
                    $(TOP)/$(MTK_PATH_SOURCE)/hardware/mtkcam/inc/acdk \
                    $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/inc/acdk \
                    $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/acdk/inc/cct \
                    $(MTK_PATH_SOURCE)/external/mhal/src/custom/inc \
                    $(MTK_PATH_SOURCE)/external/mhal/inc \
                    $(TOP)/$(MTK_PATH_CUSTOM)/kernel/imgsensor/inc \
                    $(MTK_PATH_CUSTOM)/hal/inc \
                    $(PLATFORM_PATH)/Audio \
                    mediatek/external/dfo/featured \
                    $(TARGET_OUT_HEADERS)/dfo\
                    $(TOPDIR)/hardware/libhardware/include


ifeq ($(MTK_WLAN_SUPPORT),yes)
LOCAL_C_INCLUDES += $(PLATFORM_PATH)/wifi
endif

ifeq ($(MTK_GPS_SUPPORT),yes)
LOCAL_C_INCLUDES += $(PLATFORM_PATH)/gps
endif

ifeq ($(MTK_FM_SUPPORT),yes)
LOCAL_C_INCLUDES += $(PLATFORM_PATH)/fm
endif

ifeq ($(MTK_BT_SUPPORT),yes)
LOCAL_C_INCLUDES += $(PLATFORM_PATH)/bluetooth
endif

ifeq ($(MTK_NFC_SUPPORT),yes)
LOCAL_C_INCLUDES += $(PLATFORM_PATH)/nfc
LOCAL_SHARED_LIBRARIES += libmtknfc_dynamic_load_jni
endif

ifeq ($(MTK_EMMC_SUPPORT),yes)
LOCAL_C_INCLUDES +=$(PLATFORM_PATH)/emmc
LOCAL_C_INCLUDES +=$(PLATFORM_PATH)/cryptfs
endif
ifeq ($(MTK_EMMC_SUPPORT),no)
LOCAL_C_INCLUDES +=$(PLATFORM_PATH)/emmc
LOCAL_C_INCLUDES +=$(PLATFORM_PATH)/cryptfs
endif

LOCAL_C_INCLUDES +=$(PLATFORM_PATH)/dfo \
                    $(MTK_PATH_SOURCE)/external/dfo/boot


LOCAL_SRC_FILES := ft_main.cpp \
                   ft_fnc.cpp

LOCAL_STATIC_LIBRARIES := libmeta_apeditor \
                          libmeta_lcd \
                          libmeta_lcdbk \
                          libmeta_cpu \
                          libmeta_lock \
                          libmeta_gpio \
                          libmeta_keypadbk \
                          libmeta_sdcard \
                          libmeta_adc_old \
                          libmeta_battery \
                          libmeta_pmic \
                          libmeta_vibrator \
                          libmeta_msensor \
                          libmeta_alsps \
                          libmeta_gsensor \
                          libmeta_gyroscope\
                          libmeta_touch \
                          libccap \
                          libmeta_audio \
                          libmeta_clr_emmc \
                          libmtdutil
                          #libccap
                          
LOCAL_STATIC_LIBRARIES += libfft
ifeq ($(MTK_WLAN_SUPPORT),yes)
LOCAL_STATIC_LIBRARIES +=libmeta_wifi
LOCAL_CFLAGS += \
    -DFT_WIFI_FEATURE
endif

ifeq ($(MTK_GPS_SUPPORT),yes)
LOCAL_STATIC_LIBRARIES += libmeta_gps
LOCAL_CFLAGS += \
    -DFT_GPS_FEATURE
endif

ifeq ($(MTK_FM_SUPPORT),yes)
LOCAL_STATIC_LIBRARIES += libmeta_fm
LOCAL_CFLAGS += \
    -DFT_FM_FEATURE
endif

ifeq ($(MTK_EMMC_SUPPORT),yes)
LOCAL_STATIC_LIBRARIES += libmeta_cryptfs \
                          libext4_utils_static libz \
                          libmeta_clr_emmc \
                          libmtdutil
LOCAL_CFLAGS += \
    -DFT_EMMC_FEATURE
else
LOCAL_CFLAGS += \
    -DFT_NAND_FEATURE
endif

ifeq ($(MTK_BT_SUPPORT),yes)
LOCAL_STATIC_LIBRARIES += libmeta_bluetooth
LOCAL_CFLAGS += \
    -DFT_BT_FEATURE
endif

ifeq ($(HAVE_MATV_FEATURE),yes)
LOCAL_STATIC_LIBRARIES += libmeta_matv
LOCAL_CFLAGS += \
    -DFT_MATV_FEATURE
endif

ifeq ($(MTK_NFC_SUPPORT),yes)
LOCAL_STATIC_LIBRARIES += libmeta_nfc
LOCAL_CFLAGS += \
    -DFT_NFC_FEATURE
endif

LOCAL_STATIC_LIBRARIES +=  libmeta_dfo


ifeq ($(GEMINI),yes)
LOCAL_CFLAGS += \
    -DGEMINI
endif

ifeq ($(MTK_GEMINI_3SIM_SUPPORT),yes)
LOCAL_CFLAGS += \
    -DMTK_GEMINI_3SIM_SUPPORT
endif

ifeq ($(MTK_GEMINI_4SIM_SUPPORT),yes)
LOCAL_CFLAGS += \
    -DMTK_GEMINI_4SIM_SUPPORT
endif

ifeq ($(MTK_WORLD_PHONE),yes)
LOCAL_CFLAGS += \
    -DMTK_WORLD_PHONE
endif  

ifeq ($(MTK_LTE_SUPPORT),yes)
LOCAL_CFLAGS +=-DMTK_LTE_SUPPORT
endif

LOCAL_MODULE := libft

#
# Start of common part ------------------------------------
sinclude $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/mtkcam.mk

#-----------------------------------------------------------
LOCAL_CFLAGS += $(MTKCAM_CFLAGS)

#-----------------------------------------------------------
LOCAL_C_INCLUDES += $(MTKCAM_C_INCLUDES)

#-----------------------------------------------------------
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/hardware/include
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/include

# End of common part ---------------------------------------
#

include $(BUILD_SHARED_LIBRARY)

