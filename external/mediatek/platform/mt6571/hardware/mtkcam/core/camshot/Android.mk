#
# libcam.camshot
#
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

#
LOCAL_SRC_FILES := \
    CamShotImp.cpp \
    $(call all-c-cpp-files-under, SingleShot) \
    $(call all-c-cpp-files-under, MultiShot) \
    $(call all-c-cpp-files-under, BurstShot) \

#    $(call all-c-cpp-files-under, SampleSingleShot) \
#    $(call all-c-cpp-files-under, SImager) \
#    $(call all-c-cpp-files-under, SImager/ImageTransform) \
#    $(call all-c-cpp-files-under, SImager/JpegCodec) \

#-----------------------------------------------------------
LOCAL_C_INCLUDES += $(MTKCAM_C_INCLUDES)
#
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/hardware/include
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/include
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/hardware/include/mtkcam
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/include/mtkcam
#
# Note: "/bionic" and "/external/stlport/stlport" is for stlport.
LOCAL_C_INCLUDES += $(TOP)/bionic
LOCAL_C_INCLUDES += $(TOP)/external/stlport/stlport
# 
# camera Hardware 
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/frameworks/base/include
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/hardware/camera/inc
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/hardware/camera/inc/common
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/hardware/camera/inc/common/camutils
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/camera/inc
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/camera/inc/common
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/camera/inc/common/camutils
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/kernel/include
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/kernel/core/include/mach
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/v1/hal/adapter/inc
LOCAL_C_INCLUDES += $(TOP)/bionic
LOCAL_C_INCLUDES += $(TOP)/external/stlport/stlport
LOCAL_C_INCLUDES += $(TOP)/$(MTK_ROOT)/hardware/dpframework/inc
LOCAL_C_INCLUDES += $(TOP)/external/jpeg

# jpeg encoder 
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/jpeg/inc \
# m4u 
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/m4u \

#
LOCAL_SHARED_LIBRARIES := \
    libcutils \
    liblog \
    libstlport \
    libcam.campipe \
    libcamdrv \
    libutils \
    libdpframework \
    libjpeg \
	  
# for jpeg enc use 
LOCAL_SHARED_LIBRARIES += \
    libm4u \
    libJpgEncPipe \

# for 3A 
LOCAL_SHARED_LIBRARIES +=\
    libfeatureio \

# camUtils 
LOCAL_SHARED_LIBRARIES += libcam_mmp
ifeq ($(RY_GANGYUN_CAMERA_BEAUTY),yes)
LOCAL_SHARED_LIBRARIES += libgybeauty libgybeauty420 libgybeauty422 libDetectFeature
endif
LOCAL_SHARED_LIBRARIES +=\
    libcam.utils \
    

LOCAL_STATIC_LIBRARIES := \
 
#
LOCAL_WHOLE_STATIC_LIBRARIES := \
     libcam.camshot.simager \
     libcam.camshot.utils \
     
#
LOCAL_MODULE := libcam.camshot

#
LOCAL_MODULE_TAGS := optional

PLATFORM_VERSION_MAJOR := $(word 1,$(subst .,$(space),$(PLATFORM_VERSION)))
LOCAL_CFLAGS += -DPLATFORM_VERSION_MAJOR=$(PLATFORM_VERSION_MAJOR)

ifeq ($(PLATFORM_VERSION_MAJOR),2)
LOCAL_PRELINK_MODULE := false
endif

#
include $(BUILD_SHARED_LIBRARY)


ifeq ($(RY_GANGYUN_CAMERA_BEAUTY),yes)

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := gangyun/lib/libgybeauty.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := gangyun/lib/libgybeauty420.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := gangyun/lib/libgybeauty422.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := gangyun/lib/libDetectFeature.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)

endif
#
include $(call all-makefiles-under,$(LOCAL_PATH))
