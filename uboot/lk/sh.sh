#!/bin/bash

#export PATH=$PATH:/xspace/OrangePi/MTK/SourceCode/MT6572/3G-IOT-A/iot03_export/code/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.7/bin/


#TARGET_PRODUCT=hexing72_cwet_kk

ARGS="$1 MTK_ROOT_OUT=output/obj TARGET_BUILD_VARIANT=eng BOOTLOADER_OUT=../../output/obj/BOOTLOADER_OBJ MTK_ROOT_CUSTOM_OUT=../../output/obj/CUSTGEN/custom" 
TARGET_PRODUCT=$MTK_PROJECT
FULL_PROJECT=$TARGET_PRODUCT make $TARGET_PRODUCT -j4 $ARGS
