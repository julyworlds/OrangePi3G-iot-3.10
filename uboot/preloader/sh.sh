#!/bin/bash -e

#MTK_PROJECT=hexing72_cwet_lca
#TOOLCHAIN_PATH=/xspace/OrangePi/MTK/SourceCode/3G-IOT/3G-IOT/iot03_export/code/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.7/bin



export TARGET_PRODUCT=$MTK_PROJECT
export PATH=$TOOLCHAIN_PATH:$PATH
export MTK_ROOT_OUT=output/obj

./build.sh  $TARGET_PRODUCT

