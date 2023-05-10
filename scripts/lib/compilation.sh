#!/bin/bash

# Functions:
# compile_uboot
# compile_kernel

compile_uboot()
{
	if [ ! -d $UBOOT_BIN ]; then
		mkdir -p $UBOOT_BIN
	fi

	# Perpar souce code
	if [ ! -d $UBOOT ]; then
		whiptail --title "OrangePi Build System" \
			--msgbox "u-boot doesn't exist, pls perpare u-boot source code." \
			10 50 0
		exit 0
	fi

	cd $UBOOT
	echo -e "\e[1;31m Build U-boot \e[0m"

	case "${PLATFORM}" in
		"OrangePiH3" | "OrangePiH6" | "OrangePiH6_Linux4.9")
			if [ ! -f $UBOOT/u-boot-"${CHIP}".bin -o ! -f $UBOOT/boot0_sdcard_"${CHIP}".bin ]; then
			make "${CHIP}"_config 
			fi
			make -j${CORES} CROSS_COMPILE="${UBOOT_COMPILE}"
			make spl -j${CORES} CROSS_COMPILE="${UBOOT_COMPILE}"
			cd -
			pack
			;;
		"OrangePiH3_mainline" | "OrangePiH6_mainline")
			make orangepi_"${BOARD}"_defconfig
			make -j4 ARCH=arm CROSS_COMPILE="${UBOOT_COMPILE}"

			cp "$UBOOT"/u-boot-sunxi-with-spl.bin "$UBOOT_BIN"/u-boot-sunxi-with-spl.bin-"${BOARD}" -f
			;;
		"OrangePiRK3399")
			./make.sh rk3399
			cp -rf uboot.img $UBOOT_BIN
			cp -rf trust.img $UBOOT_BIN
			cp -rf rk3399_loader_v1.22.119.bin $UBOOT_BIN
			cp -rf idbloader.img $UBOOT_BIN
			;;
		"OrangePi4G-iot")
			export CROSS_COMPILE=$TOOLS
			export TOOLCHAIN_PREFIX=$TOOLS
			[ -d ${BUILD}/preloader ] || mkdir -p ${BUILD}/preloader
			[ -d ${BUILD}/lk ] || mkdir -p ${BUILD}/lk

			cp $ROOT/external/preloader/* $BUILD/preloader -rfp
			make -C ${UBOOT}/preloader -s -f Makefile \
				PRELOADER_OUT=$BUILD/preloader TOOL_PATH=$ROOT/external/tools \
        			MTK_PROJECT=bd6737m_35g_b_m0
			make -C ${UBOOT}/lk BOOTLOADER_OUT=$BUILD/lk bd6737m_35g_b_m0
			;;
		"OrangePi3G-iot")
			set -x
			export CROSS_COMPILE=$TOOLS
			export TOOLCHAIN_PREFIX=$TOOLS
			export TOOLCHAIN_PATH=$ROOT/toolchain/arm-eabi-4.7/arm-linux-androideabi-4.7/bin

			export MTK_PROJECT
			export TARGET_PRODUCT=$MTK_PROJECT
			OBJ=${BUILD}/obj
			[ -e ${BUILD}/project ] && PROJECT=$(cat ${BUILD}/project)
			[ $MTK_PROJECT != $PROJECT ] && rm -rf ${OBJ}
			if [ ! -d $OBJ ]; then
				mkdir -p $OBJ
				echo "$MTK_PROJECT" > ${BUILD}/project
				cp -rfa ${EXTER}/${MTK_PROJECT}/* $OBJ
    				if [ $MTK_PROJECT = "hexing72_cwet_lca" ]; then
                			cp $EXTER/project/a/config/common/custom.conf $EXTER/mediatek/config/common/
                			cp $EXTER/project/a/kernel/drivers/keypad/kpd.c $EXTER/mediatek/kernel/drivers/keypad/
                			cp $EXTER/project/a/ddp_rdma.c $EXTER/mediatek/platform/mt6572/kernel/drivers/dispsys/
                			cp $EXTER/project/a/lk/* $EXTER/mediatek/platform/mt6572/lk/
    				else
                			cp $EXTER/project/b/config/common/custom.conf $EXTER/mediatek/config/common/
                			cp $EXTER/project/b/kernel/drivers/keypad/kpd.c $EXTER/mediatek/kernel/drivers/keypad/
                			rm -rf $EXTER/mediatek/platform/mt6572/kernel/drivers/dispsys/ddp_rdma.c
                			cp $EXTER/project/b/lk/* $EXTER/mediatek/platform/mt6572/lk/
    				fi
			fi
			echo -e "\e[1;35m Build Preloader\e[0m"
			cd ${UBOOT}/preloader
			./sh.sh
			echo -e "\e[1;35m Build LK\e[0m"
			cd ${UBOOT}/lk
			./sh.sh

			PRELOADERBIN=$BUILD/obj/PRELOADER_OBJ/bin/preloader_$MTK_PROJECT.bin
                	LKBIN=$BUILD/obj/BOOTLOADER_OBJ/build-$MTK_PROJECT/lk.bin
                	LOGOBIN=$BUILD/obj/BOOTLOADER_OBJ/build-$MTK_PROJECT/logo.bin
			cp ${PRELOADERBIN} ${UBOOT_BIN}
			cp ${LKBIN} ${UBOOT_BIN}
			cp ${LOGOBIN} ${UBOOT_BIN}
			;;
		*)
	        	echo -e "\e[1;31m Pls select correct platform \e[0m"
	        	exit 0
			;;
	esac

	echo -e "\e[1;31m Complete U-boot compile.... \e[0m"

	#whiptail --title "OrangePi Build System" \
	#	--msgbox "Build uboot finish. The output path: $BUILD" 10 60 0
}

compile_kernel()
{
	if [ ! -d $BUILD ]; then
		mkdir -p $BUILD
	fi

	if [ ! -d $BUILD/kernel ]; then
		mkdir -p $BUILD/kernel
	fi

	echo -e "\e[1;31m Start compiling the kernel ...\e[0m"

	case "${PLATFORM}" in 
		"OrangePiH3")
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} uImage
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} modules
			cp $LINUX/arch/"${ARCH}"/boot/uImage $BUILD/kernel/uImage_$BOARD
			;;
		"OrangePiH6" | "OrangePiH6_Linux4.9")
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} Image
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} dtbs
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} modules
			mkimage -A arm -n "${PLATFORM}" -O linux -T kernel -C none -a 0x40080000 -e 0x40080000 \
		                -d $LINUX/arch/"${ARCH}"/boot/Image "${BUILD}"/kernel/uImage_"${BOARD}"
			;;
		"OrangePiH3_mainline" | "OrangePiH6_mainline") 
			# make kernel
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES}

			if [ ! -d $BUILD/dtb ]; then
				mkdir -p $BUILD/dtb
			else
				rm -rf $BUILD/dtb/*
			fi
			# copy dtbs
			echo -e "\e[1;31m Start Copy dtbs \e[0m"

			if [ ${PLATFORM} = "OrangePiH3_mainline" ];then
       				cp $LINUX/arch/"${ARCH}"/boot/dts/sun8i-h3-orangepi*.dtb $BUILD/dtb/
       				cp $LINUX/arch/"${ARCH}"/boot/dts/sun8i-h2-plus-orangepi-*.dtb $BUILD/dtb/
			elif [ ${PLATFORM} = "OrangePiH6_mainline" ];then
       				cp $LINUX/arch/"${ARCH}"/boot/dts/allwinner/sun50i-h6-orangepi-${BOARD}.dtb $BUILD/dtb/
			fi


			if [ ${PLATFORM} = "OrangePiH6_mainline" ];then
				cp $LINUX/arch/"${ARCH}"/boot/Image $BUILD/kernel/Image_$BOARD
			elif [ ${PLATFORM} = "OrangePiH3_mainline" ];then
				cp $LINUX/arch/"${ARCH}"/boot/zImage $BUILD/kernel/zImage_$BOARD
			fi

			cp $LINUX/System.map $BUILD/kernel/System.map-$BOARD
			;;
		"OrangePiRK3399")
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS ${BOARD}_linux_defconfig
			echo -e "\e[1;31m Using ${BOARD}_linux_defconfig\e[0m"
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS -j${CORES} rk3399-orangepi-${BOARD}.img
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS -j${CORES} modules
			cp $LINUX/boot.img $BUILD/kernel
			;;
		"OrangePi4G-iot")
			export TARGET_BUILD_VARIANT=eng
			echo -e "\e[1;31m Using ${BOARD}_linux_defconfig\e[0m"
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS O=$BUILD/kernel ${BOARD}_linux_defconfig
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS -j${CORES} O=$BUILD/kernel
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS -j${CORES} O=$BUILD/kernel modules 
			;;
		"OrangePi3G-iot")
			set -x
			OBJ=${BUILD}/obj
			[ -e ${BUILD}/project ] && PROJECT=$(cat ${BUILD}/project)
			[ $MTK_PROJECT != $PROJECT ] && rm -rf ${OBJ}
			if [ ! -d $OBJ ]; then
				mkdir -p $OBJ
				echo "$MTK_PROJECT" > ${BUILD}/project
				cp -rfa ${EXTER}/${MTK_PROJECT}/* $OBJ
    				if [ $MTK_PROJECT = "hexing72_cwet_lca" ]; then
                			cp $EXTER/project/a/config/common/custom.conf $EXTER/mediatek/config/common/
                			cp $EXTER/project/a/kernel/drivers/keypad/kpd.c $EXTER/mediatek/kernel/drivers/keypad/
                			cp $EXTER/project/a/ddp_rdma.c $EXTER/mediatek/platform/mt6572/kernel/drivers/dispsys/
                			cp $EXTER/project/a/lk/* $EXTER/mediatek/platform/mt6572/lk/
    				else
                			cp $EXTER/project/b/config/common/custom.conf $EXTER/mediatek/config/common/
                			cp $EXTER/project/b/kernel/drivers/keypad/kpd.c $EXTER/mediatek/kernel/drivers/keypad/
                			rm -rf $EXTER/mediatek/platform/mt6572/kernel/drivers/dispsys/ddp_rdma.c
                			cp $EXTER/project/b/lk/* $EXTER/mediatek/platform/mt6572/lk/
    				fi
			fi
			export PATH=$ROOT/external/make:$PATH
			export TARGET_PRODUCT=$MTK_PROJECT

        		echo -e "\e[1;31m Using ${PLATFORM}_linux_defconfig \e[0m"
			make -C $LINUX ARCH=arm CROSS_COMPILE=$TOOLS O=$BUILD/obj/KERNEL_OBJ ${BOARD}_linux_defconfig

			make -C $LINUX ARCH=arm CROSS_COMPILE=$TOOLS O=$BUILD/obj/KERNEL_OBJ silentoldconfig
			make -C $LINUX ARCH=arm CROSS_COMPILE=$TOOLS -j${CORES} O=$BUILD/obj/KERNEL_OBJ
			make -C $LINUX ARCH=arm CROSS_COMPILE=$TOOLS -j${CORES} O=$BUILD/obj/KERNEL_OBJ modules

			${EXTER}/mkbootimg --kernel ${BUILD}/obj/KERNEL_OBJ/kernel_${MTK_PROJECT}.bin --board 2016.07.04  --output ${BUILD}/kernel/boot.img

			;;
		*)
	        	echo -e "\e[1;31m Pls select correct platform \e[0m"
			exit 0
	esac

	echo -e "\e[1;31m Complete kernel compilation ...\e[0m"
}

compile_module(){
	
	if [ ! -d $BUILD/lib ]; then
	        mkdir -p $BUILD/lib
	else
	        rm -rf $BUILD/lib/*
	fi
	
	# install module
	echo -e "\e[1;31m Start installing kernel modules ... \e[0m"
	if [ ${PLATFORM} = "OrangePi3G-iot" ]; then
		make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} O=$BUILD/obj/KERNEL_OBJ modules
		make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} O=$BUILD/obj/KERNEL_OBJ modules_install INSTALL_MOD_PATH=$BUILD
	elif [ ${PLATFORM} = "OrangePi4G-iot" ]; then
		make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} O=$BUILD/kernel modules
		make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} O=$BUILD/kernel modules_install INSTALL_MOD_PATH=$BUILD
	fi
	echo -e "\e[1;31m Complete kernel module installation ... \e[0m"

	#whiptail --title "OrangePi Build System" --msgbox \
	#	"Build Kernel OK. The path of output file: ${BUILD}" 10 80 0
}
