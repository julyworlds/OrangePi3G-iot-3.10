#!/bin/bash

uboot_check()
{

        for ((i = 0; i < 5; i++)); do
                UBOOT_PATH=$(whiptail --title "OrangePi Build System" \
                        --inputbox "Pls input device node of TF card.(eg: /dev/sdc)" \
                        10 60 3>&1 1>&2 2>&3)

                if [ $i = "4" ]; then
                        whiptail --title "OrangePi Build System" --msgbox "Error, Invalid Path" 10 40 0
                        exit 0
                fi

                if [ ! -b "$UBOOT_PATH" ]; then
                        whiptail --title "OrangePi Build System" --msgbox \
                                "The input path invalid! Pls input correct path!" \
                                --ok-button Continue 10 40 0
                else
                        i=200
                fi
        done
}

boot_check()
{
        for ((i = 0; i < 5; i++)); do
                BOOT_PATH=$(whiptail --title "OrangePi Build System" \
                        --inputbox "Pls input mount path of BOOT.(/media/orangepi/BOOT)" \
                        10 60 3>&1 1>&2 2>&3)

                if [ $i = "4" ]; then
                        whiptail --title "OrangePi Build System" --msgbox "Error, Invalid Path" 10 40 0
                        exit 0
                fi

                if [ ! -d "$BOOT_PATH" ]; then
                        whiptail --title "OrangePi Build System" --msgbox \
                                "The input path invalid! Pls input correct path!" \
                                --ok-button Continue 10 40 0
                else
                        i=200
                fi
        done
}

rootfs_check()
{

        for ((i = 0; i < 5; i++)); do
                ROOTFS_PATH=$(whiptail --title "OrangePi Build System" \
                        --inputbox "Pls input mount path of rootfs.(/media/orangepi/rootfs)" \
                        10 60 3>&1 1>&2 2>&3)

                if [ $i = "4" ]; then
                        whiptail --title "OrangePi Build System" --msgbox "Error, Invalid Path" 10 40 0
                        exit 0
                fi

                if [ ! -d "$ROOTFS_PATH" ]; then
                        whiptail --title "OrangePi Build System" --msgbox \
                                "The input path invalid! Pls input correct path!" \
                                --ok-button Continue 10 40 0
                else
                        i=200
                fi
        done
}

prepare_host()
{
	if ! hash apt-get 2>/dev/null; then
	        whiptail --title "Orangepi Build System" --msgbox "This scripts requires a Debian based distrbution."
		        exit 1
	fi

	apt-get -y --no-install-recommends --fix-missing install \
		        libarchive-tools mtools u-boot-tools pv bc \
		        gcc automake make binfmt-support flex \
		        lib32z1 lib32z1-dev qemu-user-static bison \
		        dosfstools libncurses5-dev debootstrap \
		        swig libpython2.7-dev libssl-dev python2-minimal dos2unix

	# Prepare toolchains
	chmod 755 -R $ROOT/toolchain/*

	if [ ! -d "${BUILD}" ]; then
		mkdir -p "${BUILD}"
	fi
}

kernel_update()
{
	
	case "${PLATFORM}" in
		"OrangePiH3" | "OrangePiH6" | "OrangePiH6_Linux4.9")
			KERNEL_IMAGE=$BUILD/kernel/uImage_${BOARD}

			# Update kernel
			rm -rf $BOOT_PATH/uImage
			cp -rf $KERNEL_IMAGE $BOOT_PATH/uImage
			;;
		"OrangePiH3_mainline")
			KERNEL_IMAGE=$BUILD/kernel/zImage_${BOARD}

			# Update kernel
			rm -rf $BOOT_PATH/zImage
			rm -rf $BOOT_PATH/dtb
			cp -rf $KERNEL_IMAGE $BOOT_PATH/zImage
			cp -rf $BUILD/dtb $BOOT_PATH/
			;;
		"OrangePiH6_mainline")
			KERNEL_IMAGE=$BUILD/kernel/Image_${BOARD}

			# Update kernel
			rm -rf $BOOT_PATH/Image
			rm -rf $BOOT_PATH/dtb/allwinner/*.dtb
			cp -rf $KERNEL_IMAGE $BOOT_PATH/Image
			cp -rf $BUILD/dtb/allwinner/* $BOOT_PATH/dtb/allwinner/
			;;
		"OrangePiRK3399")
			pv $BUILD/kernel/boot.img  | dd of=$UBOOT_PATH seek=49152 conv=notrunc
			;;
		"*")
			;;
	esac

	sync
	clear

	whiptail --title "OrangePi Build System" --msgbox "Succeed to update kernel" 10 60
}

modules_update()
{

	# Remove old modules
	rm -rf $ROOTFS_PATH/lib/modules

	cp -rfa $BUILD/lib/modules $ROOTFS_PATH/lib/

	sync
	clear

	whiptail --title "OrangePi Build System" --msgbox "Succeed to update Module" 10 40 0
}

uboot_update()
{
	
	case "${PLATFORM}" in
		"OrangePiH3" | "OrangePiH6" | "OrangePiH6_Linux4.9")
			boot0=$BUILD/uboot/boot0_sdcard_"${CHIP}".bin
			uboot=$BUILD/uboot/u-boot-"${CHIP}".bin

			# Clean TF partition
			dd bs=1K seek=8 count=1015 if=/dev/zero of="$UBOOT_PATH"
			# Update uboot
			dd if=$boot0 of=$UBOOT_PATH conv=notrunc bs=1k seek=8
			dd if=$uboot of=$UBOOT_PATH conv=notrunc bs=1k seek=16400
			;;
		"OrangePiH3_mainline" | "OrangePiH6_mainline")
			dd if=/dev/zero of=$UBOOT_PATH bs=1k seek=8 count=1015
			uboot=$BUILD/uboot/u-boot-sunxi-with-spl.bin-${BOARD}
			dd if=$uboot of=$UBOOT_PATH conv=notrunc bs=1k seek=16400
			;;
		"OrangePiRK3399")
			dd if=$BUILD/uboot/idbloader.img of=$UBOOT_PATH seek=64
                	dd if=$BUILD/uboot/uboot.img of=$UBOOT_PATH seek=24576
                	dd if=$BUILD/uboot/trust.img of=$UBOOT_PATH seek=32768
			;;
		"*")
			;;
	esac

	sync
	clear

	whiptail --title "OrangePi Build System" --msgbox "Succeed to update Uboot" 10 40 0
}

select_distro()
{
	MENUSTR="Distro Options"
	OPTION=$(whiptail --title "OrangePi Build System" \
		--menu "$MENUSTR" 20 60 10 --cancel-button Finish --ok-button Select \
                "0"   "[$SOURCES]Change repository server" \
                "1"   "Ubuntu Xenial" \
                "2"   "Debian Stretch" \
                3>&1 1>&2 2>&3)

		case "${OPTION}" in
			"0") 
				select_sources 
				;;
			"1") 
				DISTRO="xenial"
				DISTRO_NUM="16.04.1"
	                     	OS="ubuntu"
				;;
		 	"2")     
	                	DISTRO="stretch"
	                	OS="debian"
				;;
			"3")
				DISTRO="bionic"
				DISTRO_NUM="18.04"
				OS="ubuntu"
				;;
			"*")
				;;
		esac
			
        TYPE=$(whiptail --title "OrangePi Build System" \
                --menu "$MENUSTR" 20 60 3 --cancel-button Finish --ok-button Select \
                "0"   "Server" \
                3>&1 1>&2 2>&3)
        
                case "${TYPE}" in
                        "0") 
				IMAGETYPE="server" ;;
                        "1") 
				IMAGETYPE="desktop" ;;
                        "*") 
				;;
               esac
}

select_sources()
{
	SOURCES=$(whiptail --title "Repository Server" --nocancel --radiolist \
		"What is the repository server of your choice?" 20 60 5 \
	       	"CN" "The server from China." ON \
		"CDN" "Deafult CDN repository server(RCMD)." OFF \
		"OFCL" "Official repository server." OFF 3>&1 1>&2 2>&3)

	exitstatus=$?

        if [ $exitstatus = 0 ]; then
		echo "The chosen server is:" $SOURCES
		select_distro
	fi
}

pack_error()
{
	        echo -e "\033[47;31mERROR: $*\033[0m"
}

pack_warn()
{
        echo -e "\033[47;34mWARN: $*\033[0m"
}

pack_info()
{
        echo -e "\033[47;30mINFO: $*\033[0m"
}
