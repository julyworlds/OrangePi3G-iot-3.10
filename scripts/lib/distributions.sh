#!/bin/bash


install_xfce_desktop()
{
 	cp /etc/resolv.conf "$DEST/etc/resolv.conf"
	cat > "$DEST/type-phase" <<EOF
#!/bin/bash
export DEBIAN_FRONTEND=noninteractive
apt-get -y install xorg xfce4 xfce4-goodies vlc network-manager-gnome

apt-get -y autoremove
apt-get clean

EOF
        chmod +x "$DEST/type-phase"
        do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
	rm -f "$DEST/etc/resolv.conf"
}

install_lxde_desktop()
{
	_user="orangepi"
	_auto="-y -q"
	if [ $DISTRO = "bionic" -o $DISTRO = "xenial" ]; then
		_DST=Ubuntu
	else
		_DST=Debian
	fi
	cp /etc/resolv.conf "$DEST/etc/resolv.conf"
	cat > "$DEST/type-phase" <<EOF
#!/bin/bash

echo ""
date
echo -e "\033[36m======================="
echo -e "Installing LXDE Desktop"
echo -e "=======================\033[37m"
setterm -default
echo ""



echo "Package update..."
apt-get $_auto update 
#echo "Package upgrade..."
#apt-get $_auto upgrade
echo ""

echo "$_DST - $_REL, Installing LXDE DESKTOP..."

# === Install desktop =============================================================================================================================================================
echo "  installing xserver & lxde desktop, please wait..."
apt-get $_auto install xinit xserver-xorg 
apt-get $_auto install lxde lightdm lightdm-gtk-greeter policykit-1 --no-install-recommends
apt-get $_auto install net-tools
apt-get $_auto install lxsession-logout
apt-get clean

if [ "${_DST}" = "Ubuntu" ] ; then
	apt-get $_auto install humanity-icon-theme --no-install-recommends 
fi

apt-get $_auto install pulseaudio pulseaudio-utils alsa-oss alsa-utils alsa-tools libasound2-data pavucontrol
apt-get $_auto install smplayer 
apt-get $_auto install synaptic software-properties-gtk lxtask galculator policykit-1-gnome --no-install-recommends
apt-get clean

# === Install network packages & internet browser =================================================================================================================================
# === you don't have to install internet browser, you can save ~100MB ===

echo "  installing network packages, please wait..."
if [ "${_DST}" = "Ubuntu" ]; then
	apt-get $_auto install chromium-browser gvfs-fuse gvfs-backends --no-install-recommends
else
	apt-get $_auto install chromium gvfs-fuse gvfs-backends --no-install-recommends
fi
apt-get $_auto install network-manager-gnome
apt-get clean


# === Configuration ===============================================================================================================================================================
echo ""
echo "Configuring desktop..."

if [ -f /etc/X11/Xwrapper.config ]; then
    cat /etc/X11/Xwrapper.config | sed s/"allowed_users=console"/"allowed_users=anybody"/g > /tmp/_xwrap
    mv /tmp/_xwrap /etc/X11/Xwrapper.config
fi

if [ -f /etc/lightdm/lightdm-gtk-greeter.conf ]; then
    cat /etc/lightdm/lightdm-gtk-greeter.conf | sed "/background=\/usr/d" > /tmp/_greet
    mv /tmp/_greet /etc/lightdm/lightdm-gtk-greeter.conf
    cat /etc/lightdm/lightdm-gtk-greeter.conf | sed '/\[greeter\]/abackground=\/usr\/share\/lxde\/wallpapers\/orangepi.jpg' > /tmp/_greet
    mv /tmp/_greet /etc/lightdm/lightdm-gtk-greeter.conf
fi

#*********************
# ** CONFIGURE SOUND
#*********************
cat > /etc/asound.conf << _EOF_
pcm.!default {
    type hw
    card 1    #If you want to set HDMI as output ,turn 0 to 1.
    device 0
  }
  ctl.!default {
    type hw
    card 1   #If you want to set HDMI as output ,turn 0 to 1.
  }
_EOF_

if [ -f /etc/pulse/default.pa ]; then
    cat /etc/pulse/default.pa | sed s/"load-module module-udev-detect$"/"load-module module-udev-detect tsched=0"/g > /tmp/default.pa
    mv /tmp/default.pa /etc/pulse/default.pa
fi


usermod -a -G adm,dialout,cdrom,dip,video,plugdev,netdev,fuse $_user

chown -R $_user:$_user /home/$_user

EOF
	chmod +x "$DEST/type-phase"
 	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
	rm -f "$DEST/etc/resolv.conf"

}

deboostrap_rootfs() {
	dist="$1"
	tgz="$(readlink -f "$2")"
	TEMP=$(mktemp -d)

	[ "$TEMP" ] || exit 1
	cd $TEMP && pwd

	# this is updated very seldom, so is ok to hardcode
	debian_archive_keyring_deb="${SOURCES}/pool/main/d/debian-archive-keyring/debian-archive-keyring_2017.5+deb9u1_all.deb"
	wget -O keyring.deb "$debian_archive_keyring_deb"
	ar -x keyring.deb && rm -f control.tar.gz debian-binary && rm -f keyring.deb
	DATA=$(ls data.tar.*) && compress=${DATA#data.tar.}

	KR=debian-archive-keyring.gpg
	bsdtar --include ./usr/share/keyrings/$KR --strip-components 4 -xvf "$DATA"
	rm -f "$DATA"

	apt-get -y install debootstrap qemu-user-static

	qemu-debootstrap --arch=${ROOTFS_ARCH} --keyring=$TEMP/$KR $dist rootfs ${SOURCES}
	rm -f $KR

	# keeping things clean as this is copied later again
#	rm -f rootfs/usr/bin/qemu-arm-static
       if [ $ROOTFS_ARCH = "arm64" ]; then 
               rm -f rootfs/usr/bin/qemu-aarch64-static
       elif [ $ROOTFS_ARCH = "armhf" ]; then
               rm -f rootfs/usr/bin/qemu-arm-static
       fi

	bsdtar -C $TEMP/rootfs -a -cf $tgz .
	rm -fr $TEMP/rootfs

	cd -
}

do_chroot() {
	# Add qemu emulation.
#	cp /usr/bin/qemu-arm-static "$DEST/usr/bin"
       if [ $ARCH = "arm64" ]; then
               cp /usr/bin/qemu-aarch64-static "$DEST/usr/bin"
       elif [ $ARCH = "arm" ]; then
               cp /usr/bin/qemu-arm-static "$DEST/usr/bin"
       fi

	cmd="$@"
	chroot "$DEST" mount -t proc proc /proc || true
	chroot "$DEST" mount -t sysfs sys /sys || true
	chroot "$DEST" $cmd
	chroot "$DEST" umount /sys
	chroot "$DEST" umount /proc

	# Clean up
	rm -f "$DEST/usr/bin/qemu-arm-static"
}

do_conffile() {
        mkdir -p $DEST/opt/boot
	if [ "${PLATFORM}" = "OrangePiH3" ]; then
        	cp $EXTER/install_to_emmc_$OS $DEST/usr/local/sbin/install_to_emmc -f
        	cp $EXTER/uboot/*.bin $DEST/opt/boot/ -f
        	cp $EXTER/resize_rootfs.sh $DEST/usr/local/sbin/ -f
	elif [ "${PLATFORM}" = "OrangePiH3_mainline" ]; then
		cp $BUILD/uboot/u-boot-sunxi-with-spl.bin-${BOARD} $DEST/opt/boot/u-boot-sunxi-with-spl.bin -f
        	cp $EXTER/mainline/install_to_emmc_$OS $DEST/usr/local/sbin/install_to_emmc -f
        	cp $EXTER/mainline/resize_rootfs.sh $DEST/usr/local/sbin/ -f
        	cp $EXTER/mainline/boot_emmc/* $DEST/opt/boot/ -f
	elif [ "${PLATFORM}" = "OrangePiRK3399" ]; then
		cp $EXTER/install_to_emmc_$OS $DEST/usr/local/sbin/install_to_emmc -f
		mkdir -p $DEST/usr/local/lib/install_to_emmc
		cp $BUILD/uboot/*.img $DEST/usr/local/lib/install_to_emmc/ -f
		cp $BUILD/kernel/boot.img $DEST/usr/local/lib/install_to_emmc/ -f
		[ -d $DEST/system/etc/firmware ] || mkdir -p $DEST/system/etc/firmware
		cp -rf $EXTER/firmware/* $DEST/system/etc/firmware
		cp -rf $EXTER/asound.state $DEST/var/lib/alsa/
		echo "" > $DEST/etc/fstab
	elif [ "${PLATFORM}" = "OrangePi4G-iot" -o "${PLATFORM}" = "OrangePi3G-iot" ]; then
		mkdir -p ${DEST}/system/etc/firmware
        	mkdir -p ${DEST}/etc/firmware
        	cp -rfa ${EXTER}/firmware/* ${DEST}/system/etc/firmware
        	cp -rfa ${EXTER}/firmware/* ${DEST}/etc/firmware
        	cp ${EXTER}/6620_launcher ${DEST}/usr/local/sbin
        	cp ${EXTER}/wmt_loader ${DEST}/usr/local/sbin
        	cp ${EXTER}/rc.local ${DEST}/etc/
        	echo "ttyMT0" >> ${DEST}/etc/securetty
	else
	        echo -e "\e[1;31m Pls select correct platform \e[0m"
	        exit 0
	fi

        #cp $EXTER/sshd_config $DEST/etc/ssh/ -f
        #cp $EXTER/profile_for_root $DEST/root/.profile -f
        #cp $EXTER/bluetooth/bt.sh $DEST/usr/local/sbin/ -f
        #cp $EXTER/bluetooth/brcm_patchram_plus/brcm_patchram_plus $DEST/usr/local/sbin/ -f
        #chmod +x $DEST/usr/local/sbin/*
}

add_ssh_keygen_service() {
	cat > "$DEST/etc/systemd/system/ssh-keygen.service" <<EOF
[Unit]
Description=Generate SSH keys if not there
Before=ssh.service
ConditionPathExists=|!/etc/ssh/ssh_host_key
ConditionPathExists=|!/etc/ssh/ssh_host_key.pub
ConditionPathExists=|!/etc/ssh/ssh_host_rsa_key
ConditionPathExists=|!/etc/ssh/ssh_host_rsa_key.pub
ConditionPathExists=|!/etc/ssh/ssh_host_dsa_key
ConditionPathExists=|!/etc/ssh/ssh_host_dsa_key.pub
ConditionPathExists=|!/etc/ssh/ssh_host_ecdsa_key
ConditionPathExists=|!/etc/ssh/ssh_host_ecdsa_key.pub
ConditionPathExists=|!/etc/ssh/ssh_host_ed25519_key
ConditionPathExists=|!/etc/ssh/ssh_host_ed25519_key.pub

[Service]
ExecStart=/usr/bin/ssh-keygen -A
Type=oneshot
RemainAfterExit=yes

[Install]
WantedBy=ssh.service
EOF
	do_chroot systemctl enable ssh-keygen
}

add_opi_python_gpio_libs() {
        cp $EXTER/packages/OPi.GPIO $DEST/usr/local/sbin/ -rfa
        cp $EXTER/packages/OPi.GPIO/test_gpio.py $DEST/usr/local/sbin/ -f

        cat > "$DEST/install_opi_gpio" <<EOF
#!/bin/bash
apt update
apt-get install -y python3-pip python3-setuptools
cd /usr/local/sbin/OPi.GPIO
python3 setup.py install
EOF
        chmod +x "$DEST/install_opi_gpio"
        do_chroot /install_opi_gpio
	rm $DEST/install_opi_gpio
}

add_bt_service() {
        cat > "$DEST/lib/systemd/system/bt.service" <<EOF
[Unit]
Description=OrangePi BT Service

[Service]
ExecStart=/usr/local/sbin/bt.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
        do_chroot systemctl enable bt.service
}


add_opi_config_libs() {
	do_chroot apt-get install -y dialog expect bc cpufrequtils figlet toilet lsb-release
        cp $EXTER/packages/opi_config_libs $DEST/usr/local/sbin/ -rfa
        cp $EXTER/packages/opi_config_libs/opi-config $DEST/usr/local/sbin/ -rfa

	rm -rf $DEST/etc/update-motd.d/* 
        cp $EXTER/packages/opi_config_libs/overlay/* $DEST/ -rf
}

add_debian_apt_sources() {
	local release="$1"
	local aptsrcfile="$DEST/etc/apt/sources.list"
	cat > "$aptsrcfile" <<EOF
deb ${SOURCES} ${release} main contrib non-free
#deb-src ${SOURCES} ${release} main contrib non-free
EOF
	# No separate security or updates repo for unstable/sid
	[ "$release" = "sid" ] || cat >> "$aptsrcfile" <<EOF
deb ${SOURCES} ${release}-updates main contrib non-free
#deb-src ${SOURCES} ${release}-updates main contrib non-free

deb http://security.debian.org/ ${release}/updates main contrib non-free
#deb-src http://security.debian.org/ ${release}/updates main contrib non-free
EOF
}

add_ubuntu_apt_sources() {
	local release="$1"
	cat > "$DEST/etc/apt/sources.list" <<EOF
deb ${SOURCES} ${release} main restricted universe multiverse
deb-src ${SOURCES} ${release} main restricted universe multiverse

deb ${SOURCES} ${release}-updates main restricted universe multiverse
deb-src ${SOURCES} ${release}-updates main restricted universe multiverse

deb ${SOURCES} ${release}-security main restricted universe multiverse
deb-src $SOURCES ${release}-security main restricted universe multiverse

deb ${SOURCES} ${release}-backports main restricted universe multiverse
deb-src ${SOURCES} ${release}-backports main restricted universe multiverse
EOF
}

prepare_env()
{
	if [ ${ARCH} = "arm" ];then
                QEMU="/usr/bin/qemu-arm-static"
                ROOTFS_ARCH="armhf"
        elif [ ${ARCH} = "arm64" ];then
                QEMU="/usr/bin/qemu-aarch64-static"
                ROOTFS_ARCH="arm64"
        fi

	if [ ! -d "$DEST" ]; then
		echo "Destination $DEST not found or not a directory."
		echo "Create $DEST"
		mkdir -p $DEST
	fi

	if [ "$(ls -A -Ilost+found $DEST)" ]; then
		echo "Destination $DEST is not empty."
		echo "Clean up space."
		rm -rf $DEST
	fi

	cleanup() {
		if [ -e "$DEST/proc/cmdline" ]; then
			umount "$DEST/proc"
		fi
		if [ -d "$DEST/sys/kernel" ]; then
			umount "$DEST/sys"
		fi
		if [ -d "$TEMP" ]; then
			rm -rf "$TEMP"
		fi
	}
	trap cleanup EXIT

	case $DISTRO in
		xenial | bionic)
			case $SOURCES in
				"CDN"|"OFCL")
			       	        SOURCES="http://ports.ubuntu.com"
					ROOTFS="http://cdimage.ubuntu.com/ubuntu-base/releases/${DISTRO}/release/ubuntu-base-${DISTRO_NUM}-base-${ROOTFS_ARCH}.tar.gz"
				        ;;
				"CN")
				        #SOURCES="http://mirrors.aliyun.com/ubuntu-ports"
		                        #SOURCES="http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports"
				        SOURCES="http://mirrors.ustc.edu.cn/ubuntu-ports"
					ROOTFS="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-cdimage/ubuntu-base/releases/${DISTRO}/release/ubuntu-base-${DISTRO_NUM}-base-${ROOTFS_ARCH}.tar.gz"
				        ;;
				*)
					SOURCES="http://ports.ubuntu.com"
					ROOTFS="http://cdimage.ubuntu.com/ubuntu-base/releases/${DISTRO}/release/ubuntu-base-${DISTRO_NUM}-base-${ROOTFS_ARCH}.tar.gz"
					;;
			esac
			;;

		stretch)
			ROOTFS="${DISTRO}-base-${ROOTFS_ARCH}.tar.gz"
			METHOD="debootstrap"
			case $SOURCES in
		                "CDN")
		                        SOURCES="http://archive.debian.org/debian"
		                        ;;
		                "OFCL")
		                        SOURCES="http://archive.debian.org/debian"
		                        ;;
		                "CN")
		                        SOURCES="http://archive.debian.org/debian"
		                        ;;
				*)
					SOURCES="http://archive.debian.org/debian"
		                        ;;
		        esac
			;;
		*)
			echo "Unknown distribution: $DISTRO"
			exit 1
			;;
	esac

	TARBALL="$EXTER/$(basename $ROOTFS)"
	if [ ! -e "$TARBALL" ]; then
		if [ "$METHOD" = "download" ]; then
			echo "Downloading $DISTRO rootfs tarball ..."
			wget -O "$TARBALL" "$ROOTFS"
		elif [ "$METHOD" = "debootstrap" ]; then
			deboostrap_rootfs "$DISTRO" "$TARBALL"
		else
			echo "Unknown rootfs creation method"
			exit 1
		fi
	fi

	# Extract with BSD tar
	echo -n "Extracting ... "
	mkdir -p $DEST
	$UNTAR "$TARBALL" -C "$DEST"
	echo "OK"
}

prepare_rootfs_server()
{

	rm "$DEST/etc/resolv.conf"
	cp /etc/resolv.conf "$DEST/etc/resolv.conf"
	if [ "$DISTRO" = "xenial" -o "$DISTRO" = "bionic" ]; then
		DEB=ubuntu
		DEBUSER=orangepi
		EXTRADEBS="software-properties-common libjpeg8-dev usbmount zram-config ubuntu-minimal net-tools"
		ADDPPACMD=
		DISPTOOLCMD=
	elif [ "$DISTRO" = "sid" -o "$DISTRO" = "stretch" -o "$DISTRO" = "stable" ]; then
		DEB=debian
		DEBUSER=orangepi
		EXTRADEBS="sudo net-tools g++ libjpeg-dev"
		ADDPPACMD=
		DISPTOOLCMD=
	else
		echo "Unknown DISTRO=$DISTRO"
		exit 2
	fi
	add_${DEB}_apt_sources $DISTRO
	rm -rf "$DEST/etc/apt/sources.list.d/proposed.list"
	cat > "$DEST/second-phase" <<EOF
#!/bin/bash
export DEBIAN_FRONTEND=noninteractive
locale-gen en_US.UTF-8

apt-get -y update
apt-get -y install dosfstools curl xz-utils iw rfkill ifupdown
apt-get -y install wpasupplicant openssh-server alsa-utils
apt-get -y install rsync u-boot-tools vim
apt-get -y install parted network-manager git autoconf gcc libtool
apt-get -y install libsysfs-dev pkg-config libdrm-dev xutils-dev hostapd
apt-get -y install dnsmasq apt-transport-https man subversion
apt-get -y install imagemagick libv4l-dev cmake bluez
apt-get -y install $EXTRADEBS

apt-get install -f

apt-get -y remove --purge ureadahead
$ADDPPACMD
apt-get -y update
$DISPTOOLCMD
adduser --gecos $DEBUSER --disabled-login $DEBUSER --uid 1000
adduser --gecos root --disabled-login root --uid 0
echo root:orangepi | chpasswd
chown -R 1000:1000 /home/$DEBUSER
echo "$DEBUSER:$DEBUSER" | chpasswd
usermod -a -G sudo $DEBUSER
usermod -a -G adm $DEBUSER
usermod -a -G video $DEBUSER
usermod -a -G plugdev $DEBUSER
apt-get -y autoremove
apt-get clean
EOF
	chmod +x "$DEST/second-phase"
	do_chroot /second-phase
	rm -f "$DEST/second-phase"
        rm -f "$DEST/etc/resolv.conf"

	cd $BUILD
	tar czf ${DISTRO}_server_rootfs.tar.gz rootfs
	cd -
}

prepare_rootfs_desktop()
{
	install_lxde_desktop
	cd $BUILD
	tar czf ${DISTRO}_desktop_rootfs.tar.gz rootfs
	cd -
				
}

server_setup()
{
	if [ $BOARD = "zero_plus2_h3" ];then
		:
	else
	cat > "$DEST/etc/network/interfaces.d/eth0" <<EOF
auto eth0
iface eth0 inet dhcp
EOF
	fi
	cat > "$DEST/etc/hostname" <<EOF
orangepi${BOARD}
EOF
	cat > "$DEST/etc/hosts" <<EOF
127.0.0.1 localhost
127.0.1.1 orangepi${BOARD}

# The following lines are desirable for IPv6 capable hosts
::1     localhost ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
EOF
	cat > "$DEST/etc/resolv.conf" <<EOF
nameserver 8.8.8.8
EOF

	do_conffile
	add_ssh_keygen_service
	#add_opi_python_gpio_libs
	#add_opi_config_libs
	#add_bt_service
	sed -i 's|After=rc.local.service|#\0|;' "$DEST/lib/systemd/system/serial-getty@.service"
	rm -f "$DEST"/etc/ssh/ssh_host_*

	# Bring back folders
	mkdir -p "$DEST/lib"
	mkdir -p "$DEST/usr"

	# Create fstab
	cat  > "$DEST/etc/fstab" <<EOF
# <file system>	<dir>	<type>	<options>			<dump>	<pass>
/dev/mmcblk0p1	/boot	vfat	defaults			0		2
/dev/mmcblk0p2	/	ext4	defaults,noatime		0		1
EOF
	if [ ! -d $DEST/lib/modules ]; then
		mkdir "$DEST/lib/modules"
	else
		rm -rf $DEST/lib/modules
		mkdir "$DEST/lib/modules"
	fi

	if [ $PLATFORM = "OrangePiRK3399" ]; then
		echo "" > $DEST/etc/fstab
		echo "ttyFIQ0" >> $DEST/etc/securetty
		sed -i '/^TimeoutStartSec=/s/5min/15sec/' $DEST/lib/systemd/system/networking.service
		setup_resize-helper
	elif [ ${PLATFORM} = "OrangePi3G-iot" ]; then
                echo "" > $DEST/etc/fstab
		setup_resize-helper
	fi
	# Install Kernel modules
	make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS O=$BUILD/obj/KERNEL_OBJ modules_install INSTALL_MOD_PATH="$DEST"

	# Install Kernel headers
	make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS O=$BUILD/obj/KERNEL_OBJ headers_install INSTALL_HDR_PATH="$DEST/usr/local"
	#cp $EXTER/firmware $DEST/lib/ -rf

	#rm -rf $BUILD/${DISTRO}_${IMAGETYPE}_rootfs
	#cp -rfa $DEST $BUILD/${DISTRO}_${IMAGETYPE}_rootfs
}

desktop_setup()
{
	if [ $PLATFORM = "OrangePiRK3399" ]; then
		sed -i '/^wallpaper=/s/\/etc\/alternatives\/desktop-background/\/usr\/share\/lxde\/wallpapers\/newxitong_17.jpg/' $DEST/etc/xdg/pcmanfm/LXDE/pcmanfm.conf
		sed -i '/^[ ]*transparent=/s/0/1/' $DEST/etc/xdg/lxpanel/LXDE/panels/panel
		sed -i '/^[ ]*background=/s/1/0/' $DEST/etc/xdg/lxpanel/LXDE/panels/panel

		echo -e "\n[keyfile]\nunmanaged-devices=*,except:type:ethernet,except:type:wifi,except:type:wwan" >> ${DEST}/etc/NetworkManager/NetworkManager.conf
		[ $DISTRO = "stretch" ] && echo -e "\n[device]\nwifi.scan-rand-mac-address=no" >> $DEST/etc/NetworkManager/NetworkManager.conf
		[ $DISTRO = "stretch" ] && cp -rfa $EXTER/packages/others/glmark2/* $DEST
		[ $DISTRO = "xenial" ] && setup_front
		cp -rfa $EXTER/packages $DEST
		cp -rfa $EXTER/packages/overlay/* $DEST
		install_gstreamer
		install_gpu_lib
		rm -rf $DEST/packages
	fi

}

build_rootfs()
{
	prepare_env

	if [ $TYPE = "1" ]; then
		if [ -f $BUILD/${DISTRO}_desktop_rootfs.tar.gz ]; then
			rm -rf $DEST
			tar zxf $BUILD/${DISTRO}_desktop_rootfs.tar.gz -C $BUILD
		else
			if [ -f $BUILD/${DISTRO}_server_rootfs.tar.gz ]; then
				rm -rf $DEST
				tar zxf $BUILD/${DISTRO}_server_rootfs.tar.gz -C $BUILD
				prepare_rootfs_desktop
			else
				prepare_rootfs_server
				prepare_rootfs_desktop

			fi
		fi
		server_setup
		desktop_setup
	else
		if [ -f $BUILD/${DISTRO}_server_rootfs.tar.gz ]; then
			rm -rf $DEST
			tar zxf $BUILD/${DISTRO}_server_rootfs.tar.gz -C $BUILD
		else
			prepare_rootfs_server
		fi
		server_setup
	fi
}

