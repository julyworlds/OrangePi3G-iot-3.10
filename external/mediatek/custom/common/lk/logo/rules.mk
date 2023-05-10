LOCAL_DIR := $(GET_LOCAL_DIR)
BOOT_LOGO_DIR := $(LOCAL_DIR)
#$(info lk/logo/dir=$(LOCAL_DIR),builddir=$(BUILDDIR))

BMP_TO_RAW := $(BOOT_LOGO_DIR)/../logo_tool/bmp_to_raw
ZPIPE := $(BOOT_LOGO_DIR)/../logo_tool/zpipe
            
$(info BMP_TO_RAW=$(BMP_TO_RAW))
$(info ZPIPE=$(ZPIPE))
$(info CUSTOM_LK_LOGO=$(CUSTOM_LK_LOGO))
$(info BOOT_LOGO_DIR =$(BOOT_LOGO_DIR))

BOOT_LOGO_RESOURCE := $(BUILDDIR)/$(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO).raw
LOGO_IMAGE := $(BUILDDIR)/logo.bin
RESOURCE_OBJ_LIST :=   \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_uboot.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_battery.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_low_battery.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_charger_ov.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_0.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_1.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_2.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_3.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_4.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_5.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_6.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_7.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_8.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_9.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_num_percent.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_animation_01.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_animation_02.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_animation_03.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_animation_04.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_animation_05.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_animation_06.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_animation_07.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_animation_08.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_animation_09.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_animation_10.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_10_01.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_10_02.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_10_03.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_10_04.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_10_05.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_10_06.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_10_07.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_10_08.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_10_09.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_10_10.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_bg.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_img.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_bat_100.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_kernel.raw 

ifeq ($(strip $(MTK_PUMP_EXPRESS_SUPPORT)), yes)
RESOURCE_OBJ_LIST +=   \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_100.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_ani-01.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_ani-02.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_ani-03.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_ani-04.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_ani-05.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_ani-06.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_00.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_01.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_02.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_03.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_04.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_05.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_06.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_07.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_08.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_09.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_fast_charging_percent.raw 
endif

ifeq ($(strip $(MTK_WIRELESS_CHARGER_SUPPORT)), yes)
RESOURCE_OBJ_LIST +=   \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_00.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_01.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_02.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_03.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_04.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_05.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_06.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_07.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_08.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_09.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_num_percent.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_10_0.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_10_1.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_10_2.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_10_3.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_30_0.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_30_1.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_30_2.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_30_3.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_60_0.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_60_1.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_60_2.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_60_3.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_90_0.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_90_1.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_90_2.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_90_3.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_0.raw \
            $(BOOT_LOGO_DIR)/$(CUSTOM_LK_LOGO)_wireless_bat_100.raw 

endif  
                                      
GENERATED += \
            $(BOOT_LOGO_RESOURCE) \
            $(LOGO_IMAGE) \
            $(addprefix $(BUILDDIR)/,$(RESOURCE_OBJ_LIST))


all:: $(LOGO_IMAGE) 

$(LOGO_IMAGE):$(MKIMG) $(BOOT_LOGO_RESOURCE)
	$(NOECHO) if [ ! -x $(MKIMG) ]; then chmod a+x $(MKIMG); fi
	@echo "MKING $(LOGO_IMAGE)"
	$(MKIMG) $(BOOT_LOGO_RESOURCE) LOGO > $(LOGO_IMAGE)

$(BOOT_LOGO_RESOURCE): $(addprefix $(BUILDDIR)/,$(RESOURCE_OBJ_LIST)) $(ZPIPE)
	@$(MKDIR)
	$(NOECHO) if [ ! -x $(ZPIPE) ]; then chmod a+x $(ZPIPE); fi
	@echo "zpiping "
	$(ZPIPE) -l 9 $@ $(addprefix $(BUILDDIR)/,$(RESOURCE_OBJ_LIST))


$(BUILDDIR)/%.raw: %.bmp $(BMP_TO_RAW)
	@$(MKDIR)
	$(NOECHO) if [ ! -x $(BMP_TO_RAW) ]; then chmod a+x $(BMP_TO_RAW); fi
	@echo "Compiling_BMP_TO_RAW $<"
	$(BMP_TO_RAW) $@ $<

