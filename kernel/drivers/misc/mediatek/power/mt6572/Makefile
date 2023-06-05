include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/power/mediatek/$(CONFIG_MTK_PLATFORM)/$(ARCH_MTK_PROJECT)

obj-$(CONFIG_MTK_PMIC)          += pmic_mt6323.o upmu_common.o

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o 

ifeq ($(CONFIG_MTK_BQ24196_SUPPORT),y) 
    obj-$(CONFIG_MTK_SMART_BATTERY) += mt_battery_bq24196.o mt6320_fuel_gauge_bq24196.o bq24196.o
    ifeq ($(CONFIG_MTK_BQ27541_SUPPORT),y)
      obj-$(CONFIG_MTK_SMART_BATTERY) += bq27541.o
    endif
else
  ifeq ($(CONFIG_MTK_NCP1851_SUPPORT),y) 
    obj-$(CONFIG_MTK_SMART_BATTERY) += mt_battery_ncp1851.o mt6320_fuel_gauge_ncp1851.o ncp1851.o
  else
      ifeq ($(CONFIG_MTK_FAN5405_SUPPORT),y)
           obj-$(CONFIG_MTK_SMART_BATTERY) += fan5405.o charging_hw_fan5405.o
      else
           ifeq ($(CONFIG_MTK_BQ24158_SUPPORT),y)
                obj-$(CONFIG_MTK_SMART_BATTERY) += bq24158.o charging_hw_bq24158.o
           else
              ifeq ($(CONFIG_MTK_SNB1058_SUPPORT),y)     
                  obj-$(CONFIG_MTK_SMART_BATTERY) += snb1058/
              else
                  obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_pmic.o 
              endif
	   endif
      endif
  endif
endif
