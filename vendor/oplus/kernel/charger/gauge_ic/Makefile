obj-$(CONFIG_OPLUS_SM5602_GAUGE)	+= sm5602_fg.o
obj-$(CONFIG_OPLUS_CW2017_GAUGE)	+= cw2017_fuel_gauge_base.o
obj-$(CONFIG_OPLUS_CW2217_GAUGE)	+= oplus_cw2217b.o
obj-y	+= oplus_bq27541.o
ifeq ($(CONFIG_OPLUS_CHARGER_MTK6877),y)
	obj-y   += oplus_optiga/
endif
#add for 21061 optiga
ifeq ($(CONFIG_OPLUS_CHARGER_MTK6893),y)
	obj-y   += oplus_optiga/
endif
ifeq ($(CONFIG_OPLUS_CHARGER_OPTIGA),y)
	obj-y   += oplus_optiga/
endif
