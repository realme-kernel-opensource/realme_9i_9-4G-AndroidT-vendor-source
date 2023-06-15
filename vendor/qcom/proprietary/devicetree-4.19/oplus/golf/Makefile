ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_KHAJE) += \
				khaje-golf-21261-overlay.dtbo\
				khaje-golf-21262-overlay.dtbo\
				khaje-golf-21263-overlay.dtbo\
				khaje-golf-21265-overlay.dtbo\
				khaje-golf-21266-overlay.dtbo\
				khaje-golf-21267-overlay.dtbo\
				khaje-golf-21268-overlay.dtbo\
				khaje-golf-21091-overlay.dtbo

khaje-golf-21261-overlay.dtbo-base := khaje-golf.dtb
khaje-golf-21262-overlay.dtbo-base := khaje-golf.dtb
khaje-golf-21263-overlay.dtbo-base := khaje-golf.dtb
khaje-golf-21265-overlay.dtbo-base := khaje-golf.dtb
khaje-golf-21266-overlay.dtbo-base := khaje-golf.dtb
khaje-golf-21267-overlay.dtbo-base := khaje-golf.dtb
khaje-golf-21268-overlay.dtbo-base := khaje-golf.dtb
khaje-golf-21091-overlay.dtbo-base := khaje-golf.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo
