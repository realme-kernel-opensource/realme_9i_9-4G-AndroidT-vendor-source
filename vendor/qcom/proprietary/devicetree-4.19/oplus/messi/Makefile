ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_KHAJE) += \
				khaje-panther-2172F-overlay.dtbo\
				khaje-panther-21730-overlay.dtbo\
				khaje-panther-21731-overlay.dtbo

khaje-panther-2172F-overlay.dtbo-base := khaje-panther.dtb
khaje-panther-21730-overlay.dtbo-base := khaje-panther.dtb
khaje-panther-21731-overlay.dtbo-base := khaje-panther.dtb

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo
