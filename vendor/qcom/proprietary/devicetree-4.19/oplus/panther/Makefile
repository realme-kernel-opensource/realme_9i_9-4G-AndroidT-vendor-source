ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_KHAJE) += \
				khaje-panther-21311-overlay.dtbo\
				khaje-panther-21312-overlay.dtbo\
				khaje-panther-21313-overlay.dtbo

khaje-panther-21311-overlay.dtbo-base := khaje-panther.dtb
khaje-panther-21312-overlay.dtbo-base := khaje-panther.dtb
khaje-panther-21313-overlay.dtbo-base := khaje-panther.dtb

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo
