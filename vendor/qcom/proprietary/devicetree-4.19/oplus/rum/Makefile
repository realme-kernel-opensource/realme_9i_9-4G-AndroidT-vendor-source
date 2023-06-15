ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_BENGAL) += \
                bengal-rum-20211-overlay.dtbo \
		bengal-rum-20211-overlay-legacy.dtbo \
                bengal-rum-20214-overlay.dtbo

bengal-rum-20211-overlay.dtbo-base := bengal-rum.dtb
bengal-rum-20211-overlay-legacy.dtbo-base := bengal-rum.dtb
bengal-rum-20214-overlay.dtbo-base := bengal-rum.dtb

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo
