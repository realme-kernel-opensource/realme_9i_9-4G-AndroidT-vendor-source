ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_BENGAL) += \
                bengal-soda-20021-overlay.dtbo \
                bengal-soda-20021-overlay-mp.dtbo \
                bengal-soda-20221-overlay.dtbo

bengal-soda-20021-overlay.dtbo-base := bengal-soda.dtb
bengal-soda-20021-overlay-mp.dtbo-base := bengal-soda.dtb
bengal-soda-20221-overlay.dtbo-base := bengal-soda.dtb

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo
