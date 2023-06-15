ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_BENGAL) += \
                bengal-beta-20241-overlay.dtbo \
				bengal-beta-20243-overlay.dtbo \
				bengal-beta-21233-overlay.dtbo \
				bengal-beta-21261-overlay.dtbo \
				bengal-beta-21262-overlay.dtbo \
				bengal-beta-21263-overlay.dtbo \
				bengal-beta-21091-overlay.dtbo

bengal-beta-20241-overlay.dtbo-base := bengal-beta.dtb
bengal-beta-20243-overlay.dtbo-base := bengal-beta.dtb
bengal-beta-21233-overlay.dtbo-base := bengal-beta.dtb
bengal-beta-21261-overlay.dtbo-base := bengal-beta.dtb
bengal-beta-21262-overlay.dtbo-base := bengal-beta.dtb
bengal-beta-21263-overlay.dtbo-base := bengal-beta.dtb
bengal-beta-21091-overlay.dtbo-base := bengal-beta.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo
