ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_BENGAL) += \
                bengal-cocob-20671-overlay.dtbo \
				bengal-coco-20673-overlay.dtbo

bengal-cocob-20671-overlay.dtbo-base := bengal-coco.dtb
bengal-coco-20673-overlay.dtbo-base := bengal-coco.dtb

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo
