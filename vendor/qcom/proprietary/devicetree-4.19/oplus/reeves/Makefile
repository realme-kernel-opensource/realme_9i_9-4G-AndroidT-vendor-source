ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_KHAJE) += \
				khaje-reeves-21029-overlay.dtbo\
				khaje-reeves-21030-overlay.dtbo\
				khaje-reeves-21029-overlay-EVT.dtbo\
				khaje-reeves-21030-overlay-EVT.dtbo\
				khaje-reeves-21989-overlay.dtbo

khaje-reeves-21029-overlay.dtbo-base := khaje-reeves.dtb
khaje-reeves-21030-overlay.dtbo-base := khaje-reeves.dtb
khaje-reeves-21029-overlay-EVT.dtbo-base := khaje-reeves.dtb
khaje-reeves-21030-overlay-EVT.dtbo-base := khaje-reeves.dtb
khaje-reeves-21989-overlay.dtbo-base := khaje-reeves.dtb

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo
