ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_KHAJE) += \
				khaje-kasa-216C1-overlay.dtbo\
				khaje-kasa-216C2-overlay.dtbo\
				khaje-kasa-216C3-overlay.dtbo\
				khaje-kasa-216C4-overlay.dtbo\
				khaje-kasa-216C5-overlay.dtbo\
				khaje-kasa-216CB-overlay.dtbo

khaje-kasa-216C1-overlay.dtbo-base := khaje-kasa.dtb
khaje-kasa-216C2-overlay.dtbo-base := khaje-kasa.dtb
khaje-kasa-216C3-overlay.dtbo-base := khaje-kasa.dtb
khaje-kasa-216C4-overlay.dtbo-base := khaje-kasa.dtb
khaje-kasa-216C5-overlay.dtbo-base := khaje-kasa.dtb
khaje-kasa-216CB-overlay.dtbo-base := khaje-kasa.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo
