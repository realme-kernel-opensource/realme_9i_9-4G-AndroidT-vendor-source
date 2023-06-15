ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-$(CONFIG_ARCH_KHAJE) += khaje-zhaoyun-a-22071-overlay.dtbo

	khaje-zhaoyun-a-22071-overlay.dtbo-base := khaje-zhaoyun-a.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo

