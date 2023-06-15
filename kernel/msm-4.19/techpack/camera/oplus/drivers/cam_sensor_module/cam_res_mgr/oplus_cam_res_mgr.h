
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_res_mgr_private.h"


bool oplus_cam_res_mgr_gpio_is_shared(uint gpio);
int oplus_cam_res_mgr_gpio_request(struct device *dev, uint gpio,
		unsigned long flags, const char *label);
