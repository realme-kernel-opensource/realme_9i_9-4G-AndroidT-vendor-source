#ifndef OPLUS_CAM_FLASH_SOC_H
#define OPLUS_CAM_FLASH_SOC_H

#include <linux/of.h>
#include <linux/of_gpio.h>
#include "cam_res_mgr_api.h"
#include "cam_flash_soc.h"

int cam_i2c_flash_get_dt_data(struct cam_flash_ctrl *fctrl,
       struct cam_hw_soc_info *soc_info);
#endif //OPLUS_CAM_FLASH_SOC_H
