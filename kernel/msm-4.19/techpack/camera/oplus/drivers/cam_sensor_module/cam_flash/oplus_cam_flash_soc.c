
#include "oplus_cam_flash_soc.h"


int cam_i2c_flash_get_dt_data(struct cam_flash_ctrl *fctrl,
    struct cam_hw_soc_info *soc_info)
{
    int32_t rc = 0;

    if (!fctrl) {
        CAM_ERR(CAM_FLASH, "NULL flash control structure");
        return -EINVAL;
    }

    soc_info->soc_private =
        kzalloc(sizeof(struct cam_flash_private_soc), GFP_KERNEL);
    if (!soc_info->soc_private) {
        rc = -ENOMEM;
        goto release_soc_res;
    }

    rc = cam_soc_util_get_dt_properties(soc_info);
    if (rc) {
        CAM_ERR(CAM_FLASH, "Get_dt_properties failed rc %d", rc);
        goto free_soc_private;
    }

    rc =  cam_sensor_util_init_gpio_pin_tbl(soc_info,
            &fctrl->power_info.gpio_num_info);
    if (rc < 0) {
        CAM_ERR(CAM_FLASH, "Failed to read gpios %d", rc);
        goto free_soc_private;
    }
        return rc;

free_soc_private:
    kfree(soc_info->soc_private);
    soc_info->soc_private = NULL;
release_soc_res:
    cam_soc_util_release_platform_resource(soc_info);
    return rc;
}
