
#include "oplus_cam_flash_core.h"
extern  int cam_flash_high(struct cam_flash_ctrl *flash_ctrl,struct cam_flash_frame_setting *flash_data);
extern  int cam_flash_low(struct cam_flash_ctrl *flash_ctrl,struct cam_flash_frame_setting *flash_data);

int cam_flash_on(struct cam_flash_ctrl *flash_ctrl,
	struct cam_flash_frame_setting *flash_data,
	int mode) {
	int rc = 0;
	if (mode == 0) {
		if (flash_ctrl->flash_type == 2) {
			cam_flash_gpio_power_ops(flash_ctrl,true);
			rc = cam_flash_gpio_low(flash_ctrl, flash_data);
		} else {
			rc = cam_flash_low(flash_ctrl, flash_data);
		}
	} else if (mode == 1) {
		if (flash_ctrl->flash_type == 2) {
			cam_flash_gpio_power_ops(flash_ctrl,true);
			rc = cam_flash_gpio_high(flash_ctrl, flash_data);
		} else {
			rc = cam_flash_high(flash_ctrl, flash_data);
		}
	}
	return rc;
}

