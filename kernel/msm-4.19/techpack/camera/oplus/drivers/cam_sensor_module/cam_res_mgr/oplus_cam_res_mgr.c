
#include "oplus_cam_res_mgr.h"

extern struct cam_res_mgr *cam_res;
extern bool cam_res_mgr_gpio_is_shared(uint gpio);
extern int cam_res_mgr_add_device(struct device *dev,struct cam_gpio_res *gpio_res);
bool oplus_cam_res_mgr_gpio_is_shared(uint gpio)
{
	int index = 0;
	bool found = false;
	struct cam_res_mgr_dt *dt = &cam_res->dt;

	for (; index < dt->num_shared_gpio; index++) {
		if (gpio == dt->shared_gpio[index]) {
			found = true;
			break;
		}
	}

	return found;
}


int oplus_cam_res_mgr_gpio_request(struct device *dev, uint gpio,
		unsigned long flags, const char *label)
{

	int rc = 0;
	bool found = false;
	struct cam_gpio_res *gpio_res = NULL;

	mutex_lock(&cam_res->gpio_res_lock);
	if (cam_res && cam_res->shared_gpio_enabled) {
		list_for_each_entry(gpio_res, &cam_res->gpio_res_list, list) {
			if (gpio == gpio_res->gpio) {
				found = true;
				break;
			}
		}
	}

	/*
	 * found equal to false has two situation:
	 * 1. shared gpio not enabled
	 * 2. shared gpio enabled, but not find this gpio
	 *    from the gpio_res_list
	 * These two situation both need request gpio.
	 */
	CAM_DBG(CAM_RES, "shared gpio enable is %d shared gpio num is %d",
		cam_res->shared_gpio_enabled,
		cam_res->dt.num_shared_gpio);

	if (!found) {
		rc = gpio_request_one(gpio, flags, label);
		if (rc) {
			CAM_ERR(CAM_RES, "gpio %d:%s request fails",
				gpio, label);
			mutex_unlock(&cam_res->gpio_res_lock);
			return rc;
		}
	}

	/*
	 * If the gpio is in the shared list, and not find
	 * from gpio_res_list, then insert a cam_gpio_res
	 * to gpio_res_list.
	 */
	if (!found && cam_res
		&& cam_res->shared_gpio_enabled &&
		cam_res_mgr_gpio_is_shared(gpio)) {

		gpio_res = kzalloc(sizeof(struct cam_gpio_res), GFP_KERNEL);
		if (!gpio_res) {
			mutex_unlock(&cam_res->gpio_res_lock);
			return -ENOMEM;
		}

		gpio_res->gpio = gpio;
		gpio_res->power_on_count = 0;
		INIT_LIST_HEAD(&gpio_res->list);
		INIT_LIST_HEAD(&gpio_res->dev_list);

		rc = cam_res_mgr_add_device(dev, gpio_res);
		if (rc) {
			kfree(gpio_res);
			mutex_unlock(&cam_res->gpio_res_lock);
			return rc;
		}

		list_add_tail(&gpio_res->list, &cam_res->gpio_res_list);
	}

	if (found && cam_res
		&& cam_res->shared_gpio_enabled) {
		struct cam_dev_res *dev_res = NULL;

		found = 0;
		list_for_each_entry(dev_res, &gpio_res->dev_list, list) {
			if (dev_res->dev == dev) {
				found = 1;
				break;
			}
		}

		if (!found)
			rc = cam_res_mgr_add_device(dev, gpio_res);

	}

	mutex_unlock(&cam_res->gpio_res_lock);
	return rc;
}

