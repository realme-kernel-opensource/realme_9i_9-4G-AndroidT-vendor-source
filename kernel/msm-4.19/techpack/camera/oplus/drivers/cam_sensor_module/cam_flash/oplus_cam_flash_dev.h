#ifndef OPLUS_CAM_FLASH_DEV_H
#define OPLUS_CAM_FLASH_DEV_H

#include <linux/module.h>
#include "cam_flash_dev.h"
#include "cam_flash_soc.h"
#include "cam_flash_core.h"
#include "cam_common_util.h"
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/notifier.h>
#include <linux/kernel.h>
#include <linux/reboot.h>

#include "cam_res_mgr_api.h"
#include "oplus_cam_flash_core.h"

#define TOTAL_FLASH_NUM 5
#define FLASH_POWER_SETTING_SIZE 2
#define IIC_FLASH_INVALID -1
#define IIC_FLASH_OFF 0
#define IIC_FLASH_ON 1
#define IIC_FLASH_HIGH 2

struct cam_flash_ftm_reg_setting {
	struct cam_sensor_i2c_reg_array reg_setting[10];
	unsigned short size;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	unsigned short delay;
};

struct cam_flash_ftm_power_single {
	enum msm_camera_power_seq_type seq_type;
	long config_val;
	unsigned short delay;
};

struct cam_flash_ftm_power_setting {
	struct cam_flash_ftm_power_single single_power[FLASH_POWER_SETTING_SIZE];
	unsigned short size;
};


struct  cam_flash_probe_info {
	char *flash_name;
	uint32_t slave_write_address;
	uint32_t flash_id_address;
	uint16_t flash_id;
	enum camera_sensor_i2c_type  addr_type;
	enum camera_sensor_i2c_type  data_type;
};

struct cam_flash_ftm_settings {
	uint32_t need_standby_mode;                                 /* no physic flash_en gpio we should set standby mode */
	struct cam_flash_probe_info flashprobeinfo;
	struct cam_sensor_cci_client cci_client;
	struct cam_flash_ftm_reg_setting flashinitsettings;
	struct cam_flash_ftm_reg_setting flashhighsettings;
	struct cam_flash_ftm_reg_setting flashlowsettings;
	struct cam_flash_ftm_reg_setting flashoffsettings;
	struct cam_flash_ftm_power_setting flashpowerupsetting;
	struct cam_flash_ftm_power_setting flashpowerdownsetting;
};

struct cam_flash_settings {
	uint32_t total_flash_dev;
	uint32_t flash_type;
	int valid_setting_index;
	int cur_flash_status;
	struct cam_flash_ftm_settings flash_ftm_settings[TOTAL_FLASH_NUM];
};

int cam_ftm_i2c_flash_off(struct cam_flash_ctrl *flash_ctrl, struct cam_flash_ftm_settings *flash_ftm_data);
int cam_ftm_i2c_torch_on(struct cam_flash_ctrl *flash_ctrl, struct cam_flash_ftm_settings *flash_ftm_data);
int cam_ftm_i2c_flash_on(struct cam_flash_ctrl *flash_ctrl, struct cam_flash_ftm_settings *flash_ftm_data);
int cam_flash_match_id(uint32_t except_id, struct cam_flash_ctrl *flash_ctrl, struct cam_flash_ftm_settings *flash_ftm_data);
int cam_ftm_i2c_set_standby(struct cam_flash_ctrl *flash_ctrl, struct cam_flash_ftm_settings *flash_ftm_data);

volatile static int flash_mode;
volatile static int pre_flash_mode;

ssize_t ftm_i2c_flash_on_off(struct cam_flash_ctrl *flash_ctrl);

ssize_t flash_on_off(struct cam_flash_ctrl *flash_ctrl);
ssize_t flash_proc_write(struct file *filp, const char __user *buff,
						size_t len, loff_t *data);
ssize_t flash_proc_read(struct file *filp, char __user *buff,
						size_t len, loff_t *data);
static const struct file_operations led_fops = {
    .owner		= THIS_MODULE,
    .read		= flash_proc_read,
    .write		= flash_proc_write,
};
int flash_proc_init(struct cam_flash_ctrl *flash_ctrl);
ssize_t cam_flash_switch_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count);
ssize_t cam_flash_switch_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static DEVICE_ATTR(fswitch, 0660, cam_flash_switch_show,cam_flash_switch_store);


static const struct of_device_id cam_i2c_flash_dt_match[] = {
	{.compatible = "qcom,i2c_flash", .data = NULL},
	{}
};

int oplus_iic_flash_ftm(struct cam_flash_ctrl *fctrl,struct i2c_client *client);
int reigster_flash_shutdown_notifier(void);
int unreigster_flash_shutdown_notifier(void);
#endif //OPLUS_CAM_FLASH_DEV_H
