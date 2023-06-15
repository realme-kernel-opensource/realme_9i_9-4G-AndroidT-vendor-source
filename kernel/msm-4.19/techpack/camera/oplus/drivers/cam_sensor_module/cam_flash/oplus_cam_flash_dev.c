#include "oplus_cam_flash_dev.h"

static struct notifier_block flash_shutdown;

struct cam_flash_settings flash_ftm_data = {
	.total_flash_dev = TOTAL_FLASH_NUM,
	.flash_type = 0,
	.valid_setting_index = -1,
	.cur_flash_status = IIC_FLASH_INVALID,
	{
		#include "CAM_FLASH_SETTINGS.h"
	}
};

struct cam_flash_ctrl *vendor_flash_ctrl = NULL;
struct cam_flash_ctrl *front_flash_ctrl = NULL;

int cam_ftm_i2c_flash_off(struct cam_flash_ctrl *flash_ctrl, struct cam_flash_ftm_settings *flash_ftm_data)
{
	int rc = 0, i = 0;
	struct cam_flash_ftm_power_setting *power_off_setting = &(flash_ftm_data->flashpowerdownsetting);
	struct cam_flash_ftm_reg_setting *flash_off_setting = &(flash_ftm_data->flashoffsettings);
	struct cam_flash_ftm_power_setting *power_on_setting = &(flash_ftm_data->flashpowerupsetting);
	struct cam_sensor_i2c_reg_setting write_setting;

	memset(&write_setting, 0, sizeof(write_setting));
	if (flash_ctrl->io_master_info.master_type == I2C_MASTER) {
		flash_ctrl->io_master_info.client->addr = flash_ftm_data->flashprobeinfo.slave_write_address;
	} else if(flash_ctrl->io_master_info.master_type == CCI_MASTER) {
		flash_ctrl->io_master_info.cci_client->cci_i2c_master = flash_ftm_data->cci_client.cci_i2c_master;
		flash_ctrl->io_master_info.cci_client->i2c_freq_mode = flash_ftm_data->cci_client.i2c_freq_mode;
		flash_ctrl->io_master_info.cci_client->sid = flash_ftm_data->cci_client.sid >> 1;
	}

	flash_ctrl->power_info.power_setting_size = 0;
	flash_ctrl->power_info.power_setting = kzalloc(sizeof(struct cam_sensor_power_setting) * MAX_POWER_CONFIG, GFP_KERNEL);
	if (!flash_ctrl->power_info.power_setting)
		return -ENOMEM;
	flash_ctrl->power_info.power_down_setting_size = 0;
	flash_ctrl->power_info.power_down_setting = kzalloc(sizeof(struct cam_sensor_power_setting) * MAX_POWER_CONFIG, GFP_KERNEL);
	if (!flash_ctrl->power_info.power_down_setting)
		return -ENOMEM;

	if (!power_off_setting || !flash_off_setting || !power_on_setting) {
		CAM_ERR(CAM_FLASH,"Ftm flash off failed, Empty setting");
		goto free_power_settings;
	}

	//1.flash off setting
	write_setting.reg_setting = flash_off_setting->reg_setting;
	write_setting.addr_type = flash_off_setting->addr_type;
	write_setting.data_type = flash_off_setting->data_type;
	write_setting.size = flash_off_setting->size;
	write_setting.delay = flash_off_setting->delay;
	rc = camera_io_dev_write(&(flash_ctrl->io_master_info), &write_setting);
	if (rc < 0) {
		CAM_ERR(CAM_FLASH, "FTM Failed to write flash off setting rc = %d", rc);
		goto free_power_settings;
	}

	//2.power off
	for (i = 0; i < power_off_setting->size; i++) {
		flash_ctrl->power_info.power_down_setting[i].config_val = power_off_setting->single_power[i].config_val;
		flash_ctrl->power_info.power_down_setting[i].delay = power_off_setting->single_power[i].delay;
		flash_ctrl->power_info.power_down_setting[i].seq_type = power_off_setting->single_power[i].seq_type;
		flash_ctrl->power_info.power_down_setting_size = power_off_setting->size;
	}
	for (i = 0; i < power_on_setting->size; i++) {
		flash_ctrl->power_info.power_setting[i].config_val = power_on_setting->single_power[i].config_val;
		flash_ctrl->power_info.power_setting[i].delay = power_on_setting->single_power[i].delay;
		flash_ctrl->power_info.power_setting[i].seq_type = power_on_setting->single_power[i].seq_type;
		flash_ctrl->power_info.power_setting_size = power_on_setting->size;
	}
	rc = flash_ctrl->func_tbl.power_ops(flash_ctrl, false);
	if (rc) {
		CAM_ERR(CAM_FLASH,"FTM flash powerdown Failed rc = %d", rc);
		goto free_power_settings;
	}

	CAM_INFO(CAM_FLASH, "FTM success to poweroff i2c flash");

free_power_settings:
	kfree(flash_ctrl->power_info.power_down_setting);
	kfree(flash_ctrl->power_info.power_setting);
	flash_ctrl->power_info.power_down_setting = NULL;
	flash_ctrl->power_info.power_setting = NULL;
	return rc;
}

int cam_ftm_i2c_torch_on(struct cam_flash_ctrl *flash_ctrl, struct cam_flash_ftm_settings *flash_ftm_data)
{
	int rc = 0, i = 0;
	struct cam_flash_ftm_power_setting *power_on_setting = &(flash_ftm_data->flashpowerupsetting);
	struct cam_flash_ftm_reg_setting *torch_on_setting = &(flash_ftm_data->flashlowsettings);
	struct cam_flash_ftm_reg_setting *init_setting = &(flash_ftm_data->flashinitsettings);
	struct cam_sensor_i2c_reg_setting write_setting;

	memset(&write_setting, 0, sizeof(write_setting));
	if (flash_ctrl->io_master_info.master_type == I2C_MASTER) {
		flash_ctrl->io_master_info.client->addr = flash_ftm_data->flashprobeinfo.slave_write_address;
	} else if (flash_ctrl->io_master_info.master_type == CCI_MASTER) {
		flash_ctrl->io_master_info.cci_client->cci_i2c_master = flash_ftm_data->cci_client.cci_i2c_master;
		flash_ctrl->io_master_info.cci_client->i2c_freq_mode = flash_ftm_data->cci_client.i2c_freq_mode;
		flash_ctrl->io_master_info.cci_client->sid = flash_ftm_data->cci_client.sid >> 1;
	}

	flash_ctrl->power_info.power_setting_size = 0;
	flash_ctrl->power_info.power_setting = kzalloc(sizeof(struct cam_sensor_power_setting) * MAX_POWER_CONFIG, GFP_KERNEL);
	if (!flash_ctrl->power_info.power_setting)
		return -ENOMEM;

	if (!power_on_setting || !init_setting || !torch_on_setting) {
		CAM_ERR(CAM_FLASH,"Ftm torch on failed, Empty setting");
		goto free_power_settings;
	}

	//1.power on
	for (i = 0; i < power_on_setting->size; i++) {
		flash_ctrl->power_info.power_setting[i].config_val = power_on_setting->single_power[i].config_val;
		flash_ctrl->power_info.power_setting[i].delay = power_on_setting->single_power[i].delay;
		flash_ctrl->power_info.power_setting[i].seq_type = power_on_setting->single_power[i].seq_type;
		flash_ctrl->power_info.power_setting_size = power_on_setting->size;
	}
	rc = flash_ctrl->func_tbl.power_ops(flash_ctrl, true);
	if (rc) {
		CAM_ERR(CAM_FLASH,"FTM flash powerup Failed rc = %d", rc);
		goto free_power_settings;
	}

	//2.init setting
	write_setting.reg_setting = init_setting->reg_setting;
	write_setting.addr_type = init_setting->addr_type;
	write_setting.data_type = init_setting->data_type;
	write_setting.size = init_setting->size;
	write_setting.delay = init_setting->delay;
	rc = camera_io_dev_write(&(flash_ctrl->io_master_info), &write_setting);
	if (rc < 0) {
		CAM_ERR(CAM_FLASH, "FTM write init setting Failed rc = %d", rc);
		goto free_power_settings;
	}

	//3.torch on setting
	write_setting.reg_setting = torch_on_setting->reg_setting;
	write_setting.addr_type = torch_on_setting->addr_type;
	write_setting.data_type = torch_on_setting->data_type;
	write_setting.size = torch_on_setting->size;
	write_setting.delay = torch_on_setting->delay;
	rc = camera_io_dev_write(&(flash_ctrl->io_master_info), &write_setting);
	if (rc < 0) {
		CAM_ERR(CAM_FLASH, "FTM write torch on setting Failed rc = %d", rc);
		goto free_power_settings;
	}

	CAM_INFO(CAM_FLASH, "FTM success set torch on");

free_power_settings:
	kfree(flash_ctrl->power_info.power_setting);
	flash_ctrl->power_info.power_setting = NULL;
	return rc;
}

int cam_ftm_i2c_flash_on(struct cam_flash_ctrl *flash_ctrl, struct cam_flash_ftm_settings *flash_ftm_data)
{
	int rc = 0, i = 0;
	struct cam_flash_ftm_power_setting *power_on_setting = &(flash_ftm_data->flashpowerupsetting);
	struct cam_flash_ftm_reg_setting *flash_on_setting = &(flash_ftm_data->flashhighsettings);
	struct cam_flash_ftm_reg_setting *init_setting = &(flash_ftm_data->flashinitsettings);
	struct cam_sensor_i2c_reg_setting write_setting;

	memset(&write_setting, 0, sizeof(write_setting));
	if (flash_ctrl->io_master_info.master_type == I2C_MASTER) {
		flash_ctrl->io_master_info.client->addr = flash_ftm_data->flashprobeinfo.slave_write_address;
	} else if(flash_ctrl->io_master_info.master_type == CCI_MASTER) {
		flash_ctrl->io_master_info.cci_client->cci_i2c_master = flash_ftm_data->cci_client.cci_i2c_master;
		flash_ctrl->io_master_info.cci_client->i2c_freq_mode = flash_ftm_data->cci_client.i2c_freq_mode;
		flash_ctrl->io_master_info.cci_client->sid = flash_ftm_data->cci_client.sid >> 1;
	}

	flash_ctrl->power_info.power_setting_size = 0;
	flash_ctrl->power_info.power_setting = kzalloc(sizeof(struct cam_sensor_power_setting) * MAX_POWER_CONFIG, GFP_KERNEL);
	if (!flash_ctrl->power_info.power_setting)
		return -ENOMEM;

	if (!power_on_setting || !init_setting || !flash_on_setting) {
		CAM_ERR(CAM_FLASH,"Ftm flash on failed, Empty setting");
		goto free_power_settings;
	}

	//1.power on
	for (i = 0; i < power_on_setting->size; i++) {
		flash_ctrl->power_info.power_setting[i].config_val = power_on_setting->single_power[i].config_val;
		flash_ctrl->power_info.power_setting[i].delay = power_on_setting->single_power[i].delay;
		flash_ctrl->power_info.power_setting[i].seq_type = power_on_setting->single_power[i].seq_type;
		flash_ctrl->power_info.power_setting_size = power_on_setting->size;
	}
	rc = flash_ctrl->func_tbl.power_ops(flash_ctrl, true);
	if (rc) {
		CAM_ERR(CAM_FLASH,"ftm flash powerup Failed rc = %d", rc);
		goto free_power_settings;
	}

	//2.init setting
	write_setting.reg_setting = init_setting->reg_setting;
	write_setting.addr_type = init_setting->addr_type;
	write_setting.data_type = init_setting->data_type;
	write_setting.size = init_setting->size;
	write_setting.delay = init_setting->delay;
	rc = camera_io_dev_write(&(flash_ctrl->io_master_info), &write_setting);
	if (rc < 0) {
		CAM_ERR(CAM_FLASH, "FTM write init setting Failed rc = %d", rc);
		goto free_power_settings;
	}

	//3.flash on setting
	write_setting.reg_setting = flash_on_setting->reg_setting;
	write_setting.addr_type = flash_on_setting->addr_type;
	write_setting.data_type = flash_on_setting->data_type;
	write_setting.size = flash_on_setting->size;
	write_setting.delay = flash_on_setting->delay;
	rc = camera_io_dev_write(&(flash_ctrl->io_master_info), &write_setting);
	if (rc < 0) {
		CAM_ERR(CAM_FLASH, "FTM write flash on setting Failed rc = %d", rc);
		goto free_power_settings;
	}

	CAM_INFO(CAM_FLASH, "FTM success to set flash on");

free_power_settings:
	kfree(flash_ctrl->power_info.power_setting);
	flash_ctrl->power_info.power_setting = NULL;
	return rc;
}

int cam_flash_match_id(uint32_t except_id, struct cam_flash_ctrl *flash_ctrl, struct cam_flash_ftm_settings *flash_ftm_data)
{
	int rc = 0, match_flag = 0, i = 0;
	uint32_t readid = 0;
	struct cam_flash_ftm_power_setting *power_on_setting = &(flash_ftm_data->flashpowerupsetting);
	struct cam_flash_ftm_power_setting *power_off_setting = &(flash_ftm_data->flashpowerdownsetting);

	if (flash_ctrl->io_master_info.master_type == I2C_MASTER) {
		flash_ctrl->io_master_info.client->addr = flash_ftm_data->flashprobeinfo.slave_write_address;
	} else if(flash_ctrl->io_master_info.master_type == CCI_MASTER) {
		flash_ctrl->io_master_info.cci_client->cci_i2c_master = flash_ftm_data->cci_client.cci_i2c_master;
		flash_ctrl->io_master_info.cci_client->i2c_freq_mode = flash_ftm_data->cci_client.i2c_freq_mode;
		flash_ctrl->io_master_info.cci_client->sid = flash_ftm_data->cci_client.sid >> 1;
	}

	flash_ctrl->power_info.power_setting_size = 0;
	flash_ctrl->power_info.power_setting = kzalloc(sizeof(struct cam_sensor_power_setting) * MAX_POWER_CONFIG, GFP_KERNEL);
	if (!flash_ctrl->power_info.power_setting)
		return -ENOMEM;
	flash_ctrl->power_info.power_down_setting_size = 0;
	flash_ctrl->power_info.power_down_setting = kzalloc(sizeof(struct cam_sensor_power_setting) * MAX_POWER_CONFIG, GFP_KERNEL);
	if (!flash_ctrl->power_info.power_down_setting)
		return -ENOMEM;

	if (!power_on_setting || !power_off_setting) {
		CAM_ERR(CAM_FLASH,"Ftm flash on failed, Empty setting");
		goto free_power_settings;
	}

	//1.power on
	for (i = 0; i < power_on_setting->size; i++) {
		flash_ctrl->power_info.power_setting[i].config_val = power_on_setting->single_power[i].config_val;
		flash_ctrl->power_info.power_setting[i].delay = power_on_setting->single_power[i].delay;
		flash_ctrl->power_info.power_setting[i].seq_type = power_on_setting->single_power[i].seq_type;
		flash_ctrl->power_info.power_setting_size = power_on_setting->size;
	}
	rc = flash_ctrl->func_tbl.power_ops(flash_ctrl, true);
	if (rc) {
		CAM_ERR(CAM_FLASH,"ftm flash powerup Failed rc = %d", rc);
		goto free_power_settings;
	}

	//2.match id
	rc = camera_io_dev_read(&(flash_ctrl->io_master_info),flash_ftm_data->flashprobeinfo.flash_id_address,
							&readid, flash_ftm_data->flashprobeinfo.addr_type, flash_ftm_data->flashprobeinfo.data_type);

	if (readid != except_id) {
		match_flag = 0;
	} else {
		match_flag = 1;
		CAM_INFO(CAM_FLASH, "read id: 0x%x, expected id: 0x%x", readid, except_id);
	}

	//3.power off
	for (i = 0; i < power_off_setting->size; i++) {
		flash_ctrl->power_info.power_down_setting[i].config_val = power_off_setting->single_power[i].config_val;
		flash_ctrl->power_info.power_down_setting[i].delay = power_off_setting->single_power[i].delay;
		flash_ctrl->power_info.power_down_setting[i].seq_type = power_off_setting->single_power[i].seq_type;
		flash_ctrl->power_info.power_down_setting_size = power_off_setting->size;
	}
	rc = flash_ctrl->func_tbl.power_ops(flash_ctrl, false);
	if (rc) {
		CAM_ERR(CAM_FLASH,"FTM flash powerdown Failed rc = %d", rc);
		goto free_power_settings;
	}

free_power_settings:
	kfree(flash_ctrl->power_info.power_setting);
	kfree(flash_ctrl->power_info.power_down_setting);
	flash_ctrl->power_info.power_setting = NULL;
	flash_ctrl->power_info.power_down_setting = NULL;
	return match_flag;
}

int cam_ftm_i2c_set_standby(struct cam_flash_ctrl *flash_ctrl, struct cam_flash_ftm_settings *flash_ftm_data)
{
	int rc = 0;
	struct cam_flash_ftm_reg_setting *flash_off_setting = &(flash_ftm_data->flashoffsettings);
	struct cam_sensor_i2c_reg_setting write_setting;

	memset(&write_setting, 0, sizeof(write_setting));
	if (flash_ctrl->io_master_info.master_type == I2C_MASTER) {
		flash_ctrl->io_master_info.client->addr = flash_ftm_data->flashprobeinfo.slave_write_address;
	} else if(flash_ctrl->io_master_info.master_type == CCI_MASTER) {
		flash_ctrl->io_master_info.cci_client->cci_i2c_master = flash_ftm_data->cci_client.cci_i2c_master;
		flash_ctrl->io_master_info.cci_client->i2c_freq_mode = flash_ftm_data->cci_client.i2c_freq_mode;
		flash_ctrl->io_master_info.cci_client->sid = flash_ftm_data->cci_client.sid >> 1;
	}

	if (!flash_off_setting) {
		CAM_ERR(CAM_FLASH,"Ftm set standby mode failed, Empty setting");
		return rc;
	}

	//1.flash off setting
	write_setting.reg_setting = flash_off_setting->reg_setting;
	write_setting.addr_type = flash_off_setting->addr_type;
	write_setting.data_type = flash_off_setting->data_type;
	write_setting.size = flash_off_setting->size;
	write_setting.delay = flash_off_setting->delay;
	rc = camera_io_dev_write(&(flash_ctrl->io_master_info), &write_setting);
	if (rc < 0) {
		CAM_ERR(CAM_FLASH, "FTM Failed to write flash off setting rc = %d", rc);
		return rc;
	}

	CAM_INFO(CAM_FLASH, "FTM success to set standby mode");
	return rc;
}

ssize_t ftm_i2c_flash_on_off(struct cam_flash_ctrl *flash_ctrl)
{
	int rc = 1;
	struct timespec ts;
	struct rtc_time tm;

	if (flash_ftm_data.valid_setting_index < 0 && flash_ftm_data.valid_setting_index >= flash_ftm_data.total_flash_dev) {
		CAM_ERR(CAM_FLASH, "No match flash ftm setting!");
		return -1;
	}

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("flash_mode %d,%d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		flash_mode,
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	if(pre_flash_mode == flash_mode)
		return rc;

	if(pre_flash_mode == 5 && flash_mode == 0) {
		CAM_ERR(CAM_FLASH, "camera is opened,not to set flashlight off");
		return rc;
	}

	pre_flash_mode = flash_mode;
	switch (flash_mode)
	{
		CAM_INFO(CAM_FLASH, "FTM set cur flash state:%d", flash_mode);
	    // ftm & status_bar flash off
		case 0:
			cam_ftm_i2c_flash_off(flash_ctrl, &(flash_ftm_data.flash_ftm_settings[flash_ftm_data.valid_setting_index]));
			flash_ctrl->flash_state = CAM_FLASH_STATE_INIT;
			flash_ftm_data.cur_flash_status = IIC_FLASH_OFF;
			break;
		// status_bar torch on
		case 1:
			cam_ftm_i2c_torch_on(flash_ctrl, &(flash_ftm_data.flash_ftm_settings[flash_ftm_data.valid_setting_index]));
			flash_ftm_data.cur_flash_status = IIC_FLASH_ON;
			break;
		// flash on
		case 2:
			cam_ftm_i2c_flash_on(flash_ctrl, &(flash_ftm_data.flash_ftm_settings[flash_ftm_data.valid_setting_index]));
			flash_ftm_data.cur_flash_status = IIC_FLASH_HIGH;
			break;
		// ftm torch on
		case 3:
			cam_ftm_i2c_torch_on(flash_ctrl, &(flash_ftm_data.flash_ftm_settings[flash_ftm_data.valid_setting_index]));
			flash_ftm_data.cur_flash_status = IIC_FLASH_ON;
			break;
		default:
			break;
	}
	return rc;
}

ssize_t flash_on_off(struct cam_flash_ctrl *flash_ctrl)
{
	int rc = 1;
	struct timespec ts;
	struct rtc_time tm;
	struct cam_flash_frame_setting flash_data;
	memset(&flash_data, 0, sizeof(flash_data));

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("flash_mode %d,%d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		flash_mode,
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	if(pre_flash_mode == flash_mode)
		return rc;


	if(pre_flash_mode == 5 && flash_mode == 0){
		CAM_ERR(CAM_FLASH, "camera is opened,not to set flashlight off");
		return rc;
	}

	pre_flash_mode = flash_mode;
	switch (flash_mode)
	{
		case 0:
			flash_data.led_current_ma[0] = 0;
			flash_data.led_current_ma[1] = 0;
			cam_flash_off(flash_ctrl);
			flash_ctrl->flash_state = CAM_FLASH_STATE_INIT;
			break;
		case 1:
			flash_data.led_current_ma[0] = 100;
			flash_data.led_current_ma[1] = 100;
			cam_flash_on(flash_ctrl, &flash_data, 0);
			break;
		case 2:
			flash_data.led_current_ma[0] = 1000;
			flash_data.led_current_ma[1] = 1000;
			cam_flash_on(flash_ctrl, &flash_data, 1);
			break;
		case 3:
			flash_data.led_current_ma[0] = 60;
			flash_data.led_current_ma[1] = 60;
			cam_flash_on(flash_ctrl, &flash_data, 0);
			break;
		default:
			break;
	}
	return rc;
}

ssize_t flash_proc_write(struct file *filp, const char __user *buff,
						size_t len, loff_t *data)
{
	char buf[8] = {0};
	int rc = 0;
	if (len > 8)
		len = 8;
	if (copy_from_user(buf, buff, len)) {
		CAM_ERR(CAM_FLASH, "proc write error.\n");
		return -EFAULT;
	}
	mutex_lock(&(vendor_flash_ctrl->ftm_flash_lock));
	flash_mode = simple_strtoul(buf, NULL, 10);

	if (vendor_flash_ctrl->io_master_info.master_type == I2C_MASTER || vendor_flash_ctrl->io_master_info.master_type == CCI_MASTER) {
		rc = ftm_i2c_flash_on_off(vendor_flash_ctrl);
	}
	else {
		rc = flash_on_off(vendor_flash_ctrl);
	}
	if(rc < 0)
		CAM_ERR(CAM_FLASH, "%s flash write failed %d\n", __func__, __LINE__);
	mutex_unlock(&(vendor_flash_ctrl->ftm_flash_lock));
	return len;
}
ssize_t flash_proc_read(struct file *filp, char __user *buff,
						size_t len, loff_t *data)
{
	char value[2] = {0};
	snprintf(value, sizeof(value), "%d", flash_mode);
	return simple_read_from_buffer(buff, len, data, value,1);
}

int flash_proc_init(struct cam_flash_ctrl *flash_ctrl)
{
	int ret = 0;
	char proc_flash[16] = "qcom_flash";
	char strtmp[] = "0";
	struct proc_dir_entry *proc_entry;
	CAM_INFO(CAM_FLASH, "flash_name", flash_ctrl->flash_name);
	// @Todo [CAM] adjust for pmic and current_ma
	// if (flash_ctl->flash_name == NULL) {
	// 	CAM_ERR(CAM_FLASH, "%s get flash name is NULL %d\n", __func__, __LINE__);
	// 	return -1;
	// } else {
	// 	if ((strcmp(flash_ctl->flash_name, "pmic_19125") != 0)
	// 			&&(strcmp(flash_ctl->flash_name, "pmic_19015") != 0)
	// 			&&(strcmp(flash_ctl->flash_name, "pmic_19191") != 0)
	// 			&&(strcmp(flash_ctl->flash_name, "pmic") != 0)) {
	// 		CAM_ERR(CAM_FLASH, "%s get flash name is PMIC ,so return\n", __func__);
	// 		return -1;
	// 	}
	// }
	if (flash_ctrl->soc_info.index > 0) {
		sprintf(strtmp, "%d", flash_ctrl->soc_info.index);
		strcat(proc_flash, strtmp);
	}
	proc_entry = proc_create_data(proc_flash, 0666, NULL,&led_fops, NULL);
	if (proc_entry == NULL) {
		ret = -ENOMEM;
		CAM_ERR(CAM_FLASH, "[%s]: Error! Couldn't create qcom_flash proc entry\n", __func__);
	}
	vendor_flash_ctrl = flash_ctrl;
	mutex_init(&(vendor_flash_ctrl->ftm_flash_lock));

	return ret;
}

ssize_t cam_flash_switch_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int rc = 0;
	struct cam_flash_ctrl *data = dev_get_drvdata(dev);

	int enable = 0;

	if (kstrtoint(buf, 0, &enable)) {
		CAM_ERR(CAM_FLASH, "get val error.\n");
		rc = -EINVAL;
	}
	CAM_ERR(CAM_FLASH, "echo data = %d ", enable);

	mutex_lock(&(vendor_flash_ctrl->ftm_flash_lock));
	flash_mode = enable;
	if (vendor_flash_ctrl->io_master_info.master_type == I2C_MASTER || vendor_flash_ctrl->io_master_info.master_type == CCI_MASTER) {
		rc = ftm_i2c_flash_on_off(data);
	}
	else {
		rc = flash_on_off(data);
	}
	mutex_unlock(&(vendor_flash_ctrl->ftm_flash_lock));

	return count;
}

ssize_t cam_flash_switch_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 5, "%d\n", flash_mode);
	//return simple_read_from_buffer(buff, 10, buf, value,1);
}

//static DEVICE_ATTR(fswitch, 0660, cam_flash_switch_show,cam_flash_switch_store);

// register flashoff operation to reboot_notifier_list
int shutdown_flash_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	int rc = NOTIFY_DONE;
	struct cam_flash_ftm_settings *cur_flash_ftm_data = NULL;
	struct cam_flash_ftm_reg_setting *flash_off_setting = NULL;
	struct cam_sensor_i2c_reg_setting write_setting;

	if (flash_ftm_data.valid_setting_index >= TOTAL_FLASH_NUM || flash_ftm_data.valid_setting_index < 0) {
		CAM_ERR(CAM_FLASH,"Empty flash off setting!");
		return NOTIFY_BAD;
	}

	cur_flash_ftm_data = &(flash_ftm_data.flash_ftm_settings[flash_ftm_data.valid_setting_index]);
	flash_off_setting = &(cur_flash_ftm_data->flashoffsettings);
	memset(&write_setting, 0, sizeof(write_setting));

	if (!flash_off_setting || !cur_flash_ftm_data || !vendor_flash_ctrl) {
		CAM_ERR(CAM_FLASH,"Empty flash off setting!");
		return NOTIFY_BAD;
	}

	if (vendor_flash_ctrl->io_master_info.master_type == I2C_MASTER) {
		if (!vendor_flash_ctrl->io_master_info.client)
			return NOTIFY_BAD;

		vendor_flash_ctrl->io_master_info.client->addr = cur_flash_ftm_data->flashprobeinfo.slave_write_address;
	} else if (vendor_flash_ctrl->io_master_info.master_type == CCI_MASTER) {
		if (!vendor_flash_ctrl->io_master_info.cci_client)
			return NOTIFY_BAD;

		vendor_flash_ctrl->io_master_info.cci_client->cci_i2c_master = cur_flash_ftm_data->cci_client.cci_i2c_master;
		vendor_flash_ctrl->io_master_info.cci_client->i2c_freq_mode = cur_flash_ftm_data->cci_client.i2c_freq_mode;
		vendor_flash_ctrl->io_master_info.cci_client->sid = cur_flash_ftm_data->cci_client.sid >> 1;
	}

	//1.flash off setting
	write_setting.reg_setting = flash_off_setting->reg_setting;
	write_setting.addr_type = flash_off_setting->addr_type;
	write_setting.data_type = flash_off_setting->data_type;
	write_setting.size = flash_off_setting->size;
	write_setting.delay = flash_off_setting->delay;

	if((cur_flash_ftm_data->flashprobeinfo.flash_id == 0x02) && (flash_ftm_data.cur_flash_status == IIC_FLASH_ON)) {
		rc = camera_io_dev_write(&(vendor_flash_ctrl->io_master_info), &write_setting);
		if (rc < 0) {
			CAM_ERR(CAM_FLASH, "Set flash off in shutdown failed!");
			rc = NOTIFY_BAD;
		} else {
			CAM_INFO(CAM_FLASH, "Set flash off in shutdown successful, cur action:%d", action);
			rc = NOTIFY_OK;
		}
	}

	return rc;
}

int reigster_flash_shutdown_notifier()
{

	int rc = 0;
	flash_shutdown.notifier_call = shutdown_flash_notify;
	flash_shutdown.priority = 0;

	if (!register_reboot_notifier(&flash_shutdown)) {
	CAM_ERR(CAM_FLASH, "Register shutdown flash operation failed!");
		rc = -EINVAL;
	}
	return rc;
}

int unreigster_flash_shutdown_notifier()
{
	int rc = 0;
	flash_shutdown.notifier_call = shutdown_flash_notify;
	flash_shutdown.priority = 0;

	if (!unregister_reboot_notifier(&flash_shutdown)) {
		CAM_ERR(CAM_FLASH, "Unregister shutdown flash operation failed!");
		rc = -EINVAL;
	}
	return rc;
}

int oplus_iic_flash_ftm(struct cam_flash_ctrl *fctrl,struct i2c_client *client)
{
    int rc =0,retry_count = 0,i = 0;
    if (flash_proc_init(fctrl) < 0) {
        device_create_file(&client->dev, &dev_attr_fswitch);
    }

    //set cur flash type
    if (vendor_flash_ctrl->io_master_info.master_type == I2C_MASTER)
        flash_ftm_data.flash_type = 1;

    //set exist i2c flash device setting status to true;
    for (retry_count = 0; retry_count < 3; retry_count++) {
        for (i = 0; i < flash_ftm_data.total_flash_dev; i++) {
            if(cam_flash_match_id(flash_ftm_data.flash_ftm_settings[i].flashprobeinfo.flash_id,
                                fctrl,
                                &(flash_ftm_data.flash_ftm_settings[i]))) {
                flash_ftm_data.valid_setting_index = i;
                CAM_INFO(CAM_FLASH, "find exist flash name:%s, retry_count:%d",
                        flash_ftm_data.flash_ftm_settings[flash_ftm_data.valid_setting_index].flashprobeinfo.flash_name, retry_count);
                return rc;
            }
        }
    }
    return rc;
}

