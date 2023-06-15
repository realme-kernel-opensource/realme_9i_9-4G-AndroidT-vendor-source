#include "cam_sensor_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_sensor_soc.h"
#include "cam_sensor_core.h"
#include "oplus_cam_sensor_dev.h"

struct cam_sensor_settings sensor_settings = {
#include "CAM_SENSOR_SETTINGS.h"
};

struct cam_sensor_init_settings sensor_init_settings = {
#include "cam_sensor_initsettings.h"
};

bool is_ftm_current_test = false;
struct mutex cam_ldo_mutex;

void oplus_init_cam_ldo_mutex(void)
{
	/* Initialize mutex */
	mutex_init(&cam_ldo_mutex);
}

int sensor_start_thread(void *arg) {
    struct cam_sensor_ctrl_t *s_ctrl = (struct cam_sensor_ctrl_t *)arg;
    int rc = 0;
    struct cam_sensor_i2c_reg_setting sensor_init_setting;
    int vendor_id = 0;

    if (!s_ctrl)
    {
        CAM_ERR(CAM_SENSOR, "s_ctrl is NULL");
        return -1;
    }
    mutex_lock(&(s_ctrl->cam_sensor_mutex));

    //power up for sensor
    mutex_lock(&(s_ctrl->sensor_power_state_mutex));
    if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_OFF)
    {
        rc = cam_sensor_power_up(s_ctrl);
        if(rc < 0) {
            CAM_ERR(CAM_SENSOR, "sensor power up faild!");
         } else {
            CAM_INFO(CAM_SENSOR, "sensor power up success sensor id 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
            s_ctrl->sensor_power_state = CAM_SENSOR_POWER_ON;
         }
    } else {
        CAM_INFO(CAM_SENSOR, "sensor have power up!");
    }
    mutex_unlock(&(s_ctrl->sensor_power_state_mutex));

    //write initsetting for sensor
    if (rc == 0) {
        mutex_lock(&(s_ctrl->sensor_initsetting_mutex));
        if (s_ctrl->sensor_initsetting_state == CAM_SENSOR_SETTING_WRITE_INVALID){
            if (s_ctrl->sensordata->slave_info.sensor_id == 0x5664)
            {
                if (true) {
                    sensor_init_setting.reg_setting = sensor_init_settings.ov64b_setting.reg_setting;
                    sensor_init_setting.addr_type   = CAMERA_SENSOR_I2C_TYPE_WORD;
                    sensor_init_setting.data_type   = CAMERA_SENSOR_I2C_TYPE_BYTE;
                    sensor_init_setting.size        = sensor_init_settings.ov64b_setting.size;
                    sensor_init_setting.delay       = sensor_init_settings.ov64b_setting.delay;
                }
                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_init_setting);
                if(rc < 0) {
                    CAM_ERR(CAM_SENSOR, "write vendor_id %d 0x%x addr_type 0x%x setting failed!",
                        vendor_id, s_ctrl->vendorid, s_ctrl->sensor_probe_addr_type);
                } else {
                    CAM_INFO(CAM_SENSOR, "write vendor_id %d 0x%x addr_type 0x%x setting success!",
                        vendor_id, s_ctrl->vendorid, s_ctrl->sensor_probe_addr_type);
                    s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_SUCCESS;
                }
            }
            else if (s_ctrl->sensordata->slave_info.sensor_id == 0x709)
            {
                if (true) {
                    sensor_init_setting.reg_setting = sensor_init_settings.imx709_setting.reg_setting;
                    sensor_init_setting.addr_type   = CAMERA_SENSOR_I2C_TYPE_WORD;
                    sensor_init_setting.data_type   = CAMERA_SENSOR_I2C_TYPE_BYTE;
                    sensor_init_setting.size        = sensor_init_settings.imx709_setting.size;
                    sensor_init_setting.delay       = sensor_init_settings.imx709_setting.delay;
                }
                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_init_setting);
                if(rc < 0) {
                    CAM_ERR(CAM_SENSOR, "write vendor_id %d 0x%x addr_type 0x%x setting failed!",
                        vendor_id, s_ctrl->vendorid, s_ctrl->sensor_probe_addr_type);
                } else {
                    CAM_INFO(CAM_SENSOR, "write vendor_id %d 0x%x addr_type 0x%x setting success!",
                        vendor_id, s_ctrl->vendorid, s_ctrl->sensor_probe_addr_type);
                    s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_SUCCESS;
                }
            }
            else if (s_ctrl->sensordata->slave_info.sensor_id == 0xA300)
            {
                sensor_init_setting.reg_setting = sensor_init_settings.s5khm6_setting.reg_setting;
                sensor_init_setting.addr_type   = CAMERA_SENSOR_I2C_TYPE_WORD;
                sensor_init_setting.data_type   = CAMERA_SENSOR_I2C_TYPE_WORD;
                sensor_init_setting.size        = sensor_init_settings.s5khm6_setting.size;
                sensor_init_setting.delay       = sensor_init_settings.s5khm6_setting.delay;

                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_init_setting);
                if(rc < 0) {
                    CAM_ERR(CAM_SENSOR, "write vendor_id %d 0x%x addr_type 0x%x setting failed!",
                        vendor_id, s_ctrl->vendorid, s_ctrl->sensor_probe_addr_type);
                } else {
                    CAM_INFO(CAM_SENSOR, "write vendor_id %d 0x%x addr_type 0x%x setting success!",
                        vendor_id, s_ctrl->vendorid, s_ctrl->sensor_probe_addr_type);
                    s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_SUCCESS;
                }
            }
        } else {
            CAM_INFO(CAM_SENSOR, "sensor setting have write!");
        }
        mutex_unlock(&(s_ctrl->sensor_initsetting_mutex));
    }

    mutex_unlock(&(s_ctrl->cam_sensor_mutex));
    return rc;

}

int cam_sensor_start(struct cam_sensor_ctrl_t *s_ctrl) {
    int rc = 0;

    if(s_ctrl == NULL)
    {
        CAM_ERR(CAM_SENSOR, "s_ctrl is null ");
        return -1;
    }

    mutex_lock(&(s_ctrl->cam_sensor_mutex));

    mutex_lock(&(s_ctrl->sensor_power_state_mutex));
    if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_OFF)
    {
        s_ctrl->sensor_open_thread = kthread_run(sensor_start_thread, s_ctrl, s_ctrl->device_name);
        if (!s_ctrl->sensor_open_thread) {
            CAM_ERR(CAM_SENSOR, "create sensor start thread failed");
            rc = -1;
        }
        else
        {
            CAM_INFO(CAM_SENSOR, "create sensor start thread success");
        }
    }
    else
    {
        CAM_INFO(CAM_SENSOR, "sensor have power up");
    }
    mutex_unlock(&(s_ctrl->sensor_power_state_mutex));

    mutex_unlock(&(s_ctrl->cam_sensor_mutex));
    return rc;
}

int cam_sensor_stop(struct cam_sensor_ctrl_t *s_ctrl) {
    int rc = 0;
    CAM_ERR(CAM_SENSOR,"sensor do stop");
    mutex_lock(&(s_ctrl->cam_sensor_mutex));

    //power off for sensor
    mutex_lock(&(s_ctrl->sensor_power_state_mutex));
    if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_ON)
    {
        rc = cam_sensor_power_down(s_ctrl);
        if(rc < 0) {
            CAM_ERR(CAM_SENSOR, "sensor power down faild!");
         } else {
            CAM_INFO(CAM_SENSOR, "sensor power down success sensor id 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
            s_ctrl->sensor_power_state = CAM_SENSOR_POWER_OFF;
            mutex_lock(&(s_ctrl->sensor_initsetting_mutex));
            s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
            mutex_unlock(&(s_ctrl->sensor_initsetting_mutex));
         }
    } else {
        CAM_INFO(CAM_SENSOR, "sensor have power down!");
    }
    mutex_unlock(&(s_ctrl->sensor_power_state_mutex));

    mutex_unlock(&(s_ctrl->cam_sensor_mutex));
    return rc;
}

long oplus_cam_sensor_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
    int rc = 0;
    struct cam_sensor_ctrl_t *s_ctrl =
        v4l2_get_subdevdata(sd);
    struct cam_sensor_i2c_reg_setting sensor_setting;
    struct cam_sensor_i2c_reg_setting_array *ptr = NULL;

    switch (cmd) {
        case VIDIOC_CAM_CONTROL:
            rc = cam_sensor_driver_cmd(s_ctrl, arg);
            break;
        case VIDIOC_CAM_SENSOR_STATR:
            rc = cam_sensor_start(s_ctrl);
            break;
        case VIDIOC_CAM_SENSOR_STOP:
            rc = cam_sensor_stop(s_ctrl);
            break;
        case VIDIOC_CAM_FTM_POWNER_DOWN:
            CAM_INFO(CAM_SENSOR, "FTM power down");
            return cam_sensor_power_down(s_ctrl);
            break;
        case VIDIOC_CAM_FTM_POWNER_UP:
            CAM_INFO(CAM_SENSOR, "FTM power up, sensor id 0x%x", s_ctrl->sensordata->slave_info.sensor_id);
            rc = cam_sensor_power_up(s_ctrl);
            if(rc < 0) {
                CAM_ERR(CAM_SENSOR, "ftm power up failed!");
                break;
            }
            is_ftm_current_test = true;

            CAM_INFO(CAM_SENSOR,"FTM GET sensor_id=0x%x setting", s_ctrl->sensordata->slave_info.sensor_id);
            switch(s_ctrl->sensordata->slave_info.sensor_id)
            {
                case S5KHM6_SENSOR_ID:
                case S5KHM6_2ST_SENSOR_ID:
                    ptr = &sensor_settings.s5khm6_setting;
                    break;
                case OV08D10_SENSOR_ID:
                    ptr = &sensor_settings.ov08d10_setting;
                    break;
                case OV13B10_SENSOR_ID:
                    ptr = &sensor_settings.ov13b10_setting;
                    break;
                case OV48B_SENSOR_ID:
                    ptr = &sensor_settings.ov48b_setting;
                    break;
                case OV8856_SENSOR_ID:
                    ptr = &sensor_settings.ov8856_setting;
                    break;
                case OV02B10_SENSOR_ID:
                    ptr = &sensor_settings.ov02b10_setting;
                    break;
                case HI846_SENSOR_ID:
                    ptr = &sensor_settings.hi846_setting;
                    break;
                case GC02M1B_SENSOR_ID:
                    ptr = &sensor_settings.gc02m1b_setting;
                    break;
                case S5K4H7_SENSOR_ID:
                    ptr = &sensor_settings.s5k4h7_setting;
                    break;
                case IMX471_SENSOR_ID:
                    ptr = &sensor_settings.imx471_setting;
                    break;
                case GC02K0_SENSOR_ID:
                case GC02K0_SENSOR_ID_2:
                    ptr = &sensor_settings.gc02k0_setting;
                    break;
                case S5K3L6_SENSOR_ID:
                    ptr = &sensor_settings.s5k3l6_setting;
                    break;
                case S5KJN1_SENSOR_ID:
                    ptr = &sensor_settings.s5kjn1_setting;
                    break;
                case IMX355_SENSOR_ID:
                    ptr = &sensor_settings.imx355_setting;
                    break;
                case OV64B_SENSOR_ID:
                    ptr = &sensor_settings.ov64b_setting;
                    break;
                case IMX709_SENSOR_ID:
                    ptr = &sensor_settings.imx709_setting;
                    break;
                case GC02M1_SENSOR_ID:
                    ptr = &sensor_settings.gc02m1_setting;
                    break;
                case SY_GC02M1B_SENSOR_ID:
                    ptr = &sensor_settings.sy_gc02m1b_setting;
                    break;
                case HI556_SENSOR_ID:
                    ptr = &sensor_settings.hi556_setting;
                    break;
                default:
                    break;
            }
            if (ptr != NULL){
                sensor_setting.reg_setting = ptr->reg_setting;
                sensor_setting.addr_type = ptr->addr_type;
                sensor_setting.data_type = ptr->data_type;
                sensor_setting.size = ptr->size;
                sensor_setting.delay = ptr->delay;
            }
            rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

            if (rc < 0) {
                CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor setting");
            } else {
                CAM_INFO(CAM_SENSOR, "FTM successfully to write sensor setting");
            }
            break;
    default:
        CAM_ERR(CAM_SENSOR, "Invalid ioctl cmd: %d", cmd);
        rc = -EINVAL;
        break;
    }
    return rc;
}

