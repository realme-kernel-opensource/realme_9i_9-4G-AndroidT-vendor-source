#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include "cam_sensor_core.h"
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "oplus_cam_sensor_core.h"
#include "../cam_flash/oplus_cam_flash_dev.h"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
extern struct cam_flash_settings flash_ftm_data;
#endif

int oplus_get_dpc_data(struct cam_sensor_ctrl_t *s_ctrl,void *arg)
{
    int rc = 0;
    struct cam_control *cmd = (struct cam_control *)arg;
	if (0x0471 != s_ctrl->sensordata->slave_info.sensor_id) {
		rc = -EFAULT;
		return rc;
	}
	CAM_INFO(CAM_SENSOR, "imx471_dfct_tbl: fd_dfct_num=%d, sg_dfct_num=%d",
		imx471_dfct_tbl.fd_dfct_num, imx471_dfct_tbl.sg_dfct_num);
	if (copy_to_user((void __user *) cmd->handle, &imx471_dfct_tbl,
		sizeof(struct  sony_dfct_tbl_t))) {
		CAM_ERR(CAM_SENSOR, "Failed Copy to User");
		rc = -EFAULT;
		return rc;
	}
	return rc;
}

int sensor_imx471_get_dpc_data(struct cam_sensor_ctrl_t *s_ctrl)
{
    int i = 0, j = 0;
    int rc = 0;
    int check_reg_val, dfct_data_h, dfct_data_l;
    int dfct_data = 0;
    int fd_dfct_num = 0, sg_dfct_num = 0;
    int retry_cnt = 5;
    int data_h = 0, data_v = 0;
    int fd_dfct_addr = FD_DFCT_ADDR;
    int sg_dfct_addr = SG_DFCT_ADDR;

    CAM_INFO(CAM_SENSOR, "sensor_imx471_get_dpc_data enter");
    if (s_ctrl == NULL) {
        CAM_ERR(CAM_SENSOR, "Invalid Args");
        return -EINVAL;
    }

    memset(&imx471_dfct_tbl, 0, sizeof(struct sony_dfct_tbl_t));

    for (i = 0; i < retry_cnt; i++) {
        check_reg_val = 0;
        rc = camera_io_dev_read(&(s_ctrl->io_master_info),
            FD_DFCT_NUM_ADDR, &check_reg_val,
            CAMERA_SENSOR_I2C_TYPE_WORD,
            CAMERA_SENSOR_I2C_TYPE_BYTE);

        if (0 == rc) {
            fd_dfct_num = check_reg_val & 0x07;
            if (fd_dfct_num > FD_DFCT_MAX_NUM)
                fd_dfct_num = FD_DFCT_MAX_NUM;
            break;
        }
    }

    for (i = 0; i < retry_cnt; i++) {
        check_reg_val = 0;
        rc = camera_io_dev_read(&(s_ctrl->io_master_info),
            SG_DFCT_NUM_ADDR, &check_reg_val,
            CAMERA_SENSOR_I2C_TYPE_WORD,
            CAMERA_SENSOR_I2C_TYPE_WORD);

        if (0 == rc) {
            sg_dfct_num = check_reg_val & 0x01FF;
            if (sg_dfct_num > SG_DFCT_MAX_NUM)
                sg_dfct_num = SG_DFCT_MAX_NUM;
            break;
        }
    }

    CAM_INFO(CAM_SENSOR, " fd_dfct_num = %d, sg_dfct_num = %d", fd_dfct_num, sg_dfct_num);
    imx471_dfct_tbl.fd_dfct_num = fd_dfct_num;
    imx471_dfct_tbl.sg_dfct_num = sg_dfct_num;

    if (fd_dfct_num > 0) {
        for (j = 0; j < fd_dfct_num; j++) {
            dfct_data = 0;
            for (i = 0; i < retry_cnt; i++) {
                dfct_data_h = 0;
                rc = camera_io_dev_read(&(s_ctrl->io_master_info),
                        fd_dfct_addr, &dfct_data_h,
                        CAMERA_SENSOR_I2C_TYPE_WORD,
                        CAMERA_SENSOR_I2C_TYPE_WORD);
                if (0 == rc) {
                    break;
                }
            }
            for (i = 0; i < retry_cnt; i++) {
                dfct_data_l = 0;
                rc = camera_io_dev_read(&(s_ctrl->io_master_info),
                        fd_dfct_addr+2, &dfct_data_l,
                        CAMERA_SENSOR_I2C_TYPE_WORD,
                        CAMERA_SENSOR_I2C_TYPE_WORD);
                if (0 == rc) {
                    break;
                }
            }
            CAM_DBG(CAM_SENSOR, " dfct_data_h = 0x%x, dfct_data_l = 0x%x", dfct_data_h, dfct_data_l);
            dfct_data = (dfct_data_h << 16) | dfct_data_l;
            data_h = 0;
            data_v = 0;
            data_h = (dfct_data & (H_DATA_MASK >> j%8)) >> (19 - j%8); //19 = 32 -13;
            data_v = (dfct_data & (V_DATA_MASK >> j%8)) >> (7 - j%8);  // 7 = 32 -13 -12;
            CAM_DBG(CAM_SENSOR, "j = %d, H = %d, V = %d", j, data_h, data_v);
            imx471_dfct_tbl.fd_dfct_addr[j] = ((data_h & 0x1FFF) << V_ADDR_SHIFT) | (data_v & 0x0FFF);
            CAM_DBG(CAM_SENSOR, "fd_dfct_data[%d] = 0x%08x", j, imx471_dfct_tbl.fd_dfct_addr[j]);
            fd_dfct_addr = fd_dfct_addr + 3 + ((j+1)%8 == 0);
        }
    }
    if (sg_dfct_num > 0) {
        for (j = 0; j < sg_dfct_num; j++) {
            dfct_data = 0;
            for (i = 0; i < retry_cnt; i++) {
                dfct_data_h = 0;
                rc = camera_io_dev_read(&(s_ctrl->io_master_info),
                        sg_dfct_addr, &dfct_data_h,
                        CAMERA_SENSOR_I2C_TYPE_WORD,
                        CAMERA_SENSOR_I2C_TYPE_WORD);
                if (0 == rc) {
                    break;
                }
            }
            for (i = 0; i < retry_cnt; i++) {
                dfct_data_l = 0;
                rc = camera_io_dev_read(&(s_ctrl->io_master_info),
                        sg_dfct_addr+2, &dfct_data_l,
                        CAMERA_SENSOR_I2C_TYPE_WORD,
                        CAMERA_SENSOR_I2C_TYPE_WORD);
                if (0 == rc) {
                    break;
                }
            }
            CAM_DBG(CAM_SENSOR, " dfct_data_h = 0x%x, dfct_data_l = 0x%x", dfct_data_h, dfct_data_l);
            dfct_data = (dfct_data_h << 16) | dfct_data_l;
            data_h = 0;
            data_v = 0;
            data_h = (dfct_data & (H_DATA_MASK >> j%8)) >> (19 - j%8); //19 = 32 -13;
            data_v = (dfct_data & (V_DATA_MASK >> j%8)) >> (7 - j%8);  // 7 = 32 -13 -12;
            CAM_DBG(CAM_SENSOR, "j = %d, H = %d, V = %d", j, data_h, data_v);
            imx471_dfct_tbl.sg_dfct_addr[j] = ((data_h & 0x1FFF) << V_ADDR_SHIFT) | (data_v & 0x0FFF);
            CAM_DBG(CAM_SENSOR, "sg_dfct_data[%d] = 0x%08x", j, imx471_dfct_tbl.sg_dfct_addr[j]);
            sg_dfct_addr = sg_dfct_addr + 3 + ((j+1)%8 == 0);
        }
    }

    CAM_INFO(CAM_SENSOR, "exit");
    return rc;
}
int oplus_match_flash(struct cam_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
	if (flash_ftm_data.flash_type == 1) {
		if ((flash_ftm_data.valid_setting_index < 0) && (flash_ftm_data.valid_setting_index >= flash_ftm_data.total_flash_dev)) {
			CAM_ERR(CAM_FLASH, "No match flash ftm setting!");
			return -ENODEV;
		}

		if (flash_ftm_data.flash_ftm_settings[flash_ftm_data.valid_setting_index].flashprobeinfo.flash_id != s_ctrl->sensordata->slave_info.flash_id) {
			CAM_INFO(CAM_FLASH, "Cur exist flash not match sensor!");
			return -ENODEV;
		}
	}
    return rc;
}

