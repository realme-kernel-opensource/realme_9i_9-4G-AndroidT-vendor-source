#ifndef OPLUS_CAM_SENSOR_CORE_H
#define OPLUS_CAM_SENSOR_CORE_H

#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include "cam_sensor_core.h"
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "../cam_flash/oplus_cam_flash_dev.h"


extern struct cam_flash_settings flash_ftm_data;


#define FD_DFCT_NUM_ADDR 0x7678
#define SG_DFCT_NUM_ADDR 0x767A
#define FD_DFCT_ADDR 0x8B00
#define SG_DFCT_ADDR 0x8B10

#define V_ADDR_SHIFT 12
#define H_DATA_MASK 0xFFF80000
#define V_DATA_MASK 0x0007FF80

static struct sony_dfct_tbl_t imx471_dfct_tbl;
int sensor_imx471_get_dpc_data(struct cam_sensor_ctrl_t *s_ctrl);
int oplus_match_flash(struct cam_sensor_ctrl_t *s_ctrl);
int oplus_get_dpc_data(struct cam_sensor_ctrl_t *s_ctrl,void *arg);

#endif //OPLUS_CAM_SENSOR_CORE_H
