#ifndef OPLUS_CAM_SENSOR_DEV_H
#define OPLUS_CAM_SENSOR_DEV_H

#include "cam_sensor_dev.h"


#define OV13B10_SENSOR_ID		0x0d42
#define OV16A10_SENSOR_ID		0x1641
#define OV8856_SENSOR_ID		0x885A
#define OV02B10_SENSOR_ID		0x002B
#define HI846_SENSOR_ID			0x4608
#define GC02M1B_SENSOR_ID		0x02e0
#define S5K4H7_SENSOR_ID		0x487B
#define IMX471_SENSOR_ID		0x0471
#define GC02K0_SENSOR_ID		0x2395
#define GC02K0_SENSOR_ID_2		0x2385
#define S5K3L6_SENSOR_ID		0x30c6
#define OV48B_SENSOR_ID			0x5648
#define S5KJN1_SENSOR_ID		0x38e1
#define IMX355_SENSOR_ID		0x0355
#define OV64B_SENSOR_ID			0x5664
#define IMX709_SENSOR_ID		0x709
#define GC02M1_SENSOR_ID		0xe000
#define SY_GC02M1B_SENSOR_ID		0xe0

#define VIDIOC_CAM_SENSOR_STATR 0x9000
#define VIDIOC_CAM_SENSOR_STOP 0x9001
#define S5KHM6_SENSOR_ID		0xa300
#define S5KHM6_2ST_SENSOR_ID		0xa100
#define OV08D10_SENSOR_ID		0x5647
#define HI556_SENSOR_ID			0x556

struct cam_sensor_i2c_reg_setting_array {
	struct cam_sensor_i2c_reg_array reg_setting[5931];
	unsigned short size;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	unsigned short delay;
};

struct cam_sensor_settings {
	struct cam_sensor_i2c_reg_setting_array ov13b10_setting;
	struct cam_sensor_i2c_reg_setting_array ov16a10_setting;
	struct cam_sensor_i2c_reg_setting_array ov8856_setting;
	struct cam_sensor_i2c_reg_setting_array ov02b10_setting;
	struct cam_sensor_i2c_reg_setting_array hi846_setting;
	struct cam_sensor_i2c_reg_setting_array gc02m1b_setting;
	struct cam_sensor_i2c_reg_setting_array s5k4h7_setting;
	struct cam_sensor_i2c_reg_setting_array imx471_setting;
	struct cam_sensor_i2c_reg_setting_array gc02k0_setting;
	struct cam_sensor_i2c_reg_setting_array s5k3l6_setting;
	struct cam_sensor_i2c_reg_setting_array ov48b_setting;
	struct cam_sensor_i2c_reg_setting_array s5kjn1_setting;
	struct cam_sensor_i2c_reg_setting_array imx355_setting;
	struct cam_sensor_i2c_reg_setting_array ov64b_setting;
	struct cam_sensor_i2c_reg_setting_array imx709_setting;
	struct cam_sensor_i2c_reg_setting_array gc02m1_setting;
	struct cam_sensor_i2c_reg_setting_array sy_gc02m1b_setting;
	struct cam_sensor_i2c_reg_setting_array s5khm6_setting;
	struct cam_sensor_i2c_reg_setting_array ov08d10_setting;
	struct cam_sensor_i2c_reg_setting_array hi556_setting;
};

struct cam_sensor_i2c_reg_init_array {
	struct cam_sensor_i2c_reg_array reg_setting[3200];
	unsigned short size;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	unsigned short delay;
};

struct cam_sensor_i2c_reg_init_array_messi_s5khm6 {
	struct cam_sensor_i2c_reg_array reg_setting[6873];
	unsigned short size;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	unsigned short delay;
};

struct cam_sensor_init_settings {
	struct cam_sensor_i2c_reg_init_array ov64b_setting;
	struct cam_sensor_i2c_reg_init_array imx709_setting;
	struct cam_sensor_i2c_reg_init_array_messi_s5khm6 s5khm6_setting;
};

int cam_sensor_stop(struct cam_sensor_ctrl_t *s_ctrl);
int cam_sensor_start(struct cam_sensor_ctrl_t *s_ctrl);

long oplus_cam_sensor_subdev_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg);
void oplus_init_cam_ldo_mutex(void);

#endif //OPLUS_CAM_SENSOR_DEV_H
