/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_panel_power.c
** Description : oplus display panel power control
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#include "oplus_display_panel_power.h"
#if defined(OPLUS_FEATURE_PXLW_IRIS5)
#include <video/mipi_display.h>
#include "dsi_iris5_api.h"
#include "dsi_iris5_lightup.h"
#include "dsi_iris5_loop_back.h"
#endif

#define PWR_SEQ_NAME_LEN	(100)

PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX] = {{0}, {0}, {2, 0, 1, 2, ""}};
u32 panel_pwr_vg_base = 0;
int oplus_request_power_status = OPLUS_DISPLAY_POWER_OFF;
DEFINE_MUTEX(oplus_power_status_lock);

int g_tp_gesture_enable_flag = 0;
bool g_oplus_display_debug_switch = true;
PANEL_POWER_OPERATION panel_power_ctrl_elect[] = {
	{"set_vci", POWER_SET_VCI},
	{"set_vddio", POWER_SET_VDDIO},
	{"set_reset", POWER_SET_RESET},
	{"set_bias", POWER_SET_BIAS},
	{"set_regulator", POWER_SET_REGULATOR},
	{"set_pinctrl_state", POWER_SET_PINCTRL_STATE},
	{"tp_load_fw", POWER_TP_LOAD_FW},
	{"set_cmd", POWER_SET_CMD},
	{"set_reset_sel", POWER_SET_RESET_SEL},
	{"set_end", POWER_SET_END},
};

extern bool g_shutdown_flag;
extern int dsi_panel_reset(struct dsi_panel *panel);
extern int dsi_pwr_enable_regulator(struct dsi_regulator_info *regs, bool enable);
extern int dsi_panel_set_pinctrl_state(struct dsi_panel *panel, bool enable);
extern int lcd_set_bias(bool enable);
extern int turn_on_ktz8866_hw_en(bool on);

void __attribute__((weak)) lcd_queue_load_tp_fw(void) {return;}
int __attribute__((weak)) tp_gesture_enable_flag(void) { return 0;}

inline void oplus_mdelay(unsigned long msec)
{
	if (msec)
		usleep_range(msec * 1000, msec * 1000 + 100);
}

inline void oplus_udelay(unsigned long usec)
{
	if (usec)
		usleep_range(usec, usec + 10);
}

static int oplus_panel_find_vreg_by_name(const char *name)
{
	int count = 0, i = 0;
	struct dsi_vreg *vreg = NULL;
	struct dsi_regulator_info *dsi_reg = NULL;
	struct dsi_display *display = get_main_display();

	if (!display) {
		return -ENODEV;
	}

	if (!display->panel) {
		return -EINVAL;
	}

	dsi_reg = &display->panel->power_info;
        count = dsi_reg->count;
	for (i = 0; i < count; i++) {
		vreg = &dsi_reg->vregs[i];
		pr_err("%s : find  %s", __func__, vreg->vreg_name);
		if (!strcmp(vreg->vreg_name, name)) {
			pr_err("%s : find the vreg %s", __func__, name);
			return i;
		} else {
			continue;
		}
	}
	pr_err("%s : dose not find the vreg [%s]", __func__, name);

	return -EINVAL;
}


int dsi_panel_parse_panel_power_cfg(struct dsi_panel *panel)
{
	int rc = 0, i = 0;
	const char *name_vddi = NULL;
	const char *name_vddr = NULL;
	u32 *panel_vol = NULL;
	struct dsi_parser_utils *utils = &panel->utils;

	pr_err("[%s] \n", __func__);

	if (!strcmp(panel->type, "primary")) {
		panel_vol = &panel_vol_bak[PANEL_VOLTAGE_ID_VDDI].voltage_id;
		rc = utils->read_u32_array(utils->data, "qcom,panel_voltage_vddi",
					panel_vol, PANEL_VOLTAGE_VALUE_COUNT);
		if (rc) {
			pr_err("[%s] failed to parse panel_voltage vddi\n", panel->name);
			goto error;
		}

		rc = utils->read_string(utils->data, "qcom,panel_voltage_vddi_name",
					&name_vddi);
		if (rc) {
			pr_err("[%s] failed to parse vddi name\n", panel->name);
			goto error;
		} else {
			pr_err("[%s] surccess to parse vddi name %s\n", panel->name, name_vddi);
			strcpy(panel_vol_bak[PANEL_VOLTAGE_ID_VDDI].pwr_name, name_vddi);
		}

		panel_vol = &panel_vol_bak[PANEL_VOLTAGE_ID_VDDR].voltage_id;
		rc = utils->read_u32_array(utils->data, "qcom,panel_voltage_vddr",
					panel_vol, PANEL_VOLTAGE_VALUE_COUNT);
		if (rc) {
			pr_err("[%s] failed to parse panel_voltage vddr\n", panel->name);
			goto error;
		}

		rc = utils->read_string(utils->data, "qcom,panel_voltage_vddr_name",
					&name_vddr);
		if (rc) {
			pr_err("[%s] failed to parse vddr name\n", panel->name);
			goto error;
		} else {
			pr_err("[%s] surccess to parse vddr name %s\n", panel->name, name_vddr);
			strcpy(panel_vol_bak[PANEL_VOLTAGE_ID_VDDR].pwr_name, name_vddr);
		}

		/*add for debug*/
		for(i; i < PANEL_VOLTAGE_ID_MAX; i++) {
			pr_err("[%s] panel_voltage[%d] = %d,%d,%d,%d,%s\n", __func__, i, panel_vol_bak[i].voltage_id,
				panel_vol_bak[i].voltage_min, panel_vol_bak[i].voltage_current,
				panel_vol_bak[i].voltage_max, panel_vol_bak[i].pwr_name);
		}
	}

	error:
		return rc;
}

static u32 oplus_panel_update_current_voltage(u32 id)
{
	int vol_current = 0, pwr_id = 0;
	struct dsi_vreg *dsi_reg = NULL;
	struct dsi_regulator_info *dsi_reg_info = NULL;
	struct dsi_display *display = get_main_display();

	if (!display) {
		return -ENODEV;
	}
	if (!display->panel || !display->drm_conn) {
		return -EINVAL;
	}

	dsi_reg_info = &display->panel->power_info;
	pwr_id = oplus_panel_find_vreg_by_name(panel_vol_bak[id].pwr_name);
	if (pwr_id < 0) {
		pr_err("%s: can't find the pwr_id, please check the vreg name\n", __func__);
		return pwr_id;
	}

	dsi_reg = &dsi_reg_info->vregs[pwr_id];

	vol_current = regulator_get_voltage(dsi_reg->vreg);

	return vol_current;
}

int oplus_display_panel_get_pwr(void *data)
{
	int ret = 0;
	struct panel_vol_get *panel_vol = data;
	u32 vol_id = (panel_vol->panel_id - 1);
	pr_err("%s : [id] = %d\n", __func__, vol_id);

	if (vol_id < 0) {
		pr_err("%s error id: [id] = %d\n", __func__, vol_id);
		return -EINVAL;
	}

	panel_vol->panel_min = panel_vol_bak[vol_id].voltage_min;
	panel_vol->panel_max = panel_vol_bak[vol_id].voltage_max;
	panel_vol->panel_cur = panel_vol_bak[vol_id].voltage_current;

	if (vol_id < PANEL_VOLTAGE_ID_VG_BASE &&
		vol_id >= PANEL_VOLTAGE_ID_VDDI) {
		ret = oplus_panel_update_current_voltage(vol_id);
		if (ret < 0) {
			pr_err("%s : update_current_voltage error = %d\n", __func__, ret);
			return ret;
		} else {
			panel_vol->panel_cur = ret;
			pr_err("%s : [id min cur max] = [%u32, %u32, %u32, %u32]\n", __func__,
				vol_id, panel_vol->panel_min,
				panel_vol->panel_cur, panel_vol->panel_max);
			return 0;
		}
	}

	return ret;
}

int oplus_display_panel_set_pwr(void *data)
{
	struct panel_vol_set *panel_vol = data;
	int panel_vol_value = 0, rc = 0, panel_vol_id = 0, pwr_id = 0;
	struct dsi_vreg *dsi_reg = NULL;
	struct dsi_regulator_info *dsi_reg_info = NULL;
	struct dsi_display *display = get_main_display();

	panel_vol_id = ((panel_vol->panel_id & 0x0F)-1);
	panel_vol_value = panel_vol->panel_vol;

	pr_err("debug for %s, id = %d value = %d\n",
		__func__, panel_vol_id, panel_vol_value);

	if (panel_vol_id < 0 || panel_vol_id > PANEL_VOLTAGE_ID_MAX) {
		return -EINVAL;
	}

	if (panel_vol_value < panel_vol_bak[panel_vol_id].voltage_min ||
		panel_vol_id > panel_vol_bak[panel_vol_id].voltage_max)
		return -EINVAL;

	if (!display) {
		return -ENODEV;
	}
	if (!display->panel || !display->drm_conn) {
		return -EINVAL;
	}

	if (panel_vol_id == PANEL_VOLTAGE_ID_VG_BASE) {
		pr_err("%s: set the VGH_L pwr = %d \n", __func__, panel_vol_value);
		panel_pwr_vg_base = panel_vol_value;
		return rc;
	}

	dsi_reg_info = &display->panel->power_info;

	pwr_id = oplus_panel_find_vreg_by_name(panel_vol_bak[panel_vol_id].pwr_name);
	if (pwr_id < 0) {
		pr_err("%s: can't find the vreg name, please re-check vreg name: %s \n",
			__func__, panel_vol_bak[panel_vol_id].pwr_name);
		return pwr_id;
	}
	dsi_reg = &dsi_reg_info->vregs[pwr_id];

	rc = regulator_set_voltage(dsi_reg->vreg, panel_vol_value, panel_vol_value);
	if (rc) {
		pr_err("Set voltage(%s) fail, rc=%d\n",
			 dsi_reg->vreg_name, rc);
		return -EINVAL;
	}

	return rc;
}

int __oplus_display_set_power_status(int status) {
	mutex_lock(&oplus_power_status_lock);
	if(status != oplus_request_power_status) {
		oplus_request_power_status = status;
	}
	mutex_unlock(&oplus_power_status_lock);
	return 0;
}

int oplus_display_panel_get_power_status(void *data) {
	uint32_t *power_status = data;

	printk(KERN_INFO "oplus_display_get_power_status = %d\n", get_oplus_display_power_status());
	(*power_status) = get_oplus_display_power_status();

	return 0;
}

int oplus_display_panel_set_power_status(void *data) {
	uint32_t *temp_save = data;

	printk(KERN_INFO "%s oplus_display_set_power_status = %d\n", __func__, (*temp_save));
	__oplus_display_set_power_status((*temp_save));

	return 0;
}

int oplus_display_panel_regulator_control(void *data) {
	uint32_t *temp_save_user = data;
	uint32_t temp_save = (*temp_save_user);
	struct dsi_display *temp_display;

	printk(KERN_INFO "%s oplus_display_regulator_control = %d\n", __func__, temp_save);
	if(get_main_display() == NULL) {
		printk(KERN_INFO "oplus_display_regulator_control and main display is null");
		return -1;
	}
	temp_display = get_main_display();
	if(temp_save == 0) {
#if defined(OPLUS_FEATURE_PXLW_IRIS5)
		if (iris_get_feature())
			iris5_control_pwr_regulator(false);
#endif
		dsi_pwr_enable_regulator(&temp_display->panel->power_info, false);
	} else if (temp_save == 1) {
#if defined(OPLUS_FEATURE_PXLW_IRIS5)
		if (iris_get_feature())
			iris5_control_pwr_regulator(true);
#endif
		dsi_pwr_enable_regulator(&temp_display->panel->power_info, true);
	}

	return 0;
}

static int oplus_get_panel_pwr_seq_count(struct device_node *panel_node,
	char *power_seq_name)
{
	return of_property_count_strings(panel_node,
		power_seq_name);
}

static void oplus_get_panel_pwr_oper_name(struct device_node *panel_node,
	char *power_seq_name, int index, const char **operation_name)
{
	of_property_read_string_index(panel_node,
		power_seq_name, index, operation_name);
}

static u8 oplus_get_panel_pwr_oper_num(const char *power_operation_name)
{
	int i = 0;

	if (power_operation_name == NULL)
		pr_err("[%s]power_operation_name is NULL\n", __func__);

	for (i = 0; i < ARRAY_SIZE(panel_power_ctrl_elect); i++) {
		if (!strcmp(power_operation_name, panel_power_ctrl_elect[i].operation_name))
			return panel_power_ctrl_elect[i].operation_num;
	}

	pr_err("[%s]panel power not support <%s> operation\n",
		__func__, power_operation_name);
	return POWER_SET_END;
}

static void oplus_get_panel_power_set_seq(struct dsi_panel *panel,
	char *power_seq_name, u8 *power_set_seq,
	u8 *set_seq_count, int power_set_seq_size)
{
	const char *operation_name;
	int pwr_seq_count = 0;
	int index = 0;

	if (!panel || !panel->panel_of_node) {
		pr_err("[%s]panel or panel_of_node is NULL\n", __func__);
		return;
	}

	if (!power_seq_name || !power_set_seq) {
		pr_err("[%s]power_seq_name or power_set_seq is NULL\n", __func__);
		return;
	}

	pwr_seq_count = oplus_get_panel_pwr_seq_count(panel->panel_of_node, power_seq_name);
	if (power_set_seq_size < pwr_seq_count) {
		pr_err("[%s]<%s> config size=%d is overflow, support max_size=%d\n",
			panel->name, power_seq_name, pwr_seq_count, power_set_seq_size);
		pwr_seq_count = power_set_seq_size;
	}

	if (pwr_seq_count > 0) {
		pr_info("[%s]<%s> config size=%d\n", panel->name, power_seq_name, pwr_seq_count);
		for (index = 0; index < pwr_seq_count; index++) {
			oplus_get_panel_pwr_oper_name(panel->panel_of_node, power_seq_name,
				index, &operation_name);
			power_set_seq[index] = oplus_get_panel_pwr_oper_num(operation_name);
			pr_info("%s[%d]:%s,operation_num=%d\n", power_seq_name, index,
				operation_name, power_set_seq[index]);

			if(power_set_seq[index] == POWER_SET_END)
				panel->oplus_panel_power.config_set_end_flag = true;
		}

		*set_seq_count = pwr_seq_count;
	} else {
		power_set_seq[0] = POWER_SET_END;
		pr_info("[%s]<%s> not support\n", __func__, power_seq_name);
	}
}

static void oplus_get_panel_power_oper_delay(struct dsi_parser_utils *utils,
	POWER_OPER_DELAY *power_operation_delay, char *delay_name, int size)
{
	u32 power_oper_delay[POWER_OPER_DELAY_SIZE] = {0};
	int rc = 0;

	rc = utils->read_u32_array(utils->data, delay_name,
			power_oper_delay, size);
	if (rc) {
		pr_err("[%s] cannot read %s\n", __func__, delay_name);
		return;
	} else {
		memcpy(power_operation_delay, power_oper_delay, POWER_OPER_DELAY_SIZE * sizeof(u32));
		if (g_oplus_display_debug_switch) {
			pr_info("[%s]%s=<%d %d %d %d>\n", __func__, delay_name,
				power_operation_delay->on_behind_mdelay,
				power_operation_delay->off_behind_mdelay,
				power_operation_delay->gesture_on_behind_mdelay,
				power_operation_delay->gesture_off_behind_mdelay);
		}
		return;
	}
}

void oplus_get_panel_power_timing_parse(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = NULL;

	if (!panel || !panel->panel_of_node) {
		pr_err("[%s]panel or panel_of_node is NULL\n", __func__);
		return;
	}

	utils = &panel->utils;
	if (!utils) {
		pr_err("[%s] utils is NULL\n", __func__);
		return;
	}

	memset(&panel->oplus_panel_power, 0, sizeof(struct oplus_panel_power_parse));

	panel->oplus_panel_power.oplus_panel_power_ctl = utils->read_bool(utils->data,
		"oplus,oplus-panel-power-ctl");
	pr_info("[%s]oplus,oplus-panel-power-ctl: %s", panel->name,
		panel->oplus_panel_power.oplus_panel_power_ctl ? "true" : "false");
	if (panel->oplus_panel_power.oplus_panel_power_ctl) {
		panel->oplus_panel_power.panel_vci_enable_flag = false;
		panel->oplus_panel_power.panel_vddio_enable_flag = false;
		panel->oplus_panel_power.panel_bias_enable_flag = false;
		panel->oplus_panel_power.panel_reset_enable_flag = false;
		panel->oplus_panel_power.panel_regulator_enable_flag = false;
		panel->oplus_panel_power.panel_pinctrl_state_enable_flag = false;
		panel->oplus_panel_power.tp_load_fw_succ_flag = false;
		panel->oplus_panel_power.config_set_end_flag = false;

		oplus_get_panel_power_set_seq(panel, "oplus,panel-on-power-sequence",
			panel->oplus_panel_power.panel_on_power_seq,
			&panel->oplus_panel_power.panel_on_power_seq_num,
			ARRAY_SIZE(panel->oplus_panel_power.panel_on_power_seq));
		if (panel->oplus_panel_power.config_set_end_flag) {
			panel->oplus_panel_power.config_set_end_flag = false;
		} else {
			pr_err("[%s]not config POWER_SET_END in panel-on-power-sequence\n", panel->name);
		}

		oplus_get_panel_power_set_seq(panel, "oplus,panel-off-power-sequence",
			panel->oplus_panel_power.panel_off_power_seq,
			&panel->oplus_panel_power.panel_off_power_seq_num,
			ARRAY_SIZE(panel->oplus_panel_power.panel_off_power_seq));
		if (panel->oplus_panel_power.config_set_end_flag) {
			panel->oplus_panel_power.config_set_end_flag = false;
		} else {
			pr_err("[%s]not config POWER_SET_END in panel-off-power-sequence\n", panel->name);
		}

		/* For enable TP gesture func */
		oplus_get_panel_power_set_seq(panel, "oplus,panel-gesture-on-power-sequence",
			panel->oplus_panel_power.panel_gesture_on_power_seq,
			&panel->oplus_panel_power.panel_gesture_on_power_seq_num,
			ARRAY_SIZE(panel->oplus_panel_power.panel_gesture_on_power_seq));
		if (panel->oplus_panel_power.config_set_end_flag) {
			panel->oplus_panel_power.config_set_end_flag = false;
		} else {
			pr_err("[%s]not config POWER_SET_END in panel-gesture-on-power-sequence\n", panel->name);
		}

		oplus_get_panel_power_set_seq(panel, "oplus,panel-gesture-off-power-set-sequence0",
			panel->oplus_panel_power.panel_gesture_off_power_seq,
			&panel->oplus_panel_power.panel_gesture_off_power_seq_num,
			ARRAY_SIZE(panel->oplus_panel_power.panel_gesture_off_power_seq));
		if (panel->oplus_panel_power.config_set_end_flag) {
			panel->oplus_panel_power.config_set_end_flag = false;
		} else {
			pr_err("[%s]not config POWER_SET_END in panel-gesture-off-power-sequence\n", panel->name);
		}

		panel->oplus_panel_power.panel_shutdown_power_set_flag = utils->read_bool(utils->data,
			"oplus,panel-shutdown-power-set-flag");
		pr_info("[%s]oplus,panel-shutdown-power-set-flag: %s", panel->name,
			panel->oplus_panel_power.panel_shutdown_power_set_flag ? "true" : "false");
		if (panel->oplus_panel_power.panel_shutdown_power_set_flag) {
			oplus_get_panel_power_set_seq(panel, "oplus,panel-shutdown-power-sequence",
				panel->oplus_panel_power.panel_shutdown_power_seq,
				&panel->oplus_panel_power.panel_shutdown_power_seq_num,
				ARRAY_SIZE(panel->oplus_panel_power.panel_shutdown_power_seq));
			if (panel->oplus_panel_power.config_set_end_flag) {
				panel->oplus_panel_power.config_set_end_flag = false;
			} else {
				pr_err("[%s]not config POWER_SET_END in panel-shutdown-power-sequence\n", panel->name);
			}

			oplus_get_panel_power_set_seq(panel, "oplus,panel-gesture-shutdown-power-sequence",
				panel->oplus_panel_power.panel_gesture_shutdown_power_seq,
				&panel->oplus_panel_power.panel_gesture_shutdown_power_seq_num,
				ARRAY_SIZE(panel->oplus_panel_power.panel_gesture_shutdown_power_seq));
			if (panel->oplus_panel_power.config_set_end_flag) {
				panel->oplus_panel_power.config_set_end_flag = false;
			} else {
				pr_err("[%s]not config POWER_SET_END in panel-gesture-shutdown-power-sequence\n", panel->name);
			}
		}

		if (POWER_OPER_DELAY_SIZE != (sizeof(POWER_OPER_DELAY) / sizeof(u32)))
			pr_err("[%s]POWER_OPER_DELAY_SIZE=%d, is error define\n", __func__, POWER_OPER_DELAY_SIZE);
		oplus_get_panel_power_oper_delay(utils, &panel->oplus_panel_power.vci_mdelay,
			"oplus,panel-power-vci-mdelay", POWER_OPER_DELAY_SIZE);
		oplus_get_panel_power_oper_delay(utils, &panel->oplus_panel_power.vddio_mdelay,
			"oplus,panel-power-vddio-mdelay", POWER_OPER_DELAY_SIZE);
		oplus_get_panel_power_oper_delay(utils, &panel->oplus_panel_power.bias_mdelay,
			"oplus,panel-power-mipi-mdelay", POWER_OPER_DELAY_SIZE);
		oplus_get_panel_power_oper_delay(utils, &panel->oplus_panel_power.reset_mdelay,
			"oplus,panel-power-reset-mdelay", POWER_OPER_DELAY_SIZE);
		oplus_get_panel_power_oper_delay(utils, &panel->oplus_panel_power.regulator_mdelay,
			"oplus,panel-power-regulator-mdelay", POWER_OPER_DELAY_SIZE);
		oplus_get_panel_power_oper_delay(utils, &panel->oplus_panel_power.pinctrl_mdelay,
			"oplus,panel-power-pinctrl-mdelay", POWER_OPER_DELAY_SIZE);

		utils->read_u32(utils->data, "oplus,panel-on-lp11-not-enable-mdelay",
			&panel->oplus_panel_power.panel_on_lp11_not_enable_mdelay);

		utils->read_u32(utils->data, "oplus,tp-load-fw-behind_mdelay",
			&panel->oplus_panel_power.panel_on_tp_load_fw_behind_mdelay);

		utils->read_u32(utils->data, "oplus,gesture-tp-load-fw-behind_mdelay",
			&panel->oplus_panel_power.panel_gesture_on_tp_load_fw_behind_mdelay);

		utils->read_u32(utils->data, "oplus,panel-on-mdelay-bl",
			&panel->oplus_panel_power.panel_on_mdelay_bl);
		if (g_oplus_display_debug_switch) {
			pr_info("[%s]panel power vci_mdelay=<%d %d %d %d>\n", panel->name,
				panel->oplus_panel_power.vci_mdelay.on_behind_mdelay,
				panel->oplus_panel_power.vci_mdelay.off_behind_mdelay,
				panel->oplus_panel_power.vci_mdelay.gesture_on_behind_mdelay,
				panel->oplus_panel_power.vci_mdelay.gesture_off_behind_mdelay);
			pr_info("[%s]panel power vddio_mdelay=<%d %d %d %d>\n", panel->name,
				panel->oplus_panel_power.vddio_mdelay.on_behind_mdelay,
				panel->oplus_panel_power.vddio_mdelay.off_behind_mdelay,
				panel->oplus_panel_power.vddio_mdelay.gesture_on_behind_mdelay,
				panel->oplus_panel_power.vddio_mdelay.gesture_off_behind_mdelay);
			pr_info("[%s]panel power bias_mdelay=<%d %d %d %d>\n", panel->name,
				panel->oplus_panel_power.bias_mdelay.on_behind_mdelay,
				panel->oplus_panel_power.bias_mdelay.off_behind_mdelay,
				panel->oplus_panel_power.bias_mdelay.gesture_on_behind_mdelay,
				panel->oplus_panel_power.bias_mdelay.gesture_off_behind_mdelay);
			pr_info("[%s]panel power reset_mdelay=<%d %d %d %d>\n", panel->name,
				panel->oplus_panel_power.reset_mdelay.on_behind_mdelay,
				panel->oplus_panel_power.reset_mdelay.off_behind_mdelay,
				panel->oplus_panel_power.reset_mdelay.gesture_on_behind_mdelay,
				panel->oplus_panel_power.reset_mdelay.gesture_off_behind_mdelay);
			pr_info("[%s]panel power pinctrl_mdelay=<%d %d %d %d>\n", panel->name,
				panel->oplus_panel_power.pinctrl_mdelay.on_behind_mdelay,
				panel->oplus_panel_power.pinctrl_mdelay.off_behind_mdelay,
				panel->oplus_panel_power.pinctrl_mdelay.gesture_on_behind_mdelay,
				panel->oplus_panel_power.pinctrl_mdelay.gesture_off_behind_mdelay);
			pr_info("[%s]panel power regulator_delay=<%d %d %d %d>\n", panel->name,
				panel->oplus_panel_power.regulator_mdelay.on_behind_mdelay,
				panel->oplus_panel_power.regulator_mdelay.off_behind_mdelay,
				panel->oplus_panel_power.regulator_mdelay.gesture_on_behind_mdelay,
				panel->oplus_panel_power.regulator_mdelay.gesture_off_behind_mdelay);
			pr_info("[%s]panel_on_lp11_not_enable_mdelay=%d\n", panel->name,
				panel->oplus_panel_power.panel_on_lp11_not_enable_mdelay);
			pr_info("[%s]panel_on_tp_load_fw_behind_mdelay=%d\n", panel->name,
				panel->oplus_panel_power.panel_on_tp_load_fw_behind_mdelay);
			pr_info("[%s]panel_gesture_on_tp_load_fw_behind_mdelay=%d\n", panel->name,
				panel->oplus_panel_power.panel_gesture_on_tp_load_fw_behind_mdelay);
			pr_info("[%s]panel_on_mdelay_bl=%d\n", panel->name,
				panel->oplus_panel_power.panel_on_mdelay_bl);
		}
	}
}

/* For panel regulator control */
static int oplus_panel_regulator_set_sub(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel)
		return -EINVAL;

	if (enable) {
		rc = dsi_pwr_enable_regulator(&panel->power_info, true);
		if (rc) {
			pr_err("[%s] %s failed to enable vregs, rc=%d\n", __func__, panel->name, rc);
			rc = -EINVAL;
		}
	} else {
		rc = dsi_pwr_enable_regulator(&panel->power_info, false);
		if (rc)
			pr_err("[%s] %s failed to disable vregs, rc=%d\n", __func__, panel->name, rc);
	}

	return rc;
}

static int oplus_panel_regulator_set(struct dsi_panel *panel, SET_POWER_MODE mode)
{
	int rc = 0;

	if (!panel)
		return -EINVAL;

	if (mode == SET_PWR_ON) {
		rc = oplus_panel_regulator_set_sub(panel, true);
		if (rc) {
			pr_err("[%s]config panel regulator on fail\n", __func__);
			panel->oplus_panel_power.panel_regulator_enable_flag = false;
			rc = -EINVAL;
			goto exit;
		}
		panel->oplus_panel_power.panel_regulator_enable_flag = true;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.regulator_mdelay.on_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.regulator_mdelay.gesture_on_behind_mdelay);
	} else {
		rc = oplus_panel_regulator_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel regulator off fail\n", __func__);
			panel->oplus_panel_power.panel_regulator_enable_flag = true;
			return -EINVAL;
		}
		panel->oplus_panel_power.panel_regulator_enable_flag = false;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.regulator_mdelay.off_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.regulator_mdelay.gesture_off_behind_mdelay);
	}

	return rc;
exit:
	oplus_panel_regulator_set_sub(panel, false);
	return rc;
}

/* For panel pinctrl state control */
static int oplus_panel_pinctrl_set_sub(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel)
		return -EINVAL;

	if (enable) {
		rc = dsi_panel_set_pinctrl_state(panel, true);
		if (rc) {
			pr_err("[%s] %s failed to set pinctrl state on, rc=%d\n",
				__func__, panel->name, rc);
			rc = -EINVAL;
		}
	} else {
		rc = dsi_panel_set_pinctrl_state(panel, false);
		if (rc) {
			pr_err("[%s] %s failed to set pinctrl state off, rc=%d\n",
				__func__, panel->name, rc);
		}
	}

	return rc;
}

static int oplus_panel_pinctrl_set(struct dsi_panel *panel, SET_POWER_MODE mode)
{
	int rc = 0;

	if (!panel)
		return -EINVAL;

	if (mode == SET_PWR_ON) {
		rc = oplus_panel_pinctrl_set_sub(panel, true);
		if (rc) {
			pr_err("[%s]config panel pinctrl state on fail\n", __func__);
			panel->oplus_panel_power.panel_pinctrl_state_enable_flag = false;
			rc = -EINVAL;
			goto exit;
		}
		panel->oplus_panel_power.panel_pinctrl_state_enable_flag = true;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.pinctrl_mdelay.on_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.pinctrl_mdelay.gesture_on_behind_mdelay);
	} else {
		rc = oplus_panel_pinctrl_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel pinctrl state off fail\n", __func__);
			panel->oplus_panel_power.panel_pinctrl_state_enable_flag = true;
			return -EINVAL;
		}
		panel->oplus_panel_power.panel_pinctrl_state_enable_flag = false;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.pinctrl_mdelay.off_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.pinctrl_mdelay.gesture_off_behind_mdelay);
	}

	return rc;
exit:
	oplus_panel_pinctrl_set_sub(panel, false);
	return rc;
}

/* For tp load fw control, Take Care: oplus_tp_load_fw not cause panel on fail */
static void oplus_tp_load_fw_sub(bool enable)
{
	if (enable) {
		lcd_queue_load_tp_fw();
	} else {
		/*  to be realized */
	}
}

static int oplus_tp_load_fw(struct dsi_panel *panel, SET_POWER_MODE mode)
{
	if (!panel)
		return -EINVAL;

	if(!panel->incell_screen)
		return 0;

	if (mode == SET_PWR_ON) {
		oplus_tp_load_fw_sub(true);
		panel->oplus_panel_power.tp_load_fw_succ_flag = true;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.panel_on_tp_load_fw_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.panel_gesture_on_tp_load_fw_behind_mdelay);
	} else {
		oplus_tp_load_fw_sub(false);
		panel->oplus_panel_power.tp_load_fw_succ_flag = false;
	}

	return 0;
}

/* For vci control */
static int oplus_panel_vci_set_sub(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel)
		return -EINVAL;

	if (enable) {
		if (gpio_is_valid(panel->panel_vci_gpio)) {
			rc = gpio_direction_output(panel->panel_vci_gpio, 1);
			if (rc)
				pr_err("[%s]unable to set dir for panel_vci_gpio rc=%d", __func__, rc);
			gpio_set_value(panel->panel_vci_gpio, 1);
			return 0;
		}
	} else {
		if (gpio_is_valid(panel->panel_vci_gpio))
			gpio_set_value(panel->panel_vci_gpio, 0);
	}

	return rc;
}

static int oplus_panel_vci_set(struct dsi_panel *panel, SET_POWER_MODE mode)
{
	int rc = 0;

	if (!panel)
		return -EINVAL;

	if (mode == SET_PWR_ON) {
		rc = oplus_panel_vci_set_sub(panel, true);
		if (rc) {
			pr_err("[%s]config panel vci on fail\n", __func__);
			panel->oplus_panel_power.panel_vci_enable_flag = false;
			rc = -EINVAL;
			goto exit;
		}
		panel->oplus_panel_power.panel_vci_enable_flag = true;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.vci_mdelay.on_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.vci_mdelay.gesture_on_behind_mdelay);
	} else {
		rc = oplus_panel_vci_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel vci off fail\n", __func__);
			panel->oplus_panel_power.panel_vci_enable_flag = true;
			return -EINVAL;
		}
		panel->oplus_panel_power.panel_vci_enable_flag = false;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.vci_mdelay.off_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.vci_mdelay.gesture_off_behind_mdelay);
	}

	return rc;
exit:
	oplus_panel_vci_set_sub(panel, false);
	return rc;
}

/* For vddio or vddr control */
static int oplus_panel_vddio_set_sub(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel)
		return -EINVAL;

	if (enable) {
		if (gpio_is_valid(panel->panel_vddr_gpio)) {
			rc = gpio_direction_output(panel->panel_vddr_gpio, 1);
			if (rc)
				pr_err("[%s]unable to set dir for panel_vddr_gpio rc=%d", __func__, rc);
			gpio_set_value(panel->panel_vddr_gpio, 1);
			return 0;
		}
	} else {
		if (gpio_is_valid(panel->panel_vddr_gpio))
			gpio_set_value(panel->panel_vddr_gpio, 0);
	}

	return rc;
}

static int oplus_panel_vddio_set(struct dsi_panel *panel, SET_POWER_MODE mode)
{
	int rc = 0;

	if (!panel)
		return -EINVAL;

	if (mode == SET_PWR_ON) {
		rc = oplus_panel_vddio_set_sub(panel, true);
		if (rc) {
			pr_err("[%s]config panel vddio on fail\n", __func__);
			panel->oplus_panel_power.panel_vddio_enable_flag = false;
			rc = -EINVAL;
			goto exit;
		}
		panel->oplus_panel_power.panel_vddio_enable_flag = true;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.vddio_mdelay.on_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.vddio_mdelay.gesture_on_behind_mdelay);
	} else {
		rc = oplus_panel_vddio_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel vddio off fail\n", __func__);
			panel->oplus_panel_power.panel_vddio_enable_flag = true;
			return -EINVAL;
		}
		panel->oplus_panel_power.panel_vddio_enable_flag = false;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.vddio_mdelay.off_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.vddio_mdelay.gesture_off_behind_mdelay);
	}

	return rc;
exit:
	oplus_panel_vddio_set_sub(panel, false);
	return rc;
}

/* For bias correlation control */
static int oplus_panel_bias_set_sub(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel)
		return -EINVAL;

	if (enable) {
		if(panel->power_ktz8866_enable) {
			if (g_oplus_display_debug_switch)
				pr_err("enable ktz8866 hw to supply +-5V\n");
			rc = turn_on_ktz8866_hw_en(true);
			if (rc) {
				pr_err("[%s] failed to turn_on_ktz8866_hw_en, rc=%d\n",
					panel->name, rc);
			}
			rc = lcd_set_bias(true);
			if (rc) {
				pr_err("[%s] failed to lcd_set_bias, rc=%d\n",
					panel->name, rc);
			}
		}
	} else {
		if(panel->power_ktz8866_enable) {
			if (g_oplus_display_debug_switch)
				pr_err("disable ktz8866 hw to disable +-5V\n");
			rc = lcd_set_bias(false);
			if (rc) {
				pr_err("[%s] failed to lcd_set_bias, rc=%d\n",
					panel->name, rc);
			}
			rc = turn_on_ktz8866_hw_en(false);
			if (rc) {
				pr_err("[%s] failed to turn_on_ktz8866_hw_en, rc=%d\n",
					panel->name, rc);
			}
		}
	}

	return rc;
}

static int oplus_panel_bias_set(struct dsi_panel *panel, SET_POWER_MODE mode)
{
	int rc = 0;

	if (!panel) {
		pr_err("[%s]panel is NULL!\n", __func__);
		return -EINVAL;
	}

	if (mode == SET_PWR_ON) {
		rc = oplus_panel_bias_set_sub(panel, true);
		if (rc) {
			pr_err("[%s]config panel bias on fail\n", __func__);
			panel->oplus_panel_power.panel_bias_enable_flag = false;
			rc = -EINVAL;
			goto exit;
		}
		panel->oplus_panel_power.panel_bias_enable_flag = true;

		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.bias_mdelay.on_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.bias_mdelay.gesture_on_behind_mdelay);
	} else {
		rc = oplus_panel_bias_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel bias off fail\n", __func__);
			panel->oplus_panel_power.panel_bias_enable_flag = true;
			return -EINVAL;
		}
		panel->oplus_panel_power.panel_bias_enable_flag = false;

		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.bias_mdelay.off_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.bias_mdelay.gesture_off_behind_mdelay);
	}

	return rc;
exit:
	oplus_panel_bias_set_sub(panel, false);
	return rc;
}

static int oplus_panel_reset_set_sub(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel)
		return -EINVAL;

	if (enable) {
		rc = dsi_panel_reset(panel);
		if (rc) {
			pr_err("[%s] failed to reset panel, rc=%d\n", panel->name, rc);
			rc = -EINVAL;
		}
	} else {
		if (gpio_is_valid(panel->reset_config.disp_en_gpio))
			gpio_set_value(panel->reset_config.disp_en_gpio, 0);

		if (gpio_is_valid(panel->reset_config.reset_gpio))
			gpio_set_value(panel->reset_config.reset_gpio, 0);

		if (gpio_is_valid(panel->reset_config.lcd_mode_sel_gpio))
			gpio_set_value(panel->reset_config.lcd_mode_sel_gpio, 0);
	}

	return rc;
}

static int oplus_panel_reset_set(struct dsi_panel *panel, SET_POWER_MODE mode)
{
	int rc = 0;

	if (!panel) {
		pr_err("[%s]panel is NULL!\n", __func__);
		return -EINVAL;
	}

	if (mode == SET_PWR_ON) {
		rc = oplus_panel_reset_set_sub(panel, true);
		if (rc) {
			pr_err("[%s]config panel reset off fail\n", __func__);
			panel->oplus_panel_power.panel_reset_enable_flag = false;
			rc = -EINVAL;
			goto exit;
		}
		panel->oplus_panel_power.panel_reset_enable_flag = true;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.reset_mdelay.on_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.reset_mdelay.gesture_on_behind_mdelay);
	} else {
		rc = oplus_panel_reset_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel reset off fail\n", __func__);
			panel->oplus_panel_power.panel_reset_enable_flag = true;
			return -EINVAL;
		}
		panel->oplus_panel_power.panel_reset_enable_flag = false;
		if(!g_tp_gesture_enable_flag)
			oplus_mdelay(panel->oplus_panel_power.reset_mdelay.off_behind_mdelay);
		else
			oplus_mdelay(panel->oplus_panel_power.reset_mdelay.gesture_off_behind_mdelay);
	}

	return rc;
exit:
	oplus_panel_reset_set_sub(panel, false);
	return rc;
}

static int oplus_panel_reset_sel_set(struct dsi_panel *panel, SET_POWER_MODE mode)
{
	int rc = 0;

	if (!panel) {
		pr_err("[%s]panel is NULL!\n", __func__);
		return -EINVAL;
	}

	if (mode == SET_PWR_ON) {
		/* disp_en_gpio, lcd_mode_sel_gpio have been enable in dsi_panel_reset() func*/
	} else {
		if (gpio_is_valid(panel->reset_config.disp_en_gpio))
			gpio_set_value(panel->reset_config.disp_en_gpio, 0);

		if (gpio_is_valid(panel->reset_config.lcd_mode_sel_gpio))
			gpio_set_value(panel->reset_config.lcd_mode_sel_gpio, 0);
	}

	return rc;
}

static int oplus_panel_cmd_set(struct dsi_panel *panel, SET_POWER_MODE mode)
{
	int rc = 0;

	if (!panel) {
		pr_err("[%s]panel is NULL!\n", __func__);
		return -EINVAL;
	}

	/*  to be realized */
	return rc;
}

int oplus_panel_power_operation_set(struct dsi_panel *panel,
	int power_set_oper, SET_POWER_MODE set_mode)
{
	int rc = 0;

	if (!panel) {
		pr_err("[%s]panel is NULL!\n", __func__);
		return -EINVAL;
	}

	switch (power_set_oper) {
	case POWER_SET_VCI :
		rc = oplus_panel_vci_set(panel, set_mode);
		if (rc)
			pr_err("[%s]panel %s step : set vci fail !\n", __func__, set_mode ? "on" : "off");
		else
			pr_info("[%s]panel %s step : set vci ok !\n", __func__, set_mode ? "on" : "off");
		break;
	case POWER_SET_VDDIO :
		rc = oplus_panel_vddio_set(panel, set_mode);
		if (rc)
			pr_err("[%s]panel %s step : set vddio fail !\n", __func__, set_mode ? "on" : "off");
		else
			pr_info("[%s]panel %s step : set vddio ok !\n", __func__, set_mode ? "on" : "off");
		break;
	case POWER_SET_RESET :
		rc = oplus_panel_reset_set(panel, set_mode);
		if (rc)
			pr_err("[%s]panel %s step : set reset fail !\n", __func__, set_mode ? "on" : "off");
		else
			pr_info("[%s]panel %s step : set reset ok !\n", __func__, set_mode ? "on" : "off");
		break;
	case POWER_SET_BIAS :
		rc = oplus_panel_bias_set(panel, set_mode);
		if (rc)
			pr_err("[%s]panel %s step : set bias fail !\n", __func__, set_mode ? "on" : "off");
		else
			pr_info("[%s]panel %s step : set bias ok !\n", __func__, set_mode ? "on" : "off");
		break;
	case POWER_SET_REGULATOR :
		rc = oplus_panel_regulator_set(panel, set_mode);
		if (rc)
			pr_err("[%s]panel %s step : set regulator fail !\n", __func__, set_mode ? "on" : "off");
		else
			pr_info("[%s]panel %s step : set regulator ok !\n", __func__, set_mode ? "on" : "off");
		break;
	case POWER_SET_PINCTRL_STATE :
		rc = oplus_panel_pinctrl_set(panel, set_mode);
		if (rc)
			pr_err("[%s]panel %s step : set pinctrl fail !\n", __func__, set_mode ? "on" : "off");
		else
			pr_info("[%s]panel %s step : set pinctrl ok !\n", __func__, set_mode ? "on" : "off");
		break;
	case POWER_TP_LOAD_FW :
		rc = oplus_tp_load_fw(panel, set_mode);
		if (rc)
			pr_err("[%s]panel %s step : tp load fw fail !\n", __func__, set_mode ? "on" : "off");
		else
			pr_info("[%s]panel %s step : tp load fw ok !\n", __func__, set_mode ? "on" : "off");
		break;
	case POWER_SET_CMD :
		rc = oplus_panel_cmd_set(panel, set_mode);
		if (rc)
			pr_err("[%s]panel %s step : set cmd fail !\n", __func__, set_mode ? "on" : "off");
		else
			pr_info("[%s]panel %s step : set cmd ok !\n", __func__, set_mode ? "on" : "off");
		break;
	case POWER_SET_RESET_SEL :
		rc = oplus_panel_reset_sel_set(panel, set_mode);
		if (rc)
			pr_err("[%s]panel %s step : set reset_sel fail !\n", __func__, set_mode ? "on" : "off");
		else
			pr_info("[%s]panel %s step : set reset_sel ok !\n", __func__, set_mode ? "on" : "off");
		break;
	case POWER_SET_END :
		pr_info("[%s]panel %s step : POWER_SET_END !\n", __func__, set_mode ? "on" : "off");
		break;
	default:
		pr_err("[%s]panel %s step : UNKNOW oper[%d]!\n", __func__, set_mode ? "on" : "off", power_set_oper);
		break;
	}

	return rc;
}


static void oplus_panel_on_fail(struct dsi_panel *panel)
{
	int rc = 0;

	if (g_oplus_display_debug_switch)
		pr_info("[%s] start +++\n", __func__);

	if (!panel) {
		pr_err("[%s]panel is NULL!\n", __func__);
		return;
	}

	if (panel->oplus_panel_power.panel_reset_enable_flag) {
		rc = oplus_panel_reset_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel reset off fail\n", __func__);
			panel->oplus_panel_power.panel_reset_enable_flag = true;
		} else {
			panel->oplus_panel_power.panel_reset_enable_flag = false;
		}
	}

	if (panel->oplus_panel_power.panel_bias_enable_flag) {
		rc = oplus_panel_bias_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel bias off fail\n", __func__);
			panel->oplus_panel_power.panel_bias_enable_flag = true;
		} else {
			panel->oplus_panel_power.panel_bias_enable_flag = false;
		}
	}

	if (panel->oplus_panel_power.panel_vddio_enable_flag) {
		rc = oplus_panel_vddio_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel vddio off fail\n", __func__);
			panel->oplus_panel_power.panel_vddio_enable_flag = true;
		} else {
			panel->oplus_panel_power.panel_vddio_enable_flag = false;
		}
	}

	if (panel->oplus_panel_power.panel_vci_enable_flag) {
		rc = oplus_panel_vci_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel vci off fail\n", __func__);
			panel->oplus_panel_power.panel_vci_enable_flag = true;
		} else {
			panel->oplus_panel_power.panel_vci_enable_flag = false;
		}
	}

	if (panel->oplus_panel_power.panel_regulator_enable_flag) {
		rc = oplus_panel_regulator_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel regulator disable fail\n", __func__);
			panel->oplus_panel_power.panel_regulator_enable_flag = true;
		} else {
			panel->oplus_panel_power.panel_regulator_enable_flag = false;
		}
	}

	if (panel->oplus_panel_power.panel_pinctrl_state_enable_flag) {
		rc = oplus_panel_pinctrl_set_sub(panel, false);
		if (rc) {
			pr_err("[%s]config panel pinctrl state off fail\n", __func__);
			panel->oplus_panel_power.panel_pinctrl_state_enable_flag = true;
		} else {
			panel->oplus_panel_power.panel_pinctrl_state_enable_flag = false;
		}
	}

	panel->oplus_panel_power.tp_load_fw_succ_flag = false;
	if (g_oplus_display_debug_switch)
		pr_info("[%s] end ---\n", __func__);
}

static void oplus_panel_power_set_seq_debug(u8 *power_set_seq,
	char *set_seq_name, u8 set_seq_count)
{
	int i = 0, j = 0;

	pr_info("[%s]set_seq_count=%d\n", __func__, set_seq_count);
	for (i = 0; i < set_seq_count; i++) {
		for (j = 0; j < ARRAY_SIZE(panel_power_ctrl_elect); j++) {
			if (power_set_seq[i] == panel_power_ctrl_elect[j].operation_num)
				pr_info("[%s]%s[%d]:%s, operation_num=%d\n",
					__func__, set_seq_name, i,
					panel_power_ctrl_elect[j].operation_name,
					panel_power_ctrl_elect[j].operation_num);
		}
	}
}

static int oplus_panel_power_sequence_set_sub(struct dsi_panel *panel,
	SET_POWER_MODE mode, u8 *power_set_seq, u8 set_seq_count, char *set_seq_name)
{
	int step_inx = 0;
	int rc = 0;

	if (g_oplus_display_debug_switch)
		pr_info("[%s]<%s>:set_seq_count=%d\n", __func__, set_seq_name, set_seq_count);
	for (step_inx = 0; step_inx < set_seq_count; step_inx++) {
		if (power_set_seq[step_inx] == POWER_SET_END)
			break;
		rc = oplus_panel_power_operation_set(panel, power_set_seq[step_inx], mode);
		if (rc) {
			pr_err("[%s]%s fail, set_seq=%d\n",
				__func__, set_seq_name, power_set_seq[step_inx]);
			return -EINVAL;
		}
	}

	if (mode == SET_PWR_ON)
		pr_info("[%s]panel on success!\n", __func__);
	else
		pr_info("[%s]panel off success!\n", __func__);

	return rc;
}

int oplus_panel_on(struct dsi_panel *panel)
{
	u8 power_set_seq[PANEL_POWER_SEQ_MAX] = {0};
	char set_seq_name[PWR_SEQ_NAME_LEN] = {0};
	u8 set_seq_count = 0;
	int rc = 0;

	if (g_oplus_display_debug_switch)
		pr_info("[%s] start +++\n", __func__);

	if (!panel) {
		pr_err("[%s]panel is NULL!\n", __func__);
		return -EINVAL;
	}

	if (!g_tp_gesture_enable_flag) {
		memcpy(power_set_seq, panel->oplus_panel_power.panel_on_power_seq, PANEL_POWER_SEQ_MAX * sizeof(u8));
		set_seq_count = panel->oplus_panel_power.panel_on_power_seq_num;
		strncpy(set_seq_name, "panel_on_power_seq", strlen("panel_on_power_seq") + 1);
	} else {
		memcpy(power_set_seq, panel->oplus_panel_power.panel_gesture_on_power_seq, PANEL_POWER_SEQ_MAX * sizeof(u8));
		set_seq_count = panel->oplus_panel_power.panel_gesture_on_power_seq_num;
		strncpy(set_seq_name, "panel_gesture_on_power_seq", strlen("panel_gesture_on_power_seq") + 1);
	}
	/* For debug */
	if (g_oplus_display_debug_switch) {
		oplus_panel_power_set_seq_debug(power_set_seq, set_seq_name, set_seq_count);
	}

	rc = oplus_panel_power_sequence_set_sub(panel, SET_PWR_ON,
		power_set_seq, set_seq_count, set_seq_name);
	if (rc) {
		pr_err("[%s]%s fail\n",	__func__, set_seq_name);
		rc = -EINVAL;
		goto exit;
	} else {
		pr_err("[%s]%s success!\n", __func__, set_seq_name);
	}

	if (g_oplus_display_debug_switch)
		pr_info("[%s] end ---\n", __func__);
	return rc;

exit:
	oplus_panel_on_fail(panel);
	return rc;
}

int oplus_panel_off(struct dsi_panel *panel)
{
	u8 power_set_seq[PANEL_POWER_SEQ_MAX] = {0};
	char set_seq_name[PWR_SEQ_NAME_LEN] = {0};
	u8 set_seq_count = 0;
	int rc = 0;

	if (g_oplus_display_debug_switch)
		pr_info("[%s] start +++\n", __func__);

	if (!panel) {
		pr_err("[%s]panel is NULL!\n", __func__);
		return -EINVAL;
	}

	if (!g_tp_gesture_enable_flag) {
		if (panel->oplus_panel_power.panel_shutdown_power_set_flag && g_shutdown_flag) {
			memcpy(power_set_seq, panel->oplus_panel_power.panel_shutdown_power_seq, PANEL_POWER_SEQ_MAX * sizeof(u8));
			set_seq_count = panel->oplus_panel_power.panel_shutdown_power_seq_num;
			strncpy(set_seq_name, "panel_shutdown_power_seq", strlen("panel_shutdown_power_seq") + 1);
		} else {
			memcpy(power_set_seq, panel->oplus_panel_power.panel_off_power_seq, PANEL_POWER_SEQ_MAX * sizeof(u8));
			set_seq_count = panel->oplus_panel_power.panel_off_power_seq_num;
			strncpy(set_seq_name, "panel_off_power_seq", strlen("panel_off_power_seq") + 1);
		}
	} else {
		if (panel->oplus_panel_power.panel_shutdown_power_set_flag && g_shutdown_flag) {
			memcpy(power_set_seq, panel->oplus_panel_power.panel_gesture_shutdown_power_seq, PANEL_POWER_SEQ_MAX * sizeof(u8));
			set_seq_count = panel->oplus_panel_power.panel_gesture_shutdown_power_seq_num;
			strncpy(set_seq_name, "panel_gesture_shutdown_power_seq", strlen("panel_gesture_shutdown_power_seq") + 1);
		} else {
			memcpy(power_set_seq, panel->oplus_panel_power.panel_gesture_off_power_seq, PANEL_POWER_SEQ_MAX * sizeof(u8));
			set_seq_count = panel->oplus_panel_power.panel_gesture_off_power_seq_num;
			strncpy(set_seq_name, "panel_gesture_off_power_seq", strlen("panel_gesture_off_power_seq") + 1);
		}
	}
	/* For debug */
	if (g_oplus_display_debug_switch) {
		oplus_panel_power_set_seq_debug(power_set_seq, set_seq_name, set_seq_count);
	}

	rc = oplus_panel_power_sequence_set_sub(panel, SET_PWR_OFF,
		power_set_seq, set_seq_count, set_seq_name);
	if (rc) {
		pr_err("[%s]PANEL_OFF_STEP_SEQ0 fail\n",	__func__);
		return -EINVAL;
	}

	if (g_oplus_display_debug_switch)
		pr_info("[%s] end ---\n", __func__);

	return rc;
}

