/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_dc_diming.c
** Description : oplus dc_diming feature
** Version : 1.0
** Date : 2020/04/15
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Qianxu         2020/04/15        1.0           Build this moudle
******************************************************************/

#include "oplus_display_private_api.h"
#include "oplus_dc_diming.h"
#include "oplus_onscreenfingerprint.h"
#include "oplus_aod.h"
#include "oplus_display_panel_seed.h"
#include "oplus_display_panel_common.h"
#include "sde_trace.h"

int oplus_dimlayer_bl = 0;
int oplus_dc_dimlayer_bl_enabled = 0;
int oplus_datadimming_v3_skip_frame = 2;
int oplus_panel_alpha = 0;
int oplus_underbrightness_alpha = 0;
int oplus_dc_seed_backlight = 0;

static struct dsi_panel_cmd_set oplus_dc_priv_seed_cmd_set;
int oplus_datadimming_vblank_count = 0;
atomic_t oplus_datadimming_vblank_ref = ATOMIC_INIT(0);
int oplus_dc_dimlayer_bl_alpha_v2 = 260;
bool oplus_dc_v2_on = false;
extern u32 oplus_last_backlight;
extern int seed_mode;

extern int oplus_dc_dimlayer_bl_on_vblank;
extern int oplus_dc_dimlayer_bl_off_vblank;
extern int oplus_dc_dimlayer_bl_delay;
extern int oplus_dc_dimlayer_bl_delay_after;
extern int oplus_dimlayer_bl_enable;
extern int oplus_dimlayer_bl_enable_v2;
extern int oplus_dimlayer_bl_enable_v2_real;
extern int oplus_dimlayer_bl_enable_v3;
extern int oplus_dimlayer_bl_enable_v3_real;
extern int oplus_fod_on_vblank;
extern int oplus_fod_off_vblank;
extern bool oplus_skip_datadimming_sync;
extern int oplus_dimlayer_hbm_vblank_count;
extern atomic_t oplus_dimlayer_hbm_vblank_ref;
extern int oplus_dc2_alpha;
extern int oplus_dc_seed_backlight;
extern int oplus_panel_alpha;
extern ktime_t oplus_backlight_time;
#ifdef OPLUS_FEATURE_AOD_RAMLESS
extern int oplus_display_mode;
#endif /* OPLUS_FEATURE_AOD_RAMLESS */
static struct oplus_brightness_alpha brightness_seed_alpha_lut_dc[] = {
	{0, 2000},
	{10, 1959},
	{30, 1956},
	{60, 1953},
	{90, 1942},
	{100, 1938},
	{140, 1930},
	{200, 1920},
	{300, 1860},
	{400, 1790},
	{500, 1700},
	{600, 1560},
	{700, 1400},
	{800, 1210},
	{900, 980},
	{1000, 690},
	{1100, 380},
	{1200, 0},
};

/*Jiasong.ZhongPSW.MM.Display.LCD.Stable,2020-09-17 add for DC backlight */
int dsi_panel_parse_oplus_dc_config(struct dsi_panel *panel)
{
	int rc = 0;
	int i;
	u32 length = 0;
	u32 count = 0;
	u32 size = 0;
	u32 *arr_32 = NULL;
	const u32 *arr;
	struct dsi_parser_utils *utils = &panel->utils;
	struct oplus_brightness_alpha *seq;

	if (panel->host_config.ext_bridge_mode)
		return 0;

	arr = utils->get_property(utils->data, "oplus,dsi-dc-brightness", &length);
	if (!arr) {
		DSI_ERR("[%s] oplus,dsi-dc-brightness  not found\n", panel->name);
		return -EINVAL;
	}

	if (length & 0x1) {
		DSI_ERR("[%s] oplus,dsi-dc-brightness length error\n", panel->name);
		return -EINVAL;
	}

	DSI_DEBUG("oplus,dsi-dc-brightness SEQ LENGTH = %d\n", length);
	length = length / sizeof(u32);
	size = length * sizeof(u32);

	arr_32 = kzalloc(size, GFP_KERNEL);
	if (!arr_32) {
		rc = -ENOMEM;
		goto error;
	}

	rc = utils->read_u32_array(utils->data, "oplus,dsi-dc-brightness",
					arr_32, length);
	if (rc) {
		DSI_ERR("[%s] cannot read dsi-dc-brightness\n", panel->name);
		goto error_free_arr_32;
	}

	count = length / 2;
	size = count * sizeof(*seq);
	seq = kzalloc(size, GFP_KERNEL);
	if (!seq) {
		rc = -ENOMEM;
		goto error_free_arr_32;
	}

	panel->dc_ba_seq = seq;
	panel->dc_ba_count = count;

	for (i = 0; i < length; i += 2) {
		seq->brightness = arr_32[i];
		seq->alpha = arr_32[i + 1];
		seq++;
	}

error_free_arr_32:
	kfree(arr_32);
error:
	return rc;
}

int sde_connector_update_backlight(struct drm_connector *connector, bool post)
{
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct dsi_display *dsi_display;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	dsi_display = c_conn->display;
	if (!dsi_display || !dsi_display->panel || !dsi_display->panel->cur_mode) {
		SDE_ERROR("Invalid params(s) dsi_display %pK, panel %pK\n",
				dsi_display,
				((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	if (!connector->state || !connector->state->crtc)
		return 0;

	if (oplus_dimlayer_bl != oplus_dc_dimlayer_bl_enabled) {
		struct sde_connector *c_conn = to_sde_connector(connector);
		struct drm_crtc *crtc = connector->state->crtc;
		u32 current_vblank;
		int on_vblank = 0;
		int off_vblank = 0;
		int vblank = 0;
		int ret = 0;
		int vblank_get = -EINVAL;
		int on_delay = 0, on_delay_after = 0;
		int off_delay = 0, off_delay_after = 0;
		int delay = 0, delay_after = 0;
		/* int rc = 0; */

		if (sde_crtc_get_fingerprint_mode(crtc->state)) {
			oplus_dc_dimlayer_bl_enabled = oplus_dimlayer_bl;
			goto done;
		}

		if (oplus_dc_dimlayer_bl_on_vblank != INT_MAX)
			on_vblank = oplus_dc_dimlayer_bl_on_vblank;

		if (oplus_dc_dimlayer_bl_off_vblank != INT_MAX)
			off_vblank = oplus_dc_dimlayer_bl_off_vblank;


		if (oplus_dimlayer_bl) {
			vblank = on_vblank;
			delay = on_delay;
			delay_after = on_delay_after;
		} else {
			vblank = off_vblank;
			delay = off_delay;
			delay_after = off_delay_after;
		}

		if (oplus_dc_dimlayer_bl_delay >= 0)
			delay = oplus_dc_dimlayer_bl_delay;

		if (oplus_dc_dimlayer_bl_delay_after >= 0)
			delay_after = oplus_dc_dimlayer_bl_delay_after;

		vblank_get = drm_crtc_vblank_get(crtc);
		if (vblank >= 0) {
			if (!post) {
				oplus_dc_dimlayer_bl_enabled = oplus_dimlayer_bl;
				current_vblank = drm_crtc_vblank_count(crtc);
				ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
						current_vblank != drm_crtc_vblank_count(crtc),
						msecs_to_jiffies(34));
				current_vblank = drm_crtc_vblank_count(crtc) + vblank;
				if (delay > 0)
					usleep_range(delay, delay + 100);
				oplus_dc_sde_connector_update_bl_scale(c_conn);
				if (delay_after)
					usleep_range(delay_after, delay_after + 100);
				if (vblank > 0) {
					ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
							current_vblank == drm_crtc_vblank_count(crtc),
							msecs_to_jiffies(17 * 3));
				}
			}
		} else {
			if (!post) {
				current_vblank = drm_crtc_vblank_count(crtc);
				ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
						current_vblank != drm_crtc_vblank_count(crtc),
						msecs_to_jiffies(34));
			} else {
				if (vblank < -1) {
					current_vblank = drm_crtc_vblank_count(crtc) + 1 - vblank;
					ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
							current_vblank == drm_crtc_vblank_count(crtc),
							msecs_to_jiffies(17 * 3));
				}
				oplus_dc_dimlayer_bl_enabled = oplus_dimlayer_bl;

				if (delay > 0)
					usleep_range(delay, delay + 100);
				oplus_dc_sde_connector_update_bl_scale(c_conn);
				if (delay_after)
					usleep_range(delay_after, delay_after + 100);
			}
		}
		if (!vblank_get)
			drm_crtc_vblank_put(crtc);
	}

	if (oplus_dimlayer_bl_enable_v2 != oplus_dimlayer_bl_enable_v2_real) {
		struct sde_connector *c_conn = to_sde_connector(connector);

		oplus_dimlayer_bl_enable_v2_real = oplus_dimlayer_bl_enable_v2;
		oplus_dc_sde_connector_update_bl_scale(c_conn);
	}

done:
	if (post) {
		if (oplus_datadimming_vblank_count> 0) {
			oplus_datadimming_vblank_count--;
		} else {
			while (atomic_read(&oplus_datadimming_vblank_ref) > 0) {
				drm_crtc_vblank_put(connector->state->crtc);
				atomic_dec(&oplus_datadimming_vblank_ref);
			}
		}
	}

	return 0;
}

int sde_connector_update_hbm(struct drm_connector *connector)
{
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct dsi_display *dsi_display;
	struct sde_connector_state *c_state;
	int rc = 0;
	int fingerprint_mode;

	if (!c_conn) {
		SDE_ERROR("Invalid params sde_connector null\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	c_state = to_sde_connector_state(connector->state);

	dsi_display = c_conn->display;
	if (!dsi_display || !dsi_display->panel) {
		SDE_ERROR("Invalid params(s) dsi_display %pK, panel %pK\n",
			dsi_display,
			((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	if (!c_conn->encoder || !c_conn->encoder->crtc ||
	    !c_conn->encoder->crtc->state) {
		return 0;
	}

	fingerprint_mode = sde_crtc_get_fingerprint_mode(c_conn->encoder->crtc->state);

	if (OPLUS_DISPLAY_AOD_SCENE == get_oplus_display_scene()) {
		if (sde_crtc_get_fingerprint_pressed(c_conn->encoder->crtc->state)) {
			sde_crtc_set_onscreenfinger_defer_sync(c_conn->encoder->crtc->state, true);
		} else {
			sde_crtc_set_onscreenfinger_defer_sync(c_conn->encoder->crtc->state, false);
			fingerprint_mode = false;
		}
	} else {
		sde_crtc_set_onscreenfinger_defer_sync(c_conn->encoder->crtc->state, false);
	}

	if (fingerprint_mode != dsi_display->panel->is_hbm_enabled) {
		struct drm_crtc *crtc = c_conn->encoder->crtc;
		struct dsi_panel *panel = dsi_display->panel;
		int vblank = 0;
		u32 target_vblank, current_vblank;
		int ret;

		if (oplus_fod_on_vblank >= 0)
			panel->cur_mode->priv_info->fod_on_vblank = oplus_fod_on_vblank;
		if (oplus_fod_off_vblank >= 0)
			panel->cur_mode->priv_info->fod_off_vblank = oplus_fod_off_vblank;

		pr_err("OnscreenFingerprint mode: %s",
		       fingerprint_mode ? "Enter" : "Exit");

		dsi_display->panel->is_hbm_enabled = fingerprint_mode;
		if (fingerprint_mode) {
#ifdef OPLUS_FEATURE_AOD_RAMLESS
			if (!dsi_display->panel->oplus_priv.is_aod_ramless || oplus_display_mode) {
#endif /* OPLUS_FEATURE_AOD_RAMLESS */
				mutex_lock(&dsi_display->panel->panel_lock);

				if (!dsi_display->panel->panel_initialized) {
					dsi_display->panel->is_hbm_enabled = false;
					pr_err("panel not initialized, failed to Enter OnscreenFingerprint\n");
					mutex_unlock(&dsi_display->panel->panel_lock);
					return 0;
				}
				dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
						DSI_CORE_CLK, DSI_CLK_ON);

				if (oplus_dc_seed_backlight && dsi_display->panel->is_dc_support) {
						int frame_time_us;
						frame_time_us = mult_frac(1000, 1000, panel->cur_mode->timing.refresh_rate);
						oplus_dc_panel_process_dimming_v2(panel, panel->bl_config.bl_level, true);
						mipi_dsi_dcs_set_display_brightness(&panel->mipi_device, panel->bl_config.bl_level);
						oplus_dc_panel_process_dimming_v2_post(panel, true);
						usleep_range(frame_time_us, frame_time_us + 100);
				}

#ifdef OPLUS_FEATURE_AOD_RAMLESS
				if (dsi_display->panel->oplus_priv.is_aod_ramless) {
					ktime_t delta = ktime_sub(ktime_get(), oplus_backlight_time);
					s64 delta_us = ktime_to_us(delta);
					if (delta_us < 34000 && delta_us >= 0)
						usleep_range(34000 - delta_us, 34000 - delta_us + 100);
				}
#endif /* OPLUS_FEATURE_AOD_RAMLESS */

				if (OPLUS_DISPLAY_AOD_SCENE != get_oplus_display_scene() &&
						dsi_display->panel->bl_config.bl_level) {
					if (dsi_display->config.panel_mode != DSI_OP_VIDEO_MODE) {
						current_vblank = drm_crtc_vblank_count(crtc);
						ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
								current_vblank != drm_crtc_vblank_count(crtc),
								msecs_to_jiffies(17));
					}

					vblank = panel->cur_mode->priv_info->fod_on_vblank;
					target_vblank = drm_crtc_vblank_count(crtc) + vblank;

					if (!strcmp(dsi_display->panel->oplus_priv.vendor_name, "S6E3FC3"))
						mdelay(8);
					rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_HBM_ON);

					if (vblank) {
						ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
								target_vblank == drm_crtc_vblank_count(crtc),
								msecs_to_jiffies((vblank + 1) * 17));
						if (!ret) {
							pr_err("OnscreenFingerprint failed to wait vblank timeout target_vblank=%d current_vblank=%d\n",
									target_vblank, drm_crtc_vblank_count(crtc));
						}
					}
				} else {
					current_vblank = drm_crtc_vblank_count(crtc);
					ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
							current_vblank != drm_crtc_vblank_count(crtc),
							msecs_to_jiffies(17));
					SDE_ATRACE_BEGIN("DSI_CMD_AOD_HBM_ON1");
					rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_AOD_HBM_ON);
					SDE_ATRACE_END("DSI_CMD_AOD_HBM_ON1");
				}

				dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
						DSI_CORE_CLK, DSI_CLK_OFF);

				mutex_unlock(&dsi_display->panel->panel_lock);
				if (rc) {
					pr_err("failed to send DSI_CMD_HBM_ON cmds, rc=%d\n", rc);
					return rc;
				}
#ifdef OPLUS_FEATURE_AOD_RAMLESS
			}
#endif /* OPLUS_FEATURE_AOD_RAMLESS */
		} else {
			mutex_lock(&dsi_display->panel->panel_lock);

			if (!dsi_display->panel->panel_initialized) {
				dsi_display->panel->is_hbm_enabled = true;
				pr_err("panel not initialized, failed to Exit OnscreenFingerprint\n");
				mutex_unlock(&dsi_display->panel->panel_lock);
				return 0;
			}

			oplus_skip_datadimming_sync = true;
			oplus_panel_update_backlight_unlock(panel);
			oplus_skip_datadimming_sync = false;

			vblank = panel->cur_mode->priv_info->fod_off_vblank;
			target_vblank = drm_crtc_vblank_count(crtc) + vblank;

			dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
					     DSI_CORE_CLK, DSI_CLK_ON);
			if(OPLUS_DISPLAY_AOD_HBM_SCENE == get_oplus_display_scene()) {
				if (OPLUS_DISPLAY_POWER_DOZE_SUSPEND == get_oplus_display_power_status() ||
					OPLUS_DISPLAY_POWER_DOZE == get_oplus_display_power_status()) {
					rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_AOD_HBM_OFF);
					mutex_unlock(&dsi_display->panel->panel_lock);
					mutex_lock(&dsi_display->panel->panel_lock);
					oplus_update_aod_light_mode_unlock(panel);
					set_oplus_display_scene(OPLUS_DISPLAY_AOD_SCENE);
				} else {
					rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_SET_NOLP);
					set_oplus_display_scene(OPLUS_DISPLAY_NORMAL_SCENE);
					/* set nolp would exit hbm, restore when panel status on hbm */
					if(panel->bl_config.bl_level > panel->bl_config.bl_normal_max_level)
						oplus_panel_update_backlight_unlock(panel);
					if (oplus_display_get_hbm_mode())
						rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_AOD_HBM_ON);
				}
			} else if (oplus_display_get_hbm_mode()) {
				/* Do nothing to skip hbm off */
			} else if(OPLUS_DISPLAY_AOD_SCENE == get_oplus_display_scene()) {
				rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_AOD_HBM_OFF);
				mutex_unlock(&dsi_display->panel->panel_lock);
				mutex_lock(&dsi_display->panel->panel_lock);
				oplus_update_aod_light_mode_unlock(panel);
			} else {
				rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_HBM_OFF);
				oplus_panel_update_backlight_unlock(panel);
			}

			dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
					     DSI_CORE_CLK, DSI_CLK_OFF);
			mutex_unlock(&dsi_display->panel->panel_lock);
			if (vblank) {
				ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
						target_vblank == drm_crtc_vblank_count(crtc),
						msecs_to_jiffies((vblank + 1) * 17 ));
				if (!ret) {
					pr_err("OnscreenFingerprint failed to wait vblank timeout target_vblank=%d current_vblank=%d\n",
							target_vblank, drm_crtc_vblank_count(crtc));
				}
			}
		}
	}

	if (oplus_dimlayer_hbm_vblank_count > 0) {
		oplus_dimlayer_hbm_vblank_count--;
	} else {
		while (atomic_read(&oplus_dimlayer_hbm_vblank_ref) > 0) {
			drm_crtc_vblank_put(connector->state->crtc);
			atomic_dec(&oplus_dimlayer_hbm_vblank_ref);
		}
	}

	return 0;
}

int oplus_dc_seed_bright_to_alpha(int brightness)
{
	struct dsi_display *display = get_main_display();
	struct oplus_brightness_alpha *lut = NULL;
	int count = 0;
	int i = 0;
	int alpha;

	if (!display)
		return 0;

	if (oplus_panel_alpha)
		return oplus_panel_alpha;

	if (display->panel->dc_ba_seq && display->panel->dc_ba_count) {
		count = display->panel->dc_ba_count;
		lut = display->panel->dc_ba_seq;
	} else {
		count = ARRAY_SIZE(brightness_seed_alpha_lut_dc);
		lut = brightness_seed_alpha_lut_dc;
	}

	for (i = 0; i < count; i++) {
		if (lut[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = lut[0].alpha;
	else if (i == count)
		alpha = lut[count - 1].alpha;
	else
		alpha = interpolate(brightness, lut[i-1].brightness,
				    lut[i].brightness, lut[i-1].alpha,
				    lut[i].alpha, display->panel->oplus_priv.bl_interpolate_nosub);

	return alpha;
}

static struct dsi_panel_cmd_set *
_oplus_dc_dsi_update_seed_backlight(struct dsi_panel *panel, int brightness,
				enum dsi_cmd_set_type type)
{
	enum dsi_cmd_set_state state;
	struct dsi_cmd_desc *cmds;
	struct dsi_cmd_desc *oplus_cmd;
	u8 *tx_buf;
	int count, rc = 0;
	int i = 0;
	int k = 0;
	static int dc_last_seed_alpha = 0;
	int alpha = oplus_dc_seed_bright_to_alpha(brightness);
	if (((brightness < panel->dc_last_level) && (alpha < dc_last_seed_alpha)) ||
		((brightness > panel->dc_last_level) && (alpha > dc_last_seed_alpha)))
		alpha = dc_last_seed_alpha;
	panel->dc_last_level = brightness;
	dc_last_seed_alpha = alpha;

	if (type != DSI_CMD_SEED_MODE0 &&
		type != DSI_CMD_SEED_MODE1 &&
		type != DSI_CMD_SEED_MODE2 &&
		type != DSI_CMD_SEED_MODE3 &&
		type != DSI_CMD_SEED_MODE4 &&
		type != DSI_CMD_SEED_OFF) {
		return NULL;
	}

	if (type == DSI_CMD_SEED_OFF)
		type = DSI_CMD_SEED_MODE0;

	cmds = panel->cur_mode->priv_info->cmd_sets[type].cmds;
	count = panel->cur_mode->priv_info->cmd_sets[type].count;
	state = panel->cur_mode->priv_info->cmd_sets[type].state;

	oplus_cmd = kmemdup(cmds, sizeof(*cmds) * count, GFP_KERNEL);
	if (!oplus_cmd) {
		rc = -ENOMEM;
		goto error;
	}

	for (i = 0; i < count; i++)
		oplus_cmd[i].msg.tx_buf = NULL;

	for (i = 0; i < count; i++) {
		u32 size;

		size = oplus_cmd[i].msg.tx_len * sizeof(u8);

		oplus_cmd[i].msg.tx_buf = kmemdup(cmds[i].msg.tx_buf, size, GFP_KERNEL);
		if (!oplus_cmd[i].msg.tx_buf) {
			rc = -ENOMEM;
			goto error;
		}
	}

	for (i = 0; i < count; i++) {
		if (oplus_cmd[i].msg.tx_len != 0x16)
			continue;
		tx_buf = (u8 *)oplus_cmd[i].msg.tx_buf;
		for (k = 0; k < oplus_cmd[i].msg.tx_len; k++) {
			if (k == 0) {
				continue;
			}
			tx_buf[k] = tx_buf[k] * (2000 - alpha) / 2000;
		}
	}

	if (oplus_dc_priv_seed_cmd_set.cmds) {
		for (i = 0; i < oplus_dc_priv_seed_cmd_set.count; i++)
			kfree(oplus_dc_priv_seed_cmd_set.cmds[i].msg.tx_buf);
		kfree(oplus_dc_priv_seed_cmd_set.cmds);
	}

	oplus_dc_priv_seed_cmd_set.cmds = oplus_cmd;
	oplus_dc_priv_seed_cmd_set.count = count;
	oplus_dc_priv_seed_cmd_set.state = state;
	oplus_dc2_alpha = alpha;

	return &oplus_dc_priv_seed_cmd_set;

error:
	if (oplus_cmd) {
		for (i = 0; i < count; i++)
			kfree(oplus_cmd[i].msg.tx_buf);
		kfree(oplus_cmd);
	}
	return ERR_PTR(rc);
}

void oplus_dc_dsi_update_seed_backlight(struct dsi_panel *panel, int seed_value,
		enum dsi_cmd_set_type type, struct dsi_cmd_desc **cmds, u32 *count, enum dsi_cmd_set_state *state) {
	struct dsi_panel_cmd_set *oplus_cmd_set = NULL;
	oplus_cmd_set = _oplus_dc_dsi_update_seed_backlight(panel, seed_value, type);
	if(!IS_ERR_OR_NULL(oplus_cmd_set)) {
		*cmds = oplus_cmd_set->cmds;
		*count = oplus_cmd_set->count;
		*state = oplus_cmd_set->state;
	}
	else
		pr_err("%s: update seed value failed!\n", __func__);

	return;
}

static bool oplus_datadimming_v2_need_flush = false;
static bool oplus_datadimming_v2_need_sync = false;
void oplus_dc_panel_process_dimming_v2_post(struct dsi_panel *panel, bool force_disable)
{
	struct dsi_display *display = get_main_display();
	struct drm_connector *dsi_connector = display->drm_conn;

	if (oplus_datadimming_v2_need_flush) {
		if (oplus_datadimming_v2_need_sync && dsi_connector && dsi_connector->state && dsi_connector->state->crtc) {
			struct drm_crtc *crtc = dsi_connector->state->crtc;
			int frame_time_us, ret = 0;
			u32 current_vblank;

			frame_time_us = mult_frac(1000, 1000, panel->cur_mode->timing.refresh_rate);

			current_vblank = drm_crtc_vblank_count(crtc);
			ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
					current_vblank != drm_crtc_vblank_count(crtc),
					usecs_to_jiffies(frame_time_us + 1000));
			if (!ret)
				pr_err("%s: crtc wait_event_timeout \n", __func__);
		}

	if (display->config.panel_mode == DSI_OP_VIDEO_MODE)
		panel->oplus_priv.skip_mipi_last_cmd = true;
		dsi_panel_seed_mode_unlock(panel, seed_mode);
		if (display->config.panel_mode == DSI_OP_VIDEO_MODE)
			panel->oplus_priv.skip_mipi_last_cmd = false;
		oplus_datadimming_v2_need_flush = false;
	}
}

int oplus_dc_panel_process_dimming_v2(struct dsi_panel *panel, int bl_lvl, bool force_disable)
{
	struct dsi_display *display = get_main_display();
	struct drm_connector *dsi_connector = display->drm_conn;
	bool need_sync = false;
	oplus_datadimming_v2_need_flush = false;
	oplus_datadimming_v2_need_sync = false;

	if (!force_disable && oplus_dimlayer_bl_enable_v2_real &&
		bl_lvl > 1 && bl_lvl < oplus_dc_dimlayer_bl_alpha_v2) {
		if (!oplus_dc_seed_backlight) {
			pr_err("Enter DC backlight v2\n");
			oplus_dc_seed_backlight = 1;
			lcdinfo_notify(LCM_DC_MODE_TYPE, &oplus_dc_seed_backlight);
			oplus_dc_v2_on = true;
			if (!oplus_skip_datadimming_sync && oplus_last_backlight != 0 && oplus_last_backlight != 1)
				need_sync = true;
		}
		oplus_dc_seed_backlight = bl_lvl;
		bl_lvl = oplus_dc_dimlayer_bl_alpha_v2;
		oplus_datadimming_v2_need_flush = true;
	} else if (oplus_dc_seed_backlight) {
		pr_err("Exit DC backlight v2\n");
		oplus_dc_v2_on = false;
		oplus_dc_seed_backlight = 0;
		oplus_dc2_alpha = 0;
		oplus_datadimming_v2_need_flush = true;
		need_sync = true;
		lcdinfo_notify(LCM_DC_MODE_TYPE, &oplus_dc_seed_backlight);
	}

	if(need_sync && (DSI_OP_CMD_MODE == display->config.panel_mode))
		oplus_datadimming_v2_need_sync = true;

	if (oplus_datadimming_v2_need_flush) {
		if (oplus_datadimming_v2_need_sync &&
			dsi_connector && dsi_connector->state && dsi_connector->state->crtc) {
			struct drm_crtc *crtc = dsi_connector->state->crtc;
			int frame_time_us, ret = 0;
			u32 current_vblank;

			frame_time_us = mult_frac(1000, 1000, panel->cur_mode->timing.refresh_rate);

			current_vblank = drm_crtc_vblank_count(crtc);
			ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
					current_vblank != drm_crtc_vblank_count(crtc),
					usecs_to_jiffies(frame_time_us + 1000));
			if (!ret)
				pr_err("%s: crtc wait_event_timeout \n", __func__);
		}
	}

	if (oplus_datadimming_v2_need_flush)
		oplus_dc_panel_process_dimming_v2_post(panel, force_disable);
	return bl_lvl;
}

int oplus_display_panel_get_dim_alpha(void *buf) {
	unsigned int* temp_alpha = buf;
	struct dsi_display *display = get_main_display();

	if (!display->panel->is_hbm_enabled ||
		(get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON)) {
		(*temp_alpha) = 0;
		return 0;
	}

	(*temp_alpha) = oplus_underbrightness_alpha;
	return 0;
}

int oplus_display_panel_set_dim_alpha(void *buf) {
	unsigned int* temp_alpha = buf;

	(*temp_alpha) = oplus_panel_alpha;

	return 0;
}

int oplus_display_panel_get_dim_dc_alpha(void *buf) {
	int ret = 0;
	unsigned int* temp_dim_alpha = buf;
	struct dsi_display *display = get_main_display();

	if (!display || !display->panel) {
		pr_err("%s main display is NULL\n", __func__);
		(*temp_dim_alpha) = 0;
		return 0;
	}

	if (display->panel->is_hbm_enabled ||
		get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) {
		ret = 0;
	}
	if (oplus_dc2_alpha != 0) {
		ret = oplus_dc2_alpha;
	} else if (oplus_underbrightness_alpha != 0) {
		ret = 0;
	} else if (oplus_dimlayer_bl_enable_v3_real) {
		ret = 1;
	}

	(*temp_dim_alpha) = ret;
	return 0;
}

int oplus_display_panel_set_dimlayer_enable(void *data)
{
	struct dsi_display *display = NULL;
	struct drm_connector *dsi_connector = NULL;
	uint32_t *dimlayer_enable = data;

	display = get_main_display();
	if (!display) {
		return -EINVAL;
	}

	dsi_connector = display->drm_conn;
	if (display && display->name) {
		int enable = (*dimlayer_enable);
		int err = 0;

		mutex_lock(&display->display_lock);
		if (!dsi_connector || !dsi_connector->state || !dsi_connector->state->crtc) {
			pr_err("[%s]: display not ready\n", __func__);
		} else {
			err = drm_crtc_vblank_get(dsi_connector->state->crtc);
			if (err) {
				pr_err("failed to get crtc vblank, error=%d\n", err);
			} else {
				/* do vblank put after 7 frames */
				oplus_datadimming_vblank_count = 7;
				atomic_inc(&oplus_datadimming_vblank_ref);
			}
		}

		usleep_range(17000, 17100);
		if (!strcmp(display->panel->oplus_priv.vendor_name, "ANA6706")) {
			oplus_dimlayer_bl_enable = enable;
		} else {
			if (!strcmp(display->name, "qcom,mdss_dsi_oplus19101boe_nt37800_1080_2400_cmd"))
				oplus_dimlayer_bl_enable_v3 = enable;
			else
				oplus_dimlayer_bl_enable_v2 = enable;
		}
		mutex_unlock(&display->display_lock);
	}

	return 0;
}

int oplus_display_panel_get_dimlayer_enable(void *data)
{
	uint32_t *dimlayer_bl_enable = data;

	(*dimlayer_bl_enable) = oplus_dimlayer_bl_enable_v2;

	return 0;
}

