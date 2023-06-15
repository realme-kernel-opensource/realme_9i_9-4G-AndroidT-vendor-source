/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_panel_apollo.c
** Description : oplus display panel apollo feature
** Version : 1.0
** Date : 2022/01/27
**
** ------------------------------- Revision History: -----------
**  <author>		<data>		<version >		<desc>
**  Tong YuRuan	  2022/01/27	  1.0		Build this feature
******************************************************************/
#include "oplus_display_panel_apollo.h"

#define NUM_PHYS_ENCODER_APOLLO_TYPES 2
#define MAX_PHYS_ENCODERS_PER_APOLLO_VIRTUAL \
	(MAX_H_TILES_PER_DISPLAY * NUM_PHYS_ENCODER_APOLLO_TYPES)

struct oplus_apollo_bk apollo_bk;
static DEFINE_SPINLOCK(g_bk_lock);
u32 g_oplus_save_pcc = 0;
bool apollo_backlight_enable = false;
int backlight_smooth_enable = 1;
EXPORT_SYMBOL(backlight_smooth_enable);
#define MSM_BOOT_MODE_FACTORY 3
extern int get_boot_mode(void);

enum sde_enc_rc_states {
	SDE_ENC_RC_STATE_OFF,
	SDE_ENC_RC_STATE_PRE_OFF,
	SDE_ENC_RC_STATE_ON,
	SDE_ENC_RC_STATE_MODESET,
	SDE_ENC_RC_STATE_IDLE
};

struct sde_encoder_virt {
	struct drm_encoder base;
	spinlock_t enc_spinlock;
	struct mutex vblank_ctl_lock;
	uint32_t bus_scaling_client;

	uint32_t display_num_of_h_tiles;
	uint32_t te_source;

	struct sde_encoder_ops ops;

	unsigned int num_phys_encs;
	struct sde_encoder_phys *phys_encs[MAX_PHYS_ENCODERS_PER_APOLLO_VIRTUAL];
	struct sde_encoder_phys *phys_vid_encs[MAX_PHYS_ENCODERS_PER_APOLLO_VIRTUAL];
	struct sde_encoder_phys *phys_cmd_encs[MAX_PHYS_ENCODERS_PER_APOLLO_VIRTUAL];
	struct sde_encoder_phys *cur_master;
	struct sde_hw_pingpong *hw_pp[MAX_CHANNELS_PER_ENC];
	struct sde_hw_dsc *hw_dsc[MAX_CHANNELS_PER_ENC];
	struct sde_hw_pingpong *hw_dsc_pp[MAX_CHANNELS_PER_ENC];
	enum sde_dsc dirty_dsc_ids[MAX_CHANNELS_PER_ENC];

	bool intfs_swapped;
	bool qdss_status;

	void (*crtc_vblank_cb)(void *data);
	void *crtc_vblank_cb_data;

	struct dentry *debugfs_root;
	struct mutex enc_lock;
	atomic_t frame_done_cnt[MAX_PHYS_ENCODERS_PER_APOLLO_VIRTUAL];
	void (*crtc_frame_event_cb)(void *data, u32 event);
	struct sde_crtc_frame_event_cb_data crtc_frame_event_cb_data;

	struct timer_list vsync_event_timer;

	struct sde_rsc_client *rsc_client;
	bool rsc_state_init;
	struct msm_display_info disp_info;
	bool misr_enable;
	u32 misr_frame_count;

	bool idle_pc_enabled;
	struct mutex rc_lock;
	enum sde_enc_rc_states rc_state;
	struct kthread_delayed_work delayed_off_work;
	struct kthread_work vsync_event_work;
	struct kthread_work input_event_work;
	struct kthread_work esd_trigger_work;
	struct input_handler *input_handler;
	struct msm_display_topology topology;
	bool vblank_enabled;
	bool idle_pc_restore;
	enum frame_trigger_mode_type frame_trigger_mode;
	bool dynamic_hdr_updated;

	struct sde_rsc_cmd_config rsc_config;
	struct sde_rect cur_conn_roi;
	struct sde_rect prv_conn_roi;
	struct drm_crtc *crtc;

	bool recovery_events_enabled;
	bool elevated_ahb_vote;
	struct pm_qos_request pm_qos_cpu_req;
	struct msm_mode_info mode_info;
};



/*
 * init apollo function ,
 * if apollo_backlight_enable ture, open the function
 * if apollo_backlight_enable false, without apollo
*/
int dsi_panel_parse_oplus_apollo_config(struct dsi_panel *panel) {
	int ret = 0;
	struct dsi_parser_utils *utils = &panel->utils;

	/* Add for apollo */
	panel->oplus_priv.is_apollo_support = utils->read_bool(utils->data, "oplus,apollo_backlight_enable");
	apollo_backlight_enable = panel->oplus_priv.is_apollo_support;

	if (panel->oplus_priv.is_apollo_support) {
		ret = utils->read_u32(utils->data, "oplus,apollo-sync-brightness-level",
				&panel->oplus_priv.sync_brightness_level);
		if (ret) {
			pr_info("[%s] failed to get panel parameter: oplus,apollo-sync-brightness-level\n", __func__);
			/* Default sync brightness level is set to 200 */
			panel->oplus_priv.sync_brightness_level = 200;
		}
	}
	return ret;
}
EXPORT_SYMBOL(dsi_panel_parse_oplus_apollo_config);

/*
 * Add for apollo to cache brightness
 * call by sde_backlight_device_update_status
 *
*/
int sde_backlight_device_update_apollo_status(int brightness, struct dsi_display *display,
			struct sde_connector *c_conn, int bl_lvl, struct drm_event event) {
	int rc = 0;

	if (c_conn->ops.set_backlight) {
		/* skip notifying user space if bl is 0 */
		if (brightness != 0) {
			event.type = DRM_EVENT_SYS_BACKLIGHT;
			event.length = sizeof(u32);
			msm_mode_object_event_notify(&c_conn->base.base,
				c_conn->base.dev, &event, (u8 *)&brightness);
		}

		if (display->panel->oplus_priv.is_apollo_support && backlight_smooth_enable) {
			if ((MSM_BOOT_MODE_FACTORY != get_boot_mode()) && (is_spread_backlight(display, bl_lvl))) {
				spin_lock(&g_bk_lock);
				update_pending_backlight(display, bl_lvl);
				spin_unlock(&g_bk_lock);
			} else {
				spin_lock(&g_bk_lock);
				update_pending_backlight(display, bl_lvl);
				spin_unlock(&g_bk_lock);
				rc = c_conn->ops.set_backlight(&c_conn->base,
				c_conn->display, bl_lvl);
				c_conn->unset_bl_level = 0;
			}
		} else {
			rc = c_conn->ops.set_backlight(&c_conn->base,
			c_conn->display, bl_lvl);
			c_conn->unset_bl_level = 0;
		}
	}
	return rc;
}
EXPORT_SYMBOL(sde_backlight_device_update_apollo_status);

int get_current_display_framerate(struct drm_connector *connector)
{
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct dsi_display *dsi_display = NULL;
	int framerate = 0;

	dsi_display = c_conn->display;

	if (!dsi_display || !dsi_display->panel || !dsi_display->panel->cur_mode) {
		SDE_ERROR("Invalid params(s) dsi_display %pK, panel %pK\n",
			dsi_display,
			((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	framerate = dsi_display->panel->cur_mode->timing.refresh_rate;

	return framerate;
}

int get_current_display_brightness(struct drm_connector *connector)
{
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct dsi_display *dsi_display = NULL;
	int brightness_level = 0;

	dsi_display = c_conn->display;

	if (!dsi_display || !dsi_display->panel || !dsi_display->panel->cur_mode) {
		SDE_ERROR("Invalid params(s) dsi_display %pK, panel %pK\n",
			dsi_display,
			((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	brightness_level = dsi_display->panel->bl_config.bl_level;

	return brightness_level;
}

bool is_support_apollo_bk(struct drm_connector *connector)
{
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct dsi_display *dsi_display = NULL;

	dsi_display = c_conn->display;

	if (!dsi_display || !dsi_display->panel || !dsi_display->panel->oplus_priv.vendor_name) {
		SDE_ERROR("Invalid params(s) dsi_display %pK, panel %pK\n",
			dsi_display,
			((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	if(c_conn->connector_type == DRM_MODE_CONNECTOR_DSI) {
		if (dsi_display->panel->oplus_priv.is_apollo_support) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}
EXPORT_SYMBOL(is_support_apollo_bk);

bool is_spread_backlight(struct dsi_display *display, int level)
{
	if ((display != NULL) && (display->panel != NULL)
		&& (level <= display->panel->oplus_priv.sync_brightness_level) && (level >= 2)) {
		return true;
	} else {
		return false;
	}
}

void update_pending_backlight(struct dsi_display *display, int level) {
	if (display == NULL) {
		return;
	}

	if (!strcmp(display->display_type, "primary")) {
		apollo_bk.g_pri_bk_level = level;
	} else if (!strcmp(display->display_type, "secondary")) {
		apollo_bk.g_sec_bk_level = level;
	} else {
		/*Ignore*/
	}
}

bool sde_encoder_is_disabled(struct drm_encoder *drm_enc)
{
	struct sde_encoder_virt *sde_enc;
	struct sde_encoder_phys *phys;

	sde_enc = container_of(drm_enc, struct sde_encoder_virt, base);
	phys = sde_enc->phys_encs[0];
	return (phys->enable_state == SDE_ENC_DISABLED);
}

int oplus_backlight_wait_vsync(struct drm_encoder *drm_enc)
{
	SDE_ATRACE_BEGIN("wait_vsync");

	if (!drm_enc || !drm_enc->crtc) {
		SDE_ERROR("%s encoder is disabled", __func__);
		return -ENOLINK;
	}

	if (sde_encoder_is_disabled(drm_enc)) {
		SDE_ERROR("%s encoder is disabled", __func__);
		return -EIO;
	}

	/*mutex_unlock(&panel->panel_lock);*/
	sde_encoder_wait_for_event(drm_enc,  MSM_ENC_VBLANK);
	/*mutex_lock(&panel->panel_lock);*/
	SDE_ATRACE_END("wait_vsync");

	return 0;
}

int oplus_setbacklight_by_display_type(struct drm_encoder *drm_enc) {
	struct sde_encoder_virt *sde_enc = NULL;
	struct dsi_display *display = NULL;
	struct sde_connector *c_conn = NULL;
	int rc = 0;
	char tag_name[64];

	sde_enc = container_of(drm_enc, struct sde_encoder_virt, base);
	c_conn = to_sde_connector(sde_enc->phys_encs[0]->connector);

	if (sde_enc == NULL)
		return -EFAULT;
	if (c_conn == NULL)
		return -EFAULT;
	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	display = c_conn->display;
	if (display == NULL)
		return -EFAULT;

	if (!strcmp(display->display_type, "primary")) {
		snprintf(tag_name, sizeof(tag_name), "primary: %d", apollo_bk.g_pri_bk_level);
		SDE_ATRACE_BEGIN(tag_name);
		rc = c_conn->ops.set_backlight(&c_conn->base, display, apollo_bk.g_pri_bk_level);
		SDE_ATRACE_END(tag_name);
	} else if (!strcmp(display->display_type, "secondary")) {
		snprintf(tag_name, sizeof(tag_name), "secondary: %d", apollo_bk.g_sec_bk_level);
		SDE_ATRACE_BEGIN(tag_name);
		rc = c_conn->ops.set_backlight(&c_conn->base, display, apollo_bk.g_sec_bk_level);
		SDE_ATRACE_END(tag_name);
	}

	return rc;
}

void sde_encoder_apollo_kickoff(enum oplus_sync_method method, struct drm_encoder *drm_enc)
{
	struct sde_encoder_virt *sde_enc = NULL;
	sde_enc = container_of(drm_enc, struct sde_encoder_virt, base);
	if (sde_enc == NULL)
		return;
	if ((is_support_apollo_bk(sde_enc->cur_master->connector) == true) && backlight_smooth_enable) {
		if (sde_enc->num_phys_encs > 0) {
			oplus_sync_panel_brightness(OPLUS_POST_KICKOFF_METHOD, drm_enc);
		}
	}
}

int oplus_sync_panel_brightness(enum oplus_sync_method method, struct drm_encoder *drm_enc)
{
	struct sde_encoder_virt *sde_enc = NULL;
	struct sde_encoder_phys *phys_encoder = NULL;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	int rc = 0;
	struct sde_encoder_phys_cmd *cmd_enc = NULL;
	struct sde_encoder_phys_cmd_te_timestamp *te_timestamp;
	s64 us_per_frame;
	ktime_t last_te_timestamp;
	s64 delay;

	sde_enc = container_of(drm_enc, struct sde_encoder_virt, base);
	phys_encoder = sde_enc->phys_encs[0];

	if (phys_encoder == NULL)
		return -EFAULT;
	if (phys_encoder->connector == NULL)
		return -EFAULT;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return -EFAULT;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	display = c_conn->display;
	if (display == NULL)
		return -EFAULT;

	cmd_enc = to_sde_encoder_phys_cmd(phys_encoder);
	if (cmd_enc == NULL) {
		return -EFAULT;
	}

	us_per_frame = 1000000 / get_current_display_framerate(sde_enc->cur_master->connector);
	te_timestamp = list_last_entry(&cmd_enc->te_timestamp_list, struct sde_encoder_phys_cmd_te_timestamp, list);
	last_te_timestamp = te_timestamp->timestamp;
	if (apollo_bk.g_pri_bk_level == 2047)
		pr_err("light: oplus_sync_panel_brightness 1 %s, get_current_display_brightness =%d \n",
				display->display_type, get_current_display_brightness(sde_enc->cur_master->connector));
	if ((!strcmp(display->display_type, "primary") &&
			is_spread_backlight(display, apollo_bk.g_pri_bk_level) &&
			(apollo_bk.g_pri_bk_level != get_current_display_brightness(sde_enc->cur_master->connector))) ||
		(!strcmp(display->display_type, "secondary") &&
			is_spread_backlight(display, apollo_bk.g_sec_bk_level) &&
			(apollo_bk.g_sec_bk_level != get_current_display_brightness(sde_enc->cur_master->connector)))) {
		SDE_ATRACE_BEGIN("sync_panel_brightness");
		delay = (us_per_frame >> 1) - (ktime_to_us(ktime_sub(ktime_get(), last_te_timestamp)) % us_per_frame);
		if (delay > 0) {
			usleep_range(delay, delay + 100);
		}
		pr_debug("us_per_frame = %lld, delta = %lld, delay = %lld",	\
				us_per_frame, ktime_to_us(ktime_sub(ktime_get(), last_te_timestamp)), delay);
		if ((ktime_to_us(ktime_sub(ktime_get(), last_te_timestamp)) % us_per_frame) > (us_per_frame - 500)) {
			usleep_range(500, 1000);
		}

		if (method == OPLUS_PREPARE_KICKOFF_METHOD) {
			rc = oplus_setbacklight_by_display_type(drm_enc);
			c_conn->unset_bl_level = 0;
		} else if (method == OPLUS_POST_KICKOFF_METHOD) {
			rc = oplus_setbacklight_by_display_type(drm_enc);
			c_conn->unset_bl_level = 0;
		} else {
			oplus_backlight_wait_vsync(c_conn->encoder);
			rc = oplus_setbacklight_by_display_type(drm_enc);
			c_conn->unset_bl_level = 0;
		}
		SDE_ATRACE_END("sync_panel_brightness");
	}
	return rc;
}
EXPORT_SYMBOL(oplus_sync_panel_brightness);
