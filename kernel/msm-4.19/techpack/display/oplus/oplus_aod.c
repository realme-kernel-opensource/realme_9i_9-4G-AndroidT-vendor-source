/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_aod.c
** Description : oplus aod feature
** Version : 1.0
** Date : 2020/04/23
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Qianxu         2020/04/23        1.0           Build this moudle
******************************************************************/

#include "dsi_defs.h"
#include "oplus_aod.h"
#include "oplus_display_private_api.h"
#include "oplus_display_panel.h"

int aod_light_mode = 0;
DEFINE_MUTEX(oplus_aod_light_mode_lock);

int __oplus_display_set_aod_light_mode(int mode) {
	mutex_lock(&oplus_aod_light_mode_lock);
	if(mode != aod_light_mode) {
		aod_light_mode = mode;
	}
	mutex_unlock(&oplus_aod_light_mode_lock);
	return 0;
}

int oplus_update_aod_light_mode_unlock(struct dsi_panel *panel)
{
	int rc = 0;
	enum dsi_cmd_set_type type;

	if (aod_light_mode == 1)
		type = DSI_CMD_AOD_LOW_LIGHT_MODE;
	else
		type = DSI_CMD_AOD_HIGH_LIGHT_MODE;

	rc = dsi_panel_tx_cmd_set(panel, type);
	if (rc) {
		pr_err("[%s] failed to send DSI_CMD_AOD_LIGHT_MODE cmds, rc=%d\n",
		       panel->name, rc);
	}

	return rc;
}

#ifdef OPLUS_FEATURE_AOD_RAMLESS
extern bool is_oplus_display_aod_mode(void);
#endif /* OPLUS_FEATURE_AOD_RAMLESS */
int oplus_update_aod_light_mode(void)
{
	struct dsi_display *display = get_main_display();
	int ret = 0;

	if (!display || !display->panel) {
		printk(KERN_INFO "oplus_set_aod_light_mode and main display is null");
		return -EINVAL;
	}

	if (display->panel->is_hbm_enabled) {
		pr_err("%s error panel->is_hbm_enabled\n", __func__);
		return -EINVAL;
	}

	if (get_oplus_display_scene() != OPLUS_DISPLAY_AOD_SCENE) {
		pr_err("%s error get_oplus_display_scene = %d, \n", __func__, get_oplus_display_scene());
		return -EFAULT;
	}
	mutex_lock(&display->display_lock);
	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
	}

	mutex_lock(&display->panel->panel_lock);
#ifdef OPLUS_FEATURE_AOD_RAMLESS
	if (display->panel->oplus_priv.is_aod_ramless &&
		!is_oplus_display_aod_mode()) {
		pr_err("not support update aod_light_mode at non-aod mode\n");
		ret = -EINVAL;
		goto error;
	}
#endif /* OPLUS_FEATURE_AOD_RAMLESS */

	if (!dsi_panel_initialized(display->panel)) {
		pr_err("dsi_panel_aod_low_light_mode is not init\n");
		ret = -EINVAL;
		goto error;
	}

	ret = oplus_update_aod_light_mode_unlock(display->panel);

	if (ret) {
		pr_err("failed to set aod light status ret=%d", ret);
		goto error;
	}

error:
	mutex_unlock(&display->panel->panel_lock);
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_OFF);
	}
	mutex_unlock(&display->display_lock);

	return ret;
}

int oplus_panel_set_aod_light_mode(void *buf)
{
	unsigned int *temp_save = buf;

	__oplus_display_set_aod_light_mode(*temp_save);
	oplus_update_aod_light_mode();

	return 0;
}

int oplus_panel_get_aod_light_mode(void *buf)
{
	unsigned int *aod_mode = buf;
	(*aod_mode) = aod_light_mode;

	printk(KERN_INFO "oplus_get_aod_light_mode = %d\n",aod_light_mode);

	return 0;
}

#ifdef OPLUS_FEATURE_AOD_RAMLESS
extern struct aod_area oplus_aod_area[RAMLESS_AOD_AREA_NUM];

int oplus_display_panel_set_aod_area(void *buf)
{
	struct dsi_display *display = get_main_display();
	struct panel_aod_area_para *para = (struct panel_aod_area_para *)buf;
	int i, cnt = 0;
	//char payload[RAMLESS_AOD_PAYLOAD_SIZE];

	if (!display || !display->panel || !display->panel->oplus_priv.is_aod_ramless) {
		pr_err("failed to find dsi display or is not ramless\n");
		return false;
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	memset(oplus_aod_area, 0, sizeof(struct aod_area) * RAMLESS_AOD_AREA_NUM);
	if (para->size > RAMLESS_AOD_AREA_NUM) {
		pr_err("aod area size is invalid, size=%d\n", para->size);
		return -1;
	}
	pr_info("%s %d para->size = %d\n", __func__, __LINE__, para->size);
	for (i = 0; i < para->size; i++) {
		struct aod_area *area = &oplus_aod_area[cnt];

		area->x = para->aod_area[i].x;
		area->y = para->aod_area[i].y;
		area->w = para->aod_area[i].w;
		area->h = para->aod_area[i].h;
		area->color = para->aod_area[i].color;
		area->bitdepth = para->aod_area[i].bitdepth;
		area->mono = para->aod_area[i].mono;
		area->gray = para->aod_area[i].gray;
		pr_info("%s %d rect[%dx%d-%dx%d]-%d-%d-%d-%x\n", __func__, __LINE__,
			area->x, area->y, area->w, area->h,
			area->color, area->bitdepth, area->mono, area->gray);
		area->enable = true;
		cnt++;
	}
	oplus_display_update_aod_area_unlock();
	mutex_unlock(&display->panel->panel_lock);
	mutex_unlock(&display->display_lock);

	return 0;
}


int oplus_display_panel_get_video(void *buf)
{
	struct dsi_display *display = get_main_display();
	bool is_aod_ramless = false;
	int *temp_save = buf;

	if (display && display->panel && display->panel->oplus_priv.is_aod_ramless)
		is_aod_ramless = true;
	(*temp_save) = is_aod_ramless ? 1 : 0;

	return 0;
}

extern int oplus_display_mode;
extern int oplus_onscreenfp_status;
static DECLARE_WAIT_QUEUE_HEAD(oplus_aod_wait);

int oplus_display_panel_set_video(void *buf)
{
	struct dsi_display *display = get_main_display();
	struct drm_device *drm_dev;
	struct drm_connector *dsi_connector;
	struct drm_mode_config *mode_config;
	struct msm_drm_private *priv;
	struct drm_display_mode *mode;
	struct drm_atomic_state *state;
	struct drm_crtc_state *crtc_state;
	struct drm_crtc *crtc;
	bool mode_changed = false;
	int *temp_save = buf;
	int mode_id = 0;
	int vblank_get = -EINVAL;
	int err = 0;
	int i;

	if (!display || !display->panel) {
		pr_err("failed to find dsi display\n");
		return -EINVAL;
	}

	drm_dev = display->drm_dev;
	dsi_connector = display->drm_conn;
	mode_config = &drm_dev->mode_config;
	priv = drm_dev->dev_private;


	if (!display->panel->oplus_priv.is_aod_ramless)
		return -EINVAL;

	if (!dsi_connector || !dsi_connector->state || !dsi_connector->state->crtc) {
		pr_err("[%s]: display not ready\n", __func__);
		return -EINVAL;
	}
	mode_id = *temp_save;
	pr_err("setting display mode %d\n", mode_id);

	vblank_get = drm_crtc_vblank_get(dsi_connector->state->crtc);
	if (vblank_get) {
		pr_err("failed to get crtc vblank\n", vblank_get);
	}

	drm_modeset_lock_all(drm_dev);

	if (oplus_display_mode != 1)
		display->panel->dyn_clk_caps.dyn_clk_support = false;

	state = drm_atomic_state_alloc(drm_dev);
	if (!state)
		goto error;

	oplus_display_mode = mode_id;
	state->acquire_ctx = mode_config->acquire_ctx;
	crtc = dsi_connector->state->crtc;
	crtc_state = drm_atomic_get_crtc_state(state, crtc);

	 {
		struct drm_display_mode *set_mode = NULL;
		struct drm_display_mode *cmd_mode = NULL;
		struct drm_display_mode *vid_mode = NULL;

		list_for_each_entry(mode, &dsi_connector->modes, head) {
			if (drm_mode_vrefresh(mode) == 0)
				continue;
			if (mode->flags & DRM_MODE_FLAG_VID_MODE_PANEL)
				vid_mode = mode;
			if (mode->flags & DRM_MODE_FLAG_CMD_MODE_PANEL)
				cmd_mode = mode;
		}

		set_mode = oplus_display_mode ? vid_mode : cmd_mode;
		set_mode = oplus_onscreenfp_status ? vid_mode : set_mode;

		if (set_mode && drm_mode_vrefresh(set_mode) != drm_mode_vrefresh(&crtc_state->mode)) {
			mode_changed = true;
		} else {
			mode_changed = false;
		}

		if (mode_changed) {
			for (i = 0; i < priv->num_crtcs; i++) {
				if (priv->disp_thread[i].crtc_id == crtc->base.id) {
					if (priv->disp_thread[i].thread)
						kthread_flush_worker(&priv->disp_thread[i].worker);
				}
			}

			display->panel->dyn_clk_caps.dyn_clk_support = false;
			drm_atomic_set_mode_for_crtc(crtc_state, set_mode);
		}
		wake_up(&oplus_aod_wait);
	}
	err = drm_atomic_commit(state);
	drm_atomic_state_put(state);

	if (mode_changed) {
		for (i = 0; i < priv->num_crtcs; i++) {
			if (priv->disp_thread[i].crtc_id == crtc->base.id) {
				if (priv->disp_thread[i].thread)
					kthread_flush_worker(&priv->disp_thread[i].worker);
			}
		}
	}

	if (oplus_display_mode == 1)
		display->panel->dyn_clk_caps.dyn_clk_support = true;

error:
	drm_modeset_unlock_all(drm_dev);
	if (!vblank_get)
		drm_crtc_vblank_put(dsi_connector->state->crtc);

	return 0;
}
#endif /* OPLUS_FEATURE_AOD_RAMLESS */
