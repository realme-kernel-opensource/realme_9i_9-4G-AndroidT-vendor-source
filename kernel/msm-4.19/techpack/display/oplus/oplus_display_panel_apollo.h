/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_panel_apollo.h
** Description : oplus display panel apollo feature
** Version : 1.0
** Date : 2022/01/27
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Tong YuRuan    2022/01/27        1.0           Build this feature
******************************************************************/
#ifndef _OPLUS_DISPLAY_PANEL_APOLLO_H_
#define _OPLUS_DISPLAY_PANEL_APOLLO_H_

#include "sde_trace.h"
#include <linux/ktime.h>
#include "dsi_panel.h"
#include "dsi_display.h"
#include "dsi_drm.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include <drm/drm_connector.h>
#include <drm/drm_encoder.h>
#include "msm_drv.h"
#include "sde_crtc.h"
#include <linux/sde_rsc.h>
#include "sde_encoder_phys.h"

#define to_sde_encoder_phys_cmd(x) \
	container_of(x, struct sde_encoder_phys_cmd, base)
/**
 * Add for backlight smooths
 * @g_pri_bk_level: global backlight of the primary screen
 * @g_sec_bk_level: global backlight of the secondary screen
 * @g_save_pcc: global pcc save for debug
 */
struct oplus_apollo_bk {
	u32 g_pri_bk_level;
	u32 g_sec_bk_level;
};
enum oplus_sync_method {
	OPLUS_PREPARE_KICKOFF_METHOD = 0,
	OPLUS_KICKOFF_METHOD,
	OPLUS_POST_KICKOFF_METHOD,
	OPLUS_WAIT_VSYNC_METHOD,
	OPLUS_UNKNOW_METHOD,
};

int dsi_panel_parse_oplus_apollo_config(struct dsi_panel *panel);
bool is_spread_backlight(struct dsi_display *display, int level);
void update_pending_backlight(struct dsi_display *display, int level);
int sde_backlight_device_update_apollo_status(int brightness, struct dsi_display *display, \
			struct sde_connector *c_conn, int bl_lvl, struct drm_event event);
int get_current_display_framerate(struct drm_connector *connector);
int get_current_display_brightness(struct drm_connector *connector);
bool is_support_apollo_bk(struct drm_connector *connector);
bool is_spread_backlight(struct dsi_display *display, int level);
void update_pending_backlight(struct dsi_display *display, int level);
bool sde_encoder_is_disabled(struct drm_encoder *drm_enc);
int oplus_backlight_wait_vsync(struct drm_encoder *drm_enc);
int oplus_setbacklight_by_display_type(struct drm_encoder *drm_enc);
int oplus_sync_panel_brightness(enum oplus_sync_method method, struct drm_encoder *drm_enc);
void sde_encoder_apollo_kickoff(enum oplus_sync_method method, struct drm_encoder *drm_enc);
#endif /*_OPLUS_DISPLAY_PANEL_APOLLO_H_*/
