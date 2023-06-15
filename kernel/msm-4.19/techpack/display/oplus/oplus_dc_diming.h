/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_dc_diming.h
** Description : oplus dc_diming feature
** Version : 1.0
** Date : 2020/04/15
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Qianxu         2020/04/15        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DC_DIMING_H_
#define _OPLUS_DC_DIMING_H_

#include <drm/drm_connector.h>

#include "dsi_panel.h"
#include "dsi_defs.h"
#include "oplus_display_panel_hbm.h"

int sde_connector_update_backlight(struct drm_connector *connector, bool post);

int sde_connector_update_hbm(struct drm_connector *connector);

int oplus_dc_seed_bright_to_alpha(int brightness);

void oplus_dc_dsi_update_seed_backlight(struct dsi_panel *panel, int seed_value,
			enum dsi_cmd_set_type type, struct dsi_cmd_desc **cmds, u32 *count, enum dsi_cmd_set_state *state);
int oplus_display_panel_get_dim_alpha(void *buf);
int oplus_display_panel_set_dim_alpha(void *buf);
int oplus_display_panel_get_dim_dc_alpha(void *buf);
int oplus_display_panel_get_dimlayer_enable(void *data);
int oplus_display_panel_set_dimlayer_enable(void *data);
int dsi_panel_parse_oplus_dc_config(struct dsi_panel *panel);
void oplus_dc_panel_process_dimming_v2_post(struct dsi_panel *panel, bool force_disable);
int oplus_dc_panel_process_dimming_v2(struct dsi_panel *panel, int bl_lvl, bool force_disable);
#endif /*_OPLUS_DC_DIMING_H_*/
