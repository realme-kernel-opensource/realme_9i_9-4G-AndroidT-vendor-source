/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_panel_common.c
** Description : oplus display panel common feature
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_PANEL_COMMON_H_
#define _OPLUS_DISPLAY_PANEL_COMMON_H_

#include <linux/err.h>
#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_ctrl.h"
#include "dsi_ctrl_hw.h"
#include "dsi_drm.h"
#include "dsi_clk.h"
#include "dsi_pwr.h"
#include "sde_dbg.h"

#define PANEL_REG_MAX_LENS 28
#define PANEL_TX_MAX_BUF 112


/**
 * enum oplus_debug_log --       flags to control debug log; 1->enbale  0->disable
 * @OPLUS_DEBUG_LOG_DISABLED:    disable all debug log
 * @OPLUS_DEBUG_LOG_CMD:         dump register log
 * @OPLUS_DEBUG_LOG_BACKLIGHT:   backlight log
 */
enum oplus_debug_log {
	OPLUS_DEBUG_LOG_DISABLED     = 0,
	OPLUS_DEBUG_LOG_CMD          = BIT(0),
	OPLUS_DEBUG_LOG_BACKLIGHT    = BIT(1),
	OPLUS_DEBUG_LOG_COMMON       = BIT(2),
};

extern int oplus_dsi_log_type;
#define LOG_KEYWORD "[SDE]"

#define LCD_DEBUG_BACKLIGHT(X, ...)	\
		do {	\
			if (oplus_dsi_log_type & OPLUS_DEBUG_LOG_BACKLIGHT)	\
				pr_info("%s %s : "X, LOG_KEYWORD, __func__, ## __VA_ARGS__);\
			else	\
				pr_debug("%s %s : "X, LOG_KEYWORD, __func__, ## __VA_ARGS__);\
		} while (0)	\

#define LCD_DEBUG_COMMON(X, ...)	\
		do {	\
			if (oplus_dsi_log_type & OPLUS_DEBUG_LOG_COMMON) \
				pr_info("%s %s : "X, LOG_KEYWORD, __func__, ## __VA_ARGS__);\
			else	\
				pr_debug("%s %s : "X, LOG_KEYWORD, __func__, ## __VA_ARGS__);\
		} while (0) \

#define LCD_INFO(X, ...) pr_info("%s %s : "X, LOG_KEYWORD, __func__, ## __VA_ARGS__)
#define LCD_INFO_ONCE(X, ...) pr_info_once("%s %s : "X, LOG_KEYWORD, __func__, ## __VA_ARGS__)
#define LCD_ERR(X, ...) pr_err("%s %s : "X, LOG_KEYWORD, __func__, ## __VA_ARGS__)

struct panel_id
{
	uint32_t DA;
	uint32_t DB;
	uint32_t DC;
};

struct panel_info{
	char version[32];
	char manufacture[32];
};

struct panel_serial_number {
    char serial_number[40];
};

struct display_timing_info {
	uint32_t h_active;
	uint32_t v_active;
	uint32_t refresh_rate;
	uint32_t clk_rate_hz_h32;  /* the high 32bit of clk_rate_hz */
	uint32_t clk_rate_hz_l32;  /* the low 32bit of clk_rate_hz */
};

enum {
	NONE_TYPE = 0,
	LCM_DC_MODE_TYPE,
	LCM_BRIGHTNESS_TYPE,
	MAX_INFO_TYPE,
};

struct panel_reg_get {
	uint32_t reg_rw[PANEL_REG_MAX_LENS];
	uint32_t lens; /*reg_rw lens, lens represent for u32 to user space*/
};

struct panel_reg_rw {
	uint32_t rw_flags; /*1 for read, 0 for write*/
	uint32_t cmd;
	uint32_t lens;     /*lens represent for u8 to kernel space*/
	uint32_t value[PANEL_REG_MAX_LENS]; /*for read, value is empty, just user get function for read the value*/
};

int oplus_display_panel_get_id(void *buf);
int oplus_display_panel_get_max_brightness(void *buf);
int oplus_display_panel_set_max_brightness(void *buf);
int oplus_display_panel_get_brightness(void *buf);
int oplus_display_panel_get_vendor(void *buf);
int oplus_display_panel_get_ccd_check(void *buf);
int oplus_display_panel_get_serial_number(void *buf);
int oplus_display_panel_set_audio_ready(void *data);
int oplus_display_panel_dump_info(void *data);
int oplus_display_panel_get_dsc(void *data);
int oplus_display_panel_get_closebl_flag(void *data);
int oplus_display_panel_set_closebl_flag(void *data);
int oplus_display_panel_get_reg(void *data);
int oplus_display_panel_set_reg(void *data);
int oplus_display_panel_notify_blank(void *data);
int oplus_display_panel_set_spr(void *data);
int oplus_display_panel_get_spr(void *data);
int oplus_display_panel_get_roundcorner(void *data);
int oplus_display_panel_set_dynamic_osc_clock(void *data);
int oplus_display_panel_get_dynamic_osc_clock(void *data);
int __oplus_display_set_mca(int mode);
int dsi_display_set_mca(struct dsi_display *display);
int oplus_display_get_softiris_color_status(void *data);
int oplus_display_panel_set_dsi_log_switch(void *data);
int oplus_display_panel_get_dsi_log_switch(void *data);
int oplus_display_set_backlight(struct dsi_display *display, u32 bl_lvl);
int oplus_panel_update_backlight(struct dsi_panel *panel, u32 bl_lvl);
int oplus_display_panel_switch_gamma_mode(struct dsi_panel *panel, u32 bl_lvl);
int oplus_display_set_pixel_off(struct dsi_panel *panel);
void dsi_oplus_panel_parse_gpios(struct dsi_panel *panel);
int oplus_display_panel_set_framedone_idle(void *data);

#endif /*_OPLUS_DISPLAY_PANEL_COMMON_H_*/
