/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_panel_power.h
** Description : oplus display panel power control
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_PANEL_POWER_H_
#define _OPLUS_DISPLAY_PANEL_POWER_H_

#include <linux/err.h>
#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_ctrl.h"
#include "dsi_ctrl_hw.h"
#include "dsi_drm.h"
#include "dsi_clk.h"
#include "dsi_pwr.h"
#include "sde_dbg.h"

#define PANEL_VOLTAGE_VALUE_COUNT 4
#define POWER_OPER_DELAY_SIZE	(4)

struct panel_vol_set{
	uint32_t panel_id;
	uint32_t panel_vol;
};

struct panel_vol_get{
	uint32_t panel_id;
	uint32_t panel_min;
	uint32_t panel_cur;
	uint32_t panel_max;
};

enum PANEL_VOLTAGE_ENUM{
	PANEL_VOLTAGE_ID_VDDI = 0,
	PANEL_VOLTAGE_ID_VDDR,
	PANEL_VOLTAGE_ID_VG_BASE,
	PANEL_VOLTAGE_ID_MAX,
};

typedef enum SET_POWER_MODE {
	SET_PWR_OFF = 0,
	SET_PWR_ON = 1,
}SET_POWER_MODE;

typedef enum POWER_OPERATION_NUM {
	POWER_SET_VCI = 0,
	POWER_SET_VDDIO,
	POWER_SET_RESET,
	POWER_SET_BIAS,
	POWER_SET_REGULATOR,
	POWER_SET_PINCTRL_STATE,
	POWER_TP_LOAD_FW,
	POWER_SET_CMD,
	POWER_SET_RESET_SEL,
	POWER_SET_END = 0xF,
}POWER_OPERATION_NUM;

typedef struct panel_power_operation {
	char *operation_name;
	POWER_OPERATION_NUM operation_num;
}PANEL_POWER_OPERATION;

typedef struct panel_voltage_bak {
	u32 voltage_id;
	u32 voltage_min;
	u32 voltage_current;
	u32 voltage_max;
	char pwr_name[20];
}PANEL_VOLTAGE_BAK;

extern int g_tp_gesture_enable_flag;
extern bool g_oplus_display_debug_switch;

int oplus_display_panel_set_pwr(void *data);
int oplus_display_panel_get_pwr(void *data);
int oplus_display_panel_get_power_status(void *data);
int oplus_display_panel_set_power_status(void *data);
int oplus_display_panel_regulator_control(void *data);
int __oplus_display_set_power_status(int status);

void __attribute__((weak)) lcd_queue_load_tp_fw(void);
int __attribute__((weak)) tp_gesture_enable_flag(void);

inline void oplus_mdelay(unsigned long msec);
inline void oplus_udelay(unsigned long usec);
void oplus_get_panel_power_timing_parse(struct dsi_panel *panel);
int oplus_panel_on(struct dsi_panel *panel);
int oplus_panel_off(struct dsi_panel *panel);

#endif /*_OPLUS_DISPLAY_PANEL_POWER_H_*/

