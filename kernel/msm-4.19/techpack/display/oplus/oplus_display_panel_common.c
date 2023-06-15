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
#include "oplus_display_panel_common.h"
#include <linux/notifier.h>
#include <linux/msm_drm_notify.h>
#include "video/mipi_display.h"
#include "oplus_display_private_api.h"
#include "oplus_dc_diming.h"

#if defined(OPLUS_FEATURE_PXLW_IRIS5)
#include "dsi_iris5_api.h"
#include "dsi_iris5_lightup.h"
#include "dsi_iris5_loop_back.h"
#endif

#include "oplus_display_panel.h"

int oplus_debug_max_brightness = 0;
int oplus_display_audio_ready = 0;
char oplus_rx_reg[PANEL_TX_MAX_BUF] = {0x0};
char oplus_rx_len = 0;
int lcd_closebl_flag = 0;
int spr_mode = 0;
extern int dynamic_osc_clock;
extern int oplus_dimlayer_hbm;
extern int oplus_dimlayer_bl;
bool frame_done_idle = true;

enum {
	REG_WRITE = 0,
	REG_READ,
	REG_X,
};

DEFINE_MUTEX(oplus_mca_lock);
DEFINE_MUTEX(oplus_spr_lock);

extern int msm_drm_notifier_call_chain(unsigned long val, void *v);
extern int __oplus_display_set_spr(int mode);
extern int dsi_display_spr_mode(struct dsi_display *display, int mode);


int dsi_panel_spr_mode(struct dsi_panel *panel, int mode)
{
	int rc = 0;

	if (!panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		goto error;
	}

	switch (mode) {
	case 0:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SPR_MODE0);
		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SPR_MODE0 cmds, rc=%d\n",
					panel->name, rc);
		}
		break;
	case 1:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SPR_MODE1);
		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SPR_MODE1 cmds, rc=%d\n",
					panel->name, rc);
		}
		break;
	case 2:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SPR_MODE2);
		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE2 cmds, rc=%d\n",
					panel->name, rc);
		}
		break;
	default:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SPR_MODE0);  /*gaos edit default is spr mode 0*/
		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SPR_MODE0 cmds, rc=%d\n",
					panel->name, rc);
		}
		pr_err("[%s] seed mode Invalid %d\n",
			panel->name, mode);
	}

error:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

int dsi_display_spr_mode(struct dsi_display *display, int mode) {
	int rc = 0;
	if (!display || !display->panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

		/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
	}

	rc = dsi_panel_spr_mode(display->panel, mode);
		if (rc) {
			pr_err("[%s] failed to dsi_panel_spr_on, rc=%d\n",
			       display->name, rc);
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
	rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}
	mutex_unlock(&display->display_lock);
	return rc;
}

int __oplus_display_set_spr(int mode) {
	mutex_lock(&oplus_spr_lock);
	if(mode != spr_mode) {
		spr_mode = mode;
	}
	mutex_unlock(&oplus_spr_lock);
	return 0;
}

int dsi_panel_read_panel_reg(struct dsi_display_ctrl *ctrl, struct dsi_panel *panel, u8 cmd, void *rbuf,  size_t len)
{
	int rc = 0;
	struct dsi_cmd_desc cmdsreq;
	u32 flags = 0;

	if (!panel || !ctrl || !ctrl->ctrl) {
		return -EINVAL;
	}

	if (!dsi_ctrl_validate_host_state(ctrl->ctrl)) {
		return 1;
	}

	/* acquire panel_lock to make sure no commands are in progress */
	mutex_lock(&panel->panel_lock);

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		goto error;
	}

	memset(&cmdsreq, 0x0, sizeof(cmdsreq));
	cmdsreq.msg.type = 0x06;
	cmdsreq.msg.tx_buf = &cmd;
	cmdsreq.msg.tx_len = 1;
	cmdsreq.msg.rx_buf = rbuf;
	cmdsreq.msg.rx_len = len;
	cmdsreq.msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
	flags |= (DSI_CTRL_CMD_FETCH_MEMORY | DSI_CTRL_CMD_READ |
		DSI_CTRL_CMD_CUSTOM_DMA_SCHED |
		DSI_CTRL_CMD_LAST_COMMAND);

	rc = dsi_ctrl_cmd_transfer(ctrl->ctrl, &cmdsreq.msg, &flags);
	if (rc <= 0) {
		pr_err("%s, dsi_display_read_panel_reg rx cmd transfer failed rc=%d\n",
			__func__,
			rc);
		goto error;
	}
error:
	/* release panel_lock */
	mutex_unlock(&panel->panel_lock);
	return rc;
}

int dsi_display_read_panel_reg(struct dsi_display *display, u8 cmd, void *data, size_t len) {
	int rc = 0;
	struct dsi_display_ctrl *m_ctrl;
	if (!display || !display->panel || data == NULL) {
		pr_err("%s, Invalid params\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	m_ctrl = &display->ctrl[display->cmd_master_idx];

	if (display->tx_cmd_buf == NULL) {
		rc = dsi_host_alloc_cmd_tx_buffer(display);
		if (rc) {
			pr_err("%s, failed to allocate cmd tx buffer memory\n", __func__);
			goto done;
		}
	}

	rc = dsi_display_cmd_engine_enable(display);
	if (rc) {
		pr_err("%s, cmd engine enable failed\n", __func__);
		goto done;
	}

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_ON);
	}

	rc = dsi_panel_read_panel_reg(m_ctrl, display->panel, cmd, data, len);
	if (rc < 0) {
		pr_err("%s, [%s] failed to read panel register, rc=%d,cmd=%d\n",
		       __func__,
		       display->name,
		       rc,
		       cmd);
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
					  DSI_ALL_CLKS, DSI_CLK_OFF);
	}

	dsi_display_cmd_engine_disable(display);

done:
	mutex_unlock(&display->display_lock);
	pr_err("%s, return: %d\n", __func__, rc);
	return rc;
}

int oplus_display_panel_get_id(void *buf)
{
	struct dsi_display *display = get_main_display();
	int ret = 0;
	unsigned char read[30];
	struct panel_id *panel_rid = buf;

	if(get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) {
		if(display == NULL) {
			printk(KERN_INFO "oplus_display_get_panel_id and main display is null");
			ret = -1;
			return ret;
		}

		ret = dsi_display_read_panel_reg(display, 0xDA, read, 1);
		if (ret < 0) {
			pr_err("failed to read DA ret=%d\n", ret);
			return -EINVAL;
		}
		panel_rid->DA = (uint32_t)read[0];

		ret = dsi_display_read_panel_reg(display, 0xDB, read, 1);
		if (ret < 0) {
			pr_err("failed to read DB ret=%d\n", ret);
			return -EINVAL;
		}
		panel_rid->DB = (uint32_t)read[0];

		ret = dsi_display_read_panel_reg(display, 0xDC, read, 1);
		if (ret < 0) {
			pr_err("failed to read DC ret=%d\n", ret);
			return -EINVAL;
		}
		panel_rid->DC = (uint32_t)read[0];
	} else {
		printk(KERN_ERR	 "%s oplus_display_get_panel_id, but now display panel status is not on\n", __func__);
		return -EINVAL;
	}

	return 0;
}

int oplus_display_panel_get_max_brightness(void *buf)
{
	uint32_t *max_brightness = buf;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = display->panel;

	if (oplus_debug_max_brightness == 0) {
		(*max_brightness) = panel->bl_config.bl_normal_max_level;
	} else {
		(*max_brightness) = oplus_debug_max_brightness;
	}

	return 0;
}

int oplus_display_panel_set_max_brightness(void *buf)
{
	uint32_t *max_brightness = buf;

	oplus_debug_max_brightness = (*max_brightness);

	return 0;
}

int oplus_display_panel_get_brightness(void *buf)
{
	uint32_t *brightness = buf;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = display->panel;

	(*brightness) = panel->bl_config.bl_level;

	return 0;
}

int oplus_display_panel_get_vendor(void *buf)
{
	struct panel_info *p_info = buf;
	struct dsi_display *display = NULL;
	char *vendor = NULL;
	char *manu_name = NULL;

	display = get_main_display();
	if (!display || !display->panel ||
	    !display->panel->oplus_priv.vendor_name ||
	    !display->panel->oplus_priv.manufacture_name) {
		pr_err("failed to config lcd proc device");
		return -EINVAL;
	}

	vendor = (char *)display->panel->oplus_priv.vendor_name;
	manu_name = (char *)display->panel->oplus_priv.manufacture_name;

	memcpy(p_info->version, vendor, strlen(vendor) > 31?31:(strlen(vendor)+1));
	memcpy(p_info->manufacture, manu_name, strlen(manu_name) > 31?31:(strlen(manu_name)+1));

	return 0;
}

int oplus_display_panel_get_ccd_check(void *buf)
{
	struct dsi_display *display = get_main_display();
	struct mipi_dsi_device *mipi_device;
	int rc = 0;
	unsigned int *ccd_check = buf;
	if (!display || !display->panel) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) {
		printk(KERN_ERR"%s display panel in off status\n", __func__);
		return -EFAULT;
	}

	if (display->panel->panel_mode != DSI_OP_CMD_MODE) {
		pr_err("only supported for command mode\n");
		return -EFAULT;
	}

	if (!(display && display->panel->oplus_priv.vendor_name) ||
		!strcmp(display->panel->oplus_priv.vendor_name, "NT37800") ||
		!strcmp(display->panel->oplus_priv.vendor_name, "AMB655XL08")) {
		(*ccd_check) = 0;
		goto end;
	}

	mipi_device = &display->panel->mipi_device;

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	if (!dsi_panel_initialized(display->panel)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = dsi_display_cmd_engine_enable(display);
	if (rc) {
		pr_err("%s, cmd engine enable failed\n", __func__);
		goto unlock;
	}

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
	}

	if (!strcmp(display->panel->oplus_priv.vendor_name, "AMB655UV01")) {
		{
			char value[] = { 0x5A, 0x5A };
			rc = mipi_dsi_dcs_write(mipi_device, 0xF0, value, sizeof(value));
		}
		{
			char value[] = { 0x44, 0x50 };
			rc = mipi_dsi_dcs_write(mipi_device, 0xE7, value, sizeof(value));
		}
		usleep_range(1000, 1100);
		{
			char value[] = { 0x03 };
			rc = mipi_dsi_dcs_write(mipi_device, 0xB0, value, sizeof(value));
		}
	} else {
		{
			char value[] = { 0x5A, 0x5A };
			rc = mipi_dsi_dcs_write(mipi_device, 0xF0, value, sizeof(value));
		}
		{
			char value[] = { 0x02 };
			rc = mipi_dsi_dcs_write(mipi_device, 0xB0, value, sizeof(value));
		}
		{
			char value[] = { 0x44, 0x50 };
			rc = mipi_dsi_dcs_write(mipi_device, 0xCC, value, sizeof(value));
		}
		usleep_range(1000, 1100);
		{
			char value[] = { 0x05 };
			rc = mipi_dsi_dcs_write(mipi_device, 0xB0, value, sizeof(value));
		}
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}
	dsi_display_cmd_engine_disable(display);

	mutex_unlock(&display->panel->panel_lock);
	mutex_unlock(&display->display_lock);
	if (!strcmp(display->panel->oplus_priv.vendor_name, "AMB655UV01")) {
		{
			unsigned char read[10];

			rc = dsi_display_read_panel_reg(display, 0xE1, read, 1);

			pr_err("read ccd_check value = 0x%x rc=%d\n", read[0], rc);
			(*ccd_check) = read[0];
		}
	} else {
		{
			unsigned char read[10];

			rc = dsi_display_read_panel_reg(display, 0xCC, read, 1);

			pr_err("read ccd_check value = 0x%x rc=%d\n", read[0], rc);
			(*ccd_check) = read[0];
		}
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	if (!dsi_panel_initialized(display->panel)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = dsi_display_cmd_engine_enable(display);
	if (rc) {
		pr_err("%s, cmd engine enable failed\n", __func__);
		goto unlock;
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
	}

	{
		char value[] = { 0xA5, 0xA5 };
		rc = mipi_dsi_dcs_write(mipi_device, 0xF0, value, sizeof(value));
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}

	dsi_display_cmd_engine_disable(display);
unlock:

	mutex_unlock(&display->panel->panel_lock);
	mutex_unlock(&display->display_lock);
end:
	pr_err("[%s] ccd_check = %d\n",  display->panel->oplus_priv.vendor_name, (*ccd_check));
	return 0;
}

int oplus_display_panel_get_serial_number(void *buf)
{
	int ret = 0;
	unsigned char read[30];
	PANEL_SERIAL_INFO panel_serial_info;
	struct panel_serial_number* panel_rnum = buf;
	uint64_t serial_number;
	struct dsi_display *display = get_main_display();
	struct dsi_parser_utils *utils;
	unsigned base_index = 10;
	unsigned base_reg = 0xA1;
	unsigned val = 0;
	int i;

	if (!display) {
		printk(KERN_INFO "oplus_display_get_panel_serial_number and main display is null");
		return -1;
	}
	utils = &display->panel->utils;

	if(get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) {
		printk(KERN_ERR"%s display panel in off status\n", __func__);
		return ret;
	}

	ret = utils->read_u32(utils->data, "oplus,mdss-panel-serial-number-index", &val);
	if (ret) {
		DSI_DEBUG("[%s] panel-serial-number-index unspecified, defaulting to base_index\n",
			 display->panel->name);
	} else {
		base_index = val;
	}
	ret = utils->read_u32(utils->data, "oplus,mdss-panel-serial-number-reg", &val);
	if (ret) {
		DSI_DEBUG("[%s] panel-serial-number-reg unspecified, defaulting to base_index\n",
			 display->panel->name);
	} else {
		base_reg = val;
	}

	/*
	 * for some unknown reason, the panel_serial_info may read dummy,
	 * retry when found panel_serial_info is abnormal.
	 */
	for (i = 0;i < 10; i++) {
		ret = dsi_display_read_panel_reg(get_main_display(), base_reg, read, 17);
		if(ret < 0) {
			ret = scnprintf(buf, PAGE_SIZE,
					"Get panel serial number failed, reason:%d",ret);
			msleep(20);
			continue;
		}

		/*	0xA1			   11th 	   12th    13th    14th    15th
		 *	HEX 			   0x32 	   0x0C    0x0B    0x29    0x37
		 *	Bit 		  [D7:D4][D3:D0] [D5:D0] [D5:D0] [D5:D0] [D5:D0]
		 *	exp 			 3		2		C		B		29		37
		 *	Yyyy,mm,dd		2014   2m	   12d	   11h	   41min   55sec
		*/
		panel_serial_info.reg_index = base_index;

		panel_serial_info.year		= (read[panel_serial_info.reg_index] & 0xF0) >> 0x4;
		panel_serial_info.month 	= read[panel_serial_info.reg_index] & 0x0F;
		panel_serial_info.day		= read[panel_serial_info.reg_index + 1] & 0x1F;
		panel_serial_info.hour		= read[panel_serial_info.reg_index + 2] & 0x1F;
		panel_serial_info.minute	= read[panel_serial_info.reg_index + 3] & 0x3F;
		panel_serial_info.second	= read[panel_serial_info.reg_index + 4] & 0x3F;
		panel_serial_info.reserved[0] = read[panel_serial_info.reg_index + 5];
		panel_serial_info.reserved[1] = read[panel_serial_info.reg_index + 6];

		if (!strcmp(display->panel->oplus_priv.vendor_name,"AMS644VA04")) {
			/* 1-4th oplus code xxx
			 * 5th vendor code/Month
			 * 6th Year/Day
			 * 7th Hour
			 * 8th Minute
			 * 9th Second
			 * 10-11th Second(ms)
			 */
			panel_serial_info.year		= ((read[base_index + 1] & 0xE0) >> 5) + 7;;
			panel_serial_info.month 	= read[base_index] & 0x0F;
			panel_serial_info.day		= read[base_index + 1] & 0x1F;
		}

		serial_number = (panel_serial_info.year 	<< 56)\
			+ (panel_serial_info.month		<< 48)\
			+ (panel_serial_info.day		<< 40)\
			+ (panel_serial_info.hour		<< 32)\
			+ (panel_serial_info.minute << 24)\
			+ (panel_serial_info.second << 16)\
			+ (panel_serial_info.reserved[0] << 8)\
			+ (panel_serial_info.reserved[1]);
		if (!panel_serial_info.year) {
			/*
			 * the panel we use always large than 2011, so
			 * force retry when year is 2011
			 */
			msleep(20);
			continue;
		}

		ret = scnprintf(panel_rnum->serial_number, PAGE_SIZE, "Get panel serial number: %llx\n",serial_number);
		break;
	}

	return ret;
}



int oplus_display_panel_set_audio_ready(void *data) {
	uint32_t *audio_ready = data;

	oplus_display_audio_ready = (*audio_ready);
	printk("%s oplus_display_audio_ready = %d\n", __func__, oplus_display_audio_ready);

	return 0;
}

int oplus_display_panel_dump_info(void *data) {
	int ret = 0;
	struct dsi_display * temp_display;
	struct display_timing_info *timing_info = data;

	temp_display = get_main_display();

	if (temp_display == NULL) {
		printk(KERN_INFO "oplus_display_dump_info and main display is null");
		ret = -1;
		return ret;
	}

	if(temp_display->modes == NULL) {
		printk(KERN_INFO "oplus_display_dump_info and display modes is null");
		ret = -1;
		return ret;
	}

	timing_info->h_active = temp_display->modes->timing.h_active;
	timing_info->v_active = temp_display->modes->timing.v_active;
	timing_info->refresh_rate = temp_display->modes->timing.refresh_rate;
	timing_info->clk_rate_hz_l32 = (uint32_t)(temp_display->modes->timing.clk_rate_hz & 0x00000000FFFFFFFF);
	timing_info->clk_rate_hz_h32 = (uint32_t)(temp_display->modes->timing.clk_rate_hz >> 32);

	return 0;
}

int oplus_display_panel_get_dsc(void *data) {
	int ret = 0;
	uint32_t *reg_read = data;
	unsigned char read[30];

	if(get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) {
		if(get_main_display() == NULL) {
			printk(KERN_INFO "oplus_display_get_panel_dsc and main display is null");
			ret = -1;
			return ret;
		}

		ret = dsi_display_read_panel_reg(get_main_display(), 0x03, read, 1);
		if (ret < 0) {
			printk(KERN_ERR  "%s read panel dsc reg error = %d!\n", __func__, ret);
			ret = -1;
		} else {
			(*reg_read) = read[0];
			ret = 0;
		}
	} else {
		printk(KERN_ERR	 "%s but now display panel status is not on\n", __func__);
		ret = -1;
	}

	return ret;
}

int oplus_display_panel_get_closebl_flag(void *data)
{
	uint32_t *closebl_flag = data;

	(*closebl_flag) = lcd_closebl_flag;
	printk(KERN_INFO "oplus_display_get_closebl_flag = %d\n", lcd_closebl_flag);

	return 0;
}

int oplus_display_panel_set_closebl_flag(void *data)
{
	uint32_t *closebl = data;

	pr_err("lcd_closebl_flag = %d\n", (*closebl));
	if (1 != (*closebl))
		lcd_closebl_flag = 0;
	pr_err("oplus_display_set_closebl_flag = %d\n", lcd_closebl_flag);

	return 0;
}

#if defined(OPLUS_FEATURE_PXLW_IRIS5)
int iris_panel_dcs_type_set(struct dsi_cmd_desc *cmd, void *data, size_t len)
{
	switch (len) {
	case 0:
		return -EINVAL;
	case 1:
		cmd->msg.type = MIPI_DSI_DCS_SHORT_WRITE;
		break;
	case 2:
		cmd->msg.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		break;
	default:
		cmd->msg.type = MIPI_DSI_DCS_LONG_WRITE;
		break;
	}
	cmd->msg.tx_len = len;
	cmd->msg.tx_buf = data;
	return 0;
}

int iris_panel_dcs_write_wrapper(struct dsi_panel *panel, void *data, size_t len)
{
	int rc = 0;
	struct dsi_panel_cmd_set cmdset;
	struct dsi_cmd_desc dsi_cmd =
		{{0, MIPI_DSI_DCS_SHORT_WRITE, 0, 0, 0, 0, NULL, 0, NULL}, 1, 0};

	memset(&cmdset, 0x00, sizeof(cmdset));
	cmdset.cmds = &dsi_cmd;
	cmdset.count = 1;
	rc = iris_panel_dcs_type_set(&dsi_cmd, data, len);
	if (rc < 0) {
		pr_err("%s: invalid dsi cmd len\n", __func__);
		return rc;
	}

	rc = iris5_panel_cmd_passthrough(panel, &cmdset);
	if (rc < 0) {
		pr_err("%s: send panel command failed\n", __func__);
		return rc;
	}
	return rc;
}

int iris_panel_dcs_read_wrapper(struct dsi_display *display, u8 cmd, void *rbuf, size_t rlen)
{
	int rc = 0;
	struct dsi_panel_cmd_set cmdset;
	struct dsi_cmd_desc dsi_cmd =
		{{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0, 0, 1, &cmd, rlen, rbuf}, 1, 0};
	struct dsi_panel *panel;

	if (!display || !display->panel) {
		pr_err("%s, Invalid params\n", __func__);
		return -EINVAL;
	}

	memset(&cmdset, 0x00, sizeof(cmdset));
	cmdset.cmds = &dsi_cmd;
	cmdset.count = 1;

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_ON);
	}

	panel = display->panel;
	mutex_lock(&display->display_lock);
	mutex_lock(&panel->panel_lock);

	rc = iris5_panel_cmd_passthrough(panel, &cmdset);
	mutex_unlock(&panel->panel_lock);
	mutex_unlock(&display->display_lock);

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_OFF);
	}

	if (rc < 0) {
		pr_err("%s, [%s] failed to read panel register, rc=%d,cmd=%d\n",
		       __func__,
		       display->name,
		       rc,
		       cmd);
		return rc;
	}
	pr_err("%s, return: %d\n", __func__, rc);
	return rc;
}
#endif

int oplus_big_endian_copy(void *dest, void *src, int count)
{
	int index = 0, knum = 0, rc = 0;
	uint32_t *u_dest = (uint32_t*) dest;
	char *u_src = (char*) src;

	if (dest == NULL || src == NULL) {
		printk("%s null pointer\n", __func__);
		return -EINVAL;
	}

	if (dest == src) {
		return rc;
	}

	while (count > 0) {
		u_dest[index] = ((u_src[knum] << 24) | (u_src[knum+1] << 16) | (u_src[knum+2] << 8) | u_src[knum+3]);
		index += 1;
		knum += 4;
		count = count - 1;
	}

	return rc;
}

int oplus_display_panel_get_reg(void *data)
{
	struct dsi_display *display = get_main_display();
	struct panel_reg_get *panel_reg = data;
	uint32_t u32_bytes = sizeof(uint32_t)/sizeof(char);

	if (!display) {
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	u32_bytes = oplus_rx_len%u32_bytes ? (oplus_rx_len/u32_bytes + 1) : oplus_rx_len/u32_bytes;
	oplus_big_endian_copy(panel_reg->reg_rw, oplus_rx_reg, u32_bytes);
	panel_reg->lens = oplus_rx_len;

	mutex_unlock(&display->display_lock);

	return 0;
}

int oplus_display_panel_set_reg(void *data)
{
	char reg[PANEL_TX_MAX_BUF] = {0x0};
	char payload[PANEL_TX_MAX_BUF] = {0x0};
	u32 index = 0, value = 0;
	int ret = 0;
	int len = 0;
	struct dsi_display *display = get_main_display();
	struct panel_reg_rw *reg_rw = data;

	if (!display || !display->panel) {
		pr_err("debug for: %s %d\n", __func__, __LINE__);
		return -EFAULT;
	}

	if (reg_rw->lens > PANEL_REG_MAX_LENS) {
		pr_err("error: wrong input reg len\n");
		return -EINVAL;
	}

	if (reg_rw->rw_flags == REG_READ) {
		value = reg_rw->cmd;
		len = reg_rw->lens;

#if defined(OPLUS_FEATURE_PXLW_IRIS5)
		if (iris_get_feature() && (iris5_abypass_mode_get(get_main_display()->panel) == PASS_THROUGH_MODE))
			iris_panel_dcs_read_wrapper(get_main_display(), value, reg, len);
		else
			dsi_display_read_panel_reg(get_main_display(), value, reg, len);
#else
		dsi_display_read_panel_reg(get_main_display(), value, reg, len);
#endif

		for (index; index < len; index++) {
			printk("reg[%d] = %x ", index, reg[index]);
		}
		mutex_lock(&display->display_lock);
		memcpy(oplus_rx_reg, reg, PANEL_TX_MAX_BUF);
		oplus_rx_len = len;
		mutex_unlock(&display->display_lock);
		return 0;
	}

	if (reg_rw->rw_flags == REG_WRITE) {
		memcpy(payload, reg_rw->value, reg_rw->lens);
		reg[0] = reg_rw->cmd;
		len = reg_rw->lens;
		for (index; index < len; index++) {
			reg[index + 1] = payload[index];
		}

		if(get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) {
				/* enable the clk vote for CMD mode panels */
			mutex_lock(&display->display_lock);
			mutex_lock(&display->panel->panel_lock);

			if (display->panel->panel_initialized) {
				if (display->config.panel_mode == DSI_OP_CMD_MODE) {
					dsi_display_clk_ctrl(display->dsi_clk_handle,
							DSI_ALL_CLKS, DSI_CLK_ON);
				}
#if defined(OPLUS_FEATURE_PXLW_IRIS5)
				if (iris_get_feature() && (iris5_abypass_mode_get(display->panel) == PASS_THROUGH_MODE))
					ret = iris_panel_dcs_write_wrapper(display->panel, reg, len+1);
				else
					ret = mipi_dsi_dcs_write(&display->panel->mipi_device, reg[0],
								 payload, len);
#else
				ret = mipi_dsi_dcs_write(&display->panel->mipi_device, reg[0],
							 payload, len);
#endif

				if (display->config.panel_mode == DSI_OP_CMD_MODE) {
					dsi_display_clk_ctrl(display->dsi_clk_handle,
							DSI_ALL_CLKS, DSI_CLK_OFF);
				}
			}

			mutex_unlock(&display->panel->panel_lock);
			mutex_unlock(&display->display_lock);

			if (ret < 0) {
				return ret;
			}
		}
		return 0;
	}
	printk("%s error: please check the args!\n", __func__);
	return -1;
}

int oplus_display_panel_notify_blank(void *data)
{
	struct msm_drm_notifier notifier_data;
	int blank;
	uint32_t *temp_save_user = data;
	int temp_save = (*temp_save_user);

	printk(KERN_INFO "%s oplus_display_notify_panel_blank = %d\n", __func__, temp_save);

	if(temp_save == 1) {
		blank = MSM_DRM_BLANK_UNBLANK;
		notifier_data.data = &blank;
		notifier_data.id = 0;
		msm_drm_notifier_call_chain(MSM_DRM_EARLY_EVENT_BLANK,
						   &notifier_data);
		msm_drm_notifier_call_chain(MSM_DRM_EVENT_BLANK,
						   &notifier_data);
	} else if (temp_save == 0) {
		blank = MSM_DRM_BLANK_POWERDOWN;
		notifier_data.data = &blank;
		notifier_data.id = 0;
		msm_drm_notifier_call_chain(MSM_DRM_EARLY_EVENT_BLANK,
						   &notifier_data);
	}
	return 0;
}

int oplus_display_panel_get_spr(void *data)
{
	uint32_t *spr_mode_user = data;

	printk(KERN_INFO "oplus_display_get_spr = %d\n", spr_mode);
	*spr_mode_user = spr_mode;

	return 0;
}

int oplus_display_panel_set_spr(void *data)
{
	uint32_t *temp_save_user = data;
	int temp_save = (*temp_save_user);

	printk(KERN_INFO "%s oplus_display_set_spr = %d\n", __func__, temp_save);

	__oplus_display_set_spr(temp_save);
	if(get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) {
		if(get_main_display() == NULL) {
			printk(KERN_INFO "oplus_display_set_spr and main display is null");
			return 0;
		}

		dsi_display_spr_mode(get_main_display(), spr_mode);
	} else {
		printk(KERN_ERR	 "%s oplus_display_set_spr = %d, but now display panel status is not on\n", __func__, temp_save);
	}
	return 0;
}

int oplus_display_panel_get_roundcorner(void *data)
{
	uint32_t *round_corner = data;
	struct dsi_display *display = get_main_display();
	bool roundcorner = true;

	if (display && display->name &&
	    !strcmp(display->name, "qcom,mdss_dsi_oplus19101boe_nt37800_1080_2400_cmd"))
		roundcorner = false;

	*round_corner = roundcorner;

	return 0;
}

int oplus_display_panel_get_dynamic_osc_clock(void *data)
{
	struct dsi_display *display = get_main_display();
	uint32_t *osc_clock = data;

	if (!display) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	*osc_clock = dynamic_osc_clock;
	pr_debug("%s: read dsi clk rate %d\n", __func__,
			dynamic_osc_clock);

	mutex_unlock(&display->display_lock);

	return 0;
}

int oplus_display_panel_set_dynamic_osc_clock(void *data)
{
	struct dsi_display *display = get_main_display();
	uint32_t *osc_clk_user = data;
	int osc_clk = *osc_clk_user;
	int rc = 0;

	if (!display) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (!display->panel->oplus_priv.is_osc_support) {
		LCD_DEBUG_COMMON("osc is not support\n");
		return -EINVAL;
	}

	if(get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) {
		printk(KERN_ERR"%s display panel in off status\n", __func__);
		return -EFAULT;
	}

	if (display->panel->panel_mode != DSI_OP_CMD_MODE) {
		pr_err("only supported for command mode\n");
		return -EFAULT;
	}

	LCD_INFO("osc clk param value: '%d'\n", osc_clk);

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	if (!dsi_panel_initialized(display->panel)) {
		rc = -EINVAL;
		goto unlock;
	}

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
	}

	if (display->panel->oplus_priv.osc_clk_mode0_rate == osc_clk) {
		rc = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_OSC_CLK_MODEO0);
	} else if (display->panel->oplus_priv.osc_clk_mode1_rate == osc_clk) {
		rc = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_OSC_CLK_MODEO1);
	} else {
		LCD_INFO("osc clk value: '%d' not support!\n", osc_clk);
	}

	if (rc)
		pr_err("Failed to configure osc dynamic clk\n");

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}
	dynamic_osc_clock = osc_clk;

unlock:
	mutex_unlock(&display->panel->panel_lock);
	mutex_unlock(&display->display_lock);

	return rc;
}

int oplus_display_get_softiris_color_status(void *data)
{
	struct softiris_color *iris_color_status = data;
	bool color_vivid_status = false;
	bool color_srgb_status = false;
	bool color_softiris_status = false;
	struct dsi_parser_utils *utils = NULL;
	struct dsi_panel *panel = NULL;

	struct dsi_display *display = get_main_display();
	if (!display) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	panel = display->panel;
	if (!panel) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	utils = &panel->utils;
	if (!utils) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	color_vivid_status = utils->read_bool(utils->data, "oplus,color_vivid_status");
	DSI_INFO("oplus,color_vivid_status: %s", color_vivid_status ? "true" : "false");

	color_srgb_status = utils->read_bool(utils->data, "oplus,color_srgb_status");
	DSI_INFO("oplus,color_srgb_status: %s", color_srgb_status ? "true" : "false");

	color_softiris_status = utils->read_bool(utils->data, "oplus,color_softiris_status");
	DSI_INFO("oplus,color_softiris_status: %s", color_softiris_status ? "true" : "false");

	iris_color_status->color_vivid_status = (uint32_t)color_vivid_status;
	iris_color_status->color_srgb_status = (uint32_t)color_srgb_status;
	iris_color_status->color_softiris_status = (uint32_t)color_softiris_status;

	return 0;
}

int oplus_display_panel_get_dsi_log_switch(void *data)
{
	uint32_t *temp = data;

	*temp = oplus_dsi_log_type;
	LCD_INFO(" oplus_dsi_log_type = 0x%x\n", oplus_dsi_log_type);

	return 0;
}

int oplus_display_panel_set_dsi_log_switch(void *data)
{
	uint32_t *temp = data;

	oplus_dsi_log_type = *temp;
	LCD_INFO(" oplus_dsi_log_type = 0x%x\n", oplus_dsi_log_type);

	return 0;
}

int oplus_display_panel_set_framedone_idle(void *data)
{
	uint32_t *temp = data;

	frame_done_idle = (*temp) == 1 ? true:false;
	LCD_INFO(" oplus_frame_done_idle = %d\n", frame_done_idle);

	return 0;
}

int oplus_display_set_backlight(struct dsi_display *display, u32 bl_lvl)
{
	int rc = 0;
	struct dsi_display *dsi_display = display;
	struct dsi_panel *panel = display->panel;

	if ((bl_lvl == 0 && panel->bl_config.bl_level != 0) ||
		(bl_lvl != 0 && panel->bl_config.bl_level == 0)) {
		pr_info("<%s> backlight level changed %d -> %d\n",
			panel->oplus_priv.vendor_name,
			panel->bl_config.bl_level, bl_lvl);
	/* PSW.MM.Display.LCD.Stable,2019-11-22,Add key log for debug */
	} else if (panel->bl_config.bl_level == 1) {
		pr_info("<%s> aod backlight level changed %d -> %d\n",
			panel->oplus_priv.vendor_name,
			panel->bl_config.bl_level, bl_lvl);
	}
	/* Add some delay to avoid screen flash */
	if (panel->need_power_on_backlight && bl_lvl) {
		panel->need_power_on_backlight = false;
		rc = dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_POST_ON_BACKLIGHT cmds, rc=%d\n",
				panel->name, rc);
			return rc;
		}

		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_POST_ON_BACKLIGHT);

		rc = dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_OFF);

		atomic_set(&panel->esd_pending, 0);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_POST_ON_BACKLIGHT cmds, rc=%d\n",
				panel->name, rc);
			return rc;
		}
	}
	return rc;
}

ktime_t oplus_backlight_time;
u32 oplus_last_backlight = 0;
u32 oplus_backlight_delta = 0;
extern int oplus_display_mode;
extern int oplus_dc_dimlayer_bl_enabled;
extern int oplus_dc_dimlayer_bl_enable_real;
extern int oplus_dc_dimlayer_bl_alpha;
bool oplus_aod_mode = false;
extern int dsi_panel_dcs_set_display_brightness_c2(struct mipi_dsi_device *dsi, u32 bl_lvl);

int oplus_panel_update_backlight(struct dsi_panel *panel, u32 bl_lvl)
{
	int rc = 0;
	struct mipi_dsi_device *dsi;
	dsi = &panel->mipi_device;

	if ((get_oplus_display_scene() == OPLUS_DISPLAY_AOD_SCENE) && (bl_lvl == 1)) {
		pr_info("dsi_cmd AOD mode return bl_lvl:%d\n", bl_lvl);
		if (panel->oplus_priv.is_aod_ramless)
			oplus_aod_mode = true;
		goto skip_bl_update;
	}

	if (panel->is_hbm_enabled || (panel->oplus_priv.is_aod_ramless && !oplus_display_mode))
		goto skip_bl_update;

	if(panel->is_dc_support) {
		if (oplus_dc_dimlayer_bl_enabled != oplus_dc_dimlayer_bl_enable_real) {
			oplus_dc_dimlayer_bl_enable_real = oplus_dc_dimlayer_bl_enabled;
			if (oplus_dc_dimlayer_bl_enable_real) {
				pr_err("Enter DC backlight\n");
			} else {
				pr_err("Exit DC backlight\n");
			}
		}

		bl_lvl = oplus_dc_panel_process_dimming_v2(panel, bl_lvl, false);

		if (oplus_dc_dimlayer_bl_enable_real) {
			/*
			 * avoid effect power and aod mode
			 */
			if (bl_lvl > 1)
				bl_lvl = oplus_dc_dimlayer_bl_alpha;
		}
	}

	if (bl_lvl > 1) {
		if (bl_lvl > oplus_last_backlight)
			oplus_backlight_delta = bl_lvl - oplus_last_backlight;
		else
			oplus_backlight_delta = oplus_last_backlight - bl_lvl;
		oplus_backlight_time = ktime_get();
	}

	if (panel->oplus_priv.is_aod_ramless)
		oplus_aod_mode = false;

	if (panel->oplus_priv.is_samsung_panel) {
		const struct mipi_dsi_host_ops *ops = dsi->host->ops;
		char payload[] = {MIPI_DCS_WRITE_CONTROL_DISPLAY, 0xE0};
		struct mipi_dsi_msg msg;

		if (bl_lvl > panel->bl_config.bl_normal_max_level)
			payload[1] = 0xE0;
		else
			payload[1] = 0x20;

		memset(&msg, 0, sizeof(msg));
		msg.channel = dsi->channel;
		msg.tx_buf = payload;
		msg.tx_len = sizeof(payload);
		msg.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;

			rc = ops->transfer(dsi->host, &msg);

		if (rc < 0)
			pr_err("failed to backlight bl_lvl %d - ret=%d\n", bl_lvl, rc);
	}

	/* MM.Display.LCD, 2021/08/09, add for global hbm */
	if (panel->bl_config.bl_hbm_min_level) {
		if (bl_lvl > panel->bl_config.bl_normal_max_level) {
			bl_lvl = bl_lvl + panel->bl_config.bl_hbm_min_level - panel->bl_config.bl_normal_max_level;
			bl_lvl = bl_lvl < panel->bl_config.bl_max_level ? bl_lvl : panel->bl_config.bl_max_level;
		}
	}
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &bl_lvl);

	if (panel->bl_config.bl_dcs_subtype == 0xc2)
		rc = dsi_panel_dcs_set_display_brightness_c2(dsi, bl_lvl);
	else
		rc = mipi_dsi_dcs_set_display_brightness(dsi, bl_lvl);

	oplus_last_backlight = bl_lvl;

skip_bl_update:
	return rc;
}

int oplus_display_panel_switch_gamma_mode(struct dsi_panel *panel, u32 bl_lvl)
{
	int rc = 0;
	static int gamma_mode = 0;

	if (bl_lvl < 14 && bl_lvl > 0) {
		gamma_mode = 1;
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_BACKLIGHT_GAMMA_ENTER);
		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_BACKLIGHT_GAMMA_ENTER cmds, rc=%d\n",
				panel->name, rc);
		}
	} else if (bl_lvl > 13 && gamma_mode != 2) {
		gamma_mode = 2;
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_BACKLIGHT_GAMMA_EXIT);
		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_BACKLIGHT_GAMMA_EXIT cmds, rc=%d\n",
				panel->name, rc);
		}
	}

	return rc;
}

/*add for set all pixel off while display off to reduce screen red flash*/
int oplus_display_set_pixel_off(struct dsi_panel *panel)
{
	int rc;

	if (!panel)
		return -EINVAL;

	rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_PIXEL_OFF);
	if (rc) {
		pr_err("[%s] failed to send DSI_CMD_PIXEL_OFF cmds, rc=%d\n",
			panel->name, rc);
	}

	return rc;
}

void dsi_oplus_panel_parse_gpios(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = &panel->utils;

	panel->panel_vci_gpio = utils->get_named_gpio(utils->data,
					      "qcom,platform-panel-vci-gpio", 0);
	if (!gpio_is_valid(panel->panel_vci_gpio)) {
		DSI_INFO("[%s] failed get reset gpio, qcom,platform-panel-vci-gpio\n", panel->name);
	}

	panel->panel_vddr_gpio = utils->get_named_gpio(utils->data,
					      "qcom,platform-panel-vddr-gpio", 0);
	if (!gpio_is_valid(panel->panel_vddr_gpio)) {
		DSI_INFO("[%s] failed get reset gpio, qcom,platform-panel-vddr-gpio\n", panel->name);
	}

	panel->panel_vddi_gpio = utils->get_named_gpio(utils->data,
					      "qcom,platform-panel-vddi-gpio", 0);
	if (!gpio_is_valid(panel->panel_vddi_gpio)) {
		DSI_INFO("[%s] failed get reset gpio, qcom,platform-panel-vddi-gpio\n", panel->name);
	}

	panel->panel_P5V_gpio = utils->get_named_gpio(utils->data,
					      "qcom,platform-panel-P5V-gpio", 0);
	if (!gpio_is_valid(panel->panel_P5V_gpio)) {
		DSI_INFO("[%s] failed get reset gpio, qcom,platform-panel-P5V-gpio\n", panel->name);
	}

	panel->panel_N5V_gpio = utils->get_named_gpio(utils->data,
					      "qcom,platform-panel-N5V-gpio", 0);
	if (!gpio_is_valid(panel->panel_N5V_gpio)) {
		DSI_INFO("[%s] failed get reset gpio, qcom,platform-panel-N5V-gpio\n", panel->name);
	}
}

