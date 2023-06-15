/* Copyright (c) 2018-2019 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/log2.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/irq.h>
#include <linux/iio/consumer.h>
#include <linux/pmic-voter.h>
#include <linux/of_batterydata.h>
#include <asm-generic/bug.h>
//#ifndef OPLUS_FEATURE_CHG_BASIC
///* Yichun.Chen  PSW.BSP.CHG  for charge */
//#include "smb5-reg.h"
//#include "smb5-lib.h"
//#include "schgm-flash.h"
//#else
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include "../../../../../kernel/msm-4.19/drivers/power/supply/qcom/smb5-reg.h"
#include "../../../../../kernel/msm-4.19/drivers/power/supply/qcom/battery.h"
#include "../../../../../kernel/msm-4.19/drivers/power/supply/qcom/step-chg-jeita.h"
#include "../../../../../kernel/msm-4.19/drivers/power/supply/qcom/storm-watch.h"

#include <soc/oplus/boot_mode.h>
//#include <soc/oplus/device_info.h>
//#include <soc/oplus/OPLUS_project.h>

#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"
#include "../oplus_short.h"
#include "../oplus_adapter.h"
#include "../charger_ic/oplus_short_ic.h"
#include "../charger_ic/op_charge.h"
#include "../gauge_ic/oplus_bq27541.h"
#include "../charger_ic/oplus_sy697x.h"
#ifdef OPLUS_FEATURE_CHG_BASIC
#include <linux/sched/clock.h>
#include <linux/cpufreq.h>
#endif

#include "../../../../../kernel/msm-4.19/drivers/power/supply/qcom/schgm-flash.h"
#include "oplus_sy6974b.h"

#define OPLUS_CHG_MONITOR_INTERVAL round_jiffies_relative(msecs_to_jiffies(5000))
#define CC1_ATTACH 1

extern int get_boot_mode(void);
void oplus_set_usb_status(int status);
void oplus_clear_usb_status(int status);
extern int oplus_usbtemp_monitor_common(void *data);
extern void oplus_usbtemp_recover_func(struct oplus_chg_chip *chip);
extern void oplus_wake_up_usbtemp_thread(void);

extern struct sy697x* oplus_sy697x_get_oplus_chg(void);
extern struct sy697x* oplus_bq25890h_get_oplus_chg(void);
extern struct sy697x* oplus_sgm4154x_get_oplus_chg(void);

extern int typec_dir;


static int oplus_get_iio_channel(struct sy697x *chip, const char *propname, struct iio_channel **chan);

//#endif
/*Start of smb5-lib.c*/
#define smblib_err(chg, fmt, ...)		\
	pr_err("%s: %s: " fmt, chg->name,	\
		__func__, ##__VA_ARGS__)	\



int smblib_read(struct smb_charger *chg, u16 addr, u8 *val)
{
	unsigned int value;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &value);
	if (rc >= 0)
		*val = (u8)value;

	return rc;
}

int smblib_batch_read(struct smb_charger *chg, u16 addr, u8 *val,
			int count)
{
	return regmap_bulk_read(chg->regmap, addr, val, count);
}

int smblib_write(struct smb_charger *chg, u16 addr, u8 val)
{
	return regmap_write(chg->regmap, addr, val);
}

int smblib_batch_write(struct smb_charger *chg, u16 addr, u8 *val,
			int count)
{
	return regmap_bulk_write(chg->regmap, addr, val, count);
}

int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val)
{
	return regmap_update_bits(chg->regmap, addr, mask, val);
}

int smblib_get_iio_channel(struct smb_charger *chg, const char *propname,
					struct iio_channel **chan)
{
	int rc = 0;

	rc = of_property_match_string(chg->dev->of_node,
					"io-channel-names", propname);
	if (rc < 0)
		return 0;

	*chan = iio_channel_get(chg->dev, propname);
	if (IS_ERR(*chan)) {
		rc = PTR_ERR(*chan);
		if (rc != -EPROBE_DEFER)
			smblib_err(chg, "%s channel unavailable, %d\n",
							propname, rc);
		*chan = NULL;
	}

	return rc;
}

#define DIV_FACTOR_MICRO_V_I	1
#define DIV_FACTOR_MILI_V_I	1000
#define DIV_FACTOR_DECIDEGC	100
int smblib_read_iio_channel(struct smb_charger *chg, struct iio_channel *chan,
							int div, int *data)
{
	int rc = 0;
	*data = -ENODATA;

	if (chan) {
		rc = iio_read_channel_processed(chan, data);
		if (rc < 0) {
			smblib_err(chg, "Error in reading IIO channel data, rc=%d\n",
					rc);
			return rc;
		}

		if (div != 0)
			*data /= div;
	}

	return rc;
}

struct sy697x* oplus_get_chg_sy(void)
{
	struct sy697x *sy = NULL;

	if(oplus_sy697x_get_oplus_chg()) {
		sy = oplus_sy697x_get_oplus_chg();
	} else if(oplus_sy6974b_get_oplus_chg()) {
		sy = oplus_sy6974b_get_oplus_chg();
	} else if(oplus_sgm4154x_get_oplus_chg()) {
		sy = oplus_sgm4154x_get_oplus_chg();
	} else if (oplus_bq25890h_get_oplus_chg()) {
		sy = oplus_bq25890h_get_oplus_chg();
	} else {
		pr_err(" %s charger chip is NULL\n", __func__);
	}

	return sy;
}

void oplus_typec_sink_removal(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return;

	if (chg->chgic_ops->typec_sink_removal)
		chg->chgic_ops->typec_sink_removal();

	return;
}
EXPORT_SYMBOL(oplus_typec_sink_removal);

void oplus_typec_src_removal(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return;

	if (chg->chgic_ops->typec_src_removal)
		chg->chgic_ops->typec_src_removal();

	return;
}
EXPORT_SYMBOL(oplus_typec_src_removal);

void oplus_typec_sink_insertion(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return;

	if (chg->chgic_ops->typec_sink_insertion)
			chg->chgic_ops->typec_sink_insertion();

	return;
}
EXPORT_SYMBOL(oplus_typec_sink_insertion);

bool oplus_get_otg_switch_status(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return 0;

	if (chg->chgic_ops->get_otg_switch_status)
		return chg->chgic_ops->get_otg_switch_status();

	return 0;
}
EXPORT_SYMBOL(oplus_get_otg_switch_status);

void oplus_set_otg_switch_status(bool value)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return;

	if (chg->chgic_ops->set_otg_switch_status)
			chg->chgic_ops->set_otg_switch_status(value);

	return;
}
EXPORT_SYMBOL(oplus_set_otg_switch_status);

int oplus_get_otg_online_status(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return 0;

	if (chg->chgic_ops->get_otg_online_status)
		return chg->chgic_ops->get_otg_online_status();

	return 0;
}
EXPORT_SYMBOL(oplus_get_otg_online_status);

int oplus_get_typec_cc_orientation(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return 0;

	if (chg->chgic_ops->get_typec_cc_orientation){
		return chg->chgic_ops->get_typec_cc_orientation();
	}else{
		pr_err("typec_dir = %s\n", typec_dir == CC1_ATTACH ? "cc1 attach" : "cc2_attach");
		return typec_dir;
	}
}
EXPORT_SYMBOL(oplus_get_typec_cc_orientation);

int oplus_thermal_tmp_get_chg(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return 0;

	if (chg->chgic_ops->thermal_tmp_get_chg)
		return chg->chgic_ops->thermal_tmp_get_chg();

	return 0;
}
EXPORT_SYMBOL(oplus_thermal_tmp_get_chg);

int oplus_thermal_tmp_get_bb(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return 0;

	if (chg->chgic_ops->thermal_tmp_get_bb)
		return chg->chgic_ops->thermal_tmp_get_bb();

	return 0;
}
EXPORT_SYMBOL(oplus_thermal_tmp_get_bb);

int oplus_thermal_tmp_get_flash(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return 0;

	if (chg->chgic_ops->thermal_tmp_get_flash)
		return chg->chgic_ops->thermal_tmp_get_flash();

	return 0;
}
EXPORT_SYMBOL(oplus_thermal_tmp_get_flash);

int oplus_thermal_tmp_get_board(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return 0;

	if (chg->chgic_ops->thermal_tmp_get_board)
		return chg->chgic_ops->thermal_tmp_get_board();

	return 0;
}
EXPORT_SYMBOL(oplus_thermal_tmp_get_board);

void oplus_chg_set_camera_on(bool val)
{
	return;
}
EXPORT_SYMBOL(oplus_chg_set_camera_on);

int oplus_get_usb_status(void)
{
	struct sy697x *chg = NULL;

	chg = oplus_get_chg_sy();
	if (!chg)
		return 0;

	if (chg->chgic_ops->get_usb_status)
		return chg->chgic_ops->get_usb_status();

	return 0;
}
EXPORT_SYMBOL(oplus_get_usb_status);

static int oplus_get_iio_channel(struct sy697x *chip, const char *propname,
					struct iio_channel **chan)
{
	int rc;

	rc = of_property_match_string(chip->dev->of_node,
					"io-channel-names", propname);
	if (rc < 0)
		return rc;

	*chan = iio_channel_get(chip->dev, propname);
	if (IS_ERR(*chan)) {
		rc = PTR_ERR(*chan);
		if (rc != -EPROBE_DEFER)
			pr_info(" %s channel unavailable, %d\n", propname, rc);
		*chan = NULL;
	}

	return rc;
}

#ifndef BATT_BTBTMP
#define BATT_BTBTMP 25000
#endif
#define BATT_BTBTMP_DEFAULT_DEGREE_VALUE 25
#define BATT_BTBTMP_RAW_VALUE_TO_DEGREE 1000

int oplus_chg_get_battery_btb_temp_cal(void)
{
	int batt_btbntc_raw = 0;
	struct sy697x *chg = oplus_get_chg_sy();
	static int batt_btbtmp = BATT_BTBTMP;
	static int batt_btbtmp_pre = BATT_BTBTMP;
	int rc;

	if (!chg) {
		chg_err("[%s]: chip or chg not ready!\n", __func__);
		return BATT_BTBTMP_DEFAULT_DEGREE_VALUE;
	}

	if (!chg->pinctrl ||
		!chg->iio.batt_btb_temp_chan) {
		chg_err("[%s]: chip not ready!\n", __func__);

		rc = oplus_get_iio_channel(chg, "quiet_therm", &chg->iio.batt_btb_temp_chan);
		if (rc < 0 && !chg->iio.batt_btb_temp_chan) {
			chg_err(" %s batt_btb_temp_chan get failed\n", __func__);
			return -1;
		}
	}

	if (IS_ERR_OR_NULL(chg->iio.batt_btb_temp_chan)) {
		chg_err("[%s]: chg->iio.batt_btb_temp_chan  is  NULL !\n", __func__);
		batt_btbtmp = batt_btbtmp_pre;
		goto btb_ntcvolt_get_done;
	}

	rc = iio_read_channel_processed(chg->iio.batt_btb_temp_chan, &batt_btbntc_raw);
	if (rc < 0) {
		chg_err(" fail to read usb_temp1 adc rc = %d\n", rc);
		batt_btbtmp = batt_btbtmp_pre;
		goto btb_ntcvolt_get_done;
	}
	if (batt_btbntc_raw <= 0) {
		chg_err("[%s]:batt_btbntc_raw iio_read_channel_processed  get error\n", __func__);
		batt_btbtmp = batt_btbtmp_pre;
		goto btb_ntcvolt_get_done;
	}

	chg_info("[%s]:batt_btbntc_raw[%d]\n", __func__, batt_btbntc_raw);
	batt_btbtmp = batt_btbntc_raw / BATT_BTBTMP_RAW_VALUE_TO_DEGREE;
	batt_btbtmp_pre = batt_btbntc_raw / BATT_BTBTMP_RAW_VALUE_TO_DEGREE;

btb_ntcvolt_get_done:
	return batt_btbtmp;
}
EXPORT_SYMBOL(oplus_chg_get_battery_btb_temp_cal);
