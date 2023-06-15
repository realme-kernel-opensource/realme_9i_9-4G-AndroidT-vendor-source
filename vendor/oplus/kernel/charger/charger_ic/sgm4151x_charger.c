/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include "sgm4151x_charger.h"
#include "../oplus_charger.h"

static struct sgm4151x_device *g_sgm_chg;
static DEFINE_MUTEX(sgm4151x_i2c_lock);
static int sgm4151x_set_input_suspend(struct sgm4151x_device *sgm_chg, int suspend);

static int sgm4151x_read_byte(struct sgm4151x_device *sgm_chg, u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&sgm4151x_i2c_lock);
	ret = i2c_smbus_read_byte_data(sgm_chg->client, reg);
	if (ret < 0) {
		chg_err("failed to read 0x%.2x\n", reg);
		mutex_unlock(&sgm4151x_i2c_lock);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&sgm4151x_i2c_lock);

	return 0;
}

static int sgm4151x_write_byte(struct sgm4151x_device *sgm_chg, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&sgm4151x_i2c_lock);
	ret = i2c_smbus_write_byte_data(sgm_chg->client, reg, data);
	mutex_unlock(&sgm4151x_i2c_lock);

	return ret;
}

static int sgm4151x_update_bits(struct sgm4151x_device *sgm_chg,
													u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = sgm4151x_read_byte(sgm_chg, &tmp, reg);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return sgm4151x_write_byte(sgm_chg, reg, tmp);
}

static int sgm4151x_chg_enable(struct sgm4151x_device *sgm_chg, bool en)
{
	gpio_direction_output(sgm_chg->chg_en_gpio, en);
	msleep(5);

	return 0;
}

static int sgm4151x_enable_otg(struct sgm4151x_device *sgm_chg)
{
	u8 val;

	val = SGM4151X_OTG_ENABLE << SGM4151X_OTG_CONFIG_SHIFT;

	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_01,
							   SGM4151X_OTG_CONFIG_MASK, val);
}

static int sgm4151x_disable_otg(struct sgm4151x_device *sgm_chg)
{
	u8 val;

	val = SGM4151X_OTG_DISABLE << SGM4151X_OTG_CONFIG_SHIFT;

	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_01,
							   SGM4151X_OTG_CONFIG_MASK, val);
}

static int sgm4151x_set_otg(struct sgm4151x_device *sgm_chg, int enable)
{
	int ret;

	if (enable) {
		ret = sgm4151x_enable_otg(sgm_chg);
		if (ret < 0) {
			chg_err("Failed to enable otg-%d\n", ret);
			return ret;
		}
	} else {
		ret = sgm4151x_disable_otg(sgm_chg);
		if (ret < 0) {
			chg_err("Failed to disable otg-%d\n", ret);
		}
	}

	return ret;
}

static int sgm4151x_enable_charger(struct sgm4151x_device *sgm_chg)
{
	int ret = 0;
	u8 val = SGM4151X_CHG_ENABLE << SGM4151X_CHG_CONFIG_SHIFT;

	ret = sgm4151x_update_bits(sgm_chg, SGM4151X_REG_01, SGM4151X_CHG_CONFIG_MASK, val);

	return ret;
}

static int oplus_sgm4151x_enable_charger(void)
{
	int ret = 0;
	u8 val = 0;
	if (!g_sgm_chg) {
		chg_err("%s g_sgm_chg is NULL\n", __func__);
		return ret;
	}
	sgm4151x_chg_enable(g_sgm_chg, true);
	val = SGM4151X_CHG_ENABLE << SGM4151X_CHG_CONFIG_SHIFT;

	ret = sgm4151x_update_bits(g_sgm_chg, SGM4151X_REG_01, SGM4151X_CHG_CONFIG_MASK, val);
	chg_err("%s\n", __func__);
	return ret;
}

static int sgm4151x_disable_charger(struct sgm4151x_device *sgm_chg)
{
	int ret = 0;
	u8 val = SGM4151X_CHG_DISABLE << SGM4151X_CHG_CONFIG_SHIFT;

	ret = sgm4151x_update_bits(sgm_chg, SGM4151X_REG_01, SGM4151X_CHG_CONFIG_MASK, val);

	return ret;
}


static int sgm4151x_set_chg_enable(struct sgm4151x_device *sgm_chg, int enable)
{
	int ret = 0;

	chg_err("set slave charge: enable: %d\n", enable);
	if (enable) {
		sgm4151x_chg_enable(sgm_chg, true);
		sgm4151x_enable_charger(sgm_chg);
	} else {
		sgm4151x_chg_enable(sgm_chg, false);
		sgm4151x_disable_charger(sgm_chg);
	}

	return ret;
}

static int oplus_sgm4151x_disable_charger(void)
{
	int ret = 0;
	u8 val = 0;
	if (!g_sgm_chg) {
		chg_err("%s g_sgm_chg is NULL\n", __func__);
		return ret;
	}
	val = SGM4151X_CHG_DISABLE << SGM4151X_CHG_CONFIG_SHIFT;
	ret = sgm4151x_update_bits(g_sgm_chg, SGM4151X_REG_01, SGM4151X_CHG_CONFIG_MASK, val);

	sgm4151x_chg_enable(g_sgm_chg, false);
	chg_debug("oplus_sgm4151x_disable_charger\n");
	return ret;
}

static int sgm4151x_set_charge_current(struct sgm4151x_device *sgm_chg, int curr)
{
	u8 ichg;
	u8 val;

	chg_debug("set slave charge: chg current: %d\n", curr);
	ichg = (curr - SGM4151X_ICHG_BASE)/SGM4151X_ICHG_LSB;
	val = ichg << SGM4151X_ICHG_SHIFT;

	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_02, SGM4151X_ICHG_MASK, val);
}

static int oplus_sgm4151x_set_charge_current(int curr)
{
	u8 ichg;
	u8 val;

	chg_err("%s set slave charge: chg current: %d\n", __func__, curr);
	ichg = (curr - SGM4151X_ICHG_BASE)/SGM4151X_ICHG_LSB;
	val = ichg << SGM4151X_ICHG_SHIFT;

	return sgm4151x_update_bits(g_sgm_chg, SGM4151X_REG_02, SGM4151X_ICHG_MASK, val);
}

static int sgm4151x_set_term_current(struct sgm4151x_device *sgm_chg, int curr)
{
	u8 iterm;
	u8 val;

	chg_err("set slave charge: term current: %d\n", curr);
	iterm = (curr - SGM4151X_ITERM_BASE) / SGM4151X_ITERM_LSB;
	val = iterm << SGM4151X_ITERM_SHIFT;

	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_03, SGM4151X_ITERM_MASK, val);
}

static int sgm4151x_set_prechg_current(struct sgm4151x_device *sgm_chg, int curr)
{
	u8 iprechg;
	u8 val;

	iprechg = (curr - SGM4151X_IPRECHG_BASE) / SGM4151X_IPRECHG_LSB;
	val = iprechg << SGM4151X_IPRECHG_SHIFT;

	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_03, SGM4151X_IPRECHG_MASK, val);
}

static int sgm4151x_set_input_current_limit(struct sgm4151x_device *sgm_chg, int curr)
{
	u8 val;
	u8 input_curr;

	chg_err("set slave charge: input current: %d\n", curr);
	input_curr = (curr - SGM4151X_IINLIM_BASE) / SGM4151X_IINLIM_LSB;
	val = input_curr << SGM4151X_IINLIM_SHIFT;

	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_00, SGM4151X_IINLIM_MASK, val);
}

static int oplus_sgm4151x_set_input_current_limit(int curr)
{
	u8 val;
	u8 input_curr;

	chg_err("%s set slave charge: input current: %d\n", __func__, curr);
	input_curr = (curr - SGM4151X_IINLIM_BASE) / SGM4151X_IINLIM_LSB;
	val = input_curr << SGM4151X_IINLIM_SHIFT;

	return sgm4151x_update_bits(g_sgm_chg, SGM4151X_REG_00, SGM4151X_IINLIM_MASK, val);
}

static int sgm4151x_set_vac_ovp(struct sgm4151x_device *sgm_chg, int ovp)
{
	int val;

	if (ovp <= SGM4151x_OVP_5000mV) {
		val = SGM4151x_OVP_5500mV;
	} else if (ovp > SGM4151x_OVP_5000mV && ovp <= SGM4151x_OVP_6000mV) {
		val = SGM4151x_OVP_6500mV;
	} else if (ovp > SGM4151x_OVP_6000mV && ovp <= SGM4151x_OVP_8000mV) {
		val = SGM4151x_OVP_10500mV;
	} else {
		val = SGM4151x_OVP_14000mV;
	}

	val = val << SGM4151X_OVP_SHIFT;
	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_06, SGM4151X_OVP_MASK, val);
}

static int sgm4151x_set_chargevoltage(struct sgm4151x_device *sgm_chg, int volt)
{
	u8 val;
	u8 vchg;

	chg_err("set slave charge: voltage: %d\n", volt);
	vchg = (u8)((volt - SGM4151X_VREG_BASE)/SGM4151X_VREG_LSB);
	val = vchg << SGM4151X_VREG_SHIFT;

	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_04, SGM4151X_VREG_MASK, val);
}

static int sgm4151x_get_charging_status(struct sgm4151x_device *sgm_chg)
{
	u8 val = 0;
	int ret;

	ret = sgm4151x_read_byte(sgm_chg, &val, SGM4151X_REG_08);
	if (ret < 0) {
		chg_err("Failed to read register 0x08:%d\n", ret);
		return ret;
	}

	val &= SGM4151X_CHRG_STAT_MASK;
	val >>= SGM4151X_CHRG_STAT_SHIFT;

	return val;
}
EXPORT_SYMBOL_GPL(sgm4151x_get_charging_status);

static int oplus_sgm4151x_get_charging_status(void)
{
	u8 val = 0;
	int ret;

	ret = sgm4151x_read_byte(g_sgm_chg, &val, SGM4151X_REG_08);
	if (ret < 0) {
		chg_err("Failed to read register 0x08:%d\n", ret);
		return ret;
	}

	val &= SGM4151X_CHRG_STAT_MASK;
	val >>= SGM4151X_CHRG_STAT_SHIFT;

	return val;
}
EXPORT_SYMBOL_GPL(oplus_sgm4151x_get_charging_status);


static int sgm4151x_enable_term(struct sgm4151x_device *sgm_chg, bool enable)
{
	u8 val;
	int ret;

	if (enable) {
		val = SGM4151X_TERM_ENABLE << SGM4151X_EN_TERM_SHIFT;
	} else {
		val = SGM4151X_TERM_DISABLE << SGM4151X_EN_TERM_SHIFT;
	}

	ret = sgm4151x_update_bits(sgm_chg, SGM4151X_REG_05, SGM4151X_EN_TERM_MASK, val);

	return ret;
}

static int sgm4151x_disable_watchdog_timer(struct sgm4151x_device *sgm_chg)
{
	u8 val;

	val = SGM4151X_WDT_DISABLE << SGM4151X_WDT_SHIFT;

	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_05, SGM4151X_WDT_MASK, val);
}

static int sgm4151x_reset_chip(struct sgm4151x_device *sgm_chg)
{
	u8 val;
	int ret;

	val = SGM4151X_RESET << SGM4151X_RESET_SHIFT;

	ret = sgm4151x_update_bits(sgm_chg, SGM4151X_REG_0B, SGM4151X_RESET_MASK, val);

	return ret;
}


static int sgm4151x_enter_hiz_mode(struct sgm4151x_device *sgm_chg)
{
	u8 val;

	val = SGM4151X_HIZ_ENABLE << SGM4151X_ENHIZ_SHIFT;

	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_00, SGM4151X_ENHIZ_MASK, val);
}

static int oplus_sgm4151x_enter_hiz_mode(void)
{
	u8 val;

	val = SGM4151X_HIZ_ENABLE << SGM4151X_ENHIZ_SHIFT;

	return sgm4151x_update_bits(g_sgm_chg, SGM4151X_REG_00, SGM4151X_ENHIZ_MASK, val);
}

static int sgm4151x_exit_hiz_mode(struct sgm4151x_device *sgm_chg)
{
	u8 val;

	val = SGM4151X_HIZ_DISABLE << SGM4151X_ENHIZ_SHIFT;

	return sgm4151x_update_bits(sgm_chg, SGM4151X_REG_00, SGM4151X_ENHIZ_MASK, val);
}

static int oplus_sgm4151x_exit_hiz_mode(void)
{
	u8 val;

	val = SGM4151X_HIZ_DISABLE << SGM4151X_ENHIZ_SHIFT;

	return sgm4151x_update_bits(g_sgm_chg, SGM4151X_REG_00, SGM4151X_ENHIZ_MASK, val);
}

int oplus_sgm4151x_set_cv(int cur)
{
	return sgm4151x_set_chargevoltage(g_sgm_chg, cur);
}
static int sgm4151x_set_input_suspend(struct sgm4151x_device *sgm_chg, int suspend)
{
	int  ret = 0;

	chg_err("set input suspend: %d\n", suspend);
	if (suspend) {
		sgm4151x_enter_hiz_mode(sgm_chg);
	} else {
		sgm4151x_exit_hiz_mode(sgm_chg);
	}

	return ret;
}

static int sgm4151x_get_chg_status(struct sgm4151x_device *sgm_chg)
{
	int ret;
	u8 status = 0;
	u8 charge_status = 0;

	/* Read STATUS registers */
	ret = sgm4151x_read_byte(sgm_chg, &status, SGM4151X_REG_0B);
	if (ret) {
		chg_err("read regs:0x0b fail !\n");
		return sgm_chg->chg_status;
	}

	charge_status = (status & SGM4151X_CHRG_STAT_MASK) >> SGM4151X_CHRG_STAT_SHIFT;
	switch (charge_status) {
	case SGM4151X_NOT_CHARGING:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case SGM4151X_PRE_CHARGE:
	case SGM4151X_FAST_CHARGING:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case SGM4151X_CHARGE_DONE:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return sgm_chg->chg_status;
}

static void sgm4151x_dump_regs(struct sgm4151x_device *sgm_chg)
{
	int addr, ret;
	u8 val;

	for (addr = SGM4151X_REG_00; addr <= SGM4151X_REG_0B; addr++) {
		ret = sgm4151x_read_byte(sgm_chg, &val, addr);
		if (ret == 0) {
			chg_err("Reg[%.2x] = 0x%.2x \n", addr, val);
		}
	}
}

static void oplus_sgm4151x_dump_regs(void)
{
	int addr, ret;
	u8 val[25];
	char buf[BUF_NUM];
	char *regs_s = buf;

	if (!g_sgm_chg) {
		chg_err("%s g_sgm_chg is NULL\n", __func__);
		return;
	}
	for (addr = SGM4151X_REG_00; addr <= SGM4151X_REG_0B; addr++) {
		ret = sgm4151x_read_byte(g_sgm_chg, &val[addr], addr);
	}
	regs_s += sprintf(regs_s, "sgm4151x_dump_regs:");
	for (addr = SGM4151X_REG_00; addr <= SGM4151X_REG_0B; addr++) {
		regs_s += sprintf(regs_s, "[0x%.2x,0x%.2x]", addr, val[addr]);
	}
	regs_s += sprintf(regs_s, "\n");

	chg_err("%s", buf);
}

static int sgm4151x_detect_device(struct sgm4151x_device *sgm_chg)
{
	int ret;
	u8 data;

	ret = sgm4151x_read_byte(sgm_chg, &data, SGM4151X_REG_0B);
	if (ret == 0) {
		sgm_chg->part_no = (data & SGM4151X_PN_MASK) >> SGM4151X_PN_SHIFT;
		sgm_chg->revision = (data & SGM4151X_DEV_REV_MASK) >> SGM4151X_DEV_REV_SHIFT;
	}

	return ret;
}

static int sgm4151x_init_device(struct sgm4151x_device *sgm_chg)
{
	int ret;

	sgm4151x_reset_chip(sgm_chg);
	msleep(50);

	/*common initialization*/
	ret = sgm4151x_set_term_current(sgm_chg, sgm_chg->cfg.term_current);
	if (ret < 0) {
		chg_err("Failed to set termination current:%d\n", ret);
		return ret;
	}

	ret = sgm4151x_set_chargevoltage(sgm_chg, sgm_chg->cfg.charge_voltage);
	if (ret < 0) {
		chg_err("Failed to set charge voltage:%d\n",  ret);
		return ret;
	}

	ret = sgm4151x_set_charge_current(sgm_chg, sgm_chg->cfg.charge_current);
	if (ret < 0) {
		chg_err("Failed to set charge current:%d\n", ret);
		return ret;
	}

	ret = sgm4151x_set_prechg_current(sgm_chg, sgm_chg->cfg.prechg_current);
	if (ret < 0) {
		chg_err("Failed to set charge current:%d\n", ret);
		return ret;
	}

	sgm4151x_set_vac_ovp(sgm_chg, SGM4151x_OVP_10000mV);
	sgm4151x_enable_term(sgm_chg, false);
	sgm4151x_set_otg(sgm_chg, false);
	ret = sgm4151x_disable_charger(sgm_chg);
	if (ret < 0) {
		chg_err("Failed to disable charger:%d\n", ret);
		return ret;
	}

	sgm4151x_set_chg_enable(sgm_chg, false);
	sgm4151x_disable_watchdog_timer(sgm_chg);
	sgm4151x_dump_regs(sgm_chg);

	return ret;
}

static ssize_t sgm4151x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 addr;
	u8 val;
	u8 tmpbuf[BUF_NUM];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "Charger 1");
	for (addr = 0x0; addr <= SGM4151X_REG_14; addr++) {
		ret = sgm4151x_read_byte(g_sgm_chg, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx, "Reg[0x%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static DEVICE_ATTR(registers, S_IRUGO, sgm4151x_show_registers, NULL);

static struct attribute *sgm4151x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group sgm4151x_attr_group = {
	.attrs = sgm4151x_attributes,
};

static void sgm4151x_monitor_workfunc(struct work_struct *work)
{
	int ret;
	u8 status = 0;
	u8 fault = 0;
	u8 charge_status = 0;
	struct sgm4151x_device *sgm_chg = container_of(work,
								struct sgm4151x_device, monitor_work.work);

	/* Read STATUS and FAULT registers */
	ret = sgm4151x_read_byte(sgm_chg, &status, SGM4151X_REG_08);
	if (ret) {
		chg_err("read regs:0x08 fail !\n");
		return;
	}

	ret = sgm4151x_read_byte(sgm_chg, &fault, SGM4151X_REG_09);
	if (ret) {
		chg_err("read regs:0x09 fail !\n");
		return;
	}

	sgm_chg->vbus_type = (status & SGM4151X_VBUS_STAT_MASK) >> SGM4151X_VBUS_STAT_SHIFT;
	sgm_chg->usb_online = (status & SGM4151X_PG_STAT_MASK) >> SGM4151X_PG_STAT_SHIFT;
	charge_status = (status & SGM4151X_CHRG_STAT_MASK) >> SGM4151X_CHRG_STAT_SHIFT;
	switch (charge_status) {
	case SGM4151X_NOT_CHARGING:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case SGM4151X_PRE_CHARGE:
	case SGM4151X_FAST_CHARGING:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case SGM4151X_CHARGE_DONE:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	sgm4151x_dump_regs(sgm_chg);
	chg_err("%s:usb_online: %d, chg_status: %d, chg_fault: 0x%x\n",
							__func__, sgm_chg->usb_online, sgm_chg->chg_status, fault);

	schedule_delayed_work(&sgm_chg->monitor_work, msecs_to_jiffies(DELAY_TIME_IFFIES));
}

static void sgm4151x_set_charger_work(struct sgm4151x_device *sgm_chg, int work)
{
	if (work) {
		cancel_delayed_work_sync(&sgm_chg->monitor_work);
		schedule_delayed_work(&sgm_chg->monitor_work, 0);
	} else {
		cancel_delayed_work_sync(&sgm_chg->monitor_work);
	}
}

static void sgm4151x_charger_irq_workfunc(struct work_struct *work)
{
	int ret;
	u8 status = 0;
	u8 chg_fault = 0;
	u8 vindpm_status;
	u8 charge_status = 0;
	struct sgm4151x_device *sgm_chg = container_of(work,
								struct sgm4151x_device, irq_work.work);

	/* Read STATUS and FAULT registers */
	msleep(5);
	ret = sgm4151x_read_byte(sgm_chg, &status, SGM4151X_REG_08);
	if (ret) {
		chg_err("read regs:0x08 fail !\n");
		return;
	}

	ret = sgm4151x_read_byte(sgm_chg, &chg_fault, SGM4151X_REG_09);
	if (ret) {
		chg_err("read regs:0x09 fail !\n");
		return;
	}

	ret = sgm4151x_read_byte(sgm_chg, &vindpm_status, SGM4151X_REG_0A);
	if (ret) {
		chg_err("read regs:0x0A fail !\n");
		return;
	}

	sgm_chg->vbus_type = (status & SGM4151X_VBUS_STAT_MASK) >> SGM4151X_VBUS_STAT_SHIFT;
	sgm_chg->usb_online = (status & SGM4151X_PG_STAT_MASK) >> SGM4151X_PG_STAT_SHIFT;
	charge_status = (status & SGM4151X_CHRG_STAT_MASK) >> SGM4151X_CHRG_STAT_SHIFT;
	switch (charge_status) {
	case SGM4151X_NOT_CHARGING:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case SGM4151X_PRE_CHARGE:
	case SGM4151X_FAST_CHARGING:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case SGM4151X_CHARGE_DONE:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		sgm_chg->chg_status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	chg_err("%s:usb_online: %d, chg_status: %d, chg_fault: 0x%x, vindpm_status: 0x%x\n",
							__func__, sgm_chg->usb_online,
							sgm_chg->chg_status, chg_fault, vindpm_status);
}
/*
static irqreturn_t sgm4151x_charger_interrupt(int irq, void *data)
{
	struct sgm4151x_device *sgm_chg = data;

	schedule_delayed_work(&sgm_chg->irq_work, 0);
	return IRQ_HANDLED;
}
*/

/************************
 * USB PSY REGISTRATION *
 ************************/
static enum power_supply_property sgm4151x_usb_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_APSD_RERUN,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_UPDATE_NOW,
};

static int sgm4151x_usb_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct sgm4151x_device *sgm_chg = power_supply_get_drvdata(psy);
	int ret = 0;
	val->intval = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = sgm4151x_get_chg_status(sgm_chg);
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sgm_chg->usb_online;
		break;

	case POWER_SUPPLY_PROP_TYPE:
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = sgm_chg->vbus_type;
		break;

	default:
		chg_err("get prop %d is not supported in usb\n", psp);
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		chg_err("Couldn't get prop %d rc = %d\n", psp, ret);
		return -ENODATA;
	}

	return 0;
}


static int sgm4151x_usb_set_prop(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct sgm4151x_device *sgm_chg = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_DP_DM:
		sgm4151x_set_charger_work(sgm_chg, val->intval);
		break;

	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		sgm4151x_set_input_suspend(sgm_chg, val->intval);
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		sgm4151x_set_input_current_limit(sgm_chg, val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		sgm4151x_set_chg_enable(sgm_chg, val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		sgm4151x_set_chargevoltage(sgm_chg, val->intval);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		sgm4151x_set_charge_current(sgm_chg, val->intval);
		break;

	default:
		chg_err("set prop: %d is not supported\n", psp);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int sgm4151x_usb_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_APSD_RERUN:
	case POWER_SUPPLY_PROP_DP_DM:
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		return 1;

	default:
		break;
	}

	return 0;
}

static const struct power_supply_desc usb_psy_desc = {
	.name = "slave_chg",
	.type = POWER_SUPPLY_TYPE_MAIN,
	.properties = sgm4151x_usb_props,
	.num_properties = ARRAY_SIZE(sgm4151x_usb_props),
	.get_property = sgm4151x_usb_get_prop,
	.set_property = sgm4151x_usb_set_prop,
	.property_is_writeable = sgm4151x_usb_prop_is_writeable,
};

static int sgm4151x_init_usb_psy(struct sgm4151x_device *sgm_chg)
{
	struct power_supply_config wall_cfg = {};

	wall_cfg.drv_data = sgm_chg;
	wall_cfg.of_node = sgm_chg->dev->of_node;
	sgm_chg->wall_psy = devm_power_supply_register(sgm_chg->dev,
						  &usb_psy_desc,
						  &wall_cfg);
	if (IS_ERR(sgm_chg->wall_psy)) {
		chg_err("Couldn't register slave chg power supply\n");
		return PTR_ERR(sgm_chg->wall_psy);
	}

	return 0;
}

static int sgm4151x_parse_dt(struct device *dev, struct sgm4151x_device *sgm_chg)
{
	int ret;
	struct device_node *np = dev->of_node;

	sgm_chg->cfg.enable_auto_dpdm = of_property_read_bool(np, "sgm4151x,enable-auto-dpdm");
	sgm_chg->cfg.enable_term = of_property_read_bool(np, "sgm4151x,enable-termination");
	sgm_chg->cfg.enable_ico = of_property_read_bool(np, "sgm4151x,enable-ico");
	sgm_chg->cfg.use_absolute_vindpm = of_property_read_bool(np, "sgm4151x,use-absolute-vindpm");

	ret = of_property_read_u32(np, "sgm4151x,charge-voltage", &sgm_chg->cfg.charge_voltage);
	if (ret) {
		sgm_chg->cfg.charge_voltage = DEFAULT_BATT_CV;
	} else {
		chg_err("charge_voltage: %d\n", sgm_chg->cfg.charge_voltage);
	}

	ret = of_property_read_u32(np, "sgm4151x,charge-current", &sgm_chg->cfg.charge_current);
	if (ret) {
		sgm_chg->cfg.charge_current = DEFAULT_CHG_CURR;
	} else {
		chg_err("charge_current: %d\n", sgm_chg->cfg.charge_current);
	}

	ret = of_property_read_u32(np, "sgm4151x,term-current", &sgm_chg->cfg.term_current);
	if (ret) {
		sgm_chg->cfg.term_current = DEFAULT_TERM_CURR;
	} else {
		chg_err("term-curren: %d\n", sgm_chg->cfg.term_current);
	}

	ret = of_property_read_u32(np, "sgm4151x,prechg_current", &sgm_chg->cfg.prechg_current);
	if (ret) {
		sgm_chg->cfg.prechg_current = DEFAULT_PRECHG_CURR;
	} else {
		chg_err("prechg_current: %d\n", sgm_chg->cfg.prechg_current);
	}

	sgm_chg->pinctrl = devm_pinctrl_get(sgm_chg->dev);
	if (!IS_ERR_OR_NULL(sgm_chg->pinctrl)) {
		sgm_chg->gpio_default =
			pinctrl_lookup_state(sgm_chg->pinctrl, "default");
		if (!IS_ERR_OR_NULL(sgm_chg->gpio_default)) {
			pinctrl_select_state(sgm_chg->pinctrl, sgm_chg->gpio_default);
		} else {
			chg_err("[%s]sgm_chg->gpio_default  is NULL\n", __func__);
		}
	} else {
		chg_err("[%s]sgm_chg->pinctrl is NULl\n" , __func__);
	}

	sgm_chg->chg_en_gpio = of_get_named_gpio(np, "chg-en-gpio", 0);
	if (ret < 0) {
		chg_err("%s no chg_en_gpio info\n", __func__);
	} else {
		gpio_direction_output(sgm_chg->chg_en_gpio, 1);
	}
	sgm_chg->irq_gpio = of_get_named_gpio(np, "intr-gpio", 0);
	if (ret < 0) {
		chg_err("%s get intr_gpio fail !\n", __func__);
	} else {
		chg_err("%s intr_gpio info %d\n", __func__, sgm_chg->irq_gpio);
	}

	return 0;
}

struct oplus_chg_operations  oplus_chg_sgm4151x_ops = {
	.dump_registers = oplus_sgm4151x_dump_regs,
	.charging_current_write_fast = oplus_sgm4151x_set_charge_current,
	.input_current_write = oplus_sgm4151x_set_input_current_limit,
	.charging_enable = oplus_sgm4151x_enable_charger,
	.charging_disable = oplus_sgm4151x_disable_charger,
	.get_charging_enable = oplus_sgm4151x_get_charging_status,
	.charger_suspend = oplus_sgm4151x_enter_hiz_mode,
	.charger_unsuspend = oplus_sgm4151x_exit_hiz_mode,
        .float_voltage_write = oplus_sgm4151x_set_cv,
};

static int sgm4151x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret;
	int irqn;
	struct sgm4151x_device *sgm_chg;

	sgm_chg = devm_kzalloc(&client->dev, sizeof(struct sgm4151x_device), GFP_KERNEL);
	if (!sgm_chg) {
		chg_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	sgm_chg->dev = &client->dev;
	sgm_chg->client = client;
	i2c_set_clientdata(client, sgm_chg);

	ret = sgm4151x_detect_device(sgm_chg);
	if (!ret && sgm_chg->part_no == SGM41511) {
		sgm_chg->is_sgm41511 = true;
		chg_err("charger:sgm41511 detected, part_no: %d, revision:%d\n",
						sgm_chg->part_no, sgm_chg->revision);
	} else if (!ret && sgm_chg->part_no == SY6974) {
		sgm_chg->is_sgm41511 = false;
		chg_err("charger: sy6974 detected, part_no: %d, revision:%d\n",
						sgm_chg->part_no, sgm_chg->revision);
	} else {
		chg_err("no sgm41511 charger device found:%d\n",  ret);
		return -ENODEV;
	}

	g_sgm_chg = sgm_chg;
	ret = sgm4151x_init_usb_psy(sgm_chg);
	if (ret < 0) {
		chg_err("Couldn't initialize slave charge usb psy: %d\n", ret);
		goto err_1;
	}

	if (client->dev.of_node)
		sgm4151x_parse_dt(&client->dev, sgm_chg);

	ret = sgm4151x_init_device(sgm_chg);
	if (ret) {
		chg_err("device init failure: %d\n", ret);
		goto err_0;
	}
	ret = gpio_request(sgm_chg->irq_gpio, "sgm4151x irq pin");
	if (ret) {
		chg_err("%d gpio request failed\n", sgm_chg->irq_gpio);
	}

	ret = gpio_request(sgm_chg->chg_en_gpio, "chg_en_gpio");
	if (ret) {
		chg_err("%d chg_en_gpio request failed\n",
						sgm_chg->chg_en_gpio);
	}

	irqn = gpio_to_irq(sgm_chg->irq_gpio);
	if (irqn < 0) {
		chg_err("%d gpio_to_irq failed\n", irqn);
		ret = irqn;
		goto err_0;
	}
	client->irq = irqn;

	INIT_DELAYED_WORK(&sgm_chg->irq_work, sgm4151x_charger_irq_workfunc);
	INIT_DELAYED_WORK(&sgm_chg->monitor_work, sgm4151x_monitor_workfunc);
	ret = sysfs_create_group(&sgm_chg->dev->kobj, &sgm4151x_attr_group);
	if (ret) {
		chg_err("failed to register sysfs. err: %d\n", ret);
		goto err_irq;
	}
/*
	ret = request_irq(client->irq, sgm4151x_charger_interrupt,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"sgm4151x_charger1_irq", sgm_chg);
	if (ret) {
		chg_err("Request IRQ %d failed: %d\n", client->irq, ret);
		goto err_irq;
	} else {
		chg_err("Request irq = %d\n", client->irq);
	}

	enable_irq_wake(irqn);
*/
	return 0;

err_irq:
	cancel_delayed_work_sync(&sgm_chg->irq_work);
	cancel_delayed_work_sync(&sgm_chg->monitor_work);
err_1:
err_0:
	devm_kfree(&client->dev, sgm_chg);
	sgm_chg = NULL;
	g_sgm_chg = NULL;
	return ret;
}

static void sgm4151x_charger_shutdown(struct i2c_client *client)
{
	struct sgm4151x_device *sgm_chg = i2c_get_clientdata(client);

	chg_debug("shutdown\n");

	sysfs_remove_group(&sgm_chg->dev->kobj, &sgm4151x_attr_group);
	cancel_delayed_work_sync(&sgm_chg->irq_work);
	cancel_delayed_work_sync(&sgm_chg->monitor_work);

	free_irq(sgm_chg->client->irq, NULL);
}

static struct of_device_id sgm4151x_charger_match_table[] = {
	{.compatible = "sgmicro,sgm4151x", },
	{},
};


static const struct i2c_device_id sgm4151x_charger_id[] = {
	{ "sgm4151x", SGM41511 },
	{},
};

MODULE_DEVICE_TABLE(i2c, sgm4151x_charger_id);

static struct i2c_driver sgm4151x_charger_driver = {
	.driver		= {
		.name	= "sgm4151x",
		.of_match_table = sgm4151x_charger_match_table,
	},
	.id_table	= sgm4151x_charger_id,

	.probe		= sgm4151x_charger_probe,
	.shutdown   = sgm4151x_charger_shutdown,
};

module_i2c_driver(sgm4151x_charger_driver);

MODULE_DESCRIPTION("SGM4151x Charger Driver");
MODULE_LICENSE("GPL");
