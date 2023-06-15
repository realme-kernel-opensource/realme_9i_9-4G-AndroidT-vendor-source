// SPDX-License-Identifier: GPL-2.0-only
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
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/iio/consumer.h>
#include <linux/pm_wakeup.h>
#include <linux/rtc.h>
#include <linux/time.h>

#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <soc/oplus/boot_mode.h>

#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"
#include "../oplus_short.h"
#include "../oplus_adapter.h"
#include "../charger_ic/oplus_short_ic.h"
#include "../gauge_ic/oplus_bq27541.h"
#include "../voocphy/oplus_voocphy.h"
#include "../oplus_configfs.h"

#include "oplus_sy697x.h"
#include "oplus_sy6974b.h"

enum hvdcp_type {
	HVDCP_5V,
	HVDCP_9V,
	HVDCP_12V,
	HVDCP_20V,
	HVDCP_CONTINOUS,
	HVDCP_DPF_DMF,
};

#define DEFAULT_CV 4435

#ifndef USB_TEMP_HIGH
#define USB_TEMP_HIGH 0x01
#endif

extern int chg_init_done;
extern int typec_dir;

#define NORMAL_THERMAL_TEMP_DEFAULT	25000
static int chg_thermal_temp = NORMAL_THERMAL_TEMP_DEFAULT;
static int bb_thermal_temp = NORMAL_THERMAL_TEMP_DEFAULT;
static int flash_thermal_temp = NORMAL_THERMAL_TEMP_DEFAULT;
static int board_thermal_temp = NORMAL_THERMAL_TEMP_DEFAULT;

/* ntc table */
static int con_temp_855[] = {
	-20,
	-19,
	-18,
	-17,
	-16,
	-15,
	-14,
	-13,
	-12,
	-11,
	-10,
	-9,
	-8,
	-7,
	-6,
	-5,
	-4,
	-3,
	-2,
	-1,
	0,
	1,
	2,
	3,
	4,
	5,
	6,
	7,
	8,
	9,
	10,
	11,
	12,
	13,
	14,
	15,
	16,
	17,
	18,
	19,
	20,
	21,
	22,
	23,
	24,
	25,
	26,
	27,
	28,
	29,
	30,
	31,
	32,
	33,
	34,
	35,
	36,
	37,
	38,
	39,
	40,
	41,
	42,
	43,
	44,
	45,
	46,
	47,
	48,
	49,
	50,
	51,
	52,
	53,
	54,
	55,
	56,
	57,
	58,
	59,
	60,
	61,
	62,
	63,
	64,
	65,
	66,
	67,
	68,
	69,
	70,
	71,
	72,
	73,
	74,
	75,
	76,
	77,
	78,
	79,
	80,
	81,
	82,
	83,
	84,
	85,
	86,
	87,
	88,
	89,
	90,
	91,
	92,
	93,
	94,
	95,
	96,
	97,
	98,
	99,
	100,
	101,
	102,
	103,
	104,
	105,
	106,
	107,
	108,
	109,
	110,
	111,
	112,
	113,
	114,
	115,
	116,
	117,
	118,
	119,
	120,
	121,
	122,
	123,
	124,
	125,
};

static int con_volt_855[] = {
	1725,
	1716,
	1707,
	1697,
	1687,
	1677,
	1666,
	1655,
	1643,
	1631,
	1618,
	1605,
	1591,
	1577,
	1562,
	1547,
	1531,
	1515,
	1499,
	1482,
	1465,
	1447,
	1429,
	1410,
	1391,
	1372,
	1352,
	1332,
	1311,
	1291,
	1270,
	1248,
	1227,
	1205,
	1183,
	1161,
	1139,
	1117,
	1094,
	1072,
	1049,
	1027,
	1004,
	982,
	960,
	938,
	915,
	893,
	872,
	850,
	829,
	808,
	787,
	766,
	746,
	726,
	706,
	687,
	668,
	649,
	631,
	613,
	595,
	578,
	561,
	544,
	528,
	512,
	497,
	482,
	467,
	453,
	439,
	426,
	412,
	400,
	387,
	375,
	363,
	352,
	341,
	330,
	320,
	310,
	300,
	290,
	281,
	272,
	264,
	255,
	247,
	239,
	232,
	224,
	217,
	210,
	204,
	197,
	191,
	185,
	179,
	174,
	168,
	163,
	158,
	153,
	148,
	143,
	139,
	135,
	131,
	126,
	123,
	119,
	115,
	112,
	108,
	105,
	102,
	99,
	96,
	93,
	90,
	87,
	85,
	82,
	80,
	78,
	75,
	73,
	71,
	69,
	67,
	65,
	63,
	61,
	60,
	58,
	56,
	55,
	53,
	52,
	50,
	49,
	47,
	46,
};

#define OPLUS_DELAY_WORK_TIME_BASE        round_jiffies_relative(msecs_to_jiffies(100))
#define OPLUS_BC12_RETRY_TIME        round_jiffies_relative(msecs_to_jiffies(200))
#define OPLUS_BC12_RETRY_TIME_CDP        round_jiffies_relative(msecs_to_jiffies(400))

#define OPLUS_TYPEC_ATTACH_CC1	1
#define OPLUS_TYPEC_ATTACH_CC2	2
#define OPLUS_TYPEC_DETACH	0
#define OPLUS_TYPEC_SRC_DFP	1
#define OPLUS_TYPEC_SNK_UFP	2
#define OPLUS_TYPEC_ACCESSORY	3

static bool disable_qc;
static bool dumpreg_by_irq;
static int  current_percent = 70;

static struct sy697x *g_sy;
static struct task_struct *oplushg_usbtemp_kthread;
static int sy6974b_get_input_current(void);
static int oplus_sy6974b_hardware_init(void);
static int oplus_sy6974b_charger_suspend(void);
static int oplus_sy6974b_charger_unsuspend(void);

extern int oplus_usbtemp_monitor_common(void *data);
extern bool oplus_chg_wake_update_work(void);

static bool is_bq25890h(struct sy697x *sy);

#define META_BOOT	0
static int oplus_sy6974b_get_vbus(void);
static void oplus_sy6974b_dump_registers(void);
static int oplus_register_extcon(struct sy697x *chip);
static inline bool is_usb_rdy() { return true; }
extern struct oplus_chg_chip *g_oplus_chip;

extern void cpuboost_charge_event(int flag);
extern void oplus_chg_set_chargerid_switch_val(int value);
extern int oplus_vooc_get_adapter_update_real_status(void);
extern void oplus_chg_set_chargerid_switch_val(int value);
static void sy6974b_dump_regs(struct sy697x *sy);
static int sy6974b_enable_enlim(struct sy697x *sy);
static int oplus_sy6974b_set_aicr(int current_ma);
static bool oplus_get_otg_enable(void);
static int oplus_sy6974b_enable_otg(void);
static int oplus_sy6974b_disable_otg(void);
extern bool oplus_voocphy_fastchg_ing(void);
static int oplus_sy6974b_get_charger_type(void);
static int qpnp_get_prop_charger_voltage_now(void);
static bool oplus_chg_is_usb_present(void);
static int sy6974b_set_iindet(bool enable);
static int sy6974b_get_iindet(void);
static void sy6974b_get_bc12(struct sy697x *chip);
static void oplus_sy6974b_check_ic_suspend(void);
static int oplus_sy6974b_charging_enable(void);

static bool oplus_ccdetect_check_is_gpio(struct oplus_chg_chip *chip);
static int oplus_ccdetect_gpio_init(struct oplus_chg_chip *chip);
static void oplus_ccdetect_irq_init(struct oplus_chg_chip *chip);
static void oplus_ccdetect_disable(void);
static void oplus_ccdetect_enable(void);
bool oplus_sy6974b_get_otg_switch_status(void);
void oplus_sy6974b_set_otg_switch_status(bool value);
static bool oplus_ccdetect_support_check(void);
static int oplus_chg_ccdetect_parse_dt(struct oplus_chg_chip *chip);
static int oplus_sy6974b_get_otg_online_status(void);
static bool oplus_get_otg_online_status_default(void);
static irqreturn_t oplus_ccdetect_change_handler(int irq, void *data);

#define OPLUS_MODE_DRP	1
#define OPLUS_MODE_SNK	0
extern int oplus_sgm7220_set_mode(int mode);
extern void oplus_usbtemp_recover_func(struct oplus_chg_chip *chip);
extern int oplus_voocphy_get_cp_vbus(void);

extern int get_usb_enum_status(void);
static void usb_enum_check(struct work_struct *work);
static void start_usb_enum_check(void);
static void stop_usb_enum_check(void);
static int oplus_get_iio_channel(struct sy697x *chip, const char *propname,
					struct iio_channel **chan);

static int force_dcp;
static int usb_enum_check_status;

#define CHARGER_VOLTAGE_DEFAULT 5000
#define CHARGER_VOLTAGE_7600MV	7600
#define CHARGER_VOLTAGE_5800MV	5800

#define SY6974B_INPUT_VOLT_LIMIT_DEFAULT_VALUE	4400

#define SY6974B_READ_VBAT_DEFAULT	4000
#define SY6974B_READ_VBUS_DEFAULT	5000
#define SY6974B_READ_TEMPERATURE_DEFAULT	2700
#define SY6974B_READ_CHARGE_CURRENT_DEFAULT	1540

#define SY6974B_VINDPM_DEFAULT	4500
#define SY6974B_INPUT_CURRENT_LIMIT_DEFAULT	2000
#define SY6974B_FAST_CHARGE_CURRENT_LIMIT_DEFAULT	2000
#define SY6974B_CV_DEFAULT	4200
#define SY6974B_PRECHARGE_CURRENT_LIMIT_DEFAULT	256
#define SY6974B_TERMINATION_CURRENT_LIMIT_DEFAULT	250
#define SY6974B_BOOST_REGULATION_VOLTAGE_DEFAULT	5000
#define SY6974B_BOOST_MODE_CURRENT_LIMIT_DEFAULT	1200

#define SY6974B_SET_MIVR_BASE_BATT_VOL_4300MV	4300
#define SY6974B_SET_MIVR_BASE_BATT_VOL_OFFSET_400MV	400
#define SY6974B_SET_MIVR_BASE_BATT_VOL_OFFSET_300MV	300
#define SY6974B_SET_MIVR_BASE_BATT_VOL_OFFSET_200MV	200
#define SY6974B_SET_MIVR_BASE_BATT_VOL_4200MV	4200

#define SY6974B_AICL_POINT_BASE_BATT_VOL_4100MV	4100
#define SY6974B_AICL_POINT_VOL_5V_PHASE1	4500
#define SY6974B_AICL_POINT_VOL_5V_PHASE2	4550

#define OPLUS_CHG_BOOT_REASON_DEFAULT	101
#define OPLUS_CHG_RTC_UI_SOC_50_PERCENT	50
#define OPLUS_WAIT_FOR_CDP_COUNT	40

/* SET_MODE_SELECT */
#define SET_MODE_SELECT_DEFAULT	0x00
#define SET_MODE_SELECT_SNK	0x01
#define SET_MODE_SELECT_SRC	0x02
#define SET_MODE_SELECT_DRP	0x03

#define SY6974B_IIC_RW_RETRY_SLEEP_US	5000
#define SY6974B_IIC_READ_REG_RETRY_COUNT	20
#define SY6974B_IIC_WRITE_REG_RETRY_COUNT	3
#define SY6974B_IIC_READ_BLOCK_RETRY_COUNT	3

static int sy6974b_debug = 0;
#define ENABLE_DUMP_LOG BIT(0)
module_param(sy6974b_debug, int, 0644);
MODULE_PARM_DESC(sy6974b_debug, "debug sy6974b");

static int i2c_smbus_read_byte_data_with_lock(struct sy697x *chip, int reg)
{
	s32 ret;

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_read_byte_data(chip->client, reg);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int i2c_smbus_read_i2c_block_data_with_lock(struct sy697x *chip, int reg, u8 length, u8 *data)
{
	s32 ret;

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_read_i2c_block_data(chip->client, reg, length, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int i2c_smbus_write_byte_data_with_lock(struct sy697x *chip, int reg, int val)
{
	s32 ret;

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int __sy6974b_read_reg(struct sy697x *chip, int reg, int *data)
{
	s32 ret;
	int retry = SY6974B_IIC_READ_REG_RETRY_COUNT;

	do {
		ret = i2c_smbus_read_byte_data_with_lock(chip, reg);

		if (ret < 0) {
			retry--;
			usleep_range(SY6974B_IIC_RW_RETRY_SLEEP_US, SY6974B_IIC_RW_RETRY_SLEEP_US);
		}
	} while (ret < 0 && retry > 0);

	if (ret < 0) {
		chg_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*data = ret;
	}

	return 0;
}

static int __sy6974b_write_reg(struct sy697x *chip, int reg, int val)
{
	s32 ret;
	int retry = SY6974B_IIC_WRITE_REG_RETRY_COUNT;

	do {
		ret = i2c_smbus_write_byte_data_with_lock(chip, reg, val);

		if (ret < 0) {
			retry--;
			usleep_range(SY6974B_IIC_RW_RETRY_SLEEP_US, SY6974B_IIC_RW_RETRY_SLEEP_US);
		}
	} while (ret < 0 && retry > 0);

	if (ret < 0) {
		chg_err("i2c write fail: can't write %02x to %02x: %d\n", val, reg, ret);
		return ret;
	}

	return 0;
}

static int __sy6974b_read_block(struct sy697x *chip, u8 reg, u8 length, u8 *data)
{
	s32 ret;
	int retry = SY6974B_IIC_READ_BLOCK_RETRY_COUNT;

	do {
		ret = i2c_smbus_read_i2c_block_data_with_lock(chip, reg, length, data);

		if (ret < 0) {
			retry--;
			usleep_range(SY6974B_IIC_RW_RETRY_SLEEP_US, SY6974B_IIC_RW_RETRY_SLEEP_US);
		}
	} while (ret < 0 && retry > 0);

	if (ret < 0) {
		chg_err("i2c read block fail: can't read from %02x len=%d ret=%d\n", reg, length, ret);
		return ret;
	}

	return 0;
}

static int sy6974b_read_reg(struct sy697x *chip, int reg, int *data)
{
	int ret;

	ret = __sy6974b_read_reg(chip, reg, data);

	return ret;
}

static __maybe_unused int sy6974b_write_reg(struct sy697x *chip, int reg, int data)
{
	int ret;

	ret = __sy6974b_write_reg(chip, reg, data);

	return ret;
}

static __maybe_unused int sy6974b_read_block(struct sy697x *chip, u8 reg, u8 length, u8 *data)
{
	int ret;

	ret = __sy6974b_read_block(chip, reg, length, data);

	return ret;
}

static __maybe_unused int sy6974b_config_interface(struct sy697x *chip, int reg, int data, int mask)
{
	int ret;
	int tmp = 0;

	ret = __sy6974b_read_reg(chip, reg, &tmp);
	if (ret) {
		chg_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sy6974b_write_reg(chip, reg, tmp);
	if (ret)
		chg_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	return ret;
}

int __attribute__((weak)) get_usb_enum_status(void)
{
	return 1;
}

static int oplus_sy6974b_get_usb_status(void)
{
	if (g_oplus_chip && g_oplus_chip->usb_status == USB_TEMP_HIGH)
		return g_oplus_chip->usb_status;

	return 0;
}

static int oplus_set_mode(int mode)
{
	int rc = 0;

	if (mode == OPLUS_MODE_DRP)
		rc = oplus_sgm7220_set_mode(SET_MODE_SELECT_DRP);
	else
		rc = oplus_sgm7220_set_mode(SET_MODE_SELECT_SNK);
	return rc;
}

static bool oplus_ccdetect_check_is_gpio(struct oplus_chg_chip *chip)
{
	struct smb_charger *chg = NULL;

	int boot_mode = get_boot_mode();

	if (!chip) {
		chg_err("chip is null !\n");
		return false;
	}
	chg = &chip->pmic_spmi.smb5_chip->chg;

	/* HW engineer requirement */
	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN ||
		boot_mode == MSM_BOOT_MODE__FACTORY)
		return false;

	if (gpio_is_valid(chg->ccdetect_gpio))
		return true;

	return false;
}

static int oplus_ccdetect_gpio_init(struct oplus_chg_chip *chip)
{
	struct smb_charger *chg = NULL;

	if (!chip) {
		chg_err("chip is null !\n");
		return -EINVAL;
	}
	chg = &chip->pmic_spmi.smb5_chip->chg;

	chg->ccdetect_pinctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chg->ccdetect_pinctrl)) {
		chg_err("get ccdetect ccdetect_pinctrl fail\n");
		return -EINVAL;
	}

	chg->ccdetect_active = pinctrl_lookup_state(chg->ccdetect_pinctrl, "ccdetect_active");
	if (IS_ERR_OR_NULL(chg->ccdetect_active)) {
		chg_err("get ccdetect_active fail\n");
		return -EINVAL;
	}

	chg->ccdetect_sleep = pinctrl_lookup_state(chg->ccdetect_pinctrl, "ccdetect_sleep");
	if (IS_ERR_OR_NULL(chg->ccdetect_sleep)) {
		chg_err("get ccdetect_sleep fail\n");
		return -EINVAL;
	}

	if (chg->ccdetect_gpio > 0) {
		gpio_direction_input(chg->ccdetect_gpio);
	}

	pinctrl_select_state(chg->ccdetect_pinctrl, chg->ccdetect_active);

	return 0;
}

static void oplus_ccdetect_irq_init(struct oplus_chg_chip *chip)
{
	struct smb_charger *chg = NULL;

	if (!chip) {
		chg_err("chip is null !\n");
		return;
	}
	chg = &chip->pmic_spmi.smb5_chip->chg;

	chg->ccdetect_irq = gpio_to_irq(chg->ccdetect_gpio);
	chg_info("chg->ccdetect_irq[%d]!\n", chg->ccdetect_irq);
}

static void oplus_ccdetect_irq_register(struct oplus_chg_chip *chip)
{
	int ret;
	struct smb_charger *chg = NULL;

	if (!chip) {
		chg_err("chip is null !\n");
		return;
	}

	chg = &chip->pmic_spmi.smb5_chip->chg;

	ret = devm_request_threaded_irq(chip->dev, chg->ccdetect_irq,
			NULL, oplus_ccdetect_change_handler, IRQF_TRIGGER_FALLING
			| IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ccdetect-change", chip);

	if (ret < 0)
		chg_err("Unable to request ccdetect-change irq: %d\n", ret);

	chg_info("!!!!! irq register\n");

	ret = enable_irq_wake(chg->ccdetect_irq);
	if (ret != 0)
		chg_err("enable_irq_wake: ccdetect_irq failed %d\n", ret);
}

static void oplus_ccdetect_enable(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct smb_charger *chg = NULL;

	if (!chip) {
		chg_err("chip is null !\n");
		return;
	}

	chg = &chip->pmic_spmi.smb5_chip->chg;

	if (oplus_ccdetect_check_is_gpio(chip) != true)
		return;

	/* set DRP mode */
	oplus_set_mode(OPLUS_MODE_DRP);
	chg_err("set drp \n");
}

static void oplus_ccdetect_disable(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct smb_charger *chg = NULL;

	if (!chip) {
		chg_err("discrete_charger not ready!\n");
		return;
	}

	chg = &chip->pmic_spmi.smb5_chip->chg;

	if (oplus_ccdetect_check_is_gpio(chip) != true)
		return;

	/* set SINK mode */
	oplus_set_mode(OPLUS_MODE_SNK);
	chg_info("set sink \n");
}

static bool oplus_ccdetect_support_check(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return false;
	}

	if (oplus_ccdetect_check_is_gpio(chip) == true)
		return true;

	chg_info("not support, return false\n");

	return false;
}

static int oplus_chg_ccdetect_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;
	struct smb_charger *chg =  NULL;

	if (chip && chip->pmic_spmi.smb5_chip)
		chg = &chip->pmic_spmi.smb5_chip->chg;
	else
		return -EINVAL;

	if (chg)
		node = chg->dev->of_node;
	if (node == NULL) {
		chg_err("oplus_chg or device tree info. missing\n");
		return -EINVAL;
	}
	chg->ccdetect_gpio = of_get_named_gpio(node, "qcom,ccdetect-gpio", 0);
	if (chg->ccdetect_gpio <= 0) {
		chg_err("Couldn't read qcom,ccdetect-gpio rc=%d, qcom,ccdetect-gpio:%d\n",
				rc, chg->ccdetect_gpio);
	} else {
		if (oplus_ccdetect_support_check() == true) {
			rc = gpio_request(chg->ccdetect_gpio, "ccdetect-gpio");
			if (rc) {
				chg_err("unable to request ccdetect_gpio:%d\n",
						chg->ccdetect_gpio);
			} else {
				rc = oplus_ccdetect_gpio_init(chip);
				if (rc)
					chg_err("unable to init ccdetect_gpio:%d\n", chg->ccdetect_gpio);
				else
					oplus_ccdetect_irq_init(chip);
			}
		}
		chg_info("ccdetect-gpio:%d\n", chg->ccdetect_gpio);
	}

	return rc;
}

#define CCDETECT_DELAY_MS	50
static struct delayed_work usbtemp_recover_work;
static irqreturn_t oplus_ccdetect_change_handler(int irq, void *data)
{
	struct oplus_chg_chip *chip = data;
	struct smb_charger *chg = &chip->pmic_spmi.smb5_chip->chg;

	cancel_delayed_work_sync(&chg->ccdetect_work);
	chg_info("Scheduling ccdetect work!\n");
	schedule_delayed_work(&chg->ccdetect_work,
			msecs_to_jiffies(CCDETECT_DELAY_MS));
	return IRQ_HANDLED;
}

#define DISCONNECT	0
#define STANDARD_TYPEC_DEV_CONNECT	BIT(0)
#define OTG_DEV_CONNECT	BIT(1)
static bool oplus_get_otg_online_status_default(void)
{
	if (!g_sy) {
		chg_err("fail to init oplus_chg\n");
		return false;
	}

	return g_sy->otg_present;
}

#define OPLUS_CHG_CC_DETECT_GPIO_PULL_UP 1
#define OPLUS_CHG_CC_IC_DETECT_OTG_PRESENT 1
static int oplus_sy6974b_get_otg_online_status(void)
{
	int online = 0;
	int level = 0;
	int typec_otg = 0;
	static int pre_level = 1;
	static int pre_typec_otg = 0;
	struct smb_charger *chg = NULL;

	if (!g_sy || !g_oplus_chip) {
		chg_err("chg or g_oplus_chip ...\n");
		return false;
	}
	chg = &g_oplus_chip->pmic_spmi.smb5_chip->chg;

	if (oplus_ccdetect_check_is_gpio(g_oplus_chip) == true) {
		level = gpio_get_value(chg->ccdetect_gpio);
		if (level != gpio_get_value(chg->ccdetect_gpio)) {
			chg_err("ccdetect_gpio is unstable, try again...\n");
			usleep_range(5000, 5100);
			level = gpio_get_value(chg->ccdetect_gpio);
		}
	} else {
		return oplus_get_otg_online_status_default();
	}
	online = (level == OPLUS_CHG_CC_DETECT_GPIO_PULL_UP) ? DISCONNECT : STANDARD_TYPEC_DEV_CONNECT;

	typec_otg = oplus_get_otg_online_status_default();
	online = online | ((typec_otg == OPLUS_CHG_CC_IC_DETECT_OTG_PRESENT) ? OTG_DEV_CONNECT : DISCONNECT);

	if ((pre_level ^ level) || (pre_typec_otg ^ typec_otg)) {
		pre_level = level;
		pre_typec_otg = typec_otg;
		chg_info("gpio[%s], c-otg[%d], otg_online[%d]\n",
				level ? "H" : "L", typec_otg, online);
	}

	g_oplus_chip->otg_online = online;
	chg_info("otg_online[%d]\n", online);
	return online;
}
EXPORT_SYMBOL(oplus_sy6974b_get_otg_online_status);

static bool oplus_usbtemp_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		chg_err("smb5_chg not ready!\n");
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.dischg_gpio))
		return true;

	return false;
}

static bool oplus_usbtemp_check_is_support(void)
{
	if (oplus_usbtemp_check_is_gpio(g_oplus_chip) == true)
		return true;

	chg_info("dischg return false\n");

	return false;
}

static bool oplus_usbtemp_condition(void)
{
	if (!g_oplus_chip) {
		chg_err("fail to init oplus_chip\n");
		return false;
	}

	g_oplus_chip->usbtemp_check = g_oplus_chip->chg_ops->check_chrdet_status();
	chg_info("check_chrdet_status is %d\n", g_oplus_chip->usbtemp_check);
	return g_oplus_chip->usbtemp_check;
}

static void oplus_wake_up_usbtemp_thread(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip)
		return;

	if (oplus_usbtemp_check_is_support() == true) {
		chip->usbtemp_check = oplus_usbtemp_condition();
		if (chip->usbtemp_check)
			wake_up_interruptible(&chip->oplus_usbtemp_wq);
	}
}

static void oplus_ccdetect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						ccdetect_work.work);
	int level = 0;

	if (!g_oplus_chip)
		return;

	level = gpio_get_value(chg->ccdetect_gpio);
	if (level != 1) {
		oplus_ccdetect_enable();
		oplus_wake_up_usbtemp_thread();
	} else {
		g_oplus_chip->usbtemp_check = oplus_usbtemp_condition();

		if (oplus_sy6974b_get_otg_switch_status() == false)
			oplus_ccdetect_disable();
		if (g_oplus_chip->usb_status == USB_TEMP_HIGH) {
			schedule_delayed_work(&usbtemp_recover_work, 0);
		}
	}
}

static void oplus_usbtemp_recover_work(struct work_struct *work)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip)
		return;

	oplus_usbtemp_recover_func(g_oplus_chip);
	chg_info("done \n");
}

static bool opluschg_get_typec_cc_orientation(union power_supply_propval *val)
{
	chg_info("typec_dir = %s\n", typec_dir == OPLUS_TYPEC_ATTACH_CC1 ? "cc1 attach" : "cc2_attach");
	val->intval = typec_dir;
	return typec_dir;
}

static void oplus_chg_awake_init(struct sy697x *chip)
{
	if (!chip) {
		chg_err("chip is null\n");
		return;
	}
	chip->suspend_ws = wakeup_source_register(NULL, "split chg wakelock");
}

static void oplus_chg_wakelock(struct sy697x *chip, bool awake)
{
	static bool pm_flag = false;

	if (!chip || !chip->suspend_ws)
		return;

	if (awake && !pm_flag) {
		pm_flag = true;
		__pm_stay_awake(chip->suspend_ws);
		chg_info("true\n");
	} else if (!awake && pm_flag) {
		__pm_relax(chip->suspend_ws);
		pm_flag = false;
		chg_info("false\n");
	}
}

static void oplus_keep_resume_awake_init(struct sy697x *chip)
{
	if (!chip) {
		chg_err("chip is null\n");
		return;
	}
	chip->keep_resume_ws = wakeup_source_register(NULL, "split_chg_keep_resume");
}

static void oplus_keep_resume_wakelock(struct sy697x *chip, bool awake)
{
	static bool pm_flag = false;

	if (!chip || !chip->keep_resume_ws)
		return;

	if (awake && !pm_flag) {
		pm_flag = true;
		__pm_stay_awake(chip->keep_resume_ws);
		chg_info("true\n");
	} else if (!awake && pm_flag) {
		__pm_relax(chip->keep_resume_ws);
		pm_flag = false;
		chg_info("false\n");
	}
}

static void oplus_notify_extcon_props(struct sy697x *chg, int id)
{
	union extcon_property_value val;
	union power_supply_propval prop_val;

	opluschg_get_typec_cc_orientation(&prop_val);
	val.intval = ((prop_val.intval == OPLUS_TYPEC_ATTACH_CC2) ? 1 : 0);
	extcon_set_property(chg->extcon, id,
			EXTCON_PROP_USB_TYPEC_POLARITY, val);
	val.intval = true;
	extcon_set_property(chg->extcon, id,
			EXTCON_PROP_USB_SS, val);
}

static void oplus_notify_device_mode(bool enable)
{
	struct sy697x *chg = g_sy;

	if (!chg) {
		chg_err("chg is null\n");
		return;
	}
	if (enable)
		oplus_notify_extcon_props(chg, EXTCON_USB);

	extcon_set_state_sync(chg->extcon, EXTCON_USB, enable);
	chg_info("enable[%d]\n", enable);
}

static void oplus_notify_usb_host(bool enable)
{
	struct sy697x *chg = g_sy;

	if (!chg || !g_oplus_chip) {
		chg_err("chg or g_oplus_chip is null\n");
		return;
	}
	if (enable) {
		chg_info("enabling VBUS in OTG mode\n");
		oplus_sy6974b_enable_otg();
		oplus_notify_extcon_props(chg, EXTCON_USB_HOST);
	} else {
		chg_info("disabling VBUS in OTG mode\n");
		oplus_sy6974b_disable_otg();
	}

	power_supply_changed(g_oplus_chip->usb_psy);
	extcon_set_state_sync(chg->extcon, EXTCON_USB_HOST, enable);
	chg_info("enable[%d]\n", enable);
}

static void oplus_sy6974b_typec_sink_insertion(void)
{
	struct sy697x *chg = g_sy;

	if (!chg || !g_oplus_chip) {
		chg_err("chg or g_oplus_chip is null\n");
		return;
	}
	chg->otg_present = true;
	g_oplus_chip->otg_online = true;
	oplus_chg_wake_update_work();

	oplus_notify_usb_host(true);
	chg_info("wakeup done!!!\n");
}

static void oplus_sy6974b_typec_sink_removal(void)
{
	struct sy697x *chg = g_sy;

	if (!chg) {
		chg_err("chg is null\n");
		return;
	}
	if (chg->otg_present)
		oplus_notify_usb_host(false);
	chg->otg_present = false;
	g_oplus_chip->otg_online = false;
	oplus_chg_wake_update_work();
	chg_info("wakeup done!!!\n");
}

static void oplus_sy6974b_typec_src_removal(void)
{
	struct sy697x *chg = g_sy;

	if (!chg) {
		chg_err("[%s] chg is null\n", __func__);
		return;
	}
	oplus_notify_device_mode(false);
	chg_info("done!!!\n");
}

static void oplus_for_cdp(void)
{
	int timeout = OPLUS_WAIT_FOR_CDP_COUNT;

	if (is_usb_rdy() == false) {
		while (is_usb_rdy() == false && timeout > 0) {
			msleep(10);
			timeout--;
		}
		if (timeout == 0)
			chg_info("usb_rdy timeout\n");
		else
			chg_info("usb_rdy free\n");
	} else {
		chg_info("rm cdp 400ms usb_rdy PASS\n");
	}
}

static int sy6974b_enable_otg(struct sy697x *sy)
{
	int val =  REG01_SY6974B_OTG_ENABLE;

	return sy6974b_config_interface(sy, REG01_SY6974B_ADDRESS,
		val, REG01_SY6974B_OTG_MASK);
}

static int sy6974b_vmin_limit(struct sy697x *sy)
{
	return 0;
}

static int sy6974b_disable_otg(struct sy697x *sy)
{
	int val = REG01_SY6974B_OTG_DISABLE;

	return sy6974b_config_interface(sy, REG01_SY6974B_ADDRESS,
		val, REG01_SY6974B_OTG_MASK);
}

static int sy6974b_enable_hvdcp(struct sy697x *sy)
{
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_enable_hvdcp);

static int sy6974b_disable_hvdcp(struct sy697x *sy)
{
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_disable_hvdcp);

static int sy6974b_disable_batfet_rst(struct sy697x *sy)
{
	int rc;
	int val;

	val = SY6974_BATFET_RST_DISABLE << REG07_SY6974B_BATFET_RST_EN_SHIFT;
	rc = sy6974b_config_interface(sy, REG07_SY6974B_ADDRESS,
			val, REG07_SY6974B_BATFET_RST_EN_MASK);

	return rc;
}

static int bq2589x_disable_maxc(struct sy697x *bq)
{
	return 0;
}


static int sy6974b_disable_ico(struct sy697x *sy)
{
	return 0;
}
static int sy6974b_enable_charger(struct sy697x *sy)
{
	int ret;

	int val = REG01_SY6974B_CHARGING_ENABLE;

	dev_info(sy->dev, "%s\n", __func__);
	ret = sy6974b_config_interface(sy, REG01_SY6974B_ADDRESS,
			val, REG01_SY6974B_CHARGING_MASK);

	return ret;
}

static int sy6974b_disable_charger(struct sy697x *sy)
{
	int ret;

	int val =  REG01_SY6974B_CHARGING_DISABLE;

	dev_info(sy->dev, "%s\n", __func__);
	ret = sy6974b_config_interface(sy,  REG01_SY6974B_ADDRESS,
		val, REG01_SY6974B_CHARGING_MASK);
	return ret;
}
bool sy6974b_adc_ready(struct sy697x *sy)
{
	return 0;
}

int sy6974b_adc_start(struct sy697x *sy, bool enable)
{
	return 0;
}

EXPORT_SYMBOL_GPL(sy6974b_adc_start);

int sy6974b_adc_read_battery_volt(struct sy697x *sy)
{
	return SY6974B_READ_VBAT_DEFAULT;
}
EXPORT_SYMBOL_GPL(sy6974b_adc_read_battery_volt);

int sy6974b_adc_read_sys_volt(struct sy697x *sy)
{
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_adc_read_sys_volt);

int sy6974b_adc_read_vbus_volt(struct sy697x *sy)
{
	return SY6974B_READ_VBUS_DEFAULT;
}
EXPORT_SYMBOL_GPL(sy6974b_adc_read_vbus_volt);

int sy6974b_adc_read_temperature(struct sy697x *sy)
{
	return SY6974B_READ_TEMPERATURE_DEFAULT;
}
EXPORT_SYMBOL_GPL(sy6974b_adc_read_temperature);

int sy6974b_adc_read_charge_current(void)
{
	return SY6974B_READ_CHARGE_CURRENT_DEFAULT;
}
EXPORT_SYMBOL_GPL(sy6974b_adc_read_charge_current);

int sy6974b_set_chargecurrent(struct sy697x *sy, int chg_cur)
{
	int rc;
	int tmp = 0;
	struct sy697x *chip = sy;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	chg_info("set charge current = %d\n", chg_cur);

	if (chg_cur > REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_MAX)
		chg_cur = REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_MAX;

	if (chg_cur < REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_OFFSET)
		chg_cur = REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_OFFSET;

	tmp = chg_cur - REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_OFFSET;
	tmp = tmp / REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_STEP;

	rc = sy6974b_config_interface(chip, REG02_SY6974B_ADDRESS,
					tmp << REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_SHIFT,
					REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_MASK);

	return rc;
}

int sy6974b_set_term_current(struct sy697x *sy, int term_curr)
{
	int rc;
	int tmp = 0;
	struct sy697x *chip = sy;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	chg_info("term_current = %d\n", term_curr);
	tmp = term_curr - REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_OFFSET;
	tmp = tmp / REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_STEP;

	rc = sy6974b_config_interface(chip, REG03_SY6974B_ADDRESS,
					tmp << REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_SHIFT,
					REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_MASK);
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_set_term_current);

int sy6974b_set_prechg_current(struct sy697x *sy, int curr)
{
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_set_prechg_current);

int sy6974b_set_chargevolt(struct sy697x *sy, int vfloat_mv)
{
	int rc;
	int tmp = 0;
	struct sy697x *chip = sy;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	chg_info("vfloat_mv = %d\n", vfloat_mv);

	if (vfloat_mv > REG04_SY6974B_CHG_VOL_LIMIT_MAX)
		vfloat_mv = REG04_SY6974B_CHG_VOL_LIMIT_MAX;

	if (vfloat_mv < REG04_SY6974B_CHG_VOL_LIMIT_OFFSET)
		vfloat_mv = REG04_SY6974B_CHG_VOL_LIMIT_OFFSET;

	tmp = vfloat_mv - REG04_SY6974B_CHG_VOL_LIMIT_OFFSET;

	tmp = tmp / REG04_SY6974B_CHG_VOL_LIMIT_STEP;

	rc = sy6974b_config_interface(chip, REG04_SY6974B_ADDRESS,
					tmp << REG04_SY6974B_CHG_VOL_LIMIT_SHIFT,
					REG04_SY6974B_CHG_VOL_LIMIT_MASK);

	return rc;
}

int sy6974b_set_vindpm_vol(int vol)
{
	int rc;
	int tmp = 0;
	struct sy697x *chip = g_sy;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	tmp = (vol - REG06_SY6974B_VINDPM_OFFSET) / REG06_SY6974B_VINDPM_STEP_MV;
	rc = sy6974b_config_interface(chip, REG06_SY6974B_ADDRESS,
					tmp << REG06_SY6974B_VINDPM_SHIFT,
					REG06_SY6974B_VINDPM_MASK);

	return rc;
}

#define AICL_POINT_VOL_5V_PHASE1 4140
#define AICL_POINT_VOL_5V_PHASE2 4000
#define HW_AICL_POINT_VOL_5V_PHASE1 4440
#define HW_AICL_POINT_VOL_5V_PHASE2 4520
#define SW_AICL_POINT_VOL_5V_PHASE1 4500
#define SW_AICL_POINT_VOL_5V_PHASE2 4535
int sy6974b_set_input_volt_limit(struct sy697x *sy, int vbatt)
{
	struct sy697x *chip = sy;

	if (!chip)
		return -1;

	if (chip->hw_aicl_point == HW_AICL_POINT_VOL_5V_PHASE1 &&
		vbatt > AICL_POINT_VOL_5V_PHASE1) {
		chip->hw_aicl_point = HW_AICL_POINT_VOL_5V_PHASE2;
		chip->sw_aicl_point = SW_AICL_POINT_VOL_5V_PHASE2;
		sy6974b_set_vindpm_vol(chip->hw_aicl_point);
	} else if (chip->hw_aicl_point == HW_AICL_POINT_VOL_5V_PHASE2 &&
		vbatt < AICL_POINT_VOL_5V_PHASE2) {
		chip->hw_aicl_point = HW_AICL_POINT_VOL_5V_PHASE1;
		chip->sw_aicl_point = SW_AICL_POINT_VOL_5V_PHASE1;
		sy6974b_set_vindpm_vol(chip->hw_aicl_point);
	}

	return 0;
}

int sy6974b_set_input_current_limit(struct sy697x *sy, int curr)
{
	u8 val;

	int boot_mode = get_boot_mode();
	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) {
		curr = 0;
		dev_info(sy->dev, "%s: boot_mode[%d] curr = %d\n", __func__, boot_mode, curr);
	}

	dev_info(sy->dev, "%s: curr = %d\n", __func__, curr);

	if (curr < REG00_SY6974B_INPUT_CURRENT_LIMIT_OFFSET)
		curr = REG00_SY6974B_INPUT_CURRENT_LIMIT_OFFSET;

	val = (curr - REG00_SY6974B_INPUT_CURRENT_LIMIT_OFFSET) / REG00_SY6974B_INPUT_CURRENT_LIMIT_STEP;

	return sy6974b_config_interface(sy, REG00_SY6974B_ADDRESS,
		val << REG00_SY6974B_INPUT_CURRENT_LIMIT_SHIFT,
		REG00_SY6974B_INPUT_CURRENT_LIMIT_MASK);
}

int sy6974b_get_input_current(void)
{
	int reg_val = 0;
	int icl = 0;
	int ret;

	if (!g_sy)
		return icl;

	ret = sy6974b_read_reg(g_sy, REG00_SY6974B_ADDRESS, &reg_val);
	if (!ret) {
		icl = (reg_val & REG00_SY6974B_INPUT_CURRENT_LIMIT_MASK) >> REG00_SY6974B_INPUT_CURRENT_LIMIT_SHIFT;
		icl = icl * REG00_SY6974B_INPUT_CURRENT_LIMIT_STEP + REG00_SY6974B_INPUT_CURRENT_LIMIT_OFFSET;
	}
	dev_info(g_sy->dev, "%s: icl = %d ma\n", __func__, icl);

	return icl;
}

int sy6974b_kick_wdt(struct sy697x *sy)
{
	int rc;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	rc = sy6974b_config_interface(sy, REG01_SY6974B_ADDRESS,
		REG01_SY6974B_WDT_TIMER_RESET,
		REG01_SY6974B_WDT_TIMER_RESET_MASK);
	if (rc)
		chg_err("Couldn't sy6974b kick wdt rc = %d\n", rc);

	return rc;
}

int sy6974b_set_watchdog_timer(struct sy697x *sy, u8 timeout)
{
	int rc;

	if (!sy)
		return -1;

	if (atomic_read(&sy->driver_suspended) == 1)
		return -1;

	sy6974b_kick_wdt(sy);

	rc = sy6974b_config_interface(sy, REG05_SY6974B_ADDRESS,
		timeout,
		REG05_SY6974B_WATCHDOG_TIMER_MASK);

	if (rc)
		chg_err("Couldn't set recharging threshold rc = %d\n", rc);

	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_set_watchdog_timer);

int sy6974b_set_wdt_timer(int reg)
{
	int rc;
	struct sy697x *sy = g_sy;

	if (!sy)
		return -1;

	if (atomic_read(&sy->driver_suspended) == 1)
		return -1;

	sy6974b_kick_wdt(sy);

	rc = sy6974b_config_interface(sy, REG05_SY6974B_ADDRESS,
					reg,
					REG05_SY6974B_WATCHDOG_TIMER_MASK);
	if (rc)
		chg_err("Couldn't set recharging threshold rc = %d\n", rc);

	return 0;
}

int sy6974b_disable_watchdog_timer(struct sy697x *sy)
{
	if (!sy)
		return -1;

	if (atomic_read(&sy->driver_suspended) == 1)
		return -1;

	sy6974b_set_wdt_timer(REG05_SY6974B_WATCHDOG_TIMER_DISABLE);
	chg_info("sy6974b_wdt_enable true\n");

	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_disable_watchdog_timer);

int sy6974b_reset_watchdog_timer(struct sy697x *sy)
{
	int rc;

	if (!sy)
		return -1;

	if (atomic_read(&sy->driver_suspended) == 1)
		return -1;

	rc = sy6974b_config_interface(sy, REG01_SY6974B_ADDRESS,
		REG01_SY6974B_WDT_TIMER_RESET,
		REG01_SY6974B_WDT_TIMER_RESET_MASK);
	if (rc)
		chg_err("Couldn't sy6974b kick wdt rc = %d\n", rc);

	return rc;
}
EXPORT_SYMBOL_GPL(sy6974b_reset_watchdog_timer);

static int sy6974b_set_iindet(bool enable)
{
	int rc;
	struct sy697x *sy = g_sy;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	if (enable)
		rc = sy6974b_config_interface(sy, REG07_SY6974B_ADDRESS,
				REG07_SY6974B_IINDET_EN_MASK,
				REG07_SY6974B_IINDET_EN_FORCE_DET);
	else
		rc = sy6974b_config_interface(sy, REG07_SY6974B_ADDRESS,
				REG07_SY6974B_IINDET_EN_MASK,
				REG07_SY6974B_IINDET_DIS_FORCE_DET);

	if (rc < 0)
		chg_err("Couldn't set REG07_SY6974B_IINDET_EN_MASK rc = %d\n", rc);

	return rc;
}

static int sy6974b_get_iindet(void)
{
	int rc = 0;
	int reg_val = 0;
	bool is_complete = false;
	struct sy697x *sy = g_sy;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	rc = sy6974b_read_reg(sy, REG07_SY6974B_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG07_SY6974B_ADDRESS rc = %d\n", rc);
		return false;
	}

	is_complete = ((reg_val & REG07_SY6974B_IINDET_EN_MASK) == REG07_SY6974B_IINDET_EN_DET_COMPLETE) ? 1 : 0;
	return is_complete;
}

int sy6974b_force_dpdm(struct sy697x *sy, bool enable)
{
	if (enable)
		sy6974b_enable_enlim(sy);

	sy6974b_set_iindet(enable);

	chg_info("enable=%d\n", enable);
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_force_dpdm);

int sy6974b_reset_chip(struct sy697x *sy)
{
	int rc;

	if (!sy)
		return -1;

	if (atomic_read(&sy->driver_suspended) == 1)
		return -1;

	rc = sy6974b_config_interface(sy, REG0B_SY6974B_ADDRESS,
		REG0B_SY6974B_REG_RST_RESET,
		REG0B_SY6974B_REG_RST_MASK);

	if (rc)
		chg_err("Couldn't sy6974b_reset_charger rc = %d\n", rc);

	return rc;
}
EXPORT_SYMBOL_GPL(sy6974b_reset_chip);

void sy6974b_suspend_by_hz_mode(bool en)
{
	int rc;
	struct sy697x *sy = g_sy;

	if (!sy)
		return;

	if (atomic_read(&sy->driver_suspended) == 1)
		return;

	rc = sy6974b_config_interface(sy, REG00_SY6974B_ADDRESS,
		en ? REG00_SY6974B_SUSPEND_MODE_ENABLE : REG00_SY6974B_SUSPEND_MODE_DISABLE,
		REG00_SY6974B_SUSPEND_MODE_MASK);
	if (rc < 0)
		chg_err("fail en=%d rc = %d\n", en, rc);
}

int sy6974b_enter_hiz_mode(struct sy697x *sy)
{
	int boot_mode = get_boot_mode();

	chg_info("boot_mode[%d]\n", boot_mode);

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	atomic_set(&sy->charger_suspended, 1);

	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) {
		if (!is_bq25890h(sy)) {
			sy6974b_config_interface(sy, REG00_SY6974B_ADDRESS,
			REG00_SY6974B_SUSPEND_MODE_DISABLE,
			REG00_SY6974B_SUSPEND_MODE_MASK);
		}
		sy6974b_disable_charger(sy);
		sy6974b_set_input_current_limit(sy, 0);
	} else {
		sy->before_suspend_icl = sy6974b_get_input_current();
		sy6974b_set_input_current_limit(sy, SY697X_IINLIM_BASE);
		sy6974b_disable_charger(sy);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_enter_hiz_mode);

int sy6974b_exit_hiz_mode(struct sy697x *sy)
{
	int boot_mode = get_boot_mode();
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!sy || !chip)
		return 0;

	chg_info("boot_mode[%d] mmi[%d %d]\n",
		boot_mode, chip->mmi_chg, chip->mmi_fastchg);
	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	atomic_set(&sy->charger_suspended, 0);

	if (boot_mode ==  MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) {
		sy6974b_config_interface(sy, REG00_SY6974B_ADDRESS,
		REG00_SY6974B_SUSPEND_MODE_DISABLE,
		REG00_SY6974B_SUSPEND_MODE_MASK);
	} else {
		sy->before_unsuspend_icl = sy6974b_get_input_current();
		if (is_bq25890h(sy)) {
			sy6974b_config_interface(sy, REG00_SY6974B_ADDRESS,
			REG00_SY6974B_SUSPEND_MODE_DISABLE,
			REG00_SY6974B_SUSPEND_MODE_MASK);
		}

		if (!chip->mmi_chg || !chip->mmi_fastchg) {
			sy6974b_disable_charger(sy);
		} else {
			sy6974b_enable_charger(sy);
		}

		if ((sy->before_unsuspend_icl == 0) ||
				(sy->before_suspend_icl == 0) ||
				(sy->before_unsuspend_icl != 100) ||
				(sy->before_unsuspend_icl == sy->before_suspend_icl)) {
			chg_info("ignore set icl [%d %d]\n", sy->before_suspend_icl, sy->before_unsuspend_icl);
		} else {
			sy6974b_set_input_current_limit(sy, sy->before_suspend_icl);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_exit_hiz_mode);

int sy6974b_disable_enlim(struct sy697x *sy)
{
	dev_info(sy->dev, "%s ,as sy6974b has no such ENILIM BIT, so return 0!\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_disable_enlim);

int sy6974b_enable_enlim(struct sy697x *sy)
{
	dev_info(sy->dev, "%s ,as sy6974b has no such ENILIM BIT, so return 0!\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_enable_enlim);

int sy6974b_get_hiz_mode(struct sy697x *sy, u8 *state)
{
	int val = 0;
	int ret;

	ret = sy6974b_read_reg(sy, REG00_SY6974B_ADDRESS, &val);
	if (ret < 0) {
		chg_err("REG00_SY6974B_ADDRESS fail ret = %d\n", ret);
		return 0;
	}

	if ((val & REG00_SY6974B_SUSPEND_MODE_MASK) == REG00_SY6974B_SUSPEND_MODE_ENABLE) {
		*state = true;
	} else {
		*state = false;
	}
	chg_info("state[%d]\n", *state);

	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_get_hiz_mode);

int sy6974b_enable_term(struct sy697x *sy, bool enable)
{
	int rc;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	if (enable) {
		rc = sy6974b_config_interface(sy, REG05_SY6974B_ADDRESS,
				REG05_SY6974B_TERMINATION_ENABLE,
				REG05_SY6974B_TERMINATION_MASK);
	} else
		rc = sy6974b_config_interface(sy, REG05_SY6974B_ADDRESS,
				REG05_SY6974B_TERMINATION_DISABLE,
				REG05_SY6974B_TERMINATION_MASK);
	if (rc)
		chg_err("Couldn't set chging term disable rc = %d\n", rc);

	return rc;
}
EXPORT_SYMBOL_GPL(sy6974b_enable_term);

int sy6974b_set_boost_current(struct sy697x *sy, int curr)
{
	int rc;
	int ilim;

	if (!sy)
		return -1;

	if (atomic_read(&sy->driver_suspended) == 1)
		return -1;

	if (curr >= SY6974B_OTG_BOOST_CURRENT_VALUE_1200MA) {
		ilim = REG02_SY6974B_OTG_CURRENT_LIMIT_1200MA;
	} else {
		ilim = REG02_SY6974B_OTG_CURRENT_LIMIT_500MA;
	}

	chg_info("curr=%d, ilim=%d\n", curr, ilim);
	rc = sy6974b_config_interface(sy, REG02_SY6974B_ADDRESS,
		ilim,
		REG02_SY6974B_OTG_CURRENT_LIMIT_MASK);
	if (rc < 0)
		chg_err("Couldn't sy6974b_otg_ilim_set  rc = %d\n", rc);

	return rc;
}

int sy6974b_enable_auto_dpdm(struct sy697x *sy, bool enable)
{
	chg_info("as sy6974b have no AUTO_DPDM BIT, we do nothing\n");
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_enable_auto_dpdm);

int sy6974b_set_boost_voltage(struct sy697x *sy, int volt)
{
	int rc;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	chg_info("volt=%d, but fixed to 5150mV\n", volt);
	rc = sy6974b_config_interface(sy, REG06_SY6974B_ADDRESS,
			REG06_SY6974B_OTG_VLIM_5150MV,
			REG06_SY6974B_OTG_VLIM_MASK);
	return rc;
}
EXPORT_SYMBOL_GPL(sy6974b_set_boost_voltage);

int sy6974b_enable_ico(struct sy697x *sy, bool enable)
{
	chg_info("enable = %d, as sy6974b have no aicl, so return 0! \n", enable);
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_enable_ico);

int sy6974b_read_idpm_limit(struct sy697x *sy, int *icl)
{
	chg_info("as sy6974b have no IDPM_LIM(icl), so return 0! \n");
	return 0;
}
EXPORT_SYMBOL_GPL(sy6974b_read_idpm_limit);

int sy6974b_enable_safety_timer(struct sy697x *sy)
{
	int rc;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	rc = sy6974b_config_interface(sy, REG05_SY6974B_ADDRESS,
		REG05_SY6974B_CHG_SAFETY_TIMER_ENABLE,
		REG05_SY6974B_CHG_SAFETY_TIMER_MASK);
	if (rc)
		chg_err("Couldn't sy6974b set_chg_timer rc = %d\n", rc);

	return rc;
}
EXPORT_SYMBOL_GPL(sy6974b_enable_safety_timer);

int sy6974b_disable_safety_timer(struct sy697x *sy)
{
	int rc;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	rc = sy6974b_config_interface(sy, REG05_SY6974B_ADDRESS,
			REG05_SY6974B_CHG_SAFETY_TIMER_DISABLE,
			REG05_SY6974B_CHG_SAFETY_TIMER_MASK);

	if (rc)
		chg_err("Couldn't sy6974b set_chg_timer rc = %d\n", rc);

	return rc;
}
EXPORT_SYMBOL_GPL(sy6974b_disable_safety_timer);

static int sy6974b_switch_to_hvdcp(struct sy697x *sy, enum hvdcp_type type)
{
	dev_err(sy->dev, "sy697x_switch_to_hvdcp，type: %d. as sy6974b have no HVDCP, return 0 !\n", type);
	return 0;
}

static int sy6974b_check_charge_done(struct sy697x *sy, bool *done)
{
	int rc;
	int reg_full;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	rc = sy6974b_read_reg(sy, REG08_SY6974B_ADDRESS, &reg_full);
	if (rc) {
		chg_err("Couldn't read REG08_SY6974B_ADDRESS rc = %d\n", rc);
		return 0;
	}

	reg_full = ((reg_full & REG08_SY6974B_CHG_STAT_MASK) == REG08_SY6974B_CHG_STAT_CHG_TERMINATION) ? 1 : 0;
	*done = reg_full;
	if (reg_full) {
		chg_err("the sy6974b is full");
		sy6974b_dump_regs(sy);
	}

	return rc;
}

static struct sy697x_platform_data *sy6974b_parse_dt(struct device_node *np,
						      struct sy697x *sy)
{
	int ret;
	struct sy697x_platform_data *pdata;

	pdata = devm_kzalloc(sy->dev, sizeof(struct sy697x_platform_data), GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (of_property_read_string(np, "charger_name", &sy->chg_dev_name) < 0) {
		sy->chg_dev_name = "primary_chg";
		pr_warn("no charger name\n");
	}

	if (of_property_read_string(np, "eint_name", &sy->eint_name) < 0) {
		sy->eint_name = "chr_stat";
		pr_warn("no eint name\n");
	}

	sy->chg_det_enable =
		of_property_read_bool(np, "sy,sy697x,charge-detect-enable");

	ret = of_property_read_u32(np, "sy,sy697x,usb-vlim", &pdata->usb.vlim);
	if (ret) {
		pdata->usb.vlim = SY6974B_VINDPM_DEFAULT;
		chg_err("Failed to read node of sy,sy697x,usb-vlim\n");
	}

	ret = of_property_read_u32(np, "sy,sy697x,usb-ilim", &pdata->usb.ilim);
	if (ret) {
		pdata->usb.ilim = SY6974B_INPUT_CURRENT_LIMIT_DEFAULT;
		chg_err("Failed to read node of sy,sy697x,usb-ilim\n");
	}

	ret = of_property_read_u32(np, "sy,sy697x,usb-vreg", &pdata->usb.vreg);
	if (ret) {
		pdata->usb.vreg = SY6974B_CV_DEFAULT;
		chg_err("Failed to read node of sy,sy697x,usb-vreg\n");
	}

	ret = of_property_read_u32(np, "sy,sy697x,usb-ichg", &pdata->usb.ichg);
	if (ret) {
		pdata->usb.ichg = SY6974B_FAST_CHARGE_CURRENT_LIMIT_DEFAULT;
		chg_err("Failed to read node of sy,sy697x,usb-ichg\n");
	}

	ret = of_property_read_u32(np, "sy,sy697x,precharge-current",
				   &pdata->iprechg);
	if (ret) {
		pdata->iprechg = SY6974B_PRECHARGE_CURRENT_LIMIT_DEFAULT;
		chg_err("Failed to read node of sy,sy697x,precharge-current\n");
	}

	ret = of_property_read_u32(np, "sy,sy697x,termination-current",
				   &pdata->iterm);
	if (ret) {
		pdata->iterm = SY6974B_TERMINATION_CURRENT_LIMIT_DEFAULT;
		chg_err("Failed to read node of sy,sy697x,termination-current\n");
	}

	ret = of_property_read_u32(np, "sy,sy697x,boost-voltage",
				 &pdata->boostv);
	if (ret) {
		pdata->boostv = SY6974B_BOOST_REGULATION_VOLTAGE_DEFAULT;
		chg_err("Failed to read node of sy,sy697x,boost-voltage\n");
	}

	ret = of_property_read_u32(np, "sy,sy697x,boost-current",
				 &pdata->boosti);
	if (ret) {
		pdata->boosti = SY6974B_BOOST_MODE_CURRENT_LIMIT_DEFAULT;
		chg_err("Failed to read node of sy,sy697x,boost-current\n");
	}

	return pdata;
}

static int opluschg_updata_usb_type(struct sy697x *sy)
{
	union power_supply_propval propval;
	int ret;
	struct oplus_chg_chip *chgchip = g_oplus_chip;

	if (!sy) {
		chg_err("sy is null\n");
		return 0;
	}

	if (sy->power_good && (oplus_sy6974b_get_usb_status() != USB_TEMP_HIGH)
		&& ((sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB) || (sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB_CDP)))
		propval.intval = 1;
	else
		propval.intval = 0;

	chg_info("POWER_SUPPLY_PROP_ONLINE %d\n", propval.intval);
	ret = power_supply_set_property(chgchip->usb_psy, POWER_SUPPLY_PROP_ONLINE,
					&propval);

	propval.intval = sy->oplus_chg_type;
	chg_info("POWER_SUPPLY_PROP_TYPE %d\n", propval.intval);
	ret = power_supply_set_property(chgchip->usb_psy, POWER_SUPPLY_PROP_TYPE,
					&propval);
	if (ret < 0)
		pr_notice("inform power supply charge type failed:%d\n", ret);

	power_supply_changed(chgchip->usb_psy);
	chg_info("power_supply_changed POWER_SUPPLY_TYPE_USB done\n");
	return 0;
}

static int sy6974b_inform_charger_type(struct sy697x *sy)
{
	int ret;
	union power_supply_propval propval;

	struct oplus_chg_chip *chgchip = g_oplus_chip;

	if (!sy || !chgchip) {
		chg_err("sy is null\n");
		return 0;
	}

	if (sy->power_good)
		propval.intval = 1;
	else
		propval.intval = 0;
	ret = power_supply_set_property(chgchip->ac_psy, POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0)
		pr_notice("inform power supply charge type failed:%d\n", ret);

	if (g_sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB_DCP)
		power_supply_changed(chgchip->ac_psy);
	power_supply_changed(chgchip->batt_psy);
	chg_info("ac or battery [%d] done\n", propval.intval);

	return ret;
}

static int sy_charger_type_recheck(struct sy697x *sy)
{
	int ret;

	int reg_val = 0;
	int vbus_stat = 0;

	if (!g_oplus_chip)
		return 0;

	ret = sy6974b_read_reg(sy, REG08_SY6974B_ADDRESS, &reg_val);
	if (ret)
		return ret;

	vbus_stat = reg_val & REG08_SY6974B_VBUS_STAT_MASK;
	sy->vbus_type = vbus_stat;

	switch (vbus_stat) {
	case SY697X_VBUS_TYPE_NONE:
		sy->chg_type = CHARGER_UNKNOWN;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case SY697X_VBUS_TYPE_SDP:
		sy->chg_type = STANDARD_HOST;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB;
		g_oplus_chip->charger_type = POWER_SUPPLY_TYPE_USB;
		break;
	case SY697X_VBUS_TYPE_CDP:
		sy->chg_type = CHARGING_HOST;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		chg_info("g_oplus_chip->charger_type=%d\n",
			g_oplus_chip->charger_type);
		if (g_oplus_chip->charger_type != POWER_SUPPLY_TYPE_USB_CDP) {
			sy->cdp_retry_aicl = true;
			g_oplus_chip->charger_type = POWER_SUPPLY_TYPE_USB_CDP;
		}
		sy->cdp_retry_aicl = true;
		break;
	case SY697X_VBUS_TYPE_DCP:
		sy->chg_type = STANDARD_CHARGER;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case SY697X_VBUS_TYPE_HVDCP:
		sy->chg_type = STANDARD_CHARGER;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case SY697X_VBUS_TYPE_UNKNOWN:
		sy->chg_type = NONSTANDARD_CHARGER;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case SY697X_VBUS_TYPE_NON_STD:
		sy->chg_type = NONSTANDARD_CHARGER;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		sy->chg_type = NONSTANDARD_CHARGER;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	}
	chg_debug("chg_type = %d, %d, %d vbus_stat[%d] vbus_on[%d]\n",
		sy->chg_type, sy->oplus_chg_type, g_oplus_chip->charger_type, vbus_stat, sy->vbus_on);
	if ((g_sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB) ||
	    (g_sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB_CDP))
		oplus_notify_device_mode(true);

	sy6974b_inform_charger_type(sy);
	opluschg_updata_usb_type(sy);
	sy6974b_adc_start(sy, true);
	oplus_chg_wake_update_work();
	chg_info("oplus_chg_wake_update_work done\n");
	schedule_delayed_work(&sy->sy697x_aicl_work, OPLUS_DELAY_WORK_TIME_BASE*2);

	return 0;
}

static void opluschg_usbtemp_thread_init(struct oplus_chg_chip *oplus_chip)
{
	if (oplus_chip == NULL) {
		chg_err("failed to cread oplushg_usbtemp_kthread, oplus_chip == NULL\n");
		return;
	}
	if (oplus_usbtemp_check_is_support() == true) {
		oplushg_usbtemp_kthread = kthread_run(oplus_usbtemp_monitor_common, oplus_chip, "usbtemp_kthread");
		if (IS_ERR(oplushg_usbtemp_kthread))
			chg_err("failed to cread oplushg_usbtemp_kthread\n");
	}
}

static bool sy6974b_vbus_good(struct sy697x *sy)
{
	int rc;
	int reg_val = 0;
	bool bus_gd = false;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	rc = sy6974b_read_reg(sy, REG0A_SY6974B_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't oplus_chg_is_usb_present rc = %d\n", rc);
		return false;
	}

	bus_gd = ((reg_val & REG0A_SY6974B_BUS_GD_MASK) == REG0A_SY6974B_BUS_GD_YES) ? 1 : 0;
	return bus_gd;
}

static int oplus_splitchg_request_dpdm(struct sy697x *chg, bool enable)
{
	int rc = 0;

	if (!chg) {
		chg_err("chg is null\n");
		return 0;
	}

	/* fetch the DPDM regulator */
	chg_info("start enable[%d %d]\n", enable, chg->dpdm_enabled);
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
				"dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			rc = PTR_ERR(chg->dpdm_reg);
			chg_err("Couldn't get dpdm regulator rc=%d\n", rc);
			chg->dpdm_reg = NULL;
			return rc;
		}
	}

	mutex_lock(&chg->dpdm_lock);
	if (enable) {
		if (chg->dpdm_reg && !chg->dpdm_enabled) {
			chg_err(" enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
			if (rc < 0)
				chg_err("Couldn't enable dpdm regulator rc=%d\n", rc);
			else {
				chg->dpdm_enabled = true;
				chg_err("enabling DPDM success\n");
			}
		}
	} else {
		if (chg->dpdm_reg && chg->dpdm_enabled) {
			chg_err(" disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
			if (rc < 0)
				chg_err("Couldn't disable dpdm regulator rc=%d\n", rc);
			else {
				chg->dpdm_enabled = false;
				chg_err("disabling DPDM success\n");
			}
		}
	}
	mutex_unlock(&chg->dpdm_lock);
	chg_info("done\n");

	return rc;
}

#define OPLUS_BC12_RETRY_CNT 1
#define OPLUS_BC12_DELAY_CNT 18

int sy6974b_get_vbus_stat(void)
{
	int rc = 0;
	int vbus_stat = 0;
	struct sy697x *chip = g_sy;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_read_reg(chip, REG08_SY6974B_ADDRESS, &vbus_stat);
	if (rc) {
		chg_err("Couldn't read REG08_SY6974B_ADDRESS rc = %d\n", rc);
		return 0;
	}

	vbus_stat = vbus_stat & REG08_SY6974B_VBUS_STAT_MASK;

	return vbus_stat;
}

static int sy6974b_request_dpdm(struct sy697x *chip, bool enable)
{
	int ret = 0;

	if (!chip)
		return 0;
	/* fetch the DPDM regulator */
	if (!chip->dpdm_reg && of_get_property(chip->dev->of_node,
				"dpdm-supply", NULL)) {
		chip->dpdm_reg = devm_regulator_get(chip->dev, "dpdm");
		if (IS_ERR(chip->dpdm_reg)) {
			ret = PTR_ERR(chip->dpdm_reg);
			chg_err("Couldn't get dpdm regulator ret=%d\n", ret);
			chip->dpdm_reg = NULL;
			return ret;
		}
	}

	mutex_lock(&chip->dpdm_lock);
	if (enable) {
		if (chip->dpdm_reg && !chip->dpdm_enabled) {
			chg_err("enabling DPDM regulator\n");
			ret = regulator_enable(chip->dpdm_reg);
			if (ret < 0)
				chg_err("Couldn't enable dpdm regulator ret=%d\n", ret);
			else
				chip->dpdm_enabled = true;
		}
	} else {
		if (chip->dpdm_reg && chip->dpdm_enabled) {
			chg_err("disabling DPDM regulator\n");
			ret = regulator_disable(chip->dpdm_reg);
			if (ret < 0)
				chg_err("Couldn't disable dpdm regulator ret=%d\n", ret);
			else
				chip->dpdm_enabled = false;
		}
	}
	mutex_unlock(&chip->dpdm_lock);

	return ret;
}

static void sy6974b_bc12_retry_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sy697x *chip = container_of(dwork, struct sy697x, sy697x_bc12_retry_work);

	if (!oplus_chg_is_usb_present()) {
		chg_err("plugout during BC1.2,delay_cnt=%d,return\n", chip->bc12_delay_cnt);
		chip->bc12_delay_cnt = 0;
		return;
	}

	if (chip->bc12_delay_cnt >= OPLUS_BC12_DELAY_CNT) {
		chg_err("BC1.2 not complete delay_cnt to max\n");
		return;
	}
	chip->bc12_delay_cnt++;

	if (sy6974b_get_iindet()) {
		chg_err("BC1.2 complete,delay_cnt=%d\n", chip->bc12_delay_cnt);
		sy6974b_get_bc12(chip);
	} else {
		chg_err("BC1.2 not complete delay 50ms,delay_cnt=%d\n", chip->bc12_delay_cnt);
		schedule_delayed_work(&chip->sy697x_bc12_retry_work, round_jiffies_relative(msecs_to_jiffies(50)));
	}
}

static void sy6974b_start_bc12_retry(struct sy697x *chip)
{
	if (!chip)
		return;

	sy6974b_set_iindet(true);
	schedule_delayed_work(&chip->sy697x_bc12_retry_work, round_jiffies_relative(msecs_to_jiffies(100)));
}

static void sy6974b_get_bc12(struct sy697x *chip)
{
	int vbus_stat = 0;

	if (!chip)
		return;

	if (!chip->bc12_done) {
		vbus_stat = sy6974b_get_vbus_stat();
		switch (vbus_stat) {
		case REG08_SY6974B_VBUS_STAT_SDP:
			if (chip->bc12_retried < OPLUS_BC12_RETRY_CNT) {
				chip->bc12_retried++;
				chg_err("bc1.2 sdp retry cnt=%d\n", chip->bc12_retried);
				sy6974b_start_bc12_retry(chip);
				break;
			} else {
				oplus_notify_device_mode(true);
			}
			chip->bc12_done = true;
			oplus_sy6974b_check_ic_suspend();
			chip->oplus_chg_type = POWER_SUPPLY_TYPE_USB;
			sy6974b_inform_charger_type(chip);
			opluschg_updata_usb_type(chip);

			oplus_chg_wake_update_work();
			break;
		case REG08_SY6974B_VBUS_STAT_CDP:
			if (chip->bc12_retried < OPLUS_BC12_RETRY_CNT) {
				chip->bc12_retried++;
				chg_err("bc1.2 cdp retry cnt=%d\n", chip->bc12_retried);
				sy6974b_start_bc12_retry(chip);
				break;
			}

			chip->bc12_done = true;
			oplus_sy6974b_check_ic_suspend();
			chip->oplus_chg_type = POWER_SUPPLY_TYPE_USB_CDP;
			sy6974b_inform_charger_type(chip);
			opluschg_updata_usb_type(chip);

			oplus_notify_device_mode(true);
			oplus_chg_wake_update_work();
			break;
		case REG08_SY6974B_VBUS_STAT_DCP:
		case REG08_SY6974B_VBUS_STAT_OCP:
		case REG08_SY6974B_VBUS_STAT_FLOAT:
			chip->bc12_done = true;

			oplus_sy6974b_check_ic_suspend();

			chip->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;

			sy6974b_inform_charger_type(chip);
			opluschg_updata_usb_type(chip);

			oplus_chg_wake_update_work();
			break;
		case REG08_SY6974B_VBUS_STAT_OTG_MODE:
		case REG08_SY6974B_VBUS_STAT_UNKNOWN:
		default:
			break;
		}
	}
}

#define OPLUS_WAIT_RESUME_TIME	200
static irqreturn_t sy6974b_irq_handler(int irq, void *data)
{
	struct sy697x *sy = (struct sy697x *)data;
	int ret = 0;
	int reg_val = 0;
	bool prev_pg, curr_pg;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		oplus_keep_resume_wakelock(sy, false);
		return IRQ_HANDLED;
	}

	if (oplus_get_otg_online_status_default()) {
		chg_err("otg,ignore\n");
		oplus_keep_resume_wakelock(sy, false);
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;

		sy6974b_inform_charger_type(sy);
		opluschg_updata_usb_type(sy);
		return IRQ_HANDLED;
	}

	chg_err("enter improve irq time\n");
	oplus_keep_resume_wakelock(sy, true);

	/* for check bus i2c/spi is ready or not */
	if (atomic_read(&sy->driver_suspended) == 1) {
		pr_notice(" sy697x_irq_handler:suspended and wait_event_interruptible %d\n", OPLUS_WAIT_RESUME_TIME);
		wait_event_interruptible_timeout(sy->wait, atomic_read(&sy->driver_suspended) == 0, msecs_to_jiffies(OPLUS_WAIT_RESUME_TIME));
	}

	if (sy->is_force_dpdm) {
		sy->is_force_dpdm = false;
		sy6974b_force_dpdm(sy, false);
		pr_notice("sy697x_force_dpdm:false\n");
	}

	/* first start bc12 */
	if (irq == PROBE_PLUG_IN_IRQ) {
		sy->vbus_on = false;
		sy->vbus_on = sy6974b_vbus_good(sy);
		chg_info("sy697x vbus_on[%d]\n", sy->vbus_on);
	}

	ret = sy6974b_read_reg(sy, REG08_SY6974B_ADDRESS, &reg_val);
	if (ret) {
		chg_err("REG08_SY6974B_ADDRESS read failed ret[%d]\n", ret);
		oplus_keep_resume_wakelock(sy, false);
		return IRQ_HANDLED;
	}
	curr_pg = ((reg_val & REG08_SY6974B_POWER_GOOD_STAT_MASK) == REG08_SY6974B_POWER_GOOD_STAT_GOOD) ? 1 : 0;

	if (curr_pg) {
		oplus_chg_wakelock(sy, true);
	}
	prev_pg = sy->power_good;

	pr_notice("[%s]:(%d,%d %d, otg[%d])\n", __func__,
		prev_pg, sy->power_good, reg_val, oplus_get_otg_enable());

	sy6974b_dump_regs(sy);
	oplus_chg_track_check_wired_charging_break(curr_pg);

	if (oplus_vooc_get_fastchg_started() == true &&
		oplus_vooc_get_adapter_update_status() != 1) {
		chg_info("oplus_vooc_get_fastchg_started = true! (prev_pg curr_pg):(%d %d)\n",
			prev_pg, curr_pg);
		sy->power_good = curr_pg;
		if (prev_pg && !sy->power_good) {
			oplus_chg_wakelock(sy, false);
			chg_err("oplus_vooc_get_fastchg_started = true and adapter/usb pg_good removed\n");
		}
		goto power_change;
	} else {
		sy->power_good = curr_pg;
	}

	if (!prev_pg && sy->power_good) {
		oplus_chg_wakelock(sy, true);
		sy6974b_request_dpdm(sy, true);
		sy->bc12_done = false;
		sy->bc12_retried = 0;
		sy->bc12_delay_cnt = 0;
		oplus_voocphy_set_adc_enable(true);
		sy6974b_set_wdt_timer(REG05_SY6974B_WATCHDOG_TIMER_40S);
		oplus_wake_up_usbtemp_thread();
		if (sy->oplus_chg_type == POWER_SUPPLY_TYPE_UNKNOWN)
			sy6974b_get_bc12(sy);

		if (g_oplus_chip) {
			if (oplus_vooc_get_fastchg_to_normal() == false &&
				oplus_vooc_get_fastchg_to_warm() == false) {
				if (g_oplus_chip->authenticate &&
					g_oplus_chip->mmi_chg &&
					oplus_vooc_get_allow_reading() &&
					!oplus_is_rf_ftm_mode())
					oplus_sy6974b_charging_enable();
			}
		}
		goto power_change;
	} else if (prev_pg && !sy->power_good) {
		sy6974b_request_dpdm(sy, false);
		sy->bc12_done = false;
		sy->bc12_retried = 0;
		sy->bc12_delay_cnt = 0;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;

		sy6974b_inform_charger_type(sy);
		opluschg_updata_usb_type(sy);

		sy6974b_set_wdt_timer(REG05_SY6974B_WATCHDOG_TIMER_DISABLE);
		oplus_vooc_reset_fastchg_after_usbout();
		if (oplus_vooc_get_fastchg_started() == false) {
			oplus_chg_set_chargerid_switch_val(0);
			oplus_chg_clear_chargerid_info();
		}
		oplus_chg_set_charger_type_unknown();
		oplus_chg_wake_update_work();
		oplus_wake_up_usbtemp_thread();
		oplus_notify_device_mode(false);
		oplus_voocphy_set_adc_enable(false);
		oplus_chg_wakelock(sy, false);
		goto power_change;
	} else if (!prev_pg && !sy->power_good) {
		chg_err("prev_pg & now_pg is false\n");
		sy->bc12_done = false;
		sy->bc12_retried = 0;
		sy->bc12_delay_cnt = 0;
		goto power_change;
	}

	sy6974b_get_bc12(sy);

	if (g_sy->otg_enable) {
		oplus_keep_resume_wakelock(sy, false);
		return IRQ_HANDLED;
	}

power_change:
	if (dumpreg_by_irq)
		sy6974b_dump_regs(sy);
	oplus_keep_resume_wakelock(sy, false);
	return IRQ_HANDLED;
}

static int oplus_chgirq_gpio_init(struct sy697x *sy)
{
	int rc = 0;
	struct device_node *node = sy->dev->of_node;

	if (!node) {
		chg_err("device tree node missing\n");
		return -EINVAL;
	}
	sy->irq_gpio = of_get_named_gpio(node,
		"qcom,chg_irq_gpio", 0);
	if (sy->irq_gpio < 0) {
		chg_err("sy->irq_gpio not specified\n");
	} else {
		if (gpio_is_valid(sy->irq_gpio)) {
			rc = gpio_request(sy->irq_gpio,
				"chg_irq_gpio");
			if (rc) {
				chg_err("unable to request gpio [%d]\n",
					sy->irq_gpio);
			}
		}
		chg_info("sy->irq_gpio =%d\n", sy->irq_gpio);
	}

	sy->irq = gpio_to_irq(sy->irq_gpio);
	chg_info("irq way1 sy->irq =%d\n", sy->irq);

	sy->irq = irq_of_parse_and_map(node, 0);
	chg_info("irq way2 sy->irq =%d\n", sy->irq);

	/* set splitchg pinctrl */
	sy->pinctrl = devm_pinctrl_get(sy->dev);
	if (IS_ERR_OR_NULL(sy->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	sy->splitchg_inter_active =
		pinctrl_lookup_state(sy->pinctrl, "splitchg_inter_active");
	if (IS_ERR_OR_NULL(sy->splitchg_inter_active)) {
		chg_err(": %d Failed to get the state pinctrl handle\n", __LINE__);
		return -EINVAL;
	}

	sy->splitchg_inter_sleep =
		pinctrl_lookup_state(sy->pinctrl, "splitchg_inter_sleep");
	if (IS_ERR_OR_NULL(sy->splitchg_inter_sleep)) {
		chg_err(": %d Failed to get the state pinctrl handle\n", __LINE__);
		return -EINVAL;
	}

	gpio_direction_input(sy->irq_gpio);
	pinctrl_select_state(sy->pinctrl, sy->splitchg_inter_active); /* no_PULL */

	rc = gpio_get_value(sy->irq_gpio);
	chg_info("sy->irq_gpio input =%d irq_gpio_stat = %d\n", sy->irq_gpio, rc);

	return 0;
}


static int sy6974b_register_interrupt(struct device_node *np, struct sy697x *sy)
{
	int ret;

	oplus_chgirq_gpio_init(sy);

	chg_info("irq = %d\n", sy->irq);

	ret = devm_request_threaded_irq(sy->dev, sy->irq, NULL,
					sy6974b_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					sy->eint_name, sy);
	if (ret < 0) {
		chg_err("request thread irq failed:%d\n", ret);
		return ret;
	}

	enable_irq_wake(sy->irq);

	return 0;
}

static int bq2589x_init_device(struct sy697x *sy)
{
	int ret;

	sy6974b_disable_watchdog_timer(sy);
	sy->is_force_dpdm = false;
	ret = sy6974b_set_prechg_current(sy, sy->platform_data->iprechg);
	if (ret)
		chg_err("Failed to set prechg current, ret = %d\n", ret);

	ret = sy6974b_set_term_current(sy, sy->platform_data->iterm);
	if (ret)
		chg_err("Failed to set termination current, ret = %d\n", ret);

	ret = sy6974b_set_boost_voltage(sy, sy->platform_data->boostv);
	if (ret)
		chg_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = sy6974b_set_boost_current(sy, sy->platform_data->boosti);
	if (ret)
		chg_err("Failed to set boost current, ret = %d\n", ret);

	ret = sy6974b_enable_auto_dpdm(sy, false);
	if (ret)
		chg_err("Failed to stop auto dpdm, ret = %d\n", ret);

	ret = sy6974b_vmin_limit(sy);
	if (ret)
		chg_err("Failed to set vmin limit, ret = %d\n", ret);

	ret = sy6974b_adc_start(sy, true);
	if (ret)
		chg_err("Failed to stop adc, ret = %d\n", ret);

	ret = sy6974b_set_input_volt_limit(sy, SY6974B_INPUT_VOLT_LIMIT_DEFAULT_VALUE);
	if (ret)
		chg_err("Failed to set input volt limit, ret = %d\n", ret);

	sy->boot_mode = get_boot_mode();
	if (sy->boot_mode == MSM_BOOT_MODE__RF || sy->boot_mode == MSM_BOOT_MODE__WLAN)
		sy6974b_enter_hiz_mode(sy);

	return 0;
}

int sy6974b_set_ovp(int val)
{
	int rc;
	struct sy697x *chip = g_sy;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_config_interface(chip, REG06_SY6974B_ADDRESS,
			val, REG06_SY6974B_OVP_MASK);

	return rc;
}

static int sy6974b_init_device(struct sy697x *sy)
{
	int ret;

	sy6974b_disable_watchdog_timer(sy);
	sy->is_force_dpdm = false;
	sy6974b_set_ovp(REG06_SY6974B_OVP_14P0V);
	ret = sy6974b_set_prechg_current(sy, sy->platform_data->iprechg);
	if (ret)
		chg_err("Failed to set prechg current, ret = %d\n", ret);

	ret = sy6974b_set_chargevolt(sy, DEFAULT_CV);
	if (ret)
		chg_err("Failed to set default cv, ret = %d\n", ret);

	ret = sy6974b_set_term_current(sy, sy->platform_data->iterm);
	if (ret)
		chg_err("Failed to set termination current, ret = %d\n", ret);

	ret = sy6974b_set_boost_voltage(sy, sy->platform_data->boostv);
	if (ret)
		chg_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = sy6974b_set_boost_current(sy, sy->platform_data->boosti);
	if (ret)
		chg_err("Failed to set boost current, ret = %d\n", ret);
	ret = sy6974b_disable_enlim(sy);
	if (ret)
		chg_err("Failed to sy697x_disable_enlim, ret = %d\n", ret);
	ret = sy6974b_enable_auto_dpdm(sy, false);
	if (ret)
		chg_err("Failed to stop auto dpdm, ret = %d\n", ret);

	ret = sy6974b_vmin_limit(sy);
	if (ret)
		chg_err("Failed to set vmin limit, ret = %d\n", ret);

	ret = sy6974b_set_input_volt_limit(sy, SY6974B_INPUT_VOLT_LIMIT_DEFAULT_VALUE);
	if (ret)
		chg_err("Failed to set input volt limit, ret = %d\n", ret);

	sy->boot_mode = get_boot_mode();
	if (sy->boot_mode == MSM_BOOT_MODE__RF || sy->boot_mode == MSM_BOOT_MODE__WLAN) {
		sy6974b_enter_hiz_mode(sy);
	}
	return 0;
}

void sy6974b_initial_status(bool is_charger_on)
{
	if (!g_sy)
		return;

	if (is_charger_on) {
		oplus_for_cdp();
		sy6974b_enable_auto_dpdm(g_sy, false);
		g_sy->is_force_aicl = true;
		g_sy->is_retry_bc12 = true;
		sy6974b_force_dpdm(g_sy, true);
		g_sy->is_force_dpdm = true;
	} else {
		g_sy->is_force_dpdm = false;
	}
}
EXPORT_SYMBOL_GPL(sy6974b_initial_status);

#define REG0B_SY6974B_DEV_REV_MASK    (BIT(1) | BIT(0))
#define REG0B_SY6974B_DEV_REV_00    0x00

static int sy6974b_detect_device(struct sy697x *sy)
{
	int ret;
	int data;

	ret = sy6974b_read_reg(sy, REG0B_SY6974B_ADDRESS, &data);
	if (!ret) {
		sy->part_no = data & REG0B_SY6974B_PN_MASK;
		sy->revision = data & REG0B_SY6974B_DEV_REV_MASK;
	}
	chg_info("part_no=%d,revision=%d\n", sy->part_no, sy->revision);
	if (is_bq25890h(sy) == false)
		chg_info("sy6974b\n");
	else
		chg_info("bq25890h\n");

	return 0;
}

static void sy6974b_dump_regs(struct sy697x *sy)
{
	int ret;
	int addr = 0;
	u8 val_buf[SY6974B_REG_NUMBER] = {0x0};

	if (!sy)
		return;

	if (atomic_read(&sy->driver_suspended) == 1)
		return;

	ret = sy6974b_read_block(sy, SY6974B_FIRST_REG, SY6974B_REG_NUMBER, val_buf);
	if (ret)
		chg_err("Couldn't read 0x%02x length=%d ret = %d\n", addr, SY6974B_REG_NUMBER, ret);

	chg_info("[0x%02x, 0x%02x, 0x%02x, 0x%02x], [0x%02x, 0x%02x, 0x%02x, 0x%02x], "
		"[0x%02x, 0x%02x, 0x%02x, 0x%02x]\n",
		val_buf[0], val_buf[1], val_buf[2], val_buf[3],
		val_buf[4], val_buf[5], val_buf[6], val_buf[7],
		val_buf[8], val_buf[9], val_buf[10], val_buf[11]);
}

static bool is_bq25890h(struct sy697x *sy)
{
	if (sy->part_no == REG0B_SY6974B_PN && sy->revision == REG0B_SY6974B_DEV_REV)
		return false; /* chip is sy6974b */
	else
		return true; /* chip is bq25890h */
}

static int _sy6974b_get_ichg(struct sy697x *sy, u32 *curr)
{
	int rc;
	int tmp = 0;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;


	rc = sy6974b_read_reg(sy, REG02_SY6974B_ADDRESS, &tmp);
	if (rc) {
		chg_err("Couldn't read REG02_SY6974B_ADDRESS rc = %d\n", rc);
		return 0;
	}

	tmp = (tmp & REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_MASK) >> REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_SHIFT;
	*curr = (tmp * REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_STEP + REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_OFFSET) * 1000;
	return *curr;
}

static int sy6974b_enter_ship_mode(struct sy697x *sy, bool en)
{
	int val = 0;
	int rc;

	if (!sy)
		return 0;

	if (en) {
		val = SY6974_BATFET_OFF << REG07_SY6974B_BATFET_DIS_SHIFT;
	} else {
		val = SY6974_BATFET_ON << REG07_SY6974B_BATFET_DIS_SHIFT;
	}
	rc = sy6974b_config_interface(sy, REG07_SY6974B_ADDRESS, val, REG07_SY6974B_BATFET_DIS_MASK);

	chg_info("enter ship_mode:done, en=%d\n", en);
	return rc;
}

static int sy6974b_enable_shipmode(bool en)
{
	return sy6974b_enter_ship_mode(g_sy, en);
}

static bool oplus_check_chrdet_status(void);

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
			chg_info(" %s channel unavailable, %d\n", propname, rc);
		*chan = NULL;
	}

	return rc;
}

#define NTC_DEFAULT_VOLT_VALUE_MV 950
#define THERMAL_TEMP_UNIT      1000
#define NTC_TEMP_DEGREE_VALUE_DEFAULT	25
static int oplus_get_ntc_tmp(struct iio_channel *channel)
{
	int ntc_vol_cur = 0;
	struct sy697x *chip = g_sy;
	static int ntc_vol = NTC_DEFAULT_VOLT_VALUE_MV;
	static int ntc_vol_pre = NTC_DEFAULT_VOLT_VALUE_MV;
	int ntc_temp = NTC_TEMP_DEGREE_VALUE_DEFAULT;
	int ntc_vol1 = 0, ntc_temp1 = 0, ntc_vol2 = 0, ntc_temp2 = 0;
	int i = 0;
	int rc = 0;
	struct oplus_chg_chip *opluschg = g_oplus_chip;

	if (!chip || !opluschg) {
		chg_err("sy697x not ready!\n");
		return NTC_TEMP_DEGREE_VALUE_DEFAULT;
	}

	if (IS_ERR_OR_NULL(channel)) {
		chg_err("channel is  NULL !\n");
		ntc_vol = ntc_vol_pre;
		goto ntcvolt_get_done;
	}
	rc = iio_read_channel_processed(channel, &ntc_vol_cur);
	if (rc < 0) {
		chg_info("fail to read usb_temp1 adc rc = %d\n", rc);
		ntc_vol = ntc_vol_pre;
		goto ntcvolt_get_done;
	}

	if (ntc_vol_cur <= 0) {
		chg_info("ntc_vol_cur iio_read_channel_processed  get error\n");
		ntc_vol = ntc_vol_pre;
		goto ntcvolt_get_done;
	}

	ntc_vol_cur = ntc_vol_cur / 1000;
	ntc_vol = ntc_vol_cur;
	ntc_vol_pre = ntc_vol_cur;
ntcvolt_get_done:
	/* map btb temp by usbtemp table */
	if (opluschg->con_volt[opluschg->len_array- 1] >= ntc_vol) {
		ntc_temp = opluschg->con_temp[opluschg->len_array- 1] * THERMAL_TEMP_UNIT;
	} else if (opluschg->con_volt[0] <= ntc_vol) {
		ntc_temp = opluschg->con_temp[0] * THERMAL_TEMP_UNIT;
	} else {
		for (i = opluschg->len_array- 1; i >= 0; i--) {
			if (opluschg->con_volt[i] >= ntc_vol) {
				ntc_vol2 = opluschg->con_volt[i];
				ntc_temp2 = opluschg->con_temp[i];
				break;
			}
			ntc_vol1 = opluschg->con_volt[i];
			ntc_temp1 = opluschg->con_temp[i];
		}
		ntc_temp = (((ntc_vol - ntc_vol2) * ntc_temp1) + ((ntc_vol1 - ntc_vol) * ntc_temp2)) *
						THERMAL_TEMP_UNIT / (ntc_vol1 - ntc_vol2);
	}

	return ntc_temp;
}

static void oplus_sy6974b_get_usbtemp_volt(struct oplus_chg_chip *chip)
{
	int usbtemp_volt = 0;
	struct sy697x *chg = g_sy;
	static int usbtemp_volt_l_pre = NTC_DEFAULT_VOLT_VALUE_MV;
	static int usbtemp_volt_r_pre = NTC_DEFAULT_VOLT_VALUE_MV;
	int rc = 0;

	if (!chip || !chg) {
		chg_err("smb5_chg not ready!\n");
		return;
	}

	if (IS_ERR_OR_NULL(chg->iio.usb_temp_chan1)) {
		chg_err("chg->iio.usb_temp_chan1  is  NULL !\n");
		chip->usbtemp_volt_l = usbtemp_volt_l_pre;
		rc = oplus_get_iio_channel(chg, "usb_temp1", &chg->iio.usb_temp_chan1);
		if (rc < 0 && !chg->iio.usb_temp_chan1) {
			chg_err("usb_temp_chan1 get failed\n");
		}

		goto usbtemp_next;
	}

	rc = iio_read_channel_processed(chg->iio.usb_temp_chan1, &usbtemp_volt);
	if (rc < 0) {
		chg_err("fail to read usb_temp1 adc rc = %d\n", rc);
		chip->usbtemp_volt_l = usbtemp_volt_l_pre;
		goto usbtemp_next;
	}
	if (usbtemp_volt <= 0) {
		chg_err("USB_TEMPERATURE1 iio_read_channel_processed  get error\n");
		chip->usbtemp_volt_l = usbtemp_volt_l_pre;
		goto usbtemp_next;
	}

	usbtemp_volt = usbtemp_volt / 1000;
	chip->usbtemp_volt_l = usbtemp_volt;
	usbtemp_volt_l_pre = usbtemp_volt;
usbtemp_next:
	usbtemp_volt = 0;
	if (IS_ERR_OR_NULL(chg->iio.usb_temp_chan2)) {
		chg_err("chg->iio.usb_temp_chan2  is  NULL !\n");
		rc = oplus_get_iio_channel(chg, "usb_temp2", &chg->iio.usb_temp_chan2);
		if (rc < 0 && !chg->iio.usb_temp_chan2) {
			chg_err("usb_temp_chan2 get failed\n");
		}

		chip->usbtemp_volt_r = usbtemp_volt_r_pre;
		return;
	}

	rc = iio_read_channel_processed(chg->iio.usb_temp_chan2, &usbtemp_volt);
	if (rc < 0) {
		chg_err("fail to read usb_temp2 adc rc = %d\n", rc);
		chip->usbtemp_volt_r = usbtemp_volt_r_pre;
		return;
	}
	if (usbtemp_volt <= 0) {
		chg_err("USB_TEMPERATURE2 iio_read_channel_processed  get error\n");
		chip->usbtemp_volt_r = usbtemp_volt_r_pre;
		return;
	}

	usbtemp_volt = usbtemp_volt / 1000;
	chip->usbtemp_volt_r = usbtemp_volt;
	usbtemp_volt_r_pre = usbtemp_volt;
}

static int oplus_thermal_get_tmp(void)
{
	int ntcctrl_gpio_value;
	int ret = 0;
	struct sy697x *chip = g_sy;

	if (!chip) {
		chg_err("chip or chg not ready!\n");
		return -1;
	}

	if (!chip->pinctrl
		|| !chip->iio.ntc_switch1_chan || !chip->iio.ntc_switch2_chan) {
		chg_err("chip not ready!\n");

		ret = oplus_get_iio_channel(chip, "ntc_switch1_chan", &chip->iio.ntc_switch1_chan);
		if (ret < 0 && !chip->iio.ntc_switch1_chan) {
			chg_err("ntc_switch1_chan get failed\n");
			return -1;
		}

		ret = oplus_get_iio_channel(chip, "ntc_switch2_chan", &chip->iio.ntc_switch2_chan);
		if (ret < 0 && !chip->iio.ntc_switch2_chan) {
			chg_err("ntc_switch2_chan get failed\n");
			return -1;
		}
	}

	ntcctrl_gpio_value = gpio_get_value(chip->ntcctrl_gpio);
	if (ntcctrl_gpio_value == 0) {
		chg_thermal_temp = oplus_get_ntc_tmp(chip->iio.ntc_switch1_chan);
		bb_thermal_temp = oplus_get_ntc_tmp(chip->iio.ntc_switch2_chan);
		pinctrl_select_state(chip->pinctrl, chip->ntc_switch_high);
		msleep(100);
		ret = gpio_get_value(chip->ntcctrl_gpio);
		flash_thermal_temp = oplus_get_ntc_tmp(chip->iio.ntc_switch1_chan);
		board_thermal_temp = oplus_get_ntc_tmp(chip->iio.ntc_switch2_chan);
	} else if (ntcctrl_gpio_value == 1) {
		flash_thermal_temp = oplus_get_ntc_tmp(chip->iio.ntc_switch1_chan);
		board_thermal_temp = oplus_get_ntc_tmp(chip->iio.ntc_switch2_chan);
		pinctrl_select_state(chip->pinctrl, chip->ntc_switch_low);
		msleep(100);
		ret = gpio_get_value(chip->ntcctrl_gpio);
		chg_thermal_temp = oplus_get_ntc_tmp(chip->iio.ntc_switch1_chan);
		bb_thermal_temp = oplus_get_ntc_tmp(chip->iio.ntc_switch2_chan);
	}

	return 0;
}

int oplus_sy6974b_thermal_tmp_get_chg(void)
{
	if (g_sy) {
		mutex_lock(&g_sy->ntc_lock);
		oplus_thermal_get_tmp();
		mutex_unlock(&g_sy->ntc_lock);
	}

	return chg_thermal_temp;
}

int oplus_sy6974b_thermal_tmp_get_bb(void)
{
	if (g_sy) {
		mutex_lock(&g_sy->ntc_lock);
		oplus_thermal_get_tmp();
		mutex_unlock(&g_sy->ntc_lock);
	}

	return bb_thermal_temp;
}

int oplus_sy6974b_thermal_tmp_get_flash(void)
{
	if (g_sy) {
		mutex_lock(&g_sy->ntc_lock);
		oplus_thermal_get_tmp();
		mutex_unlock(&g_sy->ntc_lock);
	}

	return flash_thermal_temp;
}

int oplus_sy6974b_thermal_tmp_get_board(void)
{
	if (g_sy) {
		mutex_lock(&g_sy->ntc_lock);
		oplus_thermal_get_tmp();
		mutex_unlock(&g_sy->ntc_lock);
	}

	return board_thermal_temp;
}

static int oplus_ntc_switch_gpio_init(void)
{
	struct sy697x *chip = g_sy;

	if (!chip) {
		chg_err("chip is null ! \n");
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_info("get ntc_switch_gpio pinctrl fail\n");
		return -EINVAL;
	}

	chip->ntc_switch_high = pinctrl_lookup_state(chip->pinctrl, "ntc_switch_high");
	if (IS_ERR_OR_NULL(chip->ntc_switch_high)) {
		chg_info("get ntc_switch_high fail\n");
		return -EINVAL;
	}

	chip->ntc_switch_low = pinctrl_lookup_state(chip->pinctrl, "ntc_switch_low");
	if (IS_ERR_OR_NULL(chip->ntc_switch_low)) {
		chg_info("get ntc_switch_low fail\n");
		return -EINVAL;
	}

	/* default switch chg_thermal and bb_thermal */
	pinctrl_select_state(chip->pinctrl, chip->ntc_switch_low);

	chg_info("ntc_switch is ready!\n");
	return 0;
}

static int oplus_ntc_switch_parse_dt(void)
{
	struct sy697x *chip = g_sy;
	int rc = 0;
	struct device_node * node = NULL;

	if (!chip) {
		chg_err("chip null\n");
		return -1;
	}

	/* Parsing ntcctrl gpio */
	node = chip->dev->of_node;
	chip->ntcctrl_gpio = of_get_named_gpio(node, "qcom,ntc-switch-gpio", 0);

	if (chip->ntcctrl_gpio < 0) {
		chg_err("chip->ntcctrl_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->ntcctrl_gpio)) {
			rc = gpio_request(chip->ntcctrl_gpio,
				"ntc-switch-gpio");
			if (rc) {
				chg_err("unable to request gpio [%d]\n",
					chip->ntcctrl_gpio);
			} else {
				rc = oplus_ntc_switch_gpio_init();
				if (rc)
					chg_err("unable to init charging_sw_ctrl2-gpio:%d\n",
							chip->ntcctrl_gpio);
			}
		}
		chg_info("chip->ntcctrl_gpio =%d\n", chip->ntcctrl_gpio);
	}

	return rc;
}

static int oplus_chg_plt_init_for_qcom(struct sy697x *chg)
{
	if (!chg) {
		chg_err("sy697x is null \n");
		return -1;
	}
	if (oplus_ntc_switch_parse_dt()) {
		chg_err("ntc gpio init failed\n");
		return -1;
	}
	chg_info("ntc channel init success \n");

	return 0;
}

#define DUMP_REG_LOG_CNT_30S 6
static void oplus_sy6974b_dump_registers(void)
{
	static int dump_count = 0;

	if (!g_oplus_chip)
		return;

	if (g_oplus_chip->charger_exist ||
		g_oplus_chip->ac_online ||
		(sy6974b_debug & ENABLE_DUMP_LOG))
		dump_count = DUMP_REG_LOG_CNT_30S;

	if(dump_count >= DUMP_REG_LOG_CNT_30S) {
		dump_count = 0;
		sy6974b_dump_regs(g_sy);
	} else {
		dump_count++;
	}
}

static int oplus_sy6974b_kick_wdt(void)
{
	oplus_check_chrdet_status();
	return sy6974b_reset_watchdog_timer(g_sy);
}

static int oplus_sy6974b_set_ichg(int cur)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	u32 chg_curr = cur * 1000;
	u32 main_cur, slave_cur;
	int ret = 0;
	int boot_mode = get_boot_mode();

	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) {
		chg_curr = 0;
		cur = 0;
		dev_info(g_sy->dev, "%s: boot_mode[%d] curr = %d\n", __func__, boot_mode, chg_curr);
	}

	chg_info("curr = %d\n", cur);

	if (chip->em_mode) {
		chg_curr = chip->limits.temp_normal_phase2_fastchg_current_ma_high * 1000;
	}
	if (chip->is_double_charger_support
			&& (chip->slave_charger_enable || chip->em_mode)) {
		main_cur = chg_curr  * current_percent / 100;
		ret = sy6974b_set_chargecurrent(g_sy, main_cur / 1000);
		if (ret < 0) {
			chg_debug("set fast charge current:%d fail\n", main_cur);
		}
		ret = _sy6974b_get_ichg(g_sy, &main_cur);
		if (ret < 0) {
			chg_debug("get fast charge current:%d fail\n", main_cur);
			return ret;
		}
		slave_cur = chg_curr - main_cur;
		chip->sub_chg_ops->charging_current_write_fast(slave_cur / 1000);
	} else {
		ret = sy6974b_set_chargecurrent(g_sy, chg_curr / 1000);
		if (ret < 0)
			chg_debug("set fast charge current:%d fail\n", chg_curr);
	}
	return 0;
}

static void oplus_sy6974b_set_mivr(int vbatt)
{
	u32 mv = 0;

	if (vbatt > SY6974B_SET_MIVR_BASE_BATT_VOL_4300MV) {
		mv = vbatt + SY6974B_SET_MIVR_BASE_BATT_VOL_OFFSET_400MV;
	} else if (vbatt > SY6974B_SET_MIVR_BASE_BATT_VOL_4200MV) {
		mv = vbatt + SY6974B_SET_MIVR_BASE_BATT_VOL_OFFSET_300MV;
	} else {
		mv = vbatt + SY6974B_SET_MIVR_BASE_BATT_VOL_OFFSET_200MV;
	}

	if (mv < SY6974B_SET_MIVR_BASE_BATT_VOL_4200MV)
		mv = SY6974B_SET_MIVR_BASE_BATT_VOL_4200MV;

	sy6974b_set_input_volt_limit(g_sy, mv);
}

static int usb_icl[] = {
	100, 500, 900, 1200, 1500, 1750, 2000, 3000,
};

static int oplus_sy6974b_set_aicr(int current_ma)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	int rc = 0;
	int i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	int aicl_point_temp = 0;
	int main_cur = 0;
	int slave_cur = 0;

	g_sy->pre_current_ma = current_ma;
	if (chip->is_double_charger_support) {
		rc = chip->sub_chg_ops->charging_disable();
		if (rc < 0) {
			chg_debug("disable sub charging fail\n");
		}
		dev_info(g_sy->dev, "%s disabel subchg\n", __func__);
	}

	dev_info(g_sy->dev, "%s usb input max current limit=%d\n", __func__, current_ma);
	if (chip && chip->is_double_charger_support == true) {
		chg_vol = oplus_sy6974b_get_vbus();
		if (chg_vol > CHARGER_VOLTAGE_7600MV) {
			aicl_point_temp = aicl_point = CHARGER_VOLTAGE_7600MV;
		} else {
			if (chip->batt_volt > SY6974B_AICL_POINT_BASE_BATT_VOL_4100MV)
				aicl_point_temp = aicl_point = SY6974B_AICL_POINT_VOL_5V_PHASE2;
			else
				aicl_point_temp = aicl_point = SY6974B_AICL_POINT_VOL_5V_PHASE1;
		}
	} else {
		if (chip->batt_volt > SY6974B_AICL_POINT_BASE_BATT_VOL_4100MV)
			aicl_point_temp = aicl_point = SY6974B_AICL_POINT_VOL_5V_PHASE2;
		else
			aicl_point_temp = aicl_point = SY6974B_AICL_POINT_VOL_5V_PHASE1;
	}

	if (current_ma < 500) {
		i = 0;
		goto aicl_end;
	}

	i = 1; /* 500 */
	sy6974b_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(90);
	sy6974b_disable_enlim(g_sy);
	chg_vol = oplus_sy6974b_get_vbus();
	if (chg_vol < aicl_point_temp) {
		chg_info("use 500 here\n");
		goto aicl_end;
	} else if (current_ma < 900)
		goto aicl_end;

	i = 2; /* 900 */
	sy6974b_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(90);
	chg_vol = oplus_sy6974b_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 1200)
		goto aicl_end;

	i = 3; /* 1200 */
	sy6974b_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(90);
	chg_vol = oplus_sy6974b_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1500 */
	aicl_point_temp = aicl_point + 50;
	sy6974b_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(120);
	chg_vol = oplus_sy6974b_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 2; /* We DO NOT use 1.2A here */
		goto aicl_pre_step;
	} else if (current_ma < 1500) {
		i = i - 1; /* We use 1.2A here */
		goto aicl_end;
	} else if (current_ma < 2000)
		goto aicl_end;

	i = 5; /* 1750 */
	aicl_point_temp = aicl_point + 50;
	sy6974b_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(120);
	chg_vol = oplus_sy6974b_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 2; /* 1.2 */
		goto aicl_pre_step;
	}

	i = 6; /* 2000 */
	aicl_point_temp = aicl_point;
	sy6974b_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(90);
	chg_vol = oplus_sy6974b_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 2;/* 1.5 */
		goto aicl_pre_step;
	} else if (current_ma < 3000)
		goto aicl_end;

	i = 7; /* 3000 */
	sy6974b_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(90);
	chg_vol = oplus_sy6974b_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma >= 3000)
		goto aicl_end;

aicl_pre_step:
	if (chip->is_double_charger_support
			&& (chip->slave_charger_enable || chip->em_mode)) {
		chg_debug("enable sgm41511x for charging\n");

		main_cur = (usb_icl[i] * current_percent) / 100;
		main_cur -= main_cur % 50;
		slave_cur = usb_icl[i] - main_cur;
		sy6974b_set_input_current_limit(g_sy, main_cur);
		chip->sub_chg_ops->input_current_write(slave_cur);

		rc = chip->sub_chg_ops->charging_enable();
		if (rc < 0)
			chg_debug("enable sub charging fail\n");

		if (chip->em_mode && !chip->slave_charger_enable)
			chip->slave_charger_enable = true;

		chg_debug("usb input max current limit aicl: master and salve input current: %d, %d\n",
				main_cur, slave_cur);
	} else {
		sy6974b_set_input_current_limit(g_sy, usb_icl[i]);
	}

	dev_info(g_sy->dev, "%s:usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n",
			__func__, chg_vol, i, usb_icl[i], aicl_point_temp);
	if (g_oplus_chip && !g_oplus_chip->stop_chg) {
		sy6974b_set_input_current_limit(g_sy, SY697X_IINLIM_BASE);
		chg_info("aicl_pre_step. stop_chg flag found 0");
	} else {
		oplus_sy6974b_check_ic_suspend();
	}
	return rc;
aicl_end:
	if (chip->is_double_charger_support
			&& (chip->slave_charger_enable || chip->em_mode)) {
		chg_debug("enable sgm41511x for charging\n");

		main_cur = (usb_icl[i] * current_percent) / 100;
		main_cur -= main_cur % 50;
		slave_cur = usb_icl[i] - main_cur;
		sy6974b_set_input_current_limit(g_sy, main_cur);
		chip->sub_chg_ops->input_current_write(slave_cur);
		rc = chip->sub_chg_ops->charging_enable();
		if (rc < 0)
			chg_debug("enable sub charging fail\n");

		if (chip->em_mode && !chip->slave_charger_enable)
			chip->slave_charger_enable = true;

		chg_debug("usb input max current limit aicl: master and salve input current: %d, %d\n", main_cur, slave_cur);
	} else {
		sy6974b_set_input_current_limit(g_sy, usb_icl[i]);
	}

	if (chip->em_mode) {
		chip->charger_volt = chg_vol;
		power_supply_changed(chip->batt_psy);
	}
	dev_info(g_sy->dev, "%s:usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", __func__, chg_vol, i, usb_icl[i], aicl_point_temp);
	if (g_oplus_chip && !g_oplus_chip->stop_chg) {
		sy6974b_set_input_current_limit(g_sy, SY697X_IINLIM_BASE);
		chg_info("aicl_end. stop_chg flag found 0");
	} else {
		oplus_sy6974b_check_ic_suspend();
	}
	return rc;
}

static int oplus_sy6974b_set_input_current_limit(int current_ma)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (g_sy == NULL || chip == NULL)
		return 0;

	if (current_ma == 0) {
		chg_debug("current_ma == 0 mmi[%d %d]\n", chip->mmi_chg, chip->mmi_fastchg);
		sy6974b_set_input_current_limit(g_sy, 0);
		return 0;
	}

	if (atomic_read(&g_sy->driver_suspended) == 1)
		return 0;

	if (atomic_read(&g_sy->charger_suspended) == 1) {
		chg_err("suspend,ignore set current=%dmA\n", current_ma);
		return 0;
	}

	dev_info(g_sy->dev, " is_force_aicl false current=%d\n", current_ma);
	g_sy->aicr = current_ma;
	oplus_sy6974b_set_aicr(g_sy->aicr);
	return 0;
}

static int oplus_sy6974b_set_cv(int cur)
{
	return sy6974b_set_chargevolt(g_sy, cur);
}

static int oplus_sy6974b_set_ieoc(int cur)
{
	return sy6974b_set_term_current(g_sy, cur);
}

static int oplus_sy6974b_charging_enable(void)
{
	return sy6974b_enable_charger(g_sy);
}

static int oplus_sy6974b_charging_disable(void)
{
	return sy6974b_disable_charger(g_sy);
}

static int oplus_sy6974b_hardware_init(void)
{
	int ret = 0;

	dev_info(g_sy->dev, "%s\n", __func__);
	chg_info("enter ");

	/* Enable charging */
	if (strcmp(g_sy->chg_dev_name, "primary_chg") == 0) {
		oplus_sy6974b_set_cv(DEFAULT_CV);
		oplus_sy6974b_set_ichg(SY697X_CHARGING_INIT_CURRENT);
		sy6974b_set_term_current(g_sy, g_sy->platform_data->iterm);
		sy6974b_set_input_volt_limit(g_sy, SY697X_HW_AICL_POINT);
		if (oplus_is_rf_ftm_mode()) {
			oplus_sy6974b_charger_suspend();
		} else {
			oplus_sy6974b_charger_unsuspend();
			ret = sy6974b_enable_charger(g_sy);
			if (ret < 0)
				dev_notice(g_sy->dev, "%s: en chg fail\n", __func__);
		}

		if (atomic_read(&g_sy->charger_suspended) == 1) {
			chg_err("suspend,ignore set current=500mA\n");
		} else {
			oplus_sy6974b_set_aicr(SY697X_INPUT_INIT_CURRENT);
		}
	}
	return ret;
}

static int oplus_sy6974b_is_charging_enabled(void)
{
	struct sy697x *sy = g_sy;
	int rc;
	int reg_val = 0;
	bool charging_enable = false;

	if (!sy)
		return 0;

	if (atomic_read(&sy->driver_suspended) == 1)
		return 0;

	rc = sy6974b_read_reg(sy, REG01_SY6974B_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG01_SY6974B_ADDRESS rc = %d\n", rc);
		return 0;
	}

	charging_enable = ((reg_val & REG01_SY6974B_CHARGING_MASK) == REG01_SY6974B_CHARGING_ENABLE) ? 1 : 0;

	return charging_enable;
}

static int oplus_sy6974b_is_charging_done(void)
{
	bool done = true;

	sy6974b_check_charge_done(g_sy, &done);

	return done;
}

static int oplus_sy6974b_enable_otg(void)
{
	int ret;

	ret = sy6974b_set_boost_current(g_sy, g_sy->platform_data->boosti);
	dev_notice(g_sy->dev, "%s set boost curr %d ret %d\n", __func__, g_sy->platform_data->boosti, ret);
	ret = sy6974b_enable_otg(g_sy);

	if (ret < 0) {
		dev_notice(g_sy->dev, "%s en otg fail(%d)\n", __func__, ret);
		return ret;
	}

	g_sy->otg_enable = true;
	return ret;
}

static bool oplus_get_otg_enable(void)
{
	int rc;
	int reg_val = 0;
	bool otg_enabled = false;

	if (!g_sy)
		return false;

	if (atomic_read(&g_sy->driver_suspended) == 1)
		return false;

	rc = sy6974b_read_reg(g_sy, REG01_SY6974B_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG01_SY6974B_ADDRESS rc = %d\n", rc);
		return false;
	}

	otg_enabled = ((reg_val & REG01_SY6974B_OTG_MASK) == REG01_SY6974B_OTG_ENABLE);
	dev_notice(g_sy->dev, "%s  otg enable = %d	!!\n", __func__, otg_enabled);

	return otg_enabled;
}

static int oplus_sy6974b_disable_otg(void)
{
	int ret;

	ret = sy6974b_disable_otg(g_sy);

	if (ret < 0) {
		dev_notice(g_sy->dev, "%s disable otg fail(%d)\n", __func__, ret);
		return ret;
	}

	g_sy->otg_enable = false;
	return ret;
}

static int oplus_sy6974b_disable_te(void)
{
	return  sy6974b_enable_term(g_sy, false);
}

static int oplus_sy6974b_get_chg_current_step(void)
{
	return SY697X_ICHG_LSB;
}

static void usb_enum_check(struct work_struct *work)
{
	int charger_type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (!g_sy || !g_oplus_chip || !g_oplus_chip->chg_ops || !g_oplus_chip->chg_ops->get_charger_type) {
		chg_err("is null pointer\n");
		return;
	}

	charger_type = g_oplus_chip->chg_ops->get_charger_type();
	switch (usb_enum_check_status) {
	case CHECK_CHARGER_EXIST:
		chg_info("charger_exist :%d\n", g_oplus_chip->charger_exist);
		if (g_oplus_chip->charger_exist) {
			usb_enum_check_status = CHECK_ENUM_STATUS;
			schedule_delayed_work(&g_sy->usb_enum_check_work, msecs_to_jiffies(80*1000));
		}
		break;
	case CHECK_ENUM_STATUS:
		chg_info("charger_type :%d\n", charger_type);
		if (charger_type == POWER_SUPPLY_TYPE_USB || charger_type == POWER_SUPPLY_TYPE_USB_CDP) {
			if (get_usb_enum_status() == 0) {
				oplus_chg_set_charger_type_unknown();
				force_dcp = 1;
				chg_err("force_dcp\n");
				oplus_chg_wake_update_work();
			}
		}
		break;
	default:
		break;
	}
}

static void start_usb_enum_check(void)
{
	if (!g_sy) {
		chg_err("is null pointer\n");
		return;
	}

	usb_enum_check_status = CHECK_CHARGER_EXIST;
	chg_info("start_usb_enum_check\n");
	schedule_delayed_work(&g_sy->usb_enum_check_work, msecs_to_jiffies(5*1000));
}

static void stop_usb_enum_check(void)
{
	if (!g_sy) {
		chg_err("is null pointer\n");
		return;
	}

	force_dcp = 0;
	chg_info("stop_usb_enum_check\n");
	cancel_delayed_work(&g_sy->usb_enum_check_work);
}

int oplus_sy6974b_get_charger_type(void)
{
	int charger_type = 0;

	if (!g_sy || !g_oplus_chip) {
		chg_debug("null pointer\n");
		return 0;
	}

	charger_type = g_sy->oplus_chg_type;
	chg_debug("force_dcp[%d] charger_type[%d]\n", force_dcp, charger_type);
	if (charger_type != POWER_SUPPLY_TYPE_USB_CDP && charger_type != POWER_SUPPLY_TYPE_USB && charger_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		stop_usb_enum_check();
	}

	if ((charger_type == POWER_SUPPLY_TYPE_USB_CDP || charger_type == POWER_SUPPLY_TYPE_USB)
			&& force_dcp) {
		charger_type = POWER_SUPPLY_TYPE_USB_DCP;
		g_sy->oplus_chg_type = charger_type;
	}

	if (charger_type != POWER_SUPPLY_TYPE_UNKNOWN &&
		g_oplus_chip->charger_type != charger_type &&
		g_oplus_chip->mmi_chg != 0) {
		power_supply_changed(g_oplus_chip->ac_psy);

		if (g_oplus_chip->charger_exist && !g_oplus_chip->batt_full && g_oplus_chip->batt_exist &&
			(CHARGING_STATUS_FAIL != g_oplus_chip->charging_state) &&
			!g_oplus_chip->stop_chg && g_oplus_chip->prop_status == 0) {
			g_oplus_chip->prop_status = POWER_SUPPLY_STATUS_CHARGING;
			power_supply_changed(g_oplus_chip->batt_psy);
		}

		chg_info("g_sy->oplus_chg_type = %d, mmi_chg = %d, chip->charger_type = %d"
			"chip->charger_exist = %d, chip->batt_full = %d, chip->batt_exist = %d"
			"chip->charging_state = %d, chip->stop_chg = %d, chip->prop_status = %d\n",
			g_sy->oplus_chg_type, g_oplus_chip->mmi_chg, g_oplus_chip->charger_type, g_oplus_chip->charger_exist,
			g_oplus_chip->batt_full, g_oplus_chip->batt_exist, g_oplus_chip->charging_state,
			g_oplus_chip->stop_chg, g_oplus_chip->prop_status);
	}

	return charger_type;
}

static bool oplus_sy6974b_check_suspend_charger(void)
{
	if (g_sy)
		return atomic_read(&g_sy->charger_suspended);

	return false;
}

static void oplus_sy6974b_check_ic_suspend(void)
{
	bool fastchg_start, fastchg_commu_ing;

	if (!g_oplus_chip || !g_sy) {
		chg_err("chip is null\n");
		return;
	}

	fastchg_start = oplus_vooc_get_fastchg_started();
	fastchg_commu_ing = oplus_voocphy_get_fastchg_commu_ing();
	chg_info("mmi_chg:%d, stop_chg:%d, fastchg_start:%d, commu_ing:%d\n",
		g_oplus_chip->mmi_chg, g_oplus_chip->stop_chg,
		fastchg_start, fastchg_commu_ing);

	if (g_oplus_chip->mmi_chg && g_oplus_chip->stop_chg &&
		fastchg_start == false && fastchg_commu_ing == false)
		return;

	if (oplus_sy6974b_check_suspend_charger()) {
		if (oplus_sy6974b_is_charging_enabled())
			oplus_sy6974b_charging_disable();
		if (sy6974b_get_input_current() != SY697X_IINLIM_BASE) {
			sy6974b_set_input_current_limit(g_sy, SY697X_IINLIM_BASE);
			g_sy->aicr = SY697X_IINLIM_BASE;
		}
	}
}


static int oplus_sy6974b_charger_suspend(void)
{
	if (g_sy)
		sy6974b_enter_hiz_mode(g_sy);
	if (g_oplus_chip && g_oplus_chip->is_double_charger_support) {
		g_oplus_chip->slave_charger_enable = false;
		g_oplus_chip->sub_chg_ops->charger_suspend();
	}
	chg_info("done \n");
	return 0;
}

static int oplus_sy6974b_charger_unsuspend(void)
{
	if (g_sy)
		sy6974b_exit_hiz_mode(g_sy);

	if (g_oplus_chip && g_oplus_chip->is_double_charger_support)
		g_oplus_chip->sub_chg_ops->charger_unsuspend();

	chg_info("done \n");
	return 0;
}

static int oplus_sy6974b_set_rechg_vol(int vol)
{
	return 0;
}

static int oplus_sy6974b_reset_charger(void)
{
	return 0;
}

static bool oplus_sy6974b_check_charger_resume(void)
{
	return true;
}

static int oplus_register_extcon(struct sy697x *chip)
{
	int rc = 0;

	chip->extcon = devm_extcon_dev_allocate(chip->dev, smblib_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		rc = PTR_ERR(chip->extcon);
		dev_err(chip->dev, "failed to allocate extcon device rc=%d\n",
				rc);
		goto cleanup;
	}

	rc = devm_extcon_dev_register(chip->dev, chip->extcon);
	if (rc < 0) {
		dev_err(chip->dev, "failed to register extcon device rc=%d\n",
				rc);
		goto cleanup;
	}

	/* Support reporting polarity and speed via properties */
	rc = extcon_set_property_capability(chip->extcon,
			EXTCON_USB, EXTCON_PROP_USB_TYPEC_POLARITY);
	rc |= extcon_set_property_capability(chip->extcon,
			EXTCON_USB, EXTCON_PROP_USB_SS);
	rc |= extcon_set_property_capability(chip->extcon,
			EXTCON_USB_HOST, EXTCON_PROP_USB_TYPEC_POLARITY);
	rc |= extcon_set_property_capability(chip->extcon,
			EXTCON_USB_HOST, EXTCON_PROP_USB_SS);

	dev_err(chip->dev, "oplus_register_extcon rc=%d\n",
			rc);
cleanup:
	return rc;
}

static int opluschg_parse_custom_dt(struct oplus_chg_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc = 0;

	if (!node) {
		chg_err("device tree node missing\n");
		return -EINVAL;
	}

	/* pinctrl */
	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	chg_info("opluschg_parse_custom_dt 1\n");

	/* usb switch 1 gpio */
	chip->normalchg_gpio.chargerid_switch_gpio = of_get_named_gpio(node, "qcom,chargerid_switch-gpio", 0);
	if (chip->normalchg_gpio.chargerid_switch_gpio > 0) {
		if (gpio_is_valid(chip->normalchg_gpio.chargerid_switch_gpio)) {
			rc = gpio_request(chip->normalchg_gpio.chargerid_switch_gpio,
				"chargerid-switch1-gpio");
			if (rc) {
				chg_err("unable to request gpio [%d]\n",
					chip->normalchg_gpio.chargerid_switch_gpio);
			}
		}

		chg_info("opluschg_parse_custom_dt 3\n");
		chip->normalchg_gpio.chargerid_switch_active =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_active");

		chg_info("opluschg_parse_custom_dt 4\n");
		chip->normalchg_gpio.chargerid_switch_sleep =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_sleep");

		chg_info("opluschg_parse_custom_dt 5\n");
		pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.chargerid_switch_sleep);
		chip->normalchg_gpio.chargerid_switch_default =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_default");

		chg_info("opluschg_parse_custom_dt 6\n");
		pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.chargerid_switch_default);
	}

	/* vbus short gpio */
	chip->normalchg_gpio.dischg_gpio = of_get_named_gpio(node, "qcom,dischg-gpio", 0);
	if (chip->normalchg_gpio.dischg_gpio > 0) {
		chip->normalchg_gpio.dischg_enable = pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "dischg_enable");
		chip->normalchg_gpio.dischg_disable = pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "dischg_disable");
		pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_disable);
	}

	rc = oplus_chg_ccdetect_parse_dt(chip);
	if (rc)
		chg_err("oplus_chg_ccdetect_parse_dt fail!\n");

	chg_info("done\n");
	return 0;
}

static int oplus_sy6974b_get_chargerid_switch(void)
{
	if (!g_oplus_chip) {
		chg_err("fail to init oplus_chip\n");
		return 0;
	}

	if (g_oplus_chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return 0;
	}

	chg_info("gpio[%d] done\n",
		g_oplus_chip->normalchg_gpio.chargerid_switch_gpio);
	return gpio_get_value(g_oplus_chip->normalchg_gpio.chargerid_switch_gpio);
}

static void oplus_sy6974b_set_chargerid_switch(int value)
{
	chg_info("val[%d]\n", value);

	if (!g_oplus_chip) {
		chg_err("fail to init oplus_chip\n");
		return;
	}

	if (g_oplus_chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(g_oplus_chip->normalchg_gpio.pinctrl) ||
		IS_ERR_OR_NULL(g_oplus_chip->normalchg_gpio.chargerid_switch_active) ||
		IS_ERR_OR_NULL(g_oplus_chip->normalchg_gpio.chargerid_switch_sleep) ||
		IS_ERR_OR_NULL(g_oplus_chip->normalchg_gpio.chargerid_switch_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if ((oplus_vooc_get_adapter_update_real_status() == ADAPTER_FW_NEED_UPDATE) ||
			(oplus_vooc_get_btb_temp_over() == true)) {
		chg_err("adapter update or btb_temp_over, return\n");
		return;
	}

	if (g_oplus_chip->pmic_spmi.smb5_chip)
		mutex_lock(&g_oplus_chip->pmic_spmi.smb5_chip->chg.pinctrl_mutex);

	if (value) {
		pinctrl_select_state(g_oplus_chip->normalchg_gpio.pinctrl,
			g_oplus_chip->normalchg_gpio.chargerid_switch_active);
		gpio_direction_output(g_oplus_chip->normalchg_gpio.chargerid_switch_gpio, 1);
		chg_info("switch rxtx\n");
	} else {
		pinctrl_select_state(g_oplus_chip->normalchg_gpio.pinctrl,
			g_oplus_chip->normalchg_gpio.chargerid_switch_sleep);
		gpio_direction_output(g_oplus_chip->normalchg_gpio.chargerid_switch_gpio, 0);
		chg_info("switch dpdm\n");
	}

	if (g_oplus_chip->pmic_spmi.smb5_chip)
		mutex_unlock(&g_oplus_chip->pmic_spmi.smb5_chip->chg.pinctrl_mutex);

	chg_debug("set usb_switch_1 = %d, result = %d\n",
		value, oplus_sy6974b_get_chargerid_switch());
}

static int opluschg_get_chargerid(void)
{
	return 0;
}

static int oplus_sy6974b_get_charger_subtype(void)
{
	if (g_sy->hvdcp_can_enabled) {
#ifdef ENABLE_HVDCP
		return CHARGER_SUBTYPE_QC;
#else
		return CHARGER_SUBTYPE_DEFAULT;
#endif
	} else {
		return CHARGER_SUBTYPE_DEFAULT;
	}
}

static bool oplus_sy6974b_need_to_check_ibatt(void)
{
	return false;
}

static int oplus_sy6974b_get_dyna_aicl_result(void)
{
	int ma = 0;

	sy6974b_read_idpm_limit(g_sy, &ma);
	return ma;
}

static void vol_convert_work(struct work_struct *work)
{
	int retry = 20;

	if (!g_sy->pdqc_setup_5v) {
		chg_info("set_to_9v\n");
		if (oplus_sy6974b_get_vbus() < CHARGER_VOLTAGE_5800MV) {
			sy6974b_enable_hvdcp(g_sy);
			sy6974b_switch_to_hvdcp(g_sy, HVDCP_9V);
			oplus_for_cdp();
			g_sy->is_force_aicl = true;
			g_sy->is_retry_bc12 = true;
			sy6974b_force_dpdm(g_sy, true);
			msleep(500);
			while(retry--) {
				if (oplus_sy6974b_get_vbus() > CHARGER_VOLTAGE_7600MV) {
					chg_info("set_to_9v success\n");
					break;
				}
				msleep(500);
			}
		}
	} else {
		chg_info("set_to_5v\n");
		if (oplus_sy6974b_get_vbus() > CHARGER_VOLTAGE_7600MV) {
			sy6974b_disable_hvdcp(g_sy);
			oplus_for_cdp();
			g_sy->is_force_aicl = true;
			g_sy->is_retry_bc12 = true;
			sy6974b_force_dpdm(g_sy, true);
			msleep(500);
			while(retry--) {
				if (oplus_sy6974b_get_vbus() < CHARGER_VOLTAGE_5800MV) {
					chg_info("set_to_5v success\n");
					break;
				}
				msleep(500);
			}
		}
	}
	g_sy->is_bc12_end = true;
}

#define SY6974B_DISABLE_HIGH_VBUS	1
static int oplus_sy6974b_set_qc_config(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	static int qc_to_9v_count = 0; /* for other quick charger */

	if (!chip) {
		dev_info(g_sy->dev, "%s: error\n", __func__);
		return false;
	}

	if (disable_qc) {
		dev_info(g_sy->dev, "%s:disable_qc\n", __func__);
		return false;
	}

	if (g_sy->disable_hight_vbus == SY6974B_DISABLE_HIGH_VBUS) {
		dev_info(g_sy->dev, "%s:disable_hight_vbus\n", __func__);
		return false;
	}

	if (chip->calling_on || chip->camera_on || chip->ui_soc >= 90 || chip->cool_down_force_5v == true ||
		chip->limits.tbatt_pdqc_to_5v_thr == true) {
		dev_info(g_sy->dev, "++%s: set_qc_to 5V =%d,bc12=%d",
			__func__, g_sy->pdqc_setup_5v, g_sy->is_bc12_end);
		chg_info("%d,%d,%d,%d,%d,%d\n",
			chip->calling_on, chip->camera_on, chip->ui_soc,
			chip->cool_down_force_5v, chip->limits.tbatt_pdqc_to_5v_thr, chip->charger_volt);
		g_sy->pdqc_setup_5v = true;
		if (g_sy->is_bc12_end) {
			g_sy->is_bc12_end = false;
			schedule_delayed_work(&g_sy->sy697x_vol_convert_work, 0);
		}
		dev_info(g_sy->dev, "%s: set_qc_to 5V =%d,bc12=%d",
			__func__, g_sy->pdqc_setup_5v, g_sy->is_bc12_end);
	} else { /* 9v */
		g_sy->pdqc_setup_5v = false;
		if (g_sy->is_bc12_end) {
			g_sy->is_bc12_end = false;
			schedule_delayed_work(&g_sy->sy697x_vol_convert_work, 0);
		}
		/* add for other quick charge */
		if (oplus_sy6974b_get_vbus() < CHARGER_VOLTAGE_5800MV) {
			if (qc_to_9v_count >= 5) {
				g_sy->hvdcp_can_enabled = false;
				qc_to_9v_count = 0;
			}
			if (g_sy->hvdcp_can_enabled) {
				qc_to_9v_count++;
			}
		}

		chg_info("%d,%d,%d,%d,%d,%d\n",
			chip->calling_on, chip->camera_on, chip->ui_soc,
			chip->cool_down_force_5v, chip->limits.tbatt_pdqc_to_5v_thr, chip->charger_volt);
		dev_info(g_sy->dev, "%s: set_qc_to 9V =%d,bc12=%d",
			__func__, g_sy->pdqc_setup_5v, g_sy->is_bc12_end);
	}

	return true;
}

static int oplus_sy6974b_enable_qc_detect(void)
{
	return 0;
}

bool oplus_sy6974b_need_retry_aicl(void)
{
	static bool connect = false;

	if (!g_sy)
		return false;

	if ((g_sy->boot_mode == MSM_BOOT_MODE__RF || g_sy->boot_mode == MSM_BOOT_MODE__WLAN) && !connect) {
		if (g_oplus_chip->chg_ops->get_charger_volt() > SY6974B_INPUT_VOLT_LIMIT_DEFAULT_VALUE) {
			g_sy->chg_type = STANDARD_HOST;
			g_sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB;
			g_oplus_chip->charger_type = POWER_SUPPLY_TYPE_USB;
			g_oplus_chip->chg_ops->usb_connect();
			connect = true;
		}
	}
	if (g_sy->cdp_retry_aicl) {
		g_sy->cdp_retry_aicl = false;
		chg_info("retry aicl\n");
		return true;
	}
	return g_sy->cdp_retry_aicl;
}

static bool oplus_check_chrdet_status(void)
{
	int reg_val = 0;
	int ret = 0;

	if (!g_sy)
		return false;

	if (g_sy->is_retry_bc12 == true && g_sy->is_force_aicl == false) {
		ret = sy6974b_read_reg(g_sy, REG08_SY6974B_ADDRESS, &reg_val);
		if (!ret) {
			if ((REG08_SY6974B_VBUS_STAT_MASK & reg_val) >> REG08_SY6974B_VBUS_STAT_SHIFT) {
				chg_info("bc12 succses\n");
				g_sy->is_retry_bc12 = false;
				chg_info("[1]:chg_type = %d, %d, %d\n",
					g_sy->chg_type, g_sy->oplus_chg_type, g_oplus_chip->charger_type);
				sy_charger_type_recheck(g_sy);
				chg_info("[2]:chg_type = %d, %d, %d\n",
					g_sy->chg_type, g_sy->oplus_chg_type, g_oplus_chip->charger_type);
			} else if (!g_sy->power_good) {
				g_sy->is_retry_bc12 = false;
			}
		}
	}
	return ret;
}

static int oplus_sy6974b_chg_set_high_vbus(bool en)
{
	int subtype;

	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		dev_info(g_sy->dev, "%s: error\n", __func__);
		return false;
	}

	if (en) {
		g_sy->disable_hight_vbus = 0;
		if (chip->charger_volt > CHARGER_VOLTAGE_7600MV) {
			dev_info(g_sy->dev, "%s:charger_volt already 9v\n", __func__);
			return false;
		}

		if (g_sy->pdqc_setup_5v) {
			dev_info(g_sy->dev, "%s:pdqc already setup5v no need 9v\n", __func__);
			return false;
		}
	} else {
		g_sy->disable_hight_vbus = 1;
		if (chip->charger_volt < CHARGER_VOLTAGE_5800MV) {
			dev_info(g_sy->dev, "%s:charger_volt already 5v\n", __func__);
			return false;
		}
	}

	subtype = oplus_sy6974b_get_charger_subtype();
	if (subtype == CHARGER_SUBTYPE_QC) {
		if (en) {
			dev_info(g_sy->dev, "%s:QC Force output 9V\n", __func__);
			sy6974b_switch_to_hvdcp(g_sy, HVDCP_9V);
		} else {
			sy6974b_switch_to_hvdcp(g_sy, HVDCP_5V);
			dev_info(g_sy->dev, "%s: set qc to 5V", __func__);
		}
	} else {
		dev_info(g_sy->dev, "%s:do nothing\n", __func__);
	}

	return false;
}

extern int get_boot_mode(void);
static int opluschg_get_boot_reason(void)
{
	return OPLUS_CHG_BOOT_REASON_DEFAULT;
}

static int opluschg_get_battery_voltage(void)
{
	return SY6974B_READ_VBAT_DEFAULT;/* Not use anymore */
}

static int oplus_get_rtc_ui_soc(void)
{
	if (!g_oplus_chip) {
		chg_err("chip not ready\n");
		return 0;
	}

	if (!g_oplus_chip->external_gauge) {
		return OPLUS_CHG_RTC_UI_SOC_50_PERCENT;
	} else {
		return 0;
	}
}

static int oplus_set_rtc_ui_soc(int backup_soc)
{
	return 0;
}

static bool oplus_sy6974b_is_usb_present(void)
{
	int rc;
	int reg_val = 0;
	bool bus_gd = false;

	if (!g_sy)
		return false;

	if (atomic_read(&g_sy->driver_suspended) == 1)
		return false;

	rc = sy6974b_read_reg(g_sy, REG0A_SY6974B_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't oplus_chg_is_usb_present rc = %d\n", rc);
		return false;
	}

	bus_gd = ((reg_val & REG0A_SY6974B_BUS_GD_MASK) == REG0A_SY6974B_BUS_GD_YES) ? 1 : 0;
	dev_notice(g_sy->dev, "%s  bus_gd = %d	!!\n", __func__, bus_gd);
	return bus_gd;
}

static bool oplus_sy6974b_charger_detect(void)
{
	int val = 0;

	if (g_sy && g_oplus_chip) {
		if (sy6974b_read_reg(g_sy, REG0A_SY6974B_ADDRESS, &val) ||
			g_oplus_chip->otg_online) {
			chg_err("failed.otg[%d]\n", g_oplus_chip->otg_online);
			return false;
		}
	} else {
		chg_err("g_sy/g_oplus_chip is null return false\n");
		return false;
	}

	val = ((val & REG0A_SY6974B_BUS_GD_MASK) == REG0A_SY6974B_BUS_GD_YES) ? 1 : 0;
	dev_notice(g_sy->dev, "%s  bus_gd val = %d	!!\n", __func__, val);
	return val;
}

static int qpnp_get_prop_charger_voltage_now(void)
{
	int chg_vol = 0;
	u8 cp_adc_reg = 0;
	struct oplus_chg_chip *chip = oplus_chg_get_chg_struct();
	struct smb_charger *chg = NULL;

	if (!chip)
		return 0;

	chg = &chip->pmic_spmi.smb5_chip->chg;

	if (qpnp_is_power_off_charging()) {
		if (chg->pd_hard_reset || chg->keep_vbus_5v) {
			chg_err("pd hardreset,return 5000\n");
			return CHARGER_VOLTAGE_DEFAULT;
		}
	}

	if (!chip->charger_exist && !chip->ac_online)
		return 0;

	if (oplus_chg_get_voocphy_support() == AP_SINGLE_CP_VOOCPHY
			|| oplus_chg_get_voocphy_support() == AP_DUAL_CP_VOOCPHY) {
		if (oplus_voocphy_get_fastchg_commu_ing())
			return oplus_voocphy_get_var_vbus();

		oplus_voocphy_get_adc_enable(&cp_adc_reg);
		if (cp_adc_reg == 0) {
			oplus_voocphy_set_adc_enable(true);
			usleep_range(1000, 1500);
		}
		chg_vol = oplus_voocphy_get_cp_vbus();
	} else {
		chg_err("get chg_vol interface null\n");
	}

	return chg_vol;
}

static int oplus_sy6974b_get_vbus(void)
{
	int chg_vol = CHARGER_VOLTAGE_DEFAULT;
	chg_vol = qpnp_get_prop_charger_voltage_now();
	chg_info("get vbus voltage form cp! chg_vol: %d\n", chg_vol);
	return chg_vol;
}

static bool oplus_chg_is_usb_present(void)
{
	static bool pre_vbus_status = false;
	struct sy697x *sy = g_sy;
	struct oplus_chg_chip *g_oplus_chip = oplus_chg_get_chg_struct();

	if (oplus_get_otg_online_status_default()) {
		chg_err("otg,return false");
		pre_vbus_status = false;
		return pre_vbus_status;
	}

	if (oplus_voocphy_get_fastchg_commu_ing()) {
		chg_err("fastchg_commu_ing,return true");
		pre_vbus_status = true;
		return pre_vbus_status;
	}

	if (sy && atomic_read(&sy->driver_suspended) == 1 &&
		g_oplus_chip && g_oplus_chip->unwakelock_chg == 1 &&
		g_oplus_chip->charger_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		chg_err("unwakelock_chg=1, use pre status=%d\n", pre_vbus_status);
		return pre_vbus_status;
	}

	pre_vbus_status = oplus_sy6974b_charger_detect();
	return pre_vbus_status;
}

static void sy6974b_vooc_timeout_callback_func(bool vbus_rising)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	u8 hz_mode = 0;
	int ret = 0;
	bool bus_good = false;

	if (!g_sy)
		return;

	if (chip == NULL)
		return;

	bus_good = oplus_sy6974b_charger_detect();
	pr_notice("[%s] bus_good [%s] vbus_rising [%s]\n", __func__,
		bus_good == true ?"true":"false",
		vbus_rising == true ?"true":"false");

	if (!bus_good) {
		g_sy->power_good = bus_good;
		ret = sy6974b_get_hiz_mode(g_sy, &hz_mode);
		if (!ret && hz_mode) {
			pr_notice("hiz mode ignore\n");
			goto power_change;
		}
		g_sy->is_force_aicl = false;
		g_sy->pre_current_ma = -1;
		g_sy->usb_connect_start = false;
		g_sy->hvdcp_can_enabled = false;
		g_sy->hvdcp_checked = false;
		g_sy->sdp_retry = false;
		g_sy->cdp_retry = false;
		g_sy->chg_type = CHARGER_UNKNOWN;
		g_sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		memset(&g_sy->st_ptime[0], 0, sizeof(struct timespec));
		memset(&g_sy->st_ptime[1], 0, sizeof(struct timespec));
		if (chip) {
		chip->pd_chging = false;
		}
		if (chip->is_double_charger_support) {
			chip->sub_chg_ops->charging_disable();
		}
		sy6974b_inform_charger_type(g_sy);
		opluschg_updata_usb_type(g_sy);

		sy6974b_adc_start(g_sy, false);
		pr_notice("[%s] adapter/usb pg_good removed\n", __func__);
		chip->usbtemp_check = false;
		oplus_splitchg_request_dpdm(g_sy, false);
		oplus_notify_device_mode(false);
		oplus_chg_wakelock(g_sy, false);
		stop_usb_enum_check();
	}

power_change:
	if (dumpreg_by_irq)
		sy6974b_dump_regs(g_sy);
}

struct oplus_chgic_operations oplus_chgic_sy6974b_ops = {
	.typec_sink_removal = oplus_sy6974b_typec_sink_removal,
	.typec_src_removal = oplus_sy6974b_typec_src_removal,
	.typec_sink_insertion = oplus_sy6974b_typec_sink_insertion,
	.get_otg_switch_status = oplus_sy6974b_get_otg_switch_status,
	.set_otg_switch_status = oplus_sy6974b_set_otg_switch_status,
	.get_otg_online_status = oplus_sy6974b_get_otg_online_status,
	.thermal_tmp_get_chg = oplus_sy6974b_thermal_tmp_get_chg,
	.thermal_tmp_get_bb = oplus_sy6974b_thermal_tmp_get_bb,
	.thermal_tmp_get_flash = oplus_sy6974b_thermal_tmp_get_flash,
	.thermal_tmp_get_board = oplus_sy6974b_thermal_tmp_get_board,
	.get_usb_status = oplus_sy6974b_get_usb_status,
};

struct oplus_chg_operations  oplus_chg_sy6974b_ops = {
	.dump_registers = oplus_sy6974b_dump_registers,
	.kick_wdt = oplus_sy6974b_kick_wdt,
	.hardware_init = oplus_sy6974b_hardware_init,
	.charging_current_write_fast = oplus_sy6974b_set_ichg,
	.set_aicl_point = oplus_sy6974b_set_mivr,
	.input_current_write = oplus_sy6974b_set_input_current_limit,
	.float_voltage_write = oplus_sy6974b_set_cv,
	.term_current_set = oplus_sy6974b_set_ieoc,
	.charging_enable = oplus_sy6974b_charging_enable,
	.charging_disable = oplus_sy6974b_charging_disable,
	.get_charging_enable = oplus_sy6974b_is_charging_enabled,
	.charger_suspend = oplus_sy6974b_charger_suspend,
	.charger_unsuspend = oplus_sy6974b_charger_unsuspend,
	.set_rechg_vol = oplus_sy6974b_set_rechg_vol,
	.reset_charger = oplus_sy6974b_reset_charger,
	.read_full = oplus_sy6974b_is_charging_done,
	.otg_enable = oplus_sy6974b_enable_otg,
	.otg_disable = oplus_sy6974b_disable_otg,
	.set_charging_term_disable = oplus_sy6974b_disable_te,
	.check_charger_resume = oplus_sy6974b_check_charger_resume,
	.get_charger_type = oplus_sy6974b_get_charger_type,

	.get_charger_volt = oplus_sy6974b_get_vbus,
	.get_chargerid_volt		= opluschg_get_chargerid,
	.set_chargerid_switch_val = oplus_sy6974b_set_chargerid_switch,
	.get_chargerid_switch_val = oplus_sy6974b_get_chargerid_switch,
	.check_chrdet_status = oplus_chg_is_usb_present,

	.get_boot_mode = get_boot_mode,
	.get_boot_reason = (int (*)(void))opluschg_get_boot_reason,
	.get_instant_vbatt = opluschg_get_battery_voltage,
	.get_rtc_soc = oplus_get_rtc_ui_soc,
	.set_rtc_soc = oplus_set_rtc_ui_soc,

	.get_chg_current_step = oplus_sy6974b_get_chg_current_step,
	.need_to_check_ibatt = oplus_sy6974b_need_to_check_ibatt,
	.get_dyna_aicl_result = oplus_sy6974b_get_dyna_aicl_result,
	.get_shortc_hw_gpio_status = NULL,
	.oplus_chg_get_pd_type = NULL,
	.oplus_chg_pd_setup = NULL,
	.get_charger_subtype = oplus_sy6974b_get_charger_subtype,
	.set_qc_config = oplus_sy6974b_set_qc_config,
	.enable_qc_detect = oplus_sy6974b_enable_qc_detect,

	.oplus_chg_set_high_vbus = oplus_sy6974b_chg_set_high_vbus,
	.enable_shipmode = sy6974b_enable_shipmode,
	.get_usbtemp_volt = oplus_sy6974b_get_usbtemp_volt,
	.oplus_usbtemp_monitor_condition = oplus_usbtemp_condition,
	.vooc_timeout_callback = sy6974b_vooc_timeout_callback_func,
	.set_typec_cc_open = sgm7220_set_typec_cc_open,
	.set_typec_sinkonly = sgm7220_set_typec_sinkonly,
	.get_charger_current = sy6974b_get_input_current,
	.suspend_for_usbtemp = sy6974b_suspend_by_hz_mode,
};

static void aicl_work_callback(struct work_struct *work)
{
	int re_check_count = 0;

	if (!g_sy)
		return;

	if (g_sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB_DCP && g_sy->is_force_aicl) {
		while (g_sy->is_force_aicl) {
			if (re_check_count++ < 200) {
				msleep(20);
			} else {
				break;
			}
		}
	}
	oplus_sy6974b_set_aicr(g_sy->aicr);
}

static struct of_device_id sy6974b_charger_match_table[] = {
	{ .compatible = "ti,sy6974b", },
	{},
};

MODULE_DEVICE_TABLE(of, sy6974b_charger_match_table);

static const struct i2c_device_id sy6974b_i2c_device_id[] = {
	{ "sy6974b", 0x06 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sy6974b_i2c_device_id);

/* add by for power_supply */
static enum power_supply_property oplus_sy6974b_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_REAL_TYPE,
};

void oplus_sy6974b_set_otg_switch_status(bool value)
{
	int rc = 0;
	struct smb_charger *chg = NULL;

	if (!g_oplus_chip || !g_sy) {
		chg_err("oplus_chip is null\n");
		return;
	}
	chg = &g_oplus_chip->pmic_spmi.smb5_chip->chg;

	if (oplus_ccdetect_check_is_gpio(g_oplus_chip) == true) {
		if (gpio_get_value(chg->ccdetect_gpio) == 0) {
			chg_err("gpio[L], should set, return\n");
			return;
		}
	}

	g_oplus_chip->otg_switch = value;
	if (g_oplus_chip->otg_switch == true) {
		rc = oplus_sgm7220_set_mode(SET_MODE_SELECT_DRP);
	} else {
		rc = oplus_sgm7220_set_mode(SET_MODE_SELECT_SNK);
	}

	if (rc < 0)
		chg_err("fail to write register\n");

	chg_debug("otg_switch = %d, otg_online = %d, \n",
		g_oplus_chip->otg_switch, g_oplus_chip->otg_online);
}
EXPORT_SYMBOL(oplus_sy6974b_set_otg_switch_status);

bool oplus_sy6974b_get_otg_switch_status(void)
{
	if (!g_oplus_chip) {
		chg_err("fail to init oplus_chip\n");
		return false;
	}
	chg_info("otg_switch[%d]\n", g_oplus_chip->otg_switch);
	return g_oplus_chip->otg_switch;
}
EXPORT_SYMBOL(oplus_sy6974b_get_otg_switch_status);

static int oplus_sy6974b_usb_set_prop(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int oplus_sy6974b_get_usb_online(struct sy697x *chg,
			union power_supply_propval *val)
{
	if (!chg) {
		val->intval = 0;
		return 0;
	}

	if (g_oplus_chip) {
		if (chg->power_good && (oplus_sy6974b_get_usb_status() != USB_TEMP_HIGH)
			&& ((chg->oplus_chg_type == POWER_SUPPLY_TYPE_USB) || (chg->oplus_chg_type == POWER_SUPPLY_TYPE_USB_CDP)))
			val->intval = 1;
		else
			val->intval = 0;
	}

	return 0;
}

static int oplus_sy6974b_usb_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int rc = 0;
	struct sy697x *chg = g_sy;

	val->intval = 0;

	if (!chg) {
		chg_err("chg is null \n");
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = chg->oplus_chg_type;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = chg->oplus_chg_type;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = oplus_sy6974b_is_usb_present();
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		oplus_sy6974b_get_usb_online(chg, val);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		return -ENODATA;
	}

	return 0;
}

static int oplus_sy6974b_usb_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
	default:
		break;
	}

	return 0;
}

static struct power_supply_desc usb_psy_desc = {
	.name = "usb",
#ifndef OPLUS_FEATURE_CHG_BASIC
/* Yichun.Chen  PSW.BSP.CHG  2019-04-08  for charge */
	.type = POWER_SUPPLY_TYPE_USB_PD,
#else
	.type = POWER_SUPPLY_TYPE_USB,
#endif
	.properties = oplus_sy6974b_usb_props,
	.num_properties = ARRAY_SIZE(oplus_sy6974b_usb_props),
	.get_property = oplus_sy6974b_usb_get_prop,
	.set_property = oplus_sy6974b_usb_set_prop,
	.property_is_writeable = oplus_sy6974b_usb_prop_is_writeable,
};

static enum power_supply_property oplus_sy6974b_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static int oplus_sy6974b_batt_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
#ifdef OPLUS_FEATURE_CHG_BASIC
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		if (g_oplus_chip && (g_oplus_chip->ui_soc == 0)) {
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
			chg_err("POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL, should shutdown!\n");
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = g_oplus_chip->batt_fcc * 1000;
		break;

	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = g_oplus_chip->ui_soc * g_oplus_chip->batt_capacity_mah * 1000 / 100;
		break;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		val->intval = 0;
		break;

	default:
		rc = oplus_battery_get_property(psy, psp, val);
#endif
	}
	if (rc < 0) {
		chg_info("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return rc;
}

static int oplus_sy6974b_batt_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	return oplus_battery_set_property(psy, prop, val);
}

static int oplus_sy6974b_batt_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	return oplus_battery_property_is_writeable(psy, psp);
}

static struct power_supply_desc batt_psy_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = oplus_sy6974b_batt_props,
	.num_properties = ARRAY_SIZE(oplus_sy6974b_batt_props),
	.get_property = oplus_sy6974b_batt_get_prop,
	.set_property = oplus_sy6974b_batt_set_prop,
	.property_is_writeable = oplus_sy6974b_batt_prop_is_writeable,
};

static enum power_supply_property ac_props[] = {
/* OPLUS own ac props */
        POWER_SUPPLY_PROP_ONLINE,
};

static int oplus_sy6974b_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int rc;

	rc = oplus_ac_get_property(psy, psp, val);

	return rc;
}

static struct power_supply_desc ac_psy_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = ac_props,
	.num_properties = ARRAY_SIZE(ac_props),
	.get_property = oplus_sy6974b_ac_get_property,
};

static int oplus_power_supply_init(struct oplus_chg_chip *chip)
{
	int ret = 0;
	struct oplus_chg_chip *chgchip = NULL;

	if (chip == NULL) {
		chg_err("oplus_chip not ready!\n");
		return -EINVAL;
	}
	chgchip = chip;

	chgchip->ac_psd = ac_psy_desc;
	chgchip->ac_cfg.drv_data = chgchip;
	chgchip->usb_psd = usb_psy_desc;
	chgchip->usb_cfg.drv_data = chgchip;
	chgchip->battery_psd = batt_psy_desc;

	chgchip->ac_psy = power_supply_register(chgchip->dev, &chgchip->ac_psd,
			&chgchip->ac_cfg);
	if (IS_ERR(chgchip->ac_psy)) {
		dev_err(chgchip->dev, "Failed to register power supply ac: %ld\n",
			PTR_ERR(chgchip->ac_psy));
		ret = PTR_ERR(chgchip->ac_psy);
		goto err_ac_psy;
	}

	chgchip->usb_psy = power_supply_register(chgchip->dev, &chgchip->usb_psd,
			&chgchip->usb_cfg);
	if (IS_ERR(chgchip->usb_psy)) {
		dev_err(chgchip->dev, "Failed to register power supply usb: %ld\n",
			PTR_ERR(chgchip->usb_psy));
		ret = PTR_ERR(chgchip->usb_psy);
		goto err_usb_psy;
	}

	chgchip->batt_psy = power_supply_register(chgchip->dev, &chgchip->battery_psd,
			NULL);
	if (IS_ERR(chgchip->batt_psy)) {
		dev_err(chgchip->dev, "Failed to register power supply battery: %ld\n",
			PTR_ERR(chgchip->batt_psy));
		ret = PTR_ERR(chgchip->batt_psy);
		goto err_battery_psy;
	}

	chg_info("OK\n");
	return 0;

err_battery_psy:
	power_supply_unregister(chgchip->usb_psy);
err_usb_psy:
	power_supply_unregister(chgchip->ac_psy);
err_ac_psy:

	return ret;
}

struct oplus_chg_chip* oplus_sy6974b_get_oplus_chip(void)
{
	return g_oplus_chip;
}

struct sy697x* oplus_sy6974b_get_oplus_chg(void)
{
	return g_sy;
}

static int oplus_parse_dt_adc_channels(struct sy697x *chg)
{
	int ret;

	ret = oplus_get_iio_channel(chg, "usb_temp1", &chg->iio.usb_temp_chan1);
	if (ret < 0 && !chg->iio.usb_temp_chan1) {
		chg_err("usb_temp_chan1 get failed\n");
		return ret;
	}

	ret = oplus_get_iio_channel(chg, "usb_temp2", &chg->iio.usb_temp_chan2);
	if (ret < 0 && !chg->iio.usb_temp_chan2) {
		chg_err("usb_temp_chan2 get failed\n");
		return ret;
	}

	ret = oplus_get_iio_channel(chg, "quiet_therm", &chg->iio.batt_btb_temp_chan);
	if (ret < 0 && !chg->iio.batt_btb_temp_chan) {
		chg_err("batt_btb_temp_chan get failed\n");
		return ret;
	}

	ret = oplus_get_iio_channel(chg, "ntc_switch1_chan", &chg->iio.ntc_switch1_chan);
	if (ret < 0 && !chg->iio.ntc_switch1_chan) {
		chg_err("ntc_switch1_chan get failed\n");
		return ret;
	}

	ret = oplus_get_iio_channel(chg, "ntc_switch2_chan", &chg->iio.ntc_switch2_chan);
	if (ret < 0 && !chg->iio.ntc_switch2_chan) {
		chg_err("ntc_switch2_chan get failed\n");
		return ret;
	}

	return 0;
}

static int sy6974b_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct sy697x *sy;
	struct device_node *node = client->dev.of_node;
	int ret = 0;
	struct oplus_chg_chip *oplus_chip = NULL;
	int level = 0;
	struct smb5 *chip;
	struct smb_charger *chg;

	chg_info("enter\n");

	/* oplus_chip register */
	if (oplus_gauge_check_chip_is_null()) {
		chg_err("gauge chip null, will do after bettery init.\n");
		return -EPROBE_DEFER;
	}

	oplus_chip = devm_kzalloc(&client->dev, sizeof(*oplus_chip), GFP_KERNEL);
	if (!oplus_chip) {
		chg_err("oplus_chip null, will do after bettery init.\n");
		return -ENOMEM;
	}

	oplus_chip->dev = &client->dev;
	oplus_chg_parse_svooc_dt(oplus_chip);

	sy = devm_kzalloc(&client->dev, sizeof(struct sy697x), GFP_KERNEL);
	if (!sy)
		return -ENOMEM;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	oplus_chip->pmic_spmi.smb5_chip = chip;
	chg = &chip->chg;
	chg->dev = &client->dev;
	mutex_init(&chg->pinctrl_mutex);
	sy->dev = &client->dev;
	sy->client = client;
	g_sy = sy;

	i2c_set_clientdata(client, sy);
	mutex_init(&sy->i2c_rw_lock);
	mutex_init(&sy->ntc_lock);
	mutex_init(&sy->dpdm_lock);

	oplus_chg_awake_init(sy);
	init_waitqueue_head(&sy->wait);
	oplus_keep_resume_awake_init(sy);

	atomic_set(&sy->driver_suspended, 0);
	atomic_set(&sy->charger_suspended, 0);
	sy->before_suspend_icl = 0;
	sy->before_unsuspend_icl = 0;
	sy->chgic_ops = &oplus_chgic_sy6974b_ops;

	ret = sy6974b_detect_device(sy);
	if (ret) {
		chg_err("No sy6974b device found!\n");
		ret = -ENODEV;
		goto err_nodev;
	}

	sy->platform_data = sy6974b_parse_dt(node, sy);
	if (!sy->platform_data) {
		chg_err("No platform data provided.\n");
		ret = -EINVAL;
		goto err_parse_dt;
	}

	sy6974b_reset_chip(sy);
	if (is_bq25890h(sy) == false) {
		ret = sy6974b_init_device(sy);
	} else {
		ret = bq2589x_init_device(sy);
	}
	if (ret) {
		chg_err("Failed to init device\n");
		goto err_init;
	}
	sy->is_bc12_end = true;

	sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	sy->pre_current_ma = -1;
	sy->chg_start_check = true;
	sy->usb_connect_start = false;
	if (is_bq25890h(sy) == false) {
		sy6974b_switch_to_hvdcp(sy, HVDCP_9V);
		sy6974b_enable_hvdcp(sy);
	} else {
		sy6974b_disable_hvdcp(sy);
		bq2589x_disable_maxc(sy);
	}
	sy6974b_disable_batfet_rst(sy);
	sy6974b_disable_ico(sy);

	INIT_DELAYED_WORK(&sy->sy697x_aicl_work, aicl_work_callback);
	INIT_DELAYED_WORK(&sy->sy697x_bc12_retry_work, sy6974b_bc12_retry_work);
	INIT_DELAYED_WORK(&sy->sy697x_vol_convert_work, vol_convert_work);
	INIT_DELAYED_WORK(&sy->usb_enum_check_work, usb_enum_check);

	sy6974b_register_interrupt(node, sy);

	if (is_bq25890h(sy) == false) {
		chg_info("sy6974b\n");
	} else {
		chg_info("bq25890h\n");
	}

	g_oplus_chip = oplus_chip;
	oplus_chip->chg_ops = &oplus_chg_sy6974b_ops;
	oplus_power_supply_init(oplus_chip);
	opluschg_parse_custom_dt(oplus_chip);
	oplus_chg_parse_charger_dt(oplus_chip);

	oplus_chip->con_volt = con_volt_855;
	oplus_chip->con_temp = con_temp_855;
	oplus_chip->len_array = ARRAY_SIZE(con_temp_855);
	oplus_parse_dt_adc_channels(sy);

	/* platform init */
	oplus_chg_plt_init_for_qcom(sy);

	/* add extcon register for usb emulation */
	oplus_register_extcon(sy);

	oplus_chg_init(oplus_chip);

	if (oplus_ccdetect_support_check() == true) {
		INIT_DELAYED_WORK(&chg->ccdetect_work, oplus_ccdetect_work);
		INIT_DELAYED_WORK(&usbtemp_recover_work, oplus_usbtemp_recover_work);
		oplus_ccdetect_irq_register(oplus_chip);
		level = gpio_get_value(chg->ccdetect_gpio);
		usleep_range(2000, 2100);
		if (level != gpio_get_value(chg->ccdetect_gpio)) {
			chg_err("ccdetect_gpio is unstable, try again...\n");
			usleep_range(10000, 11000);
			level = gpio_get_value(chg->ccdetect_gpio);
		}
		if (level <= 0) {
			schedule_delayed_work(&chg->ccdetect_work, msecs_to_jiffies(6000));
		}
		chg_info("ccdetect_gpio ..level[%d]  \n", level);
	}

	oplus_chg_configfs_init(oplus_chip);
	opluschg_usbtemp_thread_init(oplus_chip);
	oplus_tbatt_power_off_task_init(oplus_chip);
	sy6974b_irq_handler(PROBE_PLUG_IN_IRQ, sy);
	start_usb_enum_check();

	chg_info("successfully Part Num:%d, Revision:%d\n!", sy->part_no, sy->revision);
	chg_init_done = 1;
	return 0;
err_init:
err_parse_dt:
err_nodev:
	mutex_destroy(&chg->pinctrl_mutex);
	mutex_destroy(&sy->ntc_lock);
	mutex_destroy(&sy->i2c_rw_lock);
	devm_kfree(sy->dev, sy);
	return ret;
}

static unsigned long suspend_tm_sec = 0;
static int get_rtc_time(unsigned long *rtc_time)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		chg_err("Failed to open rtc device (%s)\n",
		CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		chg_err("Failed to read rtc time (%s) : %d\n",
		CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		chg_err("Invalid RTC time (%s): %d\n",
		CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, rtc_time);

	close_time:
	rtc_class_close(rtc);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int sy6974b_pm_resume(struct device *dev)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;
	struct sy697x *chip = NULL;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip) {
			atomic_set(&chip->driver_suspended, 0);
				wake_up_interruptible(&g_sy->wait);
			rc = get_rtc_time(&resume_tm_sec);
			if (rc || suspend_tm_sec == -1) {
				chg_err("RTC read failed\n");
				sleep_time = 0;
			 } else {
				sleep_time = resume_tm_sec - suspend_tm_sec;
			}
			if ((resume_tm_sec > suspend_tm_sec) && (sleep_time > 60)) {
				oplus_chg_soc_update_when_resume(sleep_time);
			}
		}
	}

	 return 0;
}

static int sy6974b_pm_suspend(struct device *dev)
{
	struct sy697x *chip = NULL;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip) {
			atomic_set(&chip->driver_suspended, 1);
			if (get_rtc_time(&suspend_tm_sec)) {
				chg_err("RTC read failed\n");
				suspend_tm_sec = -1;
			}
		}
	}
	return 0;
}

static const struct dev_pm_ops sy6974b_pm_ops = {
	 .resume = sy6974b_pm_resume,
	 .suspend = sy6974b_pm_suspend,
 };
#else
static int sy6974b_resume(struct i2c_client *client)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc;
	struct sy697x *chip = i2c_get_clientdata(client);

	if (!chip)
		return 0;

	atomic_set(&chip->driver_suspended, 0);
	wake_up_interruptible(&g_sy->wait);
	rc = get_rtc_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}
	if ((resume_tm_sec > suspend_tm_sec) && (sleep_time > 60)) {
		oplus_chg_soc_update_when_resume(sleep_time);
	}

	return 0;
}

static int sy6974b_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct sy697x *chip = i2c_get_clientdata(client);

	if (!chip)
		return 0;

	atomic_set(&chip->driver_suspended, 1);
	if (get_rtc_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}

	return 0;
}
#endif

static int sy6974b_charger_remove(struct i2c_client *client)
{
	struct sy697x *sy = i2c_get_clientdata(client);

	mutex_destroy(&sy->i2c_rw_lock);

	return 0;
}

#define OPLUS_CHG_SET_CHARGERID_SWITCH_OFF 0
static void sy6974b_charger_shutdown(struct i2c_client *client)
{
	if (g_sy) {
		sy6974b_adc_start(g_sy, false);
		if (!is_bq25890h(g_sy)) {
			sy6974b_disable_otg(g_sy);
		}

		if (g_oplus_chip) {
			oplus_vooc_reset_mcu();
			oplus_chg_set_chargerid_switch_val(OPLUS_CHG_SET_CHARGERID_SWITCH_OFF);
			oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
			msleep(30);
		}

		oplus_sy6974b_charger_unsuspend();
		if (g_oplus_chip && g_oplus_chip->chg_ops && g_oplus_chip->chg_ops->set_typec_sinkonly) {
			g_oplus_chip->chg_ops->set_typec_sinkonly();
		}
		chg_info("disable adc and otg sinkonly\n!");
	}
}

static struct i2c_driver sy6974b_charger_driver = {
	.driver = {
			.name = "sy6974b-charger",
			.owner = THIS_MODULE,
			.of_match_table = sy6974b_charger_match_table,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
			.pm = &sy6974b_pm_ops,
#endif
	},

	.probe = sy6974b_charger_probe,
	.remove = sy6974b_charger_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume = sy6974b_resume,
	.suspend = sy6974b_suspend,
#endif
	.shutdown = sy6974b_charger_shutdown,
	.id_table = sy6974b_i2c_device_id,
};

module_i2c_driver(sy6974b_charger_driver);

MODULE_DESCRIPTION("TI SY6974B Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
