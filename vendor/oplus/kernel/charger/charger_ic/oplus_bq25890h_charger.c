/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include "oplus_sy697x.h"
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mt-plat/upmu_common.h>
#include <mt-plat/charger_class.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_boot.h>
#include "../../supply/mediatek/charger/mtk_charger_intf.h"
#endif	/*CONFIG_OPLUS_CHARGER_MTK*/
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

#include "../oplus_charger.h"
#include "../oplus_gauge.h"

#include "../oplus_short.h"
#include "../oplus_adapter.h"
#include "../charger_ic/oplus_short_ic.h"
#include "../gauge_ic/oplus_bq27541.h"
#include <soc/oplus/boot_mode.h>
#ifdef CONFIG_TCPC_CLASS
#include "../../../../../kernel/msm-4.19/drivers/usb/typec/pd/inc/tcpci.h"
#include "../../../../../kernel/msm-4.19/drivers/usb/typec/pd/inc/tcpm.h"
#endif
#define _BQ25890H_
#include "oplus_bq25890h_charger.h"
#include "../oplus_configfs.h"
#include <linux/time.h>
#include <linux/hardware_info.h>

#undef chg_debug
#define chg_debug chg_err
#define PROBE_PLUG_IN_IRQ	0

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
static int chg_init_done = 0;

#undef chg_debug
#define chg_debug chg_err
static struct task_struct *charger_type_kthread;
static DECLARE_WAIT_QUEUE_HEAD(oplus_chgtype_wq);
extern struct oplus_chg_operations  oplus_chg_sgm4151x_ops;
static int oplus_get_ntc_tmp(struct iio_channel *channel);
static void oplus_usbtemp_recover_cc_open(void);
extern int oplus_chg_get_curr_time_ms(unsigned long *time_ms);
#define VBUS_RETRY_CNT 100
#define CHG_HV_THR     6500
#define HVDCP_THR     5800
#define CHG_SOC_THR    90

#define NORMAL_TEMP 25000
static int typc_temp1 = NORMAL_TEMP;
static int typc_temp2 = NORMAL_TEMP;
static int fled_temp = NORMAL_TEMP;
static int board_temp = NORMAL_TEMP;
static int chg_thermal_temp = NORMAL_TEMP;
static int bb_thermal_temp = NORMAL_TEMP;
static int flash_thermal_temp = NORMAL_TEMP;
static int board_thermal_temp = NORMAL_TEMP;
extern int typec_dir;

struct power_supply *charger_psy = NULL;
struct power_supply_desc charger_psd;
struct power_supply_config charger_cfg;
/*ntc table*/
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

#define OPLUS_TYPEC_DETACH	0
#define OPLUS_TYPEC_SRC_DFP	1
#define OPLUS_TYPEC_SNK_UFP	2
#define OPLUS_TYPEC_ACCESSORY	3

static bool disable_QC = false;
static bool dumpreg_by_irq = false;
static int  current_percent = SY697X_CUR_PRECENT50;

static struct sy697x *g_sy;
static struct task_struct *oplushg_usbtemp_kthread;
extern int oplus_usbtemp_monitor_common(void *data);
extern bool oplus_chg_wake_update_work(void);
bool opluschg_get_typec_cc_orientation(union power_supply_propval *val);
static bool is_bq25890h(struct sy697x *sy);
static void oplus_wake_up_usbtemp_thread(void);
static bool oplus_usbtemp_condition(void);
#ifdef CONFIG_OPLUS_CHARGER_MTK
void oplus_wake_up_usbtemp_thread(void);
extern struct oplus_chg_chip *g_oplus_chg;
extern int oplus_battery_meter_get_battery_voltage(void);
extern int oplus_get_rtc_ui_soc(void);
extern int oplus_set_rtc_ui_soc(int value);
extern int set_rtc_spare_fg_value(int val);
extern void mt_power_off(void);
extern bool pmic_chrdet_status(void);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);

void Charger_Detect_Init(void);
void Charger_Detect_Release(void);
#else	/*CONFIG_OPLUS_CHARGER_MTK*/
#define Charger_Detect_Init()
#define Charger_Detect_Release()
#define META_BOOT	0
static int oplus_bq25890h_get_vbus(void);
void oplus_bq25890h_dump_registers(void);
static int oplus_register_extcon(struct sy697x *chip);
static bool is_usb_rdy() {return true;}
#endif

extern void cpuboost_charge_event(int flag);

extern void oplus_chg_set_chargerid_switch_val(int value);

extern void oplus_chg_set_chargerid_switch_val(int value);

void oplus_bq25890h_set_mivr_by_battery_vol(void);
static void bq25890h_dump_regs(struct sy697x *sy);
int bq25890h_enable_enlim(struct sy697x *sy);
int oplus_bq25890h_set_aicr(int current_ma);
static bool oplus_get_otg_enable(void);
int oplus_bq25890h_enable_otg(void);
int oplus_bq25890h_disable_otg(void);
int oplus_bq25890h_get_charger_type(void);
void oplus_set_prswap(bool swap);

#ifdef CONFIG_OPLUS_CHARGER_MTK
static const struct charger_properties bq25890h_chg_props = {
	.alias_name = "bq25890h",
};
#endif /*CONFIG_OPLUS_CHARGER_MTK*/

static bool oplus_ccdetect_check_is_gpio(struct sy697x *chg);
static int oplus_ccdetect_gpio_init(struct sy697x *chg);
static void oplus_ccdetect_irq_init(struct sy697x *chg);
static void oplus_ccdetect_disable(void);
static void oplus_ccdetect_enable(void);
static bool oplus_ccdetect_support_check(void);
static int oplus_chg_ccdetect_parse_dt(struct oplus_chg_chip *chip);
bool oplus_get_otg_switch_status(void);
int oplus_typec_cc_orientation_show(void);

static irqreturn_t oplus_ccdetect_change_handler(int irq, void *data);

#define OPLUS_MODE_DRP	3
#define OPLUS_MODE_SNK	1
extern int oplus_sgm7220_set_mode(int mode);
extern void oplus_usbtemp_recover_func(struct oplus_chg_chip *chip);
static bool oplus_is_prswap;
struct oplus_chg_chip *g_oplus_chg;

static int oplus_get_usb_status(void)
{
	if (g_oplus_chg && g_oplus_chg->usb_status == USB_TEMP_HIGH) {
		return g_oplus_chg->usb_status;
	} else {
		return 0;
	}
}

static int oplus_set_mode(int mode)
{
	int rc = 0;

	if (!g_sy->tcpc) {
		return -EINVAL;
	}
	if (mode == OPLUS_MODE_DRP) {
#ifdef CONFIG_TCPC_CLASS
		rc = tcpm_typec_change_role(g_sy->tcpc, TYPEC_ROLE_DRP);
	} else {
		rc = tcpm_typec_change_role(g_sy->tcpc, TYPEC_ROLE_SNK);
	}
#else
		rc = oplus_sgm7220_set_mode(OPLUS_MODE_DRP);
	} else {
		rc = oplus_sgm7220_set_mode(OPLUS_MODE_SNK);
	}
#endif
	return rc;
}

#ifdef CONFIG_TCPC_CLASS
void notify_pd_event(unsigned long evt)
{
	switch (evt) {
	case PD_CONNECT_NONE:
		g_sy->pd_type = PD_CONNECT_NONE;
		break;

	case PD_CONNECT_HARD_RESET:
		chg_err("PD Notify HardReset\n");
		g_sy->pd_type = PD_CONNECT_NONE;
		/* reset PE40 */
		break;

	case PD_CONNECT_PE_READY_SNK:
		chg_err("PD Notify fixe voltage ready\n");
		g_sy->pd_type = PD_CONNECT_PE_READY_SNK;

		/* PD is ready */
		break;

	case PD_CONNECT_PE_READY_SNK_PD30:
		chg_err("PD Notify PD30 ready\r\n");
		g_sy->pd_type = PD_CONNECT_PE_READY_SNK_PD30;

		/* PD30 is ready */
		break;

	case PD_CONNECT_PE_READY_SNK_APDO:
		chg_err("PD Notify APDO Ready\n");
		g_sy->pd_type = PD_CONNECT_PE_READY_SNK_APDO;

		break;

	case PD_CONNECT_TYPEC_ONLY_SNK:
		chg_err("PD Notify Type-C Ready\n");
		g_sy->pd_type = PD_CONNECT_TYPEC_ONLY_SNK;

		break;
		}
}

#define VBUS_9V	9000
#define VBUS_5V	5000
#define IBUS_2A	2000
#define IBUS_3A	3000
int oplus_bq25890h_get_pd_type(void)
{
	if (g_sy != NULL) {
		if (g_sy->pd_type == PD_CONNECT_PE_READY_SNK ||
				g_sy->pd_type == PD_CONNECT_PE_READY_SNK_PD30) {
			return true;
		} else if (g_sy->pd_type == PD_CONNECT_PE_READY_SNK_APDO) {
				return true;
		} else {
			return false;
		}
	}

	return false;
}
EXPORT_SYMBOL(oplus_bq25890h_get_pd_type);

static int pd_get_cap(enum adapter_cap_type type, struct adapter_power_cap *tacap)
{
	struct tcpm_power_cap_val apdo_cap;
	struct tcpm_remote_power_cap pd_cap;
	struct pd_source_cap_ext cap_ext;

	uint8_t cap_i = 0;
	int ret;
	unsigned int idx = 0;
	int i;

	if (!g_sy->tcpc) {
		chg_debug("tcpm_dpm_pd_request fail\n");
		return -EINVAL;
	}

	if (type == MTK_PD_APDO) {
		while (1) {
			ret = tcpm_inquire_pd_source_apdo(g_sy->tcpc,
					TCPM_POWER_CAP_APDO_TYPE_PPS,
					&cap_i, &apdo_cap);
			if (ret == TCPM_ERROR_NOT_FOUND) {
				break;
			} else if (ret != TCPM_SUCCESS) {
				chg_err("[%s] tcpm_inquire_pd_source_apdo failed(%d)\n",
					__func__, ret);
				break;
			}

			ret = tcpm_dpm_pd_get_source_cap_ext(g_sy->tcpc,
					NULL, &cap_ext);
			if (ret == TCPM_SUCCESS)
				tacap->pdp = cap_ext.source_pdp;
			else {
				tacap->pdp = 0;
				chg_err("[%s] tcpm_dpm_pd_get_source_cap_ext failed(%d)\n",
					__func__, ret);
			}

			tacap->pwr_limit[idx] = apdo_cap.pwr_limit;
			tacap->ma[idx] = apdo_cap.ma;
			tacap->max_mv[idx] = apdo_cap.max_mv;
			tacap->min_mv[idx] = apdo_cap.min_mv;
			tacap->maxwatt[idx] = apdo_cap.max_mv * apdo_cap.ma;
			tacap->minwatt[idx] = apdo_cap.min_mv * apdo_cap.ma;
			tacap->type[idx] = MTK_PD_APDO;

			idx++;
			chg_err("pps_boundary[%d], %d mv ~ %d mv, %d ma pl:%d\n",
				cap_i,
				apdo_cap.min_mv, apdo_cap.max_mv,
				apdo_cap.ma, apdo_cap.pwr_limit);
			if (idx >= ADAPTER_CAP_MAX_NR) {
				chg_err("CAP NR > %d\n", ADAPTER_CAP_MAX_NR);
				break;
			}
		}
		tacap->nr = idx;

		for (i = 0; i < tacap->nr; i++) {
			chg_err("pps_cap[%d:%d], %d mv ~ %d mv, %d ma pl:%d pdp:%d\n",
				i, (int)tacap->nr, tacap->min_mv[i],
				tacap->max_mv[i], tacap->ma[i],
				tacap->pwr_limit[i], tacap->pdp);
		}

		if (cap_i == 0)
			chg_err("no APDO for pps\n");

	} else if (type == MTK_PD) {
		pd_cap.nr = 0;
		pd_cap.selected_cap_idx = 0;
		tcpm_get_remote_power_cap(g_sy->tcpc, &pd_cap);

		if (pd_cap.nr != 0) {
			tacap->nr = pd_cap.nr;
			tacap->selected_cap_idx = pd_cap.selected_cap_idx - 1;
			chg_err("[%s] nr:%d idx:%d\n",
			__func__, pd_cap.nr, pd_cap.selected_cap_idx - 1);
			for (i = 0; i < pd_cap.nr; i++) {
				tacap->ma[i] = pd_cap.ma[i];
				tacap->max_mv[i] = pd_cap.max_mv[i];
				tacap->min_mv[i] = pd_cap.min_mv[i];
				tacap->maxwatt[i] =
					tacap->max_mv[i] * tacap->ma[i];
				if (pd_cap.type[i] == 0)
					tacap->type[i] = MTK_PD;
				else if (pd_cap.type[i] == 3)
					tacap->type[i] = MTK_PD_APDO;
				else
					tacap->type[i] = MTK_CAP_TYPE_UNKNOWN;
				tacap->type[i] = pd_cap.type[i];

				chg_err("[%s]:%d mv:[%d,%d] %d max:%d min:%d type:%d %d\n",
					__func__, i, tacap->min_mv[i],
					tacap->max_mv[i], tacap->ma[i],
					tacap->maxwatt[i], tacap->minwatt[i],
					tacap->type[i], pd_cap.type[i]);
			}
		}
	}

	return 0;
}
int oplus_pdc_setup(int *vbus_mv, int *ibus_ma)
{
	int ret = 0;
	int vbus_mv_t = 0;
	int ibus_ma_t = 0;

	if (!g_sy->tcpc) {
		chg_err("%s: tcpm_dpm_pd_request fail\n", __func__);
		return -EINVAL;
	}

	ret = tcpm_dpm_pd_request(g_sy->tcpc, *vbus_mv, *ibus_ma, NULL);
	if (ret != TCPM_SUCCESS) {
		chg_err("%s: tcpm_dpm_pd_request fail\n", __func__);
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(g_sy->tcpc, &vbus_mv_t, &ibus_ma_t);
	if (ret != TCPM_SUCCESS) {
		chg_err("%s: inquire current vbus_mv and ibus_ma fail\n", __func__);
		return -EINVAL;
	}

	chg_err("%s: request vbus_mv[%d], ibus_ma[%d]\n", __func__, vbus_mv_t, ibus_ma_t);
	return 0;
}
int oplus_pd_setup(void)
{
	int vbus_mv = VBUS_5V;
	int ibus_ma = IBUS_2A;
	int ret = -1;
	struct adapter_power_cap cap;
	struct oplus_chg_chip *chip = g_oplus_chg;
	int i;
	cap.nr = 0;
	cap.pdp = 0;
	for (i = 0; i < ADAPTER_CAP_MAX_NR; i++) {
			cap.max_mv[i] = 0;
			cap.min_mv[i] = 0;
			cap.ma[i] = 0;
			cap.type[i] = 0;
			cap.pwr_limit[i] = 0;
		}

	chg_err("pd_type: %d\n", g_sy->pd_type);

	if (!chip->calling_on && !chip->camera_on && chip->charger_volt < CHG_HV_THR && chip->soc < CHG_SOC_THR
		&& chip->temperature <= chip->limits.tbatt_pdqc_to_5v_thr && chip->cool_down_force_5v == false) {
		if (g_sy->pd_type == PD_CONNECT_PE_READY_SNK_APDO) {
			pd_get_cap(MTK_PD_APDO, &cap);
			for (i = 0; i < cap.nr; i++) {
				chg_err("PD APDO cap %d: mV:%d,%d mA:%d type:%d pwr_limit:%d pdp:%d\n", i,
					cap.max_mv[i], cap.min_mv[i], cap.ma[i],
					cap.type[i], cap.pwr_limit[i], cap.pdp);
			}

			for (i = 0; i < cap.nr; i++) {
				if (cap.min_mv[i] <= VBUS_9V && VBUS_9V <= cap.max_mv[i]) {
					vbus_mv = VBUS_9V;
					ibus_ma = cap.ma[i];
					if (ibus_ma > IBUS_2A)
						ibus_ma = IBUS_2A;
					break;
				}
			}
		} else if (g_sy->pd_type == PD_CONNECT_PE_READY_SNK
			|| g_sy->pd_type == PD_CONNECT_PE_READY_SNK_PD30) {
			pd_get_cap(MTK_PD, &cap);
			for (i = 0; i < cap.nr; i++) {
				chg_err("PD cap %d: mV:%d,%d mA:%d type:%d\n", i,
					cap.max_mv[i], cap.min_mv[i], cap.ma[i], cap.type[i]);
			}

			for (i = 0; i < cap.nr; i++) {
				if (VBUS_9V <= cap.max_mv[i]) {
					vbus_mv = cap.max_mv[i];
					ibus_ma = cap.ma[i];
					if (ibus_ma > IBUS_2A)
						ibus_ma = IBUS_2A;
					break;
				}
			}
		} else {
			vbus_mv = VBUS_5V;
			ibus_ma = IBUS_2A;
		}
		chg_err("PD request: %dmV, %dmA\n", vbus_mv, ibus_ma);
		ret = oplus_pdc_setup(&vbus_mv, &ibus_ma);
	} else {
		if (chip->charger_volt > CHG_HV_THR &&
			(chip->calling_on || chip->camera_on || chip->soc >= CHG_SOC_THR
			|| chip->batt_volt >= chip->limits.tbatt_pdqc_to_5v_thr
			|| chip->temperature > chip->limits.tbatt_pdqc_to_5v_thr || chip->cool_down_force_5v == true)) {
			vbus_mv = VBUS_5V;
			ibus_ma = IBUS_2A;

			chg_err("PD request: %dmV, %dmA\n", vbus_mv, ibus_ma);
			ret = oplus_pdc_setup(&vbus_mv, &ibus_ma);
		}
	}

	return ret;
}
#endif

static bool oplus_ccdetect_check_is_gpio(struct sy697x *chg)
{
	if (!chg) {
		chg_err("[OPLUS_CHG][%s]: oplus_chg not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chg->ccdetect_gpio)) {
		return true;
	}

	return false;
}

static int oplus_ccdetect_gpio_init(struct sy697x *chg)
{
	if (!chg) {
		chg_err("oplus_chg not ready!\n");
		return -EINVAL;
	}

	chg->pinctrl = devm_pinctrl_get(chg->dev);

	if (IS_ERR_OR_NULL(chg->pinctrl)) {
		chg_err("get ccdetect_pinctrl fail\n");
		return -EINVAL;
	}

	chg->ccdetect_active = pinctrl_lookup_state(chg->pinctrl, "ccdetect_active");
	if (IS_ERR_OR_NULL(chg->ccdetect_active)) {
		chg_err("get ccdetect_active fail\n");
		return -EINVAL;
	}

	chg->ccdetect_sleep = pinctrl_lookup_state(chg->pinctrl, "ccdetect_sleep");
	if (IS_ERR_OR_NULL(chg->ccdetect_sleep)) {
		chg_err("get ccdetect_sleep fail\n");
		return -EINVAL;
	}
	if (chg->ccdetect_gpio > 0) {
		gpio_direction_input(chg->ccdetect_gpio);
	}

	pinctrl_select_state(chg->pinctrl,  chg->ccdetect_active);
	return 0;
}

static void oplus_ccdetect_irq_init(struct sy697x *chg)
{
	if (!chg) {
		chg_err("[OPLUS_CHG][%s]: oplus_chg not ready!\n", __func__);
		return;
	}
	chg->ccdetect_irq = gpio_to_irq(chg->ccdetect_gpio);
	chg_err("[OPLUS_CHG][%s]: chg->ccdetect_gpio[%d]!\n", __func__, chg->ccdetect_gpio);
}

static void oplus_ccdetect_irq_register(struct oplus_chg_chip *chip)
{
	int ret = 0;
	struct sy697x *chg = g_sy;

	if (!chg) {
		chg_err("[OPLUS_CHG][%s]: oplus_chg not ready!\n", __func__);
		return;
	}

	ret = devm_request_threaded_irq(chg->dev,  chg->ccdetect_irq,
			NULL, oplus_ccdetect_change_handler, IRQF_TRIGGER_FALLING
			| IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ccdetect-change", chg);
	if (ret < 0) {
		chg_err("Unable to request ccdetect-change irq: %d\n", ret);
	}
	chg_err("%s: !!!!! irq register\n", __FUNCTION__);

	ret = enable_irq_wake(chg->ccdetect_irq);
	if (ret != 0) {
		chg_err("enable_irq_wake: ccdetect_irq failed %d\n", ret);
	}
}

static void oplus_ccdetect_enable(void)
{
	struct sy697x *chg = g_sy;

	if (!chg) {
		chg_err("[OPLUS_CHG][%s]: oplus_chg not ready!\n", __func__);
		return;
	}

	if (oplus_ccdetect_check_is_gpio(chg) != true)
		return;

	if (g_oplus_chg->usb_status == USB_TEMP_HIGH) {
		return;
	}
	/* set DRP mode */
	oplus_set_mode(OPLUS_MODE_DRP);
	chg_err("%s: set drp", __func__);
}

static void oplus_ccdetect_disable(void)
{
	struct sy697x *chg = g_sy;

	if (!chg) {
		chg_err("[OPLUS_CHG][%s]: oplus_chg not ready!\n", __func__);
		return;
	}

	if (oplus_ccdetect_check_is_gpio(chg) != true)
		return;

	/* set SINK mode */
	oplus_set_mode(OPLUS_MODE_SNK);
	chg_err("%s: set sink", __func__);
}

static bool oplus_ccdetect_support_check(void)
{
	struct sy697x *chg = g_sy;

	if (!chg) {
		chg_err("[OPLUS_CHG][%s]: oplus_chg not ready!\n", __func__);
		return false;
	}

	if (oplus_ccdetect_check_is_gpio(chg) == true)
		return true;

	chg_err("not support, return false\n");

	return false;
}

static int oplus_chg_ccdetect_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;
	struct sy697x *chg = g_sy;

	if (chg)
		node = chg->dev->of_node;
	if (node == NULL) {
		chg_err("oplus_chg or device tree info. missing\n");
		return -EINVAL;
	}
	chg->ccdetect_gpio = of_get_named_gpio(node, "qcom,ccdetect-gpio", 0);
	if (chg->ccdetect_gpio <= 0) {
		chg_err("Couldn't read qcom, ccdetect-gpio rc=%d, qcom, ccdetect-gpio:%d\n",
				rc, chg->ccdetect_gpio);
	} else {
		if (oplus_ccdetect_support_check() == true) {
			rc = gpio_request(chg->ccdetect_gpio, "ccdetect-gpio");
			if (rc) {
				chg_err("unable to request ccdetect_gpio:%d\n",
						chg->ccdetect_gpio);
			} else {
				rc = oplus_ccdetect_gpio_init(chg);
				if (rc) {
					chg_err("unable to init ccdetect_gpio:%d\n",
							chg->ccdetect_gpio);
				} else {
					oplus_ccdetect_irq_init(chg);
					}
			}
		}
		chg_err("ccdetect-gpio:%d\n", chg->ccdetect_gpio);
	}

	return rc;
}

#define CCDETECT_DELAY_MS	50
static struct delayed_work ccdetect_work;
static struct delayed_work usbtemp_recover_work;
static irqreturn_t oplus_ccdetect_change_handler(int irq, void *data)
{
	cancel_delayed_work_sync(&ccdetect_work);
	chg_debug("Scheduling ccdetect work!\n");
	schedule_delayed_work(&ccdetect_work, msecs_to_jiffies(CCDETECT_DELAY_MS));

	return IRQ_HANDLED;
}

static void oplus_ccdetect_work(struct work_struct *work)
{
	int level;
	struct sy697x *chg = g_sy;

	level = gpio_get_value(chg->ccdetect_gpio);
	chg_err("%s: level [%d]", __func__, level);
	if (level != 1) {
		oplus_ccdetect_enable();
		oplus_wake_up_usbtemp_thread();
	} else {
		if (g_oplus_chg)
			g_oplus_chg->usbtemp_check = oplus_usbtemp_condition();

                if (g_oplus_chg->usb_status == USB_TEMP_HIGH) {
			oplus_usbtemp_recover_cc_open();
		}
		if (oplus_get_otg_switch_status() == false)
			oplus_ccdetect_disable();
		if (g_oplus_chg->usb_status == USB_TEMP_HIGH) {
			schedule_delayed_work(&usbtemp_recover_work, 0);
		}
	}
}

static void oplus_usbtemp_recover_work(struct work_struct *work)
{
	struct oplus_chg_chip *chip = g_oplus_chg;

	if (!chip) {
		return;
	}
	chg_err("%s: enter", __func__);
	oplus_usbtemp_recover_func(g_oplus_chg);
}

static void oplus_notify_extcon_props(struct sy697x *chg, int id)
{
	union extcon_property_value val;
	union power_supply_propval prop_val;

	opluschg_get_typec_cc_orientation(&prop_val);
	val.intval = ((prop_val.intval == 2) ? 1 : 0);
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
		chg_err("[%s] chg is null\n", __func__);
		return;
	}
	if (enable)
		oplus_notify_extcon_props(chg, EXTCON_USB);

	extcon_set_state_sync(chg->extcon, EXTCON_USB, enable);
	chg_err("[%s] enable[%d]\n", __func__, enable);
}

static void oplus_notify_usb_host(bool enable)
{
	struct sy697x *chg = g_sy;

	if (!chg || !g_oplus_chg) {
		pr_err("[%s] chg or g_oplus_chg is null\n", __func__);
		return;
	}
	if (enable) {
		pr_debug("enabling VBUS in OTG mode\n");
		oplus_bq25890h_enable_otg();
		oplus_notify_extcon_props(chg, EXTCON_USB_HOST);
	} else {
		pr_debug("disabling VBUS in OTG mode\n");
		oplus_bq25890h_disable_otg();
	}

	power_supply_changed(g_oplus_chg->usb_psy);
	extcon_set_state_sync(chg->extcon, EXTCON_USB_HOST, enable);
	pr_debug("[%s] enable[%d]\n", __func__, enable);
}

void oplus_bq25890h_typec_sink_removal(void)
{
	struct sy697x *chg = g_sy;

	if (!chg) {
		pr_err("[%s] chg is null\n", __func__);
		return;
	}
	if (chg->otg_present)
		oplus_notify_usb_host(false);
	chg->otg_present = false;
	g_oplus_chg->otg_online = false;
	oplus_chg_wake_update_work();
	pr_debug("wakeup [%s] done!!!\n", __func__);
}

void oplus_bq25890h_typec_src_removal(void)
{
	struct sy697x *chg = g_sy;

	if (!chg) {
		pr_err("[%s] chg is null\n", __func__);
		return;
	}
	oplus_notify_device_mode(false);
	pr_debug("[%s] done!!!\n", __func__);
}

void oplus_bq25890h_typec_sink_insertion(void)
{
	struct sy697x *chg = g_sy;

	if (!chg) {
		pr_err("[%s] chg is null\n", __func__);
		return;
	}
	chg->otg_present = true;
	oplus_chg_wake_update_work();

	oplus_notify_usb_host(true);
	pr_debug("wakeup[%s] done!!!\n", __func__);
}

#define DISCONNECT						0
#define STANDARD_TYPEC_DEV_CONNECT	BIT(0)
#define OTG_DEV_CONNECT				BIT(1)
static bool oplus_get_otg_online_status_default(void)
{
	if (!g_sy || (g_sy->tcpc == NULL)) {
		chg_err("fail to init oplus_chg\n");
		return false;
	}
#ifdef CONFIG_TCPC_CLASS
	if (tcpm_inquire_typec_attach_state(g_sy->tcpc) == TYPEC_ATTACHED_SRC)
		g_sy->otg_present = true;
	else
		g_sy->otg_present = false;
#endif
	return g_sy->otg_present;
}

bool oplus_bq25890h_get_otg_switch_status(void)
{
	struct sy697x *chg = g_sy;

	if (!chg) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg not ready!\n", __func__);
		return false;
	}

	return g_oplus_chg->otg_switch;
}
#ifdef CONFIG_TCPC_CLASS
void oplus_bq25890h_set_otg_switch_status(bool value)
{
	if(g_oplus_chg != NULL && g_sy != NULL && g_sy->tcpc != NULL) {
		g_oplus_chg->ui_otg_switch = value;
		g_oplus_chg->otg_switch = value;

		if(oplus_ccdetect_check_is_gpio(g_sy) == true) {
			if(gpio_get_value(g_sy->ccdetect_gpio) == 0) {
				printk(KERN_ERR "[OPLUS_CHG][oplus_set_otg_switch_status]: gpio[L], should set, return\n");
				return;
			}
		}

		printk(KERN_ERR "[OPLUS_CHG][%s]:OTG TEST otg switch[%d]\n", __func__, value);
		tcpm_typec_change_role(g_sy->tcpc, value ? TYPEC_ROLE_DRP : TYPEC_ROLE_SNK);
	}
}
#endif
int oplus_bq25890h_get_otg_online_status(void)
{
	int online = 0;
	int level = 0;
	int typec_otg = 0;
	static int pre_level = 1;
	static int pre_typec_otg = 0;
	struct sy697x *chg = g_sy;

	if (!chg || !g_oplus_chg) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: chg or g_oplus_chg ...\n", __func__);
		return false;
	}

	if (oplus_ccdetect_check_is_gpio(chg) == true) {
		level = gpio_get_value(chg->ccdetect_gpio);
		if (level != gpio_get_value(chg->ccdetect_gpio)) {
			printk(KERN_ERR "[OPLUS_CHG][%s]: ccdetect_gpio is unstable, try again...\n", __func__);
			usleep_range(5000, 5100);
			level = gpio_get_value(chg->ccdetect_gpio);
		}
	} else {
		return oplus_get_otg_online_status_default();
	}
	online = (level == 1) ? DISCONNECT : STANDARD_TYPEC_DEV_CONNECT;

	typec_otg = oplus_get_otg_online_status_default();
	online = online | ((typec_otg == 1) ? OTG_DEV_CONNECT : DISCONNECT);

	if ((pre_level ^ level) || (pre_typec_otg ^ typec_otg)) {
		pre_level = level;
		pre_typec_otg = typec_otg;
		printk(KERN_ERR "[OPLUS_CHG][%s]: gpio[%s], c-otg[%d], otg_online[%d]\n",
				__func__, level ? "H" : "L", typec_otg, online);
	}

	g_oplus_chg->otg_online = online;
	return online;
}


static int oplus_get_iio_channel(struct sy697x *chip, const char *propname,
					struct iio_channel **chan)
{
	int rc = 0;

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

static int oplus_thermal_get_tmp(void)
{
	int ntcctrl_gpio_value = 0;
	int ret = 0;
	struct sy697x *chip = g_sy;
	if (!chip || !chip->pinctrl
		|| !chip->iio.ntc_switch1_chan || !chip->iio.ntc_switch2_chan) {

		ret = oplus_get_iio_channel(chip, "ntc_switch1_chan", &chip->iio.ntc_switch1_chan);
		if (ret < 0 && !chip->iio.ntc_switch1_chan) {
			return -1;
		}

		ret = oplus_get_iio_channel(chip, "ntc_switch2_chan", &chip->iio.ntc_switch2_chan);
		if (ret < 0 && !chip->iio.ntc_switch2_chan) {
			pr_err(" %s ntc_switch2_chan get failed\n", __func__);
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
	} else {
		/*do nothing*/
	}

	return 0;
}

int oplus_bq25890h_thermal_tmp_get_chg(void)
{
	if (g_sy) {
		mutex_lock(&g_sy->ntc_lock);
		oplus_thermal_get_tmp();
		mutex_unlock(&g_sy->ntc_lock);
	}

	return chg_thermal_temp;
}
int oplus_bq25890h_thermal_tmp_get_bb(void)
{
	if (g_sy) {
		mutex_lock(&g_sy->ntc_lock);
		oplus_thermal_get_tmp();
		mutex_unlock(&g_sy->ntc_lock);
	}

	return bb_thermal_temp;
}

int oplus_bq25890h_thermal_tmp_get_flash(void)
{
	if (g_sy) {
		mutex_lock(&g_sy->ntc_lock);
		oplus_thermal_get_tmp();
		mutex_unlock(&g_sy->ntc_lock);
	}

	return flash_thermal_temp;
}

int oplus_bq25890h_thermal_tmp_get_board(void)
{
	if (g_sy) {
		mutex_lock(&g_sy->ntc_lock);
		oplus_thermal_get_tmp();
		mutex_unlock(&g_sy->ntc_lock);
	}

	return board_thermal_temp;
}
#define DELAY10 10
#define CDP_TIMEOUT 40
static void oplus_for_cdp(void)
{
	int timeout = CDP_TIMEOUT;
	if (is_usb_rdy() == false) {
		while (is_usb_rdy() == false && timeout > 0) {
			msleep(DELAY10);
			timeout--;
		}
		if (timeout == 0)
			chg_debug("usb_rdy timeout\n");
		else
			chg_debug("usb_rdy free\n");
	} else
			chg_debug("rm cdp 400ms usb_rdy PASS\n");
}

static int g_bq25890h_read_reg(struct sy697x *sy, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(sy->client, reg);
	if (ret < 0) {
		chg_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int g_bq25890h_write_reg(struct sy697x *sy, int reg, u8 val)
{
	s32 ret;
	ret = i2c_smbus_write_byte_data(sy->client, reg, val);
	if (ret < 0) {
		chg_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

#define READ_DELAY 10000
#define READ_CNT 4
static int bq25890h_read_byte(struct sy697x *sy, u8 reg, u8 *data)
{
	int ret;
	int retry = READ_CNT;

	mutex_lock(&sy->i2c_rw_lock);
	ret = g_bq25890h_read_reg(sy, reg, data);
	if (ret < 0) {
		while(retry > 0 && atomic_read(&sy->driver_suspended) == 0) {
			usleep_range(READ_DELAY, READ_DELAY);
			ret = g_bq25890h_read_reg(sy, reg, data);
			chg_err("%s: ret = %d \n", __func__, ret);
			if (ret < 0) {
				retry--;
			} else {
				break;
			}
		}
	}
	mutex_unlock(&sy->i2c_rw_lock);

	return ret;
}

static int __bq25890h_update_bits(struct sy697x *sy, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	/*mutex_lock(&sy->i2c_rw_lock);*/
	ret = g_bq25890h_read_reg(sy, reg, &tmp);
	if (ret) {
		chg_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = g_bq25890h_write_reg(sy, reg, tmp);
	if (ret)
		chg_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	/*mutex_unlock(&sy->i2c_rw_lock);*/
	return ret;
}

static int bq25890h_update_bits(struct sy697x *sy, u8 reg, u8 mask, u8 data)
{
	int ret;
	int retry = READ_CNT;

	mutex_lock(&sy->i2c_rw_lock);
	ret = __bq25890h_update_bits(sy, reg, mask, data);
	if (ret < 0) {
		while(retry > 0 && atomic_read(&sy->driver_suspended) == 0) {
			usleep_range(READ_DELAY, READ_DELAY);
			ret = __bq25890h_update_bits(sy, reg, mask, data);
			chg_err("%s: ret = %d \n", __func__, ret);
			if (ret < 0) {
				retry--;
			} else {
				break;
			}
		}
	}
	mutex_unlock(&sy->i2c_rw_lock);
	return ret;
}

static int bq25890h_enable_otg(struct sy697x *sy)
{
	u8 val = SY697X_OTG_ENABLE << SY697X_OTG_CONFIG_SHIFT;

	return bq25890h_update_bits(sy, SY697X_REG_03,
				   SY697X_OTG_CONFIG_MASK, val);
}

static int bq25890h_vmin_limit(struct sy697x *sy)
{
	u8 val = 4 << SY697X_SYS_MINV_SHIFT;

	return bq25890h_update_bits(sy, SY697X_REG_03,
				   SY697X_SYS_MINV_MASK, val);
}

static int bq25890h_disable_otg(struct sy697x *sy)
{
	u8 val = SY697X_OTG_DISABLE << SY697X_OTG_CONFIG_SHIFT;

	return bq25890h_update_bits(sy, SY697X_REG_03,
				   SY697X_OTG_CONFIG_MASK, val);
}

static int bq25890h_enable_hvdcp(struct sy697x *sy)
{
	int ret;
	u8 val = SY697X_HVDCP_ENABLE << SY697X_HVDCPEN_SHIFT;
	chg_err("bq25890h_enable_hvdcp do nothing\n");
	ret = bq25890h_update_bits(sy, SY697X_REG_02,
				SY697X_HVDCPEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq25890h_enable_hvdcp);

static int bq25890h_disable_hvdcp(struct sy697x *sy)
{
	int ret = 0;
	u8 val = SY697X_HVDCP_DISABLE << SY697X_HVDCPEN_SHIFT;
	chg_err("bq25890h_disable_hvdcp \n");
	ret = bq25890h_update_bits(sy, SY697X_REG_02,
				SY697X_HVDCPEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq25890h_disable_hvdcp);

static int bq25890h_disable_batfet_rst(struct sy697x *sy)
{
	int ret = 0;
	u8 val = SY697X_BATFET_RST_EN_DISABLE << SY697X_BATFET_RST_EN_SHIFT;
	chg_err("disable_batfet_rst \n");
	ret = bq25890h_update_bits(sy, SY697X_REG_09,
				SY697X_BATFET_RST_EN_MASK, val);
	return ret;
}

static int bq2589x_disable_maxc(struct sy697x *bq)
{
	int ret;
	u8 val = SY697X_MAXC_DISABLE << SY697X_MAXCEN_SHIFT;

	ret = bq25890h_update_bits(bq, SY697X_REG_02,
				SY697X_MAXCEN_MASK, val);
	return ret;
}


static int bq25890h_disable_ico(struct sy697x *sy)
{
	int ret = 0;
	u8 val = SY697X_ICO_DISABLE << SY697X_ICOEN_SHIFT;
	chg_err("bq25890h_disable_ico\n");
	ret = bq25890h_update_bits(sy, SY697X_REG_02,
				SY697X_ICOEN_MASK, val);
	return ret;
}
static int bq25890h_enable_charger(struct sy697x *sy)
{
	int ret;

	u8 val = SY697X_CHG_ENABLE << SY697X_CHG_CONFIG_SHIFT;

	chg_err("%s\n", __func__);
	ret = bq25890h_update_bits(sy, SY697X_REG_03,
				SY697X_CHG_CONFIG_MASK, val);
	return ret;
}

static int bq25890h_disable_charger(struct sy697x *sy)
{
	int ret;

	u8 val = SY697X_CHG_DISABLE << SY697X_CHG_CONFIG_SHIFT;

	chg_err("%s\n", __func__);
	ret = bq25890h_update_bits(sy, SY697X_REG_03,
				SY697X_CHG_CONFIG_MASK, val);
	return ret;
}

#define ADC_RETRY 10
bool bq25890h_adc_ready(struct sy697x *sy)
{
	u8 val;
	int ret;
	int retry = ADC_RETRY;

	while (retry--) {
		ret = bq25890h_read_byte(sy, SY697X_REG_02, &val);
		if (ret < 0) {
			chg_err("%s failed to read register 0x02:%d\n", __func__, ret);
			return ret;
		}
		if (!(val & SY697X_CONV_START_MASK))
			return true;
		mdelay(DELAY10);
	}

	return false;
}

int bq25890h_adc_start(struct sy697x *sy, bool enable)
{
	int ret;

	if (enable) {
		ret = bq25890h_update_bits(sy, SY697X_REG_02, SY697X_CONV_START_MASK,
				SY697X_CONV_START_ENABLE << SY697X_CONV_START_SHIFT);
	} else {
		ret = bq25890h_update_bits(sy, SY697X_REG_02, SY697X_CONV_RATE_MASK,
				SY697X_ADC_CONTINUE_DISABLE << SY697X_CONV_RATE_SHIFT);
		ret = bq25890h_update_bits(sy, SY697X_REG_02, SY697X_CONV_START_MASK,
				SY697X_ADC_CONTINUE_DISABLE << SY697X_CONV_START_SHIFT);
	}

	return ret;
}

EXPORT_SYMBOL_GPL(bq25890h_adc_start);


#define RETRY_CNT  20
#define DELAY25 25
int bq25890h_adc_read_vbus_volt(struct sy697x *sy)
{
	uint8_t val;
	int volt;
	int ret;
	int retry = 0;

	bq25890h_adc_start(sy, true);
	while(!bq25890h_adc_ready(sy) && retry <= VBUS_RETRY_CNT) {
		mdelay(DELAY25);
		retry++;
	}
	ret = bq25890h_read_byte(sy, SY697X_REG_11, &val);
	if (ret < 0) {
		chg_err("read vbus voltage failed :%d\n", ret);
		return ret;
	} else {
		volt = SY697X_VBUSV_BASE + ((val & SY697X_VBUSV_MASK) >> SY697X_VBUSV_SHIFT) * SY697X_VBUSV_LSB;
		if (volt == SY697X_VBUSV_BASE) {
			volt = 0;
		}
	}
	if (retry >= RETRY_CNT) {
		chg_err("bq25890h_adc_read_vbus_volt mdelay retry, volt[%d, %d] times=retry x 25[%d]ms\n", retry, volt, retry * 25);
	}

	return volt;
}
EXPORT_SYMBOL_GPL(bq25890h_adc_read_vbus_volt);

int bq25890h_adc_read_charge_current(void)
{
	uint8_t val;
	int curr;
	int ret;

	if (!g_sy)
		return 0;

	ret = bq25890h_adc_ready(g_sy);
	if (ret)
		chg_err("bq25890h_adc_ready\n");
	else
		return 0;

	ret = bq25890h_read_byte(g_sy, SY697X_REG_12, &val);
	if (ret < 0) {
		chg_err("read charge current failed :%d\n", ret);
	} else {
		curr = (int)(SY697X_ICHGR_BASE + ((val & SY697X_ICHGR_MASK) >> SY697X_ICHGR_SHIFT) * SY697X_ICHGR_LSB);
		return curr;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(bq25890h_adc_read_charge_current);
int bq25890h_set_chargecurrent(struct sy697x *sy, int curr)
{
	u8 ichg;
	chg_err("%s: ichg = %d\n", __func__, curr);

	if (curr < SY697X_ICHG_BASE)
		curr = SY697X_ICHG_BASE;

	ichg = (curr - SY697X_ICHG_BASE)/SY697X_ICHG_LSB;
	return bq25890h_update_bits(sy, SY697X_REG_04,
						SY697X_ICHG_MASK, ichg << SY697X_ICHG_SHIFT);
}

#define NUM_0 0

int bq25890h_set_term_current(struct sy697x *sy, int curr)
{
	u8 iterm;

	if (curr < SY697X_ITERM_BASE)
		curr = SY697X_ITERM_BASE;

	iterm = (curr - SY697X_ITERM_BASE) / SY697X_ITERM_LSB;

	if(((curr - SY697X_ITERM_BASE) % SY697X_ITERM_LSB) != NUM_0)
		iterm++;

	return bq25890h_update_bits(sy, SY697X_REG_05,
						SY697X_ITERM_MASK, iterm << SY697X_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq25890h_set_term_current);

int bq25890h_set_prechg_current(struct sy697x *sy, int curr)
{
	u8 iprechg;

	if (curr < SY697X_IPRECHG_BASE)
		curr = SY697X_IPRECHG_BASE;

	iprechg = (curr - SY697X_IPRECHG_BASE) / SY697X_IPRECHG_LSB;

	return bq25890h_update_bits(sy, SY697X_REG_05,
						SY697X_IPRECHG_MASK, iprechg << SY697X_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq25890h_set_prechg_current);

int bq25890h_set_chargevolt(struct sy697x *sy, int volt)
{
	u8 val;

	chg_debug("%s: volt = %d\n", __func__, volt);

	if (volt < SY697X_VREG_BASE)
		volt = SY697X_VREG_BASE;

	val = (volt - SY697X_VREG_BASE)/SY697X_VREG_LSB;
	return bq25890h_update_bits(sy, SY697X_REG_06,
						SY697X_VREG_MASK, val << SY697X_VREG_SHIFT);
}

int bq25890h_set_input_volt_limit(struct sy697x *sy, int volt)
{
	u8 val;

	chg_debug("%s: volt = %d\n", __func__, volt);

	if (volt < SY697X_VINDPM_BASE)
		volt = SY697X_VINDPM_BASE;

	val = (volt - SY697X_VINDPM_BASE) / SY697X_VINDPM_LSB;

	bq25890h_update_bits(sy, SY697X_REG_0D,
						SY697X_FORCE_VINDPM_MASK, SY697X_FORCE_VINDPM_ENABLE << SY697X_FORCE_VINDPM_SHIFT);

	return bq25890h_update_bits(sy, SY697X_REG_0D,
						SY697X_VINDPM_MASK, val << SY697X_VINDPM_SHIFT);
}

int bq25890h_set_input_current_limit(struct sy697x *sy, int curr)
{
	u8 val;

	chg_debug("%s: curr = %d\n", __func__, curr);

	if (curr < SY697X_IINLIM_BASE)
		curr = SY697X_IINLIM_BASE;

	val = (curr - SY697X_IINLIM_BASE) / SY697X_IINLIM_LSB;

	return bq25890h_update_bits(sy, SY697X_REG_00, SY697X_IINLIM_MASK,
						val << SY697X_IINLIM_SHIFT);
}

int bq25890h_get_input_current(void)
{
	u8 reg_val = 0;
	int icl = 0;
	int ret = 0;
	if(!g_sy)
		return icl;

	ret = bq25890h_read_byte(g_sy, SY697X_REG_00, &reg_val);
	if (!ret) {
		icl = (reg_val & SY697X_IINLIM_MASK) >> SY697X_IINLIM_SHIFT;
		icl = icl * SY697X_IINLIM_LSB + SY697X_IINLIM_BASE;
	}
	chg_debug("%s: icl = %d ma\n", __func__, icl);
	return icl;
}


int bq25890h_set_watchdog_timer(struct sy697x *sy, u8 timeout)
{
	u8 val;

	val = (timeout - SY697X_WDT_BASE) / SY697X_WDT_LSB;
	val <<= SY697X_WDT_SHIFT;

	return bq25890h_update_bits(sy, SY697X_REG_07,
						SY697X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq25890h_set_watchdog_timer);

int bq25890h_disable_watchdog_timer(struct sy697x *sy)
{
	u8 val = SY697X_WDT_DISABLE << SY697X_WDT_SHIFT;

	return bq25890h_update_bits(sy, SY697X_REG_07,
						SY697X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq25890h_disable_watchdog_timer);

int bq25890h_reset_watchdog_timer(struct sy697x *sy)
{
	u8 val = SY697X_WDT_RESET << SY697X_WDT_RESET_SHIFT;

	return bq25890h_update_bits(sy, SY697X_REG_03,
						SY697X_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq25890h_reset_watchdog_timer);


int bq25890h_force_dpdm(struct sy697x *sy, bool enable)
{
	int ret;
	u8 val;

	if (enable) {
		bq25890h_enable_enlim(sy);
		val = SY697X_AUTO_DPDM_ENABLE << SY697X_FORCE_DPDM_SHIFT;
	}
	else
		val = SY697X_AUTO_DPDM_DISABLE << SY697X_FORCE_DPDM_SHIFT;

	ret = bq25890h_update_bits(sy, SY697X_REG_02,
						SY697X_FORCE_DPDM_MASK, val);

	chg_err("Force DPDM %s, enable=%d\n", !ret ?  "successfully" : "failed", enable);

	return ret;
}
EXPORT_SYMBOL_GPL(bq25890h_force_dpdm);

int bq25890h_reset_chip(struct sy697x *sy)
{
	int ret;
	u8 val = SY697X_RESET << SY697X_RESET_SHIFT;

	ret = bq25890h_update_bits(sy, SY697X_REG_14,
						SY697X_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq25890h_reset_chip);

void bq25890h_suspend_by_hz_mode(bool en)
{
	u8 val;
	struct sy697x *sy = g_sy;
	if (!sy) {
		chg_err("%s sy is null[%d]\n", __func__, val);
		return;
	}
	if (en)
		val = SY697X_HIZ_ENABLE << SY697X_ENHIZ_SHIFT;
	else
		val = SY697X_HIZ_DISABLE << SY697X_ENHIZ_SHIFT;
	chg_err("%s val[%d]\n", __func__, val);
	bq25890h_update_bits(sy, SY697X_REG_00, SY697X_ENHIZ_MASK, val);
	return;
}

int bq25890h_enter_hiz_mode(struct sy697x *sy)
{
	u8 val;
	int boot_mode = get_boot_mode();
	chg_err("%s disable hiz_mode boot_mode[%d]\n", __func__, boot_mode);

	if (atomic_read(&sy->driver_suspended) == 1) {
		return 0;
	}

	atomic_set(&sy->charger_suspended, 1);

	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) {
		val = SY697X_HIZ_DISABLE << SY697X_ENHIZ_SHIFT;
		bq25890h_update_bits(sy, SY697X_REG_00, SY697X_ENHIZ_MASK, val);
		bq25890h_disable_charger(sy);
		bq25890h_set_input_current_limit(sy, 0);
	} else {
		sy->before_suspend_icl = bq25890h_get_input_current();
		bq25890h_set_input_current_limit(sy, 100);
		bq25890h_disable_charger(sy);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bq25890h_enter_hiz_mode);

int bq25890h_exit_hiz_mode(struct sy697x *sy)
{
	u8 val;
	int boot_mode = get_boot_mode();
	chg_err("%s boot_mode[%d]\n", __func__, boot_mode);

	if (atomic_read(&sy->driver_suspended) == 1) {
			return 0;
	}

	atomic_set(&sy->charger_suspended, 0);

	if (boot_mode ==  MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) {
		val = SY697X_HIZ_DISABLE << SY697X_ENHIZ_SHIFT;
		bq25890h_update_bits(sy, SY697X_REG_00, SY697X_ENHIZ_MASK, val);
	} else {
		sy->before_unsuspend_icl = bq25890h_get_input_current();
		bq25890h_enable_charger(sy);
		if ((sy->before_unsuspend_icl == 0)
				|| (sy->before_suspend_icl == 0)
				|| (sy->before_unsuspend_icl != 100)
				|| (sy->before_unsuspend_icl == sy->before_suspend_icl))  {
				chg_err("ignore set icl [%d %d]\n", sy->before_suspend_icl, sy->before_unsuspend_icl);
		} else {
			bq25890h_set_input_current_limit(sy, sy->before_suspend_icl);
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bq25890h_exit_hiz_mode);

int bq25890h_disable_enlim(struct sy697x *sy)
{
	u8 val = SY697X_ENILIM_DISABLE << SY697X_ENILIM_SHIFT;

	return bq25890h_update_bits(sy, SY697X_REG_00,
						SY697X_ENILIM_MASK, val);
}
EXPORT_SYMBOL_GPL(bq25890h_disable_enlim);

int bq25890h_enable_enlim(struct sy697x *sy)
{
	u8 val = SY697X_ENILIM_ENABLE << SY697X_ENILIM_SHIFT;

	return bq25890h_update_bits(sy, SY697X_REG_00,
						SY697X_ENILIM_MASK, val);
}
EXPORT_SYMBOL_GPL(bq25890h_enable_enlim);

int bq25890h_get_hiz_mode(struct sy697x *sy, u8 *state)
{
	u8 val;
	int ret;

	ret = bq25890h_read_byte(sy, SY697X_REG_00, &val);
	if (ret)
		return ret;
	*state = (val & SY697X_ENHIZ_MASK) >> SY697X_ENHIZ_SHIFT;

	chg_err("%s state[%d]\n", __func__, *state);

	return 0;
}
EXPORT_SYMBOL_GPL(bq25890h_get_hiz_mode);

static int bq25890h_enable_term(struct sy697x *sy, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = SY697X_TERM_ENABLE << SY697X_EN_TERM_SHIFT;
	else
		val = SY697X_TERM_DISABLE << SY697X_EN_TERM_SHIFT;

	ret = bq25890h_update_bits(sy, SY697X_REG_07,
						SY697X_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq25890h_enable_term);
#define CURR_LIM750 750
#define CURR_LIM1200 1200
#define CURR_LIM1400 1400
#define CURR_LIM1650 1650
#define CURR_LIM1870 1870
#define CURR_LIM2150 2150
#define CURR_LIM2450 2450
int bq25890h_set_boost_current(struct sy697x *sy, int curr)
{
	u8 temp = 0;
	if (curr < CURR_LIM750)
		temp = SY697X_BOOST_LIM_500MA;
	else if (curr < CURR_LIM1200)
		temp = SY697X_BOOST_LIM_750MA;
	else if (curr < CURR_LIM1400)
		temp = SY697X_BOOST_LIM_1200MA;
	else if (curr < CURR_LIM1650)
		temp = SY697X_BOOST_LIM_1400MA;
	else if (curr < CURR_LIM1870)
		temp = SY697X_BOOST_LIM_1650MA;
	else if (curr < CURR_LIM2150)
		temp = SY697X_BOOST_LIM_1875MA;
	else if (curr < CURR_LIM2450)
		temp = SY697X_BOOST_LIM_2150MA;
	else
		temp = SY697X_BOOST_LIM_2450MA;
	chg_err("bq25890h_set_boost_current temp=%d\n", temp);
	return bq25890h_update_bits(sy, SY697X_REG_0A,
				SY697X_BOOST_LIM_MASK,
				temp << SY697X_BOOST_LIM_SHIFT);
}

static int bq25890h_enable_auto_dpdm(struct sy697x *sy, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = SY697X_AUTO_DPDM_ENABLE << SY697X_AUTO_DPDM_EN_SHIFT;
	else
		val = SY697X_AUTO_DPDM_DISABLE << SY697X_AUTO_DPDM_EN_SHIFT;

	ret = bq25890h_update_bits(sy, SY697X_REG_02,
						SY697X_AUTO_DPDM_EN_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq25890h_enable_auto_dpdm);

int bq25890h_set_boost_voltage(struct sy697x *sy, int volt)
{
	u8 val = 0;

	if (volt < SY697X_BOOSTV_BASE)
		volt = SY697X_BOOSTV_BASE;
	if (volt > SY697X_BOOSTV_BASE
			+ (SY697X_BOOSTV_MASK >> SY697X_BOOSTV_SHIFT)
			* SY697X_BOOSTV_LSB)
		volt = SY697X_BOOSTV_BASE
			+ (SY697X_BOOSTV_MASK >> SY697X_BOOSTV_SHIFT)
			* SY697X_BOOSTV_LSB;

	val = ((volt - SY697X_BOOSTV_BASE) / SY697X_BOOSTV_LSB)
			<< SY697X_BOOSTV_SHIFT;

	chg_err("bq25890h_set_boost_voltage val=%d\n", val);
	return bq25890h_update_bits(sy, SY697X_REG_0A,
				SY697X_BOOSTV_MASK, val);
}
EXPORT_SYMBOL_GPL(bq25890h_set_boost_voltage);


static int bq25890h_read_idpm_limit(struct sy697x *sy, int *icl)
{
	uint8_t val;
	int ret;

	ret = bq25890h_read_byte(sy, SY697X_REG_13, &val);
	if (ret < 0) {
		chg_err("read vbus voltage failed :%d\n", ret);
		return ret;
	} else {
		*icl = SY697X_IDPM_LIM_BASE + ((val & SY697X_IDPM_LIM_MASK) >> SY697X_IDPM_LIM_SHIFT) * SY697X_IDPM_LIM_LSB;
		return 0;
	}
}
EXPORT_SYMBOL_GPL(bq25890h_read_idpm_limit);

static int bq25890h_enable_safety_timer(struct sy697x *sy)
{
	const u8 val = SY697X_CHG_TIMER_ENABLE << SY697X_EN_TIMER_SHIFT;

	return bq25890h_update_bits(sy, SY697X_REG_07, SY697X_EN_TIMER_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(bq25890h_enable_safety_timer);

static int bq25890h_disable_safety_timer(struct sy697x *sy)
{
	const u8 val = SY697X_CHG_TIMER_DISABLE << SY697X_EN_TIMER_SHIFT;

	return bq25890h_update_bits(sy, SY697X_REG_07, SY697X_EN_TIMER_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(bq25890h_disable_safety_timer);

static int bq25890h_switch_to_hvdcp(struct sy697x *sy, enum hvdcp_type type)
{
	int ret;
	u8 val;
	u8 mask;

	switch (type) {
	case HVDCP_5V:
		val = (SY697X_DP_0P6V << SY697X_DPDAC_SHIFT) | (SY697X_DM_0V << SY697X_DMDAC_SHIFT);
		chg_err("set to 5v\n");
		break;

	case HVDCP_9V:
		val = (SY697X_DP_3P3V << SY697X_DPDAC_SHIFT) | (SY697X_DM_0P6V << SY697X_DMDAC_SHIFT);
		chg_err("set to 9v\n");
		break;

	case HVDCP_DPF_DMF:
		val = (SY697X_DP_HIZ << SY697X_DPDAC_SHIFT) | (SY697X_DM_HIZ << SY697X_DMDAC_SHIFT);
		chg_err("set to hiz\n");
		break;
	default:
		break;
	}

	mask = SY697X_DPDAC_MASK | SY697X_DMDAC_MASK;
	ret = bq25890h_update_bits(sy, SY697X_REG_01, mask, val);

	return ret;
}

static int bq25890h_check_charge_done(struct sy697x *sy, bool *done)
{
	int ret;
	u8 val;

	ret = bq25890h_read_byte(sy, SY697X_REG_0B, &val);
	if (!ret) {
		val = val & SY697X_CHRG_STAT_MASK;
		val = val >> SY697X_CHRG_STAT_SHIFT;
		*done = (val == SY697X_CHRG_STAT_CHGDONE);
	}

	return ret;
}

static struct sy697x_platform_data *bq25890h_parse_dt(struct device_node *np,
						      struct sy697x *sy)
{
	int ret;
	struct sy697x_platform_data *pdata;

	pdata = devm_kzalloc(sy->dev, sizeof(struct sy697x_platform_data), GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (of_property_read_string(np, "charger_name", &sy->chg_dev_name) < 0) {
		sy->chg_dev_name = "primary_chg";
		chg_err("no charger name\n");
	}

	if (of_property_read_string(np, "eint_name", &sy->eint_name) < 0) {
		sy->eint_name = "chr_stat";
		chg_err("no eint name\n");
	}

	sy->chg_det_enable =
		of_property_read_bool(np, "sy,bq25890h,charge-detect-enable");

	ret = of_property_read_u32(np, "sy,bq25890h,usb-vlim", &pdata->usb.vlim);
	if (ret) {
		pdata->usb.vlim = SY697X_USB_VLIM;
		chg_err("Failed to read node of sy,bq25890h,usb-vlim\n");
	}

	ret = of_property_read_u32(np, "sy,bq25890h,usb-ilim", &pdata->usb.ilim);
	if (ret) {
		pdata->usb.ilim = SY697X_USB_ILIM;
		chg_err("Failed to read node of sy,bq25890h,usb-ilim\n");
	}

	ret = of_property_read_u32(np, "sy,bq25890h,usb-vreg", &pdata->usb.vreg);
	if (ret) {
		pdata->usb.vreg = SY697X_USB_VREG;
		chg_err("Failed to read node of sy,bq25890h,usb-vreg\n");
	}

	ret = of_property_read_u32(np, "sy,bq25890h,usb-ichg", &pdata->usb.ichg);
	if (ret) {
		pdata->usb.ichg = SY697X_CHG_ICHG;
		chg_err("Failed to read node of sy,bq25890h,usb-ichg\n");
	}

	ret = of_property_read_u32(np, "sy,bq25890h,precharge-current",
				   &pdata->iprechg);
	if (ret) {
		pdata->iprechg = SY697X_CHG_PRCHG;
		chg_err("Failed to read node of sy,bq25890h,precharge-current\n");
	}

	ret = of_property_read_u32(np, "sy,bq25890h,termination-current",
				   &pdata->iterm);
	if (ret) {
		pdata->iterm = SY697X_CHG_TERM;
		chg_err("Failed to read node of sy,bq25890h,termination-current\n");
	}

	ret =
	    of_property_read_u32(np, "sy,bq25890h,boost-voltage",
				 &pdata->boostv);
	if (ret) {
		pdata->boostv = SY697X_OTG_BOOTSTV;
		chg_err("Failed to read node of sy,bq25890h,boost-voltage\n");
	}

	ret =
		of_property_read_u32(np, "sy,bq25890h,boost-current",
				 &pdata->boosti);
	if (ret) {
		pdata->boosti = SY697X_OTG_BOOTSTI;
		chg_err("Failed to read node of sy,bq25890h,boost-current\n");
	}

	return pdata;
}

static void bq25890h_check_hvdcp_type(struct sy697x *sy)
{
	int ret;
	u8 reg_val = 0;
	int vbus_stat = 0;

	ret = bq25890h_read_byte(sy, SY697X_REG_0B, &reg_val);
	if (ret)
		return;

	vbus_stat = (reg_val & SY697X_VBUS_STAT_MASK);
	vbus_stat >>= SY697X_VBUS_STAT_SHIFT;
	sy->vbus_type = vbus_stat;
	chg_err("bq25890h_get_charger_type:%d, reg0B = 0x%x\n", vbus_stat, reg_val);
	switch (vbus_stat) {
	case SY697X_VBUS_TYPE_HVDCP:
		if (!disable_QC) {
			sy->hvdcp_can_enabled = true;
		}
		break;
	default:
		break;
	}
}

static int opluschg_updata_usb_type(struct sy697x *sy)
{
	union power_supply_propval propval;
	int ret = 0;
	struct oplus_chg_chip *chgchip = g_oplus_chg;

	if (!sy) {
		chg_err("[%s] sy is null\n", __func__);
		return 0;
	}

	if (sy->power_good && (oplus_get_usb_status() != USB_TEMP_HIGH)
		&& ((sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB) || (sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB_CDP)))
		propval.intval = 1;
	else
		propval.intval = 0;

	chg_err("[%s] POWER_SUPPLY_PROP_ONLINE %d\n", __func__, propval.intval);
	ret = power_supply_set_property(chgchip->usb_psy, POWER_SUPPLY_PROP_ONLINE,
					&propval);

	propval.intval = sy->oplus_chg_type;
	chg_err("[%s] POWER_SUPPLY_PROP_TYPE %d\n", __func__, propval.intval);
	ret = power_supply_set_property(chgchip->usb_psy, POWER_SUPPLY_PROP_TYPE,
					&propval);
	if (ret < 0)
		chg_debug("inform power supply charge type failed:%d\n", ret);

	power_supply_changed(chgchip->usb_psy);
	chg_err("[%s] power_supply_changed POWER_SUPPLY_TYPE_USB done\n", __func__);
	return 1000;
}

static int bq25890h_inform_charger_type(struct sy697x *sy)
{
	int ret = 0;

	if (charger_psy)
		power_supply_changed(charger_psy);
	chg_debug("[%s] done\n", __func__);
	return ret;
}

static int sy_charger_type_recheck(struct sy697x *sy)
{
	int ret;

	u8 reg_val = 0;
	int vbus_stat = 0;

	if (!g_oplus_chg) {
		return 0;
	}

	ret = bq25890h_read_byte(sy, SY697X_REG_0B, &reg_val);
	if (ret)
		return ret;

	vbus_stat = (reg_val & SY697X_VBUS_STAT_MASK);
	vbus_stat >>= SY697X_VBUS_STAT_SHIFT;
	sy->vbus_type = vbus_stat;

	switch (vbus_stat) {
	case SY697X_VBUS_TYPE_NONE:
		sy->chg_type = CHARGER_UNKNOWN;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case SY697X_VBUS_TYPE_SDP:
		sy->chg_type = STANDARD_HOST;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB;
		/*g_oplus_chg->charger_type = POWER_SUPPLY_TYPE_USB;*/
		break;
	case SY697X_VBUS_TYPE_CDP:
		sy->chg_type = CHARGING_HOST;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		chg_err("%s g_oplus_chg->charger_type=%d\n", __func__, g_oplus_chg->charger_type);
	/*	if (g_oplus_chg->charger_type != POWER_SUPPLY_TYPE_USB_CDP) {
			sy->cdp_retry_aicl = true;
			g_oplus_chg->charger_type = POWER_SUPPLY_TYPE_USB_CDP;
		}*/
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
	chg_err("[%s]:chg_type = %d, %d, %d vbus_on[%d]\n", __func__, sy->chg_type, sy->oplus_chg_type, g_oplus_chg->charger_type, sy->vbus_on);
	if ((g_sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB)
			|| (g_sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB_CDP)) {
		oplus_notify_device_mode(true);
	}

	bq25890h_inform_charger_type(sy);
	opluschg_updata_usb_type(sy);
	bq25890h_adc_start(sy, true);
	oplus_chg_wake_update_work();
	chg_err("oplus_chg_wake_update_work done\n");
	schedule_delayed_work(&sy->sy697x_aicl_work, OPLUS_DELAY_WORK_TIME_BASE*2);

	return 0;
}

static void oplus_set_typec_sinkonly()
{
	if (g_sy != NULL && g_sy->tcpc != NULL) {
		chg_err("[OPLUS_CHG][%s]\n", __func__);
		tcpm_typec_change_role(g_sy->tcpc, TYPEC_ROLE_SNK);
	}
};

static void oplus_set_typec_cc_open()
{
	if (g_sy != NULL && g_sy->tcpc != NULL) {
		chg_err("[OPLUS_CHG][%s]\n", __func__);
		tcpm_typec_disable_function(g_sy->tcpc, true);
	}
};

static void oplus_usbtemp_recover_cc_open(void)
{
        if (g_sy != NULL && g_sy->tcpc != NULL) {
                tcpm_typec_disable_function(g_sy->tcpc, false);
                chg_err("[OPLUS_CHG][%s]\n", __func__);
        }
}

static bool oplus_usbtemp_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		chg_err("[OPLUS_CHG][%s]: smb5_chg not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.dischg_gpio))
		return true;

	return false;
}

static bool oplus_usbtemp_check_is_support(void)
{
	if (oplus_usbtemp_check_is_gpio(g_oplus_chg) == true)
		return true;

	chg_err("dischg return false\n");

	return false;
}

static bool oplus_usbtemp_condition(void)
{
	int level = -1;
	struct sy697x *chg = g_sy;
	if (!g_oplus_chg) {
		chg_err("fail to init oplus_chip\n");
		return false;
	}
	if (oplus_ccdetect_check_is_gpio(chg)) {
		level = gpio_get_value(chg->ccdetect_gpio);
		if (1 != level) {
			chg_err("check_chrdet_status ccdetect is %d\n", level);
			return true;
		}
	}
	g_oplus_chg->usbtemp_check = g_oplus_chg->chg_ops->check_chrdet_status();
	chg_err("check_chrdet_status is %d\n", g_oplus_chg->usbtemp_check);
	return g_oplus_chg->usbtemp_check;
}

static void oplus_wake_up_usbtemp_thread(void)
{
	struct oplus_chg_chip *chip = g_oplus_chg;

	if (!chip) {
		return;
	}
	if (oplus_usbtemp_check_is_support() == true) {
		chip->usbtemp_check = oplus_usbtemp_condition();
		chg_err("opluschip->usbtemp_check=%d \n", chip->usbtemp_check);
		if (chip->usbtemp_check)
			wake_up_interruptible(&chip->oplus_usbtemp_wq);
	}
}

static void opluschg_usbtemp_thread_init(struct oplus_chg_chip *oplus_chip)
{
	if (oplus_chip == NULL) {
		chg_err("failed to cread oplushg_usbtemp_kthread, oplus_chip == NULL\n");
		return;
	}
	if (oplus_usbtemp_check_is_support() == true) {
		oplushg_usbtemp_kthread = kthread_run(oplus_usbtemp_monitor_common, oplus_chip, "usbtemp_kthread");
		if (IS_ERR(oplushg_usbtemp_kthread)) {
			chg_err("failed to cread oplushg_usbtemp_kthread\n");
		}
	}
}

bool bq25890h_vbus_good(struct sy697x *sy)
{
	u8 reg_val = 0;
	int ret = 0;
	if (!sy) {
		return false;
	}

	ret = bq25890h_read_byte(sy, SY697X_REG_11, &reg_val);
	if (ret)
		return ret;
	if (reg_val & SY697X_VBUS_GD_MASK)
		return true;
	else
		return false;
}

static int oplus_splitchg_request_dpdm(struct sy697x *chg, bool enable)
{
	int rc = 0;

	if (!chg) {
		chg_err("[%s] chg is null\n", __func__);
		return 0;
	}

	/* fetch the DPDM regulator */
	chg_err("[%s] start enable[%d %d]\n", __func__, enable, chg->dpdm_enabled);
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
			chg_debug("golf enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
			if (rc < 0)
				chg_err("Couldn't enable dpdm regulator rc=%d\n", rc);
			else {
				chg->dpdm_enabled = true;
				chg_debug("enabling DPDM success\n");
			}
		}
	} else {
		if (chg->dpdm_reg && chg->dpdm_enabled) {
			chg_debug("golf disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
			if (rc < 0)
				chg_err("Couldn't disable dpdm regulator rc=%d\n", rc);
			else {
				chg->dpdm_enabled = false;
				chg_debug("disabling DPDM success\n");
			}
		}
	}
	mutex_unlock(&chg->dpdm_lock);
	chg_err("[%s] done\n", __func__);

	return rc;
}

void splitchg_request_dpdm(bool enable)
{
	oplus_splitchg_request_dpdm(g_sy, enable);
}

static irqreturn_t bq25890h_irq_handler(int irq, void *data)
{
	struct sy697x *sy = (struct sy697x *)data;
	int ret;
	u8 reg_val;
	u8 hz_mode = 0;
	bool prev_pg, curr_pg;
	bool bus_gd;
	struct oplus_chg_chip *chip = g_oplus_chg;

	chg_debug("bq25890h_irq_handler:enter\n");
	if (chip == NULL)
		return IRQ_HANDLED;

	if (sy->is_force_dpdm) {
		sy->is_force_dpdm = false;
		bq25890h_force_dpdm(sy, false);
		chg_debug("bq25890h_force_dpdm:false\n");
	}

	if (irq == PROBE_PLUG_IN_IRQ) {
		sy->vbus_on = false;
		sy->vbus_on = bq25890h_vbus_good(sy);
		chg_err("bq25890h vbus_on[%d]\n", sy->vbus_on);
	}

	ret = bq25890h_read_byte(sy, SY697X_REG_0B, &reg_val);
	if (ret)
		return IRQ_HANDLED;

	prev_pg = sy->power_good;
	curr_pg = !!(reg_val & SY697X_PG_STAT_MASK);
	bus_gd = bq25890h_vbus_good(sy);
	sy->power_good = curr_pg;
	chg_debug("[%s]:normal chg...(%d,%d, %d, %d, %d, otg[%d])\n", __func__,
			prev_pg, sy->power_good, curr_pg, bus_gd, reg_val, oplus_get_otg_enable());

	chg_debug("[%s]:(%d,%d %d, otg[%d])\n", __func__,
		prev_pg, sy->power_good, reg_val, oplus_get_otg_enable());
	if (!prev_pg && sy->power_good) {
		sy->is_force_aicl = true;
		Charger_Detect_Init();
		chg_debug("is_prswap:%d \n", oplus_is_prswap);
		if (oplus_is_prswap) {
			sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
			bq25890h_inform_charger_type(sy);
			oplus_is_prswap = false;
			return IRQ_HANDLED;
		} else {
			oplus_splitchg_request_dpdm(sy, true);
		}
		chg_err("adapter/usb inserted\n");
		oplus_for_cdp();
		bq25890h_force_dpdm(sy, true);
		get_monotonic_boottime(&sy->st_ptime[0]);
		sy->chg_need_check = true;
		sy->chg_start_check = false;
		wake_up_interruptible(&oplus_chgtype_wq);
		goto POWER_CHANGE;
	} else if (prev_pg && !sy->power_good) {
		ret = bq25890h_get_hiz_mode(sy, &hz_mode);
		if (!ret && hz_mode) {
			chg_debug("hiz mode ignore\n");
			goto POWER_CHANGE;
		}
		sy->is_force_aicl = false;
		sy->pre_current_ma = -1;
		sy->usb_connect_start = false;
		sy->hvdcp_can_enabled = false;
		sy->hvdcp_checked = false;
		sy->sdp_retry = false;
		sy->cdp_retry = false;
		sy->chg_type = CHARGER_UNKNOWN;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		memset(&sy->st_ptime[0], 0, sizeof(struct timespec));
		memset(&sy->st_ptime[1], 0, sizeof(struct timespec));
		if (chip) {
			chip->pd_chging = false;
		}
		if (chip->is_double_charger_support) {
			chip->sub_chg_ops->charging_disable();
		}
		bq25890h_inform_charger_type(sy);
		opluschg_updata_usb_type(sy);

		bq25890h_adc_start(sy, false);
		chg_debug("adapter/usb pg_good removed\n");

		bus_gd = bq25890h_vbus_good(sy);
		chip->usbtemp_check = false;
		bq25890h_switch_to_hvdcp(sy, HVDCP_DPF_DMF);
		oplus_splitchg_request_dpdm(sy, false);
		oplus_notify_device_mode(false);
		sy->chg_need_check = true;
		wake_up_interruptible(&oplus_chgtype_wq);
		goto POWER_CHANGE;
	} else if (!prev_pg && !sy->power_good) {
		chg_debug("prev_pg & now_pg is false\n");
		goto POWER_CHANGE;
	}

	if (g_sy->otg_enable)
		return IRQ_HANDLED;

	if (SY697X_VBUS_STAT_MASK & reg_val) {
		sy->is_force_aicl = false;
		bq25890h_check_hvdcp_type(sy);
		if (sy->usb_connect_start == true) {
			switch ((SY697X_VBUS_STAT_MASK & reg_val)>> SY697X_VBUS_STAT_SHIFT) {
			case SY697X_VBUS_TYPE_SDP:
				sy->chg_type = STANDARD_HOST;
				sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB;
				break;
			case SY697X_VBUS_TYPE_CDP:
				sy->chg_type = CHARGING_HOST;
				sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_CDP;
				break;
			}
		}
		if (sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB ||
				sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB_CDP) {
			chg_debug(" type usb %d\n", sy->usb_connect_start);
			if (sy->usb_connect_start == true) {
				Charger_Detect_Release();
				bq25890h_inform_charger_type(sy);
				opluschg_updata_usb_type(sy);
			}
		} else if (sy->oplus_chg_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			chg_debug(" type cdp or dcp type=%d\n", sy->oplus_chg_type);
			Charger_Detect_Release();
			if (sy->oplus_chg_type != chip->charger_type) {
				chg_debug(" recheck charger type = %d pre=%d\n", sy->oplus_chg_type, chip->charger_type);
				sy->pdqc_setup_5v = false;
				sy->is_bc12_end = false;
				schedule_delayed_work(&sy->sy697x_vol_convert_work, OPLUS_BC12_RETRY_TIME);
			}
		}

		if (sy->chg_need_check && sy->chg_start_check == false) {
			chg_debug("[%s 1]:chg_type = %d, %d, %d\n", __func__, sy->chg_type, sy->oplus_chg_type, g_oplus_chg->charger_type);
			sy_charger_type_recheck(sy);
			chg_debug("[%s 2]:chg_type = %d, %d, %d\n", __func__, sy->chg_type, sy->oplus_chg_type, g_oplus_chg->charger_type);
			Charger_Detect_Release();
			chg_debug("charge type check thread is hung\n");
		}
	}

POWER_CHANGE:
	if (dumpreg_by_irq)
		bq25890h_dump_regs(sy);
	return IRQ_HANDLED;
}

int sy6970_plugout_softirq(int plug)
{
	if (g_sy) {
		bq25890h_irq_handler(plug, g_sy);
	} else {
		chg_err("[%s] chip is null\n", __func__);
	}
	return  0;
}

static oplus_chgirq_gpio_init(struct sy697x *sy)
{
	int rc;
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
		chg_err("sy->irq_gpio =%d\n", sy->irq_gpio);
	}

	sy->irq = gpio_to_irq(sy->irq_gpio);
	chg_err("irq way1 sy->irq =%d\n", sy->irq);

	sy->irq = irq_of_parse_and_map(node, 0);
	chg_err("irq way2 sy->irq =%d\n", sy->irq);

	/* set splitchg pinctrl*/
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
	chg_err("irq sy->irq_gpio input =%d irq_gpio_stat = %d\n", sy->irq_gpio, rc);

	return 0;
}


static int bq25890h_register_interrupt(struct device_node *np, struct sy697x *sy)
{
	int ret = 0;

#ifdef CONFIG_OPLUS_CHARGER_MTK
	sy->irq = irq_of_parse_and_map(np, 0);
#else
	oplus_chgirq_gpio_init(sy);
#endif
	chg_err("irq = %d\n", sy->irq);

	ret = devm_request_threaded_irq(sy->dev, sy->irq, NULL,
					bq25890h_irq_handler,
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

	bq25890h_disable_watchdog_timer(sy);
	sy->is_force_dpdm = false;
	ret = bq25890h_set_prechg_current(sy, sy->platform_data->iprechg);
	if (ret)
		chg_err("Failed to set prechg current, ret = %d\n", ret);

	ret = bq25890h_set_term_current(sy, sy->platform_data->iterm);
	if (ret)
		chg_err("Failed to set termination current, ret = %d\n", ret);

	ret = bq25890h_set_boost_voltage(sy, sy->platform_data->boostv);
	if (ret)
		chg_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = bq25890h_set_boost_current(sy, sy->platform_data->boosti);
	if (ret)
		chg_err("Failed to set boost current, ret = %d\n", ret);

	ret = bq25890h_enable_auto_dpdm(sy, false);
	if (ret)
		chg_err("Failed to stop auto dpdm, ret = %d\n", ret);

	ret = bq25890h_vmin_limit(sy);
	if (ret)
		chg_err("Failed to set vmin limit, ret = %d\n", ret);

	ret = bq25890h_adc_start(sy, true);
	if (ret)
		chg_err("Failed to stop adc, ret = %d\n", ret);

	ret = bq25890h_set_input_volt_limit(sy, SY697X_VLOT_LIMIT);
	if (ret)
		chg_err("Failed to set input volt limit, ret = %d\n", ret);

	sy->boot_mode = get_boot_mode();
	if (sy->boot_mode == MSM_BOOT_MODE__RF || sy->boot_mode == MSM_BOOT_MODE__WLAN) {
		bq25890h_enter_hiz_mode(sy);
	}

	return 0;
}


static int bq25890h_init_device(struct sy697x *sy)
{
	int ret;

	bq25890h_disable_watchdog_timer(sy);
	sy->is_force_dpdm = false;
	ret = bq25890h_set_prechg_current(sy, sy->platform_data->iprechg);
	if (ret)
		chg_err("Failed to set prechg current, ret = %d\n", ret);

	ret = bq25890h_set_chargevolt(sy, DEFAULT_CV);
	if (ret)
		chg_err("Failed to set default cv, ret = %d\n", ret);

	ret = bq25890h_set_term_current(sy, sy->platform_data->iterm);
	if (ret)
		chg_err("Failed to set termination current, ret = %d\n", ret);

	ret = bq25890h_set_boost_voltage(sy, sy->platform_data->boostv);
	if (ret)
		chg_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = bq25890h_set_boost_current(sy, sy->platform_data->boosti);
	if (ret)
		chg_err("Failed to set boost current, ret = %d\n", ret);
	ret = bq25890h_disable_enlim(sy);
	if (ret)
		chg_err("Failed to bq25890h_disable_enlim, ret = %d\n", ret);
	ret = bq25890h_enable_auto_dpdm(sy, false);
	if (ret)
		chg_err("Failed to stop auto dpdm, ret = %d\n", ret);

	ret = bq25890h_vmin_limit(sy);
	if (ret)
		chg_err("Failed to set vmin limit, ret = %d\n", ret);

	ret = bq25890h_adc_start(sy, true);
	if (ret)
		chg_err("Failed to stop adc, ret = %d\n", ret);

	ret = bq25890h_set_input_volt_limit(sy, SY697X_VLOT_LIMIT);
	if (ret)
		chg_err("Failed to set input volt limit, ret = %d\n", ret);

	sy->boot_mode = get_boot_mode();
	if (sy->boot_mode == MSM_BOOT_MODE__RF || sy->boot_mode == MSM_BOOT_MODE__WLAN) {
		bq25890h_enter_hiz_mode(sy);
	}
	return 0;
}

static int bq25890h_detect_device(struct sy697x *sy)
{
	int ret;
	u8 data;

	ret = bq25890h_read_byte(sy, SY697X_REG_14, &data);
	if (!ret) {
		sy->part_no = (data & SY697X_PN_MASK) >> SY697X_PN_SHIFT;
		sy->revision =
		    (data & SY697X_DEV_REV_MASK) >> SY697X_DEV_REV_SHIFT;
	}
	chg_debug("part_no=%d,revision=%d\n", sy->part_no, sy->revision);
	if (is_bq25890h(sy) == false) {
		chg_debug("bq25890h\n");
	} else {
		chg_debug("bq25890h\n");
	}

	return 0;
}
#define DUMP_DELAY 1
static void bq25890h_dump_regs(struct sy697x *sy)
{
	int addr;
	u8 val[SY697X_VALU_NUM];
	int ret;
	char buf[SY697X_BUFF_NUM];
	char *s = buf;

	for (addr = SY697X_REG_00; addr <= SY697X_REG_14; addr++) {
		ret = bq25890h_read_byte(sy, addr, &val[addr]);
		msleep(DUMP_DELAY);
	}

	s += sprintf(s, "bq25890h_dump_regs:");
	for (addr = SY697X_REG_00; addr <= SY697X_REG_14; addr++) {
		s += sprintf(s,"[0x%.2x,0x%.2x]", addr, val[addr]);
	}
	s += sprintf(s,"\n");

	chg_err("%s", buf);
}

bool is_bq25890h(struct sy697x *sy)
{
	if (sy->part_no == 1 && sy->revision == 0) /*chip is sy6970*/
		return false;
	else /*chip is bq25890h*/
		return true;
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
static ssize_t
bq25890h_show_registers(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct sy697x *sy = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[SY697X_BUFF_NUM];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq25890h Reg");
	for (addr = SY697X_REG_00; addr <= SY697X_REG_14; addr++) {
		ret = bq25890h_read_byte(sy, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				       "Reg[%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t
bq25890h_store_registers(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < SY697X_REG_14) {
		/*bq25890h_write_byte(sy, (unsigned char) reg,
				   (unsigned char) val);*/
	}

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, bq25890h_show_registers,
		   bq25890h_store_registers);

static struct attribute *bq25890h_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq25890h_attr_group = {
	.attrs = bq25890h_attributes,
};

static int bq25890h_charging(struct charger_device *chg_dev, bool enable)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;

	if (enable)
		ret = bq25890h_enable_charger(sy);
	else
		ret = bq25890h_disable_charger(sy);

	chg_err("%s charger %s\n", enable ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	ret = bq25890h_read_byte(sy, SY697X_REG_03, &val);

	if (!ret)
		sy->charge_enabled = !!(val & SY697X_CHG_CONFIG_MASK);

	return ret;
}

static int bq25890h_plug_in(struct charger_device *chg_dev)
{
	int ret;

	ret = bq25890h_charging(chg_dev, true);

	if (ret)
		chg_err("Failed to enable charging:%d\n", ret);

	return ret;
}

static int bq25890h_plug_out(struct charger_device *chg_dev)
{
	int ret;

	ret = bq25890h_charging(chg_dev, false);

	if (ret)
		chg_err("Failed to disable charging:%d\n", ret);

	return ret;
}

static int bq25890h_dump_register(struct charger_device *chg_dev)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);

	bq25890h_dump_regs(sy);
	if (g_oplus_chg->sub_chg_ops)
		g_oplus_chg->sub_chg_ops->dump_register();

	return 0;
}

static int bq25890h_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);

	*en = sy->charge_enabled;

	return 0;
}

static int bq25890h_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);
	int ret;

	ret = bq25890h_check_charge_done(sy, done);

	return ret;
}

static int bq25890h_set_ichg(struct charger_device *chg_dev, u32 curr)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);

	chg_err("charge curr = %d\n", curr);

	return bq25890h_set_chargecurrent(sy, curr / SY697X_ICH_1000);
}
#endif /*CONFIG_OPLUS_CHARGER_MTK*/

static int _bq25890h_get_ichg(struct sy697x *sy, u32 *curr)
{
	u8 reg_val;
	int ichg;
	int ret;

	ret = bq25890h_read_byte(sy, SY697X_REG_04, &reg_val);
	if (!ret) {
		ichg = (reg_val & SY697X_ICHG_MASK) >> SY697X_ICHG_SHIFT;
		ichg = ichg * SY697X_ICHG_LSB + SY697X_ICHG_BASE;
		*curr = ichg * SY697X_ICH_1000;
	}

	return ret;
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
static int bq25890h_get_ichg(struct charger_device *chg_dev, u32 *curr)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int ichg;
	int ret;

	ret = bq25890h_read_byte(sy, SY697X_REG_04, &reg_val);
	if (!ret) {
		ichg = (reg_val & SY697X_ICHG_MASK) >> SY697X_ICHG_SHIFT;
		ichg = ichg * SY697X_ICHG_LSB + SY697X_ICHG_BASE;
		*curr = ichg * SY697X_ICH_1000;
	}

	return ret;
}

static int bq25890h_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	*curr = 60 * SY697X_ICH_1000;

	return 0;
}

static int bq25890h_set_vchg(struct charger_device *chg_dev, u32 volt)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);

	chg_err("charge volt = %d\n", volt);

	return bq25890h_set_chargevolt(sy, volt / SY697X_ICH_1000);
}

static int bq25890h_get_vchg(struct charger_device *chg_dev, u32 *volt)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int vchg;
	int ret;

	ret = bq25890h_read_byte(sy, SY697X_REG_06, &reg_val);
	if (!ret) {
		vchg = (reg_val & SY697X_VREG_MASK) >> SY697X_VREG_SHIFT;
		vchg = vchg * SY697X_VREG_LSB + SY697X_VREG_BASE;
		*volt = vchg * SY697X_ICH_1000;
	}

	return ret;
}

static int bq25890h_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);

	chg_err("vindpm volt = %d\n", volt);

	return bq25890h_set_input_volt_limit(sy, volt / SY697X_ICH_1000);
}

static int bq25890h_set_icl(struct charger_device *chg_dev, u32 curr)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);

	chg_err("indpm curr = %d\n", curr);
	return bq25890h_set_input_current_limit(sy, curr / SY697X_ICH_1000);
}

static int bq25890h_get_icl(struct charger_device *chg_dev, u32 *curr)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int icl;
	int ret;

	ret = bq25890h_read_byte(sy, SY697X_REG_00, &reg_val);
	if (!ret) {
		icl = (reg_val & SY697X_IINLIM_MASK) >> SY697X_IINLIM_SHIFT;
		icl = icl * SY697X_IINLIM_LSB + SY697X_IINLIM_BASE;
		*curr = icl * SY697X_ICH_1000;
	}

	return ret;
}

static int bq25890h_kick_wdt(struct charger_device *chg_dev)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);

	return bq25890h_reset_watchdog_timer(sy);
}

static int bq25890h_set_otg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);

	if (en)
		ret = bq25890h_enable_otg(sy);
	else
		ret = bq25890h_disable_otg(sy);

	if (!ret)
		sy->otg_enable = en;

	chg_err("%s OTG %s\n", en ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	return ret;
}

static int bq25890h_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);
	int ret;

	if (en)
		ret = bq25890h_enable_safety_timer(sy);
	else
		ret = bq25890h_disable_safety_timer(sy);

	return ret;
}

static int bq25890h_is_safety_timer_enabled(struct charger_device *chg_dev,
					   bool *en)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = bq25890h_read_byte(sy, SY697X_REG_07, &reg_val);

	if (!ret)
		*en = !!(reg_val & SY697X_EN_TIMER_MASK);

	return ret;
}

static int bq25890h_set_boost_ilmt(struct charger_device *chg_dev, u32 curr)
{
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);
	int ret;

	chg_err("otg curr = %d\n", curr);

	ret = bq25890h_set_boost_current(sy, curr / SY697X_ICH_1000);

	return ret;
}

static int bq25890h_enable_chgdet(struct charger_device *chg_dev, bool en)
{
	int ret;
	u8 val;
	struct sy697x *sy = dev_get_drvdata(&chg_dev->dev);
	struct oplus_chg_chip *chip = g_oplus_chg;
	if (chip == NULL)
		return -1;

	chg_debug("bq25890h_enable_chgdet:%d\n", en);
	sy->chg_det_enable = en;
	if (!en) {
		sy->pre_current_ma = -1;
		sy->hvdcp_can_enabled = false;
		sy->hvdcp_checked = false;
		sy->sdp_retry = false;
		sy->cdp_retry = false;
		sy->usb_connect_start = false;
		Charger_Detect_Release();
		sy->chg_type = CHARGER_UNKNOWN;
		sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		if (chip->is_double_charger_support) {
			chip->sub_chg_ops->charging_disable();
		}

		if (sy->power_good && battery_get_vbus() < SY697X_CHG_VBUS) {
			chg_debug("vbus off but pg is good\n");
			sy->power_good = false;
		}

		bq25890h_inform_charger_type(sy);
		opluschg_updata_usb_type(sy);
		bq25890h_adc_start(sy, false);
	}

	return ret;
}
#endif /*CONFIG_OPLUS_CHARGER_MTK*/

static int bq25890h_enter_ship_mode(struct sy697x *sy, bool en)
{
	int ret;
	u8 val;

	if (en) {
		ret = bq25890h_update_bits(sy, SY697X_REG_09,
				SY697X_DEFLEAT_SHIP_MODE, SY697X_DEFLEAT_SHIP_MODE);
	} else {
		val = SY697X_BATFET_ON;
		val <<= SY697X_BATFET_DIS_SHIFT;
		ret = bq25890h_update_bits(sy, SY697X_REG_09,
						SY697X_BATFET_DIS_MASK, val);
	}
	return ret;
}

static int bq25890h_enable_shipmode(bool en)
{
	int ret;

	ret = bq25890h_enter_ship_mode(g_sy, en);

	return 0;
}

static bool oplus_check_chrdet_status(void);

int oplus_bq25890h_get_usb_status(void)
{
	if (g_oplus_chg && g_oplus_chg->usb_status == USB_TEMP_HIGH)
		return g_oplus_chg->usb_status;

	return NUM_0;
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
static struct charger_ops bq25890h_chg_ops = {
	/* Normal charging */
	.plug_in = bq25890h_plug_in,
	.plug_out = bq25890h_plug_out,
	.dump_registers = bq25890h_dump_register,
	.enable = bq25890h_charging,
	.is_enabled = bq25890h_is_charging_enable,
	.get_charging_current = bq25890h_get_ichg,
	.set_charging_current = bq25890h_set_ichg,
	.get_input_current = bq25890h_get_icl,
	.set_input_current = bq25890h_set_icl,
	.get_constant_voltage = bq25890h_get_vchg,
	.set_constant_voltage = bq25890h_set_vchg,
	.kick_wdt = bq25890h_kick_wdt,
	.set_mivr = bq25890h_set_ivl,
	.is_charging_done = bq25890h_is_charging_done,
	.get_min_charging_current = bq25890h_get_min_ichg,

	/* Safety timer */
	.enable_safety_timer = bq25890h_set_safety_timer,
	.is_safety_timer_enabled = bq25890h_is_safety_timer_enabled,

	/* Power path */
	.enable_powerpath = NULL,
	.is_powerpath_enabled = NULL,

	.enable_chg_type_det = bq25890h_enable_chgdet,
	/* OTG */
	.enable_otg = bq25890h_set_otg,
	.set_boost_current_limit = bq25890h_set_boost_ilmt,
	.enable_discharge = NULL,

	/* PE+/PE+20 */
	.send_ta_current_pattern = NULL,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
	.reset_ta = NULL,
	.enable_cable_drop_comp = NULL,

	/* ADC */
	.get_tchg_adc = NULL,
};
#endif /*CONFIG_OPLUS_CHARGER_MTK*/

#define NTC_DEFAULT_VOLT_VALUE_MV 950
#define THERMAL_TEMP_UNIT      1000
static int oplus_get_ntc_tmp(struct iio_channel *channel)
{
	int ntc_vol_cur = 0;
	struct sy697x *chip = g_sy;
	static int ntc_vol = NTC_DEFAULT_VOLT_VALUE_MV;
	static int ntc_vol_pre = NTC_DEFAULT_VOLT_VALUE_MV;
	int ntc_temp = SY697X_NORMAL_TEMP25;
	int ntc_vol1 = 0, ntc_temp1 = 0, ntc_vol2 = 0, ntc_temp2 = 0;
	int i = 0;
	int rc = 0;
	struct oplus_chg_chip *opluschg = g_oplus_chg;

	if (!chip || !opluschg) {
		chg_debug("[OPLUS_CHG][%s]: bq25890h not ready!\n", __func__);
		return SY697X_NORMAL_TEMP25;
	}

	if (IS_ERR_OR_NULL(channel)) {
		chg_debug("[OPLUS_CHG][%s]: channel is  NULL !\n", __func__);
		ntc_vol = ntc_vol_pre;
		goto ntcvolt_get_done;
	}
	rc = iio_read_channel_processed(channel, &ntc_vol_cur);
	if (rc < 0) {
		chg_debug("[%s]fail to read usb_temp1 adc rc = %d\n", __func__, rc);
		ntc_vol = ntc_vol_pre;
		goto ntcvolt_get_done;
	}

	if (ntc_vol_cur <= 0) {
		chg_debug("[OPLUS_CHG][%s]:ntc_vol_cur iio_read_channel_processed  get error\n", __func__);
		ntc_vol = ntc_vol_pre;
		goto ntcvolt_get_done;
	}

	ntc_vol_cur = ntc_vol_cur / THERMAL_TEMP_UNIT;
	ntc_vol = ntc_vol_cur;
	ntc_vol_pre = ntc_vol_cur;
ntcvolt_get_done:
	/*map btb temp by usbtemp table*/

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

		ntc_temp = (((ntc_vol - ntc_vol2) * ntc_temp1) + ((ntc_vol1 - ntc_vol) * ntc_temp2)) * THERMAL_TEMP_UNIT / (ntc_vol1 - ntc_vol2);
	}

	return ntc_temp;
}

static void oplus_bq25890h_get_usbtemp_volt(struct oplus_chg_chip *chip)
{
	int usbtemp_volt = 0;
	struct sy697x *chg = g_sy;
	static int usbtemp_volt_l_pre = NTC_DEFAULT_VOLT_VALUE_MV;
	static int usbtemp_volt_r_pre = NTC_DEFAULT_VOLT_VALUE_MV;
	int rc = 0;

	if (!chip || !chg) {
		chg_err("[OPLUS_CHG][%s]: smb5_chg not ready!\n", __func__);
		return;
	}

	if (IS_ERR_OR_NULL(chg->iio.usb_temp_chan1)) {
		chg_err("[OPLUS_CHG][%s]: chg->iio.usb_temp_chan1  is  NULL !\n", __func__);
		chip->usbtemp_volt_l = usbtemp_volt_l_pre;
		rc = oplus_get_iio_channel(chg, "usb_temp1", &chg->iio.usb_temp_chan1);
		if (rc < 0 && !chg->iio.usb_temp_chan1) {
			chg_err(" %s usb_temp_chan1 get failed\n", __func__);
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
		chg_err("[OPLUS_CHG][%s]:USB_TEMPERATURE1 iio_read_channel_processed  get error\n", __func__);
		chip->usbtemp_volt_l = usbtemp_volt_l_pre;
		goto usbtemp_next;
	}

	usbtemp_volt = usbtemp_volt / THERMAL_TEMP_UNIT;
	chip->usbtemp_volt_l = usbtemp_volt;
	usbtemp_volt_l_pre = usbtemp_volt;
usbtemp_next:
	usbtemp_volt = 0;
	if (IS_ERR_OR_NULL(chg->iio.usb_temp_chan2)) {
		chg_err("[OPLUS_CHG][%s]: chg->iio.usb_temp_chan2  is  NULL !\n", __func__);
		rc = oplus_get_iio_channel(chg, "usb_temp2", &chg->iio.usb_temp_chan2);
		if (rc < 0 && !chg->iio.usb_temp_chan2) {
			chg_err(" %s usb_temp_chan2 get failed\n", __func__);
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
		chg_err("[OPLUS_CHG][%s]:USB_TEMPERATURE2 iio_read_channel_processed  get error\n", __func__);
		chip->usbtemp_volt_r = usbtemp_volt_r_pre;
		return;
	}

	usbtemp_volt = usbtemp_volt / THERMAL_TEMP_UNIT;
	chip->usbtemp_volt_r = usbtemp_volt;
	usbtemp_volt_r_pre = usbtemp_volt;
}

static int oplus_ntc_switch_gpio_init(void)
{
	struct sy697x *chip = g_sy;

	if (!chip) {
		chg_err("[OPLUS_CHG][%s]: ntc_switch not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_debug("get ntc_switch_gpio pinctrl fail\n");
		return -EINVAL;
	}

	chip->ntc_switch_high =
			pinctrl_lookup_state(chip->pinctrl,
			"ntc_switch_high");
	if (IS_ERR_OR_NULL(chip->ntc_switch_high)) {
		chg_debug("get ntc_switch_high fail\n");
		return -EINVAL;
	}

	chip->ntc_switch_low =
			pinctrl_lookup_state(chip->pinctrl,
			"ntc_switch_low");
	if (IS_ERR_OR_NULL(chip->ntc_switch_low)) {
		chg_debug("get ntc_switch_low fail\n");
		return -EINVAL;
	}

	/*default switch chg_thermal and bb_thermal*/
	pinctrl_select_state(chip->pinctrl, chip->ntc_switch_low);

	chg_debug("[OPLUS_CHG][%s]: ntc_switch is ready!\n", __func__);
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
		chg_debug("chip->ntcctrl_gpio =%d\n", chip->ntcctrl_gpio);
	}

	return rc;
}

static int oplus_chg_plt_init_for_qcom(struct sy697x *chg)
{
	int ret = -1;
	if (!chg) {
		chg_err("%s bq25890h is null, %d\n", __func__, ret);
		return ret;
	}
	if (oplus_ntc_switch_parse_dt()) {
		chg_err("%s ntc gpio init failed\n", __func__);
		return -1;
	}
	chg_debug("%s ntc channel init success, %d\n", __func__, ret);

	return 0;
}

void oplus_bq25890h_dump_registers(void)
{
	if (atomic_read(&g_sy->driver_suspended) == 1) {
		return;
	}
	bq25890h_dump_regs(g_sy);
	if (g_oplus_chg->sub_chg_ops) {
		g_oplus_chg->sub_chg_ops->dump_registers();
	}
}

int oplus_thermal_tmp_get_typc_1(void)
{
	struct sy697x *chip = g_sy;
	int ret = -1;
	if (!chip || !chip->pinctrl|| !chip->iio.usb_temp_chan1) {
		chg_err("[OPLUS_CHG][%s]: chip not ready!\n", __func__);
		ret = oplus_get_iio_channel(chip, "usb_temp1", &chip->iio.usb_temp_chan1);
		if (ret < 0 && !chip->iio.usb_temp_chan1) {
			chg_err(" %s usb_temp1 get failed\n", __func__);
			return -1;
		}
	}
	typc_temp1 = oplus_get_ntc_tmp(chip->iio.usb_temp_chan1);
	chg_err("usb_temp_chan1 is %d, typc_temp1 is %d!\n", __func__, &chip->iio.usb_temp_chan1, typc_temp1);
	return typc_temp1;
}

int oplus_thermal_tmp_get_typc_2(void)
{
	struct sy697x *chip = g_sy;
	int ret = -1;
	if (!chip || !chip->pinctrl|| !chip->iio.usb_temp_chan2) {
		chg_err("[OPLUS_CHG][%s]: chip not ready!\n", __func__);
		ret = oplus_get_iio_channel(chip, "usb_temp2", &chip->iio.usb_temp_chan2);
		if (ret < 0 && !chip->iio.usb_temp_chan2) {
			chg_err(" %s usb_temp2 get failed\n", __func__);
			return -1;
		}
	}
	typc_temp2 = oplus_get_ntc_tmp(chip->iio.usb_temp_chan2);
	chg_err("usb_temp_chan2 is %d, typc_temp2 is %d!\n", __func__, &chip->iio.usb_temp_chan2, typc_temp2);
	return typc_temp2;
}

int oplus_thermal_tmp_get_fled(void)
{
	struct sy697x *chip = g_sy;
	int ret = -1;
	if (!chip || !chip->pinctrl|| !chip->iio.ntc_switch1_chan) {
		chg_err("[OPLUS_CHG][%s]: chip not ready!\n", __func__);
		ret = oplus_get_iio_channel(chip, "ntc_switch1_chan", &chip->iio.ntc_switch1_chan);
		if (ret < 0 && !chip->iio.ntc_switch1_chan) {
			chg_err(" %s ntc_switch1_chan get failed\n", __func__);
			return -1;
		}
	}
	fled_temp = oplus_get_ntc_tmp(chip->iio.ntc_switch1_chan);
	chg_err("flash led thermal chan is %d, fled_temp is %d!\n", __func__, &chip->iio.ntc_switch1_chan, fled_temp);
	return fled_temp;
}

int oplus_thermal_temp_get_board(void)
{
	struct sy697x *chip = g_sy;
	int ret = -1;
	if (!chip || !chip->pinctrl|| !chip->iio.ntc_switch2_chan) {
		chg_err("[OPLUS_CHG][%s]: chip not ready!\n", __func__);
		ret = oplus_get_iio_channel(chip, "ntc_switch2_chan", &chip->iio.ntc_switch2_chan);
		if (ret < 0 && !chip->iio.ntc_switch2_chan) {
			chg_err(" %s ntc_switch2_chan get failed\n", __func__);
			return -1;
		}
	}
	board_temp = oplus_get_ntc_tmp(chip->iio.ntc_switch2_chan);
	chg_err("board thermal chan is %d,board_temp is %d!\n", __func__, &chip->iio.ntc_switch2_chan, board_temp);
	return board_temp;
}


int oplus_bq25890h_kick_wdt(void)
{
	oplus_check_chrdet_status();
	return bq25890h_reset_watchdog_timer(g_sy);
}
#define CURR_UNIT 100
int oplus_bq25890h_set_ichg(int cur)
{
	struct oplus_chg_chip *chip = g_oplus_chg;
	u32 chg_curr = cur*SY697X_ICH_1000;
	u32 main_cur, slave_cur;
	int ret = 0;

	chg_err("oplus_bq25890h_set_ichg curr = %d , slave_charger_enable =%d\n", cur, chip->slave_charger_enable);

	if (chip->em_mode) {
		chg_curr = chip->limits.temp_normal_phase2_fastchg_current_ma_high*SY697X_ICH_1000;
	}
	if (chip->is_double_charger_support
			&& (chip->slave_charger_enable || chip->em_mode)) {
		main_cur = chg_curr  * current_percent / CURR_UNIT;
		ret = bq25890h_set_chargecurrent(g_sy, main_cur/SY697X_ICH_1000);
		if (ret < 0) {
			chg_debug("set fast charge current:%d fail\n", main_cur);
		}
		ret = _bq25890h_get_ichg(g_sy, &main_cur);
		if (ret < 0) {
			chg_debug("get fast charge current:%d fail\n", main_cur);
			return ret;
		}
		slave_cur = chg_curr - main_cur;
		chip->sub_chg_ops->charging_current_write_fast(slave_cur/SY697X_ICH_1000);
	} else {
		ret = bq25890h_set_chargecurrent(g_sy, chg_curr/SY697X_ICH_1000);
		if (ret < 0) {
			chg_debug("set fast charge current:%d fail\n", chg_curr);
		}
	}
	return 0;
}
#define VBAT_LIM4300 4300
#define VBAT_LIM4200 4200
#define VBAT_LIM4400 4400
#define VBAT_OFFSET400 400
#define VBAT_OFFSET300 300
#define VBAT_OFFSET200 200
void oplus_bq25890h_set_mivr(int vbatt)
{
	u32 mV = 0;

	if (vbatt > VBAT_LIM4300) {
		mV = vbatt + VBAT_OFFSET400;
	} else if (vbatt > VBAT_LIM4200) {
		mV = vbatt + VBAT_OFFSET300;
	} else {
		mV = vbatt + VBAT_OFFSET200;
	}

	if (mV < VBAT_LIM4200)
		mV = VBAT_LIM4200;

	bq25890h_set_input_volt_limit(g_sy, mV);
}

void oplus_bq25890h_set_mivr_by_battery_vol(void)
{
	u32 mV = 0;
	int vbatt = 0;
	struct oplus_chg_chip *chip = g_oplus_chg;

	if (chip) {
		vbatt = chip->batt_volt;
	}

	if (vbatt > VBAT_LIM4300) {
		mV = vbatt + VBAT_OFFSET400;
	} else if (vbatt > VBAT_LIM4200) {
		mV = vbatt + VBAT_OFFSET300;
	} else {
		mV = vbatt + VBAT_OFFSET200;
	}

	if (mV < VBAT_LIM4400)
		mV = VBAT_LIM4400;

	bq25890h_set_input_volt_limit(g_sy, mV);
}

static int usb_icl[] = {
	100, 500, 900, 1000, 1200, 1500, 1750, 2000, 3000,
};

#define CHG_VOLTH 7600
#define VBAT_LIM4100 4100
#define VBAT_LIM4550 4550
#define VBAT_LIM4500 4500
#define CURR_LIM500 500
#define CURR_LIM900 900
#define CURR_LIM1000 1000
#define CURR_LIM1200 1200
#define CURR_LIM1500 1500
#define CURR_LIM2000 2000
#define CURR_LIM3000 3000
#define CURR_UNIT50 50
#define DEALY90 90
#define DEALY120 120
int oplus_bq25890h_set_aicr(int current_ma)
{
	struct oplus_chg_chip *chip = g_oplus_chg;
	int rc = 0, i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	int aicl_point_temp = 0;
	int main_cur = 0;
	int slave_cur = 0;
	int pre_input_cur = 0;
	static bool slave_set_input_current_enable = false;

	if((g_sy->chg_type == CHARGER_UNKNOWN) && (g_sy->oplus_chg_type == POWER_SUPPLY_TYPE_USB_DCP))
		current_ma =  CURR_LIM500;

	pre_input_cur = bq25890h_get_input_current();
	if ((!chip->slave_charger_enable) && (pre_input_cur < current_ma)) {
		chg_err("Input current limit = %d\n", pre_input_cur);
		g_sy->pre_current_ma = -1;
	}

	if (chip->is_double_charger_support) {
		if (((slave_set_input_current_enable && chip->slave_charger_enable)
			|| (!slave_set_input_current_enable && !chip->slave_charger_enable))
			&& g_sy->pre_current_ma == current_ma && !chip->em_mode) {
				bq25890h_disable_enlim(g_sy);
				chg_err("%s Input current limit during charging\n", __func__);
				return rc;
			}
	}

	if (chip->is_double_charger_support) {
		rc = chip->sub_chg_ops->charging_disable();
		if (rc < 0) {
			chg_debug("disable sub charging fail\n");
		}
		chg_err("%s disabel subchg\n", __func__);
	}

	if (chip->is_double_charger_support) {
		if (slave_set_input_current_enable && !chip->slave_charger_enable && g_sy->pre_current_ma == current_ma && !chip->em_mode) {
			for (i = 1;i < 9;i++) {
				if (current_ma < usb_icl[i])
					 break;
			}
			i = i-1;
			chg_err("%s Turn off the slave charger input current limit usb_icl[%d]=%d\n", __func__, i, usb_icl[i]);
			goto aicl_end;
	        }
	}

	g_sy->pre_current_ma = current_ma;
	chg_err("%s usb input max current limit=%d\n", __func__, current_ma);
	if (chip && chip->is_double_charger_support == true) {
		chg_vol = oplus_bq25890h_get_vbus();
		if (chg_vol > CHG_VOLTH) {
			aicl_point_temp = aicl_point = CHG_VOLTH;
		} else {
			if (chip->batt_volt > VBAT_LIM4100)
				aicl_point_temp = aicl_point = VBAT_LIM4550;
			else
				aicl_point_temp = aicl_point = VBAT_LIM4500;
		}
	} else {
		if (chip->batt_volt > VBAT_LIM4100)
			aicl_point_temp = aicl_point = VBAT_LIM4550;
		else
			aicl_point_temp = aicl_point = VBAT_LIM4500;
	}

	if (current_ma < CURR_LIM500) {
		i = 0;
		goto aicl_end;
	}

	i = 1; /* 500 */
	bq25890h_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(DEALY90);
	bq25890h_disable_enlim(g_sy);
	chg_vol = oplus_bq25890h_get_vbus();
	if (chg_vol < aicl_point_temp) {
		chg_debug("use 500 here\n");
		goto aicl_end;
	} else if (current_ma < CURR_LIM900)
		goto aicl_end;

	i = 2; /* 900 */
	bq25890h_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(DEALY90);
	chg_vol = oplus_bq25890h_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < CURR_LIM1000)
		goto aicl_end;

	i = 3; /* 1000 */
	bq25890h_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(DEALY90);
	chg_vol = oplus_bq25890h_get_vbus();
	if (chg_vol < aicl_point_temp) {
	    i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < CURR_LIM1200)
		goto aicl_end;

	i = 4; /* 1200 */
	bq25890h_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(DEALY90);
	chg_vol = oplus_bq25890h_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 5; /* 1500 */
	aicl_point_temp = aicl_point + CURR_UNIT50;
	bq25890h_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(DEALY120);
	chg_vol = oplus_bq25890h_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 2;
		goto aicl_pre_step;
	} else if (current_ma < CURR_LIM1500) {
		i = i - 1;
		goto aicl_end;
	} else if (current_ma < CURR_LIM2000)
		goto aicl_end;

	i = 6; /* 1750 */
	aicl_point_temp = aicl_point + CURR_UNIT50;
	bq25890h_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(DEALY120);
	chg_vol = oplus_bq25890h_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 2;
		goto aicl_pre_step;
	}

	i = 7; /* 2000 */
	aicl_point_temp = aicl_point;
	bq25890h_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(DEALY90);
	chg_vol = oplus_bq25890h_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i =  i - 2;/*1.5*/
		goto aicl_pre_step;
	} else if (current_ma < CURR_LIM3000)
		goto aicl_end;

	i = 8; /* 3000 */
	bq25890h_set_input_current_limit(g_sy, usb_icl[i]);
	msleep(DEALY90);
	chg_vol = oplus_bq25890h_get_vbus();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma >= CURR_LIM3000)
		goto aicl_end;

aicl_pre_step:
	if (chip->is_double_charger_support
			&& (chip->slave_charger_enable || chip->em_mode)) {
		chg_debug("enable sgm41511x for charging\n");

		main_cur = (usb_icl[i] * current_percent)/CURR_UNIT;
		main_cur -= main_cur % CURR_UNIT50;
		slave_cur = usb_icl[i] - main_cur;
		bq25890h_set_input_current_limit(g_sy, main_cur);
		chip->sub_chg_ops->input_current_write(slave_cur);

		rc = chip->sub_chg_ops->charging_enable();
		if (rc < 0) {
			chg_debug("enable sub charging fail\n");
		}

		if (chip->em_mode && !chip->slave_charger_enable) {
			chip->slave_charger_enable = true;
		}
		slave_set_input_current_enable = true;
		chg_debug("usb input max current limit aicl: master and salve input current: %d, %d, slave_set_input_current_enable=%d\n",
				main_cur, slave_cur, slave_set_input_current_enable);
	} else {
		bq25890h_set_input_current_limit(g_sy, usb_icl[i]);
		slave_set_input_current_enable = false;
	}

	chg_err("%s:usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d slave_set_input_current_enable=%d aicl_pre_step\n",
                          __func__, chg_vol, i, usb_icl[i], aicl_point_temp, slave_set_input_current_enable);
	return rc;
aicl_end:
	if (chip->is_double_charger_support
			&& (chip->slave_charger_enable || chip->em_mode)) {
		main_cur = (usb_icl[i] * current_percent)/CURR_UNIT;
		main_cur -= main_cur % CURR_UNIT50;
		slave_cur = usb_icl[i] - main_cur;
		bq25890h_set_input_current_limit(g_sy, main_cur);
		chip->sub_chg_ops->input_current_write(slave_cur);
		rc = chip->sub_chg_ops->charging_enable();
		if (rc < 0) {
			chg_debug("enable sub charging fail\n");
		}

		if (chip->em_mode && !chip->slave_charger_enable) {
			chip->slave_charger_enable = true;
		}
		slave_set_input_current_enable = true;
		chg_debug("usb input max current limit aicl: master and salve input current: %d, %d slave_set_input_current_enable=%d\n",
                                main_cur, slave_cur, slave_set_input_current_enable);
	} else {
		bq25890h_set_input_current_limit(g_sy, usb_icl[i]);
		slave_set_input_current_enable = false;
	}

	if (chip->em_mode) {
		chip->charger_volt = chg_vol;
		power_supply_changed(chip->batt_psy);
	}
	chg_err("%s:usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d slave_set_input_current_enable=%d aicl_end\n",
                         __func__, chg_vol, i, usb_icl[i], aicl_point_temp, slave_set_input_current_enable);
	return rc;
}

int oplus_bq25890h_set_input_current_limit(int current_ma)
{
	struct oplus_chg_chip *chip = g_oplus_chg;

	if (g_sy == NULL || chip == NULL)
		return 0;

	if (current_ma == 0) {
		chg_debug("[%s] current_ma == 0 mmi[%d %d]\n", __func__, chip->mmi_chg, chip->mmi_fastchg);
		bq25890h_set_input_current_limit(g_sy, 0);
		return 0;
	}

	if (atomic_read(&g_sy->driver_suspended) == 1) {
		return 0;
	}

	if (atomic_read(&g_sy->charger_suspended) == 1) {
		chg_err("suspend,ignore set current=%dmA\n", current_ma);
		return 0;
	}

	chg_debug("[%s] is_force_aicl current=%d\n", __func__, current_ma);
	g_sy->aicr = current_ma;
	oplus_bq25890h_set_aicr(g_sy->aicr);
	return 0;
}

int oplus_bq25890h_set_cv(int cur)
{
	int rc = 0;
	if (g_oplus_chg && g_oplus_chg->sub_chg_ops && g_oplus_chg->sub_chg_ops->float_voltage_write && g_oplus_chg->dual_charger_support) {
		rc = g_oplus_chg->sub_chg_ops->float_voltage_write(cur);
		if (rc < 0) {
			chg_debug("subcharger set vfloat fail\n");
		}
	}
	return bq25890h_set_chargevolt(g_sy, cur);
}

int oplus_bq25890h_set_ieoc(int cur)
{
	return bq25890h_set_term_current(g_sy, cur);
}

int oplus_bq25890h_charging_enable(void)
{
	return bq25890h_enable_charger(g_sy);
}

int oplus_bq25890h_charging_disable(void)
{
	struct oplus_chg_chip *chip = g_oplus_chg;

#ifdef CONFIG_OPLUS_CHARGER_MTK
	struct charger_manager *info = NULL;

	if (g_sy->chg_consumer != NULL)
		info = g_sy->chg_consumer->cm;

	if (!info) {
		dev_info(g_sy->dev, "%s:error\n", __func__);
		return false;
	}
#endif /*CONFIG_OPLUS_CHARGER_MTK*/

	if (chip && chip->sub_chg_ops && chip->is_double_charger_support) {
		chip->sub_chg_ops->charging_disable();
	}

	return bq25890h_disable_charger(g_sy);
}

int oplus_bq25890h_hardware_init(void)
{
	int ret = 0;
	chg_err("oplus_bq25890h_hardware_init enter");

	/* Enable charging */
	if (strcmp(g_sy->chg_dev_name, "primary_chg") == 0) {
		ret = bq25890h_enable_charger(g_sy);
		if (ret < 0)
			chg_debug("%s: en chg fail\n", __func__);
	}

	return ret;
}

int oplus_bq25890h_is_charging_enabled(void)
{
	int ret = 0;
	u8 val;
	struct sy697x *sy = g_sy;
	if (!sy) {
		return 0;
	}

	ret = bq25890h_read_byte(sy, SY697X_REG_03, &val);
	if (!ret) {
		return !!(val & SY697X_CHG_CONFIG_MASK);
	}
	return 0;
}


void oplus_set_prswap(bool swap)
{
	chg_debug("%s set prswap %d\n", __func__, swap);

	if (swap)
		oplus_is_prswap = true;
	else
		oplus_is_prswap = false;
}

int oplus_bq25890h_is_charging_done(void)
{
	bool done;

	bq25890h_check_charge_done(g_sy, &done);

	return done;
}

int oplus_bq25890h_enable_otg(void)
{
	int ret = 0;
	struct oplus_chg_chip *chgchip = g_oplus_chg;

	ret = bq25890h_set_boost_current(g_sy, g_sy->platform_data->boosti);
	chg_debug("%s set boost curr %d\n", __func__, g_sy->platform_data->boosti);
	ret = bq25890h_enable_otg(g_sy);

	if (ret < 0) {
		chg_debug("%s en otg fail(%d)\n", __func__, ret);
		return ret;
	}

	g_sy->otg_enable = true;
	power_supply_changed(chgchip->usb_psy);

	return ret;
}

static bool oplus_get_otg_enable(void)
{
	u8 data = 0;

	if (g_sy) {
		g_bq25890h_read_reg(g_sy, SY697X_REG_03, &data);
	}
	chg_debug("%s en otg (%d)\n", __func__, data);
	if (data & (SY697X_OTG_ENABLE << SY697X_OTG_CONFIG_SHIFT)) {
		chg_debug("%s golf otg enable (true)\n", __func__);
		return true;
	} else {
		chg_debug("%s golf otg disable (false)\n", __func__);
		return false;
	}
	return false;
}

int oplus_bq25890h_disable_otg(void)
{
	int ret = 0;
	struct oplus_chg_chip *chgchip = g_oplus_chg;

	ret = bq25890h_disable_otg(g_sy);

	if (ret < 0) {
		chg_debug("%s disable otg fail(%d)\n", __func__, ret);
		return ret;
	}

	g_sy->otg_enable = false;
	power_supply_changed(chgchip->usb_psy);

	return ret;
}

int oplus_bq25890h_disable_te(void)
{
	return  bq25890h_enable_term(g_sy, false);
}

int oplus_bq25890h_get_chg_current_step(void)
{
	return SY697X_ICHG_LSB;
}

int oplus_bq25890h_get_charger_type(void)
{
	struct oplus_chg_chip *chip = g_oplus_chg;

	if (!g_sy || !g_oplus_chg) {
		chg_err("%s null pointer \n", __func__);
		return 0;
	}

	if (chip->charger_exist && !chip->batt_full && chip->batt_exist
		&& (CHARGING_STATUS_FAIL != chip->charging_state) && (chip->mmi_chg != 0)
		&& (g_sy->oplus_chg_type != POWER_SUPPLY_TYPE_UNKNOWN)
		&& (chip->prop_status == POWER_SUPPLY_STATUS_NOT_CHARGING)) {
		chip->prop_status = POWER_SUPPLY_STATUS_CHARGING;
		power_supply_changed(chip->batt_psy);
	}

	chg_err("%s g_sy->oplus_chg_type = %d, mmi_chg = %d, chip->charger_type = %d, \
		chip->charger_exist = %d, chip->batt_full = %d, chip->batt_exist = %d, \
		chip->charging_state = %d, chip->prop_status = %d\n",
		__func__, g_sy->oplus_chg_type, chip->mmi_chg, chip->charger_type,
		chip->charger_exist, chip->batt_full, chip->batt_exist,
		chip->charging_state, chip->prop_status);

	return g_sy->oplus_chg_type;
}

int oplus_bq25890h_charger_suspend(void)
{
	if (g_sy)
		bq25890h_enter_hiz_mode(g_sy);
	if (g_oplus_chg && g_oplus_chg->is_double_charger_support) {
		/*g_oplus_chg->slave_charger_enable = false;*/
		g_oplus_chg->sub_chg_ops->charger_suspend();
	}
	chg_err("%s\n", __func__);
	return 0;
}

int oplus_bq25890h_charger_unsuspend(void)
{
	if (g_sy)
		bq25890h_exit_hiz_mode(g_sy);
	if (g_oplus_chg && g_oplus_chg->is_double_charger_support) {
		g_oplus_chg->sub_chg_ops->charger_unsuspend();
	}
	chg_err("%s\n", __func__);
	return 0;
}

int oplus_bq25890h_set_rechg_vol(int vol)
{
	return 0;
}

int oplus_bq25890h_reset_charger(void)
{
	return 0;
}

bool oplus_bq25890h_check_charger_resume(void)
{
	return true;
}

static int oplus_register_extcon(struct sy697x *chip)
{
	int rc = 0;

	chip->extcon = devm_extcon_dev_allocate(chip->dev, smblib_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		rc = PTR_ERR(chip->extcon);
		chg_err("failed to allocate extcon device rc=%d\n",
				rc);
		goto cleanup;
	}

	rc = devm_extcon_dev_register(chip->dev, chip->extcon);
	if (rc < 0) {
		chg_err("failed to register extcon device rc=%d\n",
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

	chg_err("oplus_register_extcon rc=%d\n",
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

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	chg_err("opluschg_parse_custom_dt 1\n");

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

		chg_err("opluschg_parse_custom_dt 3\n");
		chip->normalchg_gpio.chargerid_switch_active =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_active");

		chg_err("opluschg_parse_custom_dt 4\n");
		chip->normalchg_gpio.chargerid_switch_sleep =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_sleep");
		chg_err("opluschg_parse_custom_dt 5\n");
		pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.chargerid_switch_sleep);
		chip->normalchg_gpio.chargerid_switch_default =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_default");
		chg_err("opluschg_parse_custom_dt 6\n");
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
	if (rc) {
		chg_err("[OPLUS_CHG][%s]: oplus_chg_ccdetect_parse_dt fail!\n", __func__);
	}

	chg_debug("[%s] done\n", __func__);
	return 0;
}

static int oplus_bq25890h_get_chargerid_switch(void)
{
	if (!g_oplus_chg) {
		chg_err("fail to init oplus_chip\n");
		return 0;
	}

	if (g_oplus_chg->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return 0;
	}

	chg_debug("[%s] gpio[%d] done\n", __func__, g_oplus_chg->normalchg_gpio.chargerid_switch_gpio);
	return gpio_get_value(g_oplus_chg->normalchg_gpio.chargerid_switch_gpio);
}

static void oplus_bq25890h_set_chargerid_switch(int value)
{
	chg_debug("[%s] val[%d]\n", __func__, value);

	if (!g_oplus_chg) {
		chg_err("fail to init oplus_chip\n");
		return;
	}

	if (g_oplus_chg->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(g_oplus_chg->normalchg_gpio.pinctrl)
		|| IS_ERR_OR_NULL(g_oplus_chg->normalchg_gpio.chargerid_switch_active)
		|| IS_ERR_OR_NULL(g_oplus_chg->normalchg_gpio.chargerid_switch_sleep)
		|| IS_ERR_OR_NULL(g_oplus_chg->normalchg_gpio.chargerid_switch_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (g_oplus_chg->pmic_spmi.smb5_chip)
		mutex_lock(&g_oplus_chg->pmic_spmi.smb5_chip->chg.pinctrl_mutex);


	if (value) {
		pinctrl_select_state(g_oplus_chg->normalchg_gpio.pinctrl,
			g_oplus_chg->normalchg_gpio.chargerid_switch_active);
		gpio_direction_output(g_oplus_chg->normalchg_gpio.chargerid_switch_gpio, 1);
		chg_err("switch rxtx\n");
	}
	else {
		pinctrl_select_state(g_oplus_chg->normalchg_gpio.pinctrl,
			g_oplus_chg->normalchg_gpio.chargerid_switch_sleep);
		gpio_direction_output(g_oplus_chg->normalchg_gpio.chargerid_switch_gpio, 0);
		chg_err("switch dpdm\n");
	}

	if (g_oplus_chg->pmic_spmi.smb5_chip)
		mutex_unlock(&g_oplus_chg->pmic_spmi.smb5_chip->chg.pinctrl_mutex);

	chg_debug("set usb_switch_1 = %d, result = %d\n", value, oplus_bq25890h_get_chargerid_switch());
}

bool oplus_check_pdphy_ready(void)
{
	return (g_sy != NULL && g_sy->tcpc != NULL && g_sy->tcpc->pd_inited_flag);
}
int oplus_bq25890h_get_charger_subtype(void)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
	struct charger_manager *info = NULL;

	if (g_sy->chg_consumer != NULL)
		info = g_sy->chg_consumer->cm;

	if (!info) {
		chg_err("%s:error\n", __func__);
		return false;
	}
#endif /*CONFIG_OPLUS_CHARGER_MTK*/
	if (g_sy->hvdcp_can_enabled) {
		return CHARGER_SUBTYPE_QC;
	}
	if (g_sy->pd_type == PD_CONNECT_PE_READY_SNK ||
		g_sy->pd_type == PD_CONNECT_PE_READY_SNK_PD30 ||
		g_sy->pd_type == PD_CONNECT_PE_READY_SNK_APDO) {
		return CHARGER_SUBTYPE_PD;
	}

	return CHARGER_SUBTYPE_DEFAULT;
}

bool oplus_bq25890h_need_to_check_ibatt(void)
{
	return false;
}

int oplus_bq25890h_get_dyna_aicl_result(void)
{
	int sy_ma = 0;

	bq25890h_read_idpm_limit(g_sy, &sy_ma);
	return sy_ma;
}

#define DELAY500 500
#define CHG_VOLTHL 6500
#define CHG_VOLTHH 7500
#define SOC_LIM 90
#define CHG_VOL5400 5400
void vol_convert_work(struct work_struct *work)
{
	int retry = RETRY_CNT;
	if (!g_sy->pdqc_setup_5v) {
		chg_err("%s, set_to_9v\n", __func__);
		if (oplus_bq25890h_get_vbus() < HVDCP_THR) {
			bq25890h_switch_to_hvdcp(g_sy, HVDCP_9V);
			Charger_Detect_Init();
			oplus_for_cdp();
			g_sy->is_force_aicl = true;
			g_sy->is_retry_bc12 = true;

			msleep(DELAY500);
			while(retry--) {
				if (oplus_bq25890h_get_vbus() > CHG_VOLTH) {
					chg_err("%s, set_to_9v success\n", __func__);
					break;
				}
				msleep(DELAY500);
			}
		}
	} else {
		chg_err("%s, set_to_5v\n", __func__);
		if (oplus_bq25890h_get_vbus() > CHG_VOLTHL) {
			bq25890h_switch_to_hvdcp(g_sy, HVDCP_5V);
			Charger_Detect_Init();
			oplus_for_cdp();
			g_sy->is_force_aicl = true;
			g_sy->is_retry_bc12 = true;

			msleep(DELAY500);
			while(retry--) {
				if (oplus_bq25890h_get_vbus() < HVDCP_THR) {
					chg_err("%s, set_to_5v success\n", __func__);
					break;
				}
				msleep(DELAY500);
			}
		}
	}
	g_sy->is_bc12_end = true;
}

bool oplus_chg_check_qchv_condition(void)
{
	struct oplus_chg_chip *chip = g_oplus_chg;

	if (!chip) {
		chg_err("oplus_chip is null\n");
		return false;
	}

	chip->charger_volt = chip->chg_ops->get_charger_volt();
	if (!chip->calling_on && !chip->camera_on && chip->charger_volt < CHG_HV_THR
	    && chip->ui_soc < SOC_LIM
	    && (chip->temperature <= chip->limits.tbatt_pdqc_to_5v_thr)
	    && !chip->cool_down_force_5v) {
		return true;
	}

	return false;
}


int oplus_bq25890h_set_qc_config(void)
{
	struct oplus_chg_chip *chip = g_oplus_chg;
	static int qc_to_9v_count = 0; /*for huawei quick charger*/
#ifdef CONFIG_OPLUS_CHARGER_MTK
	struct charger_manager *info = NULL;
	static int qc_to_9v_count = 0; /*for huawei quick charger*/

	if (g_sy->chg_consumer != NULL)
		info = g_sy->chg_consumer->cm;

	if (!info) {
		chg_err("%s:error\n", __func__);
		return false;
	}
#endif /*CONFIG_OPLUS_CHARGER_MTK*/
	if (!chip) {
		chg_err("%s: error\n", __func__);
		return false;
	}

	if (disable_QC) {
		chg_err("%s:disable_QC\n", __func__);
		return false;
	}

	if (g_sy->disable_hight_vbus == 1) {
		chg_err("%s:disable_hight_vbus\n", __func__);
		return false;
	}

	if (chip->charger_volt > CHG_HV_THR && (chip->calling_on || chip->camera_on || chip->ui_soc >= SOC_LIM
		|| chip->cool_down_force_5v == true || chip->temperature > chip->limits.tbatt_pdqc_to_5v_thr)) {
		chg_debug("++%s: set_qc_to 5V =%d,bc12=%d", __func__, g_sy->pdqc_setup_5v, g_sy->is_bc12_end);
		chg_err("%d,%d,%d,%d,%d,%d\n", chip->calling_on, chip->camera_on, chip->ui_soc, chip->cool_down_force_5v, chip->temperature, chip->charger_volt);
		g_sy->pdqc_setup_5v = true;
		if (g_sy->is_bc12_end) {
			g_sy->is_bc12_end = false;
			schedule_delayed_work(&g_sy->sy697x_vol_convert_work, 0);
		}
		chg_debug("%s: set_qc_to 5V =%d,bc12=%d", __func__, g_sy->pdqc_setup_5v, g_sy->is_bc12_end);
	} else {
		g_sy->pdqc_setup_5v = false;
		if (g_sy->is_bc12_end) {
			g_sy->is_bc12_end = false;
			schedule_delayed_work(&g_sy->sy697x_vol_convert_work, 0);
		}
		/*add for huawei quick charge*/
		if (oplus_bq25890h_get_vbus() < HVDCP_THR) {
			if (qc_to_9v_count >= 5) {
				if (is_bq25890h(g_sy) == false) {
					g_sy->hvdcp_can_enabled = false;
				}
				qc_to_9v_count = 0;
			}
			if (g_sy->hvdcp_can_enabled) {
				qc_to_9v_count++;
			}
		}

		chg_debug("%d,%d,%d,%d,%d,%d\n", chip->calling_on, chip->camera_on, chip->ui_soc, chip->cool_down_force_5v, chip->limits.tbatt_pdqc_to_5v_thr, chip->charger_volt);
		chg_err("%s: set_qc_to 9V =%d,bc12=%d", __func__, g_sy->pdqc_setup_5v, g_sy->is_bc12_end);
	}

	return true;
}

int oplus_bq25890h_enable_qc_detect(void)
{
	return 0;
}
#define CHG_VOL4400 4400
bool oplus_bq25890h_need_retry_aicl(void)
{
	static bool connect = false;
	if (!g_sy)
		return false;
	if ((g_sy->boot_mode == MSM_BOOT_MODE__RF || g_sy->boot_mode == MSM_BOOT_MODE__WLAN) && !connect) {
		if (g_oplus_chg->chg_ops->get_charger_volt() > CHG_VOL4400) {
			g_sy->chg_type = STANDARD_HOST;
			g_sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB;
			g_oplus_chg->charger_type = POWER_SUPPLY_TYPE_USB;
			g_oplus_chg->chg_ops->usb_connect();
			connect = true;
		}
	}
	if (g_sy->cdp_retry_aicl) {
		g_sy->cdp_retry_aicl = false;
		chg_err("retry aicl\n");
		return true;
	}
	return g_sy->cdp_retry_aicl;
}
static bool oplus_check_chrdet_status(void)
{
	u8 reg_val;
	int ret;

	if (!g_sy)
		return false;

	if (g_sy->is_retry_bc12 == true && g_sy->is_force_aicl == false) {
		ret = bq25890h_read_byte(g_sy, SY697X_REG_0B, &reg_val);
		if (!ret) {
			if (SY697X_VBUS_STAT_MASK & reg_val) {
				chg_err("bc12 succses\n");
				g_sy->is_retry_bc12 = false;
				chg_debug("[%s 1]:chg_type = %d, %d, %d\n", __func__, g_sy->chg_type, g_sy->oplus_chg_type, g_oplus_chg->charger_type);
				sy_charger_type_recheck(g_sy);
				chg_debug("[%s 2]:chg_type = %d, %d, %d\n", __func__, g_sy->chg_type, g_sy->oplus_chg_type, g_oplus_chg->charger_type);
			} else if (!g_sy->power_good) {
				g_sy->is_retry_bc12 = false;
			}
		}
	}
	return ret;
}

bool oplus_bq25890h_get_shortc_hw_gpio_status(void)
{
	return false;
}

int oplus_bq25890h_chg_set_high_vbus(bool en)
{
	int subtype;
	struct oplus_chg_chip *chip = g_oplus_chg;

	if (!chip) {
		chg_debug("%s: error\n", __func__);
		return false;
	}

	if (en) {
		g_sy->disable_hight_vbus= 0;
		if (chip->charger_volt >CHG_VOLTHH) {
			chg_debug("%s:charger_volt already 9v\n", __func__);
			return false;
		}

		if (g_sy->pdqc_setup_5v) {
			chg_debug("%s:pdqc already setup5v no need 9v\n", __func__);
			return false;
		}

	} else {
		g_sy->disable_hight_vbus= 1;
		if (chip->charger_volt < CHG_VOL5400) {
			chg_debug("%s:charger_volt already 5v\n", __func__);
			return false;
		}
	}

	subtype = oplus_bq25890h_get_charger_subtype();
	if (subtype == CHARGER_SUBTYPE_QC) {
		if (en) {
			chg_debug("%s:QC Force output 9V\n", __func__);
			bq25890h_switch_to_hvdcp(g_sy, HVDCP_9V);
		} else {
			bq25890h_switch_to_hvdcp(g_sy, HVDCP_5V);
			chg_debug("%s: set qc to 5V", __func__);
		}
	} else {
		chg_debug("%s:do nothing\n", __func__);
	}

	return false;
}
#define BC_USB_WAIT_CNT 6
#define RE_CHK 100
#define RE_UNKNOWN_CHK 20
#define DELAY20 20
#define DELAY2000 2000
#define DELAY2100 2100
static int oplus_charger_type_thread(void *data)
{
	int ret;

	u8 reg_val = 0;
	int vbus_stat = 0;
	bool curr_pg = 0;
	struct sy697x *sy = (struct sy697x *) data;
	int re_check_count = 0, re_unknown_check = 0;

	while (1) {
		wait_event_interruptible(oplus_chgtype_wq, sy->chg_need_check == true);
		if (oplus_get_otg_enable()) {
			chg_err("[%s 1]:otg enable is break chg_type = %d, %d, %d\n", __func__, sy->chg_type, sy->oplus_chg_type, g_oplus_chg->charger_type);
			break;
		}

		re_check_count = 0;
		re_unknown_check = 0;
		sy->chg_start_check = true;
		sy->chg_need_check = false;
RECHECK:
		ret = bq25890h_read_byte(sy, SY697X_REG_0B, &reg_val);
		if (g_oplus_chg)
			chg_err("[%s 1]:vbus_stat: %d chg_type = %d, %d, %d\n", __func__, vbus_stat, sy->chg_type, sy->oplus_chg_type, g_oplus_chg->charger_type);
		if (ret)
			break;

		vbus_stat = (reg_val & SY697X_VBUS_STAT_MASK);
		vbus_stat >>= SY697X_VBUS_STAT_SHIFT;
		curr_pg = !!(reg_val & SY697X_PG_STAT_MASK);
		sy->vbus_type = vbus_stat;
		if(!curr_pg) {
		        sy->chg_type = CHARGER_UNKNOWN;
		        sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		        g_oplus_chg->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		} else {
			switch (vbus_stat) {
			case SY697X_VBUS_TYPE_NONE:
				re_check_count++;
				sy->chg_type = CHARGER_UNKNOWN;
				if ((sy->power_good)&&(re_check_count <= RE_CHK)) {
				       chg_err("%s check_count:%d \n", __func__, re_check_count);
				       if (re_check_count > BC_USB_WAIT_CNT) {
				                sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
				       } else if (re_check_count == BC_USB_WAIT_CNT) {
				                sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
				                bq25890h_inform_charger_type(sy);
				                oplus_chg_wake_update_work();
				        } else {
				                sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
				        }
				        msleep(30);
				        goto RECHECK;
				}
			case SY697X_VBUS_TYPE_SDP:
				/*sy->chg_type = STANDARD_HOST;*/
				/*sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB;*/
				if (!sy->sdp_retry) {
					sy->sdp_retry = true;
					schedule_delayed_work(&g_sy->sy697x_bc12_retry_work, OPLUS_BC12_RETRY_TIME);
				}
				break;
			case SY697X_VBUS_TYPE_CDP:
				/*sy->chg_type = CHARGING_HOST;*/
				/*sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_CDP;*/
				if (!sy->sdp_retry) {
					sy->sdp_retry = true;
					schedule_delayed_work(&g_sy->sy697x_bc12_retry_work, OPLUS_BC12_RETRY_TIME);
				}
				break;
			case SY697X_VBUS_TYPE_DCP:
				sy->chg_type = STANDARD_CHARGER;
				sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
				break;
			case SY697X_VBUS_TYPE_HVDCP:
				sy->chg_type = STANDARD_CHARGER;
				sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
				sy->hvdcp_can_enabled = true;
				break;
			case SY697X_VBUS_TYPE_UNKNOWN:
				re_unknown_check++;
				if ((re_unknown_check < RE_UNKNOWN_CHK) && (sy->power_good)) {
					msleep(DELAY20);
					goto RECHECK;
				} else if (sy->power_good) {
					sy->chg_type = NONSTANDARD_CHARGER;
					sy->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
					break;
				} else {
					sy->chg_type = CHARGER_UNKNOWN;
					sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
				}
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
		}
		if (g_oplus_chg) {
			chg_err("[%s 2]: chg_type = %d, %d, %d vbus_on[%d]\n", __func__, sy->chg_type,
				sy->oplus_chg_type, g_oplus_chg->charger_type, sy->vbus_on);
				/*fix first poweron adb port*/
				/*if ((oplus_bq25890h_get_charger_type() == POWER_SUPPLY_TYPE_USB) || (oplus_bq25890h_get_charger_type() == POWER_SUPPLY_TYPE_USB_CDP)) {
					oplus_notify_device_mode(true);
				}*/
		}

		if ((sy->oplus_chg_type != POWER_SUPPLY_TYPE_USB) && (sy->oplus_chg_type != POWER_SUPPLY_TYPE_USB_CDP)) {
			bq25890h_inform_charger_type(sy);
		} else {
			opluschg_updata_usb_type(sy);
		}
		oplus_chg_wake_update_work();
		chg_err("[%s] plus_chg_wake_update_work\n", __func__);
		/*bq25890h_adc_start(sy, true);*/
		schedule_delayed_work(&g_sy->sy697x_aicl_work, OPLUS_DELAY_WORK_TIME_BASE*2);
		msleep(DELAY2000);
		oplus_wake_up_usbtemp_thread();
		chg_debug("[%s] oplus_wake_up_usbtemp_thread\n", __func__);
	}

	return 0;
}

static void charger_type_thread_init(void)
{
	g_sy->chg_need_check = false;
	charger_type_kthread =
			kthread_run(oplus_charger_type_thread, g_sy, "chgtype_kthread");
	if (IS_ERR(charger_type_kthread)) {
		chg_err("failed to cread oplus_usbtemp_kthread\n");
	}
}

extern int get_boot_mode(void);
#define DEFBOOT_RESON 101
#define DEF_BATVOL 3800
#define DEF_SOC 50
#define CC_WORKTIME 6000
#define DELAY10000 10000
#define DELAY11000 11000
#define VBUS_DEF 5000
int opluschg_get_boot_reason(void)
{
	return DEFBOOT_RESON;
}
static int opluschg_get_battery_voltage(void)
{
	return DEF_BATVOL;	/* Not use anymore */
}

static int oplus_get_rtc_ui_soc(void)
{
	if (!g_oplus_chg) {
		chg_err("chip not ready\n");
		return 0;
	}
	if (!g_oplus_chg->external_gauge) {
		return DEF_SOC;
	} else {
		return 0;
	}
}

static int oplus_set_rtc_ui_soc(int backup_soc)
{
	return 0;
}

static bool oplus_bq25890h_is_usb_present(void)
{
	u8 val = 0;
	if (g_sy) {
		if (bq25890h_read_byte(g_sy, SY697X_REG_11, &val)) {
			chg_err("[%s] failed return false\n", __func__);
			return false;
		}
	} else {
		chg_err("[%s] g_sy is null return false\n", __func__);
		return false;
	}

	val = (val & SY697X_VBUS_GD_MASK);
	chg_debug("[%s] is %s\n", __func__,
		val ? "present" : "not present");
	return val;
}

static int oplus_bq25890h_get_vbus(void)
{
	int chg_vol = VBUS_DEF;
	if (g_sy)
		chg_vol = bq25890h_adc_read_vbus_volt(g_sy);
	else {
		chg_err("[%s] g_sy is null\n", __func__);
	}
	return chg_vol;
}

struct oplus_chgic_operations oplus_chgic_bq25890h_ops = {
	.typec_sink_removal = oplus_bq25890h_typec_sink_removal,
	.typec_src_removal = oplus_bq25890h_typec_src_removal,
	.typec_sink_insertion = oplus_bq25890h_typec_sink_insertion,
	.get_otg_switch_status = oplus_bq25890h_get_otg_switch_status,
	.set_otg_switch_status = oplus_bq25890h_set_otg_switch_status,
	.get_otg_online_status = oplus_bq25890h_get_otg_online_status,
	.thermal_tmp_get_chg = oplus_bq25890h_thermal_tmp_get_chg,
	.thermal_tmp_get_bb = oplus_bq25890h_thermal_tmp_get_bb,
	.thermal_tmp_get_flash = oplus_bq25890h_thermal_tmp_get_flash,
	.thermal_tmp_get_board = oplus_bq25890h_thermal_tmp_get_board,
	.get_usb_status = oplus_bq25890h_get_usb_status,
	.get_typec_cc_orientation = oplus_typec_cc_orientation_show,
};

struct oplus_chg_operations  oplus_chg_bq25890h_ops = {
	.dump_registers = oplus_bq25890h_dump_registers,
	.kick_wdt = oplus_bq25890h_kick_wdt,
	.hardware_init = oplus_bq25890h_hardware_init,
	.charging_current_write_fast = oplus_bq25890h_set_ichg,
	.set_aicl_point = oplus_bq25890h_set_mivr,
	.input_current_write = oplus_bq25890h_set_input_current_limit,
	.float_voltage_write = oplus_bq25890h_set_cv,
	.term_current_set = oplus_bq25890h_set_ieoc,
	.charging_enable = oplus_bq25890h_charging_enable,
	.charging_disable = oplus_bq25890h_charging_disable,
	.get_charging_enable = oplus_bq25890h_is_charging_enabled,
	.get_charger_current = bq25890h_adc_read_charge_current,
	.charger_suspend = oplus_bq25890h_charger_suspend,
	.charger_unsuspend = oplus_bq25890h_charger_unsuspend,
	.set_rechg_vol = oplus_bq25890h_set_rechg_vol,
	.reset_charger = oplus_bq25890h_reset_charger,
	.read_full = oplus_bq25890h_is_charging_done,
	.otg_enable = oplus_bq25890h_enable_otg,
	.otg_disable = oplus_bq25890h_disable_otg,
	.set_charging_term_disable = oplus_bq25890h_disable_te,
	.check_charger_resume = oplus_bq25890h_check_charger_resume,
	.get_charger_type = oplus_bq25890h_get_charger_type,
#ifdef CONFIG_OPLUS_CHARGER_MTK
	.get_charger_volt = battery_get_vbus,
	.get_chargerid_volt = NULL,
	.set_chargerid_switch_val = oplus_bq25890h_set_chargerid_switch_val,
	.get_chargerid_switch_val = oplus_bq25890h_get_chargerid_switch_val,
	.check_chrdet_status = (bool (*) (void)) pmic_chrdet_status,

	.get_boot_mode = (int (*)(void))get_boot_mode,
	.get_boot_reason = opluschg_get_boot_reason,
	.get_instant_vbatt = oplus_battery_meter_get_battery_voltage,
	.get_rtc_soc = oplus_get_rtc_ui_soc,
	.set_rtc_soc = oplus_set_rtc_ui_soc,
	.set_power_off = mt_power_off,
	.usb_connect = mt_usb_connect,
	.usb_disconnect = mt_usb_disconnect,
#else
	.get_charger_volt = oplus_bq25890h_get_vbus,
	.get_chargerid_volt		= NULL,
	.set_chargerid_switch_val = oplus_bq25890h_set_chargerid_switch,
	.get_chargerid_switch_val = oplus_bq25890h_get_chargerid_switch,
	.check_chrdet_status = (bool (*)(void))oplus_bq25890h_is_usb_present,

	.get_boot_mode = get_boot_mode,
	.get_boot_reason = (int (*)(void))opluschg_get_boot_reason,
	.get_instant_vbatt = opluschg_get_battery_voltage,
	.get_rtc_soc = oplus_get_rtc_ui_soc,
	.set_rtc_soc = oplus_set_rtc_ui_soc,
#endif /*CONFIG_OPLUS_CHARGER_MTK*/
	.get_chg_current_step = oplus_bq25890h_get_chg_current_step,
	.need_to_check_ibatt = oplus_bq25890h_need_to_check_ibatt,
	.get_dyna_aicl_result = oplus_bq25890h_get_dyna_aicl_result,
	.get_shortc_hw_gpio_status = oplus_bq25890h_get_shortc_hw_gpio_status,
	.oplus_chg_get_pd_type = oplus_bq25890h_get_pd_type,
	.oplus_chg_pd_setup = oplus_pd_setup,
	.check_pdphy_ready = oplus_check_pdphy_ready,
	.get_charger_subtype = oplus_bq25890h_get_charger_subtype,
	.set_qc_config = oplus_bq25890h_set_qc_config,
	.enable_qc_detect = oplus_bq25890h_enable_qc_detect,
	.oplus_chg_set_high_vbus = oplus_bq25890h_chg_set_high_vbus,
	.enable_shipmode = bq25890h_enable_shipmode,
	.get_usbtemp_volt = oplus_bq25890h_get_usbtemp_volt,
	.set_typec_sinkonly = oplus_set_typec_sinkonly,
	.set_typec_cc_open = oplus_set_typec_cc_open,
	.suspend_for_usbtemp = bq25890h_suspend_by_hz_mode,
	.oplus_usbtemp_monitor_condition = oplus_usbtemp_condition,
	.check_qchv_condition = oplus_chg_check_qchv_condition,
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
	oplus_bq25890h_set_aicr(g_sy->aicr);
}

static void bc12_retry_work_callback(struct work_struct *work)
{
	if (!g_sy)
		return;

	if (g_sy->sdp_retry || g_sy->cdp_retry) {
		Charger_Detect_Init();
		oplus_for_cdp();
		chg_debug("usb/cdp start bc1.2 once\n");
		g_sy->is_force_aicl = true;
		g_sy->is_retry_bc12 = true;
		bq25890h_force_dpdm(g_sy, true);
		g_sy->usb_connect_start = true;
	}
}

static struct of_device_id bq25890h_charger_match_table[] = {
	{.compatible = "ti, bq25890h", },
	{},
};

MODULE_DEVICE_TABLE(of, bq25890h_charger_match_table);

static const struct i2c_device_id bq25890h_i2c_device_id[] = {
	{ "bq25890h", 0x05 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, bq25890h_i2c_device_id);

static enum power_supply_property oplus_bq25890h_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_REAL_TYPE,
};

static int oplus_bq25890h_usb_set_prop(struct power_supply *psy,
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

int oplus_bq25890h_get_usb_online(struct sy697x *chg,
			union power_supply_propval *val)
{
	int rc;

	if (!chg) {
		val->intval = 0;
		return 0;
	}

	if (g_oplus_chg) {
		if (chg->power_good && (oplus_get_usb_status() != USB_TEMP_HIGH)
			&& ((chg->oplus_chg_type == POWER_SUPPLY_TYPE_USB) || (chg->oplus_chg_type == POWER_SUPPLY_TYPE_USB_CDP)))
			val->intval = 1;
		else
			val->intval = 0;
	}

	return rc;
}

bool opluschg_get_typec_cc_orientation(union power_supply_propval *val)
{
#ifdef CONFIG_TCPC_CLASS

	if (g_sy != NULL && g_sy->tcpc != NULL) {
		if (tcpm_inquire_typec_attach_state(g_sy->tcpc) != TYPEC_UNATTACHED) {
			typec_dir = (int)tcpm_inquire_cc_polarity(g_sy->tcpc) + 1;
		} else {
			typec_dir = 0;
		}
		if (typec_dir != 0)
			chg_err("[OPLUS_CHG][%s]: cc[%d]\n", __func__, val);
	} else {
		typec_dir = 0;
	}

#endif
	chg_err("typec_dir = %s  typec_dir = %d\n", typec_dir == 1 ? "cc1 attach" : "cc2_attach", typec_dir);
	val->intval = typec_dir;
	return typec_dir;
}

int oplus_typec_cc_orientation_show(void)
{
	union power_supply_propval prop_val;

	opluschg_get_typec_cc_orientation(&prop_val);
	return prop_val.intval = typec_dir;
}

static int oplus_bq25890h_usb_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int rc = 0;
	struct sy697x *chg = g_sy;
	val->intval = 0;

	if (!chg) {
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
		val->intval = oplus_bq25890h_is_usb_present();
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		oplus_bq25890h_get_usb_online(chg, val);
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

static int oplus_bq25890h_usb_prop_is_writeable(struct power_supply *psy,
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
	.properties = oplus_bq25890h_usb_props,
	.num_properties = ARRAY_SIZE(oplus_bq25890h_usb_props),
	.get_property = oplus_bq25890h_usb_get_prop,
	.set_property = oplus_bq25890h_usb_set_prop,
	.property_is_writeable = oplus_bq25890h_usb_prop_is_writeable,
};

static enum power_supply_property oplus_bq25890h_batt_props[] = {
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

static int oplus_bq25890h_batt_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		if (g_oplus_chg && (g_oplus_chg->ui_soc == 0)) {
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
			chg_err("bat pro POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL, should shutdown!!!\n");
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = g_oplus_chg->batt_fcc * 1000;
		break;

	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = g_oplus_chg->ui_soc * g_oplus_chg->batt_capacity_mah * SY697X_ICH_1000 / 100;
		break;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		val->intval = 0;
		break;

	default:
		rc = oplus_battery_get_property(psy, psp, val);
	}
	if (rc < 0) {
		return -ENODATA;
	}
	return rc;
}

static int oplus_bq25890h_batt_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	int rc = 0;
	rc = oplus_battery_set_property(psy, prop, val);
	return rc;
}

static int oplus_bq25890h_batt_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	return oplus_battery_property_is_writeable(psy, psp);
}

static struct power_supply_desc batt_psy_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = oplus_bq25890h_batt_props,
	.num_properties = ARRAY_SIZE(oplus_bq25890h_batt_props),
	.get_property = oplus_bq25890h_batt_get_prop,
	.set_property = oplus_bq25890h_batt_set_prop,
	.property_is_writeable = oplus_bq25890h_batt_prop_is_writeable,
};

 static enum power_supply_property ac_props[] = {
/*OPLUS own ac props*/
        POWER_SUPPLY_PROP_ONLINE,
};

static int oplus_bq25890h_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int rc = 0;

    rc = oplus_ac_get_property(psy, psp, val);

	return rc;
}

static struct power_supply_desc ac_psy_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = ac_props,
	.num_properties = ARRAY_SIZE(ac_props),
	.get_property = oplus_bq25890h_ac_get_property,
};
static enum power_supply_property charger_props[] = {
        POWER_SUPPLY_PROP_ONLINE,
};
static int oplus_bq25890h_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int rc = 0;
	struct sy697x *chg = g_sy;
	unsigned long cur_chg_time = 0;
	val->intval = 0;

	if (!chg) {
		chg_err("oplus_bq25890h_usb_get_prop return invalid\n");
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (chg->power_good
		  && (chg->oplus_chg_type == POWER_SUPPLY_TYPE_USB_DCP))
			val->intval = 1;
		else
			val->intval = 0;

                if (qpnp_is_power_off_charging() == true && (oplus_chg_get_curr_time_ms(&cur_chg_time) < 15000)) {
                        chg_err("oplus_bq25890h_usb_get_prop power_off_charging\n");
			val->intval = 1;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static struct power_supply_desc charger_psy_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = charger_props,
	.num_properties = ARRAY_SIZE(charger_props),
	.get_property = oplus_bq25890h_charger_get_property,
};
static int oplus_power_supply_init(struct oplus_chg_chip *chip)
{
	int ret = 0;
	struct oplus_chg_chip *chgchip = NULL;

	if (chip == NULL) {
		chg_err("[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}
	chgchip = chip;

	chgchip->ac_psd = ac_psy_desc;
	chgchip->ac_cfg.drv_data = chgchip;
	chgchip->usb_psd = usb_psy_desc;
	chgchip->usb_cfg.drv_data = chgchip;
	chgchip->battery_psd = batt_psy_desc;
	charger_psd = charger_psy_desc;
	charger_cfg.drv_data = chgchip;

	chgchip->ac_psy = power_supply_register(chgchip->dev, &chgchip->ac_psd,
			&chgchip->ac_cfg);
	if (IS_ERR(chgchip->ac_psy)) {
		chg_err("Failed to register power supply ac: %ld\n",
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
		chg_err("Failed to register power supply battery: %ld\n",
			PTR_ERR(chgchip->batt_psy));
		ret = PTR_ERR(chgchip->batt_psy);
		goto err_battery_psy;
	}

	charger_psy = power_supply_register(chgchip->dev, &charger_psd,
			&charger_cfg);
	if (IS_ERR(charger_psy)) {
		chg_err("Failed to register power supply ac: %ld\n",
			PTR_ERR(charger_psy));
		ret = PTR_ERR(charger_psy);
		goto err_charger_psy;
	}
	chg_err("%s OK\n", __func__);
	return 0;

err_charger_psy:
        power_supply_unregister(chgchip->batt_psy);
err_battery_psy:
	power_supply_unregister(chgchip->usb_psy);
err_usb_psy:
	power_supply_unregister(chgchip->ac_psy);
err_ac_psy:

	return ret;
}
struct sy697x* oplus_bq25890h_get_oplus_chg(void)
{
	return g_sy;
}

struct oplus_chg_chip* oplus_get_oplus_chip(void)
{
	return g_oplus_chg;
}

static int bq25890h_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct sy697x *sy;
	struct device_node *node = client->dev.of_node;
	int ret = 0;
	struct oplus_chg_chip *oplus_chip = NULL;
	int level = 0;
	struct smb5 *chip;
	struct smb_charger *chg;

	chg_err("lkl rm dr_mode bq25890h probe enter\n");
	if (oplus_gauge_check_chip_is_null()) {
		chg_err("gauge chip null, will do after bettery init.\n");
		return -EPROBE_DEFER;
	}
	sy = devm_kzalloc(&client->dev, sizeof(struct sy697x), GFP_KERNEL);
	if (!sy)
		return -ENOMEM;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	sy->dev = &client->dev;
	sy->client = client;
#ifdef CONFIG_TCPC_CLASS
	sy->tcpc = tcpc_dev_get_by_name("type_c_port0");
#endif
	g_sy = sy;
#ifdef CONFIG_OPLUS_CHARGER_MTK
	sy->chg_consumer =
		charger_manager_get_by_name(&client->dev, "bq25890h");
#endif /*CONFIG_OPLUS_CHARGER_MTK*/
	i2c_set_clientdata(client, sy);
	mutex_init(&sy->i2c_rw_lock);
	mutex_init(&sy->ntc_lock);
	mutex_init(&sy->dpdm_lock);

	atomic_set(&sy->driver_suspended, 0);
	atomic_set(&sy->charger_suspended, 0);
	sy->before_suspend_icl = 0;
	sy->before_unsuspend_icl = 0;

	oplus_is_prswap = false;

	ret = bq25890h_detect_device(sy);
	if (ret) {
		chg_err("No bq25890h device found!\n");
		ret = -ENODEV;
		goto err_nodev;
	}
        sy->chgic_ops = &oplus_chgic_bq25890h_ops;
	sy->platform_data = bq25890h_parse_dt(node, sy);
	if (!sy->platform_data) {
		chg_err("No platform data provided.\n");
		ret = -EINVAL;
		goto err_parse_dt;
	}

	bq25890h_reset_chip(sy);
	if ( is_bq25890h(sy) == false) {
		ret = bq25890h_init_device(sy);
	} else {
		ret = bq2589x_init_device(sy);
	}
	if (ret) {
		chg_err("Failed to init device\n");
		goto err_init;
	}
	charger_type_thread_init();
	sy->is_bc12_end = true;

	sy->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	sy->pre_current_ma = -1;
	sy->chg_start_check = true;
	sy->usb_connect_start = false;
	if (is_bq25890h(sy) == true) {
		bq25890h_enable_hvdcp(sy);
		bq2589x_disable_maxc(sy);
	} else {
		bq25890h_disable_hvdcp(sy);
		bq2589x_disable_maxc(sy);
	}
	bq25890h_disable_batfet_rst(sy);
	bq25890h_disable_ico(sy);

	INIT_DELAYED_WORK(&sy->sy697x_aicl_work, aicl_work_callback);
	INIT_DELAYED_WORK(&sy->sy697x_bc12_retry_work, bc12_retry_work_callback);
	INIT_DELAYED_WORK(&sy->sy697x_vol_convert_work, vol_convert_work);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	sy->chg_dev = charger_device_register(sy->chg_dev_name,
					      &client->dev, sy,
					      &bq25890h_chg_ops,
					      &bq25890h_chg_props);
	if (IS_ERR_OR_NULL(sy->chg_dev)) {
		ret = PTR_ERR(sy->chg_dev);
		goto err_device_register;
	}

	ret = sysfs_create_group(&sy->dev->kobj, &bq25890h_attr_group);
	if (ret) {
		/*dev_err(sy->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_sysfs_create;*/
	}
#endif /*CONFIG_OPLUS_CHARGER_MTK*/

	if ( is_bq25890h(sy) == false) {
		chg_err("bq25890h\n");
	} else {
		chg_err("bq25890h\n");
	}

	/*oplus_chip register*/
	oplus_chip = devm_kzalloc(&client->dev, sizeof(*oplus_chip), GFP_KERNEL);
	if (!oplus_chip) {
		chg_err("oplus_chip null, will do after bettery init.\n");
		return -ENOMEM;
	}

	oplus_chip->dev = &client->dev;
	oplus_chg_parse_svooc_dt(oplus_chip);


	g_oplus_chg = oplus_chip;

	oplus_chip->pmic_spmi.smb5_chip = chip;
	chg = &chip->chg;
	chg->dev = &client->dev;
	mutex_init(&chg->pinctrl_mutex);
	oplus_chip->chg_ops = &oplus_chg_bq25890h_ops;
	g_oplus_chg->sub_chg_ops = &oplus_chg_sgm4151x_ops;
	g_oplus_chg->is_double_charger_support = true;
	oplus_power_supply_init(oplus_chip);
	opluschg_parse_custom_dt(oplus_chip);
	oplus_chg_parse_charger_dt(oplus_chip);

	oplus_chip->con_volt = con_volt_855;
	oplus_chip->con_temp = con_temp_855;
	oplus_chip->len_array = ARRAY_SIZE(con_temp_855);

	/*platform init*/
	oplus_chg_plt_init_for_qcom(sy);

	/*add extcon register for usb emulation*/
	oplus_register_extcon(sy);

	oplus_chg_init(oplus_chip);
	bq25890h_register_interrupt(node, sy);

	if (oplus_ccdetect_support_check() == true) {
		INIT_DELAYED_WORK(&ccdetect_work,oplus_ccdetect_work);
		INIT_DELAYED_WORK(&usbtemp_recover_work,oplus_usbtemp_recover_work);
		oplus_ccdetect_irq_register(oplus_chip);
		level = gpio_get_value(sy->ccdetect_gpio);
		usleep_range(DELAY2000, DELAY2100);
		if (level != gpio_get_value(sy->ccdetect_gpio)) {
			chg_debug("[OPLUS_CHG][%s]: ccdetect_gpio is unstable, try again...\n", __func__);
			usleep_range(DELAY10000, DELAY11000);
			level = gpio_get_value(sy->ccdetect_gpio);
		}
		if (level <= 0) {
			schedule_delayed_work(&ccdetect_work, msecs_to_jiffies(CC_WORKTIME));
		}
		chg_debug("[OPLUS_CHG][%s]: ccdetect_gpio ..level[%d]  \n", __func__, level);
	}

	oplus_chg_configfs_init(oplus_chip);
	opluschg_usbtemp_thread_init(oplus_chip);
	oplus_tbatt_power_off_task_init(oplus_chip);
	bq25890h_irq_handler(0, sy);

	hardwareinfo_set_prop(HARDWARE_CHARGER_IC,"bq25890");

	chg_err("bq25890h probe successfully Part Num:%d, Revision:%d\n!", sy->part_no, sy->revision);
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


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int bq25890h_pm_resume(struct device *dev)
{
	struct sy697x *chip = NULL;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip) {
			atomic_set(&chip->driver_suspended, 0);
		}
	}
	return 0;
}
static int bq25890h_pm_suspend(struct device *dev)
{
	struct sy697x *chip = NULL;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip) {
			atomic_set(&chip->driver_suspended, 1);
		}
	}
	return 0;
}

static const struct dev_pm_ops bq25890h_pm_ops = {
	.resume		  = bq25890h_pm_resume,
	.suspend		  = bq25890h_pm_suspend,
};
#endif

static int bq25890h_charger_remove(struct i2c_client *client)
{
	struct sy697x *sy = i2c_get_clientdata(client);

	mutex_destroy(&sy->i2c_rw_lock);

	return 0;
}

static void bq25890h_charger_shutdown(struct i2c_client *client)
{
	int ret;
	if (g_sy) {
		bq25890h_adc_start(g_sy, false);
		if (!is_bq25890h(g_sy)) {
			bq25890h_disable_otg(g_sy);
		}
		pr_err("bq25890h_charger_shutdown disable adc and otg\n!");
	if (g_oplus_chg && g_oplus_chg->enable_shipmode) {
		printk("Set ship mode: %d!!\n", !!g_oplus_chg->enable_shipmode);
		ret = bq25890h_enter_ship_mode(g_sy, g_oplus_chg->enable_shipmode ? 1 : 0);
		if (ret < 0) {
			pr_err("bq25890h_charger_shutdown set enable_shipmode fail\n!");
		}
	}
	}
}

static struct i2c_driver bq25890h_charger_driver = {
	.driver = {
		   .name = "bq25890h-charger",
		   .owner = THIS_MODULE,
		   .of_match_table = bq25890h_charger_match_table,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		   .pm	  = &bq25890h_pm_ops,
#endif
		   },

	.probe = bq25890h_charger_probe,
	.remove = bq25890h_charger_remove,
	.shutdown = bq25890h_charger_shutdown,
	.id_table = bq25890h_i2c_device_id,

};

module_i2c_driver(bq25890h_charger_driver);

MODULE_DESCRIPTION("BQ25890h Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("bq25890h-charger");
