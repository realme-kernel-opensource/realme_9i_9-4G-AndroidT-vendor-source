/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <asm/unaligned.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/power/sm5602_fg.h>
#include <linux/iio/consumer.h>
#include "../oplus_gauge.h"
#include "../oplus_charger.h"
#include <linux/hardware_info.h>
#define	INVALID_REG_ADDR	0xFF
#define   RET_ERR -1
#define NUM0 0
#define NUM1 1
#define NUM2 2
#define NUM3 3
#define NUM4 4

enum sm_fg_reg_idx {
        SM_FG_REG_DEVICE_ID = NUM0,
        SM_FG_REG_CNTL,
        SM_FG_REG_INT,
        SM_FG_REG_INT_MASK,
        SM_FG_REG_STATUS,
        SM_FG_REG_SOC,
        SM_FG_REG_OCV,
        SM_FG_REG_VOLTAGE,
        SM_FG_REG_CURRENT,
        SM_FG_REG_TEMPERATURE_IN,
        SM_FG_REG_TEMPERATURE_EX,
        SM_FG_REG_V_L_ALARM,
        SM_FG_REG_V_H_ALARM,
        SM_FG_REG_A_H_ALARM,
        SM_FG_REG_T_IN_H_ALARM,
        SM_FG_REG_SOC_L_ALARM,
        SM_FG_REG_FG_OP_STATUS,
        SM_FG_REG_TOPOFFSOC,
        SM_FG_REG_PARAM_CTRL,
        SM_FG_REG_SHUTDOWN,
        SM_FG_REG_VIT_PERIOD,
        SM_FG_REG_CURRENT_RATE,
        SM_FG_REG_BAT_CAP,
        SM_FG_REG_CURR_OFFSET,
        SM_FG_REG_CURR_SLOPE,
        SM_FG_REG_MISC,
        SM_FG_REG_RESET,
        SM_FG_REG_RSNS_SEL,
        SM_FG_REG_VOL_COMP,
        NUMREGS,
};

static u8 sm5602_regs[NUMREGS] = {
        0x00,
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x08,
        0x09,
        0x0A,
        0x0C,
        0x0D,
        0x0E,
        0x0F,
        0x10,
        0x11,
        0x12,
        0x13,
        0x14,
        0x1A,
        0x1B,
        0x62,
        0x73,
        0x74,
        0x90,
        0x91,
        0x95,
        0x96,
};

enum sm_fg_device {
        SM5602,
};

enum sm_fg_temperature_type {
        TEMPERATURE_IN = NUM0,
        TEMPERATURE_EX,
};

const unsigned char *device2str[] = {
        "sm5602",
};

enum battery_table_type {
        BATTERY_TABLE0 = NUM0,
        BATTERY_TABLE1,
        BATTERY_TABLE2,
        BATTERY_TABLE_MAX,
};

struct sm_fg_chip;
struct sm_fg_chip *global_sm = NULL;

struct sm_fg_chip {
        struct device		*dev;
        struct i2c_client	*client;
        struct mutex i2c_rw_lock;
        struct mutex data_lock;
        u8 chip;
        u8 regs[NUMREGS];
        int	batt_id;
        int gpio_int;

        bool batt_present;
        bool batt_fc;
        bool batt_ot;
        bool batt_ut;
        bool batt_soc1;
        bool batt_socp;
        bool batt_dsg;
        int	batt_soc;
        int batt_ocv;
        int batt_fcc;
        int	batt_volt;
        int	aver_batt_volt;
        int	batt_temp;
        int	batt_curr;
        int is_charging;
        int batt_soc_cycle;
        int topoff_soc;
        int top_off;
        int iocv_error_count;

        int p_batt_voltage;
        int p_batt_current;

        bool en_temp_ex;
        bool en_temp_in;
        bool en_batt_det;
        bool iocv_man_mode;
        int aging_ctrl;
        int batt_rsns;
        int cycle_cfg;
        int fg_irq_set;
        int low_soc1;
        int low_soc2;
        int v_l_alarm;
        int v_h_alarm;
        int battery_table_num;
        int misc;
        int batt_v_max;
        int min_cap;
        u32 common_param_version;
        int t_l_alarm_in;
        int t_h_alarm_in;
        u32 t_l_alarm_ex;
        u32 t_h_alarm_ex;

        int battery_table[BATTERY_TABLE_MAX][FG_TABLE_LEN];
        signed short battery_temp_table[FG_TEMP_TABLE_CNT_MAX];
        int alpha;
        int beta;
        int rs;
        int rs_value[NUM4];
        int vit_period;
        int mix_value;
        const char		*battery_type;
        int volt_cal;
        int curr_offset;
        int curr_slope;
        int cap;
        int n_tem_poff;
        int n_tem_poff_offset;
        int batt_max_voltage_uv;
        int temp_std;
        int en_high_fg_temp_offset;
        int high_fg_temp_offset_denom;
        int high_fg_temp_offset_fact;
        int en_low_fg_temp_offset;
        int low_fg_temp_offset_denom;
        int low_fg_temp_offset_fact;
        int en_high_fg_temp_cal;
        int high_fg_temp_p_cal_denom;
        int high_fg_temp_p_cal_fact;
        int high_fg_temp_n_cal_denom;
        int high_fg_temp_n_cal_fact;
        int en_low_fg_temp_cal;
        int low_fg_temp_p_cal_denom;
        int low_fg_temp_p_cal_fact;
        int low_fg_temp_n_cal_denom;
        int low_fg_temp_n_cal_fact;
        int	en_high_temp_cal;
        int high_temp_p_cal_denom;
        int high_temp_p_cal_fact;
        int high_temp_n_cal_denom;
        int high_temp_n_cal_fact;
        int en_low_temp_cal;
        int low_temp_p_cal_denom;
        int low_temp_p_cal_fact;
        int low_temp_n_cal_denom;
        int low_temp_n_cal_fact;
        u32 battery_param_version;

        struct delayed_work monitor_work;

        int	skip_reads;
        int	skip_writes;
        int fake_soc;
        int fake_temp;
        struct dentry *debug_root;
        struct power_supply *batt_psy;
        struct iio_channel	*ntc_switch1_chan;
};


static int show_registers(struct seq_file *m, void *data);
static bool fg_init(struct i2c_client *client);
static int fg_read_current(struct sm_fg_chip *sm);


static int __fg_read_word(struct i2c_client *client, u8 reg, u16 *val)
{
        s32 ret;

        ret = i2c_smbus_read_word_data(client, reg);
        if (ret < NUM0) {
                chg_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
                return ret;
        }

        *val = (u16)ret;

        return NUM0;
}

static int __fg_write_word(struct i2c_client *client, u8 reg, u16 val)
{
        s32 ret;

        ret = i2c_smbus_write_word_data(client, reg, val);
        if (ret < NUM0) {
                chg_err("i2c write word fail: can't write 0x%02X to reg 0x%02X\n",
                                val, reg);
                return ret;
        }

        return NUM0;
}

static int fg_read_word(struct sm_fg_chip *sm, u8 reg, u16 *val)
{
        int ret;

        if (sm->skip_reads) {
                *val = NUM0;
                return NUM0;
        }

        mutex_lock(&sm->i2c_rw_lock);
        ret = __fg_read_word(sm->client, reg, val);
        mutex_unlock(&sm->i2c_rw_lock);

        return ret;
}

static int fg_write_word(struct sm_fg_chip *sm, u8 reg, u16 val)
{
        int ret;

        if (sm->skip_writes) {
                return NUM0;
        }
        mutex_lock(&sm->i2c_rw_lock);
        ret = __fg_write_word(sm->client, reg, val);
        mutex_unlock(&sm->i2c_rw_lock);

        return ret;
}

#define FG_STATUS_SLEEP				BIT(10)
#define FG_STATUS_BATT_PRESENT		BIT(9)
#define FG_STATUS_SOC_UPDATE		BIT(8)
#define FG_STATUS_TOPOFF			BIT(7)
#define FG_STATUS_LOW_SOC2			BIT(6)
#define FG_STATUS_LOW_SOC1			BIT(5)
#define FG_STATUS_HIGH_CURRENT		BIT(4)
#define FG_STATUS_HIGH_TEMPERATURE	BIT(3)
#define FG_STATUS_LOW_TEMPERATURE	BIT(2)
#define FG_STATUS_HIGH_VOLTAGE		BIT(1)
#define FG_STATUS_LOW_VOLTAGE		BIT(0)

#define FG_OP_STATUS_CHG_DISCHG		BIT(15)

static int fg_read_status(struct sm_fg_chip *sm)
{
        int ret;
        u16 flags1, flags2;

        ret = fg_read_word(sm, sm->regs[SM_FG_REG_STATUS], &flags1);
        if (ret < NUM0) {
                return ret;
        }

        ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &flags2);
        if (ret < NUM0)
                return ret;

        mutex_lock(&sm->data_lock);
        /*sm->batt_present	= !!(flags1 & FG_STATUS_BATT_PRESENT);*/
        sm->batt_present	= 1;
        sm->batt_ot			= !!(flags1 & FG_STATUS_HIGH_TEMPERATURE);
        sm->batt_ut			= !!(flags1 & FG_STATUS_LOW_TEMPERATURE);
        sm->batt_fc			= !!(flags1 & FG_STATUS_TOPOFF);
        sm->batt_soc1		= !!(flags1 & FG_STATUS_LOW_SOC2);
        sm->batt_socp		= !!(flags1 & FG_STATUS_LOW_SOC1);
        sm->batt_dsg		= !!!(flags2 & FG_OP_STATUS_CHG_DISCHG);
        mutex_unlock(&sm->data_lock);

        return NUM0;
}

#define SOC_NUM0x7f00 0x7f00
#define SOC_NUM8	8
#define MULTIPLY_10 10
#define SOC_NUM0x00ff 0x00ff
#define SOC_NUM256 256
#define SOC_ERROR_FLAG 0x8000
#define SOC_ERROR -1

static int fg_read_soc(struct sm_fg_chip *sm)
{
        int ret;
        unsigned int soc = NUM0;
        u16 data = NUM0;

        ret = fg_read_word(sm, sm->regs[SM_FG_REG_SOC], &data);
        if (ret < NUM0) {
                chg_err("could not read SOC, ret = %d\n", ret);
                return ret;
        } else {
                soc = ((data&SOC_NUM0x7f00) >> SOC_NUM8) * MULTIPLY_10;
                soc = soc + (((data&SOC_NUM0x00ff)*MULTIPLY_10)/SOC_NUM256);

                if (data & SOC_ERROR_FLAG) {
                        soc *= SOC_ERROR;
                }
        }

        return soc;
}

#define DEFAULT_OCV 4000
#define OCV_NUM0x0fff 0x0fff
#define MULTIPLY_1000 1000
#define OCV_NUM11	11
#define OCV_NUM2048	2048
#define OCV_NUM0Xf000 0xf000

static unsigned int fg_read_ocv(struct sm_fg_chip *sm)
{
        int ret;
        u16 data = NUM0;
        unsigned int ocv;

        ret = fg_read_word(sm, sm->regs[SM_FG_REG_OCV], &data);
        if (ret < NUM0) {
                chg_err("could not read OCV, ret = %d\n", ret);
                ocv = DEFAULT_OCV;
        } else {
                ocv = ((data&OCV_NUM0x0fff)*MULTIPLY_1000)/OCV_NUM2048;
                ocv += ((data&OCV_NUM0Xf000) >> OCV_NUM11)*MULTIPLY_1000;
        }

        return ocv;
}

#define TEMP_REG_MAX 0x823B
#define TEMP_REG_MIN 0x8001
#define TEMP_EX_ERROR 0x0000
#define TEMP_EX_MAX 87
#define TEMP_EX_MIN -20
#define TEMP_SUB 1
#define UOHM 7300

static int _calculate_battery_temp_ex(struct sm_fg_chip *sm, u16 uval)
{
        int i = NUM0, temp = NUM0;
        signed short val = NUM0;

        if ((uval >= TEMP_REG_MIN) && (uval <= TEMP_REG_MAX)) {
                chg_debug("sp_range uval = 0x%x\n", uval);
                uval = TEMP_EX_ERROR;
        }

        val = uval;

        if (val >= sm->battery_temp_table[NUM0]) {
                temp = TEMP_EX_MIN;
        } else if (val <= sm->battery_temp_table[FG_TEMP_TABLE_CNT_MAX-TEMP_SUB]) {
                temp = TEMP_EX_MAX;
        } else {
                for (i = NUM0; i < FG_TEMP_TABLE_CNT_MAX; i++) {
                        if  (val >= sm->battery_temp_table[i]) {
                                temp = TEMP_EX_MIN + i;
                                if ((temp >= TEMP_SUB) && (val != sm->battery_temp_table[i]))
                                        temp = temp -TEMP_SUB;
                                break;
                        }
                }
        }

        return temp;
}

#define TEMP_NUM1 0x00FF
#define TEMP_NUM2 0x8000
#define TEMP_ERROR -1

static int fg_read_temperature(struct sm_fg_chip *sm, enum sm_fg_temperature_type temperature_type)
{
        int ret, temp = NUM0;
        u16 data = NUM0;

        switch (temperature_type) {
        case TEMPERATURE_IN:
                ret = fg_read_word(sm, sm->regs[SM_FG_REG_TEMPERATURE_IN], &data);
                if (ret < NUM0) {
                        chg_err("could not read temperature in , ret = %d\n", ret);
                        return ret;
                } else {
                        temp = ((data & TEMP_NUM1));
                        if (data & TEMP_NUM2)
                                temp *= TEMP_ERROR;
                }

                break;
        case TEMPERATURE_EX:
                ret = fg_read_word(sm, sm->regs[SM_FG_REG_TEMPERATURE_EX], &data);
                if (ret < NUM0) {
                        chg_err("could not read temperature ex , ret = %d\n", ret);
                        return ret;
                } else {
                        temp = _calculate_battery_temp_ex(sm, data);
                }

                break;

        default:
                return -EINVAL;
        }

        return temp;
}

#define VOLT_1_8 1800
#define VOLT_NUM0x7FFF 0x7FFF
#define VOLT_NUM19622 19622
#define VOLT_NUM0X8000 0x8000
#define VOLT_ERROR -1
#define VOLT_BASE 2700
#define VOLT_MULTIPLY 4
#define VOLT_DIVISION 5

static int fg_read_volt(struct sm_fg_chip *sm)
{
        int ret = NUM0;
        int volt = NUM0;
        u16 data = NUM0;

        ret = fg_read_word(sm, sm->regs[SM_FG_REG_VOLTAGE], &data);
        if (ret < NUM0) {
                chg_err("could not read voltage, ret = %d\n", ret);
                return ret;
        }  else {
                volt = VOLT_1_8 * (data & VOLT_NUM0x7FFF) / VOLT_NUM19622;
                if (data&VOLT_NUM0X8000)
                        volt *= VOLT_ERROR;

                volt += VOLT_BASE;
        }

        sm->aver_batt_volt = (((sm->aver_batt_volt)*VOLT_MULTIPLY) + volt)/VOLT_DIVISION;

        return volt;
}

#define CYCLE_NUM0x01FF 0x01FF

static int fg_get_cycle(struct sm_fg_chip *sm)
{
        int ret;
        int cycle;
        u16 data = NUM0;

        ret = fg_read_word(sm, FG_REG_SOC_CYCLE, &data);
        if (ret < NUM0) {
                chg_err("read cycle reg fail ret = %d\n", ret);
                cycle = NUM0;
        } else {
                cycle = data&CYCLE_NUM0x01FF;
        }

        return cycle;
}

#define RSNS_DEFAULT 10
#define RSNS_0 0
#define RSNS_5 5
#define CURR_NUM0x7FFF 0x7FFF
#define CURR_NUM1250 1250
#define CURR_NUM511 511
#define CURR_NUM0X8000 0x8000
#define CURR_ERROR -1

static int fg_read_current(struct sm_fg_chip *sm)
{
        int ret = NUM0, rsns = NUM0;
        u16 data = NUM0;
        int curr = NUM0;

        ret = fg_read_word(sm, sm->regs[SM_FG_REG_CURRENT], &data);
        if (ret < NUM0) {
                chg_err("could not read current, ret = %d\n", ret);
                return ret;
        } else {
                if (sm->batt_rsns == -EINVAL) {
                        chg_err("could not read sm->batt_rsns, rsns = 10mohm\n");
                        rsns = RSNS_DEFAULT;
                } else {
                        sm->batt_rsns == RSNS_0 ? rsns = RSNS_5 : (rsns = sm->batt_rsns*MULTIPLY_10);
                }

                curr = ((data & CURR_NUM0x7FFF) * CURR_NUM1250 / CURR_NUM511 / rsns);

                if (data & CURR_NUM0X8000)
                        curr *= CURR_ERROR;
        }

        return curr;
}

#define NUM0x7FFF 0x7FFF
#define NUM2048 2048

static int fg_read_fcc(struct sm_fg_chip *sm)
{
        int ret = NUM0;
        int fcc = NUM0;
        u16 data = NUM0;
        int64_t temp = NUM0;

        ret = fg_read_word(sm, sm->regs[SM_FG_REG_BAT_CAP], &data);
        if (ret < NUM0) {
                chg_err("could not read FCC, ret=%d\n", ret);
                return ret;
        } else {
                temp = div_s64((data & NUM0x7FFF) * MULTIPLY_1000, NUM2048);
                fcc = temp;
        }

        return fcc;
}

#define FG_SOFT_RESET	0xA6
#define MSLEEP_600 600
static int fg_reset(struct sm_fg_chip *sm)
{
        int ret;

        ret = fg_write_word(sm, sm->regs[SM_FG_REG_RESET], FG_SOFT_RESET);
        if (ret < NUM0) {
                chg_err("could not reset, ret=%d\n", ret);
                return ret;
        }

        msleep(MSLEEP_600);

        return NUM0;
}

static int get_battery_status(struct sm_fg_chip *sm)
{
        union power_supply_propval ret = {NUM0, };
        int rc;

        if (sm->batt_psy == NULL) {
                sm->batt_psy = power_supply_get_by_name("battery");
        }
        if (sm->batt_psy) {
                rc = sm->batt_psy->desc->get_property(sm->batt_psy,
                                        POWER_SUPPLY_PROP_STATUS, &ret);
                if (rc) {
                        chg_err("Battery does not export status: %d\n", rc);
                        return POWER_SUPPLY_STATUS_UNKNOWN;
                }
                return ret.intval;
        }

        return POWER_SUPPLY_STATUS_UNKNOWN;
}

static bool is_battery_charging(struct sm_fg_chip *sm)
{
        return get_battery_status(sm) == POWER_SUPPLY_STATUS_CHARGING;
}

#define BATT_CURR_50 50
#define TOP_OFF_NUM3 3
#define BATT_SOC_NUM900 900
#define BATT_VOLT_30 30
#define BATT_VOLT_15 15
#define ERROR_COUNT_5 5
#define ERROR_COUNT_6 6
#define ERROR_COUNT_0 0
#define SHIFT_1 1
#define ARRAY_ELEMENT_0 0
#define ARRAY_ELEMENT_1 1
#define ARRAY_ELEMENT_2 2
static void fg_vbatocv_check(struct sm_fg_chip *sm)
{
        if ((abs(sm->batt_curr) < BATT_CURR_50) ||
                ((sm->is_charging) && (sm->batt_curr < (sm->top_off)) &&
                (sm->batt_curr > (sm->top_off / TOP_OFF_NUM3)) && (sm->batt_soc >=
                BATT_SOC_NUM900))) {
                if (abs(sm->batt_ocv-sm->batt_volt) > BATT_VOLT_30) {
                        sm->iocv_error_count++;
                }


                if (sm->iocv_error_count > ERROR_COUNT_5) {
                        sm->iocv_error_count = ERROR_COUNT_6;
                }
        } else {
                sm->iocv_error_count = ERROR_COUNT_0;
        }

        if (sm->iocv_error_count > ERROR_COUNT_5) {
                if (abs(sm->p_batt_voltage - sm->batt_volt) > BATT_VOLT_15) {
                        sm->iocv_error_count = ERROR_COUNT_0;
                } else {
                        fg_write_word(sm, FG_REG_RS_2, sm->rs_value[ARRAY_ELEMENT_0]);
                }
        } else {
                if ((sm->p_batt_voltage < sm->n_tem_poff) &&
                        (sm->batt_volt < sm->n_tem_poff) && (!sm->is_charging)) {
                        if ((sm->p_batt_voltage <
                                (sm->n_tem_poff - sm->n_tem_poff_offset)) &&
                                (sm->batt_volt <
                                (sm->n_tem_poff - sm->n_tem_poff_offset))) {
                                fg_write_word(sm, FG_REG_RS_2, sm->rs_value[ARRAY_ELEMENT_0] >> SHIFT_1);
                        } else {
                                fg_write_word(sm, FG_REG_RS_2, sm->rs_value[ARRAY_ELEMENT_0]);
                        }
                }
                else
                {
                        fg_write_word(sm, FG_REG_RS_2, sm->rs_value[ARRAY_ELEMENT_2]);
                }
        }
        sm->p_batt_voltage = sm->batt_volt;
        sm->p_batt_current = sm->batt_curr;
}

#define TEMP_CURR_FLAG 0x0080
#define OFFEST_0x007F 0x007F
#define TEMP_CURR_OFFEST_RIGHT 0
#define FG_TEMP_GAP_0 0
#define SHIFT_8 8
#define CURR_SLOPE_0xFF00 0xFF00
#define CURR_SLOPE_0x00FF 0x00FF
#define REG_0X28 0x28
#define REG_0X82 0x82
#define REG_0X83 0x83
#define RETURN_1 1
static int fg_cal_carc (struct sm_fg_chip *sm)
{
        int curr_cal = NUM0, p_curr_cal = NUM0;
        int n_curr_cal = NUM0, p_delta_cal = NUM0;
        int n_delta_cal = NUM0, p_fg_delta_cal = NUM0;
        int n_fg_delta_cal = NUM0, temp_curr_offset = NUM0;
        int temp_gap, fg_temp_gap = NUM0;
        int ret = NUM0;
        u16 data[NUM3] = {NUM0, };

        fg_vbatocv_check(sm);

        sm->is_charging = is_battery_charging(sm);

        fg_temp_gap = sm->batt_temp - sm->temp_std;

        temp_curr_offset = sm->curr_offset;
        if (sm->en_high_fg_temp_offset && (fg_temp_gap > FG_TEMP_GAP_0)) {
                if (temp_curr_offset & TEMP_CURR_FLAG) {
                        temp_curr_offset = -(temp_curr_offset & OFFEST_0x007F);
                }
                temp_curr_offset += (fg_temp_gap / sm->high_fg_temp_offset_denom)*sm->high_fg_temp_offset_fact;
                if (temp_curr_offset < TEMP_CURR_OFFEST_RIGHT) {
                        temp_curr_offset = -temp_curr_offset;
                        temp_curr_offset = temp_curr_offset|TEMP_CURR_FLAG;
                }
        }
        else if (sm->en_low_fg_temp_offset && (fg_temp_gap < FG_TEMP_GAP_0)) {
                if (temp_curr_offset & TEMP_CURR_FLAG) {
                        temp_curr_offset = -(temp_curr_offset & OFFEST_0x007F);
                }
                temp_curr_offset += ((-fg_temp_gap) / sm->low_fg_temp_offset_denom)*sm->low_fg_temp_offset_fact;
                if (temp_curr_offset < FG_TEMP_GAP_0) {
                        temp_curr_offset = -temp_curr_offset;
                        temp_curr_offset = temp_curr_offset|TEMP_CURR_FLAG;
                }
        }
        temp_curr_offset = temp_curr_offset | (temp_curr_offset << SHIFT_8);
        ret = fg_write_word(sm, FG_REG_CURR_IN_OFFSET, temp_curr_offset);
        if (ret < FG_TEMP_GAP_0) {
                chg_err("Failed to write CURR_IN_OFFSET, ret = %d\n", ret);
                return ret;
        }

        n_curr_cal = (sm->curr_slope & CURR_SLOPE_0xFF00) >> SHIFT_8;
        p_curr_cal = (sm->curr_slope & CURR_SLOPE_0x00FF);

        if (sm->en_high_fg_temp_cal && (fg_temp_gap > FG_TEMP_GAP_0)) {
                p_fg_delta_cal = (fg_temp_gap / sm->high_fg_temp_p_cal_denom)*sm->high_fg_temp_p_cal_fact;
                n_fg_delta_cal = (fg_temp_gap / sm->high_fg_temp_n_cal_denom)*sm->high_fg_temp_n_cal_fact;
        }
        else if (sm->en_low_fg_temp_cal && (fg_temp_gap < FG_TEMP_GAP_0)) {
                fg_temp_gap = -fg_temp_gap;
                p_fg_delta_cal = (fg_temp_gap / sm->low_fg_temp_p_cal_denom)*sm->low_fg_temp_p_cal_fact;
                n_fg_delta_cal = (fg_temp_gap / sm->low_fg_temp_n_cal_denom)*sm->low_fg_temp_n_cal_fact;
        }
        p_curr_cal = p_curr_cal + (p_fg_delta_cal);
        n_curr_cal = n_curr_cal + (n_fg_delta_cal);

        temp_gap = sm->batt_temp - sm->temp_std;
        if (sm->en_high_temp_cal && (temp_gap > FG_TEMP_GAP_0)) {
                p_delta_cal = (temp_gap / sm->high_temp_p_cal_denom)*sm->high_temp_p_cal_fact;
                n_delta_cal = (temp_gap / sm->high_temp_n_cal_denom)*sm->high_temp_n_cal_fact;
        }
        else if (sm->en_low_temp_cal && (temp_gap < FG_TEMP_GAP_0)) {
                temp_gap = -temp_gap;
                p_delta_cal = (temp_gap / sm->low_temp_p_cal_denom)*sm->low_temp_p_cal_fact;
                n_delta_cal = (temp_gap / sm->low_temp_n_cal_denom)*sm->low_temp_n_cal_fact;
        }
        p_curr_cal = p_curr_cal + (p_delta_cal);
        n_curr_cal = n_curr_cal + (n_delta_cal);

        curr_cal = (n_curr_cal << SHIFT_8) | p_curr_cal;

        ret = fg_write_word(sm, FG_REG_CURR_IN_SLOPE, curr_cal);
        if (ret < FG_TEMP_GAP_0) {
                return ret;
        }

        ret = fg_read_word(sm, REG_0X28, &data[ARRAY_ELEMENT_0]);
        ret |= fg_read_word(sm, REG_0X82, &data[ARRAY_ELEMENT_1]);
        ret |= fg_read_word(sm, REG_0X83, &data[ARRAY_ELEMENT_2]);
        if (ret < NUM0) {
                chg_err("could not read , ret = %d\n", ret);
                return ret;
        }

        return RETURN_1;
}

static void fg_monitor_workfunc(struct work_struct *work);

static const u8 fg_dump_regs[] = {
        0x00, 0x01, 0x03, 0x04,
        0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0C, 0x0D,
        0x0E, 0x0F, 0x10, 0x11,
        0x12, 0x13, 0x14, 0x1A,
        0x1B, 0x1C, 0x62, 0x73,
        0x74, 0x90, 0x91, 0x95,
        0x96
};

static int fg_dump_debug(struct sm_fg_chip *sm)
{
        int i;
        int ret;
        u16 val = NUM0;

        for (i = NUM0; i < ARRAY_SIZE(fg_dump_regs); i++) {
                ret = fg_read_word(sm, fg_dump_regs[i], &val);
                if (!ret) {
                        chg_debug("Reg[0x%02X] = 0x%02X\n",
                                                fg_dump_regs[i], val);
                }
        }
        return NUM0;
}


static int reg_debugfs_open(struct inode *inode, struct file *file)
{
        struct sm_fg_chip *sm = inode->i_private;
        return single_open(file, show_registers, sm);
}

static const struct file_operations reg_debugfs_ops = {
        .owner		= THIS_MODULE,
        .open		= reg_debugfs_open,
        .read		= seq_read,
        .llseek		= seq_lseek,
        .release	= single_release,
};

static void create_debugfs_entry(struct sm_fg_chip *sm)
{
        sm->debug_root = debugfs_create_dir("sm_fg", NULL);
        if (!sm->debug_root) {
                chg_err("Failed to create debug dir\n");
        }

        if (sm->debug_root) {
                debugfs_create_file("registers", S_IFREG | S_IRUGO,
                                                sm->debug_root, sm, &reg_debugfs_ops);

                debugfs_create_x32("fake_soc",
                                        S_IFREG | S_IWUSR | S_IRUGO,
                                        sm->debug_root,
                                        &(sm->fake_soc));

                debugfs_create_x32("fake_temp",
                                        S_IFREG | S_IWUSR | S_IRUGO,
                                        sm->debug_root,
                                        &(sm->fake_temp));

                debugfs_create_x32("skip_reads",
                                        S_IFREG | S_IWUSR | S_IRUGO,
                                        sm->debug_root,
                                        &(sm->skip_reads));
                debugfs_create_x32("skip_writes",
                                        S_IFREG | S_IWUSR | S_IRUGO,
                                        sm->debug_root,
                                        &(sm->skip_writes));
        }
}

static int show_registers(struct seq_file *m, void *data)
{
        struct sm_fg_chip *sm = m->private;
        int i;
        int ret;
        u16 val = NUM0;

        for (i = NUM0; i < ARRAY_SIZE(fg_dump_regs); i++) {
                ret = fg_read_word(sm, fg_dump_regs[i], &val);
                if (!ret)
                        seq_printf(m, "Reg[0x%02X] = 0x%02X\n",
                                                fg_dump_regs[i], val);
        }
        return NUM0;
}

#define BATT_CURR_NUM0 0
static void fg_refresh_status(struct sm_fg_chip *sm)
{
        bool last_batt_inserted;
        bool last_batt_fc;
        bool last_batt_ot;
        bool last_batt_ut;

        last_batt_inserted	= sm->batt_present;
        last_batt_fc		= sm->batt_fc;
        last_batt_ot		= sm->batt_ot;
        last_batt_ut		= sm->batt_ut;

        fg_read_status(sm);

        if (!last_batt_inserted && sm->batt_present) {
                chg_debug("Battery inserted\n");
        } else if (last_batt_inserted && !sm->batt_present) {
                chg_debug("Battery removed\n");
                sm->batt_soc	= -ENODATA;
                sm->batt_fcc	= -ENODATA;
                sm->batt_volt	= -ENODATA;
                sm->batt_curr	= -ENODATA;
                sm->batt_temp	= -ENODATA;
        }

        if (sm->batt_present) {
                sm->batt_soc = fg_read_soc(sm);
                sm->batt_ocv = fg_read_ocv(sm);
                sm->batt_volt = fg_read_volt(sm);
                sm->batt_curr = (BATT_CURR_NUM0-fg_read_current(sm));
                sm->batt_soc_cycle = fg_get_cycle(sm);
                if (sm->en_temp_in)
                        sm->batt_temp = fg_read_temperature(sm, TEMPERATURE_IN);
                else if (sm->en_temp_ex)
                        sm->batt_temp = fg_read_temperature(sm, TEMPERATURE_EX);
                else
                        sm->batt_temp = -ENODATA;
                fg_cal_carc(sm);

        }
}

#define WORK_POLL_TIME 9000
static unsigned int poll_interval = WORK_POLL_TIME;
static void fg_monitor_workfunc(struct work_struct *work)
{
        struct sm_fg_chip *sm = global_sm;
        if (sm == NULL) {
                chg_err("sm point NULL,return\n");
                return;
        }
        mutex_lock(&sm->data_lock);
        fg_init(sm->client);
        mutex_unlock(&sm->data_lock);

        fg_refresh_status(sm);

        if (poll_interval > NUM0) {
                schedule_delayed_work(&sm->monitor_work, msecs_to_jiffies(poll_interval));
        }
}

#define COMMON_PARAM_MASK		0xFF00
#define COMMON_PARAM_SHIFT		8
#define BATTERY_PARAM_MASK		0x00FF
static bool fg_check_reg_init_need(struct i2c_client *client)
{
        struct sm_fg_chip *sm = i2c_get_clientdata(client);
        int ret = NUM0;
        u16 data = NUM0;
        u16 param_ver = NUM0;

        ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &data);
        if (ret < NUM0) {
                        chg_err("Failed to read param_ctrl unlock, ret = %d\n", ret);
                        return ret;
        } else {
                ret = fg_read_word(sm, FG_PARAM_VERION, &param_ver);
                if (ret < NUM0) {
                                chg_err("Failed to read FG_PARAM_VERION, ret = %d\n", ret);
                                return ret;
                }

                if (((data & INIT_CHECK_MASK) == DISABLE_RE_INIT)
                        && (((param_ver & COMMON_PARAM_MASK) >> COMMON_PARAM_SHIFT) >= sm->common_param_version)
                        && ((param_ver & BATTERY_PARAM_MASK) >= sm->battery_param_version)) {
                        return NUM0;
                }
                else
                {
                        return RETURN_1;
                }
        }
}

#define MINVAL(a, b) ((a <= b) ? a : b)
#define MAXVAL(a, b) ((a > b) ? a : b)
#define ARRAY_1 1
#define FG_REG_V_IDX_NUM0X0010 0x0010
#define FG_REG_V_IDX_NUM0X0000 0x0000
#define ROOP_MAX_NUM0x000F 0x000F
#define I_RET_NUM0X20 0x20
#define I_RET_NUM0X4000 0x4000
#define I_RET_NUM0X3FFF 0x3FFF
#define ROOP_MAX_1 1
#define ROOP_MAX_2 2
#define ROOP_MAX_3 3
#define I_OFFSET_1 1
#define I_OFFSET_4 4
#define I_OFFSET_8 8
#define I_OFFEST_0X0080 0x0080
#define ROOP_MAX_6 6
#define REG_0X20 0x20
#define CB_INDEX_0x000F 0x000F
#define CB_INDEX_7 7
#define CB_INDEX_5 5
#define BATTERY_TABLE_0X10 0x10
#define NUM0X14 0x14
#define NUM0X67 0X67
static int fg_calculate_iocv(struct sm_fg_chip *sm)
{
        bool only_lb = false, sign_i_offset = NUM0;
        int roop_start = NUM0, roop_max = NUM0, i = NUM0, cb_last_index = NUM0, cb_pre_last_index = NUM0;
        int lb_v_buffer[FG_INIT_B_LEN+ARRAY_1] = {NUM0, NUM0, NUM0, NUM0, NUM0, NUM0, NUM0, NUM0};
        int lb_i_buffer[FG_INIT_B_LEN+ARRAY_1] = {NUM0, NUM0, NUM0, NUM0, NUM0, NUM0, NUM0, NUM0};
        int cb_v_buffer[FG_INIT_B_LEN+ARRAY_1] = {NUM0, NUM0, NUM0, NUM0, NUM0, NUM0, NUM0, NUM0};
        int cb_i_buffer[FG_INIT_B_LEN+ARRAY_1] = {NUM0, NUM0, NUM0, NUM0, NUM0, NUM0, NUM0, NUM0};
        int i_offset_margin = NUM0X14, i_vset_margin = NUM0X67;
        int v_max = NUM0, v_min = NUM0, v_sum = NUM0, lb_v_avg = NUM0;
        int cb_v_avg = NUM0, lb_v_set = NUM0, lb_i_set = NUM0, i_offset = NUM0;
        int i_max = NUM0, i_min = NUM0, i_sum = NUM0, lb_i_avg = NUM0;
        int cb_i_avg = NUM0, cb_v_set = NUM0, cb_i_set = NUM0;
        int lb_i_p_v_min = NUM0, lb_i_n_v_max = NUM0;
        int cb_i_p_v_min = NUM0, cb_i_n_v_max = NUM0;

        u16 v_ret, i_ret = NUM0;
        int ret = NUM0;

        u16 data = NUM0;

        ret = fg_read_word(sm, FG_REG_END_V_IDX, &data);
        if (ret < NUM0) {
                        chg_err("Failed to read FG_REG_END_V_IDX, ret = %d\n", ret);
                        return ret;
        } else {
                chg_debug("iocv_status_read = addr : 0x%x , data : 0x%x\n", FG_REG_END_V_IDX, data);
        }

        if ((data & FG_REG_V_IDX_NUM0X0010) == FG_REG_V_IDX_NUM0X0000) {
                only_lb = true;
        }

        roop_max = (data & ROOP_MAX_NUM0x000F);
        if (roop_max > FG_INIT_B_LEN)
                roop_max = FG_INIT_B_LEN;

        roop_start = FG_REG_START_LB_V;
        for (i = roop_start; i < roop_start + roop_max; i++) {
                ret = fg_read_word(sm, i, &v_ret);
                if (ret < NUM0) {
                        chg_err("Failed to read 0x%x, ret = %d\n", i, ret);
                        return ret;
                }
                ret = fg_read_word(sm, i+I_RET_NUM0X20, &i_ret);
                if (ret < NUM0) {
                        chg_err("Failed to read 0x%x, ret = %d\n", i, ret);
                        return ret;
                }

                if ((i_ret&I_RET_NUM0X4000) == I_RET_NUM0X4000) {
                        i_ret = -(i_ret&I_RET_NUM0X3FFF);
                }

                lb_v_buffer[i-roop_start] = v_ret;
                lb_i_buffer[i-roop_start] = i_ret;

                if (i == roop_start) {
                        v_max = v_ret;
                        v_min = v_ret;
                        v_sum = v_ret;
                        i_max = i_ret;
                        i_min = i_ret;
                        i_sum = i_ret;
                }
                else
                {
                        if (v_ret > v_max)
                                v_max = v_ret;
                        else if (v_ret < v_min)
                                v_min = v_ret;
                        v_sum = v_sum + v_ret;

                        if (i_ret > i_max)
                                i_max = i_ret;
                        else if (i_ret < i_min)
                                i_min = i_ret;
                        i_sum = i_sum + i_ret;
                }

                if (abs(i_ret) > i_vset_margin) {
                        if (i_ret > NUM0) {
                                if (lb_i_p_v_min == NUM0) {
                                        lb_i_p_v_min = v_ret;
                                } else {
                                        if (v_ret < lb_i_p_v_min)
                                                lb_i_p_v_min = v_ret;
                                }
                        } else {
                                if (lb_i_n_v_max == NUM0) {
                                        lb_i_n_v_max = v_ret;
                                } else {
                                        if (v_ret > lb_i_n_v_max)
                                                lb_i_n_v_max = v_ret;
                                }
                        }
                }
        }
        v_sum = v_sum - v_max - v_min;
        i_sum = i_sum - i_max - i_min;

        lb_v_avg = v_sum / (roop_max-ROOP_MAX_2);
        lb_i_avg = i_sum / (roop_max-ROOP_MAX_2);

        if (abs(lb_i_buffer[roop_max-ROOP_MAX_1]) < i_vset_margin) {
                if (abs(lb_i_buffer[roop_max-ROOP_MAX_2]) < i_vset_margin) {
                        lb_v_set = MAXVAL(lb_v_buffer[roop_max-ROOP_MAX_2], lb_v_buffer[roop_max-ROOP_MAX_1]);
                        if (abs(lb_i_buffer[roop_max-ROOP_MAX_3]) < i_vset_margin) {
                                lb_v_set = MAXVAL(lb_v_buffer[roop_max-ROOP_MAX_3], lb_v_set);
                        }
                } else {
                        lb_v_set = lb_v_buffer[roop_max-ROOP_MAX_1];
                }
        } else {
                lb_v_set = lb_v_avg;
        }

        if (lb_i_n_v_max > NUM0) {
                lb_v_set = MAXVAL(lb_i_n_v_max, lb_v_set);
        }

        if (roop_max > ROOP_MAX_3) {
                lb_i_set = (lb_i_buffer[ROOP_MAX_2] + lb_i_buffer[ROOP_MAX_3]) / ROOP_MAX_2;
        }

        if ((abs(lb_i_buffer[roop_max-ROOP_MAX_1]) < i_offset_margin) && (abs(lb_i_set) < i_offset_margin)) {
                lb_i_set = MAXVAL(lb_i_buffer[roop_max-ROOP_MAX_1], lb_i_set);
        } else if (abs(lb_i_buffer[roop_max-ROOP_MAX_1]) < i_offset_margin) {
                lb_i_set = lb_i_buffer[roop_max-ROOP_MAX_1];
        } else if (abs(lb_i_set) < i_offset_margin) {
        } else {
                lb_i_set = NUM0;
        }

        i_offset = lb_i_set;

        i_offset = i_offset + I_OFFSET_4;

        if (i_offset <= NUM0) {
                sign_i_offset = I_OFFSET_1;
#ifdef IGNORE_N_I_OFFSET
                i_offset = NUM0;
#else
                i_offset = -i_offset;
#endif
        }

        i_offset = i_offset >> I_OFFSET_1;

        if (sign_i_offset == NUM0) {
                i_offset = i_offset|I_OFFEST_0X0080;
        }
        i_offset = i_offset | i_offset << I_OFFSET_8;

        chg_debug("%s: max=0x%x, min=0x%x, avg=0x%x, set=0x%x, offset=0x%x, offset=%d\n",
                        __func__, i_max, i_min, lb_i_avg, lb_i_set, i_offset, sign_i_offset);

        if (!only_lb) {
                roop_start = FG_REG_START_CB_V;
                roop_max = ROOP_MAX_6;
                for (i = roop_start; i < roop_start + roop_max; i++) {
                        ret = fg_read_word(sm, i, &v_ret);
                        if (ret < NUM0) {
                                chg_err("Failed to read 0x%x, ret = %d\n", i, ret);
                                return ret;
                        }
                        ret = fg_read_word(sm, i+REG_0X20, &i_ret);
                        if (ret < NUM0) {
                                chg_err("Failed to read 0x%x, ret = %d\n", i, ret);
                                return ret;
                        }

                        if ((i_ret&I_RET_NUM0X4000) == I_RET_NUM0X4000) {
                                i_ret = -(i_ret&I_RET_NUM0X3FFF);
                        }

                        cb_v_buffer[i-roop_start] = v_ret;
                        cb_i_buffer[i-roop_start] = i_ret;

                        if (i == roop_start) {
                                v_max = v_ret;
                                v_min = v_ret;
                                v_sum = v_ret;
                                i_max = i_ret;
                                i_min = i_ret;
                                i_sum = i_ret;
                        } else {
                                if (v_ret > v_max)
                                        v_max = v_ret;
                                else if (v_ret < v_min)
                                        v_min = v_ret;
                                v_sum = v_sum + v_ret;

                                if (i_ret > i_max)
                                        i_max = i_ret;
                                else if (i_ret < i_min)
                                        i_min = i_ret;
                                i_sum = i_sum + i_ret;
                        }

                        if (abs(i_ret) > i_vset_margin) {
                                if (i_ret > NUM0) {
                                        if (cb_i_p_v_min == NUM0) {
                                                cb_i_p_v_min = v_ret;
                                        } else {
                                                if (v_ret < cb_i_p_v_min)
                                                        cb_i_p_v_min = v_ret;
                                        }
                                } else {
                                        if (cb_i_n_v_max == NUM0) {
                                                cb_i_n_v_max = v_ret;
                                        } else {
                                                if (v_ret > cb_i_n_v_max)
                                                        cb_i_n_v_max = v_ret;
                                        }
                                }
                        }
                }
                v_sum = v_sum - v_max - v_min;
                i_sum = i_sum - i_max - i_min;

                cb_v_avg = v_sum / (roop_max-ROOP_MAX_2);
                cb_i_avg = i_sum / (roop_max-ROOP_MAX_2);

                cb_last_index = (data & CB_INDEX_0x000F)-CB_INDEX_7;
                if (cb_last_index < NUM0) {
                        cb_last_index = CB_INDEX_5;
                }

                for (i = roop_max; i > NUM0; i--) {
                        if (abs(cb_i_buffer[cb_last_index]) < i_vset_margin) {
                                cb_v_set = cb_v_buffer[cb_last_index];
                                if (abs(cb_i_buffer[cb_last_index]) < i_offset_margin) {
                                        cb_i_set = cb_i_buffer[cb_last_index];
                                }

                                cb_pre_last_index = cb_last_index - RETURN_1;
                                if (cb_pre_last_index < NUM0) {
                                        cb_pre_last_index = CB_INDEX_5;
                                }

                                if (abs(cb_i_buffer[cb_pre_last_index]) < i_vset_margin) {
                                        cb_v_set = MAXVAL(cb_v_buffer[cb_pre_last_index], cb_v_set);
                                        if (abs(cb_i_buffer[cb_pre_last_index]) < i_offset_margin) {
                                                cb_i_set = MAXVAL(cb_i_buffer[cb_pre_last_index], cb_i_set);
                                        }
                                }
                        } else {
                                cb_last_index--;
                                if (cb_last_index < NUM0) {
                                        cb_last_index = CB_INDEX_5;
                                }
                        }
                }

                if (cb_v_set == NUM0) {
                        cb_v_set = cb_v_avg;
                        if (cb_i_set == NUM0) {
                                cb_i_set = cb_i_avg;
                        }
                }

                if (cb_i_n_v_max > NUM0) {
                        cb_v_set = MAXVAL(cb_i_n_v_max, cb_v_set);
                }

                if (abs(cb_i_set) < i_offset_margin) {
                        if (cb_i_set > lb_i_set) {
                                i_offset = cb_i_set;
                                i_offset = i_offset + I_OFFSET_4;

                                if (i_offset <= NUM0) {
                                        sign_i_offset = RETURN_1;
#ifdef IGNORE_N_I_OFFSET
                                        i_offset = NUM0;
#else
                                        i_offset = -i_offset;
#endif
                                }

                                i_offset = i_offset >> RETURN_1;

                                if (sign_i_offset == NUM0) {
                                        i_offset = i_offset | I_OFFEST_0X0080;
                                }
                                i_offset = i_offset | i_offset << I_OFFSET_8;
                        }
                }

                chg_debug("%s: max=0x%x, min=0x%x,avg=0x%x, set=0x%x, offset=0x%x, i_offset=%d\n",
                                __func__, i_max, i_min, cb_i_avg, cb_i_set, i_offset, sign_i_offset);
        }

        if ((abs(cb_i_set) > i_vset_margin) || only_lb) {
                ret = MAXVAL(lb_v_set, cb_i_n_v_max);
        } else {
                ret = cb_v_set;
        }

        if (ret > sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-NUM0]) {
                chg_debug("iocv ret change 0x%x -> 0x%x \n",
                ret, sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-RETURN_1]);
                ret = sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-RETURN_1];
        } else if (ret < sm->battery_table[BATTERY_TABLE0][NUM0]) {
                chg_debug("iocv ret change 0x%x -> 0x%x \n",
                ret, (sm->battery_table[BATTERY_TABLE0][ARRAY_ELEMENT_0] + BATTERY_TABLE_0X10));
                ret = sm->battery_table[BATTERY_TABLE0][ARRAY_ELEMENT_0] + BATTERY_TABLE_0X10;
        }

        return ret;
}

#define FG_REG_0X4000 0x4000
#define FG_REG_0x07FF 0x07FF
#define FG_REG_0xFFE0 0xFFE0
#define FG_REG_0xE0FF 0xE0FF
#define V_L_ALARM_2000 2000
#define V_L_ALARM_3000 3000
#define V_L_ALARM_4000 4000
#define V_L_ALARM_5000 5000
#define V_L_ALARM_NUM0xFEFF 0xFEFF
#define DIVISION_10 10
#define DIVISION_256 256
#define V_L_ALARM_0X1000 0x0100
#define T_H_ALARM_0X8000 0x8000
#define T_H_ALARM_0x7F 0x7F
#define T_H_ALARM_0X0080 0x0080
#define T_H_ALARM_NEGATICE_1 -1
#define MSLEEP_60 60
#define T_H_ALARM_0X03 0x03
#define T_H_ALARM_COUNT_3 3
#define TABLE_REG_0xA0 0xA0
#define TABLE_REG_0xD0 0xD0
#define VALUE_2 2
#define VALUE_3 3
#define FG_REG_SWADDR_0x6A 0x6A
#define FG_REG_SWADDR_0x6B 0x6B
#define FG_REG_SWADDR_0X75 0x75
#define MSLEEP_10 10
#define MSLEEP_20 20
#define MSLEEP_160 160
#define SHIFT_6 6
#define TABLE_NUM0X0003 0x0003
static bool fg_reg_init(struct i2c_client *client)
{
        struct sm_fg_chip *sm = i2c_get_clientdata(client);
        int i, j, value, ret, cnt = NUM0;
        uint8_t table_reg;
        u16 data, data_int_mask = NUM0;

        if (sm->fg_irq_set == -EINVAL) {
                chg_err("sm->fg_irq_set is invalid");
        } else {
                ret = fg_read_word(sm, sm->regs[SM_FG_REG_INT_MASK], &data_int_mask);
                if (ret < NUM0) {
                        chg_err("Failed to read INT_MASK, ret = %d\n", ret);
                        return ret;
                }
                ret = fg_write_word(sm, sm->regs[SM_FG_REG_INT_MASK],
                FG_REG_0X4000 | (data_int_mask | sm->fg_irq_set));
                if (ret < NUM0) {
                        chg_err("Failed to write 0x4000 | INIT_MASK, ret = %d\n", ret);
                        return ret;
                }
                ret = fg_write_word(sm, sm->regs[SM_FG_REG_INT_MASK],
                FG_REG_0x07FF & (data_int_mask | sm->fg_irq_set));
                if (ret < NUM0) {
                        chg_err("Failed to write INIT_MASK, ret = %d\n", ret);
                        return ret;
                }
        }

        if (sm->low_soc1 == -EINVAL) {
                chg_err("sm->low_soc1 is invalid");
        } else {
                ret = fg_read_word(sm, sm->regs[SM_FG_REG_SOC_L_ALARM], &data);
                if (ret < NUM0) {
                        chg_err("Failed to read SOC_L_ALARM (LOW_SOC1), ret = %d\n", ret);
                        return ret;
                }
                ret = fg_write_word(sm, sm->regs[SM_FG_REG_SOC_L_ALARM],
                ((data & FG_REG_0xFFE0) | sm->low_soc1));
                if (ret < NUM0) {
                        chg_err("Failed to write SOC_L_ALARM (LOW_SOC1), ret = %d\n", ret);
                        return ret;
                }
        }

        if (sm->low_soc2 == -EINVAL) {
                chg_err("sm->low_soc2 is invalid");
        } else {
                ret = fg_read_word(sm, sm->regs[SM_FG_REG_SOC_L_ALARM], &data);
                if (ret < NUM0) {
                        chg_err("Failed to read SOC_L_ALARM (LOW_SOC2), ret = %d\n", ret);
                        return ret;
                }
                ret = fg_write_word(sm, sm->regs[SM_FG_REG_SOC_L_ALARM],
                ((data & FG_REG_0xE0FF) | (sm->low_soc2 << SHIFT_8)));
                if (ret < NUM0) {
                        chg_err("Failed to write LOW_SOC2, ret = %d\n", ret);
                        return ret;
                }
        }

        if (sm->v_l_alarm == -EINVAL) {
                chg_err("sm->v_l_alarm is invalid");
        } else {
                if (sm->v_l_alarm >= V_L_ALARM_2000 && sm->v_l_alarm < V_L_ALARM_3000)
                        data = (V_L_ALARM_NUM0xFEFF & (sm->v_l_alarm/DIVISION_10 * DIVISION_256));
                else if (sm->v_l_alarm >= V_L_ALARM_3000 && sm->v_l_alarm < V_L_ALARM_4000)
                        data = (V_L_ALARM_0X1000 | (sm->v_l_alarm/DIVISION_10 * DIVISION_256));
                else {
                        ret = -EINVAL;
                        chg_err("Failed to calculate V_L_ALARM, ret = %d\n", ret);
                        return ret;
                }

                ret = fg_write_word(sm, sm->regs[SM_FG_REG_V_L_ALARM], data);
                if (ret < NUM0) {
                        chg_err("Failed to write V_L_ALARM, ret = %d\n", ret);
                        return ret;
                }
        }

        if (sm->v_h_alarm == -EINVAL) {
                chg_err("sm->v_h_alarm is invalid");
        } else {
                if (sm->v_h_alarm >= V_L_ALARM_3000 && sm->v_h_alarm < V_L_ALARM_4000)
                        data = (V_L_ALARM_NUM0xFEFF & (sm->v_h_alarm/DIVISION_10 * DIVISION_256));
                else if (sm->v_h_alarm >= V_L_ALARM_4000 && sm->v_h_alarm < V_L_ALARM_5000)
                        data = (V_L_ALARM_0X1000 | (sm->v_h_alarm/DIVISION_10 * DIVISION_256));
                else {
                        ret = -EINVAL;
                        chg_err("Failed to calculate V_H_ALARM, ret = %d\n", ret);
                        return ret;
                }

                ret = fg_write_word(sm, sm->regs[SM_FG_REG_V_H_ALARM], data);
                if (ret < NUM0) {
                        chg_err("Failed to write V_H_ALARM, ret = %d\n", ret);
                        return ret;
                }
        }

        if (sm->t_h_alarm_in == -EINVAL
                || sm->t_l_alarm_in == -EINVAL) {
                chg_err("sm->t_h_alarm_in || sm->t_l_alarm_in is invalid");
        } else {
                data = NUM0;
                if (sm->t_h_alarm_in < NUM0) {
                        data |= T_H_ALARM_0X8000;
                        data |= ((((T_H_ALARM_NEGATICE_1)*sm->t_h_alarm_in) & T_H_ALARM_0x7F) << SHIFT_8);
                } else {
                        data |= (((sm->t_h_alarm_in) & T_H_ALARM_0x7F) << SHIFT_8);
                }
                if (sm->t_l_alarm_in < NUM0) {
                        data |= T_H_ALARM_0X0080;
                        data |= ((((T_H_ALARM_NEGATICE_1)*sm->t_l_alarm_in) & T_H_ALARM_0x7F));
                } else {
                        data |= (((sm->t_l_alarm_in) & T_H_ALARM_0x7F));
                }

                ret = fg_write_word(sm, sm->regs[SM_FG_REG_T_IN_H_ALARM], data);
                if (ret < NUM0) {
                        chg_err("Failed to write SM_FG_REG_T_IN_H_ALARM, ret = %d\n", ret);
                        return ret;
                }
        }

        do {
                ret = fg_write_word(sm, sm->regs[SM_FG_REG_PARAM_CTRL],
                (FG_PARAM_UNLOCK_CODE | ((sm->battery_table_num & TABLE_NUM0X0003) << SHIFT_6)
                | (FG_TABLE_LEN-NUM1)));
                if (ret < NUM0) {
                        chg_err("Failed to write param_ctrl unlock, ret = %d\n", ret);
                        return ret;
                } else {
                        chg_debug("Param Unlock\n");
                }
                msleep(MSLEEP_60);
                ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &data);
                if (ret < NUM0) {
                        chg_err("Failed to read FG_OP_STATUS, ret = %d\n", ret);
                } else {
                        chg_debug(" FG_OP_STATUS = 0x%x\n", data);
                }
                cnt++;
        } while (((data & T_H_ALARM_0X03)!= T_H_ALARM_0X03) && cnt <= T_H_ALARM_COUNT_3);

        ret = fg_write_word(sm, sm->regs[SM_FG_REG_VIT_PERIOD], sm->vit_period);
        if (ret < NUM0) {
                chg_err("Failed to write VIT PERIOD, ret = %d\n", ret);
                return ret;
        } else {
                        chg_debug("Write VIT_PERIOD = 0x%x : 0x%x\n",
                        sm->regs[SM_FG_REG_VIT_PERIOD], sm->vit_period);
        }

        ret = fg_write_word(sm, FG_REG_AGING_CTRL, sm->aging_ctrl);
        if (ret < NUM0) {
                chg_err("Failed to write FG_REG_AGING_CTRL, ret = %d\n", ret);
                return ret;
        } else {
                        chg_debug("Write FG_REG_AGING_CTRL = 0x%x : 0x%x\n",
                        FG_REG_AGING_CTRL, sm->aging_ctrl);
        }

        ret = fg_write_word(sm, FG_REG_SOC_CYCLE_CFG, sm->cycle_cfg);
        if (ret < NUM0) {
                chg_err("Failed to write FG_REG_SOC_CYCLE_CFG, ret = %d\n", ret);
                return ret;
        } else {
                        chg_debug("Write FG_REG_SOC_CYCLE_CFG = 0x%x : 0x%x\n",
                        FG_REG_SOC_CYCLE_CFG, sm->cycle_cfg);
        }

        ret = fg_write_word(sm, sm->regs[SM_FG_REG_RSNS_SEL], sm->batt_rsns);
        if (ret < NUM0) {
                chg_err("Failed to write SM_FG_REG_RSNS_SEL, ret = %d\n", ret);
                return ret;
        } else {
                        chg_debug("Write SM_FG_REG_RSNS_SEL = 0x%x : 0x%x\n",
                        sm->regs[SM_FG_REG_RSNS_SEL], sm->batt_rsns);
        }

        for (i = BATTERY_TABLE0; i < BATTERY_TABLE2; i++) {
                table_reg = TABLE_REG_0xA0 + (i*FG_TABLE_LEN);
                for (j = NUM0; j < FG_TABLE_LEN; j++) {
                        ret = fg_write_word(sm, (table_reg + j), sm->battery_table[i][j]);
                        if (ret < NUM0) {
                                chg_err("Failed to write Battery Table, ret = %d\n", ret);
                                return ret;
                        } else {
                                chg_debug("TABLE write OK [%d][%d] = 0x%x : 0x%x\n",
                                        i, j, (table_reg + j), sm->battery_table[i][j]);
                        }
                }
        }

        for (j = NUM0; j < FG_ADD_TABLE_LEN; j++) {
                table_reg = TABLE_REG_0xD0 + j;
                ret = fg_write_word(sm, table_reg, sm->battery_table[i][j]);
                if (ret < NUM0) {
                        chg_err("Failed to write Battery Table, ret = %d\n", ret);
                        return ret;
                } else {
                        chg_debug("TABLE write OK [%d][%d] = 0x%x : 0x%x\n",
                                i, j, table_reg, sm->battery_table[i][j]);
                }
        }

        ret = fg_write_word(sm, FG_REG_RS, sm->rs);
        if (ret < NUM0) {
                chg_err("Failed to write RS, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("RS = 0x%x : 0x%x\n", FG_REG_RS, sm->rs);
        }

        ret = fg_write_word(sm, FG_REG_ALPHA, sm->alpha);
        if (ret < NUM0) {
                chg_err("Failed to write FG_REG_ALPHA, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("ALPHA = 0x%x : 0x%x\n", FG_REG_ALPHA, sm->alpha);
        }

        ret = fg_write_word(sm, FG_REG_BETA, sm->beta);
        if (ret < NUM0) {
                chg_err("Failed to write FG_REG_BETA, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("BETA = 0x%x : 0x%x\n", FG_REG_BETA, sm->beta);
        }

        ret = fg_write_word(sm, FG_REG_RS_0, sm->rs_value[NUM0]);
        if (ret < NUM0) {
                chg_err("Failed to write RS_0, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("RS = 0x%x : 0x%x\n", FG_REG_RS_0, sm->rs_value[NUM0]);
        }

        ret = fg_write_word(sm, FG_REG_RS_1, sm->rs_value[RETURN_1]);
        if (ret < NUM0) {
                chg_err("Failed to write RS_1, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("RS_1 = 0x%x : 0x%x\n", FG_REG_RS_1, sm->rs_value[RETURN_1]);
        }

        ret = fg_write_word(sm, FG_REG_RS_2, sm->rs_value[VALUE_2]);
        if (ret < NUM0) {
                chg_err("Failed to write RS_2, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("RS_2 = 0x%x : 0x%x\n", FG_REG_RS_2, sm->rs_value[VALUE_2]);
        }

        ret = fg_write_word(sm, FG_REG_RS_3, sm->rs_value[VALUE_3]);
        if (ret < NUM0) {
                chg_err("Failed to write RS_3, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("RS_3 = 0x%x : 0x%x\n", FG_REG_RS_3, sm->rs_value[VALUE_3]);
        }

        ret = fg_write_word(sm, sm->regs[SM_FG_REG_CURRENT_RATE], sm->mix_value);
        if (ret < NUM0) {
                chg_err("Failed to write CURRENT_RATE, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("CURRENT_RATE = 0x%x : 0x%x\n",
                sm->regs[SM_FG_REG_CURRENT_RATE], sm->mix_value);
        }

        chg_debug("RS_0 = 0x%x, RS_1 = 0x%x, RS_2 = 0x%x, RS_3 = 0x%x, CURRENT_RATE = 0x%x\n",
                sm->rs_value[NUM0], sm->rs_value[RETURN_1],
                sm->rs_value[VALUE_2], sm->rs_value[VALUE_3], sm->mix_value);

        ret = fg_write_word(sm, FG_REG_VOLT_CAL, sm->volt_cal);
        if (ret < NUM0) {
                chg_err("Failed to write FG_REG_VOLT_CAL, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("FG_REG_VOLT_CAL = 0x%x : 0x%x\n", FG_REG_VOLT_CAL,
                sm->volt_cal);
        }

        ret = fg_write_word(sm, FG_REG_CURR_IN_OFFSET, sm->curr_offset);
        if (ret < NUM0) {
                chg_err("Failed to write CURR_IN_OFFSET, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("CURR_IN_OFFSET = 0x%x : 0x%x\n", FG_REG_CURR_IN_OFFSET,
                sm->curr_offset);
        }
        ret = fg_write_word(sm, FG_REG_CURR_IN_SLOPE, sm->curr_slope);
        if (ret < NUM0) {
                chg_err("Failed to write CURR_IN_SLOPE, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("CURR_IN_SLOPE = 0x%x : 0x%x\n", FG_REG_CURR_IN_SLOPE,
                sm->curr_slope);
        }

        ret = fg_write_word(sm, sm->regs[SM_FG_REG_BAT_CAP], sm->cap);
        if (ret < NUM0) {
                chg_err("Failed to write BAT_CAP, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("BAT_CAP = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_BAT_CAP], sm->cap);
        }

        ret = fg_write_word(sm, sm->regs[SM_FG_REG_MISC], sm->misc);
        if (ret < NUM0) {
                chg_err("Failed to write REG_MISC, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("REG_MISC 0x%x : 0x%x\n", sm->regs[SM_FG_REG_MISC], sm->misc);
        }

        ret = fg_write_word(sm, sm->regs[SM_FG_REG_TOPOFFSOC], sm->topoff_soc);
        if (ret < NUM0) {
                chg_err("Failed to write SM_FG_REG_TOPOFFSOC, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("SM_REG_TOPOFFSOC 0x%x : 0x%x\n", sm->regs[SM_FG_REG_TOPOFFSOC],
                sm->topoff_soc);
        }

        ret = fg_read_word(sm, sm->regs[SM_FG_REG_CNTL], &data);
        if (ret < NUM0) {
                        chg_err("Failed to read CNTL, ret = %d\n", ret);
                        return ret;
        }

        if (sm->en_temp_in)
                data |= ENABLE_EN_TEMP_IN;
        if (sm->en_temp_ex)
                data |= ENABLE_EN_TEMP_EX;
        if (sm->en_batt_det)
                data |= ENABLE_EN_BATT_DET;
        if (sm->iocv_man_mode)
                data |= ENABLE_IOCV_MAN_MODE;

        ret = fg_write_word(sm, sm->regs[SM_FG_REG_CNTL], data);
        if (ret < NUM0) {
                chg_err("Failed to write CNTL, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("CNTL = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CNTL], data);
        }

        ret = fg_write_word(sm, FG_PARAM_VERION,
        ((sm->common_param_version << SHIFT_8) | sm->battery_param_version));
        if (ret < NUM0) {
                chg_err("Failed to write FG_PARAM_VERION, ret = %d\n", ret);
                return ret;
        }

        if (sm->t_l_alarm_ex == -EINVAL) {
                chg_err("sm->t_l_alarm_ex is invalid");
        } else {
                data = (sm->t_l_alarm_ex) >> RETURN_1;

                ret = fg_write_word(sm, FG_REG_SWADDR, FG_REG_SWADDR_0x6A);
                if (ret < NUM0) {
                        chg_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
                        return ret;
                }
                ret = fg_write_word(sm, FG_REG_SWDATA, data);
                if (ret < NUM0) {
                        chg_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
                        return ret;
                }

                chg_debug("write to T_EX_H_ALARM = 0x%x\n", data);
        }

        if (sm->t_h_alarm_ex == -EINVAL) {
                chg_err("sm->t_h_alarm_ex is invalid");
        } else {
                data = (sm->t_h_alarm_ex) >> RETURN_1;

                ret = fg_write_word(sm, FG_REG_SWADDR, FG_REG_SWADDR_0x6B);
                if (ret < NUM0) {
                        chg_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
                        return ret;
                }
                ret = fg_write_word(sm, FG_REG_SWDATA, data);
                if (ret < NUM0) {
                        chg_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
                        return ret;
                }
                chg_debug("write to T_EX_L_ALARM = 0x%x\n", data);
        }

        if (sm->iocv_man_mode) {
                value = fg_calculate_iocv(sm);

                msleep(MSLEEP_10);
                ret = fg_write_word(sm, FG_REG_SWADDR, FG_REG_SWADDR_0X75);
                if (ret < NUM0) {
                        chg_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
                        return ret;
                }
                ret = fg_write_word(sm, FG_REG_SWDATA, value);
                if (ret < NUM0) {
                        chg_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
                        return ret;
                }
                chg_debug("IOCV_MAN : 0x%x\n", value);
        }

        msleep(MSLEEP_20);

        ret = fg_write_word(sm, sm->regs[SM_FG_REG_PARAM_CTRL],
        ((FG_PARAM_LOCK_CODE | (sm->battery_table_num & TABLE_NUM0X0003) << SHIFT_6) | (FG_TABLE_LEN-NUM1)));
        if (ret < NUM0) {
                chg_err("Failed to write param_ctrl lock, ret = %d\n", ret);
                return ret;
        } else {
                chg_debug("Param Lock\n");
        }

        msleep(MSLEEP_160);

        return RETURN_1;
}

#define SHIFT_4 4
#define REVISION_ID_0x000f 0x000f
#define DEVICE_ID_0x00f0 0x00f0
static unsigned int fg_get_device_id(struct i2c_client *client)
{
        struct sm_fg_chip *sm = i2c_get_clientdata(client);
        int ret;
        u16 data;

        ret = fg_read_word(sm, sm->regs[SM_FG_REG_DEVICE_ID], &data);
        if (ret < NUM0) {
                chg_err("Failed to read DEVICE_ID, ret = %d\n", ret);
                return ret;
        }


        return ret;
}

static bool fg_init(struct i2c_client *client)
{
        int ret;
        struct sm_fg_chip *sm = i2c_get_clientdata(client);

        ret = fg_get_device_id(client);
        if (ret < NUM0) {
                chg_err("%s: fail to do i2c read(%d)\n", __func__, ret);
                return false;
        }

        if (fg_check_reg_init_need(client)) {
                ret = fg_reset(sm);
                if (ret < NUM0) {
                        chg_err("%s: fail to do reset(%d)\n", __func__, ret);
                        return false;
                }
                fg_reg_init(client);
        }

        return true;
}

#define PROPERTY_NAME_SIZE 128
#define MISC_0X0800 0x0800
static int fg_common_parse_dt(struct sm_fg_chip *sm)
{
        struct device *dev = &sm->client->dev;
        struct device_node *np = dev->of_node;
        int rc;

        BUG_ON(dev == NUM0);
        BUG_ON(np == NUM0);

        sm->gpio_int = of_get_named_gpio(np, "qcom,irq-gpio", NUM0);

        if (!gpio_is_valid(sm->gpio_int)) {
                chg_debug("gpio_int is not valid\n");
                sm->gpio_int = -EINVAL;
        }

        if (of_property_read_bool(np, "sm,en_temp_ex"))
                sm->en_temp_ex = true;
        else
                sm->en_temp_ex = NUM0;

        if (of_property_read_bool(np, "sm,en_temp_in"))
                sm->en_temp_in = true;
        else
                sm->en_temp_in = NUM0;

        if (of_property_read_bool(np, "sm,en_batt_det"))
                sm->en_batt_det = true;
        else
                sm->en_batt_det = NUM0;

        rc = of_property_read_u32(np, "sm,misc",
                                                &sm->misc);
        if (rc < NUM0)
                sm->misc = MISC_0X0800;

        if (of_property_read_bool(np, "sm,iocv_man_mode"))
                sm->iocv_man_mode = true;
        else
                sm->iocv_man_mode = NUM0;

        rc = of_property_read_u32(np, "sm,aging_ctrl",
                                                &sm->aging_ctrl);
        if (rc < NUM0)
                sm->aging_ctrl = -EINVAL;

        rc = of_property_read_u32(np, "sm,cycle_cfg",
                                                &sm->cycle_cfg);
        if (rc < NUM0)
                sm->cycle_cfg = -EINVAL;

        rc = of_property_read_u32(np, "sm,rsns",
                                                &sm->batt_rsns);
        if (rc < NUM0)
                sm->batt_rsns = -EINVAL;

        rc = of_property_read_u32(np, "sm,fg_irq_set",
                                                &sm->fg_irq_set);
        if (rc < NUM0)
                sm->fg_irq_set = -EINVAL;

        rc = of_property_read_u32(np, "sm,low_soc1",
                                                &sm->low_soc1);
        if (rc < NUM0)
                sm->low_soc1 = -EINVAL;

        rc = of_property_read_u32(np, "sm,low_soc2",
                                        &sm->low_soc2);
        if (rc < NUM0)
                sm->low_soc2 = -EINVAL;

        rc = of_property_read_u32(np, "sm,v_l_alarm",
                                                &sm->v_l_alarm);
        if (rc < NUM0)
                sm->v_l_alarm = -EINVAL;

        rc = of_property_read_u32(np, "sm,v_h_alarm",
                                        &sm->v_h_alarm);
        if (rc < NUM0)
                sm->v_h_alarm = -EINVAL;

        rc = of_property_read_u32(np, "sm,t_l_alarm_in",
                                                &sm->t_l_alarm_in);
        if (rc < NUM0)
                sm->t_l_alarm_in = -EINVAL;

        rc = of_property_read_u32(np, "sm,t_h_alarm_in",
                                        &sm->t_h_alarm_in);
        if (rc < NUM0)
                sm->t_h_alarm_in = -EINVAL;

        rc = of_property_read_u32(np, "sm,t_l_alarm_ex",
                                                &sm->t_l_alarm_ex);
        if (rc < NUM0)
                sm->t_l_alarm_ex = -EINVAL;

        rc = of_property_read_u32(np, "sm,t_h_alarm_ex",
                                        &sm->t_h_alarm_ex);
        if (rc < NUM0)
                sm->t_h_alarm_ex = -EINVAL;

        rc = of_property_read_u32(np, "sm,battery_table_num",
                                                &sm->battery_table_num);
        if (rc < NUM0)
                sm->battery_table_num = -EINVAL;

        rc = of_property_read_u32(np, "sm,param_version",
                                                &sm->common_param_version);
        if (rc < NUM0)
                sm->common_param_version = -EINVAL;

        return NUM0;
}

static int get_battery_id(struct sm_fg_chip *sm)
{
        return NUM0;
}

#define CYCLE_COUNT_4 4
#define VALUE_0 0
#define VALUE_1 1
#define VALUE_2 2
#define VALUE_3 3
#define VALUE_4 4
#define VALUE_5 5
#define VALUE_6 6

#define TEMP_CAL_0 0
#define TEMP_CAL_1 1
#define TEMP_CAL_2 2
#define TEMP_CAL_3 3
#define TEMP_CAL_4 4
#define TEMP_CAL_5 5
#define TEMP_CAL_6 6
#define TEMP_CAL_7 7
#define TEMP_CAL_8 8
#define TEMP_CAL_9 9
#define TEMP_CAL_10 10
#define BATTERY_ID_ERROR -1

static int fg_battery_parse_dt(struct sm_fg_chip *sm)
{
        struct device *dev = &sm->client->dev;
        struct device_node *np = dev->of_node;
        char prop_name[PROPERTY_NAME_SIZE];
        int battery_id = BATTERY_ID_ERROR;
        int battery_temp_table[FG_TEMP_TABLE_CNT_MAX];
        int table[FG_TABLE_LEN];
        int rs_value[TEMP_CAL_4];
        int topoff_soc[TEMP_CAL_3];
        int temp_offset[TEMP_CAL_6];
        int temp_cal[TEMP_CAL_10];
        int ext_temp_cal[TEMP_CAL_10];
        int battery_type[TEMP_CAL_3];
        int set_temp_poff[TEMP_CAL_4];
        int ret;
        int i, j;

        BUG_ON(dev == NUM0);
        BUG_ON(np == NUM0);

        np = of_find_node_by_name(of_node_get(np), "battery_params");
        if (np == NULL) {
                chg_debug("Cannot find child node \"battery_params\"\n");
                return -EINVAL;
        }

        if (of_property_read_u32(np, "battery,id", &battery_id) < NUM0)
                chg_err("not battery,id property\n");
        if (battery_id == RET_ERR)
                battery_id = get_battery_id(sm);

        for (i = BATTERY_TABLE0; i < BATTERY_TABLE2; i++) {
                snprintf(prop_name, PROPERTY_NAME_SIZE,
                        "battery%d,%s%d", battery_id, "battery_table", i);

                ret = of_property_read_u32_array(np, prop_name, table, FG_TABLE_LEN);
                if (ret < NUM0)
                        chg_debug("Can get prop %s (%d)\n", prop_name, ret);
                for (j = NUM0; j < FG_TABLE_LEN; j++) {
                        sm->battery_table[i][j] = table[j];
                }
        }

        i = BATTERY_TABLE2;
        snprintf(prop_name, PROPERTY_NAME_SIZE,
                "battery%d,%s%d", battery_id, "battery_table", i);
        ret = of_property_read_u32_array(np, prop_name, table, FG_ADD_TABLE_LEN);
        if (ret < NUM0)
                chg_debug("Can get prop %s (%d)\n", prop_name, ret);
        else {
                for (j = NUM0; j < FG_ADD_TABLE_LEN; j++) {
                        sm->battery_table[i][j] = table[j];
                }
        }

        snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "rs");
        ret = of_property_read_u32_array(np, prop_name, &sm->rs, RETURN_1);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);

        snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "alpha");
        ret = of_property_read_u32_array(np, prop_name, &sm->alpha, RETURN_1);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);

        snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "beta");
        ret = of_property_read_u32_array(np, prop_name, &sm->beta, RETURN_1);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);

        for (i = NUM0; i < CYCLE_COUNT_4; i++) {
                snprintf(prop_name,
                        PROPERTY_NAME_SIZE, "battery%d,%s",
                        battery_id, "rs_value");
                ret = of_property_read_u32_array(np, prop_name, rs_value, VALUE_4);
                if (ret < VALUE_0)
                        chg_err("Can get prop %s (%d)\n", prop_name, ret);
                sm->rs_value[i] = rs_value[i];
        }

        snprintf(prop_name,
                PROPERTY_NAME_SIZE, "battery%d,%s",
                battery_id, "vit_period");
        ret = of_property_read_u32_array(np,
                prop_name, &sm->vit_period, VALUE_1);
        if (ret < NUM0)
                chg_debug("Can get prop %s (%d)\n", prop_name, ret);

        snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "battery_type");
        ret = of_property_read_u32_array(np, prop_name, battery_type, VALUE_3);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);
        sm->batt_v_max = battery_type[VALUE_0];
        sm->min_cap = battery_type[VALUE_1];
        sm->cap = battery_type[VALUE_2];

        snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "tem_poff");
        ret = of_property_read_u32_array(np, prop_name, set_temp_poff, VALUE_2);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);
        sm->n_tem_poff = set_temp_poff[VALUE_0];
        sm->n_tem_poff_offset = set_temp_poff[VALUE_1];

        snprintf(prop_name,
                PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "max_voltage_uv");
        ret = of_property_read_u32(np, prop_name,
                                                &sm->batt_max_voltage_uv);
        if (ret < NUM0)
                chg_err("couldn't find battery max voltage\n");

        snprintf(prop_name,
                PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "topoff_soc");
        ret = of_property_read_u32_array(np, prop_name, topoff_soc, VALUE_2);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);
        sm->topoff_soc = topoff_soc[VALUE_0];
        sm->top_off = topoff_soc[VALUE_1];

        snprintf(prop_name,
                PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "mix_value");
        ret = of_property_read_u32_array(np, prop_name, &sm->mix_value, NUM1);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);

        snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "volt_cal");
        ret = of_property_read_u32_array(np, prop_name, &sm->volt_cal, VALUE_1);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);

        snprintf(prop_name,
                PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "curr_offset");
        ret = of_property_read_u32_array(np,
                prop_name, &sm->curr_offset, VALUE_1);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);

        snprintf(prop_name,
                PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "curr_slope");
        ret = of_property_read_u32_array(np,
                prop_name, &sm->curr_slope, VALUE_1);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);

        snprintf(prop_name,
                PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_std");
        ret = of_property_read_u32_array(np, prop_name, &sm->temp_std, NUM1);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);

        snprintf(prop_name,
                PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_offset");
        ret = of_property_read_u32_array(np, prop_name, temp_offset, VALUE_6);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);
        sm->en_high_fg_temp_offset = temp_offset[VALUE_0];
        sm->high_fg_temp_offset_denom = temp_offset[VALUE_1];
        sm->high_fg_temp_offset_fact = temp_offset[VALUE_2];
        sm->en_low_fg_temp_offset = temp_offset[VALUE_3];
        sm->low_fg_temp_offset_denom = temp_offset[VALUE_4];
        sm->low_fg_temp_offset_fact = temp_offset[VALUE_5];
        chg_debug("%s = <%d, %d, %d, %d, %d, %d>\n", prop_name,
                sm->en_high_fg_temp_offset,
                sm->high_fg_temp_offset_denom, sm->high_fg_temp_offset_fact,
                sm->en_low_fg_temp_offset,
                sm->low_fg_temp_offset_denom, sm->low_fg_temp_offset_fact);

        snprintf(prop_name,
                PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_cal");
        ret = of_property_read_u32_array(np, prop_name, temp_cal, TEMP_CAL_10);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);
        sm->en_high_fg_temp_cal = temp_cal[TEMP_CAL_0];
        sm->high_fg_temp_p_cal_denom = temp_cal[TEMP_CAL_1];
        sm->high_fg_temp_p_cal_fact = temp_cal[TEMP_CAL_2];
        sm->high_fg_temp_n_cal_denom = temp_cal[TEMP_CAL_3];
        sm->high_fg_temp_n_cal_fact = temp_cal[TEMP_CAL_4];
        sm->en_low_fg_temp_cal = temp_cal[TEMP_CAL_5];
        sm->low_fg_temp_p_cal_denom = temp_cal[TEMP_CAL_6];
        sm->low_fg_temp_p_cal_fact = temp_cal[TEMP_CAL_7];
        sm->low_fg_temp_n_cal_denom = temp_cal[TEMP_CAL_8];
        sm->low_fg_temp_n_cal_fact = temp_cal[TEMP_CAL_9];

        snprintf(prop_name,
                PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "ext_temp_cal");
        ret = of_property_read_u32_array(np, prop_name, ext_temp_cal, TEMP_CAL_10);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);
        sm->en_high_temp_cal = ext_temp_cal[TEMP_CAL_0];
        sm->high_temp_p_cal_denom = ext_temp_cal[TEMP_CAL_1];
        sm->high_temp_p_cal_fact = ext_temp_cal[TEMP_CAL_2];
        sm->high_temp_n_cal_denom = ext_temp_cal[TEMP_CAL_3];
        sm->high_temp_n_cal_fact = ext_temp_cal[TEMP_CAL_4];
        sm->en_low_temp_cal = ext_temp_cal[TEMP_CAL_5];
        sm->low_temp_p_cal_denom = ext_temp_cal[TEMP_CAL_6];
        sm->low_temp_p_cal_fact = ext_temp_cal[TEMP_CAL_7];
        sm->low_temp_n_cal_denom = ext_temp_cal[TEMP_CAL_8];
        sm->low_temp_n_cal_fact = ext_temp_cal[TEMP_CAL_9];

        snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "thermal_table");

        ret = of_property_read_u32_array(np, prop_name, battery_temp_table, FG_TEMP_TABLE_CNT_MAX);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);
        for (i = NUM0; i < FG_TEMP_TABLE_CNT_MAX; i++) {
                sm->battery_temp_table[i] = battery_temp_table[i];
                chg_err("%s = <battery_temp_table[%d] 0x%x>\n",
                        prop_name, i,	battery_temp_table[i]);
        }

        snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "param_version");
        ret = of_property_read_u32_array(np, prop_name, &sm->battery_param_version, RETURN_1);
        if (ret < NUM0)
                chg_err("Can get prop %s (%d)\n", prop_name, ret);

        return NUM0;
}

bool hal_fg_init(struct i2c_client *client)
{
        struct sm_fg_chip *sm = i2c_get_clientdata(client);

        mutex_lock(&sm->data_lock);
        if (client->dev.of_node) {
                fg_common_parse_dt(sm);
                fg_battery_parse_dt(sm);
        }

        if (!fg_init(client))
                return false;

        mutex_unlock(&sm->data_lock);
        return true;
}

#define TEMP_DEFAULT 250
static int sm5602_get_battery_temperature(void)
{
	if (global_sm == NULL)
		return TEMP_DEFAULT;

	/*fg_refresh_status(global_sm);*/
	if (global_sm->en_temp_in)
		global_sm->batt_temp = fg_read_temperature(global_sm, TEMPERATURE_IN);
	else if (global_sm->en_temp_ex)
		global_sm->batt_temp = fg_read_temperature(global_sm, TEMPERATURE_EX);
	else
		global_sm->batt_temp = -ENODATA;

	return global_sm->batt_temp * 10;
}

#define VBAT_DEFAULT 3900
static int sm5602_get_battery_mvolts(void)
{
	int ret = 0;
	if (global_sm == NULL)
		return VBAT_DEFAULT;

	ret = fg_read_volt(global_sm);
	mutex_lock(&global_sm->data_lock);
	if (ret >= 0)
		global_sm->batt_volt = ret;
	mutex_unlock(&global_sm->data_lock);

	return global_sm->batt_volt;
}

#define SOC_DEFAULT 50

static int sm5602_get_battery_soc(void)
{
	int ret = 0;

	if (global_sm == NULL)
		return SOC_DEFAULT;

	ret = fg_read_soc(global_sm);
	mutex_lock(&global_sm->data_lock);
	global_sm->batt_soc = ret/MULTIPLY_10;
	mutex_unlock(&global_sm->data_lock);

	return global_sm->batt_soc;
}

#define CHARGING_CURRENT_DEFAULT 100
static int sm5602_get_average_current(void)
{
        int ret;
        if (global_sm == NULL)
                return CHARGING_CURRENT_DEFAULT;
        ret = fg_read_current(global_sm);
        mutex_lock(&global_sm->data_lock);
        global_sm->batt_curr = (NUM0-ret);
        mutex_unlock(&global_sm->data_lock);

        return global_sm->batt_curr;
}

static int sm5602_get_battery_fcc(void)
{
        int ret;

        if (global_sm == NULL)
                return NUM0;
        ret = fg_read_fcc(global_sm);
        mutex_lock(&global_sm->data_lock);
        if (ret > NUM0)
                global_sm->batt_fcc = ret;
        mutex_unlock(&global_sm->data_lock);

        return global_sm->batt_fcc;
}

static int sm5602_get_prev_batt_fcc(void)
{
        int ret;

        if (global_sm == NULL)
                return NUM0;
        ret = fg_read_fcc(global_sm);
        mutex_lock(&global_sm->data_lock);
        if (ret > NUM0)
                global_sm->batt_fcc = ret;
        mutex_unlock(&global_sm->data_lock);

        return global_sm->batt_fcc;
}
#define NTC_DEFAULT_VOLT_VALUE_MV 950
#define BATID_LOW_MV1 200
#define BATID_HIGH_MV1 320
#define BATID_LOW_MV2 719
#define BATID_HIGH_MV2 820
#define THERMAL_TEMP_UNIT      1000
enum {
        BAT_TYPE_UNKNOWN,
        BAT_TYPE_SDI_7500MA,
};
static int oplus_get_iio_channel(struct sm_fg_chip * chip, const char *propname,
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
                chg_err(" %s channel unavailable, %d\n", propname, rc);
        *chan = NULL;
	}

	return rc;
}

static int battery_type_check(void)
{
        int battery_type = BAT_TYPE_UNKNOWN;
        int ret = -1;
        int value = 0;

        if (!global_sm->ntc_switch1_chan) {
                chg_err("[OPLUS_CHG][%s]: chip not ready!\n", __func__);
                ret = oplus_get_iio_channel(global_sm, "ntc_switch1_chan", &global_sm->ntc_switch1_chan);
                if (ret < 0 && !global_sm->ntc_switch1_chan) {
                        chg_err(" %s usb_temp1 get failed\n", __func__);
                return -1;
        }
        }
        ret = iio_read_channel_processed(global_sm->ntc_switch1_chan, &value);
        if (ret < 0) {
        chg_err("fail to read usb_temp1 adc rc = %d\n", ret);
                return -1;
        }
        if (value <= 0) {
                chg_err("[OPLUS_CHG][%s]:USB_TEMPERATURE1 iio_read_channel_processed  get error\n", __func__);
                value = NTC_DEFAULT_VOLT_VALUE_MV;
                return -1;
        }
        value = value / THERMAL_TEMP_UNIT;

        if ((value >= BATID_LOW_MV1 && value <= BATID_HIGH_MV1)
                || (value >= BATID_LOW_MV2 && value <= BATID_HIGH_MV2)) {
                battery_type = BAT_TYPE_SDI_7500MA;
        }
        chg_err(" battery_id := %d\n", value);
        return battery_type;
}
static bool sm5602_get_battery_authenticate(void)
{
        int bat_type = BAT_TYPE_UNKNOWN;

        bat_type = battery_type_check();
        if (bat_type == BAT_TYPE_SDI_7500MA) {
                return true;
        } else {
                return false;
        }
}
static int sm5602_get_prev_battery_mvolts(void)
{
        if (global_sm == NULL)
                return VBAT_DEFAULT;

        fg_refresh_status(global_sm);

        return global_sm->batt_volt;
}

static int sm5602_get_prev_battery_temperature(void)
{
	if (global_sm == NULL)
		return TEMP_DEFAULT;

	/*fg_refresh_status(global_sm);*/
	if (global_sm->en_temp_in)
		global_sm->batt_temp = fg_read_temperature(global_sm, TEMPERATURE_IN);
	else if (global_sm->en_temp_ex)
		global_sm->batt_temp = fg_read_temperature(global_sm, TEMPERATURE_EX);
	else
		global_sm->batt_temp = -ENODATA;

	return global_sm->batt_temp * 10;
}

static int sm5602_get_prev_battery_soc(void)
{
	int ret = 0;

	if (global_sm == NULL)
		return SOC_DEFAULT;

	ret = fg_read_soc(global_sm);
	mutex_lock(&global_sm->data_lock);
	global_sm->batt_soc = ret/MULTIPLY_10;
	mutex_unlock(&global_sm->data_lock);

	return global_sm->batt_soc;
}

static int sm5602_get_prev_average_current(void)
{
        int ret;

        if (global_sm == NULL)
                return CHARGING_CURRENT_DEFAULT;
        ret = fg_read_current(global_sm);
        mutex_lock(&global_sm->data_lock);
        global_sm->batt_curr = (NUM0-ret);
        mutex_unlock(&global_sm->data_lock);

        return global_sm->batt_curr;
}

static int sm5602_get_battery_mvolts_2cell_max(void)
{
	int ret = 0;
	if (global_sm == NULL)
		return VBAT_DEFAULT;

	ret = fg_read_volt(global_sm);
	mutex_lock(&global_sm->data_lock);
	if (ret >= 0)
		global_sm->batt_volt = ret;
	mutex_unlock(&global_sm->data_lock);

	return global_sm->batt_volt;
}

static int sm5602_get_battery_mvolts_2cell_min(void)
{
	int ret = 0;
	if (global_sm == NULL)
		return VBAT_DEFAULT;

	ret = fg_read_volt(global_sm);
	mutex_lock(&global_sm->data_lock);
	if (ret >= 0)
		global_sm->batt_volt = ret;
	mutex_unlock(&global_sm->data_lock);

	return global_sm->batt_volt;
}

#define PREV_VBAT 3800
static int sm5602_get_prev_battery_mvolts_2cell_max(void)
{
        return PREV_VBAT;
}

static int sm5602_get_prev_battery_mvolts_2cell_min(void)
{
        return PREV_VBAT;
}

static int sm5602_update_battery_dod0(void)
{
        return NUM0;
}

static int sm5602_update_soc_smooth_parameter(void)
{
        return NUM0;
}

static int sm5602_get_battery_cc(void)
{
		if (global_sm == NULL)
                return NUM0;

        fg_refresh_status(global_sm);

        return global_sm->batt_soc_cycle;
}


static int sm5602_fg_read_rmc(struct sm_fg_chip *sm)
{
		int ret = 0;
		int rmc = 0;
		u16 data = 0;
		int64_t temp = 0;

		ret = fg_read_word(sm, 0x84, &data);
		if (ret < 0) {
				chg_err("could not read RMC, ret=%d\n", ret);
				return ret;
		} else {
				temp = div_s64((data & 0x7FFF) * 1000, 2048);
				rmc = temp;
		}

		return rmc;
}


#define REMAIN_CAPACITY 3000
static int sm5602_get_batt_remaining_capacity(void)
{
        int rmc = 0;
        rmc = sm5602_fg_read_rmc(global_sm);

        return rmc;
}

static int sm5602_get_prev_batt_remaining_capacity(void)
{
        int rmc = 0;
        rmc = sm5602_fg_read_rmc(global_sm);
        chg_err("sm5602_get_prev_batt_remaining_capacity rmc = %d\n", rmc);

        return rmc;
}




#define SOH_DEFAULT 100
#define BATTERY_CC_TOTAL 800
static int sm5602_get_battery_soh(void)
{
        int soh;

        if (global_sm == NULL)
                return SOH_DEFAULT;
        soh = BATTERY_CC_TOTAL - sm5602_get_battery_cc();
        return (soh / BATTERY_CC_TOTAL);
}

#define BATTERY_FC 7000
static int sm5602_get_battery_fc(void)
{
        return BATTERY_FC;
}

static bool sm5602_get_battery_hmac(void)
{
        return true;
}

static int sm5602_dumy(void)
{
        return NUM0;
}

static void sm5602_void_dumy(bool full)
{
        return;
}

static struct oplus_gauge_operations sm5602_gauge_ops = {
        .get_battery_temperature = sm5602_get_battery_temperature,
        .get_battery_mvolts = sm5602_get_battery_mvolts,
        .get_battery_soc = sm5602_get_battery_soc,
        .get_average_current = sm5602_get_average_current,
        .get_battery_fcc = sm5602_get_battery_fcc,
        .get_prev_batt_fcc = sm5602_get_prev_batt_fcc,
        .get_battery_authenticate = sm5602_get_battery_authenticate,
        .get_prev_battery_mvolts = sm5602_get_prev_battery_mvolts,
        .get_prev_battery_temperature = sm5602_get_prev_battery_temperature,
        .get_prev_battery_soc = sm5602_get_prev_battery_soc,
        .get_prev_average_current = sm5602_get_prev_average_current,
        .get_battery_mvolts_2cell_max = sm5602_get_battery_mvolts_2cell_max,
        .get_battery_mvolts_2cell_min = sm5602_get_battery_mvolts_2cell_min,
        .get_prev_battery_mvolts_2cell_max = sm5602_get_prev_battery_mvolts_2cell_max,
        .get_prev_battery_mvolts_2cell_min = sm5602_get_prev_battery_mvolts_2cell_min,
        .update_battery_dod0 = sm5602_update_battery_dod0,
        .update_soc_smooth_parameter = sm5602_update_soc_smooth_parameter,
        .get_battery_cc = sm5602_get_battery_cc,
        .get_batt_remaining_capacity = sm5602_get_batt_remaining_capacity,
        .get_prev_batt_remaining_capacity = sm5602_get_prev_batt_remaining_capacity,
        .get_battery_soh = sm5602_get_battery_soh,
        .get_battery_fc = sm5602_get_battery_fc,
        .get_battery_hmac = sm5602_get_battery_hmac,
        .set_battery_full = sm5602_void_dumy,
        .get_battery_cb_status = sm5602_dumy,
        .dump_register = sm5602_dumy,
};

static int sm_fg_probe(struct i2c_client *client,
                                                        const struct i2c_device_id *id)
{
        int ret;
        struct sm_fg_chip *sm;
        u8 *regs;
        struct oplus_gauge_chip	*chip;

        sm = devm_kzalloc(&client->dev, sizeof(*sm), GFP_KERNEL);
        if (!sm)
                return -ENOMEM;

        sm->dev = &client->dev;
        sm->client = client;
        sm->chip = id->driver_data;

        sm->batt_soc	= -ENODATA;
        sm->batt_fcc	= -ENODATA;
        sm->batt_volt	= -ENODATA;
        sm->batt_temp	= -ENODATA;
        sm->batt_curr	= -ENODATA;
        sm->fake_soc	= -EINVAL;
        sm->fake_temp	= -EINVAL;

        if (sm->chip == SM5602) {
                regs = sm5602_regs;
        } else {
                chg_err("unexpected fuel gauge: %d\n", sm->chip);
                regs = sm5602_regs;
        }

        memcpy(sm->regs, regs, NUMREGS);
        i2c_set_clientdata(client, sm);
        mutex_init(&sm->i2c_rw_lock);
        mutex_init(&sm->data_lock);

        if (!hal_fg_init(client)) {
                chg_err("Failed to Initialize Fuelgauge\n");
                return ret;
        }

        INIT_DELAYED_WORK(&sm->monitor_work, fg_monitor_workfunc);

        create_debugfs_entry(sm);

        fg_dump_debug(sm);
        global_sm = sm;
        schedule_delayed_work(&sm->monitor_work, MULTIPLY_10 * HZ);

        chip = devm_kzalloc(&client->dev,
                sizeof(struct oplus_gauge_chip), GFP_KERNEL);
        if (!chip) {
                chg_err("kzalloc() failed.\n");
                return -ENOMEM;
        }
        chip->client = client;
        chip->dev = &client->dev;
        chip->gauge_ops = &sm5602_gauge_ops;
        oplus_gauge_init(chip);

        return NUM0;
}


static int sm_fg_remove(struct i2c_client *client)
{
        struct sm_fg_chip *sm = i2c_get_clientdata(client);

        cancel_delayed_work_sync(&sm->monitor_work);

        mutex_destroy(&sm->data_lock);
        mutex_destroy(&sm->i2c_rw_lock);

        debugfs_remove_recursive(sm->debug_root);

        return NUM0;
}

static void sm_fg_shutdown(struct i2c_client *client)
{
        chg_debug("sm fuel gauge driver shutdown!\n");
}

static const struct of_device_id sm_fg_match_table[] = {
        {.compatible = "sm,sm5602", },
        {},
};
MODULE_DEVICE_TABLE(of, sm_fg_match_table);

static const struct i2c_device_id sm_fg_id[] = {
        { "sm5602", SM5602 },
        {},
};
MODULE_DEVICE_TABLE(i2c, sm_fg_id);

static struct i2c_driver sm_fg_driver = {
        .driver		= {
                .name		= "sm5602",
                .owner		= THIS_MODULE,
                .of_match_table	= sm_fg_match_table,
        },
        .id_table   = sm_fg_id,
        .probe		= sm_fg_probe,
        .remove		= sm_fg_remove,
        .shutdown         = sm_fg_shutdown,
};

module_i2c_driver(sm_fg_driver);

MODULE_DESCRIPTION("SM5602 Gauge Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SM5602-gauge");

