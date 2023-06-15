// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2022 Oplus. All rights reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/sizes.h>
#include <linux/iio/consumer.h>
#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include <linux/hardware_info.h>

#define CWFG_ENABLE_LOG 1 /* CHANGE Customer need to change this for enable/disable log */

#define REG_CHIP_ID             0x00
#define REG_VCELL_H             0x02
#define REG_VCELL_L             0x03
#define REG_SOC_INT             0x04
#define REG_SOC_DECIMAL         0x05
#define REG_TEMP                0x06
#define REG_MODE_CONFIG         0x08
#define REG_GPIO_CONFIG         0x0A
#define REG_SOC_ALERT           0x0B
#define REG_TEMP_MAX            0x0C
#define REG_TEMP_MIN            0x0D
#define REG_CURRENT_H           0x0E
#define REG_CURRENT_L           0x0F
#define REG_T_HOST_H            0xA0
#define REG_T_HOST_L            0xA1
#define REG_USER_CONF           0xA2
#define REG_CYCLE_H             0xA4
#define REG_CYCLE_L             0xA5
#define REG_SOH                 0xA6
#define REG_IC_STATE            0xA7
#define REG_STB_CUR_H           0xA8
#define REG_STB_CUR_L           0xA9
#define REG_FW_VERSION          0xAB
#define REG_BAT_PROFILE         0x10

#define CONFIG_MODE_RESTART     0x30
#define CONFIG_MODE_ACTIVE      0x00
#define CONFIG_MODE_SLEEP       0xF0
#define CONFIG_UPDATE_FLG       0x80
#define IC_VCHIP_ID             0xA0
#define IC_READY_MARK           0x0C

#define GPIO_ENABLE_MIN_TEMP    0
#define GPIO_ENABLE_MAX_TEMP    0
#define GPIO_ENABLE_SOC_CHANGE  0
#define GPIO_SOC_IRQ_VALUE      0x0    /* 0x7F */
#define DEFINED_MAX_TEMP        45
#define DEFINED_MIN_TEMP        0

#define CWFG_NAME               "cw2217"
#define SIZE_OF_PROFILE         80
#define USER_RSENSE             5

#define QUEUE_DELAYED_WORK_TIME  5000
#define QUEUE_START_WORK_TIME    50

#define CW_REG_WORD             2
#define CW_REG_BYTE             1
#define CW_REG_BYTE_BITS        8
#define CW_SLEEP_1MS            1
#define CW_SLEEP_20MS           20
#define CW_SLEEP_10MS           10
#define CW_VOL_MAGIC_PART1      5
#define CW_VOL_MAGIC_PART2      16
#define CW_UI_FULL              100
#define CW_SOC_MAGIC_BASE       256
#define CW_SOC_MAGIC_100        100
#define CW_TEMP_MAGIC_PART1     10
#define CW_TEMP_MAGIC_PART2     2
#define CW_TEMP_MAGIC_PART3     400
#define COMPLEMENT_CODE_U16     0x8000
#define CW_CUR_MAGIC_PART1      160
#define CW_CUR_MAGIC_PART2      100
#define CW_STB_CUR_MAGIC_PART3  16
#define CW_SLEEP_100MS          100
#define CW_SLEEP_200MS          200
#define CW_SLEEP_COUNTS         50
#define CW_TRUE                 1
#define CW_RETRY_COUNT          3
#define CW_VOL_UNIT             1000
#define CW_CYCLE_MAGIC          16

#define CW2217_NOT_ACTIVE          1
#define CW2217_PROFILE_NOT_READY   2
#define CW2217_PROFILE_NEED_UPDATE 3

#define NUM_0 0
#define NUM_1 1
#define ERR_NUM -1

#define cw_printk(fmt, arg...) {                                                                          \
		if (CWFG_ENABLE_LOG)                                                   \
			printk("FG_CW2217 : %s-%d : " fmt, __FUNCTION__ , __LINE__, ##arg);  \
		else {}                                                                \
	}

#define BATNUM 2
static char *battery_name[BATNUM] = {"BLT004-ALT-7100MA"};
static unsigned char config_profile_info[SIZE_OF_PROFILE] = {
		0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
		0x95, 0xC7, 0xC0, 0xC8, 0xBF, 0xBA, 0xA0, 0x5F,
		0x3F, 0xE5, 0xCD, 0xAC, 0x82, 0x6D, 0x59, 0x4E,
		0x46, 0x42, 0x39, 0x57, 0xED, 0xDE, 0x6D, 0xE4,
		0xD5, 0xD2, 0xD5, 0xD7, 0xD4, 0xD2, 0xCD, 0xC9,
		0xC5, 0xC4, 0xCB, 0xBC, 0x98, 0x8C, 0x83, 0x74,
		0x64, 0x64, 0x7B, 0x8E, 0xA6, 0x98, 0x52, 0x66,
		0x20, 0x00, 0x57, 0x10, 0x00, 0xB2, 0x89, 0x04,
		0x00, 0x00, 0x64, 0x24, 0xC1, 0x7B, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65,
};

struct cw_battery {
	struct i2c_client *client;
	struct device		*dev;
	struct workqueue_struct *cwfg_workqueue;
	struct delayed_work battery_delay_work;
	int  chip_id;
	int  voltage;
	int  ic_soc_h;
	int  ic_soc_l;
	int  ui_soc;
	int  temp;
	long cw_current;
	int  cycle;
	int  soh;
	int  fw_version;
	struct iio_channel	*batt_id_chan;
};

struct cw_battery *g_cw_bat = NULL;

/* CW2217 iic read function */
static int cw_read(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, CW_REG_BYTE, buf);
	if (ret < NUM_0)
		chg_err("IIC error %d\n", ret);

	return ret;
}

/* CW2217 iic write function */
static int cw_write(struct i2c_client *client, unsigned char reg, unsigned char const buf[])
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, CW_REG_BYTE, &buf[NUM_0]);
	if (ret < NUM_0)
		chg_err("IIC error %d\n", ret);

	return ret;
}

/* CW2217 iic read word function */
static int cw_read_word(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret;
	unsigned char reg_val[CW_REG_WORD] = { NUM_0, NUM_0 };
	unsigned int temp_val_buff;
	unsigned int temp_val_second;

	msleep(CW_SLEEP_1MS);
	ret = i2c_smbus_read_i2c_block_data(client, reg, CW_REG_WORD, reg_val);
	if (ret < NUM_0)
		chg_err("IIC error %d\n", ret);
	temp_val_buff = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];

	msleep(CW_SLEEP_1MS);
	ret = i2c_smbus_read_i2c_block_data(client, reg, CW_REG_WORD, reg_val);
	if (ret < NUM_0)
		chg_err("IIC error %d\n", ret);
	temp_val_second = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];

	if (temp_val_buff != temp_val_second) {
		msleep(CW_SLEEP_1MS);
		ret = i2c_smbus_read_i2c_block_data(client, reg, CW_REG_WORD, reg_val);
		if (ret < NUM_0)
			chg_err("IIC error %d\n", ret);
		temp_val_buff = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];
	}

	buf[NUM_0] = reg_val[NUM_0];
	buf[NUM_1] = reg_val[NUM_1];

	return ret;
}

/* CW2217 iic write profile function */
static int cw_write_profile(struct i2c_client *client, unsigned char const buf[])
{
	int ret;
	int i;

	for (i = NUM_0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_write(client, REG_BAT_PROFILE + i, &buf[i]);
		if (ret < NUM_0) {
			chg_err("IIC error %d\n", ret);
			return ret;
		}
	}

	return ret;
}

/*
 * CW2217 Active function
 * The CONFIG register is used for the host MCU to configure the fuel gauge IC. The default value is 0xF0,
 * SLEEP and RESTART bits are set. To power up the IC, the host MCU needs to write 0x30 to exit shutdown
 * mode, and then write 0x00 to restart the gauge to enter active mode. To reset the IC, the host MCU needs
 * to write 0xF0, 0x30 and 0x00 in sequence to this register to complete the restart procedure. The CW2217B
 * will reload relevant parameters and settings and restart SOC calculation. Note that the SOC may be a
 * different value after reset operation since it is a brand-new calculation based on the latest battery status.
 * CONFIG [3:0] is reserved. Don't do any operation with it.
 */
static int cw2217_active(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val = CONFIG_MODE_RESTART;

	chg_err("\n");

	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;
	msleep(CW_SLEEP_20MS);  /* Here delay must >= 20 ms */

	reg_val = CONFIG_MODE_ACTIVE;
	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;
	msleep(CW_SLEEP_10MS);

	return NUM_0;
}

/*
 * CW2217 Sleep function
 * The CONFIG register is used for the host MCU to configure the fuel gauge IC. The default value is 0xF0,
 * SLEEP and RESTART bits are set. To power up the IC, the host MCU needs to write 0x30 to exit shutdown
 * mode, and then write 0x00 to restart the gauge to enter active mode. To reset the IC, the host MCU needs
 * to write 0xF0, 0x30 and 0x00 in sequence to this register to complete the restart procedure. The CW2217B
 * will reload relevant parameters and settings and restart SOC calculation. Note that the SOC may be a
 * different value after reset operation since it is a brand-new calculation based on the latest battery status.
 * CONFIG [3:0] is reserved. Don't do any operation with it.
 */
static int cw2217_sleep(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val = CONFIG_MODE_RESTART;

	chg_err("\n");

	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;
	msleep(CW_SLEEP_20MS);  /* Here delay must >= 20 ms */

	reg_val = CONFIG_MODE_SLEEP;
	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;
	msleep(CW_SLEEP_10MS);

	return NUM_0;
}

/*
 * The 0x00 register is an UNSIGNED 8bit read-only register. Its value is fixed to 0xA0 in shutdown
 * mode and active mode.
 */
static int cw_get_chip_id(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int chip_id;

	ret = cw_read(cw_bat->client, REG_CHIP_ID, &reg_val);
	if (ret < NUM_0)
		return ret;

	chip_id = reg_val;  /* This value must be 0xA0! */
	chg_err("chip_id = %d\n", chip_id);
	cw_bat->chip_id = chip_id;

	return NUM_0;
}

/*
 * The VCELL register(0x02 0x03) is an UNSIGNED 14bit read-only register that updates the battery voltage continuously.
 * Battery voltage is measured between the VCELL pin and VSS pin, which is the ground reference. A 14bit
 * sigma-delta A/D converter is used and the voltage resolution is 312.5uV. (0.3125mV is *5/16)
 */
static int cw_get_voltage(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[CW_REG_WORD] = {NUM_0 , NUM_0};
	unsigned int voltage;

	ret = cw_read_word(cw_bat->client, REG_VCELL_H, reg_val);
	if (ret < NUM_0)
		return ret;
	voltage = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];
	voltage = voltage  * CW_VOL_MAGIC_PART1 / CW_VOL_MAGIC_PART2;
	cw_bat->voltage = voltage;

	return NUM_0;
}

/*
 * The SOC register(0x04 0x05) is an UNSIGNED 16bit read-only register that indicates the SOC of the battery. The
 * SOC shows in % format, which means how much percent of the battery's total available capacity is
 * remaining in the battery now. The SOC can intrinsically adjust itself to cater to the change of battery status,
 * including load, temperature and aging etc.
 * The high byte(0x04) contains the SOC in 1% unit which can be directly used if this resolution is good
 * enough for the application. The low byte(0x05) provides more accurate fractional part of the SOC and its
 * LSB is (1/256) %.
 */
static int cw_get_capacity(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[CW_REG_WORD] = { NUM_0, NUM_0 };
	int ui_100 = CW_UI_FULL;
	int soc_h;
	int soc_l;
	int ui_soc;
	int remainder;

	ret = cw_read_word(cw_bat->client, REG_SOC_INT, reg_val);
	if (ret < NUM_0)
		return ret;
	soc_h = reg_val[NUM_0];
	soc_l = reg_val[NUM_1];
	ui_soc = ((soc_h * CW_SOC_MAGIC_BASE + soc_l) * CW_SOC_MAGIC_100)/ (ui_100 * CW_SOC_MAGIC_BASE);
	remainder = (((soc_h * CW_SOC_MAGIC_BASE + soc_l) * CW_SOC_MAGIC_100 * CW_SOC_MAGIC_100) / (ui_100 * CW_SOC_MAGIC_BASE)) % CW_SOC_MAGIC_100;
	if (ui_soc >= CW_SOC_MAGIC_100) {
		chg_err("CW2015[%d]: UI_SOC = %d larger 100!!!!\n", __LINE__, ui_soc);
		ui_soc = CW_SOC_MAGIC_100;
	}
	cw_bat->ic_soc_h = soc_h;
	cw_bat->ic_soc_l = soc_l;
	cw_bat->ui_soc = ui_soc;

	return NUM_0;
}

/*
 * The TEMP register is an UNSIGNED 8bit read only register.
 * It reports the real-time battery temperature
 * measured at TS pin. The scope is from -40 to 87.5 degrees Celsius,
 * LSB is 0.5 degree Celsius. TEMP(C) = - 40 + Value(0x06 Reg) / 2
 */
static int cw_get_temp(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int temp;

	ret = cw_read(cw_bat->client, REG_TEMP, &reg_val);
	if (ret < NUM_0)
		return ret;

	temp = (int)reg_val * CW_TEMP_MAGIC_PART1 / CW_TEMP_MAGIC_PART2 - CW_TEMP_MAGIC_PART3;
	cw_bat->temp = temp;

	return NUM_0;
}

/* get complement code function, unsigned short must be U16 */
static long get_complement_code(unsigned short raw_code)
{
	long complement_code;
	int dir;

	if (NUM_0 != (raw_code & COMPLEMENT_CODE_U16)) {
		dir = ERR_NUM;
		raw_code =  (~raw_code) + NUM_1;
	} else {
		dir = NUM_1;
	}
	complement_code = (long)raw_code * dir;

	return complement_code;
}

/*
 * CURRENT is a SIGNED 16bit register(0x0E 0x0F) that reports current A/D converter result of the voltage across the
 * current sense resistor, 10mohm typical. The result is stored as a two's complement value to show positive
 * and negative current. Voltages outside the minimum and maximum register values are reported as the
 * minimum or maximum value.
 * The register value should be divided by the sense resistance to convert to amperes. The value of the
 * sense resistor determines the resolution and the full-scale range of the current readings. The LSB of 0x0F
 * is (52.4/32768)uV.
 * The default value is 0x0000, stands for 0mA. 0x7FFF stands for the maximum charging current and 0x8001 stands for
 * the maximum discharging current.
 */
static int cw_get_current(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[CW_REG_WORD] = {NUM_0 , NUM_0};
	long cw_current;
	unsigned short current_reg;  /* unsigned short must u16 */

	ret = cw_read_word(cw_bat->client, REG_CURRENT_H, reg_val);
	if (ret < NUM_0)
		return ret;

	current_reg = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];
	cw_current = get_complement_code(current_reg);
	cw_current = cw_current  * CW_CUR_MAGIC_PART1 / USER_RSENSE / CW_CUR_MAGIC_PART2;
	cw_bat->cw_current = cw_current;

	return NUM_0;
}

/*
 * CYCLECNT is an UNSIGNED 16bit register(0xA4 0xA5) that counts cycle life of the battery. The LSB of 0xA5 stands
 * for 1/16 cycle. This register will be clear after enters shutdown mode
 */
static int cw_get_cycle_count(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[CW_REG_WORD] = {NUM_0, NUM_0};
	int cycle;

	ret = cw_read_word(cw_bat->client, REG_CYCLE_H, reg_val);
	if (ret < NUM_0)
		return ret;

	cycle = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];
	cw_bat->cycle = cycle / CW_CYCLE_MAGIC;

	return NUM_0;
}

/*
 * SOH (State of Health) is an UNSIGNED 8bit register(0xA6) that represents the level of battery aging by tracking
 * battery internal impedance increment. When the device enters active mode, this register refresh to 0x64
 * by default. Its range is 0x00 to 0x64, indicating 0 to 100%. This register will be clear after enters shutdown
 * mode.
 */
static int cw_get_soh(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int soh;

	ret = cw_read(cw_bat->client, REG_SOH, &reg_val);
	if (ret < NUM_0)
		return ret;

	soh = reg_val;
	cw_bat->soh = soh;
	return NUM_0;
}

/*
 * FW_VERSION register reports the firmware (FW) running in the chip. It is fixed to 0x00 when the chip is
 * in shutdown mode. When in active mode, Bit [7:6] are fixed to '01', which stand for the CW2217B and Bit
 * [5:0] stand for the FW version running in the chip. Note that the FW version is subject to update and contact
 * sales office for confirmation when necessary.
*/
static int cw_get_fw_version(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int fw_version;

	ret = cw_read(cw_bat->client, REG_FW_VERSION, &reg_val);
	if (ret < NUM_0)
		return ret;

	fw_version = reg_val;
	cw_bat->fw_version = fw_version;

	return NUM_0;
}

static int cw_update_data(struct cw_battery *cw_bat)
{
	int ret = NUM_0;

	ret += cw_get_voltage(cw_bat);
	ret += cw_get_capacity(cw_bat);
	ret += cw_get_temp(cw_bat);
	ret += cw_get_current(cw_bat);
	ret += cw_get_cycle_count(cw_bat);
	ret += cw_get_soh(cw_bat);
	chg_err("vol = %d  current = %ld cap = %d temp = %d\n",
		cw_bat->voltage, cw_bat->cw_current, cw_bat->ui_soc, cw_bat->temp);

	return ret;
}

static int cw_init_data(struct cw_battery *cw_bat)
{
	int ret = NUM_0;

	ret += cw_get_chip_id(cw_bat);
	ret += cw_get_voltage(cw_bat);
	ret += cw_get_capacity(cw_bat);
	ret += cw_get_temp(cw_bat);
	ret += cw_get_current(cw_bat);
	ret += cw_get_cycle_count(cw_bat);
	ret += cw_get_soh(cw_bat);
	ret += cw_get_fw_version(cw_bat);
	chg_err("chip_id = %d vol = %d  cur = %ld cap = %d temp = %d  fw_version = %d\n",
		cw_bat->chip_id, cw_bat->voltage, cw_bat->cw_current, cw_bat->ui_soc, cw_bat->temp, cw_bat->fw_version);

	return ret;
}

/*CW2217 update profile function, Often called during initialization*/
static int cw_config_start_ic(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int count = NUM_0;

	ret = cw2217_sleep(cw_bat);
	if (ret < NUM_0)
		return ret;

	/* update new battery info */
	ret = cw_write_profile(cw_bat->client, config_profile_info);
	if (ret < NUM_0)
		return ret;

	/* set UPDATE_FLAG AND SOC INTTERRUP VALUE*/
	reg_val = CONFIG_UPDATE_FLG | GPIO_SOC_IRQ_VALUE;
	ret = cw_write(cw_bat->client, REG_SOC_ALERT, &reg_val);
	if (ret < NUM_0)
		return ret;

	/*close all interruptes*/
	reg_val = NUM_0;
	ret = cw_write(cw_bat->client, REG_GPIO_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;

	ret = cw2217_active(cw_bat);
	if (ret < NUM_0)
		return ret;

	while (CW_TRUE) {
		msleep(CW_SLEEP_100MS);
		cw_read(cw_bat->client, REG_IC_STATE, &reg_val);
		if (IC_READY_MARK == (reg_val & IC_READY_MARK))
			break;
		count++;
		if (count >= CW_SLEEP_COUNTS) {
			cw2217_sleep(cw_bat);
			return ERR_NUM;
		}
	}

	return NUM_0;
}

/*
 * Get the cw2217 running state
 * Determine whether the profile needs to be updated
*/
static int cw2217_get_state(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int i;
	int reg_profile;

	ret = cw_read(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;
	if (reg_val != CONFIG_MODE_ACTIVE)
		return CW2217_NOT_ACTIVE;

	ret = cw_read(cw_bat->client, REG_SOC_ALERT, &reg_val);
	if (ret < NUM_0)
		return ret;
	if (NUM_0 == (reg_val & CONFIG_UPDATE_FLG))
		return CW2217_PROFILE_NOT_READY;

	for (i = NUM_0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_read(cw_bat->client, (REG_BAT_PROFILE + i), &reg_val);
		if (ret < NUM_0)
			return ret;
		reg_profile = REG_BAT_PROFILE + i;
		chg_err("0x%2x = 0x%2x\n", reg_profile, reg_val);
		if (config_profile_info[i] != reg_val)
			break;
	}
	if (i != SIZE_OF_PROFILE)
		return CW2217_PROFILE_NEED_UPDATE;

	return NUM_0;
}

/*CW2217 init function, Often called during initialization*/
static int cw_init(struct cw_battery *cw_bat)
{
	int ret;

	chg_err("\n");
	ret = cw_get_chip_id(cw_bat);
	if (ret < NUM_0) {
		chg_err("iic read write error");
		return ret;
	}
	if (cw_bat->chip_id != IC_VCHIP_ID) {
		chg_err("not cw2217B\n");
		return ERR_NUM;
	}

	ret = cw2217_get_state(cw_bat);
	if (ret < NUM_0) {
		chg_err("iic read write error");
		return ret;
	}

	if (ret != NUM_0) {
		ret = cw_config_start_ic(cw_bat);
		if (ret < NUM_0)
			return ret;
	}
	chg_err("cw2217 init success!\n");

	return NUM_0;
}

static void cw_bat_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct cw_battery *cw_bat;
	int ret;

	delay_work = container_of(work, struct delayed_work, work);
	cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

	ret = cw_update_data(cw_bat);
	if (ret < NUM_0)
		chg_err("iic read error when update data");

	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(QUEUE_DELAYED_WORK_TIME));
}

static int cw2217_dump(void)
{
	int ret = NUM_0;
	return ret;
}

#define DESIGN_CAPACITY					7100

static int cw2217_get_battery_mvolts(void)
{
        int ret = NUM_0;

        ret = cw_get_voltage(g_cw_bat);
        return g_cw_bat->voltage;
}

static int  cw2217_get_battery_fc(void)
{
        return DESIGN_CAPACITY;
}

static int cw2217_get_battery_temperature(void)
{
        int ret = NUM_0;

        ret = cw_get_temp(g_cw_bat);
        return g_cw_bat->temp;
}

static int cw2217_get_batt_remaining_capacity(void)
{
        return ERR_NUM;
}

static int cw2217_get_battery_soc(void)
{
        int soc = NUM_0;
        cw_get_capacity(g_cw_bat);
		soc = g_cw_bat->ui_soc;
        return soc;
}

static int cw2217_get_average_current(void)
{
        cw_get_current(g_cw_bat);
		return -g_cw_bat->cw_current;
}

static bool cw2217_get_battery_hmac(void)
{
        return true;
}

static void cw2217_void_dumy(bool full)
{
        /* Do nothing */
}

#define NTC_DEFAULT_VOLT_VALUE_MV 950
#define BATID_LOW_MV1 200
#define BATID_HIGH_MV1 320
#define BATID_LOW_MV2 719
#define BATID_HIGH_MV2 820
#define THERMAL_TEMP_UNIT      1000
enum {
        BAT_TYPE_UNKNOWN,
        BAT_TYPE_SDI_7100MA,
};
static int oplus_get_iio_channel(struct cw_battery *cw_bat, const char *propname,
                        struct iio_channel **chan)
{
        int rc = NUM_0;

        rc = of_property_match_string(cw_bat->dev->of_node,
                        "io-channel-names", propname);
        if (rc < NUM_0)
                return rc;

        *chan = iio_channel_get(cw_bat->dev, propname);
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
        int ret = ERR_NUM;
        int value = NUM_0;

        if (!g_cw_bat->batt_id_chan) {
                chg_err("[OPLUS_CHG][%s]: chip not ready!\n", __func__);
                ret = oplus_get_iio_channel(g_cw_bat, "batt_id_chan", &g_cw_bat->batt_id_chan);
                if (ret < NUM_0 && !g_cw_bat->batt_id_chan) {
                        chg_err(" %s usb_temp1 get failed\n", __func__);
                return ERR_NUM;
        }
        }
        ret = iio_read_channel_processed(g_cw_bat->batt_id_chan, &value);
        if (ret < NUM_0) {
        chg_err("fail to read usb_temp1 adc rc = %d\n", ret);
                return ERR_NUM;
        }
        if (value <= NUM_0) {
                chg_err("[OPLUS_CHG][%s]:USB_TEMPERATURE1 iio_read_channel_processed  get error\n", __func__);
                value = NTC_DEFAULT_VOLT_VALUE_MV;
                return ERR_NUM;
        }
        value = value / THERMAL_TEMP_UNIT;

        if ((value >= BATID_LOW_MV1 && value <= BATID_HIGH_MV1)
                || (value >= BATID_LOW_MV2 && value <= BATID_HIGH_MV2)) {
                battery_type = BAT_TYPE_SDI_7100MA;
        }
        chg_err(" battery_id := %d\n", value);
        return battery_type;
}

bool cw2217_get_battery_authenticate(void)
{
        int bat_type = BAT_TYPE_UNKNOWN;

        bat_type = battery_type_check();
        if (bat_type == BAT_TYPE_SDI_7100MA) {
                return true;
        } else {
                return false;
        }
}

int cw2217_get_battery_soh(void)
{
	cw_get_soh(g_cw_bat);
	return g_cw_bat->soh;
}

int cw2217_get_battery_mvolts_2cell_max(void)
{
	return cw2217_get_battery_mvolts();
}

int cw2217_get_battery_mvolts_2cell_min(void)
{
	return cw2217_get_battery_mvolts();
}

#define PREV_VBAT 3800

int cw2217_prev_battery_mvolts_2cell_max(void)
{
	return PREV_VBAT;
}

int cw2217_prev_battery_mvolts_2cell_min(void)
{
	return PREV_VBAT;
}

static struct oplus_gauge_operations battery_cw2217_gauge = {
        .get_battery_mvolts			= cw2217_get_battery_mvolts,
        .get_battery_fc				= cw2217_get_battery_fc,
        .get_battery_temperature		= cw2217_get_battery_temperature,
        .get_batt_remaining_capacity		= cw2217_get_batt_remaining_capacity,
        .get_battery_soc			= cw2217_get_battery_soc,
        .get_average_current			= cw2217_get_average_current,
        .get_battery_fcc			= cw2217_get_battery_fc,
        .get_battery_cc				= cw2217_get_battery_fc,
        .get_prev_batt_fcc			= cw2217_get_battery_fc,
        .get_battery_authenticate		= cw2217_get_battery_authenticate,
        .get_battery_hmac			= cw2217_get_battery_hmac,
        .get_prev_battery_mvolts		= cw2217_get_battery_mvolts,
        .get_prev_battery_temperature		= cw2217_get_battery_temperature,
        .set_battery_full			= cw2217_void_dumy,
        .get_prev_battery_soc			= cw2217_get_battery_soc,
        .get_prev_average_current		= cw2217_get_average_current,
        .get_prev_batt_remaining_capacity	= cw2217_get_batt_remaining_capacity,
        .get_battery_mvolts_2cell_max		= cw2217_get_battery_mvolts_2cell_max,
        .get_battery_mvolts_2cell_min		= cw2217_get_battery_mvolts_2cell_min,
        .get_prev_battery_mvolts_2cell_max	= cw2217_prev_battery_mvolts_2cell_max,
        .get_prev_battery_mvolts_2cell_min	= cw2217_prev_battery_mvolts_2cell_min,
        .update_battery_dod0			= cw2217_dump,
        .update_soc_smooth_parameter		= cw2217_dump,
        .get_battery_cb_status			= cw2217_dump,
        .get_battery_soh			= cw2217_get_battery_soh,
        .dump_register				= cw2217_dump,
};

static int cw2217_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	int loop = NUM_0;
	struct cw_battery *cw_bat;
	struct oplus_gauge_chip	*chip;

	chg_err("\n");

	cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
	if (!cw_bat) {
		chg_err("%s : cw_bat create fail!\n", __func__);
		return -ENOMEM;
	}
	cw_bat->dev = &client->dev;
	i2c_set_clientdata(client, cw_bat);
	cw_bat->client = client;

	ret = cw_init(cw_bat);
	while ((loop++ < CW_RETRY_COUNT) && (ret != 0)) {
		msleep(CW_SLEEP_200MS);
		ret = cw_init(cw_bat);
	}
	if (ret) {
		chg_err("%s : cw2217 init fail!\n", __func__);
		return ret;
	}

	ret = cw_init_data(cw_bat);
	if (ret) {
		chg_err("%s : cw2217 init data fail!\n", __func__);
		return ret;
	}

	cw_bat->cwfg_workqueue = create_singlethread_workqueue("cwfg_gauge");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work , msecs_to_jiffies(QUEUE_START_WORK_TIME));

	chg_err("cw2217 driver probe success!\n");

	chip = (struct oplus_gauge_chip*) kzalloc(sizeof(struct oplus_gauge_chip),
                        GFP_KERNEL);
	if (!chip) {
		chg_err("oplus_gauge_chip devm_kzalloc failed.\n");
		return -ENOMEM;
	}

	chip->gauge_ops = &battery_cw2217_gauge;
	oplus_gauge_init(chip);
	g_cw_bat = cw_bat;
	hardwareinfo_set_prop(HARDWARE_BATTERY_ID, battery_name[NUM_0]);
	hardwareinfo_set_prop(HARDWARE_BMS_GAUGE, "CW2217");
	return NUM_0;
}

static int cw2217_remove(struct i2c_client *client)
{
	chg_err("\n");
	return NUM_0;
}

#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	cancel_delayed_work(&cw_bat->battery_delay_work);
	return NUM_0;
}

static int cw_bat_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(20));
	return NUM_0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
	.suspend  = cw_bat_suspend,
	.resume   = cw_bat_resume,
};
#endif

static const struct i2c_device_id cw2217_id_table[] = {
	{ CWFG_NAME, NUM_0 },
	{ }
};

static struct of_device_id cw2217_match_table[] = {
	{ .compatible = "cellwise,cw2217", },
	{ },
};

static struct i2c_driver cw2217_driver = {
	.driver   = {
		.name = CWFG_NAME,
#ifdef CONFIG_PM
		.pm = &cw_bat_pm_ops,
#endif
		.owner = THIS_MODULE,
		.of_match_table = cw2217_match_table,
	},
	.probe = cw2217_probe,
	.remove = cw2217_remove,
	.id_table = cw2217_id_table,
};

static int __init cw2217_init(void)
{
	chg_err("\n");
	i2c_add_driver(&cw2217_driver);
	return NUM_0;
}

static void __exit cw2217_exit(void)
{
	i2c_del_driver(&cw2217_driver);
}

module_init(cw2217_init);
module_exit(cw2217_exit);

MODULE_DESCRIPTION("CW2217 FGADC Device Driver V1.2");
MODULE_LICENSE("GPL");
