/************************************************************************************
** VENDOR_EDIT
** Copyright (C), 2018-2019, oppo Mobile Comm Corp., Ltd
**
** Description:
**    For sgm41512 charger ic driver
**
** Version: 1.0
** Date created: 2018-09-24
**
** --------------------------- Revision History: ------------------------------------
* <version>       <date>         <author>              			<desc>
*************************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>
//#include <mt-plat/charger_type.h>
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#ifdef CONFIG_OPPO_CHARGER_6750T
#include <linux/module.h>
#include <upmu_common.h>
#include <mt-plat/mt_gpio.h>
#include <mt_boot_common.h>
#include <mt-plat/mtk_rtc.h>
#elif defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_CHARGER_SGM41512)
#include <linux/module.h>
//#include <mt-plat/mtk_gpio.h>
#include <linux/of_gpio.h>
//#include <mt-plat/mtk_rtc.h>

#else
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_boot.h>
#include <mach/mtk_rtc.h>
#endif
#include <soc/oppo/device_info.h>

#else

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

#include <mach/oplus_boot_mode.h>
#include <soc/oppo/device_info.h>

void (*enable_aggressive_segmentation_fn)(bool);

#endif

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "oppo_sgm41512.h"
#include "../oplus_charger.h"
#include <charger_class.h>
//#include <mt-plat/charger_class.h>
//#include <mt-plat/charger_type.h>
//#include "../../mediatek/charger/mtk_charger_intf.h"
//#include <mt-plat/mtk_rtc.h>
extern int oplus_get_rtc_ui_soc(void);
extern int oplus_set_rtc_ui_soc(int value);
extern signed int mt6357_get_vbus_voltage(void);
static struct chip_sgm41512 *charger_ic = NULL;
static int aicl_result = 500;
static struct delayed_work charger_modefy_work;

static DEFINE_MUTEX(sgm41512_i2c_access);

static int sgm41512_unsuspend_charger(void);
static int sgm41512_suspend_charger(void);
static int sgm41512_enable_charging(void);
static int sgm41512_disable_charging(void);

struct chip_sgm41512 {
	struct i2c_client	*client;
	struct device		*dev;

	/*mtk charger relate*/
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	const char *chg_dev_name;
	enum charger_type chg_type;

	int				hw_aicl_point;
	int				sw_aicl_point;
	atomic_t			charger_suspended;
	int				irq_gpio;
	int 			pwr_gd;
	int				chip_id;
};

enum power_supply_type charger_type_sgm = POWER_SUPPLY_TYPE_UNKNOWN;

static unsigned int bmt_find_closest_level(const unsigned int *pList,
		unsigned int number,
		unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = 1;
	else
		max_value_in_last_element = 0;

	if (max_value_in_last_element == 1) {
		for (i = (number - 1); i != 0;
		     i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				pr_debug_ratelimited("zzf_%d<=%d, i=%d\n",
					pList[i], level, i);
				return pList[i];
			}
		}

		pr_info("Can't find closest level\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		/* max value in the first element */
		for (i = 0; i < number; i++) {
			if (pList[i] <= level)
				return pList[i];
		}

		pr_info("Can't find closest level\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

unsigned int charging_parameter_to_value(const unsigned int
		*parameter, const unsigned int array_size,
		const unsigned int val)
{
	unsigned int i;

	pr_debug_ratelimited("array_size = %d\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	pr_info("NO register value match\n");
	/* TODO: ASSERT(0);    // not find the value */
	return 0;
}

static int __sgm41512_read_reg(struct chip_sgm41512 *chip, int reg, int *returnData)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_CHARGER_SGM41512)
	int ret = 0;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*returnData = ret;
	}
#else
	char cmd_buf[1] = {0x00};
	char readData = 0;
	int ret = 0;

	chip->client->ext_flag = ((chip->client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
	chip->client->timing = 300;
	cmd_buf[0] = reg;
	ret = i2c_master_send(chip->client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		chip->client->ext_flag = 0;
		return ret;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	chip->client->ext_flag = 0;
#endif
#else
	int ret = 0;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*returnData = ret;
	}
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	return 0;
}

static int sgm41512_read_reg(struct chip_sgm41512 *chip, int reg, int *returnData)
{
	int ret = 0;

	mutex_lock(&sgm41512_i2c_access);
	ret = __sgm41512_read_reg(chip, reg, returnData);
	mutex_unlock(&sgm41512_i2c_access);
	return ret;
}

static int __sgm41512_write_reg(struct chip_sgm41512 *chip, int reg, int val)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_CHARGER_SGM41512)
	int ret = 0;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write %02x to %02x: %d\n", val, reg, ret);
		return ret;
	}
#else
	char write_data[2] = {0};
	int ret = 0;

	write_data[0] = reg;
	write_data[1] = val;

	chip->client->ext_flag = ((chip->client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;
	chip->client->timing = 300;
	ret = i2c_master_send(chip->client, write_data, 2);
	if (ret < 0) {
		chip->client->ext_flag = 0;
		return ret;
	}

	chip->client->ext_flag = 0;
#endif
#else /* CONFIG_OPLUS_CHARGER_MTK */
	int ret = 0;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write %02x to %02x: %d\n", val, reg, ret);
		return ret;
	}
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	return 0;
}

static int sgm41512_config_interface (struct chip_sgm41512 *chip, int RegNum, int val, int MASK)
{
	int sgm41512_reg = 0;
	int ret = 0;

	mutex_lock(&sgm41512_i2c_access);
	ret = __sgm41512_read_reg(chip, RegNum, &sgm41512_reg);

	//pr_err("Reg[%x]=0x%x\n", RegNum, sgm41512_reg);

	sgm41512_reg &= ~MASK;
	sgm41512_reg |= val;

	ret = __sgm41512_write_reg(chip, RegNum, sgm41512_reg);

	//pr_err("write Reg[%x]=0x%x\n", RegNum, sgm41512_reg);

	__sgm41512_read_reg(chip, RegNum, &sgm41512_reg);

	//pr_err("Check Reg[%x]=0x%x\n", RegNum, sgm41512_reg);
	mutex_unlock(&sgm41512_i2c_access);

	return ret;
}

static int sgm41512_usbin_input_current_limit[] = {
	100,    300,    500,    900,
	1200,   1500,   2000,   3000,
};

#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_CHARGER_SGM41512)
#define AICL_COUNT	10
static  int __maybe_unused chg_vol_mini(int *chg_vol)
{
	int chg_vol_temp = 0;
	int i = 0;

	chg_vol_temp = chg_vol[0];
	for (i = 0; i < AICL_COUNT; i++) {
		if (chg_vol_temp > chg_vol[i]) {
			chg_vol_temp = chg_vol[i];
		}
	}
	return chg_vol_temp;
}
#endif

#define SGM41512_INLIM_OFFSET 100
#define SGM41512_INLIM_STEP   100
#define SGM41512_INLIM_SHIFT  0
static int sgm41512_input_current_limit_write(int value)
{
	int rc = 0;
	int i = 0;
	int j = 0;
    u8 reg_val;
	int chg_vol = 0;
	int aicl_point_temp = 0;
	int chg_vol_all[10] = {0};
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}
    if(value <= 0) {
	    rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, 0, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
        pr_err("%s, aicr <= 0, disable sub ic\n", __func__);
    } else {
        reg_val = (value - SGM41512_INLIM_OFFSET) / SGM41512_INLIM_STEP << SGM41512_INLIM_SHIFT;
        pr_err("%s, aicr = %d,reg = %d\n", __func__, value, reg_val);
        rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, reg_val, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
    }

    pr_err("usb input max current limit=%d setting %02x\n", value, i);
	aicl_point_temp = chip->sw_aicl_point;

	if (value < 300) {
		j = 0;
		goto aicl_end;
	} else if (value < 500) {
		j = 1;
		goto aicl_end;
	}

	j = 2; /* 500 */
	rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_500MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		j = 2;
		goto aicl_pre_step;
	} else if (value < 900)
		goto aicl_end;

	j = 3; /* 900 */
	rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_900MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value < 1200)
		goto aicl_end;

	j = 4; /* 1200 */
	rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_1200MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	}

	j = 5; /* 1500 */
	rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_1500MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 2; //We DO NOT use 1.2A here
		goto aicl_pre_step;
	} else if (value < 1500) {
		j = j - 1; //We use 1.2A here
		goto aicl_end;
	} else if (value < 2000)
		goto aicl_end;

	j = 6; /* 2000 */
	rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_2000MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_CHARGER_SGM41512)
	for (i = 0; i < AICL_COUNT; i++) {
		chg_vol = mt6357_get_vbus_voltage();
		chg_vol_all[i] = chg_vol;
		usleep_range(5000, 5000);
	}
	chg_vol = chg_vol_mini(chg_vol_all);
#else
	chg_vol = mt6357_get_vbus_voltage();
#endif
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value < 3000)
		goto aicl_end;

	j = 7; /* 3000 */
	rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_3000MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value >= 3000)
		goto aicl_end;

aicl_pre_step:
	pr_err("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, j, sgm41512_usbin_input_current_limit[j], aicl_point_temp);
	goto aicl_rerun;
aicl_end:
	pr_err("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, j, sgm41512_usbin_input_current_limit[j], aicl_point_temp);
	goto aicl_rerun;
aicl_rerun:
	if ((j >= 2) && (j <= ARRAY_SIZE(sgm41512_usbin_input_current_limit) - 1))
		aicl_result = sgm41512_usbin_input_current_limit[j];
	switch (j) {
		case 0:
			sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_100MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 1:
			sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_300MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 2:
			sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_500MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 3:
			sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_900MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 4:
			sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_1200MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 5:
			sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_1500MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 6:
			sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_2000MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 7:
			sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, REG00_SGM41512_INPUT_CURRENT_LIMIT_3000MA, REG00_SGM41512_INPUT_CURRENT_LIMIT_MASK);
			break;
		default:
			break;
	}

	return rc;
}

static int sgm41512_charging_current_write_fast(int chg_cur)
{
	int rc = 0;
	int value = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	pr_err("chg_cur = %d\r\n", chg_cur);

	if (chg_cur > REG02_SGM41512_MAX_FAST_CHG_CURRENT_MA
			|| chg_cur < REG02_SGM41512_MIN_FAST_CHG_CURRENT_MA)
		chg_cur = REG02_SGM41512_MAX_FAST_CHG_CURRENT_MA;
	value = (chg_cur - REG02_SGM41512_MIN_FAST_CHG_CURRENT_MA) / REG02_SGM41512_FAST_CHG_CURRENT_STEP_MA;
	value <<= REG02_SGM41512_FAST_CHG_CURRENT_LIMIT_SHIFT;

	rc = sgm41512_config_interface(chip, REG02_SGM41512_ADDRESS, value,
			REG02_SGM41512_FAST_CHG_CURRENT_LIMIT_MASK);
	if (rc < 0) {
		pr_err("Couldn't write fast charge current rc = %d\n", rc);
	}

	return rc;
}

static int sgm41512_set_vindpm_vol(int vol)
{
	int rc = 0;
	int value = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	if (vol < REG06_SGM41512_VINDPM_OFFSET)
		vol = REG06_SGM41512_VINDPM_OFFSET;
	value = (vol - REG06_SGM41512_VINDPM_OFFSET) / REG06_SGM41512_VINDPM_STEP_MV;
	value <<= REG06_SGM41512_VINDPM_SHIFT;

	rc = sgm41512_config_interface(chip, REG06_SGM41512_ADDRESS,
			value, REG06_SGM41512_VINDPM_MASK);
	return rc;
}

//#ifdef CONFIG_OPPO_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int sgm41512_chg_get_dyna_aicl_result(void)
{
	return aicl_result;
}
//#endif /* CONFIG_OPPO_SHORT_C_BATT_CHECK */

static void sgm41512_set_aicl_point(int vbatt)
{
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return;
	}

	if (chip->hw_aicl_point == 4400 && vbatt > 4100) {
		chip->hw_aicl_point = 4500;
		chip->sw_aicl_point = 4550;
		sgm41512_set_vindpm_vol(chip->hw_aicl_point);
	} else if (chip->hw_aicl_point == 4500 && vbatt <= 4100) {
		chip->hw_aicl_point = 4400;
		chip->sw_aicl_point = 4500;
		sgm41512_set_vindpm_vol(chip->hw_aicl_point);
	}
}

static int sgm41512_set_enable_volatile_writes(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	return rc;
}

static int sgm41512_set_complete_charge_timeout(int val)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	if (val == OVERTIME_AC) {
		val = REG05_SGM41512_CHG_SAFETY_TIMER_ENABLE | REG05_SGM41512_FAST_CHG_TIMEOUT_10H;
	} else if (val == OVERTIME_USB) {
		val = REG05_SGM41512_CHG_SAFETY_TIMER_ENABLE | REG05_SGM41512_FAST_CHG_TIMEOUT_10H;
	} else {
		val = REG05_SGM41512_CHG_SAFETY_TIMER_DISABLE | REG05_SGM41512_FAST_CHG_TIMEOUT_10H;
	}

	rc = sgm41512_config_interface(chip, REG05_SGM41512_ADDRESS, val,
			REG05_SGM41512_CHG_SAFETY_TIMER_MASK | REG05_SGM41512_FAST_CHG_TIMEOUT_MASK);
	if (rc < 0) {
		pr_err("Couldn't complete charge timeout rc = %d\n", rc);
	}

	return rc;
}

static int sgm41512_float_voltage_write(int vfloat_mv)
{
	int rc = 0;
	int value = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	if (vfloat_mv < REG04_SGM41512_MIN_FLOAT_MV) {
		pr_err("bad vfloat_mv:%d,return\n", vfloat_mv);
		return 0;
	}
	value = (vfloat_mv - REG04_SGM41512_MIN_FLOAT_MV) / REG04_SGM41512_VFLOAT_STEP_MV;
	value <<= REG04_SGM41512_CHG_VOL_LIMIT_SHIFT;
	pr_err("sgm41512_set_float_voltage vfloat_mv = %d value=%d\n", vfloat_mv, value);

	rc = sgm41512_config_interface(chip, REG04_SGM41512_ADDRESS, value, REG04_SGM41512_CHG_VOL_LIMIT_MASK);
	if (rc < 0) {
		pr_err("Couldn't set float voltage rc = %d\n", rc);
	}

	return rc;
}

static int sgm41512_set_prechg_current(int ipre_ma)
{
	int value = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	if (ipre_ma < REG03_SGM41512_MIN_PRE_CHG_CURRENT_MA)
		ipre_ma = REG03_SGM41512_MIN_PRE_CHG_CURRENT_MA;
	if (ipre_ma > REG03_SGM41512_MAX_PRE_CHG_CURRENT_MA)
		ipre_ma = REG03_SGM41512_MAX_PRE_CHG_CURRENT_MA;
	value = (ipre_ma - REG03_SGM41512_MIN_PRE_CHG_CURRENT_MA) / REG03_SGM41512_PRE_CHG_CURRENT_STEP_MA;
	value <<= REG03_SGM41512_PRE_CHG_CURRENT_LIMIT_SHIFT;

	return sgm41512_config_interface(chip, REG03_SGM41512_ADDRESS, value,
			REG03_SGM41512_PRE_CHG_CURRENT_LIMIT_MASK);
}

static int sgm41512_set_termchg_current(int term_curr)
{
	int value = 0;
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	if (term_curr < REG03_SGM41512_MIN_TERM_CHG_CURRENT_MA) {
		term_curr = REG03_SGM41512_MIN_TERM_CHG_CURRENT_MA;
	}
	value = (term_curr - REG03_SGM41512_MIN_TERM_CHG_CURRENT_MA) / REG03_SGM41512_TERM_CHG_CURRENT_STEP_MA;
	value <<= REG03_SGM41512_TERM_CHG_CURRENT_LIMIT_SHIFT;
	pr_err("value=%d\n", value);

	rc = sgm41512_config_interface(chip, REG03_SGM41512_ADDRESS, value,
			REG03_SGM41512_TERM_CHG_CURRENT_LIMIT_MASK);
	if (rc < 0) {
		pr_err("Couldn't set termination curren rc = %d\n", rc);
	}

	return rc;
}

static int sgm41512_set_rechg_voltage(int recharge_mv)
{
	int reg = 0;
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	/* set recharge voltage */
	if (recharge_mv >= 200) {
		reg = REG04_SGM41512_RECHG_THRESHOLD_VOL_200MV;
	} else {
		reg = REG04_SGM41512_RECHG_THRESHOLD_VOL_100MV;
	}
	rc = sgm41512_config_interface(chip, REG04_SGM41512_ADDRESS, reg, REG04_SGM41512_RECHG_THRESHOLD_VOL_MASK);
	if (rc) {
		pr_err("Couldn't set rechg threshold rc = %d\n", rc);
	}

	return rc;
}

static int sgm41512_set_wdt_timer(int reg)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	rc = sgm41512_config_interface(chip, REG05_SGM41512_ADDRESS, reg,
			REG05_SGM41512_WATCHDOG_TIMER_MASK);

	return rc;
}

static int sgm41512_set_chging_term_disable(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	rc = sgm41512_config_interface(chip, REG05_SGM41512_ADDRESS,
			REG05_SGM41512_TERMINATION_DISABLE, REG05_SGM41512_TERMINATION_MASK);

	return rc;
}

static int sgm41512_kick_wdt(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	rc = sgm41512_config_interface(chip, REG01_SGM41512_ADDRESS,
			REG01_SGM41512_WDT_TIMER_RESET, REG01_SGM41512_WDT_TIMER_RESET_MASK);
	if (rc < 0) {
		pr_err("Couldn't sgm41512_kick_wdt rc = %d\n", rc);
	}

	return rc;
}

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static bool sgm41512_check_charger_suspend_enable(void);
static int sgm41512_enable_charging(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
		|| get_boot_mode() == ADVMETA_BOOT
		|| get_boot_mode() == ATE_FACTORY_BOOT) {
		return 0;
	}
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	if (sgm41512_check_charger_suspend_enable()) {
		sgm41512_unsuspend_charger();
	}

	rc = sgm41512_config_interface(chip, REG01_SGM41512_ADDRESS,
			REG01_SGM41512_CHARGING_ENABLE, REG01_SGM41512_CHARGING_MASK);
	if (rc < 0) {
		pr_err("Couldn't sgm41512_enable_charging rc = %d\n", rc);
	}
	return rc;
}

static int sgm41512_disable_charging(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	/* Only for BATTERY_STATUS__HIGH_TEMP or BATTERY_STATUS__WARM_TEMP, */
	/* system power is from charger but NOT from battery to avoid temperature rise problem */
	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	rc = sgm41512_config_interface(chip, REG01_SGM41512_ADDRESS,
			REG01_SGM41512_CHARGING_DISABLE, REG01_SGM41512_CHARGING_MASK);
	if (rc < 0) {
		pr_err("Couldn't sgm41512_disable_charging  rc = %d\n", rc);
	} else {
		pr_err("battery HIGH_TEMP or WARM_TEMP, sgm41512_disable_charging\n");
	}
	return rc;
}
#else /* CONFIG_MTK_PMIC_CHIP_MT6353 */
static int sgm41512_enable_charging(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;
	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	rc = sgm41512_config_interface(chip, REG01_SGM41512_ADDRESS,
			REG01_SGM41512_CHARGING_ENABLE, REG01_SGM41512_CHARGING_MASK);
	if (rc < 0) {
		pr_err("Couldn'tsgm41512_enable_charging rc = %d\n", rc);
	}

	return rc;
}

static int sgm41512_disable_charging(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

    	sgm41512_suspend_charger();

	rc = sgm41512_config_interface(chip, REG01_SGM41512_ADDRESS,
			REG01_SGM41512_CHARGING_DISABLE, REG01_SGM41512_CHARGING_MASK);
	if (rc < 0) {
		pr_err("Couldn't sgm41512_disable_charging  rc = %d\n", rc);
	}

	return rc;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static bool sgm41512_check_charger_suspend_enable(void)
{
	int rc = 0;
	int reg_val = 0;
	bool charger_suspend_enable = false;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	rc = sgm41512_read_reg(chip, REG00_SGM41512_ADDRESS, &reg_val);
	if (rc) {
		pr_err("Couldn't read REG00_SGM41512_ADDRESS rc = %d\n", rc);
		return 0;
	}

	charger_suspend_enable = ((reg_val & REG00_SGM41512_SUSPEND_MODE_MASK) == REG00_SGM41512_SUSPEND_MODE_ENABLE) ? true : false;

	return charger_suspend_enable;
}

static int sgm41512_check_charging_enable(void)
{
	bool rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	rc = sgm41512_check_charger_suspend_enable();
	if (rc)
		return 0;
	else
		return 1;
}
#else /* CONFIG_MTK_PMIC_CHIP_MT6353 */
static int sgm41512_check_charging_enable(void)
{
	int rc = 0;
	int reg_val = 0;
	bool charging_enable = false;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	rc = sgm41512_read_reg(chip, REG01_SGM41512_ADDRESS, &reg_val);
	if (rc) {
		pr_err("Couldn't read REG01_SGM41512_ADDRESS rc = %d\n", rc);
		return 0;
	}

	charging_enable = ((reg_val & REG01_SGM41512_CHARGING_MASK) == REG01_SGM41512_CHARGING_ENABLE) ? 1 : 0;

	return charging_enable;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

static int sgm41512_registers_read_full(void)
{
	int rc = 0;
	int reg_full = 0;
	int reg_ovp = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	rc = sgm41512_read_reg(chip, REG08_SGM41512_ADDRESS, &reg_full);
	if (rc) {
		pr_err("Couldn't read REG08_SGM41512_ADDRESS rc = %d\n", rc);
		return 0;
	}
	reg_full = ((reg_full & REG08_SGM41512_CHG_STAT_MASK) == REG08_SGM41512_CHG_STAT_CHG_TERMINATION) ? 1 : 0;

	rc = sgm41512_read_reg(chip, REG09_SGM41512_ADDRESS, &reg_ovp);
	if (rc) {
		pr_err("Couldn't read REG09_SGM41512_ADDRESS rc = %d\n", rc);
		return 0;
	}
	reg_ovp = ((reg_ovp & REG09_SGM41512_BAT_FAULT_MASK) == REG09_SGM41512_BAT_FAULT_BATOVP) ? 1 : 0;

	//pr_err("sgm41512_registers_read_full, reg_full = %d, reg_ovp = %d\r\n", reg_full, reg_ovp);
	return (reg_full || reg_ovp);
}

static int oplus_otg_online = 0;

int sgm41512_otg_enable(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (!chip) {
		return 0;
	}
#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}
#endif /* CONFIG_OPLUS_CHARGER_MTK */
	oplus_otg_online = 1;
	sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS,
			REG00_SGM41512_SUSPEND_MODE_DISABLE, REG00_SGM41512_SUSPEND_MODE_MASK);

	rc = sgm41512_config_interface(chip, REG01_SGM41512_ADDRESS,
			REG01_SGM41512_OTG_ENABLE, REG01_SGM41512_OTG_MASK);
	if (rc) {
		pr_err("Couldn't enable OTG mode rc=%d\n", rc);
	} else {
		pr_err("sgm41512_otg_enable rc=%d\n", rc);
	}
	
	sgm41512_set_wdt_timer(REG05_SGM41512_WATCHDOG_TIMER_DISABLE);
	oplus_otg_online = 1;
	oplus_chg_set_otg_online(true);
	return rc;
}

int sgm41512_otg_disable(void)
{
	int rc = 0;
	int val_buf;
	struct chip_sgm41512 *chip = charger_ic;

	if (!chip) {
		return 0;
	}
#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	rc = sgm41512_config_interface(chip, REG01_SGM41512_ADDRESS,
			REG01_SGM41512_OTG_DISABLE, REG01_SGM41512_OTG_MASK);
	if (rc)
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);
	else
		pr_err("sgm41512_otg_disable rc=%d\n", rc);
	
	sgm41512_set_wdt_timer(REG05_SGM41512_WATCHDOG_TIMER_40S);
	oplus_otg_online = 0;
	oplus_chg_set_otg_online(false);

	return rc;
}

int oplus_sgm41512_get_charger_type(void)
{
	return charger_type_sgm;
}

void oplus_sgm41512_set_chargerid_switch_val(int value)
{
	return;
}

int oplus_sgm41512_get_chargerid_switch_val(void)
{
	return 0;
}

bool oplus_sgm41512_need_to_check_ibatt(void)
{
	return false;
}

int oplus_sgm41512_chg_get_charger_subtype(void)
{
	return CHARGER_SUBTYPE_DEFAULT;
}

int oplus_sgm41512_set_hz_mode(bool en)
{
	return sgm41512_suspend_charger();
}

static int sgm41512_suspend_charger(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;
	union power_supply_propval pval = {0, };
	struct power_supply *bat_psy = NULL;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	bat_psy = power_supply_get_by_name("battery");
	if(bat_psy == NULL)
		return false;

	power_supply_get_property(bat_psy, POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE, &pval);
	if ((pval.intval == 1) && (get_boot_mode() != META_BOOT)) {
		return rc;
	}

	pr_err("suspend sgm41512\n");
#if 0
	if (oplus_chg_get_otg_online() == true)
		return 0;

	if (oplus_otg_online == 1)
		return 0;
#endif
	rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS,
			REG00_SGM41512_SUSPEND_MODE_ENABLE, REG00_SGM41512_SUSPEND_MODE_MASK);
	if (rc < 0) {
		pr_err("Couldn't sgm41512_suspend_charger rc = %d\n", rc);
	} else {
		pr_err("rc = %d\n", rc);
	}

	return rc;
}

static int sgm41512_unsuspend_charger(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}
	pr_err("unsuspend sgm41512\n");

	rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS,
			REG00_SGM41512_SUSPEND_MODE_DISABLE, REG00_SGM41512_SUSPEND_MODE_MASK);
	if (rc < 0) {
		pr_err("Couldn't sgm41512_unsuspend_charger rc = %d\n", rc);
	} else {
		pr_err("rc = %d\n", rc);
	}

	return rc;
}

#if defined (CONFIG_MTK_PMIC_CHIP_MT6353) || defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_CHARGER_SGM41512)
static int sgm41512_reset_charger(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	// rc = sgm41512_config_interface(chip, REG00_SGM41512_ADDRESS, 0x0, 0xFF);
	// if (rc < 0) {
	// 	pr_err("Couldn't reset REG00 rc = %d\n", rc);
	// }
	rc = sgm41512_config_interface(chip, REG01_SGM41512_ADDRESS, 0x1A, 0xFF);
	if (rc < 0) {
		pr_err("Couldn't reset REG01 rc = %d\n", rc);
	}
	rc = sgm41512_config_interface(chip, REG02_SGM41512_ADDRESS, 0xA2, 0xFF);
	if (rc < 0) {
		pr_err("Couldn't reset REG02 rc = %d\n", rc);
	}
	rc = sgm41512_config_interface(chip, REG03_SGM41512_ADDRESS, 0x22, 0xFF);
	if (rc < 0) {
		pr_err("Couldn't reset REG03 rc = %d\n", rc);
	}
	rc = sgm41512_config_interface(chip, REG04_SGM41512_ADDRESS, 0x88, 0xFF);
	if (rc < 0) {
		pr_err("Couldn't reset REG04 rc = %d\n", rc);
	}
	rc = sgm41512_config_interface(chip, REG05_SGM41512_ADDRESS, 0x1F, 0xFF);
	if (rc < 0) {
		pr_err("Couldn't reset REG05 rc = %d\n", rc);
	}
	rc = sgm41512_config_interface(chip, REG06_SGM41512_ADDRESS, 0x76, 0xFF);
	if (rc < 0) {
		pr_err("Couldn't reset REG06 rc = %d\n", rc);
	}
	rc = sgm41512_config_interface(chip, REG07_SGM41512_ADDRESS, 0x4C, 0xFF);
	if (rc < 0) {
		pr_err("Couldn't reset REG07 rc = %d\n", rc);
	}
	rc = sgm41512_config_interface(chip, REG0A_SGM41512_ADDRESS, 0x0, 0x03);
	if (rc < 0) {
		pr_err("Couldn't reset REG0A rc = %d\n", rc);
	}
	pr_err("rc = %d\n", rc);

	return rc;
}
#else /*defined (CONFIG_MTK_PMIC_CHIP_MT6353) || defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_CHARGER_SGM41512)*/
static int sgm41512_reset_charger(void)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	rc = sgm41512_config_interface(chip, REG0B_SGM41512_ADDRESS,
			REG0B_SGM41512_REG_RST_RESET, REG0B_SGM41512_REG_RST_MASK);
	if (rc < 0) {
		pr_err("Couldn't sgm41512_reset_charger rc = %d\n", rc);
	} else {
		pr_err("rc = %d\n", rc);
	}

	return rc;
}
#endif /*defined (CONFIG_MTK_PMIC_CHIP_MT6353) || defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_CHARGER_SGM41512)*/

static bool sgm41512_check_charger_resume(void)
{
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		return false;
	} else {
		return true;
	}
}

#define DUMP_REG_LOG_CNT_30S  1
static void sgm41512_dump_registers(void)
{
	int rc = 0;
	int addr = 0;
	unsigned int val_buf[SGM41512_REG_NUMBER] = {0x0};
	static int dump_count = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return;
	}

	for (addr = SGM41512_FIRST_REG; addr <= SGM41512_LAST_REG; addr++) {
		rc = sgm41512_read_reg(chip, addr, &val_buf[addr]);
		if (rc) {
			 pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
			 return;
		}
	}

	if (dump_count == DUMP_REG_LOG_CNT_30S) {
		dump_count = 0;
		pr_err("sgm41512_reg: [0x%02x,0x%02x,0x%02x,0x%02x], [0x%02x,0x%02x,0x%02x,0x%02x], [0x%02x,0x%02x,0x%02x,0x%02x]\n",
			val_buf[0], val_buf[1], val_buf[2], val_buf[3], val_buf[4], val_buf[5], 
			val_buf[6], val_buf[7], val_buf[8], val_buf[9], val_buf[10], val_buf[11]);
	}
	dump_count++;

	return;
}

static int sgm41512_check_registers(void)
{
	int rc = 0;
	int addr = 0;
	unsigned int val_buf[SGM41512_REG_NUMBER] = {0x0};
	struct chip_sgm41512 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		pr_err("charger in suspended.\n");
		return 0;
	}

	for (addr = SGM41512_FIRST_REG; addr <= SGM41512_LAST_REG; addr++) {
		rc = sgm41512_read_reg(chip, addr, &val_buf[addr]);
		if (rc) {
			 pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
			 return -1;
		}
	}

  	return 0;
}

static int sgm41512_get_chg_current_step(void)
{
	return 60;
}

static int sgm41512_hardware_init(void)
{
	struct chip_sgm41512 *chip = charger_ic;
	
	if (!chip) {
		pr_err("%s chip null,return\n", __func__);
		return -1;
	}

	/* must be before set_vindpm_vol and set_input_current */
	chip->hw_aicl_point = 4400;
	chip->sw_aicl_point = 4500;

	sgm41512_reset_charger();
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
#ifndef CONFIG_MTK_PMIC_CHIP_MT6353
		sgm41512_disable_charging();
#endif /* CONFIG_MTK_PMIC_CHIP_MT6353 */
		sgm41512_float_voltage_write(4400);
		msleep(100);
	}

	sgm41512_float_voltage_write(4370);

	sgm41512_set_enable_volatile_writes();

	sgm41512_set_complete_charge_timeout(OVERTIME_DISABLED);

	sgm41512_set_prechg_current(300);

	sgm41512_charging_current_write_fast(512);

	sgm41512_set_termchg_current(150);

	sgm41512_set_rechg_voltage(100);

	sgm41512_set_vindpm_vol(chip->hw_aicl_point);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
			|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT) {
		sgm41512_suspend_charger();
		sgm41512_disable_charging();
	} else {
		sgm41512_unsuspend_charger();
	}
#else /* CONFIG_OPLUS_CHARGER_MTK */
	sgm41512_unsuspend_charger();
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	sgm41512_enable_charging();

	sgm41512_set_wdt_timer(REG05_SGM41512_WATCHDOG_TIMER_40S);

	return true;
}

#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
static int rtc_reset_check(void)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc = 0;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return 0;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	if ((tm.tm_year == 70) && (tm.tm_mon == 0) && (tm.tm_mday <= 1)) {
		pr_err(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  @@@ wday: %d, yday: %d, isdst: %d\n",
			tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
			tm.tm_wday, tm.tm_yday, tm.tm_isdst);
		rtc_class_close(rtc);
		return 1;
	}

	pr_err(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  ###  wday: %d, yday: %d, isdst: %d\n",
		tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
		tm.tm_wday, tm.tm_yday, tm.tm_isdst);

close_time:
	rtc_class_close(rtc);
	return 0;
}
#endif /* CONFIG_OPPO_RTC_DET_SUPPORT */

extern bool oplus_chg_get_shortc_hw_gpio_status(void);

extern void kernel_power_off(void);

static void mt_power_off(void)
{
	kernel_power_off();
}

static int sgm41512_enable_shipmode(bool en)
{
	int rc = 0;
	struct chip_sgm41512 *chip = charger_ic;
	if(en){
		rc = sgm41512_config_interface(chip, REG07_SGM41512_ADDRESS, 0x20, 0x20);
		if (rc < 0) {
			chg_err("set shipmode REG07 enable error rc = %d\n", rc);
		}
	}else{
		rc = sgm41512_config_interface(chip, REG07_SGM41512_ADDRESS, 0x00, 0x20);
		if (rc < 0) {
			chg_err("set shipmode REG07 disable error rc = %d\n", rc);
		}
	}

	return 0;
}

struct oplus_chg_operations  oplus_chg_sgm41512_ops = {
	.dump_registers = sgm41512_dump_registers,
	.kick_wdt = sgm41512_kick_wdt,
	.hardware_init = sgm41512_hardware_init,
	.charging_current_write_fast = sgm41512_charging_current_write_fast,
	.set_aicl_point = sgm41512_set_aicl_point,
	.input_current_write = sgm41512_input_current_limit_write,
	.float_voltage_write = sgm41512_float_voltage_write,
	.term_current_set = sgm41512_set_termchg_current,
	.charging_enable = sgm41512_enable_charging,
	.charging_disable = sgm41512_disable_charging,
	.get_charging_enable = sgm41512_check_charging_enable,
	.charger_suspend = sgm41512_suspend_charger,
	.charger_unsuspend = sgm41512_unsuspend_charger,
	.set_rechg_vol = sgm41512_set_rechg_voltage,
	.reset_charger = sgm41512_reset_charger,
	.read_full = sgm41512_registers_read_full,
	.otg_enable = sgm41512_otg_enable,
	.otg_disable = sgm41512_otg_disable,
	.set_charging_term_disable = sgm41512_set_chging_term_disable,
	.check_charger_resume = sgm41512_check_charger_resume,

	.get_charger_type = oplus_sgm41512_get_charger_type, 
	.get_charger_volt = mt6357_get_vbus_voltage,
//	int (*get_charger_current)(void);
	.get_chargerid_volt = NULL,
    .set_chargerid_switch_val = oplus_sgm41512_set_chargerid_switch_val,
    .get_chargerid_switch_val = oplus_sgm41512_get_chargerid_switch_val,
	.check_chrdet_status = (bool (*) (void)) pmic_chrdet_status,

	.get_boot_mode = (int (*)(void))get_boot_mode,
	.get_boot_reason = (int (*)(void))get_boot_reason,
	.get_instant_vbatt = oplus_battery_meter_get_battery_voltage,
	.get_rtc_soc = oplus_get_rtc_ui_soc,
	.set_rtc_soc = oplus_set_rtc_ui_soc,
	.set_power_off = mt_power_off,
	.usb_connect = mt_usb_connect,
	.usb_disconnect = mt_usb_disconnect,
    .get_chg_current_step = sgm41512_get_chg_current_step,
    .need_to_check_ibatt = oplus_sgm41512_need_to_check_ibatt,
    .get_dyna_aicl_result = sgm41512_chg_get_dyna_aicl_result,
    .get_shortc_hw_gpio_status = oplus_chg_get_shortc_hw_gpio_status,
//	void (*check_is_iindpm_mode) (void);
    .oplus_chg_get_pd_type = NULL,
    .oplus_chg_pd_setup = NULL,
	.get_charger_subtype = oplus_sgm41512_chg_get_charger_subtype,
	.set_qc_config = NULL,
	.enable_qc_detect = NULL,
//#ifdef ODM_WT_EDIT
	//.oplus_chg_get_pe20_type = NULL,
	//.oplus_chg_pe20_setup = NULL,
	//.oplus_chg_reset_pe20 = NULL,
	.oplus_chg_set_high_vbus = NULL,
//#endif
	.enable_shipmode = sgm41512_enable_shipmode,
	.oplus_chg_set_hz_mode = oplus_sgm41512_set_hz_mode,
};

void sgm41512_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;
	struct chip_sgm41512 *chip = charger_ic;
	if(val == 1) {
		val = 0x80;
        }
	ret = sgm41512_config_interface(chip, REG02_SGM41512_ADDRESS,
				       val,REG02_SGM41512_OTG_CURRENT_LIMIT_MASK);
}

static int sgm41512_enable_otg(struct charger_device *chg_dev, bool en)
{
	return en ? sgm41512_otg_enable() : sgm41512_otg_disable();
}

/* BQ25601 REG0A BOOST_LIM[2:0], mA */
const unsigned int BOOST_CURRENT_LIMIT[] = {
	500, 1200
};

static int sgm41512_set_boost_current_limit(struct charger_device
		*chg_dev, u32 uA)
{
	int ret = 0;
	u32 array_size = 0;
	u32 boost_ilimit = 0;
	u8 boost_reg = 0;

	uA /= 1000;
	array_size = ARRAY_SIZE(BOOST_CURRENT_LIMIT);
	boost_ilimit = bmt_find_closest_level(BOOST_CURRENT_LIMIT, array_size,
					      uA);
	boost_reg = charging_parameter_to_value(BOOST_CURRENT_LIMIT,
						array_size, boost_ilimit);
	sgm41512_set_boost_lim(boost_reg);

	return ret;
}

static struct charger_ops sgm41512_chg_ops = {

	/* OTG */
	.enable_otg = sgm41512_enable_otg,
	.set_boost_current_limit = sgm41512_set_boost_current_limit,
};

static void register_charger_devinfo(void)
{
	int ret = 0;
	char *version = "sgm41512";
	char *manufacture = "TI";
	ret = register_device_proc("charger", version, manufacture);
	if (ret)
		pr_err("register_charger_devinfo fail\n");
}

extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern bool is_usb_rdy(void);
static void hw_bc12_init(void)
{
	int timeout = 350;
	static bool first_connect = true;

	if (first_connect == true) {
		/* add make sure USB Ready */
		if (is_usb_rdy() == false) {
			pr_err("CDP, block\n");
			while (is_usb_rdy() == false && timeout > 0) {
				msleep(100);
				timeout--;
			}
			if (timeout == 0) {
				pr_err("CDP, timeout\n");
			} else {
				pr_err("CDP, free, timeout:%d\n", timeout);
			}
		} else {
			pr_err("CDP, PASS\n");
		}
		first_connect = false;
	}
	Charger_Detect_Init();
}

//static DEVICE_ATTR(sgm41512_access, 0664, show_sgm41512_access, store_sgm41512_access);
#ifdef VENDOR_EDIT
enum charger_type MTK_CHR_Type_num;
//extern unsigned int upmu_get_rgs_chrdet(void);

extern bool mt6357_get_vbus_status(void);

bool pmic_chrdet_status(void)
{
	//return charger_ic->pwr_gd;
	if(mt6357_get_vbus_status() && (oplus_otg_online != 1))
		return true;
	else
		return false;
}



unsigned int upmu_get_rgs_chrdet(void)
{
	int data_0x08 = 0;
	sgm41512_read_reg(charger_ic, 0x08, &data_0x08);
	return (!!(data_0x08 & 0x04));
}

enum charger_type mt_charger_type_detection_sgm41512(void)
{
	int rc = 0;
	int chgr_type_addr = 0;
	int val_buf = 0;
	int count = 0;
	int i = 0;
	if (!charger_ic) {
		pr_err("%s charger_ic null, return\n", __func__);
		return CHARGER_UNKNOWN;
	}

	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
			|| get_boot_mode() == ADVMETA_BOOT
			|| get_boot_mode() == ATE_FACTORY_BOOT) {
		return STANDARD_HOST;
	}

	chgr_type_addr = REG08_SGM41512_ADDRESS;

	hw_bc12_init();
	if (charger_ic->chip_id == 2) {
		usleep_range(10000, 10200);
		sgm41512_config_interface(charger_ic, REG07_SGM41512_ADDRESS,
				REG07_SGM41512_IINDET_EN_FORCE_DET, REG07_SGM41512_IINDET_EN_MASE);
		sgm41512_dump_registers();
	}
	rc = sgm41512_read_reg(charger_ic, REG07_SGM41512_ADDRESS, &val_buf);
	val_buf &= REG07_SGM41512_IINDET_EN_MASE;
	while (val_buf == REG07_SGM41512_IINDET_EN_FORCE_DET && count < 20) {
		count++;
		rc = sgm41512_read_reg(charger_ic, REG07_SGM41512_ADDRESS, &val_buf);
		val_buf &= REG07_SGM41512_IINDET_EN_MASE;
		usleep_range(50000, 50200);
		if (!pmic_chrdet_status()) {
			MTK_CHR_Type_num = CHARGER_UNKNOWN;
			return MTK_CHR_Type_num;
		}
	}

	if (count == 20) {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
		return MTK_CHR_Type_num;
	}

	rc = sgm41512_read_reg(charger_ic, chgr_type_addr, &val_buf);

	val_buf = val_buf & REG08_SGM41512_VBUS_STAT_MASK;
	pr_err("mt_charger_type_detection3, val_buf=[0x%x]\n", val_buf);

	if (val_buf == REG08_SGM41512_VBUS_STAT_UNKNOWN) {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
	} else if (val_buf == REG08_SGM41512_VBUS_STAT_SDP) {
		MTK_CHR_Type_num = STANDARD_HOST;
	} else if (val_buf == REG08_SGM41512_VBUS_STAT_CDP) {
		MTK_CHR_Type_num = CHARGING_HOST;
	} else if (val_buf == REG08_SGM41512_VBUS_STAT_DCP
			|| val_buf == REG08_SGM41512_VBUS_STAT_OCP
			|| val_buf == REG08_SGM41512_VBUS_STAT_FLOAT) {
		MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
	} else {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
	}

	/* the 2nd detection */
	if (MTK_CHR_Type_num == CHARGER_UNKNOWN && pmic_chrdet_status()) {
		pr_err("mt_charger_type_detection: 2nd...\n");
		sgm41512_config_interface(charger_ic, REG07_SGM41512_ADDRESS,
				REG07_SGM41512_IINDET_EN_FORCE_DET, REG07_SGM41512_IINDET_EN_MASE);
		sgm41512_dump_registers();

		rc = sgm41512_read_reg(charger_ic, REG07_SGM41512_ADDRESS, &val_buf);
		val_buf &= REG07_SGM41512_IINDET_EN_MASE;
		count = 0;
		while (val_buf == REG07_SGM41512_IINDET_EN_FORCE_DET && count < 20) {
			count++;
			rc = sgm41512_read_reg(charger_ic, REG07_SGM41512_ADDRESS, &val_buf);
			val_buf &= REG07_SGM41512_IINDET_EN_MASE;
			usleep_range(10000, 10200);
			if (!pmic_chrdet_status()) {
				MTK_CHR_Type_num = CHARGER_UNKNOWN;
				return MTK_CHR_Type_num;
			}
		}
		pr_err("mt_charger_type_detection: 2nd, count=%d\n", count);
		if (count == 20) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
			return MTK_CHR_Type_num;
		}

		rc = sgm41512_read_reg(charger_ic, chgr_type_addr, &val_buf);
		sgm41512_dump_registers();
		val_buf = val_buf & REG08_SGM41512_VBUS_STAT_MASK;
		pr_err("mt_charger_type_detection: 2nd, val_buf=[0x%x]\n", val_buf);
		if (val_buf == REG08_SGM41512_VBUS_STAT_UNKNOWN) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		} else if (val_buf == REG08_SGM41512_VBUS_STAT_SDP) {
			MTK_CHR_Type_num = STANDARD_HOST;
		} else if (val_buf == REG08_SGM41512_VBUS_STAT_CDP) {
			MTK_CHR_Type_num = CHARGING_HOST;
		} else if (val_buf == REG08_SGM41512_VBUS_STAT_DCP
				|| val_buf == REG08_SGM41512_VBUS_STAT_OCP
				|| val_buf == REG08_SGM41512_VBUS_STAT_FLOAT) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		} else {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		}
	}

	Charger_Detect_Release();

	return MTK_CHR_Type_num;
}
#if 0
enum charger_type mt_charger_type_detection_sgm41512(void)
{
	int rc =0;
	int addr = 0;
	int val_buf;
	int count = 0;
	int i;

	if (!charger_ic) {
		pr_err("%s charger_ic null, return\n", __func__);
		return CHARGER_UNKNOWN;
	}
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
			|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT) {
		return STANDARD_HOST;
	}
	addr = SGM41512_FIRST_REG + 8;
	pr_err("mt_charger_type_detection0\n");
	hw_bc12_init();
	usleep_range(40000, 40200);
	pr_err("mt_charger_type_detection1\n");
	sgm41512_config_interface(charger_ic, REG00_SGM41512_ADDRESS, 0, 0x80);
	usleep_range(5000, 5200);
	sgm41512_config_interface(charger_ic, REG07_SGM41512_ADDRESS, 0x80, 0);
	sgm41512_dump_registers();
	rc = sgm41512_read_reg(charger_ic, REG07_SGM41512_ADDRESS, &val_buf);
	while (val_buf == 0xcb && count < 20) {
		count++;
		rc = sgm41512_read_reg(charger_ic, REG07_SGM41512_ADDRESS, &val_buf);
		usleep_range(50000, 50200);
		if (!pmic_chrdet_status()) {
			MTK_CHR_Type_num = CHARGER_UNKNOWN;
			return MTK_CHR_Type_num;
		}
	}
	if (count == 20) {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
		pr_err("count time out,return apple adapter\n");
		return MTK_CHR_Type_num;
	}
	pr_err("mt_charger_type_detection2\n");
	rc = sgm41512_read_reg(charger_ic, addr, &val_buf);
	pr_err("mt_charger_type_detection\n");
	sgm41512_dump_registers();
	val_buf = val_buf & 0xc0;
	pr_err("val_buf = 0x%x\n",val_buf);
	if (val_buf == 0) {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
	}
	else if (val_buf == 0x40) {
		MTK_CHR_Type_num = STANDARD_HOST;
	}
	else if (val_buf == 0x80){
		MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
	}
	else {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
	}
	if ((MTK_CHR_Type_num == CHARGER_UNKNOWN || MTK_CHR_Type_num == STANDARD_HOST) && pmic_chrdet_status()) {
		if (MTK_CHR_Type_num == STANDARD_HOST) {
			for (i = 0; i < 4; i++) {
				usleep_range(100000, 100020);
			}
		}
		sgm41512_config_interface(charger_ic, REG00_SGM41512_ADDRESS, 0, 0x80);
		usleep_range(5000, 5020);
		sgm41512_config_interface(charger_ic, REG07_SGM41512_ADDRESS, 0x80, 0);
		sgm41512_dump_registers();
		rc = sgm41512_read_reg(charger_ic, REG07_SGM41512_ADDRESS, &val_buf);
		while (val_buf == 0xcb && count <20) {
			count++;
			rc = sgm41512_read_reg(charger_ic, REG07_SGM41512_ADDRESS, &val_buf);
			usleep_range(50000, 50200);
			if (!pmic_chrdet_status()) {
				MTK_CHR_Type_num = CHARGER_UNKNOWN;
				return MTK_CHR_Type_num;
			}
		}
		if (count == 20) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
			pr_err("count time out,return apple adapter\n");
			return MTK_CHR_Type_num;
		}
		rc = sgm41512_read_reg(charger_ic, addr, &val_buf);
		sgm41512_dump_registers();
		val_buf = val_buf & 0xc0;
		pr_err("val_buf =0x%x\n",val_buf);
		if (val_buf == 0) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		}
		else if(val_buf == 0x40) {
			MTK_CHR_Type_num = STANDARD_HOST;
		}
		else if(val_buf == 0x80){
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		}
		else {
			MTK_CHR_Type_num = CHARGER_UNKNOWN;
		}
	}
	pr_err("chr_type:%d\n",MTK_CHR_Type_num);
	Charger_Detect_Release();
	//MTK_CHR_Type_num = APPLE_2_1A_CHARGER;

	return MTK_CHR_Type_num;
}
#endif
#endif
static int sgm41512_enable_chg_type_det(bool en);
static int sgm41512_irq_registration(void);
static void do_charger_modefy_work(struct work_struct *data)
{
	static int cnt = 0;
	if (cnt ==0){
		sgm41512_irq_registration();
		cnt= 1;
	}
	if (charger_ic != NULL) {
		if(pmic_chrdet_status())
			sgm41512_enable_chg_type_det(true);
		else
			sgm41512_enable_chg_type_det(false);
	}
}

static int sgm41512_inform_psy_changed(bool vbus_gd)
{
	int ret = 0;
	union power_supply_propval propval;
	static struct power_supply *psy;
	struct chip_sgm41512 *chip = charger_ic;

	/* Get chg type det power supply */
	psy = power_supply_get_by_name("mtk-master-charger");
	if (!psy) {
		dev_notice(chip->dev, "%s get power supply fail\n", __func__);
		return -EINVAL;
	}
	/* inform chg det power supply */
	propval.intval = vbus_gd;
	ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_ONLINE,
					&propval);
	if (ret < 0)
		dev_notice(chip->dev, "%s psy online fail(%d)\n",
				      __func__, ret);

	power_supply_changed(psy);

	return ret;
}

static int sgm41512_enable_chg_type_det(bool en)
{
	struct chip_sgm41512 *chip = charger_ic;
	if (en) {
		chip->chg_type = mt_charger_type_detection_sgm41512();
		switch (chip->chg_type) {
		case CHARGER_UNKNOWN :
			charger_type_sgm = POWER_SUPPLY_TYPE_UNKNOWN;
			break;
		case STANDARD_HOST :
			charger_type_sgm = POWER_SUPPLY_TYPE_USB;
			break;
		case CHARGING_HOST :
			charger_type_sgm = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		case NONSTANDARD_CHARGER :	
		case STANDARD_CHARGER :
		case APPLE_2_1A_CHARGER : 
		case APPLE_1_0A_CHARGER :
		case APPLE_0_5A_CHARGER :
			charger_type_sgm = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		default:
			charger_type_sgm = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		}
	} else {
		chip->chg_type = CHARGER_UNKNOWN;
		charger_type_sgm = POWER_SUPPLY_TYPE_UNKNOWN;
	}

	pr_err("SGM41512 charger type:%d, power_good:%d\n", chip->chg_type, chip->pwr_gd);
	sgm41512_inform_psy_changed(en);
	return 0;
}

static struct delayed_work sgm41512_irq_delay_work;
static struct delayed_work sgm41512_bc_delay_work;
bool fg_sgm41512_irq_delay_work_running = false;

static void do_sgm41512_bc_delay_work(struct work_struct *data)
{
	struct chip_sgm41512 *chip = charger_ic;
	sgm41512_enable_chg_type_det(chip->pwr_gd);
}

static void do_sgm41512_irq_delay_work(struct work_struct *data)
{
	int val_buf = 0;
	int val_reg = 0;
	int i = 0;
	int otg_overcurrent_flag = 0;
	struct chip_sgm41512 *chip = charger_ic;
	
	if (!chip) {
		pr_err("%s chip null,return\n", __func__);
		return;
	}

	for (i = 0; i < 10; i++) {
		sgm41512_read_reg(chip, REG09_SGM41512_ADDRESS, &val_reg);
		val_buf = val_reg & 0x40;
		if (val_buf == 0x40) {
			otg_overcurrent_flag++;
		}
		
		usleep_range(10000, 10200);
	}
	pr_err("[OPPO_CHG] do_sgm41512_irq_delay_work disable vbus out flag=%d, val_reg[0x%x]\n", otg_overcurrent_flag, val_reg);
	if (otg_overcurrent_flag >= 8) {
		sgm41512_otg_disable();
	}
	fg_sgm41512_irq_delay_work_running = false;
	return; 
}

static irqreturn_t sgm41512_irq_handler_fn(int irq, void *dev_id)
{
	//int val_buf;
	struct chip_sgm41512 *chip = charger_ic;
	int power_good, val_reg;
	//pr_err("%s start\n", __func__);
	if (!chip) {
		pr_err("%s chip null,return\n", __func__);
		return IRQ_HANDLED;
	}

	power_good = chip->pwr_gd;
	//sgm41512_read_reg(chip, REG08_SGM41512_ADDRESS, &val_reg);
	//chip->pwr_gd = (val_reg &= REG08_SGM41512_POWER_GOOD_STAT_MASK) >> 2;
	chip->pwr_gd = pmic_chrdet_status();
	pr_err("%s, (%d:%d)\n", __func__, power_good, chip->pwr_gd);

	if(chip->pwr_gd != power_good) {
		schedule_delayed_work(&sgm41512_bc_delay_work, 0);
	}
	if (fg_sgm41512_irq_delay_work_running == false) {
		fg_sgm41512_irq_delay_work_running = true;
		schedule_delayed_work(&sgm41512_irq_delay_work, round_jiffies_relative(msecs_to_jiffies(50)));
	}

	return IRQ_HANDLED;
}

static int sgm41512_parse_dts(void)
{
	int ret = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (!chip) {
		pr_err("chip is NULL\n");
		return -1;
	}

	chip->irq_gpio = of_get_named_gpio(chip->client->dev.of_node, "chg-irq-gpio", 0);
	if (chip->irq_gpio <= 0) {
		pr_err("Couldn't read chg-irq-gpio:%d\n", chip->irq_gpio);
		return -1;
	} else {
		if (gpio_is_valid(chip->irq_gpio)) {
			ret = gpio_request(chip->irq_gpio, "chg-irq-gpio");
			if (ret) {
				pr_err("unable to request chg-irq-gpio[%d]\n", chip->irq_gpio);
				chip->irq_gpio = -EINVAL;
			} else {
				gpio_direction_input(chip->irq_gpio);
			}
		} else {
			pr_err("gpio_is_valid fail chg-irq-gpio[%d]\n", chip->irq_gpio);
			chip->irq_gpio = -EINVAL;
			return -1;
		}
	}
	pr_err("chg-irq-gpio[%d]\n", chip->irq_gpio);
	return ret;
}

static int sgm41512_irq_registration(void)
{
	int ret = 0;
	struct chip_sgm41512 *chip = charger_ic;

	if (chip->irq_gpio <= 0) {
		pr_err("chip->irq_gpio fail\n");
		return -1;
	}
	ret = request_threaded_irq(gpio_to_irq(chip->irq_gpio), NULL,
			sgm41512_irq_handler_fn,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"SGM41512-eint", chip);
	if (ret < 0) {
		pr_err("SGM41512 request_irq IRQ LINE NOT AVAILABLE!");
		return -EFAULT;
	}
	enable_irq_wake(gpio_to_irq(chip->irq_gpio));
	return 0;
}

enum sgm41512_part_no {
	BQ25601D = 0X02,
	SY6974B = 0x08,
};

#define REG0B_PN_MASK             	0x78
#define REG0B_PN_SHIFT            	3

void sgm41512_get_chip_id(void)
{
	int data, partno;
	sgm41512_read_reg(charger_ic, REG0B_SGM41512_ADDRESS, &data);
	partno = (data & REG0B_PN_MASK) >> REG0B_PN_SHIFT;
	pr_err("partno=%d\n", partno);
	if (partno == BQ25601D) {
		charger_ic->chip_id = 1;
	} else if (partno == SY6974B) {
		charger_ic->chip_id = 2;
	}
}

extern int charger_ic_flag;
static int sgm41512_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int reg = 0, ret;
	struct chip_sgm41512 *chip = NULL;

#ifndef CONFIG_OPLUS_CHARGER_MTK
	struct power_supply *usb_psy;
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}
#endif  /* CONFIG_OPLUS_CHARGER_MTK */
	pr_err("call \n");

	chip = devm_kzalloc(&client->dev, sizeof(struct chip_sgm41512), GFP_KERNEL);
	if (!chip) {
		pr_err("kzalloc() failed\n");
		return -ENOMEM;
	}
	charger_ic = chip;
	chip->client = client;
	chip->dev = &client->dev;
	chip->chg_props.alias_name = "sgm41512_chg";
	chip->chg_dev_name = "primary_chg";
	chip->pwr_gd = 0;
	
	reg = sgm41512_check_registers();
	if (reg < 0) {
		return -ENODEV;
	}

	/* Register charger device */
	chip->chg_dev = charger_device_register(chip->chg_dev_name,
						&client->dev, chip,
						&sgm41512_chg_ops,
						&chip->chg_props);
	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		pr_info("%s: register charger device  failed\n", __func__);
		ret = PTR_ERR(chip->chg_dev);
		return ret;
	}

	INIT_DELAYED_WORK(&sgm41512_irq_delay_work, do_sgm41512_irq_delay_work);
	INIT_DELAYED_WORK(&sgm41512_bc_delay_work, do_sgm41512_bc_delay_work);
	sgm41512_parse_dts();
	//sgm41512_irq_registration();
	atomic_set(&chip->charger_suspended, 0);
	sgm41512_reset_charger();

	register_charger_devinfo();
	sgm41512_get_chip_id();
	usleep_range(1000, 1200);
	INIT_DELAYED_WORK(&charger_modefy_work, do_charger_modefy_work);
	schedule_delayed_work(&charger_modefy_work, round_jiffies_relative(msecs_to_jiffies(2000)));
	pr_err("call OK!\n");
    //sgm41512_suspend_charger();
    	set_charger_ic(SGM41512);
	sgm41512_dump_registers();
    pr_info("SGM41512 charger driver probe successfully\n");

	return 0;
}

static struct i2c_driver sgm41512_i2c_driver;

static int sgm41512_driver_remove(struct i2c_client *client)
{
	return 0;
}

static unsigned long suspend_tm_sec = 0;
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc = NULL;
	int rc = 0;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int sgm41512_resume(struct device *dev)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;

	if (!charger_ic) {
		return 0;
	}
	atomic_set(&charger_ic->charger_suspended, 0);
	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		pr_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	if (sleep_time < 0) {
		sleep_time = 0;
	}
	pr_err("resume_sec:%ld,sleep_time:%ld\n\n",resume_tm_sec,sleep_time);
	oplus_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int sgm41512_suspend(struct device *dev)
{
	if (!charger_ic) {
		return 0;
	}
	atomic_set(&charger_ic->charger_suspended, 1);
	if (get_current_time(&suspend_tm_sec)) {
		pr_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	pr_err("suspend_sec:%ld\n",suspend_tm_sec);
	return 0;
}

static const struct dev_pm_ops sgm41512_pm_ops = {
	.resume		= sgm41512_resume,
	.suspend		= sgm41512_suspend,
};

#else //(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))

static int sgm41512_resume(struct i2c_client *client)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;

	if (!charger_ic) {
		return 0;
	}
	atomic_set(&charger_ic->charger_suspended, 0);
	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		pr_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	if (sleep_time < 0) {
		sleep_time = 0;
	}
	oplus_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int sgm41512_suspend(struct i2c_client *client, pm_message_t mesg)
{
	if (!charger_ic) {
		return 0;
	}
	atomic_set(&charger_ic->charger_suspended, 1);
	if (get_current_time(&suspend_tm_sec)) {
		pr_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	return 0;
}
#endif //(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))

static void sgm41512_reset(struct i2c_client *client)
{
	sgm41512_otg_disable();
}

/**********************************************************
 * *
 * *   [platform_driver API]
 * *
 * *********************************************************/

static const struct of_device_id sgm41512_match[] = {
	{ .compatible = "ti,sgm41512-charger"},
	{ },
};

static const struct i2c_device_id sgm41512_id[] = {
	{"sgm41512-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm41512_id);

static struct i2c_driver sgm41512_i2c_driver = {
	.driver		= {
		.name			= "sgm41512-charger",
		.owner			= THIS_MODULE,
		.of_match_table	= sgm41512_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm				= &sgm41512_pm_ops,
#endif
	},
	.probe		= sgm41512_driver_probe,
	.remove		= sgm41512_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= sgm41512_resume,
	.suspend		= sgm41512_suspend,
#endif

	.shutdown	= sgm41512_reset,
	.id_table		= sgm41512_id,
};

module_i2c_driver(sgm41512_i2c_driver);
MODULE_DESCRIPTION("Driver for sgm41512 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:sgm41512-charger");
