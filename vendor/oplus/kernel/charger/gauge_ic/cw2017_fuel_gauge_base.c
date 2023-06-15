/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/sizes.h>

#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include <linux/hardware_info.h>

#define CWFG_ENABLE_LOG 1
#define CWFG_I2C_BUSNUM 5
#define DOUBLE_SERIES_BATTERY 0


#define REG_VERSION			 0x00
#define REG_VCELL_H			 0x02
#define REG_VCELL_L			 0x03
#define REG_SOC_INT			 0x04
#define REG_SOC_DECIMAL		 0x05
#define REG_TEMP			 0x06
#define REG_MODE_CONFIG		 0x08
#define REG_GPIO_CONFIG		 0x0A
#define REG_SOC_ALERT		 0x0B
#define REG_TEMP_MAX		 0x0C
#define REG_TEMP_MIN		 0x0D
#define REG_VOLT_ID_H		 0x0E
#define REG_VOLT_ID_L		 0x0F
#define REG_BATINFO			 0x10

#define MODE_SLEEP			 0x30
#define MODE_NORMAL			 0x00
#define MODE_DEFAULT		 0xF0
#define CONFIG_UPDATE_FLG	 0x80
#define NO_START_VERSION	160

#define GPIO_CONFIG_MIN_TEMP			(0x00 << 4)
#define GPIO_CONFIG_MAX_TEMP			(0x00 << 5)
#define GPIO_CONFIG_SOC_CHANGE			(0x00 << 6)
#define GPIO_CONFIG_MIN_TEMP_MARK		(0x01 << 4)
#define GPIO_CONFIG_MAX_TEMP_MARK		(0x01 << 5)
#define GPIO_CONFIG_SOC_CHANGE_MARK		(0x01 << 6)
#define ATHD							0x0
#define DEFINED_MAX_TEMP				450
#define DEFINED_MIN_TEMP				0

#define DESIGN_CAPACITY					7100
#define CWFG_NAME "cw2017"
#define SIZE_BATINFO	80
#define BAT_COUNT	2

static int g_bat_id_vol[BAT_COUNT *2] = {
        498, 598,	/* 15k */
        939, 1039,	/*68K, atl*/
};

static char *battery_name[2] = {"BLT004-ALT-7100MA"};

#define QUEUE_DELAYED_WORK_TIME   8000

static unsigned char config_info[SIZE_BATINFO] = {
        0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xA3, 0xC3, 0xC9, 0xE1, 0xB9, 0x9E, 0xB4, 0x89,
        0x69, 0xFF, 0xFF, 0xCE, 0xAD, 0xA1, 0x82, 0x5E,
        0x50, 0x4B, 0x45, 0x88, 0xFF, 0xDC, 0x09, 0xD1,
        0xD1, 0xD1, 0xD2, 0xD1, 0xCF, 0xCD, 0xC7, 0xCB,
        0xBF, 0xC3, 0xC8, 0xA9, 0x94, 0x88, 0x7F, 0x73,
        0x63, 0x65, 0x78, 0x8B, 0xA5, 0x92, 0x4F, 0x63,
        0x00, 0x00, 0x90, 0x02, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78,
};

struct cw_battery {
        struct i2c_client *client;

        struct workqueue_struct *cwfg_workqueue;
        struct delayed_work battery_delay_work;
        #ifdef CW2017_INTERRUPT
        struct delayed_work interrupt_work;
        #endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
        struct power_supply cw_bat;
#else
        struct power_supply *cw_bat;
#endif

        /*User set*/
        unsigned int design_capacity;
        /*IC value*/
        int version;
        int voltage;
        int capacity;
        int temp;

        /*IC config*/
        unsigned char inter_config;
        unsigned char soc_alert;
        int temp_max;
        int temp_min;

        /*Get before profile write*/
        int volt_id;
};

struct cw_battery *cw_bat;
#define DELAY100 100
/*Define CW2017 iic read function*/
static int cw_read(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
        int ret = 0;
        ret = i2c_smbus_read_i2c_block_data(client, reg, 1, buf);
        if (ret < 0) {
                chg_err("cw2017 IIC error %d\n", ret);
        }
        return ret;
}
/*Define CW2017 iic write function*/
static int cw_write(struct i2c_client *client, unsigned char reg, unsigned char const buf[])
{
        int ret = 0;
        ret = i2c_smbus_write_i2c_block_data(client, reg, 1, &buf[0]);
        if (ret < 0) {
                chg_err("cw2017 IIC error %d\n", ret);
        }
        return ret;
}
/*Define CW2017 iic read word function*/
static int cw_read_word(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
        int ret = 0;
        ret = i2c_smbus_read_i2c_block_data(client, reg, 2, buf);
        if (ret < 0) {
                chg_err("cw2017 IIC error %d\n", ret);
        }
        return ret;
}

static int cw2017_enable(void)
{
        int ret;
        unsigned char reg_val = MODE_DEFAULT;

        ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
        if (ret < 0)
                return ret;

        reg_val = MODE_SLEEP;
        ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
        if (ret < 0)
                return ret;

        reg_val = MODE_NORMAL;
        ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
        if (ret < 0)
                return ret;

        msleep(DELAY100);
        return 0;
}

static int cw_get_version(void)
{
        int ret = 0;
        unsigned char reg_val = 0;
        int version = 0;
        ret = cw_read(cw_bat->client, REG_VERSION, &reg_val);
        if (ret < 0)
                return INT_MAX;

        version = reg_val;
        chg_debug("cw2017 version = %d\n", version);
        return version;
}
#define SHIFT2  2
#define SHIFT8  8
#define SHIFT16 16
#define FACTOR 5
static int cw_get_voltage(void)
{
        int ret = 0;
        unsigned char reg_val[2] = {0 , 0};
        unsigned int voltage = 0;

        ret = cw_read_word(cw_bat->client, REG_VCELL_H, reg_val);
        if (ret < 0)
                return cw_bat->voltage;

        voltage = (reg_val[0] << SHIFT8) + reg_val[1];
        voltage = voltage  * FACTOR / SHIFT16;
        return(voltage);
}

#define DECIMAL_MAX 178 /*255 * 0.7*/
#define DECIMAL_MIN 76  /*255 * 0.3*/
#define SOC100 100
#define RETRY_CNT 5
static int cw_get_capacity(void)
{
        int ret = 0;
        unsigned char reg_val = 0;
        int soc = 0;
        int soc_decimal = 0;
        static int reset_loop = 0;
        ret = cw_read(cw_bat->client, REG_SOC_INT, &reg_val);
        if (ret < 0)
                return cw_bat->capacity;
        soc = reg_val;

        ret = cw_read(cw_bat->client, REG_SOC_DECIMAL, &reg_val);
        if (ret < 0)
                return cw_bat->capacity;
        soc_decimal = reg_val;

        if (soc > SOC100) {
                reset_loop++;
                chg_err("IC error read soc error %d times\n", reset_loop);
                if (reset_loop > RETRY_CNT) {
                        reset_loop = 0;
                        chg_err("IC error. please reset IC");
                        cw2017_enable();
                }
                return cw_bat->capacity;
        }
        else {
                reset_loop = 0;
        }

        /* case 1 : aviod swing */
        if ((soc >= (cw_bat->capacity - 1)) && (soc <= (cw_bat->capacity + 1))
        && (soc_decimal > DECIMAL_MAX || soc_decimal < DECIMAL_MIN) && soc != SOC100) {
                soc = cw_bat->capacity;
        }

        return soc;
}

#define FACTOR10 10
#define BASE_OFFSET 400
static int cw_get_temp(void)
{
        int ret = 0;
        unsigned char reg_val = 0;
        int temp = 0;
        ret = cw_read(cw_bat->client, REG_TEMP, &reg_val);
        if (ret < 0)
                return cw_bat->temp;

        temp = reg_val * FACTOR10 / SHIFT2 - BASE_OFFSET;
        return temp;
}

static int cw_get_battery_ID_vol(void)
{
        int ret = 0;
        unsigned char reg_val[2] = {0 , 0};
        long ad_buff = 0;

        ret = cw_read_word(cw_bat->client, REG_VOLT_ID_H, reg_val);
        if (ret < 0)
                return INT_MAX;
        ad_buff = (reg_val[0] << SHIFT8) + reg_val[1];
        ad_buff = ad_buff  * FACTOR / SHIFT16;

        return ad_buff;
}

static void cw_update_data(void)
{
        cw_bat->voltage = cw_get_voltage();
        cw_bat->capacity = cw_get_capacity();
        cw_bat->temp = cw_get_temp();
        chg_debug("vol = %d  cap = %d temp = %d\n",
                cw_bat->voltage, cw_bat->capacity, cw_bat->temp);
}

static int cw_init_data(void)
{
        cw_bat->version = cw_get_version();
        cw_bat->voltage = cw_get_voltage();
        cw_bat->capacity = cw_get_capacity();
        cw_bat->temp = cw_get_temp();
        if (cw_bat->version == INT_MAX) {
                return -1;
        }
        chg_debug("cw2017_gu ver = %d vol = %d  cap = %d temp = %d\n",
        cw_bat->version, cw_bat->voltage, cw_bat->capacity, cw_bat->temp);
        return 0;
}

static int cw_init_config(void)
{
        int ret = 0;
        unsigned char reg_gpio_config = 0;
        unsigned char athd = 0;
        unsigned char reg_val = 0;

        cw_bat->design_capacity = DESIGN_CAPACITY;
        /*IC config*/
        cw_bat->inter_config = GPIO_CONFIG_MIN_TEMP | GPIO_CONFIG_MAX_TEMP | GPIO_CONFIG_SOC_CHANGE;
        cw_bat->soc_alert = ATHD;
        cw_bat->temp_max = DEFINED_MAX_TEMP;
        cw_bat->temp_min = DEFINED_MIN_TEMP;

        reg_gpio_config = cw_bat->inter_config;

        ret = cw_read(cw_bat->client, REG_SOC_ALERT, &reg_val);
        if (ret < 0)
                return ret;

        athd = reg_val & CONFIG_UPDATE_FLG;
        athd = athd | cw_bat->soc_alert;

        if (reg_gpio_config & GPIO_CONFIG_MAX_TEMP_MARK) {
                reg_val = (cw_bat->temp_max + BASE_OFFSET) * SHIFT2 /FACTOR10;
                ret = cw_write(cw_bat->client, REG_TEMP_MAX, &reg_val);
                if (ret < 0)
                        return ret;
        }
        if (reg_gpio_config & GPIO_CONFIG_MIN_TEMP_MARK) {
                reg_val = (cw_bat->temp_min + BASE_OFFSET) * SHIFT2 /FACTOR10;
                ret = cw_write(cw_bat->client, REG_TEMP_MIN, &reg_val);
                if (ret < 0)
                        return ret;
        }

        ret = cw_write(cw_bat->client, REG_GPIO_CONFIG, &reg_gpio_config);
        if (ret < 0)
                return ret;

        ret = cw_write(cw_bat->client, REG_SOC_ALERT, &athd);
        if (ret < 0)
                return ret;

        return 0;
}

/*CW2017 update profile function, Often called during initialization*/
#define CNT30 30
#define DEFVAL 100
static int cw_update_config_info(void)
{
        int ret = 0;
        unsigned char i = 0;
        unsigned char reg_val = 0;
        unsigned char reg_val_dig = 0;
        int count = 0;

        /* update new battery info */
        for (i = 0; i < SIZE_BATINFO; i++) {
                reg_val = config_info[i];
                ret = cw_write(cw_bat->client, REG_BATINFO + i, &reg_val);
                if (ret < 0)
                        return ret;
                chg_debug("w reg[%02X] = %02X\n", REG_BATINFO +i, reg_val);
        }

        ret = cw_read(cw_bat->client, REG_SOC_ALERT, &reg_val);
        if (ret < 0)
                return ret;

        reg_val |= CONFIG_UPDATE_FLG; /* set UPDATE_FLAG */
        ret = cw_write(cw_bat->client, REG_SOC_ALERT, &reg_val);
        if (ret < 0)
                return ret;

        ret = cw2017_enable();
        if (ret < 0)
                return ret;

        while (cw_get_version() == NO_START_VERSION) {
                msleep(DELAY100);
                count++;
                if (count > CNT30)
                        break;
        }

        for (i = 0; i < CNT30; i++) {
                msleep(DELAY100);
                ret = cw_read(cw_bat->client, REG_SOC_INT, &reg_val);
                ret = cw_read(cw_bat->client, REG_SOC_INT + 1, &reg_val_dig);
                chg_debug("i = %d soc = %d, .soc = %d\n", i, reg_val, reg_val_dig);
                if (ret < 0)
                        return ret;
                else if (reg_val <= DEFVAL)
                        break;
        }

        return 0;
}

static int cw_init(void)
{
        int ret;
        int i;
        unsigned char reg_val = MODE_NORMAL;
        unsigned char config_flg = 0;

        ret = cw_read(cw_bat->client, REG_MODE_CONFIG, &reg_val);
        if (ret < 0)
                return ret;

        ret = cw_read(cw_bat->client, REG_SOC_ALERT, &config_flg);
        if (ret < 0)
                return ret;

        if (reg_val != MODE_NORMAL || ((config_flg & CONFIG_UPDATE_FLG) == 0x00)) {
                ret = cw_update_config_info();
                if (ret < 0)
                        return ret;
        } else {
                for (i = 0; i < SIZE_BATINFO; i++) {
                        ret = cw_read(cw_bat->client, REG_BATINFO +i, &reg_val);
                        if (ret < 0)
                                return ret;

                        chg_debug("cw2017 reg[%02X] = %02X\n", REG_BATINFO +i, reg_val);
                        if (config_info[i] != reg_val) {
                                break;
                        }
                }
                if (i != SIZE_BATINFO) {
                        ret = cw_update_config_info();
                        if (ret < 0)
                                return ret;
                }
        }

        return 0;
}

static void cw_bat_work(struct work_struct *work)
{
        struct delayed_work *delay_work;
        delay_work = container_of(work, struct delayed_work, work);

        cw_update_data();

        #ifdef CW_PROPERTIES
        #if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
        power_supply_changed(&cw_bat->cw_bat);
        #else
        power_supply_changed(cw_bat->cw_bat);
        #endif
        #endif

        queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(QUEUE_DELAYED_WORK_TIME));
}

#ifdef CW2017_INTERRUPT

#define WAKE_LOCK_TIMEOUT	(10 * HZ)
static struct wake_lock cw2017_wakelock;

static void interrupt_work_do_wakeup(struct work_struct *work)
{
        struct delayed_work *delay_work;
        struct cw_battery *cw_bat;
        int ret = 0;
        unsigned char reg_val = 0;

        delay_work = container_of(work, struct delayed_work, work);
        cw_bat = container_of(delay_work, struct cw_battery, interrupt_work);

        ret = cw_read(cw_bat->client, REG_GPIO_CONFIG, &reg_val);
        if (ret < 0)
                return ret;
}
#define WORK_TIME 20
static irqreturn_t ops_cw2017_int_handler_int_handler(int irq, void *dev_id)
{
        wake_lock_timeout(&cw2017_wakelock, WAKE_LOCK_TIMEOUT);
        queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->interrupt_work, msecs_to_jiffies(WORK_TIME));
        return IRQ_HANDLED;
}

#endif

#define REGNUM 16
static int cw2017_dump(void)
{
        int i = 0;
        unsigned char reg_val = 0;
        int ret = 0;

        for (i = 0; i < REGNUM; i++) {
                ret = cw_read(cw_bat->client, i, &reg_val);
                if (ret < 0)
                        return ret;
                chg_debug("CW2017 reg[%d]= %x ", i, reg_val);
        }
        return ret;
}
#define DEF_CURRENT 500
static int cw2017_get_average_current(void)
{
        int ret = DEF_CURRENT;
        return ret;
}

static int cw2017_get_battery_soc(void)
{
        int soc = 0;
        soc = cw_get_capacity();
        chg_debug("[%s]battery uisoc = %d.\n",  __func__, soc);
        return soc;
}

static int cw2017_get_batt_remaining_capacity(void)
{
        return -1;
}

static int cw2017_get_battery_temperature(void)
{
        int ret = 0;

        ret = cw_get_temp();

        return ret;
}

static int cw2017_get_battery_mvolts(void)
{
        int ret = 0;

        ret = cw_get_voltage();

        return ret;
}
static bool cw2017_get_battery_hmac(void)
{
        return true;
}
bool cw2017_get_battery_authenticate(void)
{
        int bat_id = 0;
        int i = 0;
        bool flag = false;
        bat_id = cw_get_battery_ID_vol();
        if (bat_id < 0) {
                chg_err("cw2017 bat id read failed\n");
        }

        for (i = 0; i < BAT_COUNT * SHIFT2; ++i) {
                if (bat_id < g_bat_id_vol[i])
                        break;
                i++;

                if (bat_id < g_bat_id_vol[i]) {
                        flag = true;
                        break;
                }
        }
        chg_err("cw2017 bat id = %d; flag = %d\n", bat_id, flag);
        return flag;
}

static int  cw2017_get_battery_fc(void)
{
        return DESIGN_CAPACITY;
}

static int cw2017_dumy(void)
{
        return 0;
}

static void cw2017_void_dumy(bool full)
{
        /* Do nothing */
}


static struct oplus_gauge_operations battery_cw2017_gauge = {
        .get_battery_mvolts				= cw2017_get_battery_mvolts,
        .get_battery_fc 				= cw2017_get_battery_fc,
        .get_battery_temperature		= cw2017_get_battery_temperature,
        .get_batt_remaining_capacity 	= cw2017_get_batt_remaining_capacity,
        .get_battery_soc				= cw2017_get_battery_soc,
        .get_average_current			= cw2017_get_average_current,
        .get_battery_fcc				= cw2017_get_battery_fc,
        .get_battery_cc					= cw2017_get_battery_fc,
        .get_prev_batt_fcc  			= cw2017_get_battery_fc,
        .get_battery_authenticate		= cw2017_get_battery_authenticate,
        .get_battery_hmac				= cw2017_get_battery_hmac,
        .get_prev_battery_mvolts		= cw2017_get_battery_mvolts,
        .get_prev_battery_temperature   = cw2017_get_battery_temperature,
        .set_battery_full				= cw2017_void_dumy,
        .get_prev_battery_soc			= cw2017_get_battery_soc,
        .get_prev_average_current		= cw2017_dumy,
        .get_prev_batt_remaining_capacity = cw2017_get_battery_soc,
        .get_battery_mvolts_2cell_max	= cw2017_get_battery_fc,
        .get_battery_mvolts_2cell_min	= cw2017_get_battery_fc,
        .get_prev_battery_mvolts_2cell_max = cw2017_get_battery_fc,
        .get_prev_battery_mvolts_2cell_min = cw2017_get_battery_fc,
        .update_battery_dod0			= cw2017_dumy,
        .update_soc_smooth_parameter	= cw2017_dumy,
        .get_battery_cb_status			= cw2017_dumy,
        .get_battery_soh				= cw2017_dumy,
        .dump_register					= cw2017_dump,
};

#define DELAY200 200
#define LOOPCNT 3
static int cw2017_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int ret;
        int loop = 0;
        struct oplus_gauge_chip	*chip;
#ifdef CW2017_INTERRUPT
        int irq = 0;
#endif

#ifdef CW_PROPERTIES
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
        struct power_supply_desc *psy_desc;
        struct power_supply_config psy_cfg = {0};
#endif
#endif

        cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
        if (!cw_bat) {
                chg_err("cw_bat create fail!\n");
                return -ENOMEM;
        }

        i2c_set_clientdata(client, cw_bat);

        cw_bat->client = client;
        cw_bat->volt_id = 0;

        ret = cw_init();
        while ((loop++ < LOOPCNT) && (ret != 0)) {
                msleep(DELAY200);
                ret = cw_init();
        }
        if (ret) {
                chg_err("cw2017 init fail!\n");
                return ret;
        }

        ret = cw_init_config();
        if (ret) {
                chg_err("cw2017 init config fail!\n");
                return ret;
        }

        ret = cw_init_data();
        if (ret) {
                chg_err("cw2017 init data fail!\n");
                return ret;
        }

        cw_bat->cwfg_workqueue = create_singlethread_workqueue("cwfg_gauge");
        INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
        queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work , msecs_to_jiffies(50));

#ifdef CW2017_INTERRUPT
        INIT_DELAYED_WORK(&cw_bat->interrupt_work, interrupt_work_do_wakeup);
        wake_lock_init(&cw2017_wakelock, WAKE_LOCK_SUSPEND, "cw2017_detect");
        if (client->irq > 0) {
                irq = client->irq;
                ret = request_irq(irq, ops_cw2017_int_handler_int_handler, IRQF_TRIGGER_FALLING, "cw2017_detect", cw_bat);
                if (ret < 0) {
                        chg_err(KERN_ERR"fault interrupt registration failed err = %d\n", ret);
                }
                enable_irq_wake(irq);
        }
#endif

        chip = (struct oplus_gauge_chip*) kzalloc(sizeof(struct oplus_gauge_chip),
                        GFP_KERNEL);
        if (!chip) {
                chg_err("oplus_gauge_chip devm_kzalloc failed.\n");
                return -ENOMEM;
        }

        chip->gauge_ops = &battery_cw2017_gauge;
        oplus_gauge_init(chip);

        hardwareinfo_set_prop(HARDWARE_BATTERY_ID, battery_name[0]);
        hardwareinfo_set_prop(HARDWARE_BMS_GAUGE, "CW2017");

        chg_err("cw2017 driver probe success!\n");
        return 0;
}

static int cw2017_remove(struct i2c_client *client)
{
        return 0;
}

#ifdef CONFIG_PM
static int cw_bat_suspend()
{
        chg_err("cw_bat_suspend\n");
        cancel_delayed_work(&cw_bat->battery_delay_work);
        return 0;
}

static int cw_bat_resume()
{
        chg_err("cw_bat_resume\n");
        queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(2));
        return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
        .suspend = cw_bat_suspend,
        .resume = cw_bat_resume,
};
#endif

static const struct i2c_device_id cw2017_id_table[] = {
        {CWFG_NAME, 0},
        {}
};

static struct of_device_id cw2017_match_table[] = {
        { .compatible = "cellwise,cw2017", },
        { },
};

static struct i2c_driver cw2017_driver = {
        .driver = {
                .name = CWFG_NAME,
#ifdef CONFIG_PM
                .pm = &cw_bat_pm_ops,
#endif
                .owner  = THIS_MODULE,
                .of_match_table = cw2017_match_table,
        },
        .probe = cw2017_probe,
        .remove = cw2017_remove,
        .id_table = cw2017_id_table,
};

module_i2c_driver(cw2017_driver);

MODULE_AUTHOR("CW2017-gauge");
MODULE_DESCRIPTION("CW2017 FGADC Device Driver V1.2");
MODULE_LICENSE("GPL");
