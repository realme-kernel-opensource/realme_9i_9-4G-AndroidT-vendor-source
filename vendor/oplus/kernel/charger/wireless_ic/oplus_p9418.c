// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Op. All rights reserved.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>

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
#include <linux/rtc.h>
#include <linux/random.h>
#include <linux/notifier.h>
#include <linux/alarmtimer.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/iio/consumer.h>
#include <uapi/linux/qg.h>
#include <linux/timer.h>
#include <linux/fs.h>

#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>

#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "../oplus_charger.h"
#include "../oplus_wireless.h"
#include "oplus_chargepump.h"
#include "oplus_p9418.h"
#include "oplus_p9418_fw.h"
#include <soc/oplus/system/boot_mode.h>

#define DEBUG_BY_FILE_OPS
#define P9418_WAIT_TIME 600		/* sec */

struct oplus_p9418_ic *p9418_chip = NULL;

static int g_pen_ornot;
static int g_count;
static char g_int_flag_data[4];
struct delayed_work idt_timer_work;
static DECLARE_WAIT_QUEUE_HEAD(i2c_waiter);

extern struct oplus_chg_chip *g_oplus_chip;
extern int oplus_get_idt_en_val(void);
int p9418_get_idt_int_val(void);
int p9418_get_vbat_en_val(void);
int p9418_hall_notifier_callback(struct notifier_block *nb, unsigned long event, void *data);
static void p9418_power_onoff_switch(int value);

void __attribute__((weak)) notify_pen_state(int state) {return;}

extern struct blocking_notifier_head hall_notifier;
static struct notifier_block p9418_notifier ={
	.notifier_call = p9418_hall_notifier_callback,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
/* only for GKI compile */
unsigned int __attribute__((weak)) get_PCB_Version(void)
{
    return EVT2 + 1;
}
#endif

static DEFINE_MUTEX(p9418_i2c_access);

#define P9418_ADD_COUNT      2
static int __p9418_read_reg(struct oplus_p9418_ic *chip, int reg, char *returnData, int count)
{
	/* We have 16-bit i2c addresses - care for endianness */
	char cmd_buf[2]={ reg >> 8, reg & 0xff };
	int ret = 0;
	int i;
	char val_buf[20] = {0};

	for (i = 0; i < count; i++) {
		val_buf[i] = 0;
	}

	ret = i2c_master_send(chip->client, cmd_buf, P9418_ADD_COUNT);
	if (ret < P9418_ADD_COUNT) {
		chg_err("%s: i2c read error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(chip->client, val_buf, count);
	if (ret < count) {
		chg_err("%s: i2c read error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	for (i = 0; i < count; i++) {
		*(returnData + i) = val_buf[i];
	}

	return 0;
}

static int __p9418_write_reg(struct oplus_p9418_ic *chip, int reg, int val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };

	ret = i2c_master_send(chip->client, data, 3);
	if (ret < 3) {
		chg_err("%s: i2c write error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int p9418_write_reg_multi_byte(struct oplus_p9418_ic *chip, int reg, char *cbuf, int length)
{
	int ret;
	int send_length;
	unsigned char *data_w;

	send_length = length + 2;
	data_w = kzalloc(send_length, GFP_KERNEL);
	if (!data_w) {
		chg_err("can't alloc memory!\n");
		return -1;
	}

	data_w[0] = reg >> 8;
	data_w[1] = reg & 0xff;

	memcpy(data_w + 2, cbuf, length);

	mutex_lock(&p9418_i2c_access);

	ret = i2c_master_send(chip->client, data_w, send_length);
	if (ret < send_length) {
		chg_err("%s: i2c write error, reg: %x\n", __func__, reg);
		kfree(data_w);
		mutex_unlock(&p9418_i2c_access);
		return ret < 0 ? ret : -EIO;
	}

	mutex_unlock(&p9418_i2c_access);

	kfree(data_w);
	return 0;
}

static int p9418_read_reg(struct oplus_p9418_ic *chip, int reg, char *returnData, int count)
{
	int ret = 0;

	mutex_lock(&p9418_i2c_access);
	ret = __p9418_read_reg(chip, reg, returnData, count);
	mutex_unlock(&p9418_i2c_access);
	return ret;
}

static int p9418_config_interface (struct oplus_p9418_ic *chip, int RegNum, int val, int MASK)
{
	char p9418_reg = 0;
	int ret = 0;

	mutex_lock(&p9418_i2c_access);
	ret = __p9418_read_reg(chip, RegNum, &p9418_reg, 1);

	p9418_reg &= ~MASK;
	p9418_reg |= val;

	ret = __p9418_write_reg(chip, RegNum, p9418_reg);

	mutex_unlock(&p9418_i2c_access);

	return ret;
}

static void p9418_check_clear_irq(struct oplus_p9418_ic *chip)
{
	int rc;

	chg_err("p9418_check_clear_irq----------\n");

	rc = p9418_read_reg(chip, P9418_REG_INT_FLAG, chip->int_flag_data, 4);
	if (rc) {
		chg_err("Couldn't read 0x%04x rc = %x\n", P9418_REG_INT_FLAG, rc);
	} else {
		p9418_write_reg_multi_byte(chip, P9418_REG_INT_CLR, chip->int_flag_data, 4);
		p9418_config_interface(chip, P9418_REG_RTX_CMD, 0x02, 0xFF);
	}
}

static void check_int_enable(struct oplus_p9418_ic *chip)
{
	char reg_int[2];
	int rc;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: p9418_chip not ready!\n", __func__);
		return;
	}
	rc = p9418_read_reg(chip, P9418_REG_INT_EN, reg_int, 2);
	if (rc) {
		chg_err("Couldn't read 0x%04x rc = %x\n", P9418_REG_INT_EN, rc);
	} else {
		if ((reg_int[0] != 0x01) ||	(reg_int[1] != 0xFF)) {
			reg_int[0] = 0x01;
			reg_int[1] = 0xFF;
			p9418_write_reg_multi_byte(chip, P9418_REG_INT_EN, reg_int, 2);
		}
	}
}

static void p9418_set_tx_mode(int value)
{
	struct oplus_p9418_ic *chip = p9418_chip;
	char reg_tx;
	int rc;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: p9418_chip not ready!\n", __func__);
		return;
	}

	rc = p9418_read_reg(chip, P9418_REG_RTX_STATUS, &reg_tx, 1);
	if (rc) {
		chg_err("Couldn't read 0x%04x rc = %x\n", P9418_REG_INT_EN, rc);
	} else {
		if (value && (P9418_RTX_READY & reg_tx)) {
			chg_err("set tx enable\n");
			p9418_config_interface(chip, P9418_REG_RTX_CMD, 0x01, 0xFF);
		} else if (!value && (P9418_RTX_TRANSFER & reg_tx)){
			chg_err("set tx disable\n");
			p9418_config_interface(chip, P9418_REG_RTX_CMD, 0x04, 0xFF);
		} else {
			chg_err("tx status err, return\n");
			return;
		}
	}
}

static void p9418_set_ble_addr(struct oplus_p9418_ic *chip)
{
	int rc;
	unsigned char addr[6];
	unsigned char decode_addr[6];
	unsigned long int ble_addr = 0;

	rc = p9418_read_reg(chip, P9418_BLE_MAC_ADDR0, &addr[0], 1);
	if (rc) {
		chg_err("Couldn't read 0x%02x rc = %x\n", P9418_BLE_MAC_ADDR0, rc);
		return;
	}

	rc = p9418_read_reg(chip, P9418_BLE_MAC_ADDR1, &addr[1], 1);
	if (rc) {
		chg_err("Couldn't read 0x%02x rc = %x\n", P9418_BLE_MAC_ADDR1, rc);
		return;
	}

	rc = p9418_read_reg(chip, P9418_BLE_MAC_ADDR2, &addr[2], 1);
	if (rc) {
		chg_err("Couldn't read 0x%02x rc = %x\n", P9418_BLE_MAC_ADDR2, rc);
		return;
	}

	rc = p9418_read_reg(chip, P9418_BLE_MAC_ADDR3, &addr[3], 1);
	if (rc) {
		chg_err("Couldn't read 0x%02x rc = %x\n", P9418_BLE_MAC_ADDR3, rc);
		return;
	}

	rc = p9418_read_reg(chip, P9418_BLE_MAC_ADDR4, &addr[4], 1);
	if (rc) {
		chg_err("Couldn't read 0x%02x rc = %x\n", P9418_BLE_MAC_ADDR4, rc);
		return;
	}

	rc = p9418_read_reg(chip, P9418_BLE_MAC_ADDR5, &addr[5], 1);
	if (rc) {
		chg_err("Couldn't read 0x%02x rc = %x\n", P9418_BLE_MAC_ADDR5, rc);
		return;
	}

	/* decode high 3 bytes */
	decode_addr[5] = ((addr[5] & 0x0F) << 4) | ((addr[2] & 0xF0) >> 4);
	decode_addr[4] = ((addr[4] & 0x0F) << 4) | ((addr[1] & 0xF0) >> 4);
	decode_addr[3] = ((addr[3] & 0x0F) << 4) | ((addr[0] & 0xF0) >> 4);

	/* decode low 3 bytes */
	decode_addr[2] = ((addr[2] & 0x0F) << 4) | ((addr[5] & 0xF0) >> 4);
	decode_addr[1] = ((addr[1] & 0x0F) << 4) | ((addr[4] & 0xF0) >> 4);
	decode_addr[0] = ((addr[0] & 0x0F) << 4) | ((addr[3] & 0xF0) >> 4);

	/* caculate final ble mac addr */
	ble_addr = decode_addr[5]  << 16 | decode_addr[4] << 8 | decode_addr[3];
	chip->ble_mac_addr = ble_addr << 24 | decode_addr[2] << 16 | decode_addr[1] << 8 | decode_addr[0];
	chg_err("p9418 ble_mac_addr = 0x%016llx,\n", chip->ble_mac_addr);
}

static void p9418_set_protect_parameter(struct oplus_p9418_ic *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: p9418_chip not ready!\n", __func__);
		return;
	}

	p9418_config_interface(chip, P9418_REG_OCP_THRESHOLD, chip->ocp_threshold, 0xFF);
	p9418_config_interface(chip, P9418_REG_OVP_THRESHOLD, chip->ovp_threshold, 0xFF);
	p9418_config_interface(chip, P9418_REG_LVP_THRESHOLD, chip->lvp_threshold, 0xFF);
	p9418_config_interface(chip, P9418_REG_POCP_THRESHOLD1, chip->pcop_threshold1, 0xFF);
	p9418_config_interface(chip, P9418_REG_POCP_THRESHOLD2, chip->pcop_threshold2, 0xFF);
	p9418_config_interface(chip, P9418_REG_FOD_THRESHOLD, chip->fod_threshold, 0xFF);

	chg_err("config ocp_threshold = 0x%02x, ovp_threshold = 0x%02x \
			lvp_threshold = 0x%02x, fod_threshold = 0x%02x \
			pcop_threshold1 = 0x%02x, pcop_threshold2 = 0x%02x\n",
			chip->ocp_threshold, chip->ovp_threshold,
			chip->lvp_threshold, chip->fod_threshold,
			chip->pcop_threshold1, chip->pcop_threshold2);
}

static void p9418_set_private_data(struct oplus_p9418_ic *chip)
{
	int rc = 0, i = 0, count = 3;
	unsigned char data[6];
	unsigned long int private_data = 0;

	rc = p9418_read_reg(chip, P9418_PRIVATE_DATA_REG, &data[0], count);
	if (rc) {
		chg_err("Couldn't read 0x%02x rc = %x\n", P9418_PRIVATE_DATA_REG, rc);
		return;
	}

	for (i = 0; i < count; i++) {
		chg_err("p9418 private data %d = 0x%02x\n", i, data[count-1-i]);
		private_data += data[count-1-i];
		if (i < count - 1) {
			private_data = private_data << 8;
		}
	}

	chip->private_pkg_data = private_data;
	chg_err("p9418 private_pkg_data = 0x%llx,\n", chip->private_pkg_data);
}

static void p9418_send_uevent(struct device *dev, bool status, unsigned long int mac_addr)
{
	char status_string[64], addr_string[64];
	char *envp[] = { status_string, addr_string, NULL };
	int ret = 0;

	sprintf(status_string, "pencil_status=%d", status);
	sprintf(addr_string, "pencil_addr=%llx", mac_addr);
	ret = kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
	if (ret)
		chg_err("%s: kobject_uevent_fail, ret = %d", __func__, ret);

	chg_err("send uevent:%s, %s, path:%s.\n", status_string, addr_string, kobject_get_path(&dev->kobj, GFP_KERNEL));
}

static void p9418_power_enable(struct oplus_p9418_ic *chip, bool enable)
{
	if (!chip) {
		return;
	}

	if (enable) {
		p9418_set_vbat_en_val(1);
		udelay(1000);
		p9418_set_booster_en_val(1);
		chip->is_power_on = true;
	} else {
		p9418_set_booster_en_val(0);
		udelay(1000);
		p9418_set_vbat_en_val(0);
		chip->is_power_on = false;
	}

	return;
}

static void p9418_disable_tx_power(struct oplus_p9418_ic *chip)
{
		chg_err("<~WPC~> p9418_disable_tx_power\n");
		p9418_set_tx_mode(0);
		p9418_power_enable(chip, false);

		chip->present = 0;
		chip->ble_mac_addr = 0;
		chip->private_pkg_data = 0;
		p9418_send_uevent(chip->wireless_dev, chip->present, chip->ble_mac_addr);
}

void p9418_reg_print(void)
{
	char debug_data[6];

	p9418_read_reg(p9418_chip, P9418_REG_RTX_ERR_STATUS, debug_data, 2);
	chg_err("0x74 REG: 0x%02X 0x%02X\n",
			debug_data[0],debug_data[1]);

	p9418_read_reg(p9418_chip, P9418_REG_RTX_STATUS, debug_data, 1);
	chg_err("0x78 REG: 0x%02X\n", debug_data[0]);

	p9418_read_reg(p9418_chip, P9418_BLE_MAC_ADDR0, &debug_data[0], 1);
	p9418_read_reg(p9418_chip, P9418_BLE_MAC_ADDR1, &debug_data[1], 1);
	p9418_read_reg(p9418_chip, P9418_BLE_MAC_ADDR2, &debug_data[2], 1);
	p9418_read_reg(p9418_chip, P9418_BLE_MAC_ADDR3, &debug_data[3], 1);
	p9418_read_reg(p9418_chip, P9418_BLE_MAC_ADDR4, &debug_data[4], 1);
	p9418_read_reg(p9418_chip, P9418_BLE_MAC_ADDR5, &debug_data[5], 1);
	chg_err("0xbe-0xc3 REG: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			debug_data[0], debug_data[1], debug_data[2],
			debug_data[3], debug_data[4], debug_data[5]);
}

static int p9418_load_bootloader(struct oplus_p9418_ic *chip)
{
	int rc = 0;
	// configure the system
	rc = __p9418_write_reg(chip, 0x3000, 0x5a); // write key
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x3000 reg error!\n");
		return rc;
	}

	rc = __p9418_write_reg(chip, 0x3004, 0x00); // set HS clock
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x3004 reg error!\n");
		return rc;
	}

	rc = __p9418_write_reg(chip, 0x3008, 0x09); // set AHB clock
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x3008 reg error!\n");
		return rc;
	}

	rc = __p9418_write_reg(chip, 0x300C, 0x05); // configure 1us pulse
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x300c reg error!\n");
		return rc;
	}

	rc = __p9418_write_reg(chip, 0x300D, 0x1d); // configure 500ns pulse
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x300d reg error!\n");
		return rc;
	}

	rc = __p9418_write_reg(chip, 0x3040, 0x11); // Enable MTP access via I2C
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x3040 reg error!\n");
		return rc;
	}

	msleep(10);

	chg_err("<IDT UPDATE>-b-2--!\n");
	rc = __p9418_write_reg(chip, 0x3040, 0x10); // halt microcontroller M0
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x3040 reg error!\n");
		return rc;
	}

	msleep(10);

	chg_err("<IDT UPDATE>-b-3--!\n");
	rc = p9418_write_reg_multi_byte(
		chip, 0x0800, MTPBootloader9415,
		sizeof(MTPBootloader9415)); // load provided by IDT array
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x1c00 reg error!\n");
		return rc;
	}

	chg_err("<IDT UPDATE>-b-4--!\n");
	rc = __p9418_write_reg(chip, 0x400, 0); // initialize buffer
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x400 reg error!\n");
		return rc;
	}

	chg_err("<IDT UPDATE>-b-5--!\n");
	rc = __p9418_write_reg(chip, 0x3048, 0xD0); // map RAM address 0x1c00 to OTP 0x0000
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x3048 reg error!\n");
		return rc;
	}

	chg_err("<IDT UPDATE>-b-6--!\n");
	rc = __p9418_write_reg(chip, 0x3040, 0x80); // run M0

	return 0;
}

static int p9418_load_fw(struct oplus_p9418_ic *chip, unsigned char *fw_data, int CodeLength)
{
	unsigned char write_ack = 0;
	int rc = 0;

	rc = p9418_write_reg_multi_byte(chip, 0x400, fw_data,
							((CodeLength + 8 + 15) / 16) * 16);
	if (rc != 0) {
		chg_err("<IDT UPDATE>ERROR: write multi byte data error!\n");
		goto LOAD_ERR;
	}
	rc = __p9418_write_reg(chip, 0x400, 0x01);
	if (rc != 0) {
		chg_err("<IDT UPDATE>ERROR: on OTP buffer validation\n");
		goto LOAD_ERR;
	}

	do {
		msleep(20);
		rc = p9418_read_reg(chip, 0x401, &write_ack, 1);
		if (rc != 0) {
			chg_err("<IDT UPDATE>ERROR: on reading OTP buffer status\n");
			goto LOAD_ERR;
		}
	} while ((write_ack & 0x01) != 0);

	// check status
	if (write_ack != 2) { // not OK
		if (write_ack == 4)
			chg_err("<IDT UPDATE>ERROR: WRITE ERR\n");
		else if (write_ack == 8)
			chg_err("<IDT UPDATE>ERROR: CHECK SUM ERR\n");
		else
			chg_err("<IDT UPDATE>ERROR: UNKNOWN ERR\n");

		rc = -1;
	}
LOAD_ERR:
	return rc;
}

static int p9418_MTP(struct oplus_p9418_ic *chip, unsigned char *fw_buf, int fw_size)
{
	int rc;
	int i, j;
	unsigned char *fw_data;
	unsigned char write_ack;
	unsigned short int StartAddr;
	unsigned short int CheckSum;
	unsigned short int CodeLength;
	// pure fw size not contains last 128 bytes fw version.
	int pure_fw_size = fw_size - 128;

	chg_err("<IDT UPDATE>--1--!\n");

	rc = p9418_load_bootloader(chip);
	if (rc != 0) {
		chg_err("<IDT UPDATE>Update bootloader 1 error!\n");
		return rc;
	}

	msleep(100);

	chg_err("<IDT UPDATE>The idt firmware size: %d!\n", fw_size);

	// program pages of 128 bytes
	// 8-bytes header, 128-bytes data, 8-bytes padding to round to 16-byte boundary
	fw_data = kzalloc(144, GFP_KERNEL);
	if (!fw_data) {
		chg_err("<IDT UPDATE>can't alloc memory!\n");
		return -EINVAL;
	}

	//ERASE FW VERSION(the last 128 byte of the MTP)
	memset(fw_data, 0x00, 144);
	StartAddr = pure_fw_size;
	CheckSum = StartAddr;
	CodeLength = 128;
	for (j = 127; j >= 0; j--)
		CheckSum += fw_data[j + 8]; // add the non zero values.

	CheckSum += CodeLength; // finish calculation of the check sum
	memcpy(fw_data + 2, (char *)&StartAddr, 2);
	memcpy(fw_data + 4, (char *)&CodeLength, 2);
	memcpy(fw_data + 6, (char *)&CheckSum, 2);
	rc = p9418_load_fw(chip, fw_data, CodeLength);
	if (rc < 0) { // not OK
		chg_err("<IDT UPDATE>ERROR: erase fw version ERR\n");
		goto MTP_ERROR;
	}

	// upgrade fw
	memset(fw_data, 0x00, 144);
	for (i = 0; i < pure_fw_size; i += 128) {
		chg_err("<IDT UPDATE>Begin to write chunk %d!\n", i);

		StartAddr = i;
		CheckSum = StartAddr;
		CodeLength = 128;

		memcpy(fw_data + 8, fw_buf + i, 128);

		j = pure_fw_size - i;
		if (j < 128) {
			j = ((j + 15) / 16) * 16;
			CodeLength = (unsigned short int)j;
		} else {
			j = 128;
		}

		j -= 1;
		for (; j >= 0; j--)
			CheckSum += fw_data[j + 8]; // add the non zero values

		CheckSum += CodeLength; // finish calculation of the check sum

		memcpy(fw_data + 2, (char *)&StartAddr, 2);
		memcpy(fw_data + 4, (char *)&CodeLength, 2);
		memcpy(fw_data + 6, (char *)&CheckSum, 2);

		//typedef struct { // write to structure at address 0x400
		// u16 Status;
		// u16 StartAddr;
		// u16 CodeLength;
		// u16 DataChksum;
		// u8 DataBuf[128];
		//} P9220PgmStrType;
		// read status is guaranteed to be != 1 at this point

		rc = p9418_load_fw(chip, fw_data, CodeLength);
		if (rc < 0) { // not OK
			chg_err("<IDT UPDATE>ERROR: write chunk %d ERR\n", i);
			goto MTP_ERROR;
		}
	}

	msleep(100);
	//disable power P9415
	chg_err("<IDT UPDATE> Disable power P9415.\n");
	p9418_power_enable(chip, false);
	msleep(3000);

	//power P9415 again
	chg_err("<IDT UPDATE> Power P9415 again.\n");
	p9418_power_enable(chip, true);
	msleep(500);

	// Verify
	rc = p9418_load_bootloader(chip);
	if (rc != 0) {
		chg_err("<IDT UPDATE>Update bootloader 2 error!\n");
		return rc;
	}
	msleep(100);
	rc = __p9418_write_reg(chip, 0x402, 0x00); // write start address
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x402 reg error!\n");
		return rc;
	}
	rc = __p9418_write_reg(chip, 0x403, 0x00); // write start address
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x402 reg error!\n");
		return rc;
	}
	rc = __p9418_write_reg(chip, 0x404, pure_fw_size & 0xff); // write FW length low byte
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x404 reg error!\n");
		return rc;
	}
	rc = __p9418_write_reg(chip, 0x405, (pure_fw_size >> 8) & 0xff); // write FW length high byte
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x405 reg error!\n");
		return rc;
	}

	// write CRC from FW release package
	fw_data[0] = fw_buf[pure_fw_size + 0x08];
	fw_data[1] = fw_buf[pure_fw_size + 0x09];
	p9418_write_reg_multi_byte(chip, 0x406, fw_data, 2);

	rc = __p9418_write_reg(chip, 0x400, 0x11);
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x406 reg error!\n");
		return rc;
	}
	do {
		msleep(20);
		rc = p9418_read_reg(chip, 0x401, &write_ack, 1);
		if (rc != 0) {
			chg_err("<IDT UPDATE>ERROR: on reading OTP buffer status\n");
			goto MTP_ERROR;
		}
	} while ((write_ack & 0x01) != 0);
	// check status
	if (write_ack != 2) { // not OK
		if (write_ack == 4)
			chg_err("<IDT UPDATE>ERROR: CRC WRITE ERR\n");
		else if (write_ack == 8)
			chg_err("<IDT UPDATE>ERROR: CRC CHECK SUM ERR\n");
		else
			chg_err("<IDT UPDATE>ERROR: CRC UNKNOWN ERR\n");

		goto MTP_ERROR;
	}

	//Program FW VERSION(the last 128 byte of the MTP)
	// typedef struct {		// base address: 0x5f80
	//	 // Version		8 Byte
	//	 u16 ChipType;
	//	 u8	CustomerCode;
	//	 u8	empty_a;
	//	 u32 EPRFWRev;
	//	 u16 crc16;
	// } FwSettingType;
	memset(fw_data, 0x00, 144);
	StartAddr = pure_fw_size;
	CheckSum = StartAddr;
	CodeLength = 128;
	memcpy(fw_data + 8, fw_buf + StartAddr, 128);
	j = 127;
	for (; j >= 0; j--)
		CheckSum += fw_data[j + 8]; // add the non zero values.

	CheckSum += CodeLength; // finish calculation of the check sum
	memcpy(fw_data + 2, (char *)&StartAddr, 2);
	memcpy(fw_data + 4, (char *)&CodeLength, 2);
	memcpy(fw_data + 6, (char *)&CheckSum, 2);

	rc = p9418_load_fw(chip, fw_data, CodeLength);
	if (rc < 0) { // not OK
		chg_err("<IDT UPDATE>ERROR: erase fw version ERR\n");
		goto MTP_ERROR;
	}

	// restore system
	rc = __p9418_write_reg(chip, 0x3000, 0x5a); // write key
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x3000 reg error!\n");
		goto MTP_ERROR;
	}

	rc = __p9418_write_reg(chip, 0x3048, 0x00); // remove code remapping
	if (rc != 0) {
		chg_err("<IDT UPDATE>Write 0x3048 reg error!\n");
		goto MTP_ERROR;
	}

	chg_err("<IDT UPDATE>OTP Programming finished\n");

	kfree(fw_data);
	return 0;

MTP_ERROR:
	kfree(fw_data);
	return -EINVAL;
}

static int p9418_check_idt_fw_update(struct oplus_p9418_ic *chip)
{
	static int idt_update_retry_cnt = 0;
	int rc = -1;
	char temp[4] = {0, 0, 0, 0};
	unsigned char *fw_buf;
	int fw_size;
	int fw_ver_start_addr = 0;

	chg_err("<IDT UPDATE> check idt fw <><><><><><><><>\n");

	if (!chip) {
		chg_err("<IDT UPDATE> p9418 isn't ready!\n");
		return rc;
	}

	mutex_lock(&chip->flow_mutex);

	p9418_power_enable(chip, true);
	msleep(1000);

	rc = p9418_read_reg(chip, 0x001C, temp, 4);
	if (rc) {
		chg_err("<IDT UPDATE>Couldn't read 0x%04x rc = %x\n", 0x001C, rc);
	} else {
		chg_err("<IDT UPDATE>The idt fw version: %02x %02x %02x %02x\n", temp[0], temp[1], temp[2], temp[3]);

		fw_buf = p9418_idt_firmware;
		fw_size = ARRAY_SIZE(p9418_idt_firmware);

		fw_ver_start_addr = fw_size - 128;
		chg_err("<IDT UPDATE>The new fw version: %02x %02x %02x %02x\n",
				fw_buf[fw_ver_start_addr + 0x04], fw_buf[fw_ver_start_addr + 0x05],
				fw_buf[fw_ver_start_addr + 0x06], fw_buf[fw_ver_start_addr + 0x07]);

		if ((temp[0] != fw_buf[fw_ver_start_addr + 0x04]) || (temp[1] != fw_buf[fw_ver_start_addr + 0x05])
			|| (temp[2] != fw_buf[fw_ver_start_addr + 0x06]) || (temp[3] != fw_buf[fw_ver_start_addr + 0x07])) {
			chg_err("<IDT UPDATE>Need update the idt fw!\n");

			if (p9418_MTP(chip, fw_buf, fw_size) == 0) {
				chg_err("<IDT UPDATE>Update success!!!\n");
			} else {
				idt_update_retry_cnt++;
				if (idt_update_retry_cnt > 5) {
					chg_err("<IDT UPDATE>Update fail!!!\n");
				} else {
					chg_err("<IDT UPDATE>Update fail, retry %d!\n", idt_update_retry_cnt);
					rc = -1;
				}
			}
		} else {
			chg_err("<IDT UPDATE>No Need update the idt fw!\n");
		}
	}

	p9418_power_enable(chip, false);

	mutex_unlock(&chip->flow_mutex);

	return rc;
}

static int oplus_wpc_chg_parse_chg_dt(struct oplus_p9418_ic *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		chg_err("device tree info. missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,ocp_threshold",
			&chip->ocp_threshold);
	if (rc) {
		chip->ocp_threshold = 500;
	}
	chg_err("ocp_threshold[%d]\n", chip->ocp_threshold);

	rc = of_property_read_u32(node, "qcom,ovp_threshold",
			&chip->ovp_threshold);
	if (rc) {
		chip->ovp_threshold = 12000;
	}
	chg_err("ovp_threshold[%d]\n", chip->ovp_threshold);

	rc = of_property_read_u32(node, "qcom,lvp_threshold",
			&chip->lvp_threshold);
	if (rc) {
		chip->lvp_threshold = 4000;
	}
	chg_err("lvp_threshold[%d]\n", chip->lvp_threshold);

	rc = of_property_read_u32(node, "qcom,pcop_threshold1",
			&chip->pcop_threshold1);
	if (rc) {
		chip->pcop_threshold1 = 700;
	}
	chg_err("pcop_threshold1[%d]\n", chip->pcop_threshold1);

	rc = of_property_read_u32(node, "qcom,pcop_threshold2",
			&chip->pcop_threshold2);
	if (rc) {
		chip->pcop_threshold2 = 500;
	}
	chg_err("pcop_threshold2[%d]\n", chip->pcop_threshold2);

	rc = of_property_read_u32(node, "qcom,fod_threshold",
			&chip->fod_threshold);
	if (rc) {
		chip->fod_threshold = 400;
	}
	chg_err("fod_threshold[%d]\n", chip->fod_threshold);

	return 0;
}

void p9418_ept_type_detect_func(void)
{
	char recv_data[2] = {0, 0};
	int rc = 0, count_limit = 0;
	static int count = 0;
	struct oplus_p9418_ic *chip = p9418_chip;

	rc = p9418_read_reg(chip, P9418_REG_RTX_ERR_STATUS, recv_data, 2);
	if (rc) {
		chg_err("Couldn't read 0x%04x rc = %x\n", P9418_REG_RTX_ERR_STATUS, rc);
		return;
	} else {
		if ((recv_data[0] & P9418_RTX_ERR_TX_POCP) ||
				(recv_data[0] & P9418_RTX_ERR_TX_OTP) ||
				(recv_data[0] & P9418_RTX_ERR_TX_FOD) ||
				(recv_data[0] & P9418_RTX_ERR_TX_LVP) ||
				(recv_data[0] & P9418_RTX_ERR_TX_OVP) ||
				(recv_data[0] & P9418_RTX_ERR_TX_OCP)) {
			chg_err("error happen %02x%02x! p9418 will poweroff soon!\n",
					recv_data[0], recv_data[1]);
			p9418_disable_tx_power(chip);
		} else if (recv_data[1] & P9418_RTX_ERR_TX_IDAU_FAIL) {
			/* wait for private protocol design */
			return;
		} else if (recv_data[0] & P9418_RTX_ERR_TX_CEP_TIMEOUT) {
			if (!chip->cep_count_flag) {
				count_limit = 5;
				chip->cep_count_flag = 1;
				count = 0;
			} else if (!chip->cep_count_flag) {
				count_limit = 10;
				chip->cep_count_flag = 1;
				count = 0;
			}

			count++;
			if (count > count_limit) {
				chg_err("cep_timeout, count_limit = %d! p9418 will poweroff soon!\n", count_limit);
				count = 0;
				chip->cep_count_flag = 0;
				p9418_disable_tx_power(chip);
			}
		} else if (recv_data[1] & P9418_RTX_ERR_TX_EPT_CMD) {
			chg_err("power transfer terminated! p9418 will poweroff soon!\n");
			p9418_disable_tx_power(chip);
		} else {
			chg_err("ept_type detect error %02x%02x!", recv_data[0], recv_data[1]);
			return;
		}
	}
}

static bool p9418_valid_check(struct oplus_p9418_ic *chip)
{
	unsigned long int addr = 0;
	unsigned long int pdata = 0;

	if (!chip) {
		return false;
	}

	addr = chip->ble_mac_addr;
	pdata = chip->private_pkg_data;
	if ((((addr >> 40) & 0xFF) ^ ((addr >> 16) & 0xFF)) != ((pdata >> 16) & 0xFF)) {
		return false;
	}
	if ((((addr >> 32) & 0xFF) ^ ((addr >> 8) & 0xFF)) != ((pdata >> 8) & 0xFF)) {
		return false;
	}
	if ((((addr >> 24) & 0xFF) ^ (addr & 0xFF)) != (pdata & 0xFF)) {
		return false;
	}

	chg_err("p9418_valid_check: %02x %02x %02x.\n", ((pdata >> 16) & 0xFF), ((pdata >> 8) & 0xFF), (pdata & 0xFF));
	return true;
}

static void p9418_commu_data_process(struct oplus_p9418_ic *chip)
{
	int rc = 0;

	rc = p9418_read_reg(chip, P9418_REG_INT_FLAG, chip->int_flag_data, 4);
	if (rc) {
		chg_err("P9418 x30 READ FAIL\n");
	}
	chg_err("p9418_idt_event_int_func int: %02x %02x %02x %02x\n", chip->int_flag_data[0], chip->int_flag_data[1], chip->int_flag_data[2], chip->int_flag_data[3]);

	if (chip->int_flag_data[1] & P9418_INT_FLAG_BLE_ADDR) {
		chg_err("GET_BLE_ADDR!!!\n");
		p9418_set_ble_addr(chip);
		chip->upto_ble_time = ktime_to_ms(ktime_get()) - chip->tx_start_time;
		if (p9418_valid_check(chip)) {
			p9418_send_uevent(chip->wireless_dev, chip->present, chip->ble_mac_addr);
		} else {
			chg_err("check valid data failed!!!\n");
			p9418_disable_tx_power(chip);
		}
	}

	if (chip->int_flag_data[1] & P9418_INT_FLAG_PRIVATE_PKG) {
		chg_err("GET_PRIVATE_PKG!!!\n");
		p9418_set_private_data(chip);
	}

	if (chip->int_flag_data[0] & P9418_INT_FLAG_EPT) {
		p9418_ept_type_detect_func();
	}

	if (chip->int_flag_data[0] & P9418_INT_FLAG_DPING) {
		/*DPING int*/
		chg_err("DPING int!!!\n");
	}

	if (chip->int_flag_data[0] & P9418_INT_FLAG_SS) {
		/*SS int*/
		chip->present = 1;
		g_int_flag_data[0] = chip->int_flag_data[0];/* get int flag for hall ss int detect */
		notify_pen_state(1);
		chg_err("p9418 ss int ,present value :%d ,g_int_flag:0x%x", chip->present, g_int_flag_data[0]);
	}

	/*clear irq*/
	p9418_check_clear_irq(chip);
	msleep(5);

	if (p9418_get_idt_int_val() == 0) {
		chg_err("INT is 0, clear IRT again!\n");
		p9418_check_clear_irq(chip);
		schedule_delayed_work(&chip->idt_event_int_work, round_jiffies_relative(msecs_to_jiffies(10)));
	}
}

int p9418_get_idt_int_val(void)
{
	struct oplus_p9418_ic *chip = p9418_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: p9418_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->idt_int_gpio <= 0) {
		chg_err("idt_int_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->idt_int_active)
		|| IS_ERR_OR_NULL(chip->idt_int_sleep)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->idt_int_gpio);
}

static void p9418_idt_event_int_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_p9418_ic *chip = container_of(dwork, struct oplus_p9418_ic, idt_event_int_work);

	chg_err("p9418_idt_event_int_func triggered!!!\n");
	__pm_stay_awake(chip->bus_wakelock);
	wait_event_interruptible_timeout(i2c_waiter, chip->i2c_ready, msecs_to_jiffies(50));
	mutex_lock(&chip->flow_mutex);
	if (chip->i2c_ready && chip->is_power_on) {
		p9418_commu_data_process(chip);
	} else {
		chg_err("p9418_idt_event_int_func unhandled by i2c:%d, power:%d!\n", chip->i2c_ready, chip->is_power_on);
	}
	mutex_unlock(&chip->flow_mutex);
	__pm_relax(chip->bus_wakelock);
}

static irqreturn_t irq_idt_event_int_handler(int irq, void *dev_id)
{
	schedule_delayed_work(&p9418_chip->idt_event_int_work, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}

static void p9418_set_idt_int_active(struct oplus_p9418_ic *chip)
{
	gpio_direction_input(chip->idt_int_gpio);	// in
	pinctrl_select_state(chip->pinctrl,chip->idt_int_active);	// no_PULL
}

static void p9418_idt_int_irq_init(struct oplus_p9418_ic *chip)
{
	chip->idt_int_irq = gpio_to_irq(chip->idt_int_gpio);
	chg_err("chip->idt_int_irq[%d]\n",__func__, chip->idt_int_irq);
}

static void p9418_idt_int_eint_register(struct oplus_p9418_ic *chip)
{
	int retval = 0;

	p9418_set_idt_int_active(chip);
	retval = request_irq(chip->idt_int_irq, irq_idt_event_int_handler, IRQF_TRIGGER_FALLING, "p9418_idt_int", chip);	//0X01:rising edge,0x02:falling edge
	if (retval < 0) {
		chg_err("%s request idt_int irq failed.\n", __func__);
	}
	retval = enable_irq_wake(chip->idt_int_irq);
	if (retval != 0) {
		chg_err("enable_irq_wake: idt_int_irq failed %d\n", retval);
	}
}

static int p9418_idt_int_gpio_init(struct oplus_p9418_ic *chip)
{

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_p9418_ic not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	//idt_int
	chip->idt_int_active =
			pinctrl_lookup_state(chip->pinctrl, "idt_int_active");
	if (IS_ERR_OR_NULL(chip->idt_int_active)) {
		chg_err("get idt_int_active fail\n");
		return -EINVAL;
	}

	chip->idt_int_sleep =
			pinctrl_lookup_state(chip->pinctrl, "idt_int_sleep");
	if (IS_ERR_OR_NULL(chip->idt_int_sleep)) {
		chg_err("get idt_int_sleep fail\n");
		return -EINVAL;
	}

	chip->idt_int_default =
			pinctrl_lookup_state(chip->pinctrl, "idt_int_default");
	if (IS_ERR_OR_NULL(chip->idt_int_default)) {
		chg_err("get idt_int_default fail\n");
		return -EINVAL;
	}

	if (chip->idt_int_gpio > 0) {
		gpio_direction_input(chip->idt_int_gpio);
	}

	pinctrl_select_state(chip->pinctrl, chip->idt_int_active);

	return 0;
}

static int p9418_vbat_en_gpio_init(struct oplus_p9418_ic *chip)
{

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_p9418_ic not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	//vbat_en
	chip->vbat_en_active =
			pinctrl_lookup_state(chip->pinctrl, "vbat_en_active");
	if (IS_ERR_OR_NULL(chip->vbat_en_active)) {
		chg_err("get vbat_en_active fail\n");
		return -EINVAL;
	}

	chip->vbat_en_sleep =
			pinctrl_lookup_state(chip->pinctrl, "vbat_en_sleep");
	if (IS_ERR_OR_NULL(chip->vbat_en_sleep)) {
		chg_err("get vbat_en_sleep fail\n");
		return -EINVAL;
	}

	chip->vbat_en_default =
			pinctrl_lookup_state(chip->pinctrl, "vbat_en_default");
	if (IS_ERR_OR_NULL(chip->vbat_en_default)) {
		chg_err("get vbat_en_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->vbat_en_gpio, 0);
	pinctrl_select_state(chip->pinctrl,
			chip->vbat_en_sleep);

	return 0;
}

void p9418_set_vbat_en_val(int value)
{
    struct oplus_p9418_ic *chip = p9418_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_p9418_ic not ready!\n", __func__);
		return;
	}

	if (chip->vbat_en_gpio <= 0) {
		chg_err("vbat_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->vbat_en_active)
		|| IS_ERR_OR_NULL(chip->vbat_en_sleep)
		|| IS_ERR_OR_NULL(chip->vbat_en_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(chip->vbat_en_gpio, 1);
		pinctrl_select_state(chip->pinctrl,
				chip->vbat_en_default);
	} else {
		gpio_direction_output(chip->vbat_en_gpio, 0);
		pinctrl_select_state(chip->pinctrl,
				chip->vbat_en_sleep);
	}

	chg_err("<~WPC~>set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->vbat_en_gpio));
}

int p9418_get_vbat_en_val(void)
{
	struct oplus_p9418_ic *chip = p9418_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: p9418_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->vbat_en_gpio <= 0) {
		chg_err("vbat_en_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->vbat_en_active)
		|| IS_ERR_OR_NULL(chip->vbat_en_sleep)
		|| IS_ERR_OR_NULL(chip->vbat_en_default)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->vbat_en_gpio);
}

static int p9418_booster_en_gpio_init(struct oplus_p9418_ic *chip)
{

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_p9418_ic not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	//booster_en
	chip->booster_en_active =
			pinctrl_lookup_state(chip->pinctrl, "booster_en_active");
	if (IS_ERR_OR_NULL(chip->booster_en_active)) {
		chg_err("get booster_en_active fail\n");
		return -EINVAL;
	}

	chip->booster_en_sleep =
			pinctrl_lookup_state(chip->pinctrl, "booster_en_sleep");
	if (IS_ERR_OR_NULL(chip->booster_en_sleep)) {
		chg_err("get booster_en_sleep fail\n");
		return -EINVAL;
	}

	chip->booster_en_default =
			pinctrl_lookup_state(chip->pinctrl, "booster_en_default");
	if (IS_ERR_OR_NULL(chip->booster_en_default)) {
		chg_err("get booster_en_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->booster_en_gpio, 0);
	pinctrl_select_state(chip->pinctrl,
			chip->booster_en_sleep);

	chg_err("gpio_val:%d\n", gpio_get_value(chip->booster_en_gpio));

	return 0;
}

void p9418_set_booster_en_val(int value)
{
    struct oplus_p9418_ic *chip = p9418_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_p9418_ic not ready!\n", __func__);
		return;
	}

	if (chip->booster_en_gpio <= 0) {
		chg_err("booster_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->booster_en_active)
		|| IS_ERR_OR_NULL(chip->booster_en_sleep)
		|| IS_ERR_OR_NULL(chip->booster_en_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(chip->booster_en_gpio, 1);
		pinctrl_select_state(chip->pinctrl,
				chip->booster_en_active);
	} else {
		gpio_direction_output(chip->booster_en_gpio, 0);
		pinctrl_select_state(chip->pinctrl,
				chip->booster_en_sleep);
	}

	chg_err("<~WPC~>set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->booster_en_gpio));
}

int p9418_get_booster_en_val(void)
{
	struct oplus_p9418_ic *chip = p9418_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: p9418_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->booster_en_gpio <= 0) {
		chg_err("booster_en_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->booster_en_active)
		|| IS_ERR_OR_NULL(chip->booster_en_sleep)
		|| IS_ERR_OR_NULL(chip->booster_en_default)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->booster_en_gpio);
}

void p9418_update_cc_cv(struct oplus_p9418_ic *chip)
{
	int rc = 0;
	unsigned char temp[2] = {0, 0};

	if (!chip) {
		return;
	}

	rc = p9418_read_reg(chip, P9418_REG_CURRENT_MA, temp, 2);
	if (rc) {
		chg_err("Couldn't read tx_current reg 0x006E rc = %d\n", rc);
	}
	chip->tx_current = temp[1] << 8;
	chip->tx_current += temp[0];
	chg_err("tx_current: %02x %02x  dec:%d mA\n", temp[0], temp[1], chip->tx_current);

	rc = p9418_read_reg(chip, P9418_REG_VOLTAGE_MV, temp, 2);
	if (rc) {
		chg_err("Couldn't read tx_voltage reg 0x0070 rc = %d\n", rc);
	}
	chip->tx_voltage = temp[1] << 8;
	chip->tx_voltage += temp[0];
	chg_err("tx_voltage: %02x %02x  dec:%d mV\n", temp[0], temp[1], chip->tx_voltage);

	return;
}

void p9418_timer_inhall_function(struct work_struct *work)
{
	struct oplus_p9418_ic *chip = p9418_chip;

	chg_err("%s:  enter g_int_flag :0x%x\n", __func__, g_int_flag_data[0]);
	if (g_int_flag_data[0] & P9418_INT_FLAG_SS) {/*check int flag */
		g_count = 0;
		g_int_flag_data[0] = 0;
		g_pen_ornot = 1;
	}

	if (g_pen_ornot == 1) {/* pen ,read voltage current return */
		chg_err("%s:  find pen g_count: %d\n", __func__, g_count);
		if (chip->i2c_ready) {
			p9418_update_cc_cv(chip);
		}
		g_pen_ornot = 0;
		return;
	} else if (g_pen_ornot == 0) {/* not pen continue check */
		g_count++;
	}

	if (g_pen_ornot == 0 && g_count >= P9418_WAIT_TIME) {/* no ss int for P9418_WAIT_TIME sec, power off */
		chg_err("%s:  find not pen g_count: %d\n", __func__, g_count);
		p9418_power_enable(chip, false);
		g_count = 0;
		return;
	}

	schedule_delayed_work(&idt_timer_work, round_jiffies_relative(msecs_to_jiffies(500)));
	chg_err("%s:  exit,g_count:%d", __func__, g_count);
}

static void p9418_power_onoff_switch(int value)
{
	struct oplus_p9418_ic *chip = p9418_chip;

	if (!chip) {
		return;
	}
	chg_err("%s:  switch value: %d, i2c_ready:%d.\n", __func__, value, chip->i2c_ready);

	__pm_stay_awake(chip->bus_wakelock);
	wait_event_interruptible(i2c_waiter, chip->i2c_ready);
	mutex_lock(&chip->flow_mutex);

	chip->pen_status = value > 0 ? PEN_STATUS_NEAR : PEN_STATUS_FAR;
	if (chip->pen_status == PEN_STATUS_NEAR) {/* hall near, power on*/
		p9418_power_enable(chip, true);
		msleep(100);
		p9418_set_protect_parameter(chip);
		p9418_set_tx_mode(1);
		chip->tx_start_time = ktime_to_ms(ktime_get());
		schedule_delayed_work(&idt_timer_work, round_jiffies_relative(msecs_to_jiffies(50)));
	} else if (chip->pen_status == PEN_STATUS_FAR) {/* hall far, power off */
		p9418_power_enable(chip, false);

		chip->present = 0;
		chip->ble_mac_addr = 0;
		chip->private_pkg_data = 0;
		chip->tx_current = 0;
		chip->tx_voltage = 0;
		g_count = 0;
		chg_err("p9418 hall far away,present value :%d", chip->present);
		cancel_delayed_work(&idt_timer_work);
		notify_pen_state(0);
		p9418_send_uevent(chip->wireless_dev, chip->present, chip->ble_mac_addr);
	}

	mutex_unlock(&chip->flow_mutex);
	__pm_relax(chip->bus_wakelock);

	return;
}

int p9418_hall_notifier_callback(struct notifier_block *nb, unsigned long event, void *data)
{
	int value = event;
	chg_err("p9418_hall_notifier_callback enter pen_status:%d", value);
	p9418_power_onoff_switch(value);

	return NOTIFY_DONE;
}

static int p9418_idt_gpio_init(struct oplus_p9418_ic *chip)
{

    int rc=0;
	struct device_node *node = chip->dev->of_node;
    pr_err("test %s start\n",__func__);

	// Parsing gpio idt_int
	chip->idt_int_gpio = of_get_named_gpio(node, "qcom,idt_int-gpio", 0);
	if (chip->idt_int_gpio < 0 ) {
		pr_err("chip->idt_int_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->idt_int_gpio)) {
			rc = gpio_request(chip->idt_int_gpio, "idt-int-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->idt_int_gpio);
			} else {
				rc = p9418_idt_int_gpio_init(chip);
				if (rc)
					chg_err("unable to init idt_int_gpio:%d\n", chip->idt_int_gpio);
				else {
					p9418_idt_int_irq_init(chip);
					p9418_idt_int_eint_register(chip);
				}
			}
		}
		pr_err("chip->idt_int_gpio =%d\n",chip->idt_int_gpio);
	}

	// Parsing gpio vbat_en
	chip->vbat_en_gpio = of_get_named_gpio(node, "qcom,vbat_en-gpio", 0);
	if (chip->vbat_en_gpio < 0) {
		pr_err("chip->vbat_en_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->vbat_en_gpio)) {
			rc = gpio_request(chip->vbat_en_gpio, "vbat-en-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->vbat_en_gpio);
			} else {
				rc = p9418_vbat_en_gpio_init(chip);
				if (rc)
					chg_err("unable to init vbat_en_gpio:%d\n", chip->vbat_en_gpio);
			}
		}
		pr_err("chip->vbat_en_gpio =%d\n",chip->vbat_en_gpio);
	}

	// Parsing gpio booster_en
	chip->booster_en_gpio = of_get_named_gpio(node, "qcom,booster_en-gpio", 0);
	if (chip->booster_en_gpio < 0) {
		pr_err("chip->booster_en_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->booster_en_gpio)) {
			rc = gpio_request(chip->booster_en_gpio, "booster-en-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->booster_en_gpio);
			} else {
				rc = p9418_booster_en_gpio_init(chip);
				if (rc)
					chg_err("unable to init booster_en_gpio:%d\n", chip->booster_en_gpio);
			}
		}
		pr_err("chip->booster_en_gpio =%d\n",chip->booster_en_gpio);
	}

	return rc;
}

#ifdef DEBUG_BY_FILE_OPS
static int p9418_add = 0;
static ssize_t p9418_reg_store(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	char write_data[32] = {0};
	char val_buf;
	int rc;

	if (copy_from_user(&write_data, buff, len)) {
		pr_err("p9418_reg_store error.\n");
		return -EFAULT;
	}

	if (len >= ARRAY_SIZE(write_data)) {
		pr_err("data len error.\n");
		return -EFAULT;
	}
	write_data[len] = '\0';
	if (write_data[len - 1] == '\n') {
		write_data[len - 1] = '\0';
	}

	p9418_add = (int)simple_strtoul(write_data, NULL, 0);

	pr_err("%s:received data=%s, p9418_register address: 0x%02x\n", __func__, write_data, p9418_add);

	rc = p9418_read_reg(p9418_chip, p9418_add, &val_buf, 1);
	if (rc) {
		 chg_err("Couldn't read 0x%02x rc = %d\n", p9418_add, rc);
	} else {
		 chg_err("p9418_read 0x%02x = 0x%02x\n", p9418_add, val_buf);
	}

	return len;
}

static ssize_t p9418_reg_show(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char val_buf;
	int rc;
	int len = 0;

	rc = p9418_read_reg(p9418_chip, p9418_add, &val_buf, 1);
	if (rc) {
		 chg_err("Couldn't read 0x%02x rc = %d\n", p9418_add, rc);
	}

	len = sprintf(page, "reg = 0x%x, data = 0x%x\n", p9418_add, val_buf);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}

	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}

	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static const struct file_operations p9418_add_log_proc_fops = {
	.write = p9418_reg_store,
	.read = p9418_reg_show,
};

static void init_p9418_add_log(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("p9418_add_log", 0664, NULL, &p9418_add_log_proc_fops);
	if (!p) {
		pr_err("proc_create init_p9418_add_log_proc_fops fail!\n");
	}
}

static ssize_t p9418_data_log_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	char write_data[32] = {0};
	int critical_log = 0;
	int rc;

	if (copy_from_user(&write_data, buff, len)) {
		pr_err("bat_log_write error.\n");
		return -EFAULT;
	}

	if (len >= ARRAY_SIZE(write_data)) {
		pr_err("data len error.\n");
		return -EFAULT;
	}
	write_data[len] = '\0';
	if (write_data[len - 1] == '\n') {
		write_data[len - 1] = '\0';
	}

	critical_log = (int)simple_strtoul(write_data, NULL, 0);
	if (critical_log > 0xFF) {
		critical_log = 0xFF;
	}

	pr_err("%s:received data=%s, p9418_data=%x\n", __func__, write_data, critical_log);

	rc = p9418_config_interface(p9418_chip, p9418_add, critical_log, 0xFF);
	if (rc) {
		 chg_err("Couldn't write 0x%02x rc = %d\n", p9418_add, rc);
	}

	return len;
}

static const struct file_operations p9418_data_log_proc_fops = {
	.write = p9418_data_log_write,
};

static void init_p9418_data_log(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("p9418_data_log", 0664, NULL, &p9418_data_log_proc_fops);
	if (!p)
		pr_err("proc_create init_p9418_data_log_proc_fops fail!\n");

}
#endif /*DEBUG_BY_FILE_OPS*/

static void p9418_update_work_process(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_p9418_ic *chip = container_of(dwork, struct oplus_p9418_ic, p9418_update_work);
	int rc;
	int boot_mode = get_boot_mode();

	if (boot_mode == MSM_BOOT_MODE__FACTORY) {
		chg_err("<IDT UPDATE> MSM_BOOT_MODE__FACTORY do not update\n");
		return;
	}
	chg_err("<IDT UPDATE> p9418_update_work_process\n");
	rc = p9418_check_idt_fw_update(chip);
	if (rc == -1) {
		/* run again after interval */
		schedule_delayed_work(&chip->p9418_update_work, P9418_UPDATE_RETRY_INTERVAL);
	}
}

static ssize_t ble_mac_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_p9418_ic *chip = NULL;

	chip = (struct oplus_p9418_ic *)dev_get_drvdata(dev);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->present) {
		if (chip->ble_mac_addr) {
			return sprintf(buf, "0x%lx_(Time:%dms)", chip->ble_mac_addr, chip->upto_ble_time);
		} else {
			return sprintf(buf, "%s", "wait_to_get_addr.");
		}
	} else {
		return sprintf(buf, "%s", "wait_to_connect.");
	}
}
static DEVICE_ATTR_RO(ble_mac_addr);

static ssize_t present_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_p9418_ic *chip = NULL;

	chip = (struct oplus_p9418_ic *)dev_get_drvdata(dev);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d", chip->present);
}
static DEVICE_ATTR_RO(present);

static ssize_t fw_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_p9418_ic *chip = NULL;
	int rc;
	char temp[4] = {0, 0, 0, 0};

	chip = (struct oplus_p9418_ic *)dev_get_drvdata(dev);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	rc = p9418_read_reg(chip, 0x001C, temp, 4);
	if (rc) {
		chg_err("p9418 Couldn't read 0x%04x after update rc = %x\n", 0x001C, rc);
	} else {
		chg_err("fw_version_show: %02x %02x %02x %02x\n", temp[0], temp[1], temp[2], temp[3]);
	}

	chip->idt_fw_version = (temp[0] << 24) | (temp[1] << 16) | (temp[2] << 8) | temp[3];

	return sprintf(buf, "0x%x", chip->idt_fw_version);
}
static DEVICE_ATTR_RO(fw_version);

static ssize_t tx_voltage_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_p9418_ic *chip = NULL;

	chip = (struct oplus_p9418_ic *)dev_get_drvdata(dev);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d", chip->tx_voltage);
}
static DEVICE_ATTR_RO(tx_voltage);

static ssize_t tx_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_p9418_ic *chip = NULL;

	chip = (struct oplus_p9418_ic *)dev_get_drvdata(dev);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d", chip->tx_current);
}
static DEVICE_ATTR_RO(tx_current);

static ssize_t rx_soc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_p9418_ic *chip = NULL;

	chip = (struct oplus_p9418_ic *)dev_get_drvdata(dev);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d", chip->rx_soc);
}

static ssize_t rx_soc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_p9418_ic *chip = NULL;
	int val = 0;

	chip = (struct oplus_p9418_ic *)dev_get_drvdata(dev);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}
	chg_err("set rx_soc raw val=%d\n", val);
	WRITE_ONCE(chip->rx_soc, val);

	return count;
}

static DEVICE_ATTR_RW(rx_soc);

static struct device_attribute *pencil_attributes[] = {
	&dev_attr_ble_mac_addr,
	&dev_attr_present,
	&dev_attr_fw_version,
	&dev_attr_tx_current,
	&dev_attr_tx_voltage,
	&dev_attr_rx_soc,
	NULL
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
static enum power_supply_property p9418_wireless_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_BLE_MAC_ADDR,
};

static int p9418_wireless_get_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		break;
	case POWER_SUPPLY_PROP_BLE_MAC_ADDR:
		break;

	default:
		return -EINVAL;
	}
	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return 0;
}

static int p9418_wireless_set_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		break;
	case POWER_SUPPLY_PROP_BLE_MAC_ADDR:
		break;

	default:
		chg_err("set prop %d is not supported\n", psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int p9418_wireless_prop_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_BLE_MAC_ADDR:
		rc = 1;
		break;

	default:
		rc = 0;
		break;
	}

	return rc;
}


static const struct power_supply_desc wireless_psy_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = p9418_wireless_props,
	.num_properties = ARRAY_SIZE(p9418_wireless_props),
	.get_property = p9418_wireless_get_prop,
	.set_property = p9418_wireless_set_prop,
	.property_is_writeable = p9418_wireless_prop_is_writeable,
};

static int p9418_init_wireless_psy(struct oplus_p9418_ic *chip)
{
	struct power_supply_config wireless_cfg = {};

	wireless_cfg.drv_data = chip;
	wireless_cfg.of_node = chip->dev->of_node;
	chip->wireless_psy = devm_power_supply_register(
		chip->dev, &wireless_psy_desc, &wireless_cfg);
	if (IS_ERR(chip->wireless_psy)) {
		chg_err("Couldn't register wireless power supply\n");
		return PTR_ERR(chip->wireless_psy);
	}

	return 0;
}
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)*/

static int init_wireless_device(struct oplus_p9418_ic *chip)
{
	int err = 0, status = 0;
	dev_t devt;
	struct class *wireless_class = NULL;
	struct device_attribute **attrs, *attr;

	wireless_class = class_create(THIS_MODULE, "oplus_wireless");

	status = alloc_chrdev_region(&devt, 0, 1, "tx_wireless");
	chip->wireless_dev = device_create(wireless_class, NULL, devt, NULL, "%s", "pencil");
	chip->wireless_dev->devt = devt;
	dev_set_drvdata(chip->wireless_dev, chip);

	attrs = pencil_attributes;
	while ((attr = *attrs++)) {
		err = device_create_file(chip->wireless_dev, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			return err;
		}
	}

	return 0;
}

bool p9418_check_chip_is_null(void)
{
	if (p9418_chip)
		return false;
	return true;
}

static int p9418_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct oplus_p9418_ic	*chip;
	int rc = 0;

	chg_err( " call \n");
	chip = devm_kzalloc(&client->dev,
		sizeof(struct oplus_p9418_ic), GFP_KERNEL);
	if (!chip) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->cep_count_flag = 0;
	chip->i2c_ready = true;
	i2c_set_clientdata(client, chip);
	p9418_idt_gpio_init(chip);
	oplus_wpc_chg_parse_chg_dt(chip);
	chip->bus_wakelock = wakeup_source_register(NULL, "p9418_wireless_wakelock");

#ifdef DEBUG_BY_FILE_OPS
	init_p9418_add_log();
	init_p9418_data_log();
#endif

	INIT_DELAYED_WORK(&chip->idt_event_int_work, p9418_idt_event_int_func);
	INIT_DELAYED_WORK(&chip->p9418_update_work, p9418_update_work_process);
	INIT_DELAYED_WORK(&idt_timer_work, p9418_timer_inhall_function);
	p9418_chip = chip;
	mutex_init(&chip->flow_mutex);

	schedule_delayed_work(&chip->p9418_update_work, P9418_UPDATE_INTERVAL);
	rc = blocking_notifier_chain_register(&hall_notifier, &p9418_notifier);
	if (rc < 0) {
		chg_err("blocking_notifier_chain_register error");
	}

	rc = init_wireless_device(chip);
	if (rc < 0) {
		chg_err("Create wireless charge device error.");
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	p9418_init_wireless_psy(chip);
#endif
	chg_err( " call end\n");

	return 0;
}


static struct i2c_driver p9418_i2c_driver;

static int p9418_driver_remove(struct i2c_client *client)
{
	return 0;
}


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int p9418_pm_resume(struct device *dev)
{
	struct oplus_p9418_ic *chip = dev_get_drvdata(dev);

	if (chip) {
		chip->i2c_ready = true;
		chg_err("p9418_pm_resume.\n");
		wake_up_interruptible(&i2c_waiter);
	}

	return 0;
}

static int p9418_pm_suspend(struct device *dev)
{
	struct oplus_p9418_ic *chip = dev_get_drvdata(dev);

	if (chip) {
		chip->i2c_ready = false;
		chg_err("p9418_pm_suspend.\n");
	}

	return 0;
}

static const struct dev_pm_ops p9418_pm_ops = {
	.resume		= p9418_pm_resume,
	.suspend		= p9418_pm_suspend,
};
#else
static int p9418_resume(struct i2c_client *client)
{
	return 0;
}

static int p9418_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
#endif

static void p9418_reset(struct i2c_client *client)
{
	p9418_power_enable(p9418_chip, true);
	check_int_enable(p9418_chip);

	return;
}

/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/

static const struct of_device_id p9418_match[] = {
	{ .compatible = "oplus,p9418-charger"},
	{ },
};

static const struct i2c_device_id p9418_id[] = {
	{"p9418-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, p9418_id);


static struct i2c_driver p9418_i2c_driver = {
	.driver		= {
		.name = "p9418-charger",
		.owner	= THIS_MODULE,
		.of_match_table = p9418_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
					.pm 	= &p9418_pm_ops,
#endif

	},
	.probe		= p9418_driver_probe,
	.remove		= p9418_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= p9418_resume,
	.suspend	= p9418_suspend,
#endif
	.shutdown	= p9418_reset,
	.id_table	= p9418_id,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
module_i2c_driver(p9418_i2c_driver);
#else
int p9418_driver_init(void)
{
	int ret = 0;

	chg_err(" start\n");

	if (i2c_add_driver(&p9418_i2c_driver) != 0) {
		chg_err(" failed to register p9418 i2c driver.\n");
	} else {
		chg_err( " Success to register p9418 i2c driver.\n");
	}

	return ret;
}

void p9418_driver_exit(void)
{
	i2c_del_driver(&p9418_i2c_driver);
	blocking_notifier_chain_unregister(&hall_notifier, &p9418_notifier);
}
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)*/
MODULE_DESCRIPTION("Driver for p9418 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:p9418-charger");

