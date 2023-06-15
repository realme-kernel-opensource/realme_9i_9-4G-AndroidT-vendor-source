/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Op. All rights reserved.
 */
#ifndef __OPLUS_P9418_H__
#define __OPLUS_P9418_H__

#include "../oplus_wireless.h"

#define P9418_REG_INT_CLR									0x28
#define P9418_REG_INT_EN									0x34
#define P9418_REG_INT_FLAG									0x30
#define P9418_INT_FLAG_EPT									BIT(0)
#define P9418_INT_FLAG_STR_DPING							BIT(1)
#define P9418_INT_FLAG_SS									BIT(2)
#define P9418_INT_FLAG_ID									BIT(3)
#define P9418_INT_FLAG_CFG									BIT(4)
#define P9418_INT_FLAG_PPP									BIT(5)
#define P9418_INT_FLAG_DPING								BIT(6)
#define P9418_INT_FLAG_TX_INT								BIT(7)
#define P9418_INT_FLAG_BLE_ADDR								BIT(0)
#define P9418_INT_FLAG_PRIVATE_PKG							BIT(1)

#define P9418_REG_RTX_CMD									0x76
#define P9418_REG_RTX_STATUS								0x78
#define P9418_RTX_DIGITALPING								BIT(0)
#define P9418_RTX_READY										BIT(1)
#define P9418_RTX_TRANSFER									BIT(3)

#define P9418_REG_RTX_ERR_STATUS							0x74
#define P9418_RTX_ERR_TX_EPT_CMD							BIT(0)
#define P9418_RTX_ERR_TX_IDAU_FAIL							BIT(3)
#define P9418_RTX_ERR_TX_CEP_TIMEOUT						BIT(0)
#define P9418_RTX_ERR_TX_OCP								BIT(2)
#define P9418_RTX_ERR_TX_OVP								BIT(3)
#define P9418_RTX_ERR_TX_LVP								BIT(4)
#define P9418_RTX_ERR_TX_FOD								BIT(5)
#define P9418_RTX_ERR_TX_OTP								BIT(6)
#define P9418_RTX_ERR_TX_POCP								BIT(7)

#define P9418_BLE_MAC_ADDR0									0xbe
#define P9418_BLE_MAC_ADDR1									0xbf
#define P9418_BLE_MAC_ADDR2									0xc0
#define P9418_BLE_MAC_ADDR3									0xc1
#define P9418_BLE_MAC_ADDR4									0xc2
#define P9418_BLE_MAC_ADDR5									0xc3

#define P9418_REG_OCP_THRESHOLD								0x6A
#define P9418_REG_OVP_THRESHOLD								0x6C
#define P9418_REG_LVP_THRESHOLD								0x60
#define P9418_REG_POCP_THRESHOLD1							0x0A
#define P9418_REG_POCP_THRESHOLD2							0x0C
#define P9418_REG_FOD_THRESHOLD								0x92

#define P9418_PRIVATE_DATA_REG								0x14
#define P9418_REG_CURRENT_MA	 							0X6E /* ma 16bit */
#define P9418_REG_VOLTAGE_MV								0X70 /* mv 16bit */

#define P9418_UPDATE_INTERVAL							round_jiffies_relative(msecs_to_jiffies(3000))
#define P9418_UPDATE_RETRY_INTERVAL						round_jiffies_relative(msecs_to_jiffies(3000))

//extern struct oplus_chg_chip *p9418_chip;
struct oplus_p9418_ic{
	struct i2c_client				 *client;
	struct device					 *dev;
	struct device					 *wireless_dev;

	struct power_supply *wireless_psy;
	enum power_supply_type wireless_type;
	enum wireless_mode wireless_mode;
	bool present;
	bool i2c_ready;
	bool is_power_on;
	uint8_t pen_status;
	/* P9418 threshold parameter */
	int							ocp_threshold;
	int							ovp_threshold;
	int							lvp_threshold;
	int							pcop_threshold1;
	int							pcop_threshold2;
	int							fod_threshold;
	int							tx_current;
	int							tx_voltage;
	int							rx_soc;
	/* P9418 status parameter */
	char						int_flag_data[4];
	unsigned int				idt_fw_version;
	int							cep_count_flag;
	unsigned long int			ble_mac_addr;
	unsigned long int			private_pkg_data;

	int							idt_int_gpio;		//for WPC
	int							idt_int_irq;		//for WPC
	int							vbat_en_gpio;		//for WPC
	int							booster_en_gpio;	//for WPC
	int							wrx_en_gpio;		//for WPC

    struct pinctrl				*pinctrl;
	struct pinctrl_state 		*idt_int_active;	//for WPC
	struct pinctrl_state 		*idt_int_sleep;		//for WPC
	struct pinctrl_state 		*idt_int_default;	//for WPC
	struct pinctrl_state 		*vbat_en_active;	//for WPC
	struct pinctrl_state 		*vbat_en_sleep;		//for WPC
	struct pinctrl_state 		*vbat_en_default;	//for WPC
	struct pinctrl_state 		*booster_en_active;	//for WPC
	struct pinctrl_state 		*booster_en_sleep;		//for WPC
	struct pinctrl_state 		*booster_en_default;	//for WPC

	struct delayed_work			p9418_update_work;  //for WPC
	struct delayed_work			idt_event_int_work; //for WPC
	struct mutex				flow_mutex;
	struct wakeup_source		*bus_wakelock;
	u64			tx_start_time;
	u64			upto_ble_time;
};

enum PEN_STATUS {
	PEN_STATUS_UNDEFINED,
	PEN_STATUS_NEAR,
	PEN_STATUS_FAR,
};

void p9418_wpc_print_log(void);
void p9418_set_vbat_en_val(int value);
int p9418_get_vbat_en_val(void);
void p9418_set_booster_en_val(int value);
int p9418_get_booster_en_val(void);
bool p9418_firmware_is_updating(void);
bool p9418_check_chip_is_null(void);
void p9418_ept_type_detect_func(void);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
int p9418_driver_init(void);
void p9418_driver_exit(void);
#endif
#endif	//__OPLUS_P9418_H__

