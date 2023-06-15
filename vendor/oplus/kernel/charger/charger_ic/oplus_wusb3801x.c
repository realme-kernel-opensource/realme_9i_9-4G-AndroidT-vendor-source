/************************************************************************
 *
 *  WILLSEMI TypeC Chipset Driver for Linux & Android.
 *
 *
 * ######################################################################
 *
 *  Author: lei.huang (lhuang@sh-willsemi.com)
 *
 * Copyright (c) 2021, WillSemi Inc. All rights reserved.
 *
 ************************************************************************/

/************************************************************************
 *  Include files
 ************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/pm_wakeup.h>


/***************************************************************************************************************************
* Example DTS for Qcom platform, User shall change the interrupt type, number, gpio ... according to their speific board design.
****************************************************************************************************************************
        i2c@78b6000 {
                wusb3801@60 {
                        compatible = "qcom,wusb3801";
                        reg = <0xc0>;
                        qcom,irq-gpio = <&msm_gpio 21 0x8008>;
                        interrupt-parent = <&msm_gpio>;
                        interrupts = <21 0>;
                        interrupt-names = "wusb3801_int_irq";
                        wusb3801,irq-gpio = <&msm_gpio 21 0x8008>;
                        wusb3801,reset-gpio = <&msm_gpio 12 0x0>;
                        wusb3801,init-mode = <0x24>;
                        wusb3801,host-current = <0x01>;
                        wusb3801,drp-toggle-time = <40>;
                };
        };
****************************************************************************************************************************
*/

#define IS_ERR_VALUE_64(x) IS_ERR_VALUE((unsigned long)x)


/**
 * Options to enable force detection feature for DRP
 */

/*#define __WITH_POWER_BANK_MODE__*/

/*
 *Bit operations if we don't want to include #include <linux/bitops.h>
 */

#undef  __CONST_FFS
#define __CONST_FFS(_x) \
        ((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) :\
                                      ((_x) & 0x04 ? 2 : 3)) :\
                       ((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) :\
                                      ((_x) & 0x40 ? 6 : 7)))

#undef  FFS
#define FFS(_x) \
        ((_x) ? __CONST_FFS(_x) : 0)

#undef  BITS
#define BITS(_end, _start) \
        ((BIT(_end) - BIT(_start)) + BIT(_end))

#undef  __BITS_GET
#define __BITS_GET(_byte, _mask, _shift) \
        (((_byte) & (_mask)) >> (_shift))

#undef  BITS_GET
#define BITS_GET(_byte, _bit) \
        __BITS_GET(_byte, _bit, FFS(_bit))

#undef  __BITS_SET
#define __BITS_SET(_byte, _mask, _shift, _val) \
        (((_byte) & ~(_mask)) | (((_val) << (_shift)) & (_mask)))

#undef  BITS_SET
#define BITS_SET(_byte, _bit, _val) \
        __BITS_SET(_byte, _bit, FFS(_bit), _val)

#undef  BITS_MATCH
#define BITS_MATCH(_byte, _bit) \
        (((_byte) & (_bit)) == (_bit))

/* Register Map */

#define WUSB3801_REG_VERSION_ID         0x01
#define WUSB3801_REG_CONTROL0           0x02
#define WUSB3801_REG_INTERRUPT          0x03
#define WUSB3801_REG_STATUS             0x04
#define WUSB3801_REG_CONTROL1           0x05
#define WUSB3801_REG_TEST0              0x06
#define WUSB3801_REG_TEST_01            0x07
#define WUSB3801_REG_TEST_02            0x08
#define WUSB3801_REG_TEST_03            0x09
#define WUSB3801_REG_TEST_04            0x0A
#define WUSB3801_REG_TEST_05            0x0B
#define WUSB3801_REG_TEST_06            0x0C
#define WUSB3801_REG_TEST_07            0x0D
#define WUSB3801_REG_TEST_08            0x0E
#define WUSB3801_REG_TEST_09            0x0F
#define WUSB3801_REG_TEST_0A            0x10
#define WUSB3801_REG_TEST_0B            0x11
#define WUSB3801_REG_TEST_0C            0x12
#define WUSB3801_REG_TEST_0D            0x13
#define WUSB3801_REG_TEST_0E            0x14
#define WUSB3801_REG_TEST_0F            0x15
#define WUSB3801_REG_TEST_10            0x16
#define WUSB3801_REG_TEST_11            0x17
#define WUSB3801_REG_TEST_12            0x18


#define WUSB3801_SLAVE_ADDR0            0xc0
#define WUSB3801_SLAVE_ADDR1            0xd0


/*Available modes*/
#define WUSB3801_DRP_ACC                (BIT_REG_CTRL0_RLE_DRP)
#define WUSB3801_DRP                    (BIT_REG_CTRL0_RLE_DRP | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_SNK_ACC                (BIT_REG_CTRL0_RLE_SNK)
#define WUSB3801_SNK                    (BIT_REG_CTRL0_RLE_SNK | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_SRC_ACC                (BIT_REG_CTRL0_RLE_SRC)
#define WUSB3801_SRC                    (BIT_REG_CTRL0_RLE_SRC | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_DRP_PREFER_SRC_ACC     (WUSB3801_DRP_ACC | BIT_REG_CTRL0_TRY_SRC)
#define WUSB3801_DRP_PREFER_SRC         (WUSB3801_DRP     | BIT_REG_CTRL0_TRY_SRC)
#define WUSB3801_DRP_PREFER_SNK_ACC     (WUSB3801_DRP_ACC | BIT_REG_CTRL0_TRY_SNK)
#define WUSB3801_DRP_PREFER_SNK         (WUSB3801_DRP     | BIT_REG_CTRL0_TRY_SNK)


/*TODO: redefine your prefer role here*/
#define WUSB3801_INIT_MODE              (WUSB3801_DRP_PREFER_SNK_ACC)

/*Registers relevant values*/
#define WUSB3801_VENDOR_ID              0x06
#define WUSB3801_DEVICE_ID              0x16

/*Switch to enable/disable feature of specified Registers*/
#define BIT_REG_CTRL0_DIS_ACC           (0x01 << 7)
#define BIT_REG_CTRL0_TRY_SRC           (0x02 << 5)
#define BIT_REG_CTRL0_TRY_SNK           (0x01 << 5)
#define BIT_REG_CTRL0_CUR_DEF           (0x00 << 3)
#define BIT_REG_CTRL0_CUR_1P5           (0x01 << 3)
#define BIT_REG_CTRL0_CUR_3P0           (0x02 << 3)
#define BIT_REG_CTRL0_RLE_SNK           (0x00 << 1)
#define BIT_REG_CTRL0_RLE_SRC           (0x01 << 1)
#define BIT_REG_CTRL0_RLE_DRP           (0x02 << 1)
#define BIT_REG_CTRL0_INT_MSK           (0x01 << 0)

#define BIT_REG_STATUS_VBUS             (0x01 << 7)
#define BIT_REG_STATUS_STANDBY          (0x00 << 5)
#define BIT_REG_STATUS_CUR_DEF          (0x01 << 5)
#define BIT_REG_STATUS_CUR_MID          (0x02 << 5)
#define BIT_REG_STATUS_CUR_HIGH         (0x03 << 5)

#define BIT_REG_STATUS_ATC_STB          (0x00 << 1)
#define BIT_REG_STATUS_ATC_SNK          (0x01 << 1)
#define BIT_REG_STATUS_ATC_SRC          (0x02 << 1)
#define BIT_REG_STATUS_ATC_ACC          (0x03 << 1)
#define BIT_REG_STATUS_ATC_DACC         (0x04 << 1)

#define BIT_REG_STATUS_PLR_STB          (0x00 << 0)
#define BIT_REG_STATUS_PLR_CC1          (0x01 << 0)
#define BIT_REG_STATUS_PLR_CC2          (0x02 << 0)
#define BIT_REG_STATUS_PLR_BOTH         (0x03 << 0)

#define BIT_REG_CTRL1_SW02_DIN          (0x01 << 4)
#define BIT_REG_CTRL1_SW02_EN           (0x01 << 3)
#define BIT_REG_CTRL1_SW01_DIN          (0x01 << 2)
#define BIT_REG_CTRL1_SW01_EN           (0x01 << 1)
#define BIT_REG_CTRL1_SM_RST            (0x01 << 0)



#define BIT_REG_TEST02_FORCE_ERR_RCY    (0x01)

#define WUSB3801_WAIT_VBUS               0x40
/*Fixed duty cycle period. 40ms:40ms*/
#define WUSB3801_TGL_40MS                0
#define WUSB3801_HOST_DEFAULT            0
#define WUSB3801_HOST_1500MA             1
#define WUSB3801_HOST_3000MA             2
#define WUSB3801_INT_ENABLE              0x00
#define WUSB3801_INT_DISABLE             0x01
#define WUSB3801_DISABLED                0x0A
#define WUSB3801_ERR_REC                 0x01
#define WUSB3801_VBUS_OK                 0x80

#define WUSB3801_SNK_0MA                (0x00 << 5)
#define WUSB3801_SNK_DEFAULT            (0x01 << 5)
#define WUSB3801_SNK_1500MA             (0x02 << 5)
#define WUSB3801_SNK_3000MA             (0x03 << 5)
#define WUSB3801_ATTACH                  0x1C

//#define WUSB3801_TYPE_PWR_ACC           (0x00 << 2) /*Ra/Rd treated as Open*/
#define WUSB3801_TYPE_INVALID           (0x00)
#define WUSB3801_TYPE_SNK               (0x01 << 2)
#define WUSB3801_TYPE_SRC               (0x02 << 2)
#define WUSB3801_TYPE_AUD_ACC           (0x03 << 2)
#define WUSB3801_TYPE_DBG_ACC           (0x04 << 2)

#define WUSB3801_INT_DETACH              (0x01 << 1)
#define WUSB3801_INT_ATTACH              (0x01 << 0)

#define WUSB3801_REV20                   0x02

/* Masks for Read-Modified-Write operations*/
#define WUSB3801_HOST_CUR_MASK           0x18  /*Host current for IIC*/
#define WUSB3801_INT_MASK                0x01
#define WUSB3801_BCLVL_MASK              0x60
#define WUSB3801_TYPE_MASK               0x1C
#define WUSB3801_MODE_MASK               0xE6  /*Roles relevant bits*/
#define WUSB3801_INT_STS_MASK            0x03
#define WUSB3801_FORCE_ERR_RCY_MASK      0x80  /*Force Error recovery*/
#define WUSB3801_ROLE_MASK               0x06
#define WUSB3801_VENDOR_ID_MASK          0x07
#define WUSB3801_VERSION_ID_MASK         0xF8
#define WUSB3801_VENDOR_SUB_ID_MASK         0xA0
#define WUSB3801_POLARITY_CC_MASK        0x03
#define WUSB3801_CC_STS_MASK            0x03


/* WUSB3801 STATES MACHINES */
#define WUSB3801_STATE_DISABLED             0x00
#define WUSB3801_STATE_ERROR_RECOVERY       0x01
#define WUSB3801_STATE_UNATTACHED_SNK       0x02
#define WUSB3801_STATE_UNATTACHED_SRC       0x03
#define WUSB3801_STATE_ATTACHWAIT_SNK       0x04
#define WUSB3801_STATE_ATTACHWAIT_SRC       0x05
#define WUSB3801_STATE_ATTACHED_SNK         0x06
#define WUSB3801_STATE_ATTACHED_SRC         0x07
#define WUSB3801_STATE_AUDIO_ACCESSORY      0x08
#define WUSB3801_STATE_DEBUG_ACCESSORY      0x09
#define WUSB3801_STATE_TRY_SNK              0x0A
#define WUSB3801_STATE_TRYWAIT_SRC          0x0B
#define WUSB3801_STATE_TRY_SRC              0x0C
#define WUSB3801_STATE_TRYWAIT_SNK          0x0D

#define WUSB3801_CC2_CONNECTED 1
#define WUSB3801_CC1_CONNECTED 0

#define FRIST_CHECK_FLAG       0
#define RESET_FRIST_CHECK_FLAG 1


/* wake lock timeout in ms */
#define WUSB3801_WAKE_LOCK_TIMEOUT          1000
/*1.5 Seconds timeout for force detection*/
#define ROLE_SWITCH_TIMEOUT		              1500
#define DEBOUNCE_TIME_OUT 	50
#define DEBOUNCE_100        100
#define DEBOUNCE_3000       3000


typedef struct wusb3801_data
{
	uint8_t  init_mode;
	uint8_t  dfp_power;
	uint8_t  dttime;
}wusb3801_data_t;

typedef struct __WUSB3801_DRV_TIMER
{
	struct timer_list tl;
	void (*timer_function) (void *context);
	void *function_context;
	uint32_t time_period;
	uint8_t timer_is_canceled;
} WUSB3801_DRV_TIMER, *PWUSB3801_DRV_TIMER;

typedef struct
{
	struct task_struct *task;
	struct completion comp;
	pid_t pid;
	void *chip;
} wusb3801_thread;
/*Working context structure*/
typedef struct wusb3801_chip
{
	struct      i2c_client *client;
	struct      wusb3801_data *pdata;
	struct      workqueue_struct  *cc_wq;
	int         irq_gpio;
	int         ufp_power;
	uint8_t     cc_test_flag;
	uint8_t     cc_sts;
	uint8_t     mode;
	uint8_t     dev_id;
	uint8_t     dev_sub_id;
	uint8_t     type;
	uint8_t     state;
	uint8_t     bc_lvl;
	uint8_t     dfp_power;
	uint8_t     dttime;
	uint8_t     attached;
	uint8_t     init_state;
	uint8_t     defer_init;	/*work round the additional interrupt caused by mode change*/
	int         try_attcnt;
	struct      work_struct dwork;
	struct		wakeup_source    *wusb3801_ws;
	struct      mutex mlock;
	struct      power_supply *usb_psy;
	struct      dual_role_phy_instance *dual_role;
	struct      dual_role_phy_desc *desc;
	struct pinctrl *wusb3801_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	uint8_t chip_remove;
	struct delayed_work	first_check_typec_work;
	wusb3801_thread thread;
	int irq;
}wusb3801_chip_t;


#define wusb3801_update_state(chip, st)                      \
	if(chip) {                                                 \
		chip->state = st;                                        \
		dev_err(&chip->client->dev, "%s: %s\n", __func__, #st); \
		wake_up_interruptible(&mode_switch);                     \
	}



#define STR(s)    #s
#define STRV(s)   STR(s)

static int wusb3801_reset_device(struct wusb3801_chip *chip);
static void wusb3801_detach(struct wusb3801_chip *chip);

extern void oplus_typec_sink_removal(void);
extern void oplus_typec_src_removal(void);
extern void oplus_typec_src_insertion(void);
extern void oplus_typec_mode_unattached(void);
extern void oplus_typec_sink_insertion(void);
extern void oplus_typec_ra_ra_insertion(void);
extern bool oplus_get_otg_switch_status(void);
extern int chg_init_done;
extern int typec_dir;

struct wusb3801_chip *g_wusb_chip = NULL;

//static unsigned wusb3801_is_vbus_on(struct wusb3801_chip *chip);
DECLARE_WAIT_QUEUE_HEAD(mode_switch);


/************************************************************************
 *
 *       wusb3801_write_masked_byte
 *
 *  Description :
 *  -------------
 *  Read-Modified-Writeback operation through I2C communication.
 *
 *  Parameter     :
 *  -----------
 *  Client         :    Pointer to I2C client.
 *  Addr           :    Address of internal register of slave device
 *  Mask           :    Mask to prevent irrelevant bit changes
 *  val            :    Data wants to write.
 *  Return values  :    Zero if no error, else error code.
 *  ---------------
 *  None
 *
 ************************************************************************/

static int wusb3801_write_masked_byte(struct i2c_client *client,
					uint8_t addr, uint8_t mask, uint8_t val)
{
	int rc;
	if (!mask){
		/* no actual access */
		rc = -EINVAL;
		goto out;
	}
	rc = i2c_smbus_read_byte_data(client, addr);
	if (!IS_ERR_VALUE_64(rc)){
		rc = i2c_smbus_write_byte_data(client,
			addr, BITS_SET((uint8_t)rc, mask, val));
	}
out:
	return rc;
}

static int wusb3801_read_device_id(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_VERSION_ID);
	if (IS_ERR_VALUE_64(rc))
		return rc;
	dev_err(cdev, "VendorID register: 0x%02x\n", rc );
	if((rc & WUSB3801_VENDOR_ID_MASK) != WUSB3801_VENDOR_ID){
		return -EINVAL;
	}
	chip->dev_id = rc;
	dev_err(cdev, "Vendor id: 0x%02x, Version id: 0x%02x\n", rc & WUSB3801_VENDOR_ID_MASK,
	                                                         (rc & WUSB3801_VERSION_ID_MASK) >> 3);
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_TEST_01);
	if (IS_ERR_VALUE_64(rc))
		return rc;
	chip->dev_sub_id = rc & WUSB3801_VENDOR_SUB_ID_MASK;
	dev_err(cdev, "VendorSUBID register: 0x%02x\n", rc & WUSB3801_VENDOR_SUB_ID_MASK );
	return 0;
}

static int wusb3801_check_modes(uint8_t mode)
{
	switch(mode){
		case WUSB3801_DRP_ACC            :
		case WUSB3801_DRP                :
		case WUSB3801_SNK_ACC            :
		case WUSB3801_SNK                :
		case WUSB3801_SRC_ACC            :
		case WUSB3801_SRC                :
		case WUSB3801_DRP_PREFER_SRC_ACC :
		case WUSB3801_DRP_PREFER_SRC     :
		case WUSB3801_DRP_PREFER_SNK_ACC :
		case WUSB3801_DRP_PREFER_SNK     :
			return 0;
			break;
		default:
			break;
	}
	return -EINVAL;
}


/*
 *  Spec lets transitioning to below states from any state
 *  WUSB3801_STATE_DISABLED
 *  WUSB3801_STATE_ERROR_RECOVERY
 */
static int wusb3801_set_chip_state(struct wusb3801_chip *chip, uint8_t state)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	if(state > WUSB3801_STATE_UNATTACHED_SRC)
		return -EINVAL;

  rc = i2c_smbus_write_byte_data(chip->client,
			   WUSB3801_REG_CONTROL1,
			   (state == WUSB3801_STATE_DISABLED) ? \
			             WUSB3801_DISABLED :        \
			             0);

	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "failed to write state machine(%d)\n", rc);
	}

	chip->init_state = state;

	return rc;
}


static int wusb3801_set_mode(struct wusb3801_chip *chip, uint8_t mode)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	if (mode != chip->mode) {
		/*
		rc = wusb3801_write_masked_byte(chip->client,
				WUSB3801_REG_CONTROL0,
				WUSB3801_MODE_MASK,
				mode);
				*/
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_CONTROL0);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: fail to read mode\n", __func__);
		return rc;
	}
	rc &= ~WUSB3801_MODE_MASK;
	rc |= (mode | WUSB3801_INT_MASK);/* Disable the chip interrupt */
  rc = i2c_smbus_write_byte_data(chip->client,
			   WUSB3801_REG_CONTROL0, rc);

	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "failed to write mode(%d)\n", rc);
		return rc;
	}

	/* Clear the chip interrupt */
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: fail to clear chip interrupt\n", __func__);
		return rc;
	}

	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_CONTROL0);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: fail to read chip interrupt\n", __func__);
		return rc;
	}
	rc &= ~WUSB3801_INT_MASK;/* enable the chip interrupt */
	rc = i2c_smbus_write_byte_data(chip->client,
			   WUSB3801_REG_CONTROL0, rc);

	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "failed to enable chip interrupt(%d)\n", rc);
		return rc;
	}

	chip->mode = mode;
	dev_err(cdev, " mode (0x%02x) (0x%02x)\n", chip->mode , mode);
  }

	return rc;
}

bool is_wusb3801x(void)
{
	struct wusb3801_chip *chip = g_wusb_chip;
	if(chip && chip->dev_id) {
		if(chip->dev_id == WUSB3801_DEVICE_ID)
		printk("is_wusb3801\n" );
		return true;
	}
	printk("not is_wusb3801x\n" );
	return false;
}


int oplus_wusb3801x_set_mode(int mode)
{
	struct wusb3801_chip *chip = g_wusb_chip;
	int rc = 0;

	if(chip) {
		if(mode == 3) {
			rc = wusb3801_set_mode(chip, WUSB3801_DRP);
		} else {
			rc = wusb3801_set_mode(chip, WUSB3801_SNK);
		}
	}

	return rc;
}

static int wusb3801_set_dfp_power(struct wusb3801_chip *chip, uint8_t hcurrent)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	if (hcurrent == chip->dfp_power) {
		dev_dbg(cdev, "vaule is not updated(%d)\n",
					hcurrent);
		return rc;
	}

	rc = wusb3801_write_masked_byte(chip->client,
					WUSB3801_REG_CONTROL0,
					WUSB3801_HOST_CUR_MASK,
					hcurrent);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "failed to write current(%d)\n", rc);
		return rc;
	}

	chip->dfp_power = hcurrent;

	dev_dbg(cdev, "%s: host current(%d)\n", __func__, hcurrent);

	return rc;
}

/**********************************************************************
 * When 3A capable DRP device is connected without VBUS,
 * DRP always detect it as SINK device erroneously.
 * Since USB Type-C specification 1.0 and 1.1 doesn't
 * consider this corner case, apply workaround for this case.
 * Set host mode current to 1.5A initially, and then change
 * it to default USB current right after detection SINK port.
 ***********************************************************************/
static int wusb3801_init_force_dfp_power(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	rc = wusb3801_write_masked_byte(chip->client,
					WUSB3801_REG_CONTROL0,
					WUSB3801_HOST_CUR_MASK,
					WUSB3801_HOST_3000MA);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "failed to write current\n");
		return rc;
	}

	chip->dfp_power = WUSB3801_HOST_3000MA;

	dev_dbg(cdev, "%s: host current (%d)\n", __func__, rc);

	return rc;
}

/************************************************************************
 *
 *       wusb3801_set_toggle_time
 *
 *  Description :
 *  -------------
 *  Wusb3801 varints only support for fixed duty cycles periods (40ms:40ms)
 *
 ************************************************************************/
static int wusb3801_set_toggle_time(struct wusb3801_chip *chip, uint8_t toggle_time)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	if (toggle_time != WUSB3801_TGL_40MS) {
		dev_err(cdev, "toggle_time(%d) is unavailable\n", toggle_time);
		return -EINVAL;
	}

	chip->dttime = WUSB3801_TGL_40MS;

	dev_dbg(cdev, "%s: Fixed toggle time (%d)\n", __func__, chip->dttime);

	return rc;
}


static int wusb3801_init_reg(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	/* change current */
	rc = wusb3801_init_force_dfp_power(chip);
	if (IS_ERR_VALUE_64(rc))
		dev_err(cdev, "%s: failed to force dfp power\n",
				__func__);


	/* change mode */
	rc = wusb3801_set_mode(chip, chip->pdata->init_mode);
	if (IS_ERR_VALUE_64(rc))
		dev_err(cdev, "%s: failed to set mode\n",
				__func__);

  rc = wusb3801_set_chip_state(chip,
				WUSB3801_STATE_ERROR_RECOVERY);

	if (IS_ERR_VALUE_64(rc))
		dev_err(cdev, "%s: Reset state failed.\n",
				__func__);

	return rc;
}

static int wusb3801_reset_device(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
/*
	rc = i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_CONTROL1, BIT_REG_CTRL1_SM_RST);

	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "reset fails\n");
		return rc;
	}
	*/

	rc = wusb3801_init_reg(chip);
	if (IS_ERR_VALUE_64(rc))
		dev_err(cdev, "fail to init reg\n");

	schedule_delayed_work(
						&chip->first_check_typec_work, msecs_to_jiffies(DEBOUNCE_3000));

	return rc;
}

/************************************************************************
 *
 *       fregdump_show
 *
 *  Description :
 *  -------------
 *  Dump registers to user space. there is side-effects for Read/Clear
 *  registers. For example interrupt status.
 *
 ************************************************************************/
static ssize_t fregdump_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int i, rc, ret = 0;

	mutex_lock(&chip->mlock);
	for (i = WUSB3801_REG_VERSION_ID ; i <= WUSB3801_REG_TEST_12; i++) {
		rc = i2c_smbus_read_byte_data(chip->client, (uint8_t)i);
		if (IS_ERR_VALUE_64(rc)) {
			pr_err("cannot read 0x%02x\n", i);
			rc = 0;
		}
		ret += snprintf(buf + ret, 1024 - ret, "from 0x%02x read 0x%02x\n", (uint8_t)i, rc);
	}
	mutex_unlock(&chip->mlock);

	return ret;
}

DEVICE_ATTR(fregdump, S_IRUGO, fregdump_show, NULL);

#ifdef __TEST_CC_PATCH__
/************************************************************************
 *
 *       fcc_status_show
 *
 *  Description :
 *  -------------
 *  show cc_status
 *
 ************************************************************************/
static ssize_t fcc_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int rc, ret = 0;

	mutex_lock(&chip->mlock);
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_STATUS);
	mutex_unlock(&chip->mlock);

	if (IS_ERR_VALUE_64(rc)) {
		pr_err("cannot read WUSB3801_REG_STATUS\n");
		rc = 0xFF;
		ret = snprintf(buf, PAGE_SIZE, "cc_sts (%d)\n", rc);
	}
	rc  =  BITS_GET(rc, WUSB3801_CC_STS_MASK);

	if(rc == 0 && chip->cc_sts != 0xFF)
		rc = chip->cc_sts;
	else
		rc -= 1;
	ret = snprintf(buf, PAGE_SIZE, "cc_sts (%d)\n", rc);
	return ret;
}

DEVICE_ATTR(fcc_status, S_IRUGO, fcc_status_show, NULL);
#endif /*  __TEST_CC_PATCH__	 */

/************************************************************************
 *
 *       ftype_show
 *
 *  Description :
 *  -------------
 *  Dump types of attached devices
 *
 ************************************************************************/
static ssize_t ftype_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mlock);
	switch (chip->type) {
	case WUSB3801_TYPE_SNK:
		ret = snprintf(buf, PAGE_SIZE, "SINK(%d)\n", chip->type);
		break;
	case WUSB3801_TYPE_SRC:
		ret = snprintf(buf, PAGE_SIZE, "SOURCE(%d)\n", chip->type);
		break;
	case WUSB3801_TYPE_DBG_ACC:
		ret = snprintf(buf, PAGE_SIZE, "DEBUGACC(%d)\n", chip->type);
		break;
	case WUSB3801_TYPE_AUD_ACC:
		ret = snprintf(buf, PAGE_SIZE, "AUDIOACC(%d)\n", chip->type);
		break;
	default:
		ret = snprintf(buf, PAGE_SIZE, "NOTYPE(%d)\n", chip->type);
		break;
	}
	mutex_unlock(&chip->mlock);

	return ret;
}

DEVICE_ATTR(ftype, S_IRUGO , ftype_show, NULL);

/************************************************************************
 *
 *  fchip_state_show/fchip_state_store
 *
 *  Description :
 *  -------------
 *  Get/Set state to/from user spaces.
 *
 ************************************************************************/

static ssize_t fchip_state_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE,
			STRV(WUSB3801_STATE_DISABLED) " - WUSB3801_STATE_DISABLED\n"
			STRV(WUSB3801_STATE_ERROR_RECOVERY) " - WUSB3801_STATE_ERROR_RECOVERY\n"
			STRV(WUSB3801_STATE_UNATTACHED_SNK) " - WUSB3801_STATE_UNATTACHED_SNK\n"
			STRV(WUSB3801_STATE_UNATTACHED_SRC) " - WUSB3801_STATE_UNATTACHED_SRC\n");

}


static ssize_t fchip_state_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int state = 0;
	int rc = 0;

	if (sscanf(buff, "%d", &state) == 1) {
		mutex_lock(&chip->mlock);
	  if(((state == WUSB3801_STATE_UNATTACHED_SNK) &&
	    ((chip->mode & WUSB3801_ROLE_MASK) == BIT_REG_CTRL0_RLE_SRC)) || \
	    ((state == WUSB3801_STATE_UNATTACHED_SRC) &&
	    ((chip->mode & WUSB3801_ROLE_MASK) == BIT_REG_CTRL0_RLE_SNK))) {
			mutex_unlock(&chip->mlock);
			return -EINVAL;
		}

		rc = wusb3801_set_chip_state(chip, (uint8_t)state);
		if (IS_ERR_VALUE_64(rc)) {
			mutex_unlock(&chip->mlock);
			return rc;
		}

		wusb3801_detach(chip);
		mutex_unlock(&chip->mlock);
		return size;
	}

	return -EINVAL;
}

DEVICE_ATTR(fchip_state, S_IRUGO | S_IWUSR, fchip_state_show, fchip_state_store);

/************************************************************************
 *
 *  fmode_show/fmode_store
 *
 *  Description :
 *  -------------
 *  Dump/Set role to/from user spaces.
 ************************************************************************/
static ssize_t fmode_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mlock);
	switch (chip->mode) {
	case WUSB3801_DRP_ACC:
		ret = snprintf(buf, PAGE_SIZE, "DRP+ACC(%d)\n", chip->mode);
		break;
	case WUSB3801_DRP:
		ret = snprintf(buf, PAGE_SIZE, "DRP(%d)\n", chip->mode);
		break;
  case WUSB3801_SNK_ACC:
		ret = snprintf(buf, PAGE_SIZE, "SNK+ACC(%d)\n", chip->mode);
		break;
	case WUSB3801_SNK:
		ret = snprintf(buf, PAGE_SIZE, "SNK(%d)\n", chip->mode);
		break;
	case WUSB3801_SRC_ACC:
		ret = snprintf(buf, PAGE_SIZE, "SRC+ACC(%d)\n", chip->mode);
		break;
	case WUSB3801_SRC:
		ret = snprintf(buf, PAGE_SIZE, "SRC(%d)\n", chip->mode);
		break;
  case WUSB3801_DRP_PREFER_SRC_ACC:
  	ret = snprintf(buf, PAGE_SIZE, "DRP+ACC+PREFER_SRC(%d)\n", chip->mode);
    break;
  case WUSB3801_DRP_PREFER_SRC:
  	ret = snprintf(buf, PAGE_SIZE, "DRP+PREFER_SRC(%d)\n", chip->mode);
    break;
  case WUSB3801_DRP_PREFER_SNK_ACC:
  	ret = snprintf(buf, PAGE_SIZE, "DRP+ACC+PREFER_SNK(%d)\n", chip->mode);
    break;
  case WUSB3801_DRP_PREFER_SNK:
  	ret = snprintf(buf, PAGE_SIZE, "DRP+PREFER_SNK(%d)\n", chip->mode);
    break;
	default:
		ret = snprintf(buf, PAGE_SIZE, "UNKNOWN(%d)\n", chip->mode);
		break;
	}
	mutex_unlock(&chip->mlock);

	return ret;
}

static ssize_t fmode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int mode = 0;
	int rc = 0;

	if (sscanf(buff, "%d", &mode) == 1) {
		mutex_lock(&chip->mlock);

		/*
		 * since device trigger to usb happens independent
		 * from charger based on vbus, setting SRC modes
		 * doesn't prevent usb enumeration as device
		 * KNOWN LIMITATION
		 */
		rc = wusb3801_set_mode(chip, (uint8_t)mode);
		if (IS_ERR_VALUE_64(rc)) {
			mutex_unlock(&chip->mlock);
			return rc;
		}

		rc = wusb3801_set_chip_state(chip,
					WUSB3801_STATE_ERROR_RECOVERY);
		if (IS_ERR_VALUE_64(rc)) {
			mutex_unlock(&chip->mlock);
			return rc;
		}


		wusb3801_detach(chip);
		mutex_unlock(&chip->mlock);
		return size;
	}

	return -EINVAL;
}
DEVICE_ATTR(fmode, S_IRUGO | S_IWUSR, fmode_show, fmode_store);

/************************************************************************
 *
 *  fdttime_show/fdttime_store
 *
 *  Description :
 *  -------------
 *  Get/Set duty cycles period from/to user spaces.
 *  Noted that: Wusb3801 uses fixed duty cycles percentage.
 ************************************************************************/
static ssize_t fdttime_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%u\n", chip->dttime);
	mutex_unlock(&chip->mlock);
	return ret;
}

static ssize_t fdttime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int dttime = 0;
	int rc = 0;

	if (sscanf(buff, "%d", &dttime) == 1) {
		mutex_lock(&chip->mlock);
		rc = wusb3801_set_toggle_time(chip, (uint8_t)dttime);
		mutex_unlock(&chip->mlock);
		if (IS_ERR_VALUE_64(rc))
			return rc;

		return size;
	}

	return -EINVAL;
}
DEVICE_ATTR(fdttime, S_IRUGO | S_IWUSR, fdttime_show, fdttime_store);

/************************************************************************
 *
 *  fhostcur_show/fhostcur_store
 *
 *  Description :
 *  -------------
 *  Get/Set host current from/to user spaces.
 ************************************************************************/
static ssize_t fhostcur_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%u\n", chip->dfp_power);
	mutex_unlock(&chip->mlock);
	return ret;
}

static ssize_t fhostcur_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int buf = 0;
	int rc = 0;

	if (sscanf(buff, "%d", &buf) == 1) {
		mutex_lock(&chip->mlock);
		rc = wusb3801_set_dfp_power(chip, (uint8_t)buf);
		mutex_unlock(&chip->mlock);
		if (IS_ERR_VALUE_64(rc))
			return rc;

		return size;
	}

	return -EINVAL;
}
DEVICE_ATTR(fhostcur, S_IRUGO | S_IWUSR, fhostcur_show, fhostcur_store);

/************************************************************************
 *
 *  fclientcur_show
 *
 *  Description :
 *  -------------
 *  dumps client current to user spaces.
 ************************************************************************/
static ssize_t fclientcur_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", chip->ufp_power);
	mutex_unlock(&chip->mlock);
	return ret;
}
DEVICE_ATTR(fclientcur, S_IRUGO, fclientcur_show, NULL);

/************************************************************************
 *
 *  freset_store
 *
 *  Description :
 *  -------------
 *  Reset state machine of WUSB3801
 ************************************************************************/

static ssize_t freset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	uint32_t reset = 0;
	int rc = 0;

	if (sscanf(buff, "%u", &reset) == 1) {
		mutex_lock(&chip->mlock);
		rc = wusb3801_reset_device(chip);
		mutex_unlock(&chip->mlock);
		if (IS_ERR_VALUE_64(rc))
			return rc;

		return size;
	}

	return -EINVAL;
}
DEVICE_ATTR(freset, S_IWUSR, NULL, freset_store);



static int wusb3801_create_devices(struct device *cdev)
{
	int ret = 0;

	ret = device_create_file(cdev, &dev_attr_fchip_state);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fchip_state\n");
		ret = -ENODEV;
		goto err0;
	}

	ret = device_create_file(cdev, &dev_attr_ftype);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_ftype\n");
		ret = -ENODEV;
		goto err1;
	}

	ret = device_create_file(cdev, &dev_attr_fmode);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fmode\n");
		ret = -ENODEV;
		goto err2;
	}

	ret = device_create_file(cdev, &dev_attr_freset);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_freset\n");
		ret = -ENODEV;
		goto err3;
	}

	ret = device_create_file(cdev, &dev_attr_fdttime);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fdttime\n");
		ret = -ENODEV;
		goto err4;
	}

	ret = device_create_file(cdev, &dev_attr_fhostcur);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fhostcur\n");
		ret = -ENODEV;
		goto err5;
	}

	ret = device_create_file(cdev, &dev_attr_fclientcur);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fclientcur\n");
		ret = -ENODEV;
		goto err6;
	}

	ret = device_create_file(cdev, &dev_attr_fregdump);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fregdump\n");
		ret = -ENODEV;
		goto err7;
	}

err7:
	device_remove_file(cdev, &dev_attr_fclientcur);
err6:
	device_remove_file(cdev, &dev_attr_fhostcur);
err5:
	device_remove_file(cdev, &dev_attr_fdttime);
err4:
	device_remove_file(cdev, &dev_attr_freset);
err3:
	device_remove_file(cdev, &dev_attr_fmode);
err2:
	device_remove_file(cdev, &dev_attr_ftype);
err1:
	device_remove_file(cdev, &dev_attr_fchip_state);
err0:
	return ret;
}

static void wusb3801_destory_device(struct device *cdev)
{
	device_remove_file(cdev, &dev_attr_fchip_state);
	device_remove_file(cdev, &dev_attr_ftype);
	device_remove_file(cdev, &dev_attr_fmode);
	device_remove_file(cdev, &dev_attr_freset);
	device_remove_file(cdev, &dev_attr_fdttime);
	device_remove_file(cdev, &dev_attr_fhostcur);
	device_remove_file(cdev, &dev_attr_fclientcur);
	device_remove_file(cdev, &dev_attr_fregdump);
}
static void wusb3801_src_detected(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
   if((chip->mode & WUSB3801_ROLE_MASK) == BIT_REG_CTRL0_RLE_SRC || !chg_init_done){
		dev_err(cdev, "not support in source mode\n");
		return;
	}
   	wusb3801_update_state(chip, WUSB3801_STATE_ATTACHED_SNK);
	oplus_typec_src_insertion();
	chip->type = WUSB3801_TYPE_SRC;
}

static void wusb3801_snk_detected(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	dev_err(cdev, "%s \n",__func__);
	if((chip->mode & WUSB3801_ROLE_MASK) == BIT_REG_CTRL0_RLE_SNK || !chg_init_done){
		dev_err(cdev, "not support in sink mode\n");
		return;
	}

	/**
	*  TODO: Add you code here if you want doing something special during
	*        TYPEC Sink has detected.
	*/
		/*
		 * mode == WUSB3801_SRC/WUSB3801_SRC_ACC
		 */
		wusb3801_set_dfp_power(chip, chip->pdata->dfp_power);
		if(oplus_get_otg_switch_status()) {
			wusb3801_update_state(chip, WUSB3801_STATE_ATTACHED_SRC);
			oplus_typec_sink_insertion();
			chip->type = WUSB3801_TYPE_SNK;
		}

}

static void wusb3801_dbg_acc_detected(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	dev_err(cdev, "%s \n",__func__);
	if (chip->mode & BIT_REG_CTRL0_DIS_ACC) {
		dev_err(cdev, "not support accessory mode\n");
		return;
	}

	/**
	*  TODO: Add you code here if you want doing something special during
	*        TYPEC Debug accessory has detected.
	*/
	wusb3801_update_state(chip, WUSB3801_STATE_DEBUG_ACCESSORY);
	oplus_typec_ra_ra_insertion();
}

static void wusb3801_aud_acc_detected(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	dev_err(cdev, "%s \n",__func__);
	if (chip->mode & BIT_REG_CTRL0_DIS_ACC) {
		dev_err(cdev, "not support accessory mode\n");
		return;
	}

	/**
	*  TODO: Add you code here if you want doing something special during
	*        TYPEC Audio accessory has detected.
	*/
	wusb3801_update_state(chip, WUSB3801_STATE_AUDIO_ACCESSORY);
	oplus_typec_ra_ra_insertion();
}

static void wusb3801_aud_acc_detach(struct wusb3801_chip *chip)
{
/**
	*  TODO: Add you code here if you want doing something special during
	*        TYPEC Audio accessory pull out.
	*/
}

static void wusb3801_detach(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	if(!chg_init_done)
			return;
	dev_err(cdev, "%s: type[0x%02x] chipstate[0x%02x]\n",
			__func__, chip->type, chip->state);

	switch (chip->state) {
	case WUSB3801_STATE_ATTACHED_SRC:
		wusb3801_init_force_dfp_power(chip);
		oplus_typec_sink_removal();
		break;
	case WUSB3801_STATE_ATTACHED_SNK:
		oplus_typec_src_removal();
		break;
	case WUSB3801_STATE_DEBUG_ACCESSORY:
		oplus_typec_src_removal();
	  break;
	case WUSB3801_STATE_AUDIO_ACCESSORY:
	  wusb3801_aud_acc_detach(chip);
	  oplus_typec_src_removal();
		break;
	case WUSB3801_STATE_DISABLED:
	case WUSB3801_STATE_ERROR_RECOVERY:
		oplus_typec_mode_unattached();
		break;
	default:
		oplus_typec_mode_unattached();
		dev_err(cdev, "%s: Invaild chipstate[0x%02x]\n",
				__func__, chip->state);
		break;
	}
	chip->type = WUSB3801_TYPE_INVALID;
	chip->bc_lvl = WUSB3801_SNK_0MA;
	chip->ufp_power  = 0;
	chip->try_attcnt = 0;
	/* Excute the defered init mode into drp+try.snk when detached */


	wusb3801_set_mode(chip, WUSB3801_DRP);
	wusb3801_update_state(chip, WUSB3801_STATE_ERROR_RECOVERY);
}

static int test_cc_patch(struct wusb3801_chip *chip)
{
	int rc;
	int rc_reg_08,i = 0;
	struct device *cdev = &chip->client->dev;
	dev_err(cdev, "%s \n",__func__);

	i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_02, 0x82);
	msleep(DEBOUNCE_100);
	i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_09, 0xC0);
	msleep(DEBOUNCE_100);
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_TEST0);
	msleep(10);
	i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_09, 0x00);
	msleep(10);
	i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_02, 0x80);
/*huanglei add for reg 0x08 write zero fail begin */
    do{
		msleep(DEBOUNCE_100);
        i2c_smbus_write_byte_data(chip->client,
        WUSB3801_REG_TEST_02, 0x00);
	    msleep(DEBOUNCE_100);
	    rc_reg_08 = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_TEST_02);
		if (IS_ERR_VALUE_64(rc_reg_08)) {
            dev_err(cdev, "%s: WUSB3801_REG_TEST_02 failed to read\n", __func__);
        }
		i++;
	}while(rc_reg_08 !=0 && i < 5);
/* end */
	dev_err(cdev, "%s rc = [0x%02x] \n",__func__, rc);
    return BITS_GET(rc, 0x40);
}


static void wusb3801_attach(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
	uint8_t status, type;

	/* get status */
	rc = i2c_smbus_read_byte_data(chip->client,
			WUSB3801_REG_STATUS);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: failed to read status\n", __func__);
		return;
	}

	status = (rc & WUSB3801_ATTACH) ? true : false;
	type = status ? \
			rc & WUSB3801_TYPE_MASK : WUSB3801_TYPE_INVALID;

	dev_err(cdev, "%s: cc_flag = %d reg_value[0x%02x]\n", __func__,chip->cc_test_flag, rc);

	if (chip->dev_sub_id != 0xA0) {
		if(chip->cc_test_flag == 0 &&  BITS_GET(rc, WUSB3801_CC_STS_MASK) == 0 && type == WUSB3801_TYPE_SRC){
			/* chip->reg4_val = rc; */
			chip->cc_sts = test_cc_patch(chip);
			chip->cc_test_flag = 1;
			dev_err(cdev, "%s: cc_sts[0x%02x]\n", __func__, chip->cc_sts);
			return;
		}
		if(chip->cc_test_flag == 1){
			chip->cc_test_flag = 0;
			if(chip->cc_sts == WUSB3801_CC2_CONNECTED)
				rc = rc | 0x02;
			else if(chip->cc_sts == WUSB3801_CC1_CONNECTED)
				rc = rc | 0x01;
			dev_err(cdev, "%s: cc_test_patch rc[0x%02x]\n", __func__, rc);
		}
	}

	typec_dir = BITS_GET(rc, WUSB3801_CC_STS_MASK);


	dev_err(cdev, "sts[0x%02x], type[0x%02x] typec_dir[0x%02x]\n", status, type, typec_dir);

	if (chip->state != WUSB3801_STATE_ERROR_RECOVERY) {
		rc = wusb3801_set_chip_state(chip,
				WUSB3801_STATE_ERROR_RECOVERY);
		if (IS_ERR_VALUE_64(rc))
			dev_err(cdev, "%s: failed to set error recovery\n",
					__func__);
		wusb3801_detach(chip);
		dev_err(cdev, "%s: Invaild chipstate[0x%02x]\n",
				__func__, chip->state);
		return;
	}
	chip->bc_lvl = rc & WUSB3801_BCLVL_MASK;
	if(chip->bc_lvl == 0){
		chip->bc_lvl = WUSB3801_SNK_DEFAULT;
		dev_err(cdev, "%s: chip current is 0, set default\n",
				__func__);
	}
	switch (type) {
	case WUSB3801_TYPE_SRC:
		wusb3801_src_detected(chip);
		break;
	case WUSB3801_TYPE_SNK:
		wusb3801_snk_detected(chip);
		break;
	case WUSB3801_TYPE_DBG_ACC:
		wusb3801_dbg_acc_detected(chip);
		chip->type = type;
		break;
	case WUSB3801_TYPE_AUD_ACC:
		wusb3801_aud_acc_detected(chip);
		chip->type = type;
		break;
	case WUSB3801_TYPE_INVALID:
		wusb3801_detach(chip);
		dev_err(cdev, "%s: Invaild type[0x%02x]\n", __func__, type);
		break;
	default:
		rc = wusb3801_set_chip_state(chip,
				WUSB3801_STATE_ERROR_RECOVERY);
		if (IS_ERR_VALUE_64(rc))
            dev_err(cdev, "%s: failed to set error recovery\n", __func__);

		wusb3801_detach(chip);
		dev_err(cdev, "%s: Unknwon type[0x%02x]\n", __func__, type);
		break;
	}
}

static int first_check_flag = FRIST_CHECK_FLAG;
static void  wusb3801_first_check_typec_work(struct work_struct *work)
{
	struct wusb3801_chip *chip =
			container_of(work, struct wusb3801_chip, first_check_typec_work.work);
	struct device *cdev = &chip->client->dev;
	int rc;
	uint8_t int_sts;

	dev_err(cdev,"%s entry irq = %d\n",__func__, gpio_get_value(chip->irq_gpio));
	mutex_lock(&chip->mlock);

	/* get interrupt */
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
	mutex_unlock(&chip->mlock);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: failed to read interrupt\n", __func__);
		first_check_flag = RESET_FRIST_CHECK_FLAG;
		return;
	}
	int_sts = rc & WUSB3801_INT_STS_MASK;

	dev_err(cdev,"%s WUSB3801_REG_STATUS1 : 0x%02x\n", __func__, rc);

	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_STATUS);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: failed to read reg status\n", __func__);
		first_check_flag = RESET_FRIST_CHECK_FLAG;
		return;
	}
	dev_err(cdev,"%s WUSB3801_REG_STATUS2 : 0x%02x\n", __func__, rc);

	dev_err(cdev, "%s: int_sts[0x%02x]\n", __func__, int_sts);
	if (int_sts & WUSB3801_INT_DETACH){
		wusb3801_detach(chip);
	}
	if (int_sts & WUSB3801_INT_ATTACH){
		wusb3801_attach(chip);
	}
	first_check_flag = RESET_FRIST_CHECK_FLAG;

	dev_err(cdev,"%s quit irq = %d\n",__func__, gpio_get_value(chip->irq_gpio));

}


static void wusb3801_work_handler(struct work_struct *work)
{
	struct wusb3801_chip *chip =
			container_of(work, struct wusb3801_chip, dwork);
	struct device *cdev = &chip->client->dev;
	int rc;
	uint8_t int_sts;

	dev_err(cdev,"%s entry irq = %d\n",__func__, gpio_get_value(chip->irq_gpio));
	mutex_lock(&chip->mlock);

	/* get interrupt */
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
	mutex_unlock(&chip->mlock);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: failed to read interrupt\n", __func__);
		goto work_unlock;
	}
	int_sts = rc & WUSB3801_INT_STS_MASK;

	dev_err(cdev,"%s WUSB3801_REG_STATUS1 : 0x%02x\n", __func__, rc);

	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_STATUS);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: failed to read reg status\n", __func__);
		goto work_unlock;
	}
	dev_err(cdev,"%s WUSB3801_REG_STATUS2: 0x%02x ,first_check_flag = %d\n", __func__, rc, first_check_flag);

	if(first_check_flag == FRIST_CHECK_FLAG)
		goto work_unlock;

	dev_err(cdev, "%s: int_sts[0x%02x]\n", __func__, int_sts);
	if (int_sts & WUSB3801_INT_DETACH){
		wusb3801_detach(chip);
	}
	if (int_sts & WUSB3801_INT_ATTACH){
		wusb3801_attach(chip);
	}

work_unlock:
	mutex_lock(&chip->mlock);
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
	mutex_unlock(&chip->mlock);
	enable_irq(chip->irq);
	dev_err(cdev,"quit irq  = %d \n",gpio_get_value(chip->irq_gpio));
}

static irqreturn_t wusb3801_interrupt(int irq, void *data)
{
	struct wusb3801_chip *chip = (struct wusb3801_chip *)data;
	if (!chip) {
		return IRQ_HANDLED;
	}

	/*
	 * wake_lock_timeout, prevents multiple suspend entries
	 * before charger gets chance to trigger usb core for device
	 */
	 disable_irq_nosync(irq);
	__pm_wakeup_event(chip->wusb3801_ws, WUSB3801_WAKE_LOCK_TIMEOUT);
    if (!queue_work(chip->cc_wq, &chip->dwork)) {
        dev_err(&chip->client->dev, "%s: can't alloc work\n", __func__);
		enable_irq(irq);
    }

	return IRQ_HANDLED;
}

static int wusb3801_parse_dt(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	struct device_node *dev_node = cdev->of_node;
	struct wusb3801_data *data = chip->pdata;
	u32 val = 0;
	int rc = 0;
	dev_err(cdev, "dev_node->name=%s.\n",dev_node->name);
	chip->irq_gpio = of_get_named_gpio_flags(dev_node,
				"wusb3801,irq_gpio", 0,NULL);
	dev_err(cdev, "chip->irq_gpio=%d",chip->irq_gpio);
	if (chip->irq_gpio < 0) {
		dev_err(cdev, "int_gpio is not available\n");
		rc = chip->irq_gpio;
		goto out;
	}

	rc = of_property_read_u32(dev_node,
				"wusb3801,init-mode", &val);
	data->init_mode = (u8)val;
	dev_err(cdev, "data->init_mode=%d.\n",data->init_mode);
	if (rc || wusb3801_check_modes(data->init_mode)) {
		dev_err(cdev, "init mode is not available and set default\n");
		data->init_mode = WUSB3801_INIT_MODE; /* WUSB3801_DRP_ACC */
		rc = 0;
	}

	rc = of_property_read_u32(dev_node,
				"wusb3801,host-current", &val);
	data->dfp_power = (u8)val;
	dev_err(cdev, "data->dfp_power=%d.\n",data->dfp_power);
	if (rc || (data->dfp_power > WUSB3801_HOST_3000MA)) {
		dev_err(cdev, "host current is not available and set default\n");
		data->dfp_power = WUSB3801_HOST_DEFAULT;
		rc = 0;
	}

	rc = of_property_read_u32(dev_node,
				"wusb3801,drp-toggle-time", &val);
	data->dttime = (u8)val;
	if (rc || (data->dttime != WUSB3801_TGL_40MS)) {
		dev_err(cdev, "Fixed drp time and set default (40ms:40ms)\n");
		data->dttime = WUSB3801_TGL_40MS;
		rc = 0;
	}

	dev_dbg(cdev, "init_mode:%d dfp_power:%d toggle_time:%d\n",
			data->init_mode, data->dfp_power, data->dttime);

out:
	return rc;
}

static int wusb3801_pinctrl_init(struct wusb3801_chip *chip)//(struct mxt_data *data)
{
	int error;

	/* Get pinctrl if target uses pinctrl */
	chip->wusb3801_pinctrl = devm_pinctrl_get((&chip->client->dev));
	if (IS_ERR_OR_NULL(chip->wusb3801_pinctrl)) {
		printk("Device does not use pinctrl\n");
		error = PTR_ERR(chip->wusb3801_pinctrl);
		chip->wusb3801_pinctrl = NULL;
		return error;
	}

	chip->gpio_state_active
		= pinctrl_lookup_state(chip->wusb3801_pinctrl, "typec_inter_active");
	if (IS_ERR_OR_NULL(chip->gpio_state_active)) {
		printk("Can not get wusb3801 default pinstate\n");
		error = PTR_ERR(chip->gpio_state_active);
		chip->wusb3801_pinctrl = NULL;
		return error;
	}

	chip->gpio_state_suspend
		= pinctrl_lookup_state(chip->wusb3801_pinctrl, "typec_inter_sleep");
	if (IS_ERR_OR_NULL(chip->gpio_state_suspend)) {
		printk("Can not get wusb3801 sleep pinstate\n");
		error = PTR_ERR(chip->gpio_state_suspend);
		chip->wusb3801_pinctrl = NULL;
		return error;
	}

	return 0;
}

static int wusb3801_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
    struct wusb3801_chip *chip;
    struct device *cdev = &client->dev;
	struct wusb3801_data *data = NULL;
	int ret = 0,irq;

	dev_err(cdev,"wusb3801_probe start \n");
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA |
				I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(cdev, "smbus data not supported!\n");
		return -EIO;
	}

	chip = devm_kzalloc(cdev, sizeof(struct wusb3801_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(cdev, "can't alloc wusb3801_chip\n");
		return -ENOMEM;
	}

	chip->client = client;
	i2c_set_clientdata(client, chip);

	ret = wusb3801_read_device_id(chip);
	if (ret){
		dev_err(cdev, "wusb3801 doesn't find, try again\n");
		ret = wusb3801_read_device_id(chip);
		if(ret){
			dev_err(cdev, "wusb3801 doesn't find, stop find\n");
			goto err1;
		}
	}

	data = devm_kzalloc(cdev,
			sizeof(struct wusb3801_data), GFP_KERNEL);

	if (!data) {
		dev_err(cdev, "can't alloc wusb3801_data\n");
		ret = -ENOMEM;
		goto err1;
	}

	chip->pdata = data;

	ret = wusb3801_parse_dt(chip);
	if (ret) {
		dev_err(cdev, "can't parse dt\n");
		goto err2;
	}

	chip->type      = WUSB3801_TYPE_INVALID;
	chip->state     = WUSB3801_STATE_ERROR_RECOVERY;
	chip->attached  = 0;
	chip->bc_lvl    = WUSB3801_SNK_0MA;
	chip->ufp_power = 0;
	chip->defer_init = 0;
	chip->cc_sts = 0xFF;
	chip->cc_test_flag = 0;

	chip->cc_wq = alloc_ordered_workqueue("wusb3801-wq", WQ_HIGHPRI);
	if (!chip->cc_wq) {
		dev_err(cdev, "unable to create workqueue wusb3801-wq\n");
		goto err2;
	}
	INIT_DELAYED_WORK(&chip->first_check_typec_work, wusb3801_first_check_typec_work);
	INIT_WORK(&chip->dwork, wusb3801_work_handler);
	chip->wusb3801_ws = wakeup_source_register(NULL, "wusb3801_wake");
	mutex_init(&chip->mlock);

	ret = wusb3801_create_devices(cdev);
	if (IS_ERR_VALUE_64(ret)) {
		dev_err(cdev, "could not create devices\n");
		goto err4;
	}

	ret = wusb3801_reset_device(chip);
	if (ret) {
		dev_err(cdev, "failed to initialize\n");
		goto err4;
	}

	ret = wusb3801_pinctrl_init(chip);
	if (ret)
		dev_err(cdev, "No pinctrl support\n");

	chip->irq = gpio_to_irq(chip->irq_gpio);
	if (chip->irq < 0) {
		dev_err(cdev, " error gpio_to_irq returned %d\n", irq);
		goto err3;
	} else {
		dev_err(cdev, " requesting IRQ %d\n", irq);
	}

	gpio_direction_input(chip->irq_gpio);
	ret = pinctrl_select_state(chip->wusb3801_pinctrl, chip->gpio_state_active);

	if (ret)
		dev_err(cdev, "select failedt\n");
	ret = gpio_get_value(chip->irq_gpio);
	dev_err(cdev, "irq_gpio =%d irq_gpio_stat = %d\n",  chip->irq_gpio, ret);


	i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);

	ret = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_TEST_02);
	if (ret & WUSB3801_FORCE_ERR_RCY_MASK) {
		pr_err("wusb3801 [%s]enter error recovery :0x%x\n", __func__, ret);
		    i2c_smbus_write_byte_data(chip->client,
       				 WUSB3801_REG_TEST_02, 0x00);

	}
	ret = request_irq(chip->irq, wusb3801_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"wusb3801_int_irq", chip);
	if (ret) {
		dev_err(cdev, "failed to reqeust IRQ\n");
		goto err3;
	}

	enable_irq_wake(chip->irq);
	dev_err(cdev,"irq2 gpio is %d \n",gpio_get_value(chip->irq_gpio));

	ret = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_STATUS);
	if (IS_ERR_VALUE_64(ret)) {
		dev_err(cdev, "%s: failed to read reg status\n", __func__);
	}
	dev_err(cdev, "%s WUSB3801_REG_STATUS : 0x%02x\n", __func__, ret);

    i2c_smbus_write_byte_data(chip->client,
        WUSB3801_REG_TEST_02, 0x00);
    i2c_smbus_write_byte_data(chip->client,
        WUSB3801_REG_TEST_09, 0x00);

    i2c_smbus_write_byte_data(chip->client,
    WUSB3801_REG_CONTROL1,
                              BIT_REG_CTRL1_SM_RST);
	g_wusb_chip = chip;


	dev_err(cdev,"wusb3801_probe end \n");
	return 0;

err3:
	destroy_workqueue(chip->cc_wq);
	mutex_destroy(&chip->mlock);
	__pm_relax(chip->wusb3801_ws);
err4:
	wusb3801_destory_device(cdev);
err2:
	devm_kfree(cdev, chip->pdata);
err1:
	i2c_set_clientdata(client, NULL);
	devm_kfree(cdev, chip);

	return ret;
}

static int wusb3801_remove(struct i2c_client *client)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	struct device *cdev = &client->dev;

	if (!chip) {
		dev_err(cdev, "%s : chip is null\n", __func__);
		return -ENODEV;
	}

	if (chip->irq > 0)
		devm_free_irq(cdev, chip->irq, chip);



	wusb3801_destory_device(cdev);
	cancel_delayed_work_sync(&chip->first_check_typec_work);
	destroy_workqueue(chip->cc_wq);
	wakeup_source_unregister(chip->wusb3801_ws);
	mutex_destroy(&chip->mlock);

	devm_kfree(cdev, chip->pdata);

	i2c_set_clientdata(client, NULL);
	devm_kfree(cdev, chip);

	return 0;
}

static void wusb3801_shutdown(struct i2c_client *client)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	struct device *cdev = &client->dev;

	if (chip == NULL)
		return;

	if (IS_ERR_VALUE_64(wusb3801_set_mode(chip, 0x24)) ||
			IS_ERR_VALUE_64(wusb3801_set_chip_state(chip,
					WUSB3801_STATE_ERROR_RECOVERY)))
		dev_err(cdev, "%s: failed to set sink mode\n", __func__);

	if (chip != NULL) {
			if (chip->irq)
				disable_irq(chip->irq);
	}


}

#ifdef TYPEC_PM_OP
static int wusb3801_suspend(struct device *dev)
{
	return 0;
}

static int wusb3801_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops wusb3801_dev_pm_ops = {
	.suspend = wusb3801_suspend,
	.resume  = wusb3801_resume,
};
#endif

static const struct i2c_device_id wusb3801_id_table[] = {
	{.name = "wusb3801",},
};

static const struct of_device_id wusb3801_match_table[] = {
	{ .compatible = "oplus,wusb3801x",},
	{ },
};
MODULE_DEVICE_TABLE(of, wusb3801_match_table);

static struct i2c_driver wusb3801_i2c_driver = {
	.driver = {
		.name = "wusb3801",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(wusb3801_match_table),
#ifdef TYPEC_PM_OP
		.pm = &wusb3801_dev_pm_ops,
#endif
	},
	.probe = wusb3801_probe,
	.remove = wusb3801_remove,
	.shutdown = wusb3801_shutdown,
	.id_table = wusb3801_id_table,
};

static __init int wusb3801_i2c_init(void)
{
	printk("wusb3801_i2c_init\n");
	return i2c_add_driver(&wusb3801_i2c_driver);
}

static __exit void wusb3801_i2c_exit(void)
{
	printk("wusb3801_i2c_exit\n");
	i2c_del_driver(&wusb3801_i2c_driver);
}

late_initcall(wusb3801_i2c_init);
module_exit(wusb3801_i2c_exit);


MODULE_DESCRIPTION("I2C bus driver for wusb3801x USB Type-C");
MODULE_LICENSE("GPL v2");
