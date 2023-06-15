#ifndef __BQ25890H_HEADER__
#define __BQ25890H_HEADER__


#define SY697X_USB_VLIM            4500
#define SY697X_USB_ILIM            2000
#define SY697X_USB_VREG            4200
#define SY697X_CHG_ICHG            2000
#define SY697X_CHG_PRCHG           256
#define SY697X_CHG_TERM            250
#define SY697X_OTG_BOOTSTV         5000
#define SY697X_OTG_BOOTSTI         1200
#define SY697X_VLOT_LIMIT          4400
#define SY697X_CHG_VBUS            3300
#define SY697X_BUFF_NUM            400
#define SY697X_VALU_NUM            25
#define SY697X_ICH_1000            1000


#define SY697X_NORMAL_TEMP25    25
#define SY697X_CUR_PRECENT50    50
#define ADAPTER_CAP_MAX_NR 10
#define SY697X_DEFLEAT_SHIP_MODE    0x2c
struct adapter_power_cap {
	uint8_t selected_cap_idx;
	uint8_t nr;
	uint8_t pdp;
	uint8_t pwr_limit[ADAPTER_CAP_MAX_NR];
	int max_mv[ADAPTER_CAP_MAX_NR];
	int min_mv[ADAPTER_CAP_MAX_NR];
	int ma[ADAPTER_CAP_MAX_NR];
	int maxwatt[ADAPTER_CAP_MAX_NR];
	int minwatt[ADAPTER_CAP_MAX_NR];
	uint8_t type[ADAPTER_CAP_MAX_NR];
	int info[ADAPTER_CAP_MAX_NR];
};
enum adapter_cap_type {
 	MTK_PD_APDO_START,
 	MTK_PD_APDO_END,
 	MTK_PD,
 	MTK_PD_APDO,
 	MTK_CAP_TYPE_UNKNOWN,
 };
extern int opchg_get_real_charger_type(void);
extern int oplus_bq25890h_enable_otg(void);
extern int oplus_bq25890h_disable_otg(void);
extern void splitchg_request_dpdm(bool enable);
extern void oplus_set_prswap(bool swap);

#endif
