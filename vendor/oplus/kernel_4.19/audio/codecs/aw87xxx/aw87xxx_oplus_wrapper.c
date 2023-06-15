#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include "aw87xxx.h"

#include "aw87xxx_oplus_wrapper.h"
//------------------------------------------------------------------------------
#ifdef OPLUS_AUDIO_PA_BOOST_VOLTAGE

static int aw87xxx_rcv_voltage = AW87XXX_RCV_VOLTAGE_DEFAULT;
static int aw87xxx_spk_voltage = AW87XXX_SPK_VOLTAGE_DEFAULT;

extern unsigned char g_pa_voltage_addr[];

#endif /* OPLUS_AUDIO_PA_BOOST_VOLTAGE */

extern struct list_head g_aw87xxx_device_list;
extern int aw87xxx_i2c_write(struct aw87xxx *aw87xxx,
				unsigned char reg_addr, unsigned char reg_data);

#ifdef OPLUS_FEATURE_SPEAKER_MUTE
extern void aw87xxx_hw_on(struct aw87xxx *aw87xxx);
extern void aw87xxx_hw_off(struct aw87xxx *aw87xxx);
#endif /* OPLUS_FEATURE_SPEAKER_MUTE */

static const int spk_voltage_value[AW87XXX_VOLTAGE_LEVEL_COUNT] = {
	70,
	80
};

#ifdef OPLUS_AUDIO_PA_BOOST_VOLTAGE
int aw87xxx_set_spk_voltage(int level)
{
	struct aw87xxx_scene_param *current_scene = NULL;
	struct aw87xxx_container *aw87xxx_cont = NULL;
	struct list_head *pos;
	struct aw87xxx *aw87xxx;
	unsigned char value = 0;
	int i = 0;

	list_for_each(pos, &g_aw87xxx_device_list) {
		aw87xxx = list_entry(pos, struct aw87xxx, list);
		if (aw87xxx == NULL) {
			pr_err("%s: struct aw87xxx not ready!\n", __func__);
			return -EPERM;
		}

		if (!aw87xxx->hwen_flag) {
			aw87xxx_spk_voltage = level;
			pr_debug("%s, %d, aw87xxx is off! aw87xxx_spk_voltage = %d\n", __func__, __LINE__, aw87xxx_spk_voltage);

			return -1;
		}

		if (aw87xxx->current_mode != AW87XXX_MUSIC_MODE) {
			pr_debug("%s, %d, aw87xxx->current_mode = %d, no need change!\n", __func__, __LINE__, aw87xxx->current_mode);

			return -1;
		}

		current_scene = &aw87xxx->aw87xxx_scene_ls.scene_music;
		aw87xxx_cont = current_scene->scene_cont;

		pr_warning("%s, %d, level = %d, aw87xxx->product = %zu\n", __func__, __LINE__, level, aw87xxx->product);
		switch (level) {
		// 7V
		case 70:
			switch (aw87xxx->product) {
			case AW87XXX_339:
				value = 0x02;
				break;
			case AW87XXX_359:
				value = 0x04;
				break;
			default:
				value = 0x04;
				break;
			}
			break;
		// 7.5V
		case 75:
			switch (aw87xxx->product) {
			case AW87XXX_339:
				value = 0x04;
				break;
			case AW87XXX_359:
				value = 0x06;
				break;
			default:
				value = 0x06;
			}
			break;
		// Default
		default:
			pr_warning("%s, %d, aw87xxx_cont->len = %d", __func__, __LINE__, aw87xxx_cont->len);
			for (i = 0; i < aw87xxx_cont->len; i = i + 2) {
				// For most PA, reg 3 means CPOVP
				pr_warning("%s, %d, aw87xxx_cont->data[%d] = %#x, aw87xxx_cont->data[%d] = %#x", __func__, __LINE__, i, aw87xxx_cont->data[i], i + 1, aw87xxx_cont->data[i + 1]);

				if (g_pa_voltage_addr[aw87xxx->product] == aw87xxx_cont->data[i]) {
					value = aw87xxx_cont->data[i + 1];
					break;
				}
			}
		}
		pr_debug("%s, %d, level = %d, value = %#x, aw87xxx->hwen_flag = %d", __func__, __LINE__, level, value, aw87xxx->hwen_flag);

		mutex_lock(&aw87xxx->lock);
		aw87xxx_i2c_write(aw87xxx, g_pa_voltage_addr[aw87xxx->product], value);
		mutex_unlock(&aw87xxx->lock);

		aw87xxx_spk_voltage = level;

		pr_warning("%s, %d, aw87xxx_spk_voltage = %d, value = %#x", __func__, __LINE__, aw87xxx_spk_voltage, value);
	}

	return 0;
}

int aw87xxx_scene_update_set_spk_voltage(struct aw87xxx *aw87xxx)
{
	unsigned char value = 0;

	if (aw87xxx->current_mode == AW87XXX_MUSIC_MODE) {
		pr_debug("%s, %d, aw87xxx_spk_voltage = %d", __func__, __LINE__, aw87xxx_spk_voltage);
		if (aw87xxx_spk_voltage != 0) {

			// Please refer to datasheet
			switch (aw87xxx_spk_voltage) {
			// 7V
			case 70:
				switch (aw87xxx->product) {
				case AW87XXX_339:
					value = 0x02;
					break;
				case AW87XXX_359:
					value = 0x04;
					break;
				default:
					value = 0x04;
					break;
				}
				break;
			// 7.5V
			case 75:
				switch (aw87xxx->product) {
				case AW87XXX_339:
					value = 0x04;
					break;
				case AW87XXX_359:
					value = 0x06;
					break;
				default:
					value = 0x06;
			}
			break;
			// Default
			default:
				break;
			}
		}
		pr_warning("%s, %d, aw87xxx_spk_voltage = %d, value = %#x", __func__, __LINE__, aw87xxx_spk_voltage, value);

		if (value != 0) {
			mutex_lock(&aw87xxx->lock);
			aw87xxx_i2c_write(aw87xxx, g_pa_voltage_addr[aw87xxx->product], value);
			mutex_unlock(&aw87xxx->lock);
		}
	}

    return 0;
}
//------------------------------------------------------------------------------
#ifdef CONFIG_SND_SOC_OPLUS_PA_MANAGER

int aw87xxx_mode_set(int channel, int mode)
{
	int ret = 0;
	unsigned char set_mode = mode;

	pr_info("%s, %d, channel = %d, set_mode = %d, mode = %d\n", __func__, __LINE__,channel, set_mode, mode);

	ret = aw87xxx_audio_scene_load(set_mode, channel);
	if (ret < 0) {
		pr_err("%s: mode:%d set failed\n", __func__, set_mode);

		return -EPERM;
	}

	pr_info("%s: set mode:%d success", __func__, set_mode);

	return 0;
}

int aw87xxx_mode_get(int channel)
{
	unsigned char current_mode;

	current_mode = aw87xxx_show_current_mode(channel);

	pr_info("%s: get mode:%d\n", __func__, current_mode);

	return current_mode;
}

int aw87xxx_enable(struct oplus_speaker_device *speaker_device, int enable)
{
	int ret = 0;
	unsigned char set_mode = 0;
	int channel = 0;

	if (speaker_device == NULL) {
		pr_err("%s, %d, speaker_device == NULL\n", __func__, __LINE__);

		return -EINVAL;
	}

	set_mode = speaker_device->speaker_mode;
	pr_info("%s, %d, channel = %d, set_mode = %d, speaker_device->speaker_mode = %d\n", __func__, __LINE__, channel, set_mode, speaker_device->speaker_mode);

	channel = speaker_device->type - L_SPK;
	pr_info("%s, %d, channel = %d, set_mode = %d, enable = %d\n", __func__, __LINE__, channel, set_mode, enable);

	ret = aw87xxx_audio_scene_load(set_mode, channel);
	if (ret < 0) {
		pr_err("%s: mode:%d set failed\n", __func__, set_mode);

		return -EPERM;
	}

	pr_info("%s: set mode:%d success", __func__, set_mode);

	return 0;
}

int aw87xxx_get_status(struct oplus_speaker_device *speaker_device)
{
	int status = 0;
	int channel = 0;

	if (speaker_device == NULL) {
		pr_err("%s, %d, speaker_device == NULL\n", __func__, __LINE__);

		return -EINVAL;
	}

	channel = speaker_device->type - L_SPK;

	status = aw87xxx_mode_get(channel);

	pr_info("%s, %d, status = %d\n", __func__, __LINE__, status);

	return status;
}
#endif
//------------------------------------------------------------------------------
#ifdef OPLUS_AUDIO_PA_BOOST_VOLTAGE
int aw87xxx_voltage_get(int channel)
{
	int value = 0;
	pr_debug("%s, %d, channel = %d\n", __func__, __LINE__, channel);

	if (channel == AW87XXX_LEFT_CHANNEL) {
		switch (aw87xxx_spk_voltage) {
			case 70:
				value = 0;
				break;
			case 75:
				value = 1;
				break;
			default:
				value = 2;
				break;
		}
	} else if (channel == AW87XXX_RIGHT_CHANNEL) {
		value = aw87xxx_rcv_voltage;
	}

	pr_debug("%s, %d, channel = %d, value = %d\n", __func__, __LINE__, channel, value);

	return value;
}

int aw87xxx_voltage_put(int channel, int voltage)
{
	if (channel == AW87XXX_LEFT_CHANNEL) {
		if (voltage == aw87xxx_spk_voltage){
			return 1;
		}

		if (voltage > 0) {
			aw87xxx_spk_voltage = voltage;
		} else {
			aw87xxx_spk_voltage = AW87XXX_SPK_VOLTAGE_DEFAULT;
		}

		aw87xxx_set_spk_voltage(aw87xxx_spk_voltage);
	} else if (channel == AW87XXX_RIGHT_CHANNEL) {
		if (voltage == aw87xxx_rcv_voltage){
			return 1;
		}

		if (voltage > 0) {
			aw87xxx_rcv_voltage = voltage;
		} else {
			aw87xxx_rcv_voltage = AW87XXX_RCV_VOLTAGE_DEFAULT;
		}

	//	aw87xxx_set_rcv_voltage(aw87xxx_rcv_voltage);
	}


	return 0;
}

int aw87xxx_voltage_set(struct oplus_speaker_device *speaker_device, int voltage_level)
{
	int channel = 0;
	int index = 0;

	if (speaker_device == NULL) {
		pr_err("%s, %d, speaker_device == NULL\n", __func__, __LINE__);

		return -EINVAL;
	}

	channel = speaker_device->type - L_SPK;

	if (voltage_level >=0 && voltage_level < AW87XXX_VOLTAGE_LEVEL_COUNT) {
		index = voltage_level;
	} else {
		index = AW87XXX_VOLTAGE_LEVEL_COUNT - 1;
	}
	aw87xxx_voltage_put(channel, spk_voltage_value[index]);

	pr_info("%s, %d, spk_voltage_value[%d] = %d\n", __func__, __LINE__, index, spk_voltage_value[index]);

	return 0;
}
#endif
//------------------------------------------------------------------------------
#ifdef OPLUS_FEATURE_SPEAKER_MUTE
int aw87xxx_speaker_off(struct aw87xxx *aw87xxx)
{
	pr_info("%s enter\n", __func__);
	if (aw87xxx->hwen_flag)
		aw87xxx_i2c_write(aw87xxx, 0x01, 0x00);

	aw87xxx_hw_off(aw87xxx);

	return 0;
}

void aw87xxx_speaker_on(struct aw87xxx *aw87xxx)
{
	pr_info("%s enter\n", __func__);
	aw87xxx_hw_on(aw87xxx);
}

void aw87xxx_force_mute_set(int enable)
{
        struct list_head *pos;
        struct aw87xxx *aw87xxx;

	list_for_each(pos, &g_aw87xxx_device_list) {
		aw87xxx = list_entry(pos, struct aw87xxx, list);
		if (aw87xxx == NULL) {
			pr_err("%s struct aw87xxx not ready!\n", __func__);
			return;
		}
		//if current mode is receiver mode, don't mute speaker
		if(aw87xxx->current_mode == AW87XXX_RCV_MODE) {
			pr_info("%s, %d, speaker is not mute, current_mode = %d, pa_channel = %d", __func__, __LINE__, aw87xxx->current_mode, aw87xxx->pa_channel);
			return;
		} else if (enable) {
			pr_info("%s, %d, speaker force mute enable value is %d, current_mode = %d, pa_channel = %d\n",
				 __func__, __LINE__, enable, aw87xxx->current_mode, aw87xxx->pa_channel);
			aw87xxx_speaker_off(aw87xxx);
		} else {
			aw87xxx_speaker_on(aw87xxx);
			aw87xxx->current_mode = aw87xxx_show_current_mode(aw87xxx->pa_channel);
			aw87xxx_audio_scene_load(aw87xxx->current_mode, aw87xxx->pa_channel);
			pr_debug("%s, %d, speaker is  active aw87xxx->hwen_flag = %d, aw87xxx->current_mode = %d, aw87xxx->pa_channel = %d",
				__func__, __LINE__, aw87xxx->hwen_flag, aw87xxx->current_mode, aw87xxx->pa_channel);
		}
	}
}
#endif /* OPLUS_FEATURE_SPEAKER_MUTE */
//------------------------------------------------------------------------------
#endif /* OPLUS_AUDIO_PA_BOOST_VOLTAGE */
