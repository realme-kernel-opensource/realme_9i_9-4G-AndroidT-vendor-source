#ifndef __AW87XXX_OPLUS_WRAPPER_H__
#define __AW87XXX_OPLUS_WRAPPER_H__
//------------------------------------------------------------------------------
#include "aw87xxx_monitor.h"
#include "aw87xxx.h"
//------------------------------------------------------------------------------
#ifdef OPLUS_AUDIO_PA_BOOST_VOLTAGE

#define AW87XXX_RCV_VOLTAGE_DEFAULT (70)
#define AW87XXX_SPK_VOLTAGE_DEFAULT (80)
#endif /* OPLUS_AUDIO_PA_BOOST_VOLTAGE */
//------------------------------------------------------------------------------
#ifdef CONFIG_SND_SOC_OPLUS_PA_MANAGER

enum {
	AW87XXX_LEFT_CHANNEL = 0,
	AW87XXX_RIGHT_CHANNEL = 1,
};

enum {
	AW87XXX_VOLTAGE_LEVEL_1 = 0,
	AW87XXX_VOLTAGE_LEVEL_4 = 1,
	AW87XXX_VOLTAGE_LEVEL_COUNT
};
#endif /* CONFIG_SND_SOC_OPLUS_PA_MANAGER */
//------------------------------------------------------------------------------
#ifdef OPLUS_AUDIO_PA_BOOST_VOLTAGE
int aw87xxx_set_spk_voltage(int level);
int aw87xxx_scene_update_set_spk_voltage(struct aw87xxx *aw87xxx);
int aw87xxx_voltage_get(int channel);
int aw87xxx_voltage_put(int channel, int voltage);
#endif /* OPLUS_AUDIO_PA_BOOST_VOLTAGE */
//------------------------------------------------------------------------------
#ifdef CONFIG_SND_SOC_OPLUS_PA_MANAGER
int aw87xxx_mode_set(int channel, int mode);
int aw87xxx_mode_get(int channel);
int aw87xxx_enable(struct oplus_speaker_device *speaker_device, int mode);
int aw87xxx_get_status(struct oplus_speaker_device *speaker_device);
int aw87xxx_voltage_set(struct oplus_speaker_device *speaker_device, int voltage_level);
#ifdef OPLUS_FEATURE_SPEAKER_MUTE
void aw87xxx_force_mute_set(int enable);
#endif /* OPLUS_FEATURE_SPEAKER_MUTE */
#endif /* CONFIG_SND_SOC_OPLUS_PA_MANAGER */
//------------------------------------------------------------------------------
#endif /* __AW87XXX_OPLUS_WRAPPER_H__ */
