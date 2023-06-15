/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */


#ifndef __DAILINK_EXTENDS_H
#define __DAILINK_EXTENDS_H

#define MI2S_TX_HOSTLESS_DAILINK(_name, _stream_name, _cpu_dai_name) \
{                                                           \
	.name = _name,                                      \
	.stream_name = _stream_name,                        \
	.cpu_dai_name = _cpu_dai_name,                      \
	.platform_name = "msm-pcm-hostless",                \
	.dynamic = 1,                                       \
	.dpcm_capture = 1,                                  \
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,              \
			SND_SOC_DPCM_TRIGGER_POST},         \
	.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,           \
	.ignore_suspend = 1,                                \
	.ignore_pmdown_time = 1,                            \
	.codec_dai_name = "snd-soc-dummy-dai",              \
	.codec_name = "snd-soc-dummy",                      \
}                                                           \

#define TX_CDC_DMA_HOSTLESS_DAILINK(_name, _stream_name, _cpu_dai_name) \
{                                                           \
	.name = _name,                                      \
	.stream_name = _stream_name,                        \
	.cpu_dai_name = _cpu_dai_name,                      \
	.platform_name = "msm-pcm-hostless",                \
	.dynamic = 1,                                       \
	.dpcm_playback = 1,                                 \
	.dpcm_capture = 1,                                  \
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,              \
			SND_SOC_DPCM_TRIGGER_POST},         \
	.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,           \
	.ignore_suspend = 1,                                \
	.codec_dai_name = "snd-soc-dummy-dai",              \
	.codec_name = "snd-soc-dummy",                      \
}                                                           \

#endif /* __DAILINK_EXTENDS_H */
