/*
 * hico7723_tlv320aic23.c - configuration for hico7723 baseboards with tlv320aic23
 *
 * Copyright (c) 2010 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 *
 **/

#include "../codecs/tlv320aic23.h"

static int hico7723_hw_params_tlv320aic23(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;
	unsigned int rate = params_rate(params);
	int fmt = SND_SOC_DAIFMT_I2S;

	if (cpu_is_master)
		fmt |= SND_SOC_DAIFMT_CBS_CFS;
	else
		fmt |= SND_SOC_DAIFMT_CBM_CFM;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 12000000, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(rtd->dai->cpu_dai, fmt);
	if (ret < 0)
		return ret;

	codec_freq = rate * 512;
	/*
	 * This propagates the parent frequency change to children and
	 * recalculates the frequency table
	 */
	clk_set_rate(&siumck_clk, codec_freq);

	ret = snd_soc_dai_set_sysclk(rtd->dai->cpu_dai, SIU_CLKA_EXT,
				     codec_freq / 2, SND_SOC_CLOCK_IN);

	return ret;
}

static int hico7723_hw_free_tlv320aic23(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_soc_ops hico7723_dai_ops_tlv320aic23 = {
	.hw_params = hico7723_hw_params_tlv320aic23,
	.hw_free   = hico7723_hw_free_tlv320aic23,
};

static const struct snd_soc_dapm_widget hico7723_dapm_widgets_tlv320aic23[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", 	  NULL),
	SND_SOC_DAPM_MIC("Mic Jack", 	  NULL),
};

static const struct snd_soc_dapm_route audio_map_tlv320aic23[] = {
	{ "Headphone Jack", NULL, "LHPOUT"},
	{ "Headphone Jack", NULL, "RHPOUT"},

	{ "LLINEIN", NULL, "Line In"},
	{ "RLINEIN", NULL, "Line In"},

	{ "MICIN", NULL, "Mic Jack"},
};

static int hico7723_dai_init_tlv320aic23(struct snd_soc_codec *codec)
{
	int ret;

	ret = snd_soc_dapm_new_controls(codec, hico7723_dapm_widgets_tlv320aic23,
					ARRAY_SIZE(hico7723_dapm_widgets_tlv320aic23));
	if (!ret)
		ret = snd_soc_dapm_add_routes(codec, audio_map_tlv320aic23, ARRAY_SIZE(audio_map_tlv320aic23));

	snd_soc_dapm_nc_pin(codec, "LOUT");
	snd_soc_dapm_nc_pin(codec, "ROUT");

	if (!ret)
		ret = snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	if (!ret)
		ret = snd_soc_dapm_enable_pin(codec, "Line In");
	if (!ret)
		ret = snd_soc_dapm_enable_pin(codec, "Mic Jack");

	return ret;
}

/* HiCO.DIMM7723 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link hico7723_dai_tlv320aic23 = {
	.name  		= "tlv320aic23",
	.stream_name 	= "tlv320aic23",
	.cpu_dai 	= &siu_i2s_dai,
	.codec_dai 	= &tlv320aic23_dai,
	.ops 		= &hico7723_dai_ops_tlv320aic23,
	.init 		= hico7723_dai_init_tlv320aic23,
};

/* HiCO.DIMM7723 audio machine driver */
static struct snd_soc_card snd_soc_hico7723_tlv320aic23 = {
	.name 		= "HiCO.DIMM7723",
	.platform 	= &siu_platform,
	.dai_link 	= &hico7723_dai_tlv320aic23,
	.num_links 	= 1,
};

static struct snd_soc_device hico7723_snd_devdata_tlv320aic23 = {
	.card      = &snd_soc_hico7723_tlv320aic23,
	.codec_dev = &soc_codec_dev_tlv320aic23,
};
