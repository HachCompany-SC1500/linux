/*
 * hico7723_ssm2518.c - configuration for hico7723 baseboards with ssm2518
 *
 * Copyright (c) 2011 by emtrion GmbH
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
 * Author:      Dr. Frank Reither
 *
 **/

#include "../codecs/ssm2518.h"

static int hico7723_hw_params_ssm2518(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;
	int clk_div;
	unsigned int rate = params_rate(params);
	int fmt = SND_SOC_DAIFMT_I2S;
	
	if (cpu_is_master) {
		fmt |= SND_SOC_DAIFMT_CBS_CFS;
	}
	else {
		fmt |= SND_SOC_DAIFMT_CBM_CFM;
	}
	
// setting mclk for SSM2518
	codec_freq = 12288000;
	ret = snd_soc_dai_set_sysclk(codec_dai, SIU_CLKA_EXT, codec_freq, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;
	
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(rtd->dai->cpu_dai, fmt);
	if (ret < 0)
		return ret;
	
	ret = snd_soc_dai_set_sysclk(rtd->dai->cpu_dai, SIU_CLKA_EXT,
				     codec_freq, SND_SOC_CLOCK_IN);
	
	clk_div = 6144000 / (rate * 32);
	clk_div /= 2;
	clk_div -= 1;
	
//	printk("---> setting divider to %d\n", clk_div);
	
	ret = snd_soc_dai_set_clkdiv(rtd->dai->cpu_dai, SIU_CLKA_EXT, clk_div);
	
	return ret;
}

static int hico7723_hw_free_ssm2518(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_soc_ops hico7723_dai_ops_ssm2518 = {
	.hw_params = hico7723_hw_params_ssm2518,
 	.hw_free   = hico7723_hw_free_ssm2518,
};

static const struct snd_soc_dapm_widget hico7723_dapm_widgets_ssm2518[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
//		SND_SOC_DAPM_LINE("Line In", 	  NULL),
//		SND_SOC_DAPM_MIC("Mic Jack", 	  NULL),
};

static const struct snd_soc_dapm_route audio_map_ssm2518[] = {
	{ "Headphone Jack", NULL, "LHPOUT"},
	{ "Headphone Jack", NULL, "RHPOUT"},
#if 0
 { "LLINEIN", NULL, "Line In"},
		{ "RLINEIN", NULL, "Line In"},

		{ "MICIN", NULL, "Mic Jack"},
#endif
};

static int hico7723_dai_init_ssm2518(struct snd_soc_codec *codec)
{
	int ret;

	ret = snd_soc_dapm_new_controls(codec, hico7723_dapm_widgets_ssm2518,
					ARRAY_SIZE(hico7723_dapm_widgets_ssm2518));
#if 0
	if (!ret)
		ret = snd_soc_dapm_add_routes(codec, audio_map_ssm2518, ARRAY_SIZE(audio_map_ssm2518));
#endif
//	snd_soc_dapm_nc_pin(codec, "LOUT");
//	snd_soc_dapm_nc_pin(codec, "ROUT");

	if (!ret)
#if 0
		ret = snd_soc_dapm_enable_pin(codec, "Headphone Jack");
		if (!ret)
			ret = snd_soc_dapm_enable_pin(codec, "Line In");
		if (!ret)
			ret = snd_soc_dapm_enable_pin(codec, "Mic Jack");
#endif
	return ret;
}

/* HiCO.DIMM7723 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link hico7723_dai_ssm2518 = {
	.name  		= "ssm2518",
	.stream_name 	= "ssm2518",
	.cpu_dai 	= &siu_i2s_dai,
	.codec_dai 	= &ssm2518_dai,
	.ops 		= &hico7723_dai_ops_ssm2518,
	.init 		= hico7723_dai_init_ssm2518,
};

/* HiCO.DIMM7723 audio machine driver */
static struct snd_soc_card snd_soc_hico7723_ssm2518 = {
	.name 		= "HiCO.DIMM7723",
	.platform 	= &siu_platform,
	.dai_link 	= &hico7723_dai_ssm2518,
	.num_links 	= 1,
};

static struct snd_soc_device hico7723_snd_devdata_ssm2518 = {
	.card      = &snd_soc_hico7723_ssm2518,
	.codec_dev = &soc_codec_dev_ssm2518,
};
