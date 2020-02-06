/*
 * hico7723_wm8974.c - configuration for baseboards with wm8974
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
 * Author: Markus Pietrek
 *
 **/

#include "../codecs/wm8974.h"

static int hico7723_hw_params_wm8974(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;
	unsigned int rate = params_rate(params);
	int fmt = SND_SOC_DAIFMT_I2S;

	if (cpu_is_master)
		/* only codec is connected to 12MHz clock  */
		return -EINVAL;

	/* codec is master */
	fmt |= SND_SOC_DAIFMT_CBM_CFM;

	if (codec_dai->ops->set_sysclk) {
		ret = snd_soc_dai_set_sysclk(codec_dai, 0, 11289600, SND_SOC_CLOCK_IN);
		if (ret < 0)
			return ret;
	}

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
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8974_OPCLKDIV, 0x00);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8974_MCLKDIV, 0x00);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8974_BCLKDIV, 0x00);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_pll(codec_dai, 0, 0, 12000000, 11289600);

	return ret;
}

static int hico7723_hw_free_wm8974(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_soc_ops hico7723_dai_ops_wm8974 = {
	.hw_params = hico7723_hw_params_wm8974,
	.hw_free   = hico7723_hw_free_wm8974,
};

static const struct snd_soc_dapm_widget hico7723_dapm_widgets_wm8974[] = {
	SND_SOC_DAPM_HP("Speaker Out", NULL),
};

static const struct snd_soc_dapm_route audio_map_wm8974[] = {
	{ "Speaker Out", NULL, "SPKOUTN"},
	{ "Speaker Out", NULL, "SPKOUTP"},
};

static int hico7723_dai_init_wm8974(struct snd_soc_codec *codec)
{
	int ret;

	ret = snd_soc_dapm_new_controls(codec, hico7723_dapm_widgets_wm8974,
					ARRAY_SIZE(hico7723_dapm_widgets_wm8974));
	if (!ret)
		ret = snd_soc_dapm_add_routes(codec, audio_map_wm8974, ARRAY_SIZE(audio_map_wm8974));

	snd_soc_dapm_nc_pin(codec, "MONOOUT");
	snd_soc_dapm_nc_pin(codec, "Mic Bias");
	snd_soc_dapm_nc_pin(codec, "MICP");
	snd_soc_dapm_nc_pin(codec, "MICN");
	snd_soc_dapm_nc_pin(codec, "AUX");

	if (!ret)
		ret = snd_soc_dapm_enable_pin(codec, "Speaker Out");

	return ret;
}

/* HiCO.DIMM7723 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link hico7723_dai_wm8974 = {
	.name  		= "wm8974",
	.stream_name 	= "wm8974",
	.cpu_dai 	= &siu_i2s_dai,
	.codec_dai 	= &wm8974_dai,
	.ops 		= &hico7723_dai_ops_wm8974,
	.init 		= hico7723_dai_init_wm8974,
};

/* HiCO.DIMM7723 audio machine driver */
static struct snd_soc_card snd_soc_hico7723_wm8974 = {
	.name 		= "HiCO.DIMM7723_WM8974",
	.platform 	= &siu_platform,
	.dai_link 	= &hico7723_dai_wm8974,
	.num_links 	= 1,
};

/* HiCO.DIMM7723 audio subsystem */
static struct snd_soc_device hico7723_snd_devdata_wm8974 = {
	.card      = &snd_soc_hico7723_wm8974,
	.codec_dev = &soc_codec_dev_wm8974,
};
