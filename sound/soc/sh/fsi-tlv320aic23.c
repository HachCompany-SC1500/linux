/*
 * FSI-TLV320AIC23 sound support for DIMM-Base_Lothron
 *
 * Copyright (C) 2010 Emtrion GmbH
 * Michael Szafranek <michael.szafranek@emtrion.de>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <sound/sh_fsi.h>

extern struct snd_soc_dai tlv320aic23_dai;
extern struct snd_soc_codec_device soc_codec_dev_tlv320aic23;

static int fsi_tlv320aic23_dai_init(struct snd_soc_codec *codec)
{
	int ret;
	struct snd_soc_dai *dai = codec->dai;

	ret = snd_soc_dai_set_fmt(dai,  
				SND_SOC_DAIFMT_I2S | 
				SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(dai, 0, 12000000, SND_SOC_CLOCK_IN);

	return ret;
}

static struct snd_soc_dai_link fsi_dai_link = {
	.name		= "tlv320aic23",
	.stream_name	= "tlv320aic23",
	.cpu_dai	= &fsi_soc_dai[0], /* fsi */
	.codec_dai	= &tlv320aic23_dai,
	.init		= fsi_tlv320aic23_dai_init,
	.ops		= NULL,
};

static struct snd_soc_card fsi_soc_card  = {
	.name		= "FSI",
	.platform	= &fsi_soc_platform,
	.dai_link	= &fsi_dai_link,
	.num_links	= 1,
};

static struct snd_soc_device fsi_snd_devdata = {
	.card		= &fsi_soc_card,
	.codec_dev	= &soc_codec_dev_tlv320aic23,
};

static struct platform_device *fsi_snd_device;

static int __init fsi_tlv320aic23_init(void)
{
	int ret = -ENOMEM;

	fsi_snd_device = platform_device_alloc("soc-audio", -1);
	if (!fsi_snd_device)
		goto out;

	platform_set_drvdata(fsi_snd_device,
			     &fsi_snd_devdata);
	fsi_snd_devdata.dev = &fsi_snd_device->dev;
	ret = platform_device_add(fsi_snd_device);

	if (ret)
		platform_device_put(fsi_snd_device);

out:
	return ret;
}

static void __exit fsi_tlv320aic23_exit(void)
{
	platform_device_unregister(fsi_snd_device);
}

module_init(fsi_tlv320aic23_init);
module_exit(fsi_tlv320aic23_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Generic SH4 FSI-tlv320aic23 sound card");
MODULE_AUTHOR("Michael Szafranek <michael.szafranek@emtrion.de>");

