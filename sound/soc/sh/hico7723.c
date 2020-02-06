/*
 * hico7723.c - provides SOC sound interface (initialization on the module)
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/em_baseboard.h>
#include <linux/em_common.h>
//#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/clock.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <cpu/sh7723.h>

#include "../../../arch/sh/boards/mach-emtrion/include/sh7723.h"
#include "siu.h"

#if defined(CONFIG_SND_SOC_WM8974) || defined(CONFIG_SND_SOC_WM8974_MODULE)
# define HAVE_WM8974
#endif

#if defined(CONFIG_SND_SOC_TLV320AIC23) || defined(CONFIG_SND_SOC_TLV320AIC23_MODULE)
# define HAVE_TLV320AIC23
#endif

#if defined(CONFIG_SND_SOC_SSM2518) || defined(CONFIG_SND_SOC_SSM2518_MODULE)
# define HAVE_SSM2518
#endif

/* Default 8000Hz sampling frequency */
static unsigned long codec_freq = 8000 * 512;

static int cpu_is_master;
module_param(cpu_is_master, int, 0);
MODULE_PARM_DESC(cpu_is_master, "The CPU is configured as master and the codec as slave. This changes the frequency of the sound by ~6%");

/* External clock, sourced from the codec at the SIUMCKA pin */
static unsigned long siumck_recalc(struct clk *clk)
{
	return codec_freq;
}

static struct clk_ops siumck_clk_ops = {
	.recalc = siumck_recalc,
};

static struct clk siumck_clk = {
	.name		= "siumcka_clk",
	.id		= -1,
	.ops		= &siumck_clk_ops,
	.rate		= 0, /* initialised at run-time */
};

static const int hico7723_snd_gpios[] = {
	GPIO_FN_SIUAILR,
	GPIO_FN_SIUAIBT,
	GPIO_FN_SIUAISLD,
	GPIO_FN_SIUAOLR,
	GPIO_FN_SIUAOBT,
	GPIO_FN_SIUAOSLD,
	GPIO_FN_SIUAMCK,
	/* SPDIF not yet supported in linux */
};


/* include baseboard specific codec initialisation */
#ifdef HAVE_TLV320AIC23
# include "hico7723_tlv320aic23.c"
#endif

#ifdef HAVE_WM8974
# include "hico7723_wm8974.c"
#endif

#ifdef HAVE_SSM2518
# include "hico7723_ssm2518.c"
#endif

static struct platform_device *hico7723_snd_device;

static int __init hico7723_init(void)
{
	int ret;
	struct snd_soc_device *soc_device = NULL;
	
	
	/* register and configure pins */
	ret = gpio_fn_request_array(hico7723_snd_gpios, ARRAY_SIZE(hico7723_snd_gpios), "sound");
	if (ret<0)
		goto error_gpio;
	
	ret = clk_register(&siumck_clk);
	if (ret < 0)
		goto error_clk;
	
	
	// checking for ssm2518....
	soc_device = &hico7723_snd_devdata_ssm2518;
	
	/* Port number of the SIU used on this machine: port A */
	hico7723_snd_device = platform_device_alloc("soc-audio", 0);
	if (!hico7723_snd_device) {
		printk(KERN_INFO "%s platform_device_alloc failed\n", __func__);
		ret = -ENOMEM;
		goto error_plat_alloc;
	}
	
	// checking step by step which codec is available.
	// it's a real ugly hack but the best solution for the short time of development
	if (soc_device) {
		printk(KERN_INFO "scanning for ssm2518!\n");
		platform_set_drvdata(hico7723_snd_device, soc_device);

		soc_device->dev = &hico7723_snd_device->dev;

		ret = platform_device_add(hico7723_snd_device);
		if ( ret == 0 ) {
			if ( soc_device->card->codec ) {
				cpu_is_master = 1;
				/* cpu is master and codec is slave */
				/* configure sound control lines to be output */
				__raw_writew(0x003C, MSELCRA);
				printk(KERN_INFO "ssm2518 found!\n");
				return 0;
			}
			else {
				printk(KERN_INFO "ssm2518 not found!\n");
				platform_device_unregister(hico7723_snd_device);
			}
		}
		else {
			printk(KERN_INFO "ssm2518 not found!\n");
			platform_device_unregister(hico7723_snd_device);
		}
	}
	else {
		ret = -ENODEV;
	}
	
	
	// checking for tlv320aic23....
	soc_device = &hico7723_snd_devdata_tlv320aic23;
	/* Port number of the SIU used on this machine: port A */
	hico7723_snd_device = platform_device_alloc("soc-audio", 0);
	if (!hico7723_snd_device) {
		printk(KERN_INFO "%s platform_device_alloc failed\n", __func__);
		ret = -ENOMEM;
		goto error_plat_alloc;
	}
	
	// checking step by step which codec is available.
	// it's a real ugly hack but the best solution for the short time of development
	if (soc_device) {
		printk(KERN_INFO "scanning for tlv320aic23!\n");
		platform_set_drvdata(hico7723_snd_device, soc_device);

		soc_device->dev = &hico7723_snd_device->dev;

		ret = platform_device_add(hico7723_snd_device);
		if ( ret == 0 ) {
			if ( soc_device->card->codec ) {
				cpu_is_master = 0;
				/* cpu is slave and codec is master */
				/* configure sound control lines to be input */
				__raw_writew(0x0000, MSELCRA);

				/* up to U-Boot 2009.08em4 PTK5 was configured always as input.
				But on Rev. 3a or later, we need it as special function */
				__raw_writew(__raw_readw(PORT_PKCR) & ~(3<<(5*2)), PORT_PKCR);
				printk(KERN_INFO "tlv320aic23 found!\n");
				return 0;
			}
			else {
				printk(KERN_INFO "tlv320aic23 not found!\n");
				platform_device_unregister(hico7723_snd_device);
			}
		}
		else {
			printk(KERN_INFO "tlv320aic23 not found!\n");
			platform_device_unregister(hico7723_snd_device);
		}
	}
	else {
		ret = -ENODEV;
	}

error_plat_alloc:
	clk_unregister(&siumck_clk);
error_clk:
	gpio_fn_free_array(hico7723_snd_gpios, ARRAY_SIZE(hico7723_snd_gpios));
error_gpio:

	return ret;
}

static void __exit hico7723_exit(void)
{
	gpio_fn_free_array(hico7723_snd_gpios, ARRAY_SIZE(hico7723_snd_gpios));
	clk_unregister(&siumck_clk);
	platform_device_unregister(hico7723_snd_device);
}

module_init(hico7723_init);
module_exit(hico7723_exit);

MODULE_AUTHOR("Markus Pietrek");
MODULE_DESCRIPTION("ALSA SoC HiCO.DIMM7723");
MODULE_LICENSE("GPL v2");
