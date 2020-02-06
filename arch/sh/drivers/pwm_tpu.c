/*
 * pwm_tpu.c
 *
 * Copyright (c) 2010 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:	Markus Pietrek
 * Description: Provides PWM functionality with the TPU.
 *		TPU outputs 1 in disabled state and 0 in active state.
 *
 **/

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/pwm_tpu.h>

#ifdef CONFIG_CPU_SUBTYPE_SH7723
# include <cpu/sh7723.h>
#endif

/* TPU register definitions */
#define TSTR		0x00

/* TPU channel specific registers */
#define TPUC_BASE	0x10
#define TPUC_OFFSET	0x40

#define TCR		0x00
#define TMDR		0x04
#define TIOR		0x08
#define TIER		0x0C
#define TSR		0x10
#define TCNT		0x14
#define TGRA		0x18
#define TGRB		0x1C
#define TGRC		0x20
#define TGRD		0x24

/* TPU register bits */
#define TCR_CCLR_TGRB_CLEARS	0x40
#define TCR_CKEG_RISING		0x00
#define TCR_TPSC_1		0x00
#define TCR_TPSC_4		0x01
#define TCR_TPSC_16		0x02
#define TCR_TPSC_64		0x03

#define TIOR_OUTPUT_ALWAYS_0			0x00
#define TIOR_OUTPUT_ALWAYS_1			0x04
#define TIOR_INIT_0_OUTPUT_1_ON_MATCH_TGRA	0x02
#define TIOR_INIT_1_OUTPUT_0_ON_MATCH_TGRA	0x05

#define TMDR_PWM	0x02

#define CHANNELS 4

struct pwm_device {
	const char *label;
	int pwm_id;
	int active;		/* for counting whether the clock on the TPU needs to be enabled */
	int inverted;
	int divisor;
	int divisor_reg;
};

static struct pwm_tpu_priv {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct clk *bclk;
	struct mutex mutex;

	struct pwm_device devices[CHANNELS];
	int clk_enabled;
	int invert_polarity;
} *priv;

enum {
	TIOR_OUTPUT_OFF = 0,
	TIOR_OUTPUT_ON_MATCH,
};

static int output_flags[2][2] = {
	[0] = {
		/* not inverted polarity */
		[TIOR_OUTPUT_OFF] = TIOR_OUTPUT_ALWAYS_1,
		[TIOR_OUTPUT_ON_MATCH] = TIOR_INIT_1_OUTPUT_0_ON_MATCH_TGRA,
	},
	[1] = {
		/* inverted polarity */
		[TIOR_OUTPUT_OFF] = TIOR_OUTPUT_ALWAYS_0,
		[TIOR_OUTPUT_ON_MATCH] = TIOR_INIT_0_OUTPUT_1_ON_MATCH_TGRA,
	}
};

static inline void tpuc_write(const struct pwm_tpu_priv *priv, int ch, u16 val, int reg)
{
	iowrite16(val, priv->base + TPUC_BASE + (ch*TPUC_OFFSET) + reg);
}

static inline u16 tpuc_read(const struct pwm_tpu_priv *priv, int ch, int reg)
{
	return ioread16(priv->base + TPUC_BASE + (ch*TPUC_OFFSET) + reg);
}

static inline void tpuc_start(struct pwm_device *pwm)
{
	iowrite16(ioread16(priv->base+TSTR) | (1<<pwm->pwm_id), priv->base+TSTR);
}

static inline void tpuc_stop(struct pwm_device *pwm)
{
	iowrite16(ioread16(priv->base+TSTR) & ~(1<<pwm->pwm_id), priv->base+TSTR);
}

static inline int tpuc_running(struct pwm_device *pwm)
{
	return ioread16(priv->base+TSTR) & (1<<pwm->pwm_id);
}

/* exported pwm functions */
struct pwm_device *pwm_request(int pwm_id, const char *label)
{
        struct pwm_tpu_platform_data *pdata = dev_get_platdata(priv->dev);
	struct pwm_device *pwm = NULL;

	mutex_lock(&priv->mutex);

	if (!label) {
		dev_err(priv->dev, "need a name for pwm %i\n", pwm_id);
		pwm = ERR_PTR(-EINVAL);
		goto out;
	}

	if (pwm_id >= ARRAY_SIZE(priv->devices) || priv->devices[pwm_id].label) {
		dev_err(priv->dev, "TPU %i already in use %s\n", pwm_id, priv->devices[pwm_id].label);
		pwm = ERR_PTR(-EBUSY);
		goto out;
	}
	pwm = &priv->devices[pwm_id];

	pwm->label  = label;
	pwm->pwm_id = pwm_id;

	if (pdata && (pdata->invert_polarity & (1<<pwm_id)))
		/* for compatibility, the platform configuration doesn't need to be available. We stick to the old behaviour */
		pwm->inverted = 1;

	pwm->divisor = pdata ? pdata->divisor : 64;
	switch (pwm->divisor) {
	    case  1: pwm->divisor_reg = TCR_TPSC_1;  break;
	    case  4: pwm->divisor_reg = TCR_TPSC_4;  break;
	    case 16: pwm->divisor_reg = TCR_TPSC_16; break;
	    case 64: pwm->divisor_reg = TCR_TPSC_64; break;
	    default:
		dev_err(priv->dev, "Divisor %i not supported\n", pwm->divisor);
		pwm = ERR_PTR(-EINVAL);
		goto out;
	}

	dev_info(priv->dev, "TPU %i registered for %s\n", pwm_id, label);

out:
	mutex_unlock(&priv->mutex);
	return pwm;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	mutex_lock(&priv->mutex);
	pwm->label = NULL;
	mutex_unlock(&priv->mutex);
}
EXPORT_SYMBOL(pwm_free);

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	int period_hz;
	int tcr, tior, tgra, tgrb, tmdr;
	int was_running;

	if ((period_ns <= 0) || (duty_ns > period_ns)) {
		dev_err(priv->dev, "Parameters out of range\n");
		return -EINVAL;
	}

	mutex_lock(&priv->mutex);

	/* enable clock for TPU when it was off before	*/
	if (!priv->clk_enabled) {
		clk_enable(priv->clk);
		priv->clk_enabled = 1;
	}
	pwm->active = 1;

	/* calculate TGRA and TGRB settings based on the duty/period durations */
	period_hz = 1000000000 / period_ns;
	tgrb = ((clk_get_rate(priv->bclk)/pwm->divisor) / period_hz);

	if (duty_ns >= period_ns)
		tgra = 0;	/* 100% duty */
	else if (duty_ns <= 0)
		tgra = tgrb + 1; /* off */
	else
		tgra = tgrb - (tgrb * (duty_ns/1000))/(period_ns/1000);

	tcr = TCR_CCLR_TGRB_CLEARS |
	      TCR_CKEG_RISING	   |
	      pwm->divisor_reg;

	tior = output_flags[pwm->inverted][TIOR_OUTPUT_ON_MATCH];
	tmdr = TMDR_PWM;

	was_running = tpuc_running(pwm);

	if (!was_running ||
	    tpuc_read(priv, pwm->pwm_id, TCR)  != tcr  ||
	    tpuc_read(priv, pwm->pwm_id, TIOR) != tior ||
	    tpuc_read(priv, pwm->pwm_id, TGRA) != tgra ||
	    tpuc_read(priv, pwm->pwm_id, TGRB) != tgrb ||
	    tpuc_read(priv, pwm->pwm_id, TMDR) != tmdr) {
		/* don't change an active TPU when it's not necessary.
		 Otherwise a backlight controlled by the TPU would flicker. */

		if (was_running)
			/* we shouldn't modify the registers while the TPU is running */
			tpuc_stop(pwm);

		/* configure TPU */
		tpuc_write(priv, pwm->pwm_id, tcr,  TCR);
		tpuc_write(priv, pwm->pwm_id, tior, TIOR);
		tpuc_write(priv, pwm->pwm_id, tgra, TGRA);
		tpuc_write(priv, pwm->pwm_id, tgrb, TGRB);
		tpuc_write(priv, pwm->pwm_id, tmdr, TMDR);

		if (was_running)
			tpuc_start(pwm);
	}

	mutex_unlock(&priv->mutex);

	return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	/* pwm_config is called before enable, so we don't need to enable priv->clk here */
	mutex_lock(&priv->mutex);
	if (!tpuc_running(pwm))
		tpuc_start(pwm);
	mutex_unlock(&priv->mutex);

	return 0;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	int devices_active = 0;
	int i;

	mutex_lock(&priv->mutex);

	/* set output to off state */
	tpuc_write(priv, pwm->pwm_id,
		   output_flags[pwm->inverted][TIOR_OUTPUT_OFF],
		   TIOR);
	tpuc_stop(pwm);

	/* disable power when TPU is no longer in use */
	pwm->active = 0;
	for (i=0; i < ARRAY_SIZE(priv->devices); i++) {
		if (priv->devices[i].active) {
			devices_active = 1;
			break;
		}
	}

	if (!devices_active) {
		clk_disable(priv->clk);
		priv->clk_enabled = 0;
	}

	mutex_unlock(&priv->mutex);
}
EXPORT_SYMBOL(pwm_disable);

/* driver registering functions */

static int __devinit pwm_tpu_probe(struct platform_device *pdev)
{
	struct resource *res;
	int error = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "cannot get memory\n");
		error = -ENOMEM;
		goto error;
	}
	priv->dev = &pdev->dev;
	platform_set_drvdata(pdev, priv);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get platform resources\n");
		error = -EINVAL;
		goto error_map;
	}

	priv->base = ioremap(res->start, (res->end-res->start)+1);
	if (!priv->base) {
		dev_err(&pdev->dev, "cannot get io memory\n");
		error = -ENOMEM;
		goto error_map;
	}

	priv->clk = clk_get(&pdev->dev, "tpu0");
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "cannot get clock");
		goto error_clock;
	}

	priv->bclk = clk_get(&pdev->dev, "bus_clk");
	if (IS_ERR(priv->bclk)) {
		dev_err(&pdev->dev, "cannot get bus clock");
		goto error_bclock;
	}

	mutex_init(&priv->mutex);

	dev_info(&pdev->dev, "initialized\n");
	return 0;

error_bclock:
	clk_put(priv->bclk);

error_clock:
	iounmap(priv->base);

error_map:
	kfree(priv);

error:
	return error;
}

static int __devexit pwm_tpu_remove(struct platform_device *pdev)
{
	clk_put(priv->bclk);
	clk_put(priv->clk);
	iounmap(priv->base);
	kfree(priv);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver pwm_tpu_driver = {
	.probe		= pwm_tpu_probe,
	.remove		= pwm_tpu_remove,
	.driver		= {
		.name	= "pwm_tpu",
		.owner	= THIS_MODULE,
	},
};

static int __init pwm_tpu_init(void)
{
	return platform_driver_register(&pwm_tpu_driver);
}

static void __exit pwm_tpu_exit(void)
{
	platform_driver_unregister(&pwm_tpu_driver);
}

MODULE_AUTHOR("Markus Pietrek");
MODULE_DESCRIPTION("PWM/Timer Pulse Unit TPU control");
MODULE_LICENSE("GPL");

arch_initcall(pwm_tpu_init);
module_exit(pwm_tpu_exit);
