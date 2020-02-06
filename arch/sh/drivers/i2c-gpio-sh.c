/*
 * i2c-gpio-sh.c - a performance improved GPIO bitbanging driver
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
 * Description: i2c-gpio works well when an I2C device is seldomly used.
 *              But using GPIOLIB it is quite slow. On an idle SH7723,
 *		frequencies not more than 20kHz can be achieved with a CPU
 *		Therefore, i2c-gpio is unsuitable for a
 *		frequent access. This driver improves the access to GPIOs
 *		by avoiding GPIOLIB except for reserving the ports
 *
 *              Derived from i2c-gpio.c
 *
 **/

#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio-sh.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <asm/io.h>
#include <asm/gpio.h>

#define PFC_OUT		1
#define PFC_IN		3
#define PFC_MASK	3

/* defined in drivers/sh/pfc.c */
extern spinlock_t pfc_gpio_lock;

static void i2c_gpio_sh_set_dir(u32 pfc, u32 data, int bit, int in)
{
	unsigned long flags;
	u16 tmp;

	spin_lock_irqsave(&pfc_gpio_lock, flags);

	if (!in)
		__raw_writeb(__raw_readb(data) & ~(1<<bit), data);
	tmp  = __raw_readw(pfc);
	tmp &= ~(PFC_MASK<<(bit*2));
	tmp |= (in ? PFC_IN : PFC_OUT)<<(bit*2);
	__raw_writew(tmp, pfc);

	spin_unlock_irqrestore(&pfc_gpio_lock, flags);
}

static void i2c_gpio_sh_set_val(u32 data, int bit, int val)
{
	unsigned long flags;
	u8 tmp;

	spin_lock_irqsave(&pfc_gpio_lock, flags);

	tmp  = __raw_readb(data);
	if (val)
		tmp |= 1<<bit;
	else
		tmp &= ~(1<<bit);
	__raw_writeb(tmp, data);

	spin_unlock_irqrestore(&pfc_gpio_lock, flags);
}

static int i2c_gpio_sh_get_val(u32 reg, int bit)
{
	unsigned long flags;
	int val;

	spin_lock_irqsave(&pfc_gpio_lock, flags);

	val = !!(__raw_readb(reg) & (1<<bit));

	spin_unlock_irqrestore(&pfc_gpio_lock, flags);

	return val;
}

/* Toggle SDA by changing the direction of the pin */
static void i2c_gpio_sh_setsda_dir(void *data, int state)
{
	struct i2c_gpio_sh_platform_data *pdata = data;

	i2c_gpio_sh_set_dir(pdata->sda.pfc, pdata->sda.data, pdata->sda.bit, state);
}

/*
 * Toggle SCL by changing the output value of the pin. This is used
 * for pins that are configured as open drain and for output-only
 * pins. The latter case will break the i2c protocol, but it will
 * often work in practice.
 */
static void i2c_gpio_sh_setscl_val(void *data, int state)
{
	struct i2c_gpio_sh_platform_data *pdata = data;

	i2c_gpio_sh_set_val(pdata->scl.data, pdata->scl.bit, state);
}

static int i2c_gpio_sh_getsda(void *data)
{
	struct i2c_gpio_sh_platform_data *pdata = data;

	return i2c_gpio_sh_get_val(pdata->sda.data, pdata->sda.bit);
}

static int __devinit i2c_gpio_sh_probe(struct platform_device *pdev)
{
	struct i2c_gpio_sh_platform_data *pdata;
	struct i2c_algo_bit_data *bit_data;
	struct i2c_adapter *adap;
	int ret;

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -ENXIO;

	ret = -ENOMEM;
	adap = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (!adap)
		goto err_alloc_adap;
	bit_data = kzalloc(sizeof(struct i2c_algo_bit_data), GFP_KERNEL);
	if (!bit_data)
		goto err_alloc_bit_data;

	/* no other should be able to use these pins even when we access
	   it directly */
	ret = gpio_request(pdata->sda.pin, "i2c-gpio-sh:sda");
	if (ret)
		goto err_request_sda;
	ret = gpio_request(pdata->scl.pin, "i2c-gpio-sh:scl");
	if (ret)
		goto err_request_scl;

	/* use gpio_direction, as this also configures the PSEL register */
	gpio_direction_input(pdata->sda.pin);
	bit_data->setsda = i2c_gpio_sh_setsda_dir;

	gpio_direction_output(pdata->scl.pin, 1);
	bit_data->setscl = i2c_gpio_sh_setscl_val;

	bit_data->getsda = i2c_gpio_sh_getsda;

	bit_data->udelay = pdata->udelay;

	if (pdata->timeout)
		bit_data->timeout = pdata->timeout;
	else
		bit_data->timeout = HZ / 10;		/* 100 ms */

	bit_data->data = pdata;

	adap->owner = THIS_MODULE;
	snprintf(adap->name, sizeof(adap->name), "i2c-gpio-sh%d", pdev->id);
	adap->algo_data = bit_data;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.parent = &pdev->dev;

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	adap->nr = (pdev->id != -1) ? pdev->id : 0;
	ret = i2c_bit_add_numbered_bus(adap);
	if (ret)
		goto err_add_bus;

	platform_set_drvdata(pdev, adap);

	dev_info(&pdev->dev,
		 "using pins %u (SDA) and %u (SCL, no clock stretching)\n",
		 pdata->sda.pin, pdata->scl.pin);

	return 0;

err_add_bus:
	gpio_free(pdata->scl.pin);
err_request_scl:
	gpio_free(pdata->sda.pin);
err_request_sda:
	kfree(bit_data);
err_alloc_bit_data:
	kfree(adap);
err_alloc_adap:
	return ret;
}

static int __devexit i2c_gpio_sh_remove(struct platform_device *pdev)
{
	struct i2c_gpio_sh_platform_data *pdata;
	struct i2c_adapter *adap;

	adap = platform_get_drvdata(pdev);
	pdata = pdev->dev.platform_data;

	i2c_del_adapter(adap);
	gpio_free(pdata->scl.pin);
	gpio_free(pdata->sda.pin);
	kfree(adap->algo_data);
	kfree(adap);

	return 0;
}

static struct platform_driver i2c_gpio_sh_driver = {
	.driver		= {
		.name	= "i2c-gpio-sh",
		.owner	= THIS_MODULE,
	},
	.probe		= i2c_gpio_sh_probe,
	.remove		= __devexit_p(i2c_gpio_sh_remove),
};

static int __init i2c_gpio_sh_init(void)
{
	int ret;

	ret = platform_driver_register(&i2c_gpio_sh_driver);
	if (ret)
		printk(KERN_ERR "i2c-gpio-sh: probe failed: %d\n", ret);

	return ret;
}
module_init(i2c_gpio_sh_init);

static void __exit i2c_gpio_sh_exit(void)
{
	platform_driver_unregister(&i2c_gpio_sh_driver);
}
module_exit(i2c_gpio_sh_exit);

MODULE_AUTHOR("Markus Pietrek <markus.pietrek@emtrion.de>");
MODULE_DESCRIPTION("SH optimized bitbanging I2C driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-gpio-sh");
