/*
 * gpio_poweroff.c
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_poweroff.h>

static void gpio_poweroff(void)
{
	/* we don't have any private data we can use, therefore we need to look for our platform device ourself */
        struct device *dev = bus_find_device_by_name(&platform_bus_type, NULL, "gpio_poweroff.0");
	struct gpio_poweroff_platform_data *pdata;

	if (!dev)
		return;

	pdata = to_platform_device(dev)->dev.platform_data;

	gpio_set_value(pdata->gpio, pdata->active_level);
}

static int __devinit gpio_poweroff_probe(struct platform_device *pdev)
{
	int ret;
	struct gpio_poweroff_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	ret = gpio_request(pdata->gpio, dev_name(&pdev->dev));
	if (ret<0)
		goto error_gpio;

	if (pm_power_off) {
		/* If it is already defined, there is a failure in the
		   platform configuration */
		dev_err(&pdev->dev, "pm_power_off already set to %p\n",
			 pm_power_off);
		goto error_handler;
	}

	pm_power_off = gpio_poweroff;
	gpio_direction_output(pdata->gpio,
			      !pdata->active_level);

	return 0;

error_handler:
	gpio_free(pdata->gpio);

error_gpio:
	return ret;
}

static int __devexit gpio_poweroff_remove(struct platform_device *pdev)
{
	struct gpio_poweroff_platform_data *pdata = pdev->dev.platform_data;

	gpio_free(pdata->gpio);

	return 0;
}

static struct platform_driver gpio_poweroff_driver = {
	.probe		= gpio_poweroff_probe,
	.remove		= gpio_poweroff_remove,
	.driver		= {
		.name	= "gpio_poweroff",
		.owner	= THIS_MODULE,
	},
};

static int __init gpio_poweroff_init(void)
{
	return platform_driver_register(&gpio_poweroff_driver);
}

static void __exit gpio_poweroff_exit(void)
{
	platform_driver_unregister(&gpio_poweroff_driver);
}

MODULE_AUTHOR("Markus Pietrek");
MODULE_DESCRIPTION("GPIO poweroff handler");
MODULE_LICENSE("GPL");

module_init(gpio_poweroff_init);
module_exit(gpio_poweroff_exit);
