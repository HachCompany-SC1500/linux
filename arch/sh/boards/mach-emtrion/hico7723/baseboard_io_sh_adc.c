/*
 * baseboard_io_sh_adc.c
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

#include "../drivers/hwmon/sh_adc.h"

EM_CMDLINE_FLAG(hico7723_skip_adc);

static struct sh_adc_platform_data hico7723_adc_platform_data = {
	.voltage_reference = 3300,
	.ports = SH_ADC_PORT_A | SH_ADC_PORT_B | SH_ADC_PORT_C | SH_ADC_PORT_D,
};

static struct resource hico7723_adc_resources[] = {
	[0] = {
		.name	= "sh_adc",
		.start	= 0xa4610000,
		.end	= 0xa4610009,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start  = evt2irq(IRQEVT_ADC_ADI),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device hico7723_adc_device = {
	.name		= "sh_adc",
	.num_resources	= ARRAY_SIZE(hico7723_adc_resources),
	.resource	= hico7723_adc_resources,
	.dev		= {
		.platform_data	= &hico7723_adc_platform_data,
	},
	.archdata = {
		.hwblk_id = HWBLK_ADC,
	},
};

static int __init hico7723_setup_adc(void)
{
	static const int gpios[] __initdata = {
		GPIO_FN_AN3,
		GPIO_FN_AN2,
		GPIO_FN_AN1,
		GPIO_FN_AN0,
	};
	int res;

	if (hico7723_skip_adc)
		return 0;

	res = gpio_fn_request_array(gpios, ARRAY_SIZE(gpios), "adc");
	if (res<0)
		return res;

	return platform_device_register(&hico7723_adc_device);
}
