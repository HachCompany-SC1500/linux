/*
 * baseboard_io_spi.c
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

#include <linux/spi/spi.h>
#include <linux/spi/sh_msiof.h>

EM_CMDLINE_FLAG(hico7723_skip_spi);

/* MSIOF0 */
static struct sh_msiof_spi_info hico7723_spi_msiof0_data = {
	.num_chipselect = 1,
};

static struct resource hico7723_spi_msiof0_resources[] = {
	[0] = {
		.name	= "MSIOF0",
		.start	= 0xa4c40000,
		.end	= 0xa4c40063,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 84,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device hico7723_spi_msiof0_device = {
	.name		= "spi_sh_msiof",
	.id		= 0, /* MSIOF0 */
	.dev = {
		.platform_data = &hico7723_spi_msiof0_data,
	},
	.num_resources	= ARRAY_SIZE(hico7723_spi_msiof0_resources),
	.resource	= hico7723_spi_msiof0_resources,
	.archdata = {
		.hwblk_id = HWBLK_MSIOF0,
	},
};

static int __init hico7723_setup_spi(void)
{
	static const int gpios[] __initdata = {
		GPIO_FN_MSIOF0_PTF_TSYNC,
		GPIO_FN_MSIOF0_PTF_TSCK,
		GPIO_FN_MSIOF0_PTF_RXD,
		GPIO_FN_MSIOF0_PTF_TXD,
	};
	int res;

	if (hico7723_skip_spi)
		return 0;

	res = gpio_fn_request_array(gpios, ARRAY_SIZE(gpios), "spi");
	if (res<0)
		return res;

	return platform_device_register(&hico7723_spi_msiof0_device);
}

