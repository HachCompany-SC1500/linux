/*
 * baseboard_io_sd_mmc.c
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

#include <linux/mfd/sh_mobile_sdhi.h>

EM_CMDLINE_FLAG(hico7723_skip_sdhi0);
EM_CMDLINE_FLAG(hico7723_skip_sdhi1);

EM_CMDLINE_FLAG(hico7723_use_sdhi0_dma);
EM_CMDLINE_FLAG(hico7723_use_sdhi1_dma);

static struct sh_mobile_sdhi_info sh_mobile_sdhi0_data = {
	.dma_slave_tx	= SHDMA_SLAVE_SDHI0_TX,
	.dma_slave_rx	= SHDMA_SLAVE_SDHI0_RX,
};

static struct resource sdhi0_cn3_resources[] = {
	[0] = {
		.name	= "SDHI0",
		.start	= 0x04ce0000,
		.end	= 0x04ce01ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 100,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device sdhi0_cn3_device = {
	.name		= "sh_mobile_sdhi",
	.id             = 0, /* "sdhi0" clock */
	.num_resources	= ARRAY_SIZE(sdhi0_cn3_resources),
	.resource	= sdhi0_cn3_resources,
	.archdata = {
		.hwblk_id = HWBLK_SDHI0,
	},
};

static struct sh_mobile_sdhi_info sh_mobile_sdhi1_data = {
	.dma_slave_tx	= SHDMA_SLAVE_SDHI1_TX,
	.dma_slave_rx	= SHDMA_SLAVE_SDHI1_RX,
};

static struct resource sdhi1_cn7_resources[] = {
	[0] = {
		.name	= "SDHI1",
		.start	= 0x04cf0000,
		.end	= 0x04cf01ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 23,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device sdhi1_cn7_device = {
	.name		= "sh_mobile_sdhi",
	.id             = 1, /* "sdhi1" clock */
	.num_resources	= ARRAY_SIZE(sdhi1_cn7_resources),
	.resource	= sdhi1_cn7_resources,
	.archdata = {
		.hwblk_id = HWBLK_SDHI1,
	},
};


static int __init hico7723_setup_sdhi0(void)
{
	static const int gpios[] __initdata = {
		GPIO_FN_SDHI0CD_PTD,
		GPIO_FN_SDHI0WP_PTD,
		GPIO_FN_SDHI0D3_PTD,
		GPIO_FN_SDHI0D2_PTD,
		GPIO_FN_SDHI0D1_PTD,
		GPIO_FN_SDHI0D0_PTD,
		GPIO_FN_SDHI0CMD_PTD,
		GPIO_FN_SDHI0CLK_PTD,
	};
	int res;

	if (hico7723_skip_sdhi0)
		return 0;

        res=gpio_fn_request_array(gpios, ARRAY_SIZE(gpios), "sdhi0");
	if (res<0)
		return res;

	if (hico7723_use_sdhi0_dma)
		sdhi0_cn3_device.dev.platform_data = &sh_mobile_sdhi0_data;

	__raw_writew((__raw_readw(PSELA) & ~0x0C00) | 0x0400, PSELA);
	__raw_writew(__raw_readw(MSELCRB) | 0x0008, MSELCRB); /* PTD */
	__raw_writew(0x0000, PORT_PDCR); /* SDHI0 */

        return platform_device_register(&sdhi0_cn3_device);
}

static int __init hico7723_setup_sdhi1(void)
{
	static const int gpios[] __initdata = {
		GPIO_FN_SDHI1CD,
		GPIO_FN_SDHI1WP,
		GPIO_FN_SDHI1D3,
		GPIO_FN_SDHI1D2,
		GPIO_FN_SDHI1D1,
		GPIO_FN_SDHI1D0,
		GPIO_FN_SDHI1CMD,
		GPIO_FN_SDHI1CLK,
	};
	int res;

	if (hico7723_skip_sdhi1)
		return 0;

        res=gpio_fn_request_array(gpios, ARRAY_SIZE(gpios), "sdhi1");
	if (res<0)
		return res;

	if (hico7723_use_sdhi1_dma)
		sdhi1_cn7_device.dev.platform_data = &sh_mobile_sdhi1_data;

	__raw_writew((__raw_readw(PSELA) & ~0x0C00) | 0x0400, PSELA);
	__raw_writew(0x0000, PORT_PCCR); /* SDHI1 */

        return platform_device_register(&sdhi1_cn7_device);
}
