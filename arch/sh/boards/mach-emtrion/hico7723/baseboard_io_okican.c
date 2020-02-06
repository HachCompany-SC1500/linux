/*
 * baseboard_io_okican.c
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

#include <linux/okican.h>

#define PA_CAN0_BASE	(0x10000000)
#define PA_CAN1_BASE	(0x10010100) /* the address lines >=A11 are not connected. So we simulate for the second chip to be mappend on a different 64kB page */
#define PA_CAN_SIZE	(0x100)

static struct resource hico7723_okican0_resources[] = {
	[0] = {
		.start	= PA_CAN0_BASE,
		.end	= PA_CAN0_BASE+PA_CAN_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= evt2irq(IRQEVT_IRQ2),
		.end	= evt2irq(IRQEVT_IRQ2),
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
	},
};

static struct platform_device hico7723_okican0_device = {
	.name		= "okican",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(hico7723_okican0_resources),
	.resource	= hico7723_okican0_resources,
};

static int __init hico7723_setup_okican0(void)
{
	return platform_device_register(&hico7723_okican0_device);
}

static struct resource hico7723_okican1_resources[] = {
	[0] = {
		.start	= PA_CAN1_BASE,
		.end	= PA_CAN1_BASE+PA_CAN_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= evt2irq(IRQEVT_IRQ2),
		.end	= evt2irq(IRQEVT_IRQ2),
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
	},
};

static struct platform_device hico7723_okican1_device = {
	.name		= "okican",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(hico7723_okican1_resources),
	.resource	= hico7723_okican1_resources,
};

static int __init hico7723_setup_okican1(void)
{
	return platform_device_register(&hico7723_okican1_device);
}
