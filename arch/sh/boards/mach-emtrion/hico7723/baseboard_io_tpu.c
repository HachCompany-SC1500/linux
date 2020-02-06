/*
 * baseboard_io_tpu.c
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

#include <linux/pwm_backlight.h>

#define TPU_PORTS 4

static struct resource hico7723_pwm_tpu_resources[] = {
	[0] = {
		.start	= 0xA4C90000,
		.end	= 0xA4C900F6,
		.flags	= IORESOURCE_MEM,
        },
};
static struct platform_device hico7723_pwm_tpu = {
	.name		= "pwm_tpu",
	.num_resources	= ARRAY_SIZE(hico7723_pwm_tpu_resources),
	.resource	= hico7723_pwm_tpu_resources,
};

/* The HiCO.DIMM7723 is a module, running on different boards. Give a chance so the user can
   customize the board, e.g. use GPIOs instead of SDHI */
/* each used port will set the bit #port */
static int hico7723_use_pwm_tpu_ports;

static struct platform_pwm_backlight_data hico7723_pwm_tpu_port_data[TPU_PORTS];
/* will be filled with all platform devices that are configured for pwm_tpu */
static struct platform_device* hico7723_pwm_tpu_devices[1+TPU_PORTS];

#define MK(index) { \
		.name = "pwm-backlight", \
		.id = index, \
		.dev = {.platform_data = &hico7723_pwm_tpu_port_data[index]}, \
		  }

static struct platform_device hico7723_pwm_tpu_backlight[TPU_PORTS] = {
	MK(0),
	MK(1),
	MK(2),
	MK(3),
};
#undef MK

static int __init hico7723_setup_pwm_tpu(void)
{
	static const int gpios[TPU_PORTS] __initdata = {
		GPIO_FN_TPUTO0,
		GPIO_FN_TPUTO1,
		GPIO_FN_TPUTO2,
		GPIO_FN_TPUTO3,
	};
	int i = 0;
	int j;
	int res;

	if (!hico7723_use_pwm_tpu_ports)
		/* not needed */
		return 0;

	hico7723_pwm_tpu_devices[i++] = &hico7723_pwm_tpu;

	/* add all tpu devices for which a configuration has been given */
	for (j=0; j<TPU_PORTS; j++) {
		res = gpio_request(gpios[j], "pwm_tpu");
		if (res<0)
			goto error;

		if (hico7723_use_pwm_tpu_ports & (1<<j))
			hico7723_pwm_tpu_devices[i++] = &hico7723_pwm_tpu_backlight[j];
	}

	res = platform_add_devices(hico7723_pwm_tpu_devices, i);
	if (res < 0)
		goto error;

	/* configure the pins to be used by TPUTO */
	__raw_writew((__raw_readw(PSELA) & ~0x000C) | 0x0004, PSELA);

	return 0;

error:
        return res;
}

static int __init hico7723_cmdline_pwm_tpu_setup(char *options)
{
	int id;
	int max_brightness;
	int dft_brightness;
	int pwm_period_ns;

        if (options == NULL)
                return 0;

	if (sscanf(options, "%i:%i:%i:%i", &id, &max_brightness, &dft_brightness, &pwm_period_ns) != 4)
		goto error;

	if (id >= TPU_PORTS)
		/* out of range */
		goto error;

	hico7723_use_pwm_tpu_ports |= 1<<id;

	/* configure pwm_tpu_port */
	hico7723_pwm_tpu_port_data[id].pwm_id = id;
	hico7723_pwm_tpu_port_data[id].max_brightness = max_brightness;
	hico7723_pwm_tpu_port_data[id].dft_brightness = dft_brightness;
	hico7723_pwm_tpu_port_data[id].pwm_period_ns  = pwm_period_ns;

        return 1;

error:
	pr_err(KERN_ERR "pwm_tpu: Invalid arguments in %s", options);
	return 0;
}
__setup( "pwm_tpu=", hico7723_cmdline_pwm_tpu_setup );

