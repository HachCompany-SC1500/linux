/*
 * baseboard_io_display.c
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
 **/

#include <linux/i2c/pca9530_bl.h>

static struct pca9530bl_platform_data pca9530_pdata = {
	.invert = 1,
};

static struct pcf857x_platform_data pcf8574_info_display_adapter;  /* will be initialised at runtime */

static int __init hico7723_setup_display_i2c(void)
{
	static struct i2c_board_info i2c_devices[] = {
		/* PWM and display configuration on the display adapater. Are not
		 * present on NL6448, UMSH8272 and TX14 displays. */
		{       I2C_BOARD_INFO("pcf857x",     0x3A),
			.platform_data = &pcf8574_info_display_adapter }, /*   display_adapter might not be present */
		{	I2C_BOARD_INFO("pca9530",     0x60),
			.platform_data = &pca9530_pdata },
	};

        pcf8574_info_display_adapter.gpio_base = last_gpio;

	return i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
}

static int __init hico7723_backlight_polarity_setup(char *options)
{
        if (options == NULL)
                return 0;

	pca9530_pdata.invert = simple_strtoul(options, NULL, 0);

        return 1;
}
__setup("backlight_polarity=", hico7723_backlight_polarity_setup);
