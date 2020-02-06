/*
 * baseboard.c - Provides baseboard setup for baseboards supported by HiCO.DIMM7723
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/em_baseboard.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c/pcf857x.h>
#include <linux/i2c-gpio-sh.h>
#include <linux/gpio_poweroff.h>
#include <linux/em_device_setup.h>
#include <linux/em_common.h>
#include <cpu/sh7723.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "../include/hico7723.h"
#include "../include/sh7723.h"

#define INFO(board) pr_info("Configured for baseboard: %s\n", board)

static int last_gpio;

/* we have split it up in different files to not loose the overview */

#include "baseboard_io_sh_adc.c"
#include "baseboard_io_tpu.c"
#include "baseboard_io_camera.c"
#include "baseboard_io_display.c"
#include "baseboard_io_spi.c"
#include "baseboard_io_sd_mmc.c"
#include "baseboard_io_okican.c"


/*********** Baseboard ***********/

static int __init hico7723_register_dimmecobase(void)
{
	static struct i2c_board_info baseboard_i2c_devices[] = {
		{	I2C_BOARD_INFO("tlv320aic23", 0x1B),   },
	};
	static const struct device_setup base_devices[] __initdata = {
		EM_MKS(hico7723_setup_sdhi0),
		EM_MKS(hico7723_setup_adc),
		EM_MKS(hico7723_setup_spi),
 		EM_MKS(hico7723_setup_display_i2c),
	};
	int res;

	INFO("HiCO.DIMM-ECOBase");

	hico7723_usbh_oc_connected = 1;

	res = i2c_register_board_info(0, baseboard_i2c_devices,
				      ARRAY_SIZE(baseboard_i2c_devices));

	if (!res)
		res = em_devices_setup(base_devices, ARRAY_SIZE(base_devices));

	return res;
}

static int __init hico7723_register_dimmbase(void)
{
	static struct gpio_poweroff_platform_data gpio_poweroff_pdata = {
		.active_level = 0,
	};
	static struct platform_device gpio_poweroff_device = {
		.name = "gpio_poweroff",
		.id  = 0,
		.dev = {
			.platform_data = &gpio_poweroff_pdata,
		},
	};
	static struct pcf857x_platform_data pcf8574_info_dimmbase; /* will be initialised at runtime */
	static struct i2c_board_info baseboard_i2c_devices[] = {
		{       I2C_BOARD_INFO("pcf8574",     0x39),
			.platform_data = &pcf8574_info_dimmbase, },
		{	I2C_BOARD_INFO("tlv320aic23", 0x1B),   },
	};
	static const struct device_setup base_devices[] __initdata = {
		EM_MKS(hico7723_setup_sdhi0),
		EM_MKS(hico7723_setup_sdhi1),
		EM_MKS(hico7723_setup_pwm_tpu),
		EM_MKS(hico7723_setup_camera),
		EM_MKS(hico7723_setup_adc),
		EM_MKS(hico7723_setup_spi),
		EM_MKS(hico7723_setup_display_i2c),
	};
	static struct platform_device* baseboard_devices[] = {
		&gpio_poweroff_device,
		/* plugable codecs need to come first */
		&hico7723_camera_mt9v022,
		&hico7723_camera_ov772x,

		&hico7723_camera_adv7180, /* built-in codec needs to be last*/
	};
	int res;

	INFO("HiCO.DIMM-Base");

	hico7723_usbh_oc_connected = 1;

        pcf8574_info_dimmbase.gpio_base = last_gpio;
	last_gpio += 8;

	gpio_poweroff_pdata.gpio = pcf8574_info_dimmbase.gpio_base+5;

	res = i2c_register_board_info(0, baseboard_i2c_devices,
				      ARRAY_SIZE(baseboard_i2c_devices));

	if (!res)
		res = em_devices_setup(base_devices, ARRAY_SIZE(base_devices));

	if (!res)
		res = platform_add_devices(baseboard_devices, ARRAY_SIZE(baseboard_devices));

	return res;
}


static int __init hico7723_register_hachlange(void)
{
	static struct gpio_poweroff_platform_data gpio_poweroff_pdata = {
		.active_level = 0,
	};
	static struct platform_device gpio_poweroff_device = {
		.name = "gpio_poweroff",
		.id  = 0,
		.dev = {
			.platform_data = &gpio_poweroff_pdata,
		},
	};
	static struct pcf857x_platform_data pcf8574_info_dimmbase; /* will be initialised at runtime */
	static struct i2c_board_info baseboard_i2c_devices[] = {
		{       I2C_BOARD_INFO("pcf8574",     0x39),
			.platform_data = &pcf8574_info_dimmbase, },
		{	I2C_BOARD_INFO("ssm2518",     0x34),   },       // first ot the supported audio codecs
		{	I2C_BOARD_INFO("tlv320aic23", 0x1B),   },       // second audio codec
	};
	static const struct device_setup base_devices[] __initdata = {
		EM_MKS(hico7723_setup_pwm_tpu),
		EM_MKS(hico7723_setup_camera),
		EM_MKS(hico7723_setup_adc),
		EM_MKS(hico7723_setup_spi),
		EM_MKS(hico7723_setup_display_i2c),
	};
	static struct platform_device* baseboard_devices[] = {
          &gpio_poweroff_device,
          &hico7723_camera_mt9v022,
          &hico7723_camera_ov772x,
          &hico7723_camera_adv7180, /* built-in codec needs to be last*/
	};
	int res;

        pcf8574_info_dimmbase.gpio_base = last_gpio;
	last_gpio += 8;

	gpio_poweroff_pdata.gpio = pcf8574_info_dimmbase.gpio_base+5;

	res = i2c_register_board_info(0, baseboard_i2c_devices,
				      ARRAY_SIZE(baseboard_i2c_devices));

	if (!res)
		res = em_devices_setup(base_devices, ARRAY_SIZE(base_devices));

	if (!res)
		res = platform_add_devices(baseboard_devices, ARRAY_SIZE(baseboard_devices));


	/* enable clock for sound chip */
	res = gpio_request_one(GPIO_PTZ5, GPIOF_OUT_INIT_HIGH, "en-12mhz");
	gpio_free(GPIO_PTZ5);

	return res;
}


static int __init hico7723_register_pellenc(void)
{
	static struct pcf857x_platform_data pca9555_info_pellenc; /* will be initialised at runtime */
	static struct i2c_gpio_sh_platform_data i2c_gpio_sh_pdata = {
		.sda = {
			.pin  = 163,	/* GPIO_2/PTZ6 */
			.pfc  = PORT_PZCR,
			.data = PORT_PZDR,
			.bit  = 6,
		},
		.scl = {
			.pin  = 51,	/* GPIO_4/PTG0 */
			.pfc  = PORT_PGCR,
			.data = PORT_PGDR,
			.bit  = 0,
		 },
		.udelay  = 2,	/* 250kHz */
	};
	static struct platform_device i2c_gpio_device = {
		.name = "i2c-gpio-sh",
		.id  = 1,	/* secondary bus */
		.dev = {
			.platform_data = &i2c_gpio_sh_pdata,
		},
	};
	static struct i2c_board_info baseboard_i2c_devices[] = {
		{       I2C_BOARD_INFO("pca9555", 0x24),
			.platform_data = &pca9555_info_pellenc, },
		{	I2C_BOARD_INFO("wm8974",  0x1A),   },
	};
	static const struct device_setup base_devices[] __initdata = {
		EM_MKS(hico7723_setup_sdhi0),
		EM_MKS(hico7723_setup_sdhi1),
		EM_MKS(hico7723_setup_pwm_tpu),
		EM_MKS(hico7723_setup_camera),
		EM_MKS(hico7723_setup_display_i2c),
 		EM_MKS(hico7723_setup_okican0),
 		EM_MKS(hico7723_setup_okican1),
	};
	static struct platform_device* baseboard_devices[] = {
		/* plugable codecs need to come first */
		&hico7723_camera_mt9v022,
		&hico7723_camera_ov772x,

		&hico7723_camera_adv7180, /* built-in codec needs to be last*/
		&i2c_gpio_device,
	};
	int res;

	INFO("pellenc");

	hico7723_usbh_oc_connected = 1;

        pca9555_info_pellenc.gpio_base = last_gpio;
	last_gpio += 16;

	/* we have an USB high-speed capable HUB. No need for limitation */
	hico7723_usb_disable_highspeed = 0;
	pr_info("USB Speed limitiation forced to high-speed\n");

	adv7180_max_input = 3;	/* AN4 */

	/* configure CS4. The okican requires at least 500ns wait cycles.
	   The SH7723 can at maximum wait for 360ns internally, so the PLD will extend
	   the wait to at least 500ns and we will wait for the external wait */
	__raw_writel(CSxBCR_BSZ_8       |
		     CSxBCR_TYPE_NORMAL |
		     CSxBCR_IWW_4       |
		     CSxBCR_IWRWD_4     |
		     CSxBCR_IWRWS_4     |
		     CSxBCR_IWRRD_4, CS4BCR);
	__raw_writel(CSxWCR_WR_14, CS4WCR);

	/* PTW2 is configured for BS# by default, we need it as interrupt */
	__raw_writew((__raw_readw(PORT_PSELD) & ~PSELD_PTW2_MASK) | PSELD_PTW2_IRQ, PORT_PSELD);

	res = i2c_register_board_info(0, baseboard_i2c_devices,
				      ARRAY_SIZE(baseboard_i2c_devices));

	if (!res)
		res = em_devices_setup(base_devices, ARRAY_SIZE(base_devices));

	if (!res)
		res = platform_add_devices(baseboard_devices, ARRAY_SIZE(baseboard_devices));

	return res;
}

static void __init hico7723_init_irq_pellenc(void)
{
	set_irq_type(evt2irq(IRQEVT_IRQ2), IRQ_TYPE_LEVEL_LOW);	/* okican */
}

int __init em_baseboard_register(void)
{
	extern struct pinmux_info sh7723_pinmux_info;
	int res;

	last_gpio = sh7723_pinmux_info.last_gpio+1;

	if (em_baseboard_is_dimmecobase())
		res = hico7723_register_dimmecobase();
	else if (em_baseboard_is_dimmbase())
		res = hico7723_register_dimmbase();
	else if (em_baseboard_is_pellenc())
		res = hico7723_register_pellenc();
	else if (em_baseboard_is_hachlange())
		res = hico7723_register_hachlange();
	else {
		INFO("\n");
		INFO("none\n");
		res = 0;	/* unknown baseboard */
	}

	return res;
}

void __init em_baseboard_init_irq(void)
{
	if (em_baseboard_is_pellenc())
		hico7723_init_irq_pellenc();
}
