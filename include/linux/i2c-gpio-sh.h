/*
 * i2c-gpio-sh.h
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

#ifndef _LINUX_I2C_GPIO_SH_H
#define _LINUX_I2C_GPIO_SH_H

/**
 * struct i2c_gpio_platform_data - Platform-dependent data for i2c-gpio
 * @sda.pin: GPIO pin ID to use for SDA
 * @sda.data: register of the data of I/O port
 * @sda.pfc:  register of the PIN function controller of I/O port
 * @sda.bit: bit  of the I/O port
 * @scl.pin: GPIO pin ID to use for SDA
 * @scl.data: register of the data of I/O port
 * @scl.pfc:  register of the PIN function controller of I/O port
 * @scl.bit: bit  of the I/O port
 * @udelay: signal toggle delay. SCL frequency is (500 / udelay) kHz
 * @timeout: clock stretching timeout in jiffies. If the slave keeps
 *	SCL low for longer than this, the transfer will time out.
 */
struct i2c_gpio_sh_platform_data {
	struct {
		unsigned int	pin;
		u32             pfc;
		u32             data;
		int             bit;
	} sda, scl;

	int		udelay;
	int		timeout;
};

#endif
