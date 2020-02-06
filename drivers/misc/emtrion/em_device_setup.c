/*
 * em_device_setup.c
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
#include <linux/em_device_setup.h>

int __init em_devices_setup(const struct device_setup devices[], int len)
{
	int res;
	int i;

	for (i=0; i < len; i++) {
		res = devices[i].fn();
		if (res<0) {
			pr_err("setup failed at function %s\n", devices[i].name);
			return res;
		}
	}

	return 0;
}
