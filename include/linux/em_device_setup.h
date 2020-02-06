/*
 * em_device_setup.h - some simplifications for device handling and initialization
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

#ifndef _LINUX_EM_DEVICE_SETUP_H
#define _LINUX_EM_DEVICE_SETUP_H

/* provides a command line interface so device initialization can be skipped by commandline */
#define EM_CMDLINE_FLAG(name) \
	static int name;    \
	static int __init name ## _cmdline_setup(char *options)	\
	{								\
		name = 1;						\
		return 1;						\
	}								\
	__setup(#name, name ## _cmdline_setup);

/* initialization of an array of devices */
typedef int (setup_fnc_t) (void);
#define EM_MKS(_fn) {.fn = _fn, .name = #_fn }
struct device_setup {
	setup_fnc_t *fn;
	const char  *name;
};

extern int __init em_devices_setup(const struct device_setup devices[], int len);

#endif
