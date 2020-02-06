/*
 * em_module.c - Support for board/module identification
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
 * References:
 *   [1] U-Boot/extbsp/cmd_nvram/lib/include/nvram.h
 *
 **/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/em_module.h>

/* parses the command line option hw_product_type, hw_serial_nr .... */
#define MODULE_FUNC(x) \
static char __em_module_##x[20]; \
const char *em_module_##x(void)		\
{ \
	return __em_module_##x; \
} \
EXPORT_SYMBOL(__em_module_##x); \
 \
static int __init em_module_setup_##x(char *options) \
{ \
        if (options != NULL) { \
		int size = sizeof(__em_module_##x); \
		strncpy(__em_module_##x, options, size); \
		__em_module_##x[size-1]=0; \
	} \
 \
        return 0; \
} \
early_param(#x, em_module_setup_##x);

MODULE_FUNC(hw_product_type);
MODULE_FUNC(hw_revision);
MODULE_FUNC(hw_patch_level);
MODULE_FUNC(hw_serial_nr);

/**
 * em_module_hw_revision_as_hex - returns hw_revision as a number
 *
 * Interprets hw_revision as a hex number for >= comparisions.
 * 0 means hardware revision not parsable. Either the string is empty or contains invalid numbers */
int em_module_hw_revision_as_hex(void)
{
	/* skip "r1" from "r1a" */
	return simple_strtol(&__em_module_hw_revision[1], NULL, 16);
}

