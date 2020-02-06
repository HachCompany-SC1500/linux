/*
 * em_baseboard.c - Support for emtrion baseboards
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
 *   [1] http://em-srv3-kaha/edit.py?q=2831
 *
 **/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/em_baseboard.h>
#include <linux/module.h>

unsigned int __em_baseboard_code;
EXPORT_SYMBOL(__em_baseboard_code);

static int __init em_baseboard_code_setup(char *options)
{
        if (options != NULL)
		__em_baseboard_code = simple_strtoul(options, NULL, 16);

        return 0;
}
early_param("hw_base_board_code", em_baseboard_code_setup);
