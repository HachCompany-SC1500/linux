/*
 * arch/sh/boards/mach-emtrion/directfb.c
 *
 * Copyright (c) 2009 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 * Revision:    $Revision$
 * Description: Parses directfb=hw commandline
 *
 **/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/page.h>

static int __init early_directfb_setup(char *options)
{
        if ((options != NULL) && !strcmp (options, "hw")) {
                pr_notice("DirectFB HW Acceleration enabled, reserving 8MB RAM\n");
                memory_limit = CONFIG_MEMORY_SIZE - 8 * 1024 * 1024;
        }

        return 0;
}

early_param("directfb", early_directfb_setup);
