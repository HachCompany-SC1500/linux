/*
 * em_common.c - common board initialization for emtrion boards
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
 * Author: Markus Pietrek
 *
 **/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/em_common.h>
#include <asm/machvec.h>

char em_hw_revision[16];
EXPORT_SYMBOL(em_hw_revision);

static int __init em_hw_revision_setup(char *options)
{
	if (strlen(options) > ARRAY_SIZE(em_hw_revision)-1)
		return 0;

	strcpy(em_hw_revision, options);

	return 0;
}
early_param("hw_revision", em_hw_revision_setup);

int gpio_fn_request_array(const int gpios[], int size, const char *label)
{
        int i;

        for (i=0; i < size; i++) {
                if (gpio_request(gpios[i], label)) {
                        /* print error but continue booting */
                        pr_err("GPIO %u (%s) already reserved.\n", gpios[i], label );
			return -EBUSY;
		}
        }

	return 0;
}
EXPORT_SYMBOL(gpio_fn_request_array);

int gpio_fn_free_array(const int gpios[], int size)
{
        int i;

        for (i=0; i < size; i++)
		gpio_free(gpios[i]);

	return 0;
}
EXPORT_SYMBOL(gpio_fn_free_array);
