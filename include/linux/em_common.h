/*
 * em_common.h
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

#ifndef _LINUX_EM_COMMON_H
#define _LINUX_EM_COMMON_H

/* when set by the bootloader, will contain the board/module revision, e.g. "r2a".
   Otherwise it's set to an empty string. */
extern char em_hw_revision[];

extern int gpio_fn_request_array(const int gpios[], int size, const char *label);
extern int gpio_fn_free_array(const int gpios[], int size);


#endif
