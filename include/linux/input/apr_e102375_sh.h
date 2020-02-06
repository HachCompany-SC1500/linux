/*
 * apr_e102375_sh.h   touchdriver platform data
 *
 * Copyright (C) 2010 Secure Electrans Limited.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS for A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 **/

#ifndef	__INPUT_APR_E102375_SH_H
#define	__INPUT_APR_E102375_SH_H

struct apr_e102375_sh_platform_data {
        const char *clock;
        int gpio_power_down;    /* when >= 0, this GPIO is used for powering
                                 * down the APR */
};

#endif
