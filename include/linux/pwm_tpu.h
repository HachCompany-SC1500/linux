/*
 * pwm_tpu.h
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

#ifndef _LINUX_PWM_TPU_H
#define _LINUX_PWM_TPU_H

#define PWM_TPU_INVERT0	(1<<0)
#define PWM_TPU_INVERT1	(1<<1)
#define PWM_TPU_INVERT2	(1<<2)
#define PWM_TPU_INVERT3	(1<<3)

/**
 * struct pwm_tpu_platform_data - Platform-dependent data for pwm_tpu
 * @invert_polarity: inverts the polarity of the PWM signal. When unset, 1 is output in inactive phase, otherwise 0 is output. Use logical OR of PWM_TPU_INVERn
 */
struct pwm_tpu_platform_data {
	int invert_polarity;
	int divisor;
};

#endif
