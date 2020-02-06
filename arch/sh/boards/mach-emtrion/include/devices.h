/*
 * devices.h
 *
 * Copyright (c) 2010 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Michael Szafranek
 * Revision:    $Revision$
 *
 **/

#ifndef _ARCH_SH_BOARDS_EMTRION_DEVICES_H
#define _ARCH_SH_BOARDS_EMTRION_DEVICES_H

#if defined (CONFIG_CPU_SUBTYPE_SH7723)
#include <cpu/sh7723.h>
#include "sh7723.h"
#elif defined (CONFIG_CPU_SUBTYPE_SH7724)
#include <cpu/sh7724.h>
#include "sh7724.h"
#else
#error "Unknown processor type"
#endif

extern int last_gpio;
extern int pca9555_offset;   // gpio offset

int __init setup_camera(void);
int __init setup_display_i2c(void);

int __init setup_okican0(void);
int __init setup_okican1(void);

int __init setup_pwm_tpu(void);

int __init setup_spi_can(void);

int __init setup_i2c(void);

int __init setup_pwm_tpu(void);

int __init setup_lcd(void);
int __init setup_display(void);
void __init lcd_display_read_parameters(void);

int __init setup_eth(void);

int __init setup_nor(void);

int __init setup_nand(void);

int __init setup_spi_mmc(void);
int __init setup_sdhi0(void);
int __init setup_sdhi1(void);

int __init setup_usb(void);

int __init setup_scif(void);

int __init setup_msiof0_spi(void);
int __init setup_msiof1_spi(void);

#if defined (CONFIG_CPU_SUBTYPE_SH7723)
int __init setup_adc(void);
#endif

#ifdef CONFIG_CPU_SUBTYPE_SH7724
int __init setup_mmcif(void);

int __init sh7724_setup_fsi(void);
#endif

#endif /* _ARCH_SH_BOARDS_EMTRION_DEVICES_H */
