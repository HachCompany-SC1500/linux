/*
 * arch/sh/boards/emtrion/display.h
 *
 * Copyright (c) 2008 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 * Revision:    $Revision$
 * Description: Provides display=<TX14|NL6448BC20>
 *
 **/

#ifndef _ARCH_SH_BOARDS_EMTRION_DISPLAY_H
#define _ARCH_SH_BOARDS_EMTRION_DISPLAY_H

extern const char* display_name;
extern int display_phys_width;
extern int display_phys_height;

int __init display_selected(const struct sh_mobile_lcdc_info *displays, int entries);
void __init display_read_parameters(struct sh_mobile_lcdc_info *info);
void display_configure_data_pins(void *board_data, int for_lcdc);

#define LCDC_BASE       0xfe940000
#define LCDC_END        0xfe942fff

#endif /* _ARCH_SH_BOARDS_EMTRION_DISPLAY_H */
