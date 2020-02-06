/*
 * bootlogo.h
 *
 * Copyright (c) 2009 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 * Description: 
 * References:
 *   [1] file:// 
 *
 **/

#ifndef _ARCH_SH_BOARDS_EMTRION_BOOTLOGO_H
#define _ARCH_SH_BOARDS_EMTRION_BOOTLOGO_H

typedef enum {
        BL_IGNORED,
        BL_COPIED,
        BL_KEPT,
} bootlogo_e;

bootlogo_e bootlogo_init(struct sh_mobile_lcdc_chan_cfg *ch, unsigned long smem_start, int size);

void bootlogo_overlay_with_linux_logo(void *board_data);

#endif /* _ARCH_SH_BOARDS_EMTRION_BOOTLOGO_H */
