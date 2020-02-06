/*
 * em_module.h
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

#ifndef _LINUX_EM_MODULE_H
#define _LINUX_EM_MODULE_H

extern const char *em_module_hw_product_type(void);
extern const char *em_module_hw_revision(void);
extern const char *em_module_hw_patch_level(void);
extern const char *em_module_hw_serial_nr(void);
extern int em_module_hw_revision_as_hex(void);

#define EM_MODULE_REV_1a		0x1a
#define EM_MODULE_REV_2a		0x2a
#define EM_MODULE_REV_3a		0x3a
#define EM_MODULE_REV_4a		0x4a

#endif
