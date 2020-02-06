/*
 * em_baseboard.h - Configuration and autodetect support for emtrion's baseboard
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

#ifndef _LINUX_EM_BASEBOARD_H
#define _LINUX_EM_BASEBOARD_H

/* see [1] */
#define EM_BASEBOARD_DIMMECOBASE	0xC1
#define EM_BASEBOARD_DIMMBASE		0xC2
#define EM_BASEBOARD_DIMMBASE_LOTHRON	0xC3

/* customer boards */
#define EM_BASEBOARD_PELLENC		0x01
#define EM_BASEBOARD_HACHLANGE		0x02

#ifdef CONFIG_EM_BASEBOARD
extern unsigned int __em_baseboard_code;
extern int __init em_baseboard_register(void);
extern void __init em_baseboard_init_irq(void);
# define em_baseboard_code()		__em_baseboard_code
#else
# define em_baseboard_code()		(0)
# define em_baseboard_register()	(0)
# define em_baseboard_init_irq()	do {} while(0)
#endif

#define em_baseboard_type()		((em_baseboard_code() & 0xFF00) >> 8)
/* extended temperature range. BSP will typically ignore this information  */
#define em_baseboard_extended_temp()	(em_baseboard_code() & 0x0080)
/* use this to order a board by revision+patch. e.g. R2a > R1a  */
#define em_baseboard_version()		(em_baseboard_code() & 0x007F)
#define em_baseboard_revision()		(em_baseboard_code() & 0x0078)
#define em_baseboard_patch()		(em_baseboard_code() & 0x0007)

#define EM_BASEBOARD_VER_R1a		0x08
#define EM_BASEBOARD_VER_R1b		0x09
#define EM_BASEBOARD_VER_R1c		0x0A
#define EM_BASEBOARD_VER_R1d		0x0B
#define EM_BASEBOARD_VER_R2a		0x10
#define EM_BASEBOARD_VER_R2b		0x11
#define EM_BASEBOARD_VER_R3a		0x18
#define EM_BASEBOARD_VER_R3b		0x19

#ifdef CONFIG_EM_BASEBOARD_DIMMECOBASE
# define em_baseboard_is_dimmecobase() 		(em_baseboard_type() == EM_BASEBOARD_DIMMECOBASE)
#else
# define em_baseboard_is_dimmecobase() 		(0)
#endif

#ifdef CONFIG_EM_BASEBOARD_DIMMBASE
# define em_baseboard_is_dimmbase() 		(em_baseboard_type() == EM_BASEBOARD_DIMMBASE)
#else
# define em_baseboard_is_dimmbase() 		(0)
#endif

#ifdef CONFIG_EM_BASEBOARD_DIMMBASE_LOTHRON
# define em_baseboard_is_dimmbase_lothron()	(em_baseboard_type() == EM_BASEBOARD_DIMMBASE_LOTHRON)
#else
# define em_baseboard_is_dimmbase_lothron()	(0)
#endif

#ifdef CONFIG_EM_BASEBOARD_PELLENC
# define em_baseboard_is_pellenc()		(em_baseboard_type() == EM_BASEBOARD_PELLENC)
#else
# define em_baseboard_is_pellenc()		(0)
#endif

#ifdef CONFIG_EM_BASEBOARD_HACHLANGE
# define em_baseboard_is_hachlange()		(em_baseboard_type() == EM_BASEBOARD_HACHLANGE)
#else
# define em_baseboard_is_hachlange()		(0)
#endif


#endif	/* _LINUX_EM_BASEBOARD_H */
