/*
 * firmware.h - taken from U-Boot extbsp/firmware.h
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
 **/

#ifndef FIRMWARE_H
#define FIRMWARE_H

#pragma pack(push)
typedef struct {
	char name[64];
	u32 len;
	/* for easier operation we keep nv_firmware 64bit aligned */
	u32 len_padded;
	u32 reserved[4];
	u8  data[0];
} firmware_t;
#pragma pack(pop)

/* finds next firmware after firmware */
static inline firmware_t *firmware_next(firmware_t *p)
{
	if (!p || !p->len)
		return NULL;

	return (firmware_t*) (((void*) p) + sizeof(*p) + p->len_padded);
}

/* returns 1 when there are no more firmware entries after p */
static inline int firmware_at_end(const firmware_t *p)
{
	return !p || !p->len;
}

/* finds last entry in last */
static inline firmware_t *firmware_find_last(firmware_t *p)
{
	/* find last entry */
	while (!firmware_at_end(p))
		p = firmware_next((firmware_t*) p);

	return p;
}

/* for U-Boot */
const firmware_t *firmware_read(void);
void firmware_read_free(void);

#endif
