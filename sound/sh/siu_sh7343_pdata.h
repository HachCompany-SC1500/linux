/*
 * linux/sound/sh/siu_sh7343_pdata.h
 *
 * Copyright (c) 2009 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 *
 **/

#ifndef SIU_SH7343_PDATA_H
#define SIU_SH7343_PDATA_H

#define SIU_PORTA		0		/* port A */
#define SIU_PORTB		1		/* port B */

#define PCM			0x00000001	/* PCM */
#define I2S			0x00000002	/* I2S */

typedef struct {
        u_int32_t port_in_use;
        u_int32_t data_format;
        u_int32_t stereo_channels;
        u_int32_t max_volume;
        u_int32_t dma_out_ch;
        u_int32_t dma_in_ch;
        u_int8_t  master;
} siu_sh7343_pdata_t;

#endif /* SIU_SH7343_PDATA_H */
