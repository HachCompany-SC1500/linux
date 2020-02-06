/*
    siu-sh7343.c - Alsa driver for Renesas' SH7343 SIU peripheral.

    Copyright (c) 2006 Carlos Munoz <carlos@kenati.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>

#include "siu_sh7343.h"


irqreturn_t siu_sh7343_in_dma_isr(int irq, void	*dev_id)
{
	port_info_t		*port_info = (port_info_t *)dev_id;
	u_int32_t		port = port_info->port;
	u_int32_t		nowcnt;
	volatile u_int32_t	dummy;

	if ((siu_obj[port].rw_flg & RWF_STM_RD) == 0x00)
		return IRQ_HANDLED;

	/* ich Interrupt disable, DMA transfer disable */
	siu_in_dmac[port]->CHCR &= ~0x00000005;

	nowcnt = port_info->capture.cur_period;

	/* DMA re-setup next transfer set */
	if (++port_info->capture.cur_period >= 
	    GET_MAX_PERIODS(port_info->capture.buf_bytes, 
			    port_info->capture.period_bytes)) {
		port_info->capture.cur_period = 0;
	}
	siu_in_dmac[port]->SAR = dmain[port];

	/* For mono we use the mono buffer */
	if (port_info->capture.channels == 1) {
		copy_capture_period(port_info);
		siu_in_dmac[port]->DAR = 
			(u_int32_t)port_info->capture.mono_buf;
	}
	else {
		siu_in_dmac[port]->DAR = 
			(u_int32_t)PERIOD_OFFSET(port_info->capture.buf, 
					 port_info->capture.cur_period, 
					 port_info->capture.period_bytes);
	}
	siu_in_dmac[port]->DMATCR = port_info->capture.dma_xfer_cnt;

	/* DMA transfer enable */
	siu_in_dmac[port]->CHCR |= 0x00000005;
	dummy = siu_in_dmac[port]->CHCR;		/* dummy read */
	siu_in_dmac[port]->CHCR = SET_BIT32(&siu_in_dmac[port]->CHCR, 
					    CHCR_TE_BIT, CHCR_TE_WIDTH, 0x00);

#if 0
	/* detect peaklevel of volume by software? */
	if (siu_stmrd[port].pkchk == PKCHKON || 
	    siu_stmrd[port].slcchk == DSLON) {
		siu_peak_check(port, INPUT, siu_stmrd[port].size, 
			       siu_stmrd[port].buff[nowcnt], nowcnt);
		/* detect silence by software? */
		if (siu_stmrd[port].slcchk == DSLON) {
			siu_silence_check(port);
		}
	}
#endif

	/* Notify alsa a period is done */
	snd_pcm_period_elapsed(port_info->capture.substream);

	return IRQ_HANDLED;
}


irqreturn_t siu_sh7343_out_dma_isr(int irq, void *dev_id)
{
	port_info_t		*port_info = (port_info_t *)dev_id;
	u_int32_t		port = port_info->port;
	volatile u_int32_t 	dummy;

	if ((siu_obj[port].rw_flg & RWF_STM_WTRN) == 0x00)
		return IRQ_HANDLED;

	/* och Interrupt disable, DMA transfer disable */
	siu_out_dmac[port]->CHCR &= ~0x00000005;

	/* DMA re-setup next transfer set */
	if (++port_info->playback.cur_period >= 
	    GET_MAX_PERIODS(port_info->playback.buf_bytes, 
			    port_info->playback.period_bytes)) {
		port_info->playback.cur_period = 0;
	}

	/* For mono we use the mono buffer */
	if (port_info->playback.channels == 1) {
		copy_playback_period(port_info);
		siu_out_dmac[port]->SAR = 
			(u_int32_t)port_info->playback.mono_buf;
	}
	else {
		siu_out_dmac[port]->SAR = 
			(u_int32_t)PERIOD_OFFSET(port_info->playback.buf, 
					 port_info->playback.cur_period, 
					 port_info->playback.period_bytes);
	}
	siu_out_dmac[port]->DAR = dmaout[port];
	siu_out_dmac[port]->DMATCR = port_info->playback.dma_xfer_cnt;

	/* DMA transfer enable */
	siu_out_dmac[port]->CHCR |= 0x00000005;
	dummy = siu_out_dmac[port]->CHCR;		/* dummy read */
	siu_out_dmac[port]->CHCR = SET_BIT32(&siu_out_dmac[port]->CHCR, 
					     CHCR_TE_BIT, CHCR_TE_WIDTH, 0x00);

	/* Notify alsa a period is done */
	snd_pcm_period_elapsed(port_info->playback.substream);

	return IRQ_HANDLED;
}


irqreturn_t siu_sh7343_isr(int irq, void *dev_id)
{
	u_int32_t spsts_tmp;
	u_int32_t evntc_tmp;

	evntc_tmp = IN32(EVNTC);
	spsts_tmp = IN32(SPSTS);

	/* interrupt of detect silence by hardware */
	if (evntc_tmp & 0x00004000) {
		if (siu_stmrd[SIU_PORTA].dsl == DSLON) {
			/* Do something ??? */
		}
	}

	/* interrupt of SPDIF status */
	if (evntc_tmp & 0x00008000) {
		/* changed LR channel status */
		if (spsts_tmp & 0x00003000) {
			/* Do something ??? */
		}
		/* channged Q-code number of music */
		if (spsts_tmp & 0x00004000) {
			/* Do something ??? */
		}
		OUT32(SPSTS, ~spsts_tmp);
	}

	OUT32(EVNTC, ~evntc_tmp);

	return IRQ_HANDLED;
}


