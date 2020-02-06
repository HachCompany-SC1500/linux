/*
 *  siu-sh7343.c - Alsa driver for Renesas' SH7343 SIU peripheral.
 *
 *  Copyright (c) 2006 Carlos Munoz <carlos@kenati.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/
#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include "siu_sh7343.h"

typedef struct{
	u_int32_t	ena;
	u_int32_t	inport;
	u_int32_t	outport;
	u_int32_t	secbuff;
	u_int32_t	fir;
	u_int32_t	mix;
	u_int32_t	iir;
	u_int32_t	scr;
} T_DRV_SIU_SPBSELECTA;

typedef struct{
	u_int32_t	ena;
	u_int32_t	inport;
	u_int32_t	outport;
} T_DRV_SIU_SPBSELECTB;

typedef struct{
	u_int32_t	form;
	u_int32_t	mona;
} T_DRV_SIU_OPEN;

siu_info_t			siu_info;
static u_int32_t		siu_obj_status;
static u_int32_t		infs_tmp;
static u_int32_t		outfs_tmp;

T_SIU				siu_obj[MAX_SIU_PORTS];
SIU_STMRW			siu_stmrd[MAX_SIU_PORTS];
SIU_STMRW			siu_stmwt[MAX_SIU_PORTS];
static T_DRV_SIU_OPEN		siu_rdopn[MAX_SIU_PORTS]; /* Read-Open */
static T_DRV_SIU_OPEN		siu_wtopn[MAX_SIU_PORTS]; /* Write-Open */

static struct siu_sh7723_priv {
        struct device *dev;
        struct clk    *clk;
        const siu_sh7343_pdata_t *pdata;
} priv;

/* Points to the control registers of the dma channel in use */
volatile struct st_dmac		*siu_in_dmac[MAX_SIU_PORTS];

/* Points to the control registers of the dma channel in use */
volatile struct st_dmac		*siu_out_dmac[MAX_SIU_PORTS];

/* Official dma channel start control address */
struct st_dmac 			*adddat_dmac[] = {
	DMAC0, DMAC1, DMAC2, DMAC3, DMAC4, DMAC5
};

/* DMA read port address for each port */
const u_int32_t dmain[MAX_SIU_PORTS] = {
	DMAIA, DMAIB
};

/* DMA write port address for each port */
const u_int32_t	dmaout[MAX_SIU_PORTS] = {
	DMAOA, DMAOB
};

/* Port A and port B output configuration value */
const u32 out_port[MAX_SIU_PORTS] = {
	PORTAOUTPUT, PORTBOUTPUT
};

/* Port A and port B input configuration value */
const u32 in_port[MAX_SIU_PORTS] = {
	PORTAINPUT, PORTBINPUT
};

/* DMAC channel number and interrupt code */
static const u_int32_t dmac_code[] = {
	DMTE0_IRQ,
	DMTE1_IRQ,
	DMTE2_IRQ,
	DMTE3_IRQ,
	DMTE4_IRQ,
	DMTE5_IRQ
};

static u_int32_t		ydef[YRAM_DEF_SIZE];
static u_int32_t		fircoef[YRAM_FIR_SIZE];
static u_int32_t		iircoef[YRAM_IIR_SIZE];

static u_int32_t		stfifo;
static u_int32_t		trdat;

/* the SPB reads only volume control of SBDVCA, even for port B */
static const u_int32_t sbdvc[MAX_SIU_PORTS] = {
	SBDVCA, SBDVCA /*SBDVCB */
};


static int snd_siu_sh7343_start(port_info_t	*port_info, int playback)
{
	int	i;
	int	off;

	if (siu_obj_status != ST_STOP) {
		printk(KERN_ERR "snd_siu_sh7343_start() invalid status=%x\n",
		       siu_obj_status);
		return -EINVAL;
	}

	/* SIU operates (power down disabled) */
        clk_enable(priv.clk);

	/* Issue software reset to siu */
	OUT32(SRCTL, 0);

	/* Wait for the reset to take effect */
	udelay(1);

	/* Load initial value to YRAM. Note off is only initialized once */
	for (i = 0, off = 0; i < YRAM0_SIZE; i++, off++)
		ydef[off] = SIU_SPB_PRO_Y0[i];

	for (i = 0; i < YRAM1_SIZE; i++, off++)
		ydef[off] = SIU_SPB_PRO_Y1[i];

	for (i = 0; i < YRAM2_SIZE; i++, off++)
		ydef[off] = SIU_SPB_PRO_Y2[i];

	for (i = 0; i < YRAM3_SIZE; i++, off++)
		ydef[off] = SIU_SPB_PRO_Y3[i];

	for (i = 0; i < YRAM4_SIZE; i++, off++)
		ydef[off] = SIU_SPB_PRO_Y4[i];

	for (i = 0; i < YRAM_FIR_SIZE; i++)
		fircoef[i] = SIU_SRC_44100_44100[i];

	for (i = 0; i < YRAM_IIR_SIZE; i++)
		iircoef[i] = 0x0;

	stfifo = 0x0;
	trdat = 0x0;

	/* portA, portB, SIU operate */
	OUT32(SRCTL, 0x00000301);

	/* portA=256fs, portB=256fs */
	OUT32(CKCTL, 0x40400000);

	/* portA's BRG does not divide SIUCKA */
	OUT32(BRGASEL, 0x00000000);
	OUT32(BRRA, 0x00000000);

	/* portB's BRG divides SIUCKB by half */
	OUT32(BRGBSEL, 0x00000001);
	OUT32(BRRB, 0x00000000);

	OUT32(IFCTL, 0x44440000);

	if(priv.pdata->master) {
                /* MSELCR[2]=B'1 SIUA_OLR, SIUA_OBT output */
                OUT16(MSELCR, IN16(MSELCR) | MSELCR_SET_BITS);

		/* portA32bt/fd SIUmaster AportB32bt/fs SIUmaster */
		OUT32(SFORM, 0x0c0c0000);
        } else {
                /* MSELCR[3:0]=B'0000 SIUA_OLR,_OBT,_ILR,_IBT, SIUB_OLR,_OBT,
                   _ILR,_IBT input */
                OUT16(MSELCR, IN16(MSELCR) & ~0x000f);

		/* portA32bt/fs/SIUslave AportB32bt/fs SIUslave */
		OUT32(SFORM, 0x00000000);
        }

	/* Volume levels */
	OUT32(sbdvc[priv.pdata->port_in_use],
              playback ?
              port_info->playback.volume :
              port_info->capture.volume );

	/* SIU driver object information initialize */
	for (i = 0; i < MAX_SIU_PORTS; i++) {
		siu_obj[i].stat = ST_READY;
		siu_obj[i].open_mode = 0x00;
		siu_obj[i].ich = -1;		/* Input DMA channel free */
		siu_obj[i].och = -1;		/* Output DMA channel free */
		siu_obj[i].rw_flg = 0x00;	/* stream-data transfer flag */
		siu_stmrd[i].pkchk = PKCHKOFF;
		siu_stmwt[i].pkchk = PKCHKOFF;
	}

	infs_tmp = outfs_tmp = 44100;	/* sampling frequency without SRC */
	siu_obj_status = ST_READY;

	return 0;
}


static int snd_siu_sh7343_stop(void)
{
	if (!(siu_obj_status & ST_READY))
		return -EPERM;

	OUT32(SRCTL, 0x00000000);	/* SIU software reset */

	/* Clock supply to SIU halted */
        clk_disable(priv.clk);

	/* Reset status of SIU */
	siu_obj_status = ST_STOP;

	return 0;
}


static int snd_siu_sh7343_spbAselect(T_DRV_SIU_SPBSELECTA	*par)
{
	u_int32_t a2a, scr, iir, mix, fir;
	u_int32_t tmp;

	if (siu_obj_status & ST_SPBACTIV)
		return -EPERM;		/* SPB working */
	if (!(siu_obj_status & ST_READY))
		return -EPERM;
	if (par == NULL)
		return -EINVAL;
	if (par->ena == PATHAOFF) {	/* path A no use */
		ydef[0] = 0x00000000;
	}
	else if (par->ena == PATHAON) {	/* path A use */
		if(par->inport == CPUOUTPUT && par->outport == CPUINPUT)
			tmp = 0;
		else if(par->inport == CPUOUTPUT && par->outport == PORTAOUTPUT)
			tmp = 1;
		else if(par->inport == CPUOUTPUT && par->outport == PORTBOUTPUT)
			tmp = 2;
		else if(par->inport == PORTAINPUT && par->outport == CPUINPUT)
			tmp = 3;
		else if(par->inport == PORTAINPUT && 
			par->outport == PORTAOUTPUT)
			return -EINVAL;
		else if(par->inport == PORTAINPUT && 
			par->outport == PORTBOUTPUT)
			return -EINVAL;
		else if(par->inport == PORTBINPUT && par->outport == CPUINPUT)
			tmp = 4;
		else if(par->inport == PORTBINPUT && 
			par->outport == PORTAOUTPUT)
			return -EINVAL;
		else if(par->inport == PORTBINPUT && 
			par->outport == PORTBOUTPUT)
			return -EINVAL;
#if MELDUSE
		else if(par->inport == CPUOUTPUT && par->outport == MELINPUT)
			return -EINVAL;
		else if(par->inport == PORTAINPUT && par->outport == MELINPUT)
			tmp = 9;
		else if(par->inport == PORTBINPUT && par->outport == MELINPUT)
			tmp = 10;
		else if(par->inport == MELOUTPUT && par->outport == CPUINPUT)
			tmp = 11;
		else if(par->inport == MELOUTPUT && par->outport == PORTAOUTPUT)
			tmp = 12;
		else if(par->inport == MELOUTPUT && par->outport == PORTBOUTPUT)
			tmp = 13;
		else if(par->inport == MELOUTPUT && par->outport == MELINPUT)
			return -EINVAL;
#endif
		else
			return -EINVAL;

		if (par->secbuff != 16)
			return -EINVAL;

		if (par->fir == FIRTHR)
			fir = 0x1;	/* through */
		else if (par->fir==FIRLPF) {
			if(!(siu_obj_status & ST_LPF_CALL))
				return -EINVAL;
			if (siu_obj_status & ST_SRC_CALL)
				return -EINVAL;	/* Concurrent SRC impossible */
			fir = 0x2;	/* LPF */
		}
		else if (par->fir == FIRSRC) {
			if (!(siu_obj_status & ST_SRC_CALL))
				return -EINVAL;
			if(siu_obj_status & ST_LPF_CALL)
				return -EINVAL;	/* Concurrent LPF impossible */
			fir = 0x4;	/* SRC */
		}
		else
			return -EINVAL;

		if (par->mix == MIXOFF) {
			mix = 0x0;	/* MIX OFF */
			a2a = 0x0;
		}
#if MELDUSE
		else if (par->mix == MIXON) {
			mix = 0x1;			/* MIX OFF */
			a2a = 0xa;
		}
#endif
		else
			return -EINVAL;

		if (par->iir == IIROFF)
			iir = 0x0;	/* GEQ OFF */
		else if (par->iir == IIRON) {
			if (!(siu_obj_status & ST_IIR_CALL))
				return -EINVAL;
			if (ydef[0] & 0x00000020)
				iir = 0x1;	/* GEQON  start */
			else
				iir = 0x0;	/* GEQOFF start */
		}
		else 
			return -EINVAL;

		if (par->scr == SCROFF)
			scr = 0x0;	/* scramble OFF */
		else 
			return -EINVAL;

		ydef[0] = 0;
		ydef[0] |= a2a << 24;
		ydef[0] |= t_spbpar[tmp].ab1a << 16;
		ydef[0] |= t_spbpar[tmp].ab0a << 8;
		ydef[0] |= t_spbpar[tmp].dir << 7;
		ydef[0] |= scr << 6;
		ydef[0] |= iir << 5;
		ydef[0] |= mix << 4;
		ydef[0] |= fir << 1;
		ydef[0] |= 0x000000001;
		ydef[1] = SIU_SPB_PRO_Y0[1];	/* 0x03000300 */
		ydef[2] = (par->secbuff / 2) << 24;
		ydef[3] = SIU_SPB_PRO_Y0[3];	/* 0x00000000 */
		ydef[4] = SIU_SPB_PRO_Y0[4];	/* 0x00000000 */
		ydef[7] = t_spbpar[tmp].event;
		stfifo |= t_spbpar[tmp].stfifo;
		trdat |= t_spbpar[tmp].trdat;
	}
	else
		return -EINVAL;

	siu_obj_status |= ST_PATHA_CALL;

	return 0;
}


static int snd_siu_sh7343_spbBselect(T_DRV_SIU_SPBSELECTB *par)
{
	u_int32_t tmp;

	if (siu_obj_status & ST_SPBACTIV)
		return -EPERM;	/* SPB working */
	if (!(siu_obj_status & ST_READY))
		return -EPERM;
	if (par == NULL)
		return -EINVAL;
	if (par->ena == PATHBOFF) {	/* path B no use */
		ydef[5] = 0x0;
	}
	else if (par->ena == PATHBON) {	/* path B use */
		if (par->inport == CPUOUTPUT && par->outport == CPUINPUT)
			return -EINVAL;
		else if (par->inport == CPUOUTPUT && 
			 par->outport == PORTAOUTPUT)
			tmp = 5;
		else if (par->inport == CPUOUTPUT && 
			 par->outport == PORTBOUTPUT)
			tmp = 6;
		else if (par->inport == PORTAINPUT && par->outport == CPUINPUT)
			tmp = 7;
		else if (par->inport == PORTAINPUT && 
			 par->outport == PORTAOUTPUT)
			return -EINVAL;
		else if (par->inport == PORTAINPUT && 
			 par->outport == PORTBOUTPUT)
			return -EINVAL;
		else if (par->inport == PORTBINPUT && par->outport == CPUINPUT)
			tmp = 8;
		else if (par->inport == PORTBINPUT && 
			 par->outport == PORTAOUTPUT)
			return -EINVAL;
		else if (par->inport == PORTBINPUT && 
			 par->outport == PORTBOUTPUT)
			return -EINVAL;
#if MELDUSE
		else if (par->inport == CPUOUTPUT && par->outport == MELINPUT)
			return -EINVAL;
		else if (par->inport == PORTAINPUT && par->outport == MELINPUT)
			tmp = 14;
		else if (par->inport == PORTBINPUT && par->outport == MELINPUT)
			tmp = 15;
		else if (par->inport == MELOUTPUT && par->outport == CPUINPUT)
			return -EINVAL;
		else if (par->inport == MELOUTPUT && 
			 par->outport == PORTAOUTPUT)	tmp = 16;
		else if (par->inport == MELOUTPUT && 
			 par->outport == PORTBOUTPUT)	tmp = 17;
		else if (par->inport == MELOUTPUT && par->outport == MELINPUT)
			return -EINVAL;
#endif
		else return -EINVAL;


		ydef[5] = 0x0;
		ydef[5] |= t_spbpar[tmp].ab1a << 16;
		ydef[5] |= t_spbpar[tmp].ab0a << 8;
		ydef[5] |= 0x000000001;
		ydef[6] = t_spbpar[tmp].event;
		stfifo |= t_spbpar[tmp].stfifo;
		trdat |= t_spbpar[tmp].trdat;
	}
	else
		return -EINVAL;

	siu_obj_status |= ST_PATHB_CALL;

	return 0;
}


static int snd_siu_sh7343_openA(u_int32_t	openmode, 
				T_DRV_SIU_OPEN	*inpar, 
				T_DRV_SIU_OPEN  *outpar)
{
	u_int32_t tmp;

	if (siu_obj[SIU_PORTA].stat != ST_READY)
		return -EPERM;
	if (openmode != INPUT && openmode != OUTPUT && openmode != INOUTPUT)
		return -EINVAL;

	OUT32(SRCTL, IN32(SRCTL) | 0x00000200);		/* portA Operates */

	tmp = 0x00000000;
	/* input, inoutput */
	if (openmode == INPUT || openmode == INOUTPUT) {
		if (inpar == NULL)
			return -EINVAL;
		if (inpar->form == SPDIF && inpar->mona == MONAURAL)
			return -EINVAL;
		switch (inpar->form) {
		default:
			return -EINVAL;
			break;
		case NODEVICE:	/* input off */
				break;
		case PCM:
			tmp |= 0x04000000;
			break;
		case I2S:
			tmp |= 0x05000000;
			break;
		case SPDIF:
			tmp |= 0x08000000;
			break;
		}
		siu_rdopn[SIU_PORTA].form = inpar->form;
		if (inpar->form == PCM || inpar->form == I2S) {
			switch (inpar->mona) {
			default:
				return -EINVAL;
				break;
			case STEREO:
				break;
			case MONAURAL:
				tmp |= 0x00000080;
				break;
			}
			siu_rdopn[SIU_PORTA].mona = inpar->mona;
		}
	}

	/* output inoutput */
	if (openmode == OUTPUT || openmode == INOUTPUT) {
		if (outpar == NULL)
			return -EINVAL;

		switch (outpar->form) {
		default:
			return -EINVAL;
			break;
		case NODEVICE:	/* output off */
			break;
		case PCM:
			tmp |= 0x40000000;
			break;
		case I2S:
			tmp |= 0x50000000;
			break;
		}
		siu_wtopn[SIU_PORTA].form = outpar->form;
		if (outpar->form == PCM || outpar->form == I2S || 
		    outpar->form == PCMSPDIF || outpar->form == I2SSPDIF) {
			switch (outpar->mona) {
			default:
				return -EINVAL;
				break;
			case STEREO:
				break;
			case MONAURAL:
				tmp |= 0x00000080;
				break;
			}
			siu_wtopn[SIU_PORTA].mona = outpar->mona;
		}
	}
	siu_obj[SIU_PORTA].open_mode = (u_int8_t)openmode;	/* open mode */

	OUT32(IFCTL, (IN32(IFCTL) & ~0xff0000c2) | tmp);  /* portA bit set */

	siu_obj[SIU_PORTA].stat = ST_OPEN;	/* Status of port */
	siu_obj_status          &= ~ST_READY;
	siu_obj_status          |= ST_OPEN;    /* Status of SIU driver object */

	return 0;
}


static int snd_siu_sh7343_closeA(void)
{
	if (siu_obj[SIU_PORTA].stat != ST_OPEN)
		return -EPERM;
	if (siu_obj_status & ST_SPBACTIV)
		return -EPERM;	/* SPB working? */

	OUT32(SRCTL, IN32(SRCTL) & ~0x00000200); /* portA reset */

	siu_obj[SIU_PORTA].stat = ST_READY;
	siu_obj[SIU_PORTA].open_mode = 0x00;	/* open mode */

	if( siu_obj[SIU_PORTB].stat == ST_READY ){
		siu_obj_status = ST_READY;   /* Status of SIU driver object */
	}

	return 0;
}


static int snd_siu_sh7343_openB(u_int32_t	openmode, 
				T_DRV_SIU_OPEN  *inpar, 
				T_DRV_SIU_OPEN	*outpar)
{
	u_int32_t tmp;

	if (siu_obj[SIU_PORTB].stat != ST_READY)
		return -EPERM;
	if (openmode != INPUT && openmode != OUTPUT && openmode != INOUTPUT)
		return -EINVAL;

	OUT32(SRCTL, IN32(SRCTL) | 0x00000100);		/* portB Operates */

	tmp = 0x00000000;
	/* input inoutput */
	if (openmode == INPUT || openmode == INOUTPUT) {
		if (inpar == NULL)
			return -EINVAL;
		switch (inpar->form) {
		default:
			return -EINVAL;
			break;
		case NODEVICE:	/* input off */
			break;
		case PCM:
			tmp |= 0x00040000;
			break;
		case I2S:
			tmp |= 0x00050000;
			break;
		}

		switch (inpar->mona) {
		default:
			return -EINVAL;
			break;
		case STEREO:
			break;
		case MONAURAL:
			tmp |= 0x00000020;
			break;
		}

		siu_rdopn[SIU_PORTB].form = inpar->form;
		siu_rdopn[SIU_PORTB].mona = inpar->mona;
	}

	/* output inoutput */
	if (openmode == OUTPUT || openmode == INOUTPUT) {
		if (outpar == NULL)
			return -EINVAL;
		switch (outpar->form) {
		default:
			return -EINVAL;
			break;
		case NODEVICE:	/* output off */
			break;
		case PCM:
			tmp |= 0x00400000;
			break;
		case I2S:
			tmp |= 0x00500000;
			break;
		}

		switch (outpar->mona) {
		default:
			return -EINVAL;
			break;
		case STEREO:
			break;
		case MONAURAL:
			tmp |= 0x00000020;
			break;
		}

		siu_wtopn[SIU_PORTB].form = outpar->form;
		siu_wtopn[SIU_PORTB].mona = outpar->mona;
	}
	siu_obj[SIU_PORTB].open_mode = (u_int8_t)openmode;	/* open mode */

	OUT32(IFCTL, (IN32(IFCTL) & ~0x00770031) | tmp);  /* portB bit set */

	siu_obj[SIU_PORTB].stat = ST_OPEN;		/* Status of port */
	siu_obj_status          &= ~ST_READY;
	siu_obj_status          |= ST_OPEN;  /* Status of SIU driver object */

	return 0;
}


static int snd_siu_sh7343_closeB(void)
{
	if (siu_obj[SIU_PORTB].stat != ST_OPEN)
		return -EPERM;
	if (siu_obj_status & ST_SPBACTIV)
		return -EPERM;	/* SPB working? */

	OUT32(SRCTL, IN32(SRCTL) & ~0x00000100); /* portB reset */

	siu_obj[SIU_PORTB].stat = ST_READY;
	siu_obj[SIU_PORTB].open_mode = 0x00;	/* open mode */

	if( siu_obj[SIU_PORTA].stat == ST_READY ){
		siu_obj_status = ST_READY;   /* Status of SIU driver object */
	}

	return 0;
}


/* Port A/B start function */
static int (* const siu_open[MAX_SIU_PORTS])(u_int32_t, T_DRV_SIU_OPEN *, 
					     T_DRV_SIU_OPEN *) = {
	snd_siu_sh7343_openA, snd_siu_sh7343_openB
};


/* Port A/B close function */
static int (* const siu_close[MAX_SIU_PORTS])(void) = {
	snd_siu_sh7343_closeA, snd_siu_sh7343_closeB
};


static int snd_siu_sh7343_defdma(u_int32_t	port, 
				 u_int32_t	inout,
				 int32_t	ch)
{

	if (port >= MAX_SIU_PORTS) {
		PRINTK("port >= MAX_SIU_PORTS\n");
		return -EINVAL;
	}
	if (siu_obj[port].stat != ST_OPEN) {
		PRINTK("siu_obj[port].stat != ST_OPEN\n");
		return -EPERM;
	}
	if (ch > 5) {
		PRINTK("ch > 5\n");
		return -EINVAL;
	}

	/* input */
	if (inout == INPUT) {
		if (ch < 0) {	/* channel free */
			if (siu_obj[port].ich == (-1))
				return -EINVAL;
		}
		else {	/* channel secure */
			if (siu_obj[port].ich != (-1))
				return -EINVAL;	/* duplication */
		}
	}
	/* output */
	else if (inout == OUTPUT) {
		if (ch <0 ) {	/* channel free */
			if (siu_obj[port].och == (-1)) {
				PRINTK("siu_obj[port].och == (-1)\n");
				return -EINVAL;
			}
		}
		else {	/* channel secure */
			if (siu_obj[port].och != (-1)) {
				PRINTK("2 siu_obj[port].och == (-1)\n");
				return -EINVAL;	/* duplication */
			}
		}
	}
	else 
		return -EINVAL;

	return 0;
}


static int snd_siu_sh7343_pcmdatapack(u_int32_t	port,
				      u_int32_t dpak)
{
	u_int32_t tmp;

	if (port >= MAX_SIU_PORTS)
		return -EINVAL;
	if (siu_obj[port].stat != ST_OPEN)
		return -EPERM;
	if (siu_obj[port].rw_flg & (RWF_STM_RD | RWF_STM_WT))
		return -EPERM;
	if (dpak > 3)
		return -EINVAL;

	tmp = IN32(DPAK);

	if (port == SIU_PORTA) {
		dpak <<= 30;
		tmp &= ~0xc0000000;
	}
	else if (port == SIU_PORTB) {
		dpak <<= 22;
		tmp &= ~0x00c00000;
	}
	else
		return -EINVAL;

	OUT32(DPAK, tmp | dpak);

	return 0;
}


static int snd_siu_sh7343_spbstart(void)
{
	u_int32_t cnt;
	u_int32_t *add;
	u_int32_t *ptr;
	u_int32_t spbwaitcnt;

	if (!(siu_obj_status & ST_OPEN))
		return -EPERM;
	if (siu_obj_status & ST_SPBACTIV)
		return -EPERM;	/* SPB working */
	if (!(siu_obj_status & ST_PATHA_CALL))
		return -EPERM;
	if (!(siu_obj_status & ST_PATHB_CALL))
		return -EPERM;

	/* SPB Program Load to PRAM */
	ptr = (u_int32_t *)SIU_SPB_PRO_P0;
	add = (u_int32_t *)siu_p_ram;
	for (cnt = 0; cnt < PRAM0_SIZE; cnt++) {
		*add++ = (u_int32_t)(*ptr++);
	}
	ptr = (u_int32_t *)SIU_SPB_PRO_P1;
	add = (u_int32_t *)(siu_p_ram + 0x0100);
	for (cnt = 0; cnt < PRAM1_SIZE; cnt++) {
		*add++ = (u_int32_t)(*ptr++);
	}

	/* XRAM initialization */
	add = (u_int32_t *)siu_x_ram;
	for (cnt = 0; cnt < (XRAM0_SIZE + XRAM1_SIZE + XRAM2_SIZE); cnt++) {
		*add++ = 0x0;
	}
	/* YRAM variable area initialization */
	add = (u_int32_t *)siu_y_ram;
	for (cnt = 0; cnt< YRAM_DEF_SIZE; cnt++) {
		*add++ = ydef[cnt];
	}
	/* YRAM FIR coefficient area initialization */
	add = (u_int32_t *)(siu_y_ram + 0x0200);
	for (cnt = 0; cnt < YRAM_FIR_SIZE; cnt++) {
		*add++ = fircoef[cnt];
	}
	/* YRAM IIR coefficient area initialization */
	add = (u_int32_t*)(siu_y_ram + 0x0600);
	for (cnt = 0; cnt < YRAM_IIR_SIZE; cnt++) {
		*add++ = iircoef[cnt];
	}

	OUT32(TRDAT, trdat);
	trdat = 0x0;

	spbwaitcnt = 0x00010000;

	OUT32(SBACTIV, 0x00000000);			/* SPB start */
	siu_obj_status |= ST_SPBACTIV;
	OUT32(SBCTL, 0xc0000000);			/* SPB working */
	for (cnt = 0; cnt < spbwaitcnt; cnt++) {
		if (IN32(SBCTL) == 0x80000000)
			break;
	}
	if (cnt == spbwaitcnt) {
		return -EBUSY;
	}
	OUT32(SBPSET, 0x00400000);	/* SPB program start address setting */
	OUT32(SBACTIV, 0xc0000000);	/* SPB hardware start(FIFOCTL source) */

	return 0;
}


static void snd_siu_sh7343_spbstop(void)
{
	if (!(siu_obj_status & ST_SPBACTIV))
		return;	/* SPB is not working? */
	if (siu_obj[SIU_PORTA].rw_flg & (RWF_STM_RD | RWF_STM_WT))
		return;	/* during stmread or stmwrite ? */
	if (siu_obj[SIU_PORTB].rw_flg & (RWF_STM_RD | RWF_STM_WT))
		return;

	OUT32(SBACTIV, 0x00000000);
	OUT32(SBCTL, 0x00000000);			/* SPB stop */
	siu_obj_status &= ~ST_SPBACTIV;

	siu_obj_status &= ~(ST_PATHA_CALL | ST_PATHB_CALL);

	stfifo = 0x0;
}


/* transfersize is number of u_int32_t dma transfers per period */
static int snd_siu_sh7343_stmwrite(u_int32_t	port,
				   u_int32_t	transfersize)
{
	port_info_t	*port_info = &siu_info.port_info[port];

	if (port >= MAX_SIU_PORTS)
		return -EINVAL;
	if (siu_obj[port].stat != ST_OPEN)
		return -EPERM;
	if (siu_obj[port].open_mode == INPUT)
		return -EPERM;

	if (transfersize == 0) {	/* stmwrite stop */
		if (!(siu_obj[port].rw_flg & RWF_STM_WT))
			return -EPERM;

		/* output FIFO disable */
		OUT32(STFIFO, IN32(STFIFO) & ~0x0c180c18);

		/* DMA Transfer stop */
		/* DE=0 -> transfer disable */
		siu_out_dmac[port]->CHCR = 
			SET_BIT32(&siu_out_dmac[port]->CHCR, 
				  CHCR_DE_BIT, CHCR_DE_WIDTH, 0x00);

		/* during stmwrite clear */
		siu_obj[port].rw_flg &= ~(RWF_STM_WT | RWF_STM_WTRN | 
					  RWF_STM_WTUR);
	}
	else {	/* stmwrite start */
		if (transfersize > 16777216)
			return -EINVAL;
		if (!(siu_obj_status & ST_SPBACTIV))
			return -EPERM;	/* SPB stopped? */
		if (siu_obj[port].rw_flg & RWF_STM_WT)
			return -EPERM;

		if (siu_stmwt[port].pkchk == PKCHKON) {
			if (siu_stmwt[port].pk_mtx != 1) {
				return -EINVAL;
			}
		}

		/* output-data stored buffer counter(to transfer restart) */
		siu_stmwt[port].undruncnt = 1;

		/* Current period in buffer */
		port_info->playback.cur_period = 0;

		/* to transfer restart counter reset */
		siu_stmwt[port].undctr    = 0;

		/* during stmwrite flag set */
		siu_obj[port].rw_flg |= RWF_STM_WT;
	}

	return 0;
}


/*
   ----------------------------------------------------------------------------
   Local function : output DMA transfer setting and START
   ----------------------------------------------------------------------------
*/
static void snd_siu_sh7343_wr_dma_set(u_int32_t	port,
				      u_int32_t	*buff,
				      u_int32_t	size)
{
	volatile u_int32_t dummy;

	siu_out_dmac[port]->SAR    = (u_int32_t)buff;
	siu_out_dmac[port]->DAR    = dmaout[port];
	siu_out_dmac[port]->DMATCR = size;

	dummy = siu_out_dmac[port]->CHCR; /* TEbit dummy read */
	siu_out_dmac[port]->CHCR = 0x00000000; /* CHCR_(output-channel) clear */

	/* CHCR_(output-channel) set. Cycle steal mode */
	siu_out_dmac[port]->CHCR = 0x00001814;

	dummy = IN16(DMAOR);	/* AEbit, NMIFbit dummy read */

	/* AEbit clear */
	OUT16(DMAOR, SET_BIT16(DMAOR, DMAOR_AE_BIT, DMAOR_AE_WIDTH, 0x00));

	/* NMIFbit clear */
	OUT16(DMAOR, SET_BIT16(DMAOR, DMAOR_NMIF_BIT, DMAOR_NMIF_WIDTH, 0x00));

	/* normal mode */
	/* DMA Transfer enable( but output-channel transfer disable) */
	OUT16(DMAOR, IN16(DMAOR) | 0x0001);

	/* Output DMA Transfer START!! */
	/* output-channel DMA Transfer enable */
	siu_out_dmac[port]->CHCR |= 0x00000001;

	/* only output FIFO enable */
	OUT32(STFIFO, IN32(STFIFO) | (stfifo & 0x0c180c18));
}


static void snd_siu_sh7343_rd_dma_set(u_int32_t	port,
				      u_int32_t	*buff,
				      u_int32_t size)
{
	volatile u_int32_t dummy;

	siu_in_dmac[port]->SAR = dmain[port]; /* DMA source address */
	siu_in_dmac[port]->DAR = (u_int32_t)buff; /* DMA destination address */
	siu_in_dmac[port]->DMATCR = size; /* DMA transfer count */

	dummy = siu_in_dmac[port]->CHCR; /* TEbit dummy read */
	siu_in_dmac[port]->CHCR = 0x00000000; /* CHCR_(input-channel) clear */

	/* CHCR_(input-channel) set. Cycle steal mode */
	siu_in_dmac[port]->CHCR = 0x00004814;

	dummy = IN16(DMAOR);	/* AEbit, NMIFbit dummy read */

	/* AEbit clear */
	OUT16(DMAOR, SET_BIT16(DMAOR, DMAOR_AE_BIT, DMAOR_AE_WIDTH, 0x00));

	/* NMIFbit clear */
	OUT16(DMAOR, SET_BIT16(DMAOR, DMAOR_NMIF_BIT, DMAOR_NMIF_WIDTH, 0x00));

	/* normal mode */
	/* DMA Transfer enable( but input-channel transfer disable ) */
	OUT16(DMAOR, IN16(DMAOR) | 0x0001);

	/* Input DMA Transfer START!! */
	/* input-channel DMA Transfer enable */
	siu_in_dmac[port]->CHCR |= 0x00000001;

	/* only input FIFO enable */
	OUT32(STFIFO, IN32(STFIFO) | (stfifo & 0x13071307));
}


static int snd_siu_sh7343_stmwriteenable(u_int32_t	port)
{
	u_int8_t	substatus;
	port_info_t	*port_info = &siu_info.port_info[port];

	if (port >= MAX_SIU_PORTS)
		return -EINVAL;
	if (siu_obj[port].stat != ST_OPEN) {
		PRINTK("siu_obj[port].stat != ST_OPEN\n");
		return -EPERM;
	}
	if (!(siu_obj[port].rw_flg & RWF_STM_WT)) {
		PRINTK("!(siu_obj[port].rw_flg & RWF_STM_WT)\n");
		return -EPERM;
	}

#if 0
	/* detect peak by software? */
	if (siu_stmwt[port].pkchk == PKCHKON || 
	    siu_stmwt[port].slcchk == DSLON) {
		siu_peak_check(port, OUTPUT, siu_stmwt[port].size, 
			       siu_stmwt[port].buff[m], m);
	}
#endif

	substatus = (u_int8_t)(siu_obj[port].rw_flg & ~RWF_STM_RD);

	if (substatus == RWF_STM_WT) {	/* wait transfer */
		siu_obj[port].rw_flg |= RWF_STM_WTRN;
		/* DMA transfer start */
		/* For mono streams we need to use the mono buffer */
		if (port_info->playback.channels == 1) {
			copy_playback_period(port_info);
			snd_siu_sh7343_wr_dma_set(port, port_info->playback.mono_buf,
						  port_info->playback.dma_xfer_cnt);
		}
		else {
			snd_siu_sh7343_wr_dma_set(port, 
				(u_int32_t *)PERIOD_OFFSET(port_info->playback.buf, 
				port_info->playback.cur_period, 
				port_info->playback.period_bytes),
				port_info->playback.dma_xfer_cnt);
		}
	}
	else if (substatus == (RWF_STM_WT|RWF_STM_WTRN)){  /* now transfer */
		;	/* NOP */
	}
	/* transfer pause */
	else if (substatus == (RWF_STM_WT | RWF_STM_WTRN | RWF_STM_WTUR)) {
		if (++siu_stmwt[port].undctr == siu_stmwt[port].undruncnt) {
			if (++port_info->playback.cur_period >= 
			    GET_MAX_PERIODS(port_info->playback.buf_bytes, 
					    port_info->playback.period_bytes)) {
				port_info->playback.cur_period = 0;
			}

			/* mute off */
			OUT32(IFCTL, IN32(IFCTL) & ~0x00000003);

			/* DMAC regstart setting */
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
			siu_out_dmac[port]->DMATCR = 
				port_info->playback.dma_xfer_cnt;
			/* Cycle steal mode. */
			siu_out_dmac[port]->CHCR = 0x00001814;

			siu_obj[port].rw_flg &= ~RWF_STM_WTUR;

			/* only output FIFO enable */
			OUT32(STFIFO, IN32(STFIFO)|(stfifo&0x0c180c18));

			/* DMA transfer enable */
			siu_out_dmac[port]->CHCR |= 0x00000001;
		}
	}
	else{
		PRINTK("else error\n");
		return -EPERM;
	}

	return 0;
}


static int snd_siu_sh7343_stmread(u_int32_t	port,
				  u_int32_t	transfersize)
{
	port_info_t	*port_info = &siu_info.port_info[port];
	u_int32_t	*buff = NULL;

	if (port >= MAX_SIU_PORTS)
		return -EINVAL;
	if (siu_obj[port].stat != ST_OPEN)
		return -EPERM;
	if (siu_obj[port].open_mode == OUTPUT)
		return -EPERM;

	if (transfersize == 0) {	/* stmread stop */
		if (!(siu_obj[port].rw_flg & RWF_STM_RD))
			return -EPERM;

		/* input FIFO disable */
		OUT32(STFIFO, IN32(STFIFO) & ~0x13071307);

		/* DMA Transfer stop */
		/* DE=0 -> transfer disable */
		siu_in_dmac[port]->CHCR = SET_BIT32(&siu_in_dmac[port]->CHCR, 
						    CHCR_DE_BIT, CHCR_DE_WIDTH, 
						    0x00);

		/* during stmread flag clear */
		siu_obj[port].rw_flg &= ~RWF_STM_RD;
	}
	else {	/* stmread start */
		if (transfersize > 16777216)
			return -EINVAL;
		if (!(siu_obj_status & ST_SPBACTIV))
			return -EPERM;	/* SPB stopped? */
		if (siu_obj[port].rw_flg & RWF_STM_RD)
			return -EPERM;

		if (siu_stmrd[port].pkchk == PKCHKON) {
			if (siu_stmrd[port].pk_mtx != 1) {
				return -EINVAL;
			}
		}

		if (port_info->capture.channels == 1)
			buff = port_info->capture.mono_buf;
		else
			buff = port_info->capture.buf;

		/* Current period in buffer */
		port_info->capture.cur_period = 0;

		/* detect silence by software */
		if (siu_stmrd[port].slcchk == DSLON) {
			/* calculate about size of DMA transfer for detect 
			   silence */
			siu_stmrd[port].slccnt = 
				siu_stmrd[port].interval / transfersize;
			if (siu_stmrd[port].slccnt == 0)
				siu_stmrd[port].slccnt=1;
		}

		/* during stmread flag set */
		siu_obj[port].rw_flg |= RWF_STM_RD;

		/* DMA transfer start */
		snd_siu_sh7343_rd_dma_set(port, buff, transfersize);
	}

	return 0;
}


static int snd_siu_sh7343_init(void)
{
	/* External input clock, B 54MHz */
	OUT32(SCLKACR, SCLKACR_EXSRC);
	OUT32(SCLKBCR, SCLKBCR_EXSRC);

	return 0;
}


/* Playback and capture hardware properties are identical */
static struct snd_pcm_hardware snd_siu_sh7343_pcm_hw = {
	.info			= (SNDRV_PCM_INFO_INTERLEAVED),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_8000_48000,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= BUFFER_BYTES_MAX,
	.period_bytes_min	= PERIOD_BYTES_MIN,
	.period_bytes_max	= PERIOD_BYTES_MAX,
	.periods_min		= PERIODS_MIN,
	.periods_max		= PERIODS_MAX,
};


static int snd_siu_sh7343_playback_close(struct snd_pcm_substream *ss)
{
	port_info_t		*port_info = snd_pcm_substream_chip(ss);
	u_int32_t		port = port_info->port;
	u_int32_t		ch = priv.pdata->dma_out_ch;

	PRINTK("port=%d\n", port_info->port);

	port_info->playback.substream = NULL;

	if (port_info->playback.mono_buf) {
		kfree(P1SEGADDR(port_info->playback.mono_buf));
		port_info->playback.mono_buf = NULL;
		port_info->playback.mono_buf_size = 0;
	}

	/* Stop the siu if capture is not using it */
	down(&port_info->sem);
	if (siu_obj_status != ST_STOP && 
	    !(port_info->play_cap & CAPTURE_ENABLED)) {
		snd_siu_sh7343_spbstop();
		siu_close[port_info->port]();
		snd_siu_sh7343_stop();
		codec_stop();
	}
	port_info->play_cap &= ~PLAYBACK_ENABLED;
	up(&port_info->sem);

	/* DMA Extended Resource Selectors. Transfer request module clear. */
	OUT16(DMARS_ADDR(siu_obj[port].och),
			DMARS_VAL(siu_obj[port].och, 0x00));

	/* DE=0 -> transfer disable */
	siu_out_dmac[port]->CHCR = SET_BIT32(&siu_out_dmac[port]->CHCR,
			CHCR_DE_BIT, CHCR_DE_WIDTH, 0x00);
	siu_out_dmac[port]->CHCR = 0x00000000; /* clear */
	siu_out_dmac[port]->DAR = 0x00000000; /* clear */
	siu_out_dmac[port]->SAR = 0x00000000; /* clear */
	siu_out_dmac[port]->DMATCR = 0x00000000; /* clear */
	siu_out_dmac[port] = NULL;          /* clear */

	free_irq(dmac_code[ch], (void *)&siu_info.port_info[port]);
	siu_obj[port].och = -1;  /* Input DMA channel free */

	return 0;
}


static int snd_siu_sh7343_playback_open(struct snd_pcm_substream *ss)
{
	port_info_t		*port_info = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime 	*rt = ss->runtime;
	int			err;
	u_int32_t		port = port_info->port;
	u_int32_t		ch = priv.pdata->dma_out_ch;
	static const u_int16_t reswt[2] = {0xb1, 0xb5}; /* DMA tx request src */

	PRINTK("port=%d\n", port_info->port);
	err = snd_pcm_hw_constraint_integer(rt, SNDRV_PCM_HW_PARAM_PERIODS);
	if (unlikely(err < 0))
		return err;

	rt->hw = snd_siu_sh7343_pcm_hw;
	port_info->playback.substream = ss;

	/* DMA channel register address array get */
	siu_out_dmac[port]  = adddat_dmac[ch];

	/* DMA Extended Resource Selectors. Transfer request module set. */
	OUT16(DMARS_ADDR(ch), DMARS_VAL(ch, reswt[port]));

	siu_obj[port].och = ch; /*  output DMA channel store */

	/* DMA Interrupt handler entry */
	snprintf(siu_obj[port].o_int_desc, 16, "DMA-%d", ch);

	err = request_irq(dmac_code[ch], siu_sh7343_out_dma_isr,
				IRQF_DISABLED,
				siu_obj[port].o_int_desc,
				(void *)&siu_info.port_info[port]);

	if (err) {
		printk(KERN_ERR "request_irq() failed irq="
				"%d[%s], err=%d\n", dmac_code[ch],
				siu_obj[port].o_int_desc, err);
		return err;
	}

	return 0;
}


static int snd_siu_sh7343_hw_params(struct snd_pcm_substream * ss,
				    struct snd_pcm_hw_params * hw_params)
{
	port_info_t	*port_info;
	int		err;

	port_info = snd_pcm_substream_chip(ss);
	PRINTK("port=%d\n", port_info->port);

	err = snd_pcm_lib_malloc_pages(ss, params_buffer_bytes(hw_params));

	if (err < 0)
		printk(KERN_ERR "snd_pcm_lib_malloc_pages() failed\n");

	return err;
}


static int snd_siu_sh7343_hw_free(struct snd_pcm_substream * ss)
{
	port_info_t	*port_info;

	port_info = snd_pcm_substream_chip(ss);
	PRINTK("port=%d\n", port_info->port);

	return snd_pcm_lib_free_pages(ss);

}


static int snd_siu_sh7343_playback_prepare(struct snd_pcm_substream *ss)
{
	port_info_t		*port_info = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime 	*rt = ss->runtime;
	T_DRV_SIU_SPBSELECTA	par_a;
	T_DRV_SIU_SPBSELECTB	par_b;
	T_DRV_SIU_OPEN		open_par;
	int			err = 0;

	port_info->playback.buf = (u_int32_t *)rt->dma_area;
	port_info->playback.buf_bytes = snd_pcm_lib_buffer_bytes(ss);
	port_info->playback.period_bytes = snd_pcm_lib_period_bytes(ss);

	/* We only support buffers that are multiples of the period */
	if (port_info->playback.buf_bytes % port_info->playback.period_bytes) {
		printk(KERN_ERR "snd_siu_sh7343_playback_prepare() - "
		       "buffer=%d not multiple of period=%d\n",
		       port_info->playback.buf_bytes, 
		       port_info->playback.period_bytes);
		return -EINVAL;
	}

	port_info->playback.rate = rt->rate;
	port_info->playback.format = rt->format;
	port_info->playback.channels = rt->channels;
	port_info->playback.dma_xfer_cnt = 
		port_info->playback.period_bytes / (rt->frame_bits / 8);

	PRINTK("port=%d buf=%p buf_bytes=%d period_bytes=%d rate=%d format=%d "
	       "channels=%d dma_xfer_cnt=%d\n", port_info->port, 
	       port_info->playback.buf, port_info->playback.buf_bytes, 
	       port_info->playback.period_bytes, port_info->playback.rate, 
	       port_info->playback.format, port_info->playback.channels,
	       port_info->playback.dma_xfer_cnt);

	/* The hardware only supports stereo (2 channels) streams . We must 
	   convert mono streams (1 channel) to stereo streams. To do that we 
	   just copy the mono data to one of the stereo channels and instruct
	   the siu to play the data on both channels. However, the idle 
	   channel must also be present in the buffer, so we use an extra 
	   buffer twice as big as one mono period. Also since this function
	   can be called multiple times, we must adjust the the buffer size */
	if (port_info->playback.channels == 1) {
		if (port_info->playback.mono_buf && 
		    port_info->playback.mono_buf_size != 
		    port_info->playback.period_bytes * 2) {
			kfree(P1SEGADDR(port_info->playback.mono_buf));
			port_info->playback.mono_buf = NULL;
			port_info->playback.mono_buf_size = 0;
		}

		if (!port_info->playback.mono_buf) {
			if ((port_info->playback.mono_buf = 
			     kmalloc(port_info->playback.period_bytes * 2, 
				     GFP_KERNEL)) == NULL)
				return -ENOMEM;

			/* Must convert it to a p2 area buffer to disable 
			   caching since the dma engine is not cache coherent */
			port_info->playback.mono_buf = 
				P2SEGADDR(port_info->playback.mono_buf);
			port_info->playback.mono_buf_size = 
				port_info->playback.period_bytes * 2;
		}
	}

	down(&port_info->sem);

	/* Set up the siu if not already done */
	if (!(port_info->play_cap & (CAPTURE_ENABLED | PLAYBACK_ENABLED))) {
		if ((err = codec_start(priv.pdata->master, port_info->playback.rate,
                                       port_info->playback.channels,1))) {
			printk(KERN_ERR "codec_start() failed err=%d\n",
			       err);
			goto fail;
		}

		if ((err = snd_siu_sh7343_start(port_info,1))) {
			printk(KERN_ERR "snd_siu_sh7343_start() failed "
			       "err=%d\n", err);
			goto fail;
		}

		par_a.ena     = PATHAON;
		par_a.inport  = CPUOUTPUT;
		par_a.outport = out_port[port_info->port];
		par_a.secbuff = 16;
		par_a.fir     = FIRTHR;
		par_a.mix     = MIXOFF;
		par_a.iir     = IIROFF;
		par_a.scr     = SCROFF;
		if ((err = snd_siu_sh7343_spbAselect(&par_a)))
			goto fail;

		par_b.ena = PATHBON;
		par_b.inport = in_port[port_info->port];
		par_b.outport = CPUINPUT;
		if ((err = snd_siu_sh7343_spbBselect(&par_b)))
			goto fail;

		open_par.form = priv.pdata->data_format;
		if (port_info->playback.channels >= priv.pdata->stereo_channels)
			open_par.mona = STEREO;
		else
			open_par.mona = MONAURAL;
		if ((err = siu_open[port_info->port](INOUTPUT, &open_par, 
						     &open_par))) {
			goto fail;
		}

		if ((err = snd_siu_sh7343_pcmdatapack(port_info->port, 
						      LuLlRuRl))) {
			goto fail;
		}

		if ((err = snd_siu_sh7343_spbstart()))
			goto fail;
	}

	port_info->play_cap |= PLAYBACK_ENABLED;

fail:
	up(&port_info->sem);
	return err;
}


static int snd_siu_sh7343_playback_trigger(struct snd_pcm_substream *ss,
					   int			    cmd)
{
	port_info_t		*port_info = snd_pcm_substream_chip(ss);
	int			err = 0;

	PRINTK("cmd=%d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* Enable DMA */
		if ((err = snd_siu_sh7343_defdma(port_info->port, OUTPUT, priv.pdata->dma_out_ch))) {
			goto fail;
		}

		if ((err = snd_siu_sh7343_stmwrite(port_info->port, 
						   port_info->playback.dma_xfer_cnt))) {
			goto fail;
		}

		err = snd_siu_sh7343_stmwriteenable(port_info->port);

		break;

	case SNDRV_PCM_TRIGGER_STOP:
		snd_siu_sh7343_stmwrite(port_info->port, 0);
		snd_siu_sh7343_defdma(port_info->port, OUTPUT, -1);

		break;

	default:
		printk(KERN_ERR "snd_siu_sh7343_playback_trigger() invalid "
		       "cmd=%d\n", cmd);
		err = -EINVAL;
	}

fail:
	return err;
}


static snd_pcm_uframes_t snd_siu_sh7343_playback_pointer(struct snd_pcm_substream *ss)
{
	port_info_t	*port_info = snd_pcm_substream_chip(ss);
	size_t		ptr;
	int		mono_offset;

	/* ptr is the offset into the buffer where the dma is currently at. We
	   check if the dma buffer has just wrapped. For mono streams we must
	   do extra calculations. */
	if (port_info->playback.channels == 1) {
		ptr = PERIOD_OFFSET(port_info->playback.buf, 
				    port_info->playback.cur_period, 
				    port_info->playback.period_bytes) -
			(int)port_info->playback.buf;
		mono_offset = (siu_out_dmac[port_info->port]->SAR - 
			       (int)port_info->playback.mono_buf) / 2;
		ptr += mono_offset;
	}
	else {
		ptr = siu_out_dmac[port_info->port]->SAR - 
			(int)port_info->playback.buf;
	}

	if (ptr >= port_info->playback.buf_bytes)
		ptr = 0;

	return bytes_to_frames(ss->runtime, ptr);
}


static int snd_siu_sh7343_capture_close(struct snd_pcm_substream *ss)
{
	port_info_t		*port_info = snd_pcm_substream_chip(ss);
	u_int32_t		port = port_info->port;
	u_int32_t		ch = priv.pdata->dma_in_ch;

	PRINTK("port=%d\n", port_info->port);

	port_info->capture.substream = NULL;

	if (port_info->capture.mono_buf) {
		kfree(P1SEGADDR(port_info->capture.mono_buf));
		port_info->capture.mono_buf = NULL;
		port_info->capture.mono_buf_size = 0;
	}

	/* Stop the siu if playback is not using it */
	down(&port_info->sem);
	if (siu_obj_status != ST_STOP && 
	    !(port_info->play_cap & PLAYBACK_ENABLED)) {
		snd_siu_sh7343_spbstop();
		siu_close[port_info->port]();
		snd_siu_sh7343_stop();
		codec_stop();
	}
	port_info->play_cap &= ~CAPTURE_ENABLED;
	up(&port_info->sem);

	/* DMA Extended Resource Selectors. Transfer request module clear */
	OUT16(DMARS_ADDR(siu_obj[port].ich),
	DMARS_VAL(siu_obj[port].ich, 0x00));

	/* DE=0 -> transfer disable */
	siu_in_dmac[port]->CHCR = SET_BIT32(&siu_in_dmac[port]->CHCR,
				CHCR_DE_BIT, CHCR_DE_WIDTH, 0x00);
	siu_in_dmac[port]->CHCR = 0x00000000; /* clear */
	siu_in_dmac[port]->DAR = 0x00000000; /* clear */
	siu_in_dmac[port]->SAR = 0x00000000; /* clear */
	siu_in_dmac[port]->DMATCR = 0x00000000; /* clear */
	siu_in_dmac[port] = NULL;          /* clear */

	free_irq(dmac_code[ch], (void *)&siu_info.port_info[port]);
	siu_obj[port].ich = -1; /* Input DMA channel free */

	return 0;
}


static int snd_siu_sh7343_capture_open(struct snd_pcm_substream	*ss)
{
	port_info_t		*port_info = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime 	*rt = ss->runtime;
	int			err;
	u_int32_t		port = port_info->port;
	u_int32_t		ch = priv.pdata->dma_in_ch;
	static const u_int16_t resrd[2] = {0xb2, 0xb6}; /* DMA rx request src */

	PRINTK("port=%d\n", port_info->port);

	err = snd_pcm_hw_constraint_integer(rt, SNDRV_PCM_HW_PARAM_PERIODS);
	if (unlikely(err < 0))
		return err;

	rt->hw = snd_siu_sh7343_pcm_hw;
	port_info->capture.substream = ss;

	/* DMA channel register address array get */
	siu_in_dmac[port]  = adddat_dmac[ch];

	/* DMA Extended Resource Selectors. Transfer request module set. */
	OUT16(DMARS_ADDR(ch), DMARS_VAL(ch, resrd[port]));

	siu_obj[port].ich = ch; /*  input DMA channel store */

	/* DMA Interrupt handler entry */
	snprintf(siu_obj[port].i_int_desc, 16, "DMA-%d", ch);
	err = request_irq(dmac_code[ch], siu_sh7343_in_dma_isr,
				IRQF_DISABLED,
				siu_obj[port].i_int_desc,
				(void *)&siu_info.port_info[port]);
	if (err) {
		printk(KERN_ERR "request_irq() failed irq="
				"%d[%s], err=%d\n", dmac_code[ch],
				siu_obj[port].i_int_desc, err);
		return err;
	}

	return 0;
}


static int snd_siu_sh7343_capture_prepare(struct snd_pcm_substream *ss)
{
	port_info_t		*port_info = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime 	*rt = ss->runtime;
	T_DRV_SIU_SPBSELECTA	par_a;
	T_DRV_SIU_SPBSELECTB	par_b;
	T_DRV_SIU_OPEN		open_par;
	int			err = 0;

	port_info->capture.buf = (u_int32_t *)rt->dma_area;
	port_info->capture.buf_bytes = snd_pcm_lib_buffer_bytes(ss);
	port_info->capture.period_bytes = snd_pcm_lib_period_bytes(ss);

	/* We only support buffers that are multiples of the period */
	if (port_info->capture.buf_bytes % port_info->capture.period_bytes) {
		printk(KERN_ERR "snd_siu_sh7343_capture_prepare() - "
		       "buffer=%d not multiple of period=%d\n",
		       port_info->capture.buf_bytes, 
		       port_info->capture.period_bytes);
		return -EINVAL;
	}

	port_info->capture.rate = rt->rate;
	port_info->capture.format = rt->format;
	port_info->capture.channels = rt->channels;
	port_info->capture.dma_xfer_cnt =
		port_info->capture.period_bytes / (rt->frame_bits / 8);

	PRINTK("port=%d buf=%p buf_bytes=%d period_bytes=%d rate=%d format=%d "
	       "channels=%d dma_xfer_cnt=%d\n", port_info->port, 
	       port_info->capture.buf, port_info->capture.buf_bytes, 
	       port_info->capture.period_bytes, port_info->capture.rate, 
	       port_info->capture.format, port_info->capture.channels,
	       port_info->capture.dma_xfer_cnt);

	/* The hardware only supports stereo (2 channels) streams . We must 
	   convert stereo streams (2 channels) to mono streams. To do that we 
	   just copy one of the stereo channels into the alsa buffer. For
	   this we need an extra buffer twice as big as one mono period. Also 
	   since this function can be called multiple times, we must adjust 
	   the buffer size */
	if (port_info->capture.channels == 1) {
		if (port_info->capture.mono_buf &&
		    port_info->capture.mono_buf_size != 
		    port_info->capture.period_bytes * 2) {
			kfree(P1SEGADDR(port_info->capture.mono_buf));
			port_info->capture.mono_buf = NULL;
			port_info->capture.mono_buf_size = 0;
		}

		if (!port_info->capture.mono_buf) {
			if ((port_info->capture.mono_buf = 
			     kmalloc(port_info->capture.period_bytes * 2, 
				     GFP_KERNEL)) == NULL)
				return -ENOMEM;

			/* Must convert it to a p2 area buffer to disable 
			   caching since the dma engine is not cache coherent */
			port_info->capture.mono_buf = 
				P2SEGADDR(port_info->capture.mono_buf);
			port_info->capture.mono_buf_size = 
				port_info->capture.period_bytes * 2;
		}
	}

	down(&port_info->sem);

	/* Set up the siu if not already done */
	if (!(port_info->play_cap & (CAPTURE_ENABLED | PLAYBACK_ENABLED))) {
		if ((err = codec_start(priv.pdata->master, port_info->capture.rate,
                                       port_info->playback.channels,0))) {
			printk(KERN_ERR "codec_start() failed err=%d\n",
			       err);
			goto fail;
		}

		if ((err = snd_siu_sh7343_start(port_info,0))) {
			printk(KERN_ERR "snd_siu_sh7343_start() failed "
			       "err=%d\n", err);
			goto fail;
		}

		par_a.ena     = PATHAON;
		par_a.inport  = out_port[port_info->port];
		par_a.outport = CPUOUTPUT;
		par_a.secbuff = 16;
		par_a.fir     = FIRTHR;
		par_a.mix     = MIXOFF;
		par_a.iir     = IIROFF;
		par_a.scr     = SCROFF;
		if ((err = snd_siu_sh7343_spbAselect(&par_a)))
			goto fail;

		par_b.ena = PATHBON;
		par_b.inport = in_port[port_info->port];
		par_b.outport = CPUINPUT;
		if ((err = snd_siu_sh7343_spbBselect(&par_b)))
			goto fail;

		open_par.form = priv.pdata->data_format;
		if (port_info->capture.channels >= priv.pdata->stereo_channels)
			open_par.mona = STEREO;
		else
			open_par.mona = MONAURAL;
		if ((err = siu_open[port_info->port](INOUTPUT, &open_par, 
						     &open_par))) {
			goto fail;
		}

		if ((err = snd_siu_sh7343_pcmdatapack(port_info->port, 
						      LuLlRuRl))) {
			goto fail;
		}

		if ((err = snd_siu_sh7343_spbstart()))
			goto fail;
	}

	port_info->play_cap |= CAPTURE_ENABLED;

fail:
	up(&port_info->sem);
	return err;
}


static int snd_siu_sh7343_capture_trigger(struct snd_pcm_substream *ss,
					  int			   cmd)
{
	port_info_t		*port_info = snd_pcm_substream_chip(ss);
	int			err = 0;

	PRINTK("cmd=%d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* Enable DMA */
		if ((err = snd_siu_sh7343_defdma(port_info->port, INPUT, priv.pdata->dma_in_ch)))
			goto fail;

		err = snd_siu_sh7343_stmread(port_info->port,
					     port_info->capture.dma_xfer_cnt);

		break;

	case SNDRV_PCM_TRIGGER_STOP:
		snd_siu_sh7343_stmread(port_info->port, 0);
		snd_siu_sh7343_defdma(port_info->port, INPUT, -1);

		break;

	default:
		printk(KERN_ERR "snd_siu_sh7343_capture_trigger "
		       "invalid cmd=%d\n", cmd);
		err = -EINVAL;
	}

fail:
	return err;
}


static snd_pcm_uframes_t snd_siu_sh7343_capture_pointer(struct snd_pcm_substream *ss)
{
	port_info_t	*port_info = snd_pcm_substream_chip(ss);
	size_t		ptr;
	int		mono_offset;

	/* ptr is the offset into the buffer where the dma is currently at. We
	   check if the dma buffer has just wrapped. For mono streams we must
	   calculate the offset into the mono buffer. */
	if (port_info->capture.channels == 1) {
		ptr = PERIOD_OFFSET(port_info->capture.buf, 
				    port_info->capture.cur_period, 
				    port_info->capture.period_bytes) -
			(int)port_info->capture.buf;
		mono_offset = (siu_in_dmac[port_info->port]->DAR - 
			       (int)port_info->capture.mono_buf) / 2;
		ptr += mono_offset;
	}
	else {
		ptr = siu_in_dmac[port_info->port]->DAR - 
			(int)port_info->capture.buf;
	}

	if (ptr >= port_info->capture.buf_bytes)
		ptr = 0;

	return bytes_to_frames(ss->runtime, ptr);
}


static int snd_sh7343_info_volume(struct snd_kcontrol *kctrl, 
				  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = priv.pdata->max_volume;

	return 0;
}


static int snd_sh7343_get_volume(struct snd_kcontrol *kctrl,
				 struct snd_ctl_elem_value *ucontrol)
{
	port_info_t	*port_info;
	u_int32_t	vol;

	port_info = snd_kcontrol_chip(kctrl);

	switch(kctrl->private_value) {
	case VOLUME_PLAYBACK:
		/* Playback is always on port 0 */
		vol = port_info->playback.volume;
		ucontrol->value.integer.value[0] = vol & 0xffff;
		ucontrol->value.integer.value[1] = vol >> 16 & 0xffff;
		break;

	case VOLUME_CAPTURE:
		/* Capture is always on port 1 */
		vol = port_info->capture.volume;
		ucontrol->value.integer.value[0] = vol & 0xffff;
		ucontrol->value.integer.value[1] = vol >> 16 & 0xffff;
		break;
	}

	return 0;
}


static int snd_sh7343_put_volume(struct snd_kcontrol *kctrl,
				 struct snd_ctl_elem_value *ucontrol)
{
	port_info_t	*port_info;
	u_int32_t	new_vol;
	u_int32_t	cur_vol;

	port_info = snd_kcontrol_chip(kctrl);

	if(ucontrol->value.integer.value[0] < 0 || 
	   ucontrol->value.integer.value[0] > priv.pdata->max_volume ||
	   ucontrol->value.integer.value[1] < 0 ||
	   ucontrol->value.integer.value[1] > priv.pdata->max_volume)
		return -EINVAL;

	new_vol = ucontrol->value.integer.value[0] | 
		ucontrol->value.integer.value[1] << 16;

	switch(kctrl->private_value) {
	case VOLUME_PLAYBACK:
		cur_vol = port_info->playback.volume;
		port_info->playback.volume = new_vol;
		break;

	case VOLUME_CAPTURE:
		cur_vol = port_info->capture.volume;
		port_info->capture.volume = new_vol;
		break;

	default:
		printk(KERN_ERR "snd_sh7343_put_volume() invalid "
		       "private_value=%ld\n", kctrl->private_value);
		return -ENOMSG;
	}

	if (cur_vol == new_vol)
		return 1;

        /* when changing even only one PCM values we are called two times with
         * PCM Playback and PCM Capture. Only use the value for the mode we are
         * currently operating. Or just store the value when device is not open */
        if (kctrl->private_value == VOLUME_PLAYBACK &&
            (port_info->play_cap & CAPTURE_ENABLED))
                return 0;
        if (kctrl->private_value == VOLUME_CAPTURE &&
            (port_info->play_cap & PLAYBACK_ENABLED))
                return 0;

        OUT32(sbdvc[priv.pdata->port_in_use], new_vol);
        OUT32(SBDVCB, new_vol);

	return 0;
}


static void snd_siu_sh7343_pcm_free(struct snd_pcm *pcm)
{
	PRINTK("pcm_free()\n");
	snd_pcm_lib_preallocate_free_for_all(pcm);
}


static struct snd_pcm_ops snd_sh7343_pcm_playback_ops = {
	.open		=	snd_siu_sh7343_playback_open,
	.close		=	snd_siu_sh7343_playback_close,
	.ioctl		=	snd_pcm_lib_ioctl,
	.hw_params	=	snd_siu_sh7343_hw_params,
	.hw_free	=	snd_siu_sh7343_hw_free,
	.prepare	=	snd_siu_sh7343_playback_prepare,
	.trigger	=	snd_siu_sh7343_playback_trigger,
	.pointer	=	snd_siu_sh7343_playback_pointer,
};


static struct snd_pcm_ops snd_sh7343_pcm_capture_ops = {
	.open		=	snd_siu_sh7343_capture_open,
	.close		=	snd_siu_sh7343_capture_close,
	.ioctl		=	snd_pcm_lib_ioctl,
	.hw_params	=	snd_siu_sh7343_hw_params,
	.hw_free	=	snd_siu_sh7343_hw_free,
	.prepare	=	snd_siu_sh7343_capture_prepare,
	.trigger	=	snd_siu_sh7343_capture_trigger,
	.pointer	=	snd_siu_sh7343_capture_pointer,
};


static struct snd_kcontrol_new playback_controls __initdata = {
	.iface		=	SNDRV_CTL_ELEM_IFACE_MIXER,
	.name		=	"PCM Playback Volume",
	.index		=	0,
	.info		=	snd_sh7343_info_volume,
	.get		=	snd_sh7343_get_volume,
	.put		=	snd_sh7343_put_volume,
	.private_value	=	VOLUME_PLAYBACK,
};


static struct snd_kcontrol_new capture_controls __initdata = {
	.iface		=	SNDRV_CTL_ELEM_IFACE_MIXER,
	.name		=	"PCM Capture Volume",
	.index		=	0,
	.info		=	snd_sh7343_info_volume,
	.get		=	snd_sh7343_get_volume,
	.put		=	snd_sh7343_put_volume,
	.private_value	=	VOLUME_CAPTURE,
};


static int snd_siu_sh7343_dev_free(struct snd_device *dev)
{
	/* Nothing to free */
	return 0;
}


static struct snd_device_ops snd_siu_sh7343_dev_ops = {
	.dev_free		= snd_siu_sh7343_dev_free,
};


static int __init alsa_card_siu_sh7343_init(void)
{
	struct snd_card	*card;
	struct snd_pcm	*pcm;
        int		err;
	int		i;
	struct snd_kcontrol *kctrl;

	printk(KERN_INFO "siu-sh7343: ");

        priv.clk = clk_get(priv.dev, "siu0");
        BUG_ON(clk_get == NULL);

	siu_obj_status = ST_STOP;
	siu_info.card = NULL;
	for (i = 0; i < MAX_SIU_PORTS; i++) {
		siu_obj[i].stat = ST_STOP;
	}

	for (i = 0; i < MAX_SIU_PORTS; i++) {
		siu_info.port_info[i].port = i;
		init_MUTEX(&siu_info.port_info[i].sem);
		siu_info.port_info[i].play_cap = 0;
		siu_info.port_info[i].pcm = NULL;

		siu_info.port_info[i].playback.substream = NULL;
		siu_info.port_info[i].playback.rate = 0;
		siu_info.port_info[i].playback.format = 0;
		siu_info.port_info[i].playback.channels = 0;
		siu_info.port_info[i].playback.buf = NULL;
		siu_info.port_info[i].playback.buf_bytes = 0;
		siu_info.port_info[i].playback.period_bytes = 0;
		siu_info.port_info[i].playback.cur_period = 0;
		siu_info.port_info[i].playback.mono_buf = NULL;
		siu_info.port_info[i].playback.mono_buf_size = 0;
		siu_info.port_info[i].playback.volume = DFLT_VOLUME_LEVEL;

		siu_info.port_info[i].capture.substream = NULL;
		siu_info.port_info[i].capture.rate = 0;
		siu_info.port_info[i].capture.format = 0;
		siu_info.port_info[i].capture.channels = 0;
		siu_info.port_info[i].capture.buf = NULL;
		siu_info.port_info[i].capture.buf_bytes = 0;
		siu_info.port_info[i].capture.period_bytes = 0;
		siu_info.port_info[i].capture.cur_period = 0;
		siu_info.port_info[i].capture.mono_buf = NULL;
		siu_info.port_info[i].capture.mono_buf_size = 0;
		siu_info.port_info[i].capture.volume = DFLT_VOLUME_LEVEL;
	}

	/* Initialize siu pins. Siu will be left in power down mode */
	snd_siu_sh7343_init();

	/* Create card */
        err = snd_card_create(0, "siu-sh7343", THIS_MODULE, 0, &card);
	if (err < 0)
		return -err;

	strcpy(card->driver, "siu-sh7343");
	strcpy(card->shortname, "Renesas sh7343 SIU");
	sprintf(card->longname, "%s irq %d", card->shortname, SIU_IRQ);

	/* Attach components to the card */
	if ((err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, &siu_info, 
				  &snd_siu_sh7343_dev_ops)) < 0) {
		printk(KERN_ERR "snd_device_new() failed\n");
		goto fail;
	}

	/* While the siu has 2 ports, only one port can be on at a time (only 1 
	   SPB). So far all the boards using the siu had only one of the ports
	   wired to a codec. To simplify things, we only register one port with
	   alsa. In case both ports are needed, it should be changed here  */
	for (i = priv.pdata->port_in_use; i < priv.pdata->port_in_use + 1; i++) {
		if ((err = snd_pcm_new(card, 
				       "siu-sh7343",	/* ID */
				       0,		/* device */
				       1,		/* playback count */
				       1,		/* capture count */
				       &pcm)) < 0) {
			printk(KERN_ERR "snd_pcm_new() failed err=%d\n", err);
			goto fail;
		}

		/* Register pcm callbacks */
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, 
				&snd_sh7343_pcm_playback_ops);
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, 
				&snd_sh7343_pcm_capture_ops);

		if ((err = snd_pcm_lib_preallocate_pages_for_all(pcm, 
                                                                 SNDRV_DMA_TYPE_CONTINUOUS,

                                                                 /* SNDRV_DMA_TYPE_DEV,->snd_dma_continuous_data(GFP_KERNEL) needs to be replaced
   with device*/
					snd_dma_continuous_data(GFP_KERNEL),
					BUFFER_BYTES_MAX, 
					BUFFER_BYTES_MAX)) < 0) {
			printk(KERN_ERR "snd_pcm_lib_preallocate_pages_for_"
			       "all() failed err=%d", err);
			goto fail;
		}

		strcpy(pcm->name, card->shortname);
		pcm->private_free = snd_siu_sh7343_pcm_free;
		pcm->private_data = &siu_info.port_info[i];
		siu_info.port_info[i].pcm = pcm;

	}
	
	/* Add mixer support. The SPB is used to change the volume. Both ports
	   use the same SPB. Therefore, we only register one control instance
	   since it will be used by both channels */
	kctrl = snd_ctl_new1(&playback_controls, 
			     &siu_info.port_info[priv.pdata->port_in_use]);
	if ((err = snd_ctl_add(card, kctrl)) < 0) {
		printk(KERN_ERR "snd_ctl_add() failed to add playback "
		       "controls port=%d err=%d\n", i, err);
		goto fail;
	}

	kctrl = snd_ctl_new1(&capture_controls, 
			     &siu_info.port_info[priv.pdata->port_in_use]);
	if ((err = snd_ctl_add(card, kctrl)) < 0) {
		printk(KERN_ERR "snd_ctl_add() failed to add capture "
		       "controls port=%d err=%d\n", i, err);
		goto fail;
	}

        if ((err = codec_register_controls(card)) < 0) {
		printk(KERN_ERR "Codec's register control failed err=%d\n", err);
                goto fail;
        }

	strcpy(card->mixername, card->shortname);

	/* Register our isr */
	if ((err = request_irq(SIU_IRQ, siu_sh7343_isr, IRQF_DISABLED, 
					      "SIU", NULL))) {
		printk(KERN_ERR "request_irq() failed\n");
		goto fail;
	}

	/* Now we can register the card instance */
	if ((err = snd_card_register(card)) < 0) {
		printk(KERN_ERR "snd_card_register()() failed err=%d\n", err);
		free_irq(SIU_IRQ, NULL);
		goto fail;
	}

        printk("(%s) ", priv.pdata->master ? "master" : "slave" );

	siu_info.card = card;
	printk("initialized.\n");
        return 0;

fail:
	snd_card_free(card);
        clk_put(priv.clk);
	printk("failed to initialize.\n");
	return err;
}

static void alsa_card_siu_sh7343_exit(void)
{
	snd_card_free(siu_info.card);
	free_irq(SIU_IRQ, NULL);
        clk_put(priv.clk);
	printk(KERN_INFO "siu-sh7343: exited\n");
}

static int __init siu_sh7343_probe(struct platform_device *pdev)
{
	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "no platform data defined\n");
                return -EINVAL;
	}

        priv.pdata = pdev->dev.platform_data;

        alsa_card_siu_sh7343_init();

        return 0;
}

static int siu_sh7343_remove(struct platform_device *pdev)
{
        alsa_card_siu_sh7343_exit();

        return 0;
}

static struct platform_driver siu_sh7343_driver = {
        .driver = {
                .name  = "siu_sh7343",
                .owner = THIS_MODULE,
        },
        .probe  = siu_sh7343_probe,
        .remove = siu_sh7343_remove,
};

static int __init siu_sh7343_init(void)
{
        return platform_driver_register(&siu_sh7343_driver);
}

static void __exit siu_sh7343_exit(void)
{
        platform_driver_unregister(&siu_sh7343_driver);
}

MODULE_AUTHOR("Carlos Munoz <carlos@kenati.com>");
MODULE_DESCRIPTION("Alsa SH7343 SIU driver");
MODULE_LICENSE("GPL");

module_init(siu_sh7343_init)
module_exit(siu_sh7343_exit)
