/*
 *  siu-sh7343.h - Alsa driver for Renesas' SH7343 SIU peripheral.
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
#include <linux/interrupt.h>
#include <sound/core.h>
#include <sound/pcm.h>

#ifndef SIU_SH7343_H
#define SIU_SH7343_H

#include "siu_sh7343_pdata.h"

#define DEBUG 0
#if DEBUG
# define PRINTK(fmt, args...) \
printk(KERN_DEBUG "%s:%d: " fmt, __FUNCTION__, __LINE__, ##args)
#else
# define PRINTK(fmt, args...)
#endif

#define MAX_SIU_PORTS		2

#define DMTE0_IRQ	48
#define DMTE1_IRQ	49
#define DMTE2_IRQ	50
#define DMTE3_IRQ	51
#define DMTE4_IRQ	76
#define DMTE5_IRQ	77

#define SIU_IRQ		108

typedef struct {
	u_int32_t	ab1a;	/* input FIFO address */
	u_int32_t	ab0a;	/* output FIFO address */
	u_int32_t	dir;	/* 0=the ather except CPUOUTPUT, 1=CPUINPUT */
	u_int32_t	event;	/* SPB program starting conditions */
	u_int32_t	stfifo;	/* STFIFO register setting value */
	u_int32_t	trdat;	/* TRDAT register setting value */
} T_SPBPAR;

/* SIU driver dynamic information */
typedef struct {
	u_int32_t	stat;		/* SIU driver object status */
	u_int8_t	open_mode;	/* SIU driver open-mode */
	u_int8_t	rw_flg;		/* stream-data transfer status */
	int32_t		ich;		/* input DMA channel */
	int32_t		och;		/* output DMA channel */
	char		i_int_desc[16];	/* Interrupt description */
	char		o_int_desc[16];	/* Interrupt description */
} T_SIU;

typedef struct {
	u_int32_t	undruncnt;	/* buf under-run xfer restart cnt */
	u_int32_t	undctr;		/* buf under-run xfer restart cntr */
	u_int32_t	**rw_pklev;	/* L-R channel data peak level stored 
					   start address. */
	u_int8_t	pk_mtx;		/* total of peaklevel buffer array */
	u_int32_t	pkchk;		/* input or output data peakcheck 
					   status */
	u_int16_t	peak;		/* peak level of input or output 
					   stream-data */
	u_int32_t	dsl;		/* detect silence status */
	u_int32_t	slcchk;		/* silence check status */
	u_int32_t	interval;	/* period of detect silence by 
					   software.(:second) */
	u_int16_t	level;		/* level of detect silence by 
					   software. */
	u_int32_t	slccnt;		/* silence time count */
	u_int32_t	slcctr;		/* silence time counter */
} SIU_STMRW;

struct st_dmac {
	u_int32_t	SAR;
	u_int32_t	DAR;
	u_int32_t	DMATCR;
	u_int32_t	CHCR;
};

typedef struct {
	struct snd_pcm_substream	*substream;
	u_int32_t		rate;
	snd_pcm_format_t	format;
	u_int32_t		channels;
	u_int32_t		*buf;		/* dma area */
	u_int32_t		buf_bytes;
	u_int32_t		period_bytes;
	u_int32_t		cur_period;	/* Period currently in dma */
	u_int32_t		dma_xfer_cnt;
	u_int32_t		*mono_buf;
	u_int32_t		mono_buf_size;
	u_int32_t		volume;
} stream_t;

typedef struct {
	u_int32_t		port;
	struct semaphore	sem;
	u_int32_t		play_cap;	/* Used to track full duplex */
	struct snd_pcm		*pcm;
	stream_t		playback;
	stream_t		capture;
} port_info_t;

typedef struct {
	struct snd_card		*card;
	port_info_t		port_info[MAX_SIU_PORTS];
} siu_info_t;
	
extern irqreturn_t siu_sh7343_in_dma_isr(int irq, void *dev_id);
extern irqreturn_t siu_sh7343_out_dma_isr(int irq, void *dev_id);
extern irqreturn_t siu_sh7343_isr(int irq, void *dev_id);
extern int codec_start(int master, u_int32_t rate, u_int32_t channels, int playback);
extern int codec_stop(void);
extern int codec_register_controls(struct snd_card *card);
extern const u_int32_t SIU_SPB_PRO_Y0[];
extern const u_int32_t SIU_SPB_PRO_Y1[];
extern const u_int32_t SIU_SPB_PRO_Y2[];
extern const u_int32_t SIU_SPB_PRO_Y3[];
extern const u_int32_t SIU_SPB_PRO_Y4[];
extern const u_int32_t SIU_SPB_PRO_P0[];
extern const u_int32_t SIU_SPB_PRO_P1[];

extern const u_int32_t SIU_SRC_8000_44100[];
extern const u_int32_t SIU_SRC_11025_44100[];
extern const u_int32_t SIU_SRC_16000_44100[];
extern const u_int32_t SIU_SRC_22050_44100[];
extern const u_int32_t SIU_SRC_32000_44100[];
extern const u_int32_t SIU_SRC_44100_44100[];
extern const u_int32_t SIU_SRC_48000_44100[];

extern const u_int32_t SIU_SRC_44100_8000[];
extern const u_int32_t SIU_SRC_44100_16000[];
extern const u_int32_t SIU_SRC_44100_22050[];

extern siu_info_t siu_info;
extern const T_SPBPAR t_spbpar[];
extern const u_int32_t dmain[];
extern const u_int32_t dmaout[];
extern volatile struct st_dmac *siu_out_dmac[];
extern volatile struct st_dmac *siu_in_dmac[];
extern T_SIU siu_obj[];
extern SIU_STMRW siu_stmwt[];
extern SIU_STMRW siu_stmrd[];


#define PLAYBACK_ENABLED	0x00000001
#define CAPTURE_ENABLED		0x00000002

#define VOLUME_CAPTURE		0
#define VOLUME_PLAYBACK		1
#define DFLT_VOLUME_LEVEL	0x08000800

#define MELDUSE 0

#define PERIOD_BYTES_MAX	8192		/* DMA transfer/period size */
#define PERIOD_BYTES_MIN	256		/* DMA transfer/period size */
#define PERIODS_MAX		64		/* Max periods in buffer */
#define PERIODS_MIN		4		/* Min periods in buffer */
#define BUFFER_BYTES_MAX	(PERIOD_BYTES_MAX * PERIODS_MAX)
#define GET_MAX_PERIODS(buf_bytes, period_bytes) \
                                ((buf_bytes) / (period_bytes))
#define PERIOD_OFFSET(buf_addr, period_num, period_bytes) \
	                        ((int)(buf_addr) + ((period_num) * (period_bytes)))

static inline void copy_playback_period(port_info_t	*port_info)
{
	int		i;
	u_int16_t	*src;
	u_int32_t	*dst;
	int		cp_cnt;

	src = (u_int16_t *)PERIOD_OFFSET(port_info->playback.buf, 
					 port_info->playback.cur_period, 
					 port_info->playback.period_bytes);
	dst = port_info->playback.mono_buf;
	cp_cnt = port_info->playback.dma_xfer_cnt;

	for (i = 0; i < cp_cnt; i++) {
		*dst++ = *src++;
	}		
}

static inline void copy_capture_period(port_info_t	*port_info)
{
	int		i;
	u_int16_t	*src;
	u_int16_t	*dst;
	int		cp_cnt;

	dst = (u_int16_t *)PERIOD_OFFSET(port_info->capture.buf, 
					 port_info->capture.cur_period, 
					 port_info->capture.period_bytes);
	src = (u_int16_t *)port_info->capture.mono_buf;
	cp_cnt = port_info->capture.dma_xfer_cnt;

	for (i = 0; i < cp_cnt; i++) {
		*dst++ = *src;
		src += 2;
	}		
}

#define PKCHKOFF		0x00000000	/* detect peak level by soft 
						   OFF */
#define PKCHKON			0x00000001	/* detect peak level by soft ON */
#define GEQOFF			0x00000000	/* GEQ disable */
#define GEQON			0x00000001	/* GEQ enable */
#define INPUT			0x00000000	/* input */
#define OUTPUT			0x00000001	/* output */
#define INOUTPUT		0x00000002	/* input output */
#define NODEVICE		0x00000000	/* input and output off */
#define SPDIF			0x00000003	/* SPDIF */
#define PCMSPDIF		0x00000004	/* PCM & SPDIF */
#define I2SSPDIF		0x00000005	/* I2S & SPDIF */
#define STEREO			0x00000000	/* stereo */
#define MONAURAL		0x00000001	/* nonaural */
#define DSLOFF			0x00000000	/* detect silence off */
#define DSLON			0x00000001	/* detect silence on */
#define DPEAKOFF		0x00000000	/* detect peak level OFF */
#define DPEAKON			0x00000001	/* detect peak level ON */
#define SPQOFF			0x00000000	/* detect Q-code OFF */
#define SPQON			0x00000001	/* detect Q-code ON */
#define CHSTOFF			0x00000000	/* detect channel status OFF */
#define CHSTON			0x00000001	/* detect channel status ON */
#define RWF_STM_RD		0x01	/* during transfer stream-data-Read */
#define RWF_STM_WT		0x02	/* standby transfer stream-data-Write */
#define RWF_STM_WTRN		0x04	/* during transfer stream-data-Write */
#define RWF_STM_WTUR		0x08	/* standby transfer(buffer under run) 
					   stream-data-Write */
#define LuLlRuRl		0x00000000	/* Lch upper, Lch lower, 
						   Rch upper, Rch lower */
#define LlLuRlRu		0x00000001	/* Lch lower, Lch upper, 
						   Rch lower, Rch upper */
#define RuRlLuLl		0x00000002	/* Rch upper, Rch lower, 
						   Lch upper, Lch lower */
#define RlRuLlLu		0x00000003	/* Rch lower, Rch upper, 
						   Lch lower, Lch upper */

/* driver status  states */
#define ST_STOP			0x00000000
#define ST_READY		0x00000001
#define ST_OPEN			0x00000004
#define ST_LPF_CALL		0x00000010	/* _lowpassfilter() call */
#define ST_SRC_CALL		0x00000020	/* _sampling_rate() call */
#define ST_SPE_CALL		0x00000040	/* _speed() call */
#define ST_IIR_CALL		0x00000200	/* _graphic_equalizer() call */
#define ST_PATHA_CALL		0x00000400	/* path A setting called */
#define ST_PATHB_CALL		0x00000800	/* path B setting called */
#define ST_SPBACTIV		0x00001000	/* SPB working */

/* parameter macros */
#define PATHAOFF		0x00000000	/* pathA not use */
#define PATHAON			0x00000001	/* pathA use */
#define PATHBOFF		0x00000000	/* pathB not use */
#define PATHBON			0x00000001	/* pathB use */
#define CPUOUTPUT		0x00000000	/* input from CPU output */
#define PORTAINPUT		0x00000001	/* input from portA input */
#define PORTBINPUT		0x00000002	/* input from portB input */
#define MELOUTPUT		0x00000003
#define CPUINPUT		0x00000000	/* input CPU */
#define PORTAOUTPUT		0x00000001	/* output portA */
#define PORTBOUTPUT		0x00000002	/* output portB */
#define MELINPUT		0x00000003
#define FIRTHR			0x00000000	/* through */
#define FIRLPF			0x00000001	/* LPF */
#define FIRSRC			0x00000002	/* SRC */
#define MIXOFF			0x00000000	/* mix OFF */
#define MIXON			0x00000001	/* mix ON */
#define IIROFF			0x00000000	/* GEQ OFF */
#define IIRON			0x00000001	/* GEQ ON */
#define SCROFF			0x00000000	/* scramble OFF */
#define SCRON			0x00000001	/* scramble ON */


/* SIU memory address definition */
#define siu_p_ram		0xa4540000	/* PRAM */
#define siu_x_ram		0xa4544000	/* XRAM */
#define siu_y_ram		0xa4546000	/* YRAM */
#define siu_fifo_ram		0xa4548000	/* FIFO RAM */

#define YRAM0_SIZE		(0x0040/4)	/* 16 */
#define YRAM1_SIZE		(0x0080/4)	/* 32 */
#define YRAM2_SIZE		(0x0040/4)	/* 16 */
#define YRAM3_SIZE		(0x0080/4)	/* 32 */
#define YRAM4_SIZE		(0x0080/4)	/* 32 */
#define YRAM_DEF_SIZE		(YRAM0_SIZE + YRAM1_SIZE + YRAM2_SIZE + \
                                 YRAM3_SIZE + YRAM4_SIZE)
#define YRAM_FIR_SIZE		(0x0400/4)	/* 256 */
#define YRAM_IIR_SIZE		(0x0200/4)	/* 128 */

#define XRAM0_SIZE		(0x0400/4)	/* 256 */
#define XRAM1_SIZE		(0x0200/4)	/* 128 */
#define XRAM2_SIZE		(0x0200/4)	/* 128 */

/* PRAM program array size */
#define PRAM0_SIZE		(0x0100/4)	/* 64 */
#define PRAM1_SIZE		((0x2000 -0x0100) / 4)	/* 1984 */



/* Register access */
#define IN8(addr)		(*((volatile u_int8_t *)(addr)))
#define IN16(addr)		(*((volatile u_int16_t *)(addr)))
#define IN32(addr)		(*((volatile u_int32_t *)(addr)))
#if DEBUG
# define OUT8(addr, val)	\
do { \
	PRINTK("%08x=%08x\n", addr, val); \
	(*((volatile u_int8_t *)(addr)) = (val)); \
}while (0)
# define OUT16(addr, val)	\
do { \
	PRINTK("%08x=%08x\n", addr, val); \
	(*((volatile u_int16_t *)(addr)) = (val)); \
}while (0)
# define OUT32(addr, val)	\
do { \
	PRINTK("%08x=%08x\n", addr, val); \
	(*((volatile u_int32_t *)(addr)) = (val)); \
}while (0)
#else
# define OUT8(addr, val)	(*((volatile u_int8_t *)(addr)) = (val))
# define OUT16(addr, val)	(*((volatile u_int16_t *)(addr)) = (val))
# define OUT32(addr, val)	(*((volatile u_int32_t *)(addr)) = (val))
#endif

/* Clock pulse generator registers */
#define	SCLKACR_MCLKSEL		0x00000020
#define	SCLKACR			0xa4150008
#define	SCLKBCR			0xa415000c
#define SCLKBCR_EXSRC		0x00000082
#if defined(CONFIG_CPU_SUBTYPE_SH7722)
# define SCLKACR_EXSRC		0x00000082
#elif defined(CONFIG_CPU_SUBTYPE_SH7354)
# define SCLKACR_EXSRC		0x00000080
#elif defined(CONFIG_CPU_SUBTYPE_SH7343)
# define SCLKACR_EXSRC		0x00000080
#elif defined(CONFIG_CPU_SUBTYPE_SH7723)
# define SCLKACR_EXSRC		0x00000082
#else
# error Unsupported processor type. Must set the clock division ratio
#endif

/* Pin function controller (PFC) registers */
#define PECR			0xa4050108
#define PFCR			0xa405010a

#define PKCR			0xa4050112
#define PKCR_PK0MD0		0x0001
#define PKCR_PK0MD1		0x0002
#define PKCR_PK1MD0		0x0004
#define PKCR_PK1MD1		0x0008
#define PKCR_PK2MD0		0x0010
#define PKCR_PK2MD1		0x0020
#define PKCR_PK3MD0		0x0040
#define PKCR_PK3MD1		0x0080
#define PKCR_PK4MD0		0x0100
#define PKCR_PK4MD1		0x0200
#define PKCR_PK5MD0		0x0400
#define PKCR_PK5MD1		0x0800
#define PKCR_PK6MD0		0x1000
#define PKCR_PK6MD1		0x2000

#define PSELA_PSA2		0x0004

#define PSELB_PSB0		0x0001
#define PSELB_PSB1		0x0002
#define PSELB_PSB9		0x0200
#define PSELB_PSB10		0x0400
#define PSELB_PSB11		0x0800
#define PSELB_PSB12		0x1000
#define PSELB_PSB13		0x2000
#define PSELB_PSB14		0x4000
#define PSELB_PSB15		0x8000

#define PSELC_PSC15		0x8000
#define PSELC_PSC14		0x4000
#define PSELC_PSC13		0x2000

#define PSELE_PSE11		0x0800

#define HIZCRB_HIZB4		0x0010
#define HIZCRB_HIZB15		0x8000

#define MSELCR_MSEL0		0x0001
#define MSELCR_MSEL1		0x0002
#define MSELCR_MSEL2		0x0004

/* Some Pin function contoller (PFC) registers' offsets/values are different 
   for different processors */
#if defined(CONFIG_CPU_SUBTYPE_SH7722) || defined(CONFIG_CPU_SUBTYPE_SH7723)




/* !TODO adjust me */









# define PKCR_CLR_BITS		(PKCR_PK0MD0 | PKCR_PK0MD1 | PKCR_PK1MD0 | \
				 PKCR_PK1MD1 | PKCR_PK2MD0 | PKCR_PK2MD1 | \
				 PKCR_PK3MD0 | PKCR_PK3MD1 | PKCR_PK4MD0 | \
				 PKCR_PK4MD1 | PKCR_PK5MD0 | PKCR_PK5MD1 | \
				 PKCR_PK6MD0 | PKCR_PK6MD1)
# define PKCR_SET_BITS		(PKCR_PK6MD1 | PKCR_PK5MD1)

# define PFCR_CLR_BITS		0xffff

# define PSELA			0xa405014e
# define PSELA_CLR_BITS		0

# define PSELB			0xa4050150
# define PSELB_CLR_BITS		(PSELB_PSB0 | PSELB_PSB1)
# define PSELB_SET_BITS		(PSELB_PSB9 | PSELB_PSB10 | PSELB_PSB11 | \
				 PSELB_PSB12 | PSELB_PSB13 | PSELB_PSB14 | \
				 PSELB_PSB15)

# define PSELC			0xa4050152
# define PSELC_CLR_BITS		(PSELC_PSC15 | PSELC_PSC14 | PSELC_PSC13)

# define PSELE			0xa4050156
# define PSELE_CLR_BITS		PSELE_PSE11

# define HIZCRB			0xa405015a
# define HIZCRB_CLR_BITS	(HIZCRB_HIZB4 | HIZCRB_HIZB15)

# define MSELCR			0xa4050180
# define MSELCR_SET_BITS	(MSELCR_MSEL0 | MSELCR_MSEL1 | MSELCR_MSEL2)

#elif defined(CONFIG_CPU_SUBTYPE_SH7354)
# define PKCR_CLR_BITS		(PKCR_PK0MD0 | PKCR_PK0MD1 | PKCR_PK1MD0 | \
				 PKCR_PK1MD1 | PKCR_PK2MD0 | PKCR_PK2MD1 | \
				 PKCR_PK3MD0 | PKCR_PK3MD1 | PKCR_PK4MD0 | \
				 PKCR_PK4MD1)
# define PKCR_SET_BITS		0

# define PFCR_CLR_BITS		0

# define PSELA			0xa405014e
# define PSELA_CLR_BITS		PSELA_PSA2

# define PSELB			0xa4050150
# define PSELB_CLR_BITS		0
# define PSELB_SET_BITS		0

# define PSELC			0xa4050152
# define PSELC_CLR_BITS		0

# define PSELE			0xa4050156
# define PSELE_CLR_BITS		0

# define HIZCRB			0xa405015a
# define HIZCRB_CLR_BITS	HIZCRB_HIZB4

# define MSELCR			0xa4050180
# define MSELCR_SET_BITS	MSELCR_MSEL2

#elif defined(CONFIG_CPU_SUBTYPE_SH7343)
# define PKCR_CLR_BITS		(PKCR_PK0MD0 | PKCR_PK0MD1 | PKCR_PK1MD0 | \
				 PKCR_PK1MD1 | PKCR_PK2MD0 | PKCR_PK2MD1 | \
				 PKCR_PK3MD0 | PKCR_PK3MD1 | PKCR_PK4MD0 | \
				 PKCR_PK4MD1 | PKCR_PK5MD0 | PKCR_PK5MD1 | \
				 PKCR_PK6MD0 | PKCR_PK6MD1)
# define PKCR_SET_BITS		(PKCR_PK6MD1 | PKCR_PK5MD1)

# define PFCR_CLR_BITS		0

# define PSELA			0xa405014c
# define PSELA_CLR_BITS		0

# define PSELB			0xa405014e
# define PSELB_CLR_BITS		(PSELB_PSB0 | PSELB_PSB1)
# define PSELB_SET_BITS		0

# define PSELC			0xa4050150
# define PSELC_CLR_BITS		(PSELC_PSC15 | PSELC_PSC14 | PSELC_PSC13)

# define PSELE			0xa4050154
# define PSELE_CLR_BITS		0

# define HIZCRB			0xa4050158
# define HIZCRB_CLR_BITS	HIZCRB_HIZB4

# define MSELCR			0xa405015c
# define MSELCR_SET_BITS	MSELCR_MSEL2

#else
# error Unsupported processor type. Registers offsets/values must be verified.
#endif

/* Power down registers */
#define	MSTPCR0			0xa4150030
#define	MSTPCR0_MSTP022		0x00400000
#define	MSTPCR0_MSTP021		0x00200000

#define	MSTPCR2			0xa4150038
#define	MSTPCR2_MSTP208		0x00000100

/* SIU registers */
#define IFCTL			0xa454c000
#define SRCTL			0xa454c004
#define SFORM			0xa454c008
#define CKCTL			0xa454c00c
#define TRDAT			0xa454c010
#define STFIFO			0xa454c014
#define DPAK			0xa454c01c
#define CKREV			0xa454c020
#define EVNTC			0xa454c028
#define SBCTL			0xa454c040
#define SBPSET			0xa454c044
#define SBDVCA			0xa454c06c
#define SBDVCB			0xa454c070
#define SBACTIV			0xa454c074
#define DMAIA			0xa454c090
#define DMAIB			0xa454c094
#define DMAOA			0xa454c098
#define DMAOB			0xa454c09c
#define DMAML			0xa454c0a0
#define SPSTS			0xa454c0cc
#define SPCTL			0xa454c0d0
#define BRGASEL			0xa454c100
#define BRRA			0xa454c104
#define BRGBSEL			0xa454c108
#define BRRB			0xa454c10c

/* DMA registers */
#define	SAR0			0xfe008020
#define	SAR1			0xfe008030
#define	SAR2			0xfe008040
#define	SAR3			0xfe008050
#define	SAR4			0xfe008070
#define	SAR5			0xfe008080
#define	DMA0CH_BASE		SAR0
#define	DMA1CH_BASE		SAR1
#define	DMA2CH_BASE		SAR2
#define	DMA3CH_BASE		SAR3
#define	DMA4CH_BASE		SAR4
#define	DMA5CH_BASE		SAR5
#define	DMAC0			(struct st_dmac *)DMA0CH_BASE
#define	DMAC1			(struct st_dmac *)DMA1CH_BASE
#define	DMAC2			(struct st_dmac *)DMA2CH_BASE
#define	DMAC3			(struct st_dmac *)DMA3CH_BASE
#define	DMAC4			(struct st_dmac *)DMA4CH_BASE
#define	DMAC5			(struct st_dmac *)DMA5CH_BASE

#define CHCR_DE_BIT		0
#define CHCR_TE_BIT		1
#define CHCR_IE_BIT		2
#define CHCR_TS10_BIT		3
#define CHCR_TB_BIT		5
#define CHCR_DLDS_BIT		6
#define CHCR_RS_BIT		8
#define CHCR_SM_BIT		12
#define CHCR_DM_BIT		14
#define CHCR_AL_BIT		16
#define CHCR_AM_BIT		17
#define CHCR_DO_BIT		23

#define CHCR_DE_WIDTH		1
#define CHCR_TE_WIDTH		1
#define CHCR_IE_WIDTH		1
#define CHCR_TS10_WIDTH		2
#define CHCR_TB_WIDTH		1
#define CHCR_DLDS_WIDTH		2
#define CHCR_RS_WIDTH		4
#define CHCR_SM_WIDTH		2
#define CHCR_DM_WIDTH		2
#define CHCR_AL_WIDTH		1
#define CHCR_AM_WIDTH		1
#define CHCR_DO_WIDTH		1

#define	DMAOR			0xfe008060
#define	DMAOR_DME_BIT		0
#define	DMAOR_NMIF_BIT		1
#define	DMAOR_AE_BIT		2
#define	DMAOR_PR_BIT		8
#define	DMAOR_CMS_BIT		12

#define	DMAOR_DME_WIDTH		1
#define	DMAOR_NMIF_WIDTH	1
#define	DMAOR_AE_WIDTH		1
#define	DMAOR_PR_WIDTH		2
#define	DMAOR_CMS_WIDTH		4

#define DMARS_0			0xfe009000
#define DMARS_1			0xfe009004
#define DMARS_2			0xfe009008
#define DMARS_ADDR(ch)		(DMARS_0 + 4 * ((ch) / 2))
#define DMARS_VAL(ch, val)	((IN16(DMARS_ADDR(ch)) & 	\
				 0xff00 >> ((ch) % 2) * 8) |	\
				 (val) << ((ch) % 2) * 8)

#define SET_BIT16(addr, bit, width, val)			\
    ((IN16(addr) & ~(((1 << (width)) - 1) << (bit))) |		\
     (val) << (bit))
#define SET_BIT32(addr, bit, width, val)			\
    ((IN32(addr) & ~(((1 << (width)) - 1) << (bit))) |		\
     (val) << (bit))

#endif /* SIU_SH7343_H */
