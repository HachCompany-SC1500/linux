/*
 * arch/sh/boards/emtrion/hico7723/sh7723.h
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
 * References:
 *         [1] file://W:/Datenblaetter/Renesas/SH-Mobile/SH7723/SH7723_Rev1.00_Eng_edi1958.pdf
 *
 **/

#ifndef _ARCH_SH_SH7723_H
#define _ARCH_SH_SH7723_H

#define PSELA		0xA405014E
#define MSELCRA		0xA4050180
#define MSELCRB		0xA4050182
#define PORT_PCCR	0xA4050104
#define PORT_PDCR	0xA4050106
#define PORT_PGCR       0xA405010C
#define PORT_PHCR	0xA405010E
#define PORT_PKCR	0xA4050112
#define PORT_PLCR	0xA4050114
#define PORT_PMCR	0xA4050116
#define PORT_PNCR	0xA4050118
#define PORT_PRCR	0xA405011C
#define PORT_PSCR	0xA405011E
#define PORT_PUCR	0xA4050142
#define PORT_PVCR	0xA4050144
#define PORT_PXCR	0xA4050148
#define PORT_PZCR	0xA405014C
#define PORT_HIZCRA	0xA4050158
#define PORT_HIZCRC	0xA405015C
#define PORT_DRVCRA	0xA405018A
#define PORT_DRVCRB	0xA405018C

#define PORT_PCDR   0xA4050124  /* added by mgrobe */
#define PORT_PDDR   0xA4050126

#define PORT_PFDR	0xA405012A
#define PORT_PGDR       0xA405012C
#define PORT_PHDR	0xA405012E
#define PORT_PJDR	0xA4050130
#define PORT_PLDR	0xA4050134
#define PORT_PMDR	0xA4050136
#define PORT_PNDR	0xA4050138
#define PORT_PSDR	0xA405013E
#define PORT_PXDR	0xA4050168
#define PORT_PZDR	0xA405016C
#define PORT_PSELC	0xA4050152
#define PORT_PSELD	0xA4050154

#define CS4BCR		0xFEC10010
#define CS4WCR		0xFEC10030

/* memory resources */
#define FLCTL_BASE	0xa4530000
#define FLCTL_END	0xa45300ff

#define ADC_BASE	0xa4610000
#define ADC_END		0xa4610009

#define MSIOF0_BASE	0xa4c40000
#define MSIOF0_END	0xa4c40063
#define MSIOF1_BASE	0xa4c50000
#define MSIOF1_END	0xa4c50063

#define TPU_BASE	0xA4C90000
#define TPU_END		0xA4C900F6

#define SDHI0_BASE	0x04ce0000
#define SDHI0_END	0x04ce01ff
#define SDHI1_BASE	0x04cf0000
#define SDHI1_END	0x04cf01ff

#define USB0_BASE	0xa4d80000
#define USB0_END	0xa4d80123

#define CEU0_BASE	0xfe910000
#define CEU0_END	0xfe91009f

#define LCDC_BASE       0xfe940000
#define LCDC_END        0xfe942fff

/* see arch/sh/kernel/cpu/sh4a/setup-sh7723.c "static struct intc_vect vectors" */
#define IRQEVT_SDHI10	0x4E0
#define IRQEVT_LCDC	0x580
#define IRQEVT_IRQ0	0x600
#define IRQEVT_IRQ1	0x620
#define IRQEVT_IRQ2	0x640
#define IRQEVT_IRQ3	0x660
#define IRQEVT_IRQ4	0x680
#define IRQEVT_IRQ5	0x6A0
#define IRQEVT_IRQ6	0x6C0
#define IRQEVT_IRQ7	0x6E0
#define IRQEVT_CEU0I	0x880
#define IRQEVT_BEU0I	0x8A0
#define IRQEVT_VEUI	0x8C0
#define IRQEVT_VOUI	0x8E0
#define IRQEVT_ADC_ADI	0x9E0
#define IRQEVT_USI0	0xA20
#define IRQEVT_MSIOF0	0xC80
#define IRQEVT_MSIOF1	0xCA0
#define IRQEVT_SDHI00	0xE80

#define INTREQ00 	0xa4140024

#define MSTPCR2         0xa4150038
#define MSTPCR2_LCDC	(1<<0)

#define PSELD_PTW2_MASK 0x0C00
#define PSELD_PTW2_VIO  0x0800
#define PSELD_PTW2_BS	0x0400
#define PSELD_PTW2_IRQ	0x0000
#define PSELD_PTW1_SIU	0x0100
#define PSELD_PTW0_SIU	0x0040
#define PSELD_PTX_IRDA  0x0010
#define PSELD_PTZ_SIU   0x0001

#define CSxBCR_IWW_12   0x70000000
#define CSxBCR_IWW_4    0x40000000
#define CSxBCR_IWW_2    0x20000000
#define CSxBCR_IWW_1    0x10000000
#define CSxBCR_IWRWD_8  0x0A000000
#define CSxBCR_IWRWD_4  0x06000000
#define CSxBCR_IWRWD_2  0x04000000
#define CSxBCR_IWRWD_1  0x02000000
#define CSxBCR_IWRWS_10 0x01800000
#define CSxBCR_IWRWS_4  0x00C00000
#define CSxBCR_IWRWS_2  0x00800000
#define CSxBCR_IWRWS_1  0x00400000
#define CSxBCR_IWRWS_0  0x00000000
#define CSxBCR_IWRRD_8  0x00280000
#define CSxBCR_IWRRD_4  0x00180000
#define CSxBCR_IWRRD_2  0x00100000
#define CSxBCR_IWRRD_1  0x00080000
#define CSxBCR_IWRRS_10 0x00060000
#define CSxBCR_IWRRS_4  0x00030000
#define CSxBCR_IWRRS_2  0x00020000
#define CSxBCR_IWRRS_1  0x00010000
#define CSxBCR_IWRRS_0  0x00000000
#define CSxBCR_TYPE_NORMAL 0x00000000
#define CSxBCR_BSZ_32   0x00000600
#define CSxBCR_BSZ_16   0x00000400
#define CSxBCR_BSZ_8    0x00000200
#define CSxWCR_WW_4  	0x00050000
#define CSxWCR_WW_3  	0x00040000
#define CSxWCR_WW_2  	0x00030000
#define CSxWCR_SW_05  	0x00000000
#define CSxWCR_SW_15 	0x00000800
#define CSxWCR_WR_14  	0x00000500
#define CSxWCR_WR_8  	0x00000380
#define CSxWCR_WR_3  	0x00000180
#define CSxWCR_WM_IG 	0x00000040
#define CSxWCR_HW_15 	0x00000001

/* misc configuratiopn */
#define I2C_DEVICE_CAMERA	0

#endif /* _ARCH_SH_SH7723_H */
