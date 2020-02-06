/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * !!!!!!!!! CHANGE THIS FILE ONLY IN THE U-BOOT TREE !!!!!!!!!!
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * U-Boot/include/asm-sh/hico7723.h
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
 * Description: Provides pin descriptions for hico7723
 **/

#ifndef _ASM_HICO7723_H_
#define _ASM_HICO7723_H_

#define PIN_MODE_N( pin, mode )	( ( PIN_MODE_##mode ) << ( (pin) * 2 ) )

#define PIN_FUNC_N( pin )	PIN_MODE_N( pin, FUNC )
#define PIN_OUT_N( pin )	PIN_MODE_N( pin, OUT  )
#define PIN_IN_N( pin )		PIN_MODE_N( pin, IN   )

#define PIN_FUNC( pin )		PIN_FUNC_N( PIN_NUM_##pin )
#define PIN_OUT( pin )		PIN_OUT_N( PIN_NUM_##pin  )
#define PIN_IN( pin )		PIN_IN_N( PIN_NUM_##pin   )

#define PIN_DATA( pin )         ( 1 << ( PIN_NUM_##pin ) )

/* PJCR */

/* pin usage on HICO7723 */
#define PIN_NUM_LED_D2_GREEN	7
#define PIN_NUM_LED_D2_RED 	5

/* PFCR */
#define PIN_NUM_SOFTRESET	7
#define PIN_NUM_USBH_PEN	5

#define PIN_NUM_NAND_WP	7
#define PIN_NUM_NAND_CLE 	5
#define PIN_NUM_NAND_RE  	4
#define PIN_NUM_NAND_WE  	3
#define PIN_NUM_NAND_ALE 	2
#define PIN_NUM_NAND_RB  	1
#define PIN_NUM_NAND_CE  	0

#define PIN_NUM_LCD_DON		2

/* pin configuration */
/* PACR */

#define PIN_D23	PIN_FUNC_N( 7 )
#define PIN_D22	PIN_FUNC_N( 6 )
#define PIN_D21	PIN_FUNC_N( 5 )
#define PIN_D20	PIN_FUNC_N( 4 )
#define PIN_D19	PIN_FUNC_N( 3 )
#define PIN_D18	PIN_FUNC_N( 2 )
#define PIN_D17	PIN_FUNC_N( 1 )
#define PIN_D16	PIN_FUNC_N( 0 )

/* PBCR */

#define PIN_D31	PIN_FUNC_N( 7 )
#define PIN_D30	PIN_FUNC_N( 6 )
#define PIN_D29	PIN_FUNC_N( 5 )
#define PIN_D28	PIN_FUNC_N( 4 )
#define PIN_D27	PIN_FUNC_N( 3 )
#define PIN_D26	PIN_FUNC_N( 2 )
#define PIN_D25	PIN_FUNC_N( 1 )
#define PIN_D24	PIN_FUNC_N( 0 )

/* PCCR */
#define PIN_SDC2_CD	PIN_FUNC_N( 7 )
#define PIN_SDC2_WP	PIN_FUNC_N( 6 )
#define PIN_SDC2_D3	PIN_FUNC_N( 5 )
#define PIN_SDC2_D2	PIN_FUNC_N( 4 )
#define PIN_SDC2_D1	PIN_FUNC_N( 3 )
#define PIN_SDC2_D0	PIN_FUNC_N( 2 )
#define PIN_SDC2_CMD 	PIN_FUNC_N( 1 )
#define PIN_SDC2_CLK 	PIN_FUNC_N( 0 )

/* PDCR */
#define PIN_SDC1_CD	PIN_FUNC_N( 7 )
#define PIN_SDC1_WP	PIN_FUNC_N( 6 )
#define PIN_SDC1_D3	PIN_FUNC_N( 5 )
#define PIN_SDC1_D2	PIN_FUNC_N( 4 )
#define PIN_SDC1_D1	PIN_FUNC_N( 3 )
#define PIN_SDC1_D0	PIN_FUNC_N( 2 )
#define PIN_SDC1_CMD 	PIN_FUNC_N( 1 )
#define PIN_SDC1_CLK 	PIN_FUNC_N( 0 )

/* PECR */
#define PIN_E5		PIN_OUT_N(  5 )
#define PIN_SCIF5_RX	PIN_FUNC_N( 4 )
#define PIN_SCIF5_TX	PIN_FUNC_N( 3 )
#define PIN_VOU_RST	PIN_OUT_N(  2 )
#define PIN_SCIF4_RX    PIN_FUNC_N( 1 )
#define PIN_SCIF4_TX    PIN_FUNC_N( 0 )

/* PFCR */
#define PIN_SOFTRESET	  PIN_OUT(  SOFTRESET )
#define PIN_USBH_OC	  PIN_IN_N(   6 )
#define PIN_USBH_PEN      PIN_OUT_N(  USBH_PEN )
#define PIN_MSIOF0_TSYNC  PIN_FUNC_N( 4 )
#define PIN_MSIOF0_TSCK   PIN_FUNC_N( 3 )
#define PIN_MSIOF0_RXD    PIN_FUNC_N( 2 )
#define PIN_MSIOF0_TXD    PIN_FUNC_N( 1 )
#define PIN_USB_WAKE  	  PIN_IN_N(   0 )

/* PGCR */
#define PIN_AUDCK	  PIN_FUNC_N(  5 )
#define PIN_AUDSYNC	  PIN_FUNC_N(  4 )
#define PIN_AUD3	  PIN_FUNC_N(  3 )
#define PIN_AUD2	  PIN_FUNC_N(  2 )
#define PIN_AUD1	  PIN_FUNC_N(  1 )
#define PIN_AUD0	  PIN_FUNC_N(  0 )

/* PHCR */
#define PIN_LCD_VCPWC	  PIN_FUNC_N(  7 )
#define PIN_LCD_RD	  PIN_FUNC_N(  6 )
#define PIN_LCD_VSYNC	  PIN_FUNC_N(  5 )
#define PIN_LCD_DISP	  PIN_FUNC_N(  4 )
#define PIN_LCD_HSYNC	  PIN_FUNC_N(  3 )
#define PIN_LCD_DON	  PIN_OUT(   LCD_DON )
#define PIN_LCD_DCK	  PIN_FUNC_N(  1 )
#define PIN_LCD_VEPWC	  PIN_FUNC_N(  0 )

/* PJCR */
#define PIN_LED_D2_GREEN PIN_OUT( LED_D2_GREEN )
#define PIN_LED_D2_RED   PIN_OUT( LED_D2_RED   )
#define PIN_A25	  	 PIN_FUNC_N( 3 )
#define PIN_A24	  	 PIN_FUNC_N( 2 )
#define PIN_A23	  	 PIN_FUNC_N( 1 )
#define PIN_A22	  	 PIN_FUNC_N( 0 )

/* PKCR */
#define PIN_NAND_WP 	  PIN_OUT(  NAND_WP )
#define PIN_AUDIO_LRC2	  PIN_FUNC_N( 6 )
#define PIN_H1 	  	  PIN_IN_N(   5 )
#define PIN_AUDIO_DATI	  PIN_FUNC_N( 4 )
#define PIN_AUDIO_LRC 	  PIN_FUNC_N( 3 )
#define PIN_AUDIO_BCK 	  PIN_FUNC_N( 2 )
#define PIN_AUDIO_DATO	  PIN_FUNC_N( 1 )
#define PIN_AUDIO_MCLK	  PIN_FUNC_N( 0 )

/* PLCR */
#define PIN_LCD_D15 	  PIN_FUNC_N( 7 )
#define PIN_LCD_D14 	  PIN_FUNC_N( 6 )
#define PIN_LCD_D13 	  PIN_FUNC_N( 5 )
#define PIN_LCD_D12 	  PIN_FUNC_N( 4 )
#define PIN_LCD_D11 	  PIN_FUNC_N( 3 )
#define PIN_LCD_D10 	  PIN_FUNC_N( 2 )
#define PIN_LCD_D9  	  PIN_FUNC_N( 1 )
#define PIN_LCD_D8  	  PIN_FUNC_N( 0 )

/* PMCR */
#define PIN_LCD_D7 	  PIN_FUNC_N( 7 )
#define PIN_LCD_D6  	  PIN_FUNC_N( 6 )
#define PIN_LCD_D5  	  PIN_FUNC_N( 5 )
#define PIN_LCD_D4  	  PIN_FUNC_N( 4 )
#define PIN_LCD_D3  	  PIN_FUNC_N( 3 )
#define PIN_LCD_D2  	  PIN_FUNC_N( 2 )
#define PIN_LCD_D1  	  PIN_FUNC_N( 1 )
#define PIN_LCD_D0  	  PIN_FUNC_N( 0 )

/* PNCR */
#define PIN_LCD_D23	  PIN_FUNC_N( 7 )
#define PIN_LCD_D22 	  PIN_FUNC_N( 6 )
#define PIN_LCD_D21 	  PIN_FUNC_N( 5 )
#define PIN_LCD_D20 	  PIN_FUNC_N( 4 )
#define PIN_LCD_D19 	  PIN_FUNC_N( 3 )
#define PIN_LCD_D18 	  PIN_FUNC_N( 2 )
#define PIN_LCD_D17 	  PIN_FUNC_N( 1 )
#define PIN_LCD_D16 	  PIN_FUNC_N( 0 )

/* PQCR */
#define PIN_ANA4 	  PIN_FUNC_N( 3 )
#define PIN_ANA3 	  PIN_FUNC_N( 2 )
#define PIN_ANA2 	  PIN_FUNC_N( 1 )
#define PIN_ANA1 	  PIN_FUNC_N( 0 )

/* PRCR */
#define PIN_CE1B	  PIN_FUNC_N( 7 )
#define PIN_CE2B 	  PIN_FUNC_N( 6 )
#define PIN_CE5B 	  PIN_FUNC_N( 5 )
#define PIN_CE5A 	  PIN_FUNC_N( 4 )
#define PIN_R3_FUNC 	  PIN_FUNC_N( 3 )
#define PIN_WAIT 	  PIN_FUNC_N( 2 )
#define PIN_ICIOWR	  PIN_FUNC_N( 1 )
#define PIN_ICIORD 	  PIN_FUNC_N( 0 )

/* PSCR */
#define PIN_SCIF1_RXD	  PIN_FUNC_N( 6 )
#define PIN_SCIF1_TXD	  PIN_FUNC_N( 5 )
#define PIN_SCIF3_CTS	  PIN_FUNC_N( 4 )
#define PIN_SCIF3_RTS	  PIN_FUNC_N( 3 )
#define PIN_SCIF3_RXD	  PIN_FUNC_N( 1 )
#define PIN_SCIF3_TXD	  PIN_FUNC_N( 0 )

/* PTCR */
#define PIN_VIO_RST	  PIN_OUT_N(  5 )
#define PIN_VIO_SRC       PIN_OUT_N(  4 )
#define PIN_VOU_DEST      PIN_OUT_N(  3 )
#define PIN_SCIF2_SCK     PIN_FUNC_N( 2 )
#define PIN_SCIF2_RXD     PIN_FUNC_N( 1 )
#define PIN_SCIF2_TXD     PIN_FUNC_N( 0 )

/* PUCR */
#define PIN_NAND_CD       PIN_FUNC_N( 5 )
#define PIN_NAND_SC	  PIN_FUNC_N( 4 )
#define PIN_NAND_WE	  PIN_FUNC_N( 3 )
#define PIN_NAND_OE	  PIN_FUNC_N( 2 )
#define PIN_NAND_RB	  PIN_FUNC_N( 1 )
#define PIN_NAND_CE	  PIN_FUNC_N( 0 )

/* PVCR */
#define PIN_NAND_D7       PIN_FUNC_N( 7 )
#define PIN_NAND_D6  	  PIN_FUNC_N( 6 )
#define PIN_NAND_D5  	  PIN_FUNC_N( 5 )
#define PIN_NAND_D4  	  PIN_FUNC_N( 4 )
#define PIN_NAND_D3  	  PIN_FUNC_N( 3 )
#define PIN_NAND_D2  	  PIN_FUNC_N( 2 )
#define PIN_NAND_D1  	  PIN_FUNC_N( 1 )
#define PIN_NAND_D0  	  PIN_FUNC_N( 0 )

/* PWCR */
#define PIN_IRQ7          PIN_FUNC_N( 7 )
#define PIN_IRQ6     	  PIN_FUNC_N( 6 )
#define PIN_IRQ5     	  PIN_FUNC_N( 5 )
#define PIN_IRQ4     	  PIN_FUNC_N( 4 )
#define PIN_IRQ3     	  PIN_FUNC_N( 3 )
#define PIN_IRQ2     	  PIN_FUNC_N( 2 )
#define PIN_SPDIF_IN 	  PIN_FUNC_N( 1 )
#define PIN_SPDIF_OUT	  PIN_FUNC_N( 0 )

/* PXCR */
#define PIN_DREQ1         PIN_FUNC_N( 7 )
#define PIN_DACK1    	  PIN_FUNC_N( 6 )
#define PIN_IRDA_OUT 	  PIN_FUNC_N( 5 )
#define PIN_IRDA_IN  	  PIN_FUNC_N( 4 )
#define PIN_SW1_4    	  PIN_IN_N(   3 )
#define PIN_SW1_3    	  PIN_IN_N(   2 )
#define PIN_SW1_2    	  PIN_IN_N(   1 )
#define PIN_SW1_1    	  PIN_IN_N(   0 )

/* PYCR */
#define PIN_VIO_D7        PIN_FUNC_N( 7 )
#define PIN_VIO_D6   	  PIN_FUNC_N( 6 )
#define PIN_VIO_D5   	  PIN_FUNC_N( 5 )
#define PIN_VIO_D4   	  PIN_FUNC_N( 4 )
#define PIN_VIO_D3   	  PIN_FUNC_N( 3 )
#define PIN_VIO_D2   	  PIN_FUNC_N( 2 )
#define PIN_VIO_D1   	  PIN_FUNC_N( 1 )
#define PIN_VIO_D0   	  PIN_FUNC_N( 0 )

/* PZCR */
#define PIN_GPIO3_O       PIN_OUT_N(  7 )
#define PIN_GPIO2_O   	  PIN_OUT_N(  6 )
#define PIN_GPIO1_O   	  PIN_OUT_N(  5 )
#define PIN_GPIO0_O   	  PIN_OUT_N(  4 )

#define PIN_GPIO3_I       PIN_IN_N(   7 )
#define PIN_GPIO2_I   	  PIN_IN_N(   6 )
#define PIN_GPIO1_I   	  PIN_IN_N(   5 )
#define PIN_GPIO0_I   	  PIN_IN_N(   4 )
#define PIN_VIO_FLD  	  PIN_FUNC_N( 3 )
#define PIN_VIO_HD1  	  PIN_FUNC_N( 2 )
#define PIN_VIO_VD1  	  PIN_FUNC_N( 1 )
#define PIN_VIO_CLK1 	  PIN_FUNC_N( 0 )

#endif /* _ASM_HICO7723_H_ */
