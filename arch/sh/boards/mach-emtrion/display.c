/*
 * linux/arch/sh/boards/emtrion/display.c
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
 * Description: Provides display=<TX14|NL6448BC20> kernel command line option.
 *              The selected display index in sh_mobile_lcdc_info[] for that
 *              name can then be retreived with display_selected()
 *
 **/

#include <linux/init.h>
#include <linux/string.h>
#include <video/sh_mobile_lcdc.h>

#include "display.h"
#include "bootlogo.h"
#include "include/sh7723.h"

const char* display_name;
int display_phys_width;
int display_phys_height;

static int __init display_setup(char *options)
{
        display_name = options;

        return 0;
}
early_param("display", display_setup);

static int __init display_phys_width_setup(char *options)
{
        if (options != NULL)
                display_phys_width = simple_strtoul(options, NULL, 0);

        return 0;
}
early_param("display_phys_width", display_phys_width_setup);

static int __init display_phys_height_setup(char *options)
{
        if (options != NULL)
                display_phys_height = simple_strtoul(options, NULL, 0);

        return 0;
}
early_param("display_phys_height", display_phys_height_setup);

int __init display_selected(const struct sh_mobile_lcdc_info *displays, int entries)
{
        int i = 0;

        if (display_name == NULL || !strlen(display_name) )
                return -2;

        while (i < entries) {
                const char *name = displays[i].ch[0].lcd_cfg.name;
                if ((name != NULL) && (displays[i].ch[0].lcd_cfg.name != NULL) &&
		    !strcmp(name, display_name))
                        return i;
                i++;
        }

        return -1;
}

#define LDDCKPAT1R      0x0400
#define LDDCKPAT2R      0x0404
#define LDDCKR          0x0410
#define LDMT1R          0x0418
#define LDMT2R          0x041C
#define LDMT3R          0x0420
#define	LDDFR		0x0424
#define	LDSM1R		0x0428
#define	LDSM2R		0x042C
#define	LDDFR		0x0424
#define LDSA1R          0x0430
#define LDMLSR          0x0438
#define LDHCNR          0x0448
#define LDHSYNR         0x044C
#define LDVLNR          0x0450
#define LDVSYNR         0x0454
#define LDSR            0x046C
#define LDCNT2R         0x0474
#define LDRCNTR		0x0478
#define LDDDSR		0x047C

#define LDDCKR_MOSEL    0x00000040
#define LDDCKR_MDCDR    0x0000003F
#define LDCNT2R_ME      0x00000002

#define LDRCNTR_MRS_B	0x00000002
#define LDSR_MRS_B      0x00000100

static int __init lcdc_read_chan0(u32 offs)
{
        return __raw_readl(LCDC_BASE+offs);
}

static void __init lcdc_write_chan0(u32 data, u32 offs)
{
        __raw_writel(data, LCDC_BASE+offs);
}

/**
 * display_read_parameters - Reads the display parameters from hardware which have been setup by the bootloader
 */
void __init display_read_parameters(struct sh_mobile_lcdc_info *info)
{
        struct sh_mobile_lcdc_chan_cfg *ch = &info->ch[0];
        struct sh_mobile_lcdc_sys_bus_cfg *sys = &ch->sys_bus_cfg;
        struct fb_videomode *cfg = &ch->lcd_cfg;

        u32 lddckr;
        u32 ldhcnr;
        u32 ldhsynr;
        u32 ldvlnr;
        u32 ldvsynr;

	if (__raw_readl(MSTPCR2) & MSTPCR2_LCDC)
		/* LCDC clock is not enabled and therefore not running */
		return;

        if (!(lcdc_read_chan0(LDCNT2R) & LDCNT2R_ME))
                /* LCDC not yet configured */
                return;

        lddckr  = lcdc_read_chan0(LDDCKR);
        ldhcnr  = lcdc_read_chan0(LDHCNR);
        ldhsynr = lcdc_read_chan0(LDHSYNR);
        ldvlnr  = lcdc_read_chan0(LDVLNR);
        ldvsynr = lcdc_read_chan0(LDVSYNR);

        /* determine current LCD controller settings, so sh_mobile_lcdcfb.c can
           write them in it's initialization after reset of LCDC */
        cfg->name = display_name;

        cfg->xres = ((ldhcnr >> 16)&0xFFFF)*8;
        cfg->yres = ((ldvlnr) >> 16)&0xFFFF;
        cfg->hsync_len    = (ldhsynr >> 16)*8;
        cfg->right_margin = (ldhsynr & 0xFFFF)*8 - cfg->xres;
        cfg->left_margin  = (ldhcnr & 0xFFFF)*8 - cfg->xres - cfg->right_margin - cfg->hsync_len;
        cfg->vsync_len    = ldvsynr >> 16;
        cfg->lower_margin = (ldvsynr & 0xFFFF) - cfg->yres;
        cfg->upper_margin = (ldvlnr & 0xFFFF) - cfg->yres - cfg->lower_margin - cfg->vsync_len;

        cfg->sync    = 0; // sync is later or'd with ldmt1r which is already configured for LCDC

        if (lddckr & LDDCKR_MOSEL)
                ch->clock_divider = 1;
        else
                ch->clock_divider = lddckr & LDDCKR_MDCDR;

        // TODO: add support for other than RGB18 displays
        sys->ldmt2r  = lcdc_read_chan0(LDMT2R);
        sys->ldmt3r  = lcdc_read_chan0(LDMT3R);
        sys->ckpat1r = lcdc_read_chan0(LDDCKPAT1R);
        sys->ckpat2r = lcdc_read_chan0(LDDCKPAT2R);

        ch->lcd_size_cfg.width  = display_phys_width;
        ch->lcd_size_cfg.height = display_phys_height;

        info->lcdc_is_already_configured_and_running = 1;

#ifdef CONFIG_SH_BOOTLOGO
        /* sh_mobile_lcdcfb.c doubles yres_virtual by 3 to support software
         * panning/triple buffering */
        if (bootlogo_init(ch, lcdc_read_chan0(LDSA1R), lcdc_read_chan0(LDMLSR)*cfg->yres*3) == BL_COPIED) {
                /* flicker-free reconfiguration of the framebuffer pointer by
                   using side b registers */
                sh_mobile_lcdc_early_copy_registers_to_sideb(LCDC_BASE);

                /* switch to side B */

                lcdc_write_chan0(lcdc_read_chan0(LDRCNTR) | LDRCNTR_MRS_B, LDRCNTR);

                /* wait until side B is used by hardware */
                while (1) {
                        if (lcdc_read_chan0(LDSR) & LDSR_MRS_B)
                                break;
                }

                /* safely update side A */
                lcdc_write_chan0(ch->bootlogo.smem_start, LDSA1R);

                /* switch back to side A */
                lcdc_write_chan0(lcdc_read_chan0(LDRCNTR) & ~LDRCNTR_MRS_B, LDRCNTR);
        }
#endif
}

void display_configure_data_pins(void *board_data, int for_lcdc)
{
	u32 cr;

	/* No one else is accessing the LCDC/GPIO registers, so it's safe to modify them unlocked.
	   We don't use GPIOLIB as switching needs to be as fast as possible so it's not visible */

	if (!for_lcdc) {
		/* set GPIOs to low */
		__raw_writeb(0x0, PORT_PLDR);
		__raw_writeb(0x0, PORT_PMDR);
		__raw_writeb(0x0, PORT_PNDR);

		/* configure pins to GPIO */
		cr = 0x5555;
	} else {
		/* configure pins as special function */
		cr = 0x0;
	}

	__raw_writew(cr, PORT_PLCR);
	__raw_writew(cr, PORT_PMCR);
	__raw_writew(cr, PORT_PNCR);
}
