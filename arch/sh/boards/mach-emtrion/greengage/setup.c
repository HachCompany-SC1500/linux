/*
 * setup.c   Board Initialization and Driver Configuration
 *
 * Copyright (C) 2010 Secure Electrans Limited.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS for A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 **/

#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mtd/physmap.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mtd/sh_flctl.h>
#include <linux/gpio.h>
#include <linux/input/apr_e102375_sh.h>
#include <linux/usb/r8a66597.h>
#include <linux/i2c-gpio.h>
#include <linux/pwm_backlight.h>
#include <video/sh_mobile_lcdc.h>
#include <cpu/sh7723.h>
#include <asm/clock.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/machvec.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/suspend.h>

#include "../arch/sh/drivers/gpr_priority.h"
#include "../sound/sh/siu_sh7343_pdata.h"

#include "../hico7723/sh7723.h"
#include "../display.h"
#include "../bootlogo.h"
#include "../sd_firmware.h"

#define USBH_PEN	GPIO_PTF1

#define I2C_BUS_SH	0
#define I2C_BUS_GPIO	1

static int gpio_request_array(const int gpios[], int size, const char *label)
{
        int i;

        for (i=0; i < size; i++) {
                if (gpio_request(gpios[i], label)) {
                        /* print error but continue booting */
                        pr_err("GPIO %u (%s) already reserved.\n", gpios[i], label );
                        return -EBUSY;
                }
        }

        return 0;
}


static int gg_usb_init(struct platform_device *dev)
{
        gpio_set_value(USBH_PEN, 0);

        return 0;
}

static void gg_usb_deinit(struct platform_device *dev)
{
        gpio_set_value(USBH_PEN, 1);
}

enum {
        BOOTLOADER_CONFIGURED,
};

static struct sh_mobile_lcdc_info lcdc_info[] = {
        [BOOTLOADER_CONFIGURED] = {
                .clock_source = LCDC_CLK_BUS,
                .ch[0] = {
                        .chan = LCDC_CHAN_MAINLCD,
                        .bpp  = 16,
                        .interface_type = RGB18,
                },
        },
};

static struct resource lcdc_resources[] = {
	[0] = {
		.name	= "LCDC",
		.start	= LCDC_BASE, /* P4-only space */
		.end	= LCDC_END,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 28,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device lcdc_device = {
	.name		= "sh_mobile_lcdc_fb",
	.num_resources	= ARRAY_SIZE(lcdc_resources),
	.resource	= lcdc_resources,
};

static struct resource dm9000_resources[] = {
	[0] = {
		.start	= 0x10000000,
		.end	= 0x10000001,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0x10000002,
		.end	= 0x10000003,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= evt2irq( IRQEVT_IRQ5 ),
		.end	= evt2irq( IRQEVT_IRQ5 ),
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
};

static struct i2c_board_info __initdata gg_i2c_devices[] = {
        { I2C_BOARD_INFO("wm8750",  0x1A),   },
        { I2C_BOARD_INFO("24c01ro", 0x50),   },
        { I2C_BOARD_INFO("tmp75",   0x48),   },
};

static struct i2c_board_info __initdata gg_i2c_gpio_devices[] = {
        { I2C_BOARD_INFO("m41st87w", 0x68), },
};

static struct i2c_gpio_platform_data gg_i2c_gpio_data = {
        .sda_pin = GPIO_PTX5,
        .scl_pin = GPIO_PTX4,
        .udelay  = 5,           /* 1/(2*udelay)=100 kHz */
        .timeout = HZ / 10,
        .sda_is_open_drain  = 0,
        .scl_is_open_drain  = 0,
        .scl_is_output_only = 1,
};

static struct platform_device gg_i2c_gpio_device = {
	.name		= "i2c-gpio",
        .id             = I2C_BUS_GPIO,
	.dev		= {
		.platform_data = &gg_i2c_gpio_data,
	},
};

static struct platform_device dm9000_device = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(dm9000_resources),
	.resource	= dm9000_resources,
};

/*********** NOR ***********/
static struct physmap_flash_data gg_nor_flash_data = {
	.width		= 2,
};

static struct resource gg_nor_flash_resources[] = {
	[0] = {
		.name		= "NOR Flash",
		.start		= 0x00000000,
		.end		= 0x02000000,
		.flags		= IORESOURCE_MEM,
	}
};

static struct platform_device gg_nor_flash_device = {
	.name		= "physmap-flash",
	.resource	= gg_nor_flash_resources,
	.num_resources	= ARRAY_SIZE(gg_nor_flash_resources),
	.dev		= {
		.platform_data = &gg_nor_flash_data,
	},
};

/*********** NAND ***********/
static struct resource nand_flash_resources[] = {
	[0] = {
		.start	= 0xA4530000,
		.end	= 0xA45300FF,
		.flags	= IORESOURCE_MEM,
	}
};

static struct sh_flctl_platform_data nand_flash_data = {
	.has_hwecc	= 0, /* disable HWECC as it is 4-Byte ECC and different
                              * to U-Boot/Linux SWECC */
};

static struct platform_device nand_flash_device = {
	.name		= "sh_flctl",
	.resource	= nand_flash_resources,
	.num_resources	= ARRAY_SIZE(nand_flash_resources),
	.dev		= {
		.platform_data = &nand_flash_data,
	},
};

/*********** Touch/SPI ***********/

static struct apr_e102375_sh_platform_data gg_apr_e102375_sh_platform_data = {
        .clock   = "msiof0",
        .gpio_power_down = GPIO_PTF5,
};

static struct resource gg_apr_e102375_sh_resources[] = {
	[0] = {
		.start	= 0xA4C40000,
		.end	= 0xA4C40063,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= evt2irq( IRQEVT_MSIOF0 ),
		.end	= evt2irq( IRQEVT_MSIOF0 ),
		.flags	= IORESOURCE_IRQ,
	},

};
static struct platform_device gg_apr_e102375_sh_device = {
	.name		= "apr_e102375_sh",
	.resource	= gg_apr_e102375_sh_resources,
	.num_resources	= ARRAY_SIZE(gg_apr_e102375_sh_resources),
	.dev		= {
		.platform_data = &gg_apr_e102375_sh_platform_data,
	},
};

/*********** Sound ***********/

static siu_sh7343_pdata_t sound_data = {
        .port_in_use     = SIU_PORTB,
        .data_format     = I2S,
        .stereo_channels = 2,
        .max_volume      = 0x1000, /* anonymously defined by siu_sh7343 (SPB)*/
        .dma_out_ch      = 4, /* Driver doesn't use DMA-API */
        .dma_in_ch       = 5,
        .master          = 0, /* codec is master */
};

static struct platform_device sound_device = {
	.name		= "siu_sh7343",
	.dev		= {
		.platform_data = &sound_data,
	},
};

/*********** SD/MMC card ***********/

static struct resource gg_sdhi_resources[] = {
	[0] = {
		/* SDHI SDHII0 */
		.start	= 100,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		/* SDHI SDHII1 */
		.start	= 101,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* SDHI SDHII2 */
		.start	= 102,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device sdhi_device1 = {
	.name		= "sh-sdhi",
	.id		= 0,
	.dev		= {
		.dma_mask		= NULL,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(gg_sdhi_resources),
	.resource	= gg_sdhi_resources,
};

static struct resource gg_sdhi_resources2[] = {
	[0] = {
		/* SDHI SDHII0 */
		.start	= 23,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		/* SDHI SDHII1 */
		.start	= 24,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* SDHI SDHII2 */
		.start	= 25,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device sdhi_device2 = {
	.name		= "sh-sdhi",
	.id		= 1,
	.dev		= {
		.dma_mask	= NULL,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(gg_sdhi_resources2),
	.resource	= gg_sdhi_resources2,
};

static struct platform_device* gg_sdhi_devices[] __initdata = {
        &sdhi_device1,
        &sdhi_device2,
};

static struct resource gg_pwm_tpu_resources[] = {
	[0] = {
		.start	= 0xA4C90000,
		.end	= 0xA4C900F6,
		.flags	= IORESOURCE_MEM,
        },
};

static struct platform_device gg_pwm_tpu = {
	.name		= "pwm_tpu",
	.num_resources	= ARRAY_SIZE(gg_pwm_tpu_resources),
	.resource	= gg_pwm_tpu_resources,
};


static struct platform_pwm_backlight_data gg_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 111,
	.dft_brightness	= 111,
	.pwm_period_ns	= 9009009, /* 111 Hz */
#warning add .init, .notify, .exit, arm/mach-pxa/palmt5.c
};

static struct platform_device gg_backlight = {
	.name = "pwm-backlight",
	.dev = {
		.platform_data = &gg_backlight_data,
	},
};

static struct platform_device* gg_devices[] __initdata = {
	&gg_nor_flash_device,
	&dm9000_device,
	&nand_flash_device,
	&sound_device,
        &gg_pwm_tpu,
        &gg_backlight,
        &gg_apr_e102375_sh_device,
        &gg_i2c_gpio_device,
};

static struct platform_device* gg_lcd_devices[] __initdata = {
	&lcdc_device,
};

static int __init gg_setup_display(void)
{
        static const int gpios[] = {
                GPIO_PTH4,
                GPIO_PTH1,
                GPIO_PTH0,
                GPIO_PTN7,
                GPIO_PTN6,
                GPIO_PTN5,
                GPIO_PTN4,
                GPIO_PTN3,
                GPIO_PTN2,
                GPIO_PTN1,
                GPIO_PTN0,
                GPIO_PTL7,
                GPIO_PTL6,
                GPIO_PTL5,
                GPIO_PTL4,
                GPIO_PTL3,
                GPIO_PTL2,
                GPIO_PTL1,
                GPIO_PTL0,
                GPIO_PTM7,
                GPIO_PTM6,
                GPIO_PTM5,
                GPIO_PTM4,
                GPIO_PTM3,
                GPIO_PTM2,
                GPIO_PTM1,
                GPIO_PTM0,
        };

        int display;
        int res;

        /* register and configure pins */
        gpio_request_array(gpios, ARRAY_SIZE(gpios), "lcd");

        display = display_selected(lcdc_info, ARRAY_SIZE(lcdc_info));
        if (display == -1) {
                if (lcdc_info[BOOTLOADER_CONFIGURED].lcdc_is_already_configured_and_running)
                        display=BOOTLOADER_CONFIGURED;
                else
                        pr_err("Selected display \"%s\" unknown in kernel and not yet initialized by bootloader\n", display_name);
        }

        if (display >= 0) {
                pr_info("Selected LCD display %s\n",
                       lcdc_info[display].ch[0].lcd_cfg.name);

                lcdc_device.dev.platform_data = &lcdc_info[display];

                res = platform_add_devices(gg_lcd_devices, ARRAY_SIZE(gg_lcd_devices));
                if (res < 0)
                        goto error;
        } else
                pr_info("No LCD display selected\n");

        return 0;

error:
        pr_err("%s: Display couldn't be initialized\n", __FUNCTION__ );

        return res;
}

static int __init gg_setup_sdhi(void)
{
        int res;

#ifdef CONFIG_SH_EARLY_SD_FIRMWARE
        if (sdhi_device1.dev.platform_data == NULL) {
                pr_info("SD firmware not present, skipping sh_sdhi\n");
                return 0;
        }
#endif

#warning check me
	ctrl_outw((ctrl_inw(PSELA) & ~0x0C00) | 0x0400, PSELA);
	ctrl_outw(ctrl_inw(MSELCRB) | 0x0008, MSELCRB); /* PTD */
	ctrl_outw(0x0000, PORT_PCCR); /* SDHI1 */
	ctrl_outw(0x0000, PORT_PDCR); /* SDHI0 */

        res = platform_add_devices(gg_sdhi_devices,
                                   ARRAY_SIZE(gg_sdhi_devices));

        if (res)
                pr_err("%s: SDHI initialization failed\n", __FUNCTION__ );

        return res;
}

static int __init gg_setup_usb(void)
{
        struct device *dev = bus_find_device_by_name(&platform_bus_type, NULL, "r8a66597_hcd.0");

        if (dev != NULL) {
                static const int gpios[] = {USBH_PEN};

                struct r8a66597_platdata *pdata = dev_get_platdata(dev);

                if (gpio_request_array(gpios, ARRAY_SIZE(gpios), "usb"))
                        return -EBUSY;

                gpio_direction_output(USBH_PEN, 1);

                pdata->init   = gg_usb_init;
                pdata->deinit = gg_usb_deinit;
                pdata->disable_highspeed = 0;
        }

        return 0;
}

static int __init gg_setup_sound(void)
{
        static const int gpios[] = {
                GPIO_PTZ7,
                GPIO_PTZ6,
                GPIO_PTZ5,
                GPIO_PTZ4,
                GPIO_PTZ2,
                GPIO_PTZ1,
                GPIO_PTZ0,
        };

        gpio_request_array(gpios, ARRAY_SIZE(gpios), "sound");

        return 0;
}

static int __init gg_setup_nand(void)
{
        /* struct flctl is that part of sh_flctl needed for FLCMNCR.
           sh_flctl is quite big, so don't put it on kernel stack */
        struct {
                void __iomem *reg;
        } flctl = {
                .reg = (void __iomem*) nand_flash_resources[0].start,
        };

        /* reuse nand timing configured already by bootloader to support
         * different chip types which the linux kernel doesn't need to be aware of */
        nand_flash_data.flcmncr_val =
                TYPESEL_SET |
                NANWF_E |
                (readl(FLCMNCR((&flctl))) & ( QTSEL_E | FCKSEL_E ));

        return 0;
}

static int __init gg_setup_apm6633(void)
{
        static const int gpios[] = {GPIO_PTH6};

        if (gpio_request_array(gpios, ARRAY_SIZE(gpios), "apm6633") < 0)
                return -EBUSY;

        /* take APM6633 out of reset so we can use wireless and bluetooth */
        gpio_direction_output(GPIO_PTH6, 1);

        return 0;
}

extern char gg_sdram_enter_start;
extern char gg_sdram_enter_end;
extern char gg_sdram_leave_start;
extern char gg_sdram_leave_end;

static int __init gg_devices_setup(void)
{
        int res;

#if 0
	/* register board specific self-refresh code */
	sh_mobile_register_self_refresh(SUSP_SH_STANDBY | SUSP_SH_SF,
					&gg_sdram_enter_start,
					&gg_sdram_enter_end,
					&gg_sdram_leave_start,
					&gg_sdram_leave_end);
#endif

        if ((res = gg_setup_apm6633()))
                goto error;

        if ((res = gg_setup_usb()))
                goto error;

        if ((res = gg_setup_nand()))
                goto error;

        if ((res = gg_setup_sound()))
                goto error;

        if ((res = gg_setup_sdhi()))
                goto error;

        res = i2c_register_board_info(I2C_BUS_SH, gg_i2c_devices,
                                      ARRAY_SIZE(gg_i2c_devices));
        if (res < 0) {
                pr_err("%s: i2c initialization failed\n", __FUNCTION__ );
                goto error;
        }

	if ((res = i2c_register_board_info(I2C_BUS_GPIO, gg_i2c_gpio_devices,
                                           ARRAY_SIZE(gg_i2c_gpio_devices)))) {
                pr_err("%s: i2c-gpio initialization failed\n", __FUNCTION__ );
                goto error;
        }

        if ((res = platform_add_devices(gg_devices,
                                        ARRAY_SIZE(gg_devices)))) {
                pr_err("%s: platform devices couldn't be added\n", __FUNCTION__ );
                goto error;
        }

        if ((res = gg_setup_display()))
                goto error;

error:
        return res;
}

arch_initcall(gg_devices_setup);

static void __init gg_setup(char **cmdline_p)
{
        /*
          Set default priorities (based on HiCO.DIMM7723 Rev. 2a with NL6448
          display)

          o Camera(ov772x,not analog) will lock-up/timeout in shcodecs-cap when
            lmb and icb is 0x18.
          o Camera(ov772x) will return blank data when lmb and icb <=0x28
          o When Meram is not active, screen will flicker when lm and icb
            is default 0x18. When Meram is active, lmb and icb can be
            reduced to 0x18
          o 2dg/lcd will lock-up in some examples of df_dok when 2dg<0x2a
        */
        gpr_priority_set("lmb", 0x2A);
        gpr_priority_set("icb", 0x2A);
        gpr_priority_set("2dg", 0x2A);

        /* Audio channels 4 and 5 should have higher priority than the rest as
         * any disturbance is immediately noticed.
         * When using cache write-through instead of write-back, walking through
         * the alsamixer controls can already have an impact */
        gpr_priority_set("dmac0", 0x2B);

#ifdef CONFIG_SH_EARLY_SD_FIRMWARE
        /* firmware needs to be initialized while still using bootmem */
        sdhi_device1.dev.platform_data = sdhi_device2.dev.platform_data = sd_firmware_init();
#endif

        display_read_parameters(&lcdc_info[BOOTLOADER_CONFIGURED]);
}

static void gg_reset(void)
{
#warning todo
}

static struct sh_machine_vector mv_gg __initmv = {
	.mv_name	= "Greengage",
	.mv_setup	= gg_setup,
	.mv_reset	= gg_reset,
};

/* unresolved externals for other drivers */

static int __init gg_nand_bbt_setup(char *options)
{
        if (options == NULL)
                return 0;

        nand_flash_data.has_bbt = simple_strtoul(options, NULL, 0);

        return 1;
}
__setup("nand_bbt=", gg_nand_bbt_setup);
