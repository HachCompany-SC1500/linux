
/*
 * arch/sh/boards/mach-emtrion/hico7723/setup.c
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
 * Description: Provides the configuration for the HiCO.DIMM7723
 *
 * References:
 *       [1] http://www.atel.ru/docs/NL6448BC20-18D.pdf
 *       [2] file://W:/Datenblaetter/Renesas/SH-Mobile/SH7723/SH7723_Rev1.00_Eng_edi1958.pdf
 *       [3] file://R:\K-kundspezprd\Secure Electrans\HW\Baugruppen\ADA_UMSH-8272MD\Source\UMSH-8272MD-1T_V0_R1_20081024.P.pdf
 *       [4] R:\8_renesas\HiCO.DIMM\HiCO.DIMM-Base\HW\Baugruppen\Source\Info\datasheets\ICs\ADV7180_VideoIn.pdf
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
#include <linux/usb/r8a66597.h>
#include <linux/em_baseboard.h>
#include <linux/em_device_setup.h>
#include <linux/em_module.h>
#include <linux/em_common.h>
#include <linux/em_firmware.h>
#include <video/sh_mobile_lcdc.h>
#include <cpu/sh7723.h>
#include <asm/setup.h>
#include <asm/pgtable.h>
#include <asm/clock.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/machvec.h>
#include <asm/irq.h>
#include <asm/hico7723.h>

#include "../include/hico7723.h"
#include "../include/sh7723.h"
// #include "sh7723.h"
#include "../arch/sh/drivers/gpr_priority.h"

#include "../display.h"
#include "../bootlogo.h"

#define LCD_ON 		GPIO_PTH2
#define LCD_DISP	GPIO_PTH4
#define USBH_OC		GPIO_PTF6
#define USBH_PEN	GPIO_PTF5
#define USBF_WAKE 	GPIO_PTF0
#define SOFTRESET	GPIO_PTF7
#define LED_D2_GREEN	GPIO_PTJ7
#define LED_D2_RED	GPIO_PTJ5

/* using NOR and watchdog doesn't work reliable on revision <r5a, therefore it can be disabled */
EM_CMDLINE_FLAG(hico7723_skip_nor);
EM_CMDLINE_FLAG(hico7723_skip_nand);

static void hico7723_lcd_on(void *board_data)
{
        gpio_set_value(LCD_ON, 1);
}

static void hico7723_lcd_off(void *board_data)
{
        gpio_set_value(LCD_ON, 0);
}

static void hico7723_usb_power(int port, int power)
{
        gpio_set_value(USBH_PEN, power);
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
                        .board_cfg = {
                                 .display_on  = hico7723_lcd_on,
                                 .display_off = hico7723_lcd_off,
                                 .configure_data_pins  = display_configure_data_pins,
#ifdef CONFIG_SH_BOOTLOGO_OVERLAY_WITH_LINUX
                                 .framebuffer_registered = bootlogo_overlay_with_linux_logo,
#endif
                         },
                },
        },
        // add overrides or new displays
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
		.start	= 0x16000000,
		.end	= 0x16000001,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0x16000002,
		.end	= 0x16000003,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= evt2irq(IRQEVT_IRQ5),
		.end	= evt2irq(IRQEVT_IRQ5),
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
};

static struct resource dm9013_resources[] = {
		/* NOTE: The following resource
		 * items are also hard coded in the dm9006 ethernet driver:
		 */
	[0] = {
		.start	= 0x1a000000,
		.end	= 0x1a000001,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0x1a000002,
		.end	= 0x1a000003,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= evt2irq(IRQEVT_IRQ4),
		.end	= evt2irq(IRQEVT_IRQ4),
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
};


/*********** I2C ***********/

static struct i2c_board_info hico7723_i2c_devices[] = {
        {	I2C_BOARD_INFO("ds1337",      0x68),   },
};

static struct i2c_board_info hico7723_r1a_i2c_devices[] = {
        {
                I2C_BOARD_INFO("tsc2007",     0x48),
                .irq = evt2irq(IRQEVT_IRQ7)
	},
};

static struct i2c_board_info hico7723_r4a_i2c_devices[] = {
        {
                I2C_BOARD_INFO("ar1020",     0x4D),
                .irq = evt2irq(IRQEVT_IRQ7)
	},
};

/*********** Ethernet ***********/
static struct platform_device dm9000_device = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(dm9000_resources),
	.resource	= dm9000_resources,
};

//static struct platform_device dm9000_device = {
//	.name		= "dm9000",
//	.id		= -1,
//	.num_resources	= ARRAY_SIZE(dm9000_dual_resources),
//	.resource	= dm9000_dual_resources,
//};

static struct platform_device dm9013_device = {
	.name		= "dm9013",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(dm9013_resources),
	.resource	= dm9013_resources,
};

/*********** NOR ***********/
static struct physmap_flash_data hico7723_nor_flash_data = {
	.width		= 2,
};

static struct resource hico7723_nor_flash_resources[] = {
	[0] = {
		.name		= "NOR Flash",
		.start		= 0x00000000,
		.end		= 0x00800000,
		.flags		= IORESOURCE_MEM,
	}
};

static struct platform_device hico7723_nor_flash_device = {
	.name		= "physmap-flash",
	.resource	= hico7723_nor_flash_resources,
	.num_resources	= ARRAY_SIZE(hico7723_nor_flash_resources),
	.dev		= {
		.platform_data = &hico7723_nor_flash_data,
	},
};

/*********** NAND ***********/
static struct resource nand_flash_resources[] = {
	[0] = {
		.start	= 0xa4530000,
		.end	= 0xa45300ff,
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

/* ********** USB Gadget ********** */
static struct resource hico7723_isp1181_resources[] = {
	[0] = {
		.name  = "isp1181_udc",
		.start = 0x14000000,
		.end   = 0x14000003,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = evt2irq(IRQEVT_IRQ3),
		.flags = IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
	},
};
static struct platform_device hico7723_isp1181_device = {
	.name          = "isp1181_udc",
	.id            = -1,
	.num_resources = ARRAY_SIZE(hico7723_isp1181_resources),
	.resource      = hico7723_isp1181_resources,
};

static int __init hico7723_setup_usb_gadget(void)
{
	int res;

	res = gpio_request(GPIO_FN_IRQ3, "isp1181_udc");

	if (!res)
		res = gpio_request_one(USBF_WAKE, GPIOF_OUT_INIT_LOW, "isp1181");

	return res;
}
/*********** all devices ***********/

static struct platform_device* hico7723_devices[] = {
	&dm9000_device,
	&dm9013_device,
	&hico7723_isp1181_device,
};

static int __init hico7723_setup_lcd(void)
{
	static const int gpios[] __initdata = {
		GPIO_FN_LCDVCPWC,
		GPIO_FN_LCDVEPWC,
		GPIO_FN_LCDVSYN,
		GPIO_FN_LCDHSYN,
		GPIO_FN_LCDDCK,
		GPIO_FN_LCDRD,
		GPIO_FN_LCDD23,
		GPIO_FN_LCDD22,
		GPIO_FN_LCDD21,
		GPIO_FN_LCDD20,
		GPIO_FN_LCDD19,
		GPIO_FN_LCDD18,
		GPIO_FN_LCDD17,
		GPIO_FN_LCDD16,
		GPIO_FN_LCDD15,
		GPIO_FN_LCDD14,
		GPIO_FN_LCDD13,
		GPIO_FN_LCDD12,
		GPIO_FN_LCDD11,
		GPIO_FN_LCDD10,
		GPIO_FN_LCDD9,
		GPIO_FN_LCDD8,
		GPIO_FN_LCDD7,
		GPIO_FN_LCDD6,
		GPIO_FN_LCDD5,
		GPIO_FN_LCDD4,
		GPIO_FN_LCDD3,
		GPIO_FN_LCDD2,
		GPIO_FN_LCDD1,
		GPIO_FN_LCDD0,
	};
	int flag;
	int res;

        /* register and configure pins */
        // return 0; // MS
        res = gpio_fn_request_array(gpios, ARRAY_SIZE(gpios), "lcdc");
	if (res<0)
		return res;

	if (lcdc_info[BOOTLOADER_CONFIGURED].lcdc_is_already_configured_and_running)
		flag = GPIOF_OUT_INIT_HIGH;
	else
		flag = GPIOF_OUT_INIT_LOW;
	res = gpio_request_one(LCD_ON, flag, "lcdc");
	if (res<0)
		return res;

	/* keep lcddisp as it is configured by the bootloader  */
	res = gpio_request(LCD_DISP, "lcddisp");

	return res;
}

#ifndef CONFIG_SH_HICO7723_ENABLE_USB_HIGH_SPEED
int hico7723_usb_disable_highspeed=1;
#else
int hico7723_usb_disable_highspeed=0;
#endif

int hico7723_usbh_oc_connected;

static int __init hico7723_setup_usb(void)
{
        struct device *dev = bus_find_device_by_name(&platform_bus_type, NULL, "r8a66597_hcd.0");
	int res;

        if (dev != NULL) {
                struct r8a66597_platdata *pdata = dev_get_platdata(dev);

                res = gpio_request_one(USBH_PEN, GPIOF_OUT_INIT_HIGH, "usb");
		if (res<0)
			return res;

		pdata->port_power  = hico7723_usb_power;
                pdata->disable_highspeed = hico7723_usb_disable_highspeed;
		if (hico7723_usbh_oc_connected) {
			pdata->oc.poll_for_oc  = 1;
			pdata->oc.active_level = 0;
			pdata->oc.gpio         = USBH_OC;
		}
        }

	return 0;
}

static int __init hico7723_setup_nor(void)
{
	if (hico7723_skip_nor)
		return 0;

	return platform_device_register(&hico7723_nor_flash_device);
}

static int __init hico7723_setup_nand(void)
{
	static const int gpios[] __initdata = {
		GPIO_FN_FCE,
		GPIO_FN_FCDE,
		GPIO_FN_FOE,
		GPIO_FN_FSC,
		GPIO_FN_FWE,
		GPIO_FN_FRB,
 		GPIO_FN_NAF7,
		GPIO_FN_NAF6,
		GPIO_FN_NAF5,
		GPIO_FN_NAF4,
		GPIO_FN_NAF3,
		GPIO_FN_NAF2,
		GPIO_FN_NAF1,
		GPIO_FN_NAF0,
	};
	int res;

        /* struct flctl is that part of sh_flctl needed for FLCMNCR.
           sh_flctl is quite big, so don't put it on kernel stack */
        struct {
                void __iomem *reg;
        } flctl = {
                .reg = (void __iomem*) nand_flash_resources[0].start,
        };

	if (hico7723_skip_nand)
		return 0;

	res = gpio_fn_request_array(gpios, ARRAY_SIZE(gpios), "nand");
	if (res<0)
		return res;

        /* reuse nand timing configured already by bootloader to support
         * different chip types which the linux kernel doesn't need to be aware of */
        nand_flash_data.flcmncr_val =
                TYPESEL_SET |
                NANWF_E |
                (readl(FLCMNCR((&flctl))) & ( QTSEL_E | FCKSEL_E ));

	return platform_device_register(&nand_flash_device);
}

static int __init hico7723_setup_scif(void)
{
        static const int gpios[] = {
		GPIO_FN_SCIF1_PTS_TXD,
		GPIO_FN_SCIF1_PTS_RXD,
		GPIO_FN_SCIF2_PTT_TXD,
		GPIO_FN_SCIF2_PTT_RXD,
		GPIO_FN_SCIF3_PTS_TXD,
		GPIO_FN_SCIF3_PTS_RXD,
		GPIO_FN_SCIF3_PTS_CTS,
		GPIO_FN_SCIF3_PTS_RTS,
		GPIO_FN_SCIF4_PTE_TXD,
		GPIO_FN_SCIF4_PTE_RXD,
		GPIO_FN_SCIF5_PTE_TXD,
		GPIO_FN_SCIF5_PTE_RXD,
        };
        struct device *dev = bus_find_device_by_name(&platform_bus_type, NULL, "sh-sci.0");

	if (dev != NULL)
		/* HiCO.DIMM7723 has no sh-sci.0/ttySC0 connected, so remove it.
		 * The device node might still be visible but can't be opened*/
		platform_device_unregister(to_platform_device(dev));

	return gpio_fn_request_array(gpios, ARRAY_SIZE(gpios), "scif");
}

static int __init hico7723_setup_misc(void)
{
	static struct gpio gpios[] = {
		{ .gpio = SOFTRESET,    .flags = GPIOF_OUT_INIT_LOW,  "softreset" },
#ifdef CONFIG_BUSY_LEDS
		{ .gpio = LED_D2_GREEN, .flags = GPIOF_OUT_INIT_HIGH, "led d2 green" },
		{ .gpio = LED_D2_RED,   .flags = GPIOF_OUT_INIT_HIGH, "led d2 red" },
#endif
	};
        static const int gpios_bsc[] = {
		GPIO_FN_WE3_ICIOWR,
		GPIO_FN_WE2_ICIORD,
		GPIO_FN_WAIT,
		GPIO_FN_CS5A_CE2A,
		GPIO_FN_CS5B_CE1A,
		GPIO_FN_CS6A_CE2B,
		GPIO_FN_CS6B_CE1B,
        };
	int res;

#ifdef CONFIG_BUSY_LEDS
        pr_info("Enabling busy LEDs\n");
#endif

        res = gpio_request_array(gpios, ARRAY_SIZE(gpios));
	if (res<0)
		return res;

	res = gpio_fn_request_array(gpios_bsc, ARRAY_SIZE(gpios_bsc), "bsc");
	if (res<0)
		return res;

	res = gpio_request(GPIO_FN_IRQ5, "dm9000");
	if (res<0)
		return res;

	res = gpio_request(GPIO_FN_IRQ4, "dm9013");
	if (res<0)
		return res;

	res = gpio_request(GPIO_FN_IRQ6, "ds1337");
	if (res<0)
		return res;

	return gpio_request(GPIO_FN_IRQ7, "tsc2007");
}

static int __init hico7723_setup_display(void)
{
        int display = display_selected(lcdc_info, ARRAY_SIZE(lcdc_info));
        int res;

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

                res = platform_device_register(&lcdc_device);
                if (res < 0)
                        goto error;
        } else
                pr_info("No LCD display selected\n");

        return 0;

error:
        return res;
}

static int __init hico7723_setup_i2c(void)
{
	int res;
	int rev;

	res = i2c_register_board_info(0, hico7723_i2c_devices,
				      ARRAY_SIZE(hico7723_i2c_devices));
	if (res<0)
		return res;

	rev = em_module_hw_revision_as_hex();
	if (rev >= EM_MODULE_REV_4a)
		res = i2c_register_board_info(0, hico7723_r4a_i2c_devices, ARRAY_SIZE(hico7723_r4a_i2c_devices));
	else if (rev >= EM_MODULE_REV_1a)
		res = i2c_register_board_info(0, hico7723_r1a_i2c_devices, ARRAY_SIZE(hico7723_r1a_i2c_devices));
	else
		pr_err("Unknown module hardware revision %s\n", em_module_hw_revision());

	return res;
}

int __init hico7723_devices_setup(const struct device_setup devices[], int len)
{
	int res;
	int i;

	for (i=0; i < len; i++) {
		res = devices[i].fn();
		if (res<0) {
			pr_err("setup failed at function %s\n", devices[i].name);
			return res;
		}
	}

	return 0;
}

#ifdef CONFIG_FAN
static int __init hico7723_setup_fan(void)
{
	uint16_t reg_val_u16 = 0x0000;
	uint8_t reg_val_u8   = 0x00;

	printk("Starting Fan\n");

   /*
    * configure PTD.2 to operate as output pin, bits 5,4 = 01
    */
	reg_val_u16 = readw(PORT_PDCR);
/*	printk("reading PORT_PDCR = 0x%04X from address 0x%X\n", reg_val_u16, PORT_PDCR);  */
	reg_val_u16 |= 0x0010;
	reg_val_u16  &= ~0x0020;
/*	printk("Configure PORT_PDCR with 0x%04X\n", reg_val_u16);  */
	writew(reg_val_u16, PORT_PDCR);

   /*
    * set PTD.2 to high level
    */
	reg_val_u8 = readb(PORT_PDDR);
/*	printk("reading PORT_PDDR = 0x%02X from address 0x%X\n", reg_val_u8, PORT_PDDR);  */
	reg_val_u8 |= 0x04;
/*	printk("Configure PORT_PDDR with 0x%02X\n", reg_val_u8);  */
	writeb(reg_val_u8, PORT_PDDR);

	return 0;
}
#endif

static int __init hico7723_setup_led(void)
{
	uint16_t reg_val_u16 = 0x0000;
	uint8_t reg_val_u8   = 0x00;

	printk("Enabling LED (green)\n");

   /*
    * configure PTZ.6 and PTZ.7 to operate as output pin, bits 5,4 = 01
    */
	reg_val_u16 = readw(PORT_PZCR);
/*	printk("reading PORT_PZCR = 0x%04X from address 0x%X\n", reg_val_u16, PORT_PZCR);  */
	reg_val_u16 |= 0x5000;
	reg_val_u16  &= ~0xA000;
/*	printk("Configure PORT_PZCR with 0x%04X\n", reg_val_u16);  */
	writew(reg_val_u16, PORT_PZCR);

   /*
    * set PTZ.6 to high and and PTZ.7 to low level
    */
	reg_val_u8 = readb(PORT_PZDR);
/*	printk("reading PORT_PZDR = 0x%02X from address 0x%X\n", reg_val_u8, PORT_PZDR);  */
	reg_val_u8 |= 0x80;
	reg_val_u8 &= ~0x40;
/*	printk("Configure PORT_PZDR with 0x%02X\n", reg_val_u8);  */
	writeb(reg_val_u8, PORT_PZDR);

	return 0;
}



static int __init hico7723_init(void)
{
        int res;

	static const struct device_setup module_devices[] __initdata = {
		EM_MKS(hico7723_setup_nor),
		EM_MKS(hico7723_setup_misc),
		EM_MKS(hico7723_setup_lcd),
		EM_MKS(hico7723_setup_display),
		EM_MKS(hico7723_setup_usb),
		EM_MKS(hico7723_setup_nand),
		EM_MKS(hico7723_setup_i2c),
		EM_MKS(hico7723_setup_usb_gadget),
		EM_MKS(hico7723_setup_scif),
#ifdef CONFIG_FAN
		EM_MKS(hico7723_setup_fan),
#endif
		EM_MKS(hico7723_setup_led),
	};

        res = platform_add_devices(hico7723_devices,
                                   ARRAY_SIZE(hico7723_devices));
        if (res < 0) {
                pr_err("%s: platform devices couldn't be added\n", __FUNCTION__ );
                goto out;
        }

	res = em_baseboard_register();
	if (res < 0)
		goto out;

	res = em_devices_setup(module_devices, ARRAY_SIZE(module_devices));

out:
        return res;
}

arch_initcall(hico7723_init);

static void __init hico7723_setup(char **cmdline_p)
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

        display_read_parameters(&lcdc_info[BOOTLOADER_CONFIGURED]);

#ifdef CONFIG_EM_FIRMWARE_BY_BOOTLOADER
	em_firmware_init(FIRMWARE_START, FIRMWARE_SIZE);
#endif
}

static void hico7723_reset(void)
{
        /* do a reset by the reset controller */
        gpio_set_value(SOFTRESET, 1);
}


#ifdef CONFIG_BUSY_LEDS
static void hico7723_led_busy(int on)
{
        u8 status = __raw_readb(PORT_PJDR);
        status = on ?
                (status |  PIN_DATA(LED_D2_GREEN)) :
                (status & ~PIN_DATA(LED_D2_GREEN));
        __raw_writeb(status, PORT_PJDR);
}

static void hico7723_led_in_irq(int on)
{
        /* using gpio_set_value is not allowed in do_IRQ
           which is calling us. So do it directly */
        u8 status = __raw_readb(PORT_PJDR);
        status = on ?
                (status |  PIN_DATA(LED_D2_RED)) :
                (status & ~PIN_DATA(LED_D2_RED));
        __raw_writeb(status, PORT_PJDR);
}
#endif

static void __init hico7723_init_irq(void)
{
	em_baseboard_init_irq();
}

static struct sh_machine_vector mv_hico7723 __initmv = {
	.mv_name	= "HiCO.DIMM7723",
	.mv_setup	= hico7723_setup,
	.mv_reset	= hico7723_reset,
	.mv_init_irq	= hico7723_init_irq,
#ifdef CONFIG_BUSY_LEDS
	.mv_led_busy	= hico7723_led_busy,
	.mv_led_in_irq  = hico7723_led_in_irq,
#endif
};

/* unresolved externals for other drivers */

#ifdef CONFIG_TOUCHSCREEN_TSC2007
int tsc2007_pen_is_down(void)
{
	return __raw_readb(INTREQ00) & 1;
}
EXPORT_SYMBOL(tsc2007_pen_is_down);
#endif

static int __init hico7723_usb_speed_setup(char *options)
{
        if (options != NULL)
		hico7723_usb_disable_highspeed = strcmp("highspeed", options);

        return 0;
}
early_param("hico7723_usb_speed", hico7723_usb_speed_setup);

static int __init hico7723_nand_bbt_setup(char *options)
{
        if (options == NULL)
                return 0;

        nand_flash_data.has_bbt = simple_strtoul(options, NULL, 0);

        return 1;
}
__setup("nand_bbt=", hico7723_nand_bbt_setup);

static int __init hico7723_nand_hwecc_setup(char *options)
{
        if (options == NULL)
                return 0;

        nand_flash_data.has_hwecc = simple_strtoul(options, NULL, 0);

        return 1;
}
__setup("nand_hwecc=", hico7723_nand_hwecc_setup);
