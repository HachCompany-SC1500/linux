/*
 * baseboard_io_camera.c
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
 *
 **/

#include <media/sh_mobile_ceu.h>
#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>
#include <media/ov772x.h>

EM_CMDLINE_FLAG(hico7723_skip_camera);

#define VIO_SRC		GPIO_PTT4
#define VIO_RST		GPIO_PTT5

#define VIO_SRC_VIDEO	1
#define VIO_SRC_CAMERA	0

/* BEWARE. DIMM7722 and DIMM7723 have opposite signal levels for VIO_RST */
#define VIO_RST_CHIP_ENABLE     0
#define VIO_RST_CHIP_DISABLE    1

static int adv7180_max_input = 0; /* we have only AN1 */

static struct sh_mobile_ceu_info sh_mobile_ceu_info = {
	.flags = SH_CEU_FLAG_USE_8BIT_BUS,
};

static struct resource ceu_resources[] = {
	[0] = {
		.name	= "CEU",
		.start	= 0xfe910000,
		.end	= 0xfe91009f,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start  = 52,
		.flags  = IORESOURCE_IRQ,
	},
	[2] = {
		/* place holder for contiguous memory */
	},
};

static struct platform_device ceu_device = {
	.name		= "sh_mobile_ceu",
	.num_resources	= ARRAY_SIZE(ceu_resources),
	.resource	= ceu_resources,
	.dev		= {
		.platform_data	= &sh_mobile_ceu_info,
	},
	.archdata = {
		.hwblk_id = HWBLK_CEU,
	},
};

static int hico7723_camera_probe(struct soc_camera_link *icl,
                                 struct device *dev)
{
	return 1;
}

static int hico7723_camera_add(struct soc_camera_link *icl, struct device *dev);
static void hico7723_camera_del(struct soc_camera_link *icl);

static struct soc_camera_platform_info camera_info = {
	.format_name = "UYVY",
	.format_depth = 16,
	.format = {
		.code       = V4L2_MBUS_FMT_YUYV8_2X8_BE,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field      = V4L2_FIELD_NONE,
		.width      = 640,
		.height     = 480,
	},
	.bus_param = SOCAM_PCLK_SAMPLE_RISING | SOCAM_HSYNC_ACTIVE_HIGH |
	SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_MASTER | SOCAM_DATAWIDTH_8 |
	SOCAM_DATA_ACTIVE_HIGH,
};

struct soc_camera_link camera_link = {
	.bus_id		= 0,
	.add_device	= hico7723_camera_add,
	.del_device	= hico7723_camera_del,
	.module_name	= "soc_camera_platform",
};

static void hico7723_camera_release(struct device *dev)
{
	/* nothing to do */
}

static struct platform_device camera_device = {
	.name		= "soc_camera_platform",
	.dev		= {
		.platform_data	= &camera_info,
		.release	= hico7723_camera_release,
	},
};

static int hico7723_camera_add(struct soc_camera_link *icl,
			       struct device *dev)
{
	if (icl != &camera_link || hico7723_camera_probe(icl, dev) <= 0)
		return -ENODEV;

	camera_info.dev = dev;
	return platform_device_register(&camera_device);
}

static void hico7723_camera_del(struct soc_camera_link *icl)
{
	if (icl != &camera_link)
		return;

	platform_device_unregister(&camera_device);
	memset(&camera_device.dev.kobj, 0,
	       sizeof(camera_device.dev.kobj));
}

static int adv7180_reset(struct device *dev)
{
	extern int adv7180_camera_init(struct device *dev, int max_input);

        gpio_set_value(VIO_RST, VIO_RST_CHIP_DISABLE);
	/* send a reset to camera */
	mdelay(5);
	gpio_set_value(VIO_RST, VIO_RST_CHIP_ENABLE);

	return adv7180_camera_init(dev, adv7180_max_input);
}

static int adv7180_power(struct device *dev, int mode)
{
	gpio_set_value(VIO_SRC, mode ? VIO_SRC_VIDEO : VIO_SRC_CAMERA );

	return 0;
}

static int camera_power(struct device *dev, int mode)
{
	gpio_set_value(VIO_SRC, mode ? VIO_SRC_CAMERA : VIO_SRC_VIDEO );

	if (em_baseboard_is_pellenc()) {
		/* hardware has connected VIO_RST# instead of RESET_BUF# to camera */
		gpio_set_value(VIO_RST, VIO_RST_CHIP_DISABLE);
		/* send a reset to camera */
		mdelay(5);
		gpio_set_value(VIO_RST, VIO_RST_CHIP_ENABLE);
	}

	return 0;
}

static struct i2c_board_info hico7723_i2c_camera_ov772x = {
        I2C_BOARD_INFO("ov772x", 0x21),
};

static struct ov772x_camera_info ov772x_info = {
	.flags		= OV772X_FLAG_VFLIP | OV772X_FLAG_HFLIP | OV772X_FLAG_8BIT,
	.edgectrl	= OV772X_AUTO_EDGECTRL(0xf, 0),
};

static struct soc_camera_link ov772x_link = {
	.bus_id		= 0,
	.power		= camera_power,
	.board_info	= &hico7723_i2c_camera_ov772x,
	.i2c_adapter_id	= 0,
	.module_name	= "ov772x",
	.priv		= &ov772x_info,
};

static struct i2c_board_info hico7723_i2c_camera_mt9v022 = {
	I2C_BOARD_INFO("mt9v022", 0x4C),
};

static int hico7723_mt9v022_camera_set_bus_param(struct soc_camera_link *link,
				       unsigned long flags)
{
        /* mt9v022 is used without switch => nothing to do */
        return 0;
}

static unsigned long hico7723_mt9v022_camera_query_bus_param(struct soc_camera_link *link)
{
        /* without switch or switch is configured by default for 8bit */
        return SOCAM_DATAWIDTH_8;
}

static struct soc_camera_link mt9v022_link = {
        .bus_id		= 0,
	.power		= camera_power,
        .board_info	= &hico7723_i2c_camera_mt9v022,
        .i2c_adapter_id	= 0,
        .query_bus_param= hico7723_mt9v022_camera_query_bus_param,
        .set_bus_param	= hico7723_mt9v022_camera_set_bus_param,
        .module_name	= "mt9v022",
};

static struct i2c_board_info hico7723_i2c_camera_adv7180 =
{	I2C_BOARD_INFO("adv7180", 0x20), /* INT_A not yet used */
};

static struct soc_camera_link adv7180_link = {
	.bus_id		= 0,
	.reset		= adv7180_reset,
	.power		= adv7180_power,
	.board_info	= &hico7723_i2c_camera_adv7180,
	.i2c_adapter_id	= 0,
	.module_name	= "adv7180",
};

static struct platform_device hico7723_camera = {
	.name	= "soc-camera-pdrv",
	.id	= 1,
	.dev	= {
		.platform_data = &camera_link,
	},
};

static struct platform_device hico7723_camera_mt9v022 = {
	.name	= "soc-camera-pdrv",
	.id	= 2,
	.dev	= {
		.platform_data = &mt9v022_link,
	},
};

static struct platform_device hico7723_camera_adv7180 = {
	/* digital cameras have preferences to adv7180*/
	.name	= "soc-camera-pdrv",
	.id	= 3,
	.dev	= {
		.platform_data = &adv7180_link,
	},
};

static struct platform_device hico7723_camera_ov772x = {
	.name	= "soc-camera-pdrv",
	.id	= 0,
	.dev	= {
		.platform_data = &ov772x_link,
	},
};

static int __init hico7723_setup_camera(void)
{
	int res;
	static struct platform_device* camera_devices[] = {
		&ceu_device,
		&hico7723_camera,
	};
        static struct gpio gpios[] = {
		{ .gpio = VIO_SRC, .flags = GPIOF_OUT_INIT_LOW,  "camera"},
		{ .gpio = VIO_RST, .flags = GPIOF_OUT_INIT_HIGH, "camera"},
        };
        static const int gpiosf[] = {
		GPIO_FN_VIO_D7,
		GPIO_FN_VIO_D6,
		GPIO_FN_VIO_D5,
		GPIO_FN_VIO_D4,
		GPIO_FN_VIO_D3,
		GPIO_FN_VIO_D2,
		GPIO_FN_VIO_D1,
		GPIO_FN_VIO_D0,
		GPIO_FN_VIO_FLD,
		GPIO_FN_VIO_HD1,
		GPIO_FN_VIO_VD1,
		GPIO_FN_VIO_CLK1,
        };

	if (hico7723_skip_camera)
		return 0;

	res = gpio_request_array(gpios, ARRAY_SIZE(gpios));
	if (res<0)
		return res;

	res = gpio_fn_request_array(gpiosf, ARRAY_SIZE(gpiosf), "camera");
	if (res<0)
		return res;

	res = platform_add_devices(camera_devices, ARRAY_SIZE(camera_devices));
	if (res<0)
		return res;

	return platform_resource_setup_memory(&ceu_device, "ceu", 4 << 20);
}
