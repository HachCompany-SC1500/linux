/*
 * adv7180_soc.c - ADV7180 driver for the SOC interface
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
 **/

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <media/v4l2-ioctl.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <media/soc_camera.h>

#define DRIVER_NAME "adv7180"
#define USE_SH_READ_BYTE_HACK_BECAUSE_OF_STOP_SIGNAL
#define TIMEOUT_LOCK_MS		 2000
#define WAIT_INTERLACE_DETECT_MS 100
#define MBUS_FMT	V4L2_MBUS_FMT_YUYV8_2X8_BE
#define COLORSPACE	V4L2_COLORSPACE_SMPTE170M

#define ADV7180_INPUT_CONTROL_REG			0x00
#define ADV7180_INPUT_CONTROL_AD_PAL_BG_NTSC_J_SECAM	0x00
#define ADV7180_INPUT_CONTROL_AD_PAL_BG_NTSC_J_SECAM_PED 0x10
#define ADV7180_INPUT_CONTROL_AD_PAL_N_NTSC_J_SECAM	0x20
#define ADV7180_INPUT_CONTROL_AD_PAL_N_NTSC_M_SECAM	0x30
#define ADV7180_INPUT_CONTROL_NTSC_J			0x40
#define ADV7180_INPUT_CONTROL_NTSC_M			0x50
#define ADV7180_INPUT_CONTROL_PAL60			0x60
#define ADV7180_INPUT_CONTROL_NTSC_443			0x70
#define ADV7180_INPUT_CONTROL_PAL_BG			0x80
#define ADV7180_INPUT_CONTROL_PAL_N			0x90
#define ADV7180_INPUT_CONTROL_PAL_M			0xa0
#define ADV7180_INPUT_CONTROL_PAL_M_PED			0xb0
#define ADV7180_INPUT_CONTROL_PAL_COMB_N		0xc0
#define ADV7180_INPUT_CONTROL_PAL_COMB_N_PED		0xd0
#define ADV7180_INPUT_CONTROL_PAL_SECAM			0xe0
#define ADV7180_INPUT_CONTROL_PAL_SECAM_PED		0xf0

#define ADV7180_STATUS1_REG				0x10
#define ADV7180_STATUS1_IN_LOCK		0x01
#define ADV7180_STATUS1_AUTOD_MASK	0x70
#define ADV7180_STATUS1_AUTOD_NTSM_M_J	0x00
#define ADV7180_STATUS1_AUTOD_NTSC_4_43 0x10
#define ADV7180_STATUS1_AUTOD_PAL_M	0x20
#define ADV7180_STATUS1_AUTOD_PAL_60	0x30
#define ADV7180_STATUS1_AUTOD_PAL_B_G	0x40
#define ADV7180_STATUS1_AUTOD_SECAM	0x50
#define ADV7180_STATUS1_AUTOD_PAL_COMB	0x60
#define ADV7180_STATUS1_AUTOD_SECAM_525	0x70

#define ADV7180_IDENT_REG 0x11

#define ADV7180_RESET_REG          0x7
#define ADV7180_STATUS3_REG        0x13
#define ADV7180_STATUS3_INTERLACED 0x40

#define ADV7180_ADC_SWITCH1_REG	   0xC3

struct adv7180_state {
	struct v4l2_subdev	sd;
	struct v4l2_rect	rect;	/* Sensor window */
	int                     y_skip_top;
	int                     analog_input;
	int                     max_input; /* AN1==0, AN6==5 */
	struct mutex            lock;
};

/* CVBS - Composite Video */
static const unsigned char init_composite0[] = {
        0x00, 0x00, /* Input control - composite */

        0x04, 0x57, /* Extended output control - set timing signals output enable (is this needed?)*/
        0x17, 0x41, /* shaping filter control */
        0x31, 0x02,
        0x3D, 0xA2,

	0x3A, 0x17, /* mux0 on, mux1 & mux2 off */
};
static const unsigned char init_composite1[] = {
	0xC4, 0x80, /* MUX Enabled */

        0x3E, 0x6A,
        0x3F, 0xA0,
        0x0E, 0x80,
        0x55, 0x81,
        0x0E, 0x00,
	0x59, 0x1F, /* GPO */
};

#define HS_CONTROL(HSB,HSE) \
        0x34, ((((HSB&0x700)>>8)<<4)|((HSE&0x700)>>8)), \
        0x35, (HSB&0xFF),                           \
        0x36, (HSE&0xFF)
static const unsigned char init_pal[] = {
        0x31, 0x1a, /* VS/FIELD Control 1 (NEWAVMODE=1) */
	0x32, 0x81, /* VS/FIELD Control 2 */
	0x33, 0x84, /* VS/FIELD Control 3 */
        HS_CONTROL(300,0),
	0x37, 0x01, /* Polarity (default=0x1) */
	0xe8, 0x41, /* PALV Bit Begin */
	0xe9, 0x84, /* PAL V Bit End */
	0xea, 0x06, /* PAL F Bit Toggle */
};

static inline struct adv7180_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv7180_state, sd);
}

static v4l2_std_id adv7180_std_to_v4l2(u8 status1)
{
	switch (status1 & ADV7180_STATUS1_AUTOD_MASK) {
	case ADV7180_STATUS1_AUTOD_NTSM_M_J:
		return V4L2_STD_NTSC;
	case ADV7180_STATUS1_AUTOD_NTSC_4_43:
		return V4L2_STD_NTSC_443;
	case ADV7180_STATUS1_AUTOD_PAL_M:
		return V4L2_STD_PAL_M;
	case ADV7180_STATUS1_AUTOD_PAL_60:
		return V4L2_STD_PAL_60;
	case ADV7180_STATUS1_AUTOD_PAL_B_G:
		return V4L2_STD_PAL;
	case ADV7180_STATUS1_AUTOD_SECAM:
		return V4L2_STD_SECAM;
	case ADV7180_STATUS1_AUTOD_PAL_COMB:
		return V4L2_STD_PAL_Nc | V4L2_STD_PAL_N;
	case ADV7180_STATUS1_AUTOD_SECAM_525:
		return V4L2_STD_SECAM;
	default:
		return V4L2_STD_UNKNOWN;
	}
}

static u32 adv7180_status_to_v4l2(u8 status1)
{
	if (!(status1 & ADV7180_STATUS1_IN_LOCK))
		return V4L2_IN_ST_NO_SIGNAL;

	return 0;
}

static inline int adv7180_write (struct i2c_client *client, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static inline int adv7180_read (struct i2c_client *client, u8 reg)
{
	u8 value;
#ifdef USE_SH_READ_BYTE_HACK_BECAUSE_OF_STOP_SIGNAL
	extern int sh_mobile_i2c_read_byte_hack( struct i2c_adapter *adapter, 
						 uint8_t device,
						 uint8_t subaddr);
	value= sh_mobile_i2c_read_byte_hack(client->adapter, client->addr, reg);
#else
	value = i2c_smbus_read_byte_data(client, reg);
#endif
	return value;
}

static int adv7180_write_block(struct i2c_client *client, const u8 *data, unsigned int len)
{
	int ret = -1;
	u8 reg;

	/* the adv7180 has an autoincrement function, use it if
	 * the adapter understands raw I2C */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		/* do raw I2C, not smbus compatible */
		u8 block_data[32];
		int block_len;

		while (len >= 2) {
			block_len = 0;
			block_data[block_len++] = reg = data[0];
			do {
				block_data[block_len++] = data[1];
				reg++;
				len -= 2;
				data += 2;
			} while (len >= 2 && data[0] == reg &&
				 block_len < 32);
			if ((ret = i2c_master_send(client, block_data,
						   block_len)) < 0)
				break;
		}
	} else {
		/* do some slow I2C emulation kind of thing */
		while (len >= 2) {
			reg = *data++;
			if ((ret = adv7180_write(client, reg,
						 *data++)) < 0)
				break;
			len -= 2;
		}
	}

	return ret;
}

static int adv7180_status(struct i2c_client *client, u32 *status,
	v4l2_std_id *std)
{
	int status1 = adv7180_read(client, ADV7180_STATUS1_REG);

	if (status1 < 0)
		return status1;

	if (status)
		*status = adv7180_status_to_v4l2(status1);
	if (std)
		*std = adv7180_std_to_v4l2(status1);

	return 0;
}

static void adv7180_get_config(struct i2c_client *client)
{
	struct adv7180_state *state = to_state(i2c_get_clientdata(client));
	struct config {
		const char *name;
	};
#define MK(n) {.name=n,}
	static const struct config default_config[8] = {
                MK("NTM M/J"),
                MK("NTSC 4.43"),
                MK("PAL M"),
                MK("PAL 60"),
                MK("PAL B/G/H/I/D"),
                MK("SECAM"),
                MK("PAL Combination N"),
                MK("SECAM 525"),
        };
#undef MK
	const struct config *config;
        uint8_t sr1,sr3;

        sr1 = adv7180_read(client, 0x10);
        sr3 = adv7180_read(client, 0x13);

	config = &default_config[(sr1>>4)&0x7];

	state->rect.top  = 0;
	state->rect.left = 0;
/* hardcoded to 640x480 */
	state->rect.width  = 640;
	state->rect.height = 480;

        dev_notice(&client->dev,"ADV7180: detected standard: %s - %s (%s)\n",
                config->name,
                (sr3&(1<<6)) ? "interlaced" : "non-interlaced",
                (sr1&1) ? "locked" : "not locked" );
}

static int adv7180_interlaced(struct i2c_client *client, u32 *interlaced)
{
	int status3 = adv7180_read(client, ADV7180_STATUS3_REG);

	if (status3 < 0)
		return status3;

	if (interlaced)
		*interlaced = !!(status3 & ADV7180_STATUS3_INTERLACED);

	return 0;
}

static int adv7180_is_locked(struct i2c_client *client)
{
    int reg = adv7180_read(client, 0x10);
    if (reg<0)
	    return 0;

    return (reg&1);
}

static int adv7180_wait_lock(struct i2c_client *client)
{
        int timeout = 0;

        /* Wait for codec to lock, so we can read input lines */
        while (!adv7180_is_locked(client)) {

                timeout++;
                if (timeout == TIMEOUT_LOCK_MS) {
                        dev_warn(&client->dev, "not locked to video signal\n");
                        return -ENODEV;
                }

                mdelay(1);
        }


        /* When input is locked, the interlace type (interlaced yes/no) is not
         * yet correctly detected. Wait a few frames until codec had the
         * chance to correctly identify it */

        mdelay(WAIT_INTERLACE_DETECT_MS);

        return 0;
}

static int adv7180_reset(struct adv7180_state *state)
{
	struct i2c_client *client = v4l2_get_subdevdata(&state->sd);
	int ret;

	/* write composite configuration */
	ret = adv7180_write_block(client, init_composite0,
				  sizeof(init_composite0));
	/* The analog input channel must be set as part of the initialization.
	   Otherwise it seems that the analog gain is not automatically
	   detected on the new input */
	if (ret>=0)
		/* linux/userspace counts beginning with 0, hardware begins with 1  */
		ret = adv7180_write(client, ADV7180_ADC_SWITCH1_REG, state->analog_input+1);
	if (ret>=0)
		ret = adv7180_write_block(client, init_composite1,
					  sizeof(init_composite1));

	/* write PAL configuration */
	if (ret>=0)
		ret = adv7180_write_block(client, init_pal,
					  sizeof(init_pal));

        if (ret>=0)
		ret = adv7180_wait_lock(client);

	if (ret>=0)
		adv7180_get_config(client);

	return ret < 0 ? ret : 0;
}

static unsigned int adv7180_get_input(struct adv7180_state *state)
{
	struct i2c_client *client = v4l2_get_subdevdata(&state->sd);
	unsigned int adc_switch;

	mutex_lock(&state->lock);
	adc_switch = adv7180_read(client, ADV7180_ADC_SWITCH1_REG);
	mutex_unlock(&state->lock);

	/* linux/userspace counts beginning with 0, hardware begins with 1  */
	return (adc_switch & 0x7)-1;
}

int adv7180_camera_init(struct device *dev, int max_input)
{
	struct soc_camera_device *icd = to_soc_camera_dev(dev);
	struct soc_camera_link *icl;
	struct adv7180_state *state;

	WARN_ON(icd == NULL);

	icl = to_soc_camera_link(icd);
	WARN_ON(icl == NULL);

	if (icl->priv == NULL)
		/* adv7180_video_probe not passed */
		return 0;

	/* is set on every open */
	state = icl->priv;
	state->max_input = max_input;

	return adv7180_reset(state);
}


static int adv7180_set_input(struct adv7180_state *state, unsigned int input)
{
	struct i2c_client *client = v4l2_get_subdevdata(&state->sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	int ret;

	if (input > state->max_input)
		return -EINVAL;

	mutex_lock(&state->lock);

	state->analog_input = input;

	/* we need to do a real HW reset for gain to adjust */
	ret = icl->reset(icd->pdev);
	mutex_unlock(&state->lock);

	return ret;
}

static int adv7180_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct adv7180_state *state = to_state(sd);
	int ret;

	mutex_lock(&state->lock);
	ret = adv7180_status(v4l2_get_subdevdata(sd), status, NULL);
	mutex_unlock(&state->lock);

	return ret;
}

static int adv7180_g_chip_ident(struct v4l2_subdev *sd,
	struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7180_state *state = to_state(sd);
	int ret;

	mutex_lock(&state->lock);
	ret = v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_ADV7180, 0);
	mutex_unlock(&state->lock);

	return ret;
}

static int adv7180_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	/* not able to set something */
	return 0;
}

static int adv7180_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = sd->priv;
	struct adv7180_state *state = to_state(sd);
	int interlaced;
	int ret;

	mutex_lock(&state->lock);
	ret = adv7180_interlaced(client, &interlaced);
	mutex_unlock(&state->lock);

	if (ret < 0)
		return ret;

	mf->width	= state->rect.width;
	mf->height	= state->rect.height;
	mf->code	= MBUS_FMT;
	mf->colorspace	= COLORSPACE;
	mf->field	= interlaced ? V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;

	return 0;
}

static int adv7180_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = sd->priv;
	struct adv7180_state *state = to_state(sd);
	int interlaced;
	int ret;

	mutex_lock(&state->lock);
	ret = adv7180_interlaced(client, &interlaced);
	mutex_unlock(&state->lock);

	if (ret<0)
		return ret;

	mf->width	= state->rect.width;
	mf->height	= state->rect.height;
	mf->code	= MBUS_FMT;
	mf->colorspace	= COLORSPACE;
	mf->field	= interlaced ? V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;

	return 0;
}

static int adv7180_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	if (index > 0)
		return -EINVAL;

	*code = MBUS_FMT;

	return 0;
}

static int adv7180_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	/* not possible */
	return 0;
}

static int adv7180_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct adv7180_state *state = to_state(sd);

	a->c	= state->rect;
	a->type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int adv7180_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	struct adv7180_state *state = to_state(sd);

	a->bounds  = state->rect;
	a->defrect = a->bounds;
	a->type	   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int adv7180_g_input(struct soc_camera_device *icd, unsigned int *i)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct adv7180_state *state = to_state(sd);

	*i = adv7180_get_input(state);

	return 0;
}

static int adv7180_s_input(struct soc_camera_device *icd, unsigned int i)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct adv7180_state *state = to_state(sd);
	int ret = adv7180_set_input(state, i);

	return ret;
}

static int adv7180_enum_input(struct soc_camera_device *icd,
			      struct v4l2_input *inp)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct adv7180_state *state = to_state(sd);

	if (inp->index < 0 || inp->index > state->max_input)
		return -EINVAL;

	/* default is camera */
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std  = V4L2_STD_UNKNOWN;
	sprintf(inp->name, "AN%i", inp->index+1);

	return 0;
}


static const struct v4l2_subdev_video_ops adv7180_subdev_video_ops = {
	.g_input_status = adv7180_g_input_status,
	.s_mbus_fmt	= adv7180_s_fmt,
	.g_mbus_fmt	= adv7180_g_fmt,
	.try_mbus_fmt	= adv7180_try_fmt,
	.enum_mbus_fmt	= adv7180_enum_fmt,
	.s_crop		= adv7180_s_crop,
	.g_crop		= adv7180_g_crop,
	.cropcap	= adv7180_cropcap,
};

static const struct v4l2_subdev_core_ops adv7180_subdev_core_ops = {
	.g_chip_ident = adv7180_g_chip_ident,
};

static const struct v4l2_subdev_ops adv7180_subdev_ops = {
	.core = &adv7180_subdev_core_ops,
	.video = &adv7180_subdev_video_ops,
};

/*
 * Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one
 */
static int adv7180_video_probe(struct soc_camera_device *icd,
			       struct i2c_client *client)
{
	int ret;

	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

	/* check and show Product ID */
	ret = adv7180_read(client, ADV7180_IDENT_REG);
	if (ret!=0x1b && ret!= 0x1c) {
		dev_err(&client->dev, "failed to identify the chip. reports id %02x instead of 0x1b or 0x1c\n", ret);
		return -EIO;
	}

	v4l_info(client, "chip found @ 0x%02x (%s), id=0x%02x\n",
		 client->addr << 1, client->adapter->name, ret);

	return 0;
}

static void adv7180_video_remove(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	dev_dbg(&icd->dev, "Video removed: %p, %p\n",
		icd->dev.parent, icd->vdev);
	if (icl->free_bus)
		icl->free_bus(icl);
}

static int adv7180_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	/* only one configuration is supported. soc has already checked compatibility with host */
	return 0;
}

static unsigned long adv7180_query_bus_param(struct soc_camera_device *icd)
{
	return  SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_HSYNC_ACTIVE_HIGH  |
		SOCAM_VSYNC_ACTIVE_HIGH  |
		SOCAM_MASTER             |
		SOCAM_DATA_ACTIVE_HIGH   |
		SOCAM_DATAWIDTH_8;
}

static struct soc_camera_ops adv7180_ops = {
	.set_bus_param		= adv7180_set_bus_param,
	.query_bus_param	= adv7180_query_bus_param,
	.g_input                = adv7180_g_input,
	.s_input                = adv7180_s_input,
	.enum_input             = adv7180_enum_input,
};

/*
 * Generic i2c probe
 * concerning the addresses: i2c wants 7 bit (without the r/w bit), so '>>1'
 */

static __devinit int adv7180_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct adv7180_state *state;
	struct v4l2_subdev *sd;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link   *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "driver needs platform data\n");
		return -EINVAL;
	}
	/* we should be the owner of icl->priv */
	BUG_ON(icl->priv!=NULL);

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	state = kzalloc(sizeof(struct adv7180_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	state->max_input    = 0; /* AN1 */
	state->analog_input = 0;
	icl->priv = state;
	mutex_init(&state->lock);

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &adv7180_subdev_ops);

	icd->y_skip_top = 68;
	icd->ops =  &adv7180_ops;

	ret = adv7180_video_probe(icd, client);
	if (ret < 0)
		goto err_unreg_subdev;

	return 0;

err_unreg_subdev:
	v4l2_device_unregister_subdev(sd);
	kfree(state);
err:
	dev_err(&client->dev, "failed to probe: %i\n", ret);
	return ret;
}

static __devexit int adv7180_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct adv7180_state *state = to_state(sd);

	adv7180_video_remove(icd);
	v4l2_device_unregister_subdev(sd);

	icd->ops = NULL;
	i2c_set_clientdata(client, NULL);
	client->driver = NULL;
	kfree(state);

	return 0;
}

static const struct i2c_device_id adv7180_id[] = {
	{DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, adv7180_id);

static struct i2c_driver adv7180_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
	},
	.probe		= adv7180_probe,
	.remove		= __devexit_p(adv7180_remove),
	.id_table	= adv7180_id,
};

static __init int adv7180_init(void)
{
	return i2c_add_driver(&adv7180_driver);
}

static __exit void adv7180_exit(void)
{
	i2c_del_driver(&adv7180_driver);
}

module_init(adv7180_init);
module_exit(adv7180_exit);

MODULE_DESCRIPTION("Analog Devices ADV7180 video decoder driver");
MODULE_AUTHOR("Markus Pietrek");
MODULE_LICENSE("GPL v2");
