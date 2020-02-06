/*
 * AK4644AENP CODEC
 *
 * Copyright (C) 2005  ALGO SYSTEM CO.,LTD.
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
#include <linux/i2c.h>
#include <linux/delay.h>

#include "ak4644_codec.h"

#define DRV_VERSION "0.1.0"

static unsigned short normal_i2c[] = { 0x12, I2C_CLIENT_END };

/* Module parameters */
I2C_CLIENT_INSMOD;

static struct i2c_client *ak4644_client = NULL;

static unsigned char ak4644_init[] =
#ifdef CONFIG_CPU_SUBTYPE_SH7723
/*     0     1     2     3     4     5     6     7     8     9     a     b     c     d     e     f  */
{   0x6d, 0x7b, 0x14, 0x00, 0x50, 0x00, 0x00, 0x00, 0xe1, 0xe1, 0x18, 0x00, 0xe1, 0x18, 0x01, 0x01,
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0x04, 0x00, 0x00
};
#else
/*     0     1     2     3     4     5     6     7     8     9     a     b     c     d     e     f  */
{   0x6d, 0x7b, 0x14, 0x00, 0xd3, 0x00, 0x00, 0x00, 0xe1, 0xe1, 0x18, 0x00, 0xe1, 0x18, 0x01, 0x01,
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0x04, 0x00, 0x00
};
#endif
/*----------------------------------------------------------------------------*/

static int ak4644attach(struct i2c_adapter *adapter);
static int ak4644detach(struct i2c_client *client);

static struct i2c_driver ak4644driver = {
	.driver		= {
		.name	= "ak4644_codec",
	},
	.id		= I2C_DRIVERID_AK4644,
	.attach_adapter = &ak4644attach,
	.detach_client	= &ak4644detach,
};

/*----------------------------------------------------------------------------*/

static int ak4644_i2c_xfer(struct i2c_msg *msgs)
{
	msgs->addr = ak4644_client->addr;
	return i2c_transfer(ak4644_client->adapter, msgs, 1);
}

static int ak4644_codec_init(void)
{
	struct i2c_msg msg;
	int i;
	int rc = 0;
	u_int8_t buf[3];

	msg.flags = 0;
	msg.buf = buf;

	for(i = 0; i < 0x24; i++){
		buf[0] = i;			/* Register address */
		buf[1] = ak4644_init[i];	/* Register value */
		msg.len = 2;
		rc = ak4644_i2c_xfer(&msg);
		if (rc != 1)
			goto fail;
	}
	return 0;

fail:
	printk(KERN_ERR "ak4644_i2c_xfer() failed err=%d\n", rc);
	return rc;
}

int ak4644_codec_set_rate(u_int32_t rate)
{
	struct i2c_msg msg;
	int rc = 0;
	u_int8_t buf[3];

	msg.flags = 0;
	msg.buf = buf;

	buf[0] = 0x01;				/* Register address */
	buf[1] = ak4644_init[0x01] & ~0x1;	/* PMPLL=0 */
	msg.len = 2;
	rc = ak4644_i2c_xfer(&msg);
	if (rc != 1)
		goto fail;

	/* Sampling freq */
	buf[0] = 0x05;				/* Register address */
	buf[1] = ak4644_init[0x05];		/* default value */
	switch (rate) {
	case 44100:
		buf[1] |= 0x27;			/* 44.1 KHz */
		break;
	case 22050:
		buf[1] |= 0x07;			/* 22.05 KHz */
		break;
	case 11025:
		buf[1] |= 0x05;			/* 11.025 KHz */
		break;
	case 48000:
		buf[1] |= 0x23;			/* 48 KHz */
		break;
	case 32000:
		buf[1] |= 0x22;			/* 32 KHz */
		break;
	case 24000:
		buf[1] |= 0x03;			/* 24 KHz */
		break;
	case 16000:
		buf[1] |= 0x02;			/* 16 KHz */
		break;
	case 8000:
		buf[1] |= 0x00;			/* 8 KHz */
		break;
	}
	msg.len = 2;
	rc = ak4644_i2c_xfer(&msg);
	if (rc != 1)
		goto fail;

	buf[0] = 0x01;				/* Register address */
	buf[1] = ak4644_init[0x01];		/* PMPLL=1 */
	msg.len = 2;
	rc = ak4644_i2c_xfer(&msg);
	if (rc != 1)
		goto fail;

	return 0;

fail:
	printk(KERN_ERR "ak4644_i2c_xfer() failed err=%d\n", rc);
	return rc;
}

int ak4644_codec_set_outvol(u_int8_t left, u_int8_t right)
{
	struct i2c_msg msg;
	int rc = 0;
	u_int8_t buf[3];

	msg.flags = 0;
	msg.buf = buf;

	/* left */
	buf[0] = 0x0a;
	buf[1] = left;
	msg.len = 2;
	rc = ak4644_i2c_xfer(&msg);
	if (rc != 1)
		goto fail;

	/* right */
	buf[0] = 0x0d;
	buf[1] = right;
	msg.len = 2;
	rc = ak4644_i2c_xfer(&msg);
	if (rc != 1)
		goto fail;

	return 0;

fail:
	printk(KERN_ERR "ak4644_i2c_xfer() failed err=%d\n", rc);
	return rc;
}

int ak4644_codec_set_invol(u_int8_t left, u_int8_t right)
{
	struct i2c_msg msg;
	int rc = 0;
	u_int8_t buf[3];

	msg.flags = 0;
	msg.buf = buf;

	/* left */
	buf[0] = 0x09;
	buf[1] = left;
	msg.len = 2;
	rc = ak4644_i2c_xfer(&msg);
	if (rc != 1)
		goto fail;

	/* right */
	buf[0] = 0x0c;
	buf[1] = right;
	msg.len = 2;
	rc = ak4644_i2c_xfer(&msg);
	if (rc != 1)
		goto fail;

	return 0;

fail:
	printk(KERN_ERR "ak4644_i2c_xfer() failed err=%d\n", rc);
	return rc;
}

int ak4644_codec_select_capture(u_int8_t source)
{
	struct i2c_msg msg;
	int rc = 0;
	u_int8_t buf[3];

	msg.flags = 0;
	msg.buf = buf;

	/* MIC */
	if (source == 0) {
		buf[0] = 0x10;
		buf[1] = 0x06;
		msg.len = 2;
		rc = ak4644_i2c_xfer(&msg);
		if (rc != 1)
			goto fail;

		buf[0] = 0x20;
		buf[1] = 0x3f;
		msg.len = 2;
		rc = ak4644_i2c_xfer(&msg);
		if (rc != 1)
			goto fail;
	}
	/* LINE */
	else {
		buf[0] = 0x10;
		buf[1] = 0x00;
		msg.len = 2;
		rc = ak4644_i2c_xfer(&msg);
		if (rc != 1)
			goto fail;

		buf[0] = 0x20;
		buf[1] = 0x3c;
		msg.len = 2;
		rc = ak4644_i2c_xfer(&msg);
		if (rc != 1)
			goto fail;
	}

	return 0;

fail:
	printk(KERN_ERR "ak4644_i2c_xfer() failed err=%d\n", rc);
	return rc;
}

int ak4644_codec_set_micgain(u_int8_t gain)
{
	static u_int8_t reg[][2] = { {0x00, 0x00},
				     {0x01, 0x00},
				     {0x00, 0x20},
				     {0x01, 0x20} };

	struct i2c_msg msg;
	int rc = 0;
	u_int8_t buf[3];

	msg.flags = 0;
	msg.buf = buf;

	buf[0] = 0x02;
	buf[1] = ak4644_init[0x02] | reg[gain][0];
	msg.len = 2;
	rc = ak4644_i2c_xfer(&msg);
	if (rc != 1)
		goto fail;

	buf[0] = 0x03;
	buf[1] = ak4644_init[0x03] | reg[gain][1];;
	msg.len = 2;
	rc = ak4644_i2c_xfer(&msg);
	if (rc != 1)
		goto fail;

	return 0;

fail:
	printk(KERN_ERR "ak4644_i2c_xfer() failed err=%d\n", rc);
	return rc;
}

int ak4644_codec_set_bassboost(u_int8_t boost)
{
	struct i2c_msg msg;
	int rc = 0;
	u_int8_t buf[3];

	msg.flags = 0;
	msg.buf = buf;

	buf[0] = 0x0e;
	buf[1] = ak4644_init[0x0e] | (boost << 2);
	msg.len = 2;
	rc = ak4644_i2c_xfer(&msg);
	if (rc != 1)
		goto fail;

	return 0;

fail:
	printk(KERN_ERR "ak4644_i2c_xfer() failed err=%d\n", rc);
	return rc;
}

/*----------------------------------------------------------------------------*/

static int ak4644probe(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *client;
	int err = 0;

	dev_dbg(&adapter->dev, "%s\n", __FUNCTION__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (client == 0)
		return -ENOMEM;

	client->addr = address;
	client->driver = &ak4644driver;
	client->adapter	= adapter;
	strlcpy(client->name, "ak4644_codec", I2C_NAME_SIZE);

	/* Inform the i2c layer */
	if ((err = i2c_attach_client(client)))
		goto exit_kfree;

	ak4644_client = client;
	dev_info(&client->dev, "chip found, driver version " DRV_VERSION "\n");

	/* codec init */
	ak4644_codec_init();
	ak4644_codec_set_outvol(OUT_VOLUME[DEFAULT_OUT_VOLUME],
		OUT_VOLUME[DEFAULT_OUT_VOLUME]);
	ak4644_codec_set_invol(IN_VOLUME[DEFAULT_IN_VOLUME],
		IN_VOLUME[DEFAULT_IN_VOLUME]);
	ak4644_codec_select_capture(0);
	ak4644_codec_set_micgain(DEFAULT_MIC_GAIN);
	ak4644_codec_set_bassboost(DEFAULT_BASS_BOOST);

	return 0;

exit_kfree:
	kfree(client);
	return err;
}

static int ak4644attach(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, ak4644probe);
}

static int ak4644detach(struct i2c_client *client)
{
	int err;

	if ((err = i2c_detach_client(client)))
		return err;

	kfree(client);
	return 0;
}

int ak4644init(void)
{
	return i2c_add_driver(&ak4644driver);
}

void ak4644exit(void)
{
	i2c_del_driver(&ak4644driver);
}
