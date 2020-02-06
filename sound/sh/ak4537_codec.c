/*
    siu-sh7343.c - Alsa driver for Renesas' SH7343 SIU peripheral.

    Copyright (c) 2006 Carlos Munoz <carlos@kenati.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#define OUT16(addr, val)	(*((volatile u_int16_t *)(addr)) = (val))
#define IN16(addr)		(*((volatile u_int16_t *)(addr)))

#define AK4537_I2C_ADDR		0x0010

#if defined(CONFIG_CPU_SUBTYPE_SH7722)
# define I2C_CH			0
# define GENERAL_PURPOSE_REG	0xb1400004
# define CODEC_RESET		0x0001
#elif defined(CONFIG_CPU_SUBTYPE_SH7354)
# define I2C_CH			0
# define GENERAL_PURPOSE_REG	0xb1840004
# define CODEC_RESET		0x0001
#elif defined(CONFIG_CPU_SUBTYPE_SH7343)
# define I2C_CH			1
# define GENERAL_PURPOSE_REG	0xb1400002
# define CODEC_RESET		0x0100
#else
# error Must set ak4537 board specific values
#endif


extern int i2c_sh7343_xfer_mod_poll(int			ch,
				    struct i2c_msg	*msgs,
				    int			num);

int codec_start(int		master,
		u_int32_t	rate)
{
	struct i2c_msg	msg;
	int		rc = 0;
	u_int8_t	buf[3];

	/* Unreset the codec (reset is active low) */
	OUT16(GENERAL_PURPOSE_REG, IN16(GENERAL_PURPOSE_REG) | CODEC_RESET);

	msg.addr = AK4537_I2C_ADDR;
	msg.flags = 0;
	msg.buf = buf;

	/* Power up VCOM block */
	buf[0] = 0x00;			/* Register address */
	buf[1] = 0x80;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* DATTC independent */
	buf[0] = 0x06;			/* Register address */
	buf[1] = 0x01;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* Mute on */
	buf[0] = 0x0c;			/* Register address */
	buf[1] = 0xff;			/* Register value */
	buf[2] = 0xff;			/* Next consecutive register value */
	msg.len = 3;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* DAC to headphone amp enabled */
	buf[0] = 0x03;			/* Register address */
	buf[1] = 0x83;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* Master 11.2896 MHz */
	buf[0] = 0x04;			/* Register address */
	buf[1] = master ? 0x41 : 0x45;	/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* Master clock input enable */
	buf[0] = 0x01;			/* Register address */
	buf[1] = 0x00;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* Must wait for the above commad to take effect */
	udelay(20);

	/* PLL mode on, Master clk enable */
	buf[0] = 0x01;			/* Register address */
	buf[1] = 0x20;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* Must wait for the above commad to take effect */
	udelay(4);

	/* Master clk output enable */
	buf[0] = 0x04;			/* Register address */
	buf[1] = master ? 0x49 : 0x4d;	/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* Sampling freq */
	buf[0] = 0x05;			/* Register address */
	switch (rate) {
	case 44100:
		buf[1] = 0x00;			/* 44.1 KHz */
		break;
	case 22050:
		buf[1] = 0x20;			/* 22.05 KHz */
		break;
	case 11025:
		buf[1] = 0x40;			/* 11.025 KHz */
		break;
	case 48000:
		buf[1] = 0x60;			/* 48 KHz */
		break;
	case 32000:
		buf[1] = 0x80;			/* 32 KHz */
		break;
	case 24000:
		buf[1] = 0xa0;			/* 24 KHz */
		break;
	case 16000:
		buf[1] = 0xc0;			/* 16 KHz */
		break;
	case 8000:
		buf[1] = 0xe0;			/* 8 KHz */
		break;
	}

	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* ALC1 on ADC */
	buf[0] = 0x07;			/* Register address */
	buf[1] = 0x05;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* ALC1 enable */
	buf[0] = 0x09;			/* Register address */
	buf[1] = 0x20;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* ADC Lch on, IPGA Lch on, VCOM on */
	buf[0] = 0x00;			/* Register address */
	buf[1] = 0x85;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* ADC Rch, IPGA Rch, LIN2/RIN2  on */
	buf[0] = 0x10;			/* Register address */
	buf[1] = 0x1d;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* DAC, PLL, Master clk input on */
	buf[0] = 0x01;			/* Register address */
	buf[1] = 0x21;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* HPR, HPL, DAC to headphone amp on */
	buf[0] = 0x03;			/* Register address */
	buf[1] = 0x80;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* DAC, HPR, HPL, master clk input on */
	buf[0] = 0x01;			/* Register address */
	buf[1] = 0x27;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* Must wait for the above commad to take effect */
	udelay(100);

	/* Mute off */
	buf[0] = 0x0c;			/* Register address */
	buf[1] = 0x00;			/* Register value */
	buf[2] = 0x00;			/* Next consucutive register value */
	msg.len = 3;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	return 0;

fail:
	printk(KERN_ERR "i2c_sh7343_xfer_mod_poll() failed err=%d\n", rc);
	return rc;
}


int codec_stop(void)
{
	struct i2c_msg	msg;
	int		rc = 0;
	u_int8_t	buf[3];

	msg.addr = AK4537_I2C_ADDR;
	msg.flags = 0;
	msg.buf = buf;

	/* Mute on */
	buf[0] = 0x0c;			/* Register address */
	buf[1] = 0xff;			/* Register value */
	buf[2] = 0xff;			/* Next consecutive register value */
	msg.len = 3;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* PMHPL, PMHPR off */
	buf[0] = 0x01;			/* Register address */
	buf[1] = 0x01;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* Must wait for the above commad to take effect */
	udelay(200);

	/* HPL, HPR off */
	buf[0] = 0x03;			/* Register address */
	buf[1] = 0x83;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* PMDAC off */
	buf[0] = 0x01;			/* Register address */
	buf[1] = 0x00;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* PMMICL, PMADL off */
	buf[0] = 0x00;			/* Register address */
	buf[1] = 0x80;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* PMMICR, PMADR off */
	buf[0] = 0x10;			/* Register address */
	buf[1] = 0x00;			/* Register value */
	msg.len = 2;
	if ((rc = i2c_sh7343_xfer_mod_poll(I2C_CH, &msg, 1)) != 1)
		goto fail;

	/* Reset the codec (reset is active low) */
	OUT16(GENERAL_PURPOSE_REG, IN16(GENERAL_PURPOSE_REG) & ~CODEC_RESET);

	return 0;

fail:
	printk(KERN_ERR "i2c_sh7343_xfer_mod_poll() failed err=%d\n", rc);
	return rc;
}
