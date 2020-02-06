/*
    wm8978_codec.c - Driver for Wolfson's WM8978 audio codec.

    Copyright (c) 2007 Carlos Munoz <carlos@kenati.com>

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

#define WM8978_I2C_ADDR		0x001A

#if defined(CONFIG_CPU_SUBTYPE_SH7722)
# define I2C_CH			0
#elif defined(CONFIG_CPU_SUBTYPE_SH7354)
# define I2C_CH			0
#elif defined(CONFIG_CPU_SUBTYPE_SH7343)
# define I2C_CH			1
#else
# error Must set wm8978 board specific values
#endif

extern int i2c_sh7343_xfer_mod_poll(int			ch,
				    struct i2c_msg	*msgs,
				    int			num);


/*
 * reg_write:
 *
 *  reg:		Register offset to write to.
 *
 *  val:		Value to write to the register.
 *
 *  The wm8978 is a little different than most other devices in that its
 *  registers are 9 bits wide. To accommodate this, a bit from the register 
 *  offset is used. This poses no problem since 2 register offset bits are
 *  never used (largest register offset is 0x39 leaving 2 bits unused). The 
 *  register offset is shifted to the left one bit, and that freed bit is use
 *  for the ninth data bit. The devices knows to divide the register offset by
 *  2 on write commands.
 */
static void reg_write(u8	reg,
		      u16	val)
{
        struct i2c_msg  	msg;
        u8	 		buf[2];
	struct i2c_adapter	*adapter;

        msg.addr = WM8978_I2C_ADDR;
        msg.flags = 0;
        msg.buf = buf;

	/* Use LSB bit for ninth data bit */
        buf[0] = reg << 1;
	buf[0] |= (val & 0x0100) >> 8;

	/* Write remaining 8 data bits */
        buf[1] = val & 0xff;

        msg.len = 2;
	adapter = i2c_get_adapter(0);
	i2c_transfer(adapter, &msg, 1);
}


/*
 * codec_start:		Initialize and start the codec.
 *
 *  master:		Not used. Device always in slave mode. Kept for
 *			compiling reasons.
 *
 *  rate:		Audio sampling rate.
 */
int codec_start(int master, u_int32_t rate, uint32_t channels, int playback)
{
	/* Reset the codec */
        reg_write(0, 0x000);

	/* The PLL is used to generate the clocks used by each different 
	   sampling frequency (48000 Hz, 44100 Hz, etc). The input to the PLL
	   is the MCLK pin, which in the migo-r board is 13 MHz. This is a 
	   simplified version of the PLL circuit:

                                      _____________
                         r36[4]   f1  |           | f2  _______   _______ sysclk
	   MCLK-----------     -------| R = f2/f1 |-----| f/4 |---| f/n |-------
                 |            /       |           |     ------- | -------
                 | _______   /        -------------             |  r6[7:5]
                 --| f/2 |---                                   |
                   -------                                      |
                                                                |
                                      _______                   |
           GPIO1----------------------| f/n |-------------------|
                                      -------
                                      r8[5:4]

	   To calculate the PLL registers' values we start with R (PLL ratio).
	   The PLL ratio must be between 6 - 12 and it's most stable around 8.
	   Depending on r36[4], f1 can be 13 MHz or 13/2 = 6.5 MHZ. We can then
	   calculate the ranges for f2 = R * f1:

	                     |   R = 12   |    R = 8   |   R = 6
	       --------------------------------------------------------
	       f1 = 13 MHz   |  f2 = 156  |  f2 = 104  |  f2 = 78
	       f1 = 6.5 MHz  |  f2 = 78   |  f2 = 52   |  f2 = 39

	   Next we calculate sysclk. sysclk must be 256 times the sampling
	   frequency. It's used by the ADC and DAC. Also, GPIO1 must be equal to 
	   twice sysclk since the SIU divides it by 2 to generate its SIUCK 
	   (which must be 256 times the sampling frequency). Therefore, we
	   must select r36[4], r6[7:5], and r8[5:5] to divide MCLK correctly
	   for each sampling frequency.

	   So for a sampling frequency of 44100 Hz, sysclk = 44100 * 256 =
	   11.2896 MHz. We want R around 8 so f2 must be around 104 MHZ. r6[7:5]
	   must divide f2 by at least 2 so that GPIO1 can be twice sysclk. We 
	   choose r36[4] to divide by 1, r6[7:5] to divide by 2, and r8[5:4] to
	   divide by 1. Then:

	       f2 = sysclk * 2 * 4 = 90.3168 MHz
	       R = f2/f1 = 6.9475

	   From R we calculate PLLN and PLLK. PLLN (r36[3:0]) is the integer 
	   part of R (6). PLLK is equal to 2^24 (R - PLLN) => PLLK = 0x1000000
	   (.9475) => PLLK = 0xf28f5c. Since the registers are 9 bits wide 
	   r37 = 0x3c, r38 = 0x147, and r39 = 0x15c.
	*/
	switch(rate) {
	case 48000:
		reg_write(6, 0x140);
		reg_write(8, 0x004);
		reg_write(36, 0x007);
		reg_write(37, 0x023);
		reg_write(38, 0x1ea);
		reg_write(39, 0x166);
		reg_write(7, 0x000);
		break;
	default:
	case 44100:
		reg_write(6, 0x140);
		reg_write(8, 0x004);
		reg_write(36, 0x006);
		reg_write(37, 0x03c);
		reg_write(38, 0x147);
		reg_write(39, 0x15c);
		reg_write(7, 0x000);
		break;
	case 32000:
		reg_write(6, 0x180);
		reg_write(8, 0x014);
		reg_write(36, 0x00a);
		reg_write(37, 0x005);
		reg_write(38, 0x08f);
		reg_write(39, 0x0b8);
		reg_write(7, 0x002);
		break;
        case 24000:
		reg_write(6, 0x180);
		reg_write(8, 0x014);
		reg_write(36, 0x007);
		reg_write(37, 0x023);
		reg_write(38, 0x1ec);
		reg_write(39, 0x0ad);
		reg_write(7, 0x004);
		break;
	case 22050:
		reg_write(6, 0x180);
		reg_write(8, 0x014);
		reg_write(36, 0x006);
		reg_write(37, 0x03c);
		reg_write(38, 0x147);
		reg_write(39, 0x15c);
		reg_write(7, 0x004);
		break;
	case 16000:
		reg_write(6, 0x1a0);
		reg_write(8, 0x024);
		reg_write(36, 0x007);
		reg_write(37, 0x023);
		reg_write(38, 0x1ec);
		reg_write(39, 0x0ad);
		reg_write(7, 0x006);
		break;
	case 11025:
		reg_write(6, 0x180);
		reg_write(8, 0x014);
		reg_write(36, 0x016);
		reg_write(37, 0x03c);
		reg_write(38, 0x147);
		reg_write(39, 0x15c);
		reg_write(7, 0x008);
		break;
	case 8000:
		reg_write(6, 0x1a0);
		reg_write(8, 0x024);
		reg_write(36, 0x017);
		reg_write(37, 0x023);
		reg_write(38, 0x1ec);
		reg_write(39, 0x0ad);
		reg_write(7, 0x00a);
		break;
	}

	/* Power management */
	/* PLL on, analog amplifier bias */
	reg_write(1, 0x079);
	/* Out-1 enabled, left/right input channel enabled */
        reg_write(2, 0x1bf);

	/* Out-2 disabled, right/left output channel enabled, dac enabled */
	/* Set 03h[8], if stream is stereo */
	if (channels == 1)
        	reg_write(3, 0x00f);
	else
        	reg_write(3, 0x10f);

	/* Analog input */
	/* MIC bias = .9 avdd, left right channels connected to PGA */
        reg_write(44, 0x033);
	/* Left channel volume control */
        reg_write(45, 0x010);
	/* Right channel volume control */
        reg_write(46, 0x010);
	/* Update volume */
        reg_write(45, 0x150);
	reg_write(46, 0x150);
	/* No boost to left channel PGA */
        reg_write(47, 0x000);
	/* No boost to right channel PGA */
	reg_write(48, 0x000);
	/* ADC volume */
	reg_write(15, 0x0ff);
	reg_write(16, 0x0ff);
	/* Automatic Level Control (ALC) */
	/* ALC on L/R, max gain = +11.25 db, min gain = -12 db */
        reg_write(32, 0x198);
	/* Hold time = 0, target = -6 db */
        reg_write(33, 0x00b);
	/* ALC mode, decay = 26 ms/6 db, attack = 3.3 ms/6 db */
        reg_write(34, 0x033);

	/* Analog output */
	/* Left volume = 0db */
	reg_write(52, 0x139);
	/* Right volume = 0 db */
	reg_write(53, 0x139);
	/* Out-3 disabled */
        reg_write(56, 0x040);
	/* Out-4 muted */
	reg_write(57, 0x040);

	/* Digital audio */
	/* 16 bits, i2s, stereo*/
	reg_write(4, 0x090);
	/* 128x sample rate */
        reg_write(10, 0x008);

	return 0;
}

int codec_stop(void)
{
	/* Reset the codec */
        reg_write(0, 0x000);

	/* Power management. Turn everything off */
        reg_write(1, 0x000);
        reg_write(2, 0x000);
        reg_write(3, 0x000);

	return 0;
}
