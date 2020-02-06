/*
 * wm8750_codec.c   audio codec driver
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
/* References:
 *   [1] linux/sound/soc/codecs/wm8750.c
 *
 **/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <sound/soc.h>
#include "../soc/codecs/wm8750.h"

typedef enum {
        BCM_DISABLED = 0x0000,
        BCM_MCLK4    = 0x0080,
        BCM_MCLK8    = 0x0100,
        BCM_MCLK16   = 0x0180,
} bcm_e;

static DEFINE_MUTEX(io_mutex);
static struct i2c_client *wm8750_client = NULL;
static int sysclk = 12000000;

/* defaults for gg board -> can also be set by mixer settings */
static int playback_invert_switch    = 1;
static int headphone_switch_polarity = 0;
static int headphone_switch_enable   = 1;

/* defaults for gg board -> are board specific and cant be changed by mixer settings */
static int mic_power = 0x32;
/* We have a 24 MHz MCLK which is divided by 2. The SH7723 requires
 * a BCLK < 3.3 MHz. For 48kHz/16bit stereo playback we need at least 1.53Mhz,
 * therefore we require a division by 4 */
static bcm_e bcm_mode = BCM_MCLK4;

static int wm8750_powersave = 1;
static int wm8750_playback;

/* ********** codec I/O, see [1] **********  */

/*
 * wm8750 register cache (default values after reset)
 * We can't read the WM8750 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static u16 wm8750_reg_cache[] = {
	0x0097, 0x0097, 0x0079, 0x0079,  /*  0 */
	0x0000, 0x0008, 0x0000, 0x000a,  /*  4 */
	0x0000, 0x0000, 0x00ff, 0x00ff,  /*  8 */
	0x000f, 0x000f, 0x0000, 0x0000,  /* 12 */
	0x0000, 0x007b, 0x0000, 0x0032,  /* 16 */
	0x0000, 0x00c3, 0x00c3, 0x00c0,  /* 20 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 24 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 28 */
	0x0000, 0x0000, 0x0050, 0x0050,  /* 32 */
	0x0050, 0x0050, 0x0050, 0x0050,  /* 36 */
	0x0079, 0x0079, 0x0079,          /* 40 */
};

static inline unsigned int wm8750_read_reg_cache(unsigned int reg)
{
	if (reg >= ARRAY_SIZE(wm8750_reg_cache))
		return -1;
	return wm8750_reg_cache[reg];
}

static inline void wm8750_write_reg_cache(u8 reg, u16 value)
{
	if (reg >= ARRAY_SIZE(wm8750_reg_cache))
		return;
	wm8750_reg_cache[reg] = value;
}

static int wm8750_write(unsigned int reg,unsigned int value)
{
	struct i2c_msg msg;
	u8 data[2];

	/* WM8750 has 7 bit address and 9 bits of data
	 * so we need to switch one data bit into reg and rest
	 * of data into val
	 */

	if (reg < 0 || reg > 0x2A) {
		printk(KERN_WARNING "%s Invalid register R%d\n", __func__, reg);
		return -1;
	}

	data[0] = (reg << 1) | (value >> 8 & 0x01);
	data[1] = value & 0xff;

	wm8750_write_reg_cache(reg, value);

	msg.flags = 0;
	msg.buf   = data;
        msg.len   = 2;
	msg.addr  = wm8750_client->addr;
	return i2c_transfer(wm8750_client->adapter, &msg, 1);
}

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:5;
	u8 usb:1;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12288000, 8000, 1536, 0x6, 0x0},
	{11289600, 8000, 1408, 0x16, 0x0},
	{18432000, 8000, 2304, 0x7, 0x0},
	{16934400, 8000, 2112, 0x17, 0x0},
	{12000000, 8000, 1500, 0x6, 0x1},

	/* 11.025k */
	{11289600, 11025, 1024, 0x18, 0x0},
	{16934400, 11025, 1536, 0x19, 0x0},
	{12000000, 11025, 1088, 0x19, 0x1},

	/* 16k */
	{12288000, 16000, 768, 0xa, 0x0},
	{18432000, 16000, 1152, 0xb, 0x0},
	{12000000, 16000, 750, 0xa, 0x1},

	/* 22.05k */
	{11289600, 22050, 512, 0x1a, 0x0},
	{16934400, 22050, 768, 0x1b, 0x0},
	{12000000, 22050, 544, 0x1b, 0x1},

	/* 32k */
	{12288000, 32000, 384, 0xc, 0x0},
	{18432000, 32000, 576, 0xd, 0x0},
	{12000000, 32000, 375, 0xa, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x10, 0x0},
	{16934400, 44100, 384, 0x11, 0x0},
	{12000000, 44100, 272, 0x11, 0x1},

	/* 48k */
	{12288000, 48000, 256, 0x0, 0x0},
	{18432000, 48000, 384, 0x1, 0x0},
	{12000000, 48000, 250, 0x0, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0x1e, 0x0},
	{16934400, 88200, 192, 0x1f, 0x0},
	{12000000, 88200, 136, 0x1f, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0xe, 0x0},
	{18432000, 96000, 192, 0xf, 0x0},
	{12000000, 96000, 125, 0xe, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	printk(KERN_ERR "wm8750: could not get coeff for mclk %d @ rate %d\n",
		mclk, rate);
	return -EINVAL;
}


static int wm8750_hw_params(snd_pcm_format_t format, int rate)
{
	u16 iface = wm8750_read_reg_cache(WM8750_IFACE) & 0x1f3;
	u16 srate = wm8750_read_reg_cache(WM8750_SRATE) & 0x1c0;
	int coeff = get_coeff(sysclk, rate);

	/* bit size */
	switch (format) {
            case SNDRV_PCM_FORMAT_S16_LE:
		break;
            case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
            case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
            case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x000c;
		break;
            default:
                pr_err("%s: Format not supported %x\n", __FUNCTION__, format);
                return -EINVAL;
	}

	/* set iface & srate */
        wm8750_write(WM8750_IFACE, iface);

        srate |= bcm_mode;

	if (coeff >= 0)
		wm8750_write(WM8750_SRATE, srate |
                             (coeff_div[coeff].sr << 1) | coeff_div[coeff].usb);

	return 0;
}

static int wm8750_set_bias_level(enum snd_soc_bias_level level)
{
	u16 pwr_reg = wm8750_read_reg_cache(WM8750_PWR1) & 0xfe30;

	switch (level) {
            case SND_SOC_BIAS_ON:
		/* set vmid to 50k and unmute dac */
                if (!wm8750_playback)
                        pwr_reg |= mic_power;
                pwr_reg |= 0xcc;
		break;
            case SND_SOC_BIAS_PREPARE:
		/* set vmid to 5k for quick power up */
                pwr_reg |= 0x1c1;
		break;
            case SND_SOC_BIAS_STANDBY:
		/* mute dac and set vmid to 500k, enable VREF */
		pwr_reg |= 0x0141;
		break;
            case SND_SOC_BIAS_OFF:
                pwr_reg |= 0x0001;
		break;
	}
        wm8750_write(WM8750_PWR1, pwr_reg);

	return 0;
}

static void wm8750_mute(int mute)
{
	u16 mute_reg = wm8750_read_reg_cache(WM8750_ADCDAC) & 0xfff7;

        wm8750_write(WM8750_ADCDAC, mute ? (mute_reg | 0x8) : mute_reg);
}

static int wm8750_set_dai_fmt(unsigned int fmt)
{
	u16 iface = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
            case SND_SOC_DAIFMT_CBM_CFM:
		iface = 0x0040;
		break;
            case SND_SOC_DAIFMT_CBS_CFS:
		break;
            default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
            case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
            case SND_SOC_DAIFMT_RIGHT_J:
		break;
            case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
            case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
            case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
            default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
            case SND_SOC_DAIFMT_NB_NF:
		break;
            case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
            case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
            case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
            default:
		return -EINVAL;
	}

	wm8750_write(WM8750_IFACE, iface);
	return 0;
}

/*
 * initialise the WM8750 driver
 * register the mixer and dsp interfaces with the kernel
 */
static void wm8750_codec_init(void)
{
        u16 reg;

	wm8750_write(WM8750_RESET, 0);

        wm8750_mute(1);

	/* update gains simultanously */
	reg = wm8750_read_reg_cache(WM8750_LDAC);
	wm8750_write(WM8750_LDAC, reg | 0x0100);
	reg = wm8750_read_reg_cache(WM8750_RDAC);
	wm8750_write(WM8750_RDAC, reg | 0x0100);

	/* update left and right channel gains */
	reg = wm8750_read_reg_cache(WM8750_LOUT1V);
	wm8750_write(WM8750_LOUT1V, reg | 0x0100);
	reg = wm8750_read_reg_cache(WM8750_ROUT1V);
	wm8750_write(WM8750_ROUT1V, reg | 0x0100);
	reg = wm8750_read_reg_cache(WM8750_LOUT2V);
	wm8750_write(WM8750_LOUT2V, reg | 0x0100);
	reg = wm8750_read_reg_cache(WM8750_ROUT2V);
	wm8750_write(WM8750_ROUT2V, reg | 0x0100);
	reg = wm8750_read_reg_cache(WM8750_LINVOL);
	wm8750_write(WM8750_LINVOL, reg | 0x0100);
	reg = wm8750_read_reg_cache(WM8750_RINVOL);
	wm8750_write(WM8750_RINVOL, reg | 0x0100);

        /* enable thermal shutdown */
	reg = wm8750_read_reg_cache(WM8750_ADCTL1);
	wm8750_write(WM8750_ADCTL1, reg | 0x0100);

	reg = wm8750_read_reg_cache(WM8750_ADCTL2);
        reg &= ~0x70;
        if (playback_invert_switch)
                reg |= 0x10;
        if (headphone_switch_polarity)
                reg |= 0x20;
        if (headphone_switch_enable)
                reg |= 0x40;
        wm8750_write(WM8750_ADCTL2, reg);

        /* mclock divide by 2 (24Mhz->12MHz) */
        reg = wm8750_read_reg_cache(WM8750_SRATE);
        wm8750_write(WM8750_SRATE, reg | 0x40);

	wm8750_set_bias_level(SND_SOC_BIAS_OFF);
}

/*
 * initialise the WM8750 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int wm8750_codec_start(int siu_is_master, int rate)
{
	mutex_lock(&io_mutex);

        /* default format */
        if (wm8750_hw_params(SNDRV_PCM_FORMAT_S16_LE, rate) < 0)
                return -EINVAL;

        /* precharge condensators to reduce cklicking noise */
	wm8750_set_bias_level(SND_SOC_BIAS_PREPARE);
        msleep(1); /* no numbers available */

        wm8750_set_dai_fmt((siu_is_master?SND_SOC_DAIFMT_CBS_CFS:SND_SOC_DAIFMT_CBM_CFM)|
                           SND_SOC_DAIFMT_I2S);

        /* enable power */
	wm8750_set_bias_level(SND_SOC_BIAS_ON);
        if (wm8750_playback)
                wm8750_write(WM8750_PWR2, 0x1FA); /* enable DAC_LEFT, DAC_RIGHT, LOUT1,
                                                   * ROUT1, LOUT2, ROUT2, OUT3*/
        /* Left DAC to left mixer, right DAC to right mixer */
        wm8750_write(WM8750_LOUTM1, 0x100);
        wm8750_write(WM8750_ROUTM2, 0x100);

        wm8750_mute(0);

 	mutex_unlock(&io_mutex);

        return 0;
}

/*
 * initialise the WM8750 driver
 * register the mixer and dsp interfaces with the kernel
 */
static void wm8750_codec_stop(void)
{
	mutex_lock(&io_mutex);

        wm8750_mute(1);

        /* disable mixer */
        wm8750_write(WM8750_LOUTM1, 0x00);
        wm8750_write(WM8750_ROUTM2, 0x00);

        /* disable power */
        wm8750_write(WM8750_PWR2, 0x0);
        /* powersave provides a small clicking noise on opening the device. With
         * powersave off, this is reduced to the first opening at all */
	wm8750_set_bias_level(wm8750_powersave ? SND_SOC_BIAS_OFF : SND_SOC_BIAS_ON);

 	mutex_unlock(&io_mutex);
}

/* ********** DebugFS ********** */

#if defined(CONFIG_DEBUG_FS)
static int wm8750_debugfs_seq_show(struct seq_file *file,
                                        void *iter)
{
        int reg;
        for (reg=0; reg < ARRAY_SIZE(wm8750_reg_cache); reg++)
                seq_printf(file, "0x%02x=0x%03x\n",
                           reg,
                           wm8750_reg_cache[reg]);
	return 0;
}

static int wm8750_debugfs_open(struct inode *inode, struct file *file)
{
        return single_open(file, wm8750_debugfs_seq_show, inode->i_private);
}

static const struct file_operations wm8750_debugfs_fops = {
	.owner		= THIS_MODULE,
	.open		= wm8750_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int wm8750_debugfs_init(void)
{
	struct dentry *dentry;

	dentry = debugfs_create_file("wm8750_regs", S_IRUSR, NULL,
                                     NULL,
                                     &wm8750_debugfs_fops);

	if (!dentry)
		return -ENOMEM;
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

        return 0;
}
#endif  /* CONFIG_DEBUG_FS */

/* ********** I2C initialization ********** */

static int __devinit wm8750_probe(
        struct i2c_client *client,
        const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter,
			I2C_FUNC_I2C | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -EIO;

	dev_info(&client->dev, "chip found, SIU driver\n");

        wm8750_client = client;

#ifdef CONFIG_DEBUG_FS
        wm8750_debugfs_init();
#endif

	/* Initialize codec */
        wm8750_codec_init();

        return 0;
}

static const struct i2c_device_id wm8750_ids[] = {
	{ "wm8750", 0 },
	{ }
};

static struct i2c_driver wm8750_driver = {
	.driver		= {
		.name	= "wm8750",
                .owner  = THIS_MODULE,
	},
	.id_table	= wm8750_ids,
        .probe          = wm8750_probe,
};

static int __init wm8750_init(void)
{
	return i2c_add_driver(&wm8750_driver);
}
module_init(wm8750_init);

static void __exit wm8750_exit(void)
{
	i2c_del_driver(&wm8750_driver);
}
module_exit(wm8750_exit);

MODULE_DESCRIPTION("WM8750 (SIU) codec driver");
MODULE_LICENSE("GPL");

/* ********** interface to siu_sh7343.c ********** */

int codec_start(int master, u_int32_t rate, uint32_t channels, int playback)
{
        if (wm8750_client == NULL)
                /* not initialized by platform driver */
                return -ENODEV;

        wm8750_playback = playback;

        return wm8750_codec_start(master, rate);
}

int codec_stop(void)
{
        if (wm8750_client == NULL)
                /* not initialized by platform driver */
                return -ENODEV;

        wm8750_codec_stop();

        return 0;
}

#include "wm8750_codec_controls.c"

int codec_register_controls(struct snd_card *card)
{
        int i;
        int err;

        for (i = 0; i < ARRAY_SIZE(wm8750_snd_controls); i++) {
                struct snd_kcontrol *kctrl = snd_ctl_new1(&wm8750_snd_controls[i], 0);
                if ((err = snd_ctl_add(card, kctrl)) < 0) {
                        pr_err("snd_ctl_add() failed to add %s control err=%d\n", wm8750_snd_controls[i].name, err);
                        goto error;
                }
        }

        return 0;

error:
        return err;
}
