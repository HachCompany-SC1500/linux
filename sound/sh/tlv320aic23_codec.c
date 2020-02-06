/*
 * linux/sound/sh/tlv320aic23_codec.c
 *
 * Copyright (c) 2009 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 * Description: Quick&Dirty move from sound/soc/codecs/tlv320aic23.c to
 *              sound/sh. Will be replaced as soon as  digital sound driver
 *              using SIU is implemented.
 *              o Driver is not to be meant to be detached.
 *              o provides debugfs/tlv320aic23 with register contents
 * References:
 *       [1] file://R:/8_renesas/HiCO.DIMM/HiCO.DIMM-Base/HW/Baugruppen/Source/Info/datasheets/ICs/tlv320aic23b_edi1469.pdf
 *       [2] linux/sound/soc/codecs/tlv320aic23.c
 *
 **/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <sound/soc.h>
#include "../soc/codecs/tlv320aic23.h"

static struct i2c_client *tlv320aic23_client = NULL;

static int microphone_boost = 1;
static int input_microphone = 0;

/* ********** codec I/O, see [2] **********  */

static u16 tlv320aic23_reg_cache[] = {
	0x0097, 0x0097, 0x00F9, 0x00F9,	/* 0 */
	0x000A, 0x0008, 0x0007, 0x0001,	/* 4 */
	0x0020, 0x0000, 0x0000, 0x0000,	/* 8 */
	0x0000, 0x0000, 0x0000, 0x0000,	/* 12 */
};

static inline unsigned int tlv320aic23_read_reg_cache(unsigned int reg)
{
	if (reg >= ARRAY_SIZE(tlv320aic23_reg_cache))
		return -1;
	return tlv320aic23_reg_cache[reg];
}

static inline void tlv320aic23_write_reg_cache(u8 reg, u16 value)
{
	if (reg >= ARRAY_SIZE(tlv320aic23_reg_cache))
		return;
	tlv320aic23_reg_cache[reg] = value;
}

static int tlv320aic23_write(unsigned int reg,unsigned int value)
{
	struct i2c_msg msg;
	u8 data[2];

	/* TLV320AIC23 has 7 bit address and 9 bits of data
	 * so we need to switch one data bit into reg and rest
	 * of data into val
	 */

	if ((reg < 0 || reg > 9) && (reg != 15)) {
		printk(KERN_WARNING "%s Invalid register R%d\n", __func__, reg);
		return -1;
	}

	data[0] = (reg << 1) | (value >> 8 & 0x01);
	data[1] = value & 0xff;

	tlv320aic23_write_reg_cache(reg, value);

	msg.flags = 0;
	msg.buf   = data;
        msg.len   = 2;
	msg.addr  = tlv320aic23_client->addr;
	return i2c_transfer(tlv320aic23_client->adapter, &msg, 1);
}

static int tlv320aic23_hw_params(snd_pcm_format_t format)
{
	u16 iface_reg;

	iface_reg = tlv320aic23_read_reg_cache(TLV320AIC23_DIGT_FMT) & ~(0x03 << 2);

	switch (format) {
            case SNDRV_PCM_FORMAT_S16_LE:
		break;
            case SNDRV_PCM_FORMAT_S20_3LE:
		iface_reg |= (0x01 << 2);
		break;
            case SNDRV_PCM_FORMAT_S24_LE:
		iface_reg |= (0x02 << 2);
		break;
            case SNDRV_PCM_FORMAT_S32_LE:
		iface_reg |= (0x03 << 2);
		break;
	}
	tlv320aic23_write(TLV320AIC23_DIGT_FMT, iface_reg);

	return 0;
}

static int tlv320aic23_set_dai_fmt(unsigned int fmt)
{
	u16 iface_reg;

	iface_reg = tlv320aic23_read_reg_cache(TLV320AIC23_DIGT_FMT) & (~0x03);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
            case SND_SOC_DAIFMT_CBM_CFM:
		iface_reg |= TLV320AIC23_MS_MASTER;
		break;
            case SND_SOC_DAIFMT_CBS_CFS:
		break;
            default:
                goto error;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
            case SND_SOC_DAIFMT_I2S:
		iface_reg |= TLV320AIC23_FOR_I2S;
		break;
            case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= TLV320AIC23_FOR_DSP;
		break;
            case SND_SOC_DAIFMT_RIGHT_J:
		break;
            case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= TLV320AIC23_FOR_LJUST;
		break;
            default:
		goto error;
	}

	tlv320aic23_write(TLV320AIC23_DIGT_FMT, iface_reg);

	return 0;

error:
        printk( KERN_ERR "%s: Invalid DAI Format 0x%02x\n", __FUNCTION__, fmt);
        return -EINVAL;
}

static void tlv320aic23_analog_update( void )
{
        u16 reg;

	reg = tlv320aic23_read_reg_cache(TLV320AIC23_ANLG);
        reg &= ~(TLV320AIC23_BYPASS_ON |
                 TLV320AIC23_INSEL_MIC |
                 TLV320AIC23_MICM_MUTED |
                 TLV320AIC23_MICB_20DB);

        reg |= TLV320AIC23_DAC_SELECTED;
        if (microphone_boost)
                reg |= TLV320AIC23_MICB_20DB;
        if (input_microphone)
                reg |= TLV320AIC23_INSEL_MIC;

	tlv320aic23_write(TLV320AIC23_ANLG,reg);
}

/*
 * initialise the AIC23 driver
 * register the mixer and dsp interfaces with the kernel
 */
static void tlv320aic23_codec_init(int siu_is_master)
{
        u16 reg;

	/* power on device */
        tlv320aic23_write(TLV320AIC23_PWR, 0x0);

	tlv320aic23_write(TLV320AIC23_DIGT, TLV320AIC23_DEEMP_44K);

	/* Unmute input */
	reg = tlv320aic23_read_reg_cache(TLV320AIC23_LINVOL);
	tlv320aic23_write(TLV320AIC23_LINVOL,
			  (reg & (~TLV320AIC23_LIM_MUTED)) |
                          TLV320AIC23_LIV_MAX |
			  TLV320AIC23_LRS_ENABLED);

	reg = tlv320aic23_read_reg_cache(TLV320AIC23_RINVOL);
	tlv320aic23_write(TLV320AIC23_RINVOL,
			  (reg & (~TLV320AIC23_LIM_MUTED)) |
                          TLV320AIC23_LIV_MAX |
			  TLV320AIC23_LRS_ENABLED);

        tlv320aic23_analog_update();

	/* Default output volume */
	tlv320aic23_write(TLV320AIC23_LCHNVOL,
			  TLV320AIC23_DEFAULT_OUT_VOL &
			  TLV320AIC23_OUT_VOL_MASK);
	tlv320aic23_write(TLV320AIC23_RCHNVOL,
			  TLV320AIC23_DEFAULT_OUT_VOL &
			  TLV320AIC23_OUT_VOL_MASK);

	tlv320aic23_write(TLV320AIC23_ACTIVE, 0x1);

        /* default format */
        tlv320aic23_hw_params(SNDRV_PCM_FORMAT_S16_LE);
        tlv320aic23_set_dai_fmt((siu_is_master?SND_SOC_DAIFMT_CBS_CFS:SND_SOC_DAIFMT_CBM_CFM)|
                                SND_SOC_DAIFMT_I2S);

        /* swap left/right because the siu_sh7343 converts it this way*/
	tlv320aic23_write(TLV320AIC23_DIGT_FMT,
                          tlv320aic23_read_reg_cache(TLV320AIC23_DIGT_FMT) | TLV320AIC23_LRSWAP_ON);

}

int tlv320aic23_codec_set_rate(u_int32_t rate)
{
        struct tlv320aic23_srate_reg_info {
                u32 sample_rate;
                u8 control;		/* SR3, SR2, SR1, SR0 and BOSR */
                u8 divider;		/* if 0 CLKIN = MCLK, if 1 CLKIN = MCLK/2 */
        };

        /* see 3.3.2.1.*/
        static const struct tlv320aic23_srate_reg_info srate_reg_info[] = {
                {4000, 0x06, 1},
                {8000, 0x06, 0},
                {16000, 0x0C, 1},
                {22050, 0x11, 1},
                {24000, 0x00, 1},
                {32000, 0x0C, 0},
                {44100, 0x11, 0},
                {48000, 0x00, 0},
                {88200, 0x1F, 0},
                {96000, 0x0E, 0},
        };
        u16 data;
        int count = 0;

	/* Search for the right sample rate */
	while ((count < ARRAY_SIZE(srate_reg_info))) {
                if (srate_reg_info[count].sample_rate == rate) {
                        data =  (srate_reg_info[count].divider << TLV320AIC23_CLKIN_SHIFT) |
                                (srate_reg_info[count].control << TLV320AIC23_BOSR_SHIFT) |
                                TLV320AIC23_USB_CLK_ON;
                        return tlv320aic23_write(TLV320AIC23_SRATE, data);
                }
                count++;
	}

        /* not found */
        return -EINVAL;
}

/* ********** DebugFS ********** */

#if defined(CONFIG_DEBUG_FS)
static int tlv320aic23_debugfs_seq_show(struct seq_file *file,
                                        void *iter)
{
        int reg;
        for (reg=0; reg < ARRAY_SIZE(tlv320aic23_reg_cache); reg++)
                seq_printf(file, "0x%02x=0x%03x\n",
                           reg,
                           tlv320aic23_reg_cache[reg]);
	return 0;
}

static int tlv320aic23_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, tlv320aic23_debugfs_seq_show, inode->i_private);
}

static const struct file_operations tlv320aic23_debugfs_fops = {
	.owner		= THIS_MODULE,
	.open		= tlv320aic23_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int tlv320aic23_debugfs_init(void)
{
	struct dentry *dentry;

	dentry = debugfs_create_file("tlv320aic_regs", S_IRUSR, NULL,
                                     NULL,
                                     &tlv320aic23_debugfs_fops);

	if (!dentry)
		return -ENOMEM;
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

        return 0;
}
#endif  /* CONFIG_DEBUG_FS */

/* ********** I2C initialization ********** */

static int __devinit tlv320aic23_probe(
        struct i2c_client *client,
        const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter,
			I2C_FUNC_I2C | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -EIO;

	dev_info(&client->dev, "chip found, SIU driver\n");

        tlv320aic23_client = client;

#ifdef CONFIG_DEBUG_FS
        tlv320aic23_debugfs_init();
#endif

	/* Reset codec */
	tlv320aic23_write(TLV320AIC23_RESET, 0); // produces a clicking sound

        return 0;
}

static const struct i2c_device_id tlv320aic23_ids[] = {
	{ "tlv320aic23", 0 },
	{ }
};

static struct i2c_driver tlv320aic23_driver = {
	.driver		= {
		.name	= "tlv320aic23",
                .owner  = THIS_MODULE,
	},
	.id_table	= tlv320aic23_ids,
        .probe          = tlv320aic23_probe,
};

static int __init tlv320aic23_init(void)
{
	return i2c_add_driver(&tlv320aic23_driver);
}
module_init(tlv320aic23_init);

static void __exit tlv320aic23_exit(void)
{
	i2c_del_driver(&tlv320aic23_driver);
}
module_exit(tlv320aic23_exit);

MODULE_DESCRIPTION("TLV320AIC23 (SIU) codec driver");
MODULE_LICENSE("GPL");

/* ********** interface to siu_sh7343.c ********** */
int codec_start(int master, u_int32_t rate, uint32_t channels, int playback)
{
        if (tlv320aic23_client == NULL)
                /* not initialized by platform driver */
                return -ENODEV;

        tlv320aic23_codec_init(master);
        tlv320aic23_codec_set_rate(rate);

        return 0;
}

int codec_stop(void)
{
        if (tlv320aic23_client == NULL)
                /* not initialized by platform driver */
                return -ENODEV;

        tlv320aic23_write(TLV320AIC23_PWR, TLV320AIC23_DEVICE_PWR_OFF);
        return 0;
}

static int tlv320aic23_info_microphone_boost(struct snd_kcontrol *kctrl,
                                             struct snd_ctl_elem_info *uinfo)
{
        static const char *texts[] = {
                "off", "20dB"
        };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = ARRAY_SIZE(texts);

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item = uinfo->value.enumerated.items - 1;
	strcpy(uinfo->value.enumerated.name, texts[uinfo->value.enumerated.item]);

	return 0;
}

static int tlv320aic23_get_microphone_boost(struct snd_kcontrol *kctrl,
                                            struct snd_ctl_elem_value *ucontrol)
{
        ucontrol->value.integer.value[0] = microphone_boost;

        return 0;
}

static int tlv320aic23_put_microphone_boost(struct snd_kcontrol *kctrl,
                                            struct snd_ctl_elem_value *ucontrol)
{
        unsigned int val = ucontrol->value.integer.value[0];

        if (val != microphone_boost) {
                microphone_boost = val;

                tlv320aic23_analog_update();
        }

        return 0;
}

static int tlv320aic23_info_input_selection(struct snd_kcontrol *kctrl,
                                            struct snd_ctl_elem_info *uinfo)
{
        static const char *texts[] = {
                "Line", "Mic"
        };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = ARRAY_SIZE(texts);

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item = uinfo->value.enumerated.items - 1;
	strcpy(uinfo->value.enumerated.name, texts[uinfo->value.enumerated.item]);

	return 0;
}

static int tlv320aic23_get_input_selection(struct snd_kcontrol *kctrl,
                                           struct snd_ctl_elem_value *ucontrol)
{
        ucontrol->value.enumerated.item[0] = input_microphone;

        return 0;
}

static int tlv320aic23_put_input_selection(struct snd_kcontrol *kctrl,
                                           struct snd_ctl_elem_value *ucontrol)
{
        unsigned int val = ucontrol->value.enumerated.item[0];

        if (val > 1)
                return -EINVAL;

        if (val != input_microphone) {
                input_microphone = val;

                tlv320aic23_analog_update();
        }

        return 0;
}

static struct snd_kcontrol_new microphone_boost_control = {
	.iface		=	SNDRV_CTL_ELEM_IFACE_MIXER,
	.name		=	"Mic boost",
	.index		=	0,
	.info		=	tlv320aic23_info_microphone_boost,
	.get		=	tlv320aic23_get_microphone_boost,
	.put		=	tlv320aic23_put_microphone_boost,
};

static struct snd_kcontrol_new input_microphone_control = {
	.iface		=	SNDRV_CTL_ELEM_IFACE_MIXER,
	.name		=	"Input Selection",
	.index		=	0,
	.info		=	tlv320aic23_info_input_selection,
	.get		=	tlv320aic23_get_input_selection,
	.put		=	tlv320aic23_put_input_selection,
};

int codec_register_controls(struct snd_card *card)
{
	struct snd_kcontrol *kctrl;
        int err;

	kctrl = snd_ctl_new1(&input_microphone_control, 0);
	if ((err = snd_ctl_add(card, kctrl)) < 0) {
		printk(KERN_ERR "snd_ctl_add() failed to add input_microphone "
		       "controls err=%d\n", err);
		goto fail;
	}

	kctrl = snd_ctl_new1(&microphone_boost_control, 0);
	if ((err = snd_ctl_add(card, kctrl)) < 0) {
		printk(KERN_ERR "snd_ctl_add() failed to add microphone boost "
		       "controls err=%d\n", err);
		goto fail;
	}

        return 0;

fail:
        return err;
}
