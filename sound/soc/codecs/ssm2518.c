
/*
 * ALSA SoC SSM2518 codec driver
 *
 * Author:      Dr. Frank Reither, <frank.reither@emtrion.de>
 * Copyright:   (C) 2010 Emtrion GmbH,
 *
 * Based on sound/soc/codecs/wm8731.c by Richard Purdie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 *  The SSM2518 is a driver for a low power stereo audio
 *  codec ssm2518 from Analog Devices
 *
 *  The machine layer should disable unsupported inputs/outputs by
 *  snd_soc_dapm_disable_pin(codec, "LHPOUT"), etc.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>

#include "ssm2518.h"
#include <cpu/sh7723.h>

#define SSM2518_VERSION "0.0"
//#define DEBUG
#undef DEBUG

static void dump_codec_register(struct snd_soc_codec *codec);


/*
 * SSM2518 register cache
 */
static const u8 ssm2518_reg[] = {
	0x05, 0x00, 0x02, 0x00,	/* 0 */
	0x10, 0x40, 0x40, 0x81,	/* 4 */
	0x0c, 0x81, 0x3c, 0x00,	/* 8 */
	0x00, 0x88, 0x00, 0x00,	/* 12 */
	0x00, 0x00, 0x03, 0x00,	/* 16 */
};


static struct snd_soc_codec *ssm2518_codec;

typedef struct div_ssm2518 {
	int div;
	int val;
	u8 mcs_val;
} div_t;

/*
 * SSM2518 master clock divider
 */
static const div_t ssm2518_mcs[] = {
 {  64,  64, 0x00},
 { 128, 128, 0x02}, 
 { 256, 256, 0x04}, 
 { 384, 384, 0x06},
 { 512, 256, 0x04},
 { 768, 768, 0x0a},
 {1024, 256, 0x04},
 {1536, 384, 0x06},
 {2048, 512, 0x08},
 {3072, 786, 0x0a},
 {   0,   0},
};

/*
 * read ssm2518 register cache
 */
static inline unsigned int ssm2518_read_reg_cache(struct snd_soc_codec
						      *codec, u8 reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= ARRAY_SIZE(ssm2518_reg))
		return -1;
	return cache[reg];
}

/*
 * write ssm2518 register cache
 */
static inline void ssm2518_write_reg_cache(struct snd_soc_codec *codec,
					       u8 reg, u8 value)
{
	u8 *cache = codec->reg_cache;
	if (reg >= ARRAY_SIZE(ssm2518_reg))
		return;
	cache[reg] = value;
}

/*
 * write to the ssm2518 register space
 */
static int ssm2518_write(struct snd_soc_codec *codec, unsigned char reg,
			     unsigned char value)
{

	u8 data[2];

	/* SSM2518 has 8 bit address and 8 bits of data
	 */

	if (reg > 18) {
//		printk(KERN_WARNING "%s Invalid register R%u\n", __func__, reg);
		return -1;
	}

	data[0] = reg;
	data[1] = value;

	ssm2518_write_reg_cache(codec, reg, value);

	if (codec->hw_write(codec->control_data, data, 2) == 2) {
//printk(KERN_INFO "ssm2518_write: %x written to %x\n", data[1], data[0]);

// dump_codec_register(codec);
		return 0;
	}

	printk(KERN_ERR "%s cannot write %03x to register R%u\n", __func__, value, reg);

	return -EIO;
}

//static const char *rec_src_text[] = { "Line", "Mic" };
//static const char *deemph_text[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const char *on_mute_text[]  = { "on", "mute" };
static const char *on_off_text[]   = { "on", "off" };
static const char *pre_post_text[] = { "post", "pre" };
static const char *dis_en_text[]   = { "disable", "enable" };
static const char *drc_en_text[]   = { "disable", "left_en", "right_en", "enable" };
static const char *artime_text[]   = { "10ms", "20ms", "40ms", "80ms" };
static const char *zero_one_text[] = { "0", "1" };
static const char *asr_text[]      = {"automatic", "by FS register"};

static const char *drc_rms_tav_text[] = {"0ms", "1.5ms", "3ms", "6ms", "12ms", "24ms", "48ms", "96ms",
                                         "192ms", "384ms", "768ms", "1.536s", "3.072s", "6.144s", "12.288s", "24.57s"};

static const char *drc_hdt_text[] = {"0ms", "0.67ms", "1.33ms", "2.67ms", "5.33ms", "10.66ms", "21.32ms",
                                     "42.64ms", "85.28ms", "170.56ms", "341.12ms", "682.24ms", "1.354s" };

static const char *drc_drc_att_text[] = {"0ms", "0.1ms", "0.19ms", "0.37ms", "0.75ms", "1.5ms", "3ms", "6ms",
                                         "12ms", "24ms", "48ms", "96ms", "192ms", "384ms", "768ms", "1.536s" };

static const char *drc_drc_dec_text[] = {"0ms", "1.5ms", "3ms", "6ms", "12ms", "24ms", "48ms", "96ms",
                                         "192ms", "384ms", "768ms", "1.536s", "3.072s", "6.144s", "12.288s", "24.576s" };

static const char *drc_peak_att_text[] = {"0ms", "0.09ms", "0.19ms", "0.37ms", "0.75ms", "1.5ms", "3ms", "6ms",
                                         "12ms", "24ms", "48ms", "96ms", "192ms", "384ms", "768ms", "1.536s" };

static const char *drc_peak_rel_text[] = {"0ms", "1.5ms", "3ms", "6ms", "12ms", "24ms", "48ms", "96ms",
                                         "192ms", "384ms", "768ms", "1.536s", "3.072s", "6.144s", "12.288s", "24.576s" };

static const char *sdata_fmt_text[] = {"I2S", "LEFT Just", "RIGHT 24", "RIGHT 16"};
static const char *sample_rate_text[] = {"8kHz to 12kHz", "16kHz to 24kHz", "32kHz to 48kHz", "64kHz to 96kHz"};
static const char *ssm2518_sai_text[] = {"Stereo", "TDM2", "TDM4", "TDM8", "TDM16", "Mono PCM"};


/****************************************************************************************************/
/* creating controls for "Edge Speed and Clocking Control Register" at address 0x01 */
static const struct soc_enum ssm2518_asr_enum =
		SOC_ENUM_SINGLE(SSM2518_CLOCK_CTRL, 0, 2, asr_text);


/****************************************************************************************************/
/* creating controls for "Serial Audio Interface and Sample Rate Control Register" at address 0x02 */
/* settings for sdata_format */
static const struct soc_enum ssm2518_sdata_fmt_enum =
		SOC_ENUM_SINGLE(SSM2518_SER_SRATE, 5, 4, sdata_fmt_text);
/* settings for sample rate */
static const struct soc_enum ssm2518_sample_rate =
		SOC_ENUM_SINGLE(SSM2518_SER_SRATE, 0, 4, sample_rate_text);
/* settings for serial audio format */
static const struct soc_enum ssm2518_sai_enum =
	SOC_ENUM_SINGLE(SSM2518_SER_SRATE, 2, 6, ssm2518_sai_text);



/****************************************************************************************************/
/* creating controls for "Serial Audio Interface Control Register" at address 0x03                  */
static const struct soc_enum ssm2518_lrclk_mode =
		SOC_ENUM_SINGLE( SSM2518_SER_CTRL, 6, 2, zero_one_text);
static const struct soc_enum ssm2518_lrclk_pol =
		SOC_ENUM_SINGLE( SSM2518_SER_CTRL, 5, 2, zero_one_text);
static const struct soc_enum ssm2518_sai_msb =
		SOC_ENUM_SINGLE( SSM2518_SER_CTRL, 4, 2, zero_one_text);
static const struct soc_enum ssm2518_bclk_tdmc =
		SOC_ENUM_SINGLE( SSM2518_SER_CTRL, 3, 2, zero_one_text);
static const struct soc_enum ssm2518_bclk_edge =
		SOC_ENUM_SINGLE( SSM2518_SER_CTRL, 1, 2, zero_one_text);



/****************************************************************************************************/
/* creating controls for "Volume and Mute Control Register" at address 0x07                         */
static const struct soc_enum ssm2518_master_mute_enum =
		SOC_ENUM_SINGLE(SSM2518_VOL_MUTE_CTRL, 0, 2, on_mute_text);
static const struct soc_enum ssm2518_left_mute_enum =
		SOC_ENUM_SINGLE(SSM2518_VOL_MUTE_CTRL, 1, 2, on_mute_text);
static const struct soc_enum ssm2518_right_mute_enum =
		SOC_ENUM_SINGLE(SSM2518_VOL_MUTE_CTRL, 2, 2, on_mute_text);
static const struct soc_enum ssm2518_lvol_enum =
		SOC_ENUM_SINGLE(SSM2518_VOL_MUTE_CTRL, 3, 2, on_mute_text);
static const struct soc_enum ssm2518_deemp_enum =
		SOC_ENUM_SINGLE(SSM2518_VOL_MUTE_CTRL, 4, 2, on_off_text);
static const struct soc_enum ssm2518_amute_enum =
		SOC_ENUM_SINGLE(SSM2518_VOL_MUTE_CTRL, 7, 2, on_off_text);


/****************************************************************************************************/
/* creating controls for "Power Fault Control Register" at address 0x09                             */
static const struct soc_enum ssm2518_artime_enum =
		SOC_ENUM_SINGLE(SSM2518_PWR_FAULT_CTRL, 6, 4, artime_text);
static const struct soc_enum ssm2518_lpmode_enum =
		SOC_ENUM_SINGLE(SSM2518_PWR_FAULT_CTRL, 3, 2, zero_one_text);
static const struct soc_enum ssm2518_rpwdn_enum =
		SOC_ENUM_SINGLE(SSM2518_PWR_FAULT_CTRL, 2, 2, on_off_text);
static const struct soc_enum ssm2518_lpwdn_enum =
		SOC_ENUM_SINGLE(SSM2518_PWR_FAULT_CTRL, 1, 2, on_off_text);
static const struct soc_enum ssm2518_apwdn_enum =
		SOC_ENUM_SINGLE(SSM2518_PWR_FAULT_CTRL, 0, 2, dis_en_text);


/****************************************************************************************************/
/* creating controls for "Dynamic Range Control Register 1" at address 0x0A                         */
static const struct soc_enum ssm2518_prevol_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_1, 6, 2, pre_post_text);
static const struct soc_enum ssm2518_limen_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_1, 5, 2, dis_en_text);
static const struct soc_enum ssm2518_compen_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_1, 4, 2, dis_en_text);
static const struct soc_enum ssm2518_expen_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_1, 3, 2, dis_en_text);
static const struct soc_enum ssm2518_ngen_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_1, 2, 2, dis_en_text);
static const struct soc_enum ssm2518_drcen_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_1, 0, 4, drc_en_text);


/****************************************************************************************************/
/* creating controls for "Dynamic Range Control Register 2" at address 0x0B                         */
static const struct soc_enum ssm2518_peak_att_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_2, 4, 16, drc_peak_att_text);
static const struct soc_enum ssm2518_peak_rel_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_2, 0, 16, drc_peak_rel_text);

/****************************************************************************************************/
/* creating controls for "Dynamic Range Control Register 6" at address 0x0f                         */
static const struct soc_enum ssm2518_drc_att_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_6, 4, 16, drc_drc_att_text);
static const struct soc_enum ssm2518_drc_dec_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_6, 0, 16, drc_drc_dec_text);

/****************************************************************************************************/
/* creating controls for "Dynamic Range Control Register 7" at address 0x10                         */
static const struct soc_enum ssm2518_hdt_nor_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_7, 4, 13, drc_hdt_text);
static const struct soc_enum ssm2518_hdt_ng_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_7, 0, 13, drc_hdt_text);

/****************************************************************************************************/
/* creating controls for "Dynamic Range Control Register 9" at address 0x12                         */
static const struct soc_enum ssm2518_rms_tav_enum =
		SOC_ENUM_SINGLE(SSM2518_DRC_9, 0, 16, drc_rms_tav_text);


static const DECLARE_TLV_DB_SCALE(out_gain_tlv, -7125, 75, -7125);
static const DECLARE_TLV_DB_SCALE(drc_et_tlv, -8100, 300, -8100);
static const DECLARE_TLV_DB_SCALE(drc_nt_tlv, -9600, 300, -9600);
static const DECLARE_TLV_DB_SCALE(drc_lt_tlv, -4500, 300, -4500);
static const DECLARE_TLV_DB_SCALE(drc_ct_tlv, -6000, 300, -6000);
static const DECLARE_TLV_DB_SCALE(drc_smax_tlv, -4500, 300, -4500);
static const DECLARE_TLV_DB_SCALE(drc_smin_tlv, -9600, 300, -9600);

static const DECLARE_TLV_DB_SCALE(drc_postg_tlv, -2400, 300, -2400);
static const DECLARE_TLV_DB_SCALE(drc_rrh_tlv, -75, 27, -75);



static const DECLARE_TLV_DB_SCALE(input_gain_tlv, -1725, 75, 0);
static const DECLARE_TLV_DB_SCALE(sidetone_vol_tlv, -1800, 300, 0);

static int snd_soc_ssm2518_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int snd_soc_ssm2518_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;

}

#define SOC_SSM2518_SINGLE_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
        .get = snd_soc_ssm2518_get_volsw,	  \
	.put = snd_soc_ssm2518_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }



static const struct snd_kcontrol_new ssm2518_snd_controls[] = {

/* Controls for register SSM2518_RESET */


/* Controls for register SSM2518_CLOCK_CTRL */
	SOC_ENUM("ASR", ssm2518_asr_enum),


/* Controls for register SSM2518_SER_SRATE */
	SOC_ENUM("Sample-Rate", ssm2518_sample_rate),
	SOC_ENUM("SAI", ssm2518_sai_enum),
	SOC_ENUM("SDATA", ssm2518_sdata_fmt_enum),

/* Controls for register SSM2518_SER_CTRL */
	SOC_ENUM("LRCLK_Mode", ssm2518_lrclk_mode ),
	SOC_ENUM("LRCLK_Pol",  ssm2518_lrclk_pol ),
	SOC_ENUM("SAI_MSB",    ssm2518_sai_msb ),
	SOC_ENUM("BCLK_TDMC",  ssm2518_bclk_tdmc ),
	SOC_ENUM("BCLK_edge",  ssm2518_bclk_edge ),

/* Controls for register SSM2518_CHAN_MAP */


/* Controls for register SSM2518_LVOL_CTRL and SSM2518_RVOL_CTRL simultaneously */
	SOC_DOUBLE_R_TLV("Volume", SSM2518_LVOL_CTRL, SSM2518_RVOL_CTRL, 0, 127, 0, out_gain_tlv),

/* Controls for register SSM2518_VOL_MUTE_CTRL */
	SOC_ENUM("Master", ssm2518_master_mute_enum),
	SOC_ENUM("Left", ssm2518_left_mute_enum),
	SOC_ENUM("Right", ssm2518_right_mute_enum),
	SOC_ENUM("VOL-L", ssm2518_lvol_enum),
	SOC_ENUM("AMUTE", ssm2518_amute_enum),
	SOC_ENUM("DEEMP", ssm2518_deemp_enum),

/* Controls for register SSM2518_FAULT_CTRL */


/* Controls for register SSM2518_PWR_FAULT_CTRL */
	SOC_ENUM("AR_TIME", ssm2518_artime_enum),
	SOC_ENUM("LP_MODE", ssm2518_lpmode_enum),
	SOC_ENUM("R_PWDN", ssm2518_rpwdn_enum),
	SOC_ENUM("L_PWDN", ssm2518_lpwdn_enum),
	SOC_ENUM("APWDN_EN", ssm2518_apwdn_enum),

/* Controls for register SSM2518_DRC_1 */
	SOC_ENUM("PRE_VOL", ssm2518_prevol_enum),
	SOC_ENUM("LIM_EN", ssm2518_limen_enum),
	SOC_ENUM("COMP_EN", ssm2518_compen_enum),
	SOC_ENUM("EXP_EN", ssm2518_expen_enum),
	SOC_ENUM("NG_EN", ssm2518_ngen_enum),
	SOC_ENUM("DRC_EN", ssm2518_drcen_enum),

/* Controls for register SSM2518_DRC_2 */
	SOC_ENUM("PEAK_ATT", ssm2518_peak_att_enum),
	SOC_ENUM("PEAK_REL", ssm2518_peak_rel_enum),

/* Controls for register SSM2518_DRC_3 */
	SOC_SINGLE_TLV  ("DRC_LT", SSM2518_DRC_3, 4, 15, 0, drc_lt_tlv),
	SOC_SINGLE_TLV  ("DRC_CT", SSM2518_DRC_3, 0, 15, 0, drc_ct_tlv),

/* Controls for register SSM2518_DRC_4 */
	SOC_SINGLE_TLV  ("DRC_ET", SSM2518_DRC_4, 4, 15, 0, drc_et_tlv),
	SOC_SINGLE_TLV  ("DRC_NT", SSM2518_DRC_4, 0, 15, 0, drc_nt_tlv),

/* Controls for register SSM2518_DRC_5 */
	SOC_SINGLE_TLV  ("DRC_SMAX", SSM2518_DRC_5, 4, 15, 0, drc_smax_tlv),
	SOC_SINGLE_TLV  ("DRC_SMIN", SSM2518_DRC_5, 0, 15, 0, drc_smin_tlv),

/* Controls for register SSM2518_DRC_6 */
	SOC_ENUM("DRC_ATT", ssm2518_drc_att_enum),
	SOC_ENUM("DRC_DEC", ssm2518_drc_dec_enum),

/* Controls for register SSM2518_DRC_7 */
	SOC_ENUM("HDT_NOR", ssm2518_hdt_nor_enum),
	SOC_ENUM("HDT_NG", ssm2518_hdt_ng_enum),

/* Controls for register SSM2518_DRC_8 */
	SOC_SINGLE_TLV  ("DRC_POST_G", SSM2518_DRC_8, 2, 15, 0, drc_postg_tlv),
	SOC_SINGLE_TLV  ("DRC_RRH", SSM2518_DRC_8, 0, 3, 0, drc_rrh_tlv),

/* Controls for register SSM2518_DRC_9 */
	SOC_ENUM("RMS_TAV", ssm2518_rms_tav_enum),

};


static const struct snd_soc_dapm_widget ssm2518_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("LHPOUT"),
	SND_SOC_DAPM_OUTPUT("RHPOUT"),
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),

};

static const struct snd_soc_dapm_route intercon[] = {
	/* Output Mixer */
	{"Output Mixer", "Line Bypass Switch", "Line Input"},
	{"Output Mixer", "Playback Switch", "DAC"},
	{"Output Mixer", "Mic Sidetone Switch", "Mic Input"},

	/* Outputs */
	{"RHPOUT", NULL, "Output Mixer"},
	{"LHPOUT", NULL, "Output Mixer"},
	{"LOUT", NULL, "Output Mixer"},
	{"ROUT", NULL, "Output Mixer"},
};

/* SSM2518 driver data */
struct ssm2518 {
	struct snd_soc_codec codec;
	int mclk;
	int requested_adc;
	int requested_dac;
};

static void dump_codec_register(struct snd_soc_codec *codec) {
#ifdef DEBUG
  int i;
  u8 data[9];

  for ( i = 0; i < 8; i++ ) {
    data[0] = i;
    i2c_master_send(codec->control_data, &data[0], 1);
    i2c_master_recv(codec->control_data, &data[i%8 + 1], 1);
  }
  printk(KERN_INFO "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
	 data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
  for ( i = 8; i < 16; i++ ) {
    data[0] = i;
    i2c_master_send(codec->control_data, &data[0], 1);
    i2c_master_recv(codec->control_data, &data[i%8 + 1], 1);
  }
  printk(KERN_INFO "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
	 data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
  for ( i = 16; i < 19; i++ ) {
    data[0] = i;
    i2c_master_send(codec->control_data, &data[0], 1);
    i2c_master_recv(codec->control_data, &data[i%8 + 1], 1);
  }
  printk(KERN_INFO "0x%02x 0x%02x 0x%02x",
	 data[1], data[2], data[3]);
#endif
}

static int set_sample_rate_control(struct snd_soc_codec *codec, int mclk,
		u32 sample_rate_adc, u32 sample_rate_dac)
{
	int i, i_save;
	int zwi;
	u8 reg;
	
	i_save = -1;
	while ( i_save == -1 ) {
		i = 0;
	while (ssm2518_mcs[i].div != 0) {
		zwi = (int)(ssm2518_mcs[i].div) * (int)(sample_rate_dac);
//		printk("div: %d, zwi: %d, mclk: %d\n", i, zwi, mclk);
		if ( zwi > mclk ) {
			i_save = i - 1;
			break;
		}
		i++;
	}
	sample_rate_dac *= 2;
	}
//	printk("selected mcs: %d - regval: %x\n", ssm2518_mcs[i_save].val, ssm2518_mcs[i_save].mcs_val);
	
	reg = ssm2518_read_reg_cache(codec, SSM2518_RESET);
	reg &= SSM2518_MCS_MASK;
	reg |= ssm2518_mcs[i_save].mcs_val;
	reg = 0;
	ssm2518_write(codec, SSM2518_RESET, reg);
	
	return 0;
}

static int ssm2518_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, ssm2518_dapm_widgets,
				  ARRAY_SIZE(ssm2518_dapm_widgets));

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));
	return 0;
}

static int ssm2518_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 iface_reg;
	int ret;
	struct ssm2518 *ssm2518 = container_of(codec, struct ssm2518, codec);
	u32 sample_rate_adc = ssm2518->requested_adc;
	u32 sample_rate_dac = ssm2518->requested_dac;
	u32 sample_rate = params_rate(params);

	u8 sdata_fmt = 0;
	
//	printk(KERN_INFO "%s+\n", __func__);
//	printk("ssm2518_hw_params: sample_rate: %d\n", sample_rate);
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ssm2518->requested_dac = sample_rate_dac = sample_rate;
		if (!sample_rate_adc)
			sample_rate_adc = sample_rate;
	} else {
		ssm2518->requested_adc = sample_rate_adc = sample_rate;
		if (!sample_rate_dac)
			sample_rate_dac = sample_rate;
	}
	ret = set_sample_rate_control(codec, ssm2518->mclk, sample_rate_adc,
			sample_rate_dac);
	if (ret < 0)
		return ret;

	iface_reg =
	    ssm2518_read_reg_cache(codec, SSM2518_SER_SRATE);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sdata_fmt = 0x40;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sdata_fmt = 0x60;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		break;
	}
//	ssm2518_write(codec, SSM2518_SER_SRATE, 0x22);

	return 0;
}

static int ssm2518_pcm_prepare(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
//	printk(KERN_INFO "%s++\n", __func__);

	/* set active */
	ssm2518_write(codec, SSM2518_RESET, SSM2518_MCS_64);

	return 0;
}

static void ssm2518_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
  u8 reg;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct ssm2518 *ssm2518 = container_of(codec, struct ssm2518, codec);
//	printk(KERN_INFO "%s++\n", __func__);

	/* deactivate */
	reg = ssm2518_read_reg_cache(codec, SSM2518_RESET);

	if (!codec->active) {
		udelay(50);
		ssm2518_write(codec, SSM2518_RESET, reg | SSM2518_SPWDN);
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ssm2518->requested_dac = 0;
	else
		ssm2518->requested_adc = 0;
}

static int ssm2518_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 reg;
//	printk(KERN_INFO "%s+\n", __func__);

	reg = ssm2518_read_reg_cache(codec, SSM2518_VOL_MUTE_CTRL);
	if (mute)
		reg |= SSM2518_MASTER_MUTE | SSM2518_L_MUTE | SSM2518_R_MUTE;
	else
		reg &= ~(SSM2518_MASTER_MUTE | SSM2518_L_MUTE | SSM2518_R_MUTE);

	ssm2518_write(codec, SSM2518_VOL_MUTE_CTRL, reg);

	return 0;
}

static int ssm2518_set_dai_fmt(struct snd_soc_dai *codec_dai,
				   unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 iface_reg;
//	printk(KERN_INFO "%s+\n", __func__);

	iface_reg = ssm2518_read_reg_cache(codec, SSM2518_SER_SRATE);

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	  iface_reg &= ~SSM2518_SDATA_MASK;
	  break;
	case SND_SOC_DAIFMT_RIGHT_J:
	  iface_reg &= ~SSM2518_SDATA_MASK;
	  iface_reg |= SSM2518_SDATA_RIGHT_16;
	  break;
	case SND_SOC_DAIFMT_LEFT_J:
	  iface_reg &= ~SSM2518_SDATA_MASK;
	  iface_reg |= SSM2518_SDATA_LEFT_JUST;
	  break;
	default:
	  return -EINVAL;
	}

	/* this device can only be slave */
	if ( (fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS)
		return -EINVAL;
	
	ssm2518_write(codec, SSM2518_SER_SRATE, iface_reg);

	return 0;
}

static int ssm2518_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				      int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ssm2518 *ssm2518 = container_of(codec, struct ssm2518, codec);
//	printk(KERN_INFO "%s+\n", __func__);
	ssm2518->mclk = freq;
//	printk(KERN_INFO "ssm2518->mclk set to %d\n", freq);
	return 0;
}

static int ssm2518_set_bias_level(struct snd_soc_codec *codec,
				      enum snd_soc_bias_level level)
{
u8 reg = 0;
//	printk(KERN_INFO "%s+\n", __func__);
	switch (level) {
	case SND_SOC_BIAS_ON:
//	  printk(KERN_INFO "SND_SOC_BIAS_ON\n");
	/* normal operation mode */
	reg = ssm2518_read_reg_cache(codec, SSM2518_RESET);
	ssm2518_write( codec, SSM2518_RESET, reg & ~(SSM2518_SPWDN | SSM2518_S_RST) );
	/* set all channels to normal operation */
	reg = ssm2518_read_reg_cache(codec, SSM2518_VOL_MUTE_CTRL);
	ssm2518_write( codec, SSM2518_VOL_MUTE_CTRL, reg & ~(SSM2518_MASTER_MUTE | SSM2518_L_MUTE | SSM2518_R_MUTE) );

	break;
	case SND_SOC_BIAS_PREPARE:
//	  printk(KERN_INFO "SND_SOC_BIAS_PREPARE\n");
		/* setting power fault register to normal operation mode */
		break;
	case SND_SOC_BIAS_STANDBY:
//	  printk(KERN_INFO "SND_SOC_BIAS_STANDBY\n");
	  /* normal operation mode */
	  reg = ssm2518_read_reg_cache(codec, SSM2518_RESET);
	  ssm2518_write( codec, SSM2518_RESET, reg & ~(SSM2518_SPWDN | SSM2518_S_RST) );
	  /* set all channels to normal operation */
	  reg = ssm2518_read_reg_cache(codec, SSM2518_VOL_MUTE_CTRL);
	  ssm2518_write( codec, SSM2518_VOL_MUTE_CTRL, reg | (SSM2518_MASTER_MUTE | SSM2518_L_MUTE | SSM2518_R_MUTE) );
		break;
	case SND_SOC_BIAS_OFF:
//	  printk(KERN_INFO "SND_SOC_BIAS_OFF\n");
		/* software master powerdown */
		reg = ssm2518_read_reg_cache(codec, SSM2518_RESET);
    		ssm2518_write( codec, SSM2518_RESET, reg | SSM2518_SPWDN | SSM2518_S_RST );
		break;
	}
	codec->bias_level = level;
	return 0;
}


#define SSM2518_RATES	SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000


#define SSM2518_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
			 SNDRV_PCM_FMTBIT_S24_3LE | \
			 SNDRV_PCM_FMTBIT_S20_3LE | \
			 SNDRV_PCM_FMTBIT_U8 | \
			 SNDRV_PCM_FMTBIT_S32_LE )

static struct snd_soc_dai_ops ssm2518_dai_ops = {
	.prepare	= ssm2518_pcm_prepare,
	.hw_params	= ssm2518_hw_params,
	.shutdown	= ssm2518_shutdown,
	.digital_mute	= ssm2518_mute,
	.set_fmt	= ssm2518_set_dai_fmt,
	.set_sysclk	= ssm2518_set_dai_sysclk,
};

struct snd_soc_dai ssm2518_dai = {
	.name = "ssm2518",
	.playback = {
	.stream_name = "Playback",
	.channels_min = 2,
	.channels_max = 2,
	.rates = SSM2518_RATES,
	.formats = SSM2518_FORMATS,},
	.ops = &ssm2518_dai_ops,
};
EXPORT_SYMBOL_GPL(ssm2518_dai);

static int ssm2518_suspend(struct platform_device *pdev,
			       pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
//	printk(KERN_INFO "%s+\n", __func__);

	ssm2518_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int ssm2518_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 reg;
//	printk(KERN_INFO "%s+\n", __func__);

	/* Sync reg_cache with the hardware */
	for (reg = 1; reg <= SSM2518_DRC_9; reg++) {
		u8 val = ssm2518_read_reg_cache(codec, reg);
		ssm2518_write(codec, reg, val);
	}

	ssm2518_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

/*
 * initialise the AIC23 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int ssm2518_register(struct ssm2518 *ssm2518)
{
	struct snd_soc_codec *codec = &ssm2518->codec;
	int ret = 0;
	u8 reg;

	if ( ssm2518_codec ) {
		dev_err(codec->dev, "Another SSM2518 is registered\n");
		return -EINVAL;
	}
	
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	
	snd_soc_codec_set_drvdata(codec, ssm2518);
	
	codec->name = "ssm2518";
	codec->owner = THIS_MODULE;
	codec->read = ssm2518_read_reg_cache;
	codec->write = ssm2518_write;
	codec->set_bias_level = ssm2518_set_bias_level;
	codec->dai = &ssm2518_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(ssm2518_reg);
	codec->reg_cache =
	    kmemdup(ssm2518_reg, sizeof(ssm2518_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

// it's a hack but it's customer specific here, so it doesn't matter so far
// on all customer base boards PTD3 is connected to the enable input of the audio codec.
// The codec SSM2518 needs a high level for enabling.
	gpio_request_one(GPIO_PTD3, GPIOF_OUT_INIT_HIGH, "en-sound"); // for customer base boards with ssm2518 support
	gpio_free(GPIO_PTD3);

	/* Set Codec to normal operation mode but keep it in software master powerdown mode */
	reg = ssm2518_read_reg_cache(codec, SSM2518_RESET);
     	ret = ssm2518_write(codec, SSM2518_RESET, reg & ~SSM2518_S_RST);
	
	if ( ret != 0 ) {
		ret = -1;
		goto err;
	}

	udelay(10000);

	ssm2518_dai.dev = codec->dev;
	ssm2518_codec = codec;
	
	/* recovering all faults */
	ssm2518_write(codec, SSM2518_FAULT_CTRL, 0x1c);
	udelay(1000);

//	ssm2518_set_bias_level(codec, SND_SOC_BIAS_ON);


	/* power on device */
	//		ssm2518_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	


	/* disable power of the left and the right output */
	//ssm2518_write( codec, SSM2518_PWR_FAULT_CTRL, SSM2518_AR_TIME_40 | SSM2518_L_PWDN | SSM2518_R_PWDN );
	
	/* enabling automatic sample rate detection */
	ssm2518_write(codec, SSM2518_CLOCK_CTRL, 0x00);
	
	/* setting 32 BCLK Cycles per slot */
//	ssm2518_write(codec, SSM2518_SER_CTRL, 0x08);
	
//	ssm2518_write(codec, SSM2518_SER_SRATE, 0x02);

	/* mapping channel 1 to the right and channel 0 to the left */
	ssm2518_write(codec, SSM2518_CHAN_MAP, 0x10);
	
	/* setting 3.6V Gain, auto mute disabled no deemphasis and all channels to normal operation */
	ssm2518_write(codec, SSM2518_VOL_MUTE_CTRL, 0xa0);
	
	
	/* enabling DRC operates pre volume */
//	ssm2518_write(codec, SSM2518_DRC_1, 0x00);
	
	/* enable power of the left and the right output */
	ssm2518_write( codec, SSM2518_PWR_FAULT_CTRL, SSM2518_AR_TIME_40 & ~(SSM2518_L_PWDN | SSM2518_R_PWDN) );

//	ssm2518_write(codec, SSM2518_RESET, SSM2518_MCS_64 );
	
//	reg = ssm2518_read_reg_cache(codec, SSM2518_VOL_MUTE_CTRL);
//	ssm2518_write(codec, SSM2518_VOL_MUTE_CTRL, reg & 0xf0 );
	
	
//	ssm2518_write(codec, SSM2518_TEST, 0x06 );

	ret = snd_soc_register_codec(codec);
	if ( ret != 0 ) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}
	
	ret = snd_soc_register_dai(&ssm2518_dai);
	if ( ret != 0 ) {
		dev_err(codec->dev, "Failed to register dai: %d\n", ret);
		goto err_codec;
	}
	
	dump_codec_register(codec);

	return ret;

err_codec:
	snd_soc_unregister_codec(codec);
	
err:
	kfree(codec->reg_cache);
	return ret;
}
static struct snd_soc_device *ssm2518_socdev;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
/*
 * If the i2c layer weren't so broken, we could pass this kind of data
 * around
 */
 
static __devexit ssm2518_unregister(struct ssm2518 *ssm2518)
{
	snd_soc_unregister_dai(&ssm2518_dai);
	snd_soc_unregister_codec(&ssm2518->codec);
	kfree(ssm2518);
	ssm2518_codec = NULL;
	return 0;
}
 
static int ssm2518_i2c_probe(struct i2c_client *i2c,
				   const struct i2c_device_id *i2c_id)
{
	struct snd_soc_codec *codec;
	struct ssm2518 *ssm2518;
	
	ssm2518 = kzalloc(sizeof(struct ssm2518), GFP_KERNEL);
	if ( ssm2518 == NULL ) {
		return -ENOMEM;
	}
		
	codec = &ssm2518->codec;
	codec->hw_write = (hw_write_t)i2c_master_send;
	codec->hw_read = NULL;
	
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EINVAL;

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	codec->dev = &i2c->dev;
	
	return  ssm2518_register(ssm2518);
}
static int __exit ssm2518_i2c_remove(struct i2c_client *i2c)
{
	struct ssm2518 *ssm2518 = i2c_get_clientdata(i2c); 
	ssm2518_unregister(ssm2518);
	return 0;
}

static const struct i2c_device_id ssm2518_id[] = {
	{"ssm2518", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ssm2518_id);

static struct i2c_driver ssm2518_i2c_driver = {
	.driver = {
		   .name = "ssm2518",
		   },
	.probe = ssm2518_i2c_probe,
	.remove = __exit_p(ssm2518_i2c_remove),
	.id_table = ssm2518_id,
};

#endif

static int ssm2518_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;
	
	printk(KERN_INFO "SSM2518 Audio Codec %s\n", SSM2518_VERSION);

	if ( ssm2518_codec == NULL ) {
		dev_err(&pdev->dev, "Codec not registered\n");
		return -ENODEV;
	}
	
	socdev->card->codec = ssm2518_codec;
	codec = ssm2518_codec;
	
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "ssm2518: failed to create pcm\n");
		goto pcm_err;
	}

	ret = snd_soc_add_controls(codec, ssm2518_snd_controls,
			       ARRAY_SIZE(ssm2518_snd_controls));

	ssm2518_add_widgets(codec);

	return ret;
pcm_err:
	return ret;
}

static int ssm2518_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	struct ssm2518 *ssm2518 = container_of(codec, struct ssm2518, codec);

	if (codec->control_data)
		ssm2518_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&ssm2518_i2c_driver);
#endif
	kfree(codec->reg_cache);
	kfree(ssm2518);

	return 0;
}
struct snd_soc_codec_device soc_codec_dev_ssm2518 = {
	.probe = ssm2518_probe,
	.remove = ssm2518_remove,
	.suspend = ssm2518_suspend,
	.resume = ssm2518_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ssm2518);


static int __init ssm2518_modinit(void)
{
	return i2c_add_driver(&ssm2518_i2c_driver);
}
module_init(ssm2518_modinit);


static void __exit ssm2518_exit(void)
{
	i2c_del_driver(&ssm2518_i2c_driver);
}
module_exit(ssm2518_exit);

MODULE_DESCRIPTION("ASoC SSM2518 codec driver");
MODULE_AUTHOR("Dr. Frank Reither <frank.reither@emtrion.de>");
MODULE_LICENSE("GPL");
