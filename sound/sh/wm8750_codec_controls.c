/*
 * wm8750_codec_controls.c   audio codec mixer controls
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
 *   [1] soc/soc_core.c
 *   [2] soc/codecs/wm8750.c
 *
 **/

static int wm8750_update_bits(unsigned short reg,
                              unsigned int mask, unsigned int value)
{
	int change;
	unsigned int old, new;

	mutex_lock(&io_mutex);

	old = wm8750_read_reg_cache(reg);
	new = (old & ~mask) | value;
	change = old != new;
	if (change) {
                int pwr1;
                int pwr2;
                int adc_dac_disabled = (reg == WM8750_3D && mask == 0x80);
                if (adc_dac_disabled) {
                        /* MODE3D may only be changed with ADCL, ADCR, DACL and
                         * DACR disabled. It should be changed only when not
                         * activly playing or capturing */
                        pwr1 = wm8750_read_reg_cache(WM8750_PWR1);
                        pwr2 = wm8750_read_reg_cache(WM8750_PWR2);
                        wm8750_write(WM8750_PWR1, pwr1 & ~(0x00C));
                        wm8750_write(WM8750_PWR2, pwr2 & ~(0x180));
                }

		wm8750_write(reg, new);

                if (adc_dac_disabled) {
                        wm8750_write(WM8750_PWR1, pwr1);
                        wm8750_write(WM8750_PWR2, pwr2);
                }
        }

	mutex_unlock(&io_mutex);
	return change;
}

static int wm8750_info_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;

	if (max == 1 && !strstr(kcontrol->id.name, " Volume"))
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = shift == rshift ? 1 : 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}

static int wm8750_get_volsw(struct snd_kcontrol *kcontrol,
                            struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;

	ucontrol->value.integer.value[0] =
		(wm8750_read_reg_cache(reg) >> shift) & mask;
	if (shift != rshift)
		ucontrol->value.integer.value[1] =
			(wm8750_read_reg_cache(reg) >> rshift) & mask;
	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
		if (shift != rshift)
			ucontrol->value.integer.value[1] =
				max - ucontrol->value.integer.value[1];
	}

	return 0;
}

static int wm8750_put_volsw(struct snd_kcontrol *kcontrol,
                            struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val, val2, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = max - val;
	val_mask = mask << shift;
	val = val << shift;
	if (shift != rshift) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		if (invert)
			val2 = max - val2;
		val_mask |= mask << rshift;
		val |= val2 << rshift;
	}
	return wm8750_update_bits(reg, val_mask, val);
}

static int wm8750_info_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;

	if (max == 1 && !strstr(kcontrol->id.name, " Volume"))
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}

static int wm8750_get_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;

	ucontrol->value.integer.value[0] =
		(wm8750_read_reg_cache(reg) >> shift) & mask;
	ucontrol->value.integer.value[1] =
		(wm8750_read_reg_cache(reg2) >> shift) & mask;
	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
		ucontrol->value.integer.value[1] =
			max - ucontrol->value.integer.value[1];
	}

	return 0;
}


static int wm8750_put_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int err;
	unsigned int val, val2, val_mask;

	val_mask = mask << shift;
	val = (ucontrol->value.integer.value[0] & mask);
	val2 = (ucontrol->value.integer.value[1] & mask);

	if (invert) {
		val = max - val;
		val2 = max - val2;
	}

	val = val << shift;
	val2 = val2 << shift;

	err = wm8750_update_bits(reg, val_mask, val);
	if (err < 0)
		return err;

	err = wm8750_update_bits(reg2, val_mask, val2);
	return err;
}

static int wm8750_info_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = e->shift_l == e->shift_r ? 1 : 2;
	uinfo->value.enumerated.items = e->max;

	if (uinfo->value.enumerated.item > e->max - 1)
		uinfo->value.enumerated.item = e->max - 1;
	strcpy(uinfo->value.enumerated.name,
		e->texts[uinfo->value.enumerated.item]);
	return 0;
}

static int wm8750_get_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val, bitmask;

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
	val = wm8750_read_reg_cache(e->reg);
	ucontrol->value.enumerated.item[0]
		= (val >> e->shift_l) & (bitmask - 1);
	if (e->shift_l != e->shift_r)
		ucontrol->value.enumerated.item[1] =
			(val >> e->shift_r) & (bitmask - 1);

	return 0;
}

static int wm8750_put_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val;
	unsigned int mask, bitmask;

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;
	val = ucontrol->value.enumerated.item[0] << e->shift_l;
	mask = (bitmask - 1) << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->max - 1)
			return -EINVAL;
		val |= ucontrol->value.enumerated.item[1] << e->shift_r;
		mask |= (bitmask - 1) << e->shift_r;
	}

	return wm8750_update_bits(e->reg, mask, val);
}

static int wm8750_info_flag(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
        uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	return 0;
}

static int wm8750_get_flag(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
        int *flag = (int*) kcontrol->private_value;

        ucontrol->value.integer.value[0] = *flag;
	return 0;
}

static int wm8750_put_flag(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
        int *flag = (int*) kcontrol->private_value;

        *flag = ucontrol->value.integer.value[0];
	return 0;
}


#define WM8750_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xmax, xtexts) \
{	.reg = xreg, .shift_l = xshift_l, .shift_r = xshift_r, \
	.max = xmax, .texts = xtexts }
#define WM8750_ENUM_SINGLE(xreg, xshift, xmax, xtexts) \
	WM8750_ENUM_DOUBLE(xreg, xshift, xshift, xmax, xtexts)

#define WM8750_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = wm8750_info_enum_double, \
	.get = wm8750_get_enum_double, .put = wm8750_put_enum_double, \
	.private_value = (unsigned long)&xenum }
#define WM8750_SINGLE(xname, reg, shift, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = wm8750_info_volsw, .get = wm8750_get_volsw,\
	.put = wm8750_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }
#define WM8750_DOUBLE_R(xname, reg_left, reg_right, xshift, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = wm8750_info_volsw_2r, \
	.get = wm8750_get_volsw_2r, .put = wm8750_put_volsw_2r, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, .rreg = reg_right, .shift = xshift, \
		.max = xmax, .invert = xinvert} }

#define WM8750_FLAG(xname, variable) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = wm8750_info_flag, .get = wm8750_get_flag,\
	.put = wm8750_put_flag, \
        .private_value = (int) &variable }

static const char *wm8750_bass[] = {"Linear Control", "Adaptive Boost"};
static const char *wm8750_bass_filter[] = { "130Hz @ 48kHz", "200Hz @ 48kHz" };
static const char *wm8750_treble[] = {"8kHz", "4kHz"};
static const char *wm8750_3d_lc[] = {"200Hz", "500Hz"};
static const char *wm8750_3d_uc[] = {"2.2kHz", "1.5kHz"};
static const char *wm8750_3d_func[] = {"Capture", "Playback"};
static const char *wm8750_alc_func[] = {"Off", "Right", "Left", "Stereo"};
static const char *wm8750_ng_type[] = {"Constant PGA Gain",
	"Mute ADC Output"};
static const char *wm8750_line_mux[] = {"Line 1", "Line 2", "Line 3", "PGA",
	"Differential"};
static const char *wm8750_pga_sel[] = {"Line 1", "Line 2", "Line 3",
	"Differential"};
static const char *wm8750_out3[] = {"VREF", "ROUT1 + Vol", "MonoOut",
	"ROUT1"};
static const char *wm8750_diff_sel[] = {"Line 1", "Line 2"};
static const char *wm8750_adcpol[] = {"Normal", "L Invert", "R Invert",
	"L + R Invert"};
static const char *wm8750_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const char *wm8750_mono_mux[] = {"Stereo", "Mono (Left)",
	"Mono (Right)", "Digital Mono"};

static const struct soc_enum wm8750_enum[] = {
WM8750_ENUM_SINGLE(WM8750_BASS, 7, 2, wm8750_bass),
WM8750_ENUM_SINGLE(WM8750_BASS, 6, 2, wm8750_bass_filter),
WM8750_ENUM_SINGLE(WM8750_TREBLE, 6, 2, wm8750_treble),
WM8750_ENUM_SINGLE(WM8750_3D, 5, 2, wm8750_3d_lc),
WM8750_ENUM_SINGLE(WM8750_3D, 6, 2, wm8750_3d_uc),
WM8750_ENUM_SINGLE(WM8750_3D, 7, 2, wm8750_3d_func),
WM8750_ENUM_SINGLE(WM8750_ALC1, 7, 4, wm8750_alc_func),
WM8750_ENUM_SINGLE(WM8750_NGATE, 1, 2, wm8750_ng_type),
WM8750_ENUM_SINGLE(WM8750_LOUTM1, 0, 5, wm8750_line_mux),
WM8750_ENUM_SINGLE(WM8750_ROUTM1, 0, 5, wm8750_line_mux),
WM8750_ENUM_SINGLE(WM8750_LADCIN, 6, 4, wm8750_pga_sel), /* 10 */
WM8750_ENUM_SINGLE(WM8750_RADCIN, 6, 4, wm8750_pga_sel),
WM8750_ENUM_SINGLE(WM8750_ADCTL2, 7, 4, wm8750_out3),
WM8750_ENUM_SINGLE(WM8750_ADCIN, 8, 2, wm8750_diff_sel),
WM8750_ENUM_SINGLE(WM8750_ADCDAC, 5, 4, wm8750_adcpol),
WM8750_ENUM_SINGLE(WM8750_ADCDAC, 1, 4, wm8750_deemph),
WM8750_ENUM_SINGLE(WM8750_ADCIN, 6, 4, wm8750_mono_mux), /* 16 */

};

static const struct snd_kcontrol_new wm8750_snd_controls[] = {
WM8750_DOUBLE_R("Capture Volume", WM8750_LINVOL, WM8750_RINVOL, 0, 63, 0),
WM8750_DOUBLE_R("Capture ZC Switch", WM8750_LINVOL, WM8750_RINVOL, 6, 1, 0),
WM8750_DOUBLE_R("Capture Switch", WM8750_LINVOL, WM8750_RINVOL, 7, 1, 1),

WM8750_DOUBLE_R("Headphone Playback ZC Switch", WM8750_LOUT1V,
	WM8750_ROUT1V, 7, 1, 0),
WM8750_DOUBLE_R("Speaker Playback ZC Switch", WM8750_LOUT2V,
	WM8750_ROUT2V, 7, 1, 0),

WM8750_ENUM("Playback De-emphasis", wm8750_enum[15]),

WM8750_SINGLE("Playback 6dB Attenuate", WM8750_ADCDAC, 7, 1, 0),
WM8750_SINGLE("Capture 6dB Attenuate", WM8750_ADCDAC, 8, 1, 0),

/* Beware. The SIU driver already has PCM Playback Volume and alsa lib get's
 * confused between both as Playback and Volume are removed internally in alsalib */
WM8750_DOUBLE_R("PCM_Codec Volume", WM8750_LDAC, WM8750_RDAC, 0, 255, 0),

WM8750_ENUM("Bass Boost", wm8750_enum[0]),
WM8750_ENUM("Bass Filter", wm8750_enum[1]),
WM8750_SINGLE("Bass Volume", WM8750_BASS, 0, 15, 1),
WM8750_SINGLE("Treble Volume", WM8750_TREBLE, 0, 15, 1),
WM8750_ENUM("Treble Cut-off", wm8750_enum[2]),

WM8750_SINGLE("3D Switch", WM8750_3D, 0, 1, 0),
WM8750_SINGLE("3D Volume", WM8750_3D, 1, 15, 0),
WM8750_ENUM("3D Lower Cut-off", wm8750_enum[3]),
WM8750_ENUM("3D Upper Cut-off", wm8750_enum[4]),
WM8750_ENUM("3D Mode", wm8750_enum[5]),
WM8750_SINGLE("ALC Capture Target Volume", WM8750_ALC1, 0, 7, 0),
WM8750_SINGLE("ALC Capture Max Volume", WM8750_ALC1, 4, 7, 0),
WM8750_ENUM("ALC Capture Function", wm8750_enum[6]),
WM8750_SINGLE("ALC Capture ZC Switch", WM8750_ALC2, 7, 1, 0),
WM8750_SINGLE("ALC Capture Hold Time", WM8750_ALC2, 0, 15, 0),
WM8750_SINGLE("ALC Capture Decay Time", WM8750_ALC3, 4, 15, 0),
WM8750_SINGLE("ALC Capture Attack Time", WM8750_ALC3, 0, 15, 0),
WM8750_SINGLE("ALC Capture NG Threshold", WM8750_NGATE, 3, 31, 0),
WM8750_ENUM("ALC Capture NG Type", wm8750_enum[4]),
WM8750_SINGLE("ALC Capture NG Switch", WM8750_NGATE, 0, 1, 0),

WM8750_SINGLE("Left ADC Capture Volume", WM8750_LADC, 0, 255, 0),
WM8750_SINGLE("Right ADC Capture Volume", WM8750_RADC, 0, 255, 0),

WM8750_SINGLE("ZC Timeout Switch", WM8750_ADCTL1, 0, 1, 0),
WM8750_SINGLE("Playback Invert Switch", WM8750_ADCTL1, 1, 1, 0),

WM8750_SINGLE("Right Speaker Playback Invert Switch", WM8750_ADCTL2, 4, 1, 0),
WM8750_SINGLE("Headphone Polarity Switch", WM8750_ADCTL2, 5, 1, 0),
WM8750_SINGLE("Headphone Enable Switch", WM8750_ADCTL2, 6, 1, 0),

WM8750_DOUBLE_R("Mic Boost", WM8750_LADCIN, WM8750_RADCIN, 4, 3, 0),

WM8750_DOUBLE_R("Bypass Left Playback Volume", WM8750_LOUTM1,
	WM8750_LOUTM2, 4, 7, 1),
WM8750_DOUBLE_R("Bypass Right Playback Volume", WM8750_ROUTM1,
	WM8750_ROUTM2, 4, 7, 1),
WM8750_DOUBLE_R("Bypass Mono Playback Volume", WM8750_MOUTM1,
	WM8750_MOUTM2, 4, 7, 1),

WM8750_SINGLE("Mono Playback ZC Switch", WM8750_MOUTV, 7, 1, 0),

WM8750_DOUBLE_R("Headphone Playback Volume", WM8750_LOUT1V, WM8750_ROUT1V,
	0, 127, 0),
WM8750_DOUBLE_R("Speaker Playback Volume", WM8750_LOUT2V, WM8750_ROUT2V,
	0, 127, 0),

WM8750_SINGLE("Mono Playback Volume", WM8750_MOUTV, 0, 127, 0),
WM8750_FLAG("Powersave Switch", wm8750_powersave),
};

