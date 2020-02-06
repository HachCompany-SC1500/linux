#
# Makefile for ALSA
#

snd-aica-objs := aica.o
snd-sh_dac_audio-objs := sh_dac_audio.o

# Toplevel Module Dependency
obj-$(CONFIG_SND_AICA) += snd-aica.o
obj-$(CONFIG_SND_SH_DAC_AUDIO) += snd-sh_dac_audio.o

obj-$(CONFIG_SH7343_SIU) += siu-sh7343.o
siu-sh7343-y := siu_sh7343_main.o siu_sh7343_coef.o \
		siu_sh7343_spbpro.o siu_sh7343_int.o
siu-sh7343-$(CONFIG_WM8750_CODEC) += wm8750_codec.o
siu-sh7343-$(CONFIG_WM8978_CODEC) += wm8978_codec.o
siu-sh7343-$(CONFIG_TLV320AIC_CODEC) += tlv320aic23_codec.o

obj-$(CONFIG_SH_SIOF) += siof.o
siof-snd-y := siof_snd.c
