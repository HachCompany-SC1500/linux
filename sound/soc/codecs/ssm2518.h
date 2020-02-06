/*
 * ALSA SoC TLV320AIC23 codec driver
 *
 * Author:      Arun KS, <arunks@mistralsolutions.com>
 * Copyright:   (C) 2008 Mistral Solutions Pvt Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SSM2518_H
#define _SSM2518_H

/* Codec SSM2518 */

#define SSM2518_RESET                   0x00
#define SSM2518_CLOCK_CTRL              0x01
#define SSM2518_SER_SRATE               0x02
#define SSM2518_SER_CTRL                0x03
#define SSM2518_CHAN_MAP                0x04
#define SSM2518_LVOL_CTRL               0x05
#define SSM2518_RVOL_CTRL               0x06
#define SSM2518_VOL_MUTE_CTRL           0x07
#define SSM2518_FAULT_CTRL              0x08
#define SSM2518_PWR_FAULT_CTRL          0x09
#define SSM2518_DRC_1                   0x0a
#define SSM2518_DRC_2                   0x0b
#define SSM2518_DRC_3                   0x0c
#define SSM2518_DRC_4                   0x0d
#define SSM2518_DRC_5                   0x0e
#define SSM2518_DRC_6                   0x0f
#define SSM2518_DRC_7                   0x10
#define SSM2518_DRC_8                   0x11
#define SSM2518_DRC_9                   0x12
#define SSM2518_TEST                    0x16



#define SSM2518_SPWDN                   0x01
#define SSM2518_MCS_64                  0x00
#define SSM2518_MCS_128                 0x02
#define SSM2518_MCS_256                 0x04
//#define SSM2518_MCS_384                 0x06
#define SSM2518_MCS_384                 0x06
#define SSM2518_MCS_512                 0x08
#define SSM2518_MCS_768                 0x0a
#define SSM2518_MCS_MASK                0x0e
#define SSM2518_S_RST                   0x80



#define SSM2518_MASTER_MUTE		0x01
#define SSM2518_L_MUTE			0x02
#define SSM2518_R_MUTE			0x04

#define SSM2518_L_PWDN			0x02
#define SSM2518_R_PWDN			0x04
#define SSM2518_LP_MODE                 0x08
#define SSM2518_AR_TIME_10              0x00
#define SSM2518_AR_TIME_20              0x40
#define SSM2518_AR_TIME_40              0x80
#define SSM2518_AR_TIME_80              0xc0


/* Serial Audio Interface and Sample Rate Control Register */
#define SSM2518_SDATA_I2S               0x00
#define SSM2518_SDATA_LEFT_JUST         0x20
#define SSM2518_SDATA_RIGHT_24          0x40
#define SSM2518_SDATA_RIGHT_16          0x60
#define SSM2518_SDATA_MASK              0x60




extern struct snd_soc_dai ssm2518_dai;
extern struct snd_soc_codec_device soc_codec_dev_ssm2518;

#endif /* _TLV320AIC23_H */
