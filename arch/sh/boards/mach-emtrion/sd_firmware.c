/*
 * sd_firmware.c
 *
 * Copyright (c) 2009 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 * Description: Copies the SD firmware to a local buffer.
 *              We could have used reserve_bootmem(), but that would have
 *              required up to two pages
 * References:
 *   [1] file://U-Boot/extbsp/sd_firmware.c
 *
 **/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/setup.h>
#include <asm/io.h>

// stores a local copy firmware, 32bit aligned
static int sd_firmware_code[2080/4];

void* __init sd_firmware_init(void)
{
        if (!SD_FIRMWARE_START || !SD_FIRMWARE_SIZE)
                return NULL;

        if (SD_FIRMWARE_SIZE > sizeof(sd_firmware_code)) {
                pr_err("SD firmware bigger than internal buffer\n");
                return NULL;
        }

        memcpy(sd_firmware_code, (void*) SD_FIRMWARE_START, SD_FIRMWARE_SIZE);
        pr_debug("SD firmware copied\n");

        return sd_firmware_code;
}
