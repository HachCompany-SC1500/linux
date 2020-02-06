/*
 * em_firmware.c
 *
 * Copyright (c) 2010 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 * Descr:       Some drivers (e.g. audio) might require the firmware before the flash driver is
 *              available but are not GPL, therefore can't be linked into the kernel.
 *              To avoid compiling them as modules which might require initrd/initramfs as well,
 *              the bootloader is able to give the firmware images to the kernel in RAM.
 *
 **/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/bootmem.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <asm/io.h>

#include "firmware.h"

extern struct builtin_fw *__start_builtin_fw_bootloader;
extern struct builtin_fw *__end_builtin_fw_bootloader;

static firmware_t *first;

static int __init em_firmware_merge_with_builtin(void)
{
	struct builtin_fw *new_start;
	struct builtin_fw *bootloader_start;
	int n_bootloader = 0;
	int n_builtin = __end_builtin_fw_bootloader-__start_builtin_fw_bootloader;
	firmware_t *p;
	int len;

	if (!first)
		/* nothing do do */
		return 0;

	p = first;
	while (!firmware_at_end(p)) {
		n_bootloader++;
		p = firmware_next(p);
	}

	if (!n_bootloader)
		/* no need for merge */
		return 0;

	len = (n_builtin+n_bootloader)*sizeof(struct builtin_fw);

	new_start = kmalloc(len, GFP_KERNEL);
	if (!new_start) {
		pr_err("Unable to merge builtin firmware with bootloader firmware\n");
		return -ENOMEM;
	}

	/* relocate linked builtin firmware definition  */
	memcpy(new_start, __start_builtin_fw_bootloader, sizeof(struct builtin_fw)*n_builtin);

	bootloader_start = new_start + n_builtin;

	memset(bootloader_start, 0, sizeof(struct builtin_fw)*n_bootloader);

	/* tell linux to use the merged firmware list */

	__start_builtin_fw_bootloader = new_start;
	__end_builtin_fw_bootloader   = __start_builtin_fw_bootloader;

	/* append bootloader firmware to linked builtin */
	p = first;
	while (!firmware_at_end(p)) {
		pr_info("Merging firmware '%s'\n", p->name);

		__end_builtin_fw_bootloader->name = p->name;
		__end_builtin_fw_bootloader->data = p->data;
		__end_builtin_fw_bootloader->size = p->len;

		p = firmware_next(p);
		__end_builtin_fw_bootloader++;
	}

	return 0;
}

/* we need to reserve the firmware images from memory setup by the bootloader.
   Therefore we can't use _initcall as the memory has already been initialized when they are processed */
int __init em_firmware_init(unsigned long firmware_phys_start, unsigned long len)
{
	if (!firmware_phys_start || !len)
		return 0;

	pr_info("Firmware is taken from bootloader\n");

	if (firmware_phys_start >= PFN_PHYS(max_pfn)) {
		void *low_mem;

		/* Framebuffer is beyond memory accessable to linux,
		 * e.g. directfb=hw or mem=120 and framebuffer placed at 125MB.
		 * Create a new buffer in linux memory and copy image */
		low_mem = alloc_bootmem_pages_nopanic(len);
		if (low_mem != NULL) {
			memcpy(low_mem, phys_to_virt(firmware_phys_start), len);
			first = low_mem;
		}
	} else if (!reserve_bootmem(firmware_phys_start, len, BOOTMEM_DEFAULT))
		first = phys_to_virt(firmware_phys_start);

	if (!first)
		pr_err("Can't reserve firmware memory\n");

	return 0;
}

early_initcall(em_firmware_merge_with_builtin);

