/*
 * gpio_print_enum.c
 *
 * Copyright (c) 2009 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 * Description: Provides debugfs/sh/gpio_print_enum
 *
 **/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>

extern struct pinmux_info sh7723_pinmux_info;
static struct pinmux_info *pinmux = &sh7723_pinmux_info;

static int gpio_print_enum_debugfs_seq_show(struct seq_file *file,
                                        void *iter)
{
        int i;

        for (i=pinmux->first_gpio; i < pinmux->last_gpio; i++) {
                seq_printf(file, "%4i=%s\n",
                           i,
                           pinmux->gpios[i].name);
        }

	return 0;
}

static int gpio_print_enum_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpio_print_enum_debugfs_seq_show, inode->i_private);
}

static const struct file_operations gpio_print_enum_debugfs_fops = {
	.owner		= THIS_MODULE,
	.open		= gpio_print_enum_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int gpio_print_enum_debugfs_init(void)
{
	struct dentry *dentry;

	dentry = debugfs_create_file("gpio_print_enum", S_IRUSR, sh_debugfs_root,
                                     NULL,
                                     &gpio_print_enum_debugfs_fops);

	if (!dentry)
		return -ENOMEM;
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

        return 0;
}
module_init(gpio_print_enum_debugfs_init);
MODULE_LICENSE("GPL v2");
