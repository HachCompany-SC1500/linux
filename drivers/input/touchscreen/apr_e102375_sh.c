/*
 * apr_e102375_sh.c   Touch Driver specific to SH7723
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input/apr_e102375_sh.h>

/* SH7723 MSIOF interface */
#define SITMDR1		0x00
#define SITMDR2	        0x04
#define SITMDR3		0x08
#define SIRMDR1		0x10
#define SIRMDR2		0x14
#define SIRMDR3		0x18
#define SITSCR		0x20
#define SIRSCR		0x24
#define SICTR		0x28
#define SIFCTR		0x30
#define SISTR		0x40
#define SIIER		0x44
#define SITDR1		0x48
#define SITDR2		0x4C
#define SITFDR		0x50
#define SIRDR1		0x58
#define SIRDR2		0x5C
#define SIRFDR		0x60

#define SITMDR1_TRMD_SLAVE	(0 << 31)
#define SITMDR1_PCON		(1 << 30)
#define SITMDR1_SYNCMD_LR	(3 << 28)
#define SIRMDR1_TRMD_SLAVE	(0 << 31)
#define SIRMDR1_SYNCMD_LR	(3 << 28)

#define SITMDR2_GRP(v)		((((v)-1)&3)    << 30)
#define SITMDR2_BITLEN1(v)	((((v)-1)&0x1F) << 24)
#define SITMDR3_BITLEN2(v)	((((v)-1)&0x1F) << 24)

#define SIRMDR2_GRP(v)		((((v)-1)&3)    << 30)
#define SIRMDR2_BITLEN1(v)	((((v)-1)&0x1F) << 24)
#define SIRMDR3_BITLEN2(v)	((((v)-1)&0x1F) << 24)

#define SICTR_RXE		(1 << 8)
#define SICTR_RXRST		(1 << 0)

#define SISTR_RDREQ		(1 << 12)
#define SISTR_RFOVF		(1 <<  3)
#define SIIER_RDREQ		(1 << 12)

#define SIFCTR_RFWM_1		(0 << 13)

/* APR E102375 interface */

#define APR_MAX_POS		4095
/* to be compatible with most other drivers who report a pressure >= 1 and
   require "module pthres pmin=1" in their /etc/ts.conf. Technically we only
   have 0 or 1  */
#define APR_PRESSURE_MAX	255
#define APR_STATE_RESERVED	0
#define APR_STATE_INITIAL	1
#define APR_STATE_STREAM	2
#define APR_STATE_UNTOUCH	3

/**
 * union apr_e102375_msg - stores one message from the APR
 */
union apr_e102375_msg {
        u16 val;
        struct {
#if defined(__LITTLE_ENDIAN)
                u16 pos    : 12;
                u16 state  : 2;
                u16 y_axis : 1;
                u16 cmd    : 1;
#else
                u16 cmd    : 1;
                u16 y_axis : 1;
                u16 state  : 2;
                u16 pos    : 12;
#endif
        } b;
};

/**
 * apr_e102375_priv - driver instance private data
 */
struct apr_e102375_priv {
	struct platform_device *pdev;
        struct input_dev       *input_dev;
        struct clk             *clk;
        struct tasklet_struct   tasklet;
	spinlock_t		lock;
        void __iomem           *base;
        int    		        irq;

        union apr_e102375_msg   last_x; /* stores the last received x position */
        int 		        last_x_is_valid;

        /* x/y are transmitted in their own message */
        struct {
                union apr_e102375_msg x;
                union apr_e102375_msg y;
        } pos;

#ifdef CONFIG_TOUCHSCREEN_APR_E102375_SH_DEBUG
	struct {
		struct {
			struct dentry *dir;
			struct dentry *stat;
			struct dentry *data;
		} fs;

		struct {
			int cmds;
			int events;
			int mismatches;
			int x_followed_by_x;
			int y_followed_by_y;
			int fifo_overflow;
		} stat;

		struct {
			/* implement a ringbuffer. effective size is decreased by
			   one as first is required to be != next if data is
			   present */
			int msg[4096];
			int first;
			int next;
		} data;
	} dbg;
#endif
};

#ifdef CONFIG_TOUCHSCREEN_APR_E102375_SH_DEBUG
# include <linux/debugfs.h>
# include <linux/seq_file.h>
# define DEBUG_STAT_INC_VAR_LOCK(priv, name)		\
	do { \
		unsigned long flags; \
		spin_lock_irqsave(&priv->lock, flags); \
		priv->dbg.stat.name++; \
		spin_unlock_irqrestore(&priv->lock, flags); \
	} while(0)
# define DEBUG_STAT_INC_VAR(priv, name)		\
	do { \
		priv->dbg.stat.name++; \
	} while(0)

static inline void apr_e102375_sh_dbg_msg_append(struct apr_e102375_priv *priv, int msg)
{
	priv->dbg.data.msg[priv->dbg.data.next++] = msg;
	
	/* ringbuffer wrap around  */
	if (priv->dbg.data.next == ARRAY_SIZE(priv->dbg.data.msg))
		priv->dbg.data.next = 0;

	if (priv->dbg.data.first == priv->dbg.data.next) {
		/* ringbuffer overflow, remove first message  */
		priv->dbg.data.first++;
		if (priv->dbg.data.first == ARRAY_SIZE(priv->dbg.data.msg))
			priv->dbg.data.first = 0;
	}
}

# define DEBUG_MSG_APPEND(priv, msg) apr_e102375_sh_dbg_msg_append(priv, msg)

static int apr_e102375_debugfs_stats_show(struct seq_file *file, void *iter)
{
	struct apr_e102375_priv *priv = file->private;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	seq_printf(file, "Commands: %i\n", priv->dbg.stat.cmds);
	seq_printf(file, "Events: %i\n",   priv->dbg.stat.events);
	seq_printf(file, "Mismatches: %i\n",   priv->dbg.stat.mismatches);
	seq_printf(file, "X-Followed-By-X: %i\n",   priv->dbg.stat.x_followed_by_x);
	seq_printf(file, "Y-Followed-By-Y: %i\n",   priv->dbg.stat.y_followed_by_y);
	seq_printf(file, "FIFO Overflow: %i\n",   priv->dbg.stat.fifo_overflow);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int apr_e102375_debugfs_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, apr_e102375_debugfs_stats_show, inode->i_private);
}

static const struct file_operations apr_e102375_debugfs_stat_fops = {
	.owner		= THIS_MODULE,
	.open		= apr_e102375_debugfs_stat_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int apr_e102375_debugfs_data_show(struct seq_file *file, void *iter)
{
	struct apr_e102375_priv *priv = file->private;
	unsigned long flags;
	int i;
	int j;
	
	spin_lock_irqsave(&priv->lock, flags);

	if (priv->dbg.data.first == priv->dbg.data.next)
		/*  no data yet */
		goto out;

	i = priv->dbg.data.first;

	if (priv->dbg.data.first<priv->dbg.data.next)
		/* no data yet or not wrapped around */
		j = priv->dbg.data.next;
	else
		/* go to end of ringbuffer and wrap around */
		j = ARRAY_SIZE(priv->dbg.data.msg);

	while (i<j)
		seq_printf(file, "0x%08x\n", priv->dbg.data.msg[i++]);

	if (priv->dbg.data.first > priv->dbg.data.next) {
	        /* wrapped around */
		i = 0;
		while (i<priv->dbg.data.next)
			seq_printf(file, "0x%08x\n", priv->dbg.data.msg[i++]);
	}

	/* reset ringbuffer */
	priv->dbg.data.first = 0;
	priv->dbg.data.next = 0;

out:
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int apr_e102375_debugfs_data_open(struct inode *inode, struct file *file)
{
	return single_open(file, apr_e102375_debugfs_data_show, inode->i_private);
}

static const struct file_operations apr_e102375_debugfs_data_fops = {
	.owner		= THIS_MODULE,
	.open		= apr_e102375_debugfs_data_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void apr_e102375_debugfs_remove(struct apr_e102375_priv *priv)
{
	if (priv->dbg.fs.data) {
		debugfs_remove(priv->dbg.fs.data);
		priv->dbg.fs.data = NULL;
	}
	
	if (priv->dbg.fs.stat) {
		debugfs_remove(priv->dbg.fs.stat);
		priv->dbg.fs.stat = NULL;
	}
	
	if (priv->dbg.fs.dir) {
		debugfs_remove(priv->dbg.fs.dir);
		priv->dbg.fs.dir = NULL;
	}
}

static int apr_e102375_debugfs_init(struct apr_e102375_priv *priv)
{
	struct dentry *dentry;
	
	dentry = debugfs_create_dir("apr_e102375_sh", NULL);
	if (!dentry || IS_ERR(dentry))
		goto error;
	priv->dbg.fs.dir = dentry;
	
	dentry = debugfs_create_file("stat", S_IRUSR,
				     priv->dbg.fs.dir,
				     priv,
				     &apr_e102375_debugfs_stat_fops);
	if (!dentry || IS_ERR(dentry))
		goto error;
	priv->dbg.fs.stat = dentry;

	dentry = debugfs_create_file("data", S_IRUSR,
				     priv->dbg.fs.dir,
				     priv,
				     &apr_e102375_debugfs_data_fops);
	if (!dentry || IS_ERR(dentry))
		goto error;
	priv->dbg.fs.data = dentry;

        return 0;

error:
	dev_err(&priv->pdev->dev, "failed to create debugfs\n");

	apr_e102375_debugfs_remove(priv);

	return dentry ? PTR_ERR(dentry) : -ENOMEM;
}

#else
# define DEBUG_STAT_INC_VAR_LOCK(priv, name) do {} while(0)
# define DEBUG_STAT_INC_VAR(priv, name) do {} while(0)
# define DEBUG_MSG_APPEND(priv, msg) do {} while(0)
#endif

/* must be used in a locked environment  */
#define DEBUG_STAT_INC_CMD(priv) DEBUG_STAT_INC_VAR(priv, cmds)
#define DEBUG_STAT_INC_EVENT(priv) DEBUG_STAT_INC_VAR(priv, events)
#define DEBUG_STAT_INC_X_FOLLOWED_BY_X(priv) DEBUG_STAT_INC_VAR(priv, x_followed_by_x)
#define DEBUG_STAT_INC_Y_FOLLOWED_BY_Y(priv) DEBUG_STAT_INC_VAR(priv, y_followed_by_y)
#define DEBUG_STAT_INC_FIFO_OVERFLOW(priv) DEBUG_STAT_INC_VAR(priv, fifo_overflow)
/* must be used in an unlocked environment  */
#define DEBUG_STAT_INC_MISMATCHES_LOCK(priv) DEBUG_STAT_INC_VAR_LOCK(priv, mismatches)


static inline void apr_write32(struct apr_e102375_priv *priv, u32 val, int reg)
{
        iowrite32(val, priv->base+reg);
}

static inline u32 apr_read32(struct apr_e102375_priv *priv, int reg)
{
        return ioread32(priv->base+reg);
}

/**
 * apr_reset - reset's the communication with the APR(SH7723 MSIOF interface)
 */
static void apr_reset(struct apr_e102375_priv *priv)
{
        static int TIMEOUT = 10000; /* no values available in datasheet */
        int i = 0;

        apr_write32(priv, SICTR_RXRST, SICTR);

        /* wait until reset is completed */
        while (apr_read32(priv, SICTR) & SICTR_RXRST) {
                if (unlikely(i >= TIMEOUT)) {
                        /* chip internal feature should always complete */
                        dev_err(&priv->pdev->dev, "apr_reset timeout\n");
                        break;
                }

                i++;
                udelay(1);
        }
}

/**
 * apr_init - initializes the communication with the APR by SH7723 MSIOF/SPI
 */
static void apr_init(struct apr_e102375_priv *priv)
{
        /* Configure SH7723 MSIOF to receive data from APR */
        apr_reset(priv);

        apr_write32(priv,
                    SITMDR1_TRMD_SLAVE |
                    SITMDR1_PCON |
                    SITMDR1_SYNCMD_LR, SITMDR1);
        apr_write32(priv,
                    SITMDR2_GRP(2) |
                    SITMDR2_BITLEN1(16), SITMDR2);
        apr_write32(priv,
                    SITMDR3_BITLEN2(16), SITMDR3);
        apr_write32(priv,
                    SIRMDR1_TRMD_SLAVE |
                    SIRMDR1_SYNCMD_LR, SIRMDR1);
        apr_write32(priv,
                    SIRMDR2_GRP(1) |
                    SIRMDR2_BITLEN1(16), SIRMDR2);
        apr_write32(priv,
                    SIRMDR3_BITLEN2(16), SIRMDR3);
        apr_write32(priv,
                    SIFCTR_RFWM_1,
                    SIFCTR);
}

/**
 * apr_e102375_handle_event - processes x/y positions to input layer
 */
static void apr_e102375_handle_event(unsigned long data)
{
        struct apr_e102375_priv *priv = (struct apr_e102375_priv*) data;
        union apr_e102375_msg x,y;
        unsigned long flags;
        int pressed;

        /* retrieve a valid x/y position */
        spin_lock_irqsave(&priv->lock, flags);
        x = priv->pos.x;
        y = priv->pos.y;
        spin_unlock_irqrestore(&priv->lock, flags);

        /* check for correctness */
        if (x.b.state != y.b.state) {
                dev_info(&priv->pdev->dev, "APR reports different states for X (0x%x) and Y (0x%x)\n", x.b.state, y.b.state);
		DEBUG_STAT_INC_MISMATCHES_LOCK(priv);
                return;
        }
        if (x.b.state == APR_STATE_RESERVED) {
                dev_err(&priv->pdev->dev, "APR reports reserved state 0\n");
		DEBUG_STAT_INC_MISMATCHES_LOCK(priv);
                return;
        }

        /* report to input layer */
        pressed = !(x.b.state == APR_STATE_UNTOUCH);

        input_report_key(priv->input_dev, BTN_TOUCH, pressed);
        input_report_abs(priv->input_dev, ABS_X, x.b.pos);
        input_report_abs(priv->input_dev, ABS_Y, y.b.pos);
        input_report_abs(priv->input_dev, ABS_PRESSURE, pressed ? APR_PRESSURE_MAX : 0);
        input_sync(priv->input_dev);
}


/**
 * apr_e102375_isr - handles Sh7723 MSIOF interrupts == APR messages
 */
static irqreturn_t apr_e102375_isr(int irq, void *dev_id)
{
        struct apr_e102375_priv *priv = dev_id;
        u32 status;

        spin_lock(&priv->lock);

        status = apr_read32(priv, SISTR);
        apr_write32(priv, status, SISTR); /* on interrupt, status is always != 0 */

        /* read all APR messages */
        while (status & SISTR_RDREQ) {
                /* data received */
                union apr_e102375_msg msg;
                u32 data = apr_read32(priv, SIRFDR);
                msg.val = data >> 16;

		DEBUG_MSG_APPEND(priv, data);

                if (unlikely(status & SISTR_RFOVF)) {
			DEBUG_STAT_INC_FIFO_OVERFLOW(priv);
                        dev_err(&priv->pdev->dev, "APR FIFO overflow\n");
                        /* we might have missed an X message */
                        priv->last_x_is_valid = 0;
                }

                if (!msg.b.cmd) {
			DEBUG_STAT_INC_EVENT(priv);
                        /* we are interested only in touch messages */
                        /* the APR reports first X, then Y at least 1ms
                         * later. So when we have received Y, we have a
                         * complete touch event and can forward it to the
                         * input layer.*/
                        if (msg.b.y_axis) {
                                if (likely(priv->last_x_is_valid)) {
                                        priv->pos.y = msg;
                                        priv->pos.x = priv->last_x;
                                        priv->last_x_is_valid = 0;
                                        tasklet_schedule(&priv->tasklet);
                                } else {
					DEBUG_STAT_INC_Y_FOLLOWED_BY_Y(priv);
				}
                        } else {
				if (unlikely(priv->last_x_is_valid)) {
					DEBUG_STAT_INC_X_FOLLOWED_BY_X(priv);
				}
				
                                priv->last_x = msg;
                                priv->last_x_is_valid = 1;
                        }
                } else {
			static int already_initialized = 0;
			if (!already_initialized) {
				/* print the message only one time to avoid an message flood */
				dev_info(&priv->pdev->dev, "Commands from APR are not supported. No further messages are pritned\n");
				already_initialized = 1;
			}

			DEBUG_STAT_INC_CMD(priv);
		}

                status = apr_read32(priv, SISTR);
                if (status)
                        apr_write32(priv, status, SISTR);
        }

        spin_unlock(&priv->lock);

        return IRQ_HANDLED;
}

/**
 * apr_e102375_open - open and power-up the APR device
 */
static int apr_e102375_open(struct input_dev *dev)
{
        struct apr_e102375_priv *priv = input_get_drvdata(dev);
        struct apr_e102375_sh_platform_data *pdata = dev_get_platdata(&priv->pdev->dev);
        unsigned long flags;
        int error;

        if (request_irq(priv->irq, apr_e102375_isr, IRQF_SAMPLE_RANDOM | IRQF_DISABLED, dev_name(&priv->pdev->dev), priv)) {
                dev_err(&priv->pdev->dev, "cannot get interrupt\n");
                error = -ENOMEM;
                goto error;
        }

        clk_enable(priv->clk);

        if (pdata->gpio_power_down >= 0)
                gpio_set_value(pdata->gpio_power_down, 1);

        apr_init(priv);

        priv->last_x_is_valid = 0;

        spin_lock_irqsave(&priv->lock, flags);
        /* enable interrupts */
        apr_write32(priv,
                    SIIER_RDREQ,
                    SIIER);

        /* enable receiver */
        apr_write32(priv,
                    apr_read32(priv, SICTR) | SICTR_RXE,
                    SICTR);

        spin_unlock_irqrestore(&priv->lock, flags);

        return 0;

error:
        return -error;
}

/**
 * apr_e102375_close - close and power-down the APR device
 */
static void apr_e102375_close(struct input_dev *dev)
{
        struct apr_e102375_priv *priv = input_get_drvdata(dev);
        struct apr_e102375_sh_platform_data *pdata = dev_get_platdata(&priv->pdev->dev);
        unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
        /* disable receiver */
        apr_write32(priv,
                    apr_read32(priv, SICTR) & ~SICTR_RXE,
                    SICTR);

        /* disable interrupts */
        apr_write32(priv,
                    0,
                    SIIER);

        tasklet_kill(&priv->tasklet);
        spin_unlock_irqrestore(&priv->lock, flags);

        if (pdata->gpio_power_down >= 0)
                gpio_set_value(pdata->gpio_power_down, 0);

        clk_disable(priv->clk);
        free_irq(priv->irq, priv);
}

/**
 * apr_e102375_probe - checks and initializes ressources needed for APR E102375
 */
static int __devinit apr_e102375_probe(struct platform_device *pdev)
{
        struct apr_e102375_sh_platform_data *pdata = dev_get_platdata(&pdev->dev);
        struct apr_e102375_priv *priv;
 	struct resource  *res;
        int error;

        if (!pdata) {
                dev_err(&pdev->dev, "Need platform data\n");
                error = -EINVAL;
                goto error;
        }

        priv = kzalloc(sizeof(*priv), GFP_KERNEL);
        if (!priv) {
                dev_err(&pdev->dev, "cannot get platform memory\n");
                error = -EINVAL;
                goto error;
        }
        platform_set_drvdata(pdev, priv);
        priv->pdev = pdev;
	spin_lock_init(&priv->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!res) {
                dev_err(&pdev->dev, "cannot get platform memory\n");
                error = -EINVAL;
                goto error_priv;
        }

        priv->base = ioremap(res->start, (res->end-res->start)+1);
        if (!priv->base) {
                dev_err(&pdev->dev, "cannot get io memory\n");
                error = -ENOMEM;
                goto error_priv;
        }

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!res) {
                dev_err(&pdev->dev, "cannot get platform irq\n");
                error = -EINVAL;
                goto error_priv;
        }
        priv->irq = res->start;

        priv->clk = clk_get(&pdev->dev, pdata->clock);
        if (IS_ERR(priv->clk)) {
                dev_err(&pdev->dev, "cannot get clock");
                error = -ENODEV;
                goto error_irq;
        }

        if (pdata->gpio_power_down >= 0) {
                if (gpio_request(pdata->gpio_power_down, "apr_pd#")) {
                        dev_err(&pdev->dev, "GPIO %i not available\n", pdata->gpio_power_down);
                        error = -ENODEV;
                        goto error_irq;
                }
                gpio_direction_output(pdata->gpio_power_down, 0);
        }

        priv->input_dev = input_allocate_device();
        if (!priv->input_dev) {
                dev_err(&pdev->dev, "can't allocate device\n");
                error = -ENOMEM;
                goto error_gpio;
        }
	input_set_drvdata(priv->input_dev, priv);

        priv->input_dev->name = "APR E102375";

        priv->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
        priv->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(priv->input_dev, ABS_X, 0, APR_MAX_POS, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_Y, 0, APR_MAX_POS, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_PRESSURE, 0, APR_PRESSURE_MAX, 0, 0);

        priv->input_dev->open  = apr_e102375_open;
        priv->input_dev->close = apr_e102375_close;

        error = input_register_device(priv->input_dev);
        if (error)
                goto error_gpio;

#ifdef CONFIG_TOUCHSCREEN_APR_E102375_SH_DEBUG
	dev_info(&pdev->dev, "compiled with debugging information\n");
	error = apr_e102375_debugfs_init(priv);
	if (error) {
		input_unregister_device(priv->input_dev);
		goto error_gpio;
	}
#endif

        tasklet_init(&priv->tasklet, apr_e102375_handle_event, (unsigned long) priv);

	dev_info(&pdev->dev, "registered touchscreen\n");

        return 0;

error_gpio:
        if (pdata->gpio_power_down >= 0)
                /* keep it powered down */
                gpio_free(pdata->gpio_power_down);

error_irq:
        if (priv->input_dev)
                input_free_device(priv->input_dev);

        if (priv->clk)
                clk_put(priv->clk);

error_priv:
        if (priv->base)
                iounmap(priv->base);

        kfree(priv);

error:
        return error;
}

/**
 * apr_e102375_remove - removes all resources needed for the APR E102375
 */
static int __devexit apr_e102375_remove(struct platform_device *pdev)
{
	struct apr_e102375_priv *priv = platform_get_drvdata(pdev);
        struct apr_e102375_sh_platform_data *pdata = dev_get_platdata(&pdev->dev);

#ifdef CONFIG_TOUCHSCREEN_APR_E102375_SH_DEBUG
	apr_e102375_debugfs_remove(priv);
#endif

        if (pdata->gpio_power_down >= 0)
                /* keep it powered down */
                gpio_free(pdata->gpio_power_down);

        input_unregister_device(priv->input_dev);
        input_free_device(priv->input_dev);

        clk_put(priv->clk);
        iounmap(priv->base);
        kfree(priv);

	dev_dbg(&pdev->dev, "unregistered touchscreen\n");

        return 0;
}

static struct platform_driver apr_e102375_driver = {
	.probe		= apr_e102375_probe,
	.remove		= apr_e102375_remove,
	.driver		= {
		.name	= "apr_e102375_sh",
		.owner	= THIS_MODULE,
	},
};

static __init int apr_e102375_init(void)
{
	return platform_driver_register(&apr_e102375_driver);
}
module_init(apr_e102375_init);

static __exit void apr_e102375_exit(void)
{
	platform_driver_unregister(&apr_e102375_driver);
}
module_exit(apr_e102375_exit);

MODULE_DESCRIPTION("APR E102375 touch driver");
MODULE_AUTHOR("Markus Pietrek");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:apr_e102375_sh");
