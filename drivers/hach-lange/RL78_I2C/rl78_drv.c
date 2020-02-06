/*
    i2c-dev.c - i2c-bus driver, char device interface

    Copyright (C) 1995-97 Simon G. Vogl
    Copyright (C) 1998-99 Frodo Looijaard <frodol@dds.nl>
    Copyright (C) 2003 Greg Kroah-Hartman <greg@kroah.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* Note that this is a complete rewrite of Simon Vogl's i2c-dev module.
   But I have used so much of his original code and ideas that it seems
   only fair to recognize him as co-author -- Frodo */

/* The I2C_RDWR ioctl code is written by Kolja Waschk <waschk@telos.de> */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <sh7723.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <linux/rl78_drv.h>
#include <linux/poll.h>

/* SH7723 registers */
#define FRQCR		0xa4150000
#define VCLKCR		0xa4150004
#define SCLKACR		0xa4150008
#define SCLKBCR		0xa415000c
#define IRDACLKCR	0xa4150018
#define PLLCR		0xa4150024
#define DLLFRQ		0xa4150050



/*
 * wait queue for RL78 request signalisation via IRQ2+request lines
 */
DECLARE_WAIT_QUEUE_HEAD( rl78_irq_queue );

// irq flags
#define RL78_IRQ_NMI  0x1
#define RL78_IRQ      0x2

static struct i2c_driver i2cdev_driver;

static irqreturn_t rl78_irq_handler( int irq, void* ptr );
static int __init rl78_req_line_init( void );
static int8_t rl78_req_line_read( int8_t req_line );

static int rl78_irq_open( struct inode *inode, struct file *file );
static unsigned int rl78_irq_poll( struct file *file, struct poll_table_struct *poll_table );
static ssize_t rl78_irq_read( struct file *file, char __user *buf, size_t count, loff_t *offset );
static int rl78_irq_release( struct inode *inode, struct file *file );


static int rl78_i2c_open( struct inode *inode, struct file *file );
static ssize_t rl78_i2c_write( struct file *file, const char __user *buf, size_t count, loff_t *offset );
static ssize_t rl78_i2c_read( struct file *file, char __user *buf, size_t count, loff_t *offset );
static long rl78_i2c_ioctl( struct file *file, unsigned int cmd, unsigned long arg );
static int rl78_i2c_release( struct inode *inode, struct file *file );


static const struct file_operations rl78_i2c_fops = {
	.owner = THIS_MODULE,
	//    .llseek       = no_llseek,
	.read = rl78_i2c_read,
	.write = rl78_i2c_write,
	.unlocked_ioctl = rl78_i2c_ioctl,
	.open = rl78_i2c_open,
	.release = rl78_i2c_release,
};


static const struct file_operations rl78_irq_fops = {
	.owner = THIS_MODULE,
	//    .llseek       = no_llseek,
	.read = rl78_irq_read,
	//   .write          = rl78_write,
	//    .unlocked_ioctl = rl78_i2c_ioctl,
	.open = rl78_irq_open,
	.release = rl78_irq_release,
	.poll = rl78_irq_poll,
};



static char rl78_irq_flags = 0;

/*
 * An i2c_dev represents an i2c_adapter ... an I2C or SMBus master, not a
 * slave (i2c_client) with which messages will be exchanged.  It's coupled
 * with a character special file which is accessed by user mode drivers.
 *
 * The list of i2c_dev structures is parallel to the i2c_adapter lists
 * maintained by the driver model, and is updated using notifications
 * delivered to the i2cdev_driver.
 */
struct i2c_dev {
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
};

#define I2C_MINORS  256
static LIST_HEAD( i2c_dev_list );
static DEFINE_SPINLOCK( i2c_dev_list_lock );


static struct i2c_dev *i2c_dev_get_by_minor( unsigned index )
{
	struct i2c_dev *i2c_dev;

	spin_lock( &i2c_dev_list_lock );

	list_for_each_entry( i2c_dev, &i2c_dev_list, list ) {
		if (i2c_dev->adap->nr == index) {
			goto found;
		}
		else {
			printk( "i2c_dev->adap->nr = %i\n", i2c_dev->adap->nr );
		}
	}
	i2c_dev = NULL;
found:
	spin_unlock( &i2c_dev_list_lock );
	return i2c_dev;
}


static struct i2c_dev *get_free_i2c_dev( struct i2c_adapter *adap )
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk( "%s Out of device minors (%d)\n", __FUNCTION__,
			adap->nr );
		return ERR_PTR( -ENODEV );
	}

	i2c_dev = kzalloc( sizeof (*i2c_dev), GFP_KERNEL );
	if (!i2c_dev)
		return ERR_PTR( -ENOMEM );
	i2c_dev->adap = adap;

	spin_lock( &i2c_dev_list_lock );
	list_add_tail( &i2c_dev->list, &i2c_dev_list );
	spin_unlock( &i2c_dev_list_lock );
	return i2c_dev;
}


static void return_i2c_dev( struct i2c_dev *i2c_dev )
{
	spin_lock( &i2c_dev_list_lock );
	list_del( &i2c_dev->list );
	spin_unlock( &i2c_dev_list_lock );
	kfree( i2c_dev );
}


static ssize_t show_adapter_name( struct device *dev,
				  struct device_attribute *attr, char *buf )
{
	struct i2c_dev *i2c_dev = i2c_dev_get_by_minor( MINOR( dev->devt ) );

	if (!i2c_dev)
		return -ENODEV;
	return sprintf( buf, "%s\n", i2c_dev->adap->name );
}
static DEVICE_ATTR( name, S_IRUGO, show_adapter_name, NULL );

/* ------------------------------------------------------------------------- */


/*********************************************************************************
 * rl78_read_irq
 * Reads i2c data from rl78 interface.
 *
 * Parameters:
 *   file   - pointer to file structure related to rl78 device.
 *   buf    - pointer to received data .
 *   count  - amount of bytes to be read.
 *   offset - unused.
 *
 * Return Values:
 *   <0        - in case of error,
 *   >0       - number of bytes received, in case of success.
 *
 * Remark:
 *   Functionality is blocking until wake up from rl78 interrupt via rl78_irq_queue.
 *
 *********************************************************************************/
static ssize_t rl78_irq_read( struct file *file, char __user *buf, size_t count,
			      loff_t *offset )
{
	int ret = 0;
	char req_lines = 0;

	//	printk("rl78_irq_read\n");

	//	tmp = kmalloc(count, GFP_KERNEL);
	//    if (tmp == NULL)
	//    {
	//		printk("rl78_irq_read - allocation error\n");
	//        return -ENOMEM;
	//    }

	req_lines = rl78_req_line_read( REQALL );
	//	req_lines = ((req_lines ^ 0x0F) & ~0xF0);
	//	tmp=&req_lines;

	//	printk("rl78_irq_read - copy_to_user\n");
	ret = copy_to_user( buf, &req_lines, count ) ? -EFAULT : ret;

	/* clear rl78 interrupt flag */
	rl78_irq_flags = rl78_irq_flags & ~RL78_IRQ;
	//	kfree(tmp);

	return ret;
}


/*********************************************************************************
 * rl78_read_irq
 * Reads i2c data from rl78 interface.
 *
 * Parameters:
 *   file   - pointer to file structure related to rl78 device.
 *   buf    - pointer to received data .
 *   count  - amount of bytes to be read.
 *   offset - unused.
 *
 * Return Values:
 *   <0        - in case of error,
 *   >0       - number of bytes received, in case of success.
 *
 * Remark:
 *   Functionality is blocking until wake up from rl78 interrupt via rl78_irq_queue.
 *
 *********************************************************************************/
static ssize_t rl78_i2c_read( struct file *file, char __user *buf, size_t count,
			      loff_t *offset )
{
	char *tmp;
	int ret = 0;
	//static int i2c_read_cnt = 1;

	struct i2c_client *client = file->private_data;

	if (count > 8192) {
		count = 8192;
	}

	tmp = kmalloc( count, GFP_KERNEL );
	if (tmp == NULL) {
		printk( "%s - allocation error\n", __FUNCTION__ );
		return -ENOMEM;
	}

	//printk( "%s - rl78_i2c-%d reading %zu bytes from addr 0x%02x (read #%i).\n", __FUNCTION__,
	//	iminor( file->f_path.dentry->d_inode ), count, client->addr, i2c_read_cnt++ );

	ret = i2c_master_recv( client, tmp, count );
	if (ret >= 0) {
		/*
		 * copy received data to user space and
		 * clear rl78 interrupt flag
		 */
		ret = copy_to_user( buf, tmp, count ) ? -EFAULT : ret;
	}
	else {
		dev_err( &client->adapter->dev,"error i2c_master_recv returned %i\n", ret );
	}

	kfree( tmp );
	return ret;
}


/*********************************************************************************
 * rl78_i2c_write
 * Writes i2c data to rl78 interface.
 *
 * Parameters:
 *   file   - pointer to file structure related to rl78 device.
 *   buf    - pointer to data to be send.
 *   count  - amount of bytes to be send.
 *   offset - unused.
 *
 * Return Values:
 *   <0       - in case of error,
 *   >0       - number of bytes sent, in case of success.
 *
 * Remark:
 *   Function is the same as i2cdev_write taken from i2c-dev driver.
 *
 *********************************************************************************/
static ssize_t rl78_i2c_write( struct file *file, const char __user *buf,
			       size_t count, loff_t *offset )
{
	int ret;
	char *tmp;
	struct i2c_client *client = file->private_data;

	if (count > 8192) {
		count = 8192;
	}

	tmp = kmalloc( count, GFP_KERNEL );
	if (tmp == NULL)
		return -ENOMEM;
	if (copy_from_user( tmp, buf, count )) {
		kfree( tmp );
		return -EFAULT;
	}

	//printk( "%s rl78_i2c-%d writing %zu bytes to addr 0x%02x\n", __FUNCTION__,
	//	iminor( file->f_path.dentry->d_inode ), count, client->addr );

	ret = i2c_master_send( client, tmp, count );
	if (ret < 0) {
		dev_err( &client->adapter->dev,"error i2c_master_send returned %i\n", ret );
	}
	kfree( tmp );
	return ret;
}

static int i2cdev_check( struct device *dev, void *addrp )
{
	struct i2c_client *client = i2c_verify_client( dev );

	if (!client || client->addr != *(unsigned int *) addrp)
		return 0;

	return dev->driver ? -EBUSY : 0;
}


/* This address checking function differs from the one in i2c-core
   in that it considers an address with a registered device, but no
   driver bound to it, as NOT busy. */
static int i2cdev_check_addr( struct i2c_adapter *adapter, unsigned int addr )
{
	return device_for_each_child( &adapter->dev, &addr, i2cdev_check );
}


static noinline int i2cdev_ioctl_rdrw( struct i2c_client *client,
				       unsigned long arg )
{
	struct i2c_rdwr_ioctl_data rdwr_arg;
	struct i2c_msg *rdwr_pa;
	u8 __user **data_ptrs;
	int i, res;

	if (copy_from_user( &rdwr_arg,
			    (struct i2c_rdwr_ioctl_data __user *) arg,
			    sizeof (rdwr_arg) ))
		return -EFAULT;

	/* Put an arbitrary limit on the number of messages that can
	 * be sent at once */
	if (rdwr_arg.nmsgs > I2C_RDRW_IOCTL_MAX_MSGS)
		return -EINVAL;

	rdwr_pa = (struct i2c_msg *)
		kmalloc( rdwr_arg.nmsgs * sizeof (struct i2c_msg),
			 GFP_KERNEL );
	if (!rdwr_pa)
		return -ENOMEM;

	if (copy_from_user( rdwr_pa, rdwr_arg.msgs,
			    rdwr_arg.nmsgs * sizeof (struct i2c_msg) )) {
		kfree( rdwr_pa );
		return -EFAULT;
	}

	data_ptrs = kmalloc( rdwr_arg.nmsgs * sizeof (u8 __user *), GFP_KERNEL );
	if (data_ptrs == NULL) {
		kfree( rdwr_pa );
		return -ENOMEM;
	}

	res = 0;
	for (i = 0; i < rdwr_arg.nmsgs; i++) {
		/* Limit the size of the message to a sane amount;
		 * and don't let length change either. */
		if ((rdwr_pa[i].len > 8192) ||
			(rdwr_pa[i].flags & I2C_M_RECV_LEN)) {
			res = -EINVAL;
			break;
		}
		data_ptrs[i] = (u8 __user *) rdwr_pa[i].buf;
		rdwr_pa[i].buf = kmalloc( rdwr_pa[i].len, GFP_KERNEL );
		if (rdwr_pa[i].buf == NULL) {
			res = -ENOMEM;
			break;
		}
		if (copy_from_user( rdwr_pa[i].buf, data_ptrs[i],
				    rdwr_pa[i].len )) {
			++i; /* Needs to be kfreed too */
			res = -EFAULT;
			break;
		}
	}
	if (res < 0) {
		int j;
		for (j = 0; j < i; ++j)
			kfree( rdwr_pa[j].buf );
		kfree( data_ptrs );
		kfree( rdwr_pa );
		return res;
	}

	res = i2c_transfer( client->adapter, rdwr_pa, rdwr_arg.nmsgs );
	while (i-- > 0) {
		if (res >= 0 && (rdwr_pa[i].flags & I2C_M_RD)) {
			if (copy_to_user( data_ptrs[i], rdwr_pa[i].buf,
					  rdwr_pa[i].len ))
				res = -EFAULT;
		}
		kfree( rdwr_pa[i].buf );
	}
	kfree( data_ptrs );
	kfree( rdwr_pa );
	return res;
}


static noinline int i2cdev_ioctl_smbus( struct i2c_client *client,
					unsigned long arg )
{
	struct i2c_smbus_ioctl_data data_arg;
	union i2c_smbus_data temp;
	int datasize, res;

	if (copy_from_user( &data_arg,
			    (struct i2c_smbus_ioctl_data __user *) arg,
			    sizeof (struct i2c_smbus_ioctl_data) ))
		return -EFAULT;
	if ((data_arg.size != I2C_SMBUS_BYTE) &&
		(data_arg.size != I2C_SMBUS_QUICK) &&
		(data_arg.size != I2C_SMBUS_BYTE_DATA) &&
		(data_arg.size != I2C_SMBUS_WORD_DATA) &&
		(data_arg.size != I2C_SMBUS_PROC_CALL) &&
		(data_arg.size != I2C_SMBUS_BLOCK_DATA) &&
		(data_arg.size != I2C_SMBUS_I2C_BLOCK_BROKEN) &&
		(data_arg.size != I2C_SMBUS_I2C_BLOCK_DATA) &&
		(data_arg.size != I2C_SMBUS_BLOCK_PROC_CALL)) {
		dev_err( &client->adapter->dev,
			 "size out of range (%x) in ioctl I2C_SMBUS.\n",
			 data_arg.size );
		return -EINVAL;
	}
	/* Note that I2C_SMBUS_READ and I2C_SMBUS_WRITE are 0 and 1,
	   so the check is valid if size==I2C_SMBUS_QUICK too. */
	if ((data_arg.read_write != I2C_SMBUS_READ) &&
		(data_arg.read_write != I2C_SMBUS_WRITE)) {
		dev_err( &client->adapter->dev,
			 "read_write out of range (%x) in ioctl I2C_SMBUS.\n",
			 data_arg.read_write );
		return -EINVAL;
	}

	/* Note that command values are always valid! */

	if ((data_arg.size == I2C_SMBUS_QUICK) ||
		((data_arg.size == I2C_SMBUS_BYTE) &&
		(data_arg.read_write == I2C_SMBUS_WRITE)))
		/* These are special: we do not use data */
		return i2c_smbus_xfer( client->adapter, client->addr,
				       client->flags, data_arg.read_write,
				       data_arg.command, data_arg.size, NULL );

	if (data_arg.data == NULL) {
		dev_err( &client->adapter->dev,
			 "data is NULL pointer in ioctl I2C_SMBUS.\n" );
		return -EINVAL;
	}

	if ((data_arg.size == I2C_SMBUS_BYTE_DATA) ||
		(data_arg.size == I2C_SMBUS_BYTE))
		datasize = sizeof (data_arg.data->byte);
	else if ((data_arg.size == I2C_SMBUS_WORD_DATA) ||
		(data_arg.size == I2C_SMBUS_PROC_CALL))
		datasize = sizeof (data_arg.data->word);
	else /* size == smbus block, i2c block, or block proc. call */
		datasize = sizeof (data_arg.data->block);

	if ((data_arg.size == I2C_SMBUS_PROC_CALL) ||
		(data_arg.size == I2C_SMBUS_BLOCK_PROC_CALL) ||
		(data_arg.size == I2C_SMBUS_I2C_BLOCK_DATA) ||
		(data_arg.read_write == I2C_SMBUS_WRITE)) {
		if (copy_from_user( &temp, data_arg.data, datasize ))
			return -EFAULT;
	}
	if (data_arg.size == I2C_SMBUS_I2C_BLOCK_BROKEN) {
		/* Convert old I2C block commands to the new
		   convention. This preserves binary compatibility. */
		data_arg.size = I2C_SMBUS_I2C_BLOCK_DATA;
		if (data_arg.read_write == I2C_SMBUS_READ)
			temp.block[0] = I2C_SMBUS_BLOCK_MAX;
	}
	res = i2c_smbus_xfer( client->adapter, client->addr, client->flags,
			      data_arg.read_write, data_arg.command, data_arg.size, &temp );
	if (!res && ((data_arg.size == I2C_SMBUS_PROC_CALL) ||
		(data_arg.size == I2C_SMBUS_BLOCK_PROC_CALL) ||
		(data_arg.read_write == I2C_SMBUS_READ))) {
		if (copy_to_user( data_arg.data, &temp, datasize ))
			return -EFAULT;
	}
	return res;
}


#ifdef I2C_BAUDRATE_CONFIG


/*********************************************************************************
 * i2cdev_set_baudrate
 * Programs the baudrate for the i2c interface.
 *
 * Parameters:
 *   baudrate - i2c baudrate in Hz.
 *              Possible values are 100000, 397100, 397600 and 400000.
 *
 * Return Values:
 *   -1       - in case of invalid baudrate passed.
 *   =0       - in case of successful configuration.
 *
 * Remark:
 *   Functionality is blocking until wake up from rl78 interrupt via rl78_irq_queue.
 *
 *********************************************************************************/
static int i2cdev_set_baudrate( unsigned long baudrate )
{
	char *baudrate_str;
	uint8_t iccl_val = readb( I2C_CCL );
	uint8_t icch_val = readb( I2C_CCH );
	uint32_t frqcr_val = readl( FRQCR );
	uint32_t vclkcr_val = readl( VCLKCR );
	uint32_t sclkacr_val = readl( SCLKACR );
	uint32_t sclkbcr_val = readl( SCLKBCR );
	uint32_t pllcr_val = readl( PLLCR );
	uint32_t irdaclkcr_val = readl( IRDACLKCR );
	uint32_t dllfrq_val = readl( DLLFRQ );
	int32_t curr_baudrate = -1;
	int32_t curr_per_clk = -1;

	int32_t per_clk = 0;

	printk( "%s - current I2C clock register values:\n", __FUNCTION__ );
	printk( "I2C_CCL = 0x%02X\n", iccl_val );
	printk( "I2C_CCH = 0x%02X\n", icch_val );

	printk( "Current general clock register values:\n" );
	printk( "frqcr_val     = 0x%08X\n", frqcr_val );
	printk( "vclkcr_val    = 0x%08X\n", vclkcr_val );
	printk( "sclkacr_val   = 0x%08X\n", sclkacr_val );
	printk( "sclkbcr_val   = 0x%08X\n", sclkbcr_val );
	printk( "pllcr_val     = 0x%08X\n", pllcr_val );
	printk( "irdaclkcr_val = 0x%08X\n", irdaclkcr_val );
	printk( "dllfrq_val    = 0x%08X\n", dllfrq_val );

	/* analyse current peripheral clock frequency */
	if ((iccl_val == 0x42 && icch_val == 0x36) ||
		(iccl_val == 0x12 && icch_val == 0x0C)) {
		curr_per_clk = 12000;
	}
	else if ((iccl_val == 0x4A && icch_val == 0x3D) ||
		(iccl_val == 0x16 && icch_val == 0x0C)) {
		curr_per_clk = 13500;
	}
	else if ((iccl_val == 0x82 && icch_val == 0x6E) ||
		(iccl_val == 0x26 && icch_val == 0x16)) {
		curr_per_clk = 24000;
	}
	else if ((iccl_val == 0x96 && icch_val == 0x78) ||
		(iccl_val == 0x2C && icch_val == 0x18)) {
		curr_per_clk = 27000;
	}
	else if ((iccl_val == 0xAF && icch_val == 0x9B) ||
		(iccl_val == 0x33 && icch_val == 0x20)) {
		curr_per_clk = 33000;
	}
	else {
		printk( "Could not detect peripheral clock rate\n" );
		curr_per_clk = -1;
	}
	if (curr_per_clk != -1) {
		printk( "Detected peripheral clock rate of %d KHz\n", curr_per_clk );
	}

	/* analyse current i2c baudrate */
	if ((iccl_val == 0x42 && icch_val == 0x36) ||
		(iccl_val == 0x4A && icch_val == 0x3D) ||
		(iccl_val == 0x82 && icch_val == 0x6E) ||
		(iccl_val == 0x96 && icch_val == 0x78) ||
		(iccl_val == 0xAF && icch_val == 0x9B)) {
		curr_baudrate = 100000;
	}
	else if ((iccl_val == 0x16 && icch_val == 0x0C) ||
		(iccl_val == 0x2C && icch_val == 0x18)) {
		curr_baudrate = 397100;
	}
	else if ((iccl_val == 0x12 && icch_val == 0x0C) ||
		(iccl_val == 0x26 && icch_val == 0x16)) {
		curr_baudrate = 400000;
	}
	else if ((iccl_val == 0x33 && icch_val == 0x20)) {
		curr_baudrate = 397600;
	}
	else {
		printk( "Invalid icc register combination 0x%02X + 0x%02X\n", iccl_val, icch_val );
		//		return -1;
	}
	if (curr_baudrate != -1) {
		printk( "Detected i2c baudrate %d Hz\n", curr_per_clk );
	}

	if ((curr_baudrate != -1) && (curr_baudrate == baudrate)) {
		printk( "I2C is already running at desired baudrate of %i Hz\n", curr_baudrate );
		return 0;
	}

	/*
	 * find configuration variant which can be applied
	 * without peripheral clock re-configuration (if available)
	 */
	switch (baudrate) {
	case 100000:
		switch (curr_per_clk) {
		case 12000:
			/* variant 1 */
			per_clk = 12000;
			iccl_val = 0x42;
			icch_val = 0x36;
			break;
		case 13000:
			/* variant 2 */
			per_clk = 13000;
			iccl_val = 0x4A;
			icch_val = 0x3D;
			break;
		case 24000:
			/* variant 3 */
			per_clk = 24000;
			iccl_val = 0x82;
			icch_val = 0x6E;
			break;
		case 27000:
			/* variant 4 */
			per_clk = 27000;
			iccl_val = 0x96;
			icch_val = 0x78;
			break;
		case 33000:
		default:
			/* variant 5 */
			per_clk = 33000;
			iccl_val = 0xAF;
			icch_val = 0x9B;
			break;
		}
		baudrate_str = "100 kHz";
		break;
	case 397100:
		switch (curr_per_clk) {
		case 13000:
			/* variant 1 */
			per_clk = 13000;
			iccl_val = 0x4A;
			icch_val = 0x3D;
			break;
		case 27000:
		default:
			/* variant 2 */
			per_clk = 27000;
			iccl_val = 0x96;
			icch_val = 0x78;
			break;
		}
		baudrate_str = "397.1 kHz";
		break;
	case 397600:
		per_clk = 33000;
		iccl_val = 0x33;
		icch_val = 0x20;
		baudrate_str = "397.6 kHz";
		break;
	case 400000:
		switch (curr_per_clk) {
		case 12000:
			/* variant 1 */
			per_clk = 12000;
			iccl_val = 0x12;
			icch_val = 0x0C;
			break;
		case 24000:
		default:
			/* variant 2 */
			per_clk = 24000;
			iccl_val = 0x26;
			icch_val = 0x16;
			break;
		}
		baudrate_str = "400 kHz";
		break;
	default:
		printk( "Invalid baudrate value %lu\n", baudrate );
		return -2;
		//            break;
	}

	printk( "Configure I2C baud rate to %s\n", baudrate_str );
	printk( "curr_baudrate = %i\n", curr_baudrate );
	printk( "baudrate      = %lu\n", baudrate );
	printk( "curr_per_clk  = %i\n", curr_per_clk );
	printk( "per_clk       = %i\n", per_clk );
	printk( "new iccl_val  = 0x%02X\n", iccl_val );
	printk( "new icch_val  = 0x%02X\n", icch_val );



	if (per_clk != curr_per_clk) {
		printk( "Re-Configure peripheral clock to %d KHz\n", per_clk );


	}




	//    readb()
	//    writeb(iccl_val, I2C_CCL);
	//	writeb(icch_val, I2C_CCH);


	return 0;
}

#endif


static long rl78_i2c_ioctl( struct file *file, unsigned int cmd, unsigned long arg )
{
	struct i2c_client *client = file->private_data;
	unsigned long funcs;

	if (cmd != I2C_SLAVE) {
		printk( "%s - cmd=0x%02x, arg=0x%02lx\n", __FUNCTION__,
			cmd, arg );
	}

	switch (cmd) {
	case I2C_SLAVE:
	case I2C_SLAVE_FORCE:
		/* NOTE:  devices set up to work with "new style" drivers
		 * can't use I2C_SLAVE, even when the device node is not
		 * bound to a driver.  Only I2C_SLAVE_FORCE will work.
		 *
		 * Setting the PEC flag here won't affect kernel drivers,
		 * which will be using the i2c_client node registered with
		 * the driver model core.  Likewise, when that client has
		 * the PEC flag already set, the i2c-dev driver won't see
		 * (or use) this setting.
		 */
		if ((arg > 0x3ff) ||
			(((client->flags & I2C_M_TEN) == 0) && arg > 0x7f))
			return -EINVAL;
		if (cmd == I2C_SLAVE && i2cdev_check_addr( client->adapter, arg ))
			return -EBUSY;
		/* REVISIT: address could become busy later */
		client->addr = arg;
		return 0;
	case I2C_TENBIT:
		if (arg)
			client->flags |= I2C_M_TEN;
		else
			client->flags &= ~I2C_M_TEN;
		return 0;
	case I2C_PEC:
		if (arg)
			client->flags |= I2C_CLIENT_PEC;
		else
			client->flags &= ~I2C_CLIENT_PEC;
		return 0;
	case I2C_FUNCS:
		funcs = i2c_get_functionality( client->adapter );
		return put_user( funcs, (unsigned long __user *) arg );

	case I2C_RDWR:
		return i2cdev_ioctl_rdrw( client, arg );

	case I2C_SMBUS:
		return i2cdev_ioctl_smbus( client, arg );

	case I2C_RETRIES:
		client->adapter->retries = arg;
		break;
	case I2C_TIMEOUT:
		/* For historical reasons, user-space sets the timeout
		 * value in units of 10 ms.
		 */
		client->adapter->timeout = msecs_to_jiffies( arg * 10 );
		break;
#ifdef I2C_BAUDRATE_CONFIG
	case I2C_BAUDRATE:
		return (i2cdev_set_baudrate( arg ));
		break;
#endif
	default:
		/* NOTE:  returning a fault code here could cause trouble
		 * in buggy userspace code.  Some old kernel bugs returned
		 * zero in this case, and userspace code might accidentally
		 * have depended on that bug.
		 */
		return -ENOTTY;
	}
	return 0;
}


/*********************************************************************************
 * rl78_i2c_open
 *   Initialises the rl78 i2c interface.
 *
 * Parameters:
 *  inode - pointer to inode structure related to rl78 interrupt.
 *  file  - pointer to file structure directly linked to rl78 interrupt device file.
 *
 * Return Values:
 *   0        - in case of success.
 *   <0        - in case of error.
 *
 *********************************************************************************/
static int rl78_i2c_open( struct inode *inode, struct file *file )
{
	unsigned int minor = iminor( inode );
	struct i2c_client *client;
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;

	//	printk("rl78_i2c_open -> i2c_dev_get_by_minor(minor=%i)\n", minor);

	i2c_dev = i2c_dev_get_by_minor( minor );
	if (!i2c_dev) {
		printk( "%s - i2c_dev_get_by_minor (minor %ui) failed\n", __FUNCTION__, minor );
		return -ENODEV;
	}

	//	printk("rl78_i2c_open -> i2c_get_adapter\n");
	adap = i2c_get_adapter( i2c_dev->adap->nr );
	if (!adap) {
		printk( "%s - i2c_get_adapter (nr %i) failed\n", __FUNCTION__, i2c_dev->adap->nr );
		return -ENODEV;
	}
	//	printk("rl78_i2c_open - adap->nr=%d\n", adap->nr);

	/* This creates an anonymous i2c_client, which may later be
	 * pointed to some address using I2C_SLAVE or I2C_SLAVE_FORCE.
	 *
	 * This client is ** NEVER REGISTERED ** with the driver model
	 * or I2C core code!!  It just holds private copies of addressing
	 * information and maybe a PEC flag.
	 */
	client = kzalloc( sizeof (*client), GFP_KERNEL );
	if (!client) {
		printk( "%s - kzalloc failed\n", __FUNCTION__ );
		i2c_put_adapter( adap );
		return -ENOMEM;
	}
	snprintf( client->name, I2C_NAME_SIZE, "rl78_i2c-dev %d", adap->nr );
	client->driver = &i2cdev_driver;

	client->adapter = adap;
	file->private_data = client;

	return 0;
}


/*********************************************************************************
 * rl78_irq_open
 *   Initialises and configures the rl78 interrupt.
 *
 * Parameters:
 *  inode - pointer to inode structure related to rl78 interrupt.
 *  file  - pointer to file structure directly linked to rl78 interrupt device file.
 *
 * Return Values:
 *   0        - in case of success.
 *   !=0        - in case of error.
 *
 *********************************************************************************/
static int rl78_irq_open( struct inode *inode, struct file *file )
{
	int ret1 = 0;
	int ret2 = 0;
	unsigned short sTmp;


	/*
	 * program IRQ2 to falling edge (ICR1  bit 10/11: 00)
	 * via read/modify/write cycle
	 */
	sTmp = readw( ICR1 );

#if 0
	/* low level */
	//	sTmp &= ~0x400;  /* low level */
	//	sTmp |= 0x800;   /* low level */
	/* high level */
	//	sTmp |= 0xC00;  /* high level */
#endif
	/* falling edge */
	sTmp &= ~0x0C00; /* falling edge */
	//	printk("Configure ICR1 with 0x%04X\n", sTmp);
	writew( sTmp, ICR1 );

	/* request IRQ2 for RL78 requests */
	//	printk("Request IRQ2 -> rl78_irq_handler\n");
	if ((ret2 = request_irq( evt2irq( IRQEVT_IRQ2 ), rl78_irq_handler, IRQF_DISABLED, "rl78", NULL )) < 0)
		printk( "ERR: request_irq(rl78,...) failed! (%d)\n", ret2 );

#if 0
	printk( "IRQ2: PORT_PWCR=%X, PORT_PSELD=%X, PORT_HIZCRD=%X\n",
	 readw( PORT_PWCR ), readw( PORT_PSELD ), readw( PORT_HIZCRD ) );
	printk( "IRQ2: ICR1=%X,\n INTPRI00=%X,\n INTREQ00=%X,\n INTMSK00=%X\n",
	 readw( ICR1 ), readl( INTPRI00 ), readb( INTREQ00 ), readb( INTMSK00 ) );

#endif

	//	printk("request_lines=0x%02X\n", rl78_req_line_read(REQALL_MASK));

	rl78_irq_flags = 0;
	return ret1 ? ret1 : ret2;
}


/*********************************************************************************
 * rl78_req_line_init
 *   Initialises pins SDC2_D0 ... SDC2_D3 which are connected to the
 *   RL78 request lines REQ0 ... REQ3 as input pins.
 *
 * Parameters:
 *  None
 *
 * Return Values:
 *   0        - always.
 *
 *********************************************************************************/
static int __init rl78_req_line_init( void )
{
	uint16_t reg_val = 0x00;

	/*    printk("***** rl78 request line init *****\n");  */

	/*
	 * configure PTC2...PTC5 to operate as input pins
	 * for indication of RL78 requests
	 */
	reg_val = readw( PORT_PCCR );
	/*	printk("reading PORT_PCCR = 0x%04X from address 0x%X\n", reg_val, PORT_PCCR);  */
	reg_val |= 0x0FF0;
	/*	printk("Configure PORT_PCCR with 0x%04X\n", reg_val);  */
	writew( reg_val, PORT_PCCR );

	return 0;
}


/*********************************************************************************
 * rl78_req_line_read
 *   Reads RL78 request lines REQ0 ... REQ3 from input pins PTC2... PTC5.
 *
 * Parameters:
 *   req_line - REQ0, REQ1, REQ2, REQ3 or REQALL.
 *              Currently single line requests are disabled,
 *              only REQALL mode is activated.
 *
 * Return Values:
 *   0 or 1   - value of requested input line on bit position 0 for single line access.
 *   0 ... 15 - in case of multi line access
 *   -1       - in case of invalid parameter passing
 *
 * Remark:
 *   Input parameter allows single line or multi line request.
 *   For multi line request via REQALL the result contains the single line values
 *   REQ0...REQ3 on bit positions 0...3
 *
 *********************************************************************************/
static int8_t rl78_req_line_read( int8_t req_line )
{
	uint8_t mask = 0xFF;
	uint8_t shift = 0;
	uint8_t reg_val = 0x00;
	int8_t retval = -1;
	uint8_t xor_mask = 0x00;

	//    printk("rl78 request line read\n");
	switch (req_line) {
	case REQ0:
		mask = REQ0_MASK;
		xor_mask = 0x01;
		shift = REQ0_SHIFT;
		break;
	case REQ1:
		mask = REQ1_MASK;
		xor_mask = 0x02;
		shift = REQ1_SHIFT;
		break;
	case REQ2:
		mask = REQ2_MASK;
		xor_mask = 0x04;
		shift = REQ0_SHIFT;
		break;
	case REQ3:
		mask = REQ3_MASK;
		xor_mask = 0x08;
		shift = REQ3_SHIFT;
		break;
	case REQALL:
		mask = REQALL_MASK;
		xor_mask = 0x0F;
		shift = REQALL_SHIFT;
		break;
	default:
		printk( "Invalid request parameter %i\n", req_line );
		break;
	}

	if (mask != 0xFF) {
		/*
		 * read RL78 request lines from PTC2...PTC5,
		 * invert request line bits (-> 1=active, 0=inactive) and
		 * apply required mask and bit shift
		 */
		reg_val = readb( PORT_PCDR );
		//		printk("rl78_req_line_read from address 0x%08X: 0x%02X\n", PORT_PCDR, reg_val);
		reg_val = ((reg_val & mask) >> shift);

		/* request line signals are active low
		 * -> invert value of requested bits 3...0 */
		reg_val = reg_val^xor_mask;
		//		printk("regval = 0x%02X\n", reg_val);
		retval = (int8_t) reg_val;

	}
	return retval;
}


/*********************************************************************************
 * rl78_irq_init
 * Initialises the collective RL78 request interrupt via IRQ2.
 *
 * Parameters:
 *  None
 *
 * Return Values:
 *   -EIO     - in case of error,
 *   0        - in case of success.
 *
 *********************************************************************************/
int __init rl78_irq_init( void )
{
	unsigned char cTmp = 0x00;
	unsigned short sTmp;

	/*    printk("***** rl78 irq init ******\n");  */

	/*
	 * reset IRQ2 request bit and
	 * register driver for RL78 interrupt
	 */
	cTmp = readb( INTREQ00 );
	cTmp &= ~0x20;
	writeb( cTmp, INTREQ00 );

	/*
	 * program IRQ2 to falling edge (ICR1  bit 10/11: 00)
	 */
	sTmp = readw( ICR1 );
	/* low level */
	//	sTmp &= ~0x400;  /* low level */
	//	sTmp |= 0x800;   /* low level */
	/* high level */
	//	sTmp |= 0xC00;  /* high level */
	/* falling edge */
	sTmp &= ~0x0C00; /* falling edge */
	/*	printk("Configure ICR1 with 0x%04X\n", sTmp);  */
	writew( sTmp, ICR1 );

	/*
	 * configure PTW2 and PSELD to operate PTW2 as IRQ2 pin (special function)
	 * -> PORT_PWCR bits 4,5 = 00 -> special function selection
	 * -> PSELD bits 11,10 = 00   -> select IRQ2
	 */
	sTmp = readw( PORT_PSELD );
	sTmp &= ~0x0C00;
	//	printk("Configure PORT_PSELD with 0x%04X\n", sTmp);
	writew( sTmp, PORT_PSELD );

	sTmp = readw( PORT_PWCR );
	//	printk("reading PORT_PCCR = 0x%02X from address 0x%X\n", sTmp, PORT_PCCR);
	sTmp &= 0x0030;
	//	printk("Configure PORT_PCCR with 0x%04X\n", sTmp);
	writew( sTmp, PORT_PWCR );

	//	printk("rl78_irq_init - register_chrdev(rl78_irq)");
	if (register_chrdev( RL78_IRQ_MAJOR, "rl78_irq", &rl78_irq_fops )) {
		printk( "unable to get major %d for RL78_IRQ\n",
			RL78_IRQ_MAJOR );
		return -EIO;
	}
	return 0;
}

#if 0


static int rl78_irq_ioctl( struct inode *inode, struct file *file,
			   unsigned int cmd, unsigned long arg )
{
	uint8_t req_lines = 0;
	uint8_t rl78_int = -1;

	printk( "rl78_irq_ioctl, cmd=%d\n", cmd );

	switch (cmd) {
	case RL78_READ_REQ_LINES:
		req_lines = rl78_req_line_read( REQALL );
		printk( "rl78_irq_ioctl, req_lines = %X\n", req_lines );
		return put_user( (unsigned long) req_lines, (unsigned long __user *) arg );
		break;
	case RL78_READ_INT:
		rl78_int = ((readb( INTREQ00 ) & 0x20) >> 5);
		printk( "rl78_irq_ioctl, rl78_int = %X\n", rl78_int );
		return put_user( (unsigned long) rl78_int, (unsigned long __user *) arg );
		break;


	default:
		return -EINVAL;
		break;
	}
	return 0;
}
#endif


/*********************************************************************************
 * rl78_irq_handler
 *   Handler for RL78 request interrupt.
 *
 * Parameters:
 *   irq - interrupt number.
 *   ptr  - .
 *
 * Return Values:
 *   IRQ_HANDLED      - always.
 *
 * Remarks:
 *
 *********************************************************************************/
static irqreturn_t rl78_irq_handler( int irq, void* ptr )
{
	unsigned long flags;

	local_irq_save( flags );
	/* set flag to indicate rl78 interrupt occurence */
	rl78_irq_flags |= RL78_IRQ;
	/* wake up interrupt processing function and return from interrupt */
	wake_up_interruptible( &rl78_irq_queue );
	local_irq_restore( flags );

	return IRQ_HANDLED;
}


/*********************************************************************************
 * rl78_irq_poll
 * Polls for rl78 interrupt occurence.
 *
 * Parameters:
 *  file - pointer to file structure directly linked to openened
 *         rl78 interrupt device file.
 *  wait - pointer to poll_table structure.
 *
 * Return Values:
 *   >0     - if i2c device file is ready for read,
 *   0      - if i2c device is not ready for read.
 *
 *********************************************************************************/
static unsigned int rl78_irq_poll( struct file *file, struct poll_table_struct *wait )
{
	//	Scull_Pipe *dev = filp->private_data;
	unsigned int mask = 0;
	char req_lines = 0;

	poll_wait( file, &rl78_irq_queue, wait );

	/*
	 * check if RL78_IRQ has occurred and
	 * at least one request line is active ->
	 * device is ready to be read from
	 */
	if ((rl78_irq_flags & RL78_IRQ) != 0) {
		req_lines = rl78_req_line_read( REQALL );
		if (req_lines > 0) {
			/* indicate that data to be read is available at rl78 i2c interface */
			mask |= POLLIN | POLLRDNORM;
		}
		else {
			/*
			 * plausibility check,
			 * interrupt without at least one active request line should never occur
			 */
			if (req_lines == 0) {
				printk( "Error: no RL78 Request(s) detected \n" );
			}
		}
	}

	return mask;

}


/*********************************************************************************
 * rl78_irq_close
 *   Closes the rl78 interrupt device.
 *
 * Parameters:
 *  inode - pointer to inode structure related to rl78 interrupt.
 *  file - pointer to file structure directly linked to rl78 interrupt device file.
 *
 * Return Values:
 *   0      - always.
 *
 *********************************************************************************/
static int rl78_irq_close( struct inode *inode, struct file *file )
{
	/* free IRQ2 related to rl78 shared interrupt */
	free_irq( evt2irq( IRQEVT_IRQ2 ), NULL );

	rl78_irq_flags = 0;
	return 0;
}




// NMI handler
//------------------------------------------------------------------


void rl78_irq_nmi_handler( void )
{
	printk( "rl78_irq_nmi_handler()\n" );
	rl78_irq_flags |= RL78_IRQ_NMI;
	wake_up_interruptible( &rl78_irq_queue );
}


/*********************************************************************************
 * rl78_irq_release
 *   Closes the rl78 interrupt device.
 *
 * Parameters:
 *  inode - pointer to inode structure related to rl78 interrupt.
 *  file - pointer to file structure directly linked to rl78 interrupt device file.
 *
 * Return Values:
 *   0      - always.
 *
 *********************************************************************************/
static int rl78_irq_release( struct inode *inode, struct file *file )
{
	//    struct i2c_client *client = file->private_data;

	rl78_irq_close( inode, file );
	file->private_data = NULL;

	return 0;
}



/* ------------------------------------------------------------------------- */

/*
 * The legacy "i2cdev_driver" is used primarily to get notifications when
 * I2C adapters are added or removed, so that each one gets an i2c_dev
 * and is thus made available to userspace driver code.
 */

static struct class *i2c_dev_class;


static int i2cdev_attach_adapter( struct i2c_adapter *adap )
{
	struct i2c_dev *i2c_dev;
	int res;

	i2c_dev = get_free_i2c_dev( adap );
	if (IS_ERR( i2c_dev ))
		return PTR_ERR( i2c_dev );

	/* register this i2c device with the driver core */
	i2c_dev->dev = device_create( i2c_dev_class, &adap->dev,
				      MKDEV( RL78_I2C_MAJOR, adap->nr ), NULL,
				      "rl78_i2c-%d", adap->nr );
	if (IS_ERR( i2c_dev->dev )) {
		res = PTR_ERR( i2c_dev->dev );
		goto error;
	}
	res = device_create_file( i2c_dev->dev, &dev_attr_name );
	if (res)
		goto error_destroy;

	pr_debug( "%s - adapter [%s] registered as minor %d\n", __FUNCTION__,
		  adap->name, adap->nr );
	return 0;
error_destroy:
	device_destroy( i2c_dev_class, MKDEV( RL78_I2C_MAJOR, adap->nr ) );
error:
	return_i2c_dev( i2c_dev );
	return res;
}


static int i2cdev_detach_adapter( struct i2c_adapter *adap )
{
	struct i2c_dev *i2c_dev;

	i2c_dev = i2c_dev_get_by_minor( adap->nr );
	if (!i2c_dev) /* attach_adapter must have failed */
		return 0;

	device_remove_file( i2c_dev->dev, &dev_attr_name );
	return_i2c_dev( i2c_dev );
	device_destroy( i2c_dev_class, MKDEV( RL78_I2C_MAJOR, adap->nr ) );

	printk( "%s - adapter [%s] unregistered\n", __FUNCTION__, adap->name );
	return 0;
}

static struct i2c_driver i2cdev_driver = {
	.driver =
	{
		.name = "dev_driver",
	},
	.attach_adapter = i2cdev_attach_adapter,
	.detach_adapter = i2cdev_detach_adapter,
};


static int rl78_i2c_release( struct inode *inode, struct file *file )
{
	struct i2c_client *client = file->private_data;

	i2c_put_adapter( client->adapter );
	kfree( client );
	file->private_data = NULL;

	return 0;
}

/* ------------------------------------------------------------------------- */

/*
 * module load/unload record keeping
 */


/*********************************************************************************
 * rl78_i2c_init
 *   Initialises the i2c related parts of the rl78 communication.
 *
 * Parameters:
 *   none.
 *
 * Return Values:
 *   <0       - in case of error,
 *   =0       - in case of success.
 *
 * Remark:
 *   Main initialisation function executed after module_init.
 *
 *********************************************************************************/
int __init rl78_i2c_init( void )
{
	int res = -1;

	printk( "%s - register_chrdev(rl78_i2c)\n", __FUNCTION__ );

	if (register_chrdev( RL78_I2C_MAJOR, "rl78_i2c", &rl78_i2c_fops )) {
		printk(KERN_ERR "unable to get major %d for RL78_I2C\n",
			RL78_I2C_MAJOR );
		return -EIO;
	}

	/*
	 * create i2c class and add i2c driver
	 */
	i2c_dev_class = class_create( THIS_MODULE, "rl78_i2c-dev" );
	if (IS_ERR( i2c_dev_class )) {
		res = PTR_ERR( i2c_dev_class );
		printk( KERN_ERR "%s - class_create failed\n", __FUNCTION__ );
		goto out_unreg_chrdev;
	}
	printk( "%s - class_create success\n", __FUNCTION__  );

	res = i2c_add_driver( &i2cdev_driver );
	if (res) {
		printk( KERN_ERR "%s - i2c_add_driver failed!!!\n", __FUNCTION__ );
		goto out_unreg_class;
	}
	printk( "%s - i2c_add_driver success\n", __FUNCTION__ );
	return 0;

out_unreg_class:
	class_destroy( i2c_dev_class );

out_unreg_chrdev:
	unregister_chrdev( RL78_I2C_MAJOR, "rl78_i2c" );
	printk( KERN_ERR "%s: Driver Initialisation failed\n", __FILE__ );
	return res;
}


/*********************************************************************************
 * rl78_init
 *   Initialises rl78 communication via IRQ2, request lines and i2c.
 *
 * Parameters:
 *   none.
 *
 * Return Values:
 *   <0       - in case of error,
 *   =0       - in case of success.
 *
 * Remark:
 *   Main initialisation function executed on driver initialisation.
 *
 *********************************************************************************/
static int __init rl78_init( void )
{
	int ret1 = 0;
	int ret2 = 0;
	int ret3 = 0;

	printk( "Initializing RL78 IRQ+I2C Connection\n" );

	/*
	 * initialise request line pins and
	 * collective RL-78 interrupt via IRQ2
	 */
	ret1 = rl78_req_line_init( );
	ret2 = rl78_irq_init( );
	ret3 = rl78_i2c_init();

	return (ret1 || ret2 || ret3);
}


int rl78_irq_cleanup( void )
{
	unregister_chrdev( RL78_IRQ_MAJOR, "rl78_irq" );
	return 0;
}


static void __exit rl78_exit( void )
{
	i2c_del_driver( &i2cdev_driver );
	//    class_destroy(i2c_dev_class);
	unregister_chrdev( RL78_I2C_MAJOR, "rl78_i2c" );

	rl78_irq_cleanup( );
}




MODULE_AUTHOR( "Michael Grobe <Michael.Grobe@Hach-Lange.de> " );
MODULE_DESCRIPTION( "rl78 device driver" );
MODULE_LICENSE( "GPL" );

module_init( rl78_init );
module_exit( rl78_exit );
