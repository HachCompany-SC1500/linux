/*
 * drivers/char/can/okican.c
 *
 * OKI ML9620 CAN Controler driver
 *
 * Author: Martin Nylund <mnylund@emtrion.de>
 *
 * Rewritten from orginal work of Manfred Gruber
 *
 * Copyright 2004 (c) Manfred Gruber
 * Copyright 2004 (c) Contec Steuerungstechnik & Automation Ges.m.b.H
 * Copyright 2008 (c) Martin Nylund - emtrion GmbH
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/okican.h>
#include <linux/kfifo.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/io.h>

#define DRV_NAME "okican"

#define TIMEOUT 50000

static int major = 0;
module_param(major, int, S_IRUGO);

static int okican_read(struct file *filp, char *buf, size_t count,loff_t *ppos);
static int okican_write(struct file *filp, const char *buf, size_t count,loff_t *ppos);
static int okican_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
static int okican_open(struct inode *inode, struct file *filp);
static int okican_release(struct inode *inode, struct file *filp);
static unsigned int okican_poll(struct file *filp, poll_table *wait);

static irqreturn_t ml9620_irq_handler(int irq, void *dev_id);
static void ml9620_init(struct can_device *c);
static int ml9620_set_bitrate(struct can_device *c, int baud);
static int ml9620_get_status(struct can_device *c);

/* no two devices should be probed at the same time, so static variables should be safe */
static int device_count = 0;
static struct class* okican_class;

int ml9620_set_bitrate(struct can_device *c, int baud)
{
	unsigned long v = (unsigned long)c->va_regs;
	int index = 0;
	uint8_t tmp;

	/* unlock hardware */
	tmp = __raw_readb(v + CANCONT);
	tmp |= CONT_FLAG_INIT;
	__raw_writeb(tmp, v + CANCONT);

	tmp = __raw_readb(v + CANCONT);
	tmp |= CONT_FLAG_CCE;
	__raw_writeb(tmp, v + CANCONT);

	/* Lookuptable baudrate => Timing array index */
	switch (baud){
	case 10:	index = 0; break;
	case 20:	index = 1; break;
	case 50:	index = 2; break;
	case 100:	index = 3; break;
	case 125:	index = 4; break;
	case 250:	index = 5; break;
	case 500:	index = 6; break;
	case 800:	index = 7; break;
	case 1000:	index = 8; break;
	default:	
		return -EINVAL; 
	}

	/* set baudrate */
	__raw_writeb(bd_tbl[index].btiming0, v + CANBITT1);
	__raw_writeb(bd_tbl[index].btiming1, v + CANBITT2);
	__raw_writeb(bd_tbl[index].bpre, v + CANBPRE);

	tmp = __raw_readb(v + CANCONT);
	tmp &= ~CONT_FLAG_CCE;
	__raw_writeb(tmp, v + CANCONT);

	tmp = __raw_readb(v + CANCONT);
	tmp &= ~CONT_FLAG_INIT;
	__raw_writeb(tmp, v + CANCONT);

	return 0;
}

static int okican_wait_busy(unsigned long v) {
	int timeout = TIMEOUT;
	while ((__raw_readb(v + IF1BUSY)) & 0x80 && --timeout) {
		timeout--;
		udelay(1);
	}

	return !timeout ? -ETIMEDOUT : 0;
}

int __cfg_msg_object(unsigned long v,
		int num, uint32_t id, uint32_t mask, 
		uint8_t ctrl_1, uint8_t ctrl_2)
{
	if(num>32)
		return -EINVAL;

	if (okican_wait_busy(v)<0)
		return -ETIMEDOUT;

	__raw_writeb((id>>0)&0xFF,    v + IF1ID1);
	__raw_writeb((id>>8)&0xFF,    v + IF1ID2);
	__raw_writeb((id>>16)&0xFF,   v + IF1ID3);
	__raw_writeb((id>>24)&0xFF,   v + IF1ID4);
	__raw_writeb((mask>>0)&0xFF,  v + IF1MASK1);
	__raw_writeb((mask>>8)&0xFF,  v + IF1MASK2);
	__raw_writeb((mask>>16)&0xFF, v + IF1MASK3);
	__raw_writeb((mask>>24)&0xFF, v + IF1MASK4);
	__raw_writeb(ctrl_1, v + IF1MCONT1);
	__raw_writeb(ctrl_2, v + IF1MCONT2);

	/* Transfer control and ID bits */
	__raw_writeb(0xF8, v + IF1CMASK);

	/* Set the mailbox we want to configure */
	__raw_writeb(num, v + IF1CREQ);

	/* Transfer control and ID bits */
	__raw_writeb(0x78, v + IF1CMASK);

	if (okican_wait_busy(v)<0)
		return -ETIMEDOUT;

	return 0;

}

int cfg_msg_object(unsigned long v,
		int num, uint32_t id, uint32_t mask, 
		uint8_t ctrl_1, uint8_t ctrl_2)
{
	uint8_t tmp;
	int ret;
	/* Set the chip in init mode */
	tmp = __raw_readb(v + CANCONT);
	tmp |= CONT_FLAG_INIT;
	__raw_writeb(tmp, v + CANCONT);

	ret=__cfg_msg_object(v, num, id, mask, ctrl_1, ctrl_2);

	/*End Init */
	tmp = __raw_readb(v + CANCONT);
	tmp &= ~CONT_FLAG_INIT;
	__raw_writeb(tmp, v + CANCONT);

	return ret;
}
void ml9620_init(struct can_device *c)
{
	unsigned char i;
	unsigned long v = (unsigned long)c->va_regs;
	uint8_t tmp;

	/* Set the chip in init mode */
	tmp = __raw_readb(v + CANCONT);
	tmp |= CONT_FLAG_INIT;
	__raw_writeb(tmp, v + CANCONT);

	/* Initialize all msg objects. By default, 1 receives standard
	 * frames and 2 extended frames. 32 is used for transmitting */

	/* Simple default configuration: first 29 message objects are not
	 * used by default. They can be configured for some application
	 * specific messages. objects m30 and 31 are used for receiving
	 * messages and the last object, 32, is used for transmitting */

	/************************************************************/
	/*                    ID          Mask        cnt1   cnt2   */
	/************************************************************/
	i=1;
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x00000000, 0x20000000, 0x00, (UMask|RxIE));

	/* FIFO buffer of two objects for receiving messages (see chapter
	 * 3.8 in the OKI datasheet). These objectct do not mask any
	 * messages. NOTE: If we make fifo longer that 2 objects, data
	 * concistency cannot be guaranteed. It would require some extra
	 * mechanism. */
	__cfg_msg_object(v, i++, 0x80000000, 0x20000000, 0x00, (UMask|RxIE));
	__cfg_msg_object(v, i++, 0x80000000, 0x20000000, 0x80, (UMask|RxIE));

	/* All messages are sent through this object */
	__cfg_msg_object(v,TX_MSG_OBJ ,0x80000000, 0x20000000, 0x80, (TxIE));

	if(i!=32 || TX_MSG_OBJ != 32){
		dev_err(c->dev,"Concider revising your configuration!\n");
	}

	/* Enable interrupts */
	tmp = __raw_readl(v + CANCONT);
	tmp |= CONT_FLAG_IE;
	__raw_writeb(tmp, v + CANCONT);

	/*End Init */
	tmp = __raw_readb(v + CANCONT);
	tmp &= ~CONT_FLAG_INIT;
	__raw_writeb(tmp, v + CANCONT);

	/* set default bitrate 125 kBit/sec */
	ml9620_set_bitrate(c, 125);
}


void rx_fifo_put(struct can_device *c, struct can_msg *msg){
	/* Store Data */
	if( c->rx_fifo.size - (kfifo_len(&c->rx_fifo) ) >= 
			sizeof(struct can_msg)){

		/* Save overrun status into the message and clear the flag
		 * */
		msg->dos=c->dos;
		c->dos=0;

		kfifo_in(&c->rx_fifo,(unsigned char *)msg, 
			 sizeof(struct can_msg));
		wake_up_interruptible(&c->rx_waitq);
	} else {
		dev_dbg(c->dev,"rx fifo overflow\n");
		c->dos|=1;
	}
}

static irqreturn_t ml9620_irq_handler(int irq, void *dev_id)
{
	unsigned short i;
	struct can_msg msg;
	struct can_device *c = dev_id;
	unsigned long v = (unsigned long)c->va_regs;
	uint8_t msg_obj,cnt1,cnt2;
	int ret = IRQ_NONE;

	while(1){
		/* FIXME: this should be moved into a thread when do do busy polling */
		msg_obj = __raw_readb(v + CANINT1);
		if(!msg_obj)
			break;

		ret = IRQ_HANDLED;

		/* Transfer data for this message object, not the NewDat
		 * though. This would mark the object as free. */
		__raw_writeb(0x3f, v + IF2CMASK);
		__raw_writeb(msg_obj, v + IF2CREQ);
		while ((__raw_readb(v + IF2BUSY)) & 0x80);
		cnt2 = __raw_readb(v + IF2MCONT2);
		cnt1 = __raw_readb(v + IF2MCONT1);

		/* If message received */
		if( (cnt2&(RxIE|NewDat)) == (NewDat|RxIE)){


			/* Is MsgLost set? */
			if ((__raw_readb(v + IF2MCONT2)) & 0x40){
				c->dos|=2;
			}

			/* Data Length Code */
			msg.dlc = (cnt1 & 0x0F);
			if(msg.dlc>8){
				dev_err(c->dev,"Invalid dlc %d!\n",msg.dlc);
				msg.dlc=0;
			}

			/* First read the standard ID and the control
			 * field */
			msg.id = __raw_readb(v + IF2ID4);
			msg.id = msg.id << 8;
			msg.id |= __raw_readb(v + IF2ID3);

			/* Remote Transmission Request? (i.e. the Dir bit) */
			if (msg.id & 0x2000)
				msg.rtr = 1;
			else
				msg.rtr = 0;

			/* Extended Frame? */
			if(msg.id & 0x4000){
				msg.ff = FF_EXTENDED;
			} else {
				msg.ff = FF_NORMAL;
			}

			/* mask off the control bit values */
			msg.id&=0x1fff;

			if(msg.ff == FF_EXTENDED){
				/* Read the rest of the id bytes */
				msg.id = msg.id << 8;
				msg.id |= __raw_readb(v + IF2ID2);
				msg.id = msg.id << 8;
				msg.id |= __raw_readb(v + IF2ID1);
			} else {
				msg.id = msg.id>>2;
			}

			/* Save the number of the message object, which
			 * received the message */
			msg.mobj = msg_obj-1;

			
			/* Read message data, not for rtr messages though
			 * */
			if(!msg.rtr){
				for (i = 0; i < msg.dlc; i++)
					msg.data[i] = __raw_readb(v + IF2DATA1 +
							(i<<REG_SHIFT));
			}

			/* save the message into rx_fifo */
			rx_fifo_put(c,&msg);

		} else if(__raw_readb(v + IF2MCONT2) & TxIE){
			c->tx_in_progress = 0;
			wake_up_interruptible(&c->tx_waitq);
		}
	}

	return ret;
}

static struct file_operations okican_fops = {
	.owner = THIS_MODULE,
	.read = okican_read,
	.write = okican_write,
	.ioctl = okican_ioctl,
	.open = okican_open,
	.release = okican_release,
	.poll = okican_poll,
};

int okican_read(struct file *filp, char *buf, size_t count, loff_t * ppos)
{
	struct can_device *c = filp->private_data;
	int ret = 0;
	struct can_msg msg;

	if (count != sizeof(struct can_msg))
		return -EINVAL;

	if (kfifo_len(&c->rx_fifo)==0 && (filp->f_flags & O_NONBLOCK))
		return  -EAGAIN;
		
	/* Wait until we have data to read */
	ret = wait_event_interruptible(c->rx_waitq, 
			kfifo_len(&c->rx_fifo) >= sizeof(struct can_msg));
	if (ret){
		return -ERESTARTSYS;
	}

	/* Read one message from the rx fifo... */
	ret = kfifo_out(&c->rx_fifo,(unsigned char *)&msg, 
			sizeof(struct can_msg));

	/* ...and copy it to the application */
	ret = copy_to_user(buf, &msg, sizeof(struct can_msg));
	if(ret!=0){
		dev_err(c->dev,"failed to copy message to userspace\n");
		return EFAULT;
	}

	return sizeof(struct can_msg);
}

int okican_write(struct file *filp, const char *buf, size_t count, loff_t * ppos)
{
	struct can_device *c = filp->private_data;
	unsigned char i;	/* counter, temp var */
	unsigned char k;	/* counter */
	unsigned long v = (unsigned long)c->va_regs;
	struct can_msg tmp;
	struct can_msg *txbuf = &tmp;
	uint8_t id[4], status;
	int ret;

	/* only struct can_msg should be written */
	if (count != sizeof(struct can_msg)) {
		return -EINVAL;
	}

	if (0 != copy_from_user(txbuf, buf, count)) {
		dev_err(c->dev, "%s: copy_from_user failed\n", 
				__FUNCTION__);
		return -EINVAL;
	}


	if ((__raw_readb(v + CANCONT) & CONT_FLAG_INIT) == CONT_FLAG_INIT) {
		dev_err(c->dev, "%s: node in init state",__FUNCTION__);
		return -EBUSY;
	}

	/* Check the status, so we can handle warning and bus off state */
	status = ml9620_get_status(c);

	if (status & S_BOff){
		dev_err(c->dev, "node in BusOff state\n");
		return -EIO;
	}

	/* FIXME: this tx_in_progress handling is not safe, use atomic_inc or a counting semaphore*/
	/* handle non-blocking writes */
	if (c->tx_in_progress && (filp->f_flags & O_NONBLOCK))
		return  -EAGAIN;

	/* Wait until previous tx is finished */
	ret = wait_event_interruptible(c->tx_waitq, !c->tx_in_progress);
	if (ret)
		return -ERESTARTSYS;

	c->tx_in_progress = 1;

	if (okican_wait_busy(v)<0) {
		c->tx_in_progress = 0;
		return -ETIMEDOUT;
	}

	/* set, so that ID Register will get transfered */
	__raw_writeb(0xB7, v + IF1CMASK);

	if (txbuf->ff == FF_NORMAL) {
		/* DIR and WR */
		id[3] = 0xA0;
		id[3] |= (unsigned char)((txbuf->id >> 6) & 0x1F);
		__raw_writeb(id[3], v + IF1ID4);
		id[2] = (unsigned char)((txbuf->id << 2) & 0xFC);
		__raw_writeb(id[2], v + IF1ID3);
	} else {
		/* extended Frame */
		id[3] = 0xE0;
		id[3] += (unsigned char)((txbuf->id >> 24) & 0x1F);
		id[2] = (unsigned char)((txbuf->id >> 16) & 0xFF);
		id[1] = (unsigned char)((txbuf->id >> 8) & 0xFF);
		id[0] = (unsigned char)(txbuf->id & 0xFF);
		__raw_writeb(id[3], v + IF1ID4);
		__raw_writeb(id[2], v + IF1ID3);
		__raw_writeb(id[1], v + IF1ID2);
		__raw_writeb(id[0], v + IF1ID1);
	}

	k = (unsigned char)(((txbuf->dlc) <= 8) ? (txbuf->dlc) : 8);
	for (i = 0; i < k; i++) {
		__raw_writeb(txbuf->data[i], v + IF1DATA1 + (i<<REG_SHIFT));
	}

	/* Write DLC */
	__raw_writeb(0x80 + (txbuf->dlc & 0x0F), v + IF1MCONT1);

	__raw_writeb(0x09, v + IF1MCONT2);
	__raw_writeb(TX_MSG_OBJ, v + IF1CREQ);
	return count;
}

int okican_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct can_device *c = filp->private_data;
	int ret = 0;
	int value = 0;
	uint8_t tmp;
	struct msg_obj_params params;
	unsigned long v = (unsigned long)c->va_regs;
	unsigned int timeout = 50000;

	switch (cmd) {
	case IOC_START:
		do {
			tmp=__raw_readb(v + CANCONT);
			tmp &= ~CONT_FLAG_INIT;
			__raw_writeb(tmp, v + CANCONT);
			--timeout;
			udelay(1);
		}
		while (((__raw_readb(v + CANCONT)) & CONT_FLAG_INIT) && timeout);
		break;

	case IOC_STOP:
		do {
			tmp=__raw_readb(v + CANCONT);
			tmp |= CONT_FLAG_INIT;
			__raw_writeb(tmp, v + CANCONT);
			--timeout;
			udelay(1);
		}
		while (!((__raw_readb(v + CANCONT)) & CONT_FLAG_INIT) && timeout);
		break;


	case IOC_SET_BITRATE:
		ret=get_user(value, (int *)arg);
		if(ret) break;

		ret = ml9620_set_bitrate(c, value);
		if(ret) break;

		ret = put_user(value, (int *)arg);
		break;

	case IOC_GET_STATUS:
		value = ml9620_get_status(c);
		ret = put_user(value, (int *)arg);
		break;

	case IOC_CFG_MSG_OBJECT:
		if(copy_from_user(&params, (void *)arg, 
					sizeof(struct msg_obj_params))){
		    ret = -EFAULT;
		    break;
		}

		ret = cfg_msg_object(v, params.i,
				params.id, params.mask,
				params.ctrl_1, params.ctrl_2);

		break;

	default:
		ret=-ENOIOCTLCMD;
		break;
	}

	if (!timeout)
		ret = -ETIMEDOUT;

	return ret;
}

static unsigned int okican_poll(struct file *filp, poll_table * wait)
{
        unsigned int mask = 0;
	struct can_device *c = filp->private_data;

	poll_wait(filp, &c->rx_waitq, wait);
	if (kfifo_len(&c->rx_fifo)>=sizeof(struct can_msg))
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

int ml9620_get_status(struct can_device *c)
{
	unsigned long v = (unsigned long)c->va_regs;

	return __raw_readb(v + CANSTAT);

}


int okican_open(struct inode *inode, struct file *filp)
{
	struct can_device *c;

	c = container_of(inode->i_cdev, struct can_device, cdev);

	filp->private_data = c;

	c->can_opened++;

	return 0;
}

int okican_release(struct inode *inode, struct file *filp)
{
	struct can_device *c = filp->private_data;
	
	c->can_opened--;

	return 0;
}

static int okican_drv_probe(struct platform_device *pdev)
{
	struct can_device *c=NULL;
	struct resource *regs=NULL;
	dev_t devt;
	int ret=0;

	/* Allocate memory for the can device structure */
	c = kzalloc(sizeof(struct can_device),GFP_KERNEL);
	if(!c){
		ret = -ENOMEM;
		goto err_alloc;
	}

#ifdef USE_TX_FIFO
#warning "TX_FIFO not implemented yet"
	if (kfifo_alloc(&c->tx_fifo, sizeof(struct can_msg)*RX_FIFO_SIZE, 
			GFP_KERNEL)) {
		ret=-ENOMEM;
		goto err_tx_fifo;
	}
#endif
	if (kfifo_alloc(&c->rx_fifo, sizeof(struct can_msg)*RX_FIFO_SIZE, 
			GFP_KERNEL)) {
		ret=-ENOMEM;
		goto err_rx_fifo;
	}

	/* waitq init */
	init_waitqueue_head(&c->rx_waitq);
	init_waitqueue_head(&c->tx_waitq);

	ret = alloc_chrdev_region(&devt, pdev->id, 1, dev_name(&pdev->dev));
	if (ret)
		goto err_alloc_chrdev;

        cdev_init(&c->cdev, &okican_fops);
	ret = cdev_add(&c->cdev, devt, 1);
        if (ret)
                goto error_cdevadd;

        /* we need a class for okican */
	if (!device_count) {
		okican_class = class_create(THIS_MODULE, DRV_NAME);
		if (IS_ERR(okican_class)) {
			ret = PTR_ERR(okican_class);
			okican_class = NULL;
			goto error_class;
		}
	}
	device_count++;

        c->dev = device_create(okican_class, NULL, devt, NULL, dev_name(&pdev->dev));
        if (IS_ERR(c->dev)) {
		ret = PTR_ERR(c->dev);
                goto error_device;
	}

	/* Get the physical base address of the chip registers and ioremap
	 * it to a virtual address */
	regs=platform_get_resource(pdev,IORESOURCE_MEM, 0);
	if(!regs){
		dev_err(&pdev->dev, "failed to get resources");
		ret=-EINVAL;
		goto err_map;
	}

	c->va_regs = ioremap(regs->start, regs->end - regs->start );
	if (!c->va_regs) {
		dev_err(&pdev->dev, "ioremap failed (%08x - %08x)",
				regs->start,regs->end);
		ret= -ENOMEM;
		goto err_map;
	}

	platform_set_drvdata(pdev, c);

	/* Initialize the chip registers */
	ml9620_init(c);

	/* Get the interrupt number and install IRQ handler */
	c->irq = platform_get_irq(pdev,0);

	ret=request_irq(c->irq, ml9620_irq_handler, IRQF_SHARED,
			dev_name(&pdev->dev), (void *)c);
	if(ret!=0){
		dev_err(&pdev->dev,"request_irq(%d) failed\n",c->irq);
		goto err_irq;
	}

	dev_info(c->dev,"initialized\n");

	return 0;

err_irq:
	iounmap(c->va_regs);

err_map:
        device_destroy(okican_class, devt);

error_device:
	device_count--;
	if (!device_count)
		class_destroy(okican_class);

error_class:
        cdev_del(&c->cdev);

error_cdevadd:
        unregister_chrdev_region(devt, 1);

err_alloc_chrdev:
	kfifo_free(&c->rx_fifo);

err_rx_fifo:
#ifdef USE_TX_FIFO
	kfifo_free(&c->tx_fifo);

err_tx_fifo:
#endif
	kfree(c);

err_alloc:
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int okican_drv_remove(struct platform_device *pdev)
{
	struct can_device *c = platform_get_drvdata(pdev);
        dev_t devt = c->dev->devt;

	free_irq(c->irq, (void *)c);
	iounmap(c->va_regs);

        device_destroy(okican_class, devt);
	device_count--;
	if (!device_count) {
		class_destroy(okican_class);
		okican_class = NULL;
	}

        cdev_del(&c->cdev);
        unregister_chrdev_region(devt, 1);

#ifdef USE_TX_FIFO
	kfifo_free(&c->tx_fifo);
#endif
	kfifo_free(&c->rx_fifo);
	kfree(c);
	platform_set_drvdata(pdev, NULL);
	return 0;
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin Nylund <mnylund@emtrion.de>");
MODULE_DESCRIPTION("OKI ML9620 CAN Driver");

static struct platform_driver okican_driver = {
	.probe = okican_drv_probe,
	.remove = okican_drv_remove,
	.driver = {
		.name = DRV_NAME,
	},
};


/* Entry point for loading the module */
static int __init okican_init_module(void)
{
	int ret;
	dev_t devNo;

	if(major){
	    devNo=MKDEV(major,0);
	    ret=register_chrdev_region(devNo,255,DRV_NAME);
	}else{
	    ret = alloc_chrdev_region(&devNo,0,255,DRV_NAME);
	    major = MAJOR(devNo);
	}

	if(ret<0){
	    printk(KERN_WARNING "%s: can't get major %d\n",
		__FUNCTION__,major);
	    return ret;
	}

	ret = platform_driver_register(&okican_driver);
	if(ret!=0){
		printk(KERN_ERR "%s: platform_driver_register failed\n",
				__FUNCTION__);
		goto unreg_chrdev;
	}
	
	return 0;

unreg_chrdev:
	unregister_chrdev_region(MKDEV(major,0),255);

	BUG_ON(ret==0);
	return ret;

}

/* entry point for unloading the module */
static void __exit okican_cleanup_module(void)
{
	platform_driver_unregister(&okican_driver);
	unregister_chrdev_region(MKDEV(major,0),255);

}

module_init(okican_init_module);
module_exit(okican_cleanup_module);
/* vim: set sw=8:ts=8: */
