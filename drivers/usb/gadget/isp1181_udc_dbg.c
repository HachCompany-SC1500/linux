/*
 * isp1181_udc_dbg.c - Debugging/Tracing of the events on the isp1181
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
 * Author: Markus Pietrek
 *
 **/

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static struct isp1181_event* isp1181_trace(struct isp1181_priv *priv,
					   enum isp1181_event_type type, int hnum)
{
	struct isp1181_event *event;

	if (priv->events.count >= ARRAY_SIZE(priv->events.list)) {
		/* drop some older events */
		priv->events.count /= 2;
		memmove(&priv->events.list[0], &priv->events.list[priv->events.count],
			sizeof(priv->events.list[0])*(ARRAY_SIZE(priv->events.list)-priv->events.count));
	}

	event = &priv->events.list[priv->events.count];
	priv->events.count++;

	event->type   = type;
	event->hnum   = hnum;

	return event;
}

static inline void isp1181_trace_data(struct isp1181_priv *priv,
				      int hnum, int dir, int len, const void *buf)
{
	struct isp1181_event *event = isp1181_trace(priv, EVT_DATA, hnum);

	priv->stats[hnum].io += len;
	priv->stats[hnum].pkts++;

	if (event) {
		event->data.dir = dir;
		event->data.len = len;
		if (buf && len <= sizeof(event->data.buf))
			memcpy(event->data.buf, buf, len);
	}
}

static inline void isp1181_trace_irq(struct isp1181_priv *priv,
				     int hnum, u32 status, int sim)
{
	struct isp1181_event *event = isp1181_trace(priv, sim ? EVT_IRQ_SIM : EVT_IRQ, hnum);

	if (event)
		event->irq = status;

	if (hnum>=0)
		priv->stats[hnum].irqs++;
}

static inline void isp1181_trace_req(struct isp1181_priv *priv,
				     enum isp1181_event_type type,
				     int hnum,
				     void *req)
{
	struct isp1181_event *event = isp1181_trace(priv, type, hnum);

	if (event)
		event->req = req;
}

static int isp1181_debugfs_frame_seq_show(struct seq_file *file, void *iter)
{
	struct isp1181_priv *priv = file->private;
	
	seq_printf(file, "%i\n", isp1181_get_frame(&priv->gadget));

	return 0;
}

static int isp1181_debugfs_frame_open(struct inode *inode, struct file *file)
{
	return single_open(file, isp1181_debugfs_frame_seq_show, inode->i_private);
}

static const struct file_operations isp1181_debugfs_frame_fops = {
	.owner		= THIS_MODULE,
	.open		= isp1181_debugfs_frame_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int isp1181_debugfs_dbg_seq_show(struct seq_file *file, void *iter)
{
	struct isp1181_priv *priv = file->private;
	unsigned long flags;
	int hnum;
	int i;
	
	spin_lock_irqsave(&priv->lock, flags);
	seq_printf(file, "Addr: %i\n", priv->addr);

	/* display all events */
	seq_printf(file, "events:\n");
	for (i=0; i < priv->events.count; i++) {
		int j;
		
		struct isp1181_event *event = &priv->events.list[i];

		if (event->hnum>=0)
			seq_printf(file, " %2i ", event->hnum);
		else
			seq_printf(file, "    ");
		
		switch (event->type) {
		    case EVT_DATA:
			seq_printf(file, "data=%s(%i)",
				   event->data.dir ? "in" : "out",
				   event->data.len);

			for (j=0; j<min(15, event->data.len); j++)
				seq_printf(file, " %02x", (unsigned int) event->data.buf[j]);
			if (j<event->data.len)
				seq_printf(file, " [...]");

			break;
			
		    case EVT_EP_ENABLE:
			seq_printf(file, "ep_enable");
			break;

		    case EVT_EP_DISABLE:
			seq_printf(file, "ep_disable");
			break;

		    case EVT_CONFIG_EP:
			seq_printf(file, "config_endpoints");
			break;

		    case EVT_STALL:
			seq_printf(file, "stall");
			break;
			
		    case EVT_USTALL:
			seq_printf(file, "ustall");
			break;
			
		    case EVT_IRQ:
			seq_printf(file, "%s %08x", (event->hnum>=0) ? "hirq" : "irq", event->irq);
			break;

		    case EVT_IRQ_SIM:
			seq_printf(file, "%s %08x", (event->hnum>=0) ? "hirq_sim" : "irq_sim", event->irq);
			break;

		    case EVT_RESET:
			seq_printf(file, "reset");
			break;

		    case EVT_RESUME:
			seq_printf(file, "resume");
			break;

		    case EVT_SUSPEND:
			seq_printf(file, "suspend");
			break;

		    case EVT_REQ_DONE:
			seq_printf(file, "request done %p", event->req);
			break;

		    case EVT_QUEUE_REQ_IN:
			seq_printf(file, "request in %p queued", event->req);
			break;

		    case EVT_QUEUE_REQ_OUT:
			seq_printf(file, "request out %p queued", event->req);
			break;

		    case EVT_SETUP_OVERWRITTEN:
			seq_printf(file, "setup overwritten");
			break;

		    default:
			seq_printf(file, "0x%x", event->type);
			break;
			
		    break;
		}

		seq_printf(file, "\n");
	}

	/* display all pending requests. Typically there should be some for OUT endpoints,
	   but at max only one for IN endpoints */
	seq_printf(file, "Pending Requests\n");
	for (hnum=0; hnum<HW_EP_TOTAL; hnum++) {
		struct isp1181_request *req;
		i=0;

		list_for_each_entry (req, &priv->hep[hnum].queue, queue) {
			i++;
		}

		if (i)
			seq_printf(file, "  %i %i\n", hnum, i);
	}

	seq_printf(file, "HEP Status\n");
	for (hnum=0; hnum<HW_EP_TOTAL; hnum++) {
		if (hep_is_enabled(&priv->hep[hnum]))
			/* is enabled */
			seq_printf(file, "  %i %08x (%i %i bytes %i packets %i irqs)\n",
				   hnum, isp1181_cmd_inb(priv, CHK_EP(hnum)),
				   hnum, priv->stats[hnum].io, priv->stats[hnum].pkts, priv->stats[hnum].irqs);
	}

	spin_unlock_irqrestore(&priv->lock, flags);
	
	return 0;
}

static int isp1181_debugfs_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, isp1181_debugfs_dbg_seq_show, inode->i_private);
}

static const struct file_operations isp1181_debugfs_dbg_fops = {
	.owner		= THIS_MODULE,
	.open		= isp1181_debugfs_dbg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int isp1181_debugfs_init(struct isp1181_priv *priv)
{
	priv->debugfs.dir = debugfs_create_dir(dev_name(priv->dev), NULL);
	if (!priv->debugfs.dir)
		goto error;

	priv->debugfs.frame = debugfs_create_file("frame", S_IRUSR, priv->debugfs.dir,
						  priv, &isp1181_debugfs_frame_fops);
	if (!priv->debugfs.frame)
		goto error_frame;

	priv->debugfs.dbg = debugfs_create_file("dbg", S_IRUSR, priv->debugfs.dir,
						 priv, &isp1181_debugfs_dbg_fops);
	if (!priv->debugfs.dbg)
		goto error_dbg;

	return 0;

error_dbg:
	debugfs_remove(priv->debugfs.frame);

error_frame:
	debugfs_remove(priv->debugfs.dir);
	
error:
	return -ENODEV;
}

static void isp1181_debugfs_remove(struct isp1181_priv *priv)
{
	debugfs_remove(priv->debugfs.dbg);
	debugfs_remove(priv->debugfs.frame);
	debugfs_remove(priv->debugfs.dir);
}
