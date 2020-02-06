/*
 * isp1181_udc.c - driver for isp1181 USB peripheral controller
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
 * References:  ISP1181B.pdf (Rev. 03 -23 January 2009)
 *
 **/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/irq.h>

#include "isp1181_udc.h"

static int isp1181_get_frame(struct usb_gadget *gadget);

static struct isp1181_priv *the_controller;
static const char udc_name[] = "isp1181_udc";

/* ********** lowlevel I/O ********** */

static u8 isp1181_data_inb(struct isp1181_priv *priv)
{
	u8 tmp = __raw_readb(priv->base);

	return tmp;
}

static u16 isp1181_data_inw(struct isp1181_priv *priv)
{
	u16 tmp = __raw_readw(priv->base);

	return tmp;
}

static u32 isp1181_data_inl(struct isp1181_priv *priv)
{
	u16 lo  = __raw_readw(priv->base);
	u16 hi  = __raw_readw(priv->base);
	u16 res = hi << 16 | lo;
	
	return res;
}

static void isp1181_data_outb(struct isp1181_priv *priv, u8 val)
{
	__raw_writeb(val, priv->base);
}

static void isp1181_data_outw(struct isp1181_priv *priv, u16 val)
{
	__raw_writew(val, priv->base);
}

static void isp1181_data_outl(struct isp1181_priv *priv, u32 val)
{
	__raw_writew(val&0xFFFF, priv->base);
	__raw_writew(val>>16, priv->base);
}

static void isp1181_cmd(struct isp1181_priv *priv, u8 cmd)
{
	__raw_writeb(cmd, priv->base + 0x0002);
}

static void isp1181_cmd_outb(struct isp1181_priv *priv, u8 cmd, u8 val)
{
	isp1181_cmd(priv, cmd);
	isp1181_data_outb(priv, val);
}

static void isp1181_cmd_outl(struct isp1181_priv *priv, u8 cmd, u32 val)
{
	isp1181_cmd(priv, cmd);
	isp1181_data_outl(priv, val);
}

static u8 isp1181_cmd_inb(struct isp1181_priv *priv, u8 cmd)
{
	isp1181_cmd(priv, cmd);
	return isp1181_data_inb(priv);
}

static u16 isp1181_cmd_inw(struct isp1181_priv *priv, u8 cmd)
{
	isp1181_cmd(priv, cmd);
	return isp1181_data_inw(priv);
}

static u32 isp1181_cmd_inl(struct isp1181_priv *priv, u8 cmd)
{
	u32 data;
	
	isp1181_cmd(priv, cmd);
	data = isp1181_data_inl(priv);

	return data;
}

/* ********** debugging ********** */

#ifdef CONFIG_USB_ISP1181_DEBUG_TRACE
# include "isp1181_udc_dbg.c"
#endif

/* ********** higher-level I/O ********** */

static void isp1181_write_fifo(struct isp1181_priv *priv, const struct isp1181_hep *hep,
			       const void *data, int len)
{
	const struct isp1181_ep *ep = hep->ep;
	int hnum = hep->hnum;

	/* parameter sanity check */
	if (len > ep->ep.maxpacket) {
		dev_err(priv->dev,
			"hep %i, FIFO overrun: want to write %i bytes, but can only %i\n",
			hnum, len, ep->ep.maxpacket);
		return;
	}
	if (unlikely(!hep_is_enabled(hep))) {
		dev_err(priv->dev, "hep %i, try to write to not enabled FIFO\n", hnum);
		return;
	}
	if (unlikely(hep->dir_out)) {
		dev_err(priv->dev, "trying to write to OUT endpoint %i\n", hnum);
		return;
	}

	isp1181_trace_data(priv, hnum, 1, len, data);

	isp1181_cmd(priv, WRITE_EP(hnum));
	isp1181_data_outw(priv, len);
	while (len > 1) {
		u16 val = *(const u16*) data;

		data += 2;
		len  -= 2;
		isp1181_data_outw(priv, val);
	}

	if (len)
		/* write remaining byte */
		isp1181_data_outb(priv, *(const u8*) data);

	isp1181_cmd(priv, VAL_EP(hnum));
}

static int isp1181_read_fifo(struct isp1181_priv *priv, struct isp1181_hep *hep,
			     void *data, int maxlen, int setup)
{
	struct isp1181_ep *ep = hep->ep;
	int hnum = hep->hnum;
	int len;
	int i;
#ifdef CONFIG_USB_ISP1181_DEBUG_TRACE
	void *start = data;
#endif

	/* parameter sanity check */
	if (unlikely(!hep_is_enabled(hep))) {
		dev_err(priv->dev, "hep %i, trying to read from not enabled FIFO\n", hnum);
		return -EINVAL;
	}

	if (unlikely(!hep->dir_out)) {
		dev_err(priv->dev, "trying to read from IN endpoint\n");
		return -EINVAL;
	}
	
	isp1181_cmd(priv, READ_EP(hnum));
	i = len = isp1181_data_inw(priv);
	if (len > ep->ep.maxpacket) {
		dev_err(priv->dev,
			"hep %i, FIFO overrun: want to read %i bytes, but have only space for %i\n",
			hnum, len, ep->ep.maxpacket);
		/* we can't use the data */
		isp1181_cmd(priv, CLR_EP(hnum));
		return -EINVAL;
	}

	while (i > 1) {
		u16 val = isp1181_data_inw(priv);
		*(u16*) data = val;
		data += 2;
		i  -= 2;
	}

	if (i) {
		/* read remaining byte */
		u8 val = isp1181_data_inb(priv);
		*(u8*) data = val;
	}

	if (setup)
		/* we are ready to receive new setup packet */
		isp1181_cmd(priv, ACK_SETUP);

	isp1181_cmd(priv, CLR_EP(hnum));

	isp1181_trace_data(priv, hnum, 0, len, start);
	
	return len;
}

/* ********** misc ********** */

static int isp1181_get_ffosz(int size, int isochronous)
{
	int ffosz;
	/* programmable FIFO size (table 5)  */
	static const int table_non_iso[16] = {
		8, 16,  32, 64,
		0,  0,  0,  0,
		0,  0,  0,  0,
		0,  0,  0,  0 };
	static const int table_iso[16] = {
		16,    32,   48,   64,
		96,   128,  160,  192,
		256,  320,  384,  512,
		640,  867,  896, 1024 };
	static const int *table;

	table = isochronous ? table_iso : table_non_iso;

	ffosz = 0;
	while (ffosz<16) {
		if (table[ffosz] == size)
			return ffosz;
		ffosz++;
	}

	/* not found */
	return -1;
}

static void isp1181_write_fifo_active_in(struct isp1181_priv *priv, struct isp1181_hep *hep)
{
	isp1181_write_fifo(priv,
			   hep,
			   hep->active_in.req->req.buf+hep->active_in.req->req.actual,
			   hep->active_in.size);
}

/* ********** requests ********** */

/**
 * isp1181_req_done - clean-up of the request and call of complete
 * @return: 1 if the request queue was empty before calling complete, otherwise 0
 *
 * complete() might queue further requests, so when leaving, empty can be 1,
 * but list_empty(&hep->queue) might return 0
 */
static int isp1181_req_done(struct isp1181_hep *hep, struct isp1181_request *req, int status)
{
	int empty;
	
	isp1181_trace_req(hep->ep->priv, EVT_REQ_DONE, hep->hnum, req);

	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;

	if (hep->active_in.req == req)
		hep->active_in.req = NULL;

	empty = list_empty(&hep->queue);
	
	req->req.complete(&hep->ep->ep, &req->req);

	/* req.complete might have queued new requests */
	return empty;
}

static void isp1181_nuke(struct isp1181_hep *hep, int status)
{
	/* terminate all requests */
	while (!list_empty(&hep->queue)) {
		struct isp1181_request *req = list_entry(hep->queue.next, struct isp1181_request, queue);
		isp1181_req_done(hep, req, status);
	}
}

static void isp1181_hep_handle_in_req(struct isp1181_priv *priv, struct isp1181_hep *hep)
{
	if (!hep_is_enabled(hep))
		/* already removed */
		return;

	if (list_empty(&hep->queue))
		/* no requests at all */
		return;

	if (hep->active_in.req) {
		int last = 0;
		
		hep->active_in.req->req.actual += hep->active_in.size;
		
		/* check if we can complete the request */

		if (hep->active_in.size != hep->ep->ep.maxpacket)
			/* short packet */
			last = 1;
		else if (hep->active_in.req->req.length == hep->active_in.req->req.actual &&
			 (!hep->active_in.size || !hep->active_in.req->req.zero))
			 last = 1;

		hep->active_in.size = 0;

		if (last && isp1181_req_done(hep, hep->active_in.req, 0))
			/* all data from the request has been sent. No further requests were queued.
			   complete by req_done() might have queued new entries,
			   but then already started the queue handling */
			return;
	}

	/* select the next request */
	hep->active_in.req = list_entry(hep->queue.next, struct isp1181_request, queue);
	hep->active_in.size = min((int)(hep->active_in.req->req.length-hep->active_in.req->req.actual),
				  ((int) hep->ep->ep.maxpacket));

	isp1181_write_fifo_active_in(priv, hep);
}

static void isp1181_hep_handle_out_req(struct isp1181_priv *priv, struct isp1181_hep *hep)
{
	struct isp1181_request *req;
	int size;
	int len;
		
	if (list_empty(&hep->queue)) {
		/* no further request pending, ignore data */
		isp1181_cmd(priv, CLR_EP(hep->hnum));
		return;
	}
	
	req = list_entry(hep->queue.next, struct isp1181_request, queue);
	
	size = min((int)(req->req.length-req->req.actual), ((int) hep->ep->ep.maxpacket));
	
	len = isp1181_read_fifo(priv, hep, req->req.buf+req->req.actual, size, 0);
	req->req.actual += len;
	
	if (len<hep->ep->ep.maxpacket || (req->req.length == req->req.actual))
		isp1181_req_done(hep, req, 0);
}

static void isp1181_config_endpoints(struct isp1181_priv *priv)
{
	int hnum;
	u32 irqs;

	isp1181_trace(priv, EVT_CONFIG_EP, -1);

	irqs = isp1181_cmd_inl(priv, READ_INT_ENABLE);
	irqs &= ~INT_EP_ALL;

	/* clearing all buffers is necessary when one endpoint has changed */
	for (hnum=0; hnum<HW_EP_TOTAL; hnum++)
		isp1181_cmd(priv, CLR_EP(hnum));

	/* configure all endpoints in sequence */
	for (hnum=0; hnum<HW_EP_TOTAL; hnum++) {
		const struct isp1181_hep *hep = &priv->hep[hnum];
		u8 ctrl = 0;

		if (hep_is_enabled(hep)) {
			/* is enabled */
			ctrl |= CTRL_FIFOEN;
			ctrl |= hep->dir_out ? CTRL_EPDIR_OUT : CTRL_EPDIR_IN;
			if (usb_endpoint_xfer_isoc(hep->ep->desc))
				ctrl |= CTRL_FFOISO;
			ctrl |= CTRL_FFOSZ(hep->ffosz);
			irqs |= INT_EP(hnum);
		}
		
		isp1181_cmd_outb(priv, WRITE_CFG(hnum), ctrl);
	}

	/* FIFO has been cleared. Refill all active/pending IN endpoints */
	for (hnum=0; hnum<HW_EP_TOTAL; hnum++) {
		struct isp1181_hep *hep = &priv->hep[hnum];

		if (hep->active_in.req)
			isp1181_write_fifo_active_in(priv, hep);
	}

	isp1181_cmd_outl(priv, WRITE_INT_ENABLE, irqs);
}

static void isp1181_hep_queue_req(struct isp1181_priv *priv, struct isp1181_hep *hep,
				  struct isp1181_request *req)
{
	int idle = list_empty(&hep->queue);

	isp1181_trace_req(priv,
			  hep->dir_out ? EVT_QUEUE_REQ_OUT : EVT_QUEUE_REQ_IN,
			  hep->hnum, req);

	req->req.status = -EINPROGRESS;
	req->req.actual = 0;

	list_add_tail(&req->queue, &hep->queue);

	if (!hep->dir_out && idle)
		/* start transmission */
		isp1181_hep_handle_in_req(priv, hep);
}

static int isp1181_hep_dequeue_req(struct isp1181_hep *hep, struct isp1181_request *req_to_dequeue)
{
	struct isp1181_request *req;

	/* only dequeue request when it is still listed */
	list_for_each_entry (req, &hep->queue, queue) {
		if (req == req_to_dequeue) {
			isp1181_req_done(hep, req, -ECONNRESET);
			return 0;
		}
	}

	return -EINVAL;
}

/* ********** endpoints ********** */

static int isp1181_ep_enable(struct usb_ep *_ep,
			     const struct usb_endpoint_descriptor *desc)
{
	struct isp1181_ep   *ep   = to_isp1181_ep(_ep);
	struct isp1181_hep  *hep  = to_isp1181_hep(ep);
	struct isp1181_priv *priv = ep->priv;
	unsigned long flags;
	int ret = 0;
	
	spin_lock_irqsave(&priv->lock, flags);

	hep->ffosz = isp1181_get_ffosz(ep->ep.maxpacket, usb_endpoint_xfer_isoc(desc));
	if (hep->ffosz<0) {
		dev_err(priv->dev, "Invalid packet size %i for endpoint %s\n",
			ep->ep.maxpacket, ep->ep.name);
		ret = -EINVAL;
		goto out;
	}
	

	isp1181_trace(priv, EVT_EP_ENABLE, hep->hnum);

	hep->active_in.req = NULL;
	ep->desc = desc;

	if (ep->num)
		/* direction of ep0 is fixed */
		hep->dir_out = usb_endpoint_dir_out(desc) ? 1 : 0;
	
	isp1181_config_endpoints(priv);

out:
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static int isp1181_ep_disable(struct usb_ep *_ep)
{
	struct isp1181_ep  *ep    = to_isp1181_ep(_ep);
	struct isp1181_hep  *hep  = to_isp1181_hep(ep);
	struct isp1181_priv *priv = ep->priv;
	unsigned long flags;
	
	spin_lock_irqsave(&priv->lock, flags);

	isp1181_trace(priv, EVT_EP_DISABLE, hep->hnum);

	ep->desc = NULL;

	if (!ep->num) {
		isp1181_nuke(&priv->hep[HW_EP0_OUT], -ESHUTDOWN);
		isp1181_nuke(&priv->hep[HW_EP0_IN], -ESHUTDOWN);
	} else
		isp1181_nuke(hep, -ESHUTDOWN);

	isp1181_config_endpoints(priv);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int isp1181_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct isp1181_ep   *ep     = to_isp1181_ep(_ep);
	struct isp1181_hep  *hep    = to_isp1181_hep(ep);
	struct isp1181_priv *priv   = ep->priv;
	struct isp1181_request *req = to_isp1181_request(_req);
	unsigned long flags;
	
	if (!_ep || !req || !ep->desc || !_req->complete || !_req->buf) {
		dev_err(priv->dev, "invalid arguments for ep_queue\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&priv->lock, flags);
	isp1181_hep_queue_req(priv, hep, req);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int isp1181_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct isp1181_ep  *ep      = to_isp1181_ep(_ep);
	struct isp1181_hep  *hep    = to_isp1181_hep(ep);
	struct isp1181_request *req = to_isp1181_request(_req);
	struct isp1181_priv *priv   = ep->priv;
	unsigned long flags;
	int ret;
	
	spin_lock_irqsave(&priv->lock, flags);

	ret = isp1181_hep_dequeue_req(hep, req);
	if (ret && !ep->num)
		/* we didn't find it in HW_EP0_IN */
		ret = isp1181_hep_dequeue_req(&priv->hep[HW_EP0_OUT], req);
	
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static struct usb_request *isp1181_ep_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct isp1181_request *req;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void isp1181_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct isp1181_request *req = to_isp1181_request(_req);

	WARN_ON(!list_empty(&req->queue));

	/* ensure the request is not kept in the queue */
	isp1181_ep_dequeue(_ep, _req);
	kfree(req);
}

static void isp1181_hep_stall(struct isp1181_priv *priv, struct isp1181_hep *hep, int stall)
{
	isp1181_trace(priv, stall ? EVT_STALL : EVT_USTALL, hep->hnum);

	isp1181_cmd(priv, stall ? STALL_EP(hep->hnum) : USTALL_EP(hep->hnum));

	if (!stall && hep->active_in.req)
		/* fifo has been cleared on USTALL, refill it */
		isp1181_write_fifo(priv,
				   hep,
				   hep->active_in.req->req.buf+hep->active_in.req->req.actual,
				   hep->active_in.size);
}

static void isp1181_set_halt(struct isp1181_priv *priv, struct isp1181_ep *ep, int value)
{
	if (!ep->num) {
		/* do EP0-IN and EP0-OUT */
		isp1181_hep_stall(priv, &priv->hep[HW_EP0_OUT], value);
		isp1181_hep_stall(priv, &priv->hep[HW_EP0_IN], value);
	} else
		isp1181_hep_stall(priv, &priv->hep[to_hnum(ep->num)], value);
}

static int isp1181_ep_set_halt_locked(struct usb_ep *_ep, int value)
{
	struct isp1181_ep  *ep    = to_isp1181_ep(_ep);
	struct isp1181_hep  *hep  = to_isp1181_hep(ep);
	struct isp1181_priv *priv = ep->priv;
	int active;
	
	if (!_ep)
		return -EINVAL;

	active = (hep->active_in.req != NULL);
	if (!ep->num && !active)
		active = (priv->hep[HW_EP0_OUT].active_in.req != NULL);

	if (!active) {
		/* we can't halt an endpoint which might be read by the host in the moment */
		isp1181_set_halt(priv, ep, value);
		return 0;
	} else
		return -EAGAIN;
}

static int isp1181_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct isp1181_ep   *ep   = to_isp1181_ep(_ep);
	struct isp1181_priv *priv = ep->priv;
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&priv->lock, flags);

	retval = isp1181_ep_set_halt_locked(_ep, value);

	if (!value)
		ep->wedge = 0;

	spin_unlock_irqrestore(&priv->lock, flags);

	return retval;
}

static int isp1181_ep_set_wedge(struct usb_ep *_ep)
{
	struct isp1181_ep  *ep    = to_isp1181_ep(_ep);
	struct isp1181_priv *priv = ep->priv;
	unsigned long flags;
	int retval;
	
	spin_lock_irqsave(&priv->lock, flags);
	ep->wedge = 1;
	retval = isp1181_ep_set_halt_locked(_ep, 1);
	spin_unlock_irqrestore(&priv->lock, flags);

	return retval;
}

static void isp1181_ep_fifo_flush(struct usb_ep *_ep)
{
	struct isp1181_ep  *ep    = to_isp1181_ep(_ep);
	struct isp1181_hep  *hep  = to_isp1181_hep(ep);
	struct isp1181_priv *priv = ep->priv;
	unsigned long flags;
	
	spin_lock_irqsave(&priv->lock, flags);
	isp1181_cmd(priv, CLR_EP(hep->hnum));
	spin_unlock_irqrestore(&priv->lock, flags);
}

static struct usb_ep_ops isp1181_ep_ops = {
	.enable		= isp1181_ep_enable,
	.disable	= isp1181_ep_disable,

	.alloc_request	= isp1181_ep_alloc_request,
	.free_request	= isp1181_ep_free_request,

	.queue		= isp1181_ep_queue,
	.dequeue	= isp1181_ep_dequeue,

	.set_halt	= isp1181_ep_set_halt,
	.set_wedge	= isp1181_ep_set_wedge,
	.fifo_flush	= isp1181_ep_fifo_flush,
};

/* ********** interrupt handling ********** */

static int isp1181_check_for_error(struct isp1181_priv *priv, struct isp1181_hep *hep)
{
	static const char* code_readable[16] = {
		"no error",
		"PID encoding",
		"PID unknown",
		"unexpected packet",
		"token CRC",
		"data CRC",
		"timeout",
		"babble",
		"unexpected-end-of-packet",
		"NAK",
		"stalled",
		"FIFO overflow",
		"sent empty packet",
		"bit stuffing error",
		"sync error",
		"wrong toggle bit in DATA",
	};

	u8  hwerror = isp1181_cmd_inb(priv, READ_ERR(hep->hnum));
	u8  code = (hwerror >> 1) & 0xF;
	int ret = 0;

	/* ignore ERR_UNREAD as we might have been busy with other work and
	   maybe have not respond in time */

	if (code) {
		dev_err(priv->dev, "error on hep %i: %s\n", hep->hnum, code_readable[code]);
		ret = -1;
	}
	
	return ret;
}

/* ********** ep0 ********** */

static void isp1181_ep0req_complete(struct usb_ep *_ep, struct usb_request *_req)
{
	_req->length = -1;
}

static void isp1181_ep0req_queue(struct isp1181_priv *priv, int len, const void *buf)
{
	struct isp1181_hep *hep = &priv->hep[HW_EP0_IN];
	
	/* kill any former message on EP0-IN as we are overwriting it. */
	isp1181_nuke(&priv->hep[HW_EP0_IN], -EIO);

	priv->ep0req->req.length = len;
	if (len)
		memcpy(priv->ep0req->req.buf, buf, len);

	isp1181_hep_queue_req(priv, hep, priv->ep0req);
}

static void isp1181_write_zero(struct isp1181_priv *priv)
{
	isp1181_ep0req_queue(priv, 0, NULL);
}

static void isp1181_set_addr(struct isp1181_priv *priv)
{
	isp1181_cmd_outb(priv, WRITE_DEV_ADDR, ADDR_DEVEN | priv->addr);
	/* address change needs to be acknowledged with an empty package */
	isp1181_write_zero(priv);
}

static int isp1181_get_status(struct isp1181_priv *priv, const struct usb_ctrlrequest *crq)
{
	u16 status = 0;
	u8  ep_num  = crq->wIndex & 0x7F;
	int hnum;
	u8  buf[2];

	switch (crq->bRequestType & USB_RECIP_MASK) {
	    case USB_RECIP_DEVICE: /* GetHubStatus */
		status = 0; /* not self powered */
		break;

	    case USB_RECIP_ENDPOINT: /* GetPortStatus */
		if (ep_num >= ISP1181_ENDPOINTS || crq->wLength > 2)
			return -EINVAL;

		hnum = to_hnum(ep_num);

		if (!hep_is_enabled(&priv->hep[hnum]) ||
		    (isp1181_cmd_inb(priv, CHK_EP(hnum)) & STAT_EPSTAL))
			status = 1 << USB_ENDPOINT_HALT;

		break;
	}

	buf[0] = status & 0xFF;
	buf[1] = status >> 8;
	
	isp1181_ep0req_queue(priv, 2, &status);

	return 0;
}

static void isp1181_handle_setup(struct isp1181_priv *priv)
{
	struct usb_ctrlrequest crq;
	struct isp1181_ep *ep;
	int len;
	int handled = 0;
	int ep_num;

	len = isp1181_read_fifo(priv, &priv->hep[HW_EP0_OUT], &crq, sizeof(crq), 1);

	if (!len)
		/* ignore zero packet */
		return;
	
	if (len > sizeof(crq)) {
		dev_err(priv->dev, "setup packet length mismatch, need %i, have %i\n",
			sizeof(crq), len);
		goto stall;
	}

	switch (crq.bRequest) {
	    case USB_REQ_SET_ADDRESS:
		if (crq.bRequestType == USB_RECIP_DEVICE) {
			priv->addr = crq.wValue & 0x7F;
			isp1181_set_addr(priv);
			return;
		}
		break;

	    case USB_REQ_GET_STATUS:
		handled = (isp1181_get_status(priv, &crq)>=0);
		break;

	    case USB_REQ_CLEAR_FEATURE:	/* fall-through */
	    case USB_REQ_SET_FEATURE:
		if (crq.bRequestType != USB_RECIP_ENDPOINT)
			break;
		if (crq.wValue != USB_ENDPOINT_HALT || crq.wLength != 0)
			break;
		ep_num = crq.wIndex & 0x7f;
		if (!ep_num)
			break;
		if (ep_num >= ISP1181_ENDPOINTS)
			break;

		ep = &priv->ep[ep_num];
		if (crq.bRequest==USB_REQ_CLEAR_FEATURE) {
			if (!ep->wedge)
				isp1181_set_halt(priv, ep, 0);
		} else
			isp1181_set_halt(priv, ep, 1);
		
		isp1181_write_zero(priv);
		handled = 1;
		break;
		
	    default:
		break;
	}

	if (!handled && priv->driver->setup(&priv->gadget, &crq)<0)
		goto stall;

	return;

stall:	
	isp1181_set_halt(priv, &priv->ep[0], 1);
}

static void isp1181_handle_ep0out(struct isp1181_priv *priv, u8 stat)
{
	if (stat & STAT_OVERWRITE) {
		/* data is destroyed, we need to wait for the next setup */
		isp1181_cmd(priv, ACK_SETUP);
		isp1181_cmd(priv, CLR_EP(HW_EP0_OUT));

		isp1181_trace(priv, EVT_SETUP_OVERWRITTEN, HW_EP0_OUT);
		isp1181_set_halt(priv, &priv->ep[0], 1);

		return;
	}

	if (stat & STAT_SETUPT)
		isp1181_handle_setup(priv);
	else
		isp1181_hep_handle_out_req(priv, &priv->hep[HW_EP0_OUT]);
}

static void isp1181_handle_hep(struct isp1181_priv *priv, struct isp1181_hep *hep)
{
	int hnum = hep->hnum;
	u8 stat;

	stat = priv->isr.hep[hnum];
	priv->isr.hep[hnum] = 0;

	isp1181_trace_irq(priv, hnum, stat, 0);

	isp1181_check_for_error(priv, hep);

	if ((stat & STAT_CPUBUF) || (stat & STAT_EPFULL1))
		dev_err(priv->dev, "data in secondary buffer while double buffering is not implemented\n");

	if (!hep_is_enabled(hep))
		return;

	if (hnum != HW_EP0_OUT) {
		if (hep->dir_out)
			isp1181_hep_handle_out_req(priv, hep);
		else
			isp1181_hep_handle_in_req(priv, hep);
	} else
		isp1181_handle_ep0out(priv, stat);
}

/* ********** interrupt handling ********** */

static void isp1181_tasklet_isr(unsigned long data)
{
	struct isp1181_priv *priv = (struct isp1181_priv*) data;
	unsigned long flags;
	u32 status;
	int hnum;
	
	spin_lock_irqsave(&priv->lock, flags);
	status = priv->isr.status;
	priv->isr.status = 0;
	
	if (status & INT_RESUME) {
		/* remove write-protection from registers */
		isp1181_cmd(priv, UNLOCK_DEV);
		/* clear previous write-protected status register */
		status |= isp1181_cmd_inl(priv, READ_INT_STATUS);
	}
	
	if (status & INT_RESET) {
		isp1181_trace(priv, EVT_RESET, -1);

		priv->driver->disconnect(&priv->gadget);

		/* endpoints had been disabled by hardware, bring them online again*/
		for (hnum=0; hnum<HW_EP_TOTAL; hnum++)
			isp1181_nuke(&priv->hep[hnum], -ECONNRESET);

		isp1181_config_endpoints(priv);
		isp1181_set_addr(priv);
	}

	if (status & INT_RESUME) {
		isp1181_trace(priv, EVT_RESUME, -1);

		if (priv->driver && priv->driver->resume)
			priv->driver->resume(&priv->gadget);
	}

	if (status & INT_SUSPND) {
		isp1181_trace(priv, EVT_SUSPEND, -1);

		if (priv->driver && priv->driver->suspend)
			priv->driver->suspend(&priv->gadget);
	}

	/* handle endpoints.
	 * First process all IN endpoints, then all OUT endpoints.
	 * We need to complete a request before queuing the same again. Some drivers have
	 * allocated only one request, e.g. composite_setup(). If the host is fast enough
	 * or when we have missed an ACK, we will be called with interrupts for HW_EP0_IN
	 * (completes the old one) and HW_EP1_OUT (queuing the next one). This does not work with
	 * the same request used again */

	/* all IN endpoints */
	for (hnum=0; hnum < HW_EP_TOTAL; hnum++) {
		struct isp1181_hep *hep = &priv->hep[hnum];

		if (hep->dir_out)
			continue;
		
		if (status & INT_EP(hnum))
			isp1181_handle_hep(priv, hep);
	}

	/* all OUT endpoints */
	for (hnum=0; hnum < HW_EP_TOTAL; hnum++) {
		struct isp1181_hep *hep = &priv->hep[hnum];

		if (!hep->dir_out)
			continue;

		if (status & INT_EP(hnum))
			isp1181_handle_hep(priv, hep);
	}

	spin_unlock_irqrestore(&priv->lock, flags);
}

static irqreturn_t isp1181_isr(int irq, void *dev_id)
{
	struct isp1181_priv *priv = dev_id;
	int hnum;
	u32 status;
	
	spin_lock(&priv->lock);

	/* interrupts line stays active even when MODE_INTENA is cleared in WRITE_MODE.
	   Therefore we can't just disable interrupts and then call tasklet/worker thread.
	   We need to read and store all ISR status values and remember them until tasklet is
	   scheduled */
	status = isp1181_cmd_inl(priv, READ_INT_STATUS);
	priv->isr.status |= status;
	isp1181_trace_irq(priv, -1, status, 0);

	for (hnum=0; hnum < HW_EP_TOTAL; hnum++) {
		u8 stat_hep = isp1181_cmd_inb(priv, STAT_EP(hnum));

		priv->isr.hep[hnum] |= stat_hep;

		if (!(status & INT_EP(hnum))) {
			/* Sometimes the ISP1181 will not set an interrupt status for an
			   endpoint event when all requirements are met. Therefore we check whether
			   there should have been an interrupt and set the flag ourselves */
			/* Maybe this should also be done by a watchdog, if there was no
			   activity for some time. So far this seemed unnecessary as there was
			   always some activity on a different endpoint */
			struct isp1181_hep *hep = &priv->hep[hnum];

			if ((hep->active_in.req && !(stat_hep & (STAT_EPFULL1 | STAT_EPFULL0))) ||
			    /* the buffer has been read, but not acknowledged */
			    (hep->dir_out && (stat_hep & (STAT_EPFULL1 | STAT_EPFULL0)))) {
				priv->isr.status |= INT_EP(hnum);
				isp1181_trace_irq(priv, -1, priv->isr.status, 1);
			}
		}
	}
	
	tasklet_schedule(&priv->tasklet);

	spin_unlock(&priv->lock);

	return IRQ_HANDLED;
}

/* ********** high-level I/O ********** */

static int isp1181_start(struct isp1181_priv *priv)
{
	unsigned long flags;
	int error;
	int intflags;

	/* a dummy descriptor simplifies handling of ep0 */
	static struct usb_endpoint_descriptor dummy_ep0_desc = {
		.bLength =		USB_DT_ENDPOINT_SIZE,
		.bDescriptorType =	USB_DT_ENDPOINT,
		.bEndpointAddress =	USB_DIR_OUT,
		.bmAttributes =		USB_ENDPOINT_XFER_CONTROL,
	};

	/* setup hardware */
	spin_lock_irqsave(&priv->lock, flags);

	/* reset last status */
	memset(&priv->isr, 0, sizeof(priv->isr));

	intflags = 0;
	if (priv->irqflags == IRQF_TRIGGER_LOW)
		intflags = CFG_INTPOL_LOW | CFG_INTLVL_LVL;
	else if (priv->irqflags == IRQF_TRIGGER_HIGH)
		intflags = CFG_INTPOL_HIGH | CFG_INTLVL_LVL;
	else if (priv->irqflags == IRQF_TRIGGER_FALLING)
		intflags = CFG_INTPOL_HIGH | CFG_INTLVL_EDGE;
	else if (priv->irqflags == IRQF_TRIGGER_RISING)
		intflags = CFG_INTPOL_LOW | CFG_INTLVL_EDGE;

	isp1181_cmd_outl(priv, WRITE_HW_CFG,
			 CFG_NOLAZY |
			 CFG_DAKOLY |
			 CFG_PWROFF |
			 intflags);

	/* enable interrupts */
	isp1181_cmd_outl(priv, WRITE_INT_ENABLE,
			 INT_SP_EOT    |
			 INT_SUSPND    |
			 INT_RESUME    |
			 INT_RESET);

	error = isp1181_ep_enable(priv->gadget.ep0, &dummy_ep0_desc);
	if (error) {
		/* disable any transfers and interrupts */
		isp1181_cmd(priv, RESET);
		spin_unlock_irqrestore(&priv->lock, flags);
		goto error_ep0;
	}

	isp1181_cmd_outb(priv, WRITE_MODE,
			 MODE_DMAWD_16 |
			 MODE_INTENA   |
			 MODE_SOFTCT);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;

error_ep0:
	return -error;
}

static void isp1181_stop(struct isp1181_priv *priv)
{
	unsigned long flags;
	
	/* disable any transfers and interrupts */
	spin_lock_irqsave(&priv->lock, flags);
	isp1181_cmd(priv, RESET);
	spin_unlock_irqrestore(&priv->lock, flags);

	/* interrupts are disabled */
        tasklet_kill(&priv->tasklet);
}

static int isp1181_reset_hw(struct isp1181_priv *priv)
{
	u16 id;
	
	/* ensure it's an isp1181 */
	id = isp1181_cmd_inw(priv, READ_CHIP_ID);
	dev_info(priv->dev, "chip id is 0x%04x\n", id);
	if (id >> 8 != CHIPID_CODE_ISP1181) {
		dev_info(priv->dev, "chip is not valid\n");
		return -ENODEV;
	}

	isp1181_cmd(priv, RESET);
	priv->addr = 0x0;
	
	return 0;
}

static int isp1181_get_frame(struct usb_gadget *gadget)
{
	struct isp1181_priv *priv = to_priv(gadget);
	unsigned long flags;
	int frame;
	
	spin_lock_irqsave(&priv->lock, flags);
	frame = isp1181_cmd_inw(priv, READ_FRAME);
	spin_unlock_irqrestore(&priv->lock, flags);

	return frame;
}

static struct usb_gadget_ops isp1181_gadget_ops = {
	.get_frame = isp1181_get_frame,
};

/* ********** driver management ********** */

static int isp1181_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct isp1181_priv *priv;
	struct usb_request* _req;
	int error;
	int hnum;
	int num;
	
	/* allocate resources */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
 		dev_err(&pdev->dev, "cannot get memory\n");
		error = -ENOMEM;
		goto error;
	}
	priv->dev = &pdev->dev;
	spin_lock_init(&priv->lock);
	platform_set_drvdata(pdev, priv);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get platform memory\n");
		error = -EINVAL;
		goto error_map;
	}
	priv->base = ioremap_nocache(res->start, (res->end-res->start)+1);
	if (!priv->base) {
		dev_err(&pdev->dev, "cannot get io memory\n");
		error = -ENOMEM;
		goto error_map;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get platform irq\n");
		error = -EINVAL;
		goto error_irqres;
	}
	priv->irqflags = res->flags & IRQF_TRIGGER_MASK;
	priv->irq = res->start;

        tasklet_init(&priv->tasklet, isp1181_tasklet_isr, (unsigned long) priv);
	
	/* initialize debugging */
	error = isp1181_debugfs_init(priv);
	if (error)
		goto error_debugfs;

	error = isp1181_reset_hw(priv);
	if (error)
		goto error_hw;

	the_controller = priv;

	/* initialize gadget */
	priv->gadget.ops = &isp1181_gadget_ops;
	device_initialize(&priv->gadget.dev);
	dev_set_name(&priv->gadget.dev, "gadget");
	priv->gadget.is_dualspeed = 0; /* only fullspeed is supported */
	priv->gadget.dev.parent   = &pdev->dev;
	priv->gadget.dev.dma_mask = pdev->dev.dma_mask;
	priv->gadget.dev.release  = pdev->dev.release;
	priv->gadget.name         = udc_name;
	priv->gadget.speed        = USB_SPEED_FULL;

	/* initialize endpoints */

	INIT_LIST_HEAD(&priv->gadget.ep_list);
	priv->gadget.ep0 = &priv->ep[0].ep;
	for (num=0; num<ISP1181_ENDPOINTS; num++) {
		struct isp1181_ep *ep = &priv->ep[num];

		ep->ep.name = isp1181_ep_name[num];
		ep->ep.ops  = &isp1181_ep_ops;
		ep->priv    = priv;

		ep->num     = num;

		INIT_LIST_HEAD(&ep->ep.ep_list);

		/* non-isochronous and isochronous endpoints are configurable
		   for various FIFO sizes. To keep it simple, treat all endpoints identical.*/
		ep->ep.maxpacket = FIFO_SIZE_MAX_FOR_ALL;

		if (num)
			list_add_tail(&priv->ep[num].ep.ep_list, &priv->gadget.ep_list);
	}

	/* initialize hw endpoints (there are HW_EP0_IN and HW_EP0_OUT for ep0) */
	for (hnum=0; hnum<HW_EP_TOTAL; hnum++) {
		struct isp1181_hep *hep = &priv->hep[hnum];

		hep->hnum = hnum;
		INIT_LIST_HEAD(&hep->queue);

		if (hnum > 1)
			hep->ep = &priv->ep[hnum-1];
		else {
			/* HW_EP0_IN and HW_EP0_OUT share the same endpoint */
			hep->ep = &priv->ep[0];

			hep->dir_out = (hnum == HW_EP0_OUT) ? 1 : 0;
		}
	}

	/* request for setup responses created from us */
	_req = isp1181_ep_alloc_request(&priv->ep[0].ep, GFP_KERNEL);
	if (_req == NULL)
		goto error_ep0req;
	priv->ep0req 		   = to_isp1181_request(_req);
	priv->ep0req->req.complete = isp1181_ep0req_complete;
	priv->ep0req->req.length   = -1;
	priv->ep0req->req.buf      = priv->ep0req_buf;
	
	/* clear all interrupts */
	for (hnum=0; hnum<HW_EP_TOTAL; hnum++)
		(void) isp1181_cmd_inl(priv, STAT_EP(hnum));

        if (request_irq(priv->irq, isp1181_isr,
			priv->irqflags,
			dev_name(priv->dev), priv)) {
                dev_err(priv->dev, "cannot get interrupt\n");
                error = -ENOMEM;
                goto error_irq;
        }

	return 0;

error_irq:
	isp1181_ep_free_request(&priv->ep[0].ep, &priv->ep0req->req);

error_ep0req:
	isp1181_debugfs_remove(priv);
	
error_hw:
error_debugfs:
error_irqres:
	iounmap(priv->base);
	
error_map:
	kfree(priv);
	
error:
	return error;
}

static int isp1181_remove(struct platform_device *pdev)
{
	struct isp1181_priv *priv = dev_get_drvdata(&pdev->dev);

        free_irq(priv->irq, priv);
	isp1181_ep_free_request(&priv->ep[0].ep, &priv->ep0req->req);
	isp1181_debugfs_remove(priv);
	iounmap(priv->base);
	kfree(priv);
	the_controller = NULL;

	return 0;
}

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct isp1181_priv *priv = the_controller;
	int error;

	if (!driver || !driver->bind || !driver->setup)
		return -EINVAL;
	if (!priv)
		return -ENODEV;
	if (priv->driver)
		return -EBUSY;
	
	/* hook up the driver */
	driver->driver.bus = NULL;
	priv->driver = driver;
	priv->gadget.dev.driver = &driver->driver;

	dev_info(priv->dev, "binding gadget driver '%s'\n", driver->driver.name);

	error = device_add(&priv->gadget.dev);
	if (error)
		goto error_add;
	
	error = driver->bind(&priv->gadget);
	if (error)
		goto error_bind;

	error = isp1181_start(priv);
	if (error)
		goto error_start;
	
	return 0;

error_start:
	driver->unbind(&priv->gadget);

error_bind:
	device_del(&priv->gadget.dev);

error_add:
	priv->driver = NULL;
	priv->gadget.dev.driver = NULL;

	return error;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct isp1181_priv *priv = the_controller;

	if (!priv)
		return -ENODEV;
	if (!driver || driver != priv->driver || !driver->unbind)
		return -EINVAL;

	dev_info(priv->dev, "unbinding gadget driver '%s'\n", driver->driver.name);

	priv->driver->disconnect(&priv->gadget);

	isp1181_stop(priv);
	driver->unbind(&priv->gadget);
	device_del(&priv->gadget.dev);
	priv->gadget.dev.driver = NULL;
	priv->driver = NULL;

	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

static struct platform_driver isp1181_driver = {
	.driver = {
		.name  = udc_name,
		.owner = THIS_MODULE,
	},
	.probe  = isp1181_probe,
	.remove = isp1181_remove,
};

static int __init isp1181_init(void)
{
	return platform_driver_probe(&isp1181_driver, isp1181_probe);
}
module_init(isp1181_init);

static void __exit isp1181_exit(void)
{
	platform_driver_unregister(&isp1181_driver);
}
module_exit(isp1181_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Markus Pietrek");
MODULE_DESCRIPTION("ISP1181 udc driver");
