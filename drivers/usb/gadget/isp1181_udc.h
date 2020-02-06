/*
 * isp1181_udc.h
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

#ifndef _ISP1181_UDC_H
#define _ISP1181_UDC_H

/* initialization */
/* hardware has two registers for EP0-OUT and EP0-IN  */
#define HW_EP_TOTAL	16
#define HW_EP0_OUT	0
#define HW_EP0_IN	1

#define to_hnum(n) 		((n>0)?((n)+1):HW_EP0_IN) /* ep[0] is both EP0-OUT and EP0-IN */

#define WRITE_CFG(n)		(0x20+((n)&0xF))
#define READ_CFG(n)		(0x30+((n)&0xF))
#define WRITE_DEV_ADDR		0xB6
#define READ_DEV_ADDR		0xB7
#define WRITE_MODE		0xB8
#define READ_MODE		0xB9
#define WRITE_HW_CFG		0xBA
#define READ_HW_CFG		0xBB
#define WRITE_INT_ENABLE	0xC2
#define READ_INT_ENABLE		0xC3
#define WRITE_DMA_CFG		0xF0
#define READ_DMA_CFG		0xF1
#define WRITE_DMA_CNT		0xF2
#define READ_DMA_CNT		0xF3
#define RESET			0xF6

/* data flow commands */
#define WRITE_EP(n)		(0x00+((n)&0xF))
#define READ_EP(n)		(0x10+((n)&0xF))
#define STALL_EP(n)		(0x40+((n)&0xF))
#define STAT_EP(n)		(0x50+((n)&0xF))
#define VAL_EP(n)		(0x60+((n)&0xF))
#define CLR_EP(n)		(0x70+((n)&0xF))
#define USTALL_EP(n)		(0x80+((n)&0xF))
#define CHK_EP(n)		(0xD0+((n)&0xF))
#define ACK_SETUP		0xF4

/* General commands */
#define READ_ERR(n)		(0xA0+((n)&0xF))
#define UNLOCK_DEV		0xB0
#define WRITE_SCRATCH		0xB2
#define READ_SCRATCH		0xB3
#define READ_FRAME		0xB4
#define READ_CHIP_ID		0xB5
#define READ_INT_STATUS		0xC0

/* bitmasks */

#define CTRL_FIFOEN		(1<<7)
#define CTRL_EPDIR_OUT		(0<<6)
#define CTRL_EPDIR_IN		(1<<6)
#define CTRL_DBLBUF		(1<<5)
#define CTRL_FFOISO		(1<<4)
#define CTRL_FFOSZ(n)		((n)&0xF)

/* for WRITE_INT_ENABLE, READ_INT_ENABLE and READ_INT_STATUS */
#define INT_EP_ALL		(0x00FFFF00)
#define INT_EP(n)		(1<<(((n)&0xF)+8))
#define INT_BUSTATUS		(1<<7)
#define INT_SP_EOT		(1<<6)
#define INT_PSOF		(1<<5)
#define INT_SOF			(1<<4)
#define INT_EOT			(1<<3)
#define INT_SUSPND		(1<<2)
#define INT_RESUME		(1<<1)
#define INT_RESET		(1<<0)

#define CFG_EXTPUL		(1<<14)
#define CFG_NOLAZY		(1<<13)
#define CFG_CLKRUN		(1<<12)
#define CFG_CLKDIV(n)		(((n)&0xF)<<8)
#define CFG_DAKOLY		(1<<7)
#define CFG_DRQPOL		(1<<6)
#define CFG_DAKPOL		(1<<5)
#define CFG_EOTPOL		(1<<4)
#define CFG_WKUPCS		(1<<3)
#define CFG_PWROFF		(1<<2)
#define CFG_INTLVL_LVL		(0<<1)
#define CFG_INTLVL_EDGE		(1<<1)
#define CFG_INTPOL_HIGH		(1<<0)
#define CFG_INTPOL_LOW		(0<<0)

#define MODE_DMAWD_16		(1<<7)
#define MODE_GOSUSP		(1<<5)
#define MODE_INTENA		(1<<3)
#define MODE_DBGMOD		(1<<2)
#define MODE_SOFTCT		(1<<0)

#define ERR_UNREAD		(1<<7)
#define ERR_RTOK		(1<<0)

#define STAT_EPSTAL		(1<<7)
#define STAT_EPFULL1		(1<<6)
#define STAT_EPFULL0		(1<<5)
#define STAT_DATA_PID		(1<<4)
#define STAT_OVERWRITE		(1<<3)
#define STAT_SETUPT		(1<<2)
#define STAT_CPUBUF		(1<<1)

#define ADDR_DEVEN		(1<<7)

#define CHIPID_CODE_ISP1181	0x81

/* other constants */

#define FIFO_SIZE_TOTAL		2462
#define FIFO_SIZE_MAX_FOR_ALL	64

struct isp1181_priv;

struct isp1181_request {
	struct usb_request	req;
	struct list_head	queue;
};

/* endpoint from linux point of view */
struct isp1181_ep {
	struct isp1181_priv	*priv;
	struct usb_ep		ep;
	int                     wedge;
	const struct usb_endpoint_descriptor *desc;

	int                     num; /* EP0...EPn */
};

/* endpoint from isp1181 point of view */
struct isp1181_hep {
	struct isp1181_ep      *ep;
	int 			ffosz;
	int 			dir_out;

	struct list_head	queue;

	int                     hnum; /* HW_EP0_OUT...HW_EP_TOTAL-1 */

	/* active request on an IN endpoint */
	struct {
		struct isp1181_request  *req;
		int size;
	} active_in;
};

static const char *const isp1181_ep_name[] = {
	"ep0", "ep1", "ep2", "ep3", "ep4", "ep5", "ep6", "ep7",
	"ep8", "ep9", "ep10", "ep11", "ep12", "ep13", "ep14",
};
#define ISP1181_ENDPOINTS ARRAY_SIZE(isp1181_ep_name)

#ifdef CONFIG_USB_ISP1181_DEBUG_TRACE
enum isp1181_event_type {
	EVT_DATA,
	EVT_STALL,
	EVT_USTALL,
	EVT_IRQ,
	EVT_IRQ_SIM,
	EVT_RESET,
	EVT_RESUME,
	EVT_SUSPEND,
	EVT_EP_ENABLE,
	EVT_EP_DISABLE,
	EVT_CONFIG_EP,
	EVT_REQ_DONE,
	EVT_QUEUE_REQ_IN,
	EVT_QUEUE_REQ_OUT,
	EVT_SETUP_OVERWRITTEN,
};

struct isp1181_event {
	enum isp1181_event_type type;
	int hnum;

	/* only valid on EVT_DATA */
	union {
		struct {
			int dir;
			int len;
			u8  buf[FIFO_SIZE_MAX_FOR_ALL];
		} data;
		u32 irq;
		void *req;
	};
};
#endif	/* CONFIG_USB_ISP1181_DEBUG_TRACE */

struct isp1181_priv {
	spinlock_t     		 lock;
	struct device 		 *dev;
	void __iomem  		 *base;

	u8                       addr;

	struct usb_gadget	 gadget;
	struct usb_gadget_driver *driver;

	struct isp1181_ep	 ep[ISP1181_ENDPOINTS];
	struct isp1181_hep	 hep[HW_EP_TOTAL];

	/* for request we have on ep0 */
	struct isp1181_request*	 ep0req;
	u8 			 ep0req_buf[FIFO_SIZE_MAX_FOR_ALL];

	int 	       		 irq;
	int                      irqflags;
        struct tasklet_struct    tasklet;

	struct {
		/* caching of interrupt status from isr to tasklet */
		u32 status;
		u8  hep[HW_EP_TOTAL];
	} isr;

#ifdef CONFIG_USB_ISP1181_DEBUG_TRACE
	struct {
		int io;
		int pkts;
		int irqs;
	} stats[HW_EP_TOTAL];

	struct {
		int count;
		struct isp1181_event list[4096];
	} events;

	struct {
		struct dentry* 	dir;
		struct dentry* 	frame;
		struct dentry* 	dbg;
	} debugfs;
#endif	/* CONFIG_USB_ISP1181_DEBUG_TRACE */
};

static inline struct isp1181_priv* to_priv(struct usb_gadget *_gadget)
{
	return container_of(_gadget, struct isp1181_priv, gadget);
}

static inline struct isp1181_request* to_isp1181_request(struct usb_request *_req)
{
	return container_of(_req, struct isp1181_request, req);
}

static inline struct isp1181_ep* to_isp1181_ep(struct usb_ep *_ep)
{
	return container_of(_ep, struct isp1181_ep, ep);
}

static inline struct isp1181_hep* to_isp1181_hep(const struct isp1181_ep *ep)
{
	return &ep->priv->hep[to_hnum(ep->num)];
}
static inline int hep_is_enabled(const struct isp1181_hep *hep)
{
	return hep->ep->desc != NULL;
}

#ifndef CONFIG_USB_ISP1181_DEBUG_TRACE
# define isp1181_trace(...) 		do {} while(0)
# define isp1181_trace_data(...) 	do {} while(0)
# define isp1181_trace_irq(...) 	do {} while(0)
# define isp1181_trace_req(...) 	do {} while(0)
# define isp1181_debugfs_init(...) 	({0;})
# define isp1181_debugfs_remove(...) 	do {} while(0)
#endif

#endif
