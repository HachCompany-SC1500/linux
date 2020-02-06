/*

  DM9sw.c: Version 2.03

        Davicom DM9006 Local Bus/PCI NIC fast Ethernet driver for Linux.

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.


  (C)Copyright 1997-2007 DAVICOM Semiconductor,Inc. All Rights Reserved.

V0.1	bill	2006/08
	- creat
V0.2	bill	2006/12
	- enable promiscuous mode (DM9013 V2 bug)
	- remove early transmit (DM9013 don't have this function)
	- remove cont_rx_pkt_cnt
	- add dm9013_switch_config()
	- add dm9013_switch_reset() (when reseting switch core, some registers don't be reset. Driver must reset them if NO EEPROM. )
2007/05/07 bill
	- Remove promiscuous mode. Test this for DM9013 V3.
	- Avoid the continuing coming(RX) packets to cause the driver fell into the while loop. Add cont_rx_pkt_cnt.
2007/05/30 bill
	- fix driver bug when ifconfig ethx (-)promisc and (-)allmulti
2007/09/21 judes
	- Add DM9302
2008/05/16 Joseph
	- 'CHECK_STR_V105'
	- 0.Remove <linux/config.h>
	- 1.Correct the 'db->port_mode' count dor DM9003 chip's device (= 2)
	- 2.Correct the loop count in each 'db->port_mode'
2008/05/23 Joseph
	- 'CHECK_STR_V106'
	- 0."port_set_PHY_mode()" added, for re-orginize the code.
	- 1.dm9013_timer() do PHY reset per port for if link-down
	     So add "port_timer_reset()"
	- 2.Add 'pe.n4HangCounts_port[DM9013_MAX_NUM_IPHY]' varibles
	     to be used in "port_timer_reset()"
2008/05/29 Joseph
	- 'CHECK_STR_V107'
	- 1.Add "IC_Notice()"
2008/06/03 Joseph
	- 'CHECK_STR_V108'
	- 1.Add "IC_PHY_Notice()"
	- 2.(port_timer_reset() call port_set_PHY_mode() call IC_PHY_Notice())
2008/06/11 Joseph
	- 'CHECK_STR_V109'
	- enhance dm9013_tx_done() function !
2008/06/18 Joseph
	- 'CHECK_STR_V110'
2010/05/14 Joseph
  -V111
  -modify dm9013_hash_table()(driver bug)
  (-)iow(db, DM9013_RXCR, (ior(db, DM9013_RXCR)|(1<<3)));
  (+)iow(db, DM9013_RXCR, (ior(db, DM9013_RXCR)&(~(1<<3))));
2010/09/20 Miller
  -V1.20
  -support kernel v2.6.31, v2.6.32
  -Fix 'SA_SHIRQ' unclared, struct 'net_device' has no member named 'priv'
2010/09/21 Miller
  -V1.21
  -support kernel v2.6.35
  -fix warnning: moudle mismatch
2010/12/09 Miller
	-V2.00 Support DM9006
2011/08/29 Stone
	-V2.03 Support DM9006 port0 reset bug!
*/
//#define CHECKSUM

#if defined(MODVERSIONS)
#include <linux/modversions.h>
#endif

//#include <linux/config.h> //'CHECK_STR_V105.0'
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>

#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/version.h>
#include <asm/dma.h>
#include <linux/spinlock.h>
#include <linux/crc32.h>

#include "dm9sw.h"
#include "dm9000.h"
#include "linux/dm9000.h"

#define DM9013_ID		0x90130A46
#define DM9003_ID		0x90030A46
#define DM9006_ID		0x90060A46

#define DM9013_REGFF	(IMR_SRAM_autoReturn|IMR_TX_complete|IMR_RX_coming)
#define DM9013_NO_RX_INTR (IMR_SRAM_autoReturn|IMR_TX_complete)
#define DM9013_DISINTR	IMR_SRAM_autoReturn

#define CARDNAME	"dm9013"
#define DRV_VERSION	"2.03"


#define DM9013_MIN_IO		(0x1a000000)
#define DM9013_MAX_IO		(DM9013_MIN_IO + 15)
#define DM9013_IRQ		(36)


#define DM9013_TIMER_WUT  jiffies+(HZ)
#define DM9013_TX_TIMEOUT (HZ*5)

/* TODO: check these settings: */
//#define NAPI       1
//#define CHECKSUM   1

#undef DM9013_DEBUG 
#if defined(DM9013_DEBUG)

static int dm9013_debug = 1;

#define DM9013_DBUG(dbug_now, msg, vaule)\
if (dm9013_debug||dbug_now) printk(KERN_ERR "dm9013: %s %x\n", msg, vaule)
#define DM9013_DBUG_STR(dbug_now, msg)\
if (dm9013_debug||dbug_now) printk(KERN_ERR "dm9013: %s\n", msg)

#else

#define DM9013_DBUG(dbug_now, msg, vaule)\
if (dbug_now) printk(KERN_ERR "dm9013: %s %x\n", msg, vaule)
#define DM9013_DBUG_STR(dbug_now, msg)\
if (dbug_now) printk(KERN_ERR "dm9013: %s\n", msg)

#endif

#define wait_bit_clear(reg, bit)\
wait_count=0;\
while(1){\
	if(!(ior(db,reg) & (1<<bit)))\
		break;\
	if(wait_count ==10000)\
		printk("wait time out, reg0x%x\n",reg);\
	wait_count++;\
}
#define wait_PHY_bit_clear(PHY_ID,reg, bit)\
wait_count=0;\
while(1){\
	if(!(phy_read(db,reg,PHY_ID) & (1<<bit)))\
		break;\
	if(wait_count ==10000)\
		printk("wait PHY time out, reg0x%x\n",reg);\
	wait_count++;\
}
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)  //V1.21
#define DM9SW_BOARD_DB	board_info_t *db = (board_info_t *)netdev_priv(dev)
#else
#define DMF9SW_BOARD_DB	board_info_t *db = (board_info_t *)dev->priv
#endif

/* Structure declaration ------------------------------- */
typedef struct board_info {
	u32		reset_counter;		/* counter: RESET */
	u32		reset_tx_timeout;	/* RESET caused by TX Timeout */
	u32		io_addr;		/* Register I/O base address */
	u32		io_data;		/* Data I/O address */
	int		tx_pkt_cnt;
	int		port_mode;
	u8		op_mode;		/* PHY operation mode */
	u8		io_mode;		/* 0:word, 2:byte */
	u8		stop_transmit;
	int		cont_rx_pkt_cnt;	/* current number of continuos rx packets  */
	struct timer_list timer;
	struct net_device_stats stats;
	unsigned char	srom[128];
	spinlock_t	lock;
	void (*MoveData)(struct board_info *,unsigned char *, int , int);
	int		HasEEPROM;


	/* take from dm9000: */
	unsigned int	flags;
	unsigned int	in_suspend :1;
	unsigned int	wake_supported :1;
	int		debug_level;

	struct mutex	addr_lock;	/* phy and eeprom access lock */
	
	struct mii_if_info mii;
	u32		msg_enable;
	u32		wake_state;

	int		rx_csum;
	int		can_csum;
	int		ip_summed;


} board_info_t;

static inline board_info_t *to_dm9013_board(struct net_device *dev)
{
	return netdev_priv(dev);
}


typedef struct Param_Ext_Info_StartV105 {  // 'CHECK_STR_V105.x, Add'
	u32     id_val;
	u8 	n4HangCounts_port[DM9013_MAX_NUM_IPHY];

} TParamExt_StartV105;

static TParamExt_StartV105 pe;

void IC_Notice(board_info_t *db); 				//CHECK_STR_V108
void IC_PHY_Notice(board_info_t *db, int i);	//CHECK_STR_V108

/* Global variable declaration ----------------------------- */
static struct net_device * dm9013_dev = NULL;
/* For module input parameter */
static int media_mode = DM9013_AUTO;
static int  irq        = DM9013_IRQ;
static u32 iobase     = DM9013_MIN_IO;
static int wait_count;

/* define output/input functions by myself */
#if 0
#ifdef outb
	#undef outb
#endif
#ifdef outw
	#undef outw
#endif
#ifdef outl
	#undef outl
#endif
#ifdef inb
	#undef inb
#endif
#ifdef inw
	#undef inw
#endif
#ifdef inl
	#undef inl
#endif
void outb(u8 reg, u32 ioaddr)
{
	(*(volatile u8 *)(ioaddr)) = reg;
}
void outw(u16 reg, u32 ioaddr)
{
	(*(volatile u16 *)(ioaddr)) = reg;
}
void outl(u32 reg, u32 ioaddr)
{
	(*(volatile u32 *)(ioaddr)) = reg;
}
u8 inb(u32 ioaddr)
{
	return (*(volatile u8 *)(ioaddr));
}
u16 inw(u32 ioaddr)
{
	return (*(volatile u16 *)(ioaddr));
}
u32 inl(u32 ioaddr)
{
	return (*(volatile u32 *)(ioaddr));
}
#endif

/* function declaration ------------------------------------- */
int dm9013_probe(struct net_device *);
static int dm9013_open(struct net_device *);
static int dm9013_start_xmit(struct sk_buff *, struct net_device *);
static void dm9013_tx_done(unsigned long);
static void dm9013_packet_receive(struct net_device *);
static int dm9013_stop(struct net_device *);
static struct net_device_stats * dm9013_get_stats(struct net_device *);
static int dm9013_do_ioctl(struct net_device *, struct ifreq *, int);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,19) //V1.21
	static irqreturn_t dm9013_interrupt(int , void *);
	#else
	static irqreturn_t dm9013_interrupt(int , void *, struct pt_regs *);
	#endif
#else
static void dm9013_interrupt(int , void *, struct pt_regs *);
#endif
static void dm9013_timer(unsigned long);
static void dm9013_init(struct net_device *);
static unsigned long cal_CRC(unsigned char *, unsigned int, u8);
static u16 dm9013_read_eeprom_word(board_info_t *, int);
static void dm9013_hash_table(struct net_device *);
static void dm9013_timeout(struct net_device *);
static void dm9013_reset(struct net_device *);
static void dm9013_move8bit(board_info_t *,unsigned char *, int , int );
static void dm9013_move16bit(board_info_t *,unsigned char *, int , int );
static void dm9013_move32bit(board_info_t *,unsigned char *, int , int );
static u8 ior(board_info_t *, int);
static void iow(board_info_t *, int, u8);
static u16 phy_read(board_info_t *, int, int);
static void phy_write(board_info_t *, int, u16, int);
#if defined(CHECKSUM)
static u8 check_rx_ready(u8);
#endif
#ifdef NAPI /* Experiment */
static int dm9013_poll (struct net_device *, int *);
#endif
/* switch functions */
static int dm9013_vlan_port_vid(board_info_t *, int, u8);
static int dm9013_vlan_outpkt_tag(board_info_t *, int);
static void dm9013_vlan_group_map(board_info_t *,u8, u8);
static void dm9013_vlan_setup(board_info_t *,int);
static int dm9013_bw_control(board_info_t *, int, u8, u8, u8);
static int dm9013_qos_port_pri(board_info_t *, int, u8);
static int dm9013_qos_tos_enable(board_info_t *, int);
static void dm9013_QoS_setup(board_info_t *, int);
inline u8 dm9013_make_mask(int , int , int , int );


void read_MAC_store(char dest[]);   /* read + use the MAC which was set by others */

//DECLARE_TASKLET(dm9013_tx_tasklet,dm9013_tx_done,0);

/* DM9013 network baord routine ---------------------------- */

/*
 * Search DM9013 board, allocate space and register it
*/

struct net_device * __init dm9013_probe1(void)
{
	struct net_device *dev;
	int err;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	dev = init_etherdev(NULL, sizeof(struct board_info));
	ether_setup(dev);
#else
	dev= alloc_etherdev(sizeof(struct board_info));
#endif

	if(!dev)
		return ERR_PTR(-ENOMEM);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
 //V1.20
     	SET_MODULE_OWNER(dev);
#endif
	err = dm9013_probe(dev);
	if (err)
		goto out;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
	err = register_netdev(dev);
	if (err)
		goto out1;
#endif
	return dev;
out1:
	release_region(dev->base_addr,2);
out:
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	kfree(dev);
#else
	free_netdev(dev);
#endif
	return ERR_PTR(err);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)  //V1.20

static const struct net_device_ops vm_netdev_ops = { // new kernel 2.6.31  (20100920)

            .ndo_open               = dm9013_open,
            .ndo_stop               = dm9013_stop,
            .ndo_start_xmit         = dm9013_start_xmit,
            .ndo_tx_timeout         = dm9013_timeout,
	    .ndo_get_stats          = dm9013_get_stats,
            .ndo_set_multicast_list = dm9013_hash_table,
            .ndo_do_ioctl           = dm9013_do_ioctl,
};
#endif



/*
 * ###########################################################################################################
 */


/*
 * Sleep, either by using msleep() or if we are suspending, then
 * use mdelay() to sleep.
 */
static void dm9013_msleep(board_info_t *db, unsigned int ms)
{
	if (db->in_suspend)
		mdelay(ms);
	else
		msleep(ms);
}

/*
 *   Read a word from phyxcer
 */
static int
dm9013_phy_read(struct net_device *dev, int phy_reg_unused, int reg)
{
	board_info_t *db = netdev_priv(dev);
	unsigned long flags;
	unsigned int reg_save;
	int ret;
	
	
	mutex_lock(&db->addr_lock);

	spin_lock_irqsave(&db->lock,flags);

	/* Save previous register address */
	reg_save = readb(db->io_addr);

	/* Fill the phyxcer register into REG_0C */
	iow(db, DM9000_EPAR, DM9013_PHY | reg);

	iow(db, DM9000_EPCR, EPCR_ERPRR | EPCR_EPOS);	/* Issue phyxcer read command */

	writeb(reg_save, db->io_addr);
	spin_unlock_irqrestore(&db->lock,flags);

	dm9013_msleep(db, 1);		/* Wait read complete */

	spin_lock_irqsave(&db->lock,flags);
	reg_save = readb(db->io_addr);

	iow(db, DM9000_EPCR, 0x0);	/* Clear phyxcer read command */

	/* The read data keeps on REG_0D & REG_0E */
	//ret = (ior(db, DM9000_EPDRH) << 8) | ior(db, DM9000_EPDRL);

	iow(db, DM9013_PPCR, 0x0); 
	ret = (ior(db, DM9013_PPSDR) & (1<<0)) ? BMSR_LSTATUS : 0;
	
	/* restore the previous address */
	writeb(reg_save, db->io_addr);
	spin_unlock_irqrestore(&db->lock,flags);

	mutex_unlock(&db->addr_lock);

	//printk(KERN_INFO "phy_read[%02x] -> %04x\n", reg, ret);
	return ret;
}

/*
 *   Write a word to phyxcer
 */
static void
dm9013_phy_write(struct net_device *dev,
		 int phyaddr_unused, int reg, int value)
{
	board_info_t *db = netdev_priv(dev);
	unsigned long flags;
	unsigned long reg_save;

	printk(KERN_INFO "phy_write[%02x] = %04x\n", reg, value);
	mutex_lock(&db->addr_lock);

	spin_lock_irqsave(&db->lock,flags);

	/* Save previous register address */
	reg_save = readb(db->io_addr);

	/* Fill the phyxcer register into REG_0C */
	iow(db, DM9000_EPAR, DM9013_PHY | reg);

	/* Fill the written data into REG_0D & REG_0E */
	iow(db, DM9000_EPDRL, value);
	iow(db, DM9000_EPDRH, value >> 8);

	iow(db, DM9000_EPCR, EPCR_EPOS | EPCR_ERPRW);	/* Issue phyxcer write command */

	writeb(reg_save, db->io_addr);
	spin_unlock_irqrestore(&db->lock, flags);

	dm9013_msleep(db, 1);		/* Wait write complete */

	spin_lock_irqsave(&db->lock,flags);
	reg_save = readb(db->io_addr);

	iow(db, DM9000_EPCR, 0x0);	/* Clear phyxcer write command */

	/* restore the previous address */
	writeb(reg_save, db->io_addr);

	spin_unlock_irqrestore(&db->lock, flags);
	mutex_unlock(&db->addr_lock);
}


static unsigned int
dm9013_read_locked(board_info_t *db, int reg)
{
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&db->lock, flags);
	ret = ior(db, reg);
	spin_unlock_irqrestore(&db->lock, flags);

	return ret;
}

static int dm9013_wait_eeprom(board_info_t *db)
{
	unsigned int status;
	int timeout = 8;	/* wait max 8msec */

	/* The DM9000 data sheets say we should be able to
	 * poll the ERRE bit in EPCR to wait for the EEPROM
	 * operation. From testing several chips, this bit
	 * does not seem to work.
	 *
	 * We attempt to use the bit, but fall back to the
	 * timeout (which is why we do not return an error
	 * on expiry) to say that the EEPROM operation has
	 * completed.
	 */

	while (1) {
		status = dm9013_read_locked(db, DM9000_EPCR);

		if ((status & EPCR_ERRE) == 0)
			break;

		msleep(1);

		if (timeout-- < 0) {
			printk(KERN_INFO "timeout waiting EEPROM\n");
			break;
		}
	}

	return 0;
}

/*
 *  Read a word data from EEPROM
 */
static void
dm9013_read_eeprom(board_info_t *db, int offset, u8 *to)
{
	unsigned long flags;

	if (db->flags & DM9000_PLATF_NO_EEPROM) {
		to[0] = 0xff;
		to[1] = 0xff;
		return;
	}

	mutex_lock(&db->addr_lock);

	spin_lock_irqsave(&db->lock, flags);

	iow(db, DM9000_EPAR, offset);
	iow(db, DM9000_EPCR, EPCR_ERPRR);

	spin_unlock_irqrestore(&db->lock, flags);

	dm9013_wait_eeprom(db);

	/* delay for at-least 150uS */
	msleep(1);

	spin_lock_irqsave(&db->lock, flags);

	iow(db, DM9000_EPCR, 0x0);

	to[0] = ior(db, DM9000_EPDRL);
	to[1] = ior(db, DM9000_EPDRH);

	spin_unlock_irqrestore(&db->lock, flags);

	mutex_unlock(&db->addr_lock);
}

/*
 * Write a word data to SROM
 */
static void
dm9013_write_eeprom(board_info_t *db, int offset, u8 *data)
{
	unsigned long flags;

	if (db->flags & DM9000_PLATF_NO_EEPROM)
		return;

	mutex_lock(&db->addr_lock);

	spin_lock_irqsave(&db->lock, flags);
	iow(db, DM9000_EPAR, offset);
	iow(db, DM9000_EPDRH, data[1]);
	iow(db, DM9000_EPDRL, data[0]);
	iow(db, DM9000_EPCR, EPCR_WEP | EPCR_ERPRW);
	spin_unlock_irqrestore(&db->lock, flags);

	dm9013_wait_eeprom(db);

	mdelay(1);	/* wait at least 150uS to clear */

	spin_lock_irqsave(&db->lock, flags);
	iow(db, DM9000_EPCR, 0);
	spin_unlock_irqrestore(&db->lock, flags);

	mutex_unlock(&db->addr_lock);
}


/* ethtool ops */

static void dm9013_get_drvinfo(struct net_device *dev,
			       struct ethtool_drvinfo *info)
{
	//board_info_t *dm = to_dm9013_board(dev);

	strcpy(info->driver, CARDNAME);
	strcpy(info->version, DRV_VERSION);
	strcpy(info->bus_info, dev->name);
}

static u32 dm9013_get_msglevel(struct net_device *dev)
{
	board_info_t *dm = to_dm9013_board(dev);

	return dm->msg_enable;
}

static void dm9013_set_msglevel(struct net_device *dev, u32 value)
{
	board_info_t *dm = to_dm9013_board(dev);

	dm->msg_enable = value;
}

static int dm9013_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	board_info_t *dm = to_dm9013_board(dev);

	mii_ethtool_gset(&dm->mii, cmd);
	return 0;
}

static int dm9013_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	board_info_t *dm = to_dm9013_board(dev);

	return mii_ethtool_sset(&dm->mii, cmd);
}

static int dm9013_nway_reset(struct net_device *dev)
{
	board_info_t *dm = to_dm9013_board(dev);
	return mii_nway_restart(&dm->mii);
}

static uint32_t dm9013_get_rx_csum(struct net_device *dev)
{
	board_info_t *dm = to_dm9013_board(dev);
	return dm->rx_csum;
}

static int dm9013_set_rx_csum_unlocked(struct net_device *dev, uint32_t data)
{
	board_info_t *dm = to_dm9013_board(dev);

	if (dm->can_csum) {
		dm->rx_csum = data;
		iow(dm, DM9000_RCSR, dm->rx_csum ? RCSR_CSUM : 0);

		return 0;
	}

	return -EOPNOTSUPP;
}

static int dm9013_set_rx_csum(struct net_device *dev, uint32_t data)
{
	board_info_t *dm = to_dm9013_board(dev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dm->lock, flags);
	ret = dm9013_set_rx_csum_unlocked(dev, data);
	spin_unlock_irqrestore(&dm->lock, flags);

	return ret;
}

static int dm9013_set_tx_csum(struct net_device *dev, uint32_t data)
{
	board_info_t *dm = to_dm9013_board(dev);
	int ret = -EOPNOTSUPP;

	if (dm->can_csum)
		ret = ethtool_op_set_tx_csum(dev, data);
	return ret;
}

static u32 dm9013_get_link(struct net_device *dev)
{
	board_info_t *dm = to_dm9013_board(dev);
	u32 ret;

	if (dm->flags & DM9000_PLATF_EXT_PHY) {
		ret = mii_link_ok(&dm->mii);
		//printk(KERN_INFO "%s mii_link_ok() returns %d\n", __FUNCTION__, ret);
	}
	else {
		ret = dm9013_read_locked(dm, DM9000_NSR) & NSR_LINKST ? 1 : 0;
		//printk(KERN_INFO "%s dm9013_read_locked() returns %d\n", __FUNCTION__, ret);
	}

	return ret;
}

#define DM_EEPROM_MAGIC		(0x444D394B)

static int dm9013_get_eeprom_len(struct net_device *dev)
{
	DM9013_DBUG(0, "__FUNCTION__", 0);
	return 128;
}

static int dm9013_get_eeprom(struct net_device *dev,
			     struct ethtool_eeprom *ee, u8 *data)
{
	board_info_t *dm = to_dm9013_board(dev);
	int offset = ee->offset;
	int len = ee->len;
	int i;
	DM9013_DBUG(0, "__FUNCTION__", 0);

	/* EEPROM access is aligned to two bytes */

	if ((len & 1) != 0 || (offset & 1) != 0)
		return -EINVAL;

	if (dm->flags & DM9000_PLATF_NO_EEPROM)
		return -ENOENT;

	ee->magic = DM_EEPROM_MAGIC;

	for (i = 0; i < len; i += 2)
		dm9013_read_eeprom(dm, (offset + i) / 2, data + i);

	return 0;
}

static int dm9013_set_eeprom(struct net_device *dev,
			     struct ethtool_eeprom *ee, u8 *data)
{
	board_info_t *dm = to_dm9013_board(dev);
	int offset = ee->offset;
	int len = ee->len;
	int i;

	DM9013_DBUG(0, "__FUNCTION__", 0);

	/* EEPROM access is aligned to two bytes */

	if ((len & 1) != 0 || (offset & 1) != 0)
		return -EINVAL;

	if (dm->flags & DM9000_PLATF_NO_EEPROM)
		return -ENOENT;

	if (ee->magic != DM_EEPROM_MAGIC)
		return -EINVAL;

	for (i = 0; i < len; i += 2)
		dm9013_write_eeprom(dm, (offset + i) / 2, data + i);

	return 0;
}


static const struct ethtool_ops dm9013_ethtool_ops = {
	.get_drvinfo		= dm9013_get_drvinfo,
	.get_settings		= dm9013_get_settings,
	.set_settings		= dm9013_set_settings,
	.get_msglevel		= dm9013_get_msglevel,
	.set_msglevel		= dm9013_set_msglevel,
	.nway_reset		= dm9013_nway_reset,
	.get_link		= dm9013_get_link,
	.get_wol		= NULL, //dm9013_get_wol,
	.set_wol		= NULL, //dm9013_set_wol,
 	.get_eeprom_len		= dm9013_get_eeprom_len,
 	.get_eeprom		= dm9013_get_eeprom,
 	.set_eeprom		= dm9013_set_eeprom,
	.get_rx_csum		= dm9013_get_rx_csum,
	.set_rx_csum		= dm9013_set_rx_csum,
	.get_tx_csum		= ethtool_op_get_tx_csum,
	.set_tx_csum		= dm9013_set_tx_csum,
};

/*
 * ###########################################################################################################
 */


static u8 MAC_addr[6]={0x00,0x60,0x6E,0x33,0x44,0x55};

static int use_alt_macaddr = 0;
static int __init cmdl_force_alt_macaddr(char *str)
{
	int i, ret, val;
	u8 mac[6];
	char *pos;
	
	if ((str == NULL) || (*str == '\0') || (strnlen(str, 20) != 17))
		return 0;
	
	for (i = 0; i < 6; i++) {
		pos = str + 3 * i;
		ret = sscanf(pos, "%02x", &val);
		if (ret != 1)
			return 0;
		mac[i] = (u8)val & 0xff;
		use_alt_macaddr = use_alt_macaddr || (mac[i] != 0);
	}
	if (!use_alt_macaddr)
		return 0;
	
	
	for (i = 0; i < 6; i++) {
		MAC_addr[i] = mac[i];
	}
	return 1;
}



int __init dm9013_probe(struct net_device *dev)
{
	struct board_info *db;/* Point a board information structure */
	u32 pid_vid_ID = 0xffffffff;
	u16 i, dm9013_found = FALSE;
	DM9013_DBUG(0, "dm9013_probe",0);


/*
 * dm9006 uses bus 6B whereas dm9000 is connected to bus 5B.
 * Copy seting from bus 5B to bus 6B:
 */
	__raw_writel(__raw_readl(0xFEC10018), 0xFEC10020);  /* set CS6BBCR with value from CS5BBCR */
	__raw_writel(__raw_readl(0xFEC10038), 0xFEC10040);  /* set CS6BBCR with value from CS5BWCR */


/*
 * dm9006 uses IRQ4 which is a multiplexed pin (btw.: IRQ5 used by dm9000 is not a multiplexed pin)
 * set port W, pin to to "function 1" (IRQ4 input):
 */

#define PWCR      (0xa4050146)  /* Port W control register PWCR R/W H'A405 0146 */
#define PWRC_BIT  (4)

#define PSELD     (0xa4050154)  /* Pin select register D PSELD R/W H'A405 0154 */
#define PSELD_BIT (7)

	__raw_writew( __raw_readw(PWCR) & ~(((u16)3) << (2*PWRC_BIT)),
		     PWCR);
	__raw_writew( __raw_readw(PSELD) & ~(((u16)3) << (2*PSELD_BIT)),
		    PSELD);


#define IO_DATA_OFS 2  /* offset beween resources addr/index and data */

	/* Search All DM9013 serial NIC */
	do {
		/* get DM9013 ID */
		outb(DM9013_VID_L, iobase);
		pid_vid_ID = inb(iobase + IO_DATA_OFS);
		outb(DM9013_VID_H, iobase);
		pid_vid_ID |= inb(iobase + IO_DATA_OFS) << 8;
		outb(DM9013_PID_L, iobase);
		pid_vid_ID |= inb(iobase + IO_DATA_OFS) << 16;
		outb(DM9013_PID_H, iobase);
		pid_vid_ID |= inb(iobase + IO_DATA_OFS) << 24;


		pe.id_val= pid_vid_ID; // SAVE
		for (i=0;i<DM9013_MAX_NUM_IPHY;i++) pe.n4HangCounts_port[i]= 0;

		if ((pe.id_val == DM9013_ID)||(pe.id_val == DM9003_ID)||(pe.id_val == DM9006_ID))
		{
			/* Request IO from system */
			if(!request_region(iobase, 2, dev->name))
				return -ENODEV;

			printk("<DM9006> I/O: %x, VID: %x \n",iobase, pe.id_val);
			dm9013_found = TRUE;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)  //V1.20
			/* Allocated board information structure */
			memset(netdev_priv(dev), 0, sizeof(struct board_info));
			db = (board_info_t *)netdev_priv(dev);
#else
			/* Allocated board information structure */
			memset(dev->priv, 0, sizeof(struct board_info));
			db = (board_info_t *)dev->priv;
#endif
			dm9013_dev = dev;
			
			db->io_addr = (unsigned int)ioremap(iobase, IO_DATA_OFS);
			db->io_data = (unsigned int)ioremap(iobase+IO_DATA_OFS, IO_DATA_OFS);
						
			db->msg_enable = NETIF_MSG_LINK;
			
			
			/* driver system function */
			dev->base_addr = (unsigned int)db->io_addr;
			dev->irq = irq;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
			dev->netdev_ops = &vm_netdev_ops; // new kernel 2.6.31 V1.20
			dev->watchdog_timeo	= DM9013_TX_TIMEOUT;
			dev->ethtool_ops	= &dm9013_ethtool_ops;
#else
			dev->open		= &dm9013_open;
			dev->hard_start_xmit	= &dm9013_start_xmit;
			dev->watchdog_timeo	= DM9013_TX_TIMEOUT;
			dev->tx_timeout		= dm9013_timeout;
			dev->stop		= &dm9013_stop;
			dev->get_stats		= &dm9013_get_stats;
			dev->set_multicast_list = &dm9013_hash_table;
			dev->do_ioctl		= &dm9013_do_ioctl;
#endif
#ifdef NAPI /* Experiment */
			dev->weight = 16;
			dev->poll = dm9013_poll;
#endif

#ifdef CHECKSUM
			dev->features |=  NETIF_F_IP_CSUM;
#endif
			/* Read EEPROM content */
			for (i=0; i<64; i++)
				((u16 *)db->srom)[i] = dm9013_read_eeprom_word(db, i);

			/* Detect EEPROM */
			db->HasEEPROM = 0;
			if(((u16 *)db->srom)[5] == 0x9013)
				db->HasEEPROM = 1;


			/* look if an alternative MAC is given on the cmdline
			 * and store this in MAC_addr: 
			 */
			{
				__setup("alt_macaddr=", cmdl_force_alt_macaddr);
			}

			/* Set Node Address */
 			for (i=0; i<6; i++)
 			{
// 				if (db->HasEEPROM)
// 					dev->dev_addr[i] = db->srom[i];
// 				else
				dev->dev_addr[i] = MAC_addr[i];
 			}

			if (!use_alt_macaddr) {
				/* we use a foreign setting for our MAC: */
				read_MAC_store(dev->dev_addr);
			}
			
						
			db->wake_supported = 0;
			
			db->flags |= DM9000_PLATF_EXT_PHY;
			db->mii.phy_id_mask  = 0x1f;
			db->mii.reg_num_mask = 0x1f;
			db->mii.force_media  = 0;
			db->mii.full_duplex  = 0;
			db->mii.dev	     = dev;
			db->mii.mdio_read    = dm9013_phy_read;
			db->mii.mdio_write   = dm9013_phy_write;


			printk("<9013>  Fixed-MAC - Addr  %02x:%02x:%02x:%02x:%02x:%02x\n",
				dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
				dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]
      			);
		}//end of if()
		iobase += 0x10;
	} while(!dm9013_found && iobase <= DM9013_MAX_IO);

	return dm9013_found ? 0:-ENODEV;
}


/*
 * Open the interface.
 * The interface is opened whenever "ifconfig" actives it.
*/
static int dm9013_open(struct net_device *dev)
{
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	DM9013_DBUG(0, "dm9013_open", 0);

	if (netif_msg_ifup(db))
		printk(KERN_INFO"enabling %s\n", dev->name);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24) //V1.20
	if (request_irq(dev->irq, &dm9013_interrupt, IRQF_SHARED|IRQF_TRIGGER_RISING, dev->name, dev))
#else
	if (request_irq(dev->irq,&dm9013_interrupt,SA_SHIRQ,dev->name,dev))
#endif
		return -EAGAIN;

	/* Initilize DM9013 board */
	dm9013_init(dev);

 	switch(db->io_mode)
	{
		case DM9013_BYTE_MODE:
			db->MoveData = &dm9013_move8bit;
			break;
		case DM9013_WORD_MODE:
			db->MoveData = &dm9013_move16bit;
      			break;
    		case DM9013_DWORD_MODE:
       			db->MoveData = &dm9013_move32bit;
			break;
	}

	/* Init driver variable */
	db->reset_counter = 0;
	db->reset_tx_timeout = 0;
	db->cont_rx_pkt_cnt	= 0;

	/* set and active a timer process */
	init_timer(&db->timer);
	db->timer.expires = DM9013_TIMER_WUT;
	db->timer.data = (unsigned long)dev;
	db->timer.function = &dm9013_timer;
	add_timer(&db->timer);

	netif_start_queue(dev);

	return 0;
}

/* WinCE Version */
/*
inline WORD DEV_LSMode(int linkMode)
{
	WORD wData= 0;

	switch( m_LSMode= (LinkSpeed)linkMode ) { //m_LSMode= linkMode;
		case LINKSPEED_AUTO:
			wData |= 0x1200; //(bMII_BMCR_AUTO|bMII_BMCR_RESTART)
			break;
		case LINKSPEED_HALF10BASE:
		default:
			wData= 0;
			break; // 0000h
		case LINKSPEED_FULL10BASE:
			wData |= bMII_BMCR_DUPLEX; // 0100h
			break;
		case LINKSPEED_HALF100BASE:
			wData |= bMII_BMCR_SPEED; // 2000h
			break;
		case LINKSPEED_FULL100BASE:
			wData |= (bMII_BMCR_SPEED|bMII_BMCR_DUPLEX); // 2100h
			break;
		case LINKSPEED_LOOPBACK:
			// 100Base & Full Duplex
			wData |= (bMII_BMCR_LOOPBACK|bMII_BMCR_SPEED|bMII_BMCR_DUPLEX); // 6100h
			break;
	}
	return wData;
}*/
/* Linux Version */
inline u16 DEV_LSMode(u8 op_mode)
{
	u16 phy_reg0 = 0x1200;/* Auto-negotiation & Restart Auto-negotiation */

	if ( !(op_mode & DM9013_AUTO) ) // op_mode didn't auto sense */
	{
		switch(op_mode) {
			case DM9013_10MHD:
				phy_reg0 = 0;
				break;
			case DM9013_10MFD:
				phy_reg0 = (1<<8);
         			break;
			case DM9013_100MHD:
				phy_reg0 = (1<<13);
				break;
			case DM9013_100MFD:
				phy_reg0 = ((1<<13)|(1<<8));
				break;
			default:
				break;
		} // end of switch
	} // end of if
	return phy_reg0;
}

/* Set PHY Per Port //;No concider 'DM9302'

	for (i=0; i<db->port_mode; i++) // reset PHY
		-phy_write(db, 0, 0x8000, i);
	// wait for PHY ready
	for (i=0; i<db->port_mode; i++)
		-wait_PHY_bit_clear(i, 0, 15);

	for (i=0; i<db->port_mode; i++)     // phy Preamble Saving Control
		-phy_write(db, 20, 0x0C80, i);  // ==> IC_PHY_Notice()

	for (i=0; i<db->port_mode; i++)
		-phy_write(db, 0, phy_reg0,i);
*/

void port_set_PHY_mode(board_info_t *db, int i) // 'CHECK_STR_V106.0'
{
		/* reset & wait for PHY ready */
		phy_write(db, 0, 0x8000, i);
		wait_PHY_bit_clear(i, 0, 15);

		IC_PHY_Notice(db, i);

		phy_write(db, 0, DEV_LSMode(db->op_mode),i);
		//u16 phy_reg0
		//phy_reg0= DEV_LSMode(db->op_mode);
		//phy_write(db, 0, phy_reg0,i);
}

/* Set PHY operationg mode
*/
static void dm9013_set_PHY_mode(board_info_t *db)
{
	int i;


	for (i=0; i<db->port_mode; i++)
		port_set_PHY_mode(db, i);

	#ifdef DM9302	//judes
	    .../.df,. ,. .,b .trntrnrn... JJ-Test 20080516.. 'CHECK_STR_V105.x'...
		u16 tmp;
		for (i=0; i<=db->port_mode; i++)
		{
			phy_write(db,0x0,0x2100,i);  	/* Force 100MFDX */
			tmp=phy_read(db,0x10,i);
			tmp^=0x400; 									/* FX mode */
			phy_write(db,0x10,tmp|0x5000,i);	/* Force SD */
		}
	#endif
}

static void dm9013_enable_interrupt(board_info_t *db)
{
	iow(db, DM9013_IMR, DM9013_REGFF);
}

static void dm9013_disable_interrupt(board_info_t *db)
{
	iow(db, DM9013_IMR, DM9013_DISINTR);
}

static void dm9013_switch_reset(board_info_t *db)
{
	int i;
	/* reset VLAN mapping table */
	for (i=0xb0; i<=0xbf; i++)
		dm9013_vlan_group_map(db, i, dm9013_make_mask(1, 1, 1, 1));
	/* reset Per-Port VID*/
	for(i=0; i<4; i++)
		dm9013_vlan_port_vid(db,i,1);
	/* reset VLAN control */
	iow (db, DM9013_VLANCR, 0);
	/* reset Per-Port switch control */
	for(i=0; i<4; i++)
	{
		iow(db, DM9013_PIndex, i);
		iow(db, 0x61, 0);
		iow(db, 0x66, 0);
		iow(db, 0x67, 0);
		iow(db, 0x6D, 0);
		iow(db, 0x6F, 0);
	}

}

static void dm9013_switch_config(board_info_t * db)
{

	dm9013_switch_reset(db);

	/* switch control */
#if 0
	dm9013_vlan_setup(db,1); /* 0:port-base, 1:Tag-base */
#endif
#if 0
	dm9013_QoS_setup(db,0); /* 0:port, 1:Tag, 2:TOS */
#endif
#if 0
	/* Bandwidth control
	 dm9013_bw_control(db,port_no, bw_type, bit7-4, bit3-0)
	 bw_type(reg61H.[3]): 0:control with Ingress and Egress separately
		              1:control with Ingress or Egress
	 bit7-4 and bit 3-0:refer to reg66H and reg67H
	 If bw_type=1, bit7-4 must be "0".
	*/
	dm9013_bw_control(db,1,0,0,2);
	dm9013_bw_control(db,0,0,2,0);
#endif
}

void IC_Notice(board_info_t *db)
{
	//[ Un-doc, Sheng-Chung CHANG ]
	iow(db, 0x59, 0xE1);
}

void IC_PHY_Notice(board_info_t *db, int i) //IC_Notice
{
	phy_write(db, 20, 0x0C80, i); /* jfchiu, 'CHECK_STR_V105.x', phy Preamble Saving Control */
								  /* CHECK_STR_V110: After Ken's test, We use 0x0C80 */
	//printk("<DM9003, Port%d> PHY Preamble Sav Cntl, [20,0x0C80] \n", i);
}

/*
	Initilize dm9013 board
*/
static void dm9013_init(struct net_device *dev)
{
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	DM9013_DBUG(0, "dm9013_init()", 0);

	spin_lock_init(&db->lock);
	mutex_init(&db->addr_lock);

	/* switch reset */
	iow(db, DM9013_SCR,DM9013_SCR_ResetSwitch);
	wait_bit_clear(DM9013_SCR,6);

	/* software reset */
	iow(db, DM9013_NCR, NCR_Reset);
	wait_bit_clear(DM9013_NCR,0);

	/* I/O mode */
	db->io_mode = ior(db, DM9013_ISR) >> 6; /* ISR bit7:6 keeps I/O mode */

	/* DM9003 and DM9006 are 2 port mode */
	/* DM9013 is 2 port mode or 3 port mode */
	if ((pe.id_val==DM9003_ID)||(pe.id_val==DM9006_ID))
		db->port_mode= 2;
	else
		db->port_mode= (ior(db, DM9013_MONITOR2)&(1<<6)) ? 2:3; //'CHECK_STR_V105.1'

	/* Set PHY */
	db->op_mode = media_mode;
	dm9013_set_PHY_mode(db);

	IC_Notice(db);

	/* Program operating register */
	iow(db, DM9013_NCR, 0x20); /* reg.01 cleared by write 1 */
	iow(db, DM9013_FCR, 0x20); /* RX Flow Control Enable */

#ifdef CHECKSUM
	printk("<DM9013>Enable checksum offload \n");
	/* TX checksum enable */
	iow(db, DM9013_TCCR, TCCR_UDP_Chksum|TCCR_TCP_Chksum|TCCR_IP_Chksum);
	/* RX checksum enable */
	iow(db, DM9013_RCSR, RCSR_RX_Chksum_enable);
#endif
	/* switch config */
	/* If using EEPROM sets switch functions,
	   don't execute dm9013_switch_config() function */
	if (db->HasEEPROM)
		iow(db, DM9013_EPCR, (1<<5));/* reload EEPROM */
	else
		dm9013_switch_config(db);

	/* Set address filter table */
	dm9013_hash_table(dev);

	/* Activate DM9000A/DM9010 */
	iow(db, DM9013_RXCR, RXCR_RxEnable);
	dm9013_enable_interrupt(db);/* Enable TX/RX interrupt mask */

	/* Init Driver variable */
	db->tx_pkt_cnt = 0;

	netif_carrier_on(dev);

}

inline u8 dm9013_make_mask(int p3, int p2, int p1, int p0)
{
	p3 = p3 ? 1:0;
	p2 = p2 ? 1:0;
	p1 = p1 ? 1:0;
	p0 = p0 ? 1:0;

	return (p3<<3)|(p2<<2)|(p1<<1)|p0;
}
static void dm9013_vlan_group_map(board_info_t *db,u8 reg_group, u8 mapping)
{
	iow(db,reg_group,mapping);
}
/*
 *vlan_type 0:Port-base
 *          1:Tag-base
*/
static void dm9013_vlan_setup(board_info_t *db, int vlan_type)
{

	if (vlan_type)/* Tag-base */
	{
		printk("DM9013:VLAN TAG-BASE\n");
		/* enble tag-base vlan (reg 53h.0=1) */
		iow (db, DM9013_VLANCR, (ior(db,DM9013_VLANCR)&0xff)|0x1);

		/*
		input_port	output_port (1:tag-packet, 0:untag-packet)
		   0             1
		   1		 0
		   1		 1
		Per-port setting
		01: VID(reg 6fH.[3:0])-->output packet tagging(reg 6dH.7=1)-->Group mapping
		10: Group mapping
		11: output packet tagging(reg 6dH.7=1)-->Group mapping
		*/

		/*01: set port VID*/
		/*dm9013_vlan_port_vid(db, port_no, VID)*/
		dm9013_vlan_port_vid(db,0,0); /* set P0 VID=0 */
		dm9013_vlan_port_vid(db,1,2); /* set P1 VID=2*/

		/*01,11:Per-port output packet tagging enable */
		dm9013_vlan_outpkt_tag(db,0);
		dm9013_vlan_outpkt_tag(db,1);
		dm9013_vlan_outpkt_tag(db,2);
		dm9013_vlan_outpkt_tag(db,3);

		/*01,10,11:group mapping */
		dm9013_vlan_group_map(db, VLAN_GROUP1, dm9013_make_mask(1, 1, 1, 1));
		dm9013_vlan_group_map(db, VLAN_GROUP2, dm9013_make_mask(1, 1, 1, 1));
	}
	else
	{
		printk("DM9013:VLAN PORT-BASE\n");
		/* enble tag-base vlan (reg 53h.0=0) */
		iow (db, DM9013_VLANCR, (ior(db,DM9013_VLANCR)&0xfe));

		/* port-base VLAN
			step 1:set Port VID
			Step 2:set Group mapping */

		/* set port VID */
		/* vlan_pvid(board_info_t, port_num, VID)*/
		dm9013_vlan_port_vid(db,0,1);/* P0 VID=1  */
		dm9013_vlan_port_vid(db,1,2);/* P1 VID=2  */
		dm9013_vlan_port_vid(db,3,3);/* P3 VID=3  */

		/* group mapping */
		dm9013_vlan_group_map(db, VLAN_GROUP1, dm9013_make_mask(1, 0, 0, 1));/* Group1 : P3, P0 */
		dm9013_vlan_group_map(db, VLAN_GROUP2, dm9013_make_mask(1, 0, 0, 1));/* Group2 : P3, P0 */
		dm9013_vlan_group_map(db, VLAN_GROUP3, dm9013_make_mask(1, 0, 1, 1));/* Group3 : P3, P1, P0*/
	}
}

static int dm9013_vlan_port_vid(board_info_t *db, int port, u8 VID)
{
	if((port<0)||(port > 3))
	{
		printk("<DM9013>Port number error\n ");
		return 1;
	}

	iow(db, DM9013_PIndex, port);
	iow(db, DM9013_VLAN_TAGL, VID);
	return 0;
}

static int dm9013_vlan_outpkt_tag(board_info_t *db, int port)
{
	if(port < 0 || port > 3)
	{
		printk("<DM9013>Port number error \n");
		return 1;
	}

	iow(db, DM9013_PIndex, port);
	iow(db, DM9013_PPRI, ior(db,DM9013_PPRI)|0x80);
	return 0;
}

static int dm9013_qos_port_pri(board_info_t *db, int port, u8 priority)
{
	u8 tmp;
	if(port < 0 || port > 3)
	{
		printk("<DM9013>Port number error\n ");
		return 1;
	}

	iow(db, DM9013_PIndex, port);
	tmp = (ior(db,DM9013_PPRI)&0xfc)|priority;
	iow(db, DM9013_PPRI,tmp);
	return 0;
}

static int dm9013_qos_tos_enable(board_info_t *db, int port)
{
	if(port < 0 || port > 3)
	{
		printk("<DM9013>Port number error\n ");
		return 1;
	}

	iow(db, DM9013_PIndex, port);
	iow(db, DM9013_PPRI,ior(db,DM9013_PPRI)|0x8);
	return 0;
}

static void dm9013_QoS_setup(board_info_t *db, int QoS_type)
{
	switch (QoS_type)
	{
	case 0: /* port */
		printk("<DM9013>Port Priority\n");
		dm9013_qos_port_pri(db,0,0); /* set P0 queue0  */
		dm9013_qos_port_pri(db,1,1); /* set P1 queue1  */
		dm9013_qos_port_pri(db,2,2); /* set P2 queue2  */
		dm9013_qos_port_pri(db,3,3); /* set P3 queue3  */
		break;
	case 1: /* Tag */
		/* TAG-VLAN format
		   | 0x8100 | vlan-priority(3bit)| CFI(1bit) | VID(12bit)|

		   VLAN-TAG priority (default)
		   Queue 0 : vlan-priority = 0 & 1
		   Queue 1 : vlan-priority = 2 & 3
		   Queue 2 : vlan-priority = 4 & 5
		   Queue 3 : vlan-priority = 6 & 7

		   if you want to modify the priority map, please refer
		   reg0xC0 and reg0xC1
		 */
		printk("<DM9013>VLAN-TAG Priority\n");
		break;
	case 2: /* TOS */
		printk("<DM9013>TOS Priority\n");
		//check most significant 6-bit of TOS
		//iow(db, DM9013_VLANCR, ior(db,DM9013_VLANCR)|0x80);
		dm9013_qos_tos_enable(db,0);
		dm9013_qos_tos_enable(db,1);
		dm9013_qos_tos_enable(db,2);
		dm9013_qos_tos_enable(db,3);

		/*if you want to modify the priority map,
		 	if reg0x53.[7]=1
				please refer reg0xC0~reg0xCF
			else
				please refer reg0xC0 and reg0xC1
		*/
		break;
	}
}
/*
	bw_type refer to reg 61H.3
*/
static int dm9013_bw_control(board_info_t *db, int port, u8 bw_type, u8 bit74, u8 bit30)
{
	u8 tmp;
	printk("<DM9013>Bandwidth control\n");

	if(port < 0 || port > 3)
	{
		printk("<DM9013>Port number error \n");
		return 1;
	}

	iow(db, DM9013_PIndex, port);/* set port index */
	tmp = (ior(db,DM9013_PCTRL)& 0xf7); /* clear bit3 */
	bit74 &= 0x0f;
	bit30 &= 0x0f;
	if (bw_type==0)
	{
		iow(db, DM9013_PCTRL, tmp);
		/* per-port Ingress/Egress control */
		//Reg.66H [7:4]=RX,[3:0]=TX
		iow(db, DM9013_PRATE, (bit74<<4)|bit30);
	}else{ /* bw_type ==1*/
		tmp|= 0x8;
		iow(db, DM9013_PCTRL, tmp);
		tmp = (ior(db, DM9013_PBW)&0xf0)|bit30;
		iow(db, DM9013_PBW, tmp);
	}
	return 0;
}
/*
  Hardware start transmission.
  Send a packet to media from the upper layer.
*/
static int dm9013_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	char * data_ptr;

	//DM9013_DBUG(0, "dm9013_start_xmit()", 0);
	if (db->tx_pkt_cnt >= 2)
	{
		netif_stop_queue(dev);
		db->stop_transmit =1;
		return 1;
	}

	/* packet counting */
	db->tx_pkt_cnt++;

	db->stats.tx_packets++;
	db->stats.tx_bytes+=skb->len;

	// FIFO has fulled already
	if (db->tx_pkt_cnt == 2)
	{
		netif_stop_queue(dev);
		db->stop_transmit =1;
	}

	/* Disable all interrupt */
	dm9013_disable_interrupt(db);

	/* Set TX length to reg. 0xfc & 0xfd */
	iow(db, DM9013_TXPLL, (skb->len & 0xff));
	iow(db, DM9013_TXPLH, (skb->len >> 8) & 0xff);

	/* Move data to TX SRAM */
	data_ptr = (char *)skb->data;

	outb(DM9013_MWCMD, db->io_addr); /* Write data into SRAM trigger */
	db->MoveData(db, data_ptr, skb->len, 1);

	/* Issue TX polling command */
	iow(db, DM9013_TCR, TCR_TX_Request);

	/* Saved the time stamp */
	dev->trans_start = jiffies;
	db->cont_rx_pkt_cnt =0;

	/* Free this SKB */
	dev_kfree_skb(skb);

	/* Re-enable interrupt */
	dm9013_enable_interrupt(db);

	return 0;
}

/*
  Stop the interface.
  The interface is stopped when it is brought.
*/
static int dm9013_stop(struct net_device *dev)
{
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	int i;

	//DM9013_DBUG(0, "dm9013_stop", 0);

	if (netif_msg_ifdown(db))
		printk(KERN_INFO"shutting down %s\n", dev->name);


	
	/* deleted timer */
	del_timer(&db->timer);

	netif_stop_queue(dev);

	/* free interrupt */
	free_irq(dev->irq, dev);

	/* RESET devie */
	for (i=0; i<db->port_mode; i++) //'CHECK_STR_V105.2 (Orginal: <=)'
		phy_write(db, 0x00, 0x8000,i);	/* PHY RESET */

	dm9013_switch_reset(db);
	iow(db, DM9013_NCR, 1); /* software reset  */
	iow(db, DM9013_SCR, 0x60); /* reset switch core and analog PHY core  */
	iow(db, DM9013_RXCR, 0x00);	/* Disable RX */
	iow(db, DM9013_IMR, 0);/* Disable all interrupt */

	/* Dump Statistic counter */
#if FALSE
	printk("\nRX FIFO OVERFLOW %lx\n", db->stats.rx_fifo_errors);
	printk("RX CRC %lx\n", db->stats.rx_crc_errors);
	printk("RX LEN Err %lx\n", db->stats.rx_length_errors);
	printk("RESET %x\n", db->reset_counter);
	printk("RESET: TX Timeout %x\n", db->reset_tx_timeout);
	printk("g_TX_nsr %x\n", g_TX_nsr);
#endif
	return 0;
}

static void dm9013_tx_done(unsigned long unused)
{
	struct net_device *dev = dm9013_dev;
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	int time_out;
	u8  nsr, tcr;

	//DM9013_DBUG(0, "dm9013_tx_done()", 0);

	nsr = ior(db, DM9013_NSR);
	iow(db, DM9013_NSR, nsr); /* clear TX packet complete status */
	if(nsr & NSR_TX1END) db->tx_pkt_cnt--;
	if(nsr & NSR_TX2END) db->tx_pkt_cnt--;

	if (db->tx_pkt_cnt < 0)
	{

		printk(KERN_DEBUG"[dm9013_tx_done] tx_pkt_cnt ERROR!!\n");
		db->tx_pkt_cnt =0;

		// 2008.06.11 - CHECK_STR_V109
		time_out= 0;
		do
		{
			tcr = ior(db, DM9013_TCR);
			udelay(200);
			time_out++;
			if (time_out>5000) // 1 sec
			{
				dm9013_reset(dm9013_dev);
				//dm9013_init(dm9013_dev);
				break;
			}
		} while (tcr&0x01);

	}
	if(db->stop_transmit )
	{
		netif_wake_queue(dev);
		db->stop_transmit=0;
	}
	return;
}

/*
  DM9000 insterrupt handler
  receive the packet to upper layer, free the transmitted packet
*/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,19) //V1.21
	static irqreturn_t dm9013_interrupt(int irq, void *dev_id)
	#else
	static irqreturn_t dm9013_interrupt(int irq, void *dev_id, struct pt_regs *regs)
	#endif
#else
static void dm9013_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
	struct net_device *dev = dev_id;
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	int int_status;
	u8 reg_save;
	unsigned long flags;

	//DM9013_DBUG(0, "dm9013_interrupt()", 0);
	/* A real interrupt coming */
	//spin_lock(&db->lock);
	spin_lock_irqsave(&db->lock, flags);


	/* Save previous register address */
	reg_save = inb(db->io_addr);

	/* disable IMR */
	dm9013_disable_interrupt(db);

	/* Got DM9000A/DM9010 interrupt status */
	int_status = ior(db, DM9013_ISR);	/* Got ISR */
	iow(db, DM9013_ISR, int_status);		/* Clear ISR status */

	/* Received the coming packet */
	if (int_status & ISR_RX_coming)
#ifdef NAPI
	{
		if (netif_rx_schedule_prep(dev)) {
			iow(db, DM9013_IMR,DM9013_NO_RX_INTR);
      			__netif_rx_schedule(dev);
                 }
	}
#else
	dm9013_packet_receive(dev);
#endif
	if (db->cont_rx_pkt_cnt>=CONT_RX_PKT_CNT)
	{
		printk("enable interrupt without RX\n");
		iow(db, DM9013_IMR, 0xa2);
	}
	else
	{
		dm9013_enable_interrupt(db);
	}

	/* Trnasmit Interrupt check */
	if (int_status & ISR_TX_complete)
		dm9013_tx_done(0);
	/* Restore previous register address */
	outb(reg_save, db->io_addr);

	//spin_unlock(&db->lock);
	spin_unlock_irqrestore(&db->lock, flags);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
	return IRQ_HANDLED;
#endif
}

#if !defined(CHECKSUM)
#define check_rx_ready(a)       ((a) == 0x01)
#else
inline u8 check_rx_ready(u8 rxbyte)
{
        if (!(rxbyte & 0x01))
                return 0;
        return ((rxbyte >> 4) | 0x01);
}
#endif

#ifdef NAPI
static int dm9013_poll (struct net_device *dev, int *budget)
{
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	struct sk_buff *skb;
        u8 rxbyte, val;
        u16 MDRAH, MDRAL;
        u32 tmpdata;
	unsigned rx_work = dev->quota;
	unsigned rx;
	rx_t rx_desc;
        u8 * ptr = (u8 *)&rx_desc;
        u8 * rdptr;

rx_status_loop:
	rx = 0;
	iow(db, DM9013_ISR, ISR_RX_coming); //clean this bit

	while(1){
		/*store the value of Memory Data Read address register*/
		MDRAH=ior(db, DM9013_MDRAH);
		MDRAL=ior(db, DM9013_MDRAL);

		rxbyte =ior(db, DM9013_MRCMDX);
		rxbyte =ior(db, DM9013_ISR);     /* Dummy read */
		rxbyte =ior(db, DM9013_MRCMDX);  /* read the byte of packet ready */

		/* packet ready to receive check */
		if(!(val = check_rx_ready(rxbyte))) break;

		/* A packet ready now  & Get status/length */
		outb(DM9013_MRCMD, db->io_addr);

		/* Read packet status & length */
		db->MoveData(db,ptr,4,0);

		 /* Packet status check */
		if (rx_desc.desc.status & 0x03) //0xbf)
		{
			db->stats.rx_errors++;
			if (rx_desc.desc.status & RX_FIFO_over)
				db->stats.rx_fifo_errors++;
			if (rx_desc.desc.status & RX_CRCErr)
				db->stats.rx_crc_errors++;

			//drop this packet
 			db->MoveData(db, NULL, rx_desc.desc.length, 0);
			db->stats.rx_dropped++;
			goto rx_next;


		}
		skb = dev_alloc_skb(rx_desc.desc.length+4);
                if (skb == NULL )
                {
                        printk(KERN_INFO "%s: Memory squeeze.\n", dev->name);
                        /*re-load the value into Memory data read address register*/
                        iow(db,DM9013_MDRAH,MDRAH);
                        iow(db,DM9013_MDRAL,MDRAL);
                }else{
                        /* Move data from DM9000 */
                        skb->dev = dev;
                        skb_reserve(skb, 2);
                        rdptr = (u8*)skb_put(skb, rx_desc.desc.length - 4);

                        db->MoveData(db, rdptr, rx_desc.desc.length, 0);
                        /* Pass to upper layer */
                        skb->protocol = eth_type_trans(skb,dev);
#if defined(CHECKSUM)
                        if (!(rx_desc.desc.rxbyte & 0xe0))
                                skb->ip_summed = CHECKSUM_UNNECESSARY;
#endif
                        netif_rx(skb);
                        dev->last_rx=jiffies;
                        db->stats.rx_packets++;
                        db->stats.rx_bytes += rx_desc.desc.length;

			rx++;
                }

rx_next:
		if (!rx_work--)
			break;
	}//end of while()

	dev->quota -= rx;
	*budget -= rx;

	/* if we did not reach work limit, then we're done with
	 * this round of polling
	 */
	if (rx_work)
	{
		if (ior(db, DM9013_ISR) & ISR_RX_coming)
			goto rx_status_loop;

		local_irq_disable();
		dm9013_enable_interrupt(db);
		__netif_rx_complete(dev);
		local_irq_enable();

		return 0;/* done */
	}

	return 1;/* not done */
}
#endif

/*
  Get statistics from driver.
*/
static struct net_device_stats * dm9013_get_stats(struct net_device *dev)
{
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	DM9013_DBUG(0, "dm9013_get_stats", 0);
	return &db->stats;
}

/*
  Process the upper socket ioctl command
*/
static int dm9013_do_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
	board_info_t *dm = to_dm9013_board(dev);

	DM9013_DBUG(0, "dm9013_do_ioctl", cmd);
	if (!netif_running(dev))
		return -EINVAL;

	return generic_mii_ioctl(&dm->mii, if_mii(req), cmd, NULL);
}

/* Our watchdog timed out. Called by the networking layer */
static void
dm9013_timeout(struct net_device *dev)
{
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;


	DM9013_DBUG(0, "dm9013_TX_timeout()", 0);
	//printk("TX time-out -- dm9013_timeout().\n");
	db->reset_tx_timeout++;
	db->stats.tx_errors++;
#if FALSE
	printk("TX packet count = %d\n", db->tx_pkt_cnt);
	printk("TX timeout = %d\n", db->reset_tx_timeout);
	printk("22H=0x%02x  23H=0x%02x\n",ior(db,0x22),ior(db,0x23));
	printk("faH=0x%02x  fbH=0x%02x\n",ior(db,0xfa),ior(db,0xfb));
#endif
	dm9013_reset(dev);

}

static void dm9013_reset(struct net_device * dev)
{
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	u8 reg_save;

	/* Save previous register address */
	reg_save = inb(db->io_addr);

	netif_stop_queue(dev);
	db->reset_counter++;
	dm9013_init(dev);

	netif_wake_queue(dev);

	/* Restore previous register address */
	outb(reg_save, db->io_addr);

}

/*
  'CHECK_STR_V106.1'
*/
void port_timer_reset(board_info_t *db)
{
	int i;
	//char connect;
	//connect= 0;
	for (i=0; i<db->port_mode; i++) // Job&jfchiu, (2) v1.04
	{
		if (phy_read(db,1,i)&0x4)
		{
			//		connect++; // "Port i" CNNT
			pe.n4HangCounts_port[i]= 0;
		}
		else
		{
			pe.n4HangCounts_port[i]++;

			if ((pe.n4HangCounts_port[i]%6)==5) {
				if (i == 0){ //Stone add ....
					if (pe.n4HangCounts_port[0] < 20) {
						//DM9013_DBUG_STR(0, "DISCNNT, Rst, "); //printk("DISCNNT, Rst, ");
						port_set_PHY_mode(db, i);
					}
				}
				else { 		// i =1,2....
					//DM9013_DBUG_STR(0, "DISCNNT, Rst, "); //printk("DISCNNT, Rst, ");
					port_set_PHY_mode(db, i);
				}
			}
			//else printk("P%d>%d,", i, pe.n4HangCounts_port[i]); //Debug purpose, finish!
		}
	}
}
/*
  A periodic timer routine
*/
static void dm9013_timer(unsigned long data)
{
	struct net_device * dev = (struct net_device *)data;
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	//DM9013_DBUG(0, "dm9013_timer()", 0);

	if (db->cont_rx_pkt_cnt>=CONT_RX_PKT_CNT)
	{
		db->cont_rx_pkt_cnt=0;
		dm9013_enable_interrupt(db);
	}

	/* Set timer again */
	db->timer.expires = DM9013_TIMER_WUT;
	add_timer(&db->timer);

	port_timer_reset(db);
	//'CHECK_STR_V106.1'
	//'CHECK_STR_V105.x'

	return;
}


/*
 * Received a packet and pass to upper layer
 */
static void dm9013_packet_receive(struct net_device *dev)
{
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

	struct sk_buff *skb;
	u8 rxbyte, val;
	u16 MDRAH, MDRAL;

	rx_t rx;

	u8 * ptr = (u8*)&rx;
	u8* rdptr;

	//DM9013_DBUG(0, "dm9013_packet_receive()", 0);

	db->cont_rx_pkt_cnt=0;
	do {
		/*store the value of Memory Data Read address register*/
		MDRAH=ior(db, DM9013_MDRAH);
		MDRAL=ior(db, DM9013_MDRAL);

		rxbyte =ior(db, DM9013_MRCMDX);/* Dummy read */
   		rxbyte =inb(db->io_data);  /* read the byte of packet ready */

		/* packet ready to receive check */
		if(!(val = check_rx_ready(rxbyte))) break;

		/* A packet ready now  & Get status/length */
		outb(DM9013_MRCMD, db->io_addr);

		/* Read packet status & length */
		db->MoveData(db,ptr,4,0);

		/* Packet status check */
		if (rx.desc.status & 0x03) //0xbf)
    		{
			//printk("DM9013:RX ERROR\n");
       		db->stats.rx_errors++;
        		if (rx.desc.status & RX_FIFO_over)
        			db->stats.rx_fifo_errors++;
        		if (rx.desc.status & RX_CRCErr)
            		db->stats.rx_crc_errors++;

        		/* drop this packet */
        		db->MoveData(db, NULL, rx.desc.length, 0);
        		db->stats.rx_dropped++;
        		continue;
     	}

		skb = dev_alloc_skb(rx.desc.length+4);
		if (skb == NULL )
		{
			printk(KERN_INFO "%s: Memory squeeze.\n", dev->name);
			/*re-load the value into Memory data read address register*/
			iow(db,DM9013_MDRAH,MDRAH);
			iow(db,DM9013_MDRAL,MDRAL);
			return;
		}
		else
		{
			/* Move data from DM9000 */
			skb->dev = dev;
			skb_reserve(skb, 2);
			rdptr = (u8*)skb_put(skb, rx.desc.length - 4);

			db->MoveData(db, rdptr, rx.desc.length, 0);
			/* Pass to upper layer */
			skb->protocol = eth_type_trans(skb,dev);
#ifdef CHECKSUM
			skb->ip_summed = CHECKSUM_UNNECESSARY; // V1.03 (3)
			if ((rxbyte&0xE0)||(rxbyte==0x01))
				skb->ip_summed = CHECKSUM_NECESSARY; /* receive packet no checksum fail */
#endif
			netif_rx(skb);
			dev->last_rx=jiffies;
			db->stats.rx_packets++;
			db->stats.rx_bytes += rx.desc.length;
			db->cont_rx_pkt_cnt++;
			if (db->cont_rx_pkt_cnt>=CONT_RX_PKT_CNT)
			{
				printk("RX out \n");
				dm9013_tx_done(0);
				break;
			}
		}
	} while((rxbyte & 0x01) == DM9013_PKT_RDY);
	//DM9013_DBUG(0, "[END]dm9013_packet_receive()", 0);

}

/*
 * Read a word data from SROM
 */
static u16 dm9013_read_eeprom_word(board_info_t *db, int offset)
{
	iow(db, DM9013_EPAR, offset);
	iow(db, DM9013_EPCR, EPCR_Read);
	udelay(200);
	iow(db, DM9013_EPCR, 0x0);
	return (ior(db, DM9013_EPDRL) + (ior(db, DM9013_EPDRH) << 8) );
}

/*
  Set DM9000A/DM9010 multicast address
*/
static void dm9013_hash_table(struct net_device *dev)
{
		DM9SW_BOARD_DB;
//	board_info_t *db = (board_info_t *)dev->priv;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)  //V1.21
	struct netdev_hw_addr *ha;
#else
	struct dev_mc_list *mcptr = dev->mc_list;
	int mc_cnt = dev->mc_count;
#endif
	u32 hash_val;
	u16 i, oft, hash_table[4];

	DM9013_DBUG(0, "dm9013_hash_table()", 0);

	/* Set Node address */
	for (i = 0, oft = 0x10; i < 6; i++, oft++)
		iow(db, oft, dev->dev_addr[i]);

	if (dev->flags & IFF_PROMISC)
	{
		printk(KERN_INFO "%s:Enable promisc mode\n",dev->name);
		iow(db, DM9013_RXCR, (ior(db, DM9013_RXCR)|(1<<1)));
		return;
	}else{
		printk(KERN_INFO "%s:Disable promisc mode\n",dev->name);
		iow(db, DM9013_RXCR, (ior(db, DM9013_RXCR)&(~(1<<1))));
	}

	if (dev->flags & IFF_ALLMULTI)
	{
		printk(KERN_INFO "%s:Pass all multicast\n",dev->name);
		iow(db, DM9013_RXCR, (ior(db, DM9013_RXCR)|(1<<3)));
	}else{
		printk(KERN_INFO "%s:Disable Pass all multicast\n",dev->name);
		iow(db, DM9013_RXCR, (ior(db, DM9013_RXCR)&(~(1<<3)))); // 2010/05/14
	}

	/* Clear Hash Table */
	for (i = 0; i < 4; i++)
		hash_table[i] = 0x0;

	/* broadcast address */
	hash_table[3] = 0x8000;

	/* the multicast address in Hash Table : 64 bits */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)  //V1.21

	netdev_for_each_mc_addr(ha,dev)
        {
		hash_val = cal_CRC((char *)ha->addr, 6, 0) & 0x3f;
		hash_table[hash_val / 16] |= (u16) 1 << (hash_val % 16);
	}

#else
	for (i = 0; i < mc_cnt; i++, mcptr = mcptr->next)
	{
		hash_val = cal_CRC((char *)mcptr->dmi_addr, 6, 0) & 0x3f;
		hash_table[hash_val / 16] |= (u16) 1 << (hash_val % 16);
	}
#endif

	/* Write the hash table to MAC MD table */
	for (i = 0, oft = 0x16; i < 4; i++)
	{
		iow(db, oft++, hash_table[i] & 0xff);
		iow(db, oft++, (hash_table[i] >> 8) & 0xff);
	}

}
/*
  Calculate the CRC valude of the Rx packet
  flag = 1 : return the reverse CRC (for the received packet CRC)
         0 : return the normal CRC (for Hash Table index)
*/
static unsigned long cal_CRC(unsigned char * Data, unsigned int Len, u8 flag)
{

	u32 crc = ether_crc_le(Len, Data);

	if (flag)
		return ~crc;

	return crc;

}

/*
   Read a byte from I/O port
*/
static u8
ior( board_info_t * db, int reg )
{
	//static int cnt = 0;
	u8 value;
	writeb(reg, db->io_addr);
	value = readb(db->io_data);
	//printk("9013 %5d : [%02x]  = %02x\n", cnt++, (unsigned int)reg, (unsigned int)value);
	return value;
}


/*
 *   Write a byte to I/O port
 */

static void
iow( board_info_t * db, int reg, u8 value )
{
	//static int cnt = 0;
	//printk("9013 %5d : [%02x] <- %02x\n", cnt++, (unsigned int)reg, (unsigned int)value);
	writeb(reg, db->io_addr);
	writeb(value, db->io_data);
}


/*
   Read a word from phyxcer
*/
static u16 phy_read(board_info_t *db, int reg, int port)
{
	reg = (port<<6) | reg;
	/* Fill the phyxcer register into REG_0C */
	iow(db, 0xc, reg);

	iow(db, 0xb, 0x8); 	/* Clear phyxcer read command */
	iow(db, 0xb, 0xc); 	/* Issue phyxcer read command */
	iow(db, 0xb, 0x8); 	/* Clear phyxcer read command */
	do {
		if (!(ior(db,0xb) & 0x1))
			break;
	}while(1);
	/* The read data keeps on REG_0D & REG_0E */
	return ( ior(db, DM9013_EPDRH) << 8 ) | ior(db, DM9013_EPDRL);
}

/*
   Write a word to phyxcer
*/
static void phy_write(board_info_t *db, int reg, u16 value, int port)
{

	reg = (port<<6) | reg;
	/* Fill the phyxcer register into REG_0C */
	iow(db, 0xc, reg);

	/* Fill the written data into REG_0D & REG_0E */
	iow(db, 0xd, (value & 0xff));
	iow(db, 0xe, ( (value >> 8) & 0xff));

	iow(db, 0xb, 0x8);	/* Clear phyxcer write command */
	iow(db, 0xb, 0xa);	/* Issue phyxcer write command */
	iow(db, 0xb, 0x8);	/* Clear phyxcer write command */
	do {
		if (!(ior(db,0xb) & 0x1))
			break;
	}while(1);
}

/*
	selec 0:input(RX)	1:output(TX)
	if selec=0, move data from FIFO to data_ptr
	if selec=1, move data from data_ptr to FIFO
*/
static void dm9013_move8bit(board_info_t *db,unsigned char *data_ptr, int leng, int selec)
{
	int i;
	if (selec)
		for (i=0; i<leng; i++)
			outb((data_ptr[i] & 0xff), db->io_data);
	else
		for (i=0; i<leng; i++)
			data_ptr[i] = inb(db->io_data);
}

static void dm9013_move16bit(board_info_t *db,unsigned char *data_ptr, int leng, int selec)
{
	int i, tmpleng;
	tmpleng = (leng + 1) >> 1;
	if (selec)
	{
		for (i=0; i<tmpleng; i++)
			outw(((u16 *)data_ptr)[i], db->io_data);
	}
	else
		for (i=0; i<tmpleng; i++)
			((u16 *)data_ptr)[i] = inw(db->io_data);
}

static void dm9013_move32bit(board_info_t *db,unsigned char *data_ptr, int leng, int selec)
{
	int i, tmpleng;
	tmpleng = (leng + 3) >> 2;
	if (selec)
		for (i=0; i<tmpleng; i++)
			outl(((u32 *)data_ptr)[i], db->io_data);
	else
		for (i=0; i<tmpleng; i++)
			((u32 *)data_ptr)[i]=inl(db->io_data);
}

//// #ifdef MODULE

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Davicom DM9013 ISA/uP Fast Ethernet Driver");
module_param(irq, int, 0);  // 'CHECK_STR_V105.x'
module_param(iobase, int, 0); // 'CHECK_STR_V105.x'
MODULE_PARM_DESC(irq,"EtherLink IRQ number");
MODULE_PARM_DESC(iobase, "EtherLink I/O base address");

/* Description:
   when user used insmod to add module, system invoked init_module()
   to initilize and register.
*/
//V1.21 int init_module(void)
static int __init dm9sw_init_module(void)
{
	media_mode = DM9013_AUTO;
	dm9013_dev = dm9013_probe1();
	if(IS_ERR(dm9013_dev))
		return PTR_ERR(dm9013_dev);
	return 0;
}
/* Description:
   when user used rmmod to delete module, system invoked clean_module()
   to  un-register DEVICE.
*/
//V1.21 void cleanup_module(void)
static void __exit dm9sw_cleanup_module(void)
{
	struct net_device *dev = dm9013_dev;
	DM9013_DBUG(0, "clean_module()", 0);

	unregister_netdev(dm9013_dev);
	release_region(dev->base_addr, 2);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	kfree(dev);
#else
	free_netdev(dev);
#endif

	DM9013_DBUG(0, "clean_module() exit", 0);
}

module_init(dm9sw_init_module);
module_exit(dm9sw_cleanup_module);

//// #endif
