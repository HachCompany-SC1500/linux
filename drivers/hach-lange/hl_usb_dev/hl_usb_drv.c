/***************************************************************************
                          usb_drv.c  -  description
                             -------------------
    begin                : Don Sep 23 2004
    copyright            : (C) 2004 by Thomas Siegmund
    email                : tsiegmund@hach-lange.de
***************************************************************************/

#ifndef __KERNEL__
#  define __KERNEL__
#endif
#ifndef MODULE
#  define MODULE
#endif

//#define WITH_DEVICE_STATE

//#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
//#include <linux/kdev_t.h>
#include <linux/fs.h>
//#include <linux/slab.h>
#include <linux/version.h>
#include <linux/smp_lock.h>
#ifdef CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
#endif
#include <linux/proc_fs.h>

/* If you want debugging uncomment: */
/* #define DEBUG */

#include <linux/init.h>
#include <asm/uaccess.h>
// #include <asm/sc1000.h>  //  /opt/Embedix/usr/local/sh3-linux/include
#include <asm/io.h>
#include <asm/ioctl.h>
//#include <linux/ioport.h>
#include <asm/irq.h>
#include <linux/usb.h>    // /opt/Embedix/home/project/gamma_tsg/build/rpmdir/BUILD/linux/include/linux
//#include <linux/lvm.h>
#include <linux/time.h>

/**** own headers ************/
#include "hl_usb_reg.h"
#include "hl_usb_drv.h"
//#include "usb_version.h"
#include "hl_usb_desc.h"
#include "hl_usb_ioctl.h"
#include "hl_usb_viper.h"
#include "hl_usb_state.h"
/////////////////////////////////////////
#ifdef  DEBUG
//extern int usb_debug;
//extern int package;
#endif      

/***************************************************
 **                    Prototypes
 ****************************************************/
static inline BYTE USB_Receive_Data(BYTE  *rdata,EndPoint ep_num);
static inline BYTE USB_Transmit_Data(BYTE *tdata, int data_size, EndPoint ep_num);
static inline void writefifo(BYTE* txdata, BYTE ep_num, int noBYTEs);
static inline int  readfifo(BYTE* rxdata, BYTE ep_num, int bufsiz);
static inline void tx_enable_epx(int ep_num);
static inline void send_ZLP_2_finish_EPx(BYTE ep_num);
static inline void usb_device_reset(void);
static inline void SetStallEpx(int ep_no);
static int  BufferFull(void);
static void init_Statistic_Device_State(void);
//static inline void frame_event_handler(void);
static void clear_control_buffer(control_buffer*  ctrl_buf);
static void endpoint_status_init(void);
static void send_control_data(BYTE* data, int size);

typedef void (*USB_req_handler)(USB_request*);

/*=====================================================*
 *       Prototypes    standard requests
 *=====================================================*/
void usb_dev_get_status(USB_request*          req);
void usb_dev_clear_feature(USB_request*       req);
void usb_dev_set_feature(USB_request*         req);
void usb_dev_set_address(USB_request*         req);
void usb_std_dev_get_descriptor(USB_request*  req);
void usb_std_dev_set_descriptor(USB_request*  req);
void usb_dev_get_config(USB_request*   		  req);
void usb_dev_set_config(USB_request*    	  req);
void usb_dev_get_interface(USB_request* 	  req);
void usb_dev_set_interface(USB_request* 	  req);
void usb_dev_sync_frame(USB_request*    	  req);

/*********************************************************
 * requests' sequence is according to USB 1.1 spec values
 *********************************************************/
static const USB_req_handler usb_std_device_req[] =
  {
    usb_dev_get_status,
    usb_dev_clear_feature,
    NULL,
    usb_dev_set_feature,
    NULL,
    usb_dev_set_address,
    usb_std_dev_get_descriptor,
    usb_std_dev_set_descriptor,
    usb_dev_get_config,
    usb_dev_set_config,
    usb_dev_get_interface,
    usb_dev_set_interface,
    usb_dev_sync_frame,
    //    USB_req_reserved,
    //    USB_req_reserved,
    //    USB_req_reserved
  };

/***************************************
 **    Prototypes  vendor requests
 ****************************************/
void Cmd_Data_Parser(USB_request* req);
//void Data_Parser(USB_request*    req);

/****************************************
 *    Vendor device request handlers
 ****************************************/
static const USB_req_handler usb_vendor_device_req[] =
  {
    NULL,
    Cmd_Data_Parser
    //Data_Parser
  };

#pragma pack(push, old_pack_val, 1)
/**********************************
 ** Device USB device descriptor
 ***********************************/
static const USB_device_desc usb_device_desc =
  {
    sizeof(USB_device_desc),
    DEVICE_DESCRIPTOR,
    USB_VERSION,
    CLASS_NOT_DEFINED,      /* CLASS_VENDOR, vendor specific */
    0,                      /* Device Sub-Class */
    0,                      /* Device Protocol */
    EP0_FIFO_SIZE,          /* Max Packet Size for EndPoint Zero*/
    VENDOR_ID,
    PRODUCT_ID,
    BCDDEVICE,             /* device release number: 01.08 BCDDEVICE */
    STR_MANUFACTURER,
    STR_PRODUCT,
    STR_VERSION,                      /* Device's serial number */
    1                       /* Num of configurations */
  };
/*-------------------------------------
 * Device Long Configuration descriptor
 *------------------------------------*/
static const USB_long_config_desc usb_dev_long_config_desc =
  {
    //Device Configuration descriptor
    {
      sizeof(USB_config_desc),    /* CONFIG_DESC_LENGTH */
      CONFIG_DESCRIPTOR,
      sizeof(USB_long_config_desc), /* TOTAL CONFIG_DESC_LENGTH */
      1,                          /* one interface supported */
      1,                          /* Configuration number */
      0,                          /* no descriptor string STR_PRODUCT */
      SELF_POWERED,
      0
    },
    /*
      {
      sizeof(USB_interface_desc),
      INTERFACE_DESCRIPTOR,
      0,                  // The only interface concurrently supported by this configuration 
      0,                  // Alternate Setting
      NUM_OF_ENDPOINTS_FOR_ALT_0, // Num of endpoints of this interface excluding endpoint zero 
      CLASS_APP,  	// 
      SUB_CLASS_TMC,	//                   Sub class 
      0x7f,   //reserved for future ;-))     	//0x01, // Interface Protocol  Contorl/Bulk/Interrupt Protocol but without Command implementation 
      0
      },
    */
    {
      sizeof(USB_interface_desc),
      INTERFACE_DESCRIPTOR,
      0,                  // The only interface concurrently supported by this configuration 
      0,                  // Alternate Setting 
      NUM_OF_ENDPOINTS_FOR_ALT_0, // Num of endpoints of this interface excluding endpoint zero 
      CLASS_MASS_STORAGE,        // Mass-Storage device class 
      CLASS_VENDOR,      //0x06,                    // Sub class 
      0x01,              // Interface Protocol  Contorl/Bulk/Interrupt Protocol but without Command implementation 
      0
    },
	
    {
      //The IN endpoint 1 is used for bulk data transfer
      {
        sizeof(USB_endpoint_desc),
        ENDPOINT_DESCRIPTOR,
        {
          EPT1, //0x81
          0,
          IN
        },
        BULK_EP,
        TX_BULK_EP_FIFO_SIZE, /* Max Packet Size */
        0
      },
      
      //The OUT endpoint 2 is used for bulk data transfer
      {
        sizeof(USB_endpoint_desc),
        ENDPOINT_DESCRIPTOR,
        {
          EPT2, //0x02
          0,
          OUT
        },
        BULK_EP,
        RX_BULK_EP_FIFO_SIZE, /* Max Packet Size */
        0
      },
      //The IN endpoint 3 is used for interrupt transfer
      {
        sizeof(USB_endpoint_desc),
        ENDPOINT_DESCRIPTOR,
        {
          EPT3, //0x83
          0,
          IN
        },
        INTERRUPT_EP,
        TX_INTR_EP_FIFO_SIZE_COMM,    /* Max Packet Size 8 byte*/
        0xFF                          /* Interrupt Interval, 255 ms */
      },
      //The IN endpoint 5 is used for interrupt transfer for GAMMA Measurements
      {
        sizeof(USB_endpoint_desc),
        ENDPOINT_DESCRIPTOR,
        {
          EPT5, //0x85
          0,
          IN
        },
        INTERRUPT_EP,
        TX_INTR_EP_FIFO_SIZE_MEAS,    /* Max Packet Size 64 byte*/
        0x01                          /* Interrupt Interval, 1 ms */
      }   
    }  
  };

//String descriptors
const struct {
  BYTE  bLength;
  BYTE  bDescriptorType;
  WORD  bstring;
} langid_str_desc = {sizeof(langid_str_desc), STRING_DESCRIPTOR, ENGLISH_US};

const struct {
  BYTE  bLength;
  BYTE  bDescriptorType;
  char  bstring[sizeof(MANUFACTURE_STR)];
} manufacture_str_desc = {sizeof(manufacture_str_desc), STRING_DESCRIPTOR, MANUFACTURE_STR};

const struct {
  BYTE  bLength;
  BYTE  bDescriptorType;
  char  bstring[sizeof(PRODUCT_STR)];
} product_str_desc = {sizeof(product_str_desc), STRING_DESCRIPTOR, PRODUCT_STR};

const struct {
  BYTE  bLength;
  BYTE  bDescriptorType;
  char  bstring[sizeof(VERSION_STRW)];
} version_str_desc = {sizeof(version_str_desc), STRING_DESCRIPTOR, VERSION_STRW};
#pragma pack(pop, old_pack_val)

/*****************************************************************
 **if we get alternate settings then we to have revisit the place
 **and make 2 array instead of 1
 *****************************************************************/
//List of endpoint descriptors   -> usb_dev_endpoints[NUM_OF_ALTERNATESETTING][NUM_OF_ENDPOINTS]
//there isn't an other alternate stetting
static const USB_endpoint_desc  *usb_dev_endpoints[2][NUM_OF_ENDPOINTS] =
  {
    {
      NULL,                                                      /* Endpoint 0 */
      &usb_dev_long_config_desc.usb_dev_endpoint_alt_0_desc[0],  /* Endpoint 1  Bulk IN     */
      &usb_dev_long_config_desc.usb_dev_endpoint_alt_0_desc[1],  /* Endpoint 2  Bulk OUT    */
      &usb_dev_long_config_desc.usb_dev_endpoint_alt_0_desc[2],  /* Endpoint 3  Interrupt IN*/
      NULL,
      &usb_dev_long_config_desc.usb_dev_endpoint_alt_0_desc[3],  /* Endpoint 5  Interrupt IN*/
      NULL
    },
    {
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL
    }
  };

/*********************************************************
 * List of all string descriptors, Be sure that the order
 * of the list is the same as String_index enum
 **********************************************************/
const USB_string_desc *string_descs[] = {
  (USB_string_desc*)&langid_str_desc,
  (USB_string_desc*)&manufacture_str_desc,
  (USB_string_desc*)&product_str_desc,
  (USB_string_desc*)&version_str_desc
};

#ifdef _SELF_MODULE_UNLOAD_
extern int selfCleanUp(void);
static DWORD ResetEvCnt   = 0; 
#endif

#if 0
/*****************************************/
const int usbn9603_tx_endpoint_addr[] =
  {
    TXD0, /* Endpoint 0 */
    TXD1, /* Endpoint 1 */
    0xFF,
    TXD2, /* Endpoint 3 */
    0xFF,
    TXD3, /* Endpoint 5 */
    0xFF
  };

const int usbn9603_rx_endpoint_addr[] =
  {
    RXD0, /* Endpoint 0 */
    0xFF,
    RXD1, /* Endpoint 2 */
    0xFF,

    RXD2, /* Endpoint 4 */
    0xFF,
    RXD3 /* Endpoint 6 */
  };
#endif // 0

static const int fifo_sizes[] =
  {
    EP0_FIFO_SIZE,             /* control tx0, rx0   EPT0 */
    TX_BULK_EP_FIFO_SIZE,      /* bulk tx            EPT1 */
    RX_BULK_EP_FIFO_SIZE,      /* bulk rx            EPT2 */
    TX_INTR_EP_FIFO_SIZE_COMM, /* int  tx   8 byte    EPT3 */
    NO_ENDPOINT_ZERO_SIZE,
    TX_INTR_EP_FIFO_SIZE_MEAS, /* int  tx  64 byte    EPT5 */
    NO_ENDPOINT_ZERO_SIZE
  };


/******* USB device status ***********/
static DEVICE_status   device_status;
static Device_buffers  device_buffers;

/******* Endpoint states ************/
static endpoint_status tx0_ep_st;
static endpoint_status tx1_ep_st;
static endpoint_status rx1_ep_st;
static endpoint_status tx2_ep_st;
static endpoint_status tx3_ep_st;

endpoint_status *ep_stat[] =
  {
    &tx0_ep_st,
    &rx1_ep_st,
    &tx1_ep_st,
    &tx2_ep_st,
    NULL,
    &tx3_ep_st,
    NULL
  };

static control_buffer ctrl_send_buf;
static control_buffer ctrl_receive_buf;

static BYTE reqfifo[EP0_FIFO_SIZE];

void dbg_time(void)
{
  struct timeval tv;
  do_gettimeofday(&tv);
  PDBG("CLK: %d,%06d\n",(int)tv.tv_sec,(int)tv.tv_usec);
}
/**********************************************************/
static void clear_control_buffer(control_buffer*  ctrl_buf)
{
  PDBG("%s...\n",__FUNCTION__);
  ctrl_buf->data      = NULL;
  ctrl_buf->byte_cnts = 0;
}
/*********************************************************/
static void endpoint_status_init(void)
{
  int i;
  // PDBG("%s...\n",__FUNCTION__);

  for(i=0; i<NUM_OF_ENDPOINTS; i++) 
  {
    if(ep_stat[i] != NULL)  
    {
      ep_stat[i]->fifo_state = EMPTY;
      
      if(i==0) //control endpoint
        ep_stat[i]->toggle_bit = 0x01;
      else //tx & rx endpoints
        ep_stat[i]->toggle_bit = 0x00;
    }
  }
}
/*****************************************************************/
static inline void SetStallEpx(int ep_no)
{
  PDBG("%s...\n",__FUNCTION__);
  if(ep_no == EPT0) 
  {
    UsbWr0B(StallCtrlOutEP);
    UsbWr0B(StallCtrlInEP);
    // MS: ENABLE_TX0;
  }
  else 
  {
    UsbWr0B(StallCtrlInEP+ep_no); // StallEP1,... start behind StallCtrlInEP
  }                                    

  spin_lock(&DevState.lock);
  DevState.UsbState.EpState[ep_no] = EP_STALL;
  spin_unlock(&DevState.lock);
}
/*****************************************************************/
static inline void SetUnstallEpx(int ep_no)
{
  PDBG("%s...\n",__FUNCTION__);
  if(ep_no == EPT0) 
  {
    UsbWr0B(UnstallCtrlOutEP);
    UsbWr0B(UnstallCtrlInEP);
  }
  else 
  {
    UsbWr0B(UnstallCtrlInEP+ep_no);
  }                                    

  spin_lock(&DevState.lock);    
  DevState.UsbState.EpState[ep_no] = EP_ENABLED;
  spin_unlock(&DevState.lock);
}
/*****************************************************************/
static void init_Statistic_Device_State(void)
{ 
  int i;
  PDBG("%s...\n",__FUNCTION__);

  spin_lock(&DevState.lock);
  DevState.UsbState.BusState = USB_DISCONNECTED;
  
  for(i=1; i<NUM_OF_ENDPOINTS; i++)
  {
    DevState.UsbState.EpState[i] = EP_NOT_ASSIGNED;    
  }
  spin_unlock(&DevState.lock);
}
/*****************************************************************/
static inline void writefifo(BYTE *txdata, BYTE ep_num, int nobytes)
{
  unsigned short sTmp;

  PDBG("%s: (%dBytes) ", __FUNCTION__, nobytes);
  
  UsbWr0B(WriteCtrlInBuf+ep_num);
  UsbWrData2B(nobytes);
  
  while(nobytes > 0) 
  {
    if (nobytes == 1)
    {
      PDBG("%02x ",*txdata);
      sTmp    = *txdata++; // place last byte on low byte
      nobytes = 0;
    }
    else
    {
      // for intel-byte order simple cast would do, but to stay on the safe side:
      PDBG("%c",*txdata);
      sTmp  = *txdata++;        // place 1st byte on low byte
      // PDBG("%02x ",*txdata);
      sTmp += (*txdata++) << 8; // place 2nd byte on high byte
      nobytes -= 2;
    }
    UsbWrData2B(sTmp);
  }
  PDBG("\n");
}
/*****************************************************************/
static void fill_ctrl_fifo(void)
{
  BYTE *data = ctrl_send_buf.data;
  // get number of bytes to be sent
  int count = min(ctrl_send_buf.byte_cnts, (unsigned int)EP0_FIFO_SIZE);

  PDBG("%s...\n",__FUNCTION__);

  //update control buffer parameters
  ctrl_send_buf.data      += count;
  ctrl_send_buf.byte_cnts -= count;
  
  writefifo(data, EPT0, count);

  PDBG("byte_cnts after writefifo: %d\n", ctrl_send_buf.byte_cnts);
}
/*****************************************************************/
static void dev_enable_epx( const USB_endpoint_desc*  ep )
{
  EndPoint ep_no = ep->bEndpointAddress.address;
  
  if(ep->bEndpointAddress.direction == IN)
  {
    // MS: FLUSH_TXEP(ep_no);
  }
  else
  {
    // MS: FLUSH_RXEP(ep_no);
  }

  //enable endpoint & set its address
  // MS: SetUsbDevReg(EPC_ADDR(ep_no), ((BYTE)ep_no | EP_EN));

  if(ep->bEndpointAddress.direction == OUT)
    UsbWr0B(ClearCtrlOutBuf+1+ep_no);

  spin_lock(&DevState.lock);
  DevState.UsbState.EpState[ep_no] = EP_ENABLED;
  spin_unlock(&DevState.lock);
}
/*********************************************************/
static void dev_disable_epx(const USB_endpoint_desc *ep)
{
  PDBG("%s...\n",__FUNCTION__);
  if(ep->bEndpointAddress.direction == IN)  
  {
    // MS: FLUSH_TXEP(ep->bEndpointAddress.address);
  }
  else  
  {
    // MS: FLUSH_RXEP(ep->bEndpointAddress.address); 
    //PDBG("dev_disable_epx: EP: %d\n", ep->bEndpointAddress.address);
  }
  // MS: SetUsbDevReg(EPC_ADDR(ep->bEndpointAddress.address), 0);
  
  // TODO... but what???
  
  spin_lock(&DevState.lock);
  DevState.UsbState.EpState[ep->bEndpointAddress.address] = EP_DISABLED;
  spin_unlock(&DevState.lock);
}
/*****************************************************************/
static inline void tx_enable_epx(int ep_num)
{
  PDBG("%s...\n",__FUNCTION__);
  UsbWr0B(ValidateCtrlInBuf+ep_num);
}
/*********************************************************/
void zero_length_data_responseEP0(void)
{
  PDBG("%s...\n",__FUNCTION__);
  // MS: FLUSH_TX0;
  send_control_data(0,0);
  
  // tx_enable_epx(EPT0);
}
/*********************************************************/
static inline void send_ZLP_2_finish_EPx(BYTE ep_num)
{
  PDBG("%s...\n",__FUNCTION__);
  // MS: FLUSH_TXEP(ep_num);
  tx_enable_epx(ep_num);
}

/*******************************************************/
/* all 16 EPs have to be configured in a row for the   */
/* fifos to be valid                                   */
/*******************************************************/
void config_ep_fifos(void)
{
  PDBG("%s...\n",__FUNCTION__);

  UsbWr0B(ClearCtrlOutBuf);
  UsbWr0B(ClearEP1);
  UsbWr0B(ClearEP2);   
  UsbWr0B(ClearEP3);   
  UsbWr0B(ClearEP4);       
  UsbWr0B(ClearEP5);       
  UsbWr0B(ClearEP6);       
  UsbWr0B(ClearEP7);       
  UsbWr0B(ClearEP8);       
  UsbWr0B(ClearEP9);       
  UsbWr0B(ClearEP10);      
  UsbWr0B(ClearEP11);      
  UsbWr0B(ClearEP12);      
  UsbWr0B(ClearEP13);      
  UsbWr0B(ClearEP14);      

  UsbWr2B(WriteCtrlOutCfg, FIFOEN | 3 /* fifo size 64B */);
  UsbWr2B(WriteCtrlInCfg,  FIFOEN | EPDIR  | 3 /* fifo size 64B */);
  UsbWr2B(WriteEP1Cfg,     FIFOEN | EPDIR  | 3 /* fifo size 64B */);
  UsbWr2B(WriteEP2Cfg,     FIFOEN | 3  /* fifo size 64B */);
  UsbWr2B(WriteEP3Cfg,     FIFOEN | EPDIR  | 1 /* fifo size 8B */);
  UsbWr2B(WriteEP4Cfg,     0);
  UsbWr2B(WriteEP5Cfg,     FIFOEN | EPDIR  | 3 /* fifo size 64B*/);
  UsbWr2B(WriteEP6Cfg,     0);
  UsbWr2B(WriteEP7Cfg,     0);
  UsbWr2B(WriteEP8Cfg,     0);
  UsbWr2B(WriteEP9Cfg,     0);
  UsbWr2B(WriteEP10Cfg,    0);
  UsbWr2B(WriteEP11Cfg,    0);
  UsbWr2B(WriteEP12Cfg,    0);
  UsbWr2B(WriteEP13Cfg,    0);
  UsbWr2B(WriteEP14Cfg,    0); 
}

/*********************************************************/
void clear_usb_status(void)
{
  int iStat, iErr, iCfg;
  
  // EP0out:
  iCfg  = UsbRd2B(ReadCtrlOutCfg)&0xff;
  iStat = UsbRd2B(ReadCtrlOutStat)&0xff;
  iErr  = UsbRd2B(ReadCtrlOutErr)&0xff;
  UsbWr0B(ClearCtrlOutBuf);
  PDBG("EP0out: cfg=0x%x stat=0x%x err=0x%x\n",iCfg,iStat,iErr);
  
  // EP0in:
  iCfg  = UsbRd2B(ReadCtrlInCfg)&0xff;
  iStat = UsbRd2B(ReadCtrlInStat)&0xff;
  iErr  = UsbRd2B(ReadCtrlInErr)&0xff;
  PDBG("EP0in:  cfg=0x%x stat=0x%x err=0x%x\n",iCfg,iStat,iErr);
  
  // EP1in:
  iCfg  = UsbRd2B(ReadEP1Cfg)&0xff;
  iStat = UsbRd2B(ReadEP1Stat)&0xff;
  iErr  = UsbRd2B(ReadEP1Err)&0xff;
  PDBG("EP1in:  cfg=0x%x stat=0x%x err=0x%x\n",iCfg,iStat,iErr);
  rtxEP1Len = 0;

  // EP2out:
  iCfg  = UsbRd2B(ReadEP2Cfg)&0xff;
  iStat = UsbRd2B(ReadEP2Stat)&0xff;
  iErr  = UsbRd2B(ReadEP2Err)&0xff;
  UsbWr0B(ClearEP2);
  PDBG("EP2out: cfg=0x%x stat=0x%x err=0x%x\n",iCfg,iStat,iErr);

  // EP3in:
  iCfg  = UsbRd2B(ReadEP3Cfg)&0xff;
  iStat = UsbRd2B(ReadEP3Stat)&0xff;
  iErr  = UsbRd2B(ReadEP3Err)&0xff;
  PDBG("EP3in:  cfg=0x%x stat=0x%x err=0x%x\n",iCfg,iStat,iErr);
  rtxEP3Len = 0;

  // EP5in:
  iCfg  = UsbRd2B(ReadEP5Cfg)&0xff;
  iStat = UsbRd2B(ReadEP5Stat)&0xff;
  iErr  = UsbRd2B(ReadEP5Err)&0xff;
  PDBG("EP5in:  cfg=0x%x stat=0x%x err=0x%x\n",iCfg,iStat,iErr);
  rtxEP5Len = 0;
}

/*********************************************************/
void config_usb(void)
{
  unsigned short sTmp;
  PDBG("%s...\n",__FUNCTION__);

  /* hw config */
  /*************/
  // DACK-only DMA (DAKOLY)
  // powering off during suspend,
  UsbWr2B(WriteHwCfg, DAKOLY | PWROFF );
  
  /* clear status / irq flags */
  /****************************/
  clear_usb_status();
  
  /* irq config */
  /**************/
  // enable reset, resume, suspend, EP0, EP1, EP2, EP3, EP5 irq
  /*   UsbWr4B(WriteIntEnable, IERST | IESUSP | IERESM | */
  /*           IEP0IN | IEP0OUT | IEP1 | IEP2 | IEP3 | IEP5 ); */
  // HDT: enable IEP1, when opening dev
  UsbWr4B(WriteIntEnable, IERST | IESUSP | IERESM |
          IEP0IN | IEP0OUT | IEP2 | IEP3 | IEP5 );
  
  /* config fifos */
  /****************/
  config_ep_fifos();
  
  /* read/write addr (needed?)*/
  /***************************/
  sTmp  = UsbRd2B(ReadDevAddr);
  if (sTmp & DEVEN)
    UsbWr2B(WriteDevAddr,sTmp);
  else
    PDBG("%s: device not enabled!\n",__FUNCTION__);
  
}
/*******************************************************/
void deactivate_usb(void)
{
  PDBG("%s...\n",__FUNCTION__);
  
  // soft unplug USB
  UsbWr2B(WriteMode, 0);
  
  // reset isp1181b
  UsbWr0B(ResetDev);
}
/*******************************************************/
void activate_usb(void)
{
  unsigned short sTmp;
  PDBG("%s...\n",__FUNCTION__);

  // soft plug USB
  sTmp  = UsbRd2B(ReadMode)&0xff;
  sTmp |= INTENA | SOFTCT | DBGMOD;
  UsbWr2B(WriteMode, sTmp);
}

/*********************************************************/
void reset_usb(void)
{
  PDBG("%s...\n",__FUNCTION__);
  
  clear_control_buffer(&ctrl_send_buf);
  clear_control_buffer(&ctrl_receive_buf);
  endpoint_status_init();
  
  init_Statistic_Device_State();
  
  // reinitiate
  config_usb();
  clear_usb_status();
  activate_usb();
  
}
/*****************************************************************/
static inline void usb_device_reset(void)
{
  // PDBG("%s...\n",__FUNCTION__);
  if(device_status.state == ADDRESSED || 
     device_status.state == CONFIGURED)
    device_status.state = ATTACHED;
  
  device_buffers.zero_data = 0;
}
/**********************************************************/
static void send_control_data(BYTE* data, int size)
{
  // PDBG("%s...\n",__FUNCTION__);
  // MS: FLUSH_TX0;
  ctrl_send_buf.data = data;
  ctrl_send_buf.byte_cnts = size;
  fill_ctrl_fifo();
  tx_enable_epx(EPT0);
}
/***************************************************************
 ** standard device requests
 ****************************************************************/
void usb_dev_get_status(USB_request*  req)
{
  BYTE ep_no = REQ_INDEX(req).endpoint.ep_num;
  PDBG("%s...\n",__FUNCTION__);

  if((device_status.state == ATTACHED) || 
     (IS_REQ_VALUE_NOT_ZERO(req)) || 
     (REQ_LENGTH(req) != 2)) 
  {
    SetStallEpx(EPT0);
    return;
  }

  //Clear all reserved bits
  device_buffers.status.msb.as_byte = 0;

  switch(REQ_RECIPIENT(req)) 
  {
    case DEVICE_REQ:
      if(REQ_INDEX(req).as_bytes.lsb != 0)
      {//device's behaviour is not specified
        SetStallEpx(EPT0);
        return;
      }
      device_buffers.status.msb.device.wakeup = OFF;
      //self powered
      device_buffers.status.msb.device.selfpowered = ON;
      break;

    case INTERFACE_REQ:
      if(REQ_INDEX(req).interface.inf_no != 0)
      {
        SetStallEpx(EPT0);
        return;
      }
      //Reserved
      device_buffers.status.msb.interface.value = 0;
      break;

    case ENDPOINT_REQ:
      switch(ep_no) {
        case EPT0:
          device_buffers.status.msb.endpoint.stalled = (IS_EP0_STALLED)? ON : OFF;
          break;

        case EPT1:
        case EPT2:
        case EPT3:
        case EPT5:
          device_buffers.status.msb.endpoint.stalled = (IS_EP_STALLED(ep_no))? ON : OFF;
          break;
          //undefined endpoint
        case EPT4:            
        case EPT6:
        default:
          SetStallEpx(EPT0);
          return;
      }
      break;

    case OTHER_REQ:
    default: //undefined recipient
      SetStallEpx(EPT0);
      return;
  }
  //Reserved
  device_buffers.status.lsb = 0;
  send_control_data((BYTE *)&device_buffers.status, sizeof(USB_device_status));
}
/****************************************************************/
void usb_dev_clear_feature(USB_request*  req)
{
  BYTE ep_no = REQ_INDEX(req).endpoint.ep_num;
  
  PDBG("%s...\n",__FUNCTION__);
  if((device_status.state == ATTACHED) || 
     (REQ_LENGTH(req) != 0))
  {
    SetStallEpx(EPT0);
    return;
  }

  switch(REQ_RECIPIENT(req))
  {
    case DEVICE_REQ: // wakeup feature is not supported
      if(REQ_VALUE(req).feature.bSelector == DEVICE_REMOTE_WAKEUP)
        SetStallEpx(EPT0);
      break;

    case INTERFACE_REQ:
      break;

    case ENDPOINT_REQ: //clear stall state of appropriate endpoint    
      if((REQ_VALUE(req).feature.bSelector != ENDPOINT_STALL) || 
         (ep_no >= EPT_LAST) ||
         (usb_dev_endpoints[device_status.curAltSetting][ep_no] == NULL))
        SetStallEpx(EPT0);
      else  
      {
        SetUnstallEpx(ep_no);
        
        //actually the toogle bit should be reset
        ep_stat[ep_no]->toggle_bit = (ep_no == EPT0) ?  0x01 : 0x00;
        
        zero_length_data_responseEP0(); // MS: needed?
        // PDBG("clear stall EP: %d  is stall?: %d\n", ep_no, IS_EP_STALLED(ep_no));
      }
      break;
      
    case OTHER_REQ:
    default:
      SetStallEpx(EPT0);
      break;
  }
}
/****************************************************************/
void usb_dev_set_feature(USB_request*  req)
{
  BYTE ep_no = REQ_INDEX(req).endpoint.ep_num;

  PDBG("%s...\n",__FUNCTION__);
  if((device_status.state == ATTACHED) || (REQ_LENGTH(req) != 0))
  {
    SetStallEpx(EPT0);
    return;
  }
  switch(REQ_RECIPIENT(req))
  {
    case DEVICE_REQ:
      if(REQ_VALUE(req).feature.bSelector == DEVICE_REMOTE_WAKEUP)
        SetStallEpx(EPT0); //remote wakeup is not supported
      break;

    case INTERFACE_REQ:
      break;

    case ENDPOINT_REQ:
      if(REQ_VALUE(req).feature.bSelector != ENDPOINT_STALL ||
         (ep_no >= EPT_LAST) || (usb_dev_endpoints[device_status.curAltSetting][ep_no] == NULL))
        SetStallEpx(EPT0);
      else
      {//set appropriate endpoint to stall state
        //(ep_no == EPT0) ? SETSTALLEP0 : STALL_EP(ep_no);
        SetStallEpx(ep_no);            
        zero_length_data_responseEP0();
        // PDBG("SetStallEpx(%d) in usb_dev_set_feature\n", ep_no);          
      }
      break;
      
    case OTHER_REQ:
    default:
      SetStallEpx(EPT0);
      break;
  }
  //PDBG("usb_dev_set_feature -> EPT%d is set stall now \n", ep_no);
}
/****************************************************************/
void usb_dev_set_address(USB_request *req)
{
  ushort address;

  PDBG("%s...\n",__FUNCTION__);
  if(IS_REQ_INDEX_NOT_ZERO(req) || (REQ_LENGTH(req) != 0x0) || 
     (device_status.state == CONFIGURED)) 
  {
    SetStallEpx(EPT0);
    return;
  }
  switch(REQ_RECIPIENT(req))
  {
    case DEVICE_REQ://ENABLE_DEFAULT_ADDRESS
      address = REQ_VALUE(req).as_bytes.lsb;
      PDBG("WRITE_DEV_ADDRESS: 0x%x\n",address);
      UsbWr2B(WriteDevAddr,address | DEVEN);
      zero_length_data_responseEP0();
      
      if(REQ_VALUE(req).as_bytes.lsb == 0x0)   
        device_status.state =  ATTACHED;
      else   
        device_status.state =  ADDRESSED;
      break;
      
    case INTERFACE_REQ:
    case ENDPOINT_REQ:
    case OTHER_REQ:
    default:
      SetStallEpx(EPT0);
      break;
  }
}
/****************************************************************/
void usb_std_dev_get_descriptor(USB_request*   req)
{
  int desc_length = 0;
  int desc_index = REQ_VALUE(req).descriptor.bDescriptorIndex;
  BYTE *desc_buf = NULL;
  int max_desc_length = REQ_LENGTH(req);

  switch(REQ_RECIPIENT(req))   
  {
    case DEVICE_REQ:
      switch(REQ_VALUE(req).descriptor.bDescriptorType)
      {
        case DEVICE_DESCRIPTOR:
          desc_length = usb_device_desc.bLength;
          desc_buf = (BYTE *)&usb_device_desc;
          break;

        case CONFIG_DESCRIPTOR:
          desc_length = usb_dev_long_config_desc.usb_dev_config_desc.wTotalLength;
          desc_buf = (BYTE *)&usb_dev_long_config_desc;
          break;

        case STRING_DESCRIPTOR:
          if(desc_index < STR_LAST_INDEX && desc_index >= 0)   
          {
            desc_length = string_descs[desc_index]->bLength;
            desc_buf = (BYTE *)string_descs[desc_index];
            break;
          }
			  
        default: //wrong string index
          SetStallEpx(EPT0);
          return;
      }
      desc_length = (desc_length < max_desc_length)? desc_length : max_desc_length;

      //zero data length will not be required
      if(desc_length % EP0_FIFO_SIZE)
        device_buffers.zero_data=0;
      
      //zero data length will be required
      else
        device_buffers.zero_data=1;
      
      send_control_data(desc_buf, desc_length);
      break;

    case INTERFACE_REQ:
    case ENDPOINT_REQ:
    case OTHER_REQ:
    default:
      SetStallEpx(EPT0);
      break;
  }//switch(REQ_RECIPIENT)
}
/****************************************************************/
void usb_std_dev_set_descriptor(USB_request *req)
{//not supportted  optional
  PDBG("%s...\n",__FUNCTION__);
  SetStallEpx(EPT0);
}
/****************************************************************/
void usb_dev_get_config(USB_request*  req)
{
  PDBG("%s...\n",__FUNCTION__);
  if((device_status.state==ATTACHED) || IS_REQ_VALUE_NOT_ZERO(req) ||
     IS_REQ_INDEX_NOT_ZERO(req) || (REQ_LENGTH(req) != 0x1))  
  {
    SetStallEpx(EPT0);
    return;
  }
  
  switch(REQ_RECIPIENT(req))  
  {
    case DEVICE_REQ://returns configuration number as appears at the device configuration register
      device_buffers.state = (device_status.state == CONFIGURED)?
        usb_dev_long_config_desc.usb_dev_config_desc.bConfigurationValue : 0;
      
      send_control_data((BYTE *)&device_buffers.state, sizeof(device_buffers.state));
      break;
      
    case INTERFACE_REQ:
    case ENDPOINT_REQ:
      
    case OTHER_REQ:
    default:
      SetStallEpx(EPT0);
      break;
  }
}
/****************************************************************/
void usb_dev_set_config(USB_request*  req)
{
  int i;

  PDBG("%s...\n",__FUNCTION__);
  if(IS_REQ_INDEX_NOT_ZERO(req) || (REQ_LENGTH(req) != 0))  
  {
    SetStallEpx(EPT0);
    return;
  }
	
  switch(REQ_RECIPIENT(req))
  {
    case DEVICE_REQ:
      if(device_status.state == ATTACHED)  
      {
        SetStallEpx(EPT0);
        break;
      }
      
      if(REQ_VALUE(req).as_bytes.lsb == 0) 
      {
        if(device_status.state == CONFIGURED) 
        {//Deactivate current configuration
          for (i=1; i<MAX_NUM_ENDPOINTS; i++)  
          {
            if(usb_dev_endpoints[device_status.curAltSetting][i] != NULL)
              dev_disable_epx(usb_dev_endpoints[device_status.curAltSetting][i]);
          }
          device_status.state = ADDRESSED;
          zero_length_data_responseEP0();
        }
        else
          SetStallEpx(EPT0);
      }
      else if(REQ_VALUE(req).as_bytes.lsb == 
              usb_dev_long_config_desc.usb_dev_config_desc.bConfigurationValue) 
      {
        //Deactivate previous configuration
        for (i=1; i< MAX_NUM_ENDPOINTS; i++) 
        {
          if(usb_dev_endpoints[device_status.curAltSetting][i] != NULL)
            dev_disable_epx(usb_dev_endpoints[device_status.curAltSetting][i]);
        }
        device_status.curAltSetting = 0;
        
        
        //Activate this configuration
        reset_usb();
        for(i=0; i< MAX_NUM_ENDPOINTS; i++)
        {
          if(usb_dev_endpoints[device_status.curAltSetting][i] != NULL)
          {
            dev_enable_epx(usb_dev_endpoints[device_status.curAltSetting][i]);
          }
        }
        
        endpoint_status_init();
        device_status.state = CONFIGURED;
        zero_length_data_responseEP0();
        
        if(device_status.state == CONFIGURED) 
        {
          spin_lock(&DevState.lock);
          DevState.UsbState.BusState = USB_CONNECTED;
          spin_unlock(&DevState.lock);			 
#ifdef _SELF_MODULE_UNLOAD_					 
          UsbBusError = 0;
#endif		 
          PDBG("..............Device is configured and connected...............\n");
        }
      }
      else //wrong configuration value
        SetStallEpx(EPT0);
      break;
      
    case INTERFACE_REQ:
    case ENDPOINT_REQ:
    case OTHER_REQ:
    default:
      SetStallEpx(EPT0);
      break;
  }
}
/****************************************************************/
void usb_dev_get_interface(USB_request*   req)
{
  BYTE interface_no  = REQ_INDEX(req).as_bytes.lsb;
  //only single interface is supported
  PDBG("%s...\n",__FUNCTION__);
  if((device_status.state != CONFIGURED) || (interface_no != 0) || 
     IS_REQ_VALUE_NOT_ZERO(req) || (REQ_LENGTH(req) != 0x1))
  {
    SetStallEpx(EPT0);
    return;
  }
  
  switch(REQ_RECIPIENT(req)) 
  {
    case INTERFACE_REQ:
      device_buffers.state = device_status.curAltSetting;
      send_control_data((BYTE *)&device_buffers.state, sizeof(device_buffers.state));
      break;
    case DEVICE_REQ:
    case ENDPOINT_REQ:
    case OTHER_REQ:
    default:
      SetStallEpx(EPT0);
      break;
  }
}
/****************************************************************/
void usb_dev_set_interface(USB_request*   req)
{
  BYTE interface_no  = REQ_INDEX(req).as_bytes.lsb;
  BYTE altSet = REQ_VALUE(req).as_bytes.lsb;
  int i;
  //only single interface is supported
  PDBG("%s...\n",__FUNCTION__);
  if((device_status.state != CONFIGURED) || (interface_no != 0) 
     || IS_REQ_VALUE_NOT_ZERO(req) || (REQ_LENGTH(req) != 0)) 
  {
    SetStallEpx(EPT0);
    return;
  }
  switch(REQ_RECIPIENT(req))
  {
    case INTERFACE_REQ:  //we have to revisit this place if we want to support
      if(altSet > 0)   //alternate settings at least 1, now we have 0 settings
      {//Not supported Alternative setting
        SetStallEpx(EPT0);
        break;
      }
      if(altSet != device_status.curAltSetting)
      {
        //Change setting & deactivate previous setting
        for(i=1; i< MAX_NUM_ENDPOINTS; i++) 
        {
          if(usb_dev_endpoints[device_status.curAltSetting][i] != NULL)
            dev_disable_epx(usb_dev_endpoints[device_status.curAltSetting][i]);
        }
        device_status.curAltSetting = altSet;

        //Activate this configuration
        reset_usb();
      }
      zero_length_data_responseEP0();
      break;
      
    case DEVICE_REQ:
    case ENDPOINT_REQ:
    case OTHER_REQ:
    default:
      SetStallEpx(EPT0);
      break;
  }
}
/****************************************************************/
void usb_dev_sync_frame(USB_request*  req)
{//optional not supported
  SetStallEpx(EPT0);
}
/****************************************************************/

/********************************************************
 ** vendor requests
 *********************************************************/
void Cmd_Data_Parser(USB_request* req)
{
  BYTE  command = REQ_VENDOR(req);
  WORD  reqlen  = REQ_LENGTH(req);

  PDBG("%s...\n",__FUNCTION__);
  switch(command) 
  {
    case BULK_DATA_OUT_AMOUNT:
      //PDBG("Cmd_Parser bulkstate.dataSize: %d\n", bulkstate.dataSize);
      gusbtransf.BulkTfout.dataSize = ((DWORD)(REQ_VALUE(req).lsw)|(((DWORD)(REQ_INDEX(req).msw)) << 16));
      gusbtransf.BulkTfout.rest2process = gusbtransf.BulkTfout.dataSize;
      gusbtransf.BulkTfout.command = command;
      //wake up the read function
      /* if(gDeviceOpen == YES) */
      /* { */
      /*   wake_up_interruptible(&gusbtransf.CtrlTfout.waitq); */
      /* } */
      break;

    case BULK_DATA_IN_AMOUNT:
      gusbtransf.BulkTfin.dataSize = ((DWORD)(REQ_VALUE(req).lsw)|(((DWORD)(REQ_INDEX(req).msw)) << 16));
      gusbtransf.BulkTfin.rest2process = gusbtransf.BulkTfin.dataSize;
      gusbtransf.BulkTfin.command = command;
      break;

    case BULK_DATA_IN_STREAM:
      gusbtransf.BulkTfin.command = BULK_DATA_IN_STREAM;
      break;

    case CTRL_DATA_OUT:
      gusbtransf.CtrlTfout.dataSize = reqlen;
      gusbtransf.CtrlTfout.rest2process = reqlen;
      gusbtransf.CtrlTfout.command = command;

      if(gusbtransf.CtrlTfout.dataSize)   gusbtransf.CtrlTfout.ready = NO;
      break;

    case CTRL_DATA_IN:
    case BULK_DATA_OUT_STREAM:
      PDBG("CTRL_DATA_IN is not supported\n");
      break;

    default:
      SetStallEpx(EPT0);
      break;
  }

  //re-enable receive of data
  FlushRx(EPT0);
  UsbWr0B(ClearCtrlOutBuf);

  if(gusbtransf.CtrlTfout.rest2process == 0) 
    zero_length_data_responseEP0();
}
/****************************************************************/
void dev_req_handler(void)
{
  BYTE *msg = ctrl_receive_buf.data;
  USB_request *req = (USB_request*) msg;

  PDBG("%s...\n",__FUNCTION__);
  if(msg == NULL) 
    return;

  switch(REQ_TYPE(req)) 
  {
    case STANDARD_REQ:
      (*usb_std_device_req[REQ_DEVICE(req)])(req);
      break;
      
    case VENDOR_REQ:
      (*usb_vendor_device_req[REQ_VENDOR_TYPE(req)])(req);
      break;
      
    case CLASS_REQ:
    default:
      SetStallEpx(EPT0);
      break;
  }
}
/**********************************************************/
static inline BYTE USB_Transmit_Data(BYTE *tdata, int data_size, EndPoint ep_num)
{
  BYTE size;
  
  //assure that data size is no more then fifo length (and > 0)
  size = min(data_size, fifo_sizes[ep_num]);
  if (size <= 0)
    return 0;
  writefifo(tdata, ep_num, size);
  
  // prepare RTX buffer
  switch (ep_num)
  {
    case EPT1:
      memcpy(rtxEP1Buf,tdata,size);
      rtxEP1Len = size;
      break;

    case EPT3:
      memcpy(rtxEP3Buf,tdata,size);
      rtxEP3Len = size;
      break;

    case EPT5:
      memcpy(rtxEP5Buf,tdata,size);
      rtxEP5Len = size;
      break;

    default:
      // nothing
      break;
  }
  
  // transmit
  tx_enable_epx(ep_num);

  return size;
}
/**********************************************************/
static inline BYTE USB_Retransmit_Data(EndPoint ep_num)
{
  BYTE size;

  switch (ep_num)
  {
    case EPT1:
      size = min((int)rtxEP1Len, fifo_sizes[ep_num]);
      if (size <= 0)
        return 0;
      writefifo(rtxEP1Buf, ep_num, size);
      break;
      
    case EPT3:
      size = min((int)rtxEP3Len, fifo_sizes[ep_num]);
      writefifo(rtxEP3Buf, ep_num, size);
      break;
      
    case EPT5:
      size = min((int)rtxEP5Len, fifo_sizes[ep_num]);
      writefifo(rtxEP5Buf, ep_num, size);
      break;
      
    default:
      printk("ERR: %s invalid EP%d\n",__FUNCTION__,ep_num);
      break;
  }
  
  // retransmit
  tx_enable_epx(ep_num);
  
  return size;
}
/**********************************************************/
inline int SendInterruptData(BYTE *idata, DWORD size, int ep_no)
{
  return USB_Transmit_Data(idata, size, ep_no);
}
/*************************************************************/
inline void SendBulkData(void)
{ 
  BYTE cnt = 0;
  
  spin_lock(&DevState.lock);
  DevState.UsbState.EpState[EPT1] = EP_IN_PROGRESS;
  spin_unlock(&DevState.lock);

  // spin_unlock_wait(&gusbtransf.BulkTfin.lock);
  spin_lock_irq(&gusbtransf.BulkTfin.lock);

  if(BULK_MAX_BUFFER_SIZE == gusbtransf.BulkTfin.buffcnt)
  {
    spin_unlock_irq(&gusbtransf.BulkTfin.lock);
    wake_up_interruptible(&gusbtransf.BulkTfin.waitq);
    return;
  }
  
  if(gusbtransf.BulkTfin.rest2process > 0) 
  {
    //Is the is more date to be sent?
    SetBulkSendDataTimeout();
    
    cnt = min(gusbtransf.BulkTfin.rest2process, (DWORD)TX_BULK_EP_FIFO_SIZE);
    
    USB_Transmit_Data((gusbtransf.BulkTfin.buffer + gusbtransf.BulkTfin.buffcnt), cnt , EPT1);
    
    gusbtransf.BulkTfin.buffcnt      += cnt;
    gusbtransf.BulkTfin.rest2process -= cnt;
  }
  else  
  {
    gusbtransf.BulkTfin.ready = YES;
    rtxEP1Len = 0; // not rtx needed!

    if(gusbtransf.BulkTfin.exitOnZLP == YES) 
    {
      //more data to send or shall be finished with a zero length package??
      SetBulkSendDataTimeout();   
      send_ZLP_2_finish_EPx(EPT1);
      PDBG("Send ZLP!!!\n");
    }
  }

  if(gusbtransf.BulkTfin.ready == YES)  
  {
    wake_up_interruptible(&gusbtransf.BulkTfin.waitq);
  }

  spin_unlock_irq(&gusbtransf.BulkTfin.lock);
}

/*****************************************************/
static inline int readfifo(BYTE* rxdata, BYTE ep_num, int bufsiz)
{
  int keep = 0;
  int len, dat;
  
  PDBG("%s: 0x",__FUNCTION__);
  
  /* read len */
  /************/
  switch (ep_num)
  {
    case EPT0:
      keep = len = UsbRd2B(ReadCtrlOutBuf);
      break;
      
    case EPT2:
      keep = len = UsbRd2B(ReadEP2Buf);
      break;
      
    default:
      PDBG("ERR: USBDEV: read from EP%d\n",ep_num);
      return keep;
  }
  
  /* read data */
  /*************/
  len = len>bufsiz?bufsiz:len;
  while (len > 0)
  {
    dat = UsbRdData2B();
    *rxdata++ = dat & 0xff;          // 1st byte
    PDBG("%02x ", *(rxdata-1));
    if (len > 1)
    {
      *rxdata++ = (dat >> 8) & 0xff; // 2nd byte
      PDBG("%02x ", *(rxdata-1));
    }
    len -= 2;
  }
  PDBG("\n");

  return keep;
}
/**********************************************************/
static inline BYTE USB_Receive_Data(BYTE *rdata, EndPoint ep_num)
{
  BYTE bytes_count;
  BYTE bytes_sum = 0;

  // Read data from the fifo until it's empty
  PDBG("%s...\n",__FUNCTION__);
  do 
  {
    //Read count of bytes presently in the FIFO
    bytes_count = readfifo((rdata+bytes_sum), ep_num, BULK_MAX_BUFFER_SIZE);
    bytes_sum += bytes_count;
    // If the FIFO containes more than 15 bytes, a value 15 is reported in the counter.
    // Therefore continue reading from the fifo until the counter is less then 15.
  } while (bytes_count == 0x0f);
  
  FlushRx(ep_num);
  return bytes_sum;
}
/******************************************************************/
static int BufferFull(void)
{
  int ret;

  spin_unlock_wait(&gusbtransf.BulkTfout.lock);
  spin_lock(&gusbtransf.BulkTfout.lock);
#ifdef DEBUG
  if(BULK_MAX_BUFFER_SIZE == gusbtransf.BulkTfout.buffcnt)  PDBG("gusbtransf.BulkTfout.buffcnt: %d\n", gusbtransf.BulkTfout.buffcnt);
#endif
  ret = (BULK_MAX_BUFFER_SIZE == gusbtransf.BulkTfout.buffcnt) ? YES : NO;
  
  spin_unlock(&gusbtransf.BulkTfout.lock);
  
  return ret;
}


