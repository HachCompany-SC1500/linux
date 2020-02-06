/***************************************************************************
                          UsbDescriptor.h  -  description
                             -------------------
    begin                : Die Jun 29 2004
    copyright            : (C) 2004 by Thomas Siegmund
    email                : tsiegmund@hach-lange.de
***************************************************************************/
#include "hl_usb_version.h"
/*
  #define USB_VERSION    0x0110  * 1.1 *
  #define NSC_ID_VENDOR  0x1670  * assigned by USB Forum *
  #define NSC_ID_PRODUCT 0x9603           * assigned by NSC *
  #define NSC_BCDDEVICE  1       * assigned by developer *
  
  #define MANUFACTURE_STR  "H\0a\0c\0h\0-\0L\0a\0n\0g\0e\0 \0G\0m\0b\0H\0 \0E\0G\0L"
  #define PRODUCT_STR       "U\0S\0B\0 \0G\0a\0m\0m\0a\0 \0C\0o\0m\0m\0u\0n\0i\0c\0a\0t\0i\0o\0n\0 \0D\0e\0v\0i\0c\0e"
  #define VERSION_STR       "1\0.\01"
*/
//#define POLLING_INTERVALL_INT_EP3 0x01  //0xFF

//#pragma pack(1)
#pragma pack(push, old_pack_val, 1)

typedef struct {
  BYTE bLength;
  BYTE bDescriptorType;
  struct {
    BYTE address:4;
    BYTE reserved:3;
    BYTE direction:1;
  } bEndpointAddress;
  BYTE   bmAttributes;
  X2BYTES wMaxPacketSize;
  BYTE  bInterval;
} USB_endpoint_desc;

typedef struct {
  BYTE  bLength;
  BYTE  bDescriptorType;
  BYTE  bInterfaceNumber;
  BYTE  bAlternateSetting;
  BYTE  bNumEndpoints;
  BYTE  bInterfaceClass;
  BYTE  bInterfaceSubClass;
  BYTE  bInterfaceProtocol;
  BYTE  iInterface;
} USB_interface_desc;

typedef struct {
  BYTE  bLength;
  BYTE  bDescriptorType;
  char* bstring;
} USB_string_desc;

typedef struct {
  union {
    struct {
      BYTE selfpowered:1;
      BYTE wakeup:1;
      BYTE:6;
    } device;
    struct {
      BYTE value;
    } interface;
    struct {
      BYTE stalled:1;
      BYTE:7;
    } endpoint;
    BYTE as_byte;
  } msb;
  BYTE lsb;
} USB_device_status;

typedef enum {
  GET_STATUS = 0,
  CLEAR_FEATURE,
  RESERVED_REQ,
  SET_FEATURE = 3,
  SET_ADDRESS = 5,
  GET_DESCRIPTOR,
  SET_DESCRIPTOR,
  GET_CONFIG,
  SET_CONFIG,
  GET_INTERFACE,
  SET_INTERFACE,
  SYNC_FRAME
} USB_device_Request;

typedef struct {
  USB_device_state     state;
  int             curAltSetting;
} DEVICE_status;


#define MAX_STRING_LENGTH  0xFF
typedef union {
  USB_device_status  status;     /* used in GET_STATUS request */
  BYTE               state;      /* used in GET_CONFIGURATION request */
  BYTE               zero_data;  /*used while data sending */ //in GET_DESCRIPTOR request
  BYTE               desc;
  //    DWORD              DataSizMulPack;
} Device_buffers;

typedef enum {
  STANDARD_REQ = 0,
  CLASS_REQ = 1,
  VENDOR_REQ = 2
} USB_request_type;

typedef enum {
  DEVICE_REQ = 0,
  INTERFACE_REQ,
  ENDPOINT_REQ,
  OTHER_REQ
} USB_request_recipient;

typedef enum {
  ENDPOINT_STALL = 0,
  DEVICE_REMOTE_WAKEUP = 1
} USB_feature_selector;

typedef enum {
  OUT = 0,
  IN
} USB_EP_direction;

typedef struct {
  BYTE recipient:5;  /* Request Recipient */
  BYTE type:2;       /* Request Type */
  BYTE xfer:1;       /* Data xfer direction */
} bmRequestType;

typedef enum {
  DEVICE_DESCRIPTOR = 1,
  CONFIG_DESCRIPTOR,
  STRING_DESCRIPTOR,
  INTERFACE_DESCRIPTOR,
  ENDPOINT_DESCRIPTOR,
  USB_DEVICE_DESCRIPTOR = 0x21
} USB_descriptor_type;

typedef struct {
  bmRequestType bmReqType;
  union {
    BYTE Device_req;
    BYTE Vendor_req;
  } bRequest;
  union {
    struct {
      BYTE bDescriptorIndex; //bStringIndex;
      BYTE bDescriptorType;
    } descriptor;
    struct {
      BYTE bSelector;
      BYTE msb;
    } feature;
    struct {
      BYTE lsb;
      BYTE msb;
    } as_bytes;
    X2BYTES lsw;
  } wValue;
  union {
    struct {
      BYTE ep_num:4;
      BYTE:3;
      BYTE direction:1;
      BYTE:8;
    } endpoint;
    struct {
      BYTE inf_no;
      BYTE msb;
    } interface;
    struct {
      BYTE lsb;
      BYTE msb;
    } as_bytes;
    X2BYTES msw;
  } wIndex;
  X2BYTES wLength;
} USB_request;

#define REQ_DIRECTION(req)  ((USB_xfer_direction)(req->bmReqType.xfer))
#define REQ_TYPE(req)       ((USB_request_type)(req->bmReqType.type))
#define REQ_RECIPIENT(req)  ((USB_request_recipient)(req->bmReqType.recipient))
#define REQ_DEVICE(req)     ((USB_device_Request)(req->bRequest.Device_req & 0x0F))
#define REQ_VENDOR_TYPE(req)((req->bRequest.Vendor_req & 0xF0)>>4)
#define REQ_VENDOR(req)      (req->bRequest.Vendor_req & 0xFF)
#define REQ_VALUE(req)       (req->wValue)
#define REQ_INDEX(req)       (req->wIndex)
#define REQ_LENGTH(req)      (req->wLength)

typedef enum {
  CONTROL_EP= 0,
  ISOCHRONOUS_EP,
  BULK_EP,
  INTERRUPT_EP
} Endpoint_type;

typedef enum {
  CLASS_NOT_DEFINED   = 0x0,
  CLASS_AUDIO         = 0x01,
  CLASS_COMMUNICATION = 0x02,
  CLASS_HID           = 0x03,
  CLASS_MONITOR       = 0x04,
  CLASS_PRINTING      = 0x07,
  CLASS_MASS_STORAGE  = 0x08,
  CLASS_HUB           = 0x09,
  CLASS_VENDOR        = 0xFF,
  //USBTMC	
  CLASS_APP		    = 0xFE, //application class USB2.0 assigned by USB-IF
} Device_class;

typedef enum {
  //Test and Measurement Class
  SUB_CLASS_TMC        = 0x03,   //assigned by USB-IF
} Device_SubClass;

typedef enum {
  BUS_POWERED   = 0x80,
  SELF_POWERED  = 0xC0, //version1.1
  //    SELF_POWERED  = 0x40,
  REMOTE_WAKEUP = 0x20
} Power_config;

typedef enum {
  STR_LANGID = 0,
  STR_MANUFACTURER = 1,
  STR_PRODUCT,
  STR_VERSION,
  STR_LAST_INDEX
} String_index;

typedef struct {
  BYTE  bLength;
  BYTE  bDescriptorType;
  X2BYTES  bcdUSB;
  BYTE  bDeviceClass;
  BYTE  bDeviceSubClass;
  BYTE  bDeviceProtocol;
  BYTE  bMaxPacketSize;
  X2BYTES   idVendor;
  X2BYTES   idProduct;
  X2BYTES   bcdDevice;
  BYTE  iManufacturer;
  BYTE  iProduct;
  BYTE  iSerialNumber;
  BYTE  bNumConfigs;
} USB_device_desc;

typedef struct {
  BYTE  bLength;
  BYTE  bDescriptorType;
  X2BYTES  wTotalLength;
  BYTE  bNumInterfaces;
  BYTE  bConfigurationValue;
  BYTE  iConfiguration;
  BYTE  bmAttributes;
  BYTE  MaxPower;
} USB_config_desc;

#define MAX_NUM_ENDPOINTS    6

//#define NUM_OF_ENDPOINTS   7

//#define NUM_OF_ALTERNATESETTING     0
#define NUM_OF_ENDPOINTS_FOR_ALT_0  4 //now there are only 4 endpoints
#define NUM_OF_ENDPOINTS_FOR_ALT_1  0

#define CONFIG_DESC_LENGTH    0x9
#define INTERFACE_DESC_LENGTH 0x9

typedef struct {
  USB_config_desc      usb_dev_config_desc;
  USB_interface_desc   usb_interface_0_alt_0_desc;
  USB_endpoint_desc    usb_dev_endpoint_alt_0_desc[NUM_OF_ENDPOINTS_FOR_ALT_0];
  /*
    USB_interface_desc   usb_interface_0_alt_1_desc;
    USB_endpoint_desc    usb_dev_endpoint_alt_1_desc[NUM_OF_ENDPOINTS_FOR_ALT_1];
  */
} USB_long_config_desc;

/*=====================================================================
 *     Vendor USB Device Descriptor
 *=====================================================================*/
typedef struct {
  BYTE  bLength;
  BYTE  bDescriptorType;
  DWORD bDeviceIdValue;
}USB_vendor_device_desc;

//there are different language ID's but we only take this one
#define ENGLISH_US    0x0409
#define LANGID_LENGTH      2

#pragma pack(pop, old_pack_val)



