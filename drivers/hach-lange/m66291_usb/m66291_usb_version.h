/***************************************************************************
                          usb_version.h  -  description
                             -------------------
    begin                : Di Feb 22 2005
    copyright            : (C) 2005 by Thomas Siegmund
    email                : tsiegmund@hach-lange.de
    Author               : Nikhil Varghese
***************************************************************************/
#define USB_VERSION    0x0110  /* 1.1 (full speed) */
#define VENDOR_ID      0x1670  /* assigned by USB Forum */
#define PRODUCT_ID     0x00C8  /* assigned by company see PID assignments.xls */
#define BCDDEVICE      0x0108  /* assigned by developer */

#define MANUFACTURE_STR   "H\0a\0c\0h\0-\0L\0a\0n\0g\0e\0 \0G\0m\0b\0H"
#define PRODUCT_STR       "H\0a\0c\0h\0-\0L\0a\0n\0g\0e\0 \0P\0h\0o\0t\0o\0m\0e\0t\0e\0r"

#define VERSION_STRW       "1\0.\00\02" 
#ifdef _SELF_MODULE_UNLOAD_	
#define VERSION_STRA       "01.02"
#else
#define VERSION_STRA       "01.02"	
#endif

#define VERSION_USB        102
