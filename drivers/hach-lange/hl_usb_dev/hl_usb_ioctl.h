/***************************************************************************
                          usbn9603ioctl.h  -  description
                             -------------------
    begin                : Don Sep 9 2004
    copyright            : (C) 2004 by Thomas Siegmund
    email                : tsiegmund@hach-lange.de
***************************************************************************/

/***********************************************************************/
/*********************** IO Control codes ******************************/
/***********************************************************************/
/* 'M' as magic number */
#define  USB_IO_MAGIC      'M'
#define  USB_IO_RESET      _IO(USB_IO_MAGIC, 0)

#define  USB_IO_CONN_USB                _IO(USB_IO_MAGIC, 1) //, connect)
#define  USB_IO_GET_BULK_DATA_SIZE      _IOR(USB_IO_MAGIC, 2, io_data)
#define  USB_IO_BULK_DATA_IN            _IOW(USB_IO_MAGIC, 3, io_data)
//#define  USB_IO_USER_BREAK_DOWN       _IOW(USB_IO_MAGIC, 4, io_data)
#define  USB_IO_USER_BREAK              _IO(USB_IO_MAGIC, 4)
#define  USB_IO_GET_READ_STREAM_SIZE    _IOR(USB_IO_MAGIC, 5, io_data)
#define  USB_IO_GET_USB_STATUS          _IO(USB_IO_MAGIC, 6)
#define  USB_IO_SET_TIMEOUT_WRITE_MEASUREMENT       _IOW(USB_IO_MAGIC, 7, io_data)
#define  USB_IO_SET_MEAS_DATA_SIZE      _IOW(USB_IO_MAGIC, 8, io_data)
#define  USB_IO_SET_TIMEOUT_WRITE_BULK  _IOW(USB_IO_MAGIC, 9, io_data)
#define  USB_IO_GET_COMMAND             _IOR(USB_IO_MAGIC, 10, io_data)
#define  USB_IO_GET_VERSION             _IOR(USB_IO_MAGIC, 11, io_data)
#define  USB_IO_IOCTL_MAX               12
/************************************************************/

#define   IRQ_OUTP_LOW     0xC0 //bit 7,6
