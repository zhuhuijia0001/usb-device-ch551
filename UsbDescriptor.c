#include "Type.h"
#include "UsbDef.h"
#include "UsbDescriptor.h"

/* device descriptor */
UINT8C DevDesc[DEV_DESCR_SIZE] = 
{
	0x12,                 /*bLength */
	USB_DESCR_TYP_DEVICE, /*bDescriptorType*/
	0x00, 0x02,           /*bcdUSB */
	0x00,                 /*bDeviceClass*/
	0x00,                 /*bDeviceSubClass*/
	0x00,                 /*bDeviceProtocol*/
	DEFAULT_ENDP0_SIZE,   /*bMaxPacketSize*/
	0x3b, 0x41,           /*idVendor (0x413b)*/
	0x07, 0x21,           /*idProduct = 0x2107*/
	0x00, 0x00,           /*bcdDevice rel. 0.00*/
	0x01,                 /*Index of string descriptor describing
                                                  manufacturer */
	0x02,                 /*Index of string descriptor describing
                                                 product*/
    0x03,                 /*Index of string descriptor describing the
                                                 device serial number */
    0x01                  /*bNumConfigurations*/
};

/* configuration descriptor */
UINT8C CfgDesc[TOTAL_CONFIG_DESCR_SIZE] =
{
    0x09,                 /* bLength: Configuation Descriptor size */
    USB_DESCR_TYP_CONFIG, /* bDescriptorType: Configuration */
    TOTAL_CONFIG_DESCR_SIZE & 0xff, (TOTAL_CONFIG_DESCR_SIZE >> 8) & 0xff,/* wTotalLength: Bytes returned */
    0x02,                 /*bNumInterfaces: 2 interface*/
    0x01,                 /*bConfigurationValue: Configuration value*/
    0x00,                 /*iConfiguration: Index of string descriptor describing
                                     the configuration*/
    0xC0,                 /*bmAttributes: self powered */
    0x32,                 /*MaxPower 100 mA: this current is used for detecting Vbus*/

    /************** Descriptor of keyboard interface ****************/
    0x09,                 /*bLength: Interface Descriptor size*/
    USB_DESCR_TYP_INTERF, /*bDescriptorType: Interface descriptor type*/
    0x00,                 /*bInterfaceNumber: Number of Interface*/
    0x00,                 /*bAlternateSetting: Alternate setting*/
    0x01,                 /*bNumEndpoints*/
    0x03,                 /*bInterfaceClass: HID*/
    0x01,                 /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x01,                 /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0x00,                 /*iInterface: Index of string descriptor*/

    /******************** Descriptor of keyboard HID ********************/
    0x09,                 /*bLength: HID Descriptor size*/
    USB_DESCR_TYP_HID,    /*bDescriptorType: HID*/
    0x11, 0x01,           /*bcdHID: HID Class Spec release number*/
    0x00,                 /*bCountryCode: Hardware target country*/
    0x01,                 /*bNumDescriptors: Number of HID class descriptors to follow*/
    USB_DESCR_TYP_REPORT, /*bDescriptorType*/
    KEYBOARD_REPORT_DESCR_SIZE & 0xff, (KEYBOARD_REPORT_DESCR_SIZE >> 8) & 0xff, /*wItemLength: Total length of Report descriptor*/

    /******************** Descriptor of keyboard endpoint ********************/
    0x07,                 /*bLength: Endpoint Descriptor size*/
    USB_DESCR_TYP_ENDP,   /*bDescriptorType:*/
    0x81,                 /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,                 /*bmAttributes: Interrupt endpoint*/
    MAX_PACKET_SIZE & 0xff, (MAX_PACKET_SIZE >> 8) & 0xff, /*wMaxPacketSize: 32 Byte max */
    0x0a,                 /*bInterval: Polling Interval (10 ms)*/

    /************** Descriptor of mousse interface ****************/
    0x09,                 /*bLength: Interface Descriptor size*/
    USB_DESCR_TYP_INTERF, /*bDescriptorType: Interface descriptor type*/
    0x01,                 /*bInterfaceNumber: Number of Interface*/
    0x00,                 /*bAlternateSetting: Alternate setting*/
    0x01,                 /*bNumEndpoints*/
    0x03,                 /*bInterfaceClass: HID*/
    0x01,                 /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x02,                 /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0x00,                 /*iInterface: Index of string descriptor*/

    /******************** Descriptor of mouse HID ********************/
    0x09,                /*bLength: HID Descriptor size*/
    USB_DESCR_TYP_HID,   /*bDescriptorType: HID*/
    0x10, 0x01,          /*bcdHID: HID Class Spec release number*/
    0x00,                /*bCountryCode: Hardware target country*/
    0x01,                /*bNumDescriptors: Number of HID class descriptors to follow*/
    USB_DESCR_TYP_REPORT,/*bDescriptorType*/
    MOUSE_REPORT_DESCR_SIZE & 0xff, (MOUSE_REPORT_DESCR_SIZE >> 8) & 0xff, /*wItemLength: Total length of Report descriptor*/

    /******************** Descriptor of mouse endpoint ********************/
    0x07,                /*bLength: Endpoint Descriptor size*/
    USB_DESCR_TYP_ENDP,  /*bDescriptorType:*/
    0x82,                /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,                /*bmAttributes: Interrupt endpoint*/
    MAX_PACKET_SIZE & 0xff, (MAX_PACKET_SIZE >> 8) & 0xff, /*wMaxPacketSize: 32 Byte max */
    0x0a                 /*bInterval: Polling Interval (10 ms)*/
};

/* keyboard report descriptor */
UINT8C KeyRepDesc[KEYBOARD_REPORT_DESCR_SIZE] =
{
    0x05, 0x01,
    0x09, 0x06,

    0xA1, 0x01,
    0x05, 0x07,
    
    0x19, 0xe0,
    0x29, 0xe7,
    0x15, 0x00,
    0x25, 0x01,
    
    0x75, 0x01,
    0x95, 0x08,
    0x81, 0x02,

    0x95, 0x01,
    0x75, 0x08,
    0x81, 0x01,

    0x95, 0x03,
    0x75, 0x01,
    0x05, 0x08,
    0x19, 0x01,
    0x29, 0x03,
    0x91, 0x02,
    
    0x95, 0x05,
    0x75, 0x01,
    0x91, 0x01,

    0x95, 0x06,
    0x75, 0x08,
    0x26, 0xff, 0x00,
    0x05, 0x07,
    0x19, 0x00,
    0x29, 0x91,
    0x81, 0x00,

    0xC0
};

/* mouse report descriptor */
UINT8C MouseRepDesc[MOUSE_REPORT_DESCR_SIZE] =
{
    0x05, 0x01,

    0x09, 0x02,

    0xA1, 0x01,
    0x09, 0x01,
    0xA1, 0x00,
    0x05, 0x09,
    0x19, 0x01,
    0x29, 0x03,
    0x15, 0x00,
    0x25, 0x01,

    0x75, 0x01,
    0x95, 0x03,
    0x81, 0x02,

    0x75, 0x05,
    0x95, 0x01,
    0x81, 0x01,
    
    0x05, 0x01,
    0x09, 0x30,
    0x09, 0x31,
    0x09, 0x38,
    0x15, 0x81, 
    0x25, 0x7f,
    0x75, 0x08,
    0x95, 0x03,
    0x81, 0x06,

    0xC0,
    0xC0
};

UINT8C StringLangID[STRING_LANGID_SIZE] = 
{
	STRING_LANGID_SIZE,
	USB_DESCR_TYP_STRING,
	0x09, 0x04
};

UINT8C StringVecdor[STRING_VENDOR_SIZE] = 
{
	STRING_VENDOR_SIZE,
	USB_DESCR_TYP_STRING,

	/* vendor */
	'L', 0,
	'a', 0,
	'n', 0,
	'b', 0,
	'e', 0
};

UINT8C StringProduct[STRING_PRODUCT_SIZE] = 
{
	STRING_PRODUCT_SIZE,
	USB_DESCR_TYP_STRING,

	/* product */
	'K', 0,
	'V', 0,
	'M', 0,
	' ', 0,
	'D', 0,
	'e', 0,
	'v', 0,
	'i', 0,
	'c', 0,
	'e', 0	
};

UINT8X StringSerial[STRING_SERIAL_SIZE] = 
{
	STRING_SERIAL_SIZE,
	USB_DESCR_TYP_STRING,

	/* serial */
	'0', 0,
	'0', 0,
	'0', 0,
	'0', 0,
	'0', 0,
	'0', 0,
	'0', 0,
	'0', 0
};

DescriptorData StringDescriptors[4] = 
{
	{ StringLangID, sizeof(StringLangID) },
	{ StringVecdor, sizeof(StringVecdor) },
	{ StringProduct, sizeof(StringProduct) },
	{ StringSerial, sizeof(StringSerial) }
};


