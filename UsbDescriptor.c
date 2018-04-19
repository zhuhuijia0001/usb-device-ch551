#include "Type.h"
#include "UsbDef.h"
#include "UsbDescriptor.h"

/* device descriptor */
static UINT8C _DevDesc[] = 
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

/* device descriptor */
const DescriptorData DevDesc = 
{
    _DevDesc,

    sizeof(_DevDesc)
};

/* keyboard report descriptor */
static UINT8C _KeyRepDesc[] =
{
	0x05, 0x01,         /*  Usage Page (Desktop),               */
	0x09, 0x06,         /*  Usage (Keyboard),                   */
	0xA1, 0x01,         /*  Collection (Application),           */
	0x05, 0x07,         /*      Usage Page (Keyboard),          */
	0x19, 0xE0,         /*      Usage Minimum (KB Leftcontrol), */
	0x29, 0xE7,         /*      Usage Maximum (KB Right GUI),   */
	0x15, 0x00,         /*      Logical Minimum (0),            */
	0x25, 0x01,         /*      Logical Maximum (1),            */
	0x75, 0x01,         /*      Report Size (1),                */
	0x95, 0x08,         /*      Report Count (8),               */
	0x81, 0x02,         /*      Input (Variable),               */
	0x95, 0x01,         /*      Report Count (1),               */
	0x75, 0x08,         /*      Report Size (8),                */
	0x81, 0x01,         /*      Input (Constant),               */
	0x95, 0x03,         /*      Report Count (3),               */
	0x75, 0x01,         /*      Report Size (1),                */
	0x05, 0x08,         /*      Usage Page (LED),               */
	0x19, 0x01,         /*      Usage Minimum (01h),            */
	0x29, 0x03,         /*      Usage Maximum (03h),            */
	0x91, 0x02,         /*      Output (Variable),              */
	0x95, 0x05,         /*      Report Count (5),               */
	0x75, 0x01,         /*      Report Size (1),                */
	0x91, 0x01,         /*      Output (Constant),              */
	0x95, 0x06,         /*      Report Count (6),               */
	0x75, 0x08,         /*      Report Size (8),                */
	0x26, 0xFF, 0x00,   /*      Logical Maximum (255),          */
	0x05, 0x07,         /*      Usage Page (Keyboard),          */
	0x19, 0x00,         /*      Usage Minimum (None),           */
	0x29, 0x91,         /*      Usage Maximum (KB LANG2),       */
	0x81, 0x00,         /*      Input,                          */
	0xC0                /*  End Collection                      */
};


/* keyboard report descriptor */
const DescriptorData KeyRepDesc = 
{
    _KeyRepDesc,

    sizeof(_KeyRepDesc)
};

/* mouse report descriptor */
static UINT8C _MouseRepDesc[] =
{
	0x05, 0x01, /*  Usage Page (Desktop),               */
	0x09, 0x02, /*  Usage (Mouse),                      */
	0xA1, 0x01, /*  Collection (Application),           */
	0x09, 0x01, /*      Usage (Pointer),                */
	0xA1, 0x00, /*      Collection (Physical),          */
	0x05, 0x09, /*          Usage Page (Button),        */
	0x19, 0x01, /*          Usage Minimum (01h),        */
	0x29, 0x03, /*          Usage Maximum (03h),        */
	0x15, 0x00, /*          Logical Minimum (0),        */
	0x25, 0x01, /*          Logical Maximum (1),        */
	0x75, 0x01, /*          Report Size (1),            */
	0x95, 0x03, /*          Report Count (3),           */
	0x81, 0x02, /*          Input (Variable),           */
	0x75, 0x05, /*          Report Size (5),            */
	0x95, 0x01, /*          Report Count (1),           */
	0x81, 0x01, /*          Input (Constant),           */
	0x05, 0x01, /*          Usage Page (Desktop),       */
	0x09, 0x30, /*          Usage (X),                  */
	0x09, 0x31, /*          Usage (Y),                  */
	0x09, 0x38, /*          Usage (Wheel),              */
	0x15, 0x81, /*          Logical Minimum (-127),     */
	0x25, 0x7F, /*          Logical Maximum (127),      */
	0x75, 0x08, /*          Report Size (8),            */
	0x95, 0x03, /*          Report Count (3),           */
	0x81, 0x06, /*          Input (Variable, Relative), */
	0xC0,       /*      End Collection,                 */
	0xC0        /*  End Collection                      */
};

/* mouse report descriptor */
const DescriptorData MouseRepDesc	=
{
    _MouseRepDesc,

    sizeof(_MouseRepDesc)
};

#define TOTAL_CONFIG_DESCR_SIZE     sizeof(USB_CFG_DESCR) + \
									sizeof(USB_ITF_DESCR) + \
									sizeof(USB_HID_DESCR) + \
									sizeof(USB_ENDP_DESCR) + \
									sizeof(USB_ITF_DESCR) + \
									sizeof(USB_HID_DESCR) + \
									sizeof(USB_ENDP_DESCR)
									
#define KEYBOARD_REPORT_DESCR_SIZE  sizeof(_KeyRepDesc)

#define MOUSE_REPORT_DESCR_SIZE     sizeof(_MouseRepDesc)

/* configuration descriptor */
static UINT8C _CfgDesc[] =
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

/* configure descriptor */
const DescriptorData CfgDesc = 
{
    _CfgDesc,

    sizeof(_CfgDesc)
};

#define STRING_LANGID_SIZE          4
#define STRING_VENDOR_SIZE          12
#define STRING_PRODUCT_SIZE         22
#define STRING_SERIAL_SIZE          18


static UINT8C StringLangID[STRING_LANGID_SIZE] = 
{
	STRING_LANGID_SIZE,
	USB_DESCR_TYP_STRING,
	0x09, 0x04
};

static UINT8C StringVecdor[STRING_VENDOR_SIZE] = 
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

static UINT8C StringProduct[STRING_PRODUCT_SIZE] = 
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

const DescriptorData StringDescriptors[4] = 
{
	{ StringLangID, sizeof(StringLangID) },
	{ StringVecdor, sizeof(StringVecdor) },
	{ StringProduct, sizeof(StringProduct) },
	{ StringSerial, sizeof(StringSerial) }
};

