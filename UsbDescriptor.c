#include "Type.h"
#include "UsbDef.h"
#include "hid.h"
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
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),
	HID_Usage(HID_USAGE_GENERIC_KEYBOARD),
	HID_Collection(HID_Application),
		HID_UsagePage(HID_USAGE_PAGE_KEYBOARD),
		HID_UsageMin(0xE0),
		HID_UsageMax(0xE7),

		HID_LogicalMin(0),
		HID_LogicalMax(1),
		HID_ReportSize(1),
		HID_ReportCount(8),
	
		HID_Input(HID_Data | HID_Variable | HID_Absolute),

		HID_ReportSize(8),
		HID_ReportCount(1),
		
		HID_Input(HID_Constant | HID_Variable | HID_Absolute),

		HID_ReportSize(1),
		HID_ReportCount(5),
		
		HID_UsagePage(HID_USAGE_PAGE_LED),
		HID_UsageMin(0x01),
		HID_UsageMax(0x05),
		
		HID_Output(HID_Data | HID_Variable | HID_Absolute),

		HID_ReportSize(3),
		HID_ReportCount(1),
		HID_Output(HID_Constant | HID_Variable | HID_Absolute),

		HID_ReportSize(8),
		HID_ReportCount(6),
		HID_LogicalMin(0),
		HID_LogicalMax(0xFF),
		HID_UsagePage(HID_USAGE_PAGE_KEYBOARD),
		HID_UsageMin(0x00),
		HID_UsageMax(0x91),
		HID_Input(HID_Data | HID_Array | HID_Absolute),
	HID_EndCollection,
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
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),

	HID_Usage(HID_USAGE_GENERIC_MOUSE),

	HID_Collection(HID_Application),
		HID_Usage(HID_USAGE_GENERIC_POINTER),
		HID_Collection(HID_Physical),
			HID_UsagePage(HID_USAGE_PAGE_BUTTON),
			HID_UsageMin(0x01),
			HID_UsageMax(0x03),
			HID_LogicalMin(0),
			HID_LogicalMax(1),
			HID_ReportSize(1),
			HID_ReportCount(3),
			HID_Input(HID_Data | HID_Variable | HID_Absolute),

			HID_ReportSize(5),
			HID_ReportCount(1),
			HID_Input(HID_Constant | HID_Variable | HID_Absolute),

			HID_UsagePage(HID_USAGE_PAGE_GENERIC),
			HID_Usage(HID_USAGE_GENERIC_X),
			HID_Usage(HID_USAGE_GENERIC_Y),
			HID_Usage(HID_USAGE_GENERIC_WHEEL),
			HID_LogicalMin(0x81),
			HID_LogicalMax(0x7F),
			HID_ReportSize(8),
			HID_ReportCount(3),
			HID_Input(HID_Data | HID_Variable | HID_Relative),
		HID_EndCollection,
	HID_EndCollection,
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
    0x80 | (1 << 6) | (1 << 5), /*bmAttributes: self powered, remote wakeup */
    100 / 2,                 /*MaxPower 100 mA: this current is used for detecting Vbus*/

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

