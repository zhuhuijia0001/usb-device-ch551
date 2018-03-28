
#ifndef USB_DESCRIPTOR_H_
#define USB_DESCRIPTOR_H_

/* size of descriptors */
#define DEV_DESCR_SIZE    sizeof(USB_DEV_DESCR)
#define TOTAL_CONFIG_DESCR_SIZE     sizeof(USB_CFG_DESCR) + \
									sizeof(USB_ITF_DESCR) + \
									sizeof(USB_HID_DESCR) + \
									sizeof(USB_ENDP_DESCR) + \
									sizeof(USB_ITF_DESCR) + \
									sizeof(USB_HID_DESCR) + \
									sizeof(USB_ENDP_DESCR)

#define KEYBOARD_REPORT_DESCR_SIZE  62
#define MOUSE_REPORT_DESCR_SIZE     52

#define STRING_LANGID_SIZE          4
#define STRING_VENDOR_SIZE          12
#define STRING_PRODUCT_SIZE         22
#define STRING_SERIAL_SIZE          18

/* descriptors */
extern UINT8C DevDesc[DEV_DESCR_SIZE];

extern UINT8C CfgDesc[TOTAL_CONFIG_DESCR_SIZE];

extern UINT8C KeyRepDesc[KEYBOARD_REPORT_DESCR_SIZE];

extern UINT8C MouseRepDesc[MOUSE_REPORT_DESCR_SIZE];

extern UINT8C StringLangID[STRING_LANGID_SIZE];
extern UINT8C StringVecdor[STRING_VENDOR_SIZE];
extern UINT8C StringProduct[STRING_PRODUCT_SIZE];
extern UINT8X StringSerial[STRING_SERIAL_SIZE];

typedef struct
{
	PUINT8 descr;
	UINT8 size;
} DescriptorData;

/* string descriptors */
extern DescriptorData StringDescriptors[4];

#endif
