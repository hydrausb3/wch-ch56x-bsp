/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb_desc_cmd.c
* Author             : bvernoux
* Version            : V1.0
* Date               : 2022/08/20
* Description        :
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb_devbulk_desc_cmd.h"

/*
Use default USB PID/VID if not configured with usb_descriptor_set_usb_vid_pid()
https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
PID dec (hex) | VID dec (hex) | Description of use
==============+===============+============================================
1500 (0x05dc) | 5824 (0x16c0) | For Vendor Class devices with libusb
*/
// USB Product ID (defined by user code)
// Default PID 0x05DC "For Vendor Class devices with libusb"
#define PRODUCT_ID_BYTE_MSB (0x05)
#define PRODUCT_ID_BYTE_LSB (0xDC)
// USB Vendor ID (defined by user code)
// Default VID 0x16C0 "Van Ooijen Technische Informatica"
#define VENDOR_ID_BYTE_MSB (0x16)
#define VENDOR_ID_BYTE_LSB (0xC0)

/*********************/
/* Device Descriptor */
/*********************/
/* USB2.0 hs_device_descriptor */
#define USB_HS_SS_DESCRIPTOR_VID_POS (8)
uint8_t USB_HS_DeviceDescriptor[LEN_USB_HS_DeviceDescriptor] =
{
	LEN_USB_HS_DeviceDescriptor, // length of this descriptor
	0x01,   // DEVICE descriptor type
	0x10,   // USB Version 2.1 (0x10 0x02) (USB Version 2.1 required for BOS Descriptor)
	0x02,
	0x00,   // device class
	0x00,   // device sub-class
	0x00,   // vendor specific protocol
	0x40,   // max packet size 64B USB 2.0 HS

	VENDOR_ID_BYTE_LSB, // vendor id
	VENDOR_ID_BYTE_MSB,

	PRODUCT_ID_BYTE_LSB, // product id
	PRODUCT_ID_BYTE_MSB,

	0x00,   // bcdDevice 0x0011
	0x11,

	0x01,   // manufacturer index string
	0x02,   // product index string
	0x03,   // serial number index string
	0x01    // number of configurations
};

/* USB3.0 SuperSpeed device descriptor */
uint8_t USB_SS_DeviceDescriptor[LEN_USB_SS_DeviceDescriptor] =
{
	LEN_USB_SS_DeviceDescriptor, // length of this descriptor
	0x01, // DEVICE descriptor type
	0x00, // USB Version 3.0 (0x00 0x03)
	0x03,

	0x00, // device class
	0x00, // device sub-class
	0x00, // vendor specific protocol
	0x09, // max packet size 0x09 = 512B USB 3.0 SS

	VENDOR_ID_BYTE_LSB, // vendor id
	VENDOR_ID_BYTE_MSB,

	PRODUCT_ID_BYTE_LSB, // product id
	PRODUCT_ID_BYTE_MSB,

	0x00, // bcdDevice 0x0001
	0x01,

	0x01, // manufacturer index string
	0x02, // product index string
	0x03, // serial number index string
	0x01  // number of configurations
};

/*******************************************************************************
 * @fn      usb_descriptor_set_usb_vid_pid
 *
 * @brief   Set USB VID/PID
 *          Precondition: call USB30D_init()
 *
 * @param  vid_pid: USB Vendor ID and Product ID
 *
 * @return  None
 */
void usb_descriptor_set_usb_vid_pid(usb_descriptor_usb_vid_pid_t *vid_pid)
{
	/* USB VID */
	USB_HS_DeviceDescriptor[USB_HS_SS_DESCRIPTOR_VID_POS + 0] = vid_pid->vid.id_8b[0];
	USB_HS_DeviceDescriptor[USB_HS_SS_DESCRIPTOR_VID_POS + 1] = vid_pid->vid.id_8b[1];

	USB_SS_DeviceDescriptor[USB_HS_SS_DESCRIPTOR_VID_POS + 0] = vid_pid->vid.id_8b[0];
	USB_SS_DeviceDescriptor[USB_HS_SS_DESCRIPTOR_VID_POS + 1] = vid_pid->vid.id_8b[1];

	/* USB PID */
	USB_HS_DeviceDescriptor[USB_HS_SS_DESCRIPTOR_VID_POS + 2] = vid_pid->pid.id_8b[0];
	USB_HS_DeviceDescriptor[USB_HS_SS_DESCRIPTOR_VID_POS + 3] = vid_pid->pid.id_8b[1];

	USB_SS_DeviceDescriptor[USB_HS_SS_DESCRIPTOR_VID_POS + 2] = vid_pid->pid.id_8b[0];
	USB_SS_DeviceDescriptor[USB_HS_SS_DESCRIPTOR_VID_POS + 3] = vid_pid->pid.id_8b[1];
}

/*********************/
/* Config Descriptor */
/*********************/
/* USB2.0 High Speed configuration descriptor */
uint8_t USB_HS_ConfigDescriptor[LEN_USB_HS_ConfigDescriptor] =
{
	0x09,   // length of this descriptor
	0x02,   // CONFIGURATION (2)
	LEN_USB_HS_ConfigDescriptor, // total length includes endpoint descriptors (should be 1 more than last address)
	0x00,   // total length high byte
	0x01,   // number of interfaces
	0x01,   // configuration value for this one
	0x00,   // configuration - string is here, 0 means no string
	0x80,   // attributes - bus powered, no wakeup
	0x64,   // max power - 800 ma is 100 (64 hex)

	0x09,   // length of the interface descriptor
	0x04,   // INTERFACE (4)
	0x00,   // Zero based index 0f this interface
	0x00,   // Alternate setting value (?)
	0x04,   // Number of endpoints (not counting 0)
	0xff,   // Interface class, ff is vendor specific
	0xff,   // Interface sub-class
	0xff,   // Interface protocol
	0x00,   // Index to string descriptor for this interface

	//Endpoint 1 Descriptor
	0x07,   // length of this endpoint descriptor
	0x05,   // ENDPOINT (5)
	0x81,   // endpoint direction (80 is in) and address
	0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
	0x00,   // max packet size - 512 bytes
	0x02,   // max packet size - high
	0x00,   // polling interval in milliseconds (1 for iso)

	0x07,   // length of this endpoint descriptor
	0x05,   // ENDPOINT (5)
	0x01,   // endpoint direction (00 is out) and address
	0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
	0x00,   // max packet size - 512 bytes
	0x02,   // max packet size - high
	0x00,    // polling interval in milliseconds (1 for iso)

	//Endpoint 2 Descriptor
	0x07,   // length of this endpoint descriptor
	0x05,   // ENDPOINT (5)
	0x82,   // endpoint direction (80 is in) and address
	0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
	0x00,   // max packet size - 512 bytes
	0x02,   // max packet size - high
	0x00,   // polling interval in milliseconds (1 for iso)

	0x07,   // length of this endpoint descriptor
	0x05,   // ENDPOINT (5)
	0x02,   // endpoint direction (80 is in) and address
	0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
	0x00,   // max packet size - 512 bytes
	0x02,   // max packet size - high
	0x00    // polling interval in milliseconds (1 for iso)
};

/* USB3.0 SuperSpeed configuration descriptor */
uint8_t USB_SS_ConfigDescriptor[LEN_USB_SS_ConfigDescriptor] =
{
	0x09, // length of this descriptor
	0x02, // CONFIGURATION (2)
	LEN_USB_SS_ConfigDescriptor, // total length includes endpoint descriptors (should be 1 more than last address)
	0x00, // total length high byte
	0x01, // number of interfaces
	0x01, // configuration value for this one
	0x00, // configuration - string is here, 0 means no string
	0x80, // attributes - bus powered, no wakeup
	0x64, // max power - 800 ma is 100 (64 hex)
	//interface_descriptor
	0x09, // length of the interface descriptor
	0x04, // INTERFACE (4)
	0x00, // Zero based index 0f this interface
	0x00, // Alternate setting value (?)
	0x04, // Number of endpoints (not counting 0)
	0xff, // Interface class, ff is vendor specific
	0xff, // Interface sub-class
	0xff, // Interface protocol
	0x00, // Index to string descriptor for this interface

	//Endpoint 1 Descriptor
	0x07, // length of this endpoint descriptor
	0x05, // ENDPOINT (5)
	0x81, // endpoint direction (80 is in) and address
	0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
	0x00, // max packet size - 1024 bytes
	0x04, // max packet size - high
	0x00, // polling interval in milliseconds (1 for iso)
	//endp1_compansion_desc
	0x06, // length of this endpoint compansion descriptor
	0x30,
	DEF_ENDP1_IN_BURST_LEVEL - 1, // max burst size
	0x00, // no stream
	0x00,
	0x00,
	//endp1_descriptor
	0x07, // length of this endpoint descriptor
	0x05, // ENDPOINT (5)
	0x01, // endpoint direction (00 is out) and address
	0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
	0x00, // max packet size - 1024 bytes
	0x04, // max packet size - high
	0x00, // polling interval in milliseconds (1 for iso)
	//endp1_compansion_desc
	0x06, // length of this endpoint compansion descriptor
	0x30,
	DEF_ENDP1_OUT_BURST_LEVEL - 1, // max burst size
	0x00, // no stream
	0x00,
	0x00,

	//Endpoint 2 Descriptor
	0x07, // length of this endpoint descriptor
	0x05, // ENDPOINT (5)
	0x82, // endpoint direction (80 is in) and address
	0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
	0x00, // max packet size - 1024 bytes
	0x04, // max packet size - high
	0x00, // polling interval in milliseconds (1 for iso)
	//endp2_compansion_desc
	0x06, // length of this endpoint compansion descriptor
	0x30,
	DEF_ENDP2_IN_BURST_LEVEL - 1, // max burst size
	0x00, // no stream
	0x00,
	0x00,
	//endp2_descriptor
	0x07, // length of this endpoint descriptor
	0x05, // ENDPOINT (5)
	0x02, // endpoint direction (00 is out) and address
	0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
	0x00, // max packet size - 1024 bytes
	0x04, // max packet size - high
	0x00, // polling interval in milliseconds (1 for iso)
	//endp2_compansion_desc
	0x06, // length of this endpoint compansion descriptor
	0x30,
	DEF_ENDP2_OUT_BURST_LEVEL - 1, // max burst size
	0x00, // no stream
	0x00,
	0x00
};

/*********************/
/* String Descriptor */
/*********************/

/* USB 2.0 / 3.0 String descriptor Lang ID USB_DESCR_LANGID_STRING */
uint8_t USB_StringLangID[LEN_USB_StringLangID] =
{
	LEN_USB_StringLangID, // length of this descriptor
	0x03, /* USB Descriptor Type String */
	0x09, // Language ID 0 low byte
	0x04  // Language ID 0 high byte
};

/* USB 2.0 / 3.0 String Vendor USB_DESCR_VENDOR_STRING */
uint8_t USB_StringVendor[LEN_USB_StringVendor] =
{
	LEN_USB_StringVendor, // length of this descriptor
	0x03, /* USB Descriptor Type String */
	'H', 0x00,
	'y', 0x00,
	'd', 0x00,
	'r', 0x00,
	'a', 0x00,
	'B', 0x00,
	'u', 0x00,
	's', 0x00
};

/*  USB 2.0 High Speed String Product USB_DESCR_PRODUCT_STRING */
uint8_t USB_HS_StringProduct[LEN_USB_HS_StringProduct]=
{
	LEN_USB_HS_StringProduct, // length of this descriptor
	0x03, /* USB Descriptor Type String */
	'H', 0x00,
	'y', 0x00,
	'd', 0x00,
	'r', 0x00,
	'a', 0x00,
	'U', 0x00,
	'S', 0x00,
	'B', 0x00,
	'3', 0x00
};

/*  USB 3.0 SuperSpeed String Product USB_DESCR_PRODUCT_STRING */
uint8_t USB_SS_StringProduct[LEN_USB_SS_StringProduct] =
{
	LEN_USB_SS_StringProduct, // length of this descriptor
	0x03, /* USB Descriptor Type String */
	'H', 0x00,
	'y', 0x00,
	'd', 0x00,
	'r', 0x00,
	'a', 0x00,
	'U', 0x00,
	'S', 0x00,
	'B', 0x00,
	'3', 0x00
};

/*  USB 2.0 / 3.0  String Serial USB_DESCR_SERIAL_STRING */
#define USB_DESCRIPTOR_SN_POS (28)
uint8_t USB_StringSerial[LEN_USB_StringSerial] =
{
	LEN_USB_StringSerial, // length of this descriptor
	0x03, /* USB Descriptor Type String */
	'H', 0x00,
	'y', 0x00,
	'd', 0x00,
	'r', 0x00,
	'a', 0x00,
	'U', 0x00,
	'S', 0x00,
	'B', 0x00,
	'3', 0x00,
	'_', 0x00,
	'S', 0x00,
	'N', 0x00,
	':', 0x00,
	/* Data set by usb_descriptor_set_string_serial_number() with 64bits Serial Number from WCH CH569 UID to ASCII HEX */
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00,
	' ', 0x00
};

static const uint8_t htoa[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

/*******************************************************************************
 * @fn      usb_descriptor_set_string_serial_number
 *
 * @brief   Set USB string descriptor serial number
 *          Precondition: call USB30D_init()
 *
 * @param  serial_number: USB string descriptor serial number
 *
 * @return  None
 */
void usb_descriptor_set_string_serial_number(usb_descriptor_serial_number_t *serial_number)
{
	int i, j;

	j = 0;
	for(i = 0; i < 8; i++)
	{
		uint8_t data_u8 = serial_number->sn_8b[i];
		USB_StringSerial[USB_DESCRIPTOR_SN_POS + j] = htoa[(data_u8 & 0xF0) >> 4];
		j+=2;
		USB_StringSerial[USB_DESCRIPTOR_SN_POS + j] = htoa[(data_u8 & 0x0F)];
		j+=2;
	}
}

/* USB2.0 / 3.0 OS String Descriptor USB_DESCR_OS_STRING */
uint8_t USB_OSStringDescriptor[LEN_USB_OSStringDescriptor] =
{
	LEN_USB_OSStringDescriptor, // length of this descriptor
	0x03,
	'M', 0x00,
	'S', 0x00,
	'F', 0x00,
	'T', 0x00,
	'1', 0x00,
	'0', 0x00,
	'0', 0x00,
	0x01,
	0x00
};

/* USB2.0 / 3.0 Binary Device Object Store (BOS) descriptor */
uint8_t USB_BOSDescriptor[LEN_USB_BOSDescriptor] =
{
	// USB 3.0 and USB 2.0 LPM Binary Device Object Store (BOS) Descriptor
	0x05, // length of this descriptor
	0x0f, // CONFIGURATION (2)
	LEN_USB_BOSDescriptor, // total length includes endpoint descriptors (should be 1 more than last address)
	0x00, // total length high byte
	0x02, // number of device cap

	0x07,
	0x10, // DEVICE CAPABILITY type
	0x02, // USB2.0 EXTENSION
	0x1E, // bmAttributes
	// LPM Capable=1, BESL And Alternate HIRD Supported=1,
	// Baseline BESL Valid=1, Deep BESL Valid=1,
	0xF4, // Baseline BESL=4 (400 us), Deep BESL=15 (10000 us)
	0x00,
	0x00,

	0x0a, // length of this descriptor
	0x10, // DEVICE CAPABILITY type
	0x03, // superspeed usb device capability
	0x00, //
	0x0c, // ss/hs
	0x00,
	0x02, // the lowest speed is high speed
	0x0a, // u1 exit latency is 10us
	0xff, // u1 exit latency is 8us
	0x07
};

/* USB2.0 / 3.0 Microsoft OS 2.0 descriptor */
uint8_t USB_MSOS20DescriptorSet[LEN_USB_MSOS20DescriptorSet] =
{
	// Microsoft OS 2.0 Descriptor Set Header
	0x0A, 0x00,             // wLength - 10 bytes
	0x00, 0x00,             // MSOS20_SET_HEADER_DESCRIPTOR
	0x00, 0x00, 0x03, 0x06, // dwWindowsVersion 0x06030000 for Windows Blue
	LEN_USB_MSOS20DescriptorSet, 0x00, // wTotalLength 72 bytes
	// Microsoft OS 2.0 Registry Value Feature Descriptor
	0x3E, 0x00,             // wLength - 62 bytes
	0x04, 0x00,             // wDescriptorType 4 for Registry Property
	0x04, 0x00,             // wPropertyDataType - 4 for REG_DWORD
	0x30, 0x00,             // wPropertyNameLength 48 bytes
	0x53, 0x00, 0x65, 0x00, // Property Name - SelectiveSuspendEnabled??
	0x6C, 0x00, 0x65, 0x00,
	0x63, 0x00, 0x74, 0x00,
	0x69, 0x00, 0x76, 0x00,
	0x65, 0x00, 0x53, 0x00,
	0x75, 0x00, 0x73, 0x00,
	0x70, 0x00, 0x65, 0x00,
	0x6E, 0x00, 0x64, 0x00,
	0x45, 0x00, 0x6E, 0x00,
	0x61, 0x00, 0x62, 0x00,
	0x6C, 0x00, 0x65, 0x00,
	0x64, 0x00, 0x00, 0x00,
	0x04, 0x00,            // wPropertyDataLength 4 bytes
	0x01, 0x00, 0x00, 0x00 // PropertyData - 0x00000001
};

/* USB2.0 / 3.0 Compatible Id (WINUSB) */
uint8_t USB_CompatId[LEN_USB_CompatId] =
{
	LEN_USB_CompatId, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01,
	'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* USB2.0 / 3.0 Property Header (Device GUID) */
uint8_t USB_PropertyHeader[LEN_USB_PropertyHeader] =
{
	LEN_USB_PropertyHeader, 0x00, 0x00, 0x00, 0x00, 01, 05, 00, 01, 00,
	0x84, 0x00, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x00,
	0x28, 0x00,
	'D', 0x00,  'e', 0x00,  'v', 0x00,  'i', 0x00,  'c', 0x00,  'e', 0x00,
	'I', 0x00,  'n', 0x00,  't', 0x00,  'e', 0x00,  'r', 0x00,  'f', 0x00,  'a', 0x00,  'c', 0x00,  'e', 0x00,
	'G', 0x00,  'U', 0x00,  'I', 0x00,  'D', 0x00,
	0x00, 0x00,
	0x4e, 0x00, 0x00, 0x00,
	0x7b, 0x00,
	'8', 0x00,  '2', 0x00,  'C', 0x00,  '3', 0x00,  'C', 0x00,  '2', 0x00,  'F', 0x00,  '5', 0x00,
	'-', 0x00,
	'9', 0x00,  '5', 0x00,  'A', 0x00,  '4', 0x00,
	'-', 0x00,
	'4', 0x00,  'C', 0x00,  '8', 0x00,  '5', 0x00,
	'-', 0x00,
	'9', 0x00,  '5', 0x00,  'E', 0x00,  '4', 0x00,
	'-', 0x00,
	'5', 0x00,  '4', 0x00,  'F', 0x00,  '8', 0x00,  '9', 0x00,  'D', 0x00,  'C', 0x00,  '2', 0x00,  'E', 0x00,  'F', 0x00,  '1', 0x00,  'A', 0x00,
	0x7d, 0x00, 0x00, 0x00
};
