/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb_devbulk_desc_cmd.h
* Author             : bvernoux
* Version            : V1.0
* Date               : 2022/08/20
* Description        :
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef USB_DESC_CMD_H_
#define USB_DESC_CMD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56x_common.h"

typedef enum _usb_type
{
	USB_TYPE_USB2,
	USB_TYPE_USB3
} e_usb_type;

/*******************************************************************************
 * @fn      usb_cmd_rx
 *
 * @brief   Callback called by USB2 & USB3 endpoint 1
 *          - For USB3 this usb_cmd_rx() is called from IRQ(USBHS_IRQHandler)
 *            with rx_usb_dma_buff containing 4096 bytes
 *          - For USB2 this usb_cmd_rx() is called from IRQ USB30_IRQHandler->EP1_OUT_Callback)
 *            with rx_usb_dma_buff containing 4096 bytes
 *
 * @param  usb_type: USB Type (USB2 HS or USB3 SS)
 * @param  rx_usb_dma_buff: USB RX DMA buffer containing 4096 bytes of data
 *                          Data received from USB
 * @param  tx_usb_dma_buff: USB TX DMA buffer containing 4096 bytes of data
 *                          Data to be transmitted over USB
 *
 * @return  None
 */
void usb_cmd_rx(e_usb_type usb_type, uint8_t* rx_usb_dma_buff, uint8_t* tx_usb_dma_buff);

typedef union
{
	uint64_t sn_64b;
	uint32_t sn_32b[2]; /* 2*32bits 64bits Unique ID */
	uint8_t sn_8b[8]; /* 8*8bits 64bits Unique ID */
} usb_descriptor_serial_number_t;

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
void usb_descriptor_set_string_serial_number(usb_descriptor_serial_number_t *serial_number);

typedef union
{
	uint16_t id_16b;
	uint8_t id_8b[2];
} usb_id_t;
typedef struct
{
	usb_id_t vid;
	usb_id_t pid;
} usb_descriptor_usb_vid_pid_t;

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
void usb_descriptor_set_usb_vid_pid(usb_descriptor_usb_vid_pid_t *vid_pid);

/*********************/
/* Device Descriptor */
/*********************/
/* USB2.0 High Speed device descriptor */
#define LEN_USB_HS_DeviceDescriptor (18)
extern uint8_t USB_HS_DeviceDescriptor[LEN_USB_HS_DeviceDescriptor];

/* USB3.0 SuperSpeed device descriptor */
#define LEN_USB_SS_DeviceDescriptor (18)
extern uint8_t USB_SS_DeviceDescriptor[LEN_USB_SS_DeviceDescriptor];

/*********************/
/* Config Descriptor */
/*********************/
/* USB2.0 High Speed configuration descriptor */
#define LEN_USB_HS_ConfigDescriptor (46)
extern uint8_t USB_HS_ConfigDescriptor[LEN_USB_HS_ConfigDescriptor];
/* USB3.0 SuperSpeed configuration descriptor */
#define LEN_USB_SS_ConfigDescriptor (70)
extern uint8_t USB_SS_ConfigDescriptor[LEN_USB_SS_ConfigDescriptor];

/********************************/
/* StringLangID /  StringVendor */
/********************************/
/* USB 2.0 / 3.0 String Descriptor Lang ID */
#define LEN_USB_StringLangID (4)
extern uint8_t USB_StringLangID[LEN_USB_StringLangID];
/* USB 2.0 / 3.0 String Vendor */
#define LEN_USB_StringVendor (18)
extern uint8_t USB_StringVendor[LEN_USB_StringVendor];

/*****************/
/* StringProduct */
/*****************/
/* USB 2.0 HS_String Product */
#define LEN_USB_HS_StringProduct (20)
extern uint8_t USB_HS_StringProduct[LEN_USB_HS_StringProduct];
/* USB 3.0 SS_String Product */
#define LEN_USB_SS_StringProduct (20)
extern uint8_t USB_SS_StringProduct[LEN_USB_SS_StringProduct];

/* USB 2.0 / 3.0 String Serial */
#define LEN_USB_StringSerial (60)
extern uint8_t USB_StringSerial[LEN_USB_StringSerial];

/* USB 2.0 / 3.0 OS String Descriptor */
#define LEN_USB_OSStringDescriptor (18)
extern uint8_t USB_OSStringDescriptor[LEN_USB_OSStringDescriptor];

/* USB 2.0 / 3.0 BOS Descriptor */
#define LEN_USB_BOSDescriptor (22)
extern uint8_t USB_BOSDescriptor[LEN_USB_BOSDescriptor];

/***************************/
/* MS OS2.0 Descriptor Set */
/***************************/
/* USB 2.0 / 3.0 MS OS 2.0 Descriptor Set */
#define LEN_USB_MSOS20DescriptorSet (72)
extern uint8_t USB_MSOS20DescriptorSet[LEN_USB_MSOS20DescriptorSet];

/* USB2.0 / 3.0 PropertyHeader (Device GUID) */
#define LEN_USB_PropertyHeader (142)
extern uint8_t USB_PropertyHeader[LEN_USB_PropertyHeader];

/* USB2.0 / 3.0 CompactId (WINUSB) */
#define LEN_USB_CompatId (40)
extern uint8_t USB_CompatId[LEN_USB_CompatId];

#ifdef __cplusplus
}
#endif

#endif /* USB_DESC_CMD_H_ */
