/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30_devbulk.h
* Author             : WCH, bvernoux
* Version            : V1.1
* Date               : 2022/08/20
* Description        :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef CH56X_USB30_DEVBULK_H_
#define CH56X_USB30_DEVBULK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56x_common.h"

#define CH56X_USB30_DEVBULK_VERSION "1.1"
#define CH56X_USB30_DEVBULK_VERSION_MAJOR 1
#define CH56X_USB30_DEVBULK_VERSION_MINOR 1

/* Global define */
// DEF_ENDP1_OUT_BURST_LEVEL / DEF_ENDP1_IN_BURST_LEVEL maximum burst size 16 defined by the USB3 specification
// Warning USB3 enpoint bulk with 16 burst can be problematic on some PC
#define DEF_ENDP1_OUT_BURST_LEVEL 4
#define DEF_ENDP1_IN_BURST_LEVEL (DEF_ENDP1_OUT_BURST_LEVEL)
#define DEF_ENDP1_MAX_SIZE (DEF_ENDP1_OUT_BURST_LEVEL * 1024)

// DEF_ENDP2_OUT_BURST_LEVEL / DEF_ENDP2_IN_BURST_LEVEL maximum burst size 16 defined by the USB3 specification
// Warning USB3 enpoint bulk with 16 burst can be problematic on some PC
//#define DEF_ENDP2_OUT_BURST_LEVEL 16
#define DEF_ENDP2_OUT_BURST_LEVEL 8
#define DEF_ENDP2_IN_BURST_LEVEL (DEF_ENDP2_OUT_BURST_LEVEL)
#define DEF_ENDP2_MAX_SIZE (DEF_ENDP2_OUT_BURST_LEVEL * 1024)

/* Global Variable */
extern __attribute__ ((aligned(16))) uint8_t endp0RTbuff[512] __attribute__((section(".DMADATA"))); // Endpoint0 Data send/receive buffer
extern __attribute__ ((aligned(16))) uint8_t endp1Tbuff[DEF_ENDP1_MAX_SIZE] __attribute__((section(".DMADATA"))); // Endpoint1 Data send buffer
extern __attribute__ ((aligned(16))) uint8_t endp1Rbuff[DEF_ENDP1_MAX_SIZE] __attribute__((section(".DMADATA"))); // Endpoint1 Data receive buffer
extern __attribute__ ((aligned(16))) uint8_t endp2RTbuff[DEF_ENDP2_MAX_SIZE] __attribute__((section(".DMADATA"))); // Endpoint2 Data send/receive buffer

/* USB Connection Status & USB Type */
#define USB_INT_CONNECT       (0x01) /* USB device connection event detected */
#define USB_INT_DISCONNECT    (0x02) /* USB device disconnect event detected */
#define USB_INT_CONNECT_ENUM  (0x03) /* USB device connected and enumerated */
extern vuint8_t g_DeviceConnectstatus;

#define USB_U30_SPEED         (0x03)
#define USB_U20_SPEED         (0x02)
extern vuint8_t g_DeviceUsbType;

/* Function declaration */
void USB30D_init(FunctionalState sta);

void USB3_force(void);

// For USB Descriptors see CH56x_usb_devbulk_desc_cmd.c & CH56x_usb_devbulk_desc_cmd.h

#ifdef __cplusplus
}
#endif

#endif /* CH56X_USB30_DEVBULK_H_ */
