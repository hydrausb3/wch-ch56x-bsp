/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb20_devbulk.h
* Author             : WCH, bvernoux
* Version            : V1.1
* Date               : 2022/08/20
* Description        :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef CH56X_USB20_DEVBULK_H_
#define CH56X_USB20_DEVBULK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56x_common.h"

#define CH56X_USB20_VERSION "1.1"
#define CH56X_USB20_VERSION_MAJOR 1
#define CH56X_USB20_VERSION_MINOR 1

/* Global define */
#define U20_MAXPACKET_LEN       512
#define U20_UEP0_MAXSIZE        64
// 00: OUT, 01:SOF, 10:IN, 11:SETUP
#define PID_OUT     0
#define PID_SOF     1
#define PID_IN      2

typedef struct __attribute__((packed))
{
	uint8_t dev_speed;
	uint8_t dev_addr;
	uint8_t dev_config_value;
	uint8_t dev_sleep_status;
	uint8_t dev_enum_status;
}
DevInfo_Typedef;

void USB20_Device_Init(FunctionalState sta);  // USB2 device initial
uint16_t U20_NonStandard_Request();
uint16_t U20_Standard_Request();
uint16_t U20_Endp0_IN_Callback(void);

/* Switch to USB2 even if USB3 is available */
void USB2_force(void);

// For USB Descriptors see CH56x_usbd_desc.c & CH56x_usbd_desc.h

#ifdef __cplusplus
}
#endif

#endif /* CH56X_USB20_DEVBULK_H_ */
