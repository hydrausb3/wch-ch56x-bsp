/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30_devbulk_LIB.h
* Author             : WCH, bvernoux
* Version            : V1.1
* Date               : 2022/08/20
* Description        :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Copyright (c) 2022 Benjamin VERNOUX
*******************************************************************************/
#ifndef CH56X_USB30_DEVBULK_LIB_H_
#define CH56X_USB30_DEVBULK_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56x_common.h"

#define CH56X_USB30_LIB_VERSION "1.1"
#define CH56X_USB30_LIB_VERSION_MAJOR 1
#define CH56X_USB30_LIB_VERSION_MINOR 1

// link CFG
#define TERM_EN    (1<<1)
#define PIPE_RESET (1<<3)
#define LFPS_RX_PD (1<<5)
#define CFG_EQ_EN  (1<<6)
#define DEEMPH_CFG (1<<8)

#define POWER_MODE_0 ((uint32_t)0x00000000)
#define POWER_MODE_1 ((uint32_t)0x00000001)
#define POWER_MODE_2 ((uint32_t)0x00000002)
#define POWER_MODE_3 ((uint32_t)0x00000003)

#define LINK_PRESENT  (1<<0)
#define RX_WARM_RESET ((uint32_t)1<<1)

#define LINK_TXEQ   (1<<6)
#define GO_DISABLED (1<<4)
#define POLLING_EN  (1<<12)

#define TX_HOT_RESET ((uint32_t)1<<16)
#define RX_HOT_RESET ((uint32_t)1<<24)

#define TX_WARM_RESET ((uint32_t)1<<8)
#define TX_Ux_EXIT    ((uint32_t)1<<9)
// link int flag
#define LINK_RDY_FLAG       (1<<0)
#define LINK_RECOV_FLAG     (1<<1)
#define LINK_INACT_FLAG     (1<<2)
#define LINK_DISABLE_FLAG   (1<<3)
#define LINK_GO_U3_FLAG     (1<<4)
#define LINK_GO_U2_FLAG     (1<<5)
#define LINK_GO_U1_FLAG     (1<<6)
#define LINK_GO_U0_FLAG     (1<<7)
#define LINK_U3_WAKE_FLAG   (1<<8)
#define LINK_Ux_REJECT_FLAG (1<<9)
#define TERM_PRESENT_FLAG   (1<<10)
#define LINK_TXEQ_FLAG      (1<<11)
#define LINK_Ux_EXIT_FLAG   (1<<12)
#define WARM_RESET_FLAG     (1<<13)
#define U3_WAKEUP_FLAG      (1<<14)
#define HOT_RESET_FLAG      (1<<15)
#define LINK_RX_DET_FLAG    (1<<20)

#define EP0_R_EN (1<<0)
#define EP1_R_EN (1<<1)
#define EP2_R_EN (1<<2)
#define EP3_R_EN (1<<3)
#define EP4_R_EN (1<<4)
#define EP5_R_EN (1<<5)
#define EP6_R_EN (1<<6)
#define EP7_R_EN (1<<7)

#define EP0_T_EN (1<<8)
#define EP1_T_EN (1<<9)
#define EP2_T_EN (1<<10)
#define EP3_T_EN (1<<11)
#define EP4_T_EN (1<<12)
#define EP5_T_EN (1<<13)
#define EP6_T_EN (1<<14)
#define EP7_T_EN (1<<15)

#define USB_FORCE_RST (1<<2)
#define USB_ALL_CLR   (1<<1)
// LMP
#define LMP_HP            0
#define LMP_SUBTYPE_MASK (0xf<<5)
#define SET_LINK_FUNC    (0x1<<5)
#define U2_INACT_TOUT    (0x2<<5)
#define VENDOR_TEST      (0x3<<5)
#define PORT_CAP         (0x4<<5)
#define PORT_CFG         (0x5<<5)
#define PORT_CFG_RES     (0x6<<5)

#define LINK_SPEED       (1<<9)

#define NUM_HP_BUF       (4<<0)
#define DOWN_STREAM      (1<<16)
#define UP_STREAM        (2<<16)
#define TIE_BRK          (1<<20)

/**********device status***********/
typedef enum _DEVICE_STATE
{
	UNCONNECTED,
	ATTACHED,
	POWERED,
	SUSPENDED,
	ADDRESSED,
	CONFIGURED
} DEVICE_STATE;
/*********************/
typedef union
{
	uint16_t w;
	struct BW
	{
		uint8_t bb1; // low byte
		uint8_t bb0;
	}
	bw;
} UINT16_UINT8;
/**********standard request command***********/
typedef struct __PACKED
{
	uint8_t       bRequestType;
	uint8_t       bRequest;
	UINT16_UINT8 wValue;
	UINT16_UINT8 wIndex;
	uint16_t       wLength;
} *PUSB_SETUP;

#define UsbSetupBuf ((PUSB_SETUP)endp0RTbuff)// endpoint 0
#define ENDP0_MAXPACK 512

// status response
#define NRDY    0
#define ACK     0x01
#define STALL   0x02
#define INVALID 0x03

// number of NUMP
#define NUMP_0 0x00
#define NUMP_1 0x01
#define NUMP_2 0x02
#define NUMP_3 0x03
#define NUMP_4 0x04
#define NUMP_5 0x05
#define NUMP_6 0x06

/* USB endpoint direction */
#define OUT 0x00
#define IN  0x80
/* USB endpoint serial number */
#define ENDP_0 0x00
#define ENDP_1 0x01
#define ENDP_2 0x02
#define ENDP_3 0x03
#define ENDP_4 0x04
#define ENDP_5 0x05
#define ENDP_6 0x06
#define ENDP_7 0x07

#define USB_DESCR_TYP_BOS     0x0f
#define USB_DESCR_UNSUPPORTED 0xffff
#define INVALID_REQ_CODE 0xFF

/* string descriptor type */
#ifndef USB_DESCR_STRING
#define USB_DESCR_LANGID_STRING  0x00
#define USB_DESCR_VENDOR_STRING  0x01
#define USB_DESCR_PRODUCT_STRING 0x02
#define USB_DESCR_SERIAL_STRING  0x03
#define USB_DESCR_OS_STRING      0xee
#endif

#define SS_RX_CONTRL(ep) (&USBSS->UEP0_RX_CTRL)[ep*4]
#define SS_TX_CONTRL(ep) (&USBSS->UEP0_TX_CTRL)[ep*4]

// USB30_Device_Init
//static int USB30_Device_Open(void)

static int USB30_device_init(void)
{
	USBSS->LINK_CFG = 0x140;
	USBSS->LINK_CTRL = 0x12;
	uint32_t t = 0x4c4b41;
	while(USBSS->LINK_STATUS&4)
	{
		t--;
		if(t == 0)
			return -1;
	}
	for(int i = 0; i < 8; i++)
	{
		SS_TX_CONTRL(i) = 0;
		SS_RX_CONTRL(i) = 0;
	}
	USBSS->USB_STATUS = 0x13;

	USBSS->USB_CONTROL = 0x30021;
	USBSS->UEP_CFG = 0;

	USBSS->LINK_CFG |= 2;

	USBSS->LINK_INT_CTRL = 0x10bc7d;

	USBSS->LINK_CTRL = 2;
	return 0;
}

/*
static int USB30_device_close(void)
{
    USB30_Switch_Powermode(POWER_MODE_3);
    USBSS->LINK_CFG = PIPE_RESET | LFPS_RX_PD;
    USBSS->LINK_CTRL = GO_DISABLED | POWER_MODE_3;
    USBSS->LINK_INT_CTRL = 0;
    USBSS->USB_CONTROL = USB_FORCE_RST | USB_ALL_CLR;
    return 0;
}
*/

/*
static inline void USB30_BUS_RESET(void)
{
    USB30_Device_Close();
    mDelaymS(30);
    USB30_Device_Open();
}
*/

/*******************************************************************************
 * @fn      USB30_switch_powermode
 *
 * @brief   Switch USB3.0 power mode
 *
 * @return   None
 */
static inline void USB30_switch_powermode(uint8_t pwr_mode)
{
	while( (USBSS->LINK_STATUS & 4) != 0);
	USBSS->LINK_CTRL &= 0xfffffffc;
	USBSS->LINK_CTRL |= pwr_mode;
}

/*******************************************************************************
 * @fn     USB30_OUT_set
 *
 * @brief  Endpoint receive settings
 *
 * @param  endp - The endpoint number to be set
 *         nump - The number of packets remaining to be received by the endpoint
 *         status -  endpoint status 0-NRDY,1-ACK,2-STALL
 *
 * @return None
 */
static inline void USB30_OUT_set(uint8_t endp, uint8_t status, uint8_t nump)
{
	vuint32_t* p = &USBSS->UEP0_RX_CTRL;
	p+= endp*4;
	*p = *p | ((nump)<<16) | (status << 26);
}

/*******************************************************************************
 * @fn     USB30_device_setaddress
 *
 * @brief  Set device address
 *
 * @param  address - address to be set
 *
 * @return None
 */
static inline void USB30_device_setaddress(uint32_t address)
{
	USBSS->USB_CONTROL &= 0x00ffffff;
	USBSS->USB_CONTROL |= address<<24;
}

/*
static inline uint32_t USB30_device_getaddress(void)
{
  return (USBSS->USB_CONTROL >> 24);
}
*/

/*******************************************************************************
 * @fn     USB30_IN_nump
 *
 * @brief  Get the number of packets remaining to be sent by the endpoint
 *
 * @param  endp - endpoint number
 *
 * @return Number of remaining packets to be sent
 */
static inline uint8_t USB30_IN_nump(uint8_t endp)
{
	vuint32_t* p = &USBSS->UEP0_TX_CTRL;
	p+= endp*4;
	return (((*p)>>16) & 0x1f);
}

/*
static inline void USB30_EP0_IN_set(uint8_t status,uint8_t nump,uint16_t TxLen, uint8_t toggle)
{
    USBSS->UEP0_TX_CTRL |= ((nump<<16) | (status<<26) | (TxLen & 0x3ff) | (toggle << 31));
}
*/

/*******************************************************************************
 * @fn     USB30_IN_set
 *
 * @brief  Endpoint send settings
 *
 * @param  endp - endpoint number
 *         lpf -  end of burst sign   1-enable 0-disable
 *         nump -  The number of packets the endpoint can send
 *         status -  endpoint status   0-NRDY,1-ACK,2-STALL
 *         TxLen - The data length of the last packet sent by the endpoint
 *
 * @return None
 */
static inline void USB30_IN_set(uint8_t endp,FunctionalState lpf,uint8_t status,uint8_t nump,uint16_t TxLen)
{
	vuint32_t* p = &USBSS->UEP0_TX_CTRL;
	p+= endp*4;
	*p = *p | (nump<<16) | (status<<26) | (TxLen & 0x7ff) | (lpf << 28);
}

/*******************************************************************************
 * @fn     USB30_send_ERDY
 *
 * @brief  Endpoint flow control settings Send ERDY packets
 *
 * @param  endp - endpoint number   The highest bit table direction,
  *                                 the lower four bits are the endpoint number
 *         nump -  Number of packets received or sent by the endpoint
 *
 * @return None
 */
static inline void USB30_send_ERDY(uint8_t endp,uint8_t nump)
{
	uint32_t t = endp & 0xf;
	t = (t << 2) | ((uint32_t)nump << 6);
	if( (endp&0x80) == 0)
	{
		USBSS->USB_FC_CTRL = t | (uint32_t)1;
		return;
	}
	else
	{
		USBSS->USB_FC_CTRL = t | (uint32_t)3;
		return;
	}
}

/*******************************************************************************
 * @fn     USB30_OUT_clearIT
 *
 * @brief  Clear the OUT transaction complete interrupt, keep only the packet sequence number
 *
 * @param  endp - The endpoint number to be set
 *
 * @return None
 */
static inline void USB30_OUT_clearIT(uint8_t endp)
{
	vuint32_t* p = &USBSS->UEP0_RX_CTRL;
	p+= endp*4;
	*p &= 0x03e00000;
}

/*******************************************************************************
 * @fn     USB30_IN_clearIT
 *
 * @brief  Clear the IN transaction interrupt and the rest of the endpoint state,
 *         keeping only the packet sequence number
 *
 * @param  endp - endpoint number
 *
 * @return None
 */
static inline void USB30_IN_clearIT(uint8_t endp)
{
	vuint32_t* p = &USBSS->UEP0_TX_CTRL;
	p+= endp*4;
	*p &= 0x03e00000;
}

/*******************************************************************************
 * @fn     USB30_OUT_status
 *
 * @brief  Get endpoint received data length
 *
 * @param  endp - endpoint number
 *         nump - The number of packets remaining to be received by the endpoint
 *         len -  The length of the data received by the endpoint, for burst transfers,
 *                 the packet length of the last packet received by the endpoint
 *         status -  Indicates whether the host still has data packets to be sent,
 *                   1-the end of the burst received a partial packet
  *                  0-the host still has data packets to send
 *
 * @return None
 */
static inline void USB30_OUT_status(uint8_t endp,uint8_t *nump,uint16_t *len,uint8_t *status)
{
	vuint32_t* p = &USBSS->UEP0_RX_CTRL;
	p+= endp*4;
	uint32_t t = *p;
	*len = t;
	*nump = (t >>16) & 31;
	*status = (t>>28) & 7;
}

/*******************************************************************************
 * @fn      USB30_StandardReq
 *
 * @brief  USB Device Mode Standard Request Command Handling
 *
 * @return   The length of data that the host requests the device to send
 */
extern uint16_t USB30_StandardReq();

/*******************************************************************************
 * @fn      USB30_NonStandardReq
 *
 * @brief  USB Device Mode Non-Standard Request Command Handling
 *
 * @return   The length of data that the host requests the device to send
 */
extern uint16_t USB30_NonStandardReq();

/*******************************************************************************
 * @fn      EP0_IN_Callback
 *
 * @brief  endpoint0 IN Transfer completion callback function
 *
 * @return The length of data sent in an IN transfer response
 */
extern uint16_t EP0_IN_Callback();

/*******************************************************************************
 * @fn      EP0_OUT_Callback
 *
 * @brief  endpoint0 OUT Transfer completion callback function
 *
 * @return   None
 */
extern uint16_t EP0_OUT_Callback();

/*******************************************************************************
 * @fn      USB30_Setup_Status
 *
 * @brief   Control Transfer Status Phase
 *
 * @return   None
 */
extern void USB30_Setup_Status();

/*******************************************************************************
 * @fn      USB30_ITP_Callback
 *
 * @brief   ITP callback function
 *
 * @return   None
 */
extern void USB30_ITP_Callback(uint32_t ITPCounter);

/*******************************************************************************
 * @fn      EPn_IN_Callback()
 *
 * @brief   endpoint 1 to 7 IN transaction callback function
 *
 * @return   None
 */
extern void  EP1_IN_Callback(void);
extern void  EP2_IN_Callback(void);
extern void  EP3_IN_Callback(void);
extern void  EP4_IN_Callback(void);
extern void  EP5_IN_Callback(void);
extern void  EP6_IN_Callback(void);
extern void  EP7_IN_Callback(void);

/*******************************************************************************
 * @fn      EPn_IN_Callback()
 *
 * @brief   endpoint 1 to 7 OUT transaction callback function
 *
 * @return   None
 */
extern void  EP1_OUT_Callback(void);
extern void  EP2_OUT_Callback(void);
extern void  EP3_OUT_Callback(void);
extern void  EP4_OUT_Callback(void);
extern void  EP5_OUT_Callback(void);
extern void  EP6_OUT_Callback(void);
extern void  EP7_OUT_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* CH56X_USB30_DEVBULK_LIB_H_ */
