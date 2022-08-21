/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb20_devbulk.c
* Author             : WCH, bvernoux
* Version            : V1.1
* Date               : 2022/08/20
* Description        :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_usb_devbulk_desc_cmd.h"

#include "CH56x_debug_log.h"
//#define DEBUG_USB2_REQ 1 // Debug USB Req (can have impact real-time to correctly enumerate USB2)
//#define DEBUG_USB2_EP0 1 // Debug ON EP0 have timing issue with real-time to enumerate USB2
//#define DEBUG_USB2_EPX 1 // EPX is EP1 to EP7

/*
 * This routine is a USB 2.0 device usage routine
 * After the host downloads through endpoint 1, it can upload a package through endpoint 1
 * The host can directly and continuously fetch data from endpoint 2
 */

/* Global define */

/* Global Variable */
vuint16_t  U20_EndpnMaxSize = 512;
vuint16_t  SetupReqLen=0;            //Host request data length
vuint16_t  SetupLen = 0;             //The length of data actually sent or received during the data phase
vuint32_t seq_num = 0;
DevInfo_Typedef  g_devInfo;
static vuint8_t SetupReqType = 0;    //Host request descriptor type
static vuint8_t SetupReq = 0;        //Host request descriptor type
static puint8_t pDescr;
extern vuint8_t link_sta;
static puint8_t pUEP1_TX_data;
static puint8_t pUEP1_RX_data;
vuint32_t EP1_OUT_seq_num = 0;

__attribute__ ((aligned(16))) uint8_t vendor_buff[16]  __attribute__((section(".DMADATA"))); //Endpoint 0 data transceiver buffer
/* Function declaration */

/*******************************************************************************
 * @fn     USB2_force
 *
 * @brief  Switch to USB2 even if USB3 is available
 *
 * @return None
 */
void USB2_force(void)
{
	USBSS->LINK_INT_FLAG = LINK_DISABLE_FLAG;
	USB30D_init(DISABLE);
	PFIC_DisableIRQ(USBSS_IRQn);
	R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
	R8_TMR0_INTER_EN = 0;
	PFIC_DisableIRQ(TMR0_IRQn);
	PFIC_EnableIRQ(USBHS_IRQn);
	USB20_Device_Init(ENABLE);
}

/*******************************************************************************
 * @fn     USB20_Endp_Init
 *
 * @brief  USB2.0 HS endpoint initialization
 *
 * @return None
 */
void USB20_Endp_Init(void)
{
	R8_UEP4_1_MOD = RB_UEP1_RX_EN | RB_UEP1_TX_EN | RB_UEP4_RX_EN | RB_UEP4_TX_EN;
	R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN;
	R8_UEP5_6_MOD = RB_UEP5_RX_EN | RB_UEP5_TX_EN | RB_UEP6_RX_EN | RB_UEP6_TX_EN;
	R8_UEP7_MOD  = RB_UEP7_RX_EN | RB_UEP7_TX_EN; // endpoint send and receive enable

	R16_UEP0_MAX_LEN = 64;
	R16_UEP1_MAX_LEN = 512;
	R16_UEP2_MAX_LEN = 512;

	R32_UEP0_RT_DMA = (uint32_t)(uint8_t *)endp0RTbuff;

	R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
	R32_UEP1_RX_DMA = (uint32_t)(uint8_t *)endp1Rbuff;

	R32_UEP2_TX_DMA = (uint32_t)(uint8_t *)endp2RTbuff;
	R32_UEP2_RX_DMA = (uint32_t)(uint8_t *)endp2RTbuff;

	R16_UEP0_T_LEN = 0;
	R8_UEP0_TX_CTRL = 0;
	R8_UEP0_RX_CTRL = 0;

	R16_UEP1_T_LEN = U20_MAXPACKET_LEN;
	R8_UEP1_TX_CTRL = UEP_T_RES_ACK | RB_UEP_T_TOG_0;
	R8_UEP1_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_0;

	R16_UEP2_T_LEN = U20_MAXPACKET_LEN;
	R8_UEP2_TX_CTRL = UEP_T_RES_ACK | RB_UEP_T_TOG_0;
	R8_UEP2_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_0;

	R16_UEP3_T_LEN = 0;
	R8_UEP3_TX_CTRL = UEP_T_RES_NAK;
	R8_UEP3_RX_CTRL = UEP_R_RES_NAK;

	R16_UEP4_T_LEN = 0;
	R8_UEP4_TX_CTRL = UEP_T_RES_NAK ;
	R8_UEP4_RX_CTRL = UEP_R_RES_NAK;

	R16_UEP5_T_LEN = 0;
	R8_UEP5_TX_CTRL = UEP_T_RES_NAK;
	R8_UEP5_RX_CTRL = UEP_R_RES_NAK;

	R16_UEP6_T_LEN = 0;
	R8_UEP6_TX_CTRL = UEP_T_RES_NAK;
	R8_UEP6_RX_CTRL = UEP_R_RES_NAK;

	R16_UEP7_T_LEN = 0;
	R8_UEP7_TX_CTRL = UEP_T_RES_NAK;
	R8_UEP7_RX_CTRL = UEP_R_RES_NAK;
}
/*******************************************************************************
 * @fn     USB20_Device_Init
 *
 * @brief  USB2.0 device initialization
 *
 * @param  sta -
 *            ENABLE
 *            DISABLE
 *
 * @return None
 */
void USB20_Device_Init(FunctionalState sta)
{
	if(sta)
	{
		R8_USB_CTRL = 0;
		R8_USB_CTRL =  UCST_HS | RB_DEV_PU_EN | RB_USB_INT_BUSY |RB_USB_DMA_EN;
		R8_USB_INT_EN = RB_USB_IE_SETUPACT | RB_USB_IE_TRANS | RB_USB_IE_SUSPEND  |RB_USB_IE_BUSRST ;
		USB20_Endp_Init();
	}
	else
	{
		R8_USB_CTRL = RB_USB_CLR_ALL | RB_USB_RESET_SIE;
	}
}

/*******************************************************************************
 * @fn     USB20_Device_setaddress
 *
 * @brief  USB2.0 set device address (called from USBHS_IRQHandler())
 *
 * @param  address
 *
 * @return None
 **/
void USB20_Device_Setaddress(uint32_t address)
{
	R8_USB_DEV_AD = address; // SET ADDRESS
}
/*******************************************************************************
 * @fn     U20_NonStandard_Request
 *
 * @brief  USB2.0 non standard request.
 *
 * @return None
 */
uint16_t U20_NonStandard_Request(void)
{
	uint16_t len = 0;
	uint8_t endp_dir;
	SetupLen = 0;
	endp_dir = UsbSetupBuf->bRequestType & 0x80;

	switch(SetupReq)
	{
		case 0x01: // Microsoft OS 2.0 Descriptors
			switch(UsbSetupBuf->wIndex.bw.bb1)
			{
				case 0x04:
#if DEBUG_USB2_REQ
					cprintf(" 0104:CompactId\n");
#endif
					SetupLen = ( SetupReqLen > sizeof(USB_CompatId) )? sizeof(USB_CompatId):SetupReqLen;
					pDescr = (puint8_t)USB_CompatId;
					break;
				case 0x05:
#if DEBUG_USB2_REQ
					cprintf(" 0105:PropertyHeader\n");
#endif
					SetupLen = ( SetupReqLen > sizeof(USB_PropertyHeader) )? sizeof(USB_PropertyHeader):SetupReqLen;
					pDescr = (puint8_t)USB_PropertyHeader;
					break;
				case 0x06:
#if DEBUG_USB2_REQ
					cprintf(" 0106:Undefined\n");
#endif
					break;
				case 0x07:
#if DEBUG_USB2_REQ
					cprintf(" 0107:MSOS20DescriptorSet\n");
#endif
					SetupLen = ( SetupReqLen > sizeof(USB_MSOS20DescriptorSet) )? sizeof(USB_MSOS20DescriptorSet):SetupReqLen;
					pDescr = (puint8_t)USB_MSOS20DescriptorSet;
					break;
				case 0x08:
#if DEBUG_USB2_REQ
					cprintf(" 0108:Undefined\n");
#endif
					break;
				case 0x09://
#if DEBUG_USB2_REQ
					cprintf(" 0109:Undefined\n");
#endif
					break;

				case 0xc0:
#if DEBUG_USB2_REQ
					cprintf(" 01C0:Undefined\n");
#endif
					break;
				default:
#if DEBUG_USB2_REQ
					cprintf(" 01D:INVALID_REQ_CODE\n");
#endif
					return USB_DESCR_UNSUPPORTED;
					break;
			}
			break;
		case 0x02: // user-defined command
			switch(UsbSetupBuf->wIndex.bw.bb1)
			{
				/*
				    case 0x05:
				        cprintf(" NS2:PropertyHeader\n");
				        SetupLen = ( SetupReqLen > sizeof(PropertyHeader) )? sizeof(PropertyHeader):SetupReqLen;
				        pDescr = (puint8_t)PropertyHeader;
				        break;
				*/
				default:
#if DEBUG_USB2_REQ
					cprintf(" 02D:INVALID_REQ_CODE\n");
#endif
					return USB_DESCR_UNSUPPORTED;
					break;
			}
			break;
		default:
#if DEBUG_USB2_REQ
			cprintf(" D:INVALID_REQ_CODE\n");
#endif
			SetupLen = USB_DESCR_UNSUPPORTED;
			break;
	}
	/* Determine whether it can be processed normally  */
	if( (SetupLen != USB_DESCR_UNSUPPORTED) && (SetupLen != 0))
	{
		len = ( SetupLen >= U20_UEP0_MAXSIZE ) ? U20_UEP0_MAXSIZE : SetupLen; // Return send length
		if(endp_dir)
		{
			memcpy( endp0RTbuff, pDescr, len ); // Data stage prepares data to buffer for IN
			pDescr += len;
		}
		SetupLen -= len; // Remaining unsent data length
	}
	return len;
}

/*******************************************************************************
 * @fn     U20_Standard_Request
 *
 * @brief  USB2.0 standard request (called from USBHS_IRQHandler())
 *
 * @return None
 */
uint16_t U20_Standard_Request()
{
	uint16_t len = 0;
	uint8_t endp_dir;
	SetupLen = 0;
	endp_dir = UsbSetupBuf->bRequestType & 0x80;
	switch( SetupReq )
	{
		case USB_GET_DESCRIPTOR:
		{
			switch(UsbSetupBuf->wValue.bw.bb0)
			{
				case USB_DESCR_TYP_DEVICE: /* Get device descriptor */
					pDescr = (uint8_t *)USB_HS_DeviceDescriptor;
					SetupLen = ( SetupReqLen > sizeof(USB_HS_DeviceDescriptor) )? sizeof(USB_HS_DeviceDescriptor):SetupReqLen;
					break;
				case USB_DESCR_TYP_CONFIG: /* Get configuration descriptor */
					pDescr = (uint8_t *)USB_HS_ConfigDescriptor;
					SetupLen = ( SetupReqLen > sizeof(USB_HS_ConfigDescriptor) )? sizeof(USB_HS_ConfigDescriptor):SetupReqLen;
					break;
				case USB_DESCR_TYP_BOS: /* Get BOS Descriptor */
					pDescr =(uint8_t *)USB_BOSDescriptor;
					SetupLen = ( SetupReqLen > sizeof(USB_BOSDescriptor) )? sizeof(USB_BOSDescriptor):SetupReqLen;
					break;
				case USB_DESCR_TYP_STRING: /* Get string descriptor */
					switch(UsbSetupBuf->wValue.bw.bb1)
					{
						case USB_DESCR_LANGID_STRING: /* Language string descriptor */
							pDescr = (uint8_t *)USB_StringLangID;
							SetupLen = ( SetupReqLen > sizeof(USB_StringLangID) )? sizeof(USB_StringLangID):SetupReqLen;
							break;
						case USB_DESCR_VENDOR_STRING: /* Manufacturer String Descriptor */
							pDescr = (uint8_t *)USB_StringVendor;
							SetupLen = ( SetupReqLen > sizeof(USB_StringVendor) )? sizeof(USB_StringVendor):SetupReqLen;
							break;
						case USB_DESCR_PRODUCT_STRING: /* Product String Descriptor */
							pDescr =(uint8_t *) USB_HS_StringProduct;
							SetupLen = ( SetupReqLen > sizeof(USB_HS_StringProduct) )? sizeof(USB_HS_StringProduct):SetupReqLen;
							break;
						case USB_DESCR_SERIAL_STRING: /* Serial String Descriptor */
							pDescr = (uint8_t *)USB_StringSerial;
							SetupLen = ( SetupReqLen > sizeof(USB_StringSerial) )? sizeof(USB_StringSerial):SetupReqLen;
							break;
						case USB_DESCR_OS_STRING: /* Microsoft OS String Descriptor */
							pDescr = (uint8_t *)USB_OSStringDescriptor;
							SetupLen = ( SetupReqLen > sizeof(USB_OSStringDescriptor) )? sizeof(USB_OSStringDescriptor):SetupReqLen;
							break;
						default:
							SetupLen = USB_DESCR_UNSUPPORTED;
							break;
					}
					break;
				default :
					SetupLen = USB_DESCR_UNSUPPORTED;
					break;
			}
		}
		break;
		case USB_SET_ADDRESS: /* Set Address */
			g_devInfo.dev_addr = UsbSetupBuf->wValue.bw.bb1;
			break;
		case USB_GET_CONFIGURATION: /* Get configuration value */
			endp0RTbuff[ 0 ] = g_devInfo.dev_config_value;
			SetupLen = 1;
			break;
		case USB_SET_CONFIGURATION: /* Set configuration value */
			if( (R8_USB_SPD_TYPE & RB_USBSPEED_MASK)  == UST_FS )
			{
				U20_EndpnMaxSize = 64;
			}
			else if( (R8_USB_SPD_TYPE & RB_USBSPEED_MASK) == UST_LS )
			{
				U20_EndpnMaxSize = 8;
			}
			g_devInfo.dev_config_value = UsbSetupBuf->wValue.bw.bb1;
			g_devInfo.dev_enum_status = 0x01;
			break;
		case USB_CLEAR_FEATURE: /* Clear features */
			if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP ) /* Clear endpoint */
			{
				switch(UsbSetupBuf->wValue.bw.bb1) /* wIndexL */
				{
					case 0x81: /* endpoint 1 IN */
						R16_UEP1_T_LEN = 0;
						R8_UEP1_TX_CTRL = UEP_T_RES_NAK | RB_UEP_T_TOG_0;
						break;

					case 0x01: /* endpoint 1 OUT */
						R8_UEP1_RX_CTRL = UEP_T_RES_ACK | RB_UEP_R_TOG_0;
						break;

					case 0x82: /* endpoint 2 IN */
						R16_UEP2_T_LEN= 0;
						R8_UEP2_TX_CTRL = UEP_T_RES_NAK | RB_UEP_T_TOG_0;
						break;

					case 0x02: /* endpoint 2 OUT */
						R8_UEP2_TX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_0;
						break;

					default:
#if DEBUG_USB2_REQ
						cprintf(" USB_CLEAR_FEATURE:USB_REQ_RECIP_ENDP UNSUPPORTED UsbSetupBuf->wValue.bw.bb1=0x%02X\n",
								UsbSetupBuf->wValue.bw.bb1);
#endif
						SetupLen = USB_DESCR_UNSUPPORTED;
						break;
				}
			}
			else if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
			{
				if( UsbSetupBuf->wValue.bw.bb1 == 1 ) /* wIndexL */
				{
					g_devInfo.dev_sleep_status &= ~0x01;
				}
			}
			else
			{
#if DEBUG_USB2_REQ
				cprintf(" USB_CLEAR_FEATURE:UNSUPPORTED UsbSetupBuf->bRequestType=0x%02X\n", UsbSetupBuf->bRequestType);
#endif
				SetupLen = USB_DESCR_UNSUPPORTED;
			}
			break;
		case USB_SET_FEATURE: /* Set properties */
			if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 ) /* Set up the device */
			{
				if( UsbSetupBuf->wValue.w == 0x01 )
				{
					if( USB_HS_ConfigDescriptor[7] & 0x20 ) /* Set wakeup enable flag */
					{
						g_devInfo.dev_sleep_status = 0x01;
					}
					else
					{
#if DEBUG_USB2_REQ
						cprintf(" USB_SET_FEATURE:UNSUPPORTED USB_HS_ConfigDescriptor[7]=0x%02X\n",
								USB_HS_ConfigDescriptor[7]);
#endif
						SetupLen = USB_DESCR_UNSUPPORTED;
					}
				}
				else
				{
#if DEBUG_USB2_REQ
					cprintf(" USB_SET_FEATURE:UNSUPPORTED UsbSetupBuf->wValue=0x%04X\n", UsbSetupBuf->wValue.w);
#endif
					SetupLen = USB_DESCR_UNSUPPORTED;
				}
			}
			else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 ) /* Set endpoint */
			{
				if( UsbSetupBuf->wValue.w == 0x00 ) /* Set the specified endpoint STALL */
				{
					switch( ( UsbSetupBuf->wIndex.w ) & 0xff )
					{
						case 0x81: /* Set endpoint 1 IN STALL */
							R8_UEP1_TX_CTRL = ( R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK ) | UEP_T_RES_STALL;
							break;

						case 0x01: /* Set endpoint 1 OUT STALL */
							R8_UEP1_RX_CTRL = ( R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK ) | UEP_R_RES_STALL;
							break;

						case 0x82: /* Set endpoint 2 IN STALL */
							R8_UEP2_TX_CTRL = ( R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK ) | UEP_T_RES_STALL;
							break;

						case 0x02: /* Set endpoint 2 OUT STALL */
							R8_UEP2_RX_CTRL = ( R8_UEP2_RX_CTRL & ~RB_UEP_RRES_MASK ) | UEP_R_RES_STALL;
							break;

						default:
#if DEBUG_USB2_REQ
							cprintf(" USB_SET_FEATURE: UNSUPPORTED Set endpoint X UsbSetupBuf->wIndex=0x%04X\n",  UsbSetupBuf->wIndex.w);
#endif
							SetupLen = USB_DESCR_UNSUPPORTED;
							break;
					}
				}
				else
				{
#if DEBUG_USB2_REQ
					cprintf(" USB_SET_FEATURE: UNSUPPORTED UsbSetupBuf->wValue(!=0)=0x%04X\n", UsbSetupBuf->wValue.w);
#endif
					SetupLen = USB_DESCR_UNSUPPORTED;
				}
			}
			else
			{
#if DEBUG_USB2_REQ
				cprintf(" USB_SET_FEATURE: UNSUPPORTED UsbSetupBuf->bRequestType=0x%02X\n", UsbSetupBuf->bRequestType);
#endif
				SetupLen = USB_DESCR_UNSUPPORTED;
			}
			break;
		case USB_GET_INTERFACE:
#if DEBUG_USB2_REQ
			cprintf(" USB_GET_INTERFACE: wValue=0x%04X wIndex=0x%04X\n", UsbSetupBuf->wValue.w, UsbSetupBuf->wIndex.w);
#endif
			break;
		case USB_SET_INTERFACE:
#if DEBUG_USB2_REQ
			cprintf(" USB_SET_INTERFACE: wValue=0x%04X wIndex=0x%04X\n", UsbSetupBuf->wValue.w, UsbSetupBuf->wIndex.w);
#endif
			break;
		case USB_GET_STATUS: /* Reply according to the actual state of the current endpoint */
			endp0RTbuff[ 0 ] = 0x00;
			endp0RTbuff[ 1 ] = 0x00;
			SetupLen = 2;
			if( UsbSetupBuf->wIndex.w == 0x81 )
			{
				if( ( R8_UEP1_TX_CTRL & RB_UEP_TRES_MASK ) == UEP_T_RES_STALL )
				{
					endp0RTbuff[ 0 ] = 0x01;
					SetupLen = 1;
				}
			}
			else if( UsbSetupBuf->wIndex.w == 0x01 )
			{
				if( ( R8_UEP1_RX_CTRL & RB_UEP_RRES_MASK ) == UEP_R_RES_STALL )
				{
					endp0RTbuff[ 0 ] = 0x01;
					SetupLen = 1;
				}
			}
			else if( UsbSetupBuf->wIndex.w == 0x82 )
			{
				if( ( R8_UEP2_TX_CTRL & RB_UEP_TRES_MASK ) == UEP_T_RES_STALL )
				{
					endp0RTbuff[ 0 ] = 0x01;
					SetupLen = 1;
				}
			}
			else if( UsbSetupBuf->wIndex.w == 0x02 )
			{
				if( ( R8_UEP2_RX_CTRL & RB_UEP_RRES_MASK ) == UEP_R_RES_STALL )
				{
					endp0RTbuff[ 0 ] = 0x01;
					SetupLen = 1;
				}
			}
			break;
		default:
			SetupLen = USB_DESCR_UNSUPPORTED;
			break;
	}
	/* Determine whether it can be processed normally */
	if( (SetupLen != USB_DESCR_UNSUPPORTED) && (SetupLen != 0))
	{
		len = ( SetupLen >= U20_UEP0_MAXSIZE ) ? U20_UEP0_MAXSIZE : SetupLen; // Return send length
		if(endp_dir)
		{
			memcpy( endp0RTbuff, pDescr, len ); // The data stage prepares the data to the buffer for IN
			pDescr += len;
		}
		SetupLen -= len; // Remaining unsent data length
	}
	return len;
}

/*******************************************************************************
 * @fn       USBHS_IRQHandler
 *
 * @brief    USB2.0 HS Interrupt Handler.
 *
 * @return   None
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void USBHS_IRQHandler(void)
{
	uint32_t end_num;
	uint32_t rx_token;
	uint16_t ret_len;
	uint8_t int_flg;
	int_flg = R8_USB_INT_FG;
	if( int_flg & RB_USB_IF_SETUOACT ) // SETUP interrupt
	{
		SetupReqType = UsbSetupBuf->bRequestType;
		SetupReq = UsbSetupBuf->bRequest;
		SetupReqLen = UsbSetupBuf->wLength; // Data length

		/* Analyze host requests */
		if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) // Non-standard request
		{
#if DEBUG_USB2_REQ
			cprintf("NSU2 :");
			uint8_t *p8 = (uint8_t *)endp0RTbuff;
			for(int i = 0; i < 8; i++)
			{
				cprintf("%02x ", *p8++);
			}
			cprintf("\n");
#endif
			ret_len =  U20_NonStandard_Request();
		}
		else // Standard request
		{
#if DEBUG_USB2_REQ
			cprintf("SU2 :");
			uint8_t *p8 = (uint8_t *)endp0RTbuff;
			for(int i=0; i<8; i++)
			{
				cprintf("%02x ", *p8++);
			}
			cprintf("\n");
#endif
			ret_len = U20_Standard_Request();
		}
		if(ret_len == 0xFFFF) // Unsupported descriptor
		{
			R16_UEP0_T_LEN = 0;
			R8_UEP0_TX_CTRL = UEP_T_RES_STALL ;
			R8_UEP0_RX_CTRL = UEP_R_RES_STALL ;
		}
		else
		{
			R16_UEP0_T_LEN = ret_len;
			R8_UEP0_TX_CTRL = UEP_T_RES_ACK | RB_UEP_T_TOG_1; // Data process or state process
			R8_UEP0_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_1;
		}
		R8_USB_INT_FG = RB_USB_IF_SETUOACT; // Clear int flag
	}
	else if( int_flg & RB_USB_IF_TRANSFER ) // Transaction transfer complete interrupt
	{
		end_num =   R8_USB_INT_ST & 0xf; // The endpoint currently generating the interrupt
		rx_token = ( (R8_USB_INT_ST )>>4) & 0x3; // Interrupt Type 00: OUT, 01: SOF, 10: IN, 11: SETUP
#if 0
		if( !(R8_USB_INT_ST & RB_USB_ST_TOGOK) )
		{
			cprintf(" TOG MATCH FAIL : ENDP %x token %x \n", end_num, rx_token);
		}
#endif
		switch( end_num ) // endpoint number
		{
			case 0: /* Endpoint 0 */
#if DEBUG_USB2_EP0
				cprintf("EP0\n");
#endif
				if( rx_token == PID_IN ) // Endpoint Zero Transmit Complete Interrupt
				{
					ret_len = U20_Endp0_IN_Callback();
					if(ret_len == 0) // Data transmission completed
					{
						R8_UEP0_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_1; // The state process is OUT
						R16_UEP0_T_LEN = 0;                               // Clear send length
						R8_UEP0_TX_CTRL = 0;                              // Clear send controller
					}
					else
					{
						R16_UEP0_T_LEN = ret_len;
						R8_UEP0_TX_CTRL ^= RB_UEP_T_TOG_1;
						R8_UEP0_TX_CTRL = ( R8_UEP0_TX_CTRL &~RB_UEP_TRES_MASK )| UEP_T_RES_ACK ;
					}
				}
				else if( rx_token == PID_OUT ) // Endpoint zero receive complete interrupt
				{
					SetupLen -= SetupLen > R16_USB_RX_LEN ? R16_USB_RX_LEN :SetupLen;
					if( SetupLen > 0 ) // There is still data to be received
					{
						R8_UEP0_RX_CTRL ^=RB_UEP_R_TOG_1;
						R8_UEP0_RX_CTRL = ( R8_UEP0_RX_CTRL &~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;

					}
					else // No data received
					{
						R16_UEP0_T_LEN = 0;                               // Status process sends zero-length packets
						R8_UEP0_TX_CTRL = UEP_T_RES_ACK | RB_UEP_T_TOG_1; // State process is IN
						R8_UEP0_RX_CTRL = 0 ;                             // No data received Receive status zero
					}
				}
				break;
			case 1: /* Endpoint 1 */
#if DEBUG_USB2_EPX
				cprintf("EP1\n");
#endif
				if(rx_token == PID_IN)
				{
					//log_printf("EP1 IN\n");
					pUEP1_TX_data += U20_MAXPACKET_LEN;
					memcpy(endp1Tbuff, pUEP1_TX_data, U20_MAXPACKET_LEN);

					// Flip the synchronization trigger bit to be sent next time bulk transfer data0 data1 flip back and forth
					R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
					R8_UEP1_TX_CTRL ^= RB_UEP_T_TOG_1;

					// The endpoint status is set to ACK, if the transmission length remains unchanged,
					// there is no need to rewrite the R16_UEP1_T_LEN register
					R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
				}
				else if(rx_token == PID_OUT)
				{
					if(EP1_OUT_seq_num == 0)
					{
						pUEP1_TX_data = (uint8_t *)endp1Tbuff;
						pUEP1_RX_data = (uint8_t *)endp1Rbuff;
					}
					if(EP1_OUT_seq_num < 7)
					{
						pUEP1_RX_data += U20_MAXPACKET_LEN;
						R32_UEP1_RX_DMA = (uint32_t)(uint8_t *)pUEP1_RX_data;
					}
					EP1_OUT_seq_num++;
					if(EP1_OUT_seq_num == 8) // 4096 Bytes Buffer (8x512 Bytes)
					{
						usb_cmd_rx(USB_TYPE_USB2, endp1Rbuff, endp1Tbuff);
						EP1_OUT_seq_num = 0;
						pUEP1_RX_data = (uint8_t *)endp1Rbuff;
						R32_UEP1_RX_DMA = (uint32_t)(uint8_t *)pUEP1_RX_data;
					}
					R8_UEP1_RX_CTRL ^= RB_UEP_R_TOG_1;
					R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL &~RB_UEP_RRES_MASK)|UEP_R_RES_ACK;
				}
				break;
			case 2: /* Endpoint 2 */
#if DEBUG_USB2_EPX
				cprintf("EP2\n");
#endif
				if(rx_token == PID_IN)
				{
					// Flip the synchronization trigger bit to be sent next time bulk transfer data0 data1 flip back and forth
					R32_UEP2_TX_DMA = (uint32_t)(uint8_t *)endp2RTbuff;
					R8_UEP2_TX_CTRL ^= RB_UEP_T_TOG_1;

					// The endpoint status is set to ACK, if the transmission length remains unchanged,
					// there is no need to rewrite the R16_UEP2_T_LEN register
					R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
				}
				else if(rx_token == PID_OUT)
				{
					R32_UEP2_RX_DMA = (uint32_t)(uint8_t *)endp2RTbuff;
					R8_UEP2_RX_CTRL ^= RB_UEP_R_TOG_1;
					R8_UEP2_RX_CTRL = (R8_UEP2_RX_CTRL &~RB_UEP_RRES_MASK)|UEP_R_RES_ACK;
				}
				break;
			default:
				break;
		}
		R8_USB_INT_FG = RB_USB_IF_TRANSFER; // Clear USB transaction complete interrupt
	}
	else if( int_flg & RB_USB_IF_BUSRST ) // Bus reset interrupt
	{
		g_DeviceConnectstatus = USB_INT_DISCONNECT;
		g_DeviceUsbType = 0;
		USB20_Endp_Init();
		USB20_Device_Setaddress( 0 ); // Bus reset address clear to 0
		R8_USB_INT_FG = RB_USB_IF_BUSRST;
		if( link_sta == 1 )
		{
			PFIC_EnableIRQ(USBSS_IRQn);
			PFIC_EnableIRQ(LINK_IRQn);
			PFIC_EnableIRQ(TMR0_IRQn);
			R8_TMR0_INTER_EN = 1;
			TMR0_TimerInit( 67000000 ); // Set Timer to about 0.5 seconds
			USB30D_init(ENABLE);
		}
	}
	else if( int_flg & RB_USB_IF_SUSPEND ) // Pending interrupt
	{
		R8_USB_INT_FG = RB_USB_IF_SUSPEND;
	}
}

/*******************************************************************************
 * @fn       U20_Endp0_IN_Callback
 *
 * @brief    U20_Endp0_IN_Callback Handler.
 *
 * @return   None
 */
uint16_t U20_Endp0_IN_Callback(void)
{
	uint16_t len = 0;
	switch(SetupReq)
	{
		case USB_GET_DESCRIPTOR:
			len = SetupLen >= U20_UEP0_MAXSIZE ? U20_UEP0_MAXSIZE : SetupLen;
			memcpy(endp0RTbuff, pDescr, len);
			SetupLen -= len;
			pDescr += len;
			break;
		case USB_SET_ADDRESS:
			g_DeviceUsbType = USB_U20_SPEED;
			g_DeviceConnectstatus = USB_INT_CONNECT_ENUM;
			USB20_Device_Setaddress(g_devInfo.dev_addr);
			break;
		default:
			break;
	}
	return len;
}
