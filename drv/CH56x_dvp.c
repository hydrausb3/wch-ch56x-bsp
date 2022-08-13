/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_dvp.c
* Author             : WCH, bvernoux
* Version            : V1.0.1
* Date               : 2022/08/13
* Description
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"

/*******************************************************************************
 * @fn     DVP_INTCfg
 *
 * @brief  DVP interrupt configuration
 *
 * @param  s: Interrupt control state
				ENABLE  - Enable the corresponding interrupt
				DISABLE - Disable the corresponding interrupt
		   i:  Interrupt type
				RB_DVP_IE_STP_FRM  - End of frame interrupt
				RB_DVP_IE_FIFO_OV  - Receive FIFO overflow interrupt
				RB_DVP_IE_FRM_DONE - End of frame interrupt
				RB_DVP_IE_ROW_DONE - End-of-line(row) break
				RB_DVP_IE_STR_FRM  - Start of frame interrupt
 *
 * @return None
 **/
void DVP_INTCfg( uint8_t s,  uint8_t i )
{
	if(s)
	{
		R8_DVP_INT_EN |= i;
	}
	else
	{
		R8_DVP_INT_EN &= ~i;
	}
}

/*******************************************************************************
 * @fn    DVP_Mode
 *
 * @brief DVP mode
 *
 * @param s:  Data bit width
			RB_DVP_D8_MOD  - 8-bit mode
			RB_DVP_D10_MOD - 10-bit mode
			RB_DVP_D12_MOD - 12-bit mode
		  i:  Compressed Data Mode
			Video_Mode - Enable video mode
			JPEG_Mode  - Enable JPEG mode
 *
 * @return  None
 */
void DVP_Mode( uint8_t s,  DVP_Data_ModeTypeDef i)
{
	R8_DVP_CR0 &= ~RB_DVP_MSK_DAT_MOD; // Restore default mode 8bit mode

	if(s)
	{
		R8_DVP_CR0 |= s;
	}
	else
	{
		R8_DVP_CR0 &= ~(3<<4);
	}

	if(i)
	{
		R8_DVP_CR0 |= RB_DVP_JPEG;
	}
	else
	{
		R8_DVP_CR0 &= ~RB_DVP_JPEG;
	}
}

/*******************************************************************************
 * @fn      DVP_Cfg
 *
 * @brief   DVP configuration
 *
 * @param   s:  DMA enable control
				DVP_DMA_Enable  - DMA enable
				DVP_DMA_Disable - DMA disable
		    i:  Flags and FIFO Clear Control
				DVP_FLAG_FIFO_RESET_Enable  - Enable Reset Flag and FIFO
				DVP_FLAG_FIFO_RESET_Disable - Disable Reset Flag and FIFO
		    j:  Receive logic reset control
				DVP_RX_RESET_Enable - Enable Reset the receive logic circuit
				DVP_RX_RESET_Disable - Disable Reset the receive logic circuit
 *
 * @return   None
 */
void DVP_Cfg( DVP_DMATypeDef s,  DVP_FLAG_FIFO_RESETTypeDef i, DVP_RX_RESETTypeDef j)
{
	switch( s )
	{
		case DVP_DMA_Enable:
			R8_DVP_CR1 |= RB_DVP_DMA_EN;
			break;
		case DVP_DMA_Disable:
			R8_DVP_CR1 &= ~RB_DVP_DMA_EN;
			break;
		default:
			break;
	}

	switch( i )
	{
		case DVP_RX_RESET_Enable:
			R8_DVP_CR1 |= RB_DVP_ALL_CLR;
			break;
		case DVP_RX_RESET_Disable:
			R8_DVP_CR1 &= ~RB_DVP_ALL_CLR;
			break;
		default:
			break;
	}

	switch( j )
	{
		case DVP_RX_RESET_Enable:
			R8_DVP_CR1 |= RB_DVP_RCV_CLR;
			break;
		case DVP_RX_RESET_Disable:
			R8_DVP_CR1 &= ~RB_DVP_RCV_CLR;
			break;
		default:
			break;
	}

}

