/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_ecdc.c
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
 * @fn     ECDC_Init
 *
 * @brief  ECDC Initialization
 *
 * @param  ecdcmode - 0-SM4&ECB 1-AES&ECB 2-SM4&CTR 3-AES&CTR
 *         clkmode  - 1-Disable 2-240M 3-160M
 *         keylen   - 0-128bit 1-192bit 2-256bit
 *         pkey     - Key value pointer
 *         pcount   - Counter value pointer
 *
 * @return None
 */
void ECDC_Init(uint8_t ecdcmode, uint8_t clkmode, uint8_t keylen, puint32_t pkey, puint32_t pcount)
{
	R8_ECDC_INT_FG |= 0xFF;
	R16_ECEC_CTRL = 0;

	R16_ECEC_CTRL |= (ecdcmode&0x03)<<8; // Mode selection
	R16_ECEC_CTRL |= (keylen&0x03)<<10; // Key length setting
	R16_ECEC_CTRL |= (clkmode&0x03)<<4; // Encryption and decryption clock frequency division coefficient, aes encryption and decryption work under 240Mhz
	ECDC_SetKey(pkey, keylen);

	if(R16_ECEC_CTRL & RB_ECDC_CIPHER_MOD) // Only executed in CTR mode, the only difference between CTR and ECB mode programming
		ECDC_SetCount(pcount);

	R8_ECDC_INT_FG |= RB_ECDC_IF_EKDONE;
	R16_ECEC_CTRL |= RB_ECDC_KEYEX_EN;
	R16_ECEC_CTRL &= ~RB_ECDC_KEYEX_EN;

	while(!(R8_ECDC_INT_FG & RB_ECDC_IF_EKDONE));
	R8_ECDC_INT_FG |= RB_ECDC_IF_EKDONE;
}

/*******************************************************************************
 * @fn     ECDC_SetKey
 *
 * @brief  Set key
 *
 * @param  pkey -   Key value pointer
 *         keylen - 0-128bit   1-192bit   2-256bit

 * @return   None
 */
void ECDC_SetKey(puint32_t pkey, uint8_t keylen)
{
	keylen = keylen&0x03;

	R32_ECDC_KEY_31T0 = *pkey++;
	R32_ECDC_KEY_63T32 = *pkey++;
	R32_ECDC_KEY_95T64 = *pkey++;
	R32_ECDC_KEY_127T96 = *pkey++;

	if(keylen)
	{
		R32_ECDC_KEY_159T128 = *pkey++;
		R32_ECDC_KEY_191T160 = *pkey++;
	}
	if(keylen>1)
	{
		R32_ECDC_KEY_223T192 = *pkey++;
		R32_ECDC_KEY_255T224 = *pkey++;
	}
}

/*******************************************************************************
 * @fn     ECDC_SetCount
 *
 * @brief  Set counter
 *
 * @param  pcount -  Counter value pointer
 *
 * @return None
 */
void ECDC_SetCount(puint32_t pcount)
{
	R32_ECDC_IV_31T0 = *pcount++;
	R32_ECDC_IV_63T32 = *pcount++;
	R32_ECDC_IV_95T64 = *pcount++;
	R32_ECDC_IV_127T96 = *pcount++;
}

/*******************************************************************************
 * @fn     ECDC_Excute
 *
 * @brief  Set Endianness and Mode
 *
 * @param  excutemode -	RAMX Encryption - 0x84
 *					    RAMX Decryption - 0x8c
 *						128bits data one-time encryption - 0x02
 *						128bits data single decryption - 0x0a
 *					    Peripheral to RAMX encryption - 0x02
 *				    	Peripheral to RAMX decryption - 0x0a
 *						RAMX to peripheral encryption - 0x04
 *						RAMX to peripheral decryption - 0x0c
 *   	   endianmode - big_endian-1 little_endian-0
 *
 * @return None
 */
void ECDC_Excute(uint8_t excutemode, uint8_t endianmode)
{
	R16_ECEC_CTRL &= 0xDF71;
	R16_ECEC_CTRL |= excutemode;
	if(endianmode)
		R16_ECEC_CTRL |= RB_ECDC_DAT_MOD;
	else
		R16_ECEC_CTRL &= ~RB_ECDC_DAT_MOD;
}

/*******************************************************************************
 * @fn     ECDC_SingleRegister
 *
 * @brief  One-time register encryption and decryption
 *
 * @param  pWdatbuff - Write data first address
 *         pRdatbuff - Read data first address
 *
 * @return   None
 */
void ECDC_SingleRegister(puint32_t pWdatbuff, puint32_t pRdatbuff)
{
	R32_ECDC_SGSD_127T96 = pWdatbuff[3]; // low address
	R32_ECDC_SGSD_95T64 = pWdatbuff[2];
	R32_ECDC_SGSD_63T32 = pWdatbuff[1];
	R32_ECDC_SGSD_31T0 = pWdatbuff[0]; // high address

	while(!(R8_ECDC_INT_FG & RB_ECDC_IF_SINGLE));
	R8_ECDC_INT_FG |= RB_ECDC_IF_SINGLE;

	pRdatbuff[3] = R32_ECDC_SGRT_127T96;
	pRdatbuff[2] = R32_ECDC_SGRT_95T64;
	pRdatbuff[1] = R32_ECDC_SGRT_63T32;
	pRdatbuff[0] = R32_ECDC_SGRT_31T0;
}

/*******************************************************************************
 * @fn     ECDC_RAMX
 *
 * @brief  RAMX Encryption and decryption
 *
 * @param  ram_add - first address
 * 		   ram_len - length
 * @return None
 **/
void ECDC_SelfDMA(uint32_t ram_addr, uint32_t ram_len)
{
	R32_ECDC_SRAM_ADDR = ram_addr;
	R32_ECDC_SRAM_LEN = ram_len; // Start conversion

	while(!(R8_ECDC_INT_FG & RB_ECDC_IF_WRSRAM)); // Completion flag
	R8_ECDC_INT_FG |= RB_ECDC_IF_WRSRAM;
}

/*******************************************************************************
 * @fn     ECDC_RloadCount
 *
 * @brief  In CTR mode, the counter value is reloaded for each encryption/decryption block
 *
 * @param  pcount - Counter value pointer
 *
 * @return None
 */
void ECDC_RloadCount(uint8_t excutemode, uint8_t endianmode, puint32_t pcount)
{
	R16_ECEC_CTRL &= 0xDFF9; // Second position third position 0
	ECDC_SetCount(pcount);
	ECDC_Excute(excutemode, endianmode);
}
