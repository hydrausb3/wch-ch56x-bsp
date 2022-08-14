/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_gpio.c
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
 * @fn     GPIOA_ModeCfg
 *
 * @brief  GPIOA port pin mode configuration
 *
 * @param  pin:  PA0-PA23
 *               GPIO_Pin_0 - GPIO_Pin_23
 *         mode:
 *               GPIO_ModeIN_Floating - floating input/high impedance input
 *               GPIO_ModeIN_PU_NSMT - Input with pull-up resistor
 *               GPIO_ModeIN_PD_NSMT - Input with pull-down resistor
 *               GPIO_ModeIN_PU_SMT - Schmitt input with pull-up resistor
 *               GPIO_ModeIN_PD_SMT - Schmitt input with pull-down resistor
 *               GPIO_Slowascent_PP_8mA - low slope push-pull output, drive capability 8mA level
 *               GPIO_Slowascent_PP_16mA - low slope push-pull output, drive capability 16mA level
 *               GPIO_Highspeed_PP_8mA - fast push-pull output, drive capability 8mA level
 *               GPIO_Highspeed_PP_16mA - fast push-pull output, drive capability 16mA level
 *               GPIO_ModeOut_OP_8mA - push-pull output max 8mA
 *               GPIO_ModeOut_OP_16mA - push-pull output max 16mA
 *
 * @return None
 */
void GPIOA_ModeCfg(uint32_t pin, GPIOModeTypeDef mode)
{
	switch(mode)
	{
		case GPIO_ModeIN_Floating:
			R32_PA_PD &= ~pin;
			R32_PA_PU     &= ~pin;
			R32_PA_DIR    &= ~pin;
			break;

		case GPIO_ModeIN_PU_NSMT:
			R32_PA_SMT &=~ pin;
			R32_PA_PD &= ~pin;
			R32_PA_PU     |= pin;
			R32_PA_DIR    &= ~pin;
			break;

		case GPIO_ModeIN_PD_NSMT:
			R32_PA_SMT &=~ pin;
			R32_PA_PD |= pin;
			R32_PA_PU     &= ~pin;
			R32_PA_DIR    &= ~pin;
			break;

		case GPIO_ModeIN_PU_SMT:
			R32_PA_SMT |= pin;
			R32_PA_PD &= ~pin;
			R32_PA_PU     |= pin;
			R32_PA_DIR    &= ~pin;
			break;

		case GPIO_ModeIN_PD_SMT:
			R32_PA_SMT |= pin;
			R32_PA_PD |= pin;
			R32_PA_PU     &= ~pin;
			R32_PA_DIR    &= ~pin;
			break;

		case GPIO_Slowascent_PP_8mA:
			R32_PA_SMT |= pin;
			R32_PA_DRV &= ~pin;
			R32_PA_PD &= ~pin;
			R32_PA_DIR |= pin;
			break;

		case GPIO_Slowascent_PP_16mA:
			R32_PA_SMT |= pin;
			R32_PA_DRV |= pin;
			R32_PA_PD &= ~pin;
			R32_PA_DIR |= pin;
			break;

		case GPIO_Highspeed_PP_8mA:
			R32_PA_SMT &= ~pin;
			R32_PA_DRV &= ~pin;
			R32_PA_PD &= ~pin;
			R32_PA_DIR |= pin;
			break;

		case GPIO_Highspeed_PP_16mA:
			R32_PA_SMT &= ~pin;
			R32_PA_DRV |= pin;
			R32_PA_PD &= ~pin;
			R32_PA_DIR    |= pin;
			break;

		case GPIO_ModeOut_OP_8mA:
			R32_PA_DRV &= ~pin;
			R32_PA_PD  |= pin;
			R32_PA_DIR    |= pin;
			break;

		case GPIO_ModeOut_OP_16mA:
			R32_PA_DRV  |= pin;
			R32_PA_PD  |= pin;
			R32_PA_DIR    |= pin;
			break;

		default:
			break;
	}
}

/*******************************************************************************
 * @fn     GPIOB_ModeCfg
 *
 * @brief  GPIOB port pin mode configuration
 *
 * @param  pin: PB0-PB24
 *              GPIO_Pin_0 - GPIO_Pin_24
 *         mode:
 *              GPIO_ModeIN_Floating - floating input/high impedance input
 *              GPIO_ModeIN_PU_NSMT - Input with pull-up resistor
 *              GPIO_ModeIN_PU_NSMT - Input with pull-down resistor
 *              GPIO_ModeIN_PU_SMT - Schmitt input with pull-up resistor
 *              GPIO_ModeIN_PD_SMT - Schmitt input with pull-down resistor
 *              GPIO_Slowascent_PP_8mA - low slope push-pull output, drive capability 8mA level
 *              GPIO_Slowascent_PP_16mA - low slope push-pull output, drive capability 16mA level
 *              GPIO_Highspeed_PP_8mA - fast push-pull output, drive capability 8mA level
 *              GPIO_Highspeed_PP_16mA - fast push-pull output, drive capability 16mA level
 *              GPIO_ModeOut_OP_8mA - push-pull output max 8mA
 *              GPIO_ModeOut_OP_16mA - push-pull output max 16mA
 * @return None
 */

void GPIOB_ModeCfg(uint32_t pin, GPIOModeTypeDef mode)
{
	switch(mode)
	{
		case GPIO_ModeIN_Floating:
			R32_PB_PD &= ~pin;
			R32_PB_PU     &= ~pin;
			R32_PB_DIR    &= ~pin;
			break;

		case GPIO_ModeIN_PU_NSMT:
			R32_PB_SMT &=~ pin;
			R32_PB_PD &= ~pin;
			R32_PB_PU     |= pin;
			R32_PB_DIR    &= ~pin;
			break;

		case GPIO_ModeIN_PD_NSMT:
			R32_PB_SMT &=~ pin;
			R32_PB_PD |= pin;
			R32_PB_PU     &= ~pin;
			R32_PB_DIR    &= ~pin;
			break;

		case GPIO_ModeIN_PU_SMT:
			R32_PB_SMT |= pin;
			R32_PB_PD &= ~pin;
			R32_PB_PU     |= pin;
			R32_PB_DIR    &= ~pin;
			break;

		case GPIO_ModeIN_PD_SMT:
			R32_PB_SMT |= pin;
			R32_PB_PD |= pin;
			R32_PB_PU     &= ~pin;
			R32_PB_DIR    &= ~pin;
			break;

		case GPIO_Slowascent_PP_8mA:
			R32_PB_SMT |= pin;
			R32_PB_DRV &= ~pin;
			R32_PB_PD &= ~pin;
			R32_PB_DIR |= pin;
			break;

		case GPIO_Slowascent_PP_16mA:
			R32_PB_SMT |= pin;
			R32_PB_DRV |= pin;
			R32_PB_PD &= ~pin;
			R32_PB_DIR |= pin;
			break;

		case GPIO_Highspeed_PP_8mA:
			R32_PB_SMT &= ~pin;
			R32_PB_DRV &= ~pin;
			R32_PB_PD &= ~pin;
			R32_PB_DIR |= pin;
			break;

		case GPIO_Highspeed_PP_16mA:
			R32_PB_SMT &= ~pin;
			R32_PB_DRV |= pin;
			R32_PB_PD &= ~pin;
			R32_PB_DIR    |= pin;
			break;

		case GPIO_ModeOut_OP_8mA:
			R32_PB_DRV &= ~pin;
			R32_PB_PD  |= pin;
			R32_PB_DIR    |= pin;
			break;

		case GPIO_ModeOut_OP_16mA:
			R32_PB_DRV |= pin;
			R32_PB_PD  |= pin;
			R32_PB_DIR    |= pin;
			break;

		default:
			break;
	}
}

/*******************************************************************************
 * @fn     GPIOA_ITModeCfg
 *
 * @brief  GPIOA pin interrupt mode configuration
 *
 * @param  pin:   PA2-PA4
 *         mode:
 *               GPIO_ITMode_LowLevel  - low level trigger
 *               GPIO_ITMode_HighLevel - high level trigger
 *               GPIO_ITMode_FallEdge  - falling edge trigger
 *               GPIO_ITMode_RiseEdge  - rising edge trigger
 *
 * @return   None
 */
void GPIOA_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode)
{
	switch(mode)
	{
		case GPIO_ITMode_LowLevel:
			R32_PA_DIR &= ~pin;
			R8_GPIO_INT_MODE &= ~(pin>>2);
			R8_GPIO_INT_POLAR &= ~(pin>>2);
			break;

		case GPIO_ITMode_HighLevel:
			R32_PA_DIR &= ~pin;
			R8_GPIO_INT_MODE &= ~(pin>>2);
			R8_GPIO_INT_POLAR |= (pin>>2);
			break;

		case GPIO_ITMode_FallEdge:
			R32_PA_DIR &= ~pin;
			R8_GPIO_INT_MODE |= (pin>>2);
			R8_GPIO_INT_POLAR &= ~(pin>>2);
			break;

		case GPIO_ITMode_RiseEdge:
			R32_PA_DIR &= ~pin;
			R8_GPIO_INT_MODE |= (pin>>2);
			R8_GPIO_INT_POLAR |= (pin>>2);
			break;

		default :
			break;
	}
	R8_GPIO_INT_FLAG = (pin>>2);
	R8_GPIO_INT_ENABLE |= (pin>>2);
}

/*******************************************************************************
 * @fn     GPIOB_ITModeCfg
 *
 * @brief  GPIOB pin interrupt mode configuration
 *
 * @param  pin: PB3, PB4, PB11, PB12, PB15
 *         mode:
 *               GPIO_ITMode_LowLevel  - low level trigger
 *               GPIO_ITMode_HighLevel - high level trigger
 *               GPIO_ITMode_FallEdge  - falling edge trigger
 *               GPIO_ITMode_RiseEdge  - rising edge trigger
 *
 * @return   None
 */

void GPIOB_ITModeCfg( uint32_t pin, GPIOITModeTpDef mode )
{
	switch( mode )
	{
		case GPIO_ITMode_LowLevel:
			if(pin==3)
			{
				R32_PB_DIR &= ~(1<<3);
				R8_GPIO_INT_MODE &= ~(1<<3);
				R8_GPIO_INT_POLAR &= ~(1<<3);
				R8_GPIO_INT_FLAG = (1<<3);
				R8_GPIO_INT_ENABLE |= (1<<3);
			}
			else if(pin==4)
			{
				R32_PB_DIR &= ~(1<<4);
				R8_GPIO_INT_MODE &= ~(1<<4);
				R8_GPIO_INT_POLAR &= ~(1<<4);
				R8_GPIO_INT_FLAG = (1<<4);
				R8_GPIO_INT_ENABLE |= (1<<4);
			}
			else if(pin==11)
			{
				R32_PB_DIR &= ~(1<<11);
				R8_GPIO_INT_MODE &= ~(1<<5);
				R8_GPIO_INT_POLAR &= ~(1<<5);
				R8_GPIO_INT_FLAG = (1<<5);
				R8_GPIO_INT_ENABLE |= (1<<5);
			}
			else if(pin==12)
			{
				R32_PB_DIR &= ~(1<<12);
				R8_GPIO_INT_MODE &= ~(1<<6);
				R8_GPIO_INT_POLAR &= ~(1<<6);
				R8_GPIO_INT_FLAG = (1<<6);
				R8_GPIO_INT_ENABLE |= (1<<6);
			}
			else if(pin==15)
			{
				R32_PB_DIR &= ~(1<<15);
				R8_GPIO_INT_MODE &= ~(1<<7);
				R8_GPIO_INT_POLAR &= ~(1<<7);
				R8_GPIO_INT_FLAG = (1<<7);
				R8_GPIO_INT_ENABLE |= (1<<7);
			}
			break;

		case GPIO_ITMode_HighLevel:
			if(pin==3)
			{
				R32_PB_DIR &= ~(1<<3);
				R8_GPIO_INT_MODE &= ~(1<<3);
				R8_GPIO_INT_POLAR |= (1<<3);
				R8_GPIO_INT_FLAG = (1<<3);
				R8_GPIO_INT_ENABLE |= (1<<3);
			}
			else if(pin==4)
			{
				R32_PB_DIR &= ~(1<<4);
				R8_GPIO_INT_MODE &= ~(1<<4);
				R8_GPIO_INT_POLAR |= (1<<4);
				R8_GPIO_INT_FLAG = (1<<4);
				R8_GPIO_INT_ENABLE |= (1<<4);
			}
			else if(pin==11)
			{
				R32_PB_DIR &= ~(1<<11);
				R8_GPIO_INT_MODE &= ~(1<<5);
				R8_GPIO_INT_POLAR |= (1<<5);
				R8_GPIO_INT_FLAG = (1<<5);
				R8_GPIO_INT_ENABLE |= (1<<5);
			}
			else if(pin==12)
			{
				R32_PB_DIR &= ~(1<<12);
				R8_GPIO_INT_MODE &= ~(1<<6);
				R8_GPIO_INT_POLAR |= (1<<6);
				R8_GPIO_INT_FLAG = (1<<6);
				R8_GPIO_INT_ENABLE |= (1<<6);
			}
			else if(pin==15)
			{
				R32_PB_DIR &= ~(1<<15);
				R8_GPIO_INT_MODE &= ~(1<<7);
				R8_GPIO_INT_POLAR |= (1<<7);
				R8_GPIO_INT_FLAG = (1<<7);
				R8_GPIO_INT_ENABLE |= (1<<7);
			}
			break;

		case GPIO_ITMode_FallEdge:
			if(pin==3)
			{
				R32_PB_DIR &= ~(1<<3);
				R8_GPIO_INT_MODE |= (1<<3);
				R8_GPIO_INT_POLAR &= ~(1<<3);
				R8_GPIO_INT_FLAG = (1<<3);
				R8_GPIO_INT_ENABLE |= (1<<3);
			}
			else if(pin==4)
			{
				R32_PB_DIR &= ~(1<<4);
				R8_GPIO_INT_MODE |= (1<<4);
				R8_GPIO_INT_POLAR &= ~(1<<4);
				R8_GPIO_INT_FLAG = (1<<4);
				R8_GPIO_INT_ENABLE |= (1<<4);
			}
			else if(pin==11)
			{
				R32_PB_DIR &= ~(1<<11);
				R8_GPIO_INT_MODE |= (1<<5);
				R8_GPIO_INT_POLAR &= ~(1<<5);
				R8_GPIO_INT_FLAG = (1<<5);
				R8_GPIO_INT_ENABLE |= (1<<5);
			}
			else if(pin==12)
			{
				R32_PB_DIR &= ~(1<<12);
				R8_GPIO_INT_MODE |= (1<<6);
				R8_GPIO_INT_POLAR &= ~(1<<6);
				R8_GPIO_INT_FLAG = (1<<6);
				R8_GPIO_INT_ENABLE |= (1<<6);
			}
			else if(pin==15)
			{
				R32_PB_DIR &= ~(1<<15);
				R8_GPIO_INT_MODE |= (1<<7);
				R8_GPIO_INT_POLAR &= ~(1<<7);
				R8_GPIO_INT_FLAG = (1<<7);
				R8_GPIO_INT_ENABLE |= (1<<7);
			}
			break;


		case GPIO_ITMode_RiseEdge:
			if(pin==3)
			{
				R32_PB_DIR &= ~(1<<3);
				R8_GPIO_INT_MODE |= (1<<3);
				R8_GPIO_INT_POLAR |= (1<<3);
				R8_GPIO_INT_FLAG = (1<<3);
				R8_GPIO_INT_ENABLE |= (1<<3);
			}
			else if(pin==4)
			{
				R32_PB_DIR &= ~(1<<4);
				R8_GPIO_INT_MODE |= (1<<4);
				R8_GPIO_INT_POLAR |= (1<<4);
				R8_GPIO_INT_FLAG = (1<<4);
				R8_GPIO_INT_ENABLE |= (1<<4);
			}
			else if(pin==11)
			{
				R32_PB_DIR &= ~(1<<11);
				R8_GPIO_INT_MODE |= (1<<5);
				R8_GPIO_INT_POLAR |= (1<<5);
				R8_GPIO_INT_FLAG = (1<<5);
				R8_GPIO_INT_ENABLE |= (1<<5);
			}
			else if(pin==12)
			{
				R32_PB_DIR &= ~(1<<12);
				R8_GPIO_INT_MODE |= (1<<6);
				R8_GPIO_INT_POLAR |= (1<<6);
				R8_GPIO_INT_FLAG = (1<<6);
				R8_GPIO_INT_ENABLE |= (1<<6);
			}
			else if(pin==15)
			{
				R32_PB_DIR &= ~(1<<15);
				R8_GPIO_INT_MODE |= (1<<7);
				R8_GPIO_INT_POLAR |= (1<<7);
				R8_GPIO_INT_FLAG = (1<<7);
				R8_GPIO_INT_ENABLE |= (1<<7);
			}
			break;

		default :
			break;
	}
}

/*******************************************************************************
 * @fn     GPIOPinRemap
 *
 * @brief  Peripheral Function Pin Mapping
 *
 * @param  s:
 *           ENABLE  - pin map
 *           DISABLE - default pin
 *         perph:
 *           RB_PIN_UART0 - RXD0 - PB5 -> PA5
 *                        - TXD0 - PB6 -> PA6
 *           RB_PIN_TMR2  - TMR2/PWM6/CAP2 - PA4 -> PB3
 *           RB_PIN_TMR1  - TMR1/PWM5/CAP1 - PB15 -> PB0
 *           RB_PIN_MII   - Ethernet - RMII -> RGMII
 * @return None
 */
void GPIOPinRemap(uint8_t s, uint16_t perph)
{
	if( s ) R8_PIN_ALTERNATE |= perph;
	else R8_PIN_ALTERNATE &= ~perph;
}

/*******************************************************************************
 * @fn     GPIOMco
 *
 * @brief  GPIO MCO Mode configuration
 *
 * @param   s: ENABLE \ DISABLE
 *          mode:
 *                MCO_125,
 *                MCO_25,
 *                MCO_2d5,
 * @return None
 */
void GPIOMco( uint8_t s, uint16_t freq )
{
	if(s)
	{
		R8_SAFE_ACCESS_SIG = 0x57;
		R8_SAFE_ACCESS_SIG = 0xa8;

		R8_CLK_MOD_AUX = 0x10;
		R8_CLK_MOD_AUX |=0x01;
		R8_CLK_MOD_AUX |= freq;
	}
	else R8_CLK_MOD_AUX &= ~(1<<4);
}


