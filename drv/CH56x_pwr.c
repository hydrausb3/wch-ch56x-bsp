/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_pwr.c
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
 * @fn     PWR_PeriphClkCfg
 *
 * @brief  Peripheral Clock Control Bits
 * @param  s:
 *           ENABLE  - Turn on peripheral clock
 *           DISABLE - Turn off peripheral clocks
 *         perph:
 *           please refer to Peripher CLK control bit define
 * @return None
 */
void PWR_PeriphClkCfg(uint8_t s, uint16_t perph)
{
	if( s == DISABLE )
	{
		R8_SAFE_ACCESS_SIG = 0x57;
		R8_SAFE_ACCESS_SIG = 0xA8;
		R32_SLEEP_CONTROL |= perph;
	}
	else
	{
		R8_SAFE_ACCESS_SIG = 0x57;
		R8_SAFE_ACCESS_SIG = 0xA8;
		R32_SLEEP_CONTROL &= ~perph;
	}
	R8_SAFE_ACCESS_SIG = 0;
}

/*******************************************************************************
 * @fn     PWR_PeriphWakeUpCfg
 *
 * @brief  Sleep wake source configuration
 *
 * @param  s:
 *           ENABLE  - Turn on the sleep-wake function of this external device
 *           DISABLE - Disable the sleep-wake function of this peripheral
 *         perph:
 *           RB_SLP_USBHS_WAKE - USB2.0 is the wake-up source
 *           RB_SLP_USBSS_WAKE - USB3.0 is the wake-up source
 *           RB_SLP_GPIO_WAKE  - GPIO is the wake-up source
 *           RB_SLP_ETH_WAKE   - ETH is the wake-up source
 *           ALL              -  All of above
 * @return None
 */
void PWR_PeriphWakeUpCfg( uint8_t s, uint16_t perph )
{
	if( s == DISABLE )
	{
		R8_SAFE_ACCESS_SIG = 0x57;
		R8_SAFE_ACCESS_SIG = 0xA8;
		R8_SLP_WAKE_CTRL &= ~perph;
	}
	else
	{
		R8_SAFE_ACCESS_SIG = 0x57;
		R8_SAFE_ACCESS_SIG = 0xA8;
		R8_SLP_WAKE_CTRL |= perph;
	}
	R8_SAFE_ACCESS_SIG = 0;
}

/*******************************************************************************
 * @fn     LowPower_Idle
 *
 * @brief  Low Power - Idle Mode
 *
 * @return None
 */
void LowPower_Idle( void )
{
	PFIC->SCTLR &= ~1<<2; // Set the SleepDeep field of the core PFIC_SCTLR register to 0
	__WFI(); // Execute __WFI() after setting the wake-up condition
}

/*******************************************************************************
 * @fn     LowPower_Halt
 *
 * @brief  Low Power - Halt Mode
 *
 * @return None
 */
void LowPower_Halt(void)
{
	PFIC->SCTLR |= 1<<2;                      // Set the SleepDeep field of the core PFIC_SCTLR register to 1
	R8_SLP_POWER_CTRL |= RB_SLP_USBHS_PWRDN;  // RB_SLP_USBHS_PWRDN is set to 1
	__WFI();                                  // Execute __WFI() after setting the wake-up condition
}

/*******************************************************************************
 * @fn     LowPower_Sleep
 *
 * @brief  Low Power - Sleep Mode
 *
 * @return None
 */
void LowPower_Sleep(void)
{
	PFIC->SCTLR |= 1<<2;                      // Set the SleepDeep field of the core PFIC_SCTLR register to 1
	R8_SLP_POWER_CTRL |= RB_SLP_USBHS_PWRDN;  // RB_SLP_USBHS_PWRDN is set to 1
	R8_SLP_WAKE_CTRL &= ~RB_SLP_USBSS_WAKE;   // RB_SLP_USBSS_WAKE is set to 0
	__WFI();                                  // Execute __WFI() after setting the wake-up condition
}

