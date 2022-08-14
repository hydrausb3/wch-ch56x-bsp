/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_pwm.c
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
 * @fn     PWMX_CycleCfg
 *
 * @brief  PWM0-PWM3 Reference Clock Configuration
 *
 * @param  cyc: 
               Refer to PWMX_CycleTypeDef
 *
 * @return None
 */
void PWMX_CycleCfg(PWMX_CycleTypeDef cyc)
{
	switch( cyc )
	{
		case PWMX_Cycle_256:
			R8_PWM_CTRL_CFG &= ~RB_PWM_CYCLE_SEL; // PWM configuration control register, clock cycle selection
			break;

		case PWMX_Cycle_255:
			R8_PWM_CTRL_CFG |= RB_PWM_CYCLE_SEL;
			break;

		default :
			break;
	}
}

/*******************************************************************************
 * @fn     PWMX_ACTOUT
 *
 * @brief  PWM0-PWM3 channel output waveform configuration
 *
 * @param  ch: select channel of pwm
 *             Refer to channel of PWM define
 *         da: effective pulse width
 *         pr: select wave polar
 *             Refer to PWMX_PolarTypeDef
 *         s:  Control pwmx function
 *             ENABLE  - output PWM
 *             DISABLE - turn off PWM
 * @return None
 */
void PWMX_ACTOUT( uint8_t ch, uint8_t da, PWMX_PolarTypeDef pr, uint8_t s)
{
	uint8_t i;

	if(s == DISABLE)	R8_PWM_CTRL_MOD &= ~(ch); // Determine whether the PWM is output enabled
	else
	{

		(pr)?(R8_PWM_CTRL_MOD|=(ch<<4)):(R8_PWM_CTRL_MOD&=~(ch<<4)); // PWM output polarity control 1: Default high level, active low; 0: Default low level, active high.
		for(i=0; i<4; i++)
		{
			if((ch>>i)&1)		*((vpuint8_t)((&R8_PWM0_DATA)+i)) = da;
		}
		R8_PWM_CTRL_MOD |= (ch);
	}
}


