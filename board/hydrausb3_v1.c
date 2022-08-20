/********************************** (C) COPYRIGHT *******************************
* File Name          : hydrausb3_v1.c
* Author             : bvernoux
* Version            : V1.0
* Date               : 2022/08/20
* Description        : This file contains all the functions prototypes for
*                      Board Support Package(BSP) for HydraUSB3 v1 Dev Board
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "hydrausb3_v1.h"

/* ULED PB22 */
#define BSP_ULED_PIN (GPIO_Pin_22)
/* UBTN PB23 */
#define BSP_UBTN_PIN (GPIO_Pin_23)
/* SWITCH PB24 */
#define BSP_SWITCH_PIN (GPIO_Pin_24)

/* All GPIOA except Pin7 & Pin8 used by UART1 */
#define HYDRAUSB3_V1_GPIOA_Pins (GPIO_Pin_All & ~(GPIO_Pin_7 | GPIO_Pin_8) )

/*******************************************************************************
 * @fn     bsp_gpio_init
 *
 * @brief  Initializes board GPIO (mainly GPIO)
 *         Initializes HydraUSB3 board
 *
 * @return None
 **/
void bsp_gpio_init(void)
{
	/* Configure all GPIO to safe state Input Floating */
	GPIOA_ModeCfg(HYDRAUSB3_V1_GPIOA_Pins, GPIO_ModeIN_Floating);
	GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_Floating);

	/* Configure ULED(PB22) */
	GPIOB_ResetBits(BSP_ULED_PIN);
	GPIOB_ModeCfg(BSP_ULED_PIN, GPIO_Highspeed_PP_8mA); // Output

	/* Configure UBTN(PB23) Input Floating */
	GPIOB_ResetBits(BSP_UBTN_PIN);
	GPIOB_ModeCfg(BSP_UBTN_PIN, GPIO_ModeIN_Floating); // Input floating

	/* Configure SWITCH(PB24) Input Pull-up (0=ON, 1=OFF) */
	GPIOB_ResetBits(BSP_SWITCH_PIN);
	GPIOB_ModeCfg(BSP_SWITCH_PIN, GPIO_ModeIN_PU_NSMT); // Pull-up input
}

/*******************************************************************************
 * @fn     bsp_ubtn
 *
 * @brief  Read HydraUSB3 state of button UBTN
 *         Precondition: call to bsp_gpio_init()
 *
 * @return  0 (pin low), !0 (pin high)
 **/
int bsp_ubtn(void)
{
	return GPIOB_ReadPortPin(BSP_UBTN_PIN);
}

/*******************************************************************************
 * @fn     bsp_switch
 *
 * @brief  Read HydraUSB3 PB24 (Switch) GPIO state
 *         Precondition: call to bsp_gpio_init()
 *
 * @return 0 (PB24 Switch OFF), 1 (PB24 Switch ON)
 **/
int bsp_switch(void)
{
	if(GPIOB_ReadPortPin(BSP_SWITCH_PIN) == 0)
	{
		return 1; /* Logic Reversed with PullUp */
	}
	else
	{
		return 0; /* Logic Reversed with PullUp */
	}
}

/*******************************************************************************
 * @fn     bsp_uled_on
 *
 * @brief  Set ULED to ON (Light ON)
 *
 * @return None
 **/
void bsp_uled_on(void)
{
	GPIOB_SetBits(BSP_ULED_PIN);
}

/*******************************************************************************
 * @fn     bsp_uled_off
 *
 * @brief  Set ULED to OFF (Light OFF)
 *
 * @return None
 **/
void bsp_uled_off(void)
{
	GPIOB_ResetBits(BSP_ULED_PIN);
}
