/********************************** (C) COPYRIGHT *******************************
* File Name          : hydrausb3_v1.c
* Author             : bvernoux / hansfbaier
* Version            : V1.0
* Date               : 2022/08/20
* Description        : This file contains all the functions prototypes for
*                      Board Support Package(BSP) for rev2 of this
*                      development board:
* 					   https://github.com/hansfbaier/ch569-usb3-board/
* Copyright (c) 2022 Benjamin VERNOUX
* Copyright (c) 2022 Hans Baier
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "ch569-usb3-board_v1.h"

#define BSP_LED1_PIN (GPIO_Pin_23)
#define BSP_LED2_PIN (GPIO_Pin_24)

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

	GPIOB_ResetBits(BSP_LED1_PIN);
	GPIOB_ModeCfg(BSP_LED1_PIN, GPIO_Highspeed_PP_8mA); // Output

	GPIOB_ResetBits(BSP_LED2_PIN);
	GPIOB_ModeCfg(BSP_LED2_PIN, GPIO_Highspeed_PP_8mA); // Output
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
	return 0;
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
	return 0;
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
	GPIOB_SetBits(BSP_LED1_PIN);
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
	GPIOB_ResetBits(BSP_LED1_PIN);
}
