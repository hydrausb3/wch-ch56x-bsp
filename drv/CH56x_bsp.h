/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_bsp.h
* Author             : bvernoux
* Version            : V1.1.0
* Date               : 2022/08/20
* Description        : This file contains all the functions prototypes for
*                      Board Support Package(BSP) related to Init/Delays/Timebase
*                      DisableInterrupts/EnableInterrupts
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef __CH56x_BSP_H__
#define __CH56x_BSP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56xSFR.h"
#include "CH56x_gpio.h"

extern uint64_t bsp_us_nbcycles;
extern uint64_t bsp_ms_nbcycles;
extern uint32_t bsp_tick_frequency;

/* Memory mapped structure for lowlevel SysTick access */
typedef struct __attribute__((packed))
{
	__IO uint32_t CTLR;
	__IO uint32_t CNT_LSB;
	__IO uint32_t CNT_MSB;
	__IO uint32_t CMP_LSB;
	__IO uint32_t CMP_MSB;
	__IO uint32_t CNTFG;
}
SysTickU32_Type;
#define SysTickU32 ((SysTickU32_Type *)0xE000F000)

/* Used by bsp_sync2boards() */
#define BSP_SYNCHRO_TIMEOUT (12000000) // About 700ms at 120MHz with -O3 / -Os

/* Definition of GPIO PORTA/PORTB Flag used in e_bsp_PortPinType */
#define BSP_PORTA_FLAG (0x00000000)
#define BSP_PORTB_FLAG (0x80000000)
#define BSP_PINBIT_MASK (0x7FFFFFFF)

/**
  * @brief  CH56x Dual board type
  */
typedef enum
{
	BSP_HOST = 0, /* HOST = Primary/main board */
	BSP_DEVICE = 1, /* DEVICE = Secondary board */
} e_bsp_TypeDef;

/* WCH CH56x Port Pin definition */
typedef enum
{
	PA0  = (BSP_PORTA_FLAG | GPIO_Pin_0 ),
	PA1  = (BSP_PORTA_FLAG | GPIO_Pin_1 ),
	PA2  = (BSP_PORTA_FLAG | GPIO_Pin_2 ),
	PA3  = (BSP_PORTA_FLAG | GPIO_Pin_3 ),
	PA4  = (BSP_PORTA_FLAG | GPIO_Pin_4 ),
	PA5  = (BSP_PORTA_FLAG | GPIO_Pin_5 ),
	PA6  = (BSP_PORTA_FLAG | GPIO_Pin_6 ),
	/*
	// Reserved for UART1
	PA7  = (BSP_PORTA_FLAG | GPIO_Pin_7 ),
	PA8  = (BSP_PORTA_FLAG | GPIO_Pin_8 ),
	*/
	PA9  = (BSP_PORTA_FLAG | GPIO_Pin_9 ),
	PA10 = (BSP_PORTA_FLAG | GPIO_Pin_10),
	PA11 = (BSP_PORTA_FLAG | GPIO_Pin_11),
	PA12 = (BSP_PORTA_FLAG | GPIO_Pin_12),
	PA13 = (BSP_PORTA_FLAG | GPIO_Pin_13),
	PA14 = (BSP_PORTA_FLAG | GPIO_Pin_14),
	PA15 = (BSP_PORTA_FLAG | GPIO_Pin_15),
	PA16 = (BSP_PORTA_FLAG | GPIO_Pin_16),
	PA17 = (BSP_PORTA_FLAG | GPIO_Pin_17),
	PA18 = (BSP_PORTA_FLAG | GPIO_Pin_18),
	PA19 = (BSP_PORTA_FLAG | GPIO_Pin_19),
	PA20 = (BSP_PORTA_FLAG | GPIO_Pin_20),
	PA21 = (BSP_PORTA_FLAG | GPIO_Pin_21),
	PA22 = (BSP_PORTA_FLAG | GPIO_Pin_22),
	PA23 = (BSP_PORTA_FLAG | GPIO_Pin_23),

	PB0  = (BSP_PORTB_FLAG | GPIO_Pin_0 ),
	PB1  = (BSP_PORTB_FLAG | GPIO_Pin_1 ),
	PB2  = (BSP_PORTB_FLAG | GPIO_Pin_2 ),
	PB3  = (BSP_PORTB_FLAG | GPIO_Pin_3 ),
	PB4  = (BSP_PORTB_FLAG | GPIO_Pin_4 ),
	PB5  = (BSP_PORTB_FLAG | GPIO_Pin_5 ),
	PB6  = (BSP_PORTB_FLAG | GPIO_Pin_6 ),
	PB7  = (BSP_PORTB_FLAG | GPIO_Pin_7 ),
	PB8  = (BSP_PORTB_FLAG | GPIO_Pin_8 ),
	PB9  = (BSP_PORTB_FLAG | GPIO_Pin_9 ),
	PB10 = (BSP_PORTB_FLAG | GPIO_Pin_10),
	PB11 = (BSP_PORTB_FLAG | GPIO_Pin_11),
	PB12 = (BSP_PORTB_FLAG | GPIO_Pin_12),
	PB13 = (BSP_PORTB_FLAG | GPIO_Pin_13),
	PB14 = (BSP_PORTB_FLAG | GPIO_Pin_14),
	PB15 = (BSP_PORTB_FLAG | GPIO_Pin_15),
	PB16 = (BSP_PORTB_FLAG | GPIO_Pin_16),
	PB17 = (BSP_PORTB_FLAG | GPIO_Pin_17),
	PB18 = (BSP_PORTB_FLAG | GPIO_Pin_18),
	PB19 = (BSP_PORTB_FLAG | GPIO_Pin_19),
	PB20 = (BSP_PORTB_FLAG | GPIO_Pin_20),
	PB21 = (BSP_PORTB_FLAG | GPIO_Pin_21),
	PB22 = (BSP_PORTB_FLAG | GPIO_Pin_22),
	PB23 = (BSP_PORTB_FLAG | GPIO_Pin_23)
} e_bsp_PortPinType;

/*******************************************************************************
* @fn     bsp_init
*
* @brief  Set MCU frequency and SysTick
*
* @return None
**/
void bsp_init(uint32_t systemclck);

/*******************************************************************************
* @fn     bsp_disable_interrupt
*
* @brief  Disable Global Interrupt (Enter Critical Section)
*
* @return None
**/
#define bsp_disable_interrupt() __asm volatile( "csrci mstatus,0x8" ) // Disable interrupt (mie = 0)

/*******************************************************************************
* @fn     bsp_enable_interrupt
*
* @brief  Enable Global Interrupt (Exit Critical Section)
*
* @return None
**/
#define bsp_enable_interrupt() __asm volatile ( "csrsi mstatus,0x8" ) // Enable interrupt (mie = 1)

/*******************************************************************************
* @fn     bsp_get_tick
*
* @brief  Returns the number of system ticks since the system boot
*         This function is thread-safe
*         Precondition: call to bsp_init
*
* @return Returns the number of system ticks since the system boot
**/
uint64_t bsp_get_tick(void);

/*******************************************************************************
* @fn     bsp_wait_us_delay
*
* @brief  Delay, wait for N microseconds
*         This function is thread-safe
*         Precondition: call to bsp_init
*
* @return None
**/
void bsp_wait_us_delay(uint32_t us);

/*******************************************************************************
* @fn     bsp_wait_ms_delay
*
* @brief  Delay, wait for N milliseconds
*         This function is thread-safe
*         Precondition: call to bsp_init
*
* @return None
**/
void bsp_wait_ms_delay(uint32_t ms);

/*******************************************************************************
* @fn     bsp_wait_nb_cycles
*
* @brief  Delay, wait for N MCU cycles
*         This function is thread-safe
*         Precondition: call to bsp_init
*
* @return None
**/
void bsp_wait_nb_cycles(uint32_t nb_cycles);

/*******************************************************************************
* @fn     bsp_get_nbtick_1ms
*
* @brief  Returns the Number of tick for 1 microsecond
*         This function is thread-safe
*         Precondition: call to bsp_init
*
* @return Returns the Number of tick for 1 microsecond
**/
#define  bsp_get_nbtick_1us() (bsp_us_nbcycles)

/*******************************************************************************
* @fn     bsp_get_nbtick_1ms
*
* @brief  Returns the Number of tick for 1 millisecond
*         This function is thread-safe
*         Precondition: call to bsp_init
*
* @return Returns the Number of tick for 1 millisecond
**/
#define bsp_get_nbtick_1ms() (bsp_ms_nbcycles)

/*******************************************************************************
* @fn     bsp_get_tick_frequency
*
* @brief  Returns the tick frequency in Hz
*         This function is thread-safe
*         Precondition: call to bsp_init
*
* @return Returns the tick frequency in Hz
**/
#define bsp_get_tick_frequency() (bsp_tick_frequency)

/*******************************************************************************
* @fn     bsp_get_SysTickCNT_LSB
*
* @brief  Returns SysTick CNT 64bits
*         Potential rollover is managed
*         This function will always return correct 64bits CNT
*         This function is thread-safe
*         Precondition: call to bsp_init
*         Note:
*         CNT value is decremented at each MCU cycle
*         CNT value is one's complement of bsp_get_tick()
*
* @return Returns SysTick CNT 64bits
**/
uint64_t bsp_get_SysTickCNT(void);

/*******************************************************************************
* @fn     bsp_get_SysTickCNT_LSB
*
* @brief  Returns SysTick CNT 32bits
*         Shall be used only safely after startup during <36s
*         This function is thread-safe
*         Precondition: call to bsp_init
*         Note:
*         CNT value is decremented at each MCU cycle
*         CNT value is one's complement of bsp_get_tick() 32bits only
*
* @return Returns SysTick CNT 32bits
**/
#define bsp_get_SysTickCNT_LSB() (SysTickU32->CNT_LSB)

/*******************************************************************************
 * @fn     bsp_gpio_cfg
 *
 * @brief  Initializes GPIO Port Pin mode
 *
 * @param  gpioPortPin - GPIO to configure
 *         mode - GPIO mode
 *
 * @return None
 **/
void bsp_gpio_cfg(e_bsp_PortPinType gpioPortPin, GPIOModeTypeDef mode);

/*******************************************************************************
 * @fn     bsp_gpio_read
 *
 * @brief  Read GPIO PortPin state
 *         Precondition: call to bsp_gpio_cfg()
 *
 * @param  gpioPortPin - GPIO state to read
 *
 * @return 0 (pin low), !0 (pin high)
 **/
int bsp_gpio_read(e_bsp_PortPinType gpioPortPin);

/*******************************************************************************
 * @fn     bsp_gpio_set
 *
 * @brief  Set GPIO PortPin state
 *         Precondition: call to bsp_gpio_cfg()
 *
 * @param  gpioPortPin - GPIO state to set to '1'
 *
 * @return None
 **/
void bsp_gpio_set(e_bsp_PortPinType gpioPortPin);

/*******************************************************************************
 * @fn     bsp_gpio_clr
 *
 * @brief  Read GPIO PortPin state
 *         Precondition: call to bsp_gpio_cfg()
 *
 * @param  gpioPortPin - GPIO state to clear/reset to '0'
 *
 * @return None
 **/
void bsp_gpio_clr(e_bsp_PortPinType gpioPortPin);

/*******************************************************************************
 * @fn     bsp_sync2boards
 *
 * @brief  Synchronize 2x CH56x boards connected together(one on top of another)
 *         Precondition:
 *         - HOST / Main board shall have PB24 not populated
 *         - DEVICE / Secondary board shall have PB24 populate (with Short/Jumper)
 *
 * @param  gpio1 - 1st GPIO to be used for synchronization
 *         gpio2 - 2nd GPIO to be used for synchronization
 *         type - Type of Board
 *              - HOST / Main board shall have PB24 not populated
 *              - DEVICE / Secondary board shall have PB24 populate (with Short/Jumper)
 *
 * @return !=0 if success or 0 in case of error(timeout)
 **/
int bsp_sync2boards(e_bsp_PortPinType gpio1, e_bsp_PortPinType gpio2, e_bsp_TypeDef type);

/*******************************************************************************
 * @fn     bsp_gpio_init
 *
 * @brief  Initializes board GPIO (mainly GPIO)
 *         To be implemented in board/xxx.c (xxx is the board used)
 *
 * @return None
 **/
void bsp_gpio_init(void);

/*******************************************************************************
 * @fn     bsp_ubtn
 *
 * @brief  Read Board state of button UBTN
 *         Precondition: call to bsp_init()
 *         To be implemented in board/xxx.c (xxx is the board used)
 *
 * @return 0 (pin low), !0 (pin high)
 **/
int bsp_ubtn(void);

/*******************************************************************************
 * @fn     bsp_is_gpio_switch_on
 *
 * @brief  Read GPIO switch state
 *         Precondition: call to bsp_init()
 *         To be implemented in board/xxx.c (xxx is the board used)
 *
 * @return 0 (Switch OFF), 1 (Switch ON)
 **/
int bsp_switch(void);

/*******************************************************************************
 * @fn     bsp_uled_on
 *
 * @brief  Set ULED to ON (Light ON)
 *         To be implemented in board/xxx.c (xxx is the board used)
 *
 * @return None
 **/
void bsp_uled_on(void);

/*******************************************************************************
 * @fn     bsp_uled_off
 *
 * @brief  Set ULED to OFF (Light OFF)
 *         To be implemented in board/xxx.c (xxx is the board used)
 *
 * @return None
 **/
void bsp_uled_off(void);

#ifdef __cplusplus
}
#endif

#endif  // __CH56x_BSP_H__	
