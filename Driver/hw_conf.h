/*
  _    _            _____   _______
 | |  | |          |_   _| |__   __|
 | |__| | __ ___  __ | |  ___ | |
 |  __  |/ _` \ \/ / | | / _ \| |
 | |  | | (_| |>  < _| || (_) | |
 |_|  |_|\__,_/_/\_\_____\___/|_|
    (C)2017 HaxIoT

Description: contains hardware configuration Macros and Constants
License: Revised BSD License, see LICENSE.TXT file include in the project
*/
/******************************************************************************
 * File     : hw_conf.h
 * Author   : Fahad Mirza (HaxIoT)
 * Version  : V1.0.0
 * Modified : 13-April-2018
 * Brief    : contains hardware configuration Macros and Constants
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 Haxiot
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY HAXIOT AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONF_H__
#define __HW_CONF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* RTC HW definition ------------------------------------------------------------ */
#define RTC_OUTPUT                       DBG_RTC_OUTPUT
#define RTC_Alarm_IRQn                   RTC_IRQn

/* USART HW definition -----------------------------------------------------------*/
#define HXC_USARTX                       USART1
#define HXC_USARTX_CLK_ENABLE()          __USART1_CLK_ENABLE()
#define HXC_USARTX_RX_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()
#define HXC_USARTX_TX_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

#define HXC_USARTX_FORCE_RESET()         __USART1_FORCE_RESET()
#define HXC_USARTX_RELEASE_RESET()       __USART1_RELEASE_RESET()

#define HXC_USARTX_TX_PIN                GPIO_PIN_9
#define HXC_USARTX_TX_GPIO_PORT          GPIOA
#define HXC_USARTX_TX_AF                 GPIO_AF4_USART1
#define HXC_USARTX_RX_PIN                GPIO_PIN_10
#define HXC_USARTX_RX_GPIO_PORT          GPIOA
#define HXC_USARTX_RX_AF                 GPIO_AF4_USART1

// Definition for USARTx's NVIC
#define HXC_USARTX_IRQn                  USART1_IRQn
#define HXC_USARTX_IRQHandler            USART1_IRQHandler

/* --------------------------- Debug USART HW definition --------------------------*/
// Definition for UARTx clock resources
#define DBG_UARTX                        USART2
#define DBG_UARTX_CLK_ENABLE()           __USART2_CLK_ENABLE()
#define DBG_UARTX_RX_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define DBG_UARTX_TX_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()

#define DBG_UARTX_FORCE_RESET()          __USART2_FORCE_RESET()
#define DBG_UARTX_RELEASE_RESET()        __USART2_RELEASE_RESET()

#define DBG_UARTX_TX_PIN                 GPIO_PIN_2
#define DBG_UARTX_TX_GPIO_PORT           GPIOA
#define DBG_UARTX_TX_AF                  GPIO_AF4_USART2
#define DBG_UARTX_RX_PIN                 GPIO_PIN_3
#define DBG_UARTX_RX_GPIO_PORT           GPIOA
#define DBG_UARTX_RX_AF                  GPIO_AF4_USART2

// Definition for USARTx's NVIC
#define DBG_UARTX_IRQn                   USART2_IRQn
#define DBG_UARTX_IRQHandler             USART2_IRQHandler

/* --------------------------- Reset pin HW definition --------------------------*/
#define HXC_RESET_PORT                   GPIOB
#define HXC_RESET_PIN                    GPIO_PIN_10
#define HXC_RESET_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#ifdef __cplusplus
}
#endif

#endif /* __HW_CONF_H__ */

/************************ (C) COPYRIGHT Haxiot *****END OF FILE****/

