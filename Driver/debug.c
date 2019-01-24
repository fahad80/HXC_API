/******************************************************************************
 * @file    debug.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   debug API
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
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
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
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

/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include "hw.h"
#include "tiny_vsnprintf.h"

static UART_HandleTypeDef debugUart;
static char buffTx[256];

//static bool Debug_UART_Init(void);

/**
  * @brief Initializes the debug
  * @param None
  * @retval None
  */
void DBG_Init( void )
{
#ifdef DEBUG
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
  /* Enable the GPIO_B Clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure the GPIO pin */  
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull   = GPIO_PULLUP;
  gpioinitstruct.Speed  = GPIO_SPEED_HIGH;
  
  gpioinitstruct.Pin    = (GPIO_PIN_12 | GPIO_PIN_13| GPIO_PIN_14 | GPIO_PIN_15);
  HAL_GPIO_Init(GPIOB, &gpioinitstruct);

  /* Reset debug Pins */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
#if 0
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);  
#endif

  __HAL_RCC_DBGMCU_CLK_ENABLE( );

  HAL_DBGMCU_EnableDBGSleepMode( );
  HAL_DBGMCU_EnableDBGStopMode( );
  HAL_DBGMCU_EnableDBGStandbyMode( );
  
#else /* DEBUG */
  /* sw interface off*/
  GPIO_InitTypeDef GPIO_InitStructure = {0};
  
  GPIO_InitStructure.Mode   = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull   = GPIO_NOPULL;
  GPIO_InitStructure.Pin    = (GPIO_PIN_13 | GPIO_PIN_14);
  __GPIOA_CLK_ENABLE();
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  __GPIOA_CLK_DISABLE();
  
  // ToDo : Code below does something and code wont work. Need to investigate more.
  //__HAL_RCC_DBGMCU_CLK_ENABLE( );
  //HAL_DBGMCU_DisableDBGSleepMode( );
  //HAL_DBGMCU_DisableDBGStopMode( );
  //HAL_DBGMCU_DisableDBGStandbyMode( );
  //__HAL_RCC_DBGMCU_CLK_DISABLE( );
#endif

  Debug_UART_Init();
}

void Debug_UART_Init(void)
{
	debugUart.Instance          = DBG_UARTX;
	debugUart.Init.BaudRate     = 9600;
	debugUart.Init.WordLength   = UART_WORDLENGTH_8B;
	debugUart.Init.StopBits     = UART_STOPBITS_1;
	debugUart.Init.Parity       = UART_PARITY_NONE;
	debugUart.Init.Mode	        = UART_MODE_TX_RX;
	debugUart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	debugUart.Init.OverSampling = UART_OVERSAMPLING_16;
	debugUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	debugUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	HAL_UART_Init(&debugUart);
}

void Debug_UART_SendBytes(const char *format, ...)
{
  va_list args;
  static __IO uint16_t len = 0;
  uint16_t current_len;

  va_start(args, format);

  BACKUP_PRIMASK();
  DISABLE_IRQ();
  if (len != 0)
  {
    if (len != sizeof(buffTx))
    {
      current_len = len; /* use current_len instead of volatile len in below computation */
      len = current_len + tiny_vsnprintf_like(buffTx + current_len, sizeof(buffTx) - current_len, format, args);
    }
    RESTORE_PRIMASK();
    va_end(args);
    return;
  }
  else
  {
     len = tiny_vsnprintf_like(buffTx, sizeof(buffTx), format, args);
  }

  current_len = len;
  RESTORE_PRIMASK();
  HAL_UART_Transmit(&debugUart, (uint8_t *)buffTx, current_len, 5000);
  len = 0; // ToDo
  va_end(args);
}

/**
  * @brief Error_Handler
  * @param None
  * @retval None
  */
void Error_Handler( void )
{
  DBG_PRINTF("Error_Handler\n\r");
  while(1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



