/******************************************************************************
 * File    : hw_usart.c
 * Author  : Fahad Mirza (Haxiot)
 * Date    : 08-January-2018
 * Brief   : Configuration of the USART instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 Haxiot. All rights reserved.</center></h2>
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
 * 4. Redistribution and use of this software other than as permitted under
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

/*---- Includes ----------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "time_server.h"

/*---- Private variable --------------------------------------------------------*/
static struct
{
  char buffTx[256];      /* structure have to be simplified*/
  char buffRx[256];
  int rx_idx_free;
  int rx_idx_toread;
  HW_LockTypeDef Lock;
  __IO HAL_UART_StateTypeDef gState;
  __IO HAL_UART_StateTypeDef RxState;
} uart_context;

/*---- private function --------------------------------------------------------*/
static void receive(char rx);

/*---- private Global variables ------------------------------------------------*/
static UART_HandleTypeDef hxcUart;


/*---- Function Definitions ----------------------------------------------------*/

bool HW_UART_Modem_Init(uint32_t BaudRate)
{
	hxcUart.Instance        = HXC_USARTX;
	hxcUart.Init.BaudRate   = BaudRate;
	hxcUart.Init.WordLength = UART_WORDLENGTH_8B;
	hxcUart.Init.StopBits   = UART_STOPBITS_1;
	hxcUart.Init.Parity     = UART_PARITY_NONE;
	hxcUart.Init.Mode       = UART_MODE_TX_RX;
	hxcUart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	hxcUart.Init.OverSampling   = UART_OVERSAMPLING_16;
	hxcUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hxcUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	hxcUart.gState = HAL_UART_STATE_RESET;


	if(HAL_UART_Init(&hxcUart) != HAL_OK)
	{
		// Initialization Error
		return false;
	}

    /* Computation of UART mask to apply to RDR register */
    UART_MASK_COMPUTATION(&hxcUart);
	/*******************************************************************/
	/*see Application Note AN4991 : how to wake up an STM32L0 MCU from */
	/*low power mode with the USART or the LPUART                      */
	/*******************************************************************/

    /* Enable the UART Parity Error and Data Register not empty Interrupts */
    SET_BIT(hxcUart.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(hxcUart.Instance->CR3, USART_CR3_EIE);

   /*Enable UART Stop Mode*/
   HAL_UARTEx_EnableStopMode(&hxcUart);

   // Enable wakeup from stop mode active on start bit detection
   MODIFY_REG(hxcUart.Instance->CR3, USART_CR3_WUS, USART_CR3_WUS_1);

   // Enable Wake Up from Stop Mode Interrupt
   SET_BIT(hxcUart.Instance->CR3, USART_CR3_WUFIE);

   hxcUart.ErrorCode = HAL_UART_ERROR_NONE;
   hxcUart.RxState = HAL_UART_STATE_READY;

   return true;
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /***************************************************************/
  /*      GPIO Configuration   for UART2 or for LPUART1          */
  /*      for USART1 use:                                        */
  /*           PA10    ------> USART1_RX                         */
  /*           PA9     ------> USART1_TX                         */
  /*      else for USART2 use:                                   */
  /*           PA2     ------> USART2_TX                         */
  /*           PA3     ------> USART2_RX                         */
  /***************************************************************/

  /* We need both instances. One UART is used for HXC and
   * another one is used for debug printf. As HAL_UART_Init()
   * uses HAL_UART_MspInit(), we needed to cover both instances.
   */
  if(huart->Instance == USART1)
  {
    // Peripheral clock enable
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin   = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(huart->Instance == USART2)
  {
	  // Peripheral clock enable
	  __HAL_RCC_USART2_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  GPIO_InitStruct.Pin   = GPIO_PIN_2|GPIO_PIN_3;
	  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull  = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  if(huart->Instance == HXC_USARTX)
  {
	  // Peripheral interrupt init
	  HAL_NVIC_SetPriority(HXC_USARTX_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(HXC_USARTX_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance == USART1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
  }
  else if(huart->Instance == USART2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
  }

  if(huart->Instance == HXC_USARTX)
  {
	  /* Peripheral interrupt Deinit*/
	  HAL_NVIC_DisableIRQ(HXC_USARTX_IRQn);
  }
}


void HW_UART_Modem_DeInit(void)
{
	HAL_UART_DeInit(&hxcUart);
}


bool HW_UART_Modem_Receive_IT (uint8_t *aRxBuffer)
{
	return ( HAL_UART_Receive_IT(&hxcUart, aRxBuffer, 1) == HAL_OK );
}


/******************************************************************************
  * @brief Handler on Rx IRQ
  * @param handle to the UART
  * @retval void
******************************************************************************/
void HW_UART_Modem_IRQHandler(void)
{
	UART_HandleTypeDef *huart = &hxcUart; //ToDo
	uint32_t isrflags = READ_REG(huart->Instance->ISR);
    uint32_t cr1its   = READ_REG(huart->Instance->CR1);
    uint32_t cr3its   = READ_REG(huart->Instance->CR3);;
    uint32_t errorflags;
    char rxByte = '\0';
    int rx_ready = 0;

    huart->RxState = HAL_UART_STATE_BUSY_RX;

    /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
    if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
    {
    	__HAL_UART_CLEAR_IT(huart, UART_CLEAR_WUF);

        /* forbid stop mode */
	    LPM_SetStopMode(LPM_UART_RX_Id , LPM_Disable );

        /* Enable the UART Data Register not empty Interrupts */
        SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);

        /* Set the UART state ready to be able to start again the process */
        huart->gState  = HAL_UART_STATE_READY;
        huart->RxState = HAL_UART_STATE_READY;
    }

	/* UART in mode Receiver ---------------------------------------------------*/
    if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
		/* Check that a Rx process is ongoing */
		if(huart->RxState == HAL_UART_STATE_BUSY_RX)
		{
		    /*RXNE flag is auto cleared by reading the data*/
			rxByte = (uint8_t)READ_REG(huart->Instance->RDR);

            /* allow stop mode*/
            LPM_SetStopMode(LPM_UART_RX_Id , LPM_Enable );

			huart->RxState = HAL_UART_STATE_READY;
			rx_ready = 1;  /* not used RxTC callback*/
		}
		else
		{
            // Clear RXNE interrupt flag
            __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
            return;
		}
    }

	// If error occurs
    errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
    if (errorflags != RESET)
    {
    	// Error on receiving
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
        // we skip the overrun case
	    rx_ready = 1;
	}

	if(rx_ready)
	{
		// Put received character in the ring buffer
	    receive(rxByte);
	}
}

/******************************************************************************
  * @Brief : Read a complete string until <CR><LF>
  * @Param : rxBuffer - pointer for received characters
  * 		 rxBufferSize - size of the rxBuffer
  * @Retval: Return the number of characters received
******************************************************************************/
uint8_t HW_UART_Modem_GetCharactersUntilNewLine(char *rxBuffer, uint8_t rxBufferSize, uint32_t timeout)
{
	uint8_t len = 0;
	uint32_t currentTime = TimerGetCurrentTime();

	while(len < (rxBufferSize - 1)) // Keep 1 byte for NULL
	{
    	if(HW_UART_Modem_IsNewCharReceived() == false)
        {
    		if(TimerGetElapsedTime(currentTime) > timeout)
    		{
    			break;
    		}
        }
    	else
    	{
    		rxBuffer[len++] = (char)HW_UART_Modem_GetNewChar();
    		if(rxBuffer[len - 1] == '\n')
    		{
    			break;
    		}
    	}
	}

	rxBuffer[len] = '\0';
	return len;
}

/******************************************************************************
  * @brief To check if data has been received
  * @param none
  * @retval false no data / true data
******************************************************************************/
bool HW_UART_Modem_IsNewCharReceived(void)
{
	bool status;
	// ToDo : why STM used separate PRIMASK()?

    //BACKUP_PRIMASK();
    uint32_t primask_bit= __get_PRIMASK();
    //DISABLE_IRQ();
    __disable_irq();

    status = ((uart_context.rx_idx_toread == uart_context.rx_idx_free) ? false : true);

    //RESTORE_PRIMASK();
    __set_PRIMASK(primask_bit);
    return status;
}

/******************************************************************************
  * @brief Get the received character
  * @param none
  * @retval Return the data received
******************************************************************************/
uint8_t HW_UART_Modem_GetNewChar(void)
{
  uint8_t NewChar;

//  BACKUP_PRIMASK();
  uint32_t primask_bit= __get_PRIMASK();
//  DISABLE_IRQ();
  __disable_irq();

  NewChar = uart_context.buffRx[uart_context.rx_idx_toread];
  uart_context.rx_idx_toread = (uart_context.rx_idx_toread + 1) % sizeof(uart_context.buffRx);

//  RESTORE_PRIMASK();
  __set_PRIMASK(primask_bit);
  return NewChar;
}

/******************************************************************************
  * @brief Reset read and write index of the circular buffer
  * @param none
  * @retval none
******************************************************************************/
void HW_UART_ResetBuffer(void)
{
    BACKUP_PRIMASK();
    DISABLE_IRQ();

    uart_context.rx_idx_toread = uart_context.rx_idx_free;

    RESTORE_PRIMASK();
}

/******************************************************************************
  * @Brief : Send bytes
  * @param : ToDo
  * @Retval: True  - if the operation is successful
  * 		 False - if the operation isn't successful
******************************************************************************/
bool HW_UART_Modem_SendBytes(const char *pData, uint16_t size)
{
	if (HAL_UART_Transmit(&hxcUart, (uint8_t *)pData, size, 5000) != HAL_OK)
	{
		return false;
	}

	return true;
}


void HW_UART_Modem_Ready(void)
{
	hxcUart.gState = HAL_UART_STATE_READY;
    hxcUart.RxState = HAL_UART_STATE_READY;
}


/******************************************************************************
  * @brief Store in ring buffer the received character
  * @param none
  * @retval none
******************************************************************************/
static void receive(char rx)
{
  int next_free;

  /** no need to clear the RXNE flag because it is auto cleared by reading the data*/
  uart_context.buffRx[uart_context.rx_idx_free] = rx;
  next_free = (uart_context.rx_idx_free + 1) % sizeof(uart_context.buffRx);
  if (next_free != uart_context.rx_idx_toread)
  {
    /* this is ok to read as there is no buffer overflow in input */
    uart_context.rx_idx_free = next_free;
  }
//  else
//  {
//    /* force the end of a command in case of overflow so that we can process it */
//    uart_context.buffRx[uart_context.rx_idx_free] = '\r';
//    PRINTF("uart_context.buffRx buffer overflow %d\r\n");
//  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Haxiot *****END OF FILE****/

