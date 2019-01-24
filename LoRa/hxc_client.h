/**
  ******************************************************************************
  * @file     hxc_client.h
  * @author   Fahad Mirza (Haxiot)
  * @version  V1.0.0
  * @modified 24-Jan-2019
  * @brief    Header for driver hxc_client.c module
  ******************************************************************************
  * @attention
  *
  *  _    _            _____   _______
  * | |  | |          |_   _| |__   __|
  * | |__| | __ ___  __ | |  ___ | |
  * |  __  |/ _` \ \/ / | | / _ \| |
  * | |  | | (_| |>  < _| || (_) | |
  * |_|  |_|\__,_/_/\_\_____\___/|_|
  *  (C)2017 HaxIoT
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 Haxiot</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Haxiot nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HXC_CLIENT__
#define __HXC_CLIENT__

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include<stdint.h>
#include<stdbool.h>

/* Typedefs ------------------------------------------------------------------*/
/*
 * AT Command statuses, in direct relationship with AT_RetCode static array
 */
typedef enum eAtStatus
{
	AT_OK = 0,
	AT_ERROR,
	AT_PARAM_ERROR,
	AT_BUSY_ERROR,
	AT_PARAM_OVERFLOW,
	AT_INVALID_MODE,
	AT_NO_NET_JOINED,
	AT_PAYLOAD_SIZE_ERROR,
	AT_END_ERROR,
	AT_CMD_ERROR,
	AT_UART_LINK_ERROR,  // Return code to notify error during UART Tx/Rx/Config
	AT_TIMEOUT,
} eAtStatus_t;

/*
 * AT Command Index.
 * In direct relationship with "CmdTab" static array in atcmd.c
 */
typedef enum ATCmd
{
	AT,
	AT_RESET,
	AT_FD,
	AT_DEVEUI,
	AT_DEVADR,
	AT_APPKEY,
	AT_NWKSKEY,
	AT_APPSKEY,
	AT_APPEUI,
	AT_ADR,
	AT_TXP,
	AT_DR,
	AT_DCS,
	AT_PNM,
	AT_RX2WND,
	AT_RX1DL,
	AT_RX2DL,
	AT_JN1DL,
	AT_JN2DL,
	AT_NJM,
	AT_NWKID,
	AT_FCU,
	AT_FCD,
	AT_CLASS,
	AT_CH,
	AT_JOIN,
	AT_NJS,
	AT_SENDB,
	AT_SEND,
	AT_RECVB,
	AT_CFS,
	AT_SNR,
	AT_RSSI,
	AT_MODE,
	AT_RFCFG,
	AT_TXCW,
	AT_TX,
	AT_RX,
	AT_BAT,
	AT_VER,
	AT_END
} ATCmd_t;

// AT command behaviors
typedef enum ATGroup
{
  AT_CTRL = 0,
  AT_SET,
  AT_GET,
  AT_TEST,
}ATGroup_t;

// Type definition for SEND command
typedef struct sSendDataString
{
    char *Buffer;
    uint8_t Port;
    uint8_t Ack;
}sSendDataString_t;

// Type definition for SENDB command
typedef struct sSendDataBinary
{
    uint8_t *Buffer;
    uint8_t DataSize;
    uint8_t Port;
    uint8_t Ack;
}sSendDataBinary_t;

// Type definition for received data
typedef struct sRecvDataBinary
{
    uint8_t *Buffer;
    uint32_t DataSize;
    uint8_t Port;
    uint8_t Ack;
}sRecvDataBinary_t;


/* Exported functions ------------------------------------------------------- */

/********************************************************************
 * @brief  Configures modem UART interface and Reset GPIO
 * @param  None
 * @retval AT_OK in case of success
 * @retval AT_UART_LINK_ERROR in case of failure
*********************************************************************/
eAtStatus_t Modem_Init( void ) ;

/********************************************************************
 * @brief  Deinitialize modem UART interface.
 * @param  None
 * @retval None
*********************************************************************/
void Modem_IO_DeInit( void ) ;

/********************************************************************
 * @brief  Handle the AT cmd following their Group type
 * @param  at_group AT group [control, set , get)
 *         Cmd AT command
 *         pdata pointer to the IN/OUT buffer
 * @retval module status
 ********************************************************************/
eAtStatus_t Modem_AT_Cmd(ATGroup_t at_group, ATCmd_t Cmd, void *pdata);

/********************************************************************
 * @brief  Reset the HXC client modem using HXC RESET pin
 * @param  None
 * @retval module status
 ********************************************************************/
eAtStatus_t Modem_HardReset(void);

/********************************************************************
 * @brief  Check if any downlink packet is received
 * @param  None
 * @retval TRUE or FALSE
 ********************************************************************/
bool Modem_IsNewDataReceived(void);

char* Modem_GetResponseBuffer(void);


#ifdef __cplusplus
}
#endif

#endif /* __HXC_CLIENT__ */

/************************ (C) COPYRIGHT Haxiot *****END OF FILE****/

