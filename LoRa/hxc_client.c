/**
  ******************************************************************************
  * @file     hxc_client.c
  * @author   Fahad (Haxiot)
  * @version  V1.0.0
  * @date     15-July-2018
  * @brief    This file provides set of firmware functions to communicate
  * 		  with HXC Client using AT commands:
  * 		   - AT_SET
  * 		   - AT_GET
  * 		   - AT_RUN
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

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "hw_usart.h"
#include "utilities.h"
#include "tiny_vsnprintf.h"
#include "hxc_client.h"
#include "hw_conf.h"
#include "delay.h"
#include "time_server.h"
#include "hw_gpio.h"


/* Private macro -------------------------------------------------------------*/
#define AT_VPRINTF(...)         at_cmd_vprintf(__VA_ARGS__)
#define AT_VSSCANF(...)         tiny_sscanf(__VA_ARGS__)

#define BAUD_RATE               9600U
#define ARRAY_SIZE(a)           (sizeof(a) / sizeof(a[0]))

#define AT_RESPONSE_BUFF_SIZE   64U    // Max size of the received buffer.
#define DATA_TX_MAX_BUFF_SIZE   78U    // Max size of the transmit buffer
#define HXC_TIMEOUT             2000U  // 2 seconds


// These strings will be used to compare the responses return from HXC modem.
// In direct relation with sAtRetCode_t
#define OK                 "OK"
#define ERROR              "AT_ERROR"
#define PARAM_ERROR        "AT_PARAM_ERROR"
#define BUSY_ERROR         "AT_BUSY_ERROR"
#define PARAM_OVERFLOW     "AT_PARAM_OVERFLOW"
#define INVALID_MODE       "AT_INVALID_MODE"
#define NO_NETWORK_JOINED  "AT_NO_NETWORK_JOINED"
#define PAYLOAD_SIZE_ERROR "AT_PAYLOAD_SIZE_ERROR"


/* Private typedef -----------------------------------------------------------*/
// Type definition for return code analysis
typedef struct sAtRetCode
{
    char* RetCodeStr;
	uint8_t SizeRetCodeStr;
	eAtStatus_t RetCode;
}sAtRetCode_t;


/* Private functions ---------------------------------------------------------*/
static uint16_t at_cmd_format(ATGroup_t at_group, ATCmd_t Cmd, const void *ptr);
static uint16_t at_set_cmd_format(ATCmd_t Cmd, const void *ptr);
static eAtStatus_t at_cmd_send(uint16_t len);
static eAtStatus_t at_cmd_receive(void *pdata);
static eAtStatus_t at_cmd_analyzeResponse(const char *ReturnResp);
static uint16_t at_cmd_vprintf(const char *format, ...);


/* Private Variables --------------------------------------------------------*/
// NOTE: sizeof of a string take account of the NULL character too, unlike
//       strlen(). Hence subtract one from the sizeof.
static const sAtRetCode_t AT_RetCode[] = {
{OK,                 sizeof(OK) - 1,                 AT_OK},
{ERROR,              sizeof(ERROR) - 1,              AT_ERROR},
{PARAM_ERROR,        sizeof(PARAM_ERROR) - 1,        AT_PARAM_ERROR},
{NO_NETWORK_JOINED,  sizeof(NO_NETWORK_JOINED) - 1,  AT_NO_NET_JOINED},
{BUSY_ERROR,         sizeof(BUSY_ERROR) - 1,         AT_BUSY_ERROR},
{PARAM_OVERFLOW,     sizeof(PARAM_OVERFLOW) - 1,     AT_PARAM_OVERFLOW},
{INVALID_MODE,       sizeof(INVALID_MODE) - 1,       AT_INVALID_MODE},
{PAYLOAD_SIZE_ERROR, sizeof(PAYLOAD_SIZE_ERROR) - 1, AT_PAYLOAD_SIZE_ERROR}};


/*
 * List of AT cmd supported by the HXC Client Module:
 * HXC900 and HXC400
 */
static const char *CmdTab[] =
{
	"",            // AT
	"+RESET",      // Reset modem
	"+FD",         // Factory default
	"+DEVEUI",     // Device identifier
	"+DEVADR",     // Device address
	"+APPKEY",     // Application key
	"+NWKSKEY",    // Network session key
	"+APPSKEY",    // Application session key
	"+APPEUI",     // Application identifier
	"+ADR",        // Adaptive data rate
	"+TXP",        // Transmit power
	"+DR",         // Data rate
	"+DCS",        // DCS duty cycle settings
	"+PNM",        // Public/Private network
	"+RX2WND",     // Rx2 window frequency and datarate
	"+RX1DL",      // Delay of the Rx1 window
	"+RX2DL",      // Delay of the Rx2 window
	"+JN1DL",      // Join delay on Rx1 window
	"+JN2DL",      // Join delay on Rx2 window
	"+NJM",        // Network join mode
	"+NWKID",      // Network ID
	"+FCU",        // Uplink frame counter
	"+FCD",        // Downlink frame counter
	"+CLASS",      // LoRa class
	"+CH",         // Channel configuration
	"+JOIN",       // Join network server
	"+NJS",        // Network join mode
	"+SENDB",      // Send binary formatted data
	"+SEND",       // Send data in ASCII format
	"+RECVB",      // Get the last received data
	"+CFS",        // Confirm status
	"+SNR",        // Signal to noise ratio
	"+RSSI",       // Signal strength indicator on received radio signal
	"+MODE",       // Modem mode
	"+RFCFG",      // LoRa only configuration
	"+TXCW",       // Continuous Tx
	"+TX",         // Send LoRa only data in raw format
	"+RX",         // Continuous Rx
	"+BAT",        // Battery level
	"+VER",        // Firmware version of the HXC Client
};


static char AtCmdBuff[DATA_TX_MAX_BUFF_SIZE];
// Write position needed for AtCmdBuff[] during AT_SET
static uint16_t Offset = 0;

// Has to be the largest of the response e.g. APPKEY.
static char AtResponseBuff[AT_RESPONSE_BUFF_SIZE];


/* Exported functions ------------------------------------------------------- */

/*******************************************************************************
 * @Brief  : Configures HXC UART interface, Reset Pin
 * @Param  : None
 * @Return : AT_OK in case of success
 *           AT_UART_LINK_ERROR in case of UART init failure
*******************************************************************************/
eAtStatus_t Modem_Init( void )
{
	if (HW_UART_Modem_Init(BAUD_RATE) == false)
    {
	   return AT_UART_LINK_ERROR;
    }

	// Reset pin initialization
	HXC_RESET_GPIO_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HW_GPIO_Init(HXC_RESET_PORT, HXC_RESET_PIN, &GPIO_InitStruct);

	return AT_OK;
}


/*******************************************************************************
 * @Brief  : Deinitialize modem UART interface.
 * @Param  : None
 * @Return : None
*******************************************************************************/
void Modem_IO_DeInit( void )
{
	HW_UART_Modem_DeInit();
}


/*******************************************************************************
 * @Brief  : Handle the AT cmd following their Group type
 * @Param  : at_group AT group (control, set , get)
 *           Cmd AT command
 *           pdata pointer to the IN/OUT buffer
 * @Return : Module status
 ******************************************************************************/
eAtStatus_t  Modem_AT_Cmd(ATGroup_t at_group, ATCmd_t atCmd, void *pdata )
{
	eAtStatus_t atStatus = AT_END_ERROR;
	uint16_t atCmdLen;

	// Reset At_cmd buffer for each transmission
	memset1((uint8_t *)AtCmdBuff, 0x00, sizeof(AtCmdBuff));
	
	// Reset the UART circular buffer for each transmission to make sure
	// the responses we will get are for the current AT cmd.
	HW_UART_ResetBuffer();

	// Format AT cmd
	atCmdLen = at_cmd_format(at_group, atCmd, pdata);
	if(atCmdLen == 0)
	{
		// You are trying to use a command behavior that HXC Client doesn't
		// support. Check manual for possible AT command behaviors.
		return AT_CMD_ERROR;
	}

	// Send AT cmd string
	if(at_cmd_send(atCmdLen) != AT_OK)
	{
	    return AT_UART_LINK_ERROR;
	}

	// Read response from HXC client for the AT cmd
	if (at_group == AT_GET)
	{
		// Get the value
		atStatus = at_cmd_receive(pdata);
	}
	else
	{
		// Check for the return status
		atStatus = at_cmd_receive(NULL);
	}

    return atStatus;
}


/*******************************************************************************
 * @Brief : Format the cmd in order to be send
 * @Param : at_group - the behavior of AT cmd
 *          Cmd - AT command
 *          ptr - generic pointer to the IN/OUT buffer
 * @Return: Length of the formated frame to be send
 ******************************************************************************/
static uint16_t at_cmd_format(ATGroup_t at_group, ATCmd_t Cmd, const void *ptr)
{
	uint16_t len = 0;  /*length of the formated command*/

	switch (at_group)
	{
		case AT_CTRL:
		{
			len = AT_VPRINTF("AT%s\r\n", CmdTab[Cmd]);
			break;
		}
		case AT_GET:
		{
			len = AT_VPRINTF("AT%s=?\r\n", CmdTab[Cmd]);
			break;
		}
		case AT_SET:
		{
			len = at_set_cmd_format(Cmd, ptr);
			break;
		}
		default:
		{
			break;
		}
	}

   return len;
}


/*******************************************************************************
 * @Brief : Format the at set cmd
 * @Param : Cmd - AT command
 *          ptr - generic pointer to the IN/OUT buffer
 * @Return: Length of the formated frame to be send
 ******************************************************************************/
static uint16_t at_set_cmd_format(ATCmd_t Cmd, const void *ptr)
{
	uint32_t value;    /*for 32_02X and 32_D*/


	Offset = AT_VPRINTF("AT%s=", CmdTab[Cmd]);
	switch (Cmd)
	{
		case  AT_SEND:
		{
			sSendDataString_t *SendData = (sSendDataString_t *)ptr;
			Offset += AT_VPRINTF("%d,%d:%s", SendData->Ack, SendData->Port, SendData->Buffer);
			break;
		}
		case  AT_SENDB:
		{
			sSendDataBinary_t *SendData = (sSendDataBinary_t *)ptr;

			Offset += AT_VPRINTF("%d,%d:", SendData->Ack, SendData->Port);
			unsigned i;
			for (i = 0; i < SendData->DataSize; i++)
			{
				Offset+=AT_VPRINTF("%02x", SendData->Buffer[i]);
			}

			break;
		}
		case AT_APPKEY:
		case AT_NWKSKEY:
		case AT_APPSKEY:
		case AT_DEVADR:
		case AT_APPEUI:
		case AT_DEVEUI:
		{
			char *key = (char*) ptr;
			Offset += AT_VPRINTF("%s", key);
			
			break;
		}
		case  AT_RX1DL:
		case  AT_RX2DL:
		case  AT_JN1DL:
		case  AT_JN2DL:
		case  AT_FCU:
		case  AT_FCD:
		{
			value = *(uint32_t*)ptr;
			Offset += AT_VPRINTF("%u", value);
			break;
		}
		case  AT_DR:
		case  AT_TXP:
		case  AT_PNM:
		case  AT_DCS:
		case  AT_ADR:
		case  AT_BAT:
		{
			uint8_t value_8 =  *(uint8_t*)ptr;
			Offset += AT_VPRINTF("%d", value_8);
			break;
		}
		case  AT_CLASS:
		{
			char value_c =  *(char*)ptr;
			Offset += AT_VPRINTF("%c", value_c);
			break;
		}
		case  AT_NJM:
		{
			Offset += AT_VPRINTF("%s", (char*)ptr);
			break;
		}
		default:
		{
			//DBG_PRINTF ("Not supported\r\n");
			break;
		}
	}

	Offset += AT_VPRINTF("\r\n");
	uint16_t len = Offset;
	Offset = 0;

	return len;
}


/*******************************************************************************
  * @Brief : This function sends an AT cmd to the slave device
  * @Param : len - length of the AT cmd to be sent
  * @Return: eAtStatus_t return code
*******************************************************************************/
static eAtStatus_t at_cmd_send(uint16_t len)
{
  /*transmit the command from master to slave*/
  if( HW_UART_Modem_SendBytes(AtCmdBuff, len) == false)
  {
	  return AT_UART_LINK_ERROR;
  }

  return AT_OK;
}


/*******************************************************************************
  * @Brief : This function receives response from the slave device
  * @Param : pdata - pointer to the value returned by the slave
  * @Return: Return code coming from HXC slave
*******************************************************************************/
static eAtStatus_t at_cmd_receive(void *pdata)
{
	bool ResponseComplete = false;
	uint8_t i = 0;
	eAtStatus_t RetCode = AT_END_ERROR;
  
	// Cleanup the response buffer
	memset1((uint8_t *)AtResponseBuff, 0x00, AT_RESPONSE_BUFF_SIZE);

	uint32_t currentTime = TimerGetCurrentTime();
    while(ResponseComplete != true)
    {
    	if(HW_UART_Modem_IsNewCharReceived() == false)
        {
    		if(TimerGetElapsedTime(currentTime) > HXC_TIMEOUT)
    		{
    			ResponseComplete = true;
    			RetCode = AT_TIMEOUT;
    		}
        }
    	else
    	{
    	    AtResponseBuff[i++] = HW_UART_Modem_GetNewChar();
   
            // Wait up to line feed marker
            if (AtResponseBuff[i - 1] == '\n')
            {
            	// Last two bytes are <CR><LF>, set CR as NULL byte.
                AtResponseBuff[i - 2] = '\0';
    	        i = 0;

		        RetCode = at_cmd_analyzeResponse(AtResponseBuff);
		        if(RetCode != AT_END_ERROR)
		        {
			        ResponseComplete = true;
		        }
                else if(pdata != NULL)
                {
        	        // If pdata isn't null that means we are using GET cmd to get
        	        // return value. Copy the return value into pdata.

			        strcpy(pdata, AtResponseBuff);
			        memset1((uint8_t *)AtResponseBuff, 0x00, sizeof(AtResponseBuff));
			        // Now, let's get the status
		        }
            }
            else
            {
                if (i == (AT_RESPONSE_BUFF_SIZE - 1))
                {
        	        // Frame overflow. Reset index and stop reading.
                	i = 0;
                	RetCode = AT_PARAM_OVERFLOW;
                	ResponseComplete = true;
                }
            }
    	}
    } // End while(ResponseComplete != true)

    HW_UART_Modem_Ready();          
    return RetCode;
}


/*******************************************************************************
  * @Brief : Analyze the response received by the device
  * @Param : response: pointer to the received response
  * @Return: eAtStatus_t error type
*******************************************************************************/
static eAtStatus_t at_cmd_analyzeResponse(const char *ReturnResp)
{
	uint8_t i;

    for (i = 0; i < ARRAY_SIZE(AT_RetCode); i++)
    {   
		if (strncmp(ReturnResp, AT_RetCode[i].RetCodeStr, (AT_RetCode[i].SizeRetCodeStr)) == 0)
		{
			/* Command has been found */
			return AT_RetCode[i].RetCode;
		}
    }
    return AT_END_ERROR;
}


/*******************************************************************************
  * @Brief : Format the AT frame to be sent to the modem (slave)
  * @Param : Pointer to the format string
  * @Return: Length of the string to be sent
*******************************************************************************/
static uint16_t at_cmd_vprintf(const char *format, ...)
{
	va_list args;
    uint16_t len;
   
    va_start(args, format);
  
    len = tiny_vsnprintf_like(AtCmdBuff + Offset, DATA_TX_MAX_BUFF_SIZE - Offset, format, args);
  
    va_end(args);
  
    return len;
}

/*******************************************************************************
 * @Brief  : Reset the HXC client modem using HXC RESET pin
 * @Param  : None
 * @Return : Module status
 ******************************************************************************/
eAtStatus_t Modem_HardReset(void)
{
	HW_GPIO_Write(HXC_RESET_PORT, HXC_RESET_PIN, GPIO_PIN_RESET);
	DelayMs(200);
	HW_GPIO_Write(HXC_RESET_PORT, HXC_RESET_PIN, GPIO_PIN_SET);

	// Check for the return status - OK
	return at_cmd_receive(NULL);
}

/*******************************************************************************
 * @Brief : Check if any downlink packet is received
 * @Param : None
 * @Return: TRUE or FALSE
 ******************************************************************************/
bool Modem_IsNewDataReceived(void)
{
	if(HW_UART_Modem_IsNewCharReceived() == false)
	{
		return false;
	}
	// Cleanup the response buffer
	memset1((uint8_t *)AtResponseBuff, 0x00, AT_RESPONSE_BUFF_SIZE);

	HW_UART_Modem_GetCharactersUntilNewLine(AtResponseBuff, AT_RESPONSE_BUFF_SIZE, HXC_TIMEOUT);

	if(strncmp("rxdata", AtResponseBuff, 6) == 0)
	{
		return true;
	}
	return false;
}

char* Modem_GetResponseBuffer(void)
{
	return AtResponseBuff;
}
/************************ (C) COPYRIGHT Haxiot *****END OF FILE*****/

