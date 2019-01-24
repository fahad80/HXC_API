/*
  _    _            _____   _______
 | |  | |          |_   _| |__   __|
 | |__| | __ ___  __ | |  ___ | |
 |  __  |/ _` \ \/ / | | / _ \| |
 | |  | | (_| |>  < _| || (_) | |
 |_|  |_|\__,_/_/\_\_____\___/|_|
    (C)2017 HaxIoT
*/
/*******************************************************************************
  * @File    : lora_driver.c
  * @Author  : Fahad Mirza (Haxiot)
  * @Version : V1.0.0
  * @Modified: 24-Jan-2019
  * @Brief   : LoRa Driver
  ******************************************************************************
  * @attention
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
#include <stdio.h>
#include "hxc_client.h"
#include "debug.h"
#include "stm32l0xx_nucleo.h"
#include "time_server.h"
#include "lora_driver.h"
#include "tiny_sscanf.h"
#include "utilities.h"
#include "hxcclient_bsp.h"

/* Private Macros ------------------------------------------------------------*/
#define JOIN_SEND_DELAY_MAX   (10000U) // Randomization range - 10s
#define JOIN_STATUS_REQ_DELAY (7000U)  // milliseconds. Join req takes 6s.

// Re Authenticate every REJOIN_TIME
#define REJOIN_TIME           (1 * 60 * 60 * 1000U) // 1 hour

/* Private global variables --------------------------------------------------*/
static sLoraConfig_t *LoraConfigParam;
static sLoraDriverParam_t *LoraDriverParam;
static volatile eDeviceState_t DeviceState = CLIENT_INIT;

static TimerEvent_t JoinRequestTimer;
static TimerEvent_t RejoinTimer;
static TimerEvent_t SensorMeasureTimer;
static TimerEvent_t JoinStatusReqTimer;

// Object definition for data to be sent to loRa application server
static uint8_t DataBinaryBuff[MAX_PAYLOAD_SIZE];
static sSendDataBinary_t SendDataBinary = {DataBinaryBuff, 0 , 0, 0};

// RNG handler declaration
RNG_HandleTypeDef RngHandle = {.Instance = RNG};

/* Private functions ---------------------------------------------------------*/
static void OnRejoinTimerEvent(void);
static void OnJoinRequestTimerEvt(void);
static void OnJoinStatusReqTimerEvt(void);
static void OnSensorMeasureTimerEvt(void);
static void setJoinRequestTimer(void);


/* Function definitions ------------------------------------------------------*/
/******************************************************************************
  * @Brief  : Initialize LoRa Modem
  * @Param  : sLoraConfig_t
  * @Return : None
******************************************************************************/
void Lora_init(sLoraConfig_t *loraConfig, sLoraDriverParam_t *loraDriverParam)
{

	LoraConfigParam = loraConfig;
	LoraDriverParam = loraDriverParam;

	if(Modem_Init() != AT_OK)
	{
		DBG_PRINTF("Modem_Init failed\r\n");
	}

	// Initialize RNG for join request send randomization
	HAL_RNG_Init(&RngHandle);


	// Timer for join request send
	TimerInit(&JoinRequestTimer, OnJoinRequestTimerEvt);
	// Timer for join status check
	TimerInit(&JoinStatusReqTimer, OnJoinStatusReqTimerEvt);
	TimerSetValue(&JoinStatusReqTimer, JOIN_STATUS_REQ_DELAY);
	// Timer for sensor occurrence measure
	TimerInit(&SensorMeasureTimer, OnSensorMeasureTimerEvt);
	TimerSetValue(&SensorMeasureTimer, LoraDriverParam->UplinkCycle);
	// Timer for Re-Authenticate / Re-Join
	TimerInit(&RejoinTimer, OnRejoinTimerEvent);
	TimerSetValue(&RejoinTimer, REJOIN_TIME);
}

/******************************************************************************
  * @Brief  : Check if the modem responds
  * @Param  : void
  * @Return : AT_OK or other eAtStatus_t
******************************************************************************/
static eAtStatus_t is_modem_working(void)
{
	return Modem_AT_Cmd(AT_CTRL, AT, NULL);
}

/******************************************************************************
  * @Brief  : Set Device EUI
  * @Param  : Pointer to Device EUI
  * @Return : AT_OK or other eAtStatus_t
******************************************************************************/
static eAtStatus_t Lora_setDevEui(char *devEui)
{
	return Modem_AT_Cmd(AT_SET, AT_DEVEUI, devEui);
}

/******************************************************************************
  * @Brief  : Print Device EUI
  * @Param  : None
  * @Retval : AT_OK or other eAtStatus_t
******************************************************************************/
static void Lora_printDevEui(void)
{
	char deveui_str[25];
	
	Modem_AT_Cmd(AT_GET, AT_DEVEUI, deveui_str);
	
	DBG_PRINTF("DevEui: %s\n", deveui_str);
}

/******************************************************************************
  * @Brief  : Print Device Address
  * @Param  : None
  * @Retval : AT_OK or other eAtStatus_t
******************************************************************************/
static void Lora_printDevAdr(void)
{
    char devadr_str[15];

    Modem_AT_Cmd(AT_GET, AT_DEVADR, devadr_str);

    DBG_PRINTF("DevAdr: %s\n", devadr_str);
}

/******************************************************************************
  * @Brief  : Turn on or off ADR
  * @Param  : ADR_ON or ADR_OFF
  * @Return : AT_OK or other eAtStatus_t
******************************************************************************/
static eAtStatus_t Lora_setAdr(eAdrStatus_t adrStatus)
{
	//char adr = (adrStatus == ADR_ON ? '1' : '0');
	return Modem_AT_Cmd(AT_SET, AT_ADR, (uint8_t *)(&adrStatus));
}

/******************************************************************************
  * @Brief  : Set Application EUI
  * @Param  : Pointer to Application EUI
  * @Return : AT_OK or other eAtStatus_t
******************************************************************************/
static eAtStatus_t Lora_setAppEui(char *appEui)
{
	return Modem_AT_Cmd(AT_SET, AT_APPEUI, appEui);
}

/******************************************************************************
  * @Brief  : Set Application Key
  * @Param  : Pointer to Application Key
  * @Return : AT_OK or other eAtStatus_t
******************************************************************************/
static eAtStatus_t Lora_setAppKey(char *appKey)
{
	return Modem_AT_Cmd(AT_SET, AT_APPKEY, appKey);
}

/******************************************************************************
  * @Brief  : Set join mode
  * @Param  : OTAA or ABP
  * @Retval : AT_OK if successful, otherwise other eAtStatus_t
******************************************************************************/
static eAtStatus_t Lora_setJoinMode(eJoinMode_t joinMode)
{
	if(joinMode == OTAA)
	{
		return Modem_AT_Cmd(AT_SET, AT_NJM, "OTAA");
	}

	return Modem_AT_Cmd(AT_SET, AT_NJM, "ABP");
}

/******************************************************************************
  * @Brief  : Set Class
  * @Param  : CLASS_A or CLASS_C
  * @Retval : AT_OK if successful, otherwise other eAtStatus_t
******************************************************************************/
static eAtStatus_t Lora_setClass(char class)
{
	return Modem_AT_Cmd(AT_SET, AT_CLASS, &class);
}

/******************************************************************************
  * @Brief  : Join network and initiate join sleep transition timer
  * @Param  : None
  * @Retval : AT_OK or other eAtStatus_t
******************************************************************************/
static eAtStatus_t Lora_Join(void)
{
	return Modem_AT_Cmd(AT_CTRL, AT_JOIN, NULL);
}

/******************************************************************************
  * @Brief  : Check JOIN status
  * @Param  : None
  * @Retval : JOINED or NOT_JOINED
******************************************************************************/
static eJoinStatus_t Lora_getJoinStatus(void)
{
	char joinStatus[12]; // "NOT JOINED" is 10 characters + 1 Null char

	Modem_AT_Cmd(AT_GET, AT_NJS, joinStatus);

    if(strncmp("JOINED", joinStatus, 6) == 0)
    {
    	return JOINED;
    }

	return NOT_JOINED;
}

/******************************************************************************
  * @Brief  : Get firmware version
  * @Param  : None
  * @Retval : Integer firmware version
******************************************************************************/
uint16_t Lora_getFwVersion(void)
{
	char fwVersion_str[5];
	uint16_t fwVer_int;

	eAtStatus_t status = Modem_AT_Cmd(AT_GET, AT_VER, fwVersion_str);
	
	if(status != AT_OK)
	{
		return 0;
	}
	
	// FW version is a decimal number e.g. 1.18
	// Convert the float number into integer.
	uint8_t i = 0;
	while(fwVersion_str[++i] != '.');

	while(fwVersion_str[i] != '\0')
	{
		fwVersion_str[i] = fwVersion_str[i + 1];
		i++;
	}

	tiny_sscanf(fwVersion_str, "%hu", &fwVer_int);

	return fwVer_int;
}

/******************************************************************************
  * @Brief  : Get battery level
  * @Param  : None
  * @Retval : Battery level
******************************************************************************/
uint8_t Lora_getBatteryLevel(void)
{
	char batteryLevel_str[5];
	uint8_t batteryLevel = 0;

	eAtStatus_t status = Modem_AT_Cmd(AT_GET, AT_BAT, batteryLevel_str);
	
	if(status == AT_OK)
	{
	    tiny_sscanf(batteryLevel_str, "%hhu", &batteryLevel);
	}

	return batteryLevel;
}

/******************************************************************************
  * @Brief  : Update the uplink rate
  * @Param  : time_s seconds
  * @Retval : None
******************************************************************************/
void Lora_updateUplinkRate(uint32_t time_s)
{
    TimerSetValue(&SensorMeasureTimer, (time_s * 1000));
    TimerReset(&SensorMeasureTimer);
}

/******************************************************************************
  * @Brief  : Send uplink packet using binary payload
  * @Param  : Pointer of sSendDataBinary_t variable
  * @Retval : AT_OK or other eAtStatus_t statuses
******************************************************************************/
static eAtStatus_t Lora_SendDataBinary(sSendDataBinary_t *binaryData)
{
	return Modem_AT_Cmd(AT_SET, AT_SENDB, binaryData);
}

/******************************************************************************
 * @Brief  : Read the received downlink packet
 * @Param  : Pointer to sRecvDataBinary_t variable
 * @Return : AT_OK or other eAtStatus_t statuses
******************************************************************************/
static eAtStatus_t Lora_ReadData(sRecvDataBinary_t *rxData)
{
	char *rxString = Modem_GetResponseBuffer();

	// Find the position after :
	// e.g. rxdata:0,1,86GF25
	rxString = (strchr(rxString,':') + 1);

	if(rxString == NULL)
	{
		return AT_ERROR;
	}

	if(tiny_sscanf(rxString, "%hhu,%hhu", &(rxData->Ack), &(rxData->Port)) != 2)
	{
		return AT_ERROR;
	}

	// Find the position after the second comma
	rxString = strchr((strchr(rxString,',') + 1),',') + 1;
	if(rxString == NULL)
	{
		return AT_ERROR;
	}

	rxData->DataSize = stringHexToByteArray(rxString, rxData->Buffer, MAX_PAYLOAD_SIZE);

	return AT_OK;
}

/******************************************************************************
  * @Brief  : LoRa Modem state machine
  * @Param  : Void
  * @Return : None
******************************************************************************/
void Lora_fsm(void)
{
    switch(DeviceState)
    {
        case CLIENT_INIT:
        {
    	    if(is_modem_working() != AT_OK)
    	    {
    	    	DBG_PRINTF("AT failed. Resetting HW...\r\n");
    	        // Modem isn't responding. Execute hard reset.
                Modem_HardReset();
                // We stay in CLIENT_INIT state and try again.
    	    }
    	    else
    	    {
    	    	DeviceState = CLIENT_CONFIG;
    	    }

            break;
        }
        case CLIENT_CONFIG:
        {
        	eAtStatus_t loraModemRetCode = Lora_setDevEui(LoraConfigParam->DevEui);
			loraModemRetCode |= Lora_setAppEui(LoraConfigParam->AppEui);
			loraModemRetCode |= Lora_setAppKey(LoraConfigParam->AppKey);
			loraModemRetCode |= Lora_setJoinMode(LoraConfigParam->JoinMode);
			loraModemRetCode |= Lora_setClass(LoraConfigParam->Class);
			loraModemRetCode |= Lora_setAdr(LoraConfigParam->AdrStatus);				

			if(loraModemRetCode == AT_OK)
			{
				// If users use AUTO as Device EUI, print the DevEUI
			    // so that they can add that on X-ON
			    Lora_printDevEui();
				
				setJoinRequestTimer();
				DeviceState = CLIENT_SLEEP;
			}
			else if(loraModemRetCode == AT_TIMEOUT)
			{
				DeviceState = CLIENT_INIT;
			}
			else
			{
				DBG_PRINTF("Check your keys\r\n");
				while(1);
			}

			break;
        }
        case CLIENT_JOIN:
        {
            DBG_PRINTF("Joining...\r\n");
            HXC_BSP_RGB_Off();

            switch(Lora_Join())
            {
                case AT_OK:
                {
                	// Start the Join status request timer and go to sleep
                	TimerStart(&JoinStatusReqTimer);
                    DeviceState = CLIENT_SLEEP;
                    break;
                }
                case AT_TIMEOUT:
                {
                	// The modem isn't responding. Execute hard reset
                	DeviceState = CLIENT_INIT;
                	break;
                }
                default:
                {
                	DBG_PRINTF("Join cmd failed\n");
                    // We stay in CLIENT_JOIN state and redo Lora_Join()
                	break;
                }
            }

            break;
        }
        case CLIENT_JOIN_STATUS_CHECK:
        {
        	if(Lora_getJoinStatus() == JOINED)
        	{
        		// Indicate Join status using LED
    			HXC_BSP_RGB_On(PINK);
        		DBG_PRINTF("Nwk Joined\n");
        		Lora_printDevAdr();
        		
        		// Start timer for Re-Authentication
        		TimerStart(&RejoinTimer);
        		
				// Start timer for uplink transmission for sensor-data
				TimerStart(&SensorMeasureTimer);
				DeviceState = CLIENT_SLEEP;
        	}
        	else
        	{
        		// Try joining again
        		setJoinRequestTimer();
                DeviceState = CLIENT_SLEEP;
        	}
        	break;
        }
		case CLIENT_SLEEP:
        {
            /* Wake up through RTC events or asynchronous event coming from HXC modem*/
        	if(Modem_IsNewDataReceived() == true)
        	{
        		DeviceState = CLIENT_DATA_RECEIVED;
        	}
            break;
        }
		case CLIENT_SEND:
		{
			// Read sensor data and populate payload
			SendDataBinary.DataSize = LoraDriverParam->UplinkHandler(\
			                          SendDataBinary.Buffer         ,\
			                          MAX_PAYLOAD_SIZE              ,\
			                          &SendDataBinary.Ack           ,\
			                          &SendDataBinary.Port          );
			
			// Initiate uplink transmission
			eAtStatus_t status = Lora_SendDataBinary(&SendDataBinary);
			
			if (status == AT_OK)
			{
				DBG_PRINTF("Uplink sent\n");
				// Schedule the next packet
				TimerStart(&SensorMeasureTimer);
				DeviceState = CLIENT_SLEEP;
			}
			else if(status == AT_TIMEOUT)
			{
				// Device isn't responding. Go to init.
				DeviceState = CLIENT_INIT;
			}
			else if(status == AT_NO_NET_JOINED)
            {
                DeviceState = CLIENT_JOIN;
            }
			else
			{
				DBG_PRINTF("Uplink Failed (Error: %d)\n", (uint8_t)status);
			}
		    break;
		}
		case CLIENT_DATA_RECEIVED:
		{
			uint8_t rBuffer[64];
			sRecvDataBinary_t rxPacket = {.Buffer = rBuffer};

			if(Lora_ReadData(&rxPacket) == AT_OK)
			{
				// Execute users ReceivedPacketHandler function
				LoraDriverParam->DownlinkHandler(rxPacket.Buffer, rxPacket.DataSize, rxPacket.Ack, rxPacket.Port);
			}
			else
			{
				DBG_PRINTF("Dwlink read failed\n");
			}

			DeviceState = CLIENT_SLEEP;
			break;
		}
		default:
		{
			DeviceState = CLIENT_INIT;
		    break;
		}
    }
}

/******************************************************************************
 * @Brief  : Set Join request timer
 * @Param  : none
 * @Return : none
******************************************************************************/
static void setJoinRequestTimer(void)
{
	// Use a random delay to avoid synchronized join request from all LoRa node after power up
	uint32_t joinDelay = (HAL_RNG_GetRandomNumber(&RngHandle) % JOIN_SEND_DELAY_MAX) + 1;
	
	TimerSetValue(&JoinRequestTimer, joinDelay);
	TimerStart(&JoinRequestTimer);
}

/******************************************************************************
 * @Brief  : Function executed on JoinStatusDelayTimer Timeout event
 * @Param  : none
 * @Return : none
******************************************************************************/
static void OnJoinRequestTimerEvt(void)
{
	TimerStop(&JoinRequestTimer);
	// If this is a re-join SensorMeasureTimer is active
	TimerStop(&SensorMeasureTimer);
	
	DeviceState = CLIENT_JOIN;
}

/******************************************************************************
 * @Brief  : Function executed on JoinStatusDelayTimer Timeout event
 * @Param  : none
 * @Return : none
******************************************************************************/
static void OnJoinStatusReqTimerEvt(void)
{
	TimerStop(&JoinStatusReqTimer);
	DeviceState = CLIENT_JOIN_STATUS_CHECK;
}

/******************************************************************************
 * @Brief  : Function executed on SensorMeasureTimer Timeout event
 * @Param  : none
 * @Return : none
******************************************************************************/
static void OnSensorMeasureTimerEvt(void)
{
	TimerStop(&SensorMeasureTimer);
    DeviceState = CLIENT_SEND;
}

/******************************************************************************
 * @Brief  : Function executed on RejoinTimer Timeout event
 * @Param  : none
 * @Return : none
******************************************************************************/
static void OnRejoinTimerEvent(void)
{
	TimerStop(&RejoinTimer);
	setJoinRequestTimer();
}

/******************************************************************************
  * @brief RNG MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param hrng: RNG handle pointer
  * @retval None
******************************************************************************/
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{
	/* RNG Peripheral clock enable */
    __HAL_RCC_RNG_CLK_ENABLE();
}


