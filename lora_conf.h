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
  * @File    : lora_conf.h
  * @Author  : Fahad Mirza (Haxiot)
  * @Version : V1.0.0
  * @Modified: 24-Jan-2019
  * @Brief   : LoRaWAN Configuration
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

#ifndef __LORA_CONF_H
#define __LORA_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <ctype.h>
#include <string.h>
#include "lora_driver.h"
#include "hxcclient_bsp.h"


/* Macros --------------------------------------------------------------------*/
#define UPLINK_CYCLE  (15000U)      // Send packet every 15s


/* Private function declaration ----------------------------------------------*/
static uint8_t GetSensorData(uint8_t *buffer, uint8_t bufSize, uint8_t *ack, uint8_t *port);
static void LedControl(uint8_t *buffer, uint8_t dataSize);
static void UpdateUplinkRate(uint8_t *buffer, uint8_t dataSize);
static void ParseDownlink(uint8_t *buffer, uint8_t dataSize, uint8_t ack, uint8_t port);
static void TolowerArray(char *array, uint8_t arraySize);


/* Private variables ---------------------------------------------------------*/
enum DownlinkType
{
    DW_LED = 1,
    DW_UPLINK_RATE,
}eDownlinkType;

static sLoraConfig_t LoraConfigParam =
{
    .JoinMode  = OTAA,
    .AdrStatus = ADR_OFF,
    .Class     = 'C',
    .DevEui    = "AUTO",
    .AppEui    = "E0A5B81425B5D2C7",
    .AppKey    = "CDF5E6547CF354542335DCAEB657DBDF"
};

static sLoraDriverParam_t LoraDriverParam =
{
    .UplinkCycle     = UPLINK_CYCLE,
    .UplinkHandler   = GetSensorData,
    .DownlinkHandler = ParseDownlink
};

/* Private function definitions ----------------------------------------------*/
/******************************************************************************
 * @Brief : Uplink packet handler for lora_driver
 * @Param : Pointer for payload buffer, data size, ack configuration and port
 * @Return: packet size
******************************************************************************/
static uint8_t GetSensorData(uint8_t *buffer, uint8_t bufSize, uint8_t *ack, uint8_t *port)
{
    /* Prepare an unconfirmed uplink packet for port 2 */   
    uint8_t size = 0;
    
    uint16_t tempInt = HXC_BSP_GetTemperature();
    // Checkout our user manual to convert tempInt into degree Celcius
    
    

    buffer[size++] = (tempInt >> 8) & 0xFF;
    buffer[size++] = tempInt & 0xFF;
    buffer[size++] = HXC_BSP_GetSlideSwitchStatus();
    buffer[size++] = Lora_getBatteryLevel();
    buffer[size++] = (uint8_t)Lora_getFwVersion(); // This function returns uint16_t
    
    *ack = (uint8_t)UNCONFIRMED;
    *port = 2;
    
    return size;
}


/******************************************************************************
 * @Brief : Downlink packet handler for lora_driver
 *          Valid downlink messages are: 'red', 'green', 'blue' and 'off'
 * @Param : Payload buffer, data size, ack configuration and port
 * @Return: None
******************************************************************************/
static void ParseDownlink(uint8_t *buffer, uint8_t dataSize, uint8_t ack, uint8_t port)
{
    switch(port)
    {
        case DW_LED:
        {
            LedControl(buffer, dataSize);
            break;
        }
        case DW_UPLINK_RATE:
        {
            UpdateUplinkRate(buffer, dataSize);
            break;
        }
        default:
        {
            break;
        }
    }
}
static void LedControl(uint8_t *buffer, uint8_t dataSize)
{
    TolowerArray((char *)buffer, dataSize);
    
    if(strncmp("red", (const char *)buffer, 3) == 0)
    {
        HXC_BSP_RGB_On(RED);
    }
    else if(strncmp("green", (const char *)buffer, 5) == 0)
    {
        HXC_BSP_RGB_On(GREEN);
    }
    else if(strncmp("blue", (const char *)buffer, 4) == 0)
    {
        HXC_BSP_RGB_On(BLUE);
    }
    else if(strncmp("off", (const char *)buffer, 3) == 0)
    {
        HXC_BSP_RGB_Off();
    }
    else
    {
        // Users can send any combinations of the RGB
        // using last 3bits. eLedColor_t is arranged
        // in chronological order.
        HXC_BSP_RGB_On((eLedColor_t)(buffer[0] & 0x07));
    }
}

static void UpdateUplinkRate(uint8_t *buffer, uint8_t dataSize)
{
  //Lora_updateUplinkRate();
}


/******************************************************************************
 * @Brief : Upper case to lower case for an array
 * @Param : Pointer to array containing characters and array size
 * @Return: None
******************************************************************************/
static void TolowerArray(char *array, uint8_t arraySize)
{
    for(uint8_t i = 0; i < arraySize; i++)
    {
        array[i] = tolower(array[i]);
    }
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
