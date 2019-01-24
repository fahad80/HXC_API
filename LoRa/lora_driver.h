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
  * @File    : lora_driver.h
  * @Author  : Fahad Mirza (Haxiot)
  * @Version : V1.0.0
  * @Modified: 24-Jan-2019
  * @Brief   : Header for driver lora_driver.h module
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

#ifndef __LORA_DRIVER_H
#define __LORA_DRIVER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Exported define -----------------------------------------------------------*/
#define MAX_PAYLOAD_SIZE   64U

/* Exported types ------------------------------------------------------------*/
typedef enum eJoinMode
{
	OTAA,
	ABP
}eJoinMode_t;

typedef enum eConfirmationStatus
{
	UNCONFIRMED = 0,
	CONFIRMED = 1
}eConfirmationStatus_t;

typedef enum eAdrStatus
{
	ADR_OFF = 0,
	ADR_ON
}eAdrStatus_t;

typedef enum eJoinStatus
{
	JOINED,
	NOT_JOINED
}eJoinStatus_t;

// LoRaWAN State Machine states
typedef enum eDevicState
{
    CLIENT_INIT,
	CLIENT_CONFIG,
    CLIENT_JOIN,
    CLIENT_SEND,
    CLIENT_SLEEP,
	CLIENT_DATA_RECEIVED,
	CLIENT_JOIN_STATUS_CHECK
} eDeviceState_t;

typedef struct sLoraConfig
{
	eJoinMode_t JoinMode;
	eAdrStatus_t AdrStatus;
	char Class;
	char *DevEui;
	char *AppEui;
	char *AppKey;
}sLoraConfig_t;

typedef struct sLoraDriverParam
{
	uint32_t UplinkCycle;
	// Callbacks
	uint8_t (*UplinkHandler)(uint8_t *buffer, uint8_t bufSize, uint8_t *ack, uint8_t *port);
	void (*DownlinkHandler)(uint8_t *buffer, uint8_t dataSize, uint8_t ack, uint8_t port);
}sLoraDriverParam_t;

/* Public functions ----------------------------------------------------------*/
void Lora_init(sLoraConfig_t *loraConfig, sLoraDriverParam_t *loraDriverParam);
void Lora_fsm(void);
uint16_t Lora_getFwVersion(void);
uint8_t Lora_getBatteryLevel(void);
void Lora_updateUplinkRate(uint32_t time_s);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
