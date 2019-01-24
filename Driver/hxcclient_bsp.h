/*
  _    _            _____   _______
 | |  | |          |_   _| |__   __|
 | |__| | __ ___  __ | |  ___ | |
 |  __  |/ _` \ \/ / | | / _ \| |
 | |  | | (_| |>  < _| || (_) | |
 |_|  |_|\__,_/_/\_\_____\___/|_|
    (C)2019 HaxIoT
*/
/*******************************************************************************
  * File    : hxcclient_bsp.h
  * Author  : Fahad (Haxiot)
  * Version : V1.0.0
  * Modified: 22-January-2019
  * Brief   : Header file for hxcclient_bsp.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Haxiot</center></h2>
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
#ifndef HXCCLIENT_BSP_H_
#define HXCCLIENT_BSP_H_

#ifdef __cplusplus
extern "C" {
#endif

// Colors are arranged in chronological order: RGB
typedef enum eLedColor
{
    OFF = 0,
    BLUE,
    GREEN,
    CYAN,   // Green and Blue
    RED,
    PINK,   // Red and Blue
    YELLOW, // Red and Green
    WHITE   // Red, Green and Blue
}eLedColor_t;

void HXC_BSP_Init(void);
void HXC_BSP_RGB_AllOn(void);
void HXC_BSP_RGB_Off(void);
void HXC_BSP_RGB_On(eLedColor_t);
uint16_t HXC_BSP_GetTemperature(void);
uint8_t HXC_BSP_GetSlideSwitchStatus(void);


#ifdef __cplusplus
}
#endif

#endif /* HXCCLIENT_BSP_H_ */

/************************ (C) COPYRIGHT Haxiot *****END OF FILE****/
