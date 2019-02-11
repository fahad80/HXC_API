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
  * @File    : main.cpp
  * @Author  : Fahad Mirza (Haxiot)
  * @Version : V1.0.0
  * @Modified: 24-Jan-2019
  * @Brief   : Main file
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
#include "hw.h"
#include "hxcclient_bsp.h"
#include "lora_conf.h"

/* Variables -----------------------------------------------------------------*/
//Flag to indicate if the MCU is Initialized
static bool McuInitialized = false;

/* Function Declarations -----------------------------------------------------*/
void SystemClock_Config(void);
void HW_Init(void);

int main()
{
    HW_Init();
    
    DBG_PRINTF("HXC900-NucleoL053R8 Demo Application\r\n");    
    Lora_init(&LoraConfigParam, &LoraDriverParam);

    while(1) 
    {
        Lora_fsm();
    }
}

/******************************************************************************
  * @Brief : This function initializes the hardware
  * @Param : None
  * @Return: None
******************************************************************************/
void HW_Init(void)
{
  if(McuInitialized == false)
  {
  	// Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();
    SystemClock_Config();
    Debug_UART_Init();
    HW_RTC_Init();
    BSP_LED_Init(LED_GREEN);// LED on Nucleo board
    HXC_BSP_Init();
    McuInitialized = true;
  }
}

/******************************************************************************
  * @Brief :  System Clock Configuration
  * The system Clock is configured as follow :
  *      System Clock source            = PLL (HSI)
  *      SYSCLK(Hz)                     = 32000000
  *      HCLK(Hz)                       = 32000000
  *      AHB Prescaler                  = 1
  *      APB1 Prescaler                 = 1
  *      APB2 Prescaler                 = 1
  *      HSI Frequency(Hz)              = 16000000
  *      PLLMUL                         = 6
  *      PLLDIV                         = 3
  *      Flash Latency(WS)              = 1
  * @Return: None
  * @Note  : This function enables all the clock necessary for the demo
  *          including UARTs
******************************************************************************/
void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    // Enable HSI48 Oscillator for RNG analog part
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      // Initialization Error
      Error_Handler();
    }

    // Set Voltage scale1 as MCU will run at 32MHz
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

    // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
    	Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

/******************************************************************************
  * @Brief : Initializes the MSP.
  * @Param : None
  * @Return: None
******************************************************************************/
void HAL_MspInit(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();

	// Disable the Power Voltage Detector
	HAL_PWR_DisablePVD();

	// Enables the Ultra Low Power mode
	HAL_PWREx_EnableUltraLowPower();

	__HAL_FLASH_SLEEP_POWERDOWN_ENABLE();

	/* In debug mode, e.g. when DBGMCU is activated, Arm core has always clocks
	 * And will not wait that the FLACH is ready to be read. It can miss in this
	 * case the first instruction. To overcome this issue, the flash remain clocked during sleep mode
	 */
	DBG( __HAL_FLASH_SLEEP_POWERDOWN_DISABLE(); );
	/*Enable fast wakeUp*/
	HAL_PWREx_EnableFastWakeUp( );
}

#ifdef USE_FULL_ASSERT

/******************************************************************************
   * @Brief : Reports the name of the source file and the source line number
   *          where the assert_param error has occurred.
   * @Param : file: pointer to the source file name
   *          line: assert_param error line source number
   * @Return: None
******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}

#endif

/************************ (C) COPYRIGHT Haxiot ***** END OF FILE ****/

