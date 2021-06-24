/*
 * main.c
 *
 *  Created on: Jun 22, 2021
 *      Author: Venom
 */

#include "main.h"
#include <string.h>
#include <stdio.h>

void SystemClockConfig(uint8_t clock_freq);
void UART2_Init(void);
void Error_handler(void);
uint8_t ConvertoCap(uint8_t data);
UART_HandleTypeDef huart2;

char* user_data = "The VA-U\r\n";
uint32_t flash_latency;
uint8_t clock_freq;
int main()
{
	//Inits
	HAL_Init();
	/*
	 * Since various peripherals are dependent on the system clock, so its a good idea to place system config call
	 * above all the inits.
	 */
	SystemClockConfig(SYS_CLOCK_FREQ_180MHz);

	UART2_Init(); // Since the APB bus frequnecy has changed the baud rate must be re-configured

	// User Application
	char message[100];
	memset(message,0, sizeof(message));
	sprintf(message, "SYSCLK : %ldHz\r\n", HAL_RCC_GetSysClockFreq());
	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

	memset(message,0,sizeof(message));
	sprintf(message,"HCLK   : %ldHz\r\n",HAL_RCC_GetHCLKFreq());
	HAL_UART_Transmit(&huart2,(uint8_t*)message,strlen(message),HAL_MAX_DELAY);

	memset(message,0,sizeof(message));
	sprintf(message,"PCLK1  : %ldHz\r\n",HAL_RCC_GetPCLK1Freq());
	HAL_UART_Transmit(&huart2,(uint8_t*)message,strlen(message),HAL_MAX_DELAY);

	memset(message,0,sizeof(message));
	sprintf(message,"PCLK2  : %ldHz\r\n",HAL_RCC_GetPCLK2Freq());
	HAL_UART_Transmit(&huart2,(uint8_t*)message,strlen(message),HAL_MAX_DELAY);

	while(1);

	return 0;
}

void SystemClockConfig(uint8_t clock_freq)
{

	/*
			 *  Use the external clock sourced from the stlink onboard debugger mcu's clock
			 *  1. Initialize the oscillator corresponding to their respective regDef
			 *  2. Initialize the oscillator as a bypass, as it is sourced from another mcu
			 *  3. Init the RCC clock config to succefully init the HSE
			 */
			RCC_OscInitTypeDef osc_init;
			RCC_ClkInitTypeDef clk_init;

			memset(&osc_init,0, sizeof(osc_init));

			osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
			osc_init.HSIState = RCC_HSI_ON;
			osc_init.HSICalibrationValue = 16;
			osc_init.PLL.PLLState = RCC_PLL_ON;
			osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;

			switch(clock_freq)
			{
				case SYS_CLOCK_FREQ_50MHz:
				{
					osc_init.PLL.PLLM = 16;
					osc_init.PLL.PLLN = 100;
					osc_init.PLL.PLLP = 2;
					osc_init.PLL.PLLQ = 2;
					osc_init.PLL.PLLR = 2;
					// PLL is  configured
					/*
					 *  In this project configure the following
					 *  1. Config AHB bus clock as 50MHz  /1
					 *  2. Config APB1 bus clock as 25MHz /2 ie divide by 2
					 *  3. Config APB2 bus clock as 25MHz /2 ie divide by 2
					 *  Configured using the RCC config registers
					 */
					clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
					clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
					clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
					clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV2;
					clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV2;

					flash_latency = FLASH_LATENCY_1;

					break;

				}
				case SYS_CLOCK_FREQ_80MHz:
				{
					osc_init.PLL.PLLM = 16;
					osc_init.PLL.PLLN = 160;
					osc_init.PLL.PLLP = 2;
					osc_init.PLL.PLLQ = 2;
					osc_init.PLL.PLLR = 2;
					// PLL is  configured
					/*
					 *  In this project configure the following
					 *  1. Config AHB bus clock as 80MHz  /1
					 *  2. Config APB1 bus clock as 40MHz /2 ie divide by 2
					 *  3. Config APB2 bus clock as 80MHz /1 ie divide by 1
					 *  Configured using the RCC config registers
					 */
					clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
					clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
					clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
					clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV2;
					clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV1;

					flash_latency = FLASH_LATENCY_2;

					break;
				}
				case SYS_CLOCK_FREQ_120MHz:
				{
					osc_init.PLL.PLLM = 16;
					osc_init.PLL.PLLN = 240;
					osc_init.PLL.PLLP = 2;
					osc_init.PLL.PLLQ = 2;
					osc_init.PLL.PLLR = 2;
					// PLL is  configured
					/*
					 *  In this project configure the following
					 *  1. Config AHB bus clock as 120MHz  /1
					 *  2. Config APB1 bus clock as 40MHz /2 ie divide by 2
					 *  3. Config APB2 bus clock as 60MHz /2 ie divide by 2
					 *  Configured using the RCC config registers
					 */
					clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
					clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
					clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
					clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV4;
					clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV2;

					flash_latency = FLASH_LATENCY_3;
					break;
				}
				/*
				 * Side Note: PLLN value must be greater than 50 and lesser than 432
				 */
				default: return;
			}

		if(HAL_RCC_OscConfig(&osc_init) != HAL_OK) Error_handler();

	    // after this line if everything is okay HSE is succefully turned on
		if (HAL_RCC_ClockConfig(&clk_init, flash_latency) != HAL_OK) Error_handler();

		/*
		 * SYSTICK CONFIG
		 * Since we have changed the clock config from default frequency to the application specific
		 * We need to change the clock config  going into the arm cortex processor. (prcoessor side clock config).
		 */

		HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/ 1000);
		HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); // There is a pre-scalar @Ref ClockTree

}

void UART2_Init(void)
{
	/*
	 * High level initialization
	 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;

	if(HAL_UART_Init(&huart2) != HAL_OK) Error_handler();  // If there is a problem

}


uint8_t ConvertoCap(uint8_t data)
{
	if (data >= 'a' && data<='z') data = data - ('a' - 'A');

	return data;
}

void Error_handler(void)
{
	while(1);
}
