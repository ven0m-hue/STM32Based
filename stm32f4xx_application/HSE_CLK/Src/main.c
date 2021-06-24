/*
 * main.c
 *
 *  Created on: Jun 22, 2021
 *      Author: Venom
 */

#include "main.h"
#include <string.h>
#include <stdio.h>

void SystemClockConfig(void);
void UART2_Init(void);
void Error_handler(void);
uint8_t ConvertoCap(uint8_t data);
UART_HandleTypeDef huart2;

char* user_data = "The VA-U\r\n";

int main()
{
	//Inits
	HAL_Init();

	/*
	 * Before System Call config
	 */
	UART2_Init();
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


	/*
	 * Since various peripherals are dependent on the system clock, so its a good idea to place system config call
	 * above all the inits.
	 */
	SystemClockConfig();

	UART2_Init(); // Since the APB bus frequnecy has changed the baud rate must be re-configured



    // User Application
	//char message[100];
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

void SystemClockConfig(void)
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

		osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
		osc_init.HSEState = RCC_HSE_BYPASS;

		// HSE configured
		/*
		 *  In this project configure the following
		 *  1. Config AHB bus clock as 4MHz  /2
		 *  2. Config APB1 bus clock as 2MHz /4 ie divide by 2
		 *  3. Config APB2 bus clock as 2MHz /4 ie divide by 2
		 *  Configured using the RCC config registers
		 */
		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
		clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV2;
		clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV2;
		clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV2;

		if(HAL_RCC_OscConfig(&osc_init) != HAL_OK) Error_handler();


	    // after this line if everything is okay HSE is succefully turned on
		if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_0) != HAL_OK) Error_handler();
	    // We can disable the HSI hereonwards.


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
