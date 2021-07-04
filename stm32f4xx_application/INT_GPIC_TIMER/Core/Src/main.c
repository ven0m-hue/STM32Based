/*
 * main.c
 *
 *  Created on: Jun 22, 2021
 *      Author: Venom
 */

//Includes
#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

//Prototypes
void SystemClockConfig(uint8_t clock_freq);
void Error_handler(void);
void UART2_Init(void);
void Timer2_Init(void);
void GPIOLED_Init(void);
void LSE_Config(void);
// Handle variable of general purpose timer 2 made global
TIM_HandleTypeDef tim2;
UART_HandleTypeDef huart2;
//Global variables
uint32_t clock_freq;  // for clock config
uint32_t flash_latency;

uint32_t input_captures[2];
uint8_t count = 1;

char data_buffer[100];
volatile _Bool isCount_done = false;
char* user = "Hello VA-u\r\n";
int main()
{
	uint32_t capture_diff;
	//Inits
	HAL_Init();
	SystemClockConfig(SYS_CLOCK_FREQ_50MHz);
	LSE_Config();
	Timer2_Init();
	UART2_Init();

	//Application code
	uint16_t data_len = strlen(user);
	HAL_UART_Transmit(&huart2, (uint8_t*)user, data_len, HAL_MAX_DELAY);
	/* In this program we are gonna measure the frequency of the lse using timer input channel to test our timer exercise knowledge.
	 * 1. Generate a square wave clock pulse using the LSE timer in this case.
	 * 2. send it to the input channel of one of the timer. Timer channel 1 in this case
	 * 3. Issue an interrupt when the clock hits a positive edge trigger (in this case)
	 * 4. Compare the time elapsed between the two interrupts
	 * 5. Find the period and finally find the frequency
	 * General Purpose timer has all the functioning of the basic timer
	 * It also has input output channels to trigger the timer from external or internal events
	 */
	HAL_TIM_IC_Start_IT(&tim2, TIM_CHANNEL_1);

	while(1)
	{
		if(isCount_done) // if capture is done, then compute
		{
			if(input_captures[1] > input_captures[0])
				capture_diff = input_captures[1] - input_captures[0];
			else
				capture_diff = (0xFFFFFFFF - input_captures[0]) + input_captures[1];
			//Above code only gives us the count, to get something useful out of it, below code
			/*
			 * Get the timer2 peripheral clock frequency, and get the resolution
			 * Multiply the count with the resolution to get the total timer period elapsed between the interrupt.
			 */
			double timer2count_freq = (HAL_RCC_GetPCLK1Freq() * 2) / (tim2.Init.Prescaler + 1); // @ref clock_config tree
			double timer_res = 1/ timer2count_freq;
			double SignalTimerPeriod = capture_diff * timer_res;
			double Signal_freq = 1/ SignalTimerPeriod;

			// UART routine
			memset(data_buffer, 0, sizeof(data_buffer));
			sprintf(data_buffer,"The Frequency of LSE: %lf\r\n", Signal_freq);

			uint16_t data_len = strlen(data_buffer);
			HAL_UART_Transmit(&huart2, (uint8_t*)data_buffer, data_len, HAL_MAX_DELAY);

			//check the flag
			isCount_done = false;
		}
	}


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
				 *  4. PLL by sourcing HSE clock
				 */
				RCC_OscInitTypeDef osc_init;
				RCC_ClkInitTypeDef clk_init;

				memset(&osc_init,0, sizeof(osc_init));

				osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;  // For HSE or LSE since we need both
				osc_init.HSEState = RCC_HSE_BYPASS;  // This is for HSE
				osc_init.LSEState = RCC_LSE_ON;      // This is for LSE
				osc_init.PLL.PLLState = RCC_PLL_ON;
				osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

				switch(clock_freq)
				{
					case SYS_CLOCK_FREQ_50MHz:
					{
						osc_init.PLL.PLLM = 8;
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
						osc_init.PLL.PLLM = 8;
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
						osc_init.PLL.PLLM = 8;
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
					case SYS_CLOCK_FREQ_180MHz:
					{
						/*
						 * To drive the mcu to 180 MHz we need to drive power controller register
						 * 1.@Ref Power Controller and Set the regulator voltage scale 1, as per the data sheet
						 * 2.Turn on the Over drive mode
						 */
						__HAL_RCC_PWR_CLK_ENABLE(); // ALways enable the clock for anything

						__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

						__HAL_PWR_OVERDRIVE_ENABLE();

						// osc init
						osc_init.PLL.PLLM = 8;
						osc_init.PLL.PLLN = 360;
						osc_init.PLL.PLLP = 2;   // default
						osc_init.PLL.PLLQ = 2;   // default
						osc_init.PLL.PLLR = 2;   // default
						// PLL is  configured
						/*
						 *  In this project configure the following -> to know more refer @Clock Configuration from the ioc
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

						flash_latency = FLASH_LATENCY_5;
					}
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

void LSE_Config()
{
#if 0
	/*
	 * LSE config around 32.768KHz
	 */
	RCC_OscInitTypeDef osc_init;
	memset(&osc_init,0, sizeof(osc_init));

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	osc_init.LSEState = RCC_LSE_ON;
	if(HAL_RCC_OscConfig(&osc_init)) Error_handler();
#endif
	// Selects the clock soure to output on any one of the pin
	// MCO1 = PA8 and MCO2 = PA9
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCOSOURCE_LSE, RCC_MCODIV_1);
}

void Timer2_Init(void)
{
	TIM_IC_InitTypeDef timerICconfig; // Input capture and not interrupt capture.
	/*
	 * Timer 2 initialization and instantiation
	 */
	tim2.Instance = TIM2;
	tim2.Init.CounterMode = TIM_COUNTERMODE_UP; // set as up counter
	tim2.Init.Period = 0xFFFFFFFF;
	tim2.Init.Prescaler = 1;
	if(HAL_TIM_IC_Init(&tim2) != HAL_OK) Error_handler();  //Timer 2 is configured
	 /*
		 * Working with the timer2 input channel, for more info @ref general purpose timer in reference manual
		 * 1. Init the timer input to Capture the time base
		 * 2. Config input channel of timer
	 */
	timerICconfig.ICFilter = 0;
	timerICconfig.ICPolarity = TIM_ICPOLARITY_RISING;
	timerICconfig.ICPrescaler = TIM_ICPSC_DIV1;
	timerICconfig.ICSelection = TIM_ICSELECTION_DIRECTTI;

	if(HAL_TIM_IC_ConfigChannel(&tim2, &timerICconfig, TIM_CHANNEL_1) != HAL_OK) Error_handler();

}

// Now lets implement the cpature callback, cuz when the input capture occurs interrupt is issued
// Go to the time driver file
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/*
	 * 1. Save the content of the capture/compare register.
	 * 2. End of story.
	 */
	if(! isCount_done)
	{
		if (count == 1)
		{
			input_captures[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			count++;
		}

		else if (count == 2)
		{
			input_captures[1] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			count = 1;
			isCount_done = true;
		}
	}
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
void Error_handler(void)
{
	while(1);
}
