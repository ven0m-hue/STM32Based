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
void Error_handler(void);
void Timer6_Init(void);
void GPIOLED_Init(void);
TIM_HandleTypeDef tim6;

int main()
{
	//Inits
	HAL_Init();
	SystemClockConfig();
	GPIOLED_Init();
	Timer6_Init();


	//Application code
	/*
	 * Timer ticks independent of the processor, even if the processor goes to sleep.
	 * When the below function is called it call the interrupt callback function
	 * @HAL_TIM_PeriodElapsedCallback
	 */
	HAL_TIM_Base_Start_IT(&tim6);


	while(1);
	return 0;
}

void SystemClockConfig(void)
{

}

void Timer6_Init()
{
	/*
	 * Clk = Peripheral_frequency //16MHz
	 * pre-scaler_counter_clk = clk/(pre-scaler + 1)  // Around 24 (thumb rule)
	 * time_period = 1/pre-scaler_counter_clk // 640000
	 * Period = time_period * time_delay //  100ms * time_period -> 64000
	 * Load the number to the ARR register.(Only 16 bit wide)
	 * side note-> Max ARR value can be upto and not more than 65535 since its a 16 bit register
	 *
	 */
	tim6.Instance = TIM6;
	tim6.Init.Prescaler = 24;
	tim6.Init.Period = 32000-1;// Important, without which timer won't start min value is 1.
	if (HAL_TIM_Base_Init(&tim6) != HAL_OK) Error_handler();
}

void GPIOLED_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_5;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &ledgpio);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void Error_handler(void)
{
	while(1);
}
