/*
 * it.c
 *
 *  Created on: July  5, 2021
 *      Author: 91900
 */

#include "main.h"

extern TIM_HandleTypeDef tim2;
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&tim2);
}
