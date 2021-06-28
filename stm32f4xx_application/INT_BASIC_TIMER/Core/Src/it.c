/*
 * it.c
 *
 *  Created on: Jun 22, 2021
 *      Author: 91900
 */

#include "main.h"
extern TIM_HandleTypeDef tim6;
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&tim6);

}
