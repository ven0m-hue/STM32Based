/*
 * it.c
 *
 *  Created on: Jun 22, 2021
 *      Author: 91900
 */


void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
