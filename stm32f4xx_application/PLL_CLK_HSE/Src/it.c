/*
 * it.c
 *
 *  Created on: Jun 22, 2021
 *      Author: 91900
 */

#include "main.h"
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
