/*
 * msp.c
 *
 *  Created on: Jun 22, 2021
 *      Author: 91900
 */

#include "main.h"

void HAL_MspInit(void)
{
	/*
	 * Low level initialization
	 * 1. Set up the priority grouping of the arm cortex mx processor
	 * 2. Enable the required sustem exceptions of the arm cortex mx processors
	 * 3. Configure the Priority for the system exceptions
	 */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	SCB->SHCSR |= 0x7 << 16; // usage fault, memory fault, bus fault systems

	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	/*
		 * here we are going to do the low level inits. of the USART2 peripheral
		 * 1. Enable the clock of TIM6 peripheral and GPIOA peripheral
		 * 2. Enable the IRQ of IRQ and set up the priority
		 */
	__HAL_RCC_TIM6_CLK_ENABLE();

	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 15, 0);
}



