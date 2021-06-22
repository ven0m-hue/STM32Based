/*
 * it.c
 *
 *  Created on: Jun 22, 2021
 *      Author: 91900
 */

#include "main.h"
extern UART_HandleTypeDef huart2; //Since the handler variable is only availble in main.c

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

/*
 * Handling the Interrupt requests
 * 1. Populate the it.c with the respective IRQ handlers definitions.
 * 2. The name of the IRQ handler can be located in the startup file.
 * 3. In the main application, the cube framework provides the HAL_UART_IRQHandller() api nice and straight to implement.
 * 4. The above api then calls a callback() to the IRQ handler defined in this file ie. it.c
 */
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}
