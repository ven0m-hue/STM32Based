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

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	/*
	 * here we are going to do the low level inits. of the USART2 peripheral
	 * 1. Enable the clock of USART2 peripheral and GPIOA peripheral
	 * 2. Do the pin muxing config
	 * 3. Enable the IRQ and set up the priority
	 */
	GPIO_InitTypeDef gpio_uart;

	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	 gpio_uart.Pin = GPIO_PIN_2;//UART2_TX
	 gpio_uart.Mode =GPIO_MODE_AF_PP;
	 gpio_uart.Pull = GPIO_PULLUP;
	 gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;
	 gpio_uart.Alternate =  GPIO_AF7_USART2; // Alternate functionality for TX_RX
	 HAL_GPIO_Init(GPIOA,&gpio_uart);

	 gpio_uart.Pin = GPIO_PIN_3;//UART2_RX
	 HAL_GPIO_Init(GPIOA,&gpio_uart);

	 HAL_NVIC_EnableIRQ(USART2_IRQn);
	 HAL_NVIC_SetPriority(USART2_IRQn,15,0);
}






