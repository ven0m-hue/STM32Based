/*
 * INT_BTN_LED.c
 *
 *  Created on: 21-Jun-2021
 *      Author: 91900
 */

#include "stm32f446xx.h"
#include <string.h>
//#include "stm32f4xx_gpio_driver.h"
void delay(uint32_t period)
{
	for(uint32_t i = 0; i< period; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioBtn;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioLed));

	GpioLed.pGPIOx = GPIOA;
	GpioBtn.pGPIOx = GPIOC;
	//Led INIT
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	//Button INIT
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	//IRQ config
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);


	while(1);
}


void EXTI15_10_IRQHandler(void)
	{
		delay(500000);  // 500mS delay
		GPIO_IRQHandling(GPIO_PIN_NO_13);
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
	}
