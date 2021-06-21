/*
 * 02Button_Led.c
 *
 *  Created on: 20-Jun-2021
 *      Author: 91900
 */

#include "stm32f446xx.h"
//#include "stm32f4xx_gpio_driver.h"
void delay(uint32_t period)
{
	for(uint32_t i = 0; i< period; i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioBtn;

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
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);



	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			delay(500000);
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
			delay(500000);
		}
	}
	return 0;
}
