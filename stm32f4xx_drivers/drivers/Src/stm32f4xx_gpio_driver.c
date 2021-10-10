/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: 19-Jun-2021
 *      Author: 91900
 */
#include "stm32f4xx_gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if (pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE) GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF) GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG) GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH) GPIOH_PCLK_EN();
	}

	else{
		if(EnorDi == DISABLE){
				if (pGPIOx == GPIOA) GPIOA_PCLK_DI();
				else if (pGPIOx == GPIOB) GPIOB_PCLK_DI();
				else if (pGPIOx == GPIOC) GPIOC_PCLK_DI();
				else if (pGPIOx == GPIOD) GPIOD_PCLK_DI();
				else if (pGPIOx == GPIOE) GPIOE_PCLK_DI();
				else if (pGPIOx == GPIOF) GPIOF_PCLK_DI();
				else if (pGPIOx == GPIOG) GPIOG_PCLK_DI();
				else if (pGPIOx == GPIOH) GPIOH_PCLK_DI();
			}
	}

}
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Configure the pinmode, speed, output type, alternate functionality
 *
 * @param[in]         - gpio pin handle
 * @param[in]         - none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - Be careful while assigning a bit to the register, do not directly assign it, use bitwise operators.
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;
	//1. Config the mode of the gpio
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << temp);
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	// Otherwise interrupt mode
	else{
			/*
			 * GPIO Interrupt Configuration
			 * 1. Pin must be a input configured
			 * 2. Check or set whether Raising/Falling/Both
			 * 3. Enable the interrupt delivery from the peripheral to the processor (Processor side)
			 * 4. Identify the IRQ number
			 * 5. Configure the IRQ priority (Processor side)
			 * 6. Enable the interrupt reception (Processor side )
			 * 7. Implement the handler
			 */
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
			{
				EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				// Clear the corresponding RTSR bit
				EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
			else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
			{
				EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				// Clear the corresponding FTSR bit
				EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
			else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
			{
				EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}

			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] = portcode << (4 * temp2);

			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x33 << temp);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;
	temp = 0;

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x33 << temp);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << temp);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint32_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << temp);
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
		temp = 0;
	}


}
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA) GPIOA_REG_RESET();
			else if (pGPIOx == GPIOB) GPIOB_REG_RESET();
			else if (pGPIOx == GPIOC) GPIOC_REG_RESET();
			else if (pGPIOx == GPIOD) GPIOD_REG_RESET();
			else if (pGPIOx == GPIOE) GPIOE_REG_RESET();
			else if (pGPIOx == GPIOF) GPIOF_REG_RESET();
			else if (pGPIOx == GPIOG) GPIOG_REG_RESET();
			else if (pGPIOx == GPIOH) GPIOH_REG_RESET();
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - GPIO read from input pin
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - uint8_t
 *
 * @Note              -
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber & 0x00000001);
	return value;
}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - GPIO read from input port
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - uint16_t
 *
 * @Note              -
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
		value = (uint16_t)(pGPIOx->IDR);
		return value;
}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromOutputPin
 *
 * @brief             - GPIO read from Output pin
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_RESET) pGPIOx->ODR |= (1 << PinNumber);
	else pGPIOx->ODR &= ~(1 << PinNumber);
}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromOutputPort
 *
 * @brief             - GPIO read from Output port
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	if(Value) pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<< PinNumber);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
			}
		}

}
/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - Sets the IRQ Priority based on the IRQ Number
 *
 * @param[in]         - IRQNumber
 * @param[in]         - ITQPriority
 * @param[in]         - None
 *
 * @return            - None
 *
 * @Note              - Be careful with the setting of the priority register.
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	/*
	 * 1. Math for calculating the Priority register.
	 * 2. first lets find out the ipr register.
	 * 3. There are 60 Banks of Priority register as per the ARM Cortex M4 Doc. (divided into 4 sections)
	 * 4. Based on the given IRQ number, do -> IRQNUM/4 to select the register bank
	 * 5. To select the particular ITQNUM do-> IRQNUM%4 to get the IRQPriority register and multiply them by 8 as they are 8bits (1 byte wide.)
	 * 6. Out of those 8 bits only higher 4 bits are valid and lower 4 bits are reserved.
	 * 7. for eg. 237 ITQ NUM -> 237/4 is [0-]59th register bank
	 * 						  -> 237%4 is [0-]1 ie 2nd section. and mul by 8
	 * 						  -> | V | v | v | v | r | r | r | r | v- valid r- reserved (2bytes ie. 8bits)
	 * 						  ->  shift it to the higher valid bits
	 */

		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section  = IRQNumber %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

		*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}


void GPIO_IRQHandling(uint8_t PinNumber)
{
	 //Clear the exti pr register and the processor starts processing the interrupt.
	if(EXTI->PR & (1 << PinNumber)) EXTI->PR |= (1 << PinNumber);
}

