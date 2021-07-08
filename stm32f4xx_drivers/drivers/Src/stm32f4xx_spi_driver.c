/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: 23-Jun-2021
 *      Author: 91900
 */

#include "stm32f446xx.h"
//Helper function prototype
static void __SPI_TXE_Interrupt_Handle(SPI_Handle_t *pHandle);
static void __SPI_RXE_Interrupt_Handle(SPI_Handle_t *pHandle);
static void __SPI_OVR_Interrupt_Handle(SPI_Handle_t *pHandle);
/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - Enable or Disable the SPI Peripheral Clock
 *
 * @param[in]         - SPI_RegDef_t *pSPIx, Register Defination Struct
 * @param[in]         - Enable or Disable
 * @param[in]         - None
 *
 * @return            - None
 *
 * @Note              - None
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	 //((pSPIx == SPI1)? SPI1_PCLK_EN() :\(pSPIx == SPI2)? SPI2_PCLK_EN() :\(pSPIx == SPI3)? SPI3_PCLK_EN() :\(pSPIx == SPI4)? SPI4_PCLK_EN() :0)
	if(EnorDi == ENABLE)
	{
		if (pSPIx == SPI1) SPI1_PCLK_EN();
							else if (pSPIx == SPI2) SPI2_PCLK_EN();
							else if (pSPIx == SPI3) SPI3_PCLK_EN();
							else if (pSPIx == SPI4) SPI4_PCLK_EN();
	}
	else
	{
		if (pSPIx == SPI1) SPI1_PCLK_DI();
					else if (pSPIx == SPI2) SPI2_PCLK_DI();
					else if (pSPIx == SPI3) SPI3_PCLK_DI();
					else if (pSPIx == SPI4) SPI4_PCLK_DI();
	}
}
/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - SPI initilaizer, This is a handler whihc initializes the actual hardware
 *
 * @param[in]         - SPI_Handle_t *pSPIHandle SPI handle Struct
 * @param[in]         - None
 * @param[in]         - None
 *
 * @return            - None
 *
 * @Note              - None
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/* Enable the peripheral clock
	 * 1.Configure the mode of the SPI_CR1
	 */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempReg = 0;
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	/*
	 * 2.Configure the busConfig
	 *  Clear the BIDI to set the bus as FD
	 *  Set the  BIDI to set the bus as HD
	 *  For Simplex RX Clear BIDI and Set RXONLY bit
	 *  For Simplex TX Simply disconnect the MOSI
	 */
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) tempReg &= ~(1 << SPI_CR1_BIDIMODE);

	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) tempReg |= (1 << SPI_CR1_BIDIMODE);

	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
	{
		//BIDI mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		//RX only
		tempReg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. Set the baud rate
	tempReg |=  pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.Configure the DFF
	tempReg |=  pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5.Configure CPOL
	tempReg |=  pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	//6. Configure CPHL
	tempReg |=  pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	//7. Configure SSM
	tempReg |=  pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// Now assign the tempreg to the actual intended register from the SPI regdef
	pSPIHandle->pSPIx->CR1 = tempReg;


}
/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - Deinitialize the SPI Peripherals
 *
 * @param[in]         - GPIO_RegDef_t *pSPIx, Register Defination Struct
 * @param[in]         - None
 * @param[in]         - None
 *
 * @return            - None
 *
 * @Note              - None
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1) SPI1_REG_RESET();
	else if (pSPIx == SPI2) SPI2_REG_RESET();
	else if (pSPIx == SPI3) SPI3_REG_RESET();
	else if (pSPIx == SPI4) SPI4_REG_RESET();
}
/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - SPI send data to the slave
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuf, uint32_t Len)
{
	/*
	 * 1. Check the lenght
	 * 2. If len!=0 Wait untill Tx buffer is empty. To check that, in Status Reg TXE should be Set
	 * 3. If TxE is set, Tx buffer is empty
	 * 4. Check the DFF flag if set 16bit DFF else 8 Bit
	 * 5. And Load DR and increment the buffer address
	 * 6. Decrement the length untill 0
	 * side note: if 16 bit DFF then Len-2;
	 */
	while(Len > 0)
	{
		while(!(pSPIx->SR & (1 << SPI_TXE_FLAG )));

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			pSPIx->DR = *((uint16_t *)pTxBuf); // ((*(pTxBuf) << 8) | *(pTxBuf))
			Len -=2;
			(uint16_t *)pTxBuf++;
		}
		else
		{
			pSPIx->DR = *pTxBuf;
			Len--;
			pTxBuf++;
		}

	}
}
/*********************************************************************
 * @fn      		  - SPI_IT_SendData
 *
 * @brief             - SPI send data to the slave
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            - uint8_t state
 *
 * @Note              - None
 */
uint8_t SPI_IT_SendData(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuf, uint32_t Len)
{
	/*
	 * 1. Save the Tx buffer address and Len information in the global space
	 * 2. Mark the SPI state busy so that no other peripheral can take over
	 * 3. Enable the TXIE pin (which initiates the intertupt when TXE flagis set.)
	 * 4.
	 */
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxBuf = pTxBuf;
		pSPIHandle->TxLen = Len;

		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}
/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - Recieve data from the slave
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t* pRxBuf, uint32_t Len)
{
	/*
		 * 1. Check the lenght
		 * 2. If len!=0 Wait untill Rx buffer is full. To check that, in Status Reg RXE should be Set
		 * 3. If RxE is set, Rx buffer is full
		 * 4. Check the DFF flag if set 16bit DFF else 8 Bit DFF
		 * 5. Read from the  Data Register and increment the buffer address
		 * 6. Decrement the length untill 0
		 * side note: if 16 bit DFF then Len-2;
		 */
	while(Len > 0)
	{
		while(!(pSPIx->SR & (1<< SPI_RXE_FLAG)));

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			*((uint16_t *)pRxBuf) = pSPIx->DR; // ((*(pTxBuf) << 8) | *(pTxBuf))
			Len -=2;
			(uint16_t *)pRxBuf++;
		}
		else
		{
			*pRxBuf = pSPIx->DR;
			Len--;
			pRxBuf++;
		}
	}
}
/*********************************************************************
 * @fn      		  - SPI_IT_ReceiveData
 *
 * @brief             - Recieve data from the slave
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            - uint8_t state
 *
 * @Note              - None
 */
uint8_t SPI_IT_ReceiveData(SPI_Handle_t *pSPIHandle,uint8_t* pRxBuf, uint32_t Len)
{
	/*
		 * 1. Save the Rx buffer address and Len information in the global space
		 * 2. Mark the SPI state busy so that no other peripheral can take over
		 * 3. Enable the RXNEIE pin (which initiates the intertupt when TXE flagis set.)
		 */
		uint8_t state = pSPIHandle->RxState;
		if(state != SPI_BUSY_IN_TX)
		{
			pSPIHandle->pRxBuf = pRxBuf;
			pSPIHandle->RxLen = Len;

			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		}

		return state;
}
/*
 * IRQ Configuration and ISR handling
 */
/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	/*
		 * 1. Math for calculating the Priority register.
		 * 2. first lets find out the ipr register.
		 * 3. There are 60 Banks of Priority register as per the ARM Cortex M4 Doc. (divided into 4 sections)
		 * 4. Based on the given IRQ number, do -> IRQNUM/4 to select the register bank
		 * 5. To select the particular ITQNUM do-> IRQNUM%4 to get the IRQPriority register and multiply them by 8 as they are 8bits (2 bytes wide.)
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
/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	/*
	 * Error handler handles the different Inrrupt cases. Following could be the cases
	 * 1. TXE interrupt
	 * 2. RXNE interrupt
	 * 3. Overflow interrupt
	 * side note: For more information @ref Reference_manual
	 */
	uint8_t temp1, temp2;
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		__SPI_TXE_Interrupt_Handle(pHandle);
	}

	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		__SPI_RXE_Interrupt_Handle(pHandle);
	}

	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		__SPI_OVR_Interrupt_Handle(pHandle);
	}

}
/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - Enables the Internal Slave Select, Only in effect when SSM is enabled
 * 						Controls the value of NSS pin.
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - When slave is Simplex Transmit only, Always config Enable the SSI.
 * 						Else master might go into slave mode automatically.
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE) pSPIx->CR1 |= (1 << SPI_CR1_SSI);

	else pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);

}
/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - Slave Select Output Enable
 * 						Output is disabled for multi-master slave config
 * 						Enabled for single master and multi slave config
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE) pSPIx->CR2 |= (1 << SPI_CR2_SSOE);

	else pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);

}
/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE) pSPIx->CR1 |= (1 << SPI_CR1_SPE);

		else pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

/*********************************************************************
 * @fn      		  - SPI_StatusFlag
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
uint8_t SPI_StatusFlag(SPI_RegDef_t *pSPIx, uint32_t Flag)
{
	if(pSPIx->SR & Flag) return FLAG_SET;
	else return FLAG_RESET;
}
/*********************************************************************Helper Functions*********************************************************************/

static void __SPI_TXE_Interrupt_Handle(SPI_Handle_t *pHandle)
{
			if(pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
				pHandle->pSPIx->DR = *((uint16_t *)pHandle->pTxBuf); // ((*(pTxBuf) << 8) | *(pTxBuf))
				pHandle->TxLen--;
				pHandle->TxLen--;
				(uint16_t *)pHandle->pTxBuf++;
			}
			else
			{
				pHandle->pSPIx->DR = *pHandle->pTxBuf;
				pHandle->TxLen--;
				pHandle->pTxBuf++;
			}

			if(! pHandle->TxLen)
			{
				/*
				 * Txlen is 0 close the SPI trnasmission
				 * Inform that Tx is over
				 */
				SPI_CloseTransmission(pHandle);
				SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);
			}


}
static void __SPI_RXE_Interrupt_Handle(SPI_Handle_t *pHandle)
{

	if(pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		*((uint16_t *)pHandle->pRxBuf) = pHandle->pSPIx->DR; // ((*(pTxBuf) << 8) | *(pTxBuf))
		pHandle->RxLen--;
		pHandle->RxLen--;
		(uint16_t *)pHandle->pRxBuf++;
	}
	else
	{
		*pHandle->pRxBuf = pHandle->pSPIx->DR;
		pHandle->RxLen--;
		pHandle->pRxBuf++;
	}

	if(! pHandle->RxLen)
	{
		/*
		 * Rxlen is 0 close the SPI trnasmission
		 * Inform that Rx is over
		 */
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
	}
}
static void __SPI_OVR_Interrupt_Handle(SPI_Handle_t *pHandle)
{
	/*
	 * Clear the ovr flag
	 * 	Clear the read access to the SPI_DR followed by read access tp SPI_SR
	 * Infrom the application
	 */
	uint8_t temp;
	// The application might need the data, so only if its not busy checkout the data_register
	if(pHandle->TxState != SPI_BUSY_IN_TX)
	{

		temp = pHandle->pSPIx->DR;
		temp = pHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);

}


void SPI_CloseTransmission(SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE); // clear the interrupt
	pHandle->pTxBuf = NULL;  // Clear the buffer
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE); // clear the interrupt
	pHandle->pRxBuf = NULL;  // Clear the buffer
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle, uint8_t Event)
{
	//This is the weak implementation. The application may override this function
}

