/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: 09-Jul-2021
 *      Author: Venom
 */

#include "stm32f446xx.h"

//Private Function prototype
static void I2C_ExcecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t ReadorWrite);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - Enable or Disable the I2C Peripheral Clock
 *
 * @param[in]         - I2C_RegDef_t *pI2Cx, Register Defination Struct
 * @param[in]         - Enable or Disable
 * @param[in]         - None
 *
 * @return            - None
 *
 * @Note              - None
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	 //((pI2Cx == I2C1)? I2C1_PCLK_EN() :\(pI2Cx == I2C2)? I2C2_PCLK_EN() :\(pI2Cx == I2C3)? I2C3_PCLK_EN() :\(pI2Cx == I2C4)? I2C4_PCLK_EN() :0)
		if(EnorDi == ENABLE)
		{
			if (pI2Cx == I2C1) I2C1_PCLK_EN();
								else if (pI2Cx == I2C2) I2C2_PCLK_EN();
								else if (pI2Cx == I2C3) I2C3_PCLK_EN();
		}
		else
		{
			if (pI2Cx == I2C1) I2C1_PCLK_DI();
						else if (pI2Cx == I2C2) I2C2_PCLK_DI();
						else if (pI2Cx == I2C3) I2C3_PCLK_DI();
		}
}

/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - I2C initilaizer, This is a handler whihc initializes the actual hardware
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle I2C handle Struct
 * @param[in]         - None
 * @param[in]         - None
 *
 * @return            - None
 *
 * @Note              - None
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	/*
	 * 1. Configure the mode (Std or Fast)
	 * 2. Configure the speed of the Serial Clock
	 * 	  i. CCR and CR2 registers control the SCL
	 * 	  ii. After configuring the mode, config the peripheral clock freq in the CR2
	 * 	  iii. Configure the I2C Clock frequency, in the CCR register.
	 * 	  side note: Thumb rule is to take 50% dC for the S mode
	 * 	  			 Based on the DC selected calculate and set the Frequency in the CCR
	 * 	  			 For more info @Refer Reference_manual I2C_CCR register
	 * 3. Configure the device address
	 * 4. Enable the ACK (Important)
	 * 5. Configure the Rise time for I2C pins.
	 */
	uint32_t tempreg = 0;

	//Enable the ACK
	tempreg |= pI2CHandle->I2CConfig.I2C_AckControl & (1 << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//Helper function to get the clock Configurartion
	tempreg = 0;
	tempreg |= (I2C_GetClockConfigInfo()/1000000U) << I2C_CR2_FREQ;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//Program the Device Own addresss OVR
	tempreg |= pI2CHandle->I2CConfig.I2C_DeviceAddress << I2C_OAR1_ADD0;  //Note this config is only for the 7bit slave address.
	tempreg |= (1 << 14); // Since the data sheet it is mentioned to keep it at 1 (reserved bit).
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR Clock Control Register
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/*
		 * Thigh = CCR * Tpclk1 | Tlow = CCR * Tpclk1 | Thigh + Tlow = Tclk
		 * CCR = Tclk/ 2*(Tpclk1); | CCR = Fpclk1/2*Fclk
		 */
		ccr_value = I2C_GetClockConfigInfo() / 2 * pI2CHandle->I2CConfig.I2C_SCLSpeed;
		tempreg |= (ccr_value & (0xFFF));
	}
	else
	{
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (1 << I2C_CCR_DUTY);
		if(pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = I2C_GetClockConfigInfo() / 3 * pI2CHandle->I2CConfig.I2C_SCLSpeed;
			//Nothing Fancy. Just derived based on the formula. @Reference Manual
		}
		else
		{
			ccr_value = I2C_GetClockConfigInfo() / 25 * pI2CHandle->I2CConfig.I2C_SCLSpeed;
		}
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	/*
	 * Calculate the rise time
	 * 1. From the data sheet
	 *    Trise = Fpclk1 * Trisemax + 1
	 */
	if(pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			/*
			 * For SM trisemax = 1us
			 */
			tempreg |= (I2C_GetClockConfigInfo() / 1000000) + 1;
		}
	else
	{
		/*
		 * For FM trisemax = 300ns
		 */
		tempreg |= ((I2C_GetClockConfigInfo() * 300 )/ 1000000) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);



}
/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - Deinitialize the I2C Peripherals
 *
 * @param[in]         - GPIO_RegDef_t *pI2Cx, Register Defination Struct
 * @param[in]         - None
 * @param[in]         - None
 *
 * @return            - None
 *
 * @Note              - None
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1) I2C1_REG_RESET();
	else if (pI2Cx == I2C2) I2C2_REG_RESET();
	else if (pI2Cx == I2C3) I2C3_REG_RESET();
}
/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             - Send Data using the Master Mode
 *
 * @param[in]         - GPIO_RegDef_t *pI2Cx, Register Defination Struct
 * @param[in]         - None
 * @param[in]         - None
 *
 * @return            - None
 *
 * @Note              - None
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pxBuf, uint32_t Len, uint8_t Slaveaddress)
{
	/*
	 * 1. Generate the start condition
	 * 2. Confirm the start condition by checking the SBflag in SR1
	 * 3. Send the address of the slave with the r/~w to set (8Bit adderess)
	 * 4. Confirm the address phase is completed by checking the ADDR flag in SR1
	 * 5. Clear the ADDR flag according to its sowftware sequence
	 * 6. Send the data untill Len Becomes 0
	 * 7. When the Len becomes 0 wait for TXE=1 abd BTF=1 before generating the Stop condition
	 *    When TXE=1, BTF=1, indicates that transmission is successfull and stop could be initiated.
	 * 8. When Stop is signaled BTF is automatically cleared.
	 * side note: Untill an Event is successfull the clock is streatched.
	 */
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

	while(!I2C_StatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	I2C_ExcecuteAddressPhase(pI2CHandle->pI2Cx, Slaveaddress, I2C_WRITE);

	while(!I2C_StatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	I2C_ClearADDRFlag(pI2CHandle);

	while(Len > 0)
	{
		while(!I2C_StatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pxBuf;
		pxBuf++;
		Len--;
	}
	while(!(I2C_StatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_TXE) &&  I2C_StatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_BTF))); // while TXE and BTF is set

	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);


}
/*********************************************************************
 * @fn      		  - I2C_IT_MasterSendData
 *
 * @brief             - Send Data Initializer using the Master Mode using the Interrupt
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 * @param[in]         - uint8_t *pxBuf
 * @param[in]         - uint32_t Len
 * @param[in]		  - uint8_t Slaveaddress
 * @param[in]         - uint8_t Sr
 * @return            - busystate
 *
 * @Note              -

 */
uint8_t I2C_IT_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pxBuf, uint32_t Len, uint8_t Slaveaddress, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuff = pxBuf;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = Slaveaddress;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

		}

		return busystate;

}
/*********************************************************************
 * @fn      		  - I2C_IT_MasterReceiveData
 *
 * @brief             - Receive Data Initializer using the Master Mode using the Interrupt
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 * @param[in]         - uint8_t *pxBuf
 * @param[in]         - uint32_t Len
 * @param[in]		  - uint8_t Slaveaddress
 * @param[in]         - uint8_t Sr
 * @return            - busystate
 *
 * @Note              -

 */
uint8_t I2C_IT_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pxBuf, uint32_t Len, uint8_t Slaveaddress, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

			if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
			{
				pI2CHandle->pRxBuff = pxBuf;
				pI2CHandle->RxLen = Len;
				pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
				pI2CHandle->RxSize = Len;
				pI2CHandle->DevAddr = Slaveaddress;
				pI2CHandle->Sr = Sr;

				//Implement code to Generate START Condition
				pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

				//Implement the code to enable ITBUFEN Control Bit
				pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

				//Implement the code to enable ITEVFEN Control Bit
				pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

				//Implement the code to enable ITERREN Control Bit
				pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

			}

			return busystate;
}
/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
 *
 * @brief             - Receive Data using the Master Mode
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 * @param[in]         - uint8_t *pxBuf
 * @param[in]         - uint32_t Len
 * @param[in]		  - uint8_t Slaveaddress
 * @return            - None
 *
 * @Note              - None
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pxBuf, uint32_t Len, uint8_t Slaveaddress)
{
	/*
		 * 1. Generate the start condition
		 * 2. Confirm the start condition by checking the SBflag in SR1
		 * 3. Send the address of the slave with the r/~w to set (8Bit adderess)
		 * 4. Confirm the address phase is completed by checking the ADDR flag in SR1
		 * 5. Clear the ADDR flag according to its sowftware sequence
		 * 6. Receive the data 1byte and for More than 1 byte of data
		 *
		 * 8. When Stop is signaled BTF is automatically cleared.
		 * side note: Untill an Event is successfull the clock is streatched.
		 */
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

	while(!I2C_StatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	I2C_ExcecuteAddressPhase(pI2CHandle->pI2Cx, Slaveaddress, I2C_READ);

	while(!I2C_StatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	I2C_ClearADDRFlag(pI2CHandle);

	if(Len == 1)
	{
		/*
		 * For 1 byte of data
		 * 1. Disable the ACKIng
		 * 1. Clear the ADDR flag
		 * 2. Wait untill the RXNE becomes 1
		 * 3. Generate Stop condition
		 * 4. Read data inot the buffer
		 */
		I2C_EnableDisableACK(pI2CHandle->pI2Cx, ENABLE);
		I2C_ClearADDRFlag(pI2CHandle);
		while(!I2C_StatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
		*pxBuf = pI2CHandle->pI2Cx->DR;

	}
	else if(Len > 1)
	{
		/*
		 * 1.Clear the ADDR Flag
		 * 2.Read the data untill the Len becomes 0
		 */
		I2C_ClearADDRFlag(pI2CHandle);

		for(uint32_t i = Len; i > 0; i--)
		{
			while(!I2C_StatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2)
			{
				// Diable the ACK
				I2C_EnableDisableACK(pI2CHandle->pI2Cx, DISABLE);
				// Generate the stop condition
				pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
			}
			*pxBuf = pI2CHandle->pI2Cx->DR;
			pxBuf++;
		}
	}
	//Re-enable the ACKing
	if(pI2CHandle->I2CConfig.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_EnableDisableACK(pI2CHandle->pI2Cx, ENABLE);
	}
}
/*********************************************************************
 * @fn      		  - I2C_SlaveSendData
 *
 * @brief             - Send data in the slave mode
 *
 * @param[in]         - I2C_RegDef_t *pI2C
 * @param[in]         - uint8_t data
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}
/*********************************************************************
 * @fn      		  - I2C_SlaveReceiveData
 *
 * @brief             - Receive data in the slave mode
 *
 * @param[in]         - I2C_RegDef_t *pI2C
 * @param[in]         - None
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}
/*
 * IRQ Configuration and ISR handling
 */
/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn      		  - I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
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
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             - IRQ event handelling when an interrupt occurs
 *
 * @param[in]         - I2C_RegDef_t *pI2C
 * @param[in]         - uint8_t data
 * @param[in]         - None
 *
 * @return            -
 *
 * @Note              - None
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	/*
	 * Interrupt handling for the master and slave mode
	 * Refer the @reference manual for clear understanding of the below logic
	 * Logic tree refer pg 779.
	 */
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);
	// Interrupt handle for SB event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	if(temp1 && temp3)
	{
		//SB event handller
		// Excecute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			I2C_ExcecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_WRITE);
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			I2C_ExcecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_READ);
	}
	// Interrupt handle for ADDR event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		//ADDR is set
		I2C_ClearADDRFlag(pI2CHandle);
	}
	// Interrupt handle for BTF event
		temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		/*
		 * BTF is Set
		 * 1. if TXE=1 then SR and DR is empty, Trans
		 * 2. if RXNE=1 then SR and DR is full, Recep
		 */
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Confirm the TXE is set or not
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				/*
				 * BTF and TXE is set
				 * Job is done, so close the transmission
				 */
				// Generate a stop condition
				if(pI2CHandle->TxLen == 0)
				{
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
						pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
					//Rest all the member elements
					I2C_CloseSendData(pI2CHandle);
					//Call the application ISR here to give control to the Application
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;//Do nothing
		}

	}
	// Interrupt handle for STOPF event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		//STOPF is set
		//STOPF clear the flag (read from SR1 and write to CR1) according to the refernce manual
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that stop is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	// Interrupt handle for TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		//TXE is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Only do the above, if the device is in master mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
			{
				if(pI2CHandle->TxLen > 0)
				{
					/*
					 * 1. Load the data in to DR
					 * 2. Decrement the Txlen
					 * 3. Increment the buffer
					 */
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuff);
					pI2CHandle->TxLen--;
					pI2CHandle->pTxBuff++;
				}
			}else
			{
				//Slave mode
				//Confirm that the slave is in the transfer mode
				if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}
	// Interrupt handle for RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//If device mode is master only
			//RXNE event handller
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxSize == 1)
				{
					*pI2CHandle->pRxBuff = pI2CHandle->pI2Cx->DR;
					 pI2CHandle->RxLen--;
				}
				if(pI2CHandle->RxSize > 1)
				{
					if(pI2CHandle->RxLen == 2)
					{
						// Diable the ACK
						I2C_EnableDisableACK(pI2CHandle->pI2Cx, DISABLE);
						// Generate the stop condition
						I2C_StatusFlag(pI2CHandle->pI2Cx, I2C_CR1_STOP);
					}
					*pI2CHandle->pRxBuff = pI2CHandle->pI2Cx->DR;
					 pI2CHandle->pRxBuff++;
					 pI2CHandle->RxLen--;
				}
				if(pI2CHandle->RxLen == 0)
				{
					//Close the data reception and notify the application
					// Generate a stop condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
						pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
					//Rest all the member elements
					I2C_CloseReceiveData(pI2CHandle);
					//Call the application ISR here to give control to the Application
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

				}
			}
		}else
		{
			//Slave mode
			//Confirm if the device is in the receiver mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
		}
	}

}
/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             - Interrupt on Error
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO);
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}


/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}


/*********************************************************************HELPER FUNCTION*********************************************************************/
//Gneric PClock info function for PCLK1
uint8_t I2C_StatusFlag(I2C_RegDef_t *pI2Cx, uint32_t Flag)
{
	if(pI2Cx->SR1 & Flag) return FLAG_SET;
	else return FLAG_RESET;
}
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

}
/*
 * I2C Event Callback (ISR)
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, uint8_t Event)
{

}

uint32_t I2C_GetClockConfigInfo()
{
	uint16_t AHB_PreScaler[] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
	uint8_t APB_Prescaler[] = {2, 4, 6, 8, 16};
	uint32_t clcsrc, SysClk;
	uint32_t pclk1, temp, ahbp, apbp;

	clcsrc = (RCC->CFGR >> 2) & 0x03;
	// Clock Source
	if (clcsrc == 0) SysClk = 16000000;
	else if (clcsrc == 1) SysClk = 8000000;
	else if (clcsrc == 2) SysClk = Get_PLL_Frequency();

	// Find out the AHB prescaler
	temp = (RCC->CFGR >> 4) & 0x0F;
	if(temp < 8) ahbp = 1;
	else ahbp = AHB_PreScaler[temp-8];

	// Find out the APB perscaler
	temp = (RCC->CFGR >> 10) & 0x07;
	if(temp < 4) apbp = 1;
	else apbp = APB_Prescaler[temp-4];

	pclk1 = (SysClk/ ahbp/ apbp);

	return pclk1;
}

uint32_t Get_PLL_Frequency()
{
 //Dummy function
	return 0;
}

static void I2C_ExcecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t ReadorWrite)
{
	if(ReadorWrite == I2C_WRITE)
	{
		SlaveAddr = SlaveAddr << 1;  // Shifting the Slave address by one to  insert the read/~write bit
		SlaveAddr &= ~(1);   // Clearing writes to the slave
		pI2Cx->DR = SlaveAddr;
	}
	else
	{
		SlaveAddr = SlaveAddr << 1;  // Shifting the Slave address by one to  insert the read/~write bit
		SlaveAddr |= (1);   // Clearing writes to the slave
		pI2Cx->DR = SlaveAddr;
	}
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t ClearAddr;
	//Confirm for the device being a master mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//Device is in the master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//First task is to disable the ack
			I2C_EnableDisableACK(pI2CHandle->pI2Cx, DISABLE);
			//Clear the ADDR flag
			ClearAddr = pI2CHandle->pI2Cx->SR1;
			ClearAddr = pI2CHandle->pI2Cx->SR2;
			(void)ClearAddr;
		}
		else
		{
			//Clear the ADDR flag
			ClearAddr = pI2CHandle->pI2Cx->SR1;
			ClearAddr = pI2CHandle->pI2Cx->SR2;
			(void)ClearAddr;
		}
	}
	else
	{
		//Device is in Slave Mode
		//Clear the ADDR flag
		ClearAddr = pI2CHandle->pI2Cx->SR1;
		ClearAddr = pI2CHandle->pI2Cx->SR2;
		(void)ClearAddr;
	}

}

void I2C_EnableDisableACK(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE) pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	else pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Disable the interrupts
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuff = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2CConfig.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_EnableDisableACK(pI2CHandle->pI2Cx, ENABLE);
	}
}


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Disable the interrupts
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuff = NULL;
	pI2CHandle->TxLen = 0;

}







