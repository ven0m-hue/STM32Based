/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: 7-July-2021
 *      Author: Venom
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;			/*Master or Slave mode @SPI_deviceMode*/
	uint8_t SPI_BusConfig;			/*Full Duplex or Half Duplex @SPI_BUS_Cofig*/
	uint8_t SPI_SclkSpeed;			/*Clock Speed adjust @SPI_CLK_Speed*/
	uint8_t SPI_DFF;				/*Data frame rate 8 bit or 16 bit  @SPI_DFF*/
	uint8_t SPI_CPOL; 				/*Polarity of the clock Active high or low @CPOL*/
	uint8_t SPI_CPHA;				/*Phase of the clock Positive edge or the negative edge @CPHA*/
	uint8_t SPI_SSM;				/* Slave select mode if more than one Slave to a master @SPI_SSM*/
}SPI_Config_t;

typedef struct
{
	// Pointer to hold the base address of the SPI peripheral
	SPI_RegDef_t *pSPIx; // Hold the particular base address of the SPIx
	SPI_Config_t SPIConfig; // SPI peripheral configuration
	uint8_t *pTxBuf;    //Plceholder for Tx buffer to support the interrupt mode
	uint8_t *pRxBuf;	//Plceholder for Rx buffer to support the interrupt mode
	uint32_t TxLen;		//Plceholder for TxLen to support the interrupt mode
	uint32_t RxLen;		//Plceholder for RxLen to support the interrupt mode
	uint8_t TxState;	//Plceholder for Tx  to support the interrupt mode
	uint8_t RxState;	//Plceholder for Rx  to support the interrupt mode
}SPI_Handle_t;

/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuf, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t* pRxBuf, uint32_t Len);
uint8_t SPI_IT_SendData(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuf, uint32_t Len);
uint8_t SPI_IT_ReceiveData(SPI_Handle_t *pSPIHandle,uint8_t* pRxBuf, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Auxillary
 */
uint8_t SPI_StatusFlag(SPI_RegDef_t *pSPIx, uint32_t Flag);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_CloseTransmission(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);
/*
 * SPI Event Callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle, uint8_t Event);
/*
 * Macros for the device mode
 * @SPI_deviceMode
 */

#define SPI_DEVICE_MODE_MASTER 			1
#define SPI_DEVICE_MODE_SLAVE 			0

/*
 * @SPI_BUS_Cofig
 * Note: Simplex Tx only can be achieved by disconnecting the MISO Pin from the slave.
 * whereas, RX needs to be configured via a register, since slave by default cannot send the data without a clock source
 * and Clock source only triggers when there is some data at the masters end.
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
//#define SPI_BUS_CONFIG_SIMPLEX_TX		3
#define SPI_BUS_CONFIG_SIMPLEX_RX		3


/*
 * @SPI_CLK_Speed aka BaudRate
 */
#define SPI_SCLK_SPEED_DIV2 			0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7
/*
 * @SPI_DFF
 * Data Frame Rate
 */
#define SPI_DFF_8BITS 	0
#define SPI_DFF_16BITS  1

/*
 * @CPOL
 * Selecting raising edge or falling edge
 */
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

/*
 * @CPHA
 * Configuring whether 1st edge triggered or 2nd edge triggered.
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
 * @SPI_SSM
 * Software Slave Management
 */
#define SPI_SSM_EN     1
#define SPI_SSM_DI     0

/*
 * SPI Status regiter flags
 */
#define SPI_TXE_FLAG		1
#define SPI_RXE_FLAG 		0
#define SPI_BUSY_FLAG		7

/*
 * SPI EventCallback completion
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERROR		3



#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
