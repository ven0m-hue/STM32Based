/*
 * main.c
 *
 *  Created on: Jun 22, 2021
 *      Author: Venom
 */


#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>
void SystemClockConfig(void);
void UART2_Init(void);
void Error_handler(void);
uint8_t ConvertoCap(uint8_t data);

UART_HandleTypeDef huart2;

char* user_data = "The VA-U\r\n";

uint8_t received_data;

_Bool reception_complete;

uint8_t data_buffer[100];
uint32_t count = 0;

int main()
{
	//Inits
	HAL_Init();
	SystemClockConfig();
	UART2_Init();

	uint16_t data_len = strlen(user_data);

	HAL_UART_Transmit(&huart2, (uint8_t*)user_data, data_len, HAL_MAX_DELAY);
	//User Application
	while (reception_complete == false) HAL_UART_Receive_IT(&huart2, &received_data, 1);

	while(1);
	return 0;
}

void SystemClockConfig(void)
{

}

void UART2_Init(void)
{
	/*
	 * High level initialization
	 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;

	if(HAL_UART_Init(&huart2) != HAL_OK) Error_handler();  // If there is a problem

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (received_data == '\r')
  {
		  reception_complete = true;
		  data_buffer[count++] = '\r';
		  HAL_UART_Transmit(huart, data_buffer, count, HAL_MAX_DELAY);
  }
  else
  {
	  data_buffer[count++] = ConvertoCap(received_data);
  }
}

uint8_t ConvertoCap(uint8_t data)
{
	if (data >= 'a' && data<='z') data = data - ('a' - 'A');

	return data;
}

void Error_handler(void)
{
	while(1);
}
