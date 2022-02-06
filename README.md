# STM32Based
STM32-NUCLEOF446RE Microcontroller Based drivers. Low level and High level drivers from scractch. 
I'm implementing as I'm learning. In the process, so far I've developed,
1. GPIO Driver.
2. SPI Driver.
3. I2C Driver.
4. Not soon enough.

STM32-NUCLEOF446RE Microcontroller Based application programs. Low level and High level initialization using the STMCubeMx Hardware Abstraction layer(HAL) API. 
1. USART_RX_TX. (with and without interrupt)
2. STM32 Clock configuration. HSE clock up to 16MHz, HSI up to 8MHz and all using the Phase Locked Loop (PLL) engine, clock upto 180MHz(Peak). 
3. STM32 Timers. BASIC, GENERAL, ADVANCED TIMERS with interrupts.
4. STM32 PWM for LED, PWM in gneral.
5. DMA applications. UARTtoSRAM, SRAM1toSRAM2, ADCtoSRAM
6. Added UART Interrupt. For multiple interrupts. 
   (Also resolved mutiple bytes UART Interrupt recieve.)
