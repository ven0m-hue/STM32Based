################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/INT_BTN_LED.c 

OBJS += \
./Src/INT_BTN_LED.o 

C_DEPS += \
./Src/INT_BTN_LED.d 


# Each subdirectory must supply rules for building sources it contributes
Src/INT_BTN_LED.o: ../Src/INT_BTN_LED.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I"F:/STM32Workspace/stm32f4xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/INT_BTN_LED.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

