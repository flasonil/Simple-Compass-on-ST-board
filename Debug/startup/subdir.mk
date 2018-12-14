################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f401xx.s 

OBJS += \
./startup/startup_stm32f401xx.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"C:/Users/Ilario Coppola/Documenti/workspaceSTM32/bussola/Utilities/STM32F401-Discovery" -I"C:/Users/Ilario Coppola/Documenti/workspaceSTM32/bussola/StdPeriph_Driver/inc" -I"C:/Users/Ilario Coppola/Documenti/workspaceSTM32/bussola/inc" -I"C:/Users/Ilario Coppola/Documenti/workspaceSTM32/bussola/CMSIS/device" -I"C:/Users/Ilario Coppola/Documenti/workspaceSTM32/bussola/CMSIS/core" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


