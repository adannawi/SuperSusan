################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f410rx.s 

OBJS += \
./startup/startup_stm32f410rx.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo %cd%
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"C:/Users/NexusJr/eclipse-workspace/SuperSusan/inc" -I"C:/Users/NexusJr/eclipse-workspace/SuperSusan/CMSIS/core" -I"C:/Users/NexusJr/eclipse-workspace/SuperSusan/CMSIS/device" -I"C:/Users/NexusJr/eclipse-workspace/SuperSusan/HAL_Driver/Inc/Legacy" -I"C:/Users/NexusJr/eclipse-workspace/SuperSusan/HAL_Driver/Inc" -I"C:/Users/NexusJr/eclipse-workspace/SuperSusan/Utilities/STM32F4xx-Nucleo" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


