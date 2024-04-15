################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lcd16x2/lcd16x2.c 

OBJS += \
./lcd16x2/lcd16x2.o 

C_DEPS += \
./lcd16x2/lcd16x2.d 


# Each subdirectory must supply rules for building sources it contributes
lcd16x2/%.o lcd16x2/%.su lcd16x2/%.cyclo: ../lcd16x2/%.c lcd16x2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F031x6 -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../lcd16x2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-lcd16x2

clean-lcd16x2:
	-$(RM) ./lcd16x2/lcd16x2.cyclo ./lcd16x2/lcd16x2.d ./lcd16x2/lcd16x2.o ./lcd16x2/lcd16x2.su

.PHONY: clean-lcd16x2

