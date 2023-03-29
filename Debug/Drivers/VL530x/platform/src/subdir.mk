################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL530x/platform/src/vl53l0x_platform.c \
../Drivers/VL530x/platform/src/vl53l0x_platform_log.c 

OBJS += \
./Drivers/VL530x/platform/src/vl53l0x_platform.o \
./Drivers/VL530x/platform/src/vl53l0x_platform_log.o 

C_DEPS += \
./Drivers/VL530x/platform/src/vl53l0x_platform.d \
./Drivers/VL530x/platform/src/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL530x/platform/src/%.o Drivers/VL530x/platform/src/%.su: ../Drivers/VL530x/platform/src/%.c Drivers/VL530x/platform/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -I"C:/Code/STM32/RTPCombined/Drivers/VL530x/core/inc" -I"C:/Code/STM32/RTPCombined/Drivers/VL530x/core/src" -I"C:/Code/STM32/RTPCombined/Drivers/VL530x/platform/inc" -I"C:/Code/STM32/RTPCombined/Drivers/VL530x/platform/src" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL530x-2f-platform-2f-src

clean-Drivers-2f-VL530x-2f-platform-2f-src:
	-$(RM) ./Drivers/VL530x/platform/src/vl53l0x_platform.d ./Drivers/VL530x/platform/src/vl53l0x_platform.o ./Drivers/VL530x/platform/src/vl53l0x_platform.su ./Drivers/VL530x/platform/src/vl53l0x_platform_log.d ./Drivers/VL530x/platform/src/vl53l0x_platform_log.o ./Drivers/VL530x/platform/src/vl53l0x_platform_log.su

.PHONY: clean-Drivers-2f-VL530x-2f-platform-2f-src

