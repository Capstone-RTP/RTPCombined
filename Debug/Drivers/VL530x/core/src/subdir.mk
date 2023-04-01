################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL530x/core/src/vl53l0x_api.c \
../Drivers/VL530x/core/src/vl53l0x_api_calibration.c \
../Drivers/VL530x/core/src/vl53l0x_api_core.c \
../Drivers/VL530x/core/src/vl53l0x_api_ranging.c \
../Drivers/VL530x/core/src/vl53l0x_api_strings.c 

OBJS += \
./Drivers/VL530x/core/src/vl53l0x_api.o \
./Drivers/VL530x/core/src/vl53l0x_api_calibration.o \
./Drivers/VL530x/core/src/vl53l0x_api_core.o \
./Drivers/VL530x/core/src/vl53l0x_api_ranging.o \
./Drivers/VL530x/core/src/vl53l0x_api_strings.o 

C_DEPS += \
./Drivers/VL530x/core/src/vl53l0x_api.d \
./Drivers/VL530x/core/src/vl53l0x_api_calibration.d \
./Drivers/VL530x/core/src/vl53l0x_api_core.d \
./Drivers/VL530x/core/src/vl53l0x_api_ranging.d \
./Drivers/VL530x/core/src/vl53l0x_api_strings.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL530x/core/src/%.o Drivers/VL530x/core/src/%.su: ../Drivers/VL530x/core/src/%.c Drivers/VL530x/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -I"C:/Users/kylei/STM32CubeIDE/workspace1/RTPCombined/Drivers/VL530x/core/inc" -I"C:/Users/kylei/STM32CubeIDE/workspace1/RTPCombined/Drivers/VL530x/core/src" -I"C:/Users/kylei/STM32CubeIDE/workspace1/RTPCombined/Drivers/VL530x/platform/inc" -I"C:/Users/kylei/STM32CubeIDE/workspace1/RTPCombined/Drivers/VL530x/platform/src" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL530x-2f-core-2f-src

clean-Drivers-2f-VL530x-2f-core-2f-src:
	-$(RM) ./Drivers/VL530x/core/src/vl53l0x_api.d ./Drivers/VL530x/core/src/vl53l0x_api.o ./Drivers/VL530x/core/src/vl53l0x_api.su ./Drivers/VL530x/core/src/vl53l0x_api_calibration.d ./Drivers/VL530x/core/src/vl53l0x_api_calibration.o ./Drivers/VL530x/core/src/vl53l0x_api_calibration.su ./Drivers/VL530x/core/src/vl53l0x_api_core.d ./Drivers/VL530x/core/src/vl53l0x_api_core.o ./Drivers/VL530x/core/src/vl53l0x_api_core.su ./Drivers/VL530x/core/src/vl53l0x_api_ranging.d ./Drivers/VL530x/core/src/vl53l0x_api_ranging.o ./Drivers/VL530x/core/src/vl53l0x_api_ranging.su ./Drivers/VL530x/core/src/vl53l0x_api_strings.d ./Drivers/VL530x/core/src/vl53l0x_api_strings.o ./Drivers/VL530x/core/src/vl53l0x_api_strings.su

.PHONY: clean-Drivers-2f-VL530x-2f-core-2f-src

