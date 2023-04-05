################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/hx711.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/serialFromPC.c \
../Core/Src/stepperControl.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c \
../Core/Src/zeroing.c 

OBJS += \
./Core/Src/hx711.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/serialFromPC.o \
./Core/Src/stepperControl.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/zeroing.o 

C_DEPS += \
./Core/Src/hx711.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/serialFromPC.d \
./Core/Src/stepperControl.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d \
./Core/Src/zeroing.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I"C:/Code/STM32/RTPCombined/Drivers/VL53L0X/core/inc" -I"C:/Code/STM32/RTPCombined/Drivers/VL53L0X/core/src" -I"C:/Code/STM32/RTPCombined/Drivers/VL53L0X/platform/inc" -I"C:/Code/STM32/RTPCombined/Drivers/VL53L0X/platform/src" -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/hx711.d ./Core/Src/hx711.o ./Core/Src/hx711.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/serialFromPC.d ./Core/Src/serialFromPC.o ./Core/Src/serialFromPC.su ./Core/Src/stepperControl.d ./Core/Src/stepperControl.o ./Core/Src/stepperControl.su ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su ./Core/Src/zeroing.d ./Core/Src/zeroing.o ./Core/Src/zeroing.su

.PHONY: clean-Core-2f-Src

