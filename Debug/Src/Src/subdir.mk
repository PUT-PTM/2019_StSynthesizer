################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Src/MY_CS43L22.c \
../Src/Src/main.c \
../Src/Src/sounds.c \
../Src/Src/stm32f4xx_hal_msp.c \
../Src/Src/stm32f4xx_it.c \
../Src/Src/system_stm32f4xx.c 

OBJS += \
./Src/Src/MY_CS43L22.o \
./Src/Src/main.o \
./Src/Src/sounds.o \
./Src/Src/stm32f4xx_hal_msp.o \
./Src/Src/stm32f4xx_it.o \
./Src/Src/system_stm32f4xx.o 

C_DEPS += \
./Src/Src/MY_CS43L22.d \
./Src/Src/main.d \
./Src/Src/sounds.d \
./Src/Src/stm32f4xx_hal_msp.d \
./Src/Src/stm32f4xx_it.d \
./Src/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Src/%.o: ../Src/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/kusio/Documents/GitHub/2019_StSynthesizer/Inc" -I"C:/Users/kusio/Documents/GitHub/2019_StSynthesizer/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/kusio/Documents/GitHub/2019_StSynthesizer/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kusio/Documents/GitHub/2019_StSynthesizer/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/kusio/Documents/GitHub/2019_StSynthesizer/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


