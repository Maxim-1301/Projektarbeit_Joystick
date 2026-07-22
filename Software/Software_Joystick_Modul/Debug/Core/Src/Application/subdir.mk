################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Application/communication_module.c \
../Core/Src/Application/display_i2c.c \
../Core/Src/Application/joystick.c \
../Core/Src/Application/motor_rs485.c 

OBJS += \
./Core/Src/Application/communication_module.o \
./Core/Src/Application/display_i2c.o \
./Core/Src/Application/joystick.o \
./Core/Src/Application/motor_rs485.o 

C_DEPS += \
./Core/Src/Application/communication_module.d \
./Core/Src/Application/display_i2c.d \
./Core/Src/Application/joystick.d \
./Core/Src/Application/motor_rs485.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Application/%.o Core/Src/Application/%.su Core/Src/Application/%.cyclo: ../Core/Src/Application/%.c Core/Src/Application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Application

clean-Core-2f-Src-2f-Application:
	-$(RM) ./Core/Src/Application/communication_module.cyclo ./Core/Src/Application/communication_module.d ./Core/Src/Application/communication_module.o ./Core/Src/Application/communication_module.su ./Core/Src/Application/display_i2c.cyclo ./Core/Src/Application/display_i2c.d ./Core/Src/Application/display_i2c.o ./Core/Src/Application/display_i2c.su ./Core/Src/Application/joystick.cyclo ./Core/Src/Application/joystick.d ./Core/Src/Application/joystick.o ./Core/Src/Application/joystick.su ./Core/Src/Application/motor_rs485.cyclo ./Core/Src/Application/motor_rs485.d ./Core/Src/Application/motor_rs485.o ./Core/Src/Application/motor_rs485.su

.PHONY: clean-Core-2f-Src-2f-Application

