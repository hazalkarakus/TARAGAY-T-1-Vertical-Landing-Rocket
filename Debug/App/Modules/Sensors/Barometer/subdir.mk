################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Modules/Sensors/Barometer/barometer.c \
../App/Modules/Sensors/Barometer/ms5611_spi.c 

OBJS += \
./App/Modules/Sensors/Barometer/barometer.o \
./App/Modules/Sensors/Barometer/ms5611_spi.o 

C_DEPS += \
./App/Modules/Sensors/Barometer/barometer.d \
./App/Modules/Sensors/Barometer/ms5611_spi.d 


# Each subdirectory must supply rules for building sources it contributes
App/Modules/Sensors/Barometer/%.o App/Modules/Sensors/Barometer/%.su App/Modules/Sensors/Barometer/%.cyclo: ../App/Modules/Sensors/Barometer/%.c App/Modules/Sensors/Barometer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../App -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Modules-2f-Sensors-2f-Barometer

clean-App-2f-Modules-2f-Sensors-2f-Barometer:
	-$(RM) ./App/Modules/Sensors/Barometer/barometer.cyclo ./App/Modules/Sensors/Barometer/barometer.d ./App/Modules/Sensors/Barometer/barometer.o ./App/Modules/Sensors/Barometer/barometer.su ./App/Modules/Sensors/Barometer/ms5611_spi.cyclo ./App/Modules/Sensors/Barometer/ms5611_spi.d ./App/Modules/Sensors/Barometer/ms5611_spi.o ./App/Modules/Sensors/Barometer/ms5611_spi.su

.PHONY: clean-App-2f-Modules-2f-Sensors-2f-Barometer

