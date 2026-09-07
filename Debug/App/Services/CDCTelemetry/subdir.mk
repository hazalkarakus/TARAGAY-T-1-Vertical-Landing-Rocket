################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Services/CDCTelemetry/cdc_telemetry.c 

OBJS += \
./App/Services/CDCTelemetry/cdc_telemetry.o 

C_DEPS += \
./App/Services/CDCTelemetry/cdc_telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
App/Services/CDCTelemetry/%.o App/Services/CDCTelemetry/%.su App/Services/CDCTelemetry/%.cyclo: ../App/Services/CDCTelemetry/%.c App/Services/CDCTelemetry/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../App -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Services-2f-CDCTelemetry

clean-App-2f-Services-2f-CDCTelemetry:
	-$(RM) ./App/Services/CDCTelemetry/cdc_telemetry.cyclo ./App/Services/CDCTelemetry/cdc_telemetry.d ./App/Services/CDCTelemetry/cdc_telemetry.o ./App/Services/CDCTelemetry/cdc_telemetry.su

.PHONY: clean-App-2f-Services-2f-CDCTelemetry

