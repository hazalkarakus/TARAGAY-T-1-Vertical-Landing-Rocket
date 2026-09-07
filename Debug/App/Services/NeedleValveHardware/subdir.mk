################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Services/NeedleValveHardware/needle_valve_hw.c 

OBJS += \
./App/Services/NeedleValveHardware/needle_valve_hw.o 

C_DEPS += \
./App/Services/NeedleValveHardware/needle_valve_hw.d 


# Each subdirectory must supply rules for building sources it contributes
App/Services/NeedleValveHardware/%.o App/Services/NeedleValveHardware/%.su App/Services/NeedleValveHardware/%.cyclo: ../App/Services/NeedleValveHardware/%.c App/Services/NeedleValveHardware/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../App -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Services-2f-NeedleValveHardware

clean-App-2f-Services-2f-NeedleValveHardware:
	-$(RM) ./App/Services/NeedleValveHardware/needle_valve_hw.cyclo ./App/Services/NeedleValveHardware/needle_valve_hw.d ./App/Services/NeedleValveHardware/needle_valve_hw.o ./App/Services/NeedleValveHardware/needle_valve_hw.su

.PHONY: clean-App-2f-Services-2f-NeedleValveHardware

