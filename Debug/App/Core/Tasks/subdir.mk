################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Core/Tasks/app_tasks.c 

OBJS += \
./App/Core/Tasks/app_tasks.o 

C_DEPS += \
./App/Core/Tasks/app_tasks.d 


# Each subdirectory must supply rules for building sources it contributes
App/Core/Tasks/%.o App/Core/Tasks/%.su App/Core/Tasks/%.cyclo: ../App/Core/Tasks/%.c App/Core/Tasks/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../App -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Core-2f-Tasks

clean-App-2f-Core-2f-Tasks:
	-$(RM) ./App/Core/Tasks/app_tasks.cyclo ./App/Core/Tasks/app_tasks.d ./App/Core/Tasks/app_tasks.o ./App/Core/Tasks/app_tasks.su

.PHONY: clean-App-2f-Core-2f-Tasks

