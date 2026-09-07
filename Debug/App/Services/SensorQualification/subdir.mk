################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Services/SensorQualification/sensor_qualification.c 

OBJS += \
./App/Services/SensorQualification/sensor_qualification.o 

C_DEPS += \
./App/Services/SensorQualification/sensor_qualification.d 


# Each subdirectory must supply rules for building sources it contributes
App/Services/SensorQualification/%.o App/Services/SensorQualification/%.su App/Services/SensorQualification/%.cyclo: ../App/Services/SensorQualification/%.c App/Services/SensorQualification/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../App -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Services-2f-SensorQualification

clean-App-2f-Services-2f-SensorQualification:
	-$(RM) ./App/Services/SensorQualification/sensor_qualification.cyclo ./App/Services/SensorQualification/sensor_qualification.d ./App/Services/SensorQualification/sensor_qualification.o ./App/Services/SensorQualification/sensor_qualification.su

.PHONY: clean-App-2f-Services-2f-SensorQualification

