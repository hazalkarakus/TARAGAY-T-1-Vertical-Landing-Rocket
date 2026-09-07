################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari.c \
../App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_data.c \
../App/Modules/Control/GeneratedFlightControl/Generated/rtGetInf.c \
../App/Modules/Control/GeneratedFlightControl/Generated/rtGetNaN.c \
../App/Modules/Control/GeneratedFlightControl/Generated/rt_nonfinite.c 

OBJS += \
./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari.o \
./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_data.o \
./App/Modules/Control/GeneratedFlightControl/Generated/rtGetInf.o \
./App/Modules/Control/GeneratedFlightControl/Generated/rtGetNaN.o \
./App/Modules/Control/GeneratedFlightControl/Generated/rt_nonfinite.o 

C_DEPS += \
./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari.d \
./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_data.d \
./App/Modules/Control/GeneratedFlightControl/Generated/rtGetInf.d \
./App/Modules/Control/GeneratedFlightControl/Generated/rtGetNaN.d \
./App/Modules/Control/GeneratedFlightControl/Generated/rt_nonfinite.d 


# Each subdirectory must supply rules for building sources it contributes
App/Modules/Control/GeneratedFlightControl/Generated/%.o App/Modules/Control/GeneratedFlightControl/Generated/%.su App/Modules/Control/GeneratedFlightControl/Generated/%.cyclo: ../App/Modules/Control/GeneratedFlightControl/Generated/%.c App/Modules/Control/GeneratedFlightControl/Generated/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../App -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Modules-2f-Control-2f-GeneratedFlightControl-2f-Generated

clean-App-2f-Modules-2f-Control-2f-GeneratedFlightControl-2f-Generated:
	-$(RM) ./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari.cyclo ./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari.d ./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari.o ./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari.su ./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_data.cyclo ./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_data.d ./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_data.o ./App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_data.su ./App/Modules/Control/GeneratedFlightControl/Generated/rtGetInf.cyclo ./App/Modules/Control/GeneratedFlightControl/Generated/rtGetInf.d ./App/Modules/Control/GeneratedFlightControl/Generated/rtGetInf.o ./App/Modules/Control/GeneratedFlightControl/Generated/rtGetInf.su ./App/Modules/Control/GeneratedFlightControl/Generated/rtGetNaN.cyclo ./App/Modules/Control/GeneratedFlightControl/Generated/rtGetNaN.d ./App/Modules/Control/GeneratedFlightControl/Generated/rtGetNaN.o ./App/Modules/Control/GeneratedFlightControl/Generated/rtGetNaN.su ./App/Modules/Control/GeneratedFlightControl/Generated/rt_nonfinite.cyclo ./App/Modules/Control/GeneratedFlightControl/Generated/rt_nonfinite.d ./App/Modules/Control/GeneratedFlightControl/Generated/rt_nonfinite.o ./App/Modules/Control/GeneratedFlightControl/Generated/rt_nonfinite.su

.PHONY: clean-App-2f-Modules-2f-Control-2f-GeneratedFlightControl-2f-Generated

