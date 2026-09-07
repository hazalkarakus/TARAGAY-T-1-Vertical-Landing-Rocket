################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include App/Services/UARTTelemetry/subdir.mk
-include App/Services/Timebase/subdir.mk
-include App/Services/SystemMonitor/subdir.mk
-include App/Services/SolenoidOutput/subdir.mk
-include App/Services/ServoOutput/subdir.mk
-include App/Services/SensorQualification/subdir.mk
-include App/Services/SDLogger/subdir.mk
-include App/Services/PreflightTrigger/subdir.mk
-include App/Services/NeedleValveIntegrationTest/subdir.mk
-include App/Services/NeedleValveHardware/subdir.mk
-include App/Services/NeedleValveAutonomousControl/subdir.mk
-include App/Services/NeedleValveAutoControl/subdir.mk
-include App/Services/NRFTelemetry/subdir.mk
-include App/Services/DebugDashboard/subdir.mk
-include App/Services/CDCTelemetry/subdir.mk
-include App/Services/BusManager/subdir.mk
-include App/Platform/subdir.mk
-include App/Modules/Sensors/SensorManager/subdir.mk
-include App/Modules/Sensors/Lidar/subdir.mk
-include App/Modules/Sensors/IMU/subdir.mk
-include App/Modules/Sensors/Barometer/subdir.mk
-include App/Modules/RemoteControl/subdir.mk
-include App/Modules/NRF24/subdir.mk
-include App/Modules/Estimation/VerticalEKF/subdir.mk
-include App/Modules/Estimation/FullStateESKF/subdir.mk
-include App/Modules/Estimation/AttitudeEstimator/subdir.mk
-include App/Modules/Control/VerticalLandingControl/subdir.mk
-include App/Modules/Control/TaragayFlightLogic/subdir.mk
-include App/Modules/Control/NeedleValve/subdir.mk
-include App/Modules/Control/MatlabValveController/subdir.mk
-include App/Modules/Control/GeneratedFlightControl/Generated/subdir.mk
-include App/Modules/Control/GeneratedFlightControl/subdir.mk
-include App/Modules/Control/GNCActiveControl/subdir.mk
-include App/Modules/Control/ControlInputProvider/subdir.mk
-include App/Modules/Control/AttitudeControl/subdir.mk
-include App/Core/Tasks/subdir.mk
-include App/Core/Scheduler/subdir.mk
-include App/Common/Filters/subdir.mk
-include App/Common/subdir.mk
-include App/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.elf \

MAP_FILES += \
TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.list \

OBJCOPY_HEX += \
TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.hex \


# All Target
all: main-build

# Main-build Target
main-build: TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.elf secondary-outputs

# Tool invocations
TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.elf TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.map: $(OBJS) $(USER_OBJS) ../STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"../STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.list"
	@echo 'Finished building: $@'
	@echo ' '

TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.elf TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.hex TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.list TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
