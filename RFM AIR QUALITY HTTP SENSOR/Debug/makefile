################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Library/subdir.mk
-include CMSIS/GCC/subdir.mk
-include CMSIS/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
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


BUILD_ARTIFACT_NAME := RFM AIR QUALITY HTTP SENSOR
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
RFM\ AIR\ QUALITY\ HTTP\ SENSOR.hex \

SECONDARY_SIZE += \
RFM\ AIR\ QUALITY\ HTTP\ SENSOR.siz \


# All Target
all: RFM\ AIR\ QUALITY\ HTTP\ SENSOR.elf secondary-outputs

# Tool invocations
RFM\ AIR\ QUALITY\ HTTP\ SENSOR.elf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM GNU C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -T gcc_arm.ld -Xlinker --gc-sections -L"../CMSIS/GCC" -Wl,-Map,"RFM AIR QUALITY HTTP SENSOR.map" --specs=nano.specs -o "RFM AIR QUALITY HTTP SENSOR.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

RFM\ AIR\ QUALITY\ HTTP\ SENSOR.hex: RFM\ AIR\ QUALITY\ HTTP\ SENSOR.elf makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "RFM AIR QUALITY HTTP SENSOR.elf"  "RFM AIR QUALITY HTTP SENSOR.hex"
	@echo 'Finished building: $@'
	@echo ' '

RFM\ AIR\ QUALITY\ HTTP\ SENSOR.siz: RFM\ AIR\ QUALITY\ HTTP\ SENSOR.elf makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "RFM AIR QUALITY HTTP SENSOR.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) "RFM AIR QUALITY HTTP SENSOR.elf"
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
