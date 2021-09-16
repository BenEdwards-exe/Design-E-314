################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
e314.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
e314.list \

OBJCOPY_BIN += \
e314.bin \


# All Target
all: e314.elf secondary-outputs

# Tool invocations
e314.elf: $(OBJS) $(USER_OBJS) C:\Users\benjr\Documents\University\Third\ Year\ 2021\Semester\ 1\Design(E)_314\22738002-e314-proj\e314\STM32F103RBTX_FLASH.ld
	arm-none-eabi-gcc -o "e314.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\benjr\Documents\University\Third Year 2021\Semester 1\Design(E)_314\22738002-e314-proj\e314\STM32F103RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="e314.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

e314.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "e314.list"
	@echo 'Finished building: $@'
	@echo ' '

e314.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "e314.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
