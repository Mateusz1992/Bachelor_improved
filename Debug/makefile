################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Lib/subdir.mk
-include Dave/Generated/src/UART001/subdir.mk
-include Dave/Generated/src/SYSTM001/subdir.mk
-include Dave/Generated/src/RESET001/subdir.mk
-include Dave/Generated/src/LIBS/subdir.mk
-include Dave/Generated/src/IO004/subdir.mk
-include Dave/Generated/src/I2C001/subdir.mk
-include Dave/Generated/src/DAVESupport/subdir.mk
-include Dave/Generated/src/CLK001/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
Bachelor_improved.hex \

SECONDARY_LIST += \
Bachelor_improved.lst \

SECONDARY_SIZE += \
Bachelor_improved.siz \


# All Target
all: Bachelor_improved.elf secondary-outputs

# Tool invocations
Bachelor_improved.elf: $(OBJS) ../Bachelor_improved.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM-GCC C Linker'
	"C:\DAVE-3.1.10\ARM-GCC/bin/arm-none-eabi-gcc" -T"../Bachelor_improved.ld" -nostartfiles -L"C:\DAVE-3.1.10\eclipse\/../CMSIS/Infineon/Lib" -L"C:\DAVE-3.1.10\eclipse\/../Examples/Lib" -L"C:\DAVE-3.1.10\eclipse\/../emWin/Start/GUI" -Wl,-Map,"Bachelor_improved.map" -mcpu=cortex-m4 -mthumb -g3 -gdwarf-2 -o "Bachelor_improved.elf" "@makefile.rsp" $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Bachelor_improved.hex: Bachelor_improved.elf
	@echo 'Invoking: ARM-GCC Create Flash Image'
	"C:\DAVE-3.1.10\ARM-GCC/bin/arm-none-eabi-objcopy" -O ihex "Bachelor_improved.elf"  "Bachelor_improved.hex"
	@echo 'Finished building: $@'
	@echo ' '

Bachelor_improved.lst: Bachelor_improved.elf
	@echo 'Invoking: ARM-GCC Create Listing'
	"C:\DAVE-3.1.10\ARM-GCC/bin/arm-none-eabi-objdump" -h -S "Bachelor_improved.elf" > "Bachelor_improved.lst"
	@echo 'Finished building: $@'
	@echo ' '

Bachelor_improved.siz: Bachelor_improved.elf
	@echo 'Invoking: ARM-GCC Print Size'
	"C:\DAVE-3.1.10\ARM-GCC/bin/arm-none-eabi-size"  --format=berkeley Bachelor_improved.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(SECONDARY_FLASH)$(S_DEPS)$(ARM_GCC_FOR_XMC_LINKER_OUTPUT_TYPE_OUTPUTS)$(SECONDARY_LIST) Bachelor_improved.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
