################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include vendor/common/subdir.mk
-include vendor/826x_module/subdir.mk
-include vendor/826x_module/slip_protocol/subdir.mk
-include vendor/826x_module/da213/subdir.mk
-include vendor/826x_module/MC3416/subdir.mk
-include vendor/826x_module/GPS_GPRS_Communicate/subdir.mk
-include proj/mcu_spec/subdir.mk
-include proj/mcu/subdir.mk
-include proj/drivers/subdir.mk
-include proj/common/subdir.mk
-include proj/app/subdir.mk
-include 8267_ble_remote/subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LST += \
8267_module.lst \

FLASH_IMAGE += \
8267_module.bin \

SIZEDUMMY += \
sizedummy \


# All Target
all: pre-build main-build

# Main-build Target
main-build: HL200B_Telink_solution_project.elf secondary-outputs

# Tool invocations
HL200B_Telink_solution_project.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: TC32 C Linker'
	tc32-elf-ld --gc-sections -L"E:\Kyle\git_repository\git_hub\HL200B\HL200B_Telink_solution_soc_project\ble_lt_sdk\proj_lib" -T ../boot.link -o"HL200B_Telink_solution_project.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

8267_module.lst: HL200B_Telink_solution_project.elf
	@echo 'Invoking: TC32 Create Extended Listing'
	tc32-elf-objdump -x -D -l -S HL200B_Telink_solution_project.elf  >"8267_module.lst"
	@echo 'Finished building: $@'
	@echo ' '

8267_module.bin: HL200B_Telink_solution_project.elf
	@echo 'Create Flash image (binary format)'
	tc32-elf-objcopy -O binary HL200B_Telink_solution_project.elf  "8267_module.bin"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: HL200B_Telink_solution_project.elf
	@echo 'Invoking: Print Size'
	tc32-elf-size -t HL200B_Telink_solution_project.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(FLASH_IMAGE)$(ELFS)$(LST)$(SIZEDUMMY) HL200B_Telink_solution_project.elf
	-@echo ' '

pre-build:
	-"E:\Kyle\git_repository\git_hub\HL200B\HL200B_Telink_solution_soc_project\ble_lt_sdk/getver.sh"
	-@echo ' '

secondary-outputs: $(LST) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY: main-build pre-build

-include ../makefile.targets
