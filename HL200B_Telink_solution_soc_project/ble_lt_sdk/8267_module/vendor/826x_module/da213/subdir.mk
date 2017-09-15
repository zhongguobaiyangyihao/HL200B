################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/826x_module/da213/mira_da213_drv.c 

OBJS += \
./vendor/826x_module/da213/mira_da213_drv.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/826x_module/da213/%.o: ../vendor/826x_module/da213/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\Kyle\git_repository\git_hub\HL200B\HL200B_Telink_solution_soc_project\ble_lt_sdk\proj\mcu_spec" -D__PROJECT_8267_MODULE__=1 -D__MC3416_H__ -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


