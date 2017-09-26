################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/826x_module/slip_protocol/slip_protocol.c 

OBJS += \
./vendor/826x_module/slip_protocol/slip_protocol.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/826x_module/slip_protocol/%.o: ../vendor/826x_module/slip_protocol/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\Kyle\git_repository\git_sprocomm\HL200B\Telink_solution_soc_project\ble_lt_sdk\proj\mcu_spec" -D__PROJECT_8267_MODULE__=1 -D__MC3416_H__ -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


