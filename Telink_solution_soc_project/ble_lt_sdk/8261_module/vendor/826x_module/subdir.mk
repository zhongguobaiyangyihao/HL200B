################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/826x_module/app.c \
../vendor/826x_module/app_att.c \
../vendor/826x_module/main.c \
../vendor/826x_module/nv.c \
../vendor/826x_module/spp.c 

OBJS += \
./vendor/826x_module/app.o \
./vendor/826x_module/app_att.o \
./vendor/826x_module/main.o \
./vendor/826x_module/nv.o \
./vendor/826x_module/spp.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/826x_module/%.o: ../vendor/826x_module/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\Kyle\项目开发\HL200B\HL200B_Telink_solution_soc_project\ble_lt_sdk\proj\mcu_spec" -D__PROJECT_8261_MODULE__=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


