################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../proj/mcu/analog.c \
../proj/mcu/anareg.c \
../proj/mcu/audhw.c \
../proj/mcu/clock.c \
../proj/mcu/cpu.c \
../proj/mcu/otp.c \
../proj/mcu/putchar.c \
../proj/mcu/random.c \
../proj/mcu/register.c \
../proj/mcu/swire.c 

OBJS += \
./proj/mcu/analog.o \
./proj/mcu/anareg.o \
./proj/mcu/audhw.o \
./proj/mcu/clock.o \
./proj/mcu/cpu.o \
./proj/mcu/otp.o \
./proj/mcu/putchar.o \
./proj/mcu/random.o \
./proj/mcu/register.o \
./proj/mcu/swire.o 


# Each subdirectory must supply rules for building sources it contributes
proj/mcu/%.o: ../proj/mcu/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\Kyle\项目开发\HL200B\HL200B_Telink_solution_soc_project\ble_lt_sdk\proj\mcu" -I"E:\Kyle\项目开发\HL200B\HL200B_Telink_solution_soc_project\ble_lt_sdk\proj\mcu_spec" -D__PROJECT_8266_DRIVER_TEST__=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


