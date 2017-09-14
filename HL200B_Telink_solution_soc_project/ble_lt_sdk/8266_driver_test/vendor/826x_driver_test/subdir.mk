################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/826x_driver_test/app.c \
../vendor/826x_driver_test/app_adc.c \
../vendor/826x_driver_test/app_gpio_irq.c \
../vendor/826x_driver_test/app_i2c.c \
../vendor/826x_driver_test/app_pwm.c \
../vendor/826x_driver_test/app_spi.c \
../vendor/826x_driver_test/app_timer.c \
../vendor/826x_driver_test/app_uart.c \
../vendor/826x_driver_test/main.c 

OBJS += \
./vendor/826x_driver_test/app.o \
./vendor/826x_driver_test/app_adc.o \
./vendor/826x_driver_test/app_gpio_irq.o \
./vendor/826x_driver_test/app_i2c.o \
./vendor/826x_driver_test/app_pwm.o \
./vendor/826x_driver_test/app_spi.o \
./vendor/826x_driver_test/app_timer.o \
./vendor/826x_driver_test/app_uart.o \
./vendor/826x_driver_test/main.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/826x_driver_test/%.o: ../vendor/826x_driver_test/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\Kyle\项目开发\HL200B\HL200B_Telink_solution_soc_project\ble_lt_sdk\proj\mcu" -I"E:\Kyle\项目开发\HL200B\HL200B_Telink_solution_soc_project\ble_lt_sdk\proj\mcu_spec" -D__PROJECT_8266_DRIVER_TEST__=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


