################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ADC.c \
../User/config.c \
../User/gpio.c \
../User/main.c \
../User/pwm.c 

OBJS += \
./User/ADC.o \
./User/config.o \
./User/gpio.o \
./User/main.o \
./User/pwm.o 

C_DEPS += \
./User/ADC.d \
./User/config.d \
./User/gpio.d \
./User/main.d \
./User/pwm.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c User/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -I"../Library/CMSIS/Include" -I"../Library/Device/Nuvoton/M0564/Include" -I"../Library/StdDriver/inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


