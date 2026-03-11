################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library/StdDriver/adc.c \
../Library/StdDriver/gpio.c 

OBJS += \
./Library/StdDriver/adc.o \
./Library/StdDriver/gpio.o 

C_DEPS += \
./Library/StdDriver/adc.d \
./Library/StdDriver/gpio.d 


# Each subdirectory must supply rules for building sources it contributes
Library/StdDriver/%.o: ../Library/StdDriver/%.c Library/StdDriver/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -I"../Library/CMSIS/Include" -I"../Library/Device/Nuvoton/M031Series/Include" -I"../Library/StdDriver/inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


