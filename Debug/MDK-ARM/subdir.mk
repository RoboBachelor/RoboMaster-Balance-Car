################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../MDK-ARM/startup_stm32f427xx.s 

OBJS += \
./MDK-ARM/startup_stm32f427xx.o 

S_DEPS += \
./MDK-ARM/startup_stm32f427xx.d 


# Each subdirectory must supply rules for building sources it contributes
MDK-ARM/%.o: ../MDK-ARM/%.s MDK-ARM/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

