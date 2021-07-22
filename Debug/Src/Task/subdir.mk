################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Task/gimbal_task.c \
../Src/Task/imu_task.c 

OBJS += \
./Src/Task/gimbal_task.o \
./Src/Task/imu_task.o 

C_DEPS += \
./Src/Task/gimbal_task.d \
./Src/Task/imu_task.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Task/%.o: ../Src/Task/%.c Src/Task/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F427xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Src/bsp -I../Src/Fusion -I../Src/Task -I../Src/PID -I../Src/LPF -I../Src/CRC8_CRC16 -I../Src/Chassis_Power_Control -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

