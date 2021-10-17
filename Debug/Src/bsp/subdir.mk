################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/bsp/bsp_can.c \
../Src/bsp/bsp_imu.c \
../Src/bsp/bsp_lcd.c \
../Src/bsp/bsp_trigger.c \
../Src/bsp/bsp_uart.c \
../Src/bsp/myiic.c 

OBJS += \
./Src/bsp/bsp_can.o \
./Src/bsp/bsp_imu.o \
./Src/bsp/bsp_lcd.o \
./Src/bsp/bsp_trigger.o \
./Src/bsp/bsp_uart.o \
./Src/bsp/myiic.o 

C_DEPS += \
./Src/bsp/bsp_can.d \
./Src/bsp/bsp_imu.d \
./Src/bsp/bsp_lcd.d \
./Src/bsp/bsp_trigger.d \
./Src/bsp/bsp_uart.d \
./Src/bsp/myiic.d 


# Each subdirectory must supply rules for building sources it contributes
Src/bsp/%.o: ../Src/bsp/%.c Src/bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F427xx -DUSE_HAL_DRIVER -DDEBUG -DARM_MATH_CM4 -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/DSP/Include -I../Src/bsp -I../Src/Fusion -I../Src/Task -I../Src/PID -I../Src/LPF -I../Src/CRC8_CRC16 -I../Src/Chassis_Power_Control -I../Src/Kalman -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

