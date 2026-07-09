################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/can.c \
../Core/Src/detect.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/module_a.c \
../Core/Src/module_b.c \
../Core/Src/module_manager.c \
../Core/Src/motor.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_data.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/uart_task.c 

OBJS += \
./Core/Src/can.o \
./Core/Src/detect.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/module_a.o \
./Core/Src/module_b.o \
./Core/Src/module_manager.o \
./Core/Src/motor.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_data.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/uart_task.o 

C_DEPS += \
./Core/Src/can.d \
./Core/Src/detect.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/module_a.d \
./Core/Src/module_b.d \
./Core/Src/module_manager.d \
./Core/Src/motor.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_data.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/uart_task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/can.cyclo ./Core/Src/can.d ./Core/Src/can.o ./Core/Src/can.su ./Core/Src/detect.cyclo ./Core/Src/detect.d ./Core/Src/detect.o ./Core/Src/detect.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/module_a.cyclo ./Core/Src/module_a.d ./Core/Src/module_a.o ./Core/Src/module_a.su ./Core/Src/module_b.cyclo ./Core/Src/module_b.d ./Core/Src/module_b.o ./Core/Src/module_b.su ./Core/Src/module_manager.cyclo ./Core/Src/module_manager.d ./Core/Src/module_manager.o ./Core/Src/module_manager.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_data.cyclo ./Core/Src/system_data.d ./Core/Src/system_data.o ./Core/Src/system_data.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/uart_task.cyclo ./Core/Src/uart_task.d ./Core/Src/uart_task.o ./Core/Src/uart_task.su

.PHONY: clean-Core-2f-Src

