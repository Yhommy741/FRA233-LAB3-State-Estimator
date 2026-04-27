################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/KalmanFilter.c \
../Core/Src/KalmanFilterCAD.c \
../Core/Src/KalmanFilterCD.c \
../Core/Src/KalmanFilterCVD.c \
../Core/Src/KalmanFilterMSD.c \
../Core/Src/SerialFrame.c \
../Core/Src/Ultrasonic.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/KalmanFilter.o \
./Core/Src/KalmanFilterCAD.o \
./Core/Src/KalmanFilterCD.o \
./Core/Src/KalmanFilterCVD.o \
./Core/Src/KalmanFilterMSD.o \
./Core/Src/SerialFrame.o \
./Core/Src/Ultrasonic.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/KalmanFilter.d \
./Core/Src/KalmanFilterCAD.d \
./Core/Src/KalmanFilterCD.d \
./Core/Src/KalmanFilterCVD.d \
./Core/Src/KalmanFilterMSD.d \
./Core/Src/SerialFrame.d \
./Core/Src/Ultrasonic.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/KalmanFilter.cyclo ./Core/Src/KalmanFilter.d ./Core/Src/KalmanFilter.o ./Core/Src/KalmanFilter.su ./Core/Src/KalmanFilterCAD.cyclo ./Core/Src/KalmanFilterCAD.d ./Core/Src/KalmanFilterCAD.o ./Core/Src/KalmanFilterCAD.su ./Core/Src/KalmanFilterCD.cyclo ./Core/Src/KalmanFilterCD.d ./Core/Src/KalmanFilterCD.o ./Core/Src/KalmanFilterCD.su ./Core/Src/KalmanFilterCVD.cyclo ./Core/Src/KalmanFilterCVD.d ./Core/Src/KalmanFilterCVD.o ./Core/Src/KalmanFilterCVD.su ./Core/Src/KalmanFilterMSD.cyclo ./Core/Src/KalmanFilterMSD.d ./Core/Src/KalmanFilterMSD.o ./Core/Src/KalmanFilterMSD.su ./Core/Src/SerialFrame.cyclo ./Core/Src/SerialFrame.d ./Core/Src/SerialFrame.o ./Core/Src/SerialFrame.su ./Core/Src/Ultrasonic.cyclo ./Core/Src/Ultrasonic.d ./Core/Src/Ultrasonic.o ./Core/Src/Ultrasonic.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

