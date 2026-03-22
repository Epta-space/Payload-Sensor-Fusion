################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BAR.c \
../Core/Src/GNSS.c \
../Core/Src/IMU.c \
../Core/Src/IMU_FUSION.c \
../Core/Src/LORA.c \
../Core/Src/ROCKET.c \
../Core/Src/ROCKET_FSM.c \
../Core/Src/SD.c \
../Core/Src/circ_buffer.c \
../Core/Src/e22900t22d.c \
../Core/Src/fatfs_sd_card.c \
../Core/Src/icp10100.c \
../Core/Src/main.c \
../Core/Src/quaternion.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/BAR.o \
./Core/Src/GNSS.o \
./Core/Src/IMU.o \
./Core/Src/IMU_FUSION.o \
./Core/Src/LORA.o \
./Core/Src/ROCKET.o \
./Core/Src/ROCKET_FSM.o \
./Core/Src/SD.o \
./Core/Src/circ_buffer.o \
./Core/Src/e22900t22d.o \
./Core/Src/fatfs_sd_card.o \
./Core/Src/icp10100.o \
./Core/Src/main.o \
./Core/Src/quaternion.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/BAR.d \
./Core/Src/GNSS.d \
./Core/Src/IMU.d \
./Core/Src/IMU_FUSION.d \
./Core/Src/LORA.d \
./Core/Src/ROCKET.d \
./Core/Src/ROCKET_FSM.d \
./Core/Src/SD.d \
./Core/Src/circ_buffer.d \
./Core/Src/e22900t22d.d \
./Core/Src/fatfs_sd_card.d \
./Core/Src/icp10100.d \
./Core/Src/main.d \
./Core/Src/quaternion.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Oz -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/main.o: ../Core/Src/main.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Oz -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BAR.cyclo ./Core/Src/BAR.d ./Core/Src/BAR.o ./Core/Src/BAR.su ./Core/Src/GNSS.cyclo ./Core/Src/GNSS.d ./Core/Src/GNSS.o ./Core/Src/GNSS.su ./Core/Src/IMU.cyclo ./Core/Src/IMU.d ./Core/Src/IMU.o ./Core/Src/IMU.su ./Core/Src/IMU_FUSION.cyclo ./Core/Src/IMU_FUSION.d ./Core/Src/IMU_FUSION.o ./Core/Src/IMU_FUSION.su ./Core/Src/LORA.cyclo ./Core/Src/LORA.d ./Core/Src/LORA.o ./Core/Src/LORA.su ./Core/Src/ROCKET.cyclo ./Core/Src/ROCKET.d ./Core/Src/ROCKET.o ./Core/Src/ROCKET.su ./Core/Src/ROCKET_FSM.cyclo ./Core/Src/ROCKET_FSM.d ./Core/Src/ROCKET_FSM.o ./Core/Src/ROCKET_FSM.su ./Core/Src/SD.cyclo ./Core/Src/SD.d ./Core/Src/SD.o ./Core/Src/SD.su ./Core/Src/circ_buffer.cyclo ./Core/Src/circ_buffer.d ./Core/Src/circ_buffer.o ./Core/Src/circ_buffer.su ./Core/Src/e22900t22d.cyclo ./Core/Src/e22900t22d.d ./Core/Src/e22900t22d.o ./Core/Src/e22900t22d.su ./Core/Src/fatfs_sd_card.cyclo ./Core/Src/fatfs_sd_card.d ./Core/Src/fatfs_sd_card.o ./Core/Src/fatfs_sd_card.su ./Core/Src/icp10100.cyclo ./Core/Src/icp10100.d ./Core/Src/icp10100.o ./Core/Src/icp10100.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/quaternion.cyclo ./Core/Src/quaternion.d ./Core/Src/quaternion.o ./Core/Src/quaternion.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

