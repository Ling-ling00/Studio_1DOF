################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/BasicMathFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/BayesFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/CommonTables" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/ComplexMathFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/ControllerFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/DistanceFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/FastMathFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/FilteringFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/InterpolationFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/MatrixFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/QuaternionMathFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/StatisticsFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/SupportFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/SVMFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/TransformFunctions" -I"C:/Users/Ponwalai/Documents/GitHub/Studio-101/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-SVMFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-SVMFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/SVMFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-SVMFunctions

