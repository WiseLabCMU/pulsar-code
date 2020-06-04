################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/system_MK22F51212.c 

S_UPPER_SRCS += \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/gcc/startup_MK22F51212.S 

OBJS += \
./startup/startup_MK22F51212.o \
./startup/system_MK22F51212.o 

C_DEPS += \
./startup/system_MK22F51212.d 

S_UPPER_DEPS += \
./startup/startup_MK22F51212.d 


# Each subdirectory must supply rules for building sources it contributes
startup/startup_MK22F51212.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/gcc/startup_MK22F51212.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -x assembler-with-cpp -DDEBUG -D__STARTUP_CLEAR_BSS -flto  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup/system_MK22F51212.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/system_MK22F51212.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


