################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/boards/frdmk22f/demo_apps/hello_world/hello_world.c 

OBJS += \
./source/hello_world.o 

C_DEPS += \
./source/hello_world.d 


# Each subdirectory must supply rules for building sources it contributes
source/hello_world.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/boards/frdmk22f/demo_apps/hello_world/hello_world.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


