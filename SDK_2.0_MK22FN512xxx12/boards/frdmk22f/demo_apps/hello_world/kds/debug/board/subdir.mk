################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/boards/frdmk22f/demo_apps/hello_world/board.c \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/boards/frdmk22f/demo_apps/hello_world/clock_config.c \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/boards/frdmk22f/demo_apps/hello_world/pin_mux.c 

OBJS += \
./board/board.o \
./board/clock_config.o \
./board/pin_mux.o 

C_DEPS += \
./board/board.d \
./board/clock_config.d \
./board/pin_mux.d 


# Each subdirectory must supply rules for building sources it contributes
board/board.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/boards/frdmk22f/demo_apps/hello_world/board.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

board/clock_config.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/boards/frdmk22f/demo_apps/hello_world/clock_config.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

board/pin_mux.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/boards/frdmk22f/demo_apps/hello_world/pin_mux.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


