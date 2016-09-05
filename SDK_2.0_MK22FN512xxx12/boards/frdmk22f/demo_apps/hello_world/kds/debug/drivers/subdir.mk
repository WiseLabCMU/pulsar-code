################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_clock.c \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_common.c \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_gpio.c \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_lpuart.c \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_smc.c \
/Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_uart.c 

OBJS += \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_gpio.o \
./drivers/fsl_lpuart.o \
./drivers/fsl_smc.o \
./drivers/fsl_uart.o 

C_DEPS += \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_gpio.d \
./drivers/fsl_lpuart.d \
./drivers/fsl_smc.d \
./drivers/fsl_uart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/fsl_clock.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_clock.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_common.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_common.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_gpio.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_lpuart.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_lpuart.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_smc.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_smc.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_uart.o: /Users/adwait/Workspace/KDS/SDK_2.0_MK22FN512xxx12/devices/MK22F51212/drivers/fsl_uart.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK22FN512VLH12 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K22F -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../../../../../../devices/MK22F51212/drivers -I../.. -I../../../../../../devices/MK22F51212/utilities -I../../../.. -I../../../../../../devices/MK22F51212 -std=gnu99 -flto  -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


