################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../board/clock_config.c \
../board/pin_mux.c \
../board/pulsar_board.c 

OBJS += \
./board/clock_config.o \
./board/pin_mux.o \
./board/pulsar_board.o 

C_DEPS += \
./board/clock_config.d \
./board/pin_mux.d \
./board/pulsar_board.d 


# Each subdirectory must supply rules for building sources it contributes
board/%.o: ../board/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -D"CPU_MK22FN512VDC12" -I../CMSIS -I../board -I../drivers -I../freertos/Source/include -I../freertos/Source/portable/GCC/ARM_CM4F -I../source -I../startup -I../utilities -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


