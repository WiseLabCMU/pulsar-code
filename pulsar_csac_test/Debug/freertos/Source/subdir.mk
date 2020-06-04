################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos/Source/croutine.c \
../freertos/Source/event_groups.c \
../freertos/Source/list.c \
../freertos/Source/queue.c \
../freertos/Source/tasks.c \
../freertos/Source/timers.c 

OBJS += \
./freertos/Source/croutine.o \
./freertos/Source/event_groups.o \
./freertos/Source/list.o \
./freertos/Source/queue.o \
./freertos/Source/tasks.o \
./freertos/Source/timers.o 

C_DEPS += \
./freertos/Source/croutine.d \
./freertos/Source/event_groups.d \
./freertos/Source/list.d \
./freertos/Source/queue.d \
./freertos/Source/tasks.d \
./freertos/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
freertos/Source/%.o: ../freertos/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -D"CPU_MK22FN512VDC12" -I../CMSIS -I../board -I../drivers -I../freertos/Source/include -I../freertos/Source/portable/GCC/ARM_CM4F -I../source -I../startup -I../utilities -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


