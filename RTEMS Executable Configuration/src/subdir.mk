################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/punch.c 

OBJS += \
./src/punch.o 

C_DEPS += \
./src/punch.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: RTEMS C Compiler'
	/afs/ms.mff.cuni.cz/u/b/bures/BIG/ers-labs/rtems-4.10.0/bin/i386-rtems4.10-gcc -B/afs/ms.mff.cuni.cz/u/b/bures/BIG/ers-labs/rtems-4.10.0/i386-rtems4.10/pc386/lib/ -specs bsp_specs -qrtems -mtune=i386 -Os -g -Wall -c -fmessage-length=0 -pipe -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


