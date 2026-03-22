################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/S32/s32_2_2/S32DS/software/S32SDK_S32K1xx_RTM_3.0.0/rtos/osif/osif_baremetal.c 

OBJS += \
./SDK/rtos/osif/osif_baremetal.o 

C_DEPS += \
./SDK/rtos/osif/osif_baremetal.d 


# Each subdirectory must supply rules for building sources it contributes
SDK/rtos/osif/osif_baremetal.o: D:/S32/s32_2_2/S32DS/software/S32SDK_S32K1xx_RTM_3.0.0/rtos/osif/osif_baremetal.c
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C Compiler'
	arm-none-eabi-gcc "@SDK/rtos/osif/osif_baremetal.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


