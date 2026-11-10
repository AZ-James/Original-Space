################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../gpio_toggle_output.syscfg 

C_SRCS += \
../ADC.c \
../No_Mcu_Ganv_Grayscale_Sensor.c \
../Uart.c \
../delay.c \
./ti_msp_dl_config.c \
C:/ti/mspm0_sdk_2_05_01_00/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../main.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./ADC.d \
./No_Mcu_Ganv_Grayscale_Sensor.d \
./Uart.d \
./delay.d \
./ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./main.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./ADC.o \
./No_Mcu_Ganv_Grayscale_Sensor.o \
./Uart.o \
./delay.o \
./ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./main.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"ADC.o" \
"No_Mcu_Ganv_Grayscale_Sensor.o" \
"Uart.o" \
"delay.o" \
"ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"main.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"ADC.d" \
"No_Mcu_Ganv_Grayscale_Sensor.d" \
"Uart.d" \
"delay.d" \
"ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"main.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../ADC.c" \
"../No_Mcu_Ganv_Grayscale_Sensor.c" \
"../Uart.c" \
"../delay.c" \
"./ti_msp_dl_config.c" \
"C:/ti/mspm0_sdk_2_05_01_00/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../main.c" 

SYSCFG_SRCS__QUOTED += \
"../gpio_toggle_output.syscfg" 


