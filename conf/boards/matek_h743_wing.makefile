# Hey Emacs, this is a -*- makefile -*-
#
# matek_f765_wing.makefile
#
# based on STM32H7
# only compatible with ChibiOS
#

BOARD=matek
BOARD_VERSION=H743-WING
BOARD_DIR=mateksys/$(BOARD_VERSION)
BOARD_CFG=\"boards/$(BOARD_DIR)/$(BOARD)$(BOARD_VERSION).h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios
MCU=cortex-m7

## FPU on F7
USE_FPU=hard
USE_FPU_OPT= -mfpu=fpv5-d16

USE_LTO ?= yes

$(TARGET).CFLAGS += -DSTM32H7 -DPPRZLINK_ENABLE_FD -DUSE_HARD_FAULT_RECOVERY -DDSHOT_CHANNEL_FIRST_INDEX=1U

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32H7xx/platform.mk
CHIBIOS_BOARD_PORT = ARMCMx/STM32H7xx/port.mk
CHIBIOS_BOARD_LINKER = STM32H743xI.ld
CHIBIOS_BOARD_STARTUP = startup_stm32h7xx.mk

# ITCM flash is a special flash that allow faster operations
# At the moment it is not possible to flash the code in this mode using dfu-util
# but it should work with the BlackMagicProbe or STLINK
# By default, normal flash is used
ifeq ($(USE_ITCM),1)
$(TARGET).CFLAGS += -DUSE_ITCM=1
DFU_ADDR = 0x00200000
else
$(TARGET).CFLAGS += -DUSE_ITCM=0
DFU_ADDR = 0x08000000
endif

##############################################################################
# Compiler settings
#

# default flash mode is via usb dfu bootloader
# possibilities: DFU-UTIL, SWD, STLINK
FLASH_MODE ?= DFU-UTIL

HAS_LUFTBOOT = FALSE

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default UART configuration (modem, gps, spektrum)
#

MODEM_PORT ?= UART7
MODEM_BAUD ?= B57600

GPS_PORT ?= UART2
GPS_BAUD ?= B57600

RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT ?= UART6

# single mode
SBUS_PORT ?= UART6

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
