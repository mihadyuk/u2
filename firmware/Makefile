##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O0 -ggdb -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -std=c11
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -fno-exceptions -std=gnu++11
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = no
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x2000
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x200
endif

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = softfp
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch

# Imported source files and paths
CHIBIOS = ../../ChibiOS-RT
CHIBIOS_CONTRIB = $(CHIBIOS)/../ChibiOS-Contrib

include board_mnu/board.mk
#include board_bezvodiatel/board.mk

# Startup files.
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS_CONTRIB)/os/hal/hal.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# oterh files (optional).
include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk
include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk

# Define linker script file here
LDSCRIPT= $(STARTUPLD)/STM32F407xG.ld
#LDSCRIPT= STM32F407xG_ccm_noload.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(STARTUPSRC) \
       $(PORTSRC) \
       $(KERNSRC) \
       $(HALSRC) \
       $(OSALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(FATFSSRC) \
       $(CHIBIOS)/os/various/syscalls.c \
       $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
       $(CHIBIOS)/os/hal/lib/streams/memstreams.c \
       $(CHIBIOS_CONTRIB)/os/various/bitmap.c \
       lib/uav_utils/endianness.c \
       lib/uav_utils/cpu_load.c \
       lib/uav_utils/soft_crc.c \
       lib/microrl/src/microrl.c \
       lib/tlsf/tlsf.c \
       src/fault_handlers.c \
       benchmark/memcpy_bench.c \
       src/mavlink_local/mav_encoder.c \
       src/usbcfg.c \
       src/fpga/fpga.c \
       src/fpga/fpga_pwm.c \
       src/fpga/fpga_icu.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CONTROLSRC =\
		src/control/drivetrain/pwm_stm32.cpp \
		src/control/drivetrain/pwm_fpga.cpp \
		src/control/drivetrain/servo_tree.cpp \
		src/control/drivetrain/drivetrain.cpp \
		src/control/drivetrain/engine.cpp \
		src/control/drivetrain/engine_1ch.cpp \
		src/control/drivetrain/engine_2ch.cpp \
		src/control/futaba/futaba.cpp \
		src/control/futaba/receiver.cpp \
		src/control/futaba/receiver_ppm.cpp \
		src/control/futaba/receiver_pwm.cpp \
		src/control/futaba/receiver_pwm_fpga.cpp \
		src/control/futaba/receiver_synth.cpp \
		src/control/futaba/receiver_mavlink.cpp \
		src/control/acs.cpp \
		src/control/acs_loop_active.cpp \
		src/control/acs_input.cpp \
		src/control/acs_commands.cpp \
		src/control/mission_executor.cpp \
		src/control/navigator.cpp \
		src/control/stab_vm.cpp \
		src/control/hil.cpp \
		src/control/hil_cli.cpp \

NVRAMSRC = src/nvram_local.cpp \
		lib/24aa/src/mtd.cpp \
		lib/24aa/src/mtd24.cpp \
		lib/24aa/src/mtd25.cpp \
		lib/24aa/src/nvram_fs.cpp \
		lib/24aa/src/nvram_file.cpp \
		lib/24aa/src/nvram_test.cpp \
		
MAVSRC = \
        src/mavlink_local/param_receiver.cpp \
		src/mavlink_local/param_registry.cpp \
		src/mavlink_local/param_validator.cpp \
		src/mavlink_local/tlm_sender.cpp \
		src/mavlink_local/mavlink_local.cpp \
		src/mavlink_local/mav_postman.cpp \
		src/mavlink_local/mav_spam_list.cpp \
		src/mavlink_local/mav_mail.cpp \
		src/mavlink_local/mav_logger.cpp \
		src/mavlink_local/mod_telem.cpp \
		src/mavlink_local/mav_channel_serial.cpp \
		src/mavlink_local/mav_channel_usbserial.cpp \
		src/mavlink_local/mission_receiver.cpp \
		src/mavlink_local/waypoint_db.cpp \
		src/mavlink_local/link_mgr.cpp \

SENSORSRC = src/i2c_local.cpp \
		src/sensors/adis.cpp \
		src/sensors/exti_local.cpp \
		src/sensors/i2c_sensor.cpp \
		src/sensors/marg.cpp \
		src/sensors/mpu6050.cpp \
		src/sensors/ak8975.cpp \
		src/sensors/lsm303_acc.cpp \
		src/sensors/lsm303_mag.cpp \
		src/sensors/gnss/proto_nmea.cpp \
		src/sensors/gnss/proto_ubx.cpp \
		src/sensors/gnss/ublox.cpp \
		src/sensors/gnss/mtkgps.cpp \
		src/sensors/gnss/generic_nmea.cpp \
		src/sensors/gnss/msno_nmea.cpp \
		src/sensors/gnss/gnss_receiver.cpp \
		src/sensors/gnss/gnss_cli.cpp \
		src/sensors/ms5806.cpp \
		src/sensors/bmp085.cpp \
		src/sensors/npa700.cpp \
		src/sensors/pmu.cpp \
		src/sensors/maxsonar.cpp \
		src/sensors/odometer.cpp \
		src/sensors/odometer_stm.cpp \
		src/sensors/odometer_fpga.cpp \
		src/sensors/adc_local.cpp \
		src/sensors/pps.cpp \
		src/sensors/calibrator.cpp \
		src/sensors/power_monitor.cpp

CLISRC = src/cli/cli.cpp \
		src/cli/cli_cmd.cpp \
		src/param_cli.cpp \

CPPSRC = $(CHCPPSRC) \
		$(NVRAMSRC) \
		$(MAVSRC) \
		$(SENSORSRC) \
		$(CLISRC) \
		$(CONTROLSRC) \
		main.cpp \
		src/instantiator.cpp \
		src/debouncer.cpp \
		src/blinker.cpp \
		src/sanity.cpp \
		src/time_keeper.cpp \
		src/navi6d_wrapper.cpp \
		src/kinematic/ahrs.cpp \
		src/kinematic/ahrs_starlino.cpp \
		lib/filters/fir_test.cpp \


# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) \
         $(FATFSINC) \
         $(CHIBIOS)/os/various \
         $(CHIBIOS)/os/hal/lib/streams \
         $(CHIBIOS)/os/various/cpp_wrappers \
         $(CHIBIOS_CONTRIB)/os/various \
         src/sensors \
         src/sensors/gnss \
         lib/microrl/src \
         lib/uav_utils \
         lib/filters \
         lib/onewire \
         lib/24aa/src \
         lib/tlsf \
         src \
         src/mavlink_local \
         src/kinematic \
         src/cli \
         src/control \
         src/fpga \
         lib/navi6d \
         lib/mavlink/C/lapwing \
         benchmark \

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

TRGT = arm-none-eabi-
ifeq ($(OS),Windows_NT) # workaround for crappy OS
	CC   = $(TRGT)gcc
	CPPC = $(TRGT)g++
else
	CC   = ccache $(TRGT)gcc
	CPPC = ccache $(TRGT)g++
endif

# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary
SREC = $(CP) -O srec

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Common warning flags
WARN = -Wall -Wextra -Wdouble-promotion -Wformat -Wundef #-Wconversion -Werror

# Define C warning options here
CWARN = $(WARN) -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = $(WARN)

# 
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk

######## resource usage show ################
POST_MAKE_ALL_RULE_HOOK: res_usage

res_usage: $(BUILDDIR)/$(PROJECT).bin $(BUILDDIR)/$(PROJECT).elf \
			$(BUILDDIR)/$(PROJECT).hex $(BUILDDIR)/$(PROJECT).dmp
	../tools/res_usage.sh	

######### autogeneration rules ##############
$(OBJS) : src/mavlink_local/param_autogenerated.hpp \
          src/mavlink_local/mavlink_local.hpp \
          lib/mavlink/C/lapwing/mavlink.h \

src/cli/cli_cmd.cpp: firmware_version.h

../tools/firmware_version.sh: .

firmware_version.h: ../tools/firmware_version.sh
	../tools/firmware_version.sh

src/mavlink_local/param_registry.cpp: src/mavlink_local/param_autogenerated.hpp
src/mavlink_local/param_autogenerated.hpp: gen_param_struct.py param_table_uav.py Makefile
	rm -f $@
	python $< -n uav -f $@

src/mavlink_local/tlm_sender.cpp: src/mavlink_local/gen_tlm_sender.py Makefile
	python $< -f $@

src/mavlink_local/mav_encoder.c: src/mavlink_local/gen_mav_encoder.py Makefile
	python $< -f $@

lib/mavlink/C/lapwing/mavlink.h: lib/mavlink/definitions/lapwing.xml ../mavlink_generator
	cd lib/mavlink && make

src/mavlink_local/message.hpp: lib/mavlink/C/lapwing/mavlink.h

clean: autogen_clean
autogen_clean:
	rm -f src/mavlink_local/param_autogenerated.hpp
	rm -f src/mavlink_local/mav_encoder.c
	rm -f src/mavlink_local/tlm_sender.cpp
	rm -f firmware_version.h


