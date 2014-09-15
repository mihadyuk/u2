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
  USE_COPT = -std=gnu11
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

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
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
CHIBIOS = ../../chibios-svn
include $(CHIBIOS)/os/hal/hal.mk
include board/board.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_stm32f4xx.mk
include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk

# Define linker script file here
LDSCRIPT= $(PORTLD)/STM32F407xG.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(TESTSRC) \
       $(HALSRC) \
       $(OSALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/various/chprintf.c \
       $(CHIBIOS)/os/various/syscalls.c \
       lib/uav_utils/endianness.c \
       lib/uav_utils/cpu_load.c \
       src/fault_handlers.c \

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
NVRAMSRC = lib/24aa/src/mtd.cpp \
         lib/24aa/src/eeprom_mtd.cpp \
         lib/24aa/src/fram_mtd.cpp \
         lib/24aa/src/nvram_fs.cpp \
         lib/24aa/src/nvram_file.cpp

MAVSRC = src/mavlink_local/parameters.cpp \
         src/mavlink_local/param_registry.cpp \
         src/mavlink_local/param_validator.cpp \
         src/mavlink_local/mavpostman.cpp \
         src/mavlink_local/mavworker.cpp \
         src/mavlink_local/mavmail.cpp \
         src/mavlink_local/mavlogger.cpp \
         src/mavlink_local/mavchannel_serial.cpp \
         src/mavlink_local/mavchannel_usbserial.cpp \
         src/mavlink_local/message.cpp \

CPPSRC = $(CHCPPSRC) \
         $(NVRAMSRC) \
         $(MAVSRC) \
         main.cpp \
         drivers/adis.cpp \
         drivers/exti_local.cpp \
         src/i2c_local.cpp \
         src/proto.cpp \
         src/nvram_local.cpp \
         src/link_mgr.cpp \
         src/shell.cpp \
         src/usb_debouncer.cpp \
         src/blinker.cpp \
         src/sanity.cpp \
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
ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(OSALINC) $(PLATFORMINC) $(BOARDINC) \
         $(CHIBIOS)/os/various/devices_lib/accel \
         $(CHIBIOS)/os/various \
         $(CHIBIOS)/os/various/cpp_wrappers \
         drivers \
         lib/uav_utils \
         lib/filters \
         lib/24aa/src \
         src \
         src/mavlink_local \
         lib/mavlink/C/lapwing \

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4
 
#TRGT = arm-elf-
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
WARN = -Wall -Wextra -Wdouble-promotion -Wformat #-Wconversion -Werror

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
MAKE_ALL_RULE_HOOK: res_usage

res_usage: $(BUILDDIR)/$(PROJECT).bin $(BUILDDIR)/$(PROJECT).elf \
			$(BUILDDIR)/$(PROJECT).hex $(BUILDDIR)/$(PROJECT).dmp
	../tools/res_usage.sh	

######### autogeneration rules ##############
$(OBJS) : src/mavlink_local/param_autogenerated.hpp \
          src/mavlink_local/message.hpp

src/mavlink_local/param_registry.cpp: src/mavlink_local/param_autogenerated.hpp
src/mavlink_local/param_autogenerated.hpp: gen_param_struct.py param_table_uav.py Makefile
	rm -f $@
	python $< -n uav -f $@

#src/datalink/tlm_sender.cpp: ../common/link_generators/gen_tlm_sender.py Makefile
#	python $< -f $@ -n acs -c icr

lib/mavlink/C/lapwing/lapwing.h: lib/mavlink/definitions/lapwing.xml ../mavlink_generator
	cd lib/mavlink && make

src/mavlink_local/message.hpp: lib/mavlink/C/lapwing/lapwing.h

clean: autogen_clean
autogen_clean:
	rm -f src/mavlink_local/param_autogenerated.hpp

