#!/bin/bash

CHIBI_INCLUDES="-I. -I../../ChibiOS-RT/os/license -I../../ChibiOS-RT/os/common/startup/ARMCMx/compilers/GCC -I../../ChibiOS-RT/os/common/startup/ARMCMx/devices/STM32F4xx -I../../ChibiOS-RT/os/common/ext/CMSIS/include -I../../ChibiOS-RT/os/common/ext/CMSIS/ST/STM32F4xx -I../../ChibiOS-RT/os/rt/include -I../../ChibiOS-RT/os/common/oslib/include -I../../ChibiOS-RT/os/common/ports/ARMCMx -I../../ChibiOS-RT/os/common/ports/ARMCMx/compilers/GCC -I../../ChibiOS-RT/os/hal/osal/rt -I../../ChibiOS-RT/os/hal/include -I../../ChibiOS-Contrib/os/hal/include -I../../ChibiOS-RT/os/hal/ports/common/ARMCMx -I../../ChibiOS-RT/os/hal/ports/STM32/STM32F4xx -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/ADCv2 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/CANv1 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/DACv1 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/DMAv2 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/EXTIv1 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/GPIOv2 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/I2Cv1 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/MACv1 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/OTGv1 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/RTCv2 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/SDIOv1 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/SPIv1 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/TIMv1 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/USARTv1 -I../../ChibiOS-RT/os/hal/ports/STM32/LLD/xWDGv1 -I../../ChibiOS-Contrib/os/hal/ports/STM32/LLD/DMA2Dv1 -I../../ChibiOS-Contrib/os/hal/ports/STM32/LLD/FSMCv1 -I../../ChibiOS-Contrib/os/hal/ports/STM32/LLD/LTDCv1 -I../../ChibiOS-Contrib/os/hal/ports/STM32/LLD/TIMv1 -I../../ChibiOS-Contrib/os/hal/ports/STM32/LLD/USBHv1 -I../../ChibiOS-Contrib/os/hal/ports/STM32/LLD/CRCv1 -I../../ChibiOS-Contrib/os/hal/ports/STM32/LLD -Iboard_bezvodiatel -Iboard_mnu -I../../ChibiOS-RT/os/hal/lib/streams -I../../ChibiOS-RT/os/various/cpp_wrappers -I../../ChibiOS-RT/ext/fatfs/src -I../../ChibiOS-Contrib/os/various -Isrc/sensors -Isrc/sensors/gnss -Ilib/microrl/src -Ilib/uav_utils -Ilib/filters -Ilib/24aa/src -Ilib/tlsf -Isrc -Isrc/mavlink_local -Isrc/kinematic -Isrc/cli -Isrc/control -Isrc/fpga -Ilib/navi6d -Ilib/embmatrix/src -Ilib/mavlink/C/lapwing -Ibenchmark"

GCC_VERSION="5.3.1"
GCC_INCLUDES="-I/opt/arm-none-eabi/arm-none-eabi/include \
			-I/opt/arm-none-eabi/lib/gcc/arm-none-eabi/${GCC_VERSION}/include \
			-I/opt/arm-none-eabi/lib/gcc/arm-none-eabi/${GCC_VERSION}/include-fixed \
			-I/opt/arm-none-eabi/arm-none-eabi/include/c++/${GCC_VERSION}"

mkdir -p build
#cppcheck --force --enable=warning $GCC_INCLUDES $CHIBI_INCLUDES . 2> build/cppcheck_report.txt
#cppcheck --enable=warning $GCC_INCLUDES $CHIBI_INCLUDES . 2> build/cppcheck_report.txt
cppcheck --enable=warning ./src 2> build/cppcheck_report.txt

