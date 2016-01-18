#!/bin/bash

FLASH_SIZE=`cat build/ch.map | grep '^flash.*.[0..9]' | awk '{print$3}'`
FLASH_USED=`arm-none-eabi-size -B build/ch.elf | tail -1 | awk '{print $1 + $2}'`

RAM_ALIAS=ram0
RAM_SIZE=`cat build/ch.map | grep "__${RAM_ALIAS}_size__ =" | awk '{print$1}'`
STACKS=`arm-none-eabi-size -A build/ch.elf | grep .stacks | awk '{print $2}'`
BSS=`arm-none-eabi-size -A build/ch.elf | grep .bss | awk '{print $2}'`
DATA=`arm-none-eabi-size -A build/ch.elf | grep .data | awk '{print $2}'`

if [ ! $BSS ] ; then BSS=0; fi
if [ ! $DATA ] ; then DATA=0; fi
let RAM_USED=$STACKS+$BSS+$DATA

CCM_ALIAS=ram4
CCM_SIZE=`cat build/ch.map | grep "__${CCM_ALIAS}_size__ =" | awk '{print$1}'`
CCM_USED=`arm-none-eabi-size -A build/ch.elf | grep "."${CCM_ALIAS}"*[ \t]" | awk '{print $2}'`

echo "---------------------------------------------------------------------------------------"

let "RAM_SIZE		= RAM_SIZE" # to convert from hex to dec
let "RAM_PERCENT	= (100 * RAM_USED) / RAM_SIZE"
echo "SRAM:  $RAM_USED / $RAM_SIZE ($RAM_PERCENT%)"

let "FLASH_SIZE		= FLASH_SIZE" # to convert from hex to dec
let "FLASH_PERCENT	= (100 * FLASH_USED) / FLASH_SIZE"
echo "FLASH: $FLASH_USED / $FLASH_SIZE ($FLASH_PERCENT%)"

if (( $CCM_SIZE > 0 )) ; then
	let "CCM_SIZE       = CCM_SIZE" # to convert from hex to dec
	let "CCM_PERCENT    = (100 * CCM_USED) / CCM_SIZE"
	echo "CCM:   $CCM_USED / $CCM_SIZE ($CCM_PERCENT%)"
fi
