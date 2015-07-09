#!/bin/bash

S="9 15 21 33"
M="6 9 10 13 16 17"

for i in $S
do
	for j in $M
	do
		make -f ../Makefile_tests STATE=$i MEAS=$j > /dev/null
		if [ $? -ne 0 ]; then
			echo "build FAILED. Exiting."
			exit
		else
			./kalman_test_${i}_${j}
			if [ $? -ne 0 ]; then
				echo "<$i, $j> FAILED. Exiting."
				exit
			else
				echo "<$i, $j>: OK"
			fi
		fi
	done
done

