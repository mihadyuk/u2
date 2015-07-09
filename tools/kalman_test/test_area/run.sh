#!/bin/bash

S="9 15 21 33"
M="6 9 10 13 16 17"

for i in $S
do
	for j in $M
	do
		echo "CC  <$i, $j>"
		make -f ../Makefile_tests STATE=$i MEAS=$j > /dev/null
		if [ $? -ne 0 ]; then
			echo "build FAILED. Exiting."
			exit
		else
			./kalman_test_${i}_${j}
			if [ $? -ne 0 ]; then
				echo "run <$i, $j> FAILED. Exiting."
				exit
			else
				echo "run <$i, $j>: OK"
			fi
			rm ./kalman_test_${i}_${j}
		fi
	done
done

