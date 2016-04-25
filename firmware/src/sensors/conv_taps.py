#!/usr/bin/env python
import re

f = open("mpu6050_fir_taps.h", "r")

for s in f:
    if re.match("///\* ", s):
        print ('\n')
        print (s)
    else:
        L = re.sub('^.*\[', '', s)
        L = re.sub('\].*$\n', '', L)
        taps = re.search("{.*}", s)
        if taps is not None:
            # print ("// static const std::array<float,", L, "> taps =", taps.group(0))
            print ('// static const std::array<float, {}> taps = {};'.format(L, taps.group(0)))
