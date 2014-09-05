#!/bin/bash

cp firmware/lib/mavlink/definitions/lapwing.xml mavlink_generator/message_definitions/v1.0/
cd mavlink_generator/pymavlink
PYTHONPATH=generator python setup.py bdist --formats=wininst --plat-name=win32
mv dist/* ../..
rm -rf dist
rm -rf build
cd ../..
rm mavlink_generator/message_definitions/v1.0/lapwing.xml

