copy firmware\lib\mavlink\definitions\lapwing.xml mavlink_generator\message_definitions\v1.0
cd mavlink_generator\pymavlink
python setup.py bdist --formats=wininst
move dist\* ..\..
deltree /Y dist
deltree /Y build
cd ..\..
del mavlink_generator\message_definitions\v1.0\lapwing.xml

