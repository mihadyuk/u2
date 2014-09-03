# Process this file with GNU make
.PHONY: all mavlink_c clean

TOP_DIR = $(shell pwd)
MAVGEN_DIR = ../../../mavlink_generator

all: mavlink_c

mavlink_c: $(TOP_DIR)/C/lapwing/lapwing.h

# C headers
$(TOP_DIR)/C/lapwing/lapwing.h: $(TOP_DIR)/definitions/lapwing.xml $(MAVGEN_DIR)
	cp $(TOP_DIR)/definitions/lapwing.xml $(MAVGEN_DIR)/message_definitions/v1.0
	$(MAVGEN_DIR)/pymavlink/generator/mavgen.py --lang=C --wire-protocol=1.0 -o C $(MAVGEN_DIR)/message_definitions/v1.0/lapwing.xml
	rm $(MAVGEN_DIR)/message_definitions/v1.0/lapwing.xml

clean:
	rm -rf $(TOP_DIR)/C/lapwing
	rm -rf $(TOP_DIR)/C/common
	rm -rf $(TOP_DIR)/C/*.h
	rm -rf $(TOP_DIR)/C/*.hpp
