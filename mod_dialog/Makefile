include $(shell rospack find mk)/cmake.mk

MAIN_DIR=$(shell pwd)
LIB_DIR=$(MAIN_DIR)/lib/
BIN_DIR=$(MAIN_DIR)/bin/
BUILD_DIR=$(MAIN_DIR)/build/

all: installed

checkedout:
	mkdir lib
	wget http://ias.cs.tum.edu/kb/WSJ_8gau_13dCep_16k_40mel_130Hz_6800Hz.jar -O lib/WSJ_8gau_13dCep_16k_40mel_130Hz_6800Hz.jar
	wget http://ias.cs.tum.edu/kb/WSJ_8gau_13dCep_8kHz_31mel_200Hz_3500Hz.jar -O lib/WSJ_8gau_13dCep_8kHz_31mel_200Hz_3500Hz.jar
	wget http://ias.cs.tum.edu/kb/js.jar -O lib/js.jar
	wget http://ias.cs.tum.edu/kb/jsapi.jar -O lib/jsapi.jar
	wget http://ias.cs.tum.edu/kb/sphinx4.jar -O lib/sphinx4.jar

	touch checkedout

installed: wiped checkedout
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR) $(LIB_DIR)
	rm -f checkedout wiped installed

wipe: clean


