include $(shell rospack find mk)/cmake.mk

MAIN_DIR=$(shell pwd)
SOURCE_DIR=$(MAIN_DIR)/src/
BUILD_DIR=$(MAIN_DIR)/build/
BIN_DIR=$(MAIN_DIR)/bin/


all: installed

checkedout:
	wget -O opencyc-2.0-linux.tgz http://sourceforge.net/projects/opencyc/files/OpenCyc%202.0/opencyc-2.0-linux.tgz/download
	touch checkedout


installed: checkedout
	tar xzf opencyc-2.0-linux.tgz
	rm opencyc-2.0-linux.tgz
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR) opencyc-2.0 *.tgz
	rm -f checkedout wiped installed

wipe: clean

