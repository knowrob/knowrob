include $(shell rospack find mk)/cmake.mk

MAIN_DIR=$(shell pwd)
SOURCE_DIR=$(MAIN_DIR)/src/
BUILD_DIR=$(MAIN_DIR)/build/
BIN_DIR=$(MAIN_DIR)/bin/
MODELS_DIR=$(MAIN_DIR)/models/

all: installed

checkedout:
	wget http://ias.cs.tum.edu/kb/wordnet.tar.gz
	wget http://ias.cs.tum.edu/kb/comp_ehow_libs.tar.gz
	wget http://ias.cs.tum.edu/kb/englishPCFG.ser.gz
	tar xzf wordnet.tar.gz
	mv wordnet etc/wordnet
	tar xzf comp_ehow_libs.tar.gz lib/
	mv englishPCFG.ser.gz etc/englishPCFG.ser.gz
	rm wordnet.tar.gz
	rm comp_ehow_libs.tar.gz
	touch checkedout


installed: checkedout
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR) etc/wordnet/*.rdf *.rdf protege.properties logs lib/* etc/englishPCFG.ser.gz
	rm -f checkedout wiped installed

wipe: clean

