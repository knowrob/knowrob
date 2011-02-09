include $(shell rospack find mk)/cmake.mk

MAIN_DIR=$(shell pwd)
MARY_DIR=$(shell pwd)/mary
LIB_DIR=$(MAIN_DIR)/lib/
BIN_DIR=$(MAIN_DIR)/bin/
BUILD_DIR=$(MAIN_DIR)/build/

all: inst

checkedout:
	svn co https://mary.opendfki.de/repos/tags/4.1.1 $(MARY_DIR)
# 	wget http://mary.dfki.de/download/4.1.0/mary-en-GB-4.1.0.zip
	wget http://mary.dfki.de/download/4.1.0/mary-dfki-obadiah-4.1.0.zip
# 	wget http://mary.dfki.de/download/4.1.0/mary-dfki-obadiah-hsmm-4.1.0.zip
# 	wget http://mary.dfki.de/download/4.1.0/mary-dfki-poppy-4.1.0.zip
# 	wget http://mary.dfki.de/download/4.1.0/mary-dfki-poppy-hsmm-4.1.0.zip
	wget http://mary.dfki.de/download/4.1.0/mary-dfki-prudence-4.1.0.zip
# 	wget http://mary.dfki.de/download/4.1.0/mary-dfki-prudence-hsmm-4.1.0.zip
# 	wget http://mary.dfki.de/download/4.1.0/mary-dfki-spike-4.1.0.zip
# 	wget http://mary.dfki.de/download/4.1.0/mary-dfki-spike-hsmm-4.1.0.zip
	touch checkedout
 
inst: wiped checkedout
	ant -f $(MARY_DIR)/build.xml
#	unzip mary-en-GB-4.1.0.zip
#	rm mary-en-GB-4.1.0.zip
	unzip mary-dfki-obadiah-4.1.0.zip
	rm mary-dfki-obadiah-4.1.0.zip
# 	unzip mary-dfki-obadiah-hsmm-4.1.0.zip
# 	rm mary-dfki-obadiah-hsmm-4.1.0.zip
# 	unzip mary-dfki-poppy-4.1.0.zip
# 	rm mary-dfki-poppy-4.1.0.zip
# 	unzip mary-dfki-poppy-hsmm-4.1.0.zip
# 	rm mary-dfki-poppy-hsmm-4.1.0.zip
	unzip mary-dfki-prudence-4.1.0.zip
	rm mary-dfki-prudence-4.1.0.zip
# 	unzip mary-dfki-prudence-hsmm-4.1.0.zip
# 	rm mary-dfki-prudence-hsmm-4.1.0.zip
# 	unzip mary-dfki-spike-4.1.0.zip
# 	rm mary-dfki-spike-4.1.0.zip
# 	unzip mary-dfki-spike-hsmm-4.1.0.zip
# 	rm mary-dfki-spike-hsmm-4.1.0.zip
	touch inst
# 
wiped: Makefile
	make wipe
	touch wiped
 
wipe:
	rm -rf $(BIN_DIR) $(LIB_DIR) $(MARY_DIR)
	rm -rf java download examples installed lib log netbeans tmp conf dist doc gpl-3.0.txt lgpl-3.0.txt MARY_software_user_agreement.txt README.usingEclipse.txt Changelog.txt .project .classpath build.xml
	rm -f checkedout wiped inst *.zip

clean: wipe

