include $(shell rospack find mk)/cmake.mk

MAIN_DIR=$(shell pwd)
SOURCE_DIR=$(MAIN_DIR)/src/
BUILD_DIR=$(MAIN_DIR)/bin/
MODELS_DIR=$(MAIN_DIR)/models/


all: installed

checkedout:
	wget http://ias.cs.tum.edu/kb/probcog.tar.gz
	wget http://ias.cs.tum.edu/kb/models.tar.gz
	tar xvzf probcog.tar.gz
	tar xvzf models.tar.gz
	wget http://ias.cs.tum.edu/kb/fipm.jar && cp fipm.jar ./lib/
	wget http://ias.cs.tum.edu/kb/kipm.jar && cp kipm.jar ./lib/
	wget http://ias.cs.tum.edu/kb/fipmbase.jar && cp fipmbase.jar ./lib/
	wget http://ias.cs.tum.edu/kb/fipmmodels.jar && cp fipmmodels.jar ./lib/
	rm probcog.tar.gz
	rm models.tar.gz
	rm *.jar
	
	# replace global paths in the java files
	sed 's|/usr/wiss/jain/work/code/SRLDB/models/|$(MODELS_DIR)|g' $(MAIN_DIR)/src/edu/tum/cs/probcog/YarpServer.java > $(MAIN_DIR)/src/edu/tum/cs/probcog/YarpServer.java.1 && mv $(MAIN_DIR)/src/edu/tum/cs/probcog/YarpServer.java.1 $(MAIN_DIR)/src/edu/tum/cs/probcog/YarpServer.java
	sed 's|/usr/wiss/jain/work/code/SRLDB/models/|$(MODELS_DIR)|g' $(MAIN_DIR)/src/edu/tum/cs/probcog/Server.java > $(MAIN_DIR)/src/edu/tum/cs/probcog/Server.java.1 && mv $(MAIN_DIR)/src/edu/tum/cs/probcog/Server.java.1 $(MAIN_DIR)/src/edu/tum/cs/probcog/Server.java
	sed 's|/home/tenorth/work/srldb/models/|$(MODELS_DIR)|g' $(MAIN_DIR)/src/edu/tum/cs/probcog/prolog/PrologInterface.java > $(MAIN_DIR)/src/edu/tum/cs/probcog/prolog/PrologInterface.java.1 && mv $(MAIN_DIR)/src/edu/tum/cs/probcog/prolog/PrologInterface.java.1 $(MAIN_DIR)/src/edu/tum/cs/probcog/prolog/PrologInterface.java
	sed 's|/data/srldb/models|$(MODELS_DIR)|g' $(MAIN_DIR)/src/edu/tum/cs/probcog/prolog/PrologInterface.java > $(MAIN_DIR)/src/edu/tum/cs/probcog/prolog/PrologInterface.java.1 && mv $(MAIN_DIR)/src/edu/tum/cs/probcog/prolog/PrologInterface.java.1 $(MAIN_DIR)/src/edu/tum/cs/probcog/prolog/PrologInterface.java

	touch checkedout

installed: wiped checkedout	
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(BUILD_DIR) $(SOURCE_DIR) apps datagen examples experiments papers docs lib make_apps.py make_dist models python readme.txt trajvis trajvis.bat *.tar.gz build msg_gen srv_gen
	rm -f checkedout wiped installed

wipe: clean


