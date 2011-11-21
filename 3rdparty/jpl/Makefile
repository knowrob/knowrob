INSTALL_DIR = prolog
INSTALL_FILES = jpl.jar jpl.pl libjpl.so

all: installed

BUILD_TOPDIR = $(shell pwd)

JPL_DIR = jpl
JPL_URL = http://code.in.tum.de/files/jpl.tar.gz
LIBSWIPL=/usr/lib/libswipl.so

$(JPL_DIR):
	wget $(JPL_URL)
	tar xvzf jpl.tar.gz
	rm jpl.tar.gz
	touch rospack_nosubdirs

installed: $(JPL_DIR)
	cd $(JPL_DIR) && autoconf
	sed 's/-cc-options\,//' jpl/configure > jpl/configure.sed; mv jpl/configure.sed jpl/configure; chmod +x jpl/configure
	if test -e $(LIBSWIPL); then sed 's/-lpl/-lswipl/' jpl/configure > jpl/configure.sed; mv jpl/configure.sed jpl/configure; chmod +x jpl/configure; fi
	cd $(JPL_DIR) && ./configure
	make -C $(JPL_DIR)
	mkdir -p $(INSTALL_DIR)
	$(foreach FILE, $(INSTALL_FILES), cp $(JPL_DIR)/$(FILE) $(INSTALL_DIR)/$(FILE) ; )
	touch installed

clean:
	-cd $(JPL_DIR) && make clean
	-rm -f rospack_nosubdirs
	-rm -f .build_version
	-rm -f .rosgcov_files

wipe: clean
	-rm -f installed
	-rm -rf $(JPL_DIR)
	-rm -rf $(INSTALL_DIR)

.PHONY : clean download
