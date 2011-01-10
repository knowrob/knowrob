INSTALL_DIR = prolog
INSTALL_FILES = jpl.jar jpl.pl libjpl.so

all: installed

BUILD_TOPDIR = $(shell pwd)

GIT_DIR = jpl
GIT_URL = git://www.swi-prolog.org/home/pl/git/packages/jpl.git
GIT_REVISION = cf3bf6129851daa18858548f09bc2349a1219dff
GIT_PATCH = configure.patch

$(GIT_DIR):
	git clone $(GIT_URL) $(GIT_DIR)
	cd $(GIT_DIR) && git checkout $(GIT_REVISION)
	touch rospack_nosubdirs

patched: $(GIT_PATCH) Makefile
	cd $(GIT_DIR) && git reset --hard
ifneq ($(strip $(GIT_PATCH)),)
	$(foreach PATCH, $(GIT_PATCH), cd $(GIT_DIR) && git apply $(BUILD_TOPDIR)/$(PATCH) && ) echo patched
	touch rospack_nosubdirs
	touch patched
endif   

installed: $(GIT_DIR) patched
	cd $(GIT_DIR) && autoconf
	cd $(GIT_DIR) && ./configure
	make -C $(GIT_DIR)
	mkdir -p $(INSTALL_DIR)
	$(foreach FILE, $(INSTALL_FILES), cp $(GIT_DIR)/$(FILE) $(INSTALL_DIR)/$(FILE) ; )
	touch installed

clean:
	-cd $(GIT_DIR) && make clean
	-rm -f patched
	-rm -f rospack_nosubdirs
	-rm -f .build_version
	-rm -f .rosgcov_files

wipe: clean
	-rm -f installed
	-rm -rf $(GIT_DIR)
	-rm -rf $(INSTALL_DIR)

.PHONY : clean download
