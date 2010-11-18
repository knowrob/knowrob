
PACKAGE_DIR=$(shell rospack find yason)

all: installed

checkedout:
	svn co svn://bknr.net/svn/trunk/libraries/yason/
	touch checkedout

installed: checkedout
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ${PACKAGE_DIR}/yason/yason.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf asdf yason rospack_nosubdirs installed checkedout

