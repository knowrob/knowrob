################################################################
# Makefile.  Generated from Makefile.in by configure.
#
# Build JPL.  Building JPL for Unix currently relies on the following
# assumptions:
#
#	* $JAVA_HOME points to the JDK top directory
#	* $PATH includes $JAVA_HOME/bin
#	* ELF Dynamic Linker semantics
#
# Author: Jan Wielemaker, based on shell-scripts from Paul Singleton.
################################################################

.SUFFIXES: .tex .dvi .doc .pl

PL=swipl
PLBASE=/usr/lib/swi-prolog
PLARCH=amd64
PLINCL=/usr/lib/swi-prolog/include
PKGDOC=$(PLBASE)/doc/packages
EXDIR=$(PKGDOC)/examples/jpl
PCEHOME=../xpce
DESTDIR=

CC=gcc
COFLAGS=-O2 -cc-options,-fno-strict-aliasing
CWFLAGS=-Wall
CMFLAGS=-fPIC -fno-stack-protector
CIFLAGS=-I. -I/usr/lib/swi-prolog/include -I/usr/lib/jvm/java-6-sun/include -I/usr/lib/jvm/java-6-sun/include/linux
CFLAGS=$(COFLAGS) $(CWFLAGS) $(CMFLAGS) $(CIFLAGS) -DPACKAGE_NAME=\"\" -DPACKAGE_TARNAME=\"\" -DPACKAGE_VERSION=\"\" -DPACKAGE_STRING=\"\" -DPACKAGE_BUGREPORT=\"\" -DPACKAGE_URL=\"\" -D_REENTRANT=1 -DSTDC_HEADERS=1 -DHAVE_SYS_TYPES_H=1 -DHAVE_SYS_STAT_H=1 -DHAVE_STDLIB_H=1 -DHAVE_STRING_H=1 -DHAVE_MEMORY_H=1 -DHAVE_STRINGS_H=1 -DHAVE_INTTYPES_H=1 -DHAVE_STDINT_H=1 -DHAVE_UNISTD_H=1 -DHAVE_WCHAR_H=1 -DSIZEOF_WCHAR_T=4 -DSIZEOF_VOIDP=8 -DSIZEOF_LONG=8 -DSIZEOF_LONG_LONG=8
LIBS=
NETLIBS=@NETLIBS@

JAVA_HOME=/usr/lib/jvm/java-6-sun
JAVAC=javac
JAVA=java
JUNIT=
JAVALIBS=-ljava -lverify -ljvm

DOCTOTEX=$(PCEHOME)/bin/doc2tex
PLTOTEX=$(PCEHOME)/bin/pl2tex
DOC=jpl
TEX=$(DOC).tex
DVI=$(DOC).dvi
PDF=$(DOC).pdf
RUNTEX=../../man/runtex

LD=ld
LDFLAGS=-shared -lc -lpl -L/usr/lib/jvm/java-6-sun/jre/lib/amd64/server -rpath /usr/lib/jvm/java-6-sun/jre/lib/amd64/server -L/usr/lib/jvm/java-6-sun/jre/lib/amd64 -rpath /usr/lib/jvm/java-6-sun/jre/lib/amd64

INSTALL=/usr/bin/install -c
INSTALL_PROGRAM=${INSTALL}
INSTALL_DATA=${INSTALL} -m 644
INSTALL_SCRIPT=${INSTALL}

LIBPL=		jpl.pl
TARGETS=	libjpl.so
OBJ=		src/c/jpl.o

all:		$(TARGETS) jpl.jar exjava-compile jpl_doc

libjpl.so:	$(OBJ)
		$(LD) $(LDFLAGS) -o $@ $(OBJ) $(JAVALIBS)
		if [ -r libjpl.dylib ]; then \
		  rm -f libjpl.jnilib && ln -s libjpl.dylib libjpl.jnilib ; \
		fi

jpl.jar::
		(cd src/java && $(MAKE) jpl_jar)
jpl_doc::
		(cd src/java && $(MAKE) jpl_doc)

################################################################
# Verify the package
################################################################

check:		check_pl check_java

check_pl:	jpltest.jar
		$(PL) -q -f test_jpl.pl -g run_tests,halt -t 'halt(1)'
check_java:	jpltest.jar
		CLASSPATH=$(JUNIT):jpl.jar:jpltest.jar $(JAVA) junit.textui.TestRunner jpl.test.TestJUnit

jpltest.jar:
		(cd src/java && $(MAKE) test_jar)

################################################################
# Installation
################################################################

DOCDIRS=	. ./java_api ./java_api/javadoc ./java_api/javadoc/jpl \
		./java_api/javadoc/jpl/class-use \
		./java_api/javadoc/jpl/fli \
		./java_api/javadoc/jpl/fli/class-use \
		./java_api/javadoc/resources \
		./prolog_api \
		./prolog_api/overview
DOCFILES=	$(shell cd ./docs && find . -name '*.html' -o -name '*.gif' -o -name '*.jpg')

EXPL=		$(EXDIR)/prolog
EXPLS=		jpl_colour_choose_demo.pl \
		jpl_jlist_demo.pl \
		jpl_midi_demo.pl \
		jpl_table_demo.pl \
		jpl_text_entry_demo.pl \
		jpl_versions_demo.pl
EXJAVA=		$(EXDIR)/java
EXJAVAS=	Exceptions Exceptions2 Family FamilyMT Test Test2 Time \
		Versions Zahed SemWeb


install:	$(TARGETS) $(LIBPL)
		mkdir -p $(DESTDIR)$(PLBASE)/lib/$(PLARCH)
		for f in $(TARGETS); do \
		  $(INSTALL_PROGRAM) $$f $(DESTDIR)$(PLBASE)/lib/$(PLARCH); \
		done
ifeq (so,dylib)		
		(cd $(DESTDIR)$(PLBASE)/lib/$(PLARCH) && ln -sf libjpl.so libjpl.jnilib)
endif
		mkdir -p $(DESTDIR)$(PLBASE)/library
		for f in $(LIBPL); do \
		  $(INSTALL_DATA) $$f $(DESTDIR)$(PLBASE)/library; \
		done
		$(INSTALL_DATA) jpl.jar $(DESTDIR)$(PLBASE)/lib
		$(PL) -f none -g make -t halt

ln-install::
		@$(MAKE) INSTALL_DATA='../ln-install' install

rpm-install:	install html-install

html-install::	expl-install exjava-install
		mkdir -p $(DESTDIR)$(PKGDOC)/jpl
		@echo -n "Creating directories "
		@for d in $(DOCDIRS); do \
		    mkdir -p $(DESTDIR)$(PKGDOC)/jpl/$$d; echo -n .; \
		done
		@echo "ok"
		@echo -n "Copying documentation files "
		@for f in $(DOCFILES); do \
		    $(INSTALL_DATA) docs/$$f $(DESTDIR)$(PKGDOC)/jpl/$$f; echo -n .; \
		done
		@echo "ok"

expl-install::
		echo "Installing Prolog examples"
		mkdir -p $(DESTDIR)$(EXPL)
		@for f in $(EXPLS) README; do \
		    $(INSTALL_DATA) examples/prolog/$$f $(DESTDIR)$(EXPL)/$$f; \
		done

exjava-compile:	jpl.jar
		echo "Compiling Java examples"
		JPLJAR=`pwd`/jpl.jar; export JPLJAR; \
		for d in $(EXJAVAS); do \
		    (cd examples/java/$$d && env JPL_COMPILE_ONLY=yes ./run.sh); \
		done

exjava-install:	exjava-compile
		echo "Installing Java examples"
		mkdir -p $(DESTDIR)$(EXJAVA)
		$(INSTALL_DATA) examples/java/README $(DESTDIR)$(EXJAVA)
		$(INSTALL_DATA) examples/java/env.sh $(DESTDIR)$(EXJAVA)
		for d in $(EXJAVAS); do \
		    mkdir -p $(DESTDIR)$(EXJAVA)/$$d; \
		    $(INSTALL_SCRIPT) examples/java/$$d/run.sh $(DESTDIR)$(EXJAVA)/$$d ;\
		    $(INSTALL_DATA) examples/java/$$d/README $(DESTDIR)$(EXJAVA)/$$d ;\
		    $(INSTALL_DATA) examples/java/$$d/$$d.java $(DESTDIR)$(EXJAVA)/$$d ;\
		    plfile=`echo $$d.pl | tr A-Z a-z`; \
		    if [ -f examples/java/$$d/$$plfile ]; then \
			$(INSTALL_DATA) examples/java/$$d/$$plfile $(DESTDIR)$(EXJAVA)/$$d ;\
		    fi; \
		    $(INSTALL_DATA) examples/java/$$d/$$d*.class $(DESTDIR)$(EXJAVA)/$$d ;\
		done
		$(INSTALL_DATA) examples/java/SemWeb/test.rdf $(DESTDIR)$(EXJAVA)/SemWeb

pdf-install::
		mkdir -p $(DESTDIR)$(PKGDOC)
		$(INSTALL) -m 644 $(DOC).pdf $(DESTDIR)$(PKGDOC)

uninstall::
		(cd $(PLBASE)/lib/$(PLARCH) && rm -f $(TARGETS))
		(cd $(PLBASE)/library && rm -f $(LIBPL))
		$(PL) -f none -g make -t halt

################################################################
# Documentation
################################################################

pdf:		$(PDF)

$(DVI):		$(TEX)
		$(RUNTEX) $(DOC)

$(PDF):		$(TEX)
		$(RUNTEX) --pdf $(DOC)

html:		$(TEX)
		latex2html $(DOC)
		mv html/index.html $(DOC).html
		rm -r html

$(TEX):		$(DOCTOTEX)

.doc.tex:
		$(DOCTOTEX) $*.doc > $*.tex
.pl.tex:
		$(PLTOTEX) $*.pl > $*.tex

################################################################
# Clean
################################################################

clean:
		rm -f $(OBJ) *~ *.o *% a.out core config.log
		rm -f TestJPL.class
		find examples/java -name '*.class' -delete
		(cd src/java && $(MAKE) clean)

distclean:	clean
		rm -rf autom4te.cache
		rm -f $(TARGETS) config.cache config.h config.status Makefile
		rm -f $(DOC).aux $(DOC).log $(DOC).out $(DOC).toc
		rm -rf html
		(cd src/java && $(MAKE) distclean)

