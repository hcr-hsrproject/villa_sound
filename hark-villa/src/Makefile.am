#override datadir=$(FLOWDESIGNER_DATA)
override datadir=${prefix}/lib/flowdesigner/toolbox/hark-villa
#override libdir=$(FLOWDESIGNER_DATA)
override libdir=${prefix}/lib/flowdesigner/toolbox/hark-villa

lib_LTLIBRARIES = libhark-villa.la

install-data-local:
	mkdir -p $(DESTDIR)$(datadir)
	(if [ -f $(DESTDIR)$(libdir)/lib$(PACKAGE) ]; then cd $(DESTDIR)$(libdir); rm -f $(PACKAGE).tlb; ln -s lib$(PACKAGE) $(PACKAGE).tlb; else cd $(DESTDIR)$(libdir); rm -f $(PACKAGE).tlb; ln -s lib$(PACKAGE).so $(PACKAGE).tlb; fi)
	(info2def.pl $(libhark_villa_la_SOURCES) > $(DESTDIR)$(datadir)/hark-villa.def)

libhark_villa_la_SOURCES = \
	VERSION.cc \
	VillaCMLoad.cc \
	VillaSharedMicrophone.cc \
	VillaReadSharedMicrophone.cc

libhark_villa_la_CXXFLAGS = -O3 -march=core2 -mtune=core2 -mmmx -msse -msse2 -msse3 -mfpmath=sse @GTK_CFLAGS@

libhark_villa_la_LDFLAGS = -release $(LT_RELEASE) \
	`flowdesigner-config --libs config libflow audio_blocks` \
	`pkg-config libharkio --libs`

AM_CPPFLAGS  = `flowdesigner-config --cflags config libflow audio_blocks` \
	-I ../include

#libhark_villa_la_LIBADD = -lboost_thread
libhark_villa_la_LIBADD = @BOOST_THREAD_JUDGE_LIB@
