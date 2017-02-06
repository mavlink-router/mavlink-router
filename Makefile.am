DISTCLEAN_LOCAL_HOOKS =
EXTRA_DIST =
CLEANFILES = $(BUILT_FILES)
noinst_LTLIBRARIES =
bin_PROGRAMS =
noinst_PROGRAMS =
noinst_SCRIPTS =
BUILT_FILES =
ACLOCAL_AMFLAGS = -I m4 ${ACLOCAL_FLAGS}
AM_MAKEFLAGS = --no-print-directory



GCC_COLORS ?= 'yes'
export GCC_COLORS

BUILT_SOURCES = include/mavlink/common/mavlink.h

clean-local:
	rm -rf $(top_builddir)/include/mavlink

include/mavlink/common/mavlink.h: modules/mavlink/pymavlink/tools/mavgen.py modules/mavlink/message_definitions/v1.0/common.xml
	$(AM_V_GEN)python2 $(srcdir)/modules/mavlink/pymavlink/tools/mavgen.py \
		-o include/mavlink \
		--lang C \
		--wire-protocol 2.0 \
		$(srcdir)/modules/mavlink/message_definitions/v1.0/common.xml

AM_CPPFLAGS = \
	-include $(top_builddir)/config.h \
	-I$(top_builddir) \
	-I$(top_srcdir) \
	-I$(top_builddir)/include/mavlink \
	-I$(top_builddir)/include/mavlink/common \
	-DSYSCONFDIR=\""$(sysconfdir)"\"

AM_CFLAGS = \
	-pipe \
	-Wall \
	-W \
	-Wextra \
	-Wno-inline \
	-Wundef \
	-Wformat=2 \
	-Wlogical-op \
	-Wsign-compare \
	-Wformat-security \
	-Wmissing-include-dirs \
	-Wformat-nonliteral \
	-Wold-style-definition \
	-Wpointer-arith \
	-Winit-self \
	-Wdeclaration-after-statement \
	-Wfloat-equal \
	-Wmissing-prototypes \
	-Wstrict-prototypes \
	-Wredundant-decls \
	-Wmissing-declarations \
	-Wmissing-noreturn \
	-Wshadow \
	-Wendif-labels \
	-Wstrict-aliasing=3 \
	-Wwrite-strings \
	-Wno-long-long \
	-Wno-overlength-strings \
	-Wno-unused-parameter \
	-Wno-missing-field-initializers \
	-Wno-unused-result \
	-Wnested-externs \
	-Wchar-subscripts \
	-Wtype-limits \
	-Wuninitialized \
	-fno-common \
	-fdiagnostics-show-option \
	-fvisibility=hidden \
	-ffunction-sections \
	-fdata-sections

AM_CXXFLAGS = \
	-pipe \
	-Wall \
	-W \
	-Wextra \
	-Wno-inline \
	-Wundef \
	-Wformat=2 \
	-Wlogical-op \
	-Wsign-compare \
	-Wformat-security \
	-Wmissing-include-dirs \
	-Wformat-nonliteral \
	-Wpointer-arith \
	-Winit-self \
	-Wfloat-equal \
	-Wredundant-decls \
	-Wmissing-declarations \
	-Wmissing-noreturn \
	-Wshadow \
	-Wendif-labels \
	-Wstrict-aliasing=3 \
	-Wwrite-strings \
	-Wno-long-long \
	-Wno-overlength-strings \
	-Wno-unused-parameter \
	-Wno-missing-field-initializers \
	-Wno-unused-result \
	-Wchar-subscripts \
	-Wtype-limits \
	-Wuninitialized \
	-fno-common \
	-fdiagnostics-show-option \
	-fvisibility=hidden \
	-ffunction-sections \
	-fdata-sections

AM_LDFLAGS = \
	-Wl,--as-needed \
	-Wl,--no-undefined \
	-Wl,--gc-sections

bin_PROGRAMS += mavlink-routerd
mavlink_routerd_SOURCES = \
	comm.h \
	endpoint.cpp \
	endpoint.h \
	log.c \
	log.h \
	macro.h \
	main.cpp \
	mainloop.cpp \
	mainloop.h \
	pollable.h \
	pollable.cpp \
	router.h \
	util.c \
	util.h

noinst_PROGRAMS += heartbeat-print
heartbeat_print_SOURCES = \
	examples/heartbeat-print.cpp

noinst_SCRIPTS += examples/heartbeat-print.py

# ------------------------------------------------------------------------------
# coverity
# ------------------------------------------------------------------------------

mavlink-router-coverity-%.tar.xz:
	rm -rf $< cov-int
	./configure CFLAGS='-g -O2' CXXFLAGS='-g -O2' --sysconfdir=/etc --localstatedir=/var --libdir=/usr/lib
	make clean
	cov-build --dir cov-int make -j 4
	tar caf $@ cov-int

coverity-tar: mavlink-router-coverity-$(shell git rev-parse --short HEAD 2>/dev/null).tar.xz

coverity-sync: mavlink-router-coverity-$(shell git rev-parse --short HEAD 2>/dev/null).tar.xz
	@echo "uploading coverity tarball"
	@echo $(COVERITY_MAVLINK_ROUTER_TOKEN)
	@echo $(COVERITY_MAVLINK_ROUTER_USER)
	@curl --form token=$(COVERITY_MAVLINK_ROUTER_TOKEN) \
		--form email=$(COVERITY_MAVLINK_ROUTER_USER) \
		--form file=@$< \
		--form version="$(shell git rev-parse --short HEAD 2>/dev/null)" \
		--form description="" \
		https://scan.coverity.com/builds?project=01org%2Fmavlink-router

coverity-clean:
	rm -rf mavlink-router-coverity-*.tar.xz cov-int
