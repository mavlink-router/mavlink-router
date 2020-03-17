DISTCLEAN_LOCAL_HOOKS =
EXTRA_DIST =
CLEANFILES = $(BUILT_FILES)
noinst_LTLIBRARIES =
bin_PROGRAMS =
check_PROGRAMS =
noinst_PROGRAMS =
noinst_SCRIPTS =
BUILT_FILES =
ACLOCAL_AMFLAGS = -I m4 ${ACLOCAL_FLAGS}
AM_MAKEFLAGS = --no-print-directory
HAVE_GTEST = @HAVE_GTEST@
GTEST_LIBS = @GTEST_LIBS@
GTEST_CFLAGS = @GTEST_CFLAGS@

GCC_COLORS ?= 'yes'
export GCC_COLORS

BUILT_SOURCES = include/mavlink/ardupilotmega/mavlink.h

clean-local:
	rm -rf $(top_builddir)/include/mavlink
	rm -f $(top_builddir)/tests/unit_test

include/mavlink/ardupilotmega/mavlink.h: modules/mavlink/pymavlink/tools/mavgen.py modules/mavlink/message_definitions/v1.0/ardupilotmega.xml
	$(AM_V_GEN)python3 $(srcdir)/modules/mavlink/pymavlink/tools/mavgen.py \
		-o include/mavlink \
		--lang C \
		--wire-protocol 2.0 \
		$(srcdir)/modules/mavlink/message_definitions/v1.0/ardupilotmega.xml

#if SYSTEMD
systemdsystemunitdir = @SYSTEMD_SYSTEMUNITDIR@
systemdsystemunit_DATA = mavlink-router.service
#endif


AM_CPPFLAGS = \
	-include $(top_builddir)/config.h \
	-I$(top_builddir) \
	-I$(top_srcdir)/src \
	-I$(top_builddir)/include/mavlink \
	-I$(top_builddir)/include/mavlink/ardupilotmega \
	-DSYSCONFDIR=\""$(sysconfdir)"\"

AM_CFLAGS = \
	-pipe \
	-fno-common \
	-fdiagnostics-show-option \
	-fvisibility=hidden \
	-ffunction-sections \
	-fdata-sections \
	$(OUR_CFLAGS)

AM_CXXFLAGS = \
	-pipe \
	-fno-common \
	-fdiagnostics-show-option \
	-fvisibility=hidden \
	-ffunction-sections \
	-fdata-sections \
	$(OUR_CXXFLAGS)

AM_LDFLAGS = \
	-Wl,--as-needed \
	-Wl,--no-undefined \
	-Wl,--gc-sections \
	-pthread \
	-lrt

bin_PROGRAMS += mavlink-routerd
mavlink_routerd_SOURCES = \
	src/mavlink-router/autolog.cpp \
	src/mavlink-router/autolog.h \
	src/mavlink-router/binlog.cpp \
	src/mavlink-router/binlog.h \
	src/mavlink-router/comm.h \
	src/common/conf_file.cpp \
	src/common/conf_file.h \
	src/common/dbg.h \
	src/common/mavlink.h \
	src/mavlink-router/endpoint.cpp \
	src/mavlink-router/endpoint.h \
	src/common/log.cpp \
	src/common/log.h \
	src/mavlink-router/logendpoint.cpp \
	src/mavlink-router/logendpoint.h \
	src/common/macro.h \
	src/mavlink-router/main.cpp \
	src/mavlink-router/mainloop.cpp \
	src/mavlink-router/mainloop.h \
	src/mavlink-router/pollable.h \
	src/mavlink-router/pollable.cpp \
	src/mavlink-router/timeout.h \
	src/mavlink-router/timeout.cpp \
	src/mavlink-router/ulog.h \
	src/mavlink-router/ulog.cpp \
	src/common/util.c \
	src/common/util.h \
	src/common/xtermios.cpp \
	src/common/xtermios.h

noinst_PROGRAMS += heartbeat-print
heartbeat_print_SOURCES = \
	examples/heartbeat-print.cpp

noinst_PROGRAMS += px4-offboard-mode
px4_offboard_mode_SOURCES = \
	examples/px4-offboard-mode.cpp \
	src/common/util.c

noinst_PROGRAMS += arm-authorizer
arm_authorizer_SOURCES = \
	examples/arm-authorizer.cpp

SED_PROCESS = $(AM_V_GEN) $(MKDIR_P) $(dir $@) && \
	 $(SED) -e 's,@bindir\@,$(bindir),g' \
	 < $< > $@

%.service: %.service.in Makefile
	$(SED_PROCESS)

CLEANFILES += mavlink-router.service

noinst_SCRIPTS += examples/heartbeat-print.py

TESTS = \
	tests/routing_test.py


TEST_EXTENSIONS = .py
PY_LOG_COMPILER = $(PYTHON)

# ------------------------------------------------------------------------------
# unit tests
# ------------------------------------------------------------------------------

if HAVE_GTEST
check_PROGRAMS += mainloop_test
TESTS += mainloop_test
endif

mainloop_test_SOURCES = \
	src/common/log.cpp \
	src/common/log.h \
	src/common/util.c \
	src/common/util.h \
	src/mavlink-router/endpoint.cpp \
	src/mavlink-router/endpoint.h \
	src/mavlink-router/mainloop_test.cpp \
	src/mavlink-router/mainloop.cpp \
	src/mavlink-router/pollable.cpp \
	src/mavlink-router/pollable.h \
	src/mavlink-router/timeout.cpp \
	src/mavlink-router/timeout.h
mainloop_test_LDADD = $(GTEST_LIBS)

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

# ------------------------------------------------------------------------------
# mkosi
# ------------------------------------------------------------------------------

mkosi:
	mkosi -C $(top_srcdir)/tests/osi --build-sources ../../ -f
