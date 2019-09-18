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

BUILT_SOURCES = include/mavlink/ardupilotmega/mavlink.h

clean-local:
	rm -rf $(top_builddir)/include/mavlink
	rm -f $(top_builddir)/gtest.a
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

TESTS = unit_test \
		tests/routing_test.py

TEST_EXTENSIONS = .py
PY_LOG_COMPILER = $(PYTHON)

# ------------------------------------------------------------------------------
# gtest
# ------------------------------------------------------------------------------
GTEST_ROOT = $(top_srcdir)/modules/gtest/googletest
GTEST_CPPFLAGS = -isystem $(GTEST_ROOT)/include
GTEST_CXXFLAGS = -g -Wall -Wextra -pthread
GTEST_SRCS_ = $(GTEST_ROOT)/src/*.cc $(GTEST_ROOT)/src/*.h $(GTEST_HEADERS)

gtest-all.o: $(GTEST_SRCS_)
	$(CXX) $(GTEST_CPPFLAGS) -I$(GTEST_ROOT) $(GTEST_CXXFLAGS) -c \
		$(GTEST_ROOT)/src/gtest-all.cc

gtest.a : gtest-all.o
	$(AR) $(ARFLAGS) $@ $^

unit_test: $(top_srcdir)/tests/*.cpp gtest.a $(GTEST_HEADERS)
	$(CXX) $(GTEST_CPPFLAGS) $(GTEST_CXXFLAGS) -lpthread $^ -o $@

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
