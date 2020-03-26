AC_PREREQ(2.64)
AC_INIT([mavlink-router],
	[2],
	[],
	[mavlink-router],
	[https://github.com/01org/mavlink-router])

AC_CONFIG_SRCDIR([configure.ac])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_HEADERS(config.h)
AC_CONFIG_AUX_DIR([build-aux])

AC_USE_SYSTEM_EXTENSIONS
AC_SYS_LARGEFILE
AC_PREFIX_DEFAULT([/usr])
PKG_PROG_PKG_CONFIG
AM_MAINTAINER_MODE([enable])
AM_INIT_AUTOMAKE([check-news foreign 1.11 silent-rules tar-pax no-dist-gzip dist-xz subdir-objects color-tests parallel-tests])
AM_SILENT_RULES([yes])

#####################################################################
# Program checks and configurations
#####################################################################

AC_PROG_CC_C99
AX_CXX_COMPILE_STDCXX([11], [ext], [mandatory])
AM_PATH_PYTHON([2.7])

#####################################################################

LT_INIT([disable-static pic-only])

#####################################################################
# Function and structure checks
#####################################################################

AC_MSG_CHECKING([whether _Static_assert() is supported])
AC_COMPILE_IFELSE(
	[AC_LANG_SOURCE([[_Static_assert(1, "Test");]])],
        [AC_DEFINE([HAVE_STATIC_ASSERT], [1], [Define if _Static_assert() is available])
	 AC_MSG_RESULT([yes])],
	[AC_MSG_RESULT([no])])

AC_MSG_CHECKING([whether _Noreturn is supported])
AC_COMPILE_IFELSE(
	[AC_LANG_SOURCE([[_Noreturn int foo(void) { exit(0); }]])],
        [AC_DEFINE([HAVE_NORETURN], [1], [Define if _Noreturn is available])
	 AC_MSG_RESULT([yes])],
	[AC_MSG_RESULT([no])])

PKG_CHECK_MODULES([GTEST], [gtest_main], [HAVE_GTEST=true], [
	# Fallback to try linking it directly (required on some installations).
	# This assumes that having "gtest_main" implies we also have "gtest"
	# which seems reasonable.
	AC_CHECK_LIB(
		[gtest_main], [main], 
		[HAVE_GTEST=true] [GTEST_LIBS="-lgtest_main -lgtest"], 
		[HAVE_GTEST=false]) 
])
AM_CONDITIONAL([HAVE_GTEST], [test x$HAVE_GTEST = xtrue])
AC_SUBST([GTEST_LIBS])
AC_SUBST([GTEST_CFLAGS])

#####################################################################
# --with-
#####################################################################

AC_ARG_WITH([rootlibdir],
        AS_HELP_STRING([--with-rootlibdir=DIR], [rootfs directory to install shared libraries]),
        [], [with_rootlibdir=$libdir])
AC_SUBST([rootlibdir], [$with_rootlibdir])

AC_ARG_WITH([systemdsystemunitdir],
			AC_HELP_STRING([--with-systemdsystemunitdir=DIR],
			[path to systemd system unit directory]),
					[path_systemunitdir=${withval}])
if (test "${enable_systemd}" != "no" && test -z "${path_systemunitdir}"); then
	AC_MSG_CHECKING([systemd system unit dir])
	path_systemunitdir="`$PKG_CONFIG --variable=systemdsystemunitdir systemd`"
	if (test -z "${path_systemunitdir}"); then
		AC_MSG_ERROR([systemd system unit directory is required])
	fi
	AC_MSG_RESULT([${path_systemunitdir}])
fi

#####################################################################
# --enable-
#####################################################################

AC_ARG_ENABLE(systemd, AC_HELP_STRING([--disable-systemd],
		[disable systemd integration]), [enable_systemd=${enableval}])
AM_CONDITIONAL(SYSTEMD, test "${enable_systemd}" != "no")

#####################################################################
# Default CFLAGS and LDFLAGS
#####################################################################

# Check C compiler flag
AC_LANG_PUSH([C])
CC_CHECK_FLAGS_APPEND(with_cflags, [CFLAGS], [ \
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
	-Wuninitialized])
AC_SUBST([OUR_CFLAGS], "$with_cflags")
AC_LANG_POP([C])

# Check C++ compiler flags
AC_LANG_PUSH([C++])
CC_CHECK_FLAGS_APPEND(with_cxxflags, [CXXFLAGS], [ \
	-Wall \
	-W \
	-Waddress-of-packed-member \
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
	-Wuninitialized])
AC_LANG_POP([C++])

AS_VAR_IF([cc_cv_CXXFLAGS__Waddress_of_packed_member], [yes],
	  [with_cxxflags="$with_cxxflags -DHAVE_WADDRESS_OF_PACKED_MEMBER"])
AC_SUBST([OUR_CXXFLAGS], "$with_cxxflags")

#####################################################################
# Generate files from *.in
#####################################################################

AC_CONFIG_FILES([
	Makefile
])

#####################################################################
AC_SUBST(SYSTEMD_SYSTEMUNITDIR, [${path_systemunitdir}])

AC_OUTPUT
AC_MSG_RESULT([
	$PACKAGE $VERSION
	================

	prefix:			${prefix}
	sysconfdir:		${sysconfdir}
	libdir:			${libdir}
	rootlibdir:		${rootlibdir}
	includedir:		${includedir}
	bindir:			${bindir}

	C compiler:		${CC}
	C++ compiler:		${CXX}
])
