AC_PREREQ(2.64)
AC_INIT([mavlink-router],
	[1],
	[],
	[mavlink-router],
	[])

AC_CONFIG_SRCDIR([main.cpp])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_HEADERS(config.h)
AC_CONFIG_AUX_DIR([build-aux])

AC_USE_SYSTEM_EXTENSIONS
AC_SYS_LARGEFILE
AC_PREFIX_DEFAULT([/usr])
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


#####################################################################
# --with-
#####################################################################

AC_ARG_WITH([rootlibdir],
        AS_HELP_STRING([--with-rootlibdir=DIR], [rootfs directory to install shared libraries]),
        [], [with_rootlibdir=$libdir])
AC_SUBST([rootlibdir], [$with_rootlibdir])

AC_ARG_WITH([dialect],
            [AS_HELP_STRING([--with-dialect=common|ardupilotmega],[mavlink dialect to use [default=common]])],
            [],
            [with_dialect="common"])
AS_IF([test $with_dialect != ardupilotmega -a $with_dialect != common ],
      [AC_MSG_ERROR([invalid dialect $with_dialect. must be ´common´ or ´ardupilotmega´])],
      [])
AC_SUBST([DIALECT], [$with_dialect])

#####################################################################
# --enable-
#####################################################################

#####################################################################
# Default CFLAGS and LDFLAGS
#####################################################################

#####################################################################
# Generate files from *.in
#####################################################################

AC_CONFIG_FILES([
	Makefile
])

#####################################################################

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

	Mavlink dialect:	${with_dialect}
])
