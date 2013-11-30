dnl Process this file with autoconf to produce a configure script.

AC_INIT(m3skin, 0.1, info@mekabot.com)
AC_CONFIG_AUX_DIR(config)
#AC_CONFIG_SRCDIR(src/m3skin/hardware/factory_proxy.cpp)
AM_INIT_AUTOMAKE

AM_CONFIG_HEADER(config.h)
AC_PROG_CXX
AC_PROG_CC
AC_DISABLE_STATIC
AC_PROG_LIBTOOL

#Swig Stuff
AM_PATH_PYTHON(2.3)
AC_PROG_SWIG(1.3.21)
SWIG_ENABLE_CXX
SWIG_PYTHON

AC_ARG_ENABLE([virtual-only],
[  --enable-virtual-only    Turn off realtime and ethercat],
[case "${enableval}" in
  yes) rtai=false ;;
  no)  rtai=true ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-virtual-only]) ;;
esac],[rtai=true])
AM_CONDITIONAL([RTAI], [test x$rtai = xtrue])

#------------------------------------------------------------------------------
# Eigen
#------------------------------------------------------------------------------
PKG_CHECK_MODULES([EIGEN2], [eigen2 >= 2.0.15])

AC_OUTPUT( Makefile src/m3skin/hardware/Makefile python/Makefile src/m3skin/printer/Makefile )


