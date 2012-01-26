# Automatically generated - DO NOT EDIT!
ADD_REQUIRED_DEPENDENCY(roboptim-trajectory >= 0.1)
ADD_REQUIRED_DEPENDENCY(hpp-model >= 0.1)
PKG_CHECK_MODULES([ROBOPTIM_CORE], [roboptim-core])
ROBOPTIM_CORE_PREFIX=`$PKG_CONFIG roboptim-core --variable=prefix`
ROBOPTIM_CORE_DOCDIR=`$PKG_CONFIG roboptim-core --variable=docdir`
AC_SUBST([ROBOPTIM_CORE_PREFIX])
AC_SUBST([ROBOPTIM_CORE_DOCDIR])

