# - Find the libopt includes and library
# This module defines
#  OPT_INCLUDE_DIR, path to libfreenect/libfreenect.h, etc.
#  OPT_LIBRARIES, the libraries required to use FREENECT.
#  OPT_FOUND, If false, do not try to use FREENECT.
# also defined, but not for general use are
#  OPT_freenect_LIBRARY, where to find the FREENECT library.

FIND_PATH(OPT_INCLUDE_DIR Opt.h
  PATHS
  /usr/local/include
  /usr/include
  ${OPT_ROOT_DIR}
  $ENV{HOME}/Opt
  $ENV{OPT_ROOT_DIR}
  ${OPT_ROOT_DIR}
  PATH_SUFFIXES
  Opt
  include
  release/include
  API/release/include
)

FIND_LIBRARY(OPT_LIBRARY NAMES libOpt.a
  PATHS
  /usr/local/lib
  /usr/lib
  ${OPT_ROOT_DIR}
  $ENV{HOME}/Opt
  $ENV{OPT_ROOT_DIR}
  ${OPT_ROOT_DIR}
  PATH_SUFFIXES
  Opt
  lib
  release/lib
  API/release/lib
)

MARK_AS_ADVANCED(
  OPT_INCLUDE_DIR
  OPT_LIBRARY)

SET( OPT_FOUND "NO" )
IF(OPT_INCLUDE_DIR)
  IF(OPT_LIBRARY)
    SET( OPT_FOUND "YES" )
    SET( OPT_LIBRARIES
      ${OPT_LIBRARY})
  ENDIF(OPT_LIBRARY)
ENDIF(OPT_INCLUDE_DIR)

IF(OPT_FOUND)
    MESSAGE(STATUS "Found opt:")
    MESSAGE(STATUS " - Includes: ${OPT_INCLUDE_DIR}")
    MESSAGE(STATUS " - Libraries: ${OPT_LIBRARIES}")
ELSE(OPT_FOUND)
  IF(OPT_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find libOpt
-- please give some paths to CMake or make sure libOpt is installed in your system")
  ELSE(OPT_FIND_REQUIRED)
    MESSAGE(STATUS "Could not find libOpt
-- please give some paths to CMake or make sure libOpt is installed in your system")
  ENDIF(OPT_FIND_REQUIRED)
ENDIF(OPT_FOUND)
