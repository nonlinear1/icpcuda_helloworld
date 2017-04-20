# - Find the libfreenect2 includes and library
# This module defines
#  FREENECT2_INCLUDE_DIR, path to libfreenect/libfreenect.h, etc.
#  FREENECT2_LIBRARIES, the libraries required to use FREENECT.
#  FREENECT2_FOUND, If false, do not try to use FREENECT.
# also defined, but not for general use are
#  FREENECT2_freenect_LIBRARY, where to find the FREENECT library.

FIND_PATH(FREENECT2_INCLUDE_DIR libfreenect2/libfreenect2.hpp
  /usr/local/include
  /usr/include
  $ENV{HOME}/freenect2/include
  $ENV{HOME}/libfreenect2/include
  NO_DEFAULT_PATH
)

FIND_LIBRARY(FREENECT2_LIBRARY freenect2
  /usr/local/lib
  /usr/lib
  $ENV{HOME}/freenect2/lib
  $ENV{HOME}/libfreenect2/lib
)

MARK_AS_ADVANCED(
  FREENECT2_INCLUDE_DIR
  FREENECT2_LIBRARY)

SET( FREENECT2_FOUND "NO" )
IF(FREENECT2_INCLUDE_DIR)
  IF(FREENECT2_LIBRARY)
    SET( FREENECT2_FOUND "YES" )
    SET( FREENECT2_LIBRARIES
      ${FREENECT2_LIBRARY})
  ENDIF(FREENECT2_LIBRARY)
ENDIF(FREENECT2_INCLUDE_DIR)

IF(FREENECT2_FOUND)
    MESSAGE(STATUS "Found freenect2:")
    MESSAGE(STATUS " - Includes: ${FREENECT2_INCLUDE_DIR}")
    MESSAGE(STATUS " - Libraries: ${FREENECT2_LIBRARIES}")
ELSE(FREENECT2_FOUND)
  IF(FREENECT2_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find libfreenect2
-- please give some paths to CMake or make sure libfreenect2 is installed in your system")
  ELSE(FREENECT2_FIND_REQUIRED)
    MESSAGE(STATUS "Could not find libfreenect2
-- please give some paths to CMake or make sure libfreenect2 is installed in your system")
  ENDIF(FREENECT2_FIND_REQUIRED)
ENDIF(FREENECT2_FOUND)
