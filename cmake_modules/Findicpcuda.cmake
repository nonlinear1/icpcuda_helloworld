# - Try to find ICPCUDA lib
#
# Once done this will define
#
#  ICPCUDA_FOUND - system has icpcuda lib
#  ICPCUDA_INCLUDE_DIR - the icpcuda include directory

# Copyright (c) 2017 theobslhc <theobslhc@github.com>
# Redistribution and use is allowed according to the terms of the 2-clause BSD license.

find_package(PkgConfig)
pkg_check_modules(ICPCUDA QUIET icpcuda)

find_path(ICPCUDA_INCLUDE_DIR icpcuda/ICPOdometry.h
          PATHS
          $ENV{HOME}
          ${CMAKE_INSTALL_PREFIX}/include
          ${CMAKE_SOURCE_DIR}/../build/include
          ${CMAKE_SOURCE_DIR}/build/include
          PATH_SUFFIXES icpcuda )

find_library(ICPCUDA_LIBRARY NAMES libicpcuda icpcuda
             PATHS
             $ENV{HOME}
             ${CMAKE_INSTALL_PREFIX}/lib
             ${CMAKE_SOURCE_DIR}/../build/lib
             ${CMAKE_SOURCE_DIR}/build/lib
             PATH_SUFFIXES icpcuda )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ICPCUDA_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(icpcuda  DEFAULT_MSG
                                  ICPCUDA_LIBRARY ICPCUDA_INCLUDE_DIR)

mark_as_advanced(ICPCUDA_INCLUDE_DIR ICPCUDA_LIBRARY )

set(ICPCUDA_LIBRARIES ${ICPCUDA_LIBRARY} )
set(ICPCUDA_INCLUDE_DIRS ${ICPCUDA_INCLUDE_DIR} )
