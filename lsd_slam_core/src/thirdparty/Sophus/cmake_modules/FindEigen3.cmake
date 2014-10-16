# - Try to find Eigen3 lib
# Once done this will define
#
#  EIGEN3_FOUND - system has eigen lib
#  EIGEN3_INCLUDE_DIR - the eigen include directory

# Copyright (c) 2006, 2007 Montel Laurent, <montel@kde.org>
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

if( EIGEN3_INCLUDE_DIR )
    # in cache already
    set( EIGEN3_FOUND TRUE )
else (EIGEN3_INCLUDE_DIR)
    find_path( EIGEN3_INCLUDE_DIR NAMES Eigen/Core
        PATH_SUFFIXES eigen3/
        HINTS
        ${INCLUDE_INSTALL_DIR}
        /usr/local/include
        ${KDE4_INCLUDE_DIR}
        )
    include( FindPackageHandleStandardArgs )
    find_package_handle_standard_args( Eigen3 DEFAULT_MSG EIGEN3_INCLUDE_DIR )   
    mark_as_advanced( EIGEN3_INCLUDE_DIR )
endif(EIGEN3_INCLUDE_DIR)

