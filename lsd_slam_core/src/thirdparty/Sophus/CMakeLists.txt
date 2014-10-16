SET(PROJECT_NAME Sophus)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET (CMAKE_VERBOSE_MAKEFILE ON)

# Release by default
# Turn on Debug with "-DCMAKE_BUILD_TYPE=Debug"
IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
ENDIF()

IF (CMAKE_COMPILER_IS_GNUCXX )
   SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
   SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ")

  ADD_DEFINITIONS("-Wall -Werror -Wno-unused-variable
                   -Wno-unused-but-set-variable -Wno-unknown-pragmas ")
ENDIF()

################################################################################
# Add local path for finding packages, set the local version first
set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" )
list( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )

################################################################################
# Create variables used for exporting in SophusConfig.cmake
set( Sophus_LIBRARIES "" )
set( Sophus_INCLUDE_DIR ${PROJECT_SOURCE_DIR} )

################################################################################

include(FindEigen3.cmake)
#find_package( Eigen3 REQUIRED )
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )
SET( Sophus_INCLUDE_DIR ${Sophus_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} )

SET (SOURCE_DIR "sophus")

SET (TEMPLATES tests
               so2
               se2
               so3
               se3
               rxso3
               sim3
)

SET (SOURCES ${SOURCE_DIR}/sophus.hpp)

FOREACH(templ ${TEMPLATES})
  LIST(APPEND SOURCES ${SOURCE_DIR}/${templ}.hpp)
ENDFOREACH(templ)


INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Added ${SOURCES} to executables so they show up in QtCreator (and possibly
# other IDEs).
# ADD_EXECUTABLE(test_so2 sophus/test_so2.cpp ${SOURCES})
# ADD_EXECUTABLE(test_se2 sophus/test_se2.cpp ${SOURCES})
# ADD_EXECUTABLE(test_so3 sophus/test_so3.cpp ${SOURCES})
# ADD_EXECUTABLE(test_se3 sophus/test_se3.cpp ${SOURCES})
# ADD_EXECUTABLE(test_rxso3 sophus/test_rxso3.cpp ${SOURCES})
# ADD_EXECUTABLE(test_sim3 sophus/test_sim3.cpp ${SOURCES})
# ENABLE_TESTING()
# 
# ADD_TEST(test_so2 test_so2)
# ADD_TEST(test_se2 test_se2)
# ADD_TEST(test_so3 test_so3)
# ADD_TEST(test_se3 test_se3)
# ADD_TEST(test_rxso3 test_rxso3)
# ADD_TEST(test_sim3 test_sim3)

################################################################################
# Create the SophusConfig.cmake file for other cmake projects.
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/SophusConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/SophusConfig.cmake @ONLY IMMEDIATE )
export( PACKAGE Sophus )

INSTALL(DIRECTORY sophus DESTINATION ${CMAKE_INSTALL_PREFIX}/include
        FILES_MATCHING PATTERN "*.hpp" )