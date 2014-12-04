cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_core)

# Set the build type. Options are:
#  Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug : w/ debug symbols, w/o optimization
#  Release : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  #lsd_slam_viewer
  cv_bridge
  dynamic_reconfigure
  sensor_msgs
  image_transport
  roscpp
  rosbag
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(X11 REQUIRED)
include(cmake/FindG2O.cmake)
include(cmake/FindSuiteParse.cmake)

#find csparse
#FIND_PACKAGE(SuiteSparse)
IF(CSPARSE_FOUND)
MESSAGE(STATUS "CSPARSE FOUND")
ELSE(CSPARSE_FOUND)
MESSAGE(STATUS "CSPARSE NOT FOUND")
ENDIF(CSPARSE_FOUND)

#########################################################
# FIND X11
#########################################################
find_package(X11)
if(NOT X11_FOUND)
	message(FATAL_ERROR "Failed to find X11")
endif(NOT X11_FOUND)
      
message("-- CHOLMOD_INCLUDE_DIR : " ${CHOLMOD_INCLUDE_DIR})
message("-- CSPARSE_INCLUDE_DIR : " ${CSPARSE_INCLUDE_DIR})
message("-- G2O_INCLUDE_DIR : " ${G2O_INCLUDE_DIR})
message("-- EIGEN3_INCLUDE_DIR : " ${EIGEN3_INCLUDE_DIR})
message("-- EIGEN_INCLUDE_DIR : " ${EIGEN_INCLUDE_DIR})
message("-- X11_LIBRARIES : " ${X11_LIBRARIES})

# FabMap
# uncomment this part to enable fabmap
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
add_definitions("-DHAVE_FABMAP")
set(FABMAP_LIB openFABMAP )

generate_dynamic_reconfigure_options(
  cfg/LSDDebugParams.cfg
  cfg/LSDParams.cfg
)

catkin_package(
  LIBRARIES lsdslam
  DEPENDS Eigen SuiteSparse
  CATKIN_DEPENDS libg2o #lsd_slam_viewer
)

# SSE flags
add_definitions("-DUSE_ROS")
add_definitions("-DENABLE_SSE")

# Also add some useful compiler flag
#set(CMAKE_CXX_FLAGS
#   "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x"
#)

# Notes from stackoverflow:
# The short solution would be removing '-march=native' option (at least it worked for me).
# Instead of falling back to the default SSE2, if you care about performance, 
# you are better off with enabling optimization the instruction set closest to AVX 
# which would be SSE4.2. So I'd rather replace -march=native with -msse4.2. 
# Even better: -march=native -mno-avx. Or see the answer by @capisce
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -msse4.2 -Wall -D_REENTRANT --std=c++0x")

# On Mac OSX it is not possible to build with at least clang shipped by Apple
# my guess due to lack of some c++0x/C++11 features
# However gcc unable to link against libraries built by clang
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	# SET (CMAKE_C_COMPILER             "/usr/bin/clang")
	# SET (CMAKE_CXX_COMPILER             "/usr/bin/clang++")
	#     SET (CMAKE_AR      "/usr/bin/llvm-ar")
	#     SET (CMAKE_LINKER  "/usr/bin/llvm-ld")
	#     SET (CMAKE_NM      "/usr/bin/llvm-nm")
	#     SET (CMAKE_OBJDUMP "/usr/bin/llvm-objdump")
	#     SET (CMAKE_RANLIB  "/usr/bin/llvm-ranlib")
	# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")
	SET (CMAKE_C_COMPILER             "/usr/local/bin/gcc")
	SET (CMAKE_CXX_COMPILER             "/usr/local/bin/g++")
	SET (CMAKE_AR      "/usr/local/bin/ar")
	SET (CMAKE_LINKER  "/usr/local/bin/ld")
	SET (CMAKE_NM      "/usr/local/bin/nm")
	SET (CMAKE_OBJDUMP "/usr/local/bin/objdump")
	SET (CMAKE_RANLIB  "/usr/local/bin/ranlib")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.2 -Wall -D_REENTRANT --std=c++0x")
else()
	# SSE flags
	set(CMAKE_CXX_FLAGS
	"${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x"
	)
endif()



# Set source files
set(lsd_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FramePoseStruct.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FrameMemory.cpp
  ${PROJECT_SOURCE_DIR}/src/SlamSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/LiveSLAMWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMap.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalFuncs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/SophusUtil.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistorter.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Sim3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Relocalizer.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/SE3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/least_squares.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/FabMap.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/KeyFrameGraph.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/TrackableKeyFrameSearch.cpp
)
set(SOURCE_FILES
  ${lsd_SOURCE_FILES}
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSImageStreamThread.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSOutput3DWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
)

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
)


# build shared library.
add_library(lsdslam SHARED ${SOURCE_FILES})
target_link_libraries(lsdslam ${FABMAP_LIB} ${G2O_LIBRARIES} ${catkin_LIBRARIES} ${CSPARSE_LIBRARY}) 
#csparse cxsparse )
#rosbuild_link_boost(lsdslam thread)


# build live ros node
add_executable(live_slam src/main_live_odometry.cpp)
target_link_libraries(live_slam lsdslam ${catkin_LIBRARIES} ${G2O_LIBRARIES} ${X11_LIBRARIES} )


# build image node
add_executable(dataset src/main_on_images.cpp)
target_link_libraries(dataset lsdslam ${catkin_LIBRARIES} ${G2O_LIBRARIES})

# TODO add INSTALL


