cmake_minimum_required(VERSION 2.4.6)
project(lsd_slam_viewer)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/thirdparty/Sophus)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenGL REQUIRED)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
find_package(QGLViewer REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${QT_INCLUDES} ${EIGEN3_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR})

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()





# Messages & Services
rosbuild_genmsg()


# SSE flags
rosbuild_check_for_sse()
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS}"
)

# SSE Sources files
set(SOURCE_FILES         
  src/PointCloudViewer.cpp
  src/KeyFrameDisplay.cpp
  src/KeyFrameGraphDisplay.cpp
  src/settings.cpp
)

set(HEADER_FILES     
  src/PointCloudViewer.h
  src/KeyFrameDisplay.h
  src/KeyFrameGraphDisplay.h
  src/settings.h
)

include_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
)  

rosbuild_add_executable(viewer src/main_viewer.cpp ${SOURCE_FILES} ${HEADER_FILES})

target_link_libraries(viewer ${QGLViewer_LIBRARIES} ${QT_LIBRARIES} ${QGLVIEWER_LIBRARY} GL glut GLU)

rosbuild_link_boost(viewer thread)


rosbuild_add_executable(videoStitch src/main_stitchVideos.cpp)

