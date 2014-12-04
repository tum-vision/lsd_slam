cmake_minimum_required(VERSION 2.4.6)
project(lsd_slam_viewer)

# Set the build type. Options are:
#  Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug : w/ debug symbols, w/o optimization
#  Release : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/thirdparty/Sophus)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  sensor_msgs
  roscpp
  rosbag
  message_generation
  roslib
)

find_package(cmake_modules REQUIRED)
find_package(OpenGL REQUIRED)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
find_package(QGLViewer REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

#########################################################
# FIND X11
#########################################################
find_package(X11)
if(NOT X11_FOUND)
	message(FATAL_ERROR "Failed to find X11")
endif(NOT X11_FOUND)


if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	include_directories(/usr/X11R6/include/)
	link_directories(/usr/X11R6/lib)
	SET(EXTRA_LIBS GL X11 GLU glut)
	#target_link_libraries(main ${EXTRA_LIBS})
#else()
#	SET(EXTRA_LIBS GL glut GLU)
endif()


message("-- EIGEN3_INCLUDE_DIR : " ${EIGEN3_INCLUDE_DIR})
message("-- EIGEN_INCLUDE_DIR : " ${EIGEN_INCLUDE_DIR})
message("-- X11_LIBRARIES : " ${X11_LIBRARIES})

include_directories(${QGLVIEWER_INCLUDE_DIR}
		    ${catkin_INCLUDE_DIRS} 
		    ${EIGEN_INCLUDE_DIR}
		    ${QT_INCLUDES} )


# On Mac OSX it is not possible to build with at least clang shipped by Apple
# my guess due to lack of some C++11 features
# However gcc unable to link against libraries built by clang
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
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

add_message_files(DIRECTORY msg FILES keyframeMsg.msg keyframeGraphMsg.msg)
generate_messages(DEPENDENCIES)

generate_dynamic_reconfigure_options(
  cfg/LSDSLAMViewerParams.cfg
)

catkin_package(
#INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  sensor_msgs
  roscpp
  rosbag
  roslib
DEPENDS
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  sensor_msgs
  roscpp
  rosbag
  roslib
)

# Sources files
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

add_executable(viewer src/main_viewer.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(viewer ${QGLViewer_LIBRARIES}
			     ${QGLVIEWER_LIBRARY} 
			     ${catkin_LIBRARIES}
			     ${Boost_LIBRARIES}
			     ${QT_LIBRARIES}
			     ${X11_LIBRARIES}
			     ${EXTRA_LIBS}
			     ${OPENGL_LIBRARIES}
			     ${GLUT_LIBRARY}
)

#add_executable(videoStitch src/main_stitchVideos.cpp)
#target_link_libraries(viewer ${QGLViewer_LIBRARIES}
#			     ${QGLVIEWER_LIBRARY}
#			     ${catkin_LIBRARIES}
#			     ${QT_LIBRARIES}
#			     GL glut GLU
#		      )

