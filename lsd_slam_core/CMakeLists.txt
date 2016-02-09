cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_core)
if(NOT BUILD_WITHOUT_ROS)
    include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

    # Set the build type.  Options are:
    #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
    #  Debug          : w/ debug symbols, w/o optimization
    #  Release        : w/o debug symbols, w/ optimization
    #  RelWithDebInfo : w/ debug symbols, w/ optimization
    #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
    set(ROS_BUILD_TYPE Release)

    rosbuild_init()
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(SuiteParse REQUIRED) # Apparently needed by g2o
find_package(X11 REQUIRED)

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )

if(NOT BUILD_WITHOUT_ROS)
    # Dynamic Reconfigure Services
    rosbuild_find_ros_package(dynamic_reconfigure)
    include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
    gencfg()

    # SSE flags
    rosbuild_check_for_sse()
    add_definitions("-DUSE_ROS")
else()
    find_package(Boost REQUIRED COMPONENTS thread system)
    find_package(OpenCV REQUIRED)
endif()
add_definitions("-DENABLE_SSE")

# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -march=native  -std=c++0x"
)

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
  ${PROJECT_SOURCE_DIR}/src/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/FabMap.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/KeyFrameGraph.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/TrackableKeyFrameSearch.cpp
)

if(NOT BUILD_WITHOUT_ROS)
    set(ADDITIONAL_ROS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSImageStreamThread.cpp
        ${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSOutput3DWrapper.cpp
        )
endif()
set(SOURCE_FILES
  ${lsd_SOURCE_FILES}
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
  ${ADDITIONAL_ROS_SOURCES}
)
# The variables EXTRA_INCLUDE_DIRS and EXTRA_LIB_DIRS can be used to
# inject non-standard library paths for local builds of libraries that
# aren't in system directories - for example, g2o
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
  ${EXTRA_INCLUDE_DIRS}
)
link_directories(${EXTRA_LIB_DIRS})



# build shared library.
if (BUILD_WITHOUT_ROS)
    set(PLATFORM_SPECIFIC_LIBRARIES ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        )
    add_library(lsdslam SHARED ${SOURCE_FILES})
else()
    set(PLATFORM_SPECIFIC_LIBRARIES X11)
    rosbuild_add_library(lsdslam SHARED ${SOURCE_FILES})
    rosbuild_link_boost(lsdslam thread)
endif()

target_link_libraries(lsdslam ${FABMAP_LIB} g2o_core g2o_stuff cxsparse g2o_solver_csparse g2o_csparse_extension g2o_types_sim3 g2o_types_sba
    ${PLATFORM_SPECIFIC_LIBRARIES}
    )


if(NOT BUILD_WITHOUT_ROS)
    # build live ros node
    rosbuild_add_executable(live_slam src/main_live_odometry.cpp)
    target_link_libraries(live_slam lsdslam)


    # build image node
    rosbuild_add_executable(dataset_slam src/main_on_images.cpp)
    target_link_libraries(dataset_slam lsdslam)
endif()
