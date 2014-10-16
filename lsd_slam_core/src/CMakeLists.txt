cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_core)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS thread filesystem system REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(SuiteParse REQUIRED) # Apparently needed by g2o

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )

# SSE flags
add_definitions("-DENABLE_SSE")

# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -msse4.1 -msse3 -msse2 -msse -march=native -O3 -std=c++0x"
) 

# Set source files
set(lsd_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/DataStructures/FramePoseStruct.cpp
  ${PROJECT_SOURCE_DIR}/DataStructures/FrameMemory.cpp
  ${PROJECT_SOURCE_DIR}/SlamSystem.cpp
  ${PROJECT_SOURCE_DIR}/LiveSLAMWrapper.cpp
  ${PROJECT_SOURCE_DIR}/DepthEstimation/DepthMap.cpp
  ${PROJECT_SOURCE_DIR}/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${PROJECT_SOURCE_DIR}/util/globalFuncs.cpp
  ${PROJECT_SOURCE_DIR}/util/SophusUtil.cpp
  ${PROJECT_SOURCE_DIR}/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/util/Undistorter.cpp
  ${PROJECT_SOURCE_DIR}/Tracking/Sim3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/Tracking/Relocalizer.cpp
  ${PROJECT_SOURCE_DIR}/Tracking/SE3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/Tracking/least_squares.cpp
  ${PROJECT_SOURCE_DIR}/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/IOWrapper/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/GlobalMapping/FabMap.cpp
  ${PROJECT_SOURCE_DIR}/GlobalMapping/KeyFrameGraph.cpp
  ${PROJECT_SOURCE_DIR}/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${PROJECT_SOURCE_DIR}/GlobalMapping/TrackableKeyFrameSearch.cpp
)
set(SOURCE_FILES
  ${lsd_SOURCE_FILES}
  ${PROJECT_SOURCE_DIR}/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
)

# build shared library.
add_library(lsdslam SHARED ${SOURCE_FILES})
target_link_libraries(lsdslam ${FABMAP_LIB} ${Boost_LIBRARIES} ${OpenCV_LIBS} g2o_core g2o_stuff csparse cxsparse g2o_solver_csparse g2o_csparse_extension g2o_types_sim3 g2o_types_sba)

# build image node
add_executable(LSD LSD.cpp)
target_link_libraries(LSD lsdslam ${Pangolin_LIBRARIES})

