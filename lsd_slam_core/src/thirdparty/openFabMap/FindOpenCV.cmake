# This Cmake file written by Michael Warren, CyPhy Lab, Queensland University of Technology, Australia
# https://wiki.qut.edu.au/display/cyphy/Michael+Warren
# Last updated 17/05/12

# FindOpenCV.cmake: Locate OpenCV >=2.2 headers and libs (for Windows and Linux)

# This module defines
# OPENCV2_FOUND whether the OpenCV 2.2 was found
# OPENCV2_PATH where the OpenCV 2.2 or greater files are (WIN32 only)
# OPENCV2_INCLUDE_PATH where the OpenCV 2.2 or greater header files are
# OPENCV2_LIB_PATH where the OpenCV 2.2 or greater library files are
# OPENCV2_RELEASE_LIBS the list of OpenCV 2.2 or greater release version libs (WIN32 MSVC compiler only)
# OPENCV2_DEBUG_LIBS the list of OpenCV 2.2 or greater debug version libs (WIN32 MSVC compiler only)
# OPENCV2_LIBS the list of OpenCV 2.2 or greater libs (WIN32 MINGW compiler only)

IF(WIN32)

	FIND_PATH( OPENCV2_PATH include/opencv2/opencv.hpp
		$ENV{OPENCV_HOME}
		C:/OpenCV2.2/
		C:/OpenCV2.3/
		C:/OpenCV2.4/
	)
	
	if( OPENCV2_PATH )
		MESSAGE( STATUS "Looking for OpenCV2.2 or greater - found")
		MESSAGE( STATUS "OpenCV2.2 path: ${OPENCV2_PATH}" )
		SET ( OPENCV2_FOUND 1 )
		
		# test for 64 or 32 bit
		if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
			SET( BUILD_DIR ${OPENCV2_PATH}/build/x64 )
		else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
			SET( BUILD_DIR ${OPENCV2_PATH}/build/x86 )
		endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )
		
		# MINGW
		if(MINGW)
			SET(OPENCV2_LIB_PATH ${BUILD_DIR}/mingw/lib/)
			file(GLOB OPENCV2_LIBS "${OPENCV2_LIB_PATH}*[0-9][0-9][0-9].dll.a")
		endif(MINGW)
		
		# Visual Studio 9
		if(MSVC90)
			SET(OPENCV2_LIB_PATH ${BUILD_DIR}/vc9/lib/)
			file(GLOB OPENCV2_RELEASE_LIBS "${OPENCV2_LIB_PATH}*[0-9][0-9][0-9].lib")
			file(GLOB OPENCV2_DEBUG_LIBS "${OPENCV2_LIB_PATH}*[0-9][0-9][0-9]d.lib")
		endif(MSVC90)
		
		# Visual Studio 10
		if(MSVC10)
			SET(OPENCV2_LIB_PATH ${BUILD_DIR}/vc10/lib/)
			file(GLOB OPENCV2_RELEASE_LIBS "${OPENCV2_LIB_PATH}*[0-9][0-9][0-9].lib")
			file(GLOB OPENCV2_DEBUG_LIBS "${OPENCV2_LIB_PATH}*[0-9][0-9][0-9]d.lib")
		endif(MSVC10)

		# Set the includes
		SET(OPENCV2_INCLUDE_PATH ${OPENCV2_PATH}/build/include/opencv2 ${OPENCV2_PATH}/build/include)
		

	else( OPENCV2_PATH )
		message( STATUS "Looking for OpenCV2.2 or greater  - not found" )
		SET ( OPENCV2_FOUND 0 )
	endif( OPENCV2_PATH )

ELSE(WIN32) # Linux
	FIND_PATH( OPENCV2_INCLUDE_PATH opencv.hpp
	# installation selected by user
	$ENV{OPENCV_HOME}/include
	# system placed in /usr/local/include
	/usr/local/include/opencv2
	# system placed in /usr/include
	/usr/include/opencv2
	)
	
	if( OPENCV2_INCLUDE_PATH )
		MESSAGE( STATUS "Looking for OpenCV2.2 or greater - found")
		MESSAGE( STATUS "OpenCV2.2 include path: ${OPENCV2_INCLUDE_PATH}" )
		SET ( OPENCV2_FOUND 1 )
	else( OPENCV2_INCLUDE_PATH )
		message( STATUS "Looking for OpenCV2.2 or greater  - not found" )
		SET ( OPENCV2_FOUND 0 )
	endif( OPENCV2_INCLUDE_PATH )

	
ENDIF(WIN32)
IF(OPENCV2_FOUND)
		INCLUDE_DIRECTORIES( ${OPENCV2_INCLUDE_PATH})
ENDIF(OPENCV2_FOUND)
