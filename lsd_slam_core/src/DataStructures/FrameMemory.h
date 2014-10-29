/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <unordered_map>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <deque>
#include <list>
#include <boost/thread/shared_mutex.hpp>
#include <Eigen/Core> //For EIGEN MACRO

namespace lsd_slam
{

/** Singleton class for re-using buffers in the Frame class. */
class Frame;
class FrameMemory
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** Returns the global instance. Creates it when the method is first called. */
	static FrameMemory& getInstance();

	/** Allocates or fetches a buffer with length: size * sizeof(float).
	  * Corresponds to "buffer = new float[size]". */
	float* getFloatBuffer(unsigned int size);

	/** Allocates or fetches a buffer with length: size * sizeof(float).
	  * Corresponds to "buffer = new float[size]". */
	void* getBuffer(unsigned int sizeInByte);
	
	/** Returns an allocated buffer back to the global storage for re-use.
	  * Corresponds to "delete[] buffer". */
	void returnBuffer(void* buffer);
	

	boost::shared_lock<boost::shared_mutex> activateFrame(Frame* frame);
	void deactivateFrame(Frame* frame);
	void pruneActiveFrames();

	void releaseBuffes();
private:
	FrameMemory();
	void* allocateBuffer(unsigned int sizeInByte);
	
	boost::mutex accessMutex;
	std::unordered_map< void*, unsigned int > bufferSizes;
	std::unordered_map< unsigned int, std::vector< void* > > availableBuffers;


	boost::mutex activeFramesMutex;
	std::list<Frame*> activeFrames;
};

}
