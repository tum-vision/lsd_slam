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
#include "util/settings.h"
#include "util/EigenCoreInclude.h"
#include "boost/thread/mutex.hpp"
#include <boost/thread/shared_mutex.hpp>


namespace lsd_slam
{

class Frame;
class DepthMapPixelHypothesis;
class KeyFrameGraph;

/**
 * Point cloud used to track frame poses.
 * 
 * Basically this stores a point cloud generated from known frames. It is used to
 * track a new frame by finding a projection of the point cloud which makes it
 * look as much like the new frame as possible.
 * 
 * It is intended to use more than one old frame as source for the point cloud.
 * Also other data like Kinect depth data could be imported.
 * 
 * ATTENTION: as the level zero point cloud is not used for tracking, it is not
 * fully calculated. Only the weights are valid on this level!
 */
class TrackingReference
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** Creates an empty TrackingReference with optional preallocation per level. */
	TrackingReference();
	~TrackingReference();
	void importFrame(Frame* source);

	Frame* keyframe;
	boost::shared_lock<boost::shared_mutex> keyframeLock;
	int frameID;

	void makePointCloud(int level);
	void clearAll();
	void invalidate();
	Eigen::Vector3f* posData[PYRAMID_LEVELS];	// (x,y,z)
	Eigen::Vector2f* gradData[PYRAMID_LEVELS];	// (dx, dy)
	Eigen::Vector2f* colorAndVarData[PYRAMID_LEVELS];	// (I, Var)
	int* pointPosInXYGrid[PYRAMID_LEVELS];	// x + y*width
	int numData[PYRAMID_LEVELS];

private:
	int wh_allocated;
	boost::mutex accessMutex;
	void releaseAll();
};
}
