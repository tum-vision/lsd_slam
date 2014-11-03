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
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/StdVector>
#include "util/SophusUtil.h"

#ifdef HAVE_FABMAP
	#include "GlobalMapping/FabMap.h"
#endif

#include "util/settings.h"



namespace lsd_slam
{


class KeyFrameGraph;
class SE3Tracker;
class Frame;


struct TrackableKFStruct
{
	Frame* ref;
	SE3 refToFrame;
	float dist;
	float angle;
};

/**
 * Given a KeyFrame, tries to find other KeyFrames from a KeyFrameGraph which
 * can be tracked from this frame (in order to insert new constraints into
 * the graph).
 */
class TrackableKeyFrameSearch
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** Constructor. */
	TrackableKeyFrameSearch(KeyFrameGraph* graph, int w, int h, Eigen::Matrix3f K);
	~TrackableKeyFrameSearch();
	
	/**
	 * Finds candidates for trackable frames.
	 * Returns the most likely candidates first.
	 */
	std::unordered_set<Frame*, std::hash<Frame*>, std::equal_to<Frame*>, Eigen::aligned_allocator< Frame* > > findCandidates(Frame* keyframe, Frame* &fabMapResult_out, bool includeFABMAP=true, bool closenessTH=1.0);
	Frame* findRePositionCandidate(Frame* frame, float maxScore=1);
	

	inline float getRefFrameScore(float distanceSquared, float usage)
	{
		return distanceSquared*KFDistWeight*KFDistWeight
				+ (1-usage)*(1-usage) * KFUsageWeight * KFUsageWeight;
	}

	float msTrackPermaRef;
	int nTrackPermaRef;
	float nAvgTrackPermaRef;
private:
	/**
	 * Returns a possible loop closure for the keyframe or nullptr if none is found.
	 * Uses FabMap internally.
	 */
	Frame* findAppearanceBasedCandidate(Frame* keyframe);
	std::vector<TrackableKFStruct, Eigen::aligned_allocator<TrackableKFStruct> > findEuclideanOverlapFrames(Frame* frame, float distanceTH, float angleTH, bool checkBothScales = false);

#ifdef HAVE_FABMAP
	std::unordered_map<int, Frame*> fabmapIDToKeyframe;
	FabMap fabMap;
#endif
	KeyFrameGraph* graph;
	SE3Tracker* tracker;

	float fowX, fowY;

};

}
