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

#ifdef HAVE_FABMAP
#pragma once
#include <opencv2/core/core.hpp>

namespace of2 {
	class FabMap;
}
namespace cv {
	class FeatureDetector;
	class BOWImgDescriptorExtractor;
}


namespace lsd_slam
{


class Frame;

/** Interface to openFabMap. */
class FabMap
{
public:
	/** Initializes FabMap. */
	FabMap();
	
	/** Writes out the confusion matrix if enabled. */
	~FabMap();
	
	/** Adds the keyframe to the set of frames to compare against and returns
	 *  its (non-negative) ID in FabMap (different to the keyframe ID).
	 *  Returns -1 if the frame cannot be added due to an error. */
// 	int add(KeyFrame* keyframe);
	
	/** Checks if the keyframe is determined to be the same as an already
	 *  added frame and if yes, returns its ID. If not, returns -1.
	 *  Does not directly return a KeyFrame pointer to allow for KeyFrames
	 *  being deleted. */
// 	int compare(KeyFrame* keyframe);

	/** Combination of compare() followed by add() (more efficient). */
	void compareAndAdd(Frame* keyframe, int* out_newID, int* out_loopID);
	
	/** Returns if the class is initialized correctly (i.e. if the required
	 *  files could be loaded). */
	bool isValid() const;
	
private:
	int nextImageID;
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::BOWImgDescriptorExtractor> bide;
	cv::Ptr<of2::FabMap> fabMap;
	
	bool printConfusionMatrix;
	cv::Mat confusionMat;
	
	bool valid;
};

}
#endif
