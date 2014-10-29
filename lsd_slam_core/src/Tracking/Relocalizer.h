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
#include "boost/thread.hpp"
#include <stdio.h>
#include <iostream>
#include "util/SophusUtil.h"


namespace lsd_slam
{

class Frame;
class Sim3Tracker;

class Relocalizer
{
public:
	Relocalizer(int w, int h, Eigen::Matrix3f K);
	~Relocalizer();

	void updateCurrentFrame(std::shared_ptr<Frame> currentFrame);
    void start(std::vector<Frame*, Eigen::aligned_allocator<lsd_slam::Frame*> > &allKeyframesList);
	void stop();

	bool waitResult(int milliseconds);
	void getResult(Frame* &out_keyframe, std::shared_ptr<Frame> &frame, int &out_successfulFrameID, SE3 &out_frameToKeyframe);

	bool isRunning;
private:
	int w, h;
	Eigen::Matrix3f K;
	boost::thread relocThreads[RELOCALIZE_THREADS];
	bool running[RELOCALIZE_THREADS];

	// locking & signalling structures
	boost::mutex exMutex;
	boost::condition_variable newCurrentFrameSignal;
	boost::condition_variable resultReadySignal;

	// for rapid-checking
	std::vector<Frame*> KFForReloc;
	std::shared_ptr<Frame> CurrentRelocFrame;
	int nextRelocIDX;
	int maxRelocIDX;
	bool continueRunning;

	// result!
	std::shared_ptr<Frame> resultRelocFrame;
	bool hasResult;
	Frame* resultKF;
	int resultFrameID;
	SE3 resultFrameToKeyframe;


	void threadLoop(int idx);
};

}
