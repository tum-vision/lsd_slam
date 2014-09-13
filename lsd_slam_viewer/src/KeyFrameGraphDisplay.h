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
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef KEYFRAMEGRAPHDISPLAY_H_
#define KEYFRAMEGRAPHDISPLAY_H_


#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "boost/thread.hpp"

class KeyFrameDisplay;


struct GraphConstraint
{
	int from;
	int to;
	float err;
};


struct GraphConstraintPt
{
	KeyFrameDisplay* from;
	KeyFrameDisplay* to;
	float err;
};

struct GraphFramePose
{
	int id;
	float camToWorld[7];
};


class KeyFrameGraphDisplay {
public:
	KeyFrameGraphDisplay();
	virtual ~KeyFrameGraphDisplay();

	void draw();

	void addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);



	bool flushPointcloud;
	bool printNumbers;
private:
	std::map<int, KeyFrameDisplay*> keyframesByID;
	std::vector<KeyFrameDisplay*> keyframes;
	std::vector<GraphConstraintPt> constraints;

	boost::mutex dataMutex;

};

#endif /* KEYFRAMEGRAPHDISPLAY_H_ */
