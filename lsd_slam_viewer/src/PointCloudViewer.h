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

#pragma once
//#define GL_GLEXT_PROTOTYPES 1
//#define GL3_PROTOTYPES 1
//#include <GL/glew.h>

#include "QGLViewer/qglviewer.h"
#include <vector>
#include "boost/thread.hpp"
#include "qevent.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"

#include "QGLViewer/keyFrameInterpolator.h"

class QApplication;

class KeyFrameGraphDisplay;
class CameraDisplay;
class KeyFrameDisplay;

#include "settings.h"

class AnimationObject
{
public:
	double time;
	double duration;

	// settings
	float scaledTH;
	float absTH;
	int neighb;
	int sparsity;
	bool showLoopClosures;
	bool showKeyframes;
	bool showCurrentCam;

	// frame
	qglviewer::Frame frame;

	// whether is a KF or only settings change
	bool isSettings;

	bool isFix;

	AnimationObject(bool isSettings, double time, double duration, qglviewer::Frame f = qglviewer::Frame())
	{
		this->time = time;
		this->duration = duration;

		scaledTH = scaledDepthVarTH;
		absTH = absDepthVarTH;
		neighb = minNearSupport;
		showKeyframes = showKFCameras;
		showLoopClosures = showConstraints;
		showCurrentCam = showCurrentCamera;
		sparsity = sparsifyFactor;

		this->isSettings = isSettings;

		frame = f;

		isFix = false;
	}

	AnimationObject(std::string s)
	{
		int isSettings_i;
		int showLoopClosures_i;
		int showKeyframes_i;
		int showCurrentCam_i;
		int isFix_i;


		qglviewer::Quaternion orient;


		float x,y,z;

		if(17 != sscanf(s.c_str(),"Animation: %d at %lf (dur %lf) S: %f %f %d %d %d %d %d Frame: %lf %lf %lf %lf %f %f %f %d\n",
				&isSettings_i, &time, &duration,
				&scaledTH, &absTH, &showLoopClosures_i, &showKeyframes_i, &showCurrentCam_i, &sparsity, &neighb,
				&(orient[0]),&(orient[1]),&(orient[2]),&(orient[3]),
				&x, &y, &z, &isFix_i))
			printf("error parsing: %s\n", s.c_str());

		isSettings = isSettings_i;
		showLoopClosures = showLoopClosures_i;
		showKeyframes = showKeyframes_i;
		showCurrentCam = showCurrentCam_i;
		isFix = isFix_i;


		frame = qglviewer::Frame(qglviewer::Vec(0,0,0),orient);
		frame.setPosition(x,y,z);

		printf("read: %s\n",toString().c_str());
	}

    bool operator < (const AnimationObject& other) const
    {
        return (time < other.time);
    }

    std::string toString()
    {
    	char buf[1000];

		int isSettings_i = isSettings;
		int showLoopClosures_i = showLoopClosures;
		int showKeyframes_i = showKeyframes;
		int showCurrentCam_i = showCurrentCam;
		int isFix_i = isFix;

		float x,y,z;
		frame.getPosition(x,y,z);

    	snprintf(buf, 1000, "Animation: %d at %lf (dur %lf) S: %f %f %d %d %d %d %d Frame: %lf %lf %lf %lf %f %f %f %d",
				isSettings_i, time, duration,
				scaledTH, absTH, showLoopClosures_i, showKeyframes_i, showCurrentCam_i, sparsity, neighb,
				frame.orientation()[0],frame.orientation()[1],frame.orientation()[2],frame.orientation()[3],
				x,y,z, isFix_i);

    	return buf;
    }
};




class PointCloudViewer : public QGLViewer
{
public:
	PointCloudViewer();
	~PointCloudViewer();


	void reset();

	void addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);


protected :
	virtual void draw();
	virtual void init();
	virtual void keyPressEvent(QKeyEvent *e);
	virtual void keyReleaseEvent(QKeyEvent *e);
	virtual QString helpString() const;

//	virtual void drawText(int x, int y, const QString & text, const QFont & fnt) {printf(text.toStdString().c_str());};


private:

	// displays kf-graph
	KeyFrameGraphDisplay* graphDisplay;

	// displays only current keyframe (which is not yet in the graph).
	KeyFrameDisplay* currentCamDisplay;



	// meddle mutex
	boost::mutex meddleMutex;


	void setToVideoSize();
	bool resetRequested;

	// for saving stuff
	std::string save_folder;
	double localMsBetweenSaves;
	double simMsBetweenSaves;
	double lastSaveTime;
	double lastCamTime;
	int lastCamID;


	double lastLocalSaveTime;
	double lastRealSaveTime;


	// for keyframe interpolation
	int KFLastPCSeq;
	int KFcurrent;
	double KFautoPlayIdx[10];
	bool KFexists[10];
	double lastAutoplayCheckedSaveTime;

	// for display settings autoplay
	std::vector<AnimationObject> animationList;
	qglviewer::KeyFrameInterpolator* kfInt;
	bool customAnimationEnabled;

	bool animationPlaybackEnabled;
	double animationPlaybackTime;
	int animationPlaybackID;



	double lastAnimTime;


	void remakeAnimation();
};


