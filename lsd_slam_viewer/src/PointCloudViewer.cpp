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

#define GL_GLEXT_PROTOTYPES 1
#include "PointCloudViewer.h"
#include "qfiledialog.h"
#include "qcoreapplication.h"
#include <stdio.h>
#include "settings.h"
#include "ros/package.h"

#include <zlib.h>
#include <iostream>


#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "QGLViewer/manipulatedCameraFrame.h"

#include "KeyFrameDisplay.h"
#include "KeyFrameGraphDisplay.h"

#include <iostream>
#include <fstream>

PointCloudViewer::PointCloudViewer()
{
	setPathKey(Qt::Key_0,0);
	setPathKey(Qt::Key_1,1);
	setPathKey(Qt::Key_2,2);
	setPathKey(Qt::Key_3,3);
	setPathKey(Qt::Key_4,4);
	setPathKey(Qt::Key_5,5);
	setPathKey(Qt::Key_6,6);
	setPathKey(Qt::Key_7,7);
	setPathKey(Qt::Key_8,8);
	setPathKey(Qt::Key_9,9);


	currentCamDisplay = 0;
	graphDisplay = 0;


	for(int i=0;i<10;i++)
	{
		KFexists[i] = 0;
		KFautoPlayIdx[i] = -1;
	}

	kfInt = new qglviewer::KeyFrameInterpolator(new qglviewer::Frame());
	customAnimationEnabled = false;

	setSnapshotFormat(QString("PNG"));

	reset();
}


PointCloudViewer::~PointCloudViewer()
{
	delete currentCamDisplay;
	delete graphDisplay;
}


void PointCloudViewer::reset()
{
	if(currentCamDisplay != 0)
		delete currentCamDisplay;
	if(graphDisplay != 0)
		delete graphDisplay;

	currentCamDisplay = new KeyFrameDisplay();
	graphDisplay = new KeyFrameGraphDisplay();

	KFcurrent = 0;
	KFLastPCSeq = -1;

	resetRequested=false;

	save_folder = ros::package::getPath("lsd_slam_viewer")+"/save/";
	localMsBetweenSaves = 1;
	simMsBetweenSaves = 1;
	lastCamID = -1;
	lastAnimTime = lastCamTime = lastSaveTime = 0;
	char buf[500];
	snprintf(buf,500,"rm -rf %s",save_folder.c_str());
	int k = system(buf);
	snprintf(buf,500,"mkdir %s",save_folder.c_str());
	k += system(buf);


	assert(k != -42);

	setSceneRadius(80);
	setTextIsEnabled(false);
	lastAutoplayCheckedSaveTime = -1;

	animationPlaybackEnabled = false;
}

void PointCloudViewer::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	meddleMutex.lock();

	if(!msg->isKeyframe)
	{
		if(currentCamDisplay->id > msg->id)
		{
			printf("detected backward-jump in id (%d to %d), resetting!\n", currentCamDisplay->id, msg->id);
			resetRequested = true;
		}
		currentCamDisplay->setFrom(msg);
		lastAnimTime = lastCamTime = msg->time;
		lastCamID = msg->id;
	}
	else
		graphDisplay->addMsg(msg);

	meddleMutex.unlock();
}

void PointCloudViewer::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	meddleMutex.lock();

	graphDisplay->addGraphMsg(msg);

	meddleMutex.unlock();
}


void PointCloudViewer::init()
{
	setAnimationPeriod(30);
	startAnimation();
}

QString PointCloudViewer::helpString() const
{
	return QString("");
}

void PointCloudViewer::draw()
{
	meddleMutex.lock();


	if(resetRequested)
	{
		reset();
		resetRequested = false;
	}


	glPushMatrix();


	if(animationPlaybackEnabled)
	{
		double tm = ros::Time::now().toSec() - animationPlaybackTime;

		if(tm > kfInt->lastTime())
		{
			animationPlaybackEnabled = false;
			tm = kfInt->lastTime();
		}

		if(tm < kfInt->firstTime())
			tm = kfInt->firstTime();

		printf("anim at %.2f (%.2f to %.2f)\n", tm, kfInt->firstTime(), kfInt->lastTime());


		kfInt->interpolateAtTime(tm);
		camera()->frame()->setFromMatrix(kfInt->frame()-> matrix());



		double accTime = 0;
		for(unsigned int i=0;i<animationList.size();i++)
		{
			if(tm >= accTime && tm < accTime+animationList[i].duration && animationList[i].isFix)
			{
				camera()->frame()->setFromMatrix(animationList[i].frame.matrix());

				printf("fixFrameto %d at %.2f (%.2f to %.2f)\n", i, tm, kfInt->firstTime(), kfInt->lastTime());
			}

			accTime += animationList[i].duration;
		}


		accTime = 0;
		AnimationObject* lastAnimObj = 0;
		for(unsigned int i=0;i<animationList.size();i++)
		{
			accTime += animationList[i].duration;
			if(animationList[i].isSettings && accTime <= tm)
				lastAnimObj = &(animationList[i]);
		}
		if(lastAnimObj != 0)
		{
			absDepthVarTH = lastAnimObj->absTH;
			scaledDepthVarTH = lastAnimObj->scaledTH;
			minNearSupport = lastAnimObj->neighb;
			sparsifyFactor = lastAnimObj->sparsity;
			showKFCameras = lastAnimObj->showKeyframes;
			showConstraints = lastAnimObj->showLoopClosures;
		}
	}



	if(showCurrentCamera)
		currentCamDisplay->drawCam(2*lineTesselation, 0);

	if(showCurrentPointcloud)
		currentCamDisplay->drawPC(pointTesselation, 1);


	graphDisplay->draw();


	glPopMatrix();

	meddleMutex.unlock();




	if(saveAllVideo)
	{
		double span = ros::Time::now().toSec() - lastRealSaveTime;
		if(span > 0.4)
		{
			setSnapshotQuality(100);

			printf("saved (img %d @ time %lf, saveHZ %f)!\n", lastCamID, lastAnimTime, 1.0/localMsBetweenSaves);

			char buf[500];
			snprintf(buf,500,"%s%lf.png",save_folder.c_str(),  ros::Time::now().toSec());
			saveSnapshot(QString(buf));
			lastRealSaveTime = ros::Time::now().toSec();
		}


	}
}

void PointCloudViewer::keyReleaseEvent(QKeyEvent *e)
  {

  }


void PointCloudViewer::setToVideoSize()
{
	this->setFixedSize(1600,900);
}


void PointCloudViewer::remakeAnimation()
{
	delete kfInt;
	kfInt = new qglviewer::KeyFrameInterpolator(new qglviewer::Frame());
	std::sort(animationList.begin(), animationList.end());

	float tm=0;
	for(unsigned int i=0;i<animationList.size();i++)
	{
	  if(!animationList[i].isSettings)
	  {
		  kfInt->addKeyFrame(&animationList[i].frame, tm);
		  tm += animationList[i].duration;
	  }
	}

	printf("made animation with %d keyframes, spanning %f s!\n", kfInt->numberOfKeyFrames(), tm);
}

void PointCloudViewer::keyPressEvent(QKeyEvent *e)
  {
    switch (e->key())
    {
      case Qt::Key_S :
    	    setToVideoSize();
    	  break;

      case Qt::Key_R :
    	    resetRequested = true;

    	  break;

      case Qt::Key_T:	// add settings item
    	  meddleMutex.lock();
    	  animationList.push_back(AnimationObject(true, lastAnimTime, 0));
    	  meddleMutex.unlock();
    	  printf("added St: %s\n", animationList.back().toString().c_str());

    	  break;

      case Qt::Key_K:	// add keyframe item
    	  meddleMutex.lock();


    	  float x,y,z;
    	  camera()->frame()->getPosition(x,y,z);
    	  animationList.push_back(AnimationObject(false, lastAnimTime, 2, qglviewer::Frame(qglviewer::Vec(0,0,0), camera()->frame()->orientation())));
    	  animationList.back().frame.setPosition(x,y,z);
    	  meddleMutex.unlock();
    	  printf("added KF: %s\n", animationList.back().toString().c_str());



    	  remakeAnimation();

    	  break;

      case Qt::Key_I :	// reset animation list
			meddleMutex.lock();
			animationList.clear();
			meddleMutex.unlock();
			printf("resetted animation list!\n");

			remakeAnimation();

    	  break;


      case Qt::Key_F :	// save list
      {
			meddleMutex.lock();
			std::ofstream myfile;
			myfile.open ("animationPath.txt");
			for(unsigned int i=0;i<animationList.size();i++)
			{
				myfile << animationList[i].toString() << "\n";
			}
			myfile.close();
			meddleMutex.unlock();

			printf("saved animation list (%d items)!\n", (int)animationList.size());
      }
    	  break;


      case Qt::Key_L :	// load list
      {
			meddleMutex.lock();
			animationList.clear();

			std::ifstream myfile;
			std::string line;
			myfile.open ("animationPath.txt");

			if (myfile.is_open())
			{
				while ( getline (myfile,line) )
				{
					if(!(line[0] == '#'))
						animationList.push_back(AnimationObject(line));
				}
				myfile.close();
			}
			else
				std::cout << "Unable to open file";
			myfile.close();
			meddleMutex.unlock();

			printf("loaded animation list! (%d items)!\n", (int)animationList.size());
			remakeAnimation();
      }
    	  break;


      case Qt::Key_A:
    	  if(customAnimationEnabled)
    		  printf("DISABLE custom animation!\n)");
    	  else
    		  printf("ENABLE custom animation!\n");
    	  customAnimationEnabled = !customAnimationEnabled;
    	  break;

      case Qt::Key_O:
    	  if(animationPlaybackEnabled)
    	  {
    		  animationPlaybackEnabled=false;
    	  }
    	  else
    	  {
    		  animationPlaybackEnabled = true;
    		  animationPlaybackTime = ros::Time::now().toSec();
    	  }
      	  break;


      case Qt::Key_P:
    	  graphDisplay->flushPointcloud = true;
    	  break;

      case Qt::Key_W:
    	  graphDisplay->printNumbers = true;
    	  break;

      default:
    	  QGLViewer::keyPressEvent(e);
    	  break;
    }
  }

