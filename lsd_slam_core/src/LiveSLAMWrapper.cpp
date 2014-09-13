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

#include "LiveSLAMWrapper.h"
#include <vector>
#include "util/SophusUtil.h"

#include "SlamSystem.h"

#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/InputImageStream.h"
#include "util/globalFuncs.h"

#include <iostream>

namespace lsd_slam
{


LiveSLAMWrapper::LiveSLAMWrapper(InputImageStream* imageStream, Output3DWrapper* outputWrapper)
{
	this->imageStream = imageStream;
	this->outputWrapper = outputWrapper;
	imageStream->getBuffer()->setReceiver(this);

	fx = imageStream->fx();
	fy = imageStream->fy();
	cx = imageStream->cx();
	cy = imageStream->cy();
	width = imageStream->width();
	height = imageStream->height();

	outFileName = packagePath+"estimated_poses.txt";


	isInitialized = false;


	Sophus::Matrix3f K_sophus;
	K_sophus << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	outFile = nullptr;


	// make Odometry
	monoOdometry = new SlamSystem(width, height, K_sophus, doSlam);

	monoOdometry->setVisualization(outputWrapper);

	imageSeqNumber = 0;
}


LiveSLAMWrapper::~LiveSLAMWrapper()
{
	if(monoOdometry != 0)
		delete monoOdometry;
	if(outFile != 0)
	{
		outFile->flush();
		outFile->close();
		delete outFile;
	}
}

void LiveSLAMWrapper::Loop()
{
	while (true) {
		boost::unique_lock<boost::recursive_mutex> waitLock(imageStream->getBuffer()->getMutex());
		while (!fullResetRequested && !(imageStream->getBuffer()->size() > 0)) {
			notifyCondition.wait(waitLock);
		}
		waitLock.unlock();
		
		
		if(fullResetRequested)
		{
			resetAll();
			fullResetRequested = false;
			if (!(imageStream->getBuffer()->size() > 0))
				continue;
		}
		
		TimestampedMat image = imageStream->getBuffer()->first();
		imageStream->getBuffer()->popFront();
		
		// process image
		//Util::displayImage("MyVideo", image.data);
		newImageCallback(image.data, image.timestamp);
	}
}


void LiveSLAMWrapper::newImageCallback(const cv::Mat& img, Timestamp imgTime)
{
	++ imageSeqNumber;

	// Convert image to grayscale, if necessary
	cv::Mat grayImg;
	if (img.channels() == 1)
		grayImg = img;
	else
		cvtColor(img, grayImg, CV_RGB2GRAY);
	

	// Assert that we work with 8 bit images
	assert(grayImg.elemSize() == 1);
	assert(fx != 0 || fy != 0);


	// need to initialize
	if(!isInitialized)
	{
		monoOdometry->randomInit(grayImg.data, imgTime.toSec(), 1);
		isInitialized = true;
	}
	else if(isInitialized && monoOdometry != nullptr)
	{
		monoOdometry->trackFrame(grayImg.data,imageSeqNumber,false,imgTime.toSec());
	}
}

void LiveSLAMWrapper::logCameraPose(const SE3& camToWorld, double time)
{
	Sophus::Quaternionf quat = camToWorld.unit_quaternion().cast<float>();
	Eigen::Vector3f trans = camToWorld.translation().cast<float>();

	char buffer[1000];
	int num = snprintf(buffer, 1000, "%f %f %f %f %f %f %f %f\n",
			time,
			trans[0],
			trans[1],
			trans[2],
			quat.x(),
			quat.y(),
			quat.z(),
			quat.w());

	if(outFile == 0)
		outFile = new std::ofstream(outFileName.c_str());
	outFile->write(buffer,num);
	outFile->flush();
}

void LiveSLAMWrapper::requestReset()
{
	fullResetRequested = true;
	notifyCondition.notify_all();
}

void LiveSLAMWrapper::resetAll()
{
	if(monoOdometry != nullptr)
	{
		delete monoOdometry;
		printf("Deleted SlamSystem Object!\n");

		Sophus::Matrix3f K;
		K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
		monoOdometry = new SlamSystem(width,height,K, doSlam);
		monoOdometry->setVisualization(outputWrapper);

	}
	imageSeqNumber = 0;
	isInitialized = false;

	Util::closeAllWindows();

}

}
