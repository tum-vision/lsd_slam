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

#include "OpenCVImageStreamThread.h"

#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "util/settings.h"

#include <iostream>
#include <fstream>


namespace lsd_slam
{


using namespace cv;

OpenCVImageStreamThread::OpenCVImageStreamThread()
{

	// wait for cam calib
	width_ = height_ = fx_ = fy_ = cx_ = cy_ = 0;


	// imagebuffer
	imageBuffer = new NotifyBuffer<TimestampedMat>(8);
	undistorter = 0;

	haveCalib = false;
}

OpenCVImageStreamThread::~OpenCVImageStreamThread()
{
	delete imageBuffer;
}

void OpenCVImageStreamThread::setCalibration(std::string file)
{

	undistorter = Undistorter::getUndistorterForFile(file.c_str());

	if(undistorter==0)
	{
		printf("Failed to read camera calibration from file... wrong syntax?\n");
		exit(0);
	}

	fx_ = undistorter->getK().at<double>(0, 0);
	fy_ = undistorter->getK().at<double>(1, 1);
	cx_ = undistorter->getK().at<double>(2, 0);
	cy_ = undistorter->getK().at<double>(2, 1);

	width_ = undistorter->getOutputWidth();
	height_ = undistorter->getOutputHeight();

	haveCalib = true;
}

void OpenCVImageStreamThread::run()
{
	boost::thread thread(boost::ref(*this));
}

void OpenCVImageStreamThread::operator()()
{
	// Main thread here. Grab and decode images:
	VideoCapture cap;
	// open the default camera, use something different from 0 otherwise;
	// Check VideoCapture documentation.
	if(!cap.open(0)) {
		std::cout << "Camera could not be opened" << std::endl;
		exit(0);
	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH,320); // TODO change to match calib file
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	cap.set(CV_CAP_PROP_FPS,60);
	{
		Mat frame;
		cap >> frame;
		if( !frame.empty() )
		{
			printf( "Camera opend with parameters: W:%d, H:%d\n", frame.size().width, frame.size().height );
			printf( "requested: W:%f, H:%f\n", 
				cap.get(CV_CAP_PROP_FRAME_WIDTH),
				cap.get(CV_CAP_PROP_FRAME_HEIGHT) );
		}
	}
	for(;;)
	{
		TimestampedMat bufferItem;
		Mat frame;

		bufferItem.timestamp =  Timestamp(0);
		cap >> frame;

		if( frame.empty() ) { 
			printf( "Camera error!\n");
			break;
		}; // end of video stream

		cv::cvtColor(frame, frame, CV_BGR2GRAY);
		undistorter->undistort(frame, bufferItem.data);

		imageBuffer->pushBack(bufferItem);
	}

	// For some reason the camera failed. End program. (Probably should restart it to prevent crash, or alt least perform safe landing)
	printf( "Camera error, shutting down (camera unplugged?)\n" );
	exit(0);
}


}
