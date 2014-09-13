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

#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"



namespace lsd_slam
{

/**
 * Virtual ImageStream. Can be from OpenCV's ImageCapture, ROS or Android.
 * Also has to provide the camera calibration for that stream, as well as the respective undistorter object (if required).
 * Runs in it's own thread, and has a NotifyBuffer, in which received images are stored.
 */
class InputImageStream
{
public:
	virtual ~InputImageStream() {};
	
	/**
	 * Starts the thread.
	 */
	virtual void run() {};


	virtual void setCalibration(std::string file) {};

	/**
	 * Gets the NotifyBuffer to which incoming images are stored.
	 */
	inline NotifyBuffer<TimestampedMat>* getBuffer() {return imageBuffer;};


	/**
	 * Gets the Camera Calibration. To avoid any dependencies, just as simple float / int's.
	 */
	inline float fx() {return fx_;}
	inline float fy() {return fy_;}
	inline float cx() {return cx_;}
	inline float cy() {return cy_;}
	inline int width() {return width_;}
	inline int height() {return height_;}

protected:
	NotifyBuffer<TimestampedMat>* imageBuffer;
	float fx_, fy_, cx_, cy_;
	int width_, height_;
};
}
