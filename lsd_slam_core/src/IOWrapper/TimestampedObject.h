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

#ifndef _TIME_STAMPED_OBJECT_HPP_
#define _TIME_STAMPED_OBJECT_HPP_

#include <chrono>

#include <opencv2/core/core.hpp>

#include "IOWrapper/Timestamp.h"


namespace lsd_slam
{

template<typename T>
struct TimestampedObject
{
	T data;
	Timestamp timestamp;
};

typedef TimestampedObject< cv::Mat > TimestampedMat;

}
#endif
