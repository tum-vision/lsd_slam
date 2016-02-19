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

#include "IOWrapper/Timestamp.h"

#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>

namespace lsd_slam
{


const std::chrono::steady_clock::time_point Timestamp::startupTimePoint = std::chrono::steady_clock::now();
boost::mutex Timestamp::localtimeMutex;

Timestamp::Timestamp()
{
	externalStamp = 0;
}

Timestamp::Timestamp(double seconds)
{
	externalStamp = seconds;
}

double Timestamp::toSec() const
{
	if(externalStamp!=0) return externalStamp;
	return std::chrono::duration<double>(timePoint - startupTimePoint).count();
}

std::string Timestamp::toDateStr(const char* format) const
{
	auto in_time_t = std::chrono::system_clock::to_time_t(systemTimePoint);
	struct tm* loc_time_t;
	
	boost::unique_lock<boost::mutex> lock(localtimeMutex);
	// localtime is not re-entrant.
	loc_time_t = std::localtime(&in_time_t);
	char buffer[128];
	std::strftime(buffer, 128, format, loc_time_t);
	lock.unlock();
	
    return buffer;
}

double Timestamp::secondsUntil(const Timestamp& other) const
{
	return std::chrono::duration<double>(other.timePoint - timePoint).count();
}

Timestamp Timestamp::now()
{
	Timestamp result;
	result.timePoint = std::chrono::steady_clock::now();
	result.systemTimePoint = std::chrono::system_clock::now();
	return result;
}

}
