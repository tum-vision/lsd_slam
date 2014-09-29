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

#ifndef _TIMING_HPP_
#define _TIMING_HPP_

#include <string>
#include <chrono>
#include <boost/thread/mutex.hpp>



// TODO: remove this hack
namespace std {
	namespace chrono {
		#if (__GNUC__ > 4) || (__GNUC_MINOR__ >= 8)
			#define monotonic_clock steady_clock
		#endif
	}
}



namespace lsd_slam
{

/**
 * Represents a specific point in time.
 */
class Timestamp
{
public:
	/**
	 * Creates an uninitialized timestamp.
	 */
	Timestamp();
	Timestamp(double seconds);
	
	/**
	 * Returns the timestamp as the time in seconds which has passed since the
	 * start of the program until the timestamp was taken.
	 */
	double toSec() const;
	
	/**
	 * Returns the timestamp as a date string with format TODO.
	 */
	std::string toDateStr(const char* format) const;
	
	/**
	 * Returns the seconds from this timestamp to the other.
	 */
	double secondsUntil(const Timestamp& other) const;
	
	/**
	 * Returns a timestamp representing the current point in time.
	 */
	static Timestamp now();
	
private:
	std::chrono::monotonic_clock::time_point timePoint;
	std::chrono::system_clock::time_point systemTimePoint;
	
	static const std::chrono::monotonic_clock::time_point startupTimePoint;
	static boost::mutex localtimeMutex;

	double externalStamp;
};
}
#endif
