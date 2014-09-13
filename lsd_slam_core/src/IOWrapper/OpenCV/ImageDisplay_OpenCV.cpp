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

#include "IOWrapper/ImageDisplay.h"

#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <unordered_set>

namespace lsd_slam
{


namespace Util
{

	std::unordered_set<std::string> openWindows;


void displayImage(const char* windowName, const cv::Mat& image, bool autoSize)
{
	if(!autoSize)
	{
		if(openWindows.find(windowName) == openWindows.end())
		{
			cv::namedWindow(windowName, cv::WINDOW_NORMAL);
			cv::resizeWindow(windowName, image.cols, image.rows);
			openWindows.insert(windowName);
		}
	}
	cv::imshow(windowName, image);
	//cv::waitKey(1);
}

int waitKey(int milliseconds)
{
	return cv::waitKey(milliseconds);
}

int waitKeyNoConsume(int milliseconds)
{
	// Cannot implement this with OpenCV functions.
	return cv::waitKey(milliseconds);
}

void closeAllWindows()
{
	cv::destroyAllWindows();
	openWindows.clear();
}
}

}
