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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>



namespace lsd_slam
{

namespace Util
{

	struct DisplayImageObect
	{
		cv::Mat img;
		std::string name;
		bool autoSize;
	};


/// Image display function working on different platforms.
/// On Android, the window name is ignored as all images are output fullscreen.
void displayImage(const char* windowName, const cv::Mat& image, bool autoSize = true);

/// Convenience function which internally converts the image to a cv::Mat
inline void displayImage(const char* windowName, const float* image, int width, int height)
{
	cv::Mat floatWrapper(height, width, CV_32F, const_cast<float*>(image));
	cv::Mat tempImage(height, width, CV_8UC1);
	floatWrapper.convertTo(tempImage, CV_8UC1);
	cv::cvtColor(tempImage, tempImage, CV_GRAY2RGB);
	displayImage(windowName, tempImage);
}

/// Waits for key input at most the given amount of milliseconds and returns the keycode.
/// If milliseconds is zero, waits until a key is pressed.
/// This may be a no-op on some platforms (e.g. Android).
/// A window shown with displayImage must be active for this to work.
int waitKey(int milliseconds);

/// Just like waitKey(), but does not consume the pressed key, so that the next
/// call to waitKey() will still return this key (as long as no other key is
/// pressed in between).
int waitKeyNoConsume(int milliseconds);

void closeAllWindows();

}

}
