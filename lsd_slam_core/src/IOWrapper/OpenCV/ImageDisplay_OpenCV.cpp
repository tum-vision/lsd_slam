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

#include <boost/thread.hpp>

namespace lsd_slam
{


namespace Util
{

	const bool useImageDisplayThread = true;


	std::unordered_set<std::string> openWindows;
	boost::mutex openCVdisplayMutex;
	boost::condition_variable  openCVdisplaySignal;


	boost::thread* imageDisplayThread = 0;
	std::vector<DisplayImageObect> displayQueue;
	bool imageThreadKeepRunning = true;


void displayThreadLoop()
{
	printf("started image display thread!\n");
	boost::unique_lock<boost::mutex> lock(openCVdisplayMutex);
	while(imageThreadKeepRunning)
	{
		openCVdisplaySignal.wait(lock);

		if(!imageThreadKeepRunning)
			break;

		while(displayQueue.size() > 0)
		{
			if(!displayQueue.back().autoSize)
			{
				if(openWindows.find(displayQueue.back().name) == openWindows.end())
				{
					cv::namedWindow(displayQueue.back().name, cv::WINDOW_NORMAL);
					cv::resizeWindow(displayQueue.back().name, displayQueue.back().img.cols, displayQueue.back().img.rows);
					openWindows.insert(displayQueue.back().name);
				}
			}
			cv::imshow(displayQueue.back().name, displayQueue.back().img);
			displayQueue.pop_back();
		}
	}
	cv::destroyAllWindows();
	openWindows.clear();

	printf("ended image display thread!\n");
}
void makeDisplayThread()
{
	imageThreadKeepRunning = true;
	imageDisplayThread = new boost::thread(&displayThreadLoop);
}
void displayImage(const char* windowName, const cv::Mat& image, bool autoSize)
{
	if(useImageDisplayThread)
	{
		if(imageDisplayThread == 0)
			makeDisplayThread();

		boost::unique_lock<boost::mutex> lock(openCVdisplayMutex);
		displayQueue.push_back(DisplayImageObect());
		displayQueue.back().autoSize = autoSize;
		displayQueue.back().img = image.clone();
		displayQueue.back().name = windowName;

		openCVdisplaySignal.notify_one();
	}
	else
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
	}
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
	boost::unique_lock<boost::mutex> lock(openCVdisplayMutex);

	if(useImageDisplayThread)
	{

		if(imageDisplayThread != 0)
		{
			imageThreadKeepRunning = false;
			openCVdisplaySignal.notify_all();
			printf("waiting for image display thread to end!\n");
			lock.unlock();
			imageDisplayThread->join();
			printf("done waiting for image display thread to end!\n");
			imageDisplayThread = 0;
		}
	}
	else
	{
		cv::destroyAllWindows();
		openWindows.clear();
	}
}
}

}
