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
#include "util/settings.h"
#include "IOWrapper/TimestampedObject.h"
#include "util/SophusUtil.h"



namespace lsd_slam
{

template< typename T >
class NotifyBuffer;

class Frame;

SE3 SE3CV2Sophus(const cv::Mat& R, const cv::Mat& t);

void printMessageOnCVImage(cv::Mat &image, std::string line1,std::string line2);

// reads interpolated element from a uchar* array
// SSE2 optimization possible
inline float getInterpolatedElement(const float* const mat, const float x, const float y, const int width)
{
	//stats.num_pixelInterpolations++;

	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const float* bp = mat +ix+iy*width;


	float res =   dxdy * bp[1+width]
				+ (dy-dxdy) * bp[width]
				+ (dx-dxdy) * bp[1]
				+ (1-dx-dy+dxdy) * bp[0];

	return res;
}

inline Eigen::Vector3f getInterpolatedElement43(const Eigen::Vector4f* const mat, const float x, const float y, const int width)
{
	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const Eigen::Vector4f* bp = mat +ix+iy*width;


	return dxdy * *(const Eigen::Vector3f*)(bp+1+width)
	        + (dy-dxdy) * *(const Eigen::Vector3f*)(bp+width)
	        + (dx-dxdy) * *(const Eigen::Vector3f*)(bp+1)
			+ (1-dx-dy+dxdy) * *(const Eigen::Vector3f*)(bp);
}

inline Eigen::Vector4f getInterpolatedElement44(const Eigen::Vector4f* const mat, const float x, const float y, const int width)
{
	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const Eigen::Vector4f* bp = mat +ix+iy*width;


	return dxdy * *(bp+1+width)
	        + (dy-dxdy) * *(bp+width)
	        + (dx-dxdy) * *(bp+1)
			+ (1-dx-dy+dxdy) * *(bp);
}

inline Eigen::Vector2f getInterpolatedElement42(const Eigen::Vector4f* const mat, const float x, const float y, const int width)
{
	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const Eigen::Vector4f* bp = mat +ix+iy*width;


	return dxdy * *(const Eigen::Vector2f*)(bp+1+width)
	        + (dy-dxdy) * *(const Eigen::Vector2f*)(bp+width)
	        + (dx-dxdy) * *(const Eigen::Vector2f*)(bp+1)
			+ (1-dx-dy+dxdy) * *(const Eigen::Vector2f*)(bp);
}
inline void fillCvMat(cv::Mat* mat, cv::Vec3b color)
{
	for(int y=0;y<mat->size().height;y++)
		for(int x=0;x<mat->size().width;x++)
			mat->at<cv::Vec3b>(y,x) = color;
}

inline void setPixelInCvMat(cv::Mat* mat, cv::Vec3b color, int xx, int yy, int lvlFac)
{
	for(int x=xx*lvlFac; x < (xx+1)*lvlFac && x < mat->size().width;x++)
		for(int y=yy*lvlFac; y < (yy+1)*lvlFac && y < mat->size().height;y++)
			mat->at<cv::Vec3b>(y,x) = color;
}

inline cv::Vec3b getGrayCvPixel(float val)
{
	if(val < 0) val = 0; if(val>255) val=255;
	return cv::Vec3b(val,val,val);
}

cv::Mat getDepthRainbowPlot(Frame* kf, int lvl=0);
cv::Mat getDepthRainbowPlot(const float* idepth, const float* idepthVar, const float* gray, int width, int height);
cv::Mat getVarRedGreenPlot(const float* idepthVar, const float* gray, int width, int height);
}
