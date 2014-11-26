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
#include "util/EigenCoreInclude.h"
#include "util/SophusUtil.h"
#include "Tracking/LGSX.h"


namespace lsd_slam
{

class TrackingReference;
class Frame;


class SE3Tracker
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	int width, height;

	// camera matrix
	Eigen::Matrix3f K, KInv;
	float fx,fy,cx,cy;
	float fxi,fyi,cxi,cyi;

	DenseDepthTrackerSettings settings;


	// debug images
	cv::Mat debugImageResiduals;
	cv::Mat debugImageWeights;
	cv::Mat debugImageSecondFrame;
	cv::Mat debugImageOldImageSource;
	cv::Mat debugImageOldImageWarped;


	SE3Tracker(int w, int h, Eigen::Matrix3f K);
	SE3Tracker(const SE3Tracker&) = delete;
	SE3Tracker& operator=(const SE3Tracker&) = delete;
	~SE3Tracker();


	SE3 trackFrame(
			TrackingReference* reference,
			Frame* frame,
			const SE3& frameToReference_initialEstimate);
	

	SE3 trackFrameOnPermaref(
			Frame* reference,
			Frame* frame,
			SE3 referenceToFrame);


	float checkPermaRefOverlap(
			Frame* reference,
			SE3 referenceToFrame);


	float pointUsage;
	float lastGoodCount;
	float lastMeanRes;
	float lastBadCount;
	float lastResidual;

	float affineEstimation_a;
	float affineEstimation_b;


	bool diverged;
	bool trackingWasGood;
private:



	float* buf_warped_residual;
	float* buf_warped_dx;
	float* buf_warped_dy;
	float* buf_warped_x;
	float* buf_warped_y;
	float* buf_warped_z;

	float* buf_d;
	float* buf_idepthVar;
	float* buf_weight_p;

	int buf_warped_size;


	float calcResidualAndBuffers(
			const Eigen::Vector3f* refPoint,
			const Eigen::Vector2f* refColVar,
			int* idxBuf,
			int refNum,
			Frame* frame,
			const Sophus::SE3f& referenceToFrame,
			int level,
			bool plotResidual = false);

#if defined(ENABLE_SSE)
	float calcResidualAndBuffersSSE(
			const Eigen::Vector3f* refPoint,
			const Eigen::Vector2f* refColVar,
			int* idxBuf,
			int refNum,
			Frame* frame,
			const Sophus::SE3f& referenceToFrame,
			int level,
			bool plotResidual = false);
#endif
#if defined(ENABLE_NEON)
	float calcResidualAndBuffersNEON(
			const Eigen::Vector3f* refPoint,
			const Eigen::Vector2f* refColVar,
			int* idxBuf,
			int refNum,
			Frame* frame,
			const Sophus::SE3f& referenceToFrame,
			int level,
			bool plotResidual = false);
#endif






	float calcWeightsAndResidual(
			const Sophus::SE3f& referenceToFrame);
#if defined(ENABLE_SSE)
	float calcWeightsAndResidualSSE(
			const Sophus::SE3f& referenceToFrame);
#endif
#if defined(ENABLE_NEON)
	float calcWeightsAndResidualNEON(
			const Sophus::SE3f& referenceToFrame);
#endif






	void calculateWarpUpdate(
			LGS6 &ls);
#if defined(ENABLE_SSE)
	void calculateWarpUpdateSSE(
			LGS6 &ls);
#endif
#if defined(ENABLE_NEON)
	void calculateWarpUpdateNEON(
			LGS6 &ls);
#endif

	void calcResidualAndBuffers_debugStart();
	void calcResidualAndBuffers_debugFinish(int w);


	// used for image saving
	int iterationNumber;


	float affineEstimation_a_lastIt;
	float affineEstimation_b_lastIt;
};

}
