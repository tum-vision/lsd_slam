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
#include "util/EigenCoreInclude.h"
#include "opencv2/core/core.hpp"
#include "util/settings.h"
#include "util/IndexThreadReduce.h"
#include "util/SophusUtil.h"



namespace lsd_slam
{

class DepthMapPixelHypothesis;
class Frame;
class KeyFrameGraph;


/**
 * Keeps a detailed depth map (consisting of DepthMapPixelHypothesis) and does
 * stereo comparisons and regularization to update it.
 */
class DepthMap
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	DepthMap(int w, int h, const Eigen::Matrix3f& K);
	DepthMap(const DepthMap&) = delete;
	DepthMap& operator=(const DepthMap&) = delete;
	~DepthMap();

	/** Resets everything. */
	void reset();
	
	/**
	 * does obervation and regularization only.
	 **/
	void updateKeyframe(std::deque< std::shared_ptr<Frame> > referenceFrames);

	/**
	 * does propagation and whole-filling-regularization (no observation, for that need to call updateKeyframe()!)
	 **/
	void createKeyFrame(Frame* new_keyframe);
	
	/**
	 * does one fill holes iteration
	 */
	void finalizeKeyFrame();

	void invalidate();
	inline bool isValid() {return activeKeyFrame!=0;};

	int debugPlotDepthMap();

	// ONLY for debugging, their memory is managed (created & deleted) by this object.
	cv::Mat debugImageHypothesisHandling;
	cv::Mat debugImageHypothesisPropagation;
	cv::Mat debugImageStereoLines;
	cv::Mat debugImageDepth;

	void initializeFromGTDepth(Frame* new_frame);
	void initializeRandomly(Frame* new_frame);

	void setFromExistingKF(Frame* kf);

	void addTimingSample();
	float msUpdate, msCreate, msFinalize;
	float msObserve, msRegularize, msPropagate, msFillHoles, msSetDepth;
	int nUpdate, nCreate, nFinalize;
	int nObserve, nRegularize, nPropagate, nFillHoles, nSetDepth;
	struct timeval lastHzUpdate;
	float nAvgUpdate, nAvgCreate, nAvgFinalize;
	float nAvgObserve, nAvgRegularize, nAvgPropagate, nAvgFillHoles, nAvgSetDepth;



	// pointer to global keyframe graph
	IndexThreadReduce threadReducer;

private:
	// camera matrix etc.
	Eigen::Matrix3f K, KInv;
	float fx,fy,cx,cy;
	float fxi,fyi,cxi,cyi;
	int width, height;


	// ============= parameter copies for convenience ===========================
	// these are just copies of the pointers given to this function, for convenience.
	// these are NOT managed by this object!
	Frame* activeKeyFrame;
	boost::shared_lock<boost::shared_mutex> activeKeyFramelock;
	const float* activeKeyFrameImageData;
	bool activeKeyFrameIsReactivated;

	Frame* oldest_referenceFrame;
	Frame* newest_referenceFrame;
	std::vector<Frame*> referenceFrameByID;
	int referenceFrameByID_offset;

	// ============= internally used buffers for intermediate calculations etc. =============
	// for internal depth tracking, their memory is managed (created & deleted) by this object.
	DepthMapPixelHypothesis* otherDepthMap;
	DepthMapPixelHypothesis* currentDepthMap;
	int* validityIntegralBuffer;

	

	// ============ internal functions ==================================================
	// does the line-stereo seeking.
	// takes a lot of parameters, because they all have been pre-computed before.
	inline float doLineStereo(
			const float u, const float v, const float epxn, const float epyn,
			const float min_idepth, const float prior_idepth, float max_idepth,
			const Frame* const referenceFrame, const float* referenceFrameImage,
			float &result_idepth, float &result_var, float &result_eplLength,
			RunningStats* const stats);


	void propagateDepth(Frame* new_keyframe);
	

	void observeDepth();
	void observeDepthRow(int yMin, int yMax, RunningStats* stats);
	bool observeDepthCreate(const int &x, const int &y, const int &idx, RunningStats* const &stats);
	bool observeDepthUpdate(const int &x, const int &y, const int &idx, const float* keyFrameMaxGradBuf, RunningStats* const &stats);
	bool makeAndCheckEPL(const int x, const int y, const Frame* const ref, float* pepx, float* pepy, RunningStats* const stats);


	void regularizeDepthMap(bool removeOcclusion, int validityTH);
	template<bool removeOcclusions> void regularizeDepthMapRow(int validityTH, int yMin, int yMax, RunningStats* stats);


	void buildRegIntegralBuffer();
	void buildRegIntegralBufferRow1(int yMin, int yMax, RunningStats* stats);
	void regularizeDepthMapFillHoles();
	void regularizeDepthMapFillHolesRow(int yMin, int yMax, RunningStats* stats);


	void resetCounters();

	//float clocksPropagate, clocksPropagateKF, clocksObserve, msObserve, clocksReg1, clocksReg2, msReg1, msReg2, clocksFinalize;
};

}
