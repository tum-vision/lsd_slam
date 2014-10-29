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
#include "util/SophusUtil.h"
#include "util/settings.h"
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include "DataStructures/FramePoseStruct.h"
#include "DataStructures/FrameMemory.h"
#include "unordered_set"
#include "util/settings.h"


namespace lsd_slam
{


class DepthMapPixelHypothesis;
class TrackingReference;
/**
 */

class Frame
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	friend class FrameMemory;


	Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image);

	Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const float* image);

	~Frame();
	
	
	/** Sets or updates idepth and idepthVar on level zero. Invalidates higher levels. */
	void setDepth(const DepthMapPixelHypothesis* newDepth);

	/** Calculates mean information for statistical purposes. */
	void calculateMeanInformation();
	
	/** Sets ground truth depth (real, not inverse!) from a float array on level zero. Invalidates higher levels. */
	void setDepthFromGroundTruth(const float* depth, float cov_scale = 1.0f);
	
	/** Prepares this frame for stereo comparisons with the other frame (computes some intermediate values that will be needed) */
	void prepareForStereoWith(Frame* other, Sim3 thisToOther, const Eigen::Matrix3f& K, const int level);

	

	// Accessors
	/** Returns the unique frame id. */
	inline int id() const;
	
	/** Returns the frame's image width. */
	inline int width(int level = 0) const;
	/** Returns the frame's image height. */
	inline int height(int level = 0) const;
	
	/** Returns the frame's intrinsics matrix. */
	inline const Eigen::Matrix3f& K(int level = 0) const;
	/** Returns the frame's inverse intrinsics matrix. */
	inline const Eigen::Matrix3f& KInv(int level = 0) const;
	/** Returns K(0, 0). */
	inline float fx(int level = 0) const;
	/** Returns K(1, 1). */
	inline float fy(int level = 0) const;
	/** Returns K(0, 2). */
	inline float cx(int level = 0) const;
	/** Returns K(1, 2). */
	inline float cy(int level = 0) const;
	/** Returns KInv(0, 0). */
	inline float fxInv(int level = 0) const;
	/** Returns KInv(1, 1). */
	inline float fyInv(int level = 0) const;
	/** Returns KInv(0, 2). */
	inline float cxInv(int level = 0) const;
	/** Returns KInv(1, 2). */
	inline float cyInv(int level = 0) const;
	
	/** Returns the frame's recording timestamp. */
	inline double timestamp() const;
	
	inline float* image(int level = 0);
	inline const Eigen::Vector4f* gradients(int level = 0);
	inline const float* maxGradients(int level = 0);
	inline bool hasIDepthBeenSet() const;
	inline const float* idepth(int level = 0);
	inline const float* idepthVar(int level = 0);
	inline const unsigned char* validity_reAct();
	inline const float* idepth_reAct();
	inline const float* idepthVar_reAct();

	inline bool* refPixelWasGood();
	inline bool* refPixelWasGoodNoCreate();
	inline void clear_refPixelWasGood();

	/** Flags for use with require() and requirePyramid(). See the Frame class
	  * documentation for their exact meaning. */
	enum DataFlags
	{
		IMAGE			= 1<<0,
		GRADIENTS		= 1<<1,
		MAX_GRADIENTS	= 1<<2,
		IDEPTH			= 1<<3,
		IDEPTH_VAR		= 1<<4,
		REF_ID			= 1<<5,
		
		ALL = IMAGE | GRADIENTS | MAX_GRADIENTS | IDEPTH | IDEPTH_VAR | REF_ID
	};
	

	void setPermaRef(TrackingReference* reference);
	void takeReActivationData(DepthMapPixelHypothesis* depthMap);


	// shared_lock this as long as any minimizable arrays are being used.
	// the minimizer will only minimize frames after getting
	// an exclusive lock on this.
	inline boost::shared_lock<boost::shared_mutex> getActiveLock()
	{
		return FrameMemory::getInstance().activateFrame(this);
	}


	/*
	 * ==================================================================================
	 * Here are ALL central pose and scale informations.
	 * generally, everything is stored relative to the frame
	 */
	FramePoseStruct* pose;
	Sim3 getScaledCamToWorld(int num=0) { return pose->getCamToWorld();}
	bool hasTrackingParent() { return pose->trackingParent != nullptr;}
	Frame* getTrackingParent() { return pose->trackingParent->frame;}

	Sim3 lastConstraintTrackedCamToWorld;



	/** Pointers to all adjacent Frames in graph. empty for non-keyframes.*/
	std::unordered_set< Frame*, std::hash<Frame*>, std::equal_to<Frame*>,
		Eigen::aligned_allocator< Frame* > > neighbors;

	/** Multi-Map indicating for which other keyframes with which initialization tracking failed.*/
	std::unordered_multimap< Frame*, Sim3, std::hash<Frame*>, std::equal_to<Frame*>,
		Eigen::aligned_allocator< std::pair<const Frame*,Sim3> > > trackingFailed;


	// flag set when depth is updated.
	bool depthHasBeenUpdatedFlag;


	// Tracking Reference for quick test. Always available, never taken out of memory.
	// this is used for re-localization and re-Keyframe positioning.
	boost::mutex permaRef_mutex;
	Eigen::Vector3f* permaRef_posData;	// (x,y,z)
	Eigen::Vector2f* permaRef_colorAndVarData;	// (I, Var)
	int permaRefNumPts;



	// Temporary values
	int referenceID;
	int referenceLevel;
	float distSquared;
	Eigen::Matrix3f K_otherToThis_R;
	Eigen::Vector3f K_otherToThis_t;
	Eigen::Vector3f otherToThis_t;
	Eigen::Vector3f K_thisToOther_t;
	Eigen::Matrix3f thisToOther_R;
	Eigen::Vector3f otherToThis_R_row0;
	Eigen::Vector3f otherToThis_R_row1;
	Eigen::Vector3f otherToThis_R_row2;
	Eigen::Vector3f thisToOther_t;



	// statistics
	float initialTrackedResidual;
	int numFramesTrackedOnThis;
	int numMappedOnThis;
	int numMappedOnThisTotal;
	float meanIdepth;
	int numPoints;
	int idxInKeyframes;
	float edgeErrorSum, edgesNum;
	int numMappablePixels;
	float meanInformation;

private:

	void require(int dataFlags, int level = 0);
	void release(int dataFlags, bool pyramidsOnly, bool invalidateOnly);

	void initialize(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp);
	void setDepth_Allocate();
	
	void buildImage(int level);
	void releaseImage(int level);
	
	void buildGradients(int level);
	void releaseGradients(int level);
	
	void buildMaxGradients(int level);
	void releaseMaxGradients(int level);
	
	void buildIDepthAndIDepthVar(int level);
	void releaseIDepth(int level);
	void releaseIDepthVar(int level);
	
	void printfAssert(const char* message) const;
	
	struct Data
	{
		int id;
		
		int width[PYRAMID_LEVELS], height[PYRAMID_LEVELS];

		Eigen::Matrix3f K[PYRAMID_LEVELS], KInv[PYRAMID_LEVELS];
		float fx[PYRAMID_LEVELS], fy[PYRAMID_LEVELS], cx[PYRAMID_LEVELS], cy[PYRAMID_LEVELS];
		float fxInv[PYRAMID_LEVELS], fyInv[PYRAMID_LEVELS], cxInv[PYRAMID_LEVELS], cyInv[PYRAMID_LEVELS];
		
		double timestamp;

		
		float* image[PYRAMID_LEVELS];
		bool imageValid[PYRAMID_LEVELS];
		
		Eigen::Vector4f* gradients[PYRAMID_LEVELS];
		bool gradientsValid[PYRAMID_LEVELS];
		
		float* maxGradients[PYRAMID_LEVELS];
		bool maxGradientsValid[PYRAMID_LEVELS];
		

		bool hasIDepthBeenSet;

		// negative depthvalues are actually allowed, so setting this to -1 does NOT invalidate the pixel's depth.
		// a pixel is valid iff idepthVar[i] > 0.
		float* idepth[PYRAMID_LEVELS];
		bool idepthValid[PYRAMID_LEVELS];
		
		// MUST contain -1 for invalid pixel (that dont have depth)!!
		float* idepthVar[PYRAMID_LEVELS];
		bool idepthVarValid[PYRAMID_LEVELS];

		// data needed for re-activating the frame. theoretically, this is all data the
		// frame contains.
		unsigned char* validity_reAct;
		float* idepth_reAct;
		float* idepthVar_reAct;
		bool reActivationDataValid;


		// data from initial tracking, indicating which pixels in the reference frame ware good or not.
		// deleted as soon as frame is used for mapping.
		bool* refPixelWasGood;
	};
	Data data;


	// used internally. locked while something is being built, such that no
	// two threads build anything simultaneously. not locked on require() if nothing is changed.
	boost::mutex buildMutex;

	boost::shared_mutex activeMutex;
	bool isActive;

	/** Releases everything which can be recalculated, but keeps the minimal
	  * representation in memory. Use release(Frame::ALL, false) to store on disk instead.
	  * ONLY CALL THIS, if an exclusive lock on activeMutex is owned! */
	bool minimizeInMemory();
};



inline int Frame::id() const
{
	return data.id;
}

inline int Frame::width(int level) const
{
	return data.width[level];
}

inline int Frame::height(int level) const
{
	return data.height[level];
}

inline const Eigen::Matrix3f& Frame::K(int level) const
{
	return data.K[level];
}
inline const Eigen::Matrix3f& Frame::KInv(int level) const
{
	return data.KInv[level];
}
inline float Frame::fx(int level) const
{
	return data.fx[level];
}
inline float Frame::fy(int level) const
{
	return data.fy[level];
}
inline float Frame::cx(int level) const
{
	return data.cx[level];
}
inline float Frame::cy(int level) const
{
	return data.cy[level];
}
inline float Frame::fxInv(int level) const
{
	return data.fxInv[level];
}
inline float Frame::fyInv(int level) const
{
	return data.fyInv[level];
}
inline float Frame::cxInv(int level) const
{
	return data.cxInv[level];
}
inline float Frame::cyInv(int level) const
{
	return data.cyInv[level];
}

inline double Frame::timestamp() const
{
	return data.timestamp;
}


inline float* Frame::image(int level)
{
	if (! data.imageValid[level])
		require(IMAGE, level);
	return data.image[level];
}
inline const Eigen::Vector4f* Frame::gradients(int level)
{
	if (! data.gradientsValid[level])
		require(GRADIENTS, level);
	return data.gradients[level];
}
inline const float* Frame::maxGradients(int level)
{
	if (! data.maxGradientsValid[level])
		require(MAX_GRADIENTS, level);
	return data.maxGradients[level];
}
inline bool Frame::hasIDepthBeenSet() const
{
	return data.hasIDepthBeenSet;
}
inline const float* Frame::idepth(int level)
{
	if (! data.hasIDepthBeenSet)
	{
		printfAssert("Frame::idepth(): idepth has not been set yet!");
		return nullptr;
	}
	if (! data.idepthValid[level])
		require(IDEPTH, level);
	return data.idepth[level];
}
inline const unsigned char* Frame::validity_reAct()
{
	if( !data.reActivationDataValid)
		return 0;
	return data.validity_reAct;
}
inline const float* Frame::idepth_reAct()
{
	if( !data.reActivationDataValid)
		return 0;
	return data.idepth_reAct;
}
inline const float* Frame::idepthVar_reAct()
{
	if( !data.reActivationDataValid)
		return 0;
	return data.idepthVar_reAct;
}
inline const float* Frame::idepthVar(int level)
{
	if (! data.hasIDepthBeenSet)
	{
		printfAssert("Frame::idepthVar(): idepth has not been set yet!");
		return nullptr;
	}
	if (! data.idepthVarValid[level])
		require(IDEPTH_VAR, level);
	return data.idepthVar[level];
}


inline bool* Frame::refPixelWasGood()
{
	if( data.refPixelWasGood == 0)
	{
		boost::unique_lock<boost::mutex> lock2(buildMutex);

		if(data.refPixelWasGood == 0)
		{
			int width = data.width[SE3TRACKING_MIN_LEVEL];
			int height = data.height[SE3TRACKING_MIN_LEVEL];
			data.refPixelWasGood = (bool*)FrameMemory::getInstance().getBuffer(sizeof(bool) * width * height);

			memset(data.refPixelWasGood, 0xFFFFFFFF, sizeof(bool) * (width * height));
		}
	}
	return data.refPixelWasGood;
}


inline bool* Frame::refPixelWasGoodNoCreate()
{
	return data.refPixelWasGood;
}

inline void Frame::clear_refPixelWasGood()
{
	FrameMemory::getInstance().returnBuffer(reinterpret_cast<float*>(data.refPixelWasGood));
	data.refPixelWasGood=0;
}


}
