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

#include <string.h>
#include <string>



namespace lsd_slam
{


#define ALIGN __attribute__((__aligned__(16)))
#define SSEE(val,idx) (*(((float*)&val)+idx))
#define DIVISION_EPS 1e-10f
#define UNZERO(val) (val < 0 ? (val > -1e-10 ? -1e-10 : val) : (val < 1e-10 ? 1e-10 : val))

#if defined(ENABLE_SSE)
	#define USESSE true
#else
	#define USESSE false
#endif


#if defined(NDEBUG)
	#define enablePrintDebugInfo false
#else
	#define enablePrintDebugInfo true
#endif

/** ============== constants for validity handeling ======================= */

// validity can take values between 0 and X, where X depends on the abs. gradient at that location:
// it is calculated as VALIDITY_COUNTER_MAX + (absGrad/255)*VALIDITY_COUNTER_MAX_VARIABLE
#define VALIDITY_COUNTER_MAX (5.0f)		// validity will never be higher than this
#define VALIDITY_COUNTER_MAX_VARIABLE (250.0f)		// validity will never be higher than this

#define VALIDITY_COUNTER_INC 5		// validity is increased by this on sucessfull stereo
#define VALIDITY_COUNTER_DEC 5		// validity is decreased by this on failed stereo
#define VALIDITY_COUNTER_INITIAL_OBSERVE 5	// initial validity for first observations


#define VAL_SUM_MIN_FOR_CREATE (30) // minimal summed validity over 5x5 region to create a new hypothesis for non-blacklisted pixel (hole-filling)
#define VAL_SUM_MIN_FOR_KEEP (24) // minimal summed validity over 5x5 region to keep hypothesis (regularization)
#define VAL_SUM_MIN_FOR_UNBLACKLIST (100) // if summed validity surpasses this, a pixel is un-blacklisted.

#define MIN_BLACKLIST -1	// if blacklist is SMALLER than this, pixel gets ignored. blacklist starts with 0.




/** ============== Depth Variance Handeling ======================= */
#define SUCC_VAR_INC_FAC (1.01f) // before an ekf-update, the variance is increased by this factor.
#define FAIL_VAR_INC_FAC 1.1f // after a failed stereo observation, the variance is increased by this factor.
#define MAX_VAR (0.5f*0.5f) // initial variance on creation - if variance becomes larter than this, hypothesis is removed.

#define VAR_GT_INIT_INITIAL 0.01f*0.01f	// initial variance vor Ground Truth Initialization
#define VAR_RANDOM_INIT_INITIAL (0.5f*MAX_VAR)	// initial variance vor Random Initialization





// Whether to use the gradients of source and target frame for tracking,
// or only the target frame gradient
#define USE_ESM_TRACKING 1


#ifdef ANDROID
	// tracking pyramid levels.
	#define MAPPING_THREADS 2
	#define RELOCALIZE_THREADS 4
#else
	// tracking pyramid levels.
	#define MAPPING_THREADS 4
	#define RELOCALIZE_THREADS 6
#endif

#define SE3TRACKING_MIN_LEVEL 1
#define SE3TRACKING_MAX_LEVEL 5

#define SIM3TRACKING_MIN_LEVEL 1
#define SIM3TRACKING_MAX_LEVEL 5

#define QUICK_KF_CHECK_LVL 4

#define PYRAMID_LEVELS (SE3TRACKING_MAX_LEVEL > SIM3TRACKING_MAX_LEVEL ? SE3TRACKING_MAX_LEVEL : SIM3TRACKING_MAX_LEVEL)





// ============== stereo & gradient calculation ======================
#define MIN_DEPTH 0.05f // this is the minimal depth tested for stereo.

// particularely important for initial pixel.
#define MAX_EPL_LENGTH_CROP 30.0f // maximum length of epl to search.
#define MIN_EPL_LENGTH_CROP (3.0f) // minimum length of epl to search.

// this is the distance of the sample points used for the stereo descriptor.
#define GRADIENT_SAMPLE_DIST 1.0f

// pixel a point needs to be away from border... if too small: segfaults!
#define SAMPLE_POINT_TO_BORDER 7

// pixels with too big an error are definitely thrown out.
#define MAX_ERROR_STEREO (1300.0f) // maximal photometric error for stereo to be successful (sum over 5 squared intensity differences)
#define MIN_DISTANCE_ERROR_STEREO (1.5f) // minimal multiplicative difference to second-best match to not be considered ambiguous.

// defines how large the stereo-search region is. it is [mean] +/- [std.dev]*STEREO_EPL_VAR_FAC
#define STEREO_EPL_VAR_FAC 2.0f




// ============== Smoothing and regularization ======================
// distance factor for regularization.
// is used as assumed inverse depth variance between neighbouring pixel.
// basically determines the amount of spacial smoothing (small -> more smoothing).
#define REG_DIST_VAR (0.075f*0.075f*depthSmoothingFactor*depthSmoothingFactor)

// define how strict the merge-processes etc. are.
// are multiplied onto the difference, so the larger, the more restrictive.
#define DIFF_FAC_SMOOTHING (1.0f*1.0f)
#define DIFF_FAC_OBSERVE (1.0f*1.0f)
#define DIFF_FAC_PROP_MERGE (1.0f*1.0f)
#define DIFF_FAC_INCONSISTENT (1.0f * 1.0f)




// ============== initial stereo pixel selection ======================
#define MIN_EPL_GRAD_SQUARED (2.0f*2.0f)
#define MIN_EPL_LENGTH_SQUARED (1.0f*1.0f)
#define MIN_EPL_ANGLE_SQUARED (0.3f*0.3f)

// abs. grad at that location needs to be larger than this.
#define MIN_ABS_GRAD_CREATE (minUseGrad)
#define MIN_ABS_GRAD_DECREASE (minUseGrad)

// ============== RE-LOCALIZATION, KF-REACTIVATION etc. ======================
// defines the level on which we do the quick tracking-check for relocalization.



#define MAX_DIFF_CONSTANT (40.0f*40.0f)
#define MAX_DIFF_GRAD_MULT (0.5f*0.5f)

#define MIN_GOODPERGOODBAD_PIXEL (0.5f)
#define MIN_GOODPERALL_PIXEL (0.04f)
#define MIN_GOODPERALL_PIXEL_ABSMIN (0.01f)

#define INITIALIZATION_PHASE_COUNT 5

#define MIN_NUM_MAPPED 5

// settings variables
// controlled via keystrokes
extern bool autoRun;
extern bool autoRunWithinFrame;
extern int debugDisplay;
extern bool displayDepthMap;
extern bool onSceenInfoDisplay;
extern bool dumpMap;
extern bool doFullReConstraintTrack;


// dyn config
extern bool printPropagationStatistics;
extern bool printFillHolesStatistics;
extern bool printObserveStatistics;
extern bool printObservePurgeStatistics;
extern bool printRegularizeStatistics;
extern bool printLineStereoStatistics;
extern bool printLineStereoFails;

extern bool printTrackingIterationInfo;
extern bool printThreadingInfo;

extern bool printKeyframeSelectionInfo;
extern bool printConstraintSearchInfo;
extern bool printOptimizationInfo;
extern bool printRelocalizationInfo;

extern bool printFrameBuildDebugInfo;
extern bool printMemoryDebugInfo;

extern bool printMappingTiming;
extern bool printOverallTiming;
extern bool plotTrackingIterationInfo;
extern bool plotSim3TrackingIterationInfo;
extern bool plotStereoImages;
extern bool plotTracking;


extern bool allowNegativeIdepths;
extern bool useMotionModel;
extern bool useSubpixelStereo;
extern bool multiThreading;
extern bool useAffineLightningEstimation;

extern float freeDebugParam1;
extern float freeDebugParam2;
extern float freeDebugParam3;
extern float freeDebugParam4;
extern float freeDebugParam5;


extern float KFDistWeight;
extern float KFUsageWeight;
extern int maxLoopClosureCandidates;
extern int propagateKeyFrameDepthCount;
extern float loopclosureStrictness;
extern float relocalizationTH;


extern float minUseGrad;
extern float cameraPixelNoise2;
extern float depthSmoothingFactor;

extern bool useFabMap;
extern bool doSlam;
extern bool doKFReActivation;
extern bool doMapping;

extern bool saveKeyframes;
extern bool saveAllTracked;
extern bool saveLoopClosureImages;
extern bool saveAllTrackingStages;
extern bool saveAllTrackingStagesInternal;

extern bool continuousPCOutput;


/// Relative path of calibration file, map saving directory etc. for live_odometry
extern std::string packagePath;

extern bool fullResetRequested;
extern bool manualTrackingLossIndicated;
class RunningStats
{
public:
	int num_stereo_comparisons;
	int num_stereo_calls;
	int num_pixelInterpolations;

	int num_stereo_rescale_oob;
	int num_stereo_inf_oob;
	int num_stereo_near_oob;
	int num_stereo_invalid_unclear_winner;
	int num_stereo_invalid_atEnd;
	int num_stereo_invalid_inexistantCrossing;
	int num_stereo_invalid_twoCrossing;
	int num_stereo_invalid_noCrossing;
	int num_stereo_invalid_bigErr;
	int num_stereo_interpPre;
	int num_stereo_interpPost;
	int num_stereo_interpNone;
	int num_stereo_negative;
	int num_stereo_successfull;


	int num_observe_created;
	int num_observe_blacklisted;
	int num_observe_updated;
	int num_observe_skipped_small_epl;
	int num_observe_skipped_small_epl_grad;
	int num_observe_skipped_small_epl_angle;
	int num_observe_transit_finalizing;
	int num_observe_transit_idle_oob;
	int num_observe_transit_idle_scale_angle;
	int num_observe_trans_idle_exhausted;
	int num_observe_inconsistent_finalizing;
	int num_observe_inconsistent;
	int num_observe_notfound_finalizing2;
	int num_observe_notfound_finalizing;
	int num_observe_notfound;
	int num_observe_skip_fail;
	int num_observe_skip_oob;
	int num_observe_good;
	int num_observe_good_finalizing;
	int num_observe_state_finalizing;
	int num_observe_state_initializing;


	int num_observe_skip_alreadyGood;
	int num_observe_addSkip;



	int num_observe_no_grad_removed;
	int num_observe_no_grad_left;
	int num_observe_update_attempted;
	int num_observe_create_attempted;
	int num_observe_updated_ignored;
	int num_observe_spread_unsuccessfull;

	int num_prop_removed_out_of_bounds;
	int num_prop_removed_colorDiff;
	int num_prop_removed_validity;
	int num_prop_grad_decreased;
	int num_prop_color_decreased;
	int num_prop_attempts;
	int num_prop_occluded;
	int num_prop_created;
	int num_prop_merged;

	int num_reg_created;
	int num_reg_smeared;
	int num_reg_total;
	int num_reg_deleted_secondary;
	int num_reg_deleted_occluded;
	int num_reg_blacklisted;
	int num_reg_setBlacklisted;

	inline RunningStats()
	{
		setZero();
	}

	inline void setZero()
	{
		memset(this,0,sizeof(RunningStats));
	}

	inline void add(RunningStats* r)
	{
		int* pt = (int*)this;
		int* pt_r = (int*)r;
		for(int i=0;i<static_cast<int>(sizeof(RunningStats)/sizeof(int));i++)
			pt[i] += pt_r[i];
	}
};


class DenseDepthTrackerSettings
{
public:
	inline DenseDepthTrackerSettings()
	{
		// Set default settings
		if (PYRAMID_LEVELS > 6)
			printf("WARNING: Sim3Tracker(): default settings are intended for a maximum of 6 levels!");

		lambdaSuccessFac = 0.5f;
		lambdaFailFac = 2.0f;

		const float stepSizeMinc[6] = {1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8};
		const int maxIterations[6] = {5, 20, 50, 100, 100, 100};


		for (int level = 0; level < PYRAMID_LEVELS; ++ level)
		{
			lambdaInitial[level] = 0;
			stepSizeMin[level] = stepSizeMinc[level];
			convergenceEps[level] = 0.999f;
			maxItsPerLvl[level] = maxIterations[level];
		}

		lambdaInitialTestTrack = 0;
		stepSizeMinTestTrack = 1e-3;
		convergenceEpsTestTrack = 0.98;
		maxItsTestTrack = 5;

		var_weight = 1.0;
		huber_d = 3;
	}

	float lambdaSuccessFac;
	float lambdaFailFac;
	float lambdaInitial[PYRAMID_LEVELS];
	float stepSizeMin[PYRAMID_LEVELS];
	float convergenceEps[PYRAMID_LEVELS];
	int maxItsPerLvl[PYRAMID_LEVELS];

	float lambdaInitialTestTrack;
	float stepSizeMinTestTrack;
	float convergenceEpsTestTrack;
	float maxItsTestTrack;

	float huber_d;
	float var_weight;
};

extern RunningStats runningStats;

void handleKey(char k);
}
