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


#include <dynamic_reconfigure/server.h>
#include "lsd_slam_core/LSDParamsConfig.h"
#include "lsd_slam_core/LSDDebugParamsConfig.h"
#include "util/settings.h"



namespace lsd_slam
{


void dynConfCbDebug(lsd_slam_core::LSDDebugParamsConfig &config, uint32_t level)
{
	freeDebugParam1 = config.freeDebugParam1;
	freeDebugParam2 = config.freeDebugParam2;
	freeDebugParam3 = config.freeDebugParam3;
	freeDebugParam4 = config.freeDebugParam4;
	freeDebugParam5 = config.freeDebugParam5;


	plotStereoImages = config.plotStereoImages;
	plotTrackingIterationInfo = config.plotTrackingIterationInfo;
	plotTracking = config.plotTracking;

	printPropagationStatistics = config.printPropagationStatistics;
	printFillHolesStatistics = config.printFillHolesStatistics;
	printObserveStatistics = config.printObserveStatistics;
	printObservePurgeStatistics = config.printObservePurgeStatistics;
	printRegularizeStatistics = config.printRegularizeStatistics;
	printLineStereoStatistics = config.printLineStereoStatistics;
	printLineStereoFails = config.printLineStereoFails;

	printFrameBuildDebugInfo = config.printFrameBuildDebugInfo;
	printMemoryDebugInfo = config.printMemoryDebugInfo;

	printTrackingIterationInfo = config.printTrackingIterationInfo;
	printThreadingInfo = config.printThreadingInfo;
	printMappingTiming = config.printMappingTiming;
	printOverallTiming = config.printOverallTiming;

	printKeyframeSelectionInfo = config.printKeyframeSelectionInfo;
	printConstraintSearchInfo = config.printConstraintSearchInfo;
	printOptimizationInfo = config.printOptimizationInfo;
	printRelocalizationInfo = config.printRelocalizationInfo;


	continuousPCOutput = config.continuousPCOutput;


	saveKeyframes = config.saveKeyframes;
	saveAllTracked = config.saveAllTracked;
	saveLoopClosureImages = config.saveLoopClosureImages;
	saveAllTrackingStages = config.saveAllTrackingStages;
}

void dynConfCb(lsd_slam_core::LSDParamsConfig &config, uint32_t level)
{
	allowNegativeIdepths = config.allowNegativeIdepths;
	useSubpixelStereo = config.useSubpixelStereo;
	multiThreading = config.multiThreading;
	useAffineLightningEstimation = config.useAffineLightningEstimation;

	KFDistWeight = config.KFDistWeight;
	KFUsageWeight = config.KFUsageWeight;

	minUseGrad = config.minUseGrad;
	cameraPixelNoise2 = config.cameraPixelNoise*config.cameraPixelNoise;
	depthSmoothingFactor = config.depthSmoothingFactor;


	doSlam = config.doSLAM;
	useFabMap = config.useFabMap;
	doKFReActivation = config.doKFReActivation;
	doMapping = config.doMapping;

	maxLoopClosureCandidates = config.maxLoopClosureCandidates;
	loopclosureStrictness = config.loopclosureStrictness;
	relocalizationTH = config.relocalizationTH;
}

}
