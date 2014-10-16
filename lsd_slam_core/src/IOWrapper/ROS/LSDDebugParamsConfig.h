/*
 * LSDDebugParamsConfig.h
 *
 *  Created on: 16 Oct 2014
 *      Author: thomas
 */

#ifndef LSDDEBUGPARAMSCONFIG_H_
#define LSDDEBUGPARAMSCONFIG_H_

namespace lsd_slam_core
{
    class LSDDebugParamsConfig
    {
        public:
            LSDDebugParamsConfig()
             : plotStereoImages(false),
               plotTracking(false),
               freeDebugParam1(1),
               freeDebugParam2(1),
               freeDebugParam3(1),
               freeDebugParam4(1),
               freeDebugParam5(1),
               plotTrackingIterationInfo(false),
               printPropagationStatistics(false),
               printFillHolesStatistics(false),
               printObserveStatistics(false),
               printObservePurgeStatistics(false),
               printRegularizeStatistics(false),
               printLineStereoStatistics(false),
               printLineStereoFails(false),
               printFrameBuildDebugInfo(false),
               printMemoryDebugInfo(false),
               printTrackingIterationInfo(false),
               printThreadingInfo(false),
               printMappingTiming(false),
               printOverallTiming(false),
               printKeyframeSelectionInfo(false),
               printConstraintSearchInfo(false),
               printOptimizationInfo(false),
               printRelocalizationInfo(false),
               continuousPCOutput(false),
               saveKeyframes(false),
               saveAllTracked(false),
               saveLoopClosureImages(false),
               saveAllTrackingStages(false)
            {}

            bool plotStereoImages;
            bool plotTracking;

            double freeDebugParam1;
            double freeDebugParam2;
            double freeDebugParam3;
            double freeDebugParam4;
            double freeDebugParam5;

            bool plotTrackingIterationInfo;

            bool printPropagationStatistics;
            bool printFillHolesStatistics;
            bool printObserveStatistics;
            bool printObservePurgeStatistics;
            bool printRegularizeStatistics;
            bool printLineStereoStatistics;
            bool printLineStereoFails;

            bool printFrameBuildDebugInfo;
            bool printMemoryDebugInfo;

            bool printTrackingIterationInfo;
            bool printThreadingInfo;
            bool printMappingTiming;
            bool printOverallTiming;

            bool printKeyframeSelectionInfo;
            bool printConstraintSearchInfo;
            bool printOptimizationInfo;
            bool printRelocalizationInfo;

            bool continuousPCOutput;
            bool saveKeyframes;
            bool saveAllTracked;
            bool saveLoopClosureImages;
            bool saveAllTrackingStages;
    };
}

#endif /* LSDDEBUGPARAMSCONFIG_H_ */
