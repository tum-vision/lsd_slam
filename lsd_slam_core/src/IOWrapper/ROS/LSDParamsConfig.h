/*
 * LSDParamsConfig.h
 *
 *  Created on: 16 Oct 2014
 *      Author: thomas
 */

#ifndef LSDPARAMSCONFIG_H_
#define LSDPARAMSCONFIG_H_

namespace lsd_slam_core
{
    class LSDParamsConfig
    {
        public:
            LSDParamsConfig()
             : minUseGrad(5),
               cameraPixelNoise(4),
               KFUsageWeight(4),
               KFDistWeight(3),
               doSLAM(true),
               doKFReActivation(true),
               doMapping(true),
               useFabMap(false),
               allowNegativeIdepths(true),
               useSubpixelStereo(true),
               useAffineLightningEstimation(false),
               multiThreading(true),
               maxLoopClosureCandidates(10),
               loopclosureStrictness(1.5),
               relocalizationTH(0.7),
               depthSmoothingFactor(1)
            {}

            double minUseGrad;
            double cameraPixelNoise;
            double KFUsageWeight;
            double KFDistWeight;
            bool doSLAM;
            bool doKFReActivation;
            bool doMapping;
            bool useFabMap;
            bool allowNegativeIdepths;
            bool useSubpixelStereo;
            bool useAffineLightningEstimation;
            bool multiThreading;
            int maxLoopClosureCandidates;
            bool loopclosureStrictness;
            double relocalizationTH;
            double depthSmoothingFactor;
    };
}

#endif /* LSDPARAMSCONFIG_H_ */
