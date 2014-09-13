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

#ifdef HAVE_FABMAP
#include "GlobalMapping/FabMap.h"

#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "openfabmap.hpp"

#include "util/settings.h"
#include "DataStructures/Frame.h"

namespace lsd_slam
{


FabMap::FabMap()
{
	valid = false;
	
	std::string fabmapTrainDataPath = packagePath + "thirdparty/openFabMap/trainingdata/StLuciaShortTraindata.yml";
	std::string vocabPath = packagePath + "thirdparty/openFabMap/trainingdata/StLuciaShortVocabulary.yml";
	std::string chowliutreePath = packagePath + "thirdparty/openFabMap/trainingdata/StLuciaShortTree.yml";
	
	// Load training data
	cv::FileStorage fsTraining;
	fsTraining.open(fabmapTrainDataPath, cv::FileStorage::READ);
	cv::Mat fabmapTrainData;
	fsTraining["BOWImageDescs"] >> fabmapTrainData;
	if (fabmapTrainData.empty()) {
		std::cerr << fabmapTrainDataPath << ": FabMap Training Data not found" << std::endl;
		return;
	}
	fsTraining.release();
	
	// Load vocabulary
	cv::FileStorage fsVocabulary;
	fsVocabulary.open(vocabPath, cv::FileStorage::READ);
	cv::Mat vocabulary;
	fsVocabulary["Vocabulary"] >> vocabulary;
	if (vocabulary.empty()) {
		std::cerr << vocabPath << ": Vocabulary not found" << std::endl;
		return;
	}
	fsVocabulary.release();

	//load a chow-liu tree
	cv::FileStorage fsTree;
	fsTree.open(chowliutreePath, cv::FileStorage::READ);
	cv::Mat clTree;
	fsTree["ChowLiuTree"] >> clTree;
	if (clTree.empty()) {
		std::cerr << chowliutreePath << ": Chow-Liu tree not found" << std::endl;
		return;
	}
	fsTree.release();
	
	// Generate openFabMap object (FabMap2 - needs training bow data!)
	int options = 0;
	options |= of2::FabMap::SAMPLED;
	options |= of2::FabMap::CHOW_LIU;
	fabMap = new of2::FabMap2(clTree, 0.39, 0, options);
	//add the training data for use with the sampling method
	fabMap->addTraining(fabmapTrainData);
	
// 	// Generate openFabMap object (FabMap1 with look up table)
// 	int options = 0;
// 	options |= of2::FabMap::MEAN_FIELD;
// 	options |= of2::FabMap::CHOW_LIU;
// 	//fabMap = new of2::FabMap(clTree, 0.39, 0, options);
// 	fabMap = new of2::FabMapLUT(clTree, 0.39, 0, options, 3000, 6);
// 	//fabMap = new of2::FabMapFBO(clTree, 0.39, 0, options, 3000, 1e-6, 1e-6, 512, 9);
	
	// Create detector & extractor
	detector = new cv::StarFeatureDetector(32, 10, 18, 18, 20);
	cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SURF(1000, 4, 2, false, true); // new cv::SIFT();
	
	//use a FLANN matcher to generate bag-of-words representations
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce"
	bide = new cv::BOWImgDescriptorExtractor(extractor, matcher);
	bide->setVocabulary(vocabulary);
	
	printConfusionMatrix = false;
	confusionMat = cv::Mat(0, 0, CV_32F);
	
	nextImageID = 0;
	valid = true;
}

FabMap::~FabMap()
{
	if (printConfusionMatrix)
	{
		std::ofstream writer((packagePath + "fabMapResult.txt").c_str());
		for(int i = 0; i < confusionMat.rows; i++) {
			for(int j = 0; j < confusionMat.cols; j++) {
				writer << confusionMat.at<float>(i, j) << " ";
			}
			writer << std::endl;
		}
		writer.close();
	}
}



void FabMap::compareAndAdd(Frame* keyframe, int* out_newID, int* out_loopID)
{
	// Convert keyframe image data to 3-channel OpenCV Mat (theoretically unneccessary)
	cv::Mat frame;
	cv::Mat keyFrameImage(keyframe->height(), keyframe->width(), CV_32F, const_cast<float*>(keyframe->image()));
	keyFrameImage.convertTo(frame, CV_8UC1);
	//cv::cvtColor(frame, frame, CV_GRAY2RGB);
	
	// Generate FabMap bag-of-words data (image descriptor)
	cv::Mat bow;
	std::vector<cv::KeyPoint> kpts;
	detector->detect(frame, kpts);
	if (kpts.empty())
	{
		*out_newID = -1;
		*out_loopID = -1;
		return;
	}
	bide->compute(frame, kpts, bow);
	
	// Run FabMap
	std::vector<of2::IMatch> matches;
	if (nextImageID > 0)
		fabMap->compare(bow, matches);
	fabMap->add(bow);
	*out_newID = nextImageID;
	++nextImageID;
	
	if (printConfusionMatrix)
	{
		cv::Mat resizedMat(nextImageID, nextImageID, confusionMat.type(), cv::Scalar(0));
		if (confusionMat.rows > 0)
			confusionMat.copyTo(resizedMat(cv::Rect(cv::Point(0, 0), confusionMat.size())));
		confusionMat = resizedMat;
		
		for(auto l = matches.begin(); l != matches.end(); ++ l)
		{
			int col = (l->imgIdx < 0) ? (nextImageID-1) : l->imgIdx;
			confusionMat.at<float>(nextImageID-1, col) = l->match;
		}
	}
	
	const float minLoopProbability = 0.8f;
	float accumulatedProbability = 0;
	const bool debugProbabilites = false;
	if (debugProbabilites)
		printf("FabMap probabilities:");
	for(auto l = matches.begin(); l != matches.end(); ++ l)
	{
		if (debugProbabilites)
			printf(" (%i: %f)", l->imgIdx, l->match);
		
		if(l->imgIdx < 0)
		{
			// Probability for new place
			accumulatedProbability += l->match;
		}
		else
		{
			// Probability for existing place
			if (l->match >= minLoopProbability)
			{
				*out_loopID = l->imgIdx;
				if (debugProbabilites)
					printf("\n");
				return;
			}
			accumulatedProbability += l->match;
		}
		
		if (! debugProbabilites && accumulatedProbability > 1 - minLoopProbability)
			break; // not possible anymore to find a frame with high enough probability
	}
	if (debugProbabilites)
		printf("\n");
	
	*out_loopID = -1;
	return;
}

bool FabMap::isValid() const
{
	return valid;
}

}

#endif
