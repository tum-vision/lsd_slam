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

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/Parse.h"
#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "IOWrapper/Pangolin/PangolinOutput3DWrapper.h"
#include "SlamSystem.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "util/Undistorter.h"
#include "util/RawLogReader.h"

#include "opencv2/opencv.hpp"

#include "GUI.h"

std::vector<std::string> files;
int w, h, w_inp, h_inp;
ThreadMutexObject<bool> lsdDone(false);
GUI gui;
RawLogReader * logReader = 0;
int numFrames = 0;

std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}
std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}
std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}
int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string name = std::string(dirp->d_name);

    	if(name != "." && name != "..")
    		files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}

int getFile (std::string source, std::vector<std::string> &files)
{
	std::ifstream f(source.c_str());

	if(f.good() && f.is_open())
	{
		while(!f.eof())
		{
			std::string l;
			std::getline(f,l);

			l = trim(l);

			if(l == "" || l[0] == '#')
				continue;

			files.push_back(l);
		}

		f.close();

		size_t sp = source.find_last_of('/');
		std::string prefix;
		if(sp == std::string::npos)
			prefix = "";
		else
			prefix = source.substr(0,sp);

		for(unsigned int i=0;i<files.size();i++)
		{
			if(files[i].at(0) != '/')
				files[i] = prefix + "/" + files[i];
		}

		return (int)files.size();
	}
	else
	{
		f.close();
		return -1;
	}

}

using namespace lsd_slam;

void run(SlamSystem * system, Undistorter* undistorter, Output3DWrapper* outputWrapper, Sophus::Matrix3f K)
{
    // get HZ
    double hz = 30;

    cv::Mat image = cv::Mat(h, w, CV_8U);
    int runningIDX=0;
    float fakeTimeStamp = 0;

    for(unsigned int i = 0; i < numFrames; i++)
    {
        if(lsdDone.getValue())
            break;

        cv::Mat imageDist = cv::Mat(h, w, CV_8U);

        if(logReader)
        {
            logReader->getNext();

            cv::Mat3b img(h, w, (cv::Vec3b *)logReader->rgb);

            cv::cvtColor(img, imageDist, CV_RGB2GRAY);
        }
        else
        {
            imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);

            if(imageDist.rows != h_inp || imageDist.cols != w_inp)
            {
                if(imageDist.rows * imageDist.cols == 0)
                    printf("failed to load image %s! skipping.\n", files[i].c_str());
                else
                    printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
                            files[i].c_str(),
                            w,h,imageDist.cols, imageDist.rows);
                continue;
            }
        }

        assert(imageDist.type() == CV_8U);

        undistorter->undistort(imageDist, image);

        assert(image.type() == CV_8U);

        if(runningIDX == 0)
        {
            system->randomInit(image.data, fakeTimeStamp, runningIDX);
        }
        else
        {
            system->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp);
        }

        gui.pose.assignValue(system->getCurrentPoseEstimateScale());

        runningIDX++;
        fakeTimeStamp+=0.03;

        if(fullResetRequested)
        {
            printf("FULL RESET!\n");
            delete system;

            system = new SlamSystem(w, h, K, doSlam);
            system->setVisualization(outputWrapper);

            fullResetRequested = false;
            runningIDX = 0;
        }
    }

    lsdDone.assignValue(true);
}

int main( int argc, char** argv )
{
	// get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
	std::string calibFile;
	Undistorter* undistorter = 0;

	if(Parse::arg(argc, argv, "-c", calibFile) > 0)
	{
		 undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
	}

	if(undistorter == 0)
	{
		printf("need camera calibration file! (set using -c FILE)\n");
		exit(0);
	}

	w = undistorter->getOutputWidth();
	h = undistorter->getOutputHeight();

	w_inp = undistorter->getInputWidth();
	h_inp = undistorter->getInputHeight();

	float fx = undistorter->getK().at<double>(0, 0);
	float fy = undistorter->getK().at<double>(1, 1);
	float cx = undistorter->getK().at<double>(2, 0);
	float cy = undistorter->getK().at<double>(2, 1);
	Sophus::Matrix3f K;
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	Resolution::getInstance(w, h);
	Intrinsics::getInstance(fx, fy, cx, cy);

	gui.initImages();

	Output3DWrapper* outputWrapper = new PangolinOutput3DWrapper(w, h, gui);

	// make slam system
	SlamSystem * system = new SlamSystem(w, h, K, doSlam);
	system->setVisualization(outputWrapper);


	// open image files: first try to open as file.
	std::string source;
	if(!(Parse::arg(argc, argv, "-f", source) > 0))
	{
		printf("need source files! (set using -f FOLDER or KLG)\n");
		exit(0);
	}

	Bytef * decompressionBuffer = new Bytef[Resolution::getInstance().numPixels() * 2];
    IplImage * deCompImage = 0;

    if(source.substr(source.find_last_of(".") + 1) == "klg")
    {
        logReader = new RawLogReader(decompressionBuffer,
                                     deCompImage,
                                     source);

        numFrames = logReader->getNumFrames();
    }
    else
    {
        if(getdir(source, files) >= 0)
        {
            printf("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
        }
        else if(getFile(source, files) >= 0)
        {
            printf("found %d image files in file %s!\n", (int)files.size(), source.c_str());
        }
        else
        {
            printf("could not load file list! wrong path / file?\n");
        }

        numFrames = (int)files.size();
    }

	boost::thread lsdThread(run, system, undistorter, outputWrapper, K);

	while(!pangolin::ShouldQuit())
	{
	    if(lsdDone.getValue() && !system->finalized)
	    {
	        system->finalize();
	    }

	    gui.preCall();

	    gui.drawKeyframes();

	    gui.drawFrustum();

	    gui.drawImages();

	    gui.postCall();
	}

	lsdDone.assignValue(true);

	lsdThread.join();

	delete system;
	delete undistorter;
	delete outputWrapper;
	return 0;
}
