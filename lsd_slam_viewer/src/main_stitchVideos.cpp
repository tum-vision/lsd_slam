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
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/


#include "ros/ros.h"

#include "opencv2/opencv.hpp"

#include <dirent.h>
#include <vector>
#include <stdio.h>
#include <algorithm>
using std::string;


std::vector<string> getFileList(string s, string filter)
{
	printf("getting files for %s!\n", s.c_str());
	DIR *dpdf;
	struct dirent *epdf;

	std::vector<string> v;
	dpdf = opendir(s.c_str());
	if (dpdf != NULL){
	   while ((epdf = readdir(dpdf)))
		   if(epdf->d_name[0] != '.' && (filter=="" || string(epdf->d_name).find(filter) != string::npos))
			   v.push_back(s+epdf->d_name);
	}

	return v;
}


int dilate( int argc, char** argv )
{
	/* DILATE */
	char bufer[1000];

	snprintf(bufer,1000,"%s.png",argv[1]);


	cv::Mat i = cv::imread(bufer);
	cv::Mat i2 = cv::imread(bufer);

	printf("loaded.\n");
	for(int x=0;x<i.cols;x++)
		for(int y=0;y<i.rows;y++)
		{
			int px = x + y*i.cols;

			if(x==0 ||y==0 || x == i.cols-1 || y == i.rows-1)
			{
				i2.data[3*px+0] = i2.data[3*px+1] = i2.data[3*px+2] = 255;
				continue;
			}



			if(i.data[3*px+0] != 255 || i.data[3*px+1] != 255 || i.data[3*px+2] != 255)
				continue;

			int num = 0;
			int sum = 0;
			for(int ax = -1; ax <=1; ax++)
				for(int ay = -1; ay <=1; ay++)
				{
					int apx =  (ax + x) + (y+ay)*i.cols;
					if(i.data[3*apx+0] == 51 || i.data[3*apx+0] != i.data[3*apx+1] || i.data[3*apx+0] != i.data[3*apx+2])
						continue;

					num++;
					sum += i.data[3*apx+0];
				}

			if(num > 0)
				i2.data[3*px+0] = i2.data[3*px+1] = i2.data[3*px+2] = sum / num;
			else
				i2.data[3*px+0] = i2.data[3*px+1] = i2.data[3*px+2] = 255;
		}


printf("done\n");

	snprintf(bufer,1000,"%s_dilated.png",argv[1]);
	cv::imwrite(bufer,i2);
	return 0;
}

void rgb2bgr(cv::Mat* image)
{
	cv::Mat imageOut = image->clone();

	for(int x=0;x<image->cols;x++)
		for(int y=0;y<image->rows;y++)
			image->at<cv::Vec3b>(y,x) = cv::Vec3b(image->at<cv::Vec3b>(y,x)[2], image->at<cv::Vec3b>(y,x)[1], image->at<cv::Vec3b>(y,x)[0]);
}


int invertColor( int argc, char** argv )
{

	cv::Mat image = cv::imread(argv[1]);

	cv::imshow("image", image); cv::waitKey(0);

	rgb2bgr(&image);

	cv::imshow("image", image); cv::waitKey(0);
	cv::imwrite(argv[1],image);

	return 0;
}

void dilateColorImage(cv::Mat* image, int lowTH = 0, int highTH = 0)
{
	cv::Mat imageOut = image->clone();

	for(int x=1;x<image->cols-1;x++)
		for(int y=1;y<image->rows-1;y++)
	{
		// if color: skip.
		cv::Vec3b c = image->at<cv::Vec3b>(y,x);
		int cVal = (c[0]-c[1])*(c[0]-c[1]) + (c[0]-c[2])*(c[0]-c[2]) + (c[2]-c[1])*(c[2]-c[1]);

		if(cVal > lowTH)
			continue;


		// else get av of colored neighbours
		float r = 0, g = 0, b=0;
		int num = 0;
		for(int dx = -1; dx <= 1; dx++)
			for(int dy = -1; dy <= 1; dy++)
			{
				cv::Vec3b c2 = image->at<cv::Vec3b>(y+dy,x+dx);
				int c2Val = (c2[0]-c2[1])*(c2[0]-c2[1]) + (c2[0]-c2[2])*(c2[0]-c2[2]) + (c2[2]-c2[1])*(c2[2]-c2[1]);
				if(c2Val < highTH)
					continue;

				r += c2[0];
				g += c2[1];
				b += c2[2];

				num++;
			}

		if(num > 4)
			imageOut.at<cv::Vec3b>(y,x) = cv::Vec3b(r/num, g/num, b/num);
	}

	imageOut.copyTo(*image);
}


int inlayVid3( int argc, char** argv )
{
	std::vector<string> pointcloudFiles =  getFileList("save_pointcloud/","");
	std::vector<string> odometryFilesIdepth =  getFileList("save_odometry/","idepth");
	std::vector<string> odometryFilesVar =  getFileList("save_odometry/","var");
	std::vector<string> odometryFilesOrg =  getFileList("save_odometry/","org");
	std::vector<string> odometryFilesAge =  getFileList("save_odometry/","age");

	std::sort(odometryFilesIdepth.begin(), odometryFilesIdepth.end());
	std::sort(odometryFilesVar.begin(), odometryFilesVar.end());
	std::sort(odometryFilesOrg.begin(), odometryFilesOrg.end());
	std::sort(odometryFilesAge.begin(), odometryFilesAge.end());



	printf("pcFiles: %d, odometry: idepth: %d, var: %d, org: %d, age: %d\n",
			pointcloudFiles.size(),
			odometryFilesIdepth.size(),
			odometryFilesVar.size(),
			odometryFilesOrg.size(),
			odometryFilesAge.size());


	assert(odometryFilesIdepth.size() == odometryFilesVar.size());
	assert(odometryFilesIdepth.size() == odometryFilesOrg.size());
	assert(odometryFilesIdepth.size() == odometryFilesAge.size());


	// get odometry timestamps
	std::vector<long> odometryTimestamps;
	for(size_t i=0; i<odometryFilesIdepth.size(); ++i)
	{
		long id = -1;

		size_t pos = odometryFilesIdepth[i].find_last_of("/");
		string tmpString = odometryFilesIdepth[i];
		if(pos != string::npos)
			tmpString = odometryFilesIdepth[i].substr(pos+1);


		if(1 != sscanf(tmpString.c_str(),"idepth-%ld.png",&id))
			printf("could not parse name as odometry %s!\n", tmpString.c_str());
		odometryTimestamps.push_back(id);
	}

	// get pc timestamps
	std::vector<long> pointcloudTimestamps;
	std::vector<long> pointcloudIDs;
	for(size_t i=0; i<pointcloudFiles.size(); ++i)
	{
		long id = -1, time = -1;

		size_t pos = pointcloudFiles[i].find_last_of("/");
		string tmpString = pointcloudFiles[i];
		if(pos != string::npos)
			tmpString = pointcloudFiles[i].substr(pos+1);


		if(2 != sscanf(tmpString.c_str(),"%ld-%ld.jpg",&id,&time))
			printf("could not parse name as pc %s!\n", tmpString.c_str());
		pointcloudTimestamps.push_back(time);
		pointcloudIDs.push_back(id);
	}


	// sort pc's (brutally slow, but hey)
	for(size_t i=0; i<pointcloudIDs.size(); ++i)
		for(size_t j=i+1; j<pointcloudIDs.size(); ++j)
			if(pointcloudIDs[i] > pointcloudIDs[j])
			{
				long tmp = pointcloudTimestamps[i];
				pointcloudTimestamps[i] = pointcloudTimestamps[j];
				pointcloudTimestamps[j] = tmp;

				tmp = pointcloudIDs[i];
				pointcloudIDs[i] = pointcloudIDs[j];
				pointcloudIDs[j] = tmp;

				string tmp2 = pointcloudFiles[i];
				pointcloudFiles[i] = pointcloudFiles[j];
				pointcloudFiles[j] = tmp2;
			}




	if(!system("rm -rf save_output_age"))
		printf("system call failed!");
	if(!system("rm -rf save_output_var"))
		printf("system call failed!");
	if(!system("rm -rf save_output_idepth"))
		printf("system call failed!");


	if(!system("mkdir save_output_age"))
		printf("system call failed!");
	if(!system("mkdir save_output_var"))
		printf("system call failed!");
	if(!system("mkdir save_output_idepth"))
		printf("system call failed!");



	char buf[1000];
	int c = 0;
	float currentShift = 0;
	for(size_t i=0; i<pointcloudFiles.size(); ++i)
	{
		// skip if invalid
		if(pointcloudTimestamps[i] == -1)
		{
			printf("skipping pc file %s!\n",pointcloudFiles[i].c_str());
			continue;
		}

		// find closest
		long mindiff = pointcloudTimestamps[i];
		int mindiffIDX = 0;
		for(size_t j=0; j<odometryTimestamps.size(); ++j)
			if(pointcloudTimestamps[i] - odometryTimestamps[j] < mindiff &&
					- pointcloudTimestamps[i] + odometryTimestamps[j] < mindiff)
			{
				mindiff = pointcloudTimestamps[i] - odometryTimestamps[j];
				if(mindiff < 0) mindiff = -mindiff;
				mindiffIDX = j;
			}

		if(mindiff > 10)
			printf("pairing %ld with %ld (diff is: %ld)\n",pointcloudTimestamps[i],odometryTimestamps[mindiffIDX],mindiff);


		// load all
		cv::Mat p = cv::imread(pointcloudFiles[i]);
		cv::Mat oIdepth = cv::imread(odometryFilesIdepth[mindiffIDX]);
		cv::Mat oVar = cv::imread(odometryFilesVar[mindiffIDX]);
		cv::Mat oOrg = cv::imread(odometryFilesOrg[mindiffIDX]);
		cv::Mat oAge = cv::imread(odometryFilesAge[mindiffIDX]);

		dilateColorImage(&oVar);
		dilateColorImage(&oIdepth);
		dilateColorImage(&oAge);
		//dilatePCImage(&p, 500000, 500);
		rgb2bgr(&oIdepth);



		// rescale odometries
		int osheight2 = 350;
		int oswidth2 = 476;
		cv::Mat p_roi_top2 = p(cv::Rect(7,7,oswidth2,osheight2));
		cv::Mat p_roi_bot2 = p(cv::Rect(7,7+8+osheight2,oswidth2,osheight2));



		//* rescale for 3 images
		int osheight = 233;
		int oswidth = 311;
		cv::Mat p_roi_top = p(cv::Rect(5,5,oswidth,osheight));
		cv::Mat p_roi_mid = p(cv::Rect(5,11+osheight,oswidth,osheight));
		cv::Mat p_roi_bot = p(cv::Rect(5,16+2*osheight,oswidth,osheight));


		cv::Mat osIdepth = cv::Mat(cv::Size(oswidth,osheight),CV_8UC3);
		cv::resize(oIdepth,osIdepth,cv::Size(oswidth,osheight));

		cv::Mat osVar = cv::Mat(cv::Size(oswidth,osheight),CV_8UC3);
		cv::resize(oVar,osVar,cv::Size(oswidth,osheight));

		cv::Mat osOrg = cv::Mat(cv::Size(oswidth,osheight),CV_8UC3);
		cv::resize(oOrg,osOrg,cv::Size(oswidth,osheight));

		cv::Mat osAge = cv::Mat(cv::Size(oswidth,osheight),CV_8UC3);
		cv::resize(oAge,osAge,cv::Size(oswidth,osheight));


		osOrg.copyTo(p_roi_top);

		osIdepth.copyTo(p_roi_mid);

		snprintf(buf,1000,"save_output_idepth/img%03d.png",c);
		cv::imwrite(buf,p);

		osVar.copyTo(p_roi_bot);
		snprintf(buf,1000,"save_output_var/img%03d.png",c);
		cv::imwrite(buf,p);


		osAge.copyTo(p_roi_bot);
		snprintf(buf,1000,"save_output_age/img%03d.png",c);
		cv::imwrite(buf,p);


		cv::imshow("out",p);
		cv::waitKey(10);
		c++;
	}

	return 0;
}

std::vector<long> getTimestamps(std::vector<string> names, string prefix)
{
	std::vector<long> ts;

	for(size_t i=0; i<names.size(); ++i)
	{
		long id = -1;
		long tss = -1;

		size_t pos = names[i].find_last_of("/");
		string tmpString = names[i];
		if(pos != string::npos)
			tmpString = names[i].substr(pos+1);

		char buf[100];
		if(2 != sscanf(tmpString.c_str(),(prefix+"%ld-%ld.png").c_str(),&id, &tss))
			printf("ERROR: could not parse name as odometry %s!\n", tmpString.c_str());
		ts.push_back(tss);
	}

	return ts;
}

int inlayVidNew1( int argc, char** argv )
{
	printf("inlayVidNew12\n");
	std::vector<string> pointcloudFiles =  getFileList("/home/engelj/fuerte_workspace/bags/videoData/pc-slam8/","");
	std::vector<string> odometryFilesMapped =  getFileList("/home/engelj/fuerte_workspace/bags/videoData/odometry-slam8-02/","mapped-");
	std::vector<string> odometryFilesTracked =  getFileList("/home/engelj/fuerte_workspace/bags/videoData/odometry-slam8-02/","tracked-");

	std::string target = "/home/engelj/fuerte_workspace/bags/videoData/stitched-slam8";

	printf("pcFiles: %d, mapped: %d, tracked: %d\n",
			pointcloudFiles.size(),
			odometryFilesMapped.size(),
			odometryFilesTracked.size());

	std::sort(pointcloudFiles.begin(), pointcloudFiles.end());
	std::sort(odometryFilesMapped.begin(), odometryFilesMapped.end());
	std::sort(odometryFilesTracked.begin(), odometryFilesTracked.end());

	std::vector<long> pcTimes = getTimestamps(pointcloudFiles, "");
	std::vector<long> mappedTimes = getTimestamps(odometryFilesMapped,"mapped");
	std::vector<long> trackedTimes = getTimestamps(odometryFilesTracked,"tracked");


	if(!system(("rm -rf "+target).c_str()))
		printf("system call failed!");
	if(!system(("mkdir "+target).c_str()))
		printf("system call failed!");


	// put the best fitting frame into each pc file
	int fixedMappingOffset = 200;

	for(int i=0;i<pointcloudFiles.size();i++)
	{

		// find corresp. images
		long pcTs = pcTimes[i];
		int mappedIdx=0, trackedIdx=0;
		int preDiff = 10000000;
		for(int j=0;j<mappedTimes.size();j++)
			if((mappedTimes[j] - (pcTs - fixedMappingOffset))*(mappedTimes[j] - (pcTs - fixedMappingOffset)) < preDiff)
			{
				mappedIdx = j;
				preDiff = (mappedTimes[j] - (pcTs - fixedMappingOffset))*(mappedTimes[j] - (pcTs - fixedMappingOffset));
			}

		preDiff = 10000;
		for(int j=0;j<trackedTimes.size();j++)
			if(trackedTimes[j] <= pcTs &&
					(trackedTimes[j] - pcTs)*(trackedTimes[j] - pcTs) < preDiff)
			{
				trackedIdx = j;
				preDiff = (trackedTimes[j] - pcTs)*(trackedTimes[j] - pcTs);
			}

		// load corresp images.
		cv::Mat p = cv::imread(pointcloudFiles[i]);
		cv::Mat mapped = cv::imread(odometryFilesMapped[mappedIdx]);
		cv::Mat tracked = cv::imread(odometryFilesTracked[trackedIdx]);

		dilateColorImage(&mapped,5,5);

		int osheight = 350;
		int oswidth = 476;

		cv::Mat smapped = cv::Mat(cv::Size(oswidth,osheight),CV_8UC3);
		cv::resize(mapped,smapped,cv::Size(oswidth,osheight));

		cv::Mat stracked = cv::Mat(cv::Size(oswidth,osheight),CV_8UC3);
		cv::resize(tracked,stracked,cv::Size(oswidth,osheight));

		cv::Mat p_roi_top = p(cv::Rect(7,7,oswidth,osheight));
		cv::Mat p_roi_bot = p(cv::Rect(7,7+8+osheight,oswidth,osheight));

		char buf[1000];
		stracked.copyTo(p_roi_top);
		smapped.copyTo(p_roi_bot);

		snprintf(buf,1000,"%s/img%05d.png",target.c_str(),i);
		cv::imwrite(buf,p);

		cv::imshow("vid",p);
		cv::waitKey(10);
	}
	return 0;
}



int inlayVid( int argc, char** argv )
{
	std::vector<string> pointcloudFiles =  getFileList("save_pointcloud/","");
	std::vector<string> odometryFilesAge =  getFileList("save_odometry/","age");
	std::vector<string> odometryFilesIdepth =  getFileList("save_odometry/","idepth");
	std::vector<string> odometryFilesStereo =  getFileList("save_odometry/","stereo");
	std::vector<string> odometryFilesVar =  getFileList("save_odometry/","var");

	std::sort(odometryFilesAge.begin(), odometryFilesAge.end());
	std::sort(odometryFilesIdepth.begin(), odometryFilesIdepth.end());
	std::sort(odometryFilesStereo.begin(), odometryFilesStereo.end());
	std::sort(odometryFilesVar.begin(), odometryFilesVar.end());



	printf("pcFiles: %d, odometry: age: %d, idepth: %d, stereo: %d, var: %d\n",
			pointcloudFiles.size(),
			odometryFilesAge.size(),
			odometryFilesIdepth.size(),
			odometryFilesStereo.size(),
			odometryFilesVar.size());


	assert(odometryFilesAge.size() == odometryFilesIdepth.size());
	assert(odometryFilesAge.size() == odometryFilesStereo.size());
	assert(odometryFilesAge.size() == odometryFilesVar.size());


	// get odometry timestamps
	std::vector<long> odometryTimestamps;
	for(int i=0; i<odometryFilesAge.size(); i++)
	{
		long id = -1;

		size_t pos = odometryFilesAge[i].find_last_of("/");
		string tmpString = odometryFilesAge[i];
		if(pos != string::npos)
			tmpString = odometryFilesAge[i].substr(pos+1);


		if(1 != sscanf(tmpString.c_str(),"age-%ld.png",&id))
			printf("could not parse name as odometry %s!\n", tmpString.c_str());
		odometryTimestamps.push_back(id);
	}

	// get pc timestamps
	std::vector<long> pointcloudTimestamps;
	std::vector<long> pointcloudIDs;
	for(int i=0; i<pointcloudFiles.size(); i++)
	{
		long id = -1, time = -1;

		size_t pos = pointcloudFiles[i].find_last_of("/");
		string tmpString = pointcloudFiles[i];
		if(pos != string::npos)
			tmpString = pointcloudFiles[i].substr(pos+1);


		if(2 != sscanf(tmpString.c_str(),"%ld-%ld.jpg",&id,&time))
			printf("could not parse name as pc %s!\n", tmpString.c_str());
		pointcloudTimestamps.push_back(time);
		pointcloudIDs.push_back(id);
	}


	// sort pc's (brutally slow, but hey)
	for(int i=0;i<pointcloudIDs.size();i++)
		for(int j=i+1;j<pointcloudIDs.size();j++)
			if(pointcloudIDs[i] > pointcloudIDs[j])
			{
				long tmp = pointcloudTimestamps[i];
				pointcloudTimestamps[i] = pointcloudTimestamps[j];
				pointcloudTimestamps[j] = tmp;

				tmp = pointcloudIDs[i];
				pointcloudIDs[i] = pointcloudIDs[j];
				pointcloudIDs[j] = tmp;

				string tmp2 = pointcloudFiles[i];
				pointcloudFiles[i] = pointcloudFiles[j];
				pointcloudFiles[j] = tmp2;
			}



	if(!system("rm -rf save_output_age"))
		printf("system call failed!");
	if(!system("rm -rf save_output_idepth"))
		printf("system call failed!");
	if(!system("rm -rf save_output_stereo"))
		printf("system call failed!");
	if(!system("rm -rf save_output_var"))
		printf("system call failed!");

	if(!system("mkdir save_output_age"))
		printf("system call failed!");
	if(!system("mkdir save_output_idepth"))
		printf("system call failed!");
	if(!system("mkdir save_output_stereo"))
		printf("system call failed!");
	if(!system("mkdir save_output_var"))
		printf("system call failed!");


	char buf[1000];
	int c = 0;
	float currentShift = 0;
	for(int i=0;i<pointcloudFiles.size();i++)
	{
		// skip if invalid
		if(pointcloudTimestamps[i] == -1)
		{
			printf("skipping pc file %s!\n",pointcloudFiles[i].c_str());
			continue;
		}

		// find closest
		long mindiff = pointcloudTimestamps[i];
		int mindiffIDX = 0;
		for(int j=0;j<odometryTimestamps.size();j++)
			if(pointcloudTimestamps[i] - odometryTimestamps[j] < mindiff &&
					- pointcloudTimestamps[i] + odometryTimestamps[j] < mindiff)
			{
				mindiff = pointcloudTimestamps[i] - odometryTimestamps[j];
				if(mindiff < 0) mindiff = -mindiff;
				mindiffIDX = j;
			}

		if(mindiff > 10)
			printf("pairing %ld with %ld (diff is: %ld)\n",pointcloudTimestamps[i],odometryTimestamps[mindiffIDX],mindiff);


		// load all
		cv::Mat p = cv::imread(pointcloudFiles[i]);
		cv::Mat oAge = cv::imread(odometryFilesAge[mindiffIDX]);
		cv::Mat oIdepth = cv::imread(odometryFilesIdepth[mindiffIDX]);
		cv::Mat oVar = cv::imread(odometryFilesVar[mindiffIDX]);
		cv::Mat oStereo = cv::imread(odometryFilesStereo[mindiffIDX]);


		// change stereo color to light gray
		for(int px=0;px<oStereo.cols*oStereo.rows;px++)
		{
			if(oStereo.data[3*px+0] == oStereo.data[3*px+1] && oStereo.data[3*px+0] == oStereo.data[3*px+2])
				oStereo.data[3*px+0] = oStereo.data[3*px+1] = oStereo.data[3*px+2] = 200;
		}



		/* rescale for 2 images
		// rescale odometries
		int osheight = 350;
		int oswidth = 476;
		*/


		//* rescale for 3 images
		int osheight = 233;
		int oswidth = 311;


		cv::Mat osAge = cv::Mat(cv::Size(oswidth,osheight),CV_8UC3);
		cv::resize(oAge,osAge,cv::Size(oswidth,osheight));

		cv::Mat osIdepth = cv::Mat(cv::Size(oswidth,osheight),CV_8UC3);
		cv::resize(oIdepth,osIdepth,cv::Size(oswidth,osheight));

		cv::Mat osVar = cv::Mat(cv::Size(oswidth,osheight),CV_8UC3);
		cv::resize(oVar,osVar,cv::Size(oswidth,osheight));

		cv::Mat osStereo = cv::Mat(cv::Size(oswidth,osheight),CV_8UC3);
		cv::resize(oStereo,osStereo,cv::Size(oswidth,osheight));


		/*/ shift for sn

		if(pointcloudIDs[i] > 462 && pointcloudIDs[i] <= 544)
			currentShift += 350.0f / (544-462);

		if(pointcloudIDs[i] > 566 && pointcloudIDs[i] <= 660)
			currentShift += -180.0f / (660-566);

		if(pointcloudIDs[i] > 777 && pointcloudIDs[i] <= 837)
			currentShift += -170.0f / (837-777);

		if(pointcloudIDs[i] > 1350 && pointcloudIDs[i] <= 1370)
			currentShift += 50.0f / (1370-1350);


		if(currentShift < 0) currentShift = 0;
		int si = currentShift;
		printf("shift: %d\n",si);
		for(int y=0;y<p.rows;y++)
		{
			for(int x=p.cols-1;x>=0;x--)
				p.at<cv::Vec3b>(y,x) = p.at<cv::Vec3b>(y,x-si);
			for(int x=0;x<si;x++)
				p.at<cv::Vec3b>(y,x) = cv::Vec3b(51,51,51);
		}

		*/



		/* rois and save for 2 images
		// get roi's for 2 images
		cv::Mat p_roi_top = p(cv::Rect(7,7,oswidth,osheight));
		cv::Mat p_roi_bot = p(cv::Rect(7,7+8+osheight,oswidth,osheight));
		// save idepth
		osIdepth.copyTo(p_roi_top);
		snprintf(buf,1000,"save_output_idepth/img%03d.png",c);
		cv::imwrite(buf,p);

		osVar.copyTo(p_roi_bot);
		snprintf(buf,1000,"save_output_var/img%03d.png",c);
		cv::imwrite(buf,p);

		osAge.copyTo(p_roi_bot);
		snprintf(buf,1000,"save_output_age/img%03d.png",c);
		cv::imwrite(buf,p);

		osStereo.copyTo(p_roi_bot);
		snprintf(buf,1000,"save_output_stereo/img%03d.png",c);
		cv::imwrite(buf,p);

		*/



		//* rois and save for 3 images
		cv::Mat p_roi_top = p(cv::Rect(5,5,oswidth,osheight));
		cv::Mat p_roi_mid = p(cv::Rect(5,11+osheight,oswidth,osheight));
		cv::Mat p_roi_bot = p(cv::Rect(5,16+2*osheight,oswidth,osheight));


		// save idepth
		osIdepth.copyTo(p_roi_top);
		osVar.copyTo(p_roi_mid);
		osStereo.copyTo(p_roi_bot);

		snprintf(buf,1000,"save_output_idepth/img%03d.png",c);
		cv::imwrite(buf,p);





		cv::imshow("out",p);
		cv::waitKey(10);
		c++;
	}

	return 0;
}

int inlayVid2( int argc, char** argv )
{
	std::vector<string> imgFiles =  getFileList("save_cam2/","img");
	std::vector<string> idepthFiles =  getFileList("save_cam2/","idepth");

	std::sort(imgFiles.begin(), imgFiles.end());
	std::sort(idepthFiles.begin(), idepthFiles.end());

	assert(imgFiles.size() == idepthFiles.size());
	if(!system("rm -rf save_cam2_out"))
		printf("system call failed!");
	if(!system("mkdir save_cam2_out"))
		printf("system call failed!");

	cv::Mat plot = cv::imread("plot.png");
	/*
	float plot0Px = 141;
	float plotEndPx = 1657;
	double plotTime = 64;						// time between plot0Px and plotEndPx
	double plotTimeOffset = 1364295759.4671;	// offset of plot-0s in seconds
	*/

	/*
	int leftBarStart = 0, leftBarMid = 23, leftBarRight = 40;
	int rightBarStart = 1782, rightBarMid = 1800, rightBarRight = 1811;
	*/




	char buf[1000];
	for(int i=0;i<imgFiles.size();i++)
	{
		cv::Mat idepth = cv::imread(idepthFiles[i]);
		cv::Mat img = cv::imread(imgFiles[i]);

		cv::Mat imgR = cv::Mat(cv::Size(img.cols/3,img.rows/3),CV_8UC3);
		cv::resize(img,imgR,cv::Size(img.cols/3,img.rows/3));


		cv::Mat roi = idepth(cv::Rect(4,4,img.cols/3,img.rows/3));
		imgR.copyTo(roi);


		/*
		int plotWidth = 213;
		int plotHeight = 160;
		cv::Mat roiPlot = idepth(cv::Rect(4,4+4+img.rows/3,plotWidth,plotHeight));
		// make brighter
		for(int x=0; x < plotWidth; x++)
			for(int y=0;y<plotHeight;y++)
			{
				roiPlot.at<cv::Vec3b>(y,x)[0] = (100.0 / 255.0) * roiPlot.at<cv::Vec3b>(y,x)[0] + 155;
				roiPlot.at<cv::Vec3b>(y,x)[1] = (100.0 / 255.0) * roiPlot.at<cv::Vec3b>(y,x)[1] + 155;
				roiPlot.at<cv::Vec3b>(y,x)[2] = (100.0 / 255.0) * roiPlot.at<cv::Vec3b>(y,x)[2] + 155;
			}


		// copy middle
		double tstamp = 0;
		sscanf(idepthFiles[i].c_str(),"save_cam2/idepth-%lf.png", &tstamp);
		tstamp *= 0.001;

		if(tstamp == 0)
			printf("skipping file %s\n!",idepthFiles[i].c_str());
		else
		{

			int middleOffset = plot0Px + (plotEndPx - plot0Px) * (tstamp - plotTimeOffset) / plotTime;

			if(middleOffset < 0 || middleOffset > plot.cols - plotWidth)
				printf("midOffset out of bounds: %d\n!",middleOffset);
			else
			{
				for(int x=leftBarMid; x < plotWidth - rightBarRight + rightBarStart; x++)
					for(int y=0;y<plotHeight;y++)
					{
						if(plot.at<cv::Vec3b>(y,x+middleOffset)[0] == 255 &&
								plot.at<cv::Vec3b>(y,x+middleOffset)[1]== 255 &&
								plot.at<cv::Vec3b>(y,x+middleOffset)[2] == 255)
							continue;

						roiPlot.at<cv::Vec3b>(y,x) = plot.at<cv::Vec3b>(y,x+middleOffset);

					}
			}
		}

		// copy left
		for(int x=leftBarStart; x < leftBarRight; x++)
			for(int y=0;y<plotHeight;y++)
				if(plot.at<cv::Vec3b>(y,x)[0] != 255 ||
						plot.at<cv::Vec3b>(y,x)[1] != 255 ||
						plot.at<cv::Vec3b>(y,x)[2] != 255)
					roiPlot.at<cv::Vec3b>(y,x) = plot.at<cv::Vec3b>(y,x);


		// copy right
		for(int x=rightBarStart; x < rightBarRight; x++)
			for(int y=0;y<plotHeight;y++)
				if(plot.at<cv::Vec3b>(y,x)[0] != 255 ||
						plot.at<cv::Vec3b>(y,x)[1] != 255 ||
						plot.at<cv::Vec3b>(y,x)[2] != 255)
					roiPlot.at<cv::Vec3b>(y,plotWidth + (x - rightBarStart) - (rightBarRight - rightBarStart)) = plot.at<cv::Vec3b>(y,x);


*/

		imshow("out",idepth);
		cv::waitKey(10);


		snprintf(buf,1000,"save_cam2_out/img%03d.png",i);
		cv::imwrite(buf,idepth);
	}

	return 0;
}


int makeBW( int argc, char** argv )
{
	uchar colors[255][3];
	float fac = 1;
	char k;
	cv::Mat image = cv::imread(argv[1],-1);
	cv::Mat imageOut = cv::Mat(image.rows, image.cols, CV_8UC3);

	printf("type depth1: %d\n", image.type());
	printf("type depth: %d\n", imageOut.type());


	while('s' != (k = cv::waitKey(0)))
	{
		if(k == 'p')
			fac *= 1.05;
		if(k == 'm')
			fac /= 1.05;


		for(int y=0;y<255;y++)
		{
			float id = fac * (y / 100.0);

			float r = (0-id) * 255 / 1.0; if(r < 0) r = -r;
			float g = (1-id) * 255 / 1.0; if(g < 0) g = -g;
			float b = (2-id) * 255 / 1.0; if(b < 0) b = -b;

			uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
			uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
			uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);

			colors[y][0] = 255-bc;
			colors[y][1] = 255-gc;
			colors[y][2] = 255-rc;
			//stripe.at<cv::Vec3b>(y,x) = cv::Vec3b(255-bc,255-gc,255-rc);
		}

		for(int x=0;x<image.cols;x++)
			for(int y=0;y<image.rows;y++)
			{
				cv::Vec3b col = image.at<cv::Vec3b>(y,x);

				int err = (col[0] - col[1])*(col[0] - col[1]) + (col[0] - col[2])*(col[0] - col[2]) + (col[2] - col[1])*(col[2] - col[1]);

				if(err < 50)
					imageOut.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);//image.at<cv::Vec3b>(y,x);
				else
				{
					// find closest and set
					int best=0, bestScore=999999999;
					for(int i=0;i<255;i++)
					{
						int score = (col[0] - colors[i][0])*(col[0] - colors[i][0]) +
								(col[1] - colors[i][1])*(col[1] - colors[i][1])+
								(col[2] - colors[i][2])*(col[2] - colors[i][2]);
						if(score < bestScore)
						{
							bestScore = score;
							best = i;
						}
					}
					imageOut.at<cv::Vec3b>(y,x) = cv::Vec3b(best,best,best);
				}
			}

		printf("fac: %f\n", fac);
		cv::imshow("out", imageOut);

	}

	std::string n = argv[1];
	n.replace(n.length()-4,std::string::npos,"_bw.png");

	printf("output: %s\n",n.c_str());

	cv::imshow("image", imageOut); cv::waitKey(0);
	cv::imwrite(n,imageOut);


	return 0;
}

int shiftRainbow( int argc, char** argv )
{
	uchar colors[500][3];
	float fac = 1;
	float off = 0;
	char k;
	cv::Mat image = cv::imread(argv[1],-1);
	cv::Mat imageOut = cv::Mat(image.rows, image.cols, CV_8UC3);

	printf("type depth1: %d\n", image.type());
	printf("type depth: %d\n", imageOut.type());


	while('s' != (k = cv::waitKey(0)))
	{
		if(k == 'p')
			fac *= 1.05;
		if(k == 'm')
			fac /= 1.05;

		if(k == 'o')
			off += 0.05;
		if(k == 'n')
			off -= 0.05;



		for(int y=0;y<500;y++)
		{
			float id = fac * (y / 200.0 + off);

			float r = (0-id) * 255 / 1.0; if(r < 0) r = -r;
			float g = (1-id) * 255 / 1.0; if(g < 0) g = -g;
			float b = (2-id) * 255 / 1.0; if(b < 0) b = -b;

			uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
			uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
			uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);

			colors[y][0] = 255-bc;
			colors[y][1] = 255-gc;
			colors[y][2] = 255-rc;
			//stripe.at<cv::Vec3b>(y,x) = cv::Vec3b(255-bc,255-gc,255-rc);
		}

		for(int x=0;x<image.cols;x++)
			for(int y=0;y<image.rows;y++)
			{
				cv::Vec3b col = image.at<cv::Vec3b>(y,x);

				int err = (col[0] - col[1])*(col[0] - col[1]) + (col[0] - col[2])*(col[0] - col[2]) + (col[2] - col[1])*(col[2] - col[1]);

				if(err < 50)
					imageOut.at<cv::Vec3b>(y,x) = image.at<cv::Vec3b>(y,x);
				else
				{
					// find closest and set
					int best=0, bestScore=999999999;
					for(int i=0;i<500;i++)
					{
						int score = (col[0] - colors[i][0])*(col[0] - colors[i][0]) +
								(col[1] - colors[i][1])*(col[1] - colors[i][1])+
								(col[2] - colors[i][2])*(col[2] - colors[i][2]);
						if(score < bestScore)
						{
							bestScore = score;
							best = i;
						}
					}

					float id = 1 * (best / 200.0 + 0);

					float r = (0-id) * 255 / 1.0; if(r < 0) r = -r;
					float g = (1-id) * 255 / 1.0; if(g < 0) g = -g;
					float b = (2-id) * 255 / 1.0; if(b < 0) b = -b;

					uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
					uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
					uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);

					imageOut.at<cv::Vec3b>(y,x) = cv::Vec3b(255-bc,255-gc,255-rc);
				}
			}

		printf("fac: %f, off: %f\n", fac,off);
		cv::imshow("out", imageOut);

	}

	std::string n = argv[1];
	n.replace(n.length()-4,std::string::npos,"_recolored.png");

	printf("output: %s\n",n.c_str());

	cv::imshow("image", imageOut); cv::waitKey(0);
	cv::imwrite(n,imageOut);


	return 0;
}



int makeRainbow( int argc, char** argv )
{
	cv::Mat depth1 = cv::imread(argv[1],0);
	cv::Mat image = cv::imread(argv[2],0);
	cv::Mat depth;
	depth1.convertTo(depth,CV_32FC1,1.0 / 6502.0);
	cv::Mat depthColor = cv::Mat(depth.rows, depth.cols, CV_8UC3);

	printf("type depth1: %d\n", depth1.type());
	printf("type depth: %d\n", depth.type());
	printf("type depthC: %d\n", depthColor.type());

	float fac = 1;
	char k;
	cv::imshow("depth", depth);
	while('s' != (k = cv::waitKey(0)))
	{
		if(k == 'p')
			fac *= 1.05;
		if(k == 'm')
			fac /= 1.05;


		for(int x=0;x<depth.cols;x++)
			for(int y=0;y<depth.rows;y++)
		{

			float d = 1.0 / depth.at<float>(y,x);


			if(d == 0)
				depthColor.at<cv::Vec3b>(y,x) = cv::Vec3b(image.at<uchar>(y,x),image.at<uchar>(y,x),image.at<uchar>(y,x));
			else
			{
				float id = fac / d;

				float r = (0-id) * 255 / 1.0; if(r < 0) r = -r;
				float g = (1-id) * 255 / 1.0; if(g < 0) g = -g;
				float b = (2-id) * 255 / 1.0; if(b < 0) b = -b;

				uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
				uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
				uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);

				depthColor.at<cv::Vec3b>(y,x) = cv::Vec3b(255-bc,255-gc,255-rc);
			}
		}

		printf("fac: %f\n", fac);
		cv::imshow("depth", depthColor);
	}

	cv::imwrite("colored.png",depthColor);

	return 0;
}


int dilateColor( int argc, char** argv )
{

	cv::Mat image = cv::imread(argv[1]);
	cv::Mat imageOut = cv::imread(argv[1]);

	cv::imshow("image", image); cv::waitKey(0);
	for(int x=1;x<image.cols-1;x++)
		for(int y=1;y<image.rows-1;y++)
	{
		// if color: skip.
		cv::Vec3b c = image.at<cv::Vec3b>(y,x);

		if(c[0] != c[1] || c[1] != c[2])
			continue;

		// else get av of colored neighbours
		float r = 0, g = 0, b=0;
		int num = 0;
		for(int dx = -1; dx <= 1; dx++)
			for(int dy = -1; dy <= 1; dy++)
			{
				cv::Vec3b c2 = image.at<cv::Vec3b>(y+dy,x+dx);
				if(c2[0] == c2[1] && c2[1] == c2[2])
					continue;
				r += c2[0];
				g += c2[1];
				b += c2[2];

				num++;
			}
		if(num > 0)
			imageOut.at<cv::Vec3b>(y,x) = cv::Vec3b(r/num, g/num, b/num);
	}

	std::string n = argv[1];
	n.replace(n.length()-4,std::string::npos,"_dilated.png");

	printf("output: %s\n",n.c_str());

	cv::imshow("image", imageOut); cv::waitKey(0);
	cv::imwrite(n,imageOut);

	return 0;
}

int makeRainbowStripe( int argc, char** argv )
{


	cv::Mat stripe = cv::Mat(480, 50, CV_8UC3);

	float fac = 1;
	char k;
	cv::imshow("stripe", stripe);
	while('s' != (k = cv::waitKey(0)))
	{

		if(k == 'p')
			fac *= 1.05;
		if(k == 'm')
			fac /= 1.05;


		for(int x=0;x<stripe.cols;x++)
			for(int y=0;y<stripe.rows;y++)
		{

				float id = fac * (y / 200.0);

				float r = (0-id) * 255 / 1.0; if(r < 0) r = -r;
				float g = (1-id) * 255 / 1.0; if(g < 0) g = -g;
				float b = (2-id) * 255 / 1.0; if(b < 0) b = -b;

				uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
				uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
				uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);

				stripe.at<cv::Vec3b>(y,x) = cv::Vec3b(255-bc,255-gc,255-rc);

		}


		cv::imshow("stripe", stripe); cv::waitKey(0);
	}


	cv::imwrite("stripe.png",stripe);
	return 0;
}


void downsample(std::string folder, int lvl)
{


	{
	std::vector<string> d =  getFileList(folder+"/rgb/","");

	int fac = (int)1<<lvl;
	int w = 640;
	int h = 480;

	for(size_t i=0; i<d.size(); ++i)
	{
		cv::Mat depth = cv::imread(d[i], CV_LOAD_IMAGE_UNCHANGED);
		cv::Mat depthDown = cv::Mat(depth.rows / fac, depth.cols / fac, depth.type());

		printf("color type %d, u16 %d\n", depth.type(), CV_8UC3);

	 	 for(int y=0;y<(int)h/fac;y++)
		 	for(int x=0;x<(int)w/fac;x++)
			 {

				 float sr=0, sg=0, sb=0, n=0;
				 for(int dx=0; dx<fac; ++dx)
					 for(int dy=0; dy<fac; ++dy)
					 {
						 cv::Vec3b val = depth.at<cv::Vec3b>(y*fac + dy, x*fac + dx);

						 sr += val[0];
						 sg += val[1];
						 sb += val[2];
						 ++n;

					 }
				 depthDown.at<cv::Vec3b>(y,x) = cv::Vec3b(sr/n, sg/n, sb/n);
			 }

	 	 cv::imwrite(d[i], depthDown);

	}
	}


	{
	std::vector<string> d =  getFileList(folder+"/depth/","");

	int fac = (int)1<<lvl;
	int w = 640;
	int h = 480;

	for(int i=0;i<d.size();i++)
	{
		cv::Mat depth = cv::imread(d[i], CV_LOAD_IMAGE_UNCHANGED);
		cv::Mat depthDown = cv::Mat(depth.rows / fac, depth.cols / fac, depth.type());

		printf("depth type %d, u16 %d\n", depth.type(), CV_16U);

	 	 for(int y=0;y<(int)h/fac;y++)
		 	for(int x=0;x<(int)w/fac;x++)
			 {

				 float sid=0, n=0;
				 for(int dx=0;dx<fac;dx++)
					 for(int dy=0;dy<fac;dy++)
					 {
						 float val = depth.at<ushort>(y*fac + dy, x*fac + dx);
						 if(val > 0 && !isnanf(val))
						 {
							 sid += 1.0f/val;
							 n ++;
						 }
					 }
				 depthDown.at<ushort>(y,x) = sid == 0 ? 0 : n/sid;
			 }

	 	 cv::imwrite(d[i], depthDown);

	}
	}

}



int main( int argc, char** argv )
{
//	invertColor(argc, argv);
//	downsample("/home/engelj/fuerte_workspace/mono_depth_odometry/datasets/rgbd_dataset_freiburg2_desk_l2", 2);
//	return inlayVidNew1(argc, argv);

//	return dilateColor(argc, argv);
//	return inlayVid(argc, argv);
//	return inlayVid2(argc, argv);

//	return makeRainbow(argc, argv);
//	return makeBW(argc, argv);

 return shiftRainbow(argc, argv);
// return inlayVid3(argc, argv);


	//return dilateColor(argc, argv);



//	return makeRainbowStripe(argc, argv);
}

