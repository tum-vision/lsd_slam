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

#include "DataStructures/Frame.h"
#include "DataStructures/FrameMemory.h"
#include "DepthEstimation/DepthMapPixelHypothesis.h"
#include "Tracking/TrackingReference.h"

namespace lsd_slam
{

int privateFrameAllocCount = 0;





Frame::Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image)
{
	initialize(id, width, height, K, timestamp);
	
	data.image[0] = FrameMemory::getInstance().getFloatBuffer(data.width[0]*data.height[0]);
	float* maxPt = data.image[0] + data.width[0]*data.height[0];

	for(float* pt = data.image[0]; pt < maxPt; pt++)
	{
		*pt = *image;
		image++;
	}

	data.imageValid[0] = true;

	privateFrameAllocCount++;

	if(enablePrintDebugInfo && printMemoryDebugInfo)
		printf("ALLOCATED frame %d, now there are %d\n", this->id(), privateFrameAllocCount);
}

Frame::Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const float* image)
{
	initialize(id, width, height, K, timestamp);
	
	data.image[0] = FrameMemory::getInstance().getFloatBuffer(data.width[0]*data.height[0]);
	memcpy(data.image[0], image, data.width[0]*data.height[0] * sizeof(float));
	data.imageValid[0] = true;

	privateFrameAllocCount++;

	if(enablePrintDebugInfo && printMemoryDebugInfo)
		printf("ALLOCATED frame %d, now there are %d\n", this->id(), privateFrameAllocCount);
}

Frame::~Frame()
{

	if(enablePrintDebugInfo && printMemoryDebugInfo)
		printf("DELETING frame %d\n", this->id());

	FrameMemory::getInstance().deactivateFrame(this);

	if(!pose->isRegisteredToGraph)
		delete pose;
	else
		pose->frame = 0;

	for (int level = 0; level < PYRAMID_LEVELS; ++ level)
	{
		FrameMemory::getInstance().returnBuffer(data.image[level]);
		FrameMemory::getInstance().returnBuffer(reinterpret_cast<float*>(data.gradients[level]));
		FrameMemory::getInstance().returnBuffer(data.maxGradients[level]);
		FrameMemory::getInstance().returnBuffer(data.idepth[level]);
		FrameMemory::getInstance().returnBuffer(data.idepthVar[level]);
	}

	FrameMemory::getInstance().returnBuffer((float*)data.validity_reAct);
	FrameMemory::getInstance().returnBuffer(data.idepth_reAct);
	FrameMemory::getInstance().returnBuffer(data.idepthVar_reAct);

	if(permaRef_colorAndVarData != 0)
		delete permaRef_colorAndVarData;
	if(permaRef_posData != 0)
		delete permaRef_posData;

	privateFrameAllocCount--;
	if(enablePrintDebugInfo && printMemoryDebugInfo)
		printf("DELETED frame %d, now there are %d\n", this->id(), privateFrameAllocCount);
}


void Frame::takeReActivationData(DepthMapPixelHypothesis* depthMap)
{
	boost::shared_lock<boost::shared_mutex> lock = getActiveLock();

	if(data.validity_reAct == 0)
		data.validity_reAct = (unsigned char*) FrameMemory::getInstance().getBuffer(data.width[0]*data.height[0]);

	if(data.idepth_reAct == 0)
		data.idepth_reAct = FrameMemory::getInstance().getFloatBuffer((data.width[0]*data.height[0]));

	if(data.idepthVar_reAct == 0)
		data.idepthVar_reAct = FrameMemory::getInstance().getFloatBuffer((data.width[0]*data.height[0]));


	float* id_pt = data.idepth_reAct;
	float* id_pt_max = data.idepth_reAct + (data.width[0]*data.height[0]);
	float* idv_pt = data.idepthVar_reAct;
	unsigned char* val_pt = data.validity_reAct;

	for (; id_pt < id_pt_max; ++ id_pt, ++ idv_pt, ++ val_pt, ++depthMap)
	{
		if(depthMap->isValid)
		{
			*id_pt = depthMap->idepth;
			*idv_pt = depthMap->idepth_var;
			*val_pt = depthMap->validity_counter;
		}
		else if(depthMap->blacklisted < MIN_BLACKLIST)
		{
			*idv_pt = -2;
		}
		else
		{
			*idv_pt = -1;
		}
	}

	data.reActivationDataValid = true;
}



void Frame::setPermaRef(TrackingReference* reference)
{
	assert(reference->frameID == id());
	reference->makePointCloud(QUICK_KF_CHECK_LVL);

	permaRef_mutex.lock();

	if(permaRef_colorAndVarData != 0)
		delete permaRef_colorAndVarData;
	if(permaRef_posData != 0)
		delete permaRef_posData;

	permaRefNumPts = reference->numData[QUICK_KF_CHECK_LVL];
	permaRef_colorAndVarData = new Eigen::Vector2f[permaRefNumPts];
	permaRef_posData = new Eigen::Vector3f[permaRefNumPts];

	memcpy(permaRef_colorAndVarData,
			reference->colorAndVarData[QUICK_KF_CHECK_LVL],
			sizeof(Eigen::Vector2f) * permaRefNumPts);

	memcpy(permaRef_posData,
			reference->posData[QUICK_KF_CHECK_LVL],
			sizeof(Eigen::Vector3f) * permaRefNumPts);

	permaRef_mutex.unlock();
}

void Frame::calculateMeanInformation()
{
	return;

	if(numMappablePixels < 0)
		maxGradients(0);

	const float* idv = idepthVar(0);
	const float* idv_max = idv + width(0)*height(0);
	float sum = 0; int goodpx = 0;
	for(const float* pt=idv; pt < idv_max; pt++)
	{
		if(*pt > 0)
		{
			sum += sqrtf(1.0f / *pt);
			goodpx++;
		}
	}

	meanInformation = sum / goodpx;
}


void Frame::setDepth(const DepthMapPixelHypothesis* newDepth)
{

	boost::shared_lock<boost::shared_mutex> lock = getActiveLock();
	boost::unique_lock<boost::mutex> lock2(buildMutex);

	if(data.idepth[0] == 0)
		data.idepth[0] = FrameMemory::getInstance().getFloatBuffer(data.width[0]*data.height[0]);
	if(data.idepthVar[0] == 0)
		data.idepthVar[0] = FrameMemory::getInstance().getFloatBuffer(data.width[0]*data.height[0]);

	float* pyrIDepth = data.idepth[0];
	float* pyrIDepthVar = data.idepthVar[0];
	float* pyrIDepthMax = pyrIDepth + (data.width[0]*data.height[0]);
	
	float sumIdepth=0;
	int numIdepth=0;

	for (; pyrIDepth < pyrIDepthMax; ++ pyrIDepth, ++ pyrIDepthVar, ++ newDepth) //, ++ pyrRefID)
	{
		if (newDepth->isValid && newDepth->idepth_smoothed >= -0.05)
		{
			*pyrIDepth = newDepth->idepth_smoothed;
			*pyrIDepthVar = newDepth->idepth_var_smoothed;

			numIdepth++;
			sumIdepth += newDepth->idepth_smoothed;
		}
		else
		{
			*pyrIDepth = -1;
			*pyrIDepthVar = -1;
		}
	}
	
	meanIdepth = sumIdepth / numIdepth;
	numPoints = numIdepth;


	data.idepthValid[0] = true;
	data.idepthVarValid[0] = true;
	release(IDEPTH | IDEPTH_VAR, true, true);
	data.hasIDepthBeenSet = true;
	depthHasBeenUpdatedFlag = true;
}

void Frame::setDepthFromGroundTruth(const float* depth, float cov_scale)
{
	boost::shared_lock<boost::shared_mutex> lock = getActiveLock();
	const float* pyrMaxGradient = maxGradients(0);



	boost::unique_lock<boost::mutex> lock2(buildMutex);
	if(data.idepth[0] == 0)
		data.idepth[0] = FrameMemory::getInstance().getFloatBuffer(data.width[0]*data.height[0]);
	if(data.idepthVar[0] == 0)
		data.idepthVar[0] = FrameMemory::getInstance().getFloatBuffer(data.width[0]*data.height[0]);

	float* pyrIDepth = data.idepth[0];
	float* pyrIDepthVar = data.idepthVar[0];

	int width0 = data.width[0];
	int height0 = data.height[0];

	for(int y=0;y<height0;y++)
	{
		for(int x=0;x<width0;x++)
		{
			if (x > 0 && x < width0-1 && y > 0 && y < height0-1 && // pyramidMaxGradient is not valid for the border
					pyrMaxGradient[x+y*width0] >= MIN_ABS_GRAD_CREATE &&
					!isnanf(*depth) && *depth > 0)
			{
				*pyrIDepth = 1.0f / *depth;
				*pyrIDepthVar = VAR_GT_INIT_INITIAL * cov_scale;
			}
			else
			{
				*pyrIDepth = -1;
				*pyrIDepthVar = -1;
			}

			++ depth;
			++ pyrIDepth;
			++ pyrIDepthVar;
		}
	}
	
	data.idepthValid[0] = true;
	data.idepthVarValid[0] = true;
// 	data.refIDValid[0] = true;
	// Invalidate higher levels, they need to be updated with the new data
	release(IDEPTH | IDEPTH_VAR, true, true);
	data.hasIDepthBeenSet = true;
}

void Frame::prepareForStereoWith(Frame* other, Sim3 thisToOther, const Eigen::Matrix3f& K, const int level)
{
	Sim3 otherToThis = thisToOther.inverse();

	//otherToThis = data.worldToCam * other->data.camToWorld;
	K_otherToThis_R = K * otherToThis.rotationMatrix().cast<float>() * otherToThis.scale();
	otherToThis_t = otherToThis.translation().cast<float>();
	K_otherToThis_t = K * otherToThis_t;



	thisToOther_t = thisToOther.translation().cast<float>();
	K_thisToOther_t = K * thisToOther_t;
	thisToOther_R = thisToOther.rotationMatrix().cast<float>() * thisToOther.scale();
	otherToThis_R_row0 = thisToOther_R.col(0);
	otherToThis_R_row1 = thisToOther_R.col(1);
	otherToThis_R_row2 = thisToOther_R.col(2);

	distSquared = otherToThis.translation().dot(otherToThis.translation());

	referenceID = other->id();
	referenceLevel = level;
}

void Frame::require(int dataFlags, int level)
{
	if ((dataFlags & IMAGE) && ! data.imageValid[level])
	{
		buildImage(level);
	}
	if ((dataFlags & GRADIENTS) && ! data.gradientsValid[level])
	{
		buildGradients(level);
	}
	if ((dataFlags & MAX_GRADIENTS) && ! data.maxGradientsValid[level])
	{
		buildMaxGradients(level);
	}
	if (((dataFlags & IDEPTH) && ! data.idepthValid[level])
		|| ((dataFlags & IDEPTH_VAR) && ! data.idepthVarValid[level]))
	{
		buildIDepthAndIDepthVar(level);
	}
}

void Frame::release(int dataFlags, bool pyramidsOnly, bool invalidateOnly)
{
	for (int level = (pyramidsOnly ? 1 : 0); level < PYRAMID_LEVELS; ++ level)
	{
		if ((dataFlags & IMAGE) && data.imageValid[level])
		{
			data.imageValid[level] = false;
			if(!invalidateOnly)
				releaseImage(level);
		}
		if ((dataFlags & GRADIENTS) && data.gradientsValid[level])
		{
			data.gradientsValid[level] = false;
			if(!invalidateOnly)
				releaseGradients(level);
		}
		if ((dataFlags & MAX_GRADIENTS) && data.maxGradientsValid[level])
		{
			data.maxGradientsValid[level] = false;
			if(!invalidateOnly)
				releaseMaxGradients(level);
		}
		if ((dataFlags & IDEPTH) && data.idepthValid[level])
		{
			data.idepthValid[level] = false;
			if(!invalidateOnly)
				releaseIDepth(level);
		}
		if ((dataFlags & IDEPTH_VAR) && data.idepthVarValid[level])
		{
			data.idepthVarValid[level] = false;
			if(!invalidateOnly)
				releaseIDepthVar(level);
		}
	}
}

bool Frame::minimizeInMemory()
{
	if(activeMutex.timed_lock(boost::posix_time::milliseconds(10)))
	{
		buildMutex.lock();
		if(enablePrintDebugInfo && printMemoryDebugInfo)
			printf("minimizing frame %d\n",id());

		release(IMAGE | IDEPTH | IDEPTH_VAR, true, false);
		release(GRADIENTS | MAX_GRADIENTS, false, false);

		clear_refPixelWasGood();

		buildMutex.unlock();
		activeMutex.unlock();
		return true;
	}
	return false;
}

void Frame::initialize(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp)
{
	data.id = id;
	
	pose = new FramePoseStruct(this);

	data.K[0] = K;
	data.fx[0] = K(0,0);
	data.fy[0] = K(1,1);
	data.cx[0] = K(0,2);
	data.cy[0] = K(1,2);
	
	data.KInv[0] = K.inverse();
	data.fxInv[0] = data.KInv[0](0,0);
	data.fyInv[0] = data.KInv[0](1,1);
	data.cxInv[0] = data.KInv[0](0,2);
	data.cyInv[0] = data.KInv[0](1,2);
	
	data.timestamp = timestamp;

	data.hasIDepthBeenSet = false;
	depthHasBeenUpdatedFlag = false;
	
	referenceID = -1;
	referenceLevel = -1;
	
	numMappablePixels = -1;

	for (int level = 0; level < PYRAMID_LEVELS; ++ level)
	{
		data.width[level] = width >> level;
		data.height[level] = height >> level;

		data.imageValid[level] = false;
		data.gradientsValid[level] = false;
		data.maxGradientsValid[level] = false;
		data.idepthValid[level] = false;
		data.idepthVarValid[level] = false;

		data.image[level] = 0;
		data.gradients[level] = 0;
		data.maxGradients[level] = 0;
		data.idepth[level] = 0;
		data.idepthVar[level] = 0;
		data.reActivationDataValid = false;

// 		data.refIDValid[level] = false;
		
		if (level > 0)
		{
			data.fx[level] = data.fx[level-1] * 0.5;
			data.fy[level] = data.fy[level-1] * 0.5;
			data.cx[level] = (data.cx[0] + 0.5) / ((int)1<<level) - 0.5;
			data.cy[level] = (data.cy[0] + 0.5) / ((int)1<<level) - 0.5;

			data.K[level]  << data.fx[level], 0.0, data.cx[level], 0.0, data.fy[level], data.cy[level], 0.0, 0.0, 1.0;	// synthetic
			data.KInv[level] = (data.K[level]).inverse();

			data.fxInv[level] = data.KInv[level](0,0);
			data.fyInv[level] = data.KInv[level](1,1);
			data.cxInv[level] = data.KInv[level](0,2);
			data.cyInv[level] = data.KInv[level](1,2);
		}
	}

	data.validity_reAct = 0;
	data.idepthVar_reAct = 0;
	data.idepth_reAct = 0;

	data.refPixelWasGood = 0;

	permaRefNumPts = 0;
	permaRef_colorAndVarData = 0;
	permaRef_posData = 0;

	meanIdepth = 1;
	numPoints = 0;

	numFramesTrackedOnThis = numMappedOnThis = numMappedOnThisTotal = 0;

	idxInKeyframes = -1;

	edgeErrorSum = edgesNum = 1;

	lastConstraintTrackedCamToWorld = Sim3();

	isActive = false;
}

void Frame::setDepth_Allocate()
{
	return;
}

void Frame::buildImage(int level)
{
	if (level == 0)
	{
		printf("Frame::buildImage(0): Loading image from disk is not implemented yet! No-op.\n");
		return;
	}
	
	require(IMAGE, level - 1);
	boost::unique_lock<boost::mutex> lock2(buildMutex);

	if(data.imageValid[level])
		return;

	if(enablePrintDebugInfo && printFrameBuildDebugInfo)
		printf("CREATE Image lvl %d for frame %d\n", level, id());

	int width = data.width[level - 1];
	int height = data.height[level - 1];
	const float* source = data.image[level - 1];

	if (data.image[level] == 0)
		data.image[level] = FrameMemory::getInstance().getFloatBuffer(data.width[level] * data.height[level]);
	float* dest = data.image[level];

#if defined(ENABLE_SSE)
	// I assume all all subsampled width's are a multiple of 8.
	// if this is not the case, this still works except for the last * pixel, which will produce a segfault.
	// in that case, reduce this loop and calculate the last 0-3 dest pixels by hand....
	if (width % 8 == 0)
	{
		__m128 p025 = _mm_setr_ps(0.25f,0.25f,0.25f,0.25f);

		const float* maxY = source+width*height;
		for(const float* y = source; y < maxY; y+=width*2)
		{
			const float* maxX = y+width;
			for(const float* x=y; x < maxX; x += 8)
			{
				// i am calculating four dest pixels at a time.

				__m128 top_left = _mm_load_ps((float*)x);
				__m128 bot_left = _mm_load_ps((float*)x+width);
				__m128 left = _mm_add_ps(top_left,bot_left);

				__m128 top_right = _mm_load_ps((float*)x+4);
				__m128 bot_right = _mm_load_ps((float*)x+width+4);
				__m128 right = _mm_add_ps(top_right,bot_right);

				__m128 sumA = _mm_shuffle_ps(left,right, _MM_SHUFFLE(2,0,2,0));
				__m128 sumB = _mm_shuffle_ps(left,right, _MM_SHUFFLE(3,1,3,1));

				__m128 sum = _mm_add_ps(sumA,sumB);
				sum = _mm_mul_ps(sum,p025);

				_mm_store_ps(dest, sum);
				dest += 4;
			}
		}

		data.imageValid[level] = true;
		return;
	}
#elif defined(ENABLE_NEON)
	// I assume all all subsampled width's are a multiple of 8.
	// if this is not the case, this still works except for the last * pixel, which will produce a segfault.
	// in that case, reduce this loop and calculate the last 0-3 dest pixels by hand....
	if (width % 8 == 0)
	{
		static const float p025[] = {0.25, 0.25, 0.25, 0.25};
		int width_iteration_count = width / 8;
		int height_iteration_count = height / 2;
		const float* cur_px = source;
		const float* next_row_px = source + width;
		
		__asm__ __volatile__
		(
			"vldmia %[p025], {q10}                        \n\t" // p025(q10)
			
			".height_loop:                                \n\t"
			
				"mov r5, %[width_iteration_count]             \n\t" // store width_iteration_count
				".width_loop:                                 \n\t"
				
					"vldmia   %[cur_px]!, {q0-q1}             \n\t" // top_left(q0), top_right(q1)
					"vldmia   %[next_row_px]!, {q2-q3}        \n\t" // bottom_left(q2), bottom_right(q3)
		
					"vadd.f32 q0, q0, q2                      \n\t" // left(q0)
					"vadd.f32 q1, q1, q3                      \n\t" // right(q1)
		
					"vpadd.f32 d0, d0, d1                     \n\t" // pairwise add into sum(q0)
					"vpadd.f32 d1, d2, d3                     \n\t"
					"vmul.f32 q0, q0, q10                     \n\t" // multiply with 0.25 to get average
					
					"vstmia %[dest]!, {q0}                    \n\t"
				
				"subs     %[width_iteration_count], %[width_iteration_count], #1 \n\t"
				"bne      .width_loop                     \n\t"
				"mov      %[width_iteration_count], r5    \n\t" // restore width_iteration_count
				
				// Advance one more line
				"add      %[cur_px], %[cur_px], %[rowSize]    \n\t"
				"add      %[next_row_px], %[next_row_px], %[rowSize] \n\t"
			
			"subs     %[height_iteration_count], %[height_iteration_count], #1 \n\t"
			"bne      .height_loop                       \n\t"

			: /* outputs */ [cur_px]"+&r"(cur_px),
							[next_row_px]"+&r"(next_row_px),
							[width_iteration_count]"+&r"(width_iteration_count),
							[height_iteration_count]"+&r"(height_iteration_count),
							[dest]"+&r"(dest)
			: /* inputs  */ [p025]"r"(p025),
							[rowSize]"r"(width * sizeof(float))
			: /* clobber */ "memory", "cc", "r5",
							"q0", "q1", "q2", "q3", "q10"
		);

		data.imageValid[level] = true;
		return;
	}
#endif

	int wh = width*height;
	const float* s;
	for(int y=0;y<wh;y+=width*2)
	{
		for(int x=0;x<width;x+=2)
		{
			s = source + x + y;
			*dest = (s[0] +
					s[1] +
					s[width] +
					s[1+width]) * 0.25f;
			dest++;
		}
	}

	data.imageValid[level] = true;
}

void Frame::releaseImage(int level)
{
	if (level == 0)
	{
		printf("Frame::releaseImage(0): Storing image on disk is not supported yet! No-op.\n");
		return;
	}
	FrameMemory::getInstance().returnBuffer(data.image[level]);
	data.image[level] = 0;
}

void Frame::buildGradients(int level)
{
	require(IMAGE, level);
	boost::unique_lock<boost::mutex> lock2(buildMutex);

	if(data.gradientsValid[level])
		return;

	if(enablePrintDebugInfo && printFrameBuildDebugInfo)
		printf("CREATE Gradients lvl %d for frame %d\n", level, id());

	int width = data.width[level];
	int height = data.height[level];
	if(data.gradients[level] == 0)
		data.gradients[level] = (Eigen::Vector4f*)FrameMemory::getInstance().getBuffer(sizeof(Eigen::Vector4f) * width * height);
	const float* img_pt = data.image[level] + width;
	const float* img_pt_max = data.image[level] + width*(height-1);
	Eigen::Vector4f* gradxyii_pt = data.gradients[level] + width;
	
	// in each iteration i need -1,0,p1,mw,pw
	float val_m1 = *(img_pt-1);
	float val_00 = *img_pt;
	float val_p1;

	for(; img_pt < img_pt_max; img_pt++, gradxyii_pt++)
	{
		val_p1 = *(img_pt+1);

		*((float*)gradxyii_pt) = 0.5f*(val_p1 - val_m1);
		*(((float*)gradxyii_pt)+1) = 0.5f*(*(img_pt+width) - *(img_pt-width));
		*(((float*)gradxyii_pt)+2) = val_00;

		val_m1 = val_00;
		val_00 = val_p1;
	}

	data.gradientsValid[level] = true;
}

void Frame::releaseGradients(int level)
{
	FrameMemory::getInstance().returnBuffer(reinterpret_cast<float*>(data.gradients[level]));
	data.gradients[level] = 0;
}



void Frame::buildMaxGradients(int level)
{
	require(GRADIENTS, level);
	boost::unique_lock<boost::mutex> lock2(buildMutex);

	if(data.maxGradientsValid[level]) return;

	if(enablePrintDebugInfo && printFrameBuildDebugInfo)
		printf("CREATE AbsGrad lvl %d for frame %d\n", level, id());

	int width = data.width[level];
	int height = data.height[level];
	if (data.maxGradients[level] == 0)
		data.maxGradients[level] = FrameMemory::getInstance().getFloatBuffer(width * height);
	
	float* maxGradTemp = FrameMemory::getInstance().getFloatBuffer(width * height);


	// 1. write abs gradients in real data.
	Eigen::Vector4f* gradxyii_pt = data.gradients[level] + width;
	float* maxgrad_pt = data.maxGradients[level] + width;
	float* maxgrad_pt_max = data.maxGradients[level] + width*(height-1);

	for(; maxgrad_pt < maxgrad_pt_max; maxgrad_pt++, gradxyii_pt++)
	{
		float dx = *((float*)gradxyii_pt);
		float dy = *(1+(float*)gradxyii_pt);
		*maxgrad_pt = sqrtf(dx*dx + dy*dy);
	}

	// 2. smear up/down direction into temp buffer
	maxgrad_pt = data.maxGradients[level] + width+1;
	maxgrad_pt_max = data.maxGradients[level] + width*(height-1)-1;
	float* maxgrad_t_pt = maxGradTemp + width+1;
	for(;maxgrad_pt<maxgrad_pt_max; maxgrad_pt++, maxgrad_t_pt++)
	{
		float g1 = maxgrad_pt[-width];
		float g2 = maxgrad_pt[0];
		if(g1 < g2) g1 = g2;
		float g3 = maxgrad_pt[width];
		if(g1 < g3)
			*maxgrad_t_pt = g3;
		else
			*maxgrad_t_pt = g1;
	}

	float numMappablePixels = 0;
	// 2. smear left/right direction into real data
	maxgrad_pt = data.maxGradients[level] + width+1;
	maxgrad_pt_max = data.maxGradients[level] + width*(height-1)-1;
	maxgrad_t_pt = maxGradTemp + width+1;
	for(;maxgrad_pt<maxgrad_pt_max; maxgrad_pt++, maxgrad_t_pt++)
	{
		float g1 = maxgrad_t_pt[-1];
		float g2 = maxgrad_t_pt[0];
		if(g1 < g2) g1 = g2;
		float g3 = maxgrad_t_pt[1];
		if(g1 < g3)
		{
			*maxgrad_pt = g3;
			if(g3 >= MIN_ABS_GRAD_CREATE)
				numMappablePixels++;
		}
		else
		{
			*maxgrad_pt = g1;
			if(g1 >= MIN_ABS_GRAD_CREATE)
				numMappablePixels++;
		}
	}

	if(level==0)
		this->numMappablePixels = numMappablePixels;

	FrameMemory::getInstance().returnBuffer(maxGradTemp);

	data.maxGradientsValid[level] = true;
}

void Frame::releaseMaxGradients(int level)
{
	FrameMemory::getInstance().returnBuffer(data.maxGradients[level]);
	data.maxGradients[level] = 0;
}

void Frame::buildIDepthAndIDepthVar(int level)
{
	if (! data.hasIDepthBeenSet)
	{
		printfAssert("Frame::buildIDepthAndIDepthVar(): idepth has not been set yet!\n");
		return;
	}
	if (level == 0)
	{
		printf("Frame::buildIDepthAndIDepthVar(0): Loading depth from disk is not implemented yet! No-op.\n");
		return;
	}

	require(IDEPTH, level - 1);
	boost::unique_lock<boost::mutex> lock2(buildMutex);
	
	if(data.idepthValid[level] && data.idepthVarValid[level])
		return;

	if(enablePrintDebugInfo && printFrameBuildDebugInfo)
		printf("CREATE IDepth lvl %d for frame %d\n", level, id());
	
	int width = data.width[level];
	int height = data.height[level];
	
	if (data.idepth[level] == 0)
		data.idepth[level] = FrameMemory::getInstance().getFloatBuffer(width * height);
	if (data.idepthVar[level] == 0)
		data.idepthVar[level] = FrameMemory::getInstance().getFloatBuffer(width * height);

	int sw = data.width[level - 1];

	const float* idepthSource = data.idepth[level - 1];
	const float* idepthVarSource = data.idepthVar[level - 1];
	float* idepthDest = data.idepth[level];
	float* idepthVarDest = data.idepthVar[level];
	
	for(int y=0;y<height;y++)
	{
		for(int x=0;x<width;x++)
		{
			int idx = 2*(x+y*sw);
			int idxDest = (x+y*width);

			float idepthSumsSum = 0;
			float ivarSumsSum = 0;
			int num=0;

			// build sums
			float ivar;
			float var = idepthVarSource[idx];
			if(var > 0)
			{
				ivar = 1.0f / var;
				ivarSumsSum += ivar;
				idepthSumsSum += ivar * idepthSource[idx];
				num++;
			}

			var = idepthVarSource[idx+1];
			if(var > 0)
			{
				ivar = 1.0f / var;
				ivarSumsSum += ivar;
				idepthSumsSum += ivar * idepthSource[idx+1];
				num++;
			}

			var = idepthVarSource[idx+sw];
			if(var > 0)
			{
				ivar = 1.0f / var;
				ivarSumsSum += ivar;
				idepthSumsSum += ivar * idepthSource[idx+sw];
				num++;
			}

			var = idepthVarSource[idx+sw+1];
			if(var > 0)
			{
				ivar = 1.0f / var;
				ivarSumsSum += ivar;
				idepthSumsSum += ivar * idepthSource[idx+sw+1];
				num++;
			}
			
			if(num > 0)
			{
				float depth = ivarSumsSum / idepthSumsSum;
				idepthDest[idxDest] = 1.0f / depth;
				idepthVarDest[idxDest] = num / ivarSumsSum;
			}
			else
			{
				idepthDest[idxDest] = -1;
				idepthVarDest[idxDest] = -1;
			}
		}
	}

	data.idepthValid[level] = true;
	data.idepthVarValid[level] = true;
}

void Frame::releaseIDepth(int level)
{
	if (level == 0)
	{
		printf("Frame::releaseIDepth(0): Storing depth on disk is not supported yet! No-op.\n");
		return;
	}
	
	FrameMemory::getInstance().returnBuffer(data.idepth[level]);
	data.idepth[level] = 0;
}


void Frame::releaseIDepthVar(int level)
{
	if (level == 0)
	{
		printf("Frame::releaseIDepthVar(0): Storing depth variance on disk is not supported yet! No-op.\n");
		return;
	}
	FrameMemory::getInstance().returnBuffer(data.idepthVar[level]);
	data.idepthVar[level] = 0;
}

void Frame::printfAssert(const char* message) const
{
	assert(!message);
	printf("%s\n", message);
}
}
