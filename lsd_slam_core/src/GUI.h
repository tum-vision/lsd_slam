/*
 * GUI.h
 *
 *  Created on: 15 Aug 2014
 *      Author: thomas
 */

#ifndef GUI_H_
#define GUI_H_

#define GLM_FORCE_RADIANS

#include <pangolin/pangolin.h>
#include <pangolin/gl.h>
#include <pangolin/gldraw.h>
#include <map>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "util/Resolution.h"
#include "util/Intrinsics.h"
#include "IOWrapper/Pangolin/Keyframe.h"
#include "util/ThreadMutexObject.h"
#include "DataStructures/Frame.h"

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

class GUI
{
    public:
        GUI();

        virtual ~GUI();

        void initImages();

        void preCall();

        void drawFrustum();

        void postCall();

        void addKeyframe(Keyframe * newFrame);

        void updateImage(unsigned char * data);

        void updateKeyframePoses(GraphFramePose* framePoseData, int num);

        void drawKeyframes();

        void drawImages();

        ThreadMutexObject<Sophus::Sim3f> pose;

    private:
        void drawGrid();

        pangolin::GlTexture * depthImg;

        ThreadMutexObject<unsigned char * > depthImgBuffer;

        pangolin::Var<int> * gpuMem;

        pangolin::Var<std::string> * totalPoints;

        pangolin::OpenGlRenderState s_cam;

        ThreadMutexObject<std::map<int, Keyframe *> > keyframes;
};


#endif /* GUI_H_ */
