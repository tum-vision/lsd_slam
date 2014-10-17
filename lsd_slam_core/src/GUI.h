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

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

class GUI
{
    public:
        GUI();

        virtual ~GUI();

        void preCall();

        void drawFrustum(const glm::mat4 & pose);

        void postCall();

        void addKeyframe(Keyframe * newFrame);

        void drawKeyframes();

    private:
        void drawGrid();

        pangolin::Var<bool> * pause,
                            * step;
        pangolin::Var<int> * gpuMem;

        pangolin::Var<std::string> * totalPoints;

        pangolin::OpenGlRenderState s_cam;

        ThreadMutexObject<std::map<int, Keyframe *> > keyframes;
};


#endif /* GUI_H_ */
