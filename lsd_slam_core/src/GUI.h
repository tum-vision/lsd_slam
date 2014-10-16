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

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

class GUI
{
    public:
        GUI()
        {
            pangolin::CreateGlutWindowAndBind("Main", 1280 + 180, 960, GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);

            glDisable(GL_MULTISAMPLE);

            glEnable(GL_DEPTH_TEST);

            s_cam = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                                                pangolin::ModelViewLookAt(-1, -5, -1, 0, 0, 0, pangolin::AxisNegY));

            pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
                                    .SetHandler(new pangolin::Handler3D(s_cam));

            pangolin::Display("Image").SetAspect(640.0f / 480.0f);

            pangolin::Display("multi").SetBounds(pangolin::Attach::Pix(0), 1 / 4.0f, pangolin::Attach::Pix(180), 1.0)
                                      .SetLayout(pangolin::LayoutEqualHorizontal)
                                      .AddDisplay(pangolin::Display("Image"));

            pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

            pause = new pangolin::Var<bool>("ui.Pause", false, true);
            step = new pangolin::Var<bool>("ui.Step", false, false);

            gpuMem = new pangolin::Var<int>("ui.GPU memory free", 0);

            totalPoints = new pangolin::Var<std::string>("ui.Total points", "0");
        }

        virtual ~GUI()
        {
            delete pause;
            delete step;
            delete totalPoints;
            delete gpuMem;
        }

        void preCall()
        {
            glClearColor(0.05, 0.05, 0.3, 0.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            pangolin::Display("cam").Activate(s_cam);

            drawGrid();
        }

        inline void drawFrustum(const glm::mat4 & pose)
        {
            Eigen::Matrix4f lastPose;

            memcpy(lastPose.data(), glm::value_ptr(pose), sizeof(Eigen::Matrix4f));

            Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
            K(0, 0) = Intrinsics::getInstance().fx();
            K(1, 1) = Intrinsics::getInstance().fy();
            K(0, 2) = Intrinsics::getInstance().cx();
            K(1, 2) = Intrinsics::getInstance().cy();

            Eigen::Matrix3f Kinv = K.inverse();

            pangolin::glDrawFrustrum(Kinv,
                                     Resolution::getInstance().width(),
                                     Resolution::getInstance().height(),
                                     lastPose,
                                     0.1f);
        }

        //adapted from PTAM, MapViewer.cc
        inline void drawGrid()
        {
            //set pose
            glPushMatrix();

            Eigen::Matrix4f m;
            m <<  0,  0, 1, 0,
                 -1,  0, 0, 0,
                  0, -1, 0, 0,
                  0,  0, 0, 1;
            glMultTransposeMatrixf((float*)m.data());

            glLineWidth(1);

            glBegin(GL_LINES);

            // Draw a larger grid around the outside..
            double dGridInterval = 0.1;

            double dMin = -100.0 * dGridInterval;
            double dMax = 100.0 * dGridInterval;

            double height = -4;

            for(int x = -10; x <= 10; x += 1)
            {
                if(x == 0)
                    glColor3f(1, 1, 1);
                else
                    glColor3f(0.3, 0.3, 0.3);
                glVertex3d((double) x * 10 * dGridInterval, dMin, height);
                glVertex3d((double) x * 10 * dGridInterval, dMax, height);
            }

            for(int y = -10; y <= 10; y += 1)
            {
                if(y == 0)
                    glColor3f(1, 1, 1);
                else
                    glColor3f(0.3, 0.3, 0.3);
                glVertex3d(dMin, (double) y * 10 * dGridInterval, height);
                glVertex3d(dMax, (double) y * 10 * dGridInterval, height);
            }

            glEnd();

            glBegin(GL_LINES);
            dMin = -10.0 * dGridInterval;
            dMax = 10.0 * dGridInterval;

            for(int x = -10; x <= 10; x++)
            {
                if(x == 0)
                    glColor3f(1, 1, 1);
                else
                    glColor3f(0.5, 0.5, 0.5);

                glVertex3d((double) x * dGridInterval, dMin, height);
                glVertex3d((double) x * dGridInterval, dMax, height);
            }

            for(int y = -10; y <= 10; y++)
            {
                if(y == 0)
                    glColor3f(1, 1, 1);
                else
                    glColor3f(0.5, 0.5, 0.5);
                glVertex3d(dMin, (double) y * dGridInterval, height);
                glVertex3d(dMax, (double) y * dGridInterval, height);
            }

            glColor3f(1, 0, 0);
            glVertex3d(0, 0, height);
            glVertex3d(1, 0, height);
            glColor3f(0, 1, 0);
            glVertex3d(0, 0, height);
            glVertex3d(0, 1, height);
            glColor3f(1, 1, 1);
            glVertex3d(0, 0, height);
            glVertex3d(0, 0, height + 1);
            glEnd();

            glPopMatrix();
        }

        void postCall()
        {
            GLint cur_avail_mem_kb = 0;
            glGetIntegerv(GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX, &cur_avail_mem_kb);

            int memFree = cur_avail_mem_kb / 1024;

            gpuMem->operator=(memFree);

            pangolin::FinishFrame();

            glFinish();
        }

        pangolin::Var<bool> * pause,
                            * step;
        pangolin::Var<int> * gpuMem;

        pangolin::Var<std::string> * totalPoints;

        pangolin::OpenGlRenderState s_cam;
};


#endif /* GUI_H_ */
