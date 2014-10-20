/*
 * PangolinOutput3DWrapper.cpp
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#include "PangolinOutput3DWrapper.h"

#include "util/SophusUtil.h"
#include "util/settings.h"
#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "GlobalMapping/g2oTypeSim3Sophus.h"

namespace lsd_slam
{

PangolinOutput3DWrapper::PangolinOutput3DWrapper(int width, int height, GUI & gui)
 : width(width),
   height(height),
   gui(gui),
   publishLvl(0)
{

}

PangolinOutput3DWrapper::~PangolinOutput3DWrapper()
{

}

void PangolinOutput3DWrapper::updateImage(unsigned char * data)
{
    gui.updateImage(data);
}

void PangolinOutput3DWrapper::publishKeyframe(Frame* f)
{
    Keyframe * fMsg = new Keyframe;

    boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();

    fMsg->id = f->id();
    fMsg->time = f->timestamp();
    fMsg->isKeyframe = true;

    int w = f->width(publishLvl);
    int h = f->height(publishLvl);

    fMsg->camToWorld = f->getScaledCamToWorld().cast<float>();

    fMsg->fx = f->fx(publishLvl);
    fMsg->fy = f->fy(publishLvl);
    fMsg->cx = f->cx(publishLvl);
    fMsg->cy = f->cy(publishLvl);

    fMsg->width = w;
    fMsg->height = h;

    fMsg->pointData = new unsigned char[w * h * sizeof(InputPointDense)];

    InputPointDense * pc = (InputPointDense*)fMsg->pointData;

    const float* idepth = f->idepth(publishLvl);
    const float* idepthVar = f->idepthVar(publishLvl);
    const float* color = f->image(publishLvl);

    for(int idx = 0;idx < w * h; idx++)
    {
        pc[idx].idepth = idepth[idx];
        pc[idx].idepth_var = idepthVar[idx];
        pc[idx].color[0] = color[idx];
        pc[idx].color[1] = color[idx];
        pc[idx].color[2] = color[idx];
        pc[idx].color[3] = color[idx];
    }

    lock.unlock();

    gui.addKeyframe(fMsg);
}

void PangolinOutput3DWrapper::publishTrackedFrame(Frame* kf)
{
//    lsd_slam_viewer::keyframeMsg fMsg;
//
//
//    fMsg.id = kf->id();
//    fMsg.time = kf->timestamp();
//    fMsg.isKeyframe = false;
//
//
//    memcpy(fMsg.camToWorld.data(),kf->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
//    fMsg.fx = kf->fx(publishLvl);
//    fMsg.fy = kf->fy(publishLvl);
//    fMsg.cx = kf->cx(publishLvl);
//    fMsg.cy = kf->cy(publishLvl);
//    fMsg.width = kf->width(publishLvl);
//    fMsg.height = kf->height(publishLvl);
//
//    fMsg.pointcloud.clear();
//
//    liveframe_publisher.publish(fMsg);
//
//
//    SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());
//
//    geometry_msgs::PoseStamped pMsg;
//
//    pMsg.pose.position.x = camToWorld.translation()[0];
//    pMsg.pose.position.y = camToWorld.translation()[1];
//    pMsg.pose.position.z = camToWorld.translation()[2];
//    pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
//    pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
//    pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
//    pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();
//
//    if (pMsg.pose.orientation.w < 0)
//    {
//        pMsg.pose.orientation.x *= -1;
//        pMsg.pose.orientation.y *= -1;
//        pMsg.pose.orientation.z *= -1;
//        pMsg.pose.orientation.w *= -1;
//    }
//
//    pMsg.header.stamp = ros::Time(kf->timestamp());
//    pMsg.header.frame_id = "world";
//    pose_publisher.publish(pMsg);
}

void PangolinOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
{
    graph->keyframesAllMutex.lock_shared();

    int num = graph->keyframesAll.size();

    unsigned char * buffer = new unsigned char[num * sizeof(GraphFramePose)];

    GraphFramePose* framePoseData = (GraphFramePose*)buffer;

    for(unsigned int i = 0; i < graph->keyframesAll.size(); i++)
    {
        framePoseData[i].id = graph->keyframesAll[i]->id();
        memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(), sizeof(float) * 7);
    }

    graph->keyframesAllMutex.unlock_shared();

    gui.updateKeyframePoses(framePoseData, num);

    delete [] buffer;
}

void PangolinOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
{
    //TODO
}

void PangolinOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{
    //TODO
}

void PangolinOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{
    //TODO
}

}
