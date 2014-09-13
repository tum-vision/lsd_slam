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

#include "GlobalMapping/g2oTypeSim3Sophus.h"

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

namespace lsd_slam
{


G2O_USE_TYPE_GROUP(sba);

G2O_REGISTER_TYPE_GROUP(sim3sophus);

G2O_REGISTER_TYPE(VERTEX_SIM3_SOPHUS:EXPMAP, VertexSim3);
G2O_REGISTER_TYPE(EDGE_SIM3_SOPHUS:EXPMAP, EdgeSim3);

VertexSim3::VertexSim3() : g2o::BaseVertex<7, Sophus::Sim3d>()
{
	_marginalized=false;
	_fix_scale = false;
}

bool VertexSim3::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;
//     Sim3 cam2world(estimate().inverse());
//     Vector7d lv=cam2world.log();
//     for (int i=0; i<7; i++){
//       os << lv[i] << " ";
//     }
//     for (int i=0; i<2; i++)
//     {
//       os << _focal_length[i] << " ";
//     }
//     for (int i=0; i<2; i++)
//     {
//       os << _principle_point[i] << " ";
//     }
//     return os.good();
}

bool VertexSim3::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
//     Vector7d cam2world;
//     for (int i=0; i<6; i++){
//       is >> cam2world[i];
//     }
//     is >> cam2world[6];
// //    if (! is) {
// //      // if the scale is not specified we set it to 1;
// //      std::cerr << "!s";
// //      cam2world[6]=0.;
// //    }
// 
//     for (int i=0; i<2; i++)
//     {
//       is >> _focal_length[i];
//     }
//     for (int i=0; i<2; i++)
//     {
//       is >> _principle_point[i];
//     }
// 
//     setEstimate(Sim3(cam2world).inverse());
//     return true;
}


EdgeSim3::EdgeSim3() :
	g2o::BaseBinaryEdge<7, Sophus::Sim3d, VertexSim3, VertexSim3>()
{
}

bool EdgeSim3::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;
//     Sim3 cam2world(measurement().inverse());
//     Vector7d v7 = cam2world.log();
//     for (int i=0; i<7; i++)
//     {
//       os  << v7[i] << " ";
//     }
//     for (int i=0; i<7; i++)
//       for (int j=i; j<7; j++){
//         os << " " <<  information()(i,j);
//     }
//     return os.good();
}

bool EdgeSim3::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
//     Vector7d v7;
//     for (int i=0; i<7; i++){
//       is >> v7[i];
//     }
// 
//     Sim3 cam2world(v7);
//     setMeasurement(cam2world.inverse());
// 
//     for (int i=0; i<7; i++)
//       for (int j=i; j<7; j++)
//       {
//         is >> information()(i,j);
//         if (i!=j)
//           information()(j,i)=information()(i,j);
//       }
//     return true;
}
}
