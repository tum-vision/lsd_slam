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

#pragma once

#include "sophus/sim3.hpp"
#include "sophus/se3.hpp"



// Typedef and conversion macro for Eigen matrices to currently used type.
// NOTE: a "no-op conversion" is free in terms of performance, as it should be compiled out.
#ifdef SOPHUS_USE_FLOAT
	typedef Sophus::SE3f SE3;
	typedef Sophus::Sim3f Sim3;
	typedef Sophus::SO3f SO3;
	#define toSophus(x) ((x).cast<float>())
	#define sophusType float
#else
//	typedef Sophus::Vector3d Vector3;
//	typedef Sophus::Vector4d Vector4;
//	typedef Sophus::Vector6d Vector6;
//	typedef Sophus::Vector7d Vector7;
//	typedef Sophus::Quaterniond Quaternion;
	typedef Sophus::SE3d SE3;
	typedef Sophus::Sim3d Sim3;
	typedef Sophus::SO3d SO3;
	#define toSophus(x) ((x).cast<double>())
	#define sophusType double
#endif


namespace lsd_slam
{

inline Sim3 sim3FromSE3(const SE3& se3, sophusType scale)
{
	Sim3 result(se3.unit_quaternion(), se3.translation());
	result.setScale(scale);
	return result;
}

inline SE3 se3FromSim3(const Sim3& sim3)
{
	return SE3(sim3.quaternion(), sim3.translation());
}


}

// Extern templates (see SophusUtil.cpp)
extern template class Eigen::Quaternion<float>;
extern template class Eigen::Quaternion<double>;

extern template class Sophus::SE3Group<float, 0>;
extern template class Sophus::SE3Group<double, 0>;

extern template class Sophus::Sim3Group<float, 0>;
extern template class Sophus::Sim3Group<double, 0>;
