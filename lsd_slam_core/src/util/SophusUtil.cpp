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

#include "sophus/se3.hpp"
#include "sophus/sim3.hpp"

// Compile the templates here once so they don't need to be compiled in every
// other file using them.
// 
// Other files then include SophusUtil.h which contains extern template
// declarations to prevent compiling them there again. (For this reason,
// this header must not be included here).
//
// Eigen::Matrix seemingly cannot be instantiated this way, as it tries to
// compile a constructor variant for 4-component vectors, resulting in a
// static assertion failure.


template class Eigen::Quaternion<float>;
template class Eigen::Quaternion<double>;

template class Sophus::SE3Group<float, 0>;
template class Sophus::SE3Group<double, 0>;

template class Sophus::Sim3Group<float, 0>;
template class Sophus::Sim3Group<double, 0>;

