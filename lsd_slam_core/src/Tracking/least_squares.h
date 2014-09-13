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

#include "util/EigenCoreInclude.h"
#include <opencv2/core/core.hpp>



namespace lsd_slam
{


typedef Eigen::Matrix<float, 6, 1> Vector6;
typedef Eigen::Matrix<float, 6, 6> Matrix6x6;

typedef Eigen::Matrix<float, 7, 1> Vector7;
typedef Eigen::Matrix<float, 7, 7> Matrix7x7;

typedef Eigen::Matrix<float, 4, 1> Vector4;
typedef Eigen::Matrix<float, 4, 4> Matrix4x4;


/**
 * A 6x6 self adjoint matrix with optimized "rankUpdate(u, scale)" (10x faster than Eigen impl, 1.8x faster than MathSse::addOuterProduct(...)).
 */
class OptimizedSelfAdjointMatrix6x6f
{
public:
  OptimizedSelfAdjointMatrix6x6f();

  void rankUpdate(const Eigen::Matrix<float, 6, 1>& u, const float alpha);

  void operator +=(const OptimizedSelfAdjointMatrix6x6f& other);

  void setZero();

  void toEigen(Eigen::Matrix<float, 6, 6>& m) const;
private:
  enum {
    Size = 24
  };
  EIGEN_ALIGN16 float data[Size];
};




/**
 * Builds normal equations and solves them with Cholesky decomposition.
 */
class NormalEquationsLeastSquares
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  OptimizedSelfAdjointMatrix6x6f A_opt;
  Matrix6x6 A;
  Vector6 b;

  bool solved;
  float error;
  size_t maxnum_constraints, num_constraints;

  virtual ~NormalEquationsLeastSquares();

  virtual void initialize(const size_t maxnum_constraints);
  virtual void update(const Vector6& J, const float& res, const float& weight = 1.0f);
  virtual void finish();
  virtual void finishNoDivide();
  virtual void solve(Vector6& x);

  void combine(const NormalEquationsLeastSquares& other);
};



/**
 * Builds 4dof LGS (used for depth-lgs, at it has only 7 non-zero entries in jacobian)
 * only used to accumulate data, NOT really as LGS
 */
class NormalEquationsLeastSquares4
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Matrix4x4 A;
  Vector4 b;

  bool solved;
  float error;
  size_t maxnum_constraints, num_constraints;

  virtual ~NormalEquationsLeastSquares4();

  virtual void initialize(const size_t maxnum_constraints);
  virtual void update(const Vector4& J, const float& res, const float& weight = 1.0f);

  void combine(const NormalEquationsLeastSquares4& other);

  virtual void finishNoDivide();
};



class NormalEquationsLeastSquares7
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Matrix7x7 A;
  Vector7 b;

  bool solved;
  float error;
  size_t maxnum_constraints, num_constraints;

  virtual ~NormalEquationsLeastSquares7();

  virtual void initializeFrom(const NormalEquationsLeastSquares& ls6, const NormalEquationsLeastSquares4& ls4);
  void combine(const NormalEquationsLeastSquares7& other);
};



}
