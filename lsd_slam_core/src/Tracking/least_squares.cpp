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

#include "least_squares.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <stdio.h>

namespace lsd_slam
{


NormalEquationsLeastSquares::~NormalEquationsLeastSquares() { }

void NormalEquationsLeastSquares::initialize(const size_t maxnum_constraints)
{
  A.setZero();
  A_opt.setZero();
  b.setZero();
  solved = false;
  error = 0;
  this->num_constraints = 0;
  this->maxnum_constraints = maxnum_constraints;
}

inline void NormalEquationsLeastSquares::update(const Vector6& J, const float& res, const float& weight)
{
//	printf("up: %f, %f, %f, %f, %f, %f; res: %f; w: %f\n",
//			J[0],J[1],J[2],J[3],J[4],J[5],res, weight);

  A_opt.rankUpdate(J, weight);
  //MathSse<Sse::Enabled, float>::addOuterProduct(A, J, factor);
  //A += J * J.transpose() * factor;
  //MathSse<Sse::Enabled, float>::add(b, J, -res * factor); // not much difference :(
  b -= J * (res * weight);

  error += res * res * weight;
  num_constraints += 1;
}

void NormalEquationsLeastSquares::combine(const NormalEquationsLeastSquares& other)
{
  A_opt += other.A_opt;
  b += other.b;
  error += other.error;
  num_constraints += other.num_constraints;
}

void NormalEquationsLeastSquares::finish()
{
  A_opt.toEigen(A);
  A /= (float) num_constraints;
  b /= (float) num_constraints;
  error /= (float) num_constraints;
}
void NormalEquationsLeastSquares::finishNoDivide()
{
  A_opt.toEigen(A);
}

void NormalEquationsLeastSquares::solve(Vector6& x)
{
  x = A.ldlt().solve(b);
  solved = true;
}


OptimizedSelfAdjointMatrix6x6f::OptimizedSelfAdjointMatrix6x6f()
{
}

void OptimizedSelfAdjointMatrix6x6f::setZero()
{
  for(size_t idx = 0; idx < Size; idx++)
    data[idx] = 0.0f;
}

#if !defined(ENABLE_SSE) && !defined(__SSE__)
	
	// TODO: Ugly temporary replacement for SSE instructions to make rankUpdate() work.
	// TODO: code faster version
	
	struct __m128 {
		__m128(float a, float b, float c, float d) {
			data[0] = a;
			data[1] = b;
			data[2] = c;
			data[3] = d;
		}
		float& operator[](int idx) {
			return data[idx];
		}
		const float& operator[](int idx) const {
			return data[idx];
		}
		float data[4];
	};
	__m128 _mm_set1_ps(float v) {
		return __m128(v, v, v, v);
	}
	__m128 _mm_loadu_ps(const float* d) {
		return __m128(d[0], d[1], d[2], d[3]);
	}
	__m128 _mm_load_ps(const float* d) {
		return __m128(d[0], d[1], d[2], d[3]);
	}
	__m128 _mm_movelh_ps(const __m128& a, const __m128& b) {
		return __m128(a[0], a[1], b[0], b[1]);
	}
	__m128 _mm_movehl_ps(const __m128& a, const __m128& b) {
		return __m128(b[2], b[3], a[2], a[3]);
	}
	__m128 _mm_unpacklo_ps(const __m128& a, const __m128& b) {
		return __m128(a[0], b[0], a[1], b[1]);
	}
	__m128 _mm_mul_ps(const __m128& a, const __m128& b) {
		return __m128(a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]);
	}
	__m128 _mm_add_ps(const __m128& a, const __m128& b) {
		return __m128(a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]);
	}
	void _mm_store_ps(float* p, const __m128& a) {
		p[0] = a[0];
		p[1] = a[1];
		p[2] = a[2];
		p[3] = a[3];
	}
	
#endif

inline void OptimizedSelfAdjointMatrix6x6f::rankUpdate(const Eigen::Matrix<float, 6, 1>& u, const float alpha)
{
#if defined(ENABLE_NEON)
	
	const float* in_ptr = u.data();
	float* out_ptr = data;
	__asm__ __volatile__
	(
		// NOTE: may reduce count of used registers by calculating some value(s?) later.
		
		"vdup.32  q15, %[alpha]               \n\t" // alpha(q15)
		"vldmia   %[in_ptr], {q9-q10}         \n\t" // v1234(q9), v56xx(q10)
		"vldmia   %[out_ptr], {q0-q5}         \n\t"

		"vmov     d21, d20                    \n\t" // v5656(q10)
		"vmov     q11, q10                    \n\t"
		"vmov     q12, q10                    \n\t"
		"vzip.32  q11, q12                    \n\t" // v5566(q11)
		"vmul.f32 q11, q11, q15               \n\t" // alpha*v3344(q14)
	
		"vmov     q12, q9                     \n\t"
		"vmov     d19, d18                    \n\t" // v1212(q9)
		"vmov     d24, d25                    \n\t" // v3434(q12)
	
		"vmov     q13, q9                     \n\t"
		"vmov     q14, q9                     \n\t"
		"vzip.32  q13, q14                    \n\t" // v1122(q13)
		"vmul.f32 q13, q13, q15               \n\t" // alpha*v1122(q13)
	
		"vmov     q14, q12                    \n\t"
		"vmov     q8, q12                     \n\t"
		"vzip.32  q14, q8                     \n\t" // v3344(q14)
		"vmul.f32 q14, q14, q15               \n\t" // alpha*v3344(q14)
	
		"vmla.f32 q0, q13, q9                 \n\t"
		"vmla.f32 q1, q13, q12                \n\t"
		"vmla.f32 q2, q13, q10                \n\t"
		
		"vmla.f32 q3, q14, q12                \n\t"
		"vmla.f32 q4, q14, q10                \n\t"
		
		"vmla.f32 q5, q11, q10                \n\t"
		
		"vstmia %[out_ptr], {q0-q5}           \n\t"

	: /* outputs */ 
	: /* inputs  */ [alpha]"r"(alpha), [in_ptr]"r"(in_ptr), [out_ptr]"r"(out_ptr)
	: /* clobber */ "memory", "cc", // TODO: is cc necessary?
					"q0", "q1", "q2", "q3", "q4", "q5", "q8", "q9", "q10", "q11", "q12", "q13", "q14"
	);
	
#else
	
  __m128 s = _mm_set1_ps(alpha);
  __m128 v1234 = _mm_loadu_ps(u.data());
  __m128 v56xx = _mm_loadu_ps(u.data() + 4);

  __m128 v1212 = _mm_movelh_ps(v1234, v1234);
  __m128 v3434 = _mm_movehl_ps(v1234, v1234);
  __m128 v5656 = _mm_movelh_ps(v56xx, v56xx);

  __m128 v1122 = _mm_mul_ps(s, _mm_unpacklo_ps(v1212, v1212));

  _mm_store_ps(data + 0, _mm_add_ps(_mm_load_ps(data + 0), _mm_mul_ps(v1122, v1212)));
  _mm_store_ps(data + 4, _mm_add_ps(_mm_load_ps(data + 4), _mm_mul_ps(v1122, v3434)));
  _mm_store_ps(data + 8, _mm_add_ps(_mm_load_ps(data + 8), _mm_mul_ps(v1122, v5656)));

  __m128 v3344 = _mm_mul_ps(s, _mm_unpacklo_ps(v3434, v3434));

  _mm_store_ps(data + 12, _mm_add_ps(_mm_load_ps(data + 12), _mm_mul_ps(v3344, v3434)));
  _mm_store_ps(data + 16, _mm_add_ps(_mm_load_ps(data + 16), _mm_mul_ps(v3344, v5656)));

  __m128 v5566 = _mm_mul_ps(s, _mm_unpacklo_ps(v5656, v5656));

  _mm_store_ps(data + 20, _mm_add_ps(_mm_load_ps(data + 20), _mm_mul_ps(v5566, v5656)));
  
#endif
}

inline void OptimizedSelfAdjointMatrix6x6f::operator +=(const OptimizedSelfAdjointMatrix6x6f& other)
{
#if defined(ENABLE_SSE)
  _mm_store_ps(data +  0, _mm_add_ps(_mm_load_ps(data +  0), _mm_load_ps(other.data +  0)));
  _mm_store_ps(data +  4, _mm_add_ps(_mm_load_ps(data +  4), _mm_load_ps(other.data +  4)));
  _mm_store_ps(data +  8, _mm_add_ps(_mm_load_ps(data +  8), _mm_load_ps(other.data +  8)));
  _mm_store_ps(data + 12, _mm_add_ps(_mm_load_ps(data + 12), _mm_load_ps(other.data + 12)));
  _mm_store_ps(data + 16, _mm_add_ps(_mm_load_ps(data + 16), _mm_load_ps(other.data + 16)));
  _mm_store_ps(data + 20, _mm_add_ps(_mm_load_ps(data + 20), _mm_load_ps(other.data + 20)));
#elif defined(ENABLE_NEON)
    const float* other_data_ptr = other.data;
  	__asm__ __volatile__
	(
		// NOTE: The way of loading the data was benchmarked and this was the
		// fastest variant (faster than loading everything in order, and faster
		// than loading both blocks at once).
		"vldmia   %[other_data]!, {q0-q2}     \n\t"
		"vldmia   %[data], {q9-q14}           \n\t"
		"vldmia   %[other_data], {q3-q5}      \n\t"
		
		"vadd.f32 q0, q0, q9                  \n\t"
		"vadd.f32 q1, q1, q10                 \n\t"
		"vadd.f32 q2, q2, q11                 \n\t"
		"vadd.f32 q3, q3, q12                 \n\t"
		"vadd.f32 q4, q4, q13                 \n\t"
		"vadd.f32 q5, q5, q14                 \n\t"
		
		"vstmia %[data], {q0-q5}              \n\t"

	: /* outputs */ [other_data]"+&r"(other_data_ptr)
	: /* inputs  */ [data]"r"(data)
	: /* clobber */ "memory",
					"q0", "q1", "q2", "q3", "q4", "q5", "q9", "q10", "q11", "q12", "q13", "q14"
	);
#else
  for(size_t idx = 0; idx < Size; idx++)
    data[idx] += other.data[idx];
#endif
}

void OptimizedSelfAdjointMatrix6x6f::toEigen(Eigen::Matrix<float, 6, 6>& m) const
{
  Eigen::Matrix<float, 6, 6> tmp;
  size_t idx = 0;

  for(size_t i = 0; i < 6; i += 2)
  {
    for(size_t j = i; j < 6; j += 2)
    {
      tmp(i  , j  ) = data[idx++];
      tmp(i  , j+1) = data[idx++];
      tmp(i+1, j  ) = data[idx++];
      tmp(i+1, j+1) = data[idx++];
    }
  }

  tmp.selfadjointView<Eigen::Upper>().evalTo(m);
}









NormalEquationsLeastSquares4::~NormalEquationsLeastSquares4() { }

void NormalEquationsLeastSquares4::initialize(const size_t maxnum_constraints)
{
  A.setZero();
  b.setZero();
  solved = false;
  error = 0;
  this->num_constraints = 0;
  this->maxnum_constraints = maxnum_constraints;
}

inline void NormalEquationsLeastSquares4::update(const Vector4& J, const float& res, const float& weight)
{
	// TODO: SSE optimization
  A.noalias() += J * J.transpose() * weight;
  b.noalias() -= J * (res * weight);
  error += res * res * weight;
  num_constraints += 1;
}

void NormalEquationsLeastSquares4::combine(const NormalEquationsLeastSquares4& other)
{
  A += other.A;
  b += other.b;
  error += other.error;
  num_constraints += other.num_constraints;
}

void NormalEquationsLeastSquares4::finishNoDivide()
{
	// TODO: SSE optimization
}

NormalEquationsLeastSquares7::~NormalEquationsLeastSquares7() { }

void NormalEquationsLeastSquares7::initializeFrom(const NormalEquationsLeastSquares& ls6, const NormalEquationsLeastSquares4& ls4)
{
	// set zero
	A.setZero();
	b.setZero();

	// add ls6
	A.topLeftCorner<6,6>() = ls6.A;
	b.head<6>() = ls6.b;

	// add ls4
	int remap[4] = {2,3,4,6};
	for(int i=0;i<4;i++)
	{
		b[remap[i]] += ls4.b[i];
		for(int j=0;j<4;j++)
			A(remap[i], remap[j]) += ls4.A(i,j);
	}

	num_constraints = ls6.num_constraints + ls4.num_constraints;
}
void NormalEquationsLeastSquares7::combine(const NormalEquationsLeastSquares7& other)
{
	  A += other.A;
	  b += other.b;
	  error += other.error;
	  num_constraints += other.num_constraints;
}

}
