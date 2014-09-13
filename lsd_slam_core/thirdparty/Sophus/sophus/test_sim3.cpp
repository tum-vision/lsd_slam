// This file is part of Sophus.
//
// Copyright 2012-2013 Hauke Strasdat
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include <iostream>
#include <vector>

#include <unsupported/Eigen/MatrixFunctions>

#include "sim3.hpp"
#include "tests.hpp"

using namespace Sophus;
using namespace std;

template<class Scalar>
void tests() {

  typedef Sim3Group<Scalar> Sim3Type;
  typedef RxSO3Group<Scalar> RxSO3Type;
  typedef typename Sim3Group<Scalar>::Point Point;
  typedef typename Sim3Group<Scalar>::Tangent Tangent;
  typedef Matrix<Scalar,4,1> Vector4Type;

  vector<Sim3Type> sim3_vec;
  sim3_vec.push_back(Sim3Type(RxSO3Type::exp(Vector4Type(0.2, 0.5, 0.0,1.)),
                              Point(0,0,0)));
  sim3_vec.push_back(Sim3Type(RxSO3Type::exp(Vector4Type(0.2, 0.5, -1.0,1.1)),
                              Point(10,0,0)));
  sim3_vec.push_back(Sim3Type(RxSO3Type::exp(Vector4Type(0., 0., 0.,1.1)),
                              Point(0,10,5)));
  sim3_vec.push_back(Sim3Type(RxSO3Type::exp(Vector4Type(0., 0., 0.00001, 0.)),
                              Point(0,0,0)));
  sim3_vec.push_back(Sim3Type(RxSO3Type::exp(
                                Vector4Type(0., 0., 0.00001, 0.0000001)),
                              Point(1,-1.00000001,2.0000000001)));
  sim3_vec.push_back(Sim3Type(RxSO3Type::exp(Vector4Type(0., 0., 0.00001, 0)),
                              Point(0.01,0,0)));
  sim3_vec.push_back(Sim3Type(RxSO3Type::exp(Vector4Type(M_PI, 0, 0,0.9)),
                              Point(4,-5,0)));
  sim3_vec.push_back(Sim3Type(RxSO3Type::exp(Vector4Type(0.2, 0.5, 0.0,0)),
                              Point(0,0,0))
                     *Sim3Type(RxSO3Type::exp(Vector4Type(M_PI, 0, 0,0)),
                               Point(0,0,0))
                     *Sim3Type(RxSO3Type::exp(Vector4Type(-0.2, -0.5, -0.0,0)),
                               Point(0,0,0)));
  sim3_vec.push_back(Sim3Type(RxSO3Type::exp(Vector4Type(0.3, 0.5, 0.1,0)),
                              Point(2,0,-7))
                     *Sim3Type(RxSO3Type::exp(Vector4Type(M_PI, 0, 0,0)),
                               Point(0,0,0))
                     *Sim3Type(RxSO3Type::exp(Vector4Type(-0.3, -0.5, -0.1,0)),
                               Point(0,6,0)));
  vector<Tangent> tangent_vec;
  Tangent tmp;
  tmp << 0,0,0,0,0,0,0;
  tangent_vec.push_back(tmp);
  tmp << 1,0,0,0,0,0,0;
  tangent_vec.push_back(tmp);
  tmp << 0,1,0,1,0,0,0.1;
  tangent_vec.push_back(tmp);
  tmp << 0,0,1,0,1,0,0.1;
  tangent_vec.push_back(tmp);
  tmp << -1,1,0,0,0,1,-0.1;
  tangent_vec.push_back(tmp);
  tmp << 20,-1,0,-1,1,0,-0.1;
  tangent_vec.push_back(tmp);
  tmp << 30,5,-1,20,-1,0,1.5;
  tangent_vec.push_back(tmp);


  vector<Point> point_vec;
  point_vec.push_back(Point(1,2,4));

  Tests<Sim3Type> tests;
  tests.setGroupElements(sim3_vec);
  tests.setTangentVectors(tangent_vec);
  tests.setPoints(point_vec);

  tests.runAllTests();
}

int main() {
  cerr << "Test Sim3" << endl << endl;

  cerr << "Double tests: " << endl;
  tests<double>();

  cerr << "Float tests: " << endl;
  tests<float>();
  return 0;
}
