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

#include "so3.hpp"
#include "tests.hpp"

using namespace Sophus;
using namespace std;

template<class Scalar>
void tests() {

  typedef SO3Group<Scalar> SO3Type;
  typedef typename SO3Group<Scalar>::Point Point;
  typedef typename SO3Group<Scalar>::Tangent Tangent;

  vector<SO3Type> so3_vec;

  so3_vec.push_back(SO3Type(Quaternion<Scalar>(0.1e-11, 0., 1., 0.)));
  so3_vec.push_back(SO3Type(Quaternion<Scalar>(-1,0.00001,0.0,0.0)));
  so3_vec.push_back(SO3Type::exp(Point(0.2, 0.5, 0.0)));
  so3_vec.push_back(SO3Type::exp(Point(0.2, 0.5, -1.0)));
  so3_vec.push_back(SO3Type::exp(Point(0., 0., 0.)));
  so3_vec.push_back(SO3Type::exp(Point(0., 0., 0.00001)));
  so3_vec.push_back(SO3Type::exp(Point(M_PI, 0, 0)));
  so3_vec.push_back(SO3Type::exp(Point(0.2, 0.5, 0.0))
                    *SO3Type::exp(Point(M_PI, 0, 0))
                    *SO3Type::exp(Point(-0.2, -0.5, -0.0)));
  so3_vec.push_back(SO3Type::exp(Point(0.3, 0.5, 0.1))
                    *SO3Type::exp(Point(M_PI, 0, 0))
                    *SO3Type::exp(Point(-0.3, -0.5, -0.1)));

  vector<Tangent> tangent_vec;
  tangent_vec.push_back(Tangent(0,0,0));
  tangent_vec.push_back(Tangent(1,0,0));
  tangent_vec.push_back(Tangent(0,1,0));
  tangent_vec.push_back(Tangent(M_PI_2,M_PI_2,0.0));
  tangent_vec.push_back(Tangent(-1,1,0));
  tangent_vec.push_back(Tangent(20,-1,0));
  tangent_vec.push_back(Tangent(30,5,-1));

  vector<Point> point_vec;
  point_vec.push_back(Point(1,2,4));

  Tests<SO3Type> tests;
  tests.setGroupElements(so3_vec);
  tests.setTangentVectors(tangent_vec);
  tests.setPoints(point_vec);

  tests.runAllTests();
}

int main() {
  cerr << "Test SO3" << endl << endl;

  cerr << "Double tests: " << endl;
  tests<double>();

  cerr << "Float tests: " << endl;
  tests<float>();
  return 0;
}
