#ifndef SOPUHS_TESTS_HPP
#define SOPUHS_TESTS_HPP

#include <vector>
#include <unsupported/Eigen/MatrixFunctions>

#include "sophus.hpp"

namespace Sophus {

using namespace std;
using namespace Eigen;

template <class LieGroup>
class Tests {

public:
  typedef typename LieGroup::Scalar Scalar;
  typedef typename LieGroup::Transformation Transformation;
  typedef typename LieGroup::Tangent Tangent;
  typedef typename LieGroup::Point Point;
  typedef typename LieGroup::Adjoint Adjoint;
  static const int N = LieGroup::N;
  static const int DoF = LieGroup::DoF;

  const Scalar SMALL_EPS;

  Tests() : SMALL_EPS(SophusConstants<Scalar>::epsilon()) {
  }

  void setGroupElements(const vector<LieGroup> & group_vec) {
    group_vec_  = group_vec;
  }

  void setTangentVectors(const vector<Tangent> & tangent_vec) {
    tangent_vec_  = tangent_vec;
  }

  void setPoints(const vector<Point> & point_vec) {
    point_vec_  = point_vec;
  }

  bool adjointTest() {
    bool passed = true;
    for (size_t i=0; i<group_vec_.size(); ++i) {
      Transformation T = group_vec_[i].matrix();
      Adjoint Ad = group_vec_[i].Adj();
      for (size_t j=0; j<tangent_vec_.size(); ++j) {
        Tangent x = tangent_vec_[j];

        Transformation I;
        I.setIdentity();
        Tangent ad1 = Ad*x;
        Tangent ad2 = LieGroup::vee(T*LieGroup::hat(x)
                                    *group_vec_[i].inverse().matrix());
        Scalar nrm = norm(ad1-ad2);

        if (isnan(nrm) || nrm>20.*SMALL_EPS) {
          cerr << "Adjoint" << endl;
          cerr  << "Test case: " << i << "," << j <<endl;
          cerr << (ad1-ad2) <<endl;
          cerr << endl;
          passed = false;
        }
      }
    }
    return passed;
  }

  bool expLogTest() {
    bool passed = true;

    for (size_t i=0; i<group_vec_.size(); ++i) {
      Transformation T1 = group_vec_[i].matrix();
      Transformation T2 = LieGroup::exp(group_vec_[i].log()).matrix();
      Transformation DiffT = T1-T2;
      Scalar nrm = DiffT.norm();

      if (isnan(nrm) || nrm>SMALL_EPS) {
        cerr << "G - exp(log(G))" << endl;
        cerr  << "Test case: " << i << endl;
        cerr << DiffT <<endl;
        cerr << endl;
        passed = false;
      }
    }
    return passed;
  }

  bool expMapTest() {
    bool passed = true;
    for (size_t i=0; i<tangent_vec_.size(); ++i) {

      Tangent omega = tangent_vec_[i];
      Transformation exp_x = LieGroup::exp(omega).matrix();
      Transformation expmap_hat_x = (LieGroup::hat(omega)).exp();
      Transformation DiffR = exp_x-expmap_hat_x;
      Scalar nrm = DiffR.norm();

      if (isnan(nrm) || nrm>10.*SMALL_EPS) {
        cerr << "expmap(hat(x)) - exp(x)" << endl;
        cerr  << "Test case: " << i << endl;
        cerr << exp_x <<endl;
        cerr << expmap_hat_x <<endl;
        cerr << DiffR <<endl;
        cerr << endl;
        passed = false;
      }
    }
    return passed;
  }

  bool groupActionTest() {
    bool passed = true;

    for (size_t i=0; i<group_vec_.size(); ++i) {
      for (size_t j=0; j<point_vec_.size(); ++j) {
        const Point & p = point_vec_[j];
        Transformation T = group_vec_[i].matrix();
        Point res1 = group_vec_[i]*p;
        Point res2 = map(T, p);
        Scalar nrm = (res1-res2).norm();
        if (isnan(nrm) || nrm>SMALL_EPS) {
          cerr << "Transform vector" << endl;
          cerr  << "Test case: " << i << endl;
          cerr << (res1-res2) <<endl;
          cerr << endl;
          passed = false;
        }
      }
    }
    return passed;
  }


  bool lieBracketTest() {
    bool passed = true;
    for (size_t i=0; i<tangent_vec_.size(); ++i) {
      for (size_t j=0; j<tangent_vec_.size(); ++j) {
        Tangent res1 = LieGroup::lieBracket(tangent_vec_[i],tangent_vec_[j]);
        Transformation hati = LieGroup::hat(tangent_vec_[i]);
        Transformation hatj = LieGroup::hat(tangent_vec_[j]);

        Tangent res2 = LieGroup::vee(hati*hatj-hatj*hati);
        Tangent resDiff = res1-res2;
        if (norm(resDiff)>SMALL_EPS) {
          cerr << "Lie Bracket Test" << endl;
          cerr  << "Test case: " << i << ", " <<j<< endl;
          cerr << resDiff << endl;
          cerr << endl;
          passed = false;
        }
      }
    }
    return passed;
  }

  bool mapAndMultTest() {
    bool passed = true;
    for (size_t i=0; i<group_vec_.size(); ++i) {
      for (size_t j=0; j<group_vec_.size(); ++j) {
        Transformation mul_resmat = (group_vec_[i]*group_vec_[j]).matrix();
        Scalar fastmul_res_raw[LieGroup::num_parameters];
        Eigen::Map<LieGroup> fastmul_res(fastmul_res_raw);
        Eigen::Map<const LieGroup> group_j_constmap(group_vec_[j].data());
        fastmul_res = group_vec_[i];
        fastmul_res.fastMultiply(group_j_constmap);
        Transformation diff =  mul_resmat-fastmul_res.matrix();
        Scalar nrm = diff.norm();
        if (isnan(nrm) || nrm>SMALL_EPS) {
          cerr << "Map & Multiply" << endl;
          cerr  << "Test case: " << i  << "," << j << endl;
          cerr << diff <<endl;
          cerr << endl;
          passed = false;
        }
      }
    }
    return passed;
  }

  bool veeHatTest() {
    bool passed = true;
    for (size_t i=0; i<tangent_vec_.size(); ++i) {
      Tangent resDiff
          = tangent_vec_[i] - LieGroup::vee(LieGroup::hat(tangent_vec_[i]));
      if (norm(resDiff)>SMALL_EPS) {
        cerr << "Hat-vee Test" << endl;
        cerr  << "Test case: " << i <<  endl;
        cerr << resDiff << endl;
        cerr << endl;
        passed = false;
      }
    }
    return passed;
  }



  void runAllTests() {
    bool passed = adjointTest();
    if (!passed) {
      cerr << "failed!" << endl << endl;
      exit(-1);
    }
    passed = expLogTest();
    if (!passed) {
      cerr << "failed!" << endl << endl;
      exit(-1);
    }
    passed = expMapTest();
    if (!passed) {
      cerr << "failed!" << endl << endl;
      exit(-1);
    }
    passed = groupActionTest();
    if (!passed) {
      cerr << "failed!" << endl << endl;
      exit(-1);
    }
    passed = lieBracketTest();
    if (!passed) {
      cerr << "failed!" << endl << endl;
      exit(-1);
    }
    passed = mapAndMultTest();
    if (!passed) {
      cerr << "failed!" << endl << endl;
      exit(-1);
    }
    passed = veeHatTest();
    if (!passed) {
      cerr << "failed!" << endl << endl;
      exit(-1);
    }
    cerr << "passed." << endl << endl;
  }

private:
  Matrix<Scalar,N-1,1> map(const Matrix<Scalar,N,N> & T,
                           const Matrix<Scalar,N-1,1> & p) {
    return T.template topLeftCorner<N-1,N-1>()*p
        + T.template topRightCorner<N-1,1>();
  }

  Matrix<Scalar,N,1> map(const Matrix<Scalar,N,N> & T,
                         const Matrix<Scalar,N,1> & p) {
    return T*p;
  }

  Scalar norm(const Scalar & v) {
    return std::abs(v);
  }

  Scalar norm(const Matrix<Scalar,DoF,1> & T) {
    return T.norm();
  }

  std::vector<LieGroup> group_vec_;
  std::vector<Tangent> tangent_vec_;
  std::vector<Point> point_vec_;
};
}
#endif // TESTS_HPP
