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

#ifndef SOPHUS_SE2_HPP
#define SOPHUS_SE2_HPP

#include "so2.hpp"

////////////////////////////////////////////////////////////////////////////
// Forward Declarations / typedefs
////////////////////////////////////////////////////////////////////////////

namespace Sophus {
template<typename _Scalar, int _Options=0> class SE2Group;
typedef SE2Group<double> SE2 EIGEN_DEPRECATED;
typedef SE2Group<double> SE2d; /**< double precision SE2 */
typedef SE2Group<float> SE2f;  /**< single precision SE2 */
}

////////////////////////////////////////////////////////////////////////////
// Eigen Traits (For querying derived types in CRTP hierarchy)
////////////////////////////////////////////////////////////////////////////

namespace Eigen {
namespace internal {

template<typename _Scalar, int _Options>
struct traits<Sophus::SE2Group<_Scalar,_Options> > {
  typedef _Scalar Scalar;
  typedef Matrix<Scalar,2,1> TranslationType;
  typedef Sophus::SO2Group<Scalar> SO2Type;
};

template<typename _Scalar, int _Options>
struct traits<Map<Sophus::SE2Group<_Scalar>, _Options> >
    : traits<Sophus::SE2Group<_Scalar, _Options> > {
  typedef _Scalar Scalar;
  typedef Map<Matrix<Scalar,2,1>,_Options> TranslationType;
  typedef Map<Sophus::SO2Group<Scalar>,_Options> SO2Type;
};

template<typename _Scalar, int _Options>
struct traits<Map<const Sophus::SE2Group<_Scalar>, _Options> >
    : traits<const Sophus::SE2Group<_Scalar, _Options> > {
  typedef _Scalar Scalar;
  typedef Map<const Matrix<Scalar,2,1>,_Options> TranslationType;
  typedef Map<const Sophus::SO2Group<Scalar>,_Options> SO2Type;
};

}
}

namespace Sophus {
using namespace Eigen;
using namespace std;

/**
 * \brief SE2 base type - implements SE2 class but is storage agnostic
 *
 * [add more detailed description/tutorial]
 */
template<typename Derived>
class SE2GroupBase {
public:
  /** \brief scalar type */
  typedef typename internal::traits<Derived>::Scalar Scalar;
  /** \brief translation reference type */
  typedef typename internal::traits<Derived>::TranslationType &
  TranslationReference;
  /** \brief translation const reference type */
  typedef const typename internal::traits<Derived>::TranslationType &
  ConstTranslationReference;
  /** \brief SO2 reference type */
  typedef typename internal::traits<Derived>::SO2Type &
  SO2Reference;
  /** \brief SO2 type */
  typedef const typename internal::traits<Derived>::SO2Type &
  ConstSO2Reference;

  /** \brief degree of freedom of group
    *        (two for translation, one for in-plane rotation) */
  static const int DoF = 3;
  /** \brief number of internal parameters used
    *        (unit complex number for rotation + translation 2-vector) */
  static const int num_parameters = 4;
  /** \brief group transformations are NxN matrices */
  static const int N = 3;
  /** \brief group transfomation type */
  typedef Matrix<Scalar,N,N> Transformation;
  /** \brief point type */
  typedef Matrix<Scalar,2,1> Point;
  /** \brief tangent vector type */
  typedef Matrix<Scalar,DoF,1> Tangent;
  /** \brief adjoint transformation type */
  typedef Matrix<Scalar,DoF,DoF> Adjoint;

  /**
   * \brief Adjoint transformation
   *
   * This function return the adjoint transformation \f$ Ad \f$ of the
   * group instance \f$ A \f$  such that for all \f$ x \f$
   * it holds that \f$ \widehat{Ad_A\cdot x} = A\widehat{x}A^{-1} \f$
   * with \f$\ \widehat{\cdot} \f$ being the hat()-operator.
   */
  inline
  const Adjoint Adj() const {
    const Matrix<Scalar,2,2> & R = so2().matrix();
    Transformation res;
    res.setIdentity();
    res.template topLeftCorner<2,2>() = R;
    res(0,2) =  translation()[1];
    res(1,2) = -translation()[0];
    return res;
  }

  /**
   * \returns copy of instance casted to NewScalarType
   */
  template<typename NewScalarType>
  inline SE2Group<NewScalarType> cast() const {
    return
        SE2Group<NewScalarType>(so2().template cast<NewScalarType>(),
                                translation().template cast<NewScalarType>() );
  }

  /**
   * \brief Fast group multiplication
   *
   * This method is a fast version of operator*=(), since it does not perform
   * normalization. It is up to the user to call normalize() once in a while.
   *
   * \see operator*=()
   */
  inline
  void fastMultiply(const SE2Group<Scalar>& other) {
    translation() += so2()*(other.translation());
    so2().fastMultiply(other.so2());
  }

  /**
   * \returns Group inverse of instance
   */
  inline
  const SE2Group<Scalar> inverse() const {
    const SO2Group<Scalar> invR = so2().inverse();
    return SE2Group<Scalar>(invR, invR*(translation()
                                        *static_cast<Scalar>(-1) ) );
  }

  /**
   * \brief Logarithmic map
   *
   * \returns tangent space representation
   *          (translational part and rotation angle) of instance
   *
   * \see  log().
   */
  inline
  const Tangent log() const {
    return log(*this);
  }

  /**
   * \brief Normalize SO2 element
   *
   * It re-normalizes the SO2 element. This method only needs to
   * be called in conjunction with fastMultiply() or data() write access.
   */
  inline
  void normalize() {
    so2().normalize();
  }

  /**
   * \returns 3x3 matrix representation of instance
   */
  inline
  const Transformation matrix() const {
    Transformation homogenious_matrix;
    homogenious_matrix.setIdentity();
    homogenious_matrix.block(0,0,2,2) = rotationMatrix();
    homogenious_matrix.col(2).head(2) = translation();
    return homogenious_matrix;
  }

  /**
   * \returns 2x3 matrix representation of instance
   *
   * It returns the three first row of matrix().
   */
  inline
  const Matrix<Scalar,2,3> matrix2x3() const {
    Matrix<Scalar,2,3> matrix;
    matrix.block(0,0,2,2) = rotationMatrix();
    matrix.col(2) = translation();
    return matrix;
  }

  /**
   * \brief Assignment operator
   */
  template<typename OtherDerived> inline
  SE2GroupBase<Derived>& operator= (const SE2GroupBase<OtherDerived> & other) {
    so2() = other.so2();
    translation() = other.translation();
    return *this;
  }

  /**
   * \brief Group multiplication
   * \see operator*=()
   */
  inline
  const SE2Group<Scalar> operator*(const SE2Group<Scalar>& other) const {
    SE2Group<Scalar> result(*this);
    result *= other;
    return result;
  }

  /**
   * \brief Group action on \f$ \mathbf{R}^2 \f$
   *
   * \param p point \f$p \in \mathbf{R}^2 \f$
   * \returns point \f$p' \in \mathbf{R}^2 \f$,
   *          rotated and translated version of \f$p\f$
   *
   * This function rotates and translates point \f$ p \f$
   * in \f$ \mathbf{R}^2 \f$ by the SE2 transformation \f$R,t\f$
   * (=rotation matrix, translation vector): \f$ p' = R\cdot p + t \f$.
   */
  inline
  const Point operator*(const Point & p) const {
    return so2()*p + translation();
  }

  /**
   * \brief In-place group multiplication
   *
   * \see fastMultiply()
   * \see operator*()
   */
  inline
  void operator*=(const SE2Group<Scalar>& other) {
    fastMultiply(other);
    normalize();
  }


  /**
   * \returns Rotation matrix
   */
  inline
  const Matrix<Scalar,2,2> rotationMatrix() const {
    return so2().matrix();
  }

  /**
   * \brief Setter of internal unit complex number representation
   *
   * \param complex
   * \pre   the complex number must not be zero
   *
   * The complex number is normalized to unit length.
   */
  inline
  void setComplex(const Matrix<Scalar,2,1> & complex) {
    return so2().setComplex(complex);
  }

  /**
   * \brief Setter of unit complex number using rotation matrix
   *
   * \param R a 2x2 matrix
   * \pre     the 2x2 matrix should be orthogonal and have a determinant of 1
   */
  inline
  void setRotationMatrix(const Matrix<Scalar,2,2> & R) {
    so2().setComplex(static_cast<Scalar>(0.5)*(R(0,0)+R(1,1)),
                     static_cast<Scalar>(0.5)*(R(1,0)-R(0,1)));
  }

  /**
   * \brief Mutator of SO2 group
   */
  EIGEN_STRONG_INLINE
  SO2Reference so2() {
    return static_cast<Derived*>(this)->so2();
  }

  /**
   * \brief Accessor of SO2 group
   */
  EIGEN_STRONG_INLINE
  ConstSO2Reference so2() const {
    return static_cast<const Derived*>(this)->so2();
  }

  /**
   * \brief Mutator of translation vector
   */
  EIGEN_STRONG_INLINE
  TranslationReference translation() {
    return static_cast<Derived*>(this)->translation();
  }

  /**
   * \brief Accessor of translation vector
   */
  EIGEN_STRONG_INLINE
  ConstTranslationReference translation() const {
    return static_cast<const Derived*>(this)->translation();
  }

  /**
   * \brief Accessor of unit complex number
   *
   * No direct write access is given to ensure the complex number stays
   * normalized.
   */
  inline
  typename internal::traits<Derived>::SO2Type::ConstComplexReference
  unit_complex() const {
    return so2().unit_complex();
  }

  ////////////////////////////////////////////////////////////////////////////
  // public static functions
  ////////////////////////////////////////////////////////////////////////////

  /**
   * \param   b 3-vector representation of Lie algebra element
   * \returns   derivative of Lie bracket
   *
   * This function returns \f$ \frac{\partial}{\partial a} [a, b]_{se2} \f$
   * with \f$ [a, b]_{se2} \f$ being the lieBracket() of the Lie algebra se2.
   *
   * \see lieBracket()
   */
  inline static
  const Transformation d_lieBracketab_by_d_a(const Tangent & b) {
    static const Scalar zero = static_cast<Scalar>(0);
    Matrix<Scalar,2,1> upsilon2 = b.template head<2>();
    Scalar theta2 = b[2];

    Transformation res;
    res <<    zero, theta2, -upsilon2[1]
        ,  -theta2,   zero,  upsilon2[0]
        ,     zero,   zero,         zero;
    return res;
  }

  /**
   * \brief Group exponential
   *
   * \param a tangent space element (3-vector)
   * \returns corresponding element of the group SE2
   *
   * The first two components of \f$ a \f$ represent the translational
   * part \f$ \upsilon \f$ in the tangent space of SE2, while the last
   * components of \f$ a \f$ is the rotation angle \f$ \theta \f$.
   *
   * To be more specific, this function computes \f$ \exp(\widehat{a}) \f$
   * with \f$ \exp(\cdot) \f$ being the matrix exponential
   * and \f$ \widehat{\cdot} \f$ the hat()-operator of SE2.
   *
   * \see hat()
   * \see log()
   */
  inline static
  const SE2Group<Scalar> exp(const Tangent & a) {
    Scalar theta = a[2];
    const SO2Group<Scalar> & so2 = SO2Group<Scalar>::exp(theta);
    Scalar sin_theta_by_theta;
    Scalar one_minus_cos_theta_by_theta;

    if(std::abs(theta)<SophusConstants<Scalar>::epsilon()) {
      Scalar theta_sq = theta*theta;
      sin_theta_by_theta
          = static_cast<Scalar>(1.) - static_cast<Scalar>(1./6.)*theta_sq;
      one_minus_cos_theta_by_theta
          = static_cast<Scalar>(0.5)*theta
          - static_cast<Scalar>(1./24.)*theta*theta_sq;
    } else {
      sin_theta_by_theta = so2.unit_complex().y()/theta;
      one_minus_cos_theta_by_theta
          = (static_cast<Scalar>(1.) - so2.unit_complex().x())/theta;
    }
    Matrix<Scalar,2,1> trans
        (sin_theta_by_theta*a[0] - one_minus_cos_theta_by_theta*a[1],
        one_minus_cos_theta_by_theta * a[0]+sin_theta_by_theta*a[1]);
    return SE2Group<Scalar>(so2, trans);
  }

  /**
   * \brief Generators
   *
   * \pre \f$ i \in \{0,1,2\} \f$
   * \returns \f$ i \f$th generator \f$ G_i \f$ of SE2
   *
   * The infinitesimal generators of SE2 are: \f[
   *        G_0 = \left( \begin{array}{ccc}
   *                          0&  0&  1\\
   *                          0&  0&  0\\
   *                          0&  0&  0\\
   *                     \end{array} \right),
   *        G_1 = \left( \begin{array}{cccc}
   *                          0&  0&  0\\
   *                          0&  0&  1\\
   *                          0&  0&  0\\
   *                     \end{array} \right),
   *        G_2 = \left( \begin{array}{cccc}
   *                          0&  0&  0&\\
   *                          0&  0& -1&\\
   *                          0&  1&  0&\\
   *                     \end{array} \right),
   * \f]
   * \see hat()
   */
  inline static
  const Transformation generator(int i) {
    if (i<0 || i>2) {
      throw SophusException("i is not in range [0,2].");
    }
    Tangent e;
    e.setZero();
    e[i] = static_cast<Scalar>(1);
    return hat(e);
  }

  /**
   * \brief hat-operator
   *
   * \param omega 3-vector representation of Lie algebra element
   * \returns     3x3-matrix representatin of Lie algebra element
   *
   * Formally, the hat-operator of SE2 is defined
   * as \f$ \widehat{\cdot}: \mathbf{R}^3 \rightarrow \mathbf{R}^{2\times 2},
   * \quad \widehat{\omega} = \sum_{i=0}^2 G_i \omega_i \f$
   * with \f$ G_i \f$ being the ith infinitesial generator().
   *
   * \see generator()
   * \see vee()
   */
  inline static
  const Transformation hat(const Tangent & v) {
    Transformation Omega;
    Omega.setZero();
    Omega.template topLeftCorner<2,2>() = SO2Group<Scalar>::hat(v[2]);
    Omega.col(2).template head<2>() = v.template head<2>();
    return Omega;
  }

  /**
   * \brief Lie bracket
   *
   * \param a 3-vector representation of Lie algebra element
   * \param b 3-vector representation of Lie algebra element
   * \returns 3-vector representation of Lie algebra element
   *
   * It computes the bracket of SE2. To be more specific, it
   * computes \f$ [a, b]_{se2}
   * := [\widehat{a_1}, \widehat{b_2}]^\vee \f$
   * with \f$ [A,B] = AB-BA \f$ being the matrix
   * commutator, \f$ \widehat{\cdot} \f$ the
   * hat()-operator and \f$ (\cdot)^\vee \f$ the vee()-operator of SE2.
   *
   * \see hat()
   * \see vee()
   */
  inline static
  const Tangent lieBracket(const Tangent & a,
                           const Tangent & b) {
    Matrix<Scalar,2,1> upsilon1 = a.template head<2>();
    Matrix<Scalar,2,1> upsilon2 = b.template head<2>();
    Scalar theta1 = a[2];
    Scalar theta2 = b[2];

    return Tangent(-theta1*upsilon2[1] + theta2*upsilon1[1],
        theta1*upsilon2[0] - theta2*upsilon1[0],
        static_cast<Scalar>(0));
  }

  /**
   * \brief Logarithmic map
   *
   * \param other element of the group SE2
   * \returns     corresponding tangent space element
   *              (translational part \f$ \upsilon \f$
   *               and rotation vector \f$ \omega \f$)
   *
   * Computes the logarithmic, the inverse of the group exponential.
   * To be specific, this function computes \f$ \log({\cdot})^\vee \f$
   * with \f$ \vee(\cdot) \f$ being the matrix logarithm
   * and \f$ \vee{\cdot} \f$ the vee()-operator of SE2.
   *
   * \see exp()
   * \see vee()
   */
  inline static
  const Tangent log(const SE2Group<Scalar> & other) {
    Tangent upsilon_theta;
    const SO2Group<Scalar> & so2 = other.so2();
    Scalar theta = SO2Group<Scalar>::log(so2);
    upsilon_theta[2] = theta;
    Scalar halftheta = static_cast<Scalar>(0.5)*theta;
    Scalar halftheta_by_tan_of_halftheta;

    const Matrix<Scalar,2,1> & z = so2.unit_complex();
    Scalar real_minus_one = z.x()-static_cast<Scalar>(1.);
    if (std::abs(real_minus_one)<SophusConstants<Scalar>::epsilon()) {
      halftheta_by_tan_of_halftheta
          = static_cast<Scalar>(1.)
          - static_cast<Scalar>(1./12)*theta*theta;
    } else {
      halftheta_by_tan_of_halftheta
          = -(halftheta*z.y())/(real_minus_one);
    }
    Matrix<Scalar,2,2> V_inv;
    V_inv <<  halftheta_by_tan_of_halftheta,                      halftheta
        ,                        -halftheta,  halftheta_by_tan_of_halftheta;
    upsilon_theta.template head<2>() = V_inv*other.translation();
    return upsilon_theta;
  }

  /**
   * \brief vee-operator
   *
   * \param Omega 3x3-matrix representation of Lie algebra element
   * \returns     3-vector representatin of Lie algebra element
   *
   * This is the inverse of the hat()-operator.
   *
   * \see hat()
   */
  inline static
  const Tangent vee(const Transformation & Omega) {
    Tangent upsilon_omega;
    upsilon_omega.template head<2>() = Omega.col(2).template head<2>();
    upsilon_omega[2]
        = SO2Group<Scalar>::vee(Omega.template topLeftCorner<2,2>());
    return upsilon_omega;
  }
};

/**
 * \brief SE2 default type - Constructors and default storage for SE2 Type
 */
template<typename _Scalar, int _Options>
class SE2Group : public SE2GroupBase<SE2Group<_Scalar,_Options> > {
  typedef SE2GroupBase<SE2Group<_Scalar,_Options> > Base;

public:
  /** \brief scalar type */
  typedef typename internal::traits<SE2Group<_Scalar,_Options> >
  ::Scalar Scalar;
  /** \brief translation reference type */
  typedef typename internal::traits<SE2Group<_Scalar,_Options> >
  ::TranslationType & TranslationReference;
  typedef const typename internal::traits<SE2Group<_Scalar,_Options> >
  ::TranslationType & ConstTranslationReference;
  /** \brief SO2 reference type */
  typedef typename internal::traits<SE2Group<_Scalar,_Options> >
  ::SO2Type & SO2Reference;
  /** \brief SO2 const reference type */
  typedef const typename internal::traits<SE2Group<_Scalar,_Options> >
  ::SO2Type & ConstSO2Reference;

  /** \brief degree of freedom of group */
  static const int DoF = Base::DoF;
  /** \brief number of internal parameters used */
  static const int num_parameters = Base::num_parameters;
  /** \brief group transformations are NxN matrices */
  static const int N = Base::N;
  /** \brief group transfomation type */
  typedef typename Base::Transformation Transformation;
  /** \brief point type */
  typedef typename Base::Point Point;
  /** \brief tangent vector type */
  typedef typename Base::Tangent Tangent;
  /** \brief adjoint transformation type */
  typedef typename Base::Adjoint Adjoint;


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * \brief Default constructor
   *
   * Initialize Complex to identity rotation and translation to zero.
   */
  inline
  SE2Group()
    : translation_( Matrix<Scalar,2,1>::Zero() )
  {
  }

  /**
   * \brief Copy constructor
   */
  template<typename OtherDerived> inline
  SE2Group(const SE2GroupBase<OtherDerived> & other)
    : so2_(other.so2()), translation_(other.translation()) {
  }

  /**
   * \brief Constructor from SO2 and translation vector
   */
  template<typename OtherDerived> inline
  SE2Group(const SO2GroupBase<OtherDerived> & so2,
           const Point & translation)
    : so2_(so2), translation_(translation) {
  }

  /**
   * \brief Constructor from rotation matrix and translation vector
   *
   * \pre rotation matrix need to be orthogonal with determinant of 1
   */
  inline
  SE2Group(const typename SO2Group<Scalar>::Transformation & rotation_matrix,
           const Point & translation)
    : so2_(rotation_matrix), translation_(translation) {
  }

  /**
   * \brief Constructor from rotation angle and translation vector
   */
  inline
  SE2Group(const Scalar & theta,
           const Point & translation)
    : so2_(theta), translation_(translation) {
  }

  /**
   * \brief Constructor from complex number and translation vector
   *
   * \pre complex must not be zero
   */
  inline
  SE2Group(const std::complex<Scalar> & complex,
           const Point & translation)
    : so2_(complex), translation_(translation) {
  }

  /**
   * \brief Constructor from 3x3 matrix
   *
   * \pre 2x2 sub-matrix need to be orthogonal with determinant of 1
   */
  inline explicit
  SE2Group(const Transformation & T)
    : so2_(T.template topLeftCorner<2,2>()),
      translation_(T.template block<2,1>(0,2)) {
  }

  /**
   * \returns pointer to internal data
   *
   * This provides unsafe read/write access to internal data. SE2 is represented
   * by a pair of an SO2 element (two parameters) and a translation vector (two
   * parameters). The user needs to take care of that the complex
   * stays normalized.
   *
   * /see normalize()
   */
  EIGEN_STRONG_INLINE
  Scalar* data() {
    // so2_ and translation_ are layed out sequentially with no padding
    return so2_.data();
  }

  /**
   * \returns const pointer to internal data
   *
   * Const version of data().
   */
  EIGEN_STRONG_INLINE
  const Scalar* data() const {
    // so2_ and translation_ are layed out sequentially with no padding
    return so2_.data();
  }

  /**
   * \brief Accessor of SO2
   */
  EIGEN_STRONG_INLINE
  SO2Reference so2() {
    return so2_;
  }

  /**
   * \brief Mutator of SO2
   */
  EIGEN_STRONG_INLINE
  ConstSO2Reference so2() const {
    return so2_;
  }

  /**
   * \brief Mutator of translation vector
   */
  EIGEN_STRONG_INLINE
  TranslationReference translation() {
    return translation_;
  }

  /**
   * \brief Accessor of translation vector
   */
  EIGEN_STRONG_INLINE
  ConstTranslationReference translation() const {
    return translation_;
  }

protected:
  Sophus::SO2Group<Scalar> so2_;
  Matrix<Scalar,2,1> translation_;
};


} // end namespace


namespace Eigen {
/**
 * \brief Specialisation of Eigen::Map for SE2GroupBase
 *
 * Allows us to wrap SE2 Objects around POD array
 * (e.g. external c style complex)
 */
template<typename _Scalar, int _Options>
class Map<Sophus::SE2Group<_Scalar>, _Options>
    : public Sophus::SE2GroupBase<Map<Sophus::SE2Group<_Scalar>, _Options> >
{
  typedef Sophus::SE2GroupBase<Map<Sophus::SE2Group<_Scalar>, _Options> > Base;

public:
  /** \brief scalar type */
  typedef typename internal::traits<Map>::Scalar Scalar;
  /** \brief translation reference type */
  typedef typename internal::traits<Map>::TranslationType &
  TranslationReference;
  /** \brief translation reference type */
  typedef const typename internal::traits<Map>::TranslationType &
  ConstTranslationReference;
  /** \brief SO2 reference type */
  typedef typename internal::traits<Map>::SO2Type & SO2Reference;
  /** \brief SO2 const reference type */
  typedef const typename internal::traits<Map>::SO2Type & ConstSO2Reference;

  /** \brief degree of freedom of group */
  static const int DoF = Base::DoF;
  /** \brief number of internal parameters used */
  static const int num_parameters = Base::num_parameters;
  /** \brief group transformations are NxN matrices */
  static const int N = Base::N;
  /** \brief group transfomation type */
  typedef typename Base::Transformation Transformation;
  /** \brief point type */
  typedef typename Base::Point Point;
  /** \brief tangent vector type */
  typedef typename Base::Tangent Tangent;
  /** \brief adjoint transformation type */
  typedef typename Base::Adjoint Adjoint;

  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)
  using Base::operator*=;
  using Base::operator*;

  EIGEN_STRONG_INLINE
  Map(Scalar* coeffs)
    : so2_(coeffs),
      translation_(coeffs+Sophus::SO2Group<Scalar>::num_parameters) {
  }

  /**
   * \brief Mutator of SO2
   */
  EIGEN_STRONG_INLINE
  SO2Reference so2() {
    return so2_;
  }

  /**
   * \brief Accessor of SO2
   */
  EIGEN_STRONG_INLINE
  ConstSO2Reference so2() const {
    return so2_;
  }

  /**
   * \brief Mutator of translation vector
   */
  EIGEN_STRONG_INLINE
  TranslationReference translation() {
    return translation_;
  }

  /**
   * \brief Accessor of translation vector
   */
  EIGEN_STRONG_INLINE
  ConstTranslationReference translation() const {
    return translation_;
  }

protected:
  Map<Sophus::SO2Group<Scalar>,_Options> so2_;
  Map<Matrix<Scalar,2,1>,_Options> translation_;
};

/**
 * \brief Specialisation of Eigen::Map for const SE2GroupBase
 *
 * Allows us to wrap SE2 Objects around POD array
 * (e.g. external c style complex)
 */
template<typename _Scalar, int _Options>
class Map<const Sophus::SE2Group<_Scalar>, _Options>
    : public Sophus::SE2GroupBase<
    Map<const Sophus::SE2Group<_Scalar>, _Options> > {
  typedef Sophus::SE2GroupBase<Map<const Sophus::SE2Group<_Scalar>, _Options> >
  Base;

public:
  /** \brief scalar type */
  typedef typename internal::traits<Map>::Scalar Scalar;
  /** \brief translation reference type */
  typedef const typename internal::traits<Map>::TranslationType &
  ConstTranslationReference;
  /** \brief SO2 const reference type */
  typedef const typename internal::traits<Map>::SO2Type & ConstSO2Reference;

  /** \brief degree of freedom of group */
  static const int DoF = Base::DoF;
  /** \brief number of internal parameters used */
  static const int num_parameters = Base::num_parameters;
  /** \brief group transformations are NxN matrices */
  static const int N = Base::N;
  /** \brief group transfomation type */
  typedef typename Base::Transformation Transformation;
  /** \brief point type */
  typedef typename Base::Point Point;
  /** \brief tangent vector type */
  typedef typename Base::Tangent Tangent;
  /** \brief adjoint transformation type */
  typedef typename Base::Adjoint Adjoint;

  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)
  using Base::operator*=;
  using Base::operator*;

  EIGEN_STRONG_INLINE
  Map(const Scalar* coeffs)
    : so2_(coeffs),
      translation_(coeffs+Sophus::SO2Group<Scalar>::num_parameters) {
  }

  EIGEN_STRONG_INLINE
  Map(const Scalar* trans_coeffs, const Scalar* rot_coeffs)
    : translation_(trans_coeffs), so2_(rot_coeffs){
  }

  /**
   * \brief Accessor of SO2
   */
  EIGEN_STRONG_INLINE
  ConstSO2Reference so2() const {
    return so2_;
  }

  /**
   * \brief Accessor of translation vector
   */
  EIGEN_STRONG_INLINE
  ConstTranslationReference translation() const {
    return translation_;
  }

protected:
  const Map<const Sophus::SO2Group<Scalar>,_Options> so2_;
  const Map<const Matrix<Scalar,2,1>,_Options> translation_;
};

}

#endif
