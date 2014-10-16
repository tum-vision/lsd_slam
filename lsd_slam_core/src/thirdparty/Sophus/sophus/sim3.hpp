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

#ifndef SOPHUS_SIM3_HPP
#define SOPHUS_SIM3_HPP

#include "rxso3.hpp"

////////////////////////////////////////////////////////////////////////////
// Forward Declarations / typedefs
////////////////////////////////////////////////////////////////////////////

namespace Sophus {
template<typename _Scalar, int _Options=0> class Sim3Group;
typedef Sim3Group<double> Sim3 EIGEN_DEPRECATED;
typedef Sim3Group<double> Sim3d; /**< double precision Sim3 */
typedef Sim3Group<float> Sim3f;  /**< single precision Sim3 */
typedef Matrix<double,7,1> Vector7d;
typedef Matrix<double,7,7> Matrix7d;
typedef Matrix<float,7,1> Vector7f;
typedef Matrix<float,7,7> Matrix7f;
}

////////////////////////////////////////////////////////////////////////////
// Eigen Traits (For querying derived types in CRTP hierarchy)
////////////////////////////////////////////////////////////////////////////

namespace Eigen {
namespace internal {

template<typename _Scalar, int _Options>
struct traits<Sophus::Sim3Group<_Scalar,_Options> > {
  typedef _Scalar Scalar;
  typedef Matrix<Scalar,3,1> TranslationType;
  typedef Sophus::RxSO3Group<Scalar> RxSO3Type;
};

template<typename _Scalar, int _Options>
struct traits<Map<Sophus::Sim3Group<_Scalar>, _Options> >
    : traits<Sophus::Sim3Group<_Scalar, _Options> > {
  typedef _Scalar Scalar;
  typedef Map<Matrix<Scalar,3,1>,_Options> TranslationType;
  typedef Map<Sophus::RxSO3Group<Scalar>,_Options> RxSO3Type;
};

template<typename _Scalar, int _Options>
struct traits<Map<const Sophus::Sim3Group<_Scalar>, _Options> >
    : traits<const Sophus::Sim3Group<_Scalar, _Options> > {
  typedef _Scalar Scalar;
  typedef Map<const Matrix<Scalar,3,1>,_Options> TranslationType;
  typedef Map<const Sophus::RxSO3Group<Scalar>,_Options> RxSO3Type;
};

}
}

namespace Sophus {
using namespace Eigen;
using namespace std;

/**
 * \brief Sim3 base type - implements Sim3 class but is storage agnostic
 *
 * [add more detailed description/tutorial]
 */
template<typename Derived>
class Sim3GroupBase {
public:
  /** \brief scalar type */
  typedef typename internal::traits<Derived>::Scalar Scalar;
  /** \brief translation reference type */
  typedef typename internal::traits<Derived>::TranslationType &
  TranslationReference;
  /** \brief translation const reference type */
  typedef const typename internal::traits<Derived>::TranslationType &
  ConstTranslationReference;
  /** \brief RxSO3 reference type */
  typedef typename internal::traits<Derived>::RxSO3Type &
  RxSO3Reference;
  /** \brief RxSO3 const reference type */
  typedef const typename internal::traits<Derived>::RxSO3Type &
  ConstRxSO3Reference;


  /** \brief degree of freedom of group
    *        (three for translation, three for rotation, one for scale) */
  static const int DoF = 7;
  /** \brief number of internal parameters used
   *         (quaternion for rotation and scale + translation 3-vector) */
  static const int num_parameters = 7;
  /** \brief group transformations are NxN matrices */
  static const int N = 4;
  /** \brief group transfomation type */
  typedef Matrix<Scalar,N,N> Transformation;
  /** \brief point type */
  typedef Matrix<Scalar,3,1> Point;
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
    const Matrix<Scalar,3,3> & R = rxso3().rotationMatrix();
    Adjoint res;
    res.setZero();
    res.block(0,0,3,3) = scale()*R;
    res.block(0,3,3,3) = SO3Group<Scalar>::hat(translation())*R;
    res.block(0,6,3,1) = -translation();
    res.block(3,3,3,3) = R;
    res(6,6) = 1;
    return res;
  }

  /**
   * \returns copy of instance casted to NewScalarType
   */
  template<typename NewScalarType>
  inline Sim3Group<NewScalarType> cast() const {
    return
        Sim3Group<NewScalarType>(rxso3().template cast<NewScalarType>(),
                                 translation().template cast<NewScalarType>() );
  }

  /**
   * \brief In-place group multiplication
   *
   * Same as operator*=() for Sim3.
   *
   * \see operator*()
   */
  inline
  void fastMultiply(const Sim3Group<Scalar>& other) {
    translation() += (rxso3() * other.translation());
    rxso3() *= other.rxso3();
  }

  /**
   * \returns Group inverse of instance
   */
  inline
  const Sim3Group<Scalar> inverse() const {
    const RxSO3Group<Scalar> invR = rxso3().inverse();
    return Sim3Group<Scalar>(invR, invR*(translation()
                                         *static_cast<Scalar>(-1) ) );
  }

  /**
   * \brief Logarithmic map
   *
   * \returns tangent space representation
   *          (translational part and rotation vector) of instance
   *
   * \see  log().
   */
  inline
  const Tangent log() const {
    return log(*this);
  }

  /**
   * \returns 4x4 matrix representation of instance
   */
  inline
  const Transformation matrix() const {
    Transformation homogenious_matrix;
    homogenious_matrix.setIdentity();
    homogenious_matrix.block(0,0,3,3) = rxso3().matrix();
    homogenious_matrix.col(3).head(3) = translation();
    return homogenious_matrix;
  }

  /**
   * \returns 3x4 matrix representation of instance
   *
   * It returns the three first row of matrix().
   */
  inline
  const Matrix<Scalar,3,4> matrix3x4() const {
    Matrix<Scalar,3,4> matrix;
    matrix.block(0,0,3,3) = rxso3().matrix();
    matrix.col(3) = translation();
    return matrix;
  }

  /**
   * \brief Assignment operator
   */
  template<typename OtherDerived> inline
  Sim3GroupBase<Derived>& operator=
  (const Sim3GroupBase<OtherDerived> & other) {
    rxso3() = other.rxso3();
    translation() = other.translation();
    return *this;
  }

  /**
   * \brief Group multiplication
   * \see operator*=()
   */
  inline
  const Sim3Group<Scalar> operator*(const Sim3Group<Scalar>& other) const {
    Sim3Group<Scalar> result(*this);
    result *= other;
    return result;
  }

  /**
   * \brief Group action on \f$ \mathbf{R}^3 \f$
   *
   * \param p point \f$p \in \mathbf{R}^3 \f$
   * \returns point \f$p' \in \mathbf{R}^3 \f$,
   *          rotated, scaled and translated version of \f$p\f$
   *
   * This function scales, rotates and translates point \f$ p \f$
   * in \f$ \mathbf{R}^3 \f$ by the Sim3 transformation \f$sR,t\f$
   * (=scaled rotation matrix, translation vector): \f$ p' = sR\cdot p + t \f$.
   */
  inline
  const Point operator*(const Point & p) const {
    return rxso3()*p + translation();
  }

  /**
   * \brief In-place group multiplication
   *
   * \see operator*()
   */
  inline
  void operator*=(const Sim3Group<Scalar>& other) {
    translation() += (rxso3() * other.translation());
    rxso3() *= other.rxso3();
  }

  /**
   * \brief Mutator of quaternion
   */
  inline
  typename internal::traits<Derived>::RxSO3Type::QuaternionReference
  quaternion() {
    return rxso3().quaternion();
  }

  /**
   * \brief Accessor of quaternion
   */
  inline
  typename internal::traits<Derived>::RxSO3Type::ConstQuaternionReference
  quaternion() const {
    return rxso3().quaternion();
  }

  /**
   * \returns Rotation matrix
   *
   * deprecated: use rotationMatrix() instead.
   */
  inline
  EIGEN_DEPRECATED const Transformation rotation_matrix() const {
    return rxso3().rotationMatrix();
  }

  /**
   * \returns Rotation matrix
   */
  inline
  const Matrix<Scalar,3,3> rotationMatrix() const {
    return rxso3().rotationMatrix();
  }

  /**
   * \brief Mutator of RxSO3 group
   */
  EIGEN_STRONG_INLINE
  RxSO3Reference rxso3() {
    return static_cast<Derived*>(this)->rxso3();
  }

  /**
   * \brief Accessor of RxSO3 group
   */
  EIGEN_STRONG_INLINE
  ConstRxSO3Reference rxso3() const {
    return static_cast<const Derived*>(this)->rxso3();
  }

  /**
   * \returns scale
   */
  EIGEN_STRONG_INLINE
  const Scalar scale() const {
    return rxso3().scale();
  }

  /**
   * \brief Setter of quaternion using rotation matrix, leaves scale untouched
   *
   * \param R a 3x3 rotation matrix
   * \pre       the 3x3 matrix should be orthogonal and have a determinant of 1
   */
  inline
  void setRotationMatrix
  (const Matrix<Scalar,3,3> & R) {
    rxso3().setRotationMatrix(R);
  }

  /**
   * \brief Scale setter
   */
  EIGEN_STRONG_INLINE
  void setScale(const Scalar & scale) {
    rxso3().setScale(scale);
  }

  /**
   * \brief Setter of quaternion using scaled rotation matrix
   *
   * \param sR a 3x3 scaled rotation matrix
   * \pre        the 3x3 matrix should be "scaled orthogonal"
   *             and have a positive determinant
   */
  inline
  void setScaledRotationMatrix
  (const Matrix<Scalar,3,3> & sR) {
    rxso3().setScaledRotationMatrix(sR);
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

  ////////////////////////////////////////////////////////////////////////////
  // public static functions
  ////////////////////////////////////////////////////////////////////////////

  /**
   * \param   b 7-vector representation of Lie algebra element
   * \returns   derivative of Lie bracket
   *
   * This function returns \f$ \frac{\partial}{\partial a} [a, b]_{sim3} \f$
   * with \f$ [a, b]_{sim3} \f$ being the lieBracket() of the Lie algebra sim3.
   *
   * \see lieBracket()
   */
  inline static
  const Adjoint d_lieBracketab_by_d_a(const Tangent & b) {
    const Matrix<Scalar,3,1> & upsilon2 = b.template head<3>();
    const Matrix<Scalar,3,1> & omega2 = b.template segment<3>(3);
    Scalar sigma2 = b[6];

    Adjoint res;
    res.setZero();
    res.template topLeftCorner<3,3>()
        = -SO3::hat(omega2)-sigma2*Matrix3d::Identity();
    res.template block<3,3>(0,3) = -SO3::hat(upsilon2);
    res.template topRightCorner<3,1>() = upsilon2;
    res.template block<3,3>(3,3) = -SO3::hat(omega2);
    return res;
  }

  /**
   * \brief Group exponential
   *
   * \param a tangent space element (7-vector)
   * \returns corresponding element of the group Sim3
   *
   * The first three components of \f$ a \f$ represent the translational
   * part \f$ \upsilon \f$ in the tangent space of Sim3, while the last three
   * components of \f$ a \f$ represents the rotation vector \f$ \omega \f$.
   *
   * To be more specific, this function computes \f$ \exp(\widehat{a}) \f$
   * with \f$ \exp(\cdot) \f$ being the matrix exponential
   * and \f$ \widehat{\cdot} \f$ the hat()-operator of Sim3.
   *
   * \see hat()
   * \see log()
   */
  inline static
  const Sim3Group<Scalar> exp(const Tangent & a) {
    const Matrix<Scalar,3,1> & upsilon = a.segment(0,3);
    const Matrix<Scalar,3,1> & omega = a.segment(3,3);
    Scalar sigma = a[6];
    Scalar theta;
    RxSO3Group<Scalar> rxso3
        = RxSO3Group<Scalar>::expAndTheta(a.template tail<4>(), &theta);
    const Matrix<Scalar,3,3> & Omega = SO3Group<Scalar>::hat(omega);
    const Matrix<Scalar,3,3> & W = calcW(theta, sigma, rxso3.scale(), Omega);
    return Sim3Group<Scalar>(rxso3, W*upsilon);
  }

  /**
   * \brief Generators
   *
   * \pre \f$ i \in \{0,1,2,3,4,5,6\} \f$
   * \returns \f$ i \f$th generator \f$ G_i \f$ of Sim3
   *
   * The infinitesimal generators of Sim3 are: \f[
   *        G_0 = \left( \begin{array}{cccc}
   *                          0&  0&  0&  1\\
   *                          0&  0&  0&  0\\
   *                          0&  0&  0&  0\\
   *                          0&  0&  0&  0\\
   *                     \end{array} \right),
   *        G_1 = \left( \begin{array}{cccc}
   *                          0&  0&  0&  0\\
   *                          0&  0&  0&  1\\
   *                          0&  0&  0&  0\\
   *                          0&  0&  0&  0\\
   *                     \end{array} \right),
   *        G_2 = \left( \begin{array}{cccc}
   *                          0&  0&  0&  0\\
   *                          0&  0&  0&  0\\
   *                          0&  0&  0&  1\\
   *                          0&  0&  0&  0\\
   *                     \end{array} \right).
   *        G_3 = \left( \begin{array}{cccc}
   *                          0&  0&  0&  0\\
   *                          0&  0& -1&  0\\
   *                          0&  1&  0&  0\\
   *                          0&  0&  0&  0\\
   *                     \end{array} \right),
   *        G_4 = \left( \begin{array}{cccc}
   *                          0&  0&  1&  0\\
   *                          0&  0&  0&  0\\
   *                         -1&  0&  0&  0\\
   *                          0&  0&  0&  0\\
   *                     \end{array} \right),
   *        G_5 = \left( \begin{array}{cccc}
   *                          0& -1&  0&  0\\
   *                          1&  0&  0&  0\\
   *                          0&  0&  0&  0\\
   *                          0&  0&  0&  0\\
   *                     \end{array} \right),
   *        G_6 = \left( \begin{array}{cccc}
   *                          1&  0&  0&  0\\
   *                          0&  1&  0&  0\\
   *                          0&  0&  1&  0\\
   *                          0&  0&  0&  0\\
   *                     \end{array} \right).
   * \f]
   * \see hat()
   */
  inline static
  const Transformation generator(int i) {
    if (i<0 || i>6) {
      throw SophusException("i is not in range [0,6].");
    }
    Tangent e;
    e.setZero();
    e[i] = static_cast<Scalar>(1);
    return hat(e);
  }

  /**
   * \brief hat-operator
   *
   * \param omega 7-vector representation of Lie algebra element
   * \returns     4x4-matrix representatin of Lie algebra element
   *
   * Formally, the hat-operator of Sim3 is defined
   * as \f$ \widehat{\cdot}: \mathbf{R}^7 \rightarrow \mathbf{R}^{4\times 4},
   * \quad \widehat{\omega} = \sum_{i=0}^5 G_i \omega_i \f$
   * with \f$ G_i \f$ being the ith infinitesial generator().
   *
   * \see generator()
   * \see vee()
   */
  inline static
  const Transformation hat(const Tangent & v) {
    Transformation Omega;
    Omega.template topLeftCorner<3,3>()
        = RxSO3Group<Scalar>::hat(v.template tail<4>());
    Omega.col(3).template head<3>() = v.template head<3>();
    Omega.row(3).setZero();
    return Omega;
  }

  /**
   * \brief Lie bracket
   *
   * \param a 7-vector representation of Lie algebra element
   * \param b 7-vector representation of Lie algebra element
   * \returns 7-vector representation of Lie algebra element
   *
   * It computes the bracket of Sim3. To be more specific, it
   * computes \f$ [a, b]_{sim3}
   * := [\widehat{a}, \widehat{b}]^\vee \f$
   * with \f$ [A,B] = AB-BA \f$ being the matrix
   * commutator, \f$ \widehat{\cdot} \f$ the
   * hat()-operator and \f$ (\cdot)^\vee \f$ the vee()-operator of Sim3.
   *
   * \see hat()
   * \see vee()
   */
  inline static
  const Tangent lieBracket(const Tangent & a,
                           const Tangent & b) {
    const Matrix<Scalar,3,1> & upsilon1 = a.template head<3>();
    const Matrix<Scalar,3,1> & upsilon2 = b.template head<3>();
    const Matrix<Scalar,3,1> & omega1 = a.template segment<3>(3);
    const Matrix<Scalar,3,1> & omega2 = b.template segment<3>(3);
    Scalar sigma1 = a[6];
    Scalar sigma2 = b[6];

    Tangent res;
    res.template head<3>() =
        SO3Group<Scalar>::hat(omega1)*upsilon2
        + SO3Group<Scalar>::hat(upsilon1)*omega2
        + sigma1*upsilon2 - sigma2*upsilon1;
    res.template segment<3>(3) = omega1.cross(omega2);
    res[6] = static_cast<Scalar>(0);

    return res;
  }

  /**
   * \brief Logarithmic map
   *
   * \param other element of the group Sim3
   * \returns     corresponding tangent space element
   *              (translational part \f$ \upsilon \f$
   *               and rotation vector \f$ \omega \f$)
   *
   * Computes the logarithmic, the inverse of the group exponential.
   * To be specific, this function computes \f$ \log({\cdot})^\vee \f$
   * with \f$ \vee(\cdot) \f$ being the matrix logarithm
   * and \f$ \vee{\cdot} \f$ the vee()-operator of Sim3.
   *
   * \see exp()
   * \see vee()
   */
  inline static
  const Tangent log(const Sim3Group<Scalar> & other) {
    Tangent res;
    Scalar theta;
    const Matrix<Scalar,4,1> & omega_sigma
        = RxSO3Group<Scalar>::logAndTheta(other.rxso3(), &theta);
    const Matrix<Scalar,3,1> & omega = omega_sigma.template head<3>();
    Scalar sigma = omega_sigma[3];
    const Matrix<Scalar,3,3> & W
        = calcW(theta, sigma, other.scale(), SO3Group<Scalar>::hat(omega));
    res.segment(0,3) = W.partialPivLu().solve(other.translation());
    res.segment(3,3) = omega;
    res[6] = sigma;
    return res;
  }

  /**
   * \brief vee-operator
   *
   * \param Omega 4x4-matrix representation of Lie algebra element
   * \returns     7-vector representatin of Lie algebra element
   *
   * This is the inverse of the hat()-operator.
   *
   * \see hat()
   */
  inline static
  const Tangent vee(const Transformation & Omega) {
    Tangent upsilon_omega_sigma;
    upsilon_omega_sigma.template head<3>()
        = Omega.col(3).template head<3>();
    upsilon_omega_sigma.template tail<4>()
        = RxSO3Group<Scalar>::vee(Omega.template topLeftCorner<3,3>());
    return upsilon_omega_sigma;
  }

private:
  static
  Matrix<Scalar,3,3> calcW(const Scalar & theta,
                           const Scalar & sigma,
                           const Scalar & scale,
                           const Matrix<Scalar,3,3> & Omega){
    static const Matrix<Scalar,3,3> I
        = Matrix<Scalar,3,3>::Identity();
    static const Scalar one = static_cast<Scalar>(1.);
    static const Scalar half = static_cast<Scalar>(1./2.);
    Matrix<Scalar,3,3> Omega2 = Omega*Omega;

    Scalar A,B,C;
    if (std::abs(sigma)<SophusConstants<Scalar>::epsilon()) {
      C = one;
      if (std::abs(theta)<SophusConstants<Scalar>::epsilon()) {
        A = half;
        B = static_cast<Scalar>(1./6.);
      } else {
        Scalar theta_sq = theta*theta;
        A = (one-std::cos(theta))/theta_sq;
        B = (theta-std::sin(theta))/(theta_sq*theta);
      }
    } else {
      C = (scale-one)/sigma;
      if (std::abs(theta)<SophusConstants<Scalar>::epsilon()) {
        Scalar sigma_sq = sigma*sigma;
        A = ((sigma-one)*scale+one)/sigma_sq;
        B = ((half*sigma*sigma-sigma+one)*scale)/(sigma_sq*sigma);
      } else {
        Scalar theta_sq = theta*theta;
        Scalar a = scale*std::sin(theta);
        Scalar b = scale*std::cos(theta);
        Scalar c = theta_sq+sigma*sigma;
        A = (a*sigma+ (one-b)*theta)/(theta*c);
        B = (C-((b-one)*sigma+a*theta)/(c))*one/(theta_sq);
      }
    }
    return A*Omega + B*Omega2 + C*I;
  }
};

/**
 * \brief Sim3 default type - Constructors and default storage for Sim3 Type
 */
template<typename _Scalar, int _Options>
class Sim3Group : public Sim3GroupBase<Sim3Group<_Scalar,_Options> > {
  typedef Sim3GroupBase<Sim3Group<_Scalar,_Options> > Base;
public:
  /** \brief scalar type */
  typedef typename internal::traits<Sim3Group<_Scalar,_Options> >
  ::Scalar Scalar;
  /** \brief RxSO3 reference type */
  typedef typename internal::traits<Sim3Group<_Scalar,_Options> >
  ::RxSO3Type & RxSO3Reference;
  /** \brief RxSO3 const reference type */
  typedef const typename internal::traits<Sim3Group<_Scalar,_Options> >
  ::RxSO3Type & ConstRxSO3Reference;
  /** \brief translation reference type */
  typedef typename internal::traits<Sim3Group<_Scalar,_Options> >
  ::TranslationType & TranslationReference;
  /** \brief translation const reference type */
  typedef const typename internal::traits<Sim3Group<_Scalar,_Options> >
  ::TranslationType & ConstTranslationReference;

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
   * Initialize Quaternion to identity rotation and translation to zero.
   */
  inline
  Sim3Group()
    : translation_( Matrix<Scalar,3,1>::Zero() )
  {
  }

  /**
   * \brief Copy constructor
   */
  template<typename OtherDerived> inline
  Sim3Group(const Sim3GroupBase<OtherDerived> & other)
    : rxso3_(other.rxso3()), translation_(other.translation()) {
  }

  /**
   * \brief Constructor from RxSO3 and translation vector
   */
  template<typename OtherDerived> inline
  Sim3Group(const RxSO3GroupBase<OtherDerived> & rxso3,
            const Point & translation)
    : rxso3_(rxso3), translation_(translation) {
  }

  /**
   * \brief Constructor from quaternion and translation vector
   *
   * \pre quaternion must not be zero
   */
  inline
  Sim3Group(const Quaternion<Scalar> & quaternion,
            const Point & translation)
    : rxso3_(quaternion), translation_(translation) {
  }

  /**
   * \brief Constructor from 4x4 matrix
   *
   * \pre top-left 3x3 sub-matrix need to be "scaled orthogonal"
   *      with positive determinant of
   */
  inline explicit
  Sim3Group(const Eigen::Matrix<Scalar,4,4>& T)
    : rxso3_(T.template topLeftCorner<3,3>()),
      translation_(T.template block<3,1>(0,3)) {
  }

  /**
   * \returns pointer to internal data
   *
   * This provides unsafe read/write access to internal data. Sim3 is
   * represented by a pair of an RxSO3 element (4 parameters) and translation
   * vector (three parameters).
   *
   * Note: The first three Scalars represent the imaginary parts, while the
   */
  EIGEN_STRONG_INLINE
  Scalar* data() {
    // rxso3_ and translation_ are layed out sequentially with no padding
    return rxso3_.data();
  }

  /**
   * \returns const pointer to internal data
   *
   * Const version of data().
   */
  EIGEN_STRONG_INLINE
  const Scalar* data() const {
    // rxso3_ and translation_ are layed out sequentially with no padding
    return rxso3_.data();
  }

  /**
   * \brief Accessor of RxSO3
   */
  EIGEN_STRONG_INLINE
  RxSO3Reference rxso3() {
    return rxso3_;
  }

  /**
   * \brief Mutator of RxSO3
   */
  EIGEN_STRONG_INLINE
  ConstRxSO3Reference rxso3() const {
    return rxso3_;
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
  Sophus::RxSO3Group<Scalar> rxso3_;
  Matrix<Scalar,3,1> translation_;
};


} // end namespace


namespace Eigen {
/**
 * \brief Specialisation of Eigen::Map for Sim3GroupBase
 *
 * Allows us to wrap Sim3 Objects around POD array
 * (e.g. external c style quaternion)
 */
template<typename _Scalar, int _Options>
class Map<Sophus::Sim3Group<_Scalar>, _Options>
    : public Sophus::Sim3GroupBase<Map<Sophus::Sim3Group<_Scalar>, _Options> > {
  typedef Sophus::Sim3GroupBase<Map<Sophus::Sim3Group<_Scalar>, _Options> >
  Base;

public:
  /** \brief scalar type */
  typedef typename internal::traits<Map>::Scalar Scalar;
  /** \brief translation reference type */
  typedef typename internal::traits<Map>::TranslationType &
  TranslationReference;
  /** \brief translation const reference type */
  typedef const typename internal::traits<Map>::TranslationType &
  ConstTranslationReference;
  /** \brief RxSO3 reference type */
  typedef typename internal::traits<Map>::RxSO3Type &
  RxSO3Reference;
  /** \brief RxSO3 const reference type */
  typedef const typename internal::traits<Map>::RxSO3Type &
  ConstRxSO3Reference;


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
    : rxso3_(coeffs),
      translation_(coeffs+Sophus::RxSO3Group<Scalar>::num_parameters) {
  }

  /**
   * \brief Mutator of RxSO3
   */
  EIGEN_STRONG_INLINE
  RxSO3Reference rxso3() {
    return rxso3_;
  }

  /**
   * \brief Accessor of RxSO3
   */
  EIGEN_STRONG_INLINE
  ConstRxSO3Reference rxso3() const {
    return rxso3_;
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
  Map<Sophus::RxSO3Group<Scalar>,_Options> rxso3_;
  Map<Matrix<Scalar,3,1>,_Options> translation_;
};

/**
 * \brief Specialisation of Eigen::Map for const Sim3GroupBase
 *
 * Allows us to wrap Sim3 Objects around POD array
 * (e.g. external c style quaternion)
 */
template<typename _Scalar, int _Options>
class Map<const Sophus::Sim3Group<_Scalar>, _Options>
    : public Sophus::Sim3GroupBase<
    Map<const Sophus::Sim3Group<_Scalar>, _Options> > {
  typedef Sophus::Sim3GroupBase<
  Map<const Sophus::Sim3Group<_Scalar>, _Options> > Base;

public:
  /** \brief scalar type */
  typedef typename internal::traits<Map>::Scalar Scalar;
  /** \brief translation type */
  typedef const typename internal::traits<Map>::TranslationType &
  ConstTranslationReference;
  /** \brief RxSO3 const reference type */
  typedef const typename internal::traits<Map>::RxSO3Type &
  ConstRxSO3Reference;

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
    : rxso3_(coeffs),
      translation_(coeffs+Sophus::RxSO3Group<Scalar>::num_parameters) {
  }

  EIGEN_STRONG_INLINE
  Map(const Scalar* trans_coeffs, const Scalar* rot_coeffs)
    : translation_(trans_coeffs), rxso3_(rot_coeffs){
  }

  /**
   * \brief Accessor of RxSO3
   */
  EIGEN_STRONG_INLINE
  ConstRxSO3Reference rxso3() const {
    return rxso3_;
  }

  /**
   * \brief Accessor of translation vector
   */
  EIGEN_STRONG_INLINE
  ConstTranslationReference translation() const {
    return translation_;
  }

protected:
  const Map<const Sophus::RxSO3Group<Scalar>,_Options> rxso3_;
  const Map<const Matrix<Scalar,3,1>,_Options> translation_;
};

}

#endif
