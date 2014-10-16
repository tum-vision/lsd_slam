// This file is part of Sophus.
//
// Copyright 2011-2013 Hauke Strasdat
//           2012-2013 Steven Lovegrove
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

#ifndef SOPHUS_SE3_HPP
#define SOPHUS_SE3_HPP

#include "so3.hpp"

////////////////////////////////////////////////////////////////////////////
// Forward Declarations / typedefs
////////////////////////////////////////////////////////////////////////////

namespace Sophus {
template<typename _Scalar, int _Options=0> class SE3Group;
typedef SE3Group<double> SE3 EIGEN_DEPRECATED;
typedef SE3Group<double> SE3d; /**< double precision SE3 */
typedef SE3Group<float> SE3f;  /**< single precision SE3 */
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<float,6,1> Vector6f;
typedef Matrix<float,6,6> Matrix6f;
}

////////////////////////////////////////////////////////////////////////////
// Eigen Traits (For querying derived types in CRTP hierarchy)
////////////////////////////////////////////////////////////////////////////

namespace Eigen {
namespace internal {

template<typename _Scalar, int _Options>
struct traits<Sophus::SE3Group<_Scalar,_Options> > {
  typedef _Scalar Scalar;
  typedef Matrix<Scalar,3,1> TranslationType;
  typedef Sophus::SO3Group<Scalar> SO3Type;
};

template<typename _Scalar, int _Options>
struct traits<Map<Sophus::SE3Group<_Scalar>, _Options> >
    : traits<Sophus::SE3Group<_Scalar, _Options> > {
  typedef _Scalar Scalar;
  typedef Map<Matrix<Scalar,3,1>,_Options> TranslationType;
  typedef Map<Sophus::SO3Group<Scalar>,_Options> SO3Type;
};

template<typename _Scalar, int _Options>
struct traits<Map<const Sophus::SE3Group<_Scalar>, _Options> >
    : traits<const Sophus::SE3Group<_Scalar, _Options> > {
  typedef _Scalar Scalar;
  typedef Map<const Matrix<Scalar,3,1>,_Options> TranslationType;
  typedef Map<const Sophus::SO3Group<Scalar>,_Options> SO3Type;
};

}
}

namespace Sophus {
using namespace Eigen;
using namespace std;

/**
 * \brief SE3 base type - implements SE3 class but is storage agnostic
 *
 * [add more detailed description/tutorial]
 */
template<typename Derived>
class SE3GroupBase {
public:
  /** \brief scalar type */
  typedef typename internal::traits<Derived>::Scalar Scalar;
  /** \brief translation reference type */
  typedef typename internal::traits<Derived>::TranslationType &
  TranslationReference;
  /** \brief translation const reference type */
  typedef const typename internal::traits<Derived>::TranslationType &
  ConstTranslationReference;
  /** \brief SO3 reference type */
  typedef typename internal::traits<Derived>::SO3Type &
  SO3Reference;
  /** \brief SO3 const reference type */
  typedef const typename internal::traits<Derived>::SO3Type &
  ConstSO3Reference;

  /** \brief degree of freedom of group
    *        (three for translation, three for rotation) */
  static const int DoF = 6;
  /** \brief number of internal parameters used
   *         (unit quaternion for rotation + translation 3-vector) */
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
    const Matrix<Scalar,3,3> & R = so3().matrix();
    Adjoint res;
    res.block(0,0,3,3) = R;
    res.block(3,3,3,3) = R;
    res.block(0,3,3,3) = SO3Group<Scalar>::hat(translation())*R;
    res.block(3,0,3,3) = Matrix<Scalar,3,3>::Zero(3,3);
    return res;
  }

  /**
   * \returns copy of instance casted to NewScalarType
   */
  template<typename NewScalarType>
  inline SE3Group<NewScalarType> cast() const {
    return
        SE3Group<NewScalarType>(so3().template cast<NewScalarType>(),
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
  void fastMultiply(const SE3Group<Scalar>& other) {
    translation() += so3()*(other.translation());
    so3().fastMultiply(other.so3());
  }

  /**
   * \returns Group inverse of instance
   */
  inline
  const SE3Group<Scalar> inverse() const {
    const SO3Group<Scalar> invR = so3().inverse();
    return SE3Group<Scalar>(invR, invR*(translation()
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
   * \brief Normalize SO3 element
   *
   * It re-normalizes the SO3 element. This method only needs to
   * be called in conjunction with fastMultiply() or data() write access.
   */
  inline
  void normalize() {
    so3().normalize();
  }

  /**
   * \returns 4x4 matrix representation of instance
   */
  inline
  const Transformation matrix() const {
    Transformation homogenious_matrix;
    homogenious_matrix.setIdentity();
    homogenious_matrix.block(0,0,3,3) = rotationMatrix();
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
    matrix.block(0,0,3,3) = rotationMatrix();
    matrix.col(3) = translation();
    return matrix;
  }

  /**
   * \brief Assignment operator
   */
  template<typename OtherDerived> inline
  SE3GroupBase<Derived>& operator= (const SE3GroupBase<OtherDerived> & other) {
    so3() = other.so3();
    translation() = other.translation();
    return *this;
  }

  /**
   * \brief Group multiplication
   * \see operator*=()
   */
  inline
  const SE3Group<Scalar> operator*(const SE3Group<Scalar>& other) const {
    SE3Group<Scalar> result(*this);
    result *= other;
    return result;
  }

  /**
   * \brief Group action on \f$ \mathbf{R}^3 \f$
   *
   * \param p point \f$p \in \mathbf{R}^3 \f$
   * \returns point \f$p' \in \mathbf{R}^3 \f$,
   *          rotated and translated version of \f$p\f$
   *
   * This function rotates and translates point \f$ p \f$
   * in \f$ \mathbf{R}^3 \f$ by the SE3 transformation \f$R,t\f$
   * (=rotation matrix, translation vector): \f$ p' = R\cdot p + t \f$.
   */
  inline
  const Point operator*(const Point & p) const {
    return so3()*p + translation();
  }

  /**
   * \brief In-place group multiplication
   *
   * \see fastMultiply()
   * \see operator*()
   */
  inline
  void operator*=(const SE3Group<Scalar>& other) {
    fastMultiply(other);
    normalize();
  }


  /**
   * \returns Rotation matrix
   *
   * deprecated: use rotationMatrix() instead.
   */
  typedef Transformation M3_marcos_dont_like_commas;
  inline
  EIGEN_DEPRECATED const M3_marcos_dont_like_commas rotation_matrix() const {
    return so3().matrix();
  }

  /**
   * \returns Rotation matrix
   */
  inline
  const Matrix<Scalar,3,3> rotationMatrix() const {
    return so3().matrix();
  }


  /**
   * \brief Mutator of SO3 group
   */
  EIGEN_STRONG_INLINE
  SO3Reference so3() {
    return static_cast<Derived*>(this)->so3();
  }

  /**
   * \brief Accessor of SO3 group
   */
  EIGEN_STRONG_INLINE
  ConstSO3Reference so3() const {
    return static_cast<const Derived*>(this)->so3();
  }

  /**
   * \brief Setter of internal unit quaternion representation
   *
   * \param quaternion
   * \pre   the quaternion must not be zero
   *
   * The quaternion is normalized to unit length.
   */
  inline
  void setQuaternion(const Quaternion<Scalar> & quat) {
    return so3().setQuaternion(quat);
  }

  /**
   * \brief Setter of unit quaternion using rotation matrix
   *
   * \param rotation_matrix a 3x3 rotation matrix
   * \pre   the 3x3 matrix should be orthogonal and have a determinant of 1
   */
  inline
  void setRotationMatrix
  (const Matrix<Scalar,3,3> & rotation_matrix) {
    so3().setQuaternion(Quaternion<Scalar>(rotation_matrix));
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
   * \brief Accessor of unit quaternion
   *
   * No direct write access is given to ensure the quaternion stays normalized.
   */
  inline
  typename internal::traits<Derived>::SO3Type::ConstQuaternionReference
  unit_quaternion() const {
    return so3().unit_quaternion();
  }

  ////////////////////////////////////////////////////////////////////////////
  // public static functions
  ////////////////////////////////////////////////////////////////////////////

  /**
   * \param   b 6-vector representation of Lie algebra element
   * \returns   derivative of Lie bracket
   *
   * This function returns \f$ \frac{\partial}{\partial a} [a, b]_{se3} \f$
   * with \f$ [a, b]_{se3} \f$ being the lieBracket() of the Lie algebra se3.
   *
   * \see lieBracket()
   */
  inline static
  const Adjoint d_lieBracketab_by_d_a(const Tangent & b) {
    Adjoint res;
    res.setZero();

    const Matrix<Scalar,3,1> & upsilon2 = b.template head<3>();
    const Matrix<Scalar,3,1> & omega2 = b.template tail<3>();

    res.template topLeftCorner<3,3>() = -SO3Group<Scalar>::hat(omega2);
    res.template topRightCorner<3,3>() = -SO3Group<Scalar>::hat(upsilon2);
    res.template bottomRightCorner<3,3>() = -SO3Group<Scalar>::hat(omega2);
    return res;
  }

  /**
   * \brief Group exponential
   *
   * \param a tangent space element (6-vector)
   * \returns corresponding element of the group SE3
   *
   * The first three components of \f$ a \f$ represent the translational
   * part \f$ \upsilon \f$ in the tangent space of SE3, while the last three
   * components of \f$ a \f$ represents the rotation vector \f$ \omega \f$.
   *
   * To be more specific, this function computes \f$ \exp(\widehat{a}) \f$
   * with \f$ \exp(\cdot) \f$ being the matrix exponential
   * and \f$ \widehat{\cdot} \f$ the hat()-operator of SE3.
   *
   * \see hat()
   * \see log()
   */
  inline static
  const SE3Group<Scalar> exp(const Tangent & a) {
    const Matrix<Scalar,3,1> & omega = a.template tail<3>();

    Scalar theta;
    const SO3Group<Scalar> & so3
        = SO3Group<Scalar>::expAndTheta(omega, &theta);

    const Matrix<Scalar,3,3> & Omega = SO3Group<Scalar>::hat(omega);
    const Matrix<Scalar,3,3> & Omega_sq = Omega*Omega;
    Matrix<Scalar,3,3> V;

    if(theta<SophusConstants<Scalar>::epsilon()) {
      V = so3.matrix();
      //Note: That is an accurate expansion!
    } else {
      Scalar theta_sq = theta*theta;
      V = (Matrix<Scalar,3,3>::Identity()
           + (static_cast<Scalar>(1)-std::cos(theta))/(theta_sq)*Omega
           + (theta-std::sin(theta))/(theta_sq*theta)*Omega_sq);
    }
    return SE3Group<Scalar>(so3,V*a.template head<3>());
  }

  /**
   * \brief Generators
   *
   * \pre \f$ i \in \{0,1,2,3,4,5\} \f$
   * \returns \f$ i \f$th generator \f$ G_i \f$ of SE3
   *
   * The infinitesimal generators of SE3 are: \f[
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
   *                     \end{array} \right).
   * \f]
   * \see hat()
   */
  inline static
  const Transformation generator(int i) {
    if (i<0 || i>5) {
      throw SophusException("i is not in range [0,5].");
    }
    Tangent e;
    e.setZero();
    e[i] = static_cast<Scalar>(1);
    return hat(e);
  }

  /**
   * \brief hat-operator
   *
   * \param omega 6-vector representation of Lie algebra element
   * \returns     4x4-matrix representatin of Lie algebra element
   *
   * Formally, the hat-operator of SE3 is defined
   * as \f$ \widehat{\cdot}: \mathbf{R}^6 \rightarrow \mathbf{R}^{4\times 4},
   * \quad \widehat{\omega} = \sum_{i=0}^5 G_i \omega_i \f$
   * with \f$ G_i \f$ being the ith infinitesial generator().
   *
   * \see generator()
   * \see vee()
   */
  inline static
  const Transformation hat(const Tangent & v) {
    Transformation Omega;
    Omega.setZero();
    Omega.template topLeftCorner<3,3>()
        = SO3Group<Scalar>::hat(v.template tail<3>());
    Omega.col(3).template head<3>() = v.template head<3>();
    return Omega;
  }

  /**
   * \brief Lie bracket
   *
   * \param a 6-vector representation of Lie algebra element
   * \param b 6-vector representation of Lie algebra element
   * \returns      6-vector representation of Lie algebra element
   *
   * It computes the bracket of SE3. To be more specific, it
   * computes \f$ [a, b]_{se3}
   * := [\widehat{a}, \widehat{b}]^\vee \f$
   * with \f$ [A,B] = AB-BA \f$ being the matrix
   * commutator, \f$ \widehat{\cdot} \f$ the
   * hat()-operator and \f$ (\cdot)^\vee \f$ the vee()-operator of SE3.
   *
   * \see hat()
   * \see vee()
   */
  inline static
  const Tangent lieBracket(const Tangent & a,
                           const Tangent & b) {
    Matrix<Scalar,3,1> upsilon1 = a.template head<3>();
    Matrix<Scalar,3,1> upsilon2 = b.template head<3>();
    Matrix<Scalar,3,1> omega1 = a.template tail<3>();
    Matrix<Scalar,3,1> omega2 = b.template tail<3>();

    Tangent res;
    res.template head<3>() = omega1.cross(upsilon2) + upsilon1.cross(omega2);
    res.template tail<3>() = omega1.cross(omega2);

    return res;
  }

  /**
   * \brief Logarithmic map
   *
   * \param other element of the group SE3
   * \returns     corresponding tangent space element
   *              (translational part \f$ \upsilon \f$
   *               and rotation vector \f$ \omega \f$)
   *
   * Computes the logarithmic, the inverse of the group exponential.
   * To be specific, this function computes \f$ \log({\cdot})^\vee \f$
   * with \f$ \vee(\cdot) \f$ being the matrix logarithm
   * and \f$ \vee{\cdot} \f$ the vee()-operator of SE3.
   *
   * \see exp()
   * \see vee()
   */
  inline static
  const Tangent log(const SE3Group<Scalar> & se3) {
    Tangent upsilon_omega;
    Scalar theta;
    upsilon_omega.template tail<3>()
        = SO3Group<Scalar>::logAndTheta(se3.so3(), &theta);

    if (std::abs(theta)<SophusConstants<Scalar>::epsilon()) {
      const Matrix<Scalar,3,3> & Omega
          = SO3Group<Scalar>::hat(upsilon_omega.template tail<3>());
      const Matrix<Scalar,3,3> & V_inv =
          Matrix<Scalar,3,3>::Identity() -
          static_cast<Scalar>(0.5)*Omega
          + static_cast<Scalar>(1./12.)*(Omega*Omega);

      upsilon_omega.template head<3>() = V_inv*se3.translation();
    } else {
      const Matrix<Scalar,3,3> & Omega
          = SO3Group<Scalar>::hat(upsilon_omega.template tail<3>());
      const Matrix<Scalar,3,3> & V_inv =
          ( Matrix<Scalar,3,3>::Identity() - static_cast<Scalar>(0.5)*Omega
            + ( static_cast<Scalar>(1)
                - theta/(static_cast<Scalar>(2)*tan(theta/Scalar(2)))) /
            (theta*theta)*(Omega*Omega) );
      upsilon_omega.template head<3>() = V_inv*se3.translation();
    }
    return upsilon_omega;
  }

  /**
   * \brief vee-operator
   *
   * \param Omega 4x4-matrix representation of Lie algebra element
   * \returns     6-vector representatin of Lie algebra element
   *
   * This is the inverse of the hat()-operator.
   *
   * \see hat()
   */
  inline static
  const Tangent vee(const Transformation & Omega) {
    Tangent upsilon_omega;
    upsilon_omega.template head<3>() = Omega.col(3).template head<3>();
    upsilon_omega.template tail<3>()
        = SO3Group<Scalar>::vee(Omega.template topLeftCorner<3,3>());
    return upsilon_omega;
  }
};

/**
 * \brief SE3 default type - Constructors and default storage for SE3 Type
 */
template<typename _Scalar, int _Options>
class SE3Group : public SE3GroupBase<SE3Group<_Scalar,_Options> > {
  typedef SE3GroupBase<SE3Group<_Scalar,_Options> > Base;
public:
  /** \brief scalar type */
  typedef typename internal::traits<SE3Group<_Scalar,_Options> >
  ::Scalar Scalar;
  /** \brief SO3 reference type */
  typedef typename internal::traits<SE3Group<_Scalar,_Options> >
  ::SO3Type & SO3Reference;
  /** \brief SO3 const reference type */
  typedef const typename internal::traits<SE3Group<_Scalar,_Options> >
  ::SO3Type & ConstSO3Reference;
  /** \brief translation reference type */
  typedef typename internal::traits<SE3Group<_Scalar,_Options> >
  ::TranslationType & TranslationReference;
  /** \brief translation const reference type */
  typedef const typename internal::traits<SE3Group<_Scalar,_Options> >
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
  SE3Group()
    : translation_( Matrix<Scalar,3,1>::Zero() )
  {
  }

  /**
   * \brief Copy constructor
   */
  template<typename OtherDerived> inline
  SE3Group(const SE3GroupBase<OtherDerived> & other)
    : so3_(other.so3()), translation_(other.translation()) {
  }

  /**
   * \brief Constructor from SO3 and translation vector
   */
  template<typename OtherDerived> inline
  SE3Group(const SO3GroupBase<OtherDerived> & so3,
           const Point & translation)
    : so3_(so3), translation_(translation) {
  }

  /**
   * \brief Constructor from rotation matrix and translation vector
   *
   * \pre rotation matrix need to be orthogonal with determinant of 1
   */
  inline
  SE3Group(const Matrix<Scalar,3,3> & rotation_matrix,
           const Point & translation)
    : so3_(rotation_matrix), translation_(translation) {
  }

  /**
   * \brief Constructor from quaternion and translation vector
   *
   * \pre quaternion must not be zero
   */
  inline
  SE3Group(const Quaternion<Scalar> & quaternion,
           const Point & translation)
    : so3_(quaternion), translation_(translation) {
  }

  /**
   * \brief Constructor from 4x4 matrix
   *
   * \pre top-left 3x3 sub-matrix need to be orthogonal with determinant of 1
   */
  inline explicit
  SE3Group(const Eigen::Matrix<Scalar,4,4>& T)
    : so3_(T.template topLeftCorner<3,3>()),
      translation_(T.template block<3,1>(0,3)) {
  }

  /**
   * \returns pointer to internal data
   *
   * This provides unsafe read/write access to internal data. SE3 is represented
   * by a pair of an SO3 element (4 parameters) and translation vector (three
   * parameters). The user needs to take care of that the quaternion
   * stays normalized.
   *
   * Note: The first three Scalars represent the imaginary parts, while the
   * forth Scalar represent the real part.
   *
   * /see normalize()
   */
  EIGEN_STRONG_INLINE
  Scalar* data() {
    // so3_ and translation_ are layed out sequentially with no padding
    return so3_.data();
  }

  /**
   * \returns const pointer to internal data
   *
   * Const version of data().
   */
  EIGEN_STRONG_INLINE
  const Scalar* data() const {
    // so3_ and translation_ are layed out sequentially with no padding
    return so3_.data();
  }

  /**
   * \brief Accessor of SO3
   */
  EIGEN_STRONG_INLINE
  SO3Reference so3() {
    return so3_;
  }

  /**
   * \brief Mutator of SO3
   */
  EIGEN_STRONG_INLINE
  ConstSO3Reference so3() const {
    return so3_;
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
  Sophus::SO3Group<Scalar> so3_;
  Matrix<Scalar,3,1> translation_;
};


} // end namespace


namespace Eigen {
/**
 * \brief Specialisation of Eigen::Map for SE3GroupBase
 *
 * Allows us to wrap SE3 Objects around POD array
 * (e.g. external c style quaternion)
 */
template<typename _Scalar, int _Options>
class Map<Sophus::SE3Group<_Scalar>, _Options>
    : public Sophus::SE3GroupBase<Map<Sophus::SE3Group<_Scalar>, _Options> > {
  typedef Sophus::SE3GroupBase<Map<Sophus::SE3Group<_Scalar>, _Options> > Base;

public:
  /** \brief scalar type */
  typedef typename internal::traits<Map>::Scalar Scalar;
  /** \brief translation reference type */
  typedef typename internal::traits<Map>::TranslationType &
  TranslationReference;
  /** \brief translation const reference type */
  typedef const typename internal::traits<Map>::TranslationType &
  ConstTranslationReference;
  /** \brief SO3 reference type */
  typedef typename internal::traits<Map>::SO3Type &
  SO3Reference;
  /** \brief SO3 const reference type */
  typedef const typename internal::traits<Map>::SO3Type &
  ConstSO3Reference;

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
    : so3_(coeffs),
      translation_(coeffs+Sophus::SO3Group<Scalar>::num_parameters) {
  }

  /**
   * \brief Mutator of SO3
   */
  EIGEN_STRONG_INLINE
  SO3Reference so3() {
    return so3_;
  }

  /**
   * \brief Accessor of SO3
   */
  EIGEN_STRONG_INLINE
  ConstSO3Reference so3() const {
    return so3_;
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
  Map<Sophus::SO3Group<Scalar>,_Options> so3_;
  Map<Matrix<Scalar,3,1>,_Options> translation_;
};

/**
 * \brief Specialisation of Eigen::Map for const SE3GroupBase
 *
 * Allows us to wrap SE3 Objects around POD array
 * (e.g. external c style quaternion)
 */
template<typename _Scalar, int _Options>
class Map<const Sophus::SE3Group<_Scalar>, _Options>
    : public Sophus::SE3GroupBase<
    Map<const Sophus::SE3Group<_Scalar>, _Options> > {
  typedef Sophus::SE3GroupBase<Map<const Sophus::SE3Group<_Scalar>, _Options> >
  Base;

public:
  /** \brief scalar type */
  typedef typename internal::traits<Map>::Scalar Scalar;
  /** \brief translation const reference type */
  typedef const typename internal::traits<Map>::TranslationType &
  ConstTranslationReference;
  /** \brief SO3 const reference type */
  typedef const typename internal::traits<Map>::SO3Type &
  ConstSO3Reference;

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
    : so3_(coeffs),
      translation_(coeffs+Sophus::SO3Group<Scalar>::num_parameters) {
  }

  EIGEN_STRONG_INLINE
  Map(const Scalar* trans_coeffs, const Scalar* rot_coeffs)
    : translation_(trans_coeffs), so3_(rot_coeffs){
  }

  /**
   * \brief Accessor of SO3
   */
  EIGEN_STRONG_INLINE
  ConstSO3Reference so3() const {
    return so3_;
  }

  /**
   * \brief Accessor of translation vector
   */
  EIGEN_STRONG_INLINE
  ConstTranslationReference translation() const {
    return translation_;
  }

protected:
  const Map<const Sophus::SO3Group<Scalar>,_Options> so3_;
  const Map<const Matrix<Scalar,3,1>,_Options> translation_;
};

}

#endif
