// This file is part of Sophus.
//
// Copyright 2011-2013 Hauke Strasdat
// Copyrifht 2012-2013 Steven Lovegrove
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

#ifndef SOPHUS_SO3_HPP
#define SOPHUS_SO3_HPP

#include "sophus.hpp"

////////////////////////////////////////////////////////////////////////////
// Forward Declarations / typedefs
////////////////////////////////////////////////////////////////////////////

namespace Sophus {
template<typename _Scalar, int _Options=0> class SO3Group;
typedef EIGEN_DEPRECATED SO3Group<double> SO3;
typedef SO3Group<double> SO3d; /**< double precision SO3 */
typedef SO3Group<float> SO3f;  /**< single precision SO3 */
}

////////////////////////////////////////////////////////////////////////////
// Eigen Traits (For querying derived types in CRTP hierarchy)
////////////////////////////////////////////////////////////////////////////

namespace Eigen {
namespace internal {

template<typename _Scalar, int _Options>
struct traits<Sophus::SO3Group<_Scalar,_Options> > {
  typedef _Scalar Scalar;
  typedef Quaternion<Scalar> QuaternionType;
};

template<typename _Scalar, int _Options>
struct traits<Map<Sophus::SO3Group<_Scalar>, _Options> >
    : traits<Sophus::SO3Group<_Scalar, _Options> > {
  typedef _Scalar Scalar;
  typedef Map<Quaternion<Scalar>,_Options> QuaternionType;
};

template<typename _Scalar, int _Options>
struct traits<Map<const Sophus::SO3Group<_Scalar>, _Options> >
    : traits<const Sophus::SO3Group<_Scalar, _Options> > {
  typedef _Scalar Scalar;
  typedef Map<const Quaternion<Scalar>,_Options> QuaternionType;
};

}
}

namespace Sophus {
using namespace Eigen;

/**
 * \brief SO3 base type - implements SO3 class but is storage agnostic
 *
 * [add more detailed description/tutorial]
 */
template<typename Derived>
class SO3GroupBase {
public:
  /** \brief scalar type */
  typedef typename internal::traits<Derived>::Scalar Scalar;
  /** \brief quaternion reference type  */
  typedef typename internal::traits<Derived>::QuaternionType &
  QuaternionReference;
  /** \brief quaternion const reference type  */
  typedef const typename internal::traits<Derived>::QuaternionType &
  ConstQuaternionReference;

  /** \brief degree of freedom of group
   *         (three for rotation) */
  static const int DoF = 3;
  /** \brief number of internal parameters used
   *         (unit quaternion for rotation) */
  static const int num_parameters = 4;
  /** \brief group transformations are NxN matrices */
  static const int N = 3;
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
   * with \f$\ \widehat{\cdot} \f$ being the hat()-Vector4Scalaror.
   *
   * For SO3, it simply returns the rotation matrix corresponding to \f$ A \f$.
   */
  inline
  const Adjoint Adj() const {
    return matrix();
  }

  /**
   * \returns copy of instance casted to NewScalarType
   */
  template<typename NewScalarType>
  inline SO3Group<NewScalarType> cast() const {
    return SO3Group<NewScalarType>(unit_quaternion()
                                   .template cast<NewScalarType>() );
  }

  /**
   * \returns pointer to internal data
   *
   * This provides unsafe read/write access to internal data. SO3 is represented
   * by an Eigen::Quaternion (four parameters). When using direct write access,
   * the user needs to take care of that the quaternion stays normalized.
   *
   * Note: The first three Scalars represent the imaginary parts, while the
   * forth Scalar represent the real part.
   *
   * \see normalize()
   */
  inline Scalar* data() {
    return unit_quaternion_nonconst().coeffs().data();
  }

  /**
   * \returns const pointer to internal data
   *
   * Const version of data().
   */
  inline const Scalar* data() const {
    return unit_quaternion().coeffs().data();
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
  void fastMultiply(const SO3Group<Scalar>& other) {
    unit_quaternion_nonconst() *= other.unit_quaternion();
  }

  /**
   * \returns group inverse of instance
   */
  inline
  const SO3Group<Scalar> inverse() const {
    return SO3Group<Scalar>(unit_quaternion().conjugate());
  }

  /**
   * \brief Logarithmic map
   *
   * \returns tangent space representation (=rotation vector) of instance
   *
   * \see  log().
   */
  inline
  const Tangent log() const {
    return SO3Group<Scalar>::log(*this);
  }

  /**
   * \brief Normalize quaternion
   *
   * It re-normalizes unit_quaternion to unit length. This method only needs to
   * be called in conjunction with fastMultiply() or data() write access.
   */
  inline
  void normalize() {
    Scalar length = unit_quaternion_nonconst().norm();
    if (length < SophusConstants<Scalar>::epsilon()) {
      throw SophusException("Quaternion is (near) zero!");
    }
    unit_quaternion_nonconst().coeffs() /= length;
  }

  /**
   * \returns 3x3 matrix representation of instance
   *
   * For SO3, the matrix representation is an orthogonal matrix R with det(R)=1,
   * thus the so-called rotation matrix.
   */
  inline
  const Transformation matrix() const {
    return unit_quaternion().toRotationMatrix();
  }

  /**
   * \brief Assignment operator
   */
  template<typename OtherDerived> inline
  SO3GroupBase<Derived>& operator=(const SO3GroupBase<OtherDerived> & other) {
    unit_quaternion_nonconst() = other.unit_quaternion();
    return *this;
  }

  /**
   * \brief Group multiplication
   * \see operator*=()
   */
  inline
  const SO3Group<Scalar> operator*(const SO3Group<Scalar>& other) const {
    SO3Group<Scalar> result(*this);
    result *= other;
    return result;
  }

  /**
   * \brief Group action on \f$ \mathbf{R}^3 \f$
   *
   * \param p point \f$p \in \mathbf{R}^3 \f$
   * \returns point \f$p' \in \mathbf{R}^3 \f$, rotated version of \f$p\f$
   *
   * This function rotates a point \f$ p \f$ in  \f$ \mathbf{R}^3 \f$ by the
   * SO3 transformation \f$R\f$ (=rotation matrix): \f$ p' = R\cdot p \f$.
   *
   *
   * Since SO3 is intenally represented by a unit quaternion \f$q\f$, it is
   * implemented as \f$ p' = q\cdot p \cdot q^{*} \f$
   * with \f$ q^{*} \f$ being the quaternion conjugate of \f$ q \f$.
   *
   * Geometrically, \f$ p \f$  is rotated by angle \f$|\omega|\f$ around the
   * axis \f$\frac{\omega}{|\omega|}\f$ with \f$ \omega = \log(R)^\vee \f$.
   *
   * \see log()
   */
  inline
  const Point operator*(const Point & p) const {
    return unit_quaternion()._transformVector(p);
  }

  /**
   * \brief In-place group multiplication
   *
   * \see fastMultiply()
   * \see operator*()
   */
  inline
  void operator*=(const SO3Group<Scalar>& other) {
    fastMultiply(other);
    normalize();
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
  void setQuaternion(const Quaternion<Scalar>& quaternion) {
    unit_quaternion_nonconst() = quaternion;
    normalize();
  }

  /**
   * \brief Accessor of unit quaternion
   *
   * No direct write access is given to ensure the quaternion stays normalized.
   */
  EIGEN_STRONG_INLINE
  ConstQuaternionReference unit_quaternion() const {
    return static_cast<const Derived*>(this)->unit_quaternion();
  }

  ////////////////////////////////////////////////////////////////////////////
  // public static functions
  ////////////////////////////////////////////////////////////////////////////

  /**
   * \param   b 3-vector representation of Lie algebra element
   * \returns   derivative of Lie bracket
   *
   * This function returns \f$ \frac{\partial}{\partial a} [a, b]_{so3} \f$
   * with \f$ [a, b]_{so3} \f$ being the lieBracket() of the Lie algebra so3.
   *
   * \see lieBracket()
   */
  inline static
  const Adjoint d_lieBracketab_by_d_a(const Tangent & b) {
    return -hat(b);
  }

  /**
   * \brief Group exponential
   *
   * \param omega tangent space element (=rotation vector \f$ \omega \f$)
   * \returns     corresponding element of the group SO3
   *
   * To be more specific, this function computes \f$ \exp(\widehat{\omega}) \f$
   * with \f$ \exp(\cdot) \f$ being the matrix exponential
   * and \f$ \widehat{\cdot} \f$ the hat()-operator of SO3.
   *
   * \see expAndTheta()
   * \see hat()
   * \see log()
   */
  inline static
  const SO3Group<Scalar> exp(const Tangent & omega) {
    Scalar theta;
    return expAndTheta(omega, &theta);
  }

  /**
   * \brief Group exponential and theta
   *
   * \param      omega tangent space element (=rotation vector \f$ \omega \f$)
   * \param[out] theta angle of rotation \f$ \theta = |\omega| \f$
   * \returns          corresponding element of the group SO3
   *
   * \see exp() for details
   */
  inline static
  const SO3Group<Scalar> expAndTheta(const Tangent & omega,
                                     Scalar * theta) {
    const Scalar theta_sq = omega.squaredNorm();
    *theta = std::sqrt(theta_sq);
    const Scalar half_theta = static_cast<Scalar>(0.5)*(*theta);

    Scalar imag_factor;
    Scalar real_factor;;
    if((*theta)<SophusConstants<Scalar>::epsilon()) {
      const Scalar theta_po4 = theta_sq*theta_sq;
      imag_factor = static_cast<Scalar>(0.5)
                    - static_cast<Scalar>(1.0/48.0)*theta_sq
                    + static_cast<Scalar>(1.0/3840.0)*theta_po4;
      real_factor = static_cast<Scalar>(1)
                    - static_cast<Scalar>(0.5)*theta_sq +
                    static_cast<Scalar>(1.0/384.0)*theta_po4;
    } else {
      const Scalar sin_half_theta = std::sin(half_theta);
      imag_factor = sin_half_theta/(*theta);
      real_factor = std::cos(half_theta);
    }

    return SO3Group<Scalar>(Quaternion<Scalar>(real_factor,
                                               imag_factor*omega.x(),
                                               imag_factor*omega.y(),
                                               imag_factor*omega.z()));
  }

  /**
   * \brief Generators
   *
   * \pre \f$ i \in \{0,1,2\} \f$
   * \returns \f$ i \f$th generator \f$ G_i \f$ of SO3
   *
   * The infinitesimal generators of SO3
   * are \f$
   *        G_0 = \left( \begin{array}{ccc}
   *                          0&  0&  0& \\
   *                          0&  0& -1& \\
   *                          0&  1&  0&
   *                     \end{array} \right),
   *        G_1 = \left( \begin{array}{ccc}
   *                          0&  0&  1& \\
   *                          0&  0&  0& \\
   *                         -1&  0&  0&
   *                     \end{array} \right),
   *        G_2 = \left( \begin{array}{ccc}
   *                          0& -1&  0& \\
   *                          1&  0&  0& \\
   *                          0&  0&  0&
   *                     \end{array} \right).
   * \f$
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
   * Formally, the hat-operator of SO3 is defined
   * as \f$ \widehat{\cdot}: \mathbf{R}^3 \rightarrow \mathbf{R}^{3\times 3},
   * \quad \widehat{\omega} = \sum_{i=0}^2 G_i \omega_i \f$
   * with \f$ G_i \f$ being the ith infinitesial generator().
   *
   * \see generator()
   * \see vee()
   */
  inline static
  const Transformation hat(const Tangent & omega) {
    Transformation Omega;
    Omega <<  static_cast<Scalar>(0), -omega(2),  omega(1)
        ,  omega(2),     static_cast<Scalar>(0), -omega(0)
        , -omega(1),  omega(0),     static_cast<Scalar>(0);
    return Omega;
  }

  /**
   * \brief Lie bracket
   *
   * \param omega1 3-vector representation of Lie algebra element
   * \param omega2 3-vector representation of Lie algebra element
   * \returns      3-vector representation of Lie algebra element
   *
   * It computes the bracket of SO3. To be more specific, it
   * computes \f$ [\omega_1, \omega_2]_{so3}
   * := [\widehat{\omega_1}, \widehat{\omega_2}]^\vee \f$
   * with \f$ [A,B] = AB-BA \f$ being the matrix
   * commutator, \f$ \widehat{\cdot} \f$ the
   * hat()-operator and \f$ (\cdot)^\vee \f$ the vee()-operator of SO3.
   *
   * For the Lie algebra so3, the Lie bracket is simply the
   * cross product: \f$ [\omega_1, \omega_2]_{so3}
   *                    = \omega_1 \times \omega_2 \f$.
   *
   * \see hat()
   * \see vee()
   */
  inline static
  const Tangent lieBracket(const Tangent & omega1,
                           const Tangent & omega2) {
    return omega1.cross(omega2);
  }

  /**
   * \brief Logarithmic map
   *
   * \param other element of the group SO3
   * \returns     corresponding tangent space element
   *              (=rotation vector \f$ \omega \f$)
   *
   * Computes the logarithmic, the inverse of the group exponential.
   * To be specific, this function computes \f$ \log({\cdot})^\vee \f$
   * with \f$ \vee(\cdot) \f$ being the matrix logarithm
   * and \f$ \vee{\cdot} \f$ the vee()-operator of SO3.
   *
   * \see exp()
   * \see logAndTheta()
   * \see vee()
   */
  inline static
  const Tangent log(const SO3Group<Scalar> & other) {
    Scalar theta;
    return logAndTheta(other, &theta);
  }

  /**
   * \brief Logarithmic map and theta
   *
   * \param      other element of the group SO3
   * \param[out] theta angle of rotation \f$ \theta = |\omega| \f$
   * \returns          corresponding tangent space element
   *                   (=rotation vector \f$ \omega \f$)
   *
   * \see log() for details
   */
  inline static
  const Tangent logAndTheta(const SO3Group<Scalar> & other,
                            Scalar * theta) {
    const Scalar squared_n
        = other.unit_quaternion().vec().squaredNorm();
    const Scalar n = std::sqrt(squared_n);
    const Scalar w = other.unit_quaternion().w();

    Scalar two_atan_nbyw_by_n;

    // Atan-based log thanks to
    //
    // C. Hertzberg et al.:
    // "Integrating Generic Sensor Fusion Algorithms with Sound State
    // Representation through Encapsulation of Manifolds"
    // Information Fusion, 2011

    if (n < SophusConstants<Scalar>::epsilon()) {
      // If quaternion is normalized and n=0, then w should be 1;
      // w=0 should never happen here!
      if (std::abs(w) < SophusConstants<Scalar>::epsilon()) {
        throw SophusException("Quaternion is not normalized!");
      }
      const Scalar squared_w = w*w;
      two_atan_nbyw_by_n = static_cast<Scalar>(2) / w
                           - static_cast<Scalar>(2)*(squared_n)/(w*squared_w);
    } else {
      if (std::abs(w)<SophusConstants<Scalar>::epsilon()) {
        if (w > static_cast<Scalar>(0)) {
          two_atan_nbyw_by_n = M_PI/n;
        } else {
          two_atan_nbyw_by_n = -M_PI/n;
        }
      }else{
        two_atan_nbyw_by_n = static_cast<Scalar>(2) * atan(n/w) / n;
      }
    }

    *theta = two_atan_nbyw_by_n*n;

    return two_atan_nbyw_by_n * other.unit_quaternion().vec();
  }

  /**
   * \brief vee-operator
   *
   * \param Omega 3x3-matrix representation of Lie algebra element
   * \pr          Omega must be a skew-symmetric matrix
   * \returns     3-vector representatin of Lie algebra element
   *
   * This is the inverse of the hat()-operator.
   *
   * \see hat()
   */
  inline static
  const Tangent vee(const Transformation & Omega) {
    return static_cast<Scalar>(0.5) * Tangent(Omega(2,1) - Omega(1,2),
                                              Omega(0,2) - Omega(2,0),
                                              Omega(1,0) - Omega(0,1));
  }

private:
  // Mutator of unit_quaternion is private so users are hampered
  // from setting non-unit quaternions.
  EIGEN_STRONG_INLINE
  QuaternionReference unit_quaternion_nonconst() {
    return static_cast<Derived*>(this)->unit_quaternion_nonconst();
  }

};

/**
 * \brief SO3 default type - Constructors and default storage for SO3 Type
 */
template<typename _Scalar, int _Options>
class SO3Group : public SO3GroupBase<SO3Group<_Scalar,_Options> > {
  typedef SO3GroupBase<SO3Group<_Scalar,_Options> > Base;
public:
  /** \brief scalar type */
  typedef typename internal::traits<SO3Group<_Scalar,_Options> >
  ::Scalar Scalar;
  /** \brief quaternion type */
  typedef typename internal::traits<SO3Group<_Scalar,_Options> >
  ::QuaternionType & QuaternionReference;
  typedef const typename internal::traits<SO3Group<_Scalar,_Options> >
  ::QuaternionType & ConstQuaternionReference;

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

  // base is friend so unit_quaternion_nonconst can be accessed from base
  friend class SO3GroupBase<SO3Group<_Scalar,_Options> >;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * \brief Default constructor
   *
   * Initialize Quaternion to identity rotation.
   */
  inline
  SO3Group()
    : unit_quaternion_(static_cast<Scalar>(1), static_cast<Scalar>(0),
                       static_cast<Scalar>(0), static_cast<Scalar>(0)) {
  }

  /**
   * \brief Copy constructor
   */
  template<typename OtherDerived> inline
  SO3Group(const SO3GroupBase<OtherDerived> & other)
    : unit_quaternion_(other.unit_quaternion()) {
  }

  /**
   * \brief Constructor from rotation matrix
   *
   * \pre rotation matrix need to be orthogonal with determinant of 1
   */
  inline SO3Group(const Transformation & R)
    : unit_quaternion_(R) {
  }

  /**
   * \brief Constructor from quaternion
   *
   * \pre quaternion must not be zero
   */
  inline explicit
  SO3Group(const Quaternion<Scalar> & quat) : unit_quaternion_(quat) {
    Base::normalize();
  }

  /**
   * \brief Constructor from Euler angles
   *
   * \param alpha1 rotation around x-axis
   * \param alpha2 rotation around y-axis
   * \param alpha3 rotation around z-axis
   *
   * Since rotations in 3D do not commute, the order of the individual rotations
   * matter. Here, the following convention is used. We calculate a SO3 member
   * corresponding to the rotation matrix \f$ R \f$ such
   * that \f$ R=\exp\left(\begin{array}{c}\alpha_1\\ 0\\ 0\end{array}\right)
   *    \cdot   \exp\left(\begin{array}{c}0\\ \alpha_2\\ 0\end{array}\right)
   *    \cdot   \exp\left(\begin{array}{c}0\\ 0\\ \alpha_3\end{array}\right)\f$.
   */
  inline
  SO3Group(Scalar alpha1, Scalar alpha2, Scalar alpha3) {
    const static Scalar zero = static_cast<Scalar>(0);
    unit_quaternion_
        = ( SO3Group::exp(Tangent(alpha1,   zero,   zero))
            *SO3Group::exp(Tangent(  zero, alpha2,   zero))
            *SO3Group::exp(Tangent(  zero,   zero, alpha3)) )
          .unit_quaternion_;
  }

  /**
   * \brief Accessor of unit quaternion
   *
   * No direct write access is given to ensure the quaternion stays normalized.
   */
  EIGEN_STRONG_INLINE
  ConstQuaternionReference unit_quaternion() const {
    return unit_quaternion_;
  }

protected:
  // Mutator of unit_quaternion is protected so users are hampered
  // from setting non-unit quaternions.
  EIGEN_STRONG_INLINE
  QuaternionReference unit_quaternion_nonconst() {
    return unit_quaternion_;
  }

  Quaternion<Scalar> unit_quaternion_;
};

} // end namespace


namespace Eigen {
/**
 * \brief Specialisation of Eigen::Map for SO3GroupBase
 *
 * Allows us to wrap SO3 Objects around POD array
 * (e.g. external c style quaternion)
 */
template<typename _Scalar, int _Options>
class Map<Sophus::SO3Group<_Scalar>, _Options>
    : public Sophus::SO3GroupBase<Map<Sophus::SO3Group<_Scalar>, _Options> > {
  typedef Sophus::SO3GroupBase<Map<Sophus::SO3Group<_Scalar>, _Options> > Base;

public:
  /** \brief scalar type */
  typedef typename internal::traits<Map>::Scalar Scalar;
  /** \brief quaternion reference type */
  typedef typename internal::traits<Map>::QuaternionType &
  QuaternionReference;
  /** \brief quaternion const reference type */
  typedef const typename internal::traits<Map>::QuaternionType &
  ConstQuaternionReference;

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

  // base is friend so unit_quaternion_nonconst can be accessed from base
  friend class Sophus::SO3GroupBase<Map<Sophus::SO3Group<_Scalar>, _Options> >;

  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)
  using Base::operator*=;
  using Base::operator*;

  EIGEN_STRONG_INLINE
  Map(Scalar* coeffs) : unit_quaternion_(coeffs) {
  }

  /**
   * \brief Accessor of unit quaternion
   *
   * No direct write access is given to ensure the quaternion stays normalized.
   */
  EIGEN_STRONG_INLINE
  ConstQuaternionReference unit_quaternion() const {
    return unit_quaternion_;
  }

protected:
  // Mutator of unit_quaternion is protected so users are hampered
  // from setting non-unit quaternions.
  EIGEN_STRONG_INLINE
  QuaternionReference unit_quaternion_nonconst() {
    return unit_quaternion_;
  }

  Map<Quaternion<Scalar>,_Options> unit_quaternion_;
};

/**
 * \brief Specialisation of Eigen::Map for const SO3GroupBase
 *
 * Allows us to wrap SO3 Objects around POD array
 * (e.g. external c style quaternion)
 */
template<typename _Scalar, int _Options>
class Map<const Sophus::SO3Group<_Scalar>, _Options>
    : public Sophus::SO3GroupBase<
    Map<const Sophus::SO3Group<_Scalar>, _Options> > {
  typedef Sophus::SO3GroupBase<Map<const Sophus::SO3Group<_Scalar>, _Options> >
  Base;

public:
  /** \brief scalar type */
  typedef typename internal::traits<Map>::Scalar Scalar;
  /** \brief quaternion const reference type */
  typedef const typename internal::traits<Map>::QuaternionType &
  ConstQuaternionReference;

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
  Map(const Scalar* coeffs) : unit_quaternion_(coeffs) {
  }

  /**
   * \brief Accessor of unit quaternion
   *
   * No direct write access is given to ensure the quaternion stays normalized.
   */
  EIGEN_STRONG_INLINE
  const ConstQuaternionReference unit_quaternion() const {
    return unit_quaternion_;
  }

protected:
  const Map<const Quaternion<Scalar>,_Options> unit_quaternion_;
};

}

#endif
